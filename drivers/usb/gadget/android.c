/*
 * Gadget Driver for Android
 *
 * Copyright (C) 2008 Google, Inc.
 * Copyright (c) 2011-2012, The Linux Foundation. All rights reserved.
 * Author: Mike Lockwood <lockwood@android.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/* #define DEBUG */
/* #define VERBOSE_DEBUG */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/utsname.h>
#include <linux/platform_device.h>

#include <linux/usb/ch9.h>
#include <linux/usb/composite.h>
#include <linux/usb/gadget.h>
#include <linux/usb/android.h>

#include "gadget_chips.h"

/*
 * Kbuild is not very cooperative with respect to linking separately
 * compiled library objects into one module.  So for now we won't use
 * separate compilation ... ensuring init/exit sections work to shrink
 * the runtime footprint, and giving us at least some parts of what
 * a "gcc --combine ... part1.c part2.c part3.c ... " build would.
 */
#include "usbstring.c"
#include "config.c"
#include "epautoconf.c"
#include "composite.c"

#include "f_diag.c"
#include "f_rmnet_smd.c"
#include "f_rmnet_sdio.c"
#include "f_rmnet_smd_sdio.c"
#include "f_rmnet.c"
#include "f_audio_source.c"
#include "f_mass_storage.c"
#include "u_serial.c"
#include "u_sdio.c"
#include "u_smd.c"
#include "u_bam.c"
#include "u_rmnet_ctrl_smd.c"
#include "u_ctrl_hsic.c"
#include "u_data_hsic.c"
#include "f_serial.c"
#include "f_acm.c"
#include "f_adb.c"
#include "f_ccid.c"
#include "f_mtp.c"
#include "f_accessory.c"
#define USB_ETH_RNDIS y
#include "f_rndis.c"
#include "rndis.c"
#include "u_ether.c"
#include "f_usbnet.c"
#include "f_fs.c"

MODULE_AUTHOR("Mike Lockwood");
MODULE_DESCRIPTION("Android Composite USB Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");

static const char longname[] = "Gadget Android";

/* Default vendor and product IDs, overridden by userspace */
#define VENDOR_ID		0x18D1
#define PRODUCT_ID		0x0001

enum function_type {
	FT_UNKNOWN = 0,
	FT_CDROM,
	FT_MTPUSBNET
};

struct android_usb_function {
	char *name;
	void *config;

	struct device *dev;
	char *dev_name;
	struct device_attribute **attributes;

	/* for android_dev.enabled_functions */
	struct list_head enabled_list;

	/* Optional: initialization during gadget bind */
	int (*init)(struct android_usb_function *, struct usb_composite_dev *);
	/* Optional: cleanup during gadget unbind */
	void (*cleanup)(struct android_usb_function *);
	/* Optional: called when the function is added the list of
	 *		enabled functions */
	void (*enable)(struct android_usb_function *);
	/* Optional: called when it is removed */
	void (*disable)(struct android_usb_function *);

	int (*bind_config)(struct android_usb_function *, struct usb_configuration *);

	/* Optional: called when the configuration is removed */
	void (*unbind_config)(struct android_usb_function *, struct usb_configuration *);
	/* Optional: handle ctrl requests before the device is configured */
	int (*ctrlrequest)(struct android_usb_function *,
					struct usb_composite_dev *,
					const struct usb_ctrlrequest *);
};

struct android_dev {
	struct android_usb_function **functions;
	struct list_head enabled_functions;
	struct usb_composite_dev *cdev;
	struct device *dev;
	struct android_usb_platform_data *pdata;

	bool enabled;
	int disable_depth;
	struct mutex mutex;
	bool connected;
	bool sw_connected;
	int switch_index;
	bool usb_configured;
	bool cable_connected;
	bool cdrom_mounted;
	enum function_type curr_func_type;
	struct work_struct work;
	struct work_struct async_work;

        char ffs_aliases[256];
};

static struct class *android_class;
static struct android_dev *_android_dev;
static int android_bind_config(struct usb_configuration *c);
static void android_unbind_config(struct usb_configuration *c);

/* string IDs are assigned dynamically */
#define STRING_MANUFACTURER_IDX		0
#define STRING_PRODUCT_IDX		1
#define STRING_SERIAL_IDX		2

static char manufacturer_string[256];
static char product_string[256];
static char serial_string[256];
static char cdrom_blkdev_path[256];

/* String Table */
static struct usb_string strings_dev[] = {
	[STRING_MANUFACTURER_IDX].s = manufacturer_string,
	[STRING_PRODUCT_IDX].s = product_string,
	[STRING_SERIAL_IDX].s = serial_string,
	{  }			/* end of list */
};

static struct usb_gadget_strings stringtab_dev = {
	.language	= 0x0409,	/* en-us */
	.strings	= strings_dev,
};

static struct usb_gadget_strings *dev_strings[] = {
	&stringtab_dev,
	NULL,
};

static struct usb_device_descriptor device_desc = {
	.bLength              = sizeof(device_desc),
	.bDescriptorType      = USB_DT_DEVICE,
	.bcdUSB               = __constant_cpu_to_le16(0x0200),
	.bDeviceClass         = USB_CLASS_PER_INTERFACE,
	.idVendor             = __constant_cpu_to_le16(VENDOR_ID),
	.idProduct            = __constant_cpu_to_le16(PRODUCT_ID),
	.bcdDevice            = __constant_cpu_to_le16(0xffff),
	.bNumConfigurations   = 1,
};

static struct usb_configuration android_config_driver = {
	.label		= "android",
	.unbind		= android_unbind_config,
	.bConfigurationValue = 1,
};

static int mass_storage_function_set_cdrom_lun(char *lunpath);
static void send_usb_action_uevents(void);

static void android_work(struct work_struct *data)
{
	struct android_dev *dev = container_of(data, struct android_dev, work);
	struct usb_composite_dev *cdev = dev->cdev;
	char *disconnected[2] = { "USB_STATE=DISCONNECTED", NULL };
	char *connected[2]    = { "USB_STATE=CONNECTED", NULL };
	char *configured[2]   = { "USB_STATE=CONFIGURED", NULL };
	char **uevent_envp = NULL;
	unsigned long flags;
	int rc = 0;
	char ch = 0;

	spin_lock_irqsave(&cdev->lock, flags);
        if (cdev->config)
		uevent_envp = configured;
	else if (dev->connected != dev->sw_connected)
		uevent_envp = dev->connected ? connected : disconnected;
	dev->sw_connected = dev->connected;
	spin_unlock_irqrestore(&cdev->lock, flags);

	/* Handle all CDROM / MTPUSBNET events */
	if ((dev->curr_func_type == FT_CDROM) && (uevent_envp == configured)) {

		/* Mount CDROM partition when we get configured */
		rc = mass_storage_function_set_cdrom_lun(cdrom_blkdev_path);

	} else if ((dev->cdrom_mounted) && (uevent_envp != configured)) {

		/* Unmount CDROM partition when disconnected */
		rc = mass_storage_function_set_cdrom_lun(&ch);

	} else if ((dev->curr_func_type == FT_MTPUSBNET) &&
		(uevent_envp == configured)) {

		/* We have switched from CDROM to MTPUSBNET */
		dev->usb_configured = true;

	} else if ((dev->curr_func_type == FT_MTPUSBNET) &&
		(uevent_envp == connected) && (dev->usb_configured)) {

		/* Handle a mode change when we connected in MTPUSBNET mode */
		dev->usb_configured = false;

	} else if ((dev->curr_func_type == FT_MTPUSBNET) &&
		(!dev->cable_connected) && (dev->usb_configured)) {

		/* Handle a disconnection when connected in MTPUSBNET */
		dev->switch_index = SWITCH_INDEX_CDROM;
		send_usb_action_uevents();
		dev->usb_configured = false;

	}


	if (uevent_envp) {
		kobject_uevent_env(&dev->dev->kobj, KOBJ_CHANGE, uevent_envp);
		pr_info("android_usb: sent uevent %s\n", uevent_envp[0]);
	} else {
		pr_info("android_usb: did not send uevent (%d %d %p)\n",
			 dev->connected, dev->sw_connected, cdev->config);
	}
}

static void send_usb_action_uevents()
{
	struct android_dev *dev = _android_dev;
	char tmp_str[255];
	char *uevent_envp[3];

	snprintf(tmp_str, 255, "USB_ACTION_VALUE=switch%02d",
			dev->switch_index);

	uevent_envp[0] = "USB_ACTION_NAME=SETPROP";
	uevent_envp[1] = tmp_str;
	uevent_envp[2] = NULL;

	kobject_uevent_env(&dev->dev->kobj, KOBJ_CHANGE, uevent_envp);
	pr_info("android_usb: sent uevent %s %s\n", uevent_envp[0],
			uevent_envp[1]);
}

static void android_async_work(struct work_struct *data)
{
	send_usb_action_uevents();
	return;
}

void handle_switch_index(int index)
{
	struct android_dev *dev = _android_dev;

	if ((index == SWITCH_INDEX_RESET) ||
		((dev->curr_func_type == FT_CDROM) &&
		 (index == SWITCH_INDEX_MTPUSBNET))) {

		dev->switch_index = index;
		schedule_work(&dev->async_work);

	}
}

static int android_enable(struct android_dev *dev)
{
	struct usb_composite_dev *cdev = dev->cdev;
    int err = 0;

	BUG_ON(!mutex_is_locked(&dev->mutex));

    if (WARN_ON(!dev->disable_depth))
        return -1;

	if (--dev->disable_depth == 0) {
		err = usb_add_config(cdev, &android_config_driver,
					android_bind_config);
        if (err < 0) {
            pr_err("%s: usb_add_config failed : err: %d\n",
                   __func__, err);
            return err;
        }
		usb_gadget_connect(cdev->gadget);
	}
    return err;
}

static void android_disable(struct android_dev *dev)
{
	struct usb_composite_dev *cdev = dev->cdev;

	BUG_ON(!mutex_is_locked(&dev->mutex));

	if (dev->disable_depth++ == 0) {
		usb_gadget_disconnect(cdev->gadget);
		/* Cancel pending control requests */
		usb_ep_dequeue(cdev->gadget->ep0, cdev->req);
		usb_remove_config(cdev, &android_config_driver);
		usb_ep_autoconfig_reset(cdev->gadget);
	}
}

/*-------------------------------------------------------------------------*/
/* Supported functions initialization */

struct adb_data {
	bool opened;
	bool enabled;
	bool usb_compliance_mode;
};

static int
adb_function_init(struct android_usb_function *f,
		struct usb_composite_dev *cdev)
{
	f->config = kzalloc(sizeof(struct adb_data), GFP_KERNEL);
	if (!f->config)
		return -ENOMEM;

	return adb_setup();
}

static void adb_function_cleanup(struct android_usb_function *f)
{
	adb_cleanup();
	kfree(f->config);
}

static int
adb_function_bind_config(struct android_usb_function *f,
		struct usb_configuration *c)
{
	return adb_bind_config(c);
}


static ssize_t adb_usb_compliance_mode_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct adb_data *data = f->config;

	if (data->usb_compliance_mode)
		return snprintf(buf, PAGE_SIZE, "enabled\n");
	else
		return snprintf(buf, PAGE_SIZE, "disabled\n");
}

static ssize_t adb_usb_compliance_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct adb_data *data = f->config;
	int value;

	if (sscanf(buf, "%d", &value) == 1) {
		pr_info("android_usb: adb usb_compliance_mode = %d\n", value);
		if (value)
			data->usb_compliance_mode = true;
		else
			data->usb_compliance_mode = false;

		return size;
	}

	return -EINVAL;
}

static DEVICE_ATTR(usb_compliance_mode, S_IRUGO | S_IWUSR,
					adb_usb_compliance_mode_show,
					adb_usb_compliance_mode_store);


static struct device_attribute *adb_function_attributes[] = {
	&dev_attr_usb_compliance_mode,
	NULL
};

static void adb_android_function_enable(struct android_usb_function *f)
{
	struct android_dev *dev = _android_dev;
	struct adb_data *data = f->config;

	data->enabled = true;

	/* Disable the gadget until adbd is ready */
	/* Donot do disable/enable if usb_compliance_mode is enabled */
	if (!data->opened && !data->usb_compliance_mode)
		android_disable(dev);
}

static void adb_android_function_disable(struct android_usb_function *f)
{
	struct android_dev *dev = _android_dev;
	struct adb_data *data = f->config;

	data->enabled = false;

	/* Balance the disable that was called in closed_callback */
	/* Donot do disable/enable if usb_compliance_mode is enabled */
	if (!data->opened && !data->usb_compliance_mode)
		android_enable(dev);
}

static struct android_usb_function adb_function = {
	.name		= "adb",
	.enable		= adb_android_function_enable,
	.disable	= adb_android_function_disable,
	.init		= adb_function_init,
	.cleanup	= adb_function_cleanup,
	.bind_config	= adb_function_bind_config,
	.attributes	= adb_function_attributes,
};

static void adb_ready_callback(void)
{
	struct android_dev *dev = _android_dev;
	struct adb_data *data = adb_function.config;

	/* Donot do disable/enable if usb_compliance_mode is enabled */
	if (data->usb_compliance_mode)
		return;

	mutex_lock(&dev->mutex);

	data->opened = true;

	if (data->enabled)
		android_enable(dev);

	mutex_unlock(&dev->mutex);
}

static void adb_closed_callback(void)
{
	struct android_dev *dev = _android_dev;
	struct adb_data *data = adb_function.config;

	/* Donot do disable/enable if usb_compliance_mode is enabled */
	if (data->usb_compliance_mode)
		return;

	mutex_lock(&dev->mutex);

	data->opened = false;

	if (data->enabled)
		android_disable(dev);

	mutex_unlock(&dev->mutex);
}

/*-------------------------------------------------------------------------*/
/* Supported functions initialization */

/* RMNET_SMD */
static int rmnet_smd_function_bind_config(struct android_usb_function *f,
					  struct usb_configuration *c)
{
	return rmnet_smd_bind_config(c);
}

static struct android_usb_function rmnet_smd_function = {
	.name		= "rmnet_smd",
	.bind_config	= rmnet_smd_function_bind_config,
};

/* RMNET_SDIO */
static int rmnet_sdio_function_bind_config(struct android_usb_function *f,
					  struct usb_configuration *c)
{
	return rmnet_sdio_function_add(c);
}

static struct android_usb_function rmnet_sdio_function = {
	.name		= "rmnet_sdio",
	.bind_config	= rmnet_sdio_function_bind_config,
};

/* RMNET_SMD_SDIO */
static int rmnet_smd_sdio_function_init(struct android_usb_function *f,
				 struct usb_composite_dev *cdev)
{
	return rmnet_smd_sdio_init();
}

static void rmnet_smd_sdio_function_cleanup(struct android_usb_function *f)
{
	rmnet_smd_sdio_cleanup();
}

static int rmnet_smd_sdio_bind_config(struct android_usb_function *f,
					  struct usb_configuration *c)
{
	return rmnet_smd_sdio_function_add(c);
}

static struct device_attribute *rmnet_smd_sdio_attributes[] = {
					&dev_attr_transport, NULL };

static struct android_usb_function rmnet_smd_sdio_function = {
	.name		= "rmnet_smd_sdio",
	.init		= rmnet_smd_sdio_function_init,
	.cleanup	= rmnet_smd_sdio_function_cleanup,
	.bind_config	= rmnet_smd_sdio_bind_config,
	.attributes	= rmnet_smd_sdio_attributes,
};

/*rmnet transport string format(per port):"ctrl0,data0,ctrl1,data1..." */
#define MAX_XPORT_STR_LEN 50
static char rmnet_transports[MAX_XPORT_STR_LEN];

static void rmnet_function_cleanup(struct android_usb_function *f)
{
	frmnet_cleanup();
}

static int rmnet_function_bind_config(struct android_usb_function *f,
					 struct usb_configuration *c)
{
	int i;
	int err = 0;
	char *ctrl_name;
	char *data_name;
	char buf[MAX_XPORT_STR_LEN], *b;
	static int rmnet_initialized, ports;

	if (!rmnet_initialized) {
		rmnet_initialized = 1;
		strlcpy(buf, rmnet_transports, sizeof(buf));
		b = strim(buf);
		while (b) {
			ctrl_name = strsep(&b, ",");
			data_name = strsep(&b, ",");
			if (ctrl_name && data_name) {
				err = frmnet_init_port(ctrl_name, data_name);
				if (err) {
					pr_err("rmnet: Cannot open ctrl port:"
						"'%s' data port:'%s'\n",
						ctrl_name, data_name);
					goto out;
				}
				ports++;
			}
		}

		err = rmnet_gport_setup();
		if (err) {
			pr_err("rmnet: Cannot setup transports");
			goto out;
		}
	}

	for (i = 0; i < ports; i++) {
		err = frmnet_bind_config(c, i);
		if (err) {
			pr_err("Could not bind rmnet%u config\n", i);
			break;
		}
	}
out:
	return err;
}

static ssize_t rmnet_transports_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", rmnet_transports);
}

static ssize_t rmnet_transports_store(
		struct device *device, struct device_attribute *attr,
		const char *buff, size_t size)
{
	strlcpy(rmnet_transports, buff, sizeof(rmnet_transports));

	return size;
}

static struct device_attribute dev_attr_rmnet_transports =
					__ATTR(transports, S_IRUGO | S_IWUSR,
						rmnet_transports_show,
						rmnet_transports_store);
static struct device_attribute *rmnet_function_attributes[] = {
					&dev_attr_rmnet_transports,
					NULL };

static struct android_usb_function rmnet_function = {
	.name		= "rmnet",
	.cleanup	= rmnet_function_cleanup,
	.bind_config	= rmnet_function_bind_config,
	.attributes	= rmnet_function_attributes,
};

/* DIAG */
static char diag_clients[32];	    /*enabled DIAG clients- "diag[,diag_mdm]" */
static ssize_t clients_store(
		struct device *device, struct device_attribute *attr,
		const char *buff, size_t size)
{
	strlcpy(diag_clients, buff, sizeof(diag_clients));

	return size;
}

static DEVICE_ATTR(clients, S_IWUSR, NULL, clients_store);
static struct device_attribute *diag_function_attributes[] =
					 { &dev_attr_clients, NULL };

static int diag_function_init(struct android_usb_function *f,
				 struct usb_composite_dev *cdev)
{
	return diag_setup();
}

static void diag_function_cleanup(struct android_usb_function *f)
{
	diag_cleanup();
}

static int diag_function_bind_config(struct android_usb_function *f,
					struct usb_configuration *c)
{
	char *name;
	char buf[32], *b;
	int once = 0, err = -1;
	int (*notify)(uint32_t, const char *);

	strlcpy(buf, diag_clients, sizeof(buf));
	b = strim(buf);

	while (b) {
		notify = NULL;
		name = strsep(&b, ",");
		/* Allow only first diag channel to update pid and serial no */
		if (_android_dev->pdata && !once++)
			notify = _android_dev->pdata->update_pid_and_serial_num;

		if (name) {
			err = diag_function_add(c, name, notify);
			if (err)
				pr_err("diag: Cannot open channel '%s'", name);
		}
	}

	return err;
}

static struct android_usb_function diag_function = {
	.name		= "diag",
	.init		= diag_function_init,
	.cleanup	= diag_function_cleanup,
	.bind_config	= diag_function_bind_config,
	.attributes	= diag_function_attributes,
};

/* SERIAL */
static char serial_transports[32];	/*enabled FSERIAL ports - "tty[,sdio]"*/
static ssize_t serial_transports_store(
		struct device *device, struct device_attribute *attr,
		const char *buff, size_t size)
{
	strlcpy(serial_transports, buff, sizeof(serial_transports));

	return size;
}

static DEVICE_ATTR(transports, S_IWUSR, NULL, serial_transports_store);
static struct device_attribute *serial_function_attributes[] =
					 { &dev_attr_transports, NULL };

static void serial_function_cleanup(struct android_usb_function *f)
{
	gserial_cleanup();
}

static int serial_function_bind_config(struct android_usb_function *f,
					struct usb_configuration *c)
{
	char *name;
	char buf[32], *b;
	int err = -1, i;
	static int serial_initialized = 0, ports = 0;

	if (serial_initialized)
		goto bind_config;

	serial_initialized = 1;
	strlcpy(buf, serial_transports, sizeof(buf));
	b = strim(buf);

	while (b) {
		name = strsep(&b, ",");

		if (name) {
			err = gserial_init_port(ports, name);
			if (err) {
				pr_err("serial: Cannot open port '%s'", name);
				goto out;
			}
			ports++;
		}
	}
	err = gport_setup(c);
	if (err) {
		pr_err("serial: Cannot setup transports");
		goto out;
	}

bind_config:
	for (i = 0; i < ports; i++) { 
		err = gser_bind_config(c, i);
		if (err) {
			pr_err("serial: bind_config failed for port %d", i);
			goto out;
		}
	}

out:
	return err;
}

static struct android_usb_function serial_function = {
	.name		= "serial",
	.cleanup	= serial_function_cleanup,
	.bind_config	= serial_function_bind_config,
	.attributes	= serial_function_attributes,
};

/* ACM */
static char acm_transports[32];	/*enabled ACM ports - "tty[,sdio]"*/
static ssize_t acm_transports_store(
		struct device *device, struct device_attribute *attr,
		const char *buff, size_t size)
{
	strlcpy(acm_transports, buff, sizeof(acm_transports));

	return size;
}

static DEVICE_ATTR(acm_transports, S_IWUSR, NULL, acm_transports_store);
static struct device_attribute *acm_function_attributes[] = {
		&dev_attr_acm_transports, NULL };

static void acm_function_cleanup(struct android_usb_function *f)
{
	gserial_cleanup();
}

static int acm_function_bind_config(struct android_usb_function *f,
					struct usb_configuration *c)
{
	char *name;
	char buf[32], *b;
	int err = -1, i;
	static int acm_initialized, ports;

	if (acm_initialized)
		goto bind_config;

	acm_initialized = 1;
	strlcpy(buf, acm_transports, sizeof(buf));
	b = strim(buf);

	while (b) {
		name = strsep(&b, ",");

		if (name) {
			err = acm_init_port(ports, name);
			if (err) {
				pr_err("acm: Cannot open port '%s'", name);
				goto out;
			}
			ports++;
		}
	}
	err = acm_port_setup(c);
	if (err) {
		pr_err("acm: Cannot setup transports");
		goto out;
	}

bind_config:
	for (i = 0; i < ports; i++) {
		err = acm_bind_config(c, i);
		if (err) {
			pr_err("acm: bind_config failed for port %d", i);
			goto out;
		}
	}

out:
	return err;
}
static struct android_usb_function acm_function = {
	.name		= "acm",
	.cleanup	= acm_function_cleanup,
	.bind_config	= acm_function_bind_config,
	.attributes	= acm_function_attributes,
};

/* CCID */
static int ccid_function_init(struct android_usb_function *f,
					struct usb_composite_dev *cdev)
{
	return ccid_setup();
}

static void ccid_function_cleanup(struct android_usb_function *f)
{
	ccid_cleanup();
}

static int ccid_function_bind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	return ccid_bind_config(c);
}

static struct android_usb_function ccid_function = {
	.name		= "ccid",
	.init		= ccid_function_init,
	.cleanup	= ccid_function_cleanup,
	.bind_config	= ccid_function_bind_config,
};

static int mtp_function_init(struct android_usb_function *f, struct usb_composite_dev *cdev)
{
	return mtp_setup();
}

static void mtp_function_cleanup(struct android_usb_function *f)
{
	mtp_cleanup();
}

static int mtp_function_bind_config(struct android_usb_function *f, struct usb_configuration *c)
{
	return mtp_bind_config(c, false);
}

static int ptp_function_init(struct android_usb_function *f, struct usb_composite_dev *cdev)
{
	/* nothing to do - initialization is handled by mtp_function_init */
	return 0;
}

static void ptp_function_cleanup(struct android_usb_function *f)
{
	/* nothing to do - cleanup is handled by mtp_function_cleanup */
}

static int ptp_function_bind_config(struct android_usb_function *f, struct usb_configuration *c)
{
	return mtp_bind_config(c, true);
}

static int mtp_function_ctrlrequest(struct android_usb_function *f,
						struct usb_composite_dev *cdev,
						const struct usb_ctrlrequest *c)
{
	return mtp_ctrlrequest(cdev, c);
}

static struct android_usb_function mtp_function = {
	.name		= "mtp",
	.init		= mtp_function_init,
	.cleanup	= mtp_function_cleanup,
	.bind_config	= mtp_function_bind_config,
	.ctrlrequest	= mtp_function_ctrlrequest,
};

/* PTP function is same as MTP with slightly different interface descriptor */
static struct android_usb_function ptp_function = {
	.name		= "ptp",
	.init		= ptp_function_init,
	.cleanup	= ptp_function_cleanup,
	.bind_config	= ptp_function_bind_config,
};


struct rndis_function_config {
	u8      ethaddr[ETH_ALEN];
	u32     vendorID;
	char	manufacturer[256];
	bool	wceis;
};

static int rndis_function_init(struct android_usb_function *f, struct usb_composite_dev *cdev)
{
	f->config = kzalloc(sizeof(struct rndis_function_config), GFP_KERNEL);
	if (!f->config)
		return -ENOMEM;
	return 0;
}

static void rndis_function_cleanup(struct android_usb_function *f)
{
	kfree(f->config);
	f->config = NULL;
}

static int rndis_function_bind_config(struct android_usb_function *f,
					struct usb_configuration *c)
{
	int ret;
	struct rndis_function_config *rndis = f->config;

	if (!rndis) {
		pr_err("%s: rndis_pdata\n", __func__);
		return -1;
	}

	pr_info("%s MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", __func__,
		rndis->ethaddr[0], rndis->ethaddr[1], rndis->ethaddr[2],
		rndis->ethaddr[3], rndis->ethaddr[4], rndis->ethaddr[5]);

	ret = gether_setup_name(c->cdev->gadget, rndis->ethaddr, "rndis");
	if (ret) {
		pr_err("%s: gether_setup failed\n", __func__);
		return ret;
	}

	if (rndis->wceis) {
		/* "Wireless" RNDIS; auto-detected by Windows */
		rndis_iad_descriptor.bFunctionClass =
						USB_CLASS_WIRELESS_CONTROLLER;
		rndis_iad_descriptor.bFunctionSubClass = 0x01;
		rndis_iad_descriptor.bFunctionProtocol = 0x03;
		rndis_control_intf.bInterfaceClass =
						USB_CLASS_WIRELESS_CONTROLLER;
		rndis_control_intf.bInterfaceSubClass =	 0x01;
		rndis_control_intf.bInterfaceProtocol =	 0x03;
	}

	return rndis_bind_config(c, rndis->ethaddr, rndis->vendorID,
				    rndis->manufacturer);
}

static void rndis_function_unbind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	gether_cleanup();
}

static ssize_t rndis_manufacturer_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;
	return snprintf(buf, PAGE_SIZE, "%s\n", config->manufacturer);
}

static ssize_t rndis_manufacturer_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;

	if (size >= sizeof(config->manufacturer))
		return -EINVAL;
	if (sscanf(buf, "%255s", config->manufacturer) == 1)
		return size;
	return -1;
}

static DEVICE_ATTR(manufacturer, S_IRUGO | S_IWUSR, rndis_manufacturer_show,
						    rndis_manufacturer_store);

static ssize_t rndis_wceis_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;
	return snprintf(buf, PAGE_SIZE, "%d\n", config->wceis);
}

static ssize_t rndis_wceis_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;
	int value;

	if (sscanf(buf, "%d", &value) == 1) {
		config->wceis = value;
		return size;
	}
	return -EINVAL;
}

static DEVICE_ATTR(wceis, S_IRUGO | S_IWUSR, rndis_wceis_show,
					     rndis_wceis_store);

static ssize_t rndis_ethaddr_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *rndis = f->config;
	return snprintf(buf, PAGE_SIZE, "%02x:%02x:%02x:%02x:%02x:%02x\n",
		rndis->ethaddr[0], rndis->ethaddr[1], rndis->ethaddr[2],
		rndis->ethaddr[3], rndis->ethaddr[4], rndis->ethaddr[5]);
}

static ssize_t rndis_ethaddr_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *rndis = f->config;

	if (sscanf(buf, "%02x:%02x:%02x:%02x:%02x:%02x\n",
		    (int *)&rndis->ethaddr[0], (int *)&rndis->ethaddr[1],
		    (int *)&rndis->ethaddr[2], (int *)&rndis->ethaddr[3],
		    (int *)&rndis->ethaddr[4], (int *)&rndis->ethaddr[5]) == 6)
		return size;
	return -EINVAL;
}

static DEVICE_ATTR(ethaddr, S_IRUGO | S_IWUSR, rndis_ethaddr_show,
					       rndis_ethaddr_store);

static ssize_t rndis_vendorID_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;
	return snprintf(buf, PAGE_SIZE, "%04x\n", config->vendorID);
}

static ssize_t rndis_vendorID_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;
	int value;

	if (sscanf(buf, "%04x", &value) == 1) {
		config->vendorID = value;
		return size;
	}
	return -EINVAL;
}

static DEVICE_ATTR(vendorID, S_IRUGO | S_IWUSR, rndis_vendorID_show,
						rndis_vendorID_store);

static struct device_attribute *rndis_function_attributes[] = {
	&dev_attr_manufacturer,
	&dev_attr_wceis,
	&dev_attr_ethaddr,
	&dev_attr_vendorID,
	NULL
};

static struct android_usb_function rndis_function = {
	.name		= "rndis",
	.init		= rndis_function_init,
	.cleanup	= rndis_function_cleanup,
	.bind_config	= rndis_function_bind_config,
	.unbind_config	= rndis_function_unbind_config,
	.attributes	= rndis_function_attributes,
};


struct mass_storage_function_config {
	struct fsg_config fsg;
	struct fsg_common *common;
};

/* sizes based on inquiry string requirements */
static char ms_vendor[9];
static char ms_product[17];

static int mass_storage_function_init(struct android_usb_function *f,
					struct usb_composite_dev *cdev)
{
	struct mass_storage_function_config *config;
	struct fsg_common *common;
	int err, i;

	config = kzalloc(sizeof(struct mass_storage_function_config),
								GFP_KERNEL);
	if (!config)
		return -ENOMEM;

	config->fsg.cdrom_lun_num = 0;
    config->fsg.nluns = FSG_MAX_LUNS;

    for (i = 0; i < config->fsg.nluns; i++) {
        config->fsg.luns[i].removable = 1;
        config->fsg.luns[i].ro = 0;
        config->fsg.luns[i].cdrom = 0;
        config->fsg.luns[i].nofua = 1;
    }

	/* defaults that should be overriden by user space */
	strncpy(ms_vendor, "Android", sizeof(ms_vendor));
	strncpy(ms_product, "Android", sizeof(ms_product));
	config->fsg.vendor_name = ms_vendor;
	config->fsg.product_name = ms_product;

	common = fsg_common_init(NULL, cdev, &config->fsg);
	if (IS_ERR(common)) {
		kfree(config);
		return PTR_ERR(common);
	}

    for (i = 0; i < config->fsg.nluns; i++) {
        char name[32];

        memset(name, 0, 32);

        if (i) {
            snprintf(name, 5, "lun%d\n", i);
        } else {
            strcpy(name, "lun");
        }

        pr_debug("lun name: %s\n", name);

        err = sysfs_create_link(&f->dev->kobj,
                    &common->luns[i].dev.kobj,
                    name);
        if (err) {
            fsg_common_release(&common->ref);
            kfree(config);
            return err;
        }
    }

	config->common = common;
	f->config = config;
	return 0;
}

static void mass_storage_function_cleanup(struct android_usb_function *f)
{
	kfree(f->config);
	f->config = NULL;
}

static int mass_storage_function_bind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	struct mass_storage_function_config *config = f->config;
	return fsg_bind_config(c->cdev, c, config->common);
}

static ssize_t mass_storage_inquiry_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct mass_storage_function_config *config = f->config;
	return snprintf(buf, PAGE_SIZE, "%s\n", config->common->inquiry_string);
}

static ssize_t mass_storage_inquiry_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct mass_storage_function_config *config = f->config;
	if (size >= sizeof(config->common->inquiry_string))
		return -EINVAL;
	if (sscanf(buf, "%28s", config->common->inquiry_string) != 1)
		return -EINVAL;
	return size;
}

static DEVICE_ATTR(inquiry_string, S_IRUGO | S_IWUSR,
					mass_storage_inquiry_show,
					mass_storage_inquiry_store);

static ssize_t mass_storage_nluns_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "%d\n", enabled_nluns);
}

static ssize_t mass_storage_nluns_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size)
{
    uint value;

    if (sscanf(buf, "%u", &value) == 1) {
        if (value > FSG_MAX_LUNS)
            return -EINVAL;
        pr_info("android_usb: nluns =  %u\n", value);
        enabled_nluns = value;
        return size;
    }

    return -EINVAL;
}

static DEVICE_ATTR(nluns, S_IRUGO | S_IWUSR, mass_storage_nluns_show,
                                             mass_storage_nluns_store);

static ssize_t mass_storage_vendor_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct mass_storage_function_config *config = f->config;

	if ((size - 1) >= sizeof(ms_vendor))
		return -EINVAL;

	strncpy(ms_vendor, buf, sizeof(ms_vendor));
	ms_vendor[sizeof(ms_vendor)-1] = 0;
	snprintf(config->common->inquiry_string,
		sizeof config->common->inquiry_string,
		"%-8s%-16s%04x", ms_vendor, ms_product, 1);

	return size;
}

static DEVICE_ATTR(vendor, S_IWUSR, NULL, mass_storage_vendor_store);

static ssize_t mass_storage_product_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct mass_storage_function_config *config = f->config;

	if ((size - 1) >= sizeof(ms_product))
		return -EINVAL;

	strncpy(ms_product, buf, sizeof(ms_product));
	ms_product[sizeof(ms_product)-1] = 0;
	snprintf(config->common->inquiry_string,
		sizeof config->common->inquiry_string,
		"%-8s%-16s%04x", ms_vendor, ms_product, 1);

	return size;
}

static DEVICE_ATTR(product, S_IWUSR, NULL, mass_storage_product_store);

static ssize_t mass_storage_cdrom_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int value;

	if (sscanf(buf, "%d", &value) == 1) {
		pr_info("android_usb: ms_cdrom_enable =  %d\n", value);
		ms_cdrom_enable = value;
		return size;
	}

	return -EINVAL;
}


static DEVICE_ATTR(cdrom, S_IWUSR, NULL, mass_storage_cdrom_store);

static struct device_attribute *mass_storage_function_attributes[] = {
	&dev_attr_inquiry_string,
	&dev_attr_vendor,
	&dev_attr_product,
	&dev_attr_cdrom,
    &dev_attr_nluns,
	NULL
};

static int mass_storage_function_set_cdrom_lun(char *lunpath)
{
	struct android_dev *dev = _android_dev;
	struct android_usb_function **functions = dev->functions;
	struct android_usb_function *f;
	struct mass_storage_function_config *config;
	int cdromlun_num;
	struct fsg_lun  *cdromlun;
	int rc = 0;

	/* look up for mass storage function instance */
	while ((f = *functions++)) {
		if (!strcmp("mass_storage", f->name))
			break;
	}

	if (f == NULL) {
		pr_info("android_usb: set_cdrom_lun - no mass storage\n");
		return -1;
	}
	config = f->config;
	cdromlun_num = config->fsg.cdrom_lun_num;
	cdromlun = &(config->common->luns[cdromlun_num]);

	dev->cdrom_mounted = false;
	/* Eject current medium */
	if (fsg_lun_is_open(cdromlun)) {
		pr_info("android_usb: set_cdrom_lun - ejected\n");
		fsg_lun_close(cdromlun);
		cdromlun->unit_attention_data = SS_MEDIUM_NOT_PRESENT;
	}

	/* Load new medium */
	if (lunpath[0]) {
		pr_info("android_usb: set_cdrom_lun = %s\n", lunpath);
		rc = fsg_lun_open(cdromlun, lunpath);
		if (rc)
			pr_err("android_usb: set_cdrom_lun failed %d\n", rc);
		else
			cdromlun->unit_attention_data =
				SS_NOT_READY_TO_READY_TRANSITION;

		dev->cdrom_mounted = true;
	}

	return rc;
}

static struct android_usb_function mass_storage_function = {
	.name		= "mass_storage",
	.init		= mass_storage_function_init,
	.cleanup	= mass_storage_function_cleanup,
	.bind_config	= mass_storage_function_bind_config,
	.attributes	= mass_storage_function_attributes,
};


static int accessory_function_init(struct android_usb_function *f,
					struct usb_composite_dev *cdev)
{
	return acc_setup();
}

static void accessory_function_cleanup(struct android_usb_function *f)
{
	acc_cleanup();
}

static int accessory_function_bind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	return acc_bind_config(c);
}

static int accessory_function_ctrlrequest(struct android_usb_function *f,
						struct usb_composite_dev *cdev,
						const struct usb_ctrlrequest *c)
{
	return acc_ctrlrequest(cdev, c);
}

static struct android_usb_function accessory_function = {
	.name		= "accessory",
	.init		= accessory_function_init,
	.cleanup	= accessory_function_cleanup,
	.bind_config	= accessory_function_bind_config,
	.ctrlrequest	= accessory_function_ctrlrequest,
};

static int usbnet_function_init(struct android_usb_function *f,
				struct usb_composite_dev *cdev)
{
	struct usbnet_device *dev;
	struct usbnet_context *context;
	struct net_device *net_dev;
	int ret;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	net_dev = alloc_netdev(sizeof(struct usbnet_context),
			   "usb%d", usb_ether_setup);
	if (!net_dev) {
		pr_err("%s: alloc_netdev error\n", __func__);
		return -EINVAL;
	}

	ret = register_netdev(net_dev);
	if (ret) {
		pr_err("%s: register_netdev error\n", __func__);
		free_netdev(net_dev);
		return -EINVAL;
	}

	ret = device_create_file(&net_dev->dev, &dev_attr_description);
	if (ret < 0) {
		pr_err("%s: sys file creation  error\n", __func__);
		unregister_netdev(net_dev);
		free_netdev(net_dev);
		return -EINVAL;
	}

	context = netdev_priv(net_dev);
	INIT_WORK(&context->usbnet_config_wq, usbnet_if_config);

	context->config = 0;
	dev->net_ctxt = context;

	f->config = dev;

	switch_dev_register(&usbnet_enable_device);
	return 0;
}

static void usbnet_function_cleanup(struct android_usb_function *f)
{
	struct usbnet_device *dev = f->config;

	usbnet_cleanup(dev);
	switch_dev_unregister(&usbnet_enable_device);
}

static int usbnet_function_bind_config(struct android_usb_function *f, struct usb_configuration *c)
{
	struct usbnet_device *dev = f->config;

	return usbnet_bind_config(dev, c);
}

static int usbnet_function_ctrlrequest(struct android_usb_function *f, struct usb_composite_dev *cdev,
					const struct usb_ctrlrequest *c)
{
	struct usbnet_device *dev = f->config;

	return usbnet_ctrlrequest(dev, cdev, c);
}

static struct android_usb_function usbnet_function = {
	.name		= "usbnet",
	.init		= usbnet_function_init,
	.cleanup	= usbnet_function_cleanup,
	.bind_config	= usbnet_function_bind_config,
	.ctrlrequest	= usbnet_function_ctrlrequest,
};

static int audio_source_function_init(struct android_usb_function *f,
			struct usb_composite_dev *cdev)
{
	struct audio_source_config *config;

	config = kzalloc(sizeof(struct audio_source_config), GFP_KERNEL);
	if (!config)
		return -ENOMEM;
	config->card = -1;
	config->device = -1;
	f->config = config;
	return 0;
}

static void audio_source_function_cleanup(struct android_usb_function *f)
{
	kfree(f->config);
}

static int audio_source_function_bind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	struct audio_source_config *config = f->config;

	return audio_source_bind_config(c, config);
}

static void audio_source_function_unbind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	struct audio_source_config *config = f->config;

	config->card = -1;
	config->device = -1;
}

static ssize_t audio_source_pcm_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct audio_source_config *config = f->config;

	/* print PCM card and device numbers */
	return sprintf(buf, "%d %d\n", config->card, config->device);
}

static DEVICE_ATTR(pcm, S_IRUGO | S_IWUSR, audio_source_pcm_show, NULL);

static struct device_attribute *audio_source_function_attributes[] = {
	&dev_attr_pcm,
	NULL
};

static struct android_usb_function audio_source_function = {
	.name		= "audio_source",
	.init		= audio_source_function_init,
	.cleanup	= audio_source_function_cleanup,
	.bind_config	= audio_source_function_bind_config,
	.unbind_config	= audio_source_function_unbind_config,
	.attributes	= audio_source_function_attributes,
};

/* functionfs */
struct functionfs_config {
        bool opened;
        bool enabled;
        struct ffs_data *data;
};

static int ffs_function_init(struct android_usb_function *f,
                             struct usb_composite_dev *cdev)
{
        f->config = kzalloc(sizeof(struct functionfs_config), GFP_KERNEL);
        if (!f->config)
                return -ENOMEM;

        return functionfs_init();
}

static void ffs_function_cleanup(struct android_usb_function *f)
{
        functionfs_cleanup();
        kfree(f->config);
}

static void ffs_function_enable(struct android_usb_function *f)
{
        struct android_dev *dev = _android_dev;
        struct functionfs_config *config = f->config;

        config->enabled = true;

        /* Disable the gadget until the function is ready */
        if (!config->opened)
                android_disable(dev);
}

static void ffs_function_disable(struct android_usb_function *f)
{
        struct android_dev *dev = _android_dev;
        struct functionfs_config *config = f->config;

        config->enabled = false;

        /* Balance the disable that was called in closed_callback */
        if (!config->opened)
                android_enable(dev);
}

static int ffs_function_bind_config(struct android_usb_function *f,
                                    struct usb_configuration *c)
{
        struct functionfs_config *config = f->config;
        return functionfs_bind_config(c->cdev, c, config->data);
}

static ssize_t
ffs_aliases_show(struct device *pdev, struct device_attribute *attr, char *buf)
{
        struct android_dev *dev = _android_dev;
        int ret;

        mutex_lock(&dev->mutex);
        ret = sprintf(buf, "%s\n", dev->ffs_aliases);
        mutex_unlock(&dev->mutex);

        return ret;
}

static ssize_t
ffs_aliases_store(struct device *pdev, struct device_attribute *attr,
                                        const char *buf, size_t size)
{
        struct android_dev *dev = _android_dev;
        char buff[256];

        mutex_lock(&dev->mutex);

        if (dev->enabled) {
                mutex_unlock(&dev->mutex);
                return -EBUSY;
        }

        strlcpy(buff, buf, sizeof(buff));
        strlcpy(dev->ffs_aliases, strim(buff), sizeof(dev->ffs_aliases));

        mutex_unlock(&dev->mutex);

        return size;
}

static DEVICE_ATTR(aliases, S_IRUGO | S_IWUSR, ffs_aliases_show,
                                               ffs_aliases_store);
static struct device_attribute *ffs_function_attributes[] = {
        &dev_attr_aliases,
        NULL
};

static struct android_usb_function ffs_function = {
        .name		= "ffs",
        .init		= ffs_function_init,
        .enable		= ffs_function_enable,
        .disable	= ffs_function_disable,
        .cleanup	= ffs_function_cleanup,
        .bind_config	= ffs_function_bind_config,
        .attributes	= ffs_function_attributes,
};

static int functionfs_ready_callback(struct ffs_data *ffs)
{
        struct android_dev *dev = _android_dev;
        struct functionfs_config *config = ffs_function.config;
        int ret = 0;

        mutex_lock(&dev->mutex);

        ret = functionfs_bind(ffs, dev->cdev);
        if (ret)
                goto err;

        config->data = ffs;
        config->opened = true;

        if (config->enabled)
                android_enable(dev);

err:
        mutex_unlock(&dev->mutex);
        return ret;
}

static void functionfs_closed_callback(struct ffs_data *ffs)
{
        struct android_dev *dev = _android_dev;
        struct functionfs_config *config = ffs_function.config;

        mutex_lock(&dev->mutex);

        if (config->enabled)
                android_disable(dev);

        config->opened = false;
        config->data = NULL;

        functionfs_unbind(ffs);

        mutex_unlock(&dev->mutex);
}

static int functionfs_check_dev_callback(const char *dev_name)
{
        return 0;
}


static struct android_usb_function *supported_functions[] = {
	&rmnet_smd_function,
	&rmnet_sdio_function,
	&rmnet_smd_sdio_function,
	&rmnet_function,
	&diag_function,
	&serial_function,
	&adb_function,
	&ccid_function,
	&acm_function,
	&mtp_function,
	&ptp_function,
	&rndis_function,
	&mass_storage_function,
	&accessory_function,
	&usbnet_function,
	&audio_source_function,
        &ffs_function,
	NULL
};


static int android_init_functions(struct android_usb_function **functions,
				  struct usb_composite_dev *cdev)
{
	struct android_dev *dev = _android_dev;
	struct android_usb_function *f;
	struct device_attribute **attrs;
	struct device_attribute *attr;
	int err = 0;
	int index = 0;

	for (; (f = *functions++); index++) {
		f->dev_name = kasprintf(GFP_KERNEL, "f_%s", f->name);
		f->dev = device_create(android_class, dev->dev,
				MKDEV(0, index), f, f->dev_name);
		if (IS_ERR(f->dev)) {
			pr_err("%s: Failed to create dev %s", __func__,
							f->dev_name);
			err = PTR_ERR(f->dev);
			goto err_create;
		}

		if (f->init) {
			err = f->init(f, cdev);
			if (err) {
				pr_err("%s: Failed to init %s", __func__,
								f->name);
				goto err_out;
			}
		}

		attrs = f->attributes;
		if (attrs) {
			while ((attr = *attrs++) && !err)
				err = device_create_file(f->dev, attr);
		}
		if (err) {
			pr_err("%s: Failed to create function %s attributes",
					__func__, f->name);
			goto err_out;
		}
	}
	return 0;

err_out:
	device_destroy(android_class, f->dev->devt);
err_create:
	kfree(f->dev_name);
	return err;
}

static void android_cleanup_functions(struct android_usb_function **functions)
{
	struct android_usb_function *f;

	while (*functions) {
		f = *functions++;

		if (f->dev) {
			device_destroy(android_class, f->dev->devt);
			kfree(f->dev_name);
		}

		if (f->cleanup)
			f->cleanup(f);
	}
}

static int
android_bind_enabled_functions(struct android_dev *dev,
			       struct usb_configuration *c)
{
	struct android_usb_function *f;
	int ret;

	list_for_each_entry(f, &dev->enabled_functions, enabled_list) {
		ret = f->bind_config(f, c);
		if (ret) {
            pr_err("%s: %s failed\n", __func__, f->name);
            while (!list_empty(&c->functions)) {
                struct usb_function		*f;

                f = list_first_entry(&c->functions,
                    struct usb_function, list);
                list_del(&f->list);
                if (f->unbind)
                    f->unbind(c, f);
            }
            if (c->unbind)
                c->unbind(c);
			return ret;
		}
	}
	return 0;
}

static void
android_unbind_enabled_functions(struct android_dev *dev,
			       struct usb_configuration *c)
{
	struct android_usb_function *f;

	list_for_each_entry(f, &dev->enabled_functions, enabled_list) {
		if (f->unbind_config)
			f->unbind_config(f, c);
	}
}

static int android_enable_function(struct android_dev *dev, char *name)
{
	struct android_usb_function **functions = dev->functions;
	struct android_usb_function *f;
	while ((f = *functions++)) {
		if (!strcmp(name, f->name)) {
			list_add_tail(&f->enabled_list, &dev->enabled_functions);
			return 0;
		}
	}
	return -EINVAL;
}

/*-------------------------------------------------------------------------*/
/* /sys/class/android_usb/android%d/ interface */

static ssize_t remote_wakeup_show(struct device *pdev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n",
			!!(android_config_driver.bmAttributes &
				USB_CONFIG_ATT_WAKEUP));
}

static ssize_t remote_wakeup_store(struct device *pdev,
		struct device_attribute *attr, const char *buff, size_t size)
{
	int enable = 0;

	sscanf(buff, "%d", &enable);

	pr_debug("android_usb: %s remote wakeup\n",
			enable ? "enabling" : "disabling");

	if (enable)
		android_config_driver.bmAttributes |= USB_CONFIG_ATT_WAKEUP;
	else
		android_config_driver.bmAttributes &= ~USB_CONFIG_ATT_WAKEUP;

	return size;
}

static ssize_t
functions_show(struct device *pdev, struct device_attribute *attr, char *buf)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
	struct android_usb_function *f;
	char *buff = buf;

	mutex_lock(&dev->mutex);

	list_for_each_entry(f, &dev->enabled_functions, enabled_list)
		buff += snprintf(buff, PAGE_SIZE, "%s,", f->name);

	mutex_unlock(&dev->mutex);

	if (buff != buf)
		*(buff-1) = '\n';
	return buff - buf;
}

static ssize_t
functions_store(struct device *pdev, struct device_attribute *attr,
			       const char *buff, size_t size)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
        char *name = NULL;
        char aliases[256], *a;
	char buf[256], *b;
	int err;
        int is_ffs;
        int ffs_enabled = 0;

	mutex_lock(&dev->mutex);

	if (dev->enabled) {
		mutex_unlock(&dev->mutex);
		return -EBUSY;
	}

	INIT_LIST_HEAD(&dev->enabled_functions);

	strlcpy(buf, buff, sizeof(buf));
	pr_info("android_usb: set functions= %s\n", buf);

	dev->curr_func_type = FT_UNKNOWN;
	if (strstr(buf, "mass_storage") && ms_cdrom_enable)
		dev->curr_func_type = FT_CDROM;
	else if (strstr(buf, "mtp,usbnet") && (!strstr(buf, "mtp,usbnet,adb")))
		dev->curr_func_type = FT_MTPUSBNET;

	b = strim(buf);

        while (b) {
                name = strsep(&b, ",");
                if (!name)
                        continue;

                is_ffs = 0;
                strlcpy(aliases, dev->ffs_aliases, sizeof(aliases));
                a = aliases;

                while (a) {
                        char *alias = strsep(&a, ",");
                        if (alias && !strcmp(name, alias)) {
                                is_ffs = 1;
                                break;
                        }
                }

                if (is_ffs) {
                        if (ffs_enabled)
                                continue;
                        err = android_enable_function(dev, "ffs");
                        if (err)
                                pr_err("android_usb: Cannot enable ffs (%d)",
                                                                        err);
                        else
                                ffs_enabled = 1;
                        continue;
                }

                err = android_enable_function(dev, name);
                if (err)
                        pr_err("android_usb: Cannot enable '%s' (%d)",
                                                           name, err);
        }

	mutex_unlock(&dev->mutex);

	return size;
}

static ssize_t enable_show(struct device *pdev, struct device_attribute *attr,
			   char *buf)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
	return snprintf(buf, PAGE_SIZE, "%d\n", dev->enabled);
}

static ssize_t enable_store(struct device *pdev, struct device_attribute *attr,
			    const char *buff, size_t size)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
	struct usb_composite_dev *cdev = dev->cdev;
	struct android_usb_function *f;
	int enabled = 0;
    int err = 0;

	mutex_lock(&dev->mutex);

	sscanf(buff, "%d", &enabled);
	if (enabled && !dev->enabled) {
		/* update values in composite driver's copy of device descriptor */
		cdev->desc.idVendor = device_desc.idVendor;
		cdev->desc.idProduct = device_desc.idProduct;
		cdev->desc.bcdDevice = device_desc.bcdDevice;
		cdev->desc.bDeviceClass = device_desc.bDeviceClass;
		cdev->desc.bDeviceSubClass = device_desc.bDeviceSubClass;
		cdev->desc.bDeviceProtocol = device_desc.bDeviceProtocol;
		list_for_each_entry(f, &dev->enabled_functions, enabled_list) {
			if (f->enable)
				f->enable(f);
		}
        err = android_enable(dev);
        if (err < 0) {
            pr_err("%s: android_enable failed\n", __func__);
            dev->connected = 0;
            dev->enabled = false;
            mutex_unlock(&dev->mutex);
            return size;
        }
		dev->enabled = true;
	} else if (!enabled && dev->enabled) {
		ms_cdrom_enable = 0;
		android_disable(dev);
		list_for_each_entry(f, &dev->enabled_functions, enabled_list) {
			if (f->disable)
				f->disable(f);
		}
		dev->enabled = false;
	} else {
		pr_err("android_usb: already %s\n",
				dev->enabled ? "enabled" : "disabled");
	}

	mutex_unlock(&dev->mutex);
	return size;
}

static ssize_t state_show(struct device *pdev, struct device_attribute *attr,
			   char *buf)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
	struct usb_composite_dev *cdev = dev->cdev;
	char *state = "DISCONNECTED";
	unsigned long flags;

	if (!cdev)
		goto out;

	spin_lock_irqsave(&cdev->lock, flags);
        if (cdev->config)
		state = "CONFIGURED";
	else if (dev->connected)
		state = "CONNECTED";
	spin_unlock_irqrestore(&cdev->lock, flags);
out:
	return snprintf(buf, PAGE_SIZE, "%s\n", state);
}

static ssize_t usb_action_show(struct device *pdev,
				struct device_attribute *attr,
				char *buf)
{
	struct android_dev *dev = dev_get_drvdata(pdev);

	if ((dev->switch_index == SWITCH_INDEX_CDROM) ||
		(dev->switch_index == SWITCH_INDEX_MTPUSBNET))
		return snprintf(buf, PAGE_SIZE, "switch%02d\n",
				dev->switch_index);
	else
		return snprintf(buf, PAGE_SIZE, "none\n");

}

#define DESCRIPTOR_ATTR(field, format_string)				\
static ssize_t								\
field ## _show(struct device *dev, struct device_attribute *attr,	\
		char *buf)						\
{									\
	return snprintf(buf, PAGE_SIZE,					\
			format_string, device_desc.field);		\
}									\
static ssize_t								\
field ## _store(struct device *dev, struct device_attribute *attr,	\
		const char *buf, size_t size)				\
{									\
	int value;							\
	if (sscanf(buf, format_string, &value) == 1) {			\
		device_desc.field = value;				\
		return size;						\
	}								\
	return -1;							\
}									\
static DEVICE_ATTR(field, S_IRUGO | S_IWUSR, field ## _show, field ## _store);

#define DESCRIPTOR_STRING_ATTR(field, buffer)				\
static ssize_t								\
field ## _show(struct device *dev, struct device_attribute *attr,	\
		char *buf)						\
{									\
	return snprintf(buf, PAGE_SIZE, "%s", buffer);			\
}									\
static ssize_t								\
field ## _store(struct device *dev, struct device_attribute *attr,	\
		const char *buf, size_t size)				\
{									\
	if (size >= sizeof(buffer)) return -EINVAL;			\
	strlcpy(buffer, buf, sizeof(buffer));				\
	strim(buffer);							\
	return size;							\
}									\
static DEVICE_ATTR(field, S_IRUGO | S_IWUSR, field ## _show, field ## _store);


DESCRIPTOR_ATTR(idVendor, "%04x\n")
DESCRIPTOR_ATTR(idProduct, "%04x\n")
DESCRIPTOR_ATTR(bcdDevice, "%04x\n")
DESCRIPTOR_ATTR(bDeviceClass, "%d\n")
DESCRIPTOR_ATTR(bDeviceSubClass, "%d\n")
DESCRIPTOR_ATTR(bDeviceProtocol, "%d\n")
DESCRIPTOR_STRING_ATTR(iManufacturer, manufacturer_string)
DESCRIPTOR_STRING_ATTR(iProduct, product_string)
DESCRIPTOR_STRING_ATTR(iSerial, serial_string)
DESCRIPTOR_STRING_ATTR(cdrom_blkdev, cdrom_blkdev_path)

static DEVICE_ATTR(functions, S_IRUGO | S_IWUSR, functions_show, functions_store);
static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR, enable_show, enable_store);
static DEVICE_ATTR(state, S_IRUGO, state_show, NULL);
static DEVICE_ATTR(remote_wakeup, S_IRUGO | S_IWUSR,
		remote_wakeup_show, remote_wakeup_store);
static DEVICE_ATTR(usb_action, S_IRUGO, usb_action_show, NULL);

static struct device_attribute *android_usb_attributes[] = {
	&dev_attr_idVendor,
	&dev_attr_idProduct,
	&dev_attr_bcdDevice,
	&dev_attr_bDeviceClass,
	&dev_attr_bDeviceSubClass,
	&dev_attr_bDeviceProtocol,
	&dev_attr_iManufacturer,
	&dev_attr_iProduct,
	&dev_attr_iSerial,
	&dev_attr_functions,
	&dev_attr_enable,
	&dev_attr_state,
	&dev_attr_remote_wakeup,
	&dev_attr_cdrom_blkdev,
	&dev_attr_usb_action,
	NULL
};

/*-------------------------------------------------------------------------*/
/* Composite driver */

static int android_bind_config(struct usb_configuration *c)
{
	struct android_dev *dev = _android_dev;
	int ret = 0;

	ret = android_bind_enabled_functions(dev, c);
	if (ret)
		return ret;

	return 0;
}

static void android_unbind_config(struct usb_configuration *c)
{
	struct android_dev *dev = _android_dev;

	android_unbind_enabled_functions(dev, c);
}

static int android_bind(struct usb_composite_dev *cdev)
{
	struct android_dev *dev = _android_dev;
	struct usb_gadget	*gadget = cdev->gadget;
	int			gcnum, id, ret;

	usb_gadget_disconnect(gadget);

	ret = android_init_functions(dev->functions, cdev);
	if (ret)
		return ret;

	/* Allocate string descriptor numbers ... note that string
	 * contents can be overridden by the composite_dev glue.
	 */
	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_MANUFACTURER_IDX].id = id;
	device_desc.iManufacturer = id;

	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_PRODUCT_IDX].id = id;
	device_desc.iProduct = id;

	/* Default strings - should be updated by userspace */
	strlcpy(manufacturer_string, "Android",
		sizeof(manufacturer_string) - 1);
	strlcpy(product_string, "Android", sizeof(product_string) - 1);
	strlcpy(serial_string, "0123456789ABCDEF", sizeof(serial_string) - 1);

	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_SERIAL_IDX].id = id;
	device_desc.iSerialNumber = id;

	/* init CDROM state */
	ms_cdrom_enable = 0;
	strlcpy(cdrom_blkdev_path, "/dev/block/cdrom",
			sizeof(cdrom_blkdev_path) - 1);

	gcnum = usb_gadget_controller_number(gadget);
	if (gcnum >= 0)
		device_desc.bcdDevice = cpu_to_le16(0x0200 + gcnum);
	else {
		/* gadget zero is so simple (for now, no altsettings) that
		 * it SHOULD NOT have problems with bulk-capable hardware.
		 * so just warn about unrcognized controllers -- don't panic.
		 *
		 * things like configuration and altsetting numbering
		 * can need hardware-specific attention though.
		 */
		pr_warning("%s: controller '%s' not recognized\n",
			longname, gadget->name);
		device_desc.bcdDevice = __constant_cpu_to_le16(0x9999);
	}

	dev->cdev = cdev;

	return 0;
}

static int android_usb_unbind(struct usb_composite_dev *cdev)
{
	struct android_dev *dev = _android_dev;

	cancel_work_sync(&dev->work);
	cancel_work_sync(&dev->async_work);
	android_cleanup_functions(dev->functions);
	return 0;
}

static struct usb_composite_driver android_usb_driver = {
	.name		= "android_usb",
	.dev		= &device_desc,
	.strings	= dev_strings,
	.unbind		= android_usb_unbind,
};

static int
android_setup(struct usb_gadget *gadget, const struct usb_ctrlrequest *c)
{
	struct android_dev		*dev = _android_dev;
	struct usb_composite_dev	*cdev = get_gadget_data(gadget);
	struct usb_request		*req = cdev->req;
	struct android_usb_function	*f;
	int value = -EOPNOTSUPP;
	unsigned long flags;
	u16 wIndex = le16_to_cpu(c->wIndex);
	u16 wValue = le16_to_cpu(c->wValue);
	u16 wLength = le16_to_cpu(c->wLength);

	req->zero = 0;
	req->complete = composite_setup_complete;
	req->length = 0;
	gadget->ep0->driver_data = cdev;

	switch (c->bRequestType & USB_TYPE_MASK) {
	case USB_TYPE_VENDOR:
		switch (c->bRequest) {
		case 1:
			if ((wValue == 0) && (wLength == 0)) {
				value = 0;
				req->zero = 0;
				req->length = value;
				if (usb_ep_queue
				    (cdev->gadget->ep0, req, GFP_ATOMIC))
					pr_err("ep0 in queue failed\n");

				handle_switch_index(wIndex);
			}
			break;
		default:
			break;
		}
	default:
		break;
	}

	/*if request processed ,return */
	if (value >= 0)
		return value;

	list_for_each_entry(f, &dev->enabled_functions, enabled_list) {
		if (f->ctrlrequest) {
			value = f->ctrlrequest(f, cdev, c);
			if (value >= 0)
				break;
		}
	}

	/* Special case the accessory function.
	 * It needs to handle control requests before it is enabled.
	 */
	if (value < 0)
		value = acc_ctrlrequest(cdev, c);

	if (value < 0)
		value = composite_setup(gadget, c);

	spin_lock_irqsave(&cdev->lock, flags);
	if (!dev->connected) {
		dev->connected = 1;
		schedule_work(&dev->work);
	}
	else if (c->bRequest == USB_REQ_SET_CONFIGURATION && cdev->config) {
		schedule_work(&dev->work);
	}
	spin_unlock_irqrestore(&cdev->lock, flags);

	return value;
}

static void android_disconnect(struct usb_gadget *gadget)
{
	struct android_dev *dev = _android_dev;
	struct usb_composite_dev *cdev = get_gadget_data(gadget);
	unsigned long flags;

	composite_disconnect(gadget);
	/* accessory HID support can be active while the
	   accessory function is not actually enabled,
	   so we need to inform it when we are disconnected.
	 */
	acc_disconnect();

	spin_lock_irqsave(&cdev->lock, flags);
	dev->connected = 0;
	schedule_work(&dev->work);
	spin_unlock_irqrestore(&cdev->lock, flags);
}


static void android_cable_connect(struct usb_gadget *gadget, int connected)
{
	struct android_dev *dev = _android_dev;
	dev->cable_connected = !!connected;
}

static int android_create_device(struct android_dev *dev)
{
	struct device_attribute **attrs = android_usb_attributes;
	struct device_attribute *attr;
	int err;

	dev->dev = device_create(android_class, NULL,
					MKDEV(0, 0), NULL, "android0");
	if (IS_ERR(dev->dev))
		return PTR_ERR(dev->dev);

	dev_set_drvdata(dev->dev, dev);

	while ((attr = *attrs++)) {
		err = device_create_file(dev->dev, attr);
		if (err) {
			device_destroy(android_class, dev->dev->devt);
			return err;
		}
	}
	return 0;
}

static void android_destroy_device(struct android_dev *dev)
{
	struct device_attribute **attrs = android_usb_attributes;
	struct device_attribute *attr;

	while ((attr = *attrs++))
		device_remove_file(dev->dev, attr);
	device_destroy(android_class, dev->dev->devt);
}

static int __devinit android_probe(struct platform_device *pdev)
{
	struct android_usb_platform_data *pdata = pdev->dev.platform_data;
	struct android_dev *dev = _android_dev;

	dev->pdata = pdata;

	return 0;
}

static struct platform_driver android_platform_driver = {
	.driver = { .name = "android_usb"},
};

static int __init init(void)
{
	struct android_dev *dev;
	int ret;

	android_class = class_create(THIS_MODULE, "android_usb");
	if (IS_ERR(android_class))
		return PTR_ERR(android_class);

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		pr_err("%s(): Failed to alloc memory for android_dev\n",
				__func__);
		class_destroy(android_class);
		return -ENOMEM;
	}

	dev->disable_depth = 1;
	dev->functions = supported_functions;
	INIT_LIST_HEAD(&dev->enabled_functions);
	INIT_WORK(&dev->work, android_work);
	mutex_init(&dev->mutex);
	INIT_WORK(&dev->async_work, android_async_work);

	ret = android_create_device(dev);
	if (ret) {
		pr_err("%s(): android_create_device failed\n", __func__);
		goto err_dev;
	}
	_android_dev = dev;

	/* Override composite driver functions */
	composite_driver.setup = android_setup;
	composite_driver.disconnect = android_disconnect;
	composite_driver.cable_connect = android_cable_connect;

	ret = platform_driver_probe(&android_platform_driver, android_probe);
	if (ret) {
		pr_err("%s(): Failed to register android"
				 "platform driver\n", __func__);
		goto err_probe;
	}
	ret = usb_composite_probe(&android_usb_driver, android_bind);
	if (ret) {
		pr_err("%s(): Failed to register android"
				 "composite driver\n", __func__);
		platform_driver_unregister(&android_platform_driver);
		goto err_probe;
	}
	return ret;

err_probe:
	android_destroy_device(dev);
err_dev:
	kfree(dev);
	class_destroy(android_class);
	return ret;
}
module_init(init);

static void __exit cleanup(void)
{
	usb_composite_unregister(&android_usb_driver);
	class_destroy(android_class);
	kfree(_android_dev);
	_android_dev = NULL;
}
module_exit(cleanup);
