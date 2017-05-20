/*
 * arch/arm/mach-msm/board-mot-vibrator.c
 *
 * Copyright (C) 2011 Motorola Mobility Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <linux/vib-timed.h>
#include <linux/hrtimer.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/wakelock.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/of_fdt.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <mach/system.h>

#include <linux/slab.h>


#define MAX_VIBS		1
#define MAX_PWMS		9
#define MAX_VOLT		4

#define VIB_TYPE_GENENIC_ROTARY	0x00000017
#define VIB_TYPE_GENENIC_LINEAR	0x00000002

#define ACTIVE_HIGH		1
#define ACTIVE_LOW		0

#define SIGNAL_GPIO		0x1F
#define SIGNAL_PWM		0x2F

#define MIN_TIMEOUT		0
#define MAX_TIMEOUT		60000000

#define REGULATOR_NAME_SIZE	16

#define VIB_EN			"vib_en"
#define VIB_DIR			"vib_dir"

#define SIGNAL_ENABLE		0x2A
#define SIGNAL_DIRECTION	0x3A

#define GPIO_STAGE_ACTIVE	1
#define GPIO_STAGE_INACTIVE	2

#define GPIO_VIB_ENABLE 47
#define GPIO_VIB_DIRECTION 79
#define VIB_REGULATOR_NAME  "8921_l16"

/* DT Vibrator Node */
#define DT_PATH_VIB		"/System@0/Vibrator@"
#define DT_PROP_VIB_TYPE	"type"

#define MAX_FF_MAGNITUDE        0xFF

static unsigned int pwm_period_us;
module_param(pwm_period_us, uint, 0644);

static unsigned int pwm_duty_min_us;
module_param(pwm_duty_min_us, uint, 0644);

struct vib_pwm {
        unsigned int time;
        unsigned int period;
        unsigned int duty_en;
        unsigned int duty_dir;
};

struct vib_ctrl_gpio {
        struct hrtimer hrtimer;
        int stage;
        unsigned int active_us;
        unsigned int inactive_us;
};

struct vib_ctrl_pwm {
        struct omap_dm_timer *pwm_timer;
        unsigned int active_us;
        int cycles;
};

struct vib_signal;

struct vib_of_signal {
        int type;
        int active_level;
        int pwm;
        int gpio;
};

struct vib_ctrl_ops {
        int (*init)(struct vib_signal *);
        int (*configure)(struct vib_signal *, unsigned int,
                        unsigned int, unsigned int);
        int (*activate)(struct vib_signal *);
        int (*deactivate)(struct vib_signal *);
};

struct vib_signal {
        int signal_type;
        struct vib_of_signal of;
        struct vib_ctrl_pwm pwmc;
        struct vib_ctrl_gpio gpioc;
        struct vib_ctrl_ops *ops;
        const char *name;
};


struct vib_control {
        struct vib_signal vib_en;
        struct vib_signal vib_dir;
        struct vib_pwm vib_pwm[MAX_PWMS];
};

struct vib_voltage {
        unsigned int time;
        u32 min_uV;
        u32 max_uV;
};

struct vib_regulator {
        struct regulator *regulator;
        int enabled;
        u32 deferred_off;	/* in us */
        struct vib_voltage volt[MAX_VOLT];
        char name[REGULATOR_NAME_SIZE];
};

struct vibrator {
        int type;
        struct vib_regulator reg;
        struct wake_lock wakelock;
        struct vib_control ctrl;
        unsigned int min_us;
        unsigned int max_us;
};

static const char *vib_name = "vibrator";

static struct vibrator *vib;


static void vib_signal_print(struct vib_signal *vibs)
{
        struct vib_of_signal *of = &vibs->of;
        if (vibs->name)
                zprintk("%s: %x %d %d %d\n", vibs->name, of->type,
                        of->active_level, of->pwm, of->gpio);
}

static enum hrtimer_restart gpioc_hrtimer_func(struct hrtimer *hrtimer)
{
        struct vib_ctrl_gpio *gpioc = container_of(hrtimer,
                                struct vib_ctrl_gpio, hrtimer);
        struct vib_signal *vibs = container_of(gpioc,
                                struct vib_signal, gpioc);
        struct vib_of_signal *of = &vibs->of;

        if (gpioc->stage == GPIO_STAGE_ACTIVE) {
                if (vibs->signal_type == SIGNAL_ENABLE) {
                    if (gpioc->inactive_us == 0) {
                        gpio_set_value(of->gpio, of->active_level);
                        hrtimer_start(&gpioc->hrtimer,
                                ns_to_ktime((u64) gpioc->active_us
                                        * NSEC_PER_USEC),
                                HRTIMER_MODE_REL);
                    } else {
                        gpio_set_value(of->gpio, !of->active_level);
                        gpioc->stage = GPIO_STAGE_INACTIVE;
                    }
                }
                if (gpioc->inactive_us) {
                        if (vibs->signal_type == SIGNAL_DIRECTION) {
                                gpio_set_value(of->gpio,
                                                !of->active_level);
                                gpioc->stage = GPIO_STAGE_INACTIVE;
                        }
                        hrtimer_start(&gpioc->hrtimer,
                                ns_to_ktime((u64) gpioc->inactive_us
                                        * NSEC_PER_USEC),
                                HRTIMER_MODE_REL);
                }
        } else {
                if (gpioc->active_us) {
                        gpio_set_value(of->gpio, of->active_level);
                        gpioc->stage = GPIO_STAGE_ACTIVE;
                        hrtimer_start(&gpioc->hrtimer,
                                ns_to_ktime((u64) gpioc->active_us
                                        * NSEC_PER_USEC),
                                HRTIMER_MODE_REL);
                }
        }
        return HRTIMER_NORESTART;
}

static int vib_ctrl_gpio_init(struct vib_signal *vibs)
{
        struct vib_ctrl_gpio *gpioc = &vibs->gpioc;
        struct vib_of_signal *of = &vibs->of;
        int ret;

        ret = gpio_request(of->gpio, vibs->name);
        if (ret) {
                zfprintk("gpio request %d failed %d\n", of->gpio, ret);
                return ret;
        }
        ret = gpio_direction_output(of->gpio, !of->active_level);
        if (ret) {
                zfprintk("gpio %d output %d failed %d\n", of->gpio,
                        !of->active_level, ret);
                return ret;
        }

        hrtimer_init(&gpioc->hrtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
        gpioc->hrtimer.function = gpioc_hrtimer_func;
        return 0;
}

static int vib_ctrl_gpio_activate(struct vib_signal *vibs)
{
        struct vib_ctrl_gpio *gpioc = &vibs->gpioc;
        struct vib_of_signal *of = &vibs->of;
        int ret;

        dvib_tprint("g+ %s\n", vibs->name);
        if (!gpioc->active_us)
                return 0;
        gpio_set_value(of->gpio, of->active_level);
        gpioc->stage = GPIO_STAGE_ACTIVE;
        ret = hrtimer_start(&gpioc->hrtimer,
                ns_to_ktime((u64) gpioc->active_us * NSEC_PER_USEC),
                HRTIMER_MODE_REL);
        if (ret)
                dvib_tprint("started timer %p while active.\n",
                                &gpioc->hrtimer);
        return 0;
}

static int vib_ctrl_gpio_deactivate(struct vib_signal *vibs)
{
        struct vib_ctrl_gpio *gpioc = &vibs->gpioc;
        struct vib_of_signal *of = &vibs->of;
        int ret;

        ret = hrtimer_cancel(&gpioc->hrtimer);
        gpio_set_value(of->gpio, !of->active_level);
        dvib_tprint("g- %s %s\n", vibs->name, ret ? "a" : "na");

        gpioc->stage = GPIO_STAGE_INACTIVE;
        return 0;
}

static int vib_ctrl_gpio_config(struct vib_signal *vibs, unsigned int total_us,
                        unsigned int period_us, unsigned int duty_us)
{
        struct vib_ctrl_gpio *gpioc = &vibs->gpioc;
        vib_ctrl_gpio_deactivate(vibs);
        gpioc->active_us = duty_us;
        gpioc->inactive_us = period_us - duty_us;
        dvib_print("\t\t%s\tT %u A %u N %u\n", vibs->name,
                total_us, gpioc->active_us, gpioc->inactive_us);
        return 0;
}

static struct vib_ctrl_ops vib_ctrl_gpio_ops = {
        .init		= vib_ctrl_gpio_init,
        .configure	= vib_ctrl_gpio_config,
        .activate	= vib_ctrl_gpio_activate,
        .deactivate	= vib_ctrl_gpio_deactivate,
};

static int vib_signal_config(struct vib_signal *vibs, unsigned int total_us,
                unsigned int period_us, unsigned int duty_us)
{
        int ret = 0;
        if (vibs->of.type) {
                if (duty_us == period_us) {
                        period_us = total_us;
                        duty_us = total_us;
                }
                dvib_print("\t\t%s: %u %u %u\n", vibs->name, total_us,
                                period_us, duty_us);
                ret = vibs->ops->configure(vibs, total_us,
                                        period_us, duty_us);
        }
        return ret;
}

static int vib_signal_activate(struct vib_signal *vibs)
{
        int ret = 0;
        if (vibs->of.type)
                ret = vibs->ops->activate(vibs);
        return ret;
}

static int vib_signal_deactivate(struct vib_signal *vibs)
{
        int ret = 0;
        if (vibs->of.type)
                ret = vibs->ops->deactivate(vibs);
        return ret;
}

static int vibrator_regulator_init(void)
{
        struct regulator *reg;
        int ret = 0;
        if (!vib->reg.name[0])
                return 0;

        reg = regulator_get(NULL, vib->reg.name);
        if (IS_ERR(reg))
                return PTR_ERR(reg);
        vib->reg.regulator = reg;
        vib->reg.enabled = 0;

        ret = regulator_set_voltage(vib->reg.regulator,
                vib->reg.volt[0].max_uV, vib->reg.volt[0].max_uV);
        if (ret)
                dvib_print("\t\trv %d\n", ret);

        return 0;
}

static int vibrator_regulator_enable(void)
{
        int ret = 0;
        if (vib->reg.regulator && !vib->reg.enabled) {
                ret = regulator_enable(vib->reg.regulator);
                dvib_print("r+ %d\n", ret);
                vib->reg.enabled = 1;
        }
        return ret;
}

static int vibrator_regulator_disable(void)
{
        int ret = 0;
        if (vib->reg.regulator && vib->reg.enabled) {
                ret = regulator_disable(vib->reg.regulator);
                dvib_tprint("r-\n");
                vib->reg.enabled = 0;
        }
        return ret;
}


static void vibrator_dump(void)
{
        zprintk("---------\n");
        zprintk("type 0x%x %s\n", vib->type, vib->reg.name);
        zprintk("%u ~ %u us\n", vib->min_us, vib->max_us);
        vib_signal_print(&vib->ctrl.vib_en);
        vib_signal_print(&vib->ctrl.vib_dir);

        zprintk("---------\n");
}

static int vibrator_init(void)
{
        int ret;
        ret = vibrator_regulator_init();
        if (ret)
                zfprintk("regulator init %s failed %d\n",
                        vib->reg.name, ret);
        return ret;
}

static void vibrator_power_on(int magnitude)
{
        struct vib_pwm pwm;
        struct vib_of_signal *of;
        unsigned long total_us = 60000;
        int ret;
        int time_range;

        wake_lock(&vib->wakelock);

        ret = vibrator_regulator_enable();
        if (ret) {
                zfprintk("regulator enable %s failed\n",
                        vib->reg.name);
                return;
        }

        pwm.period = pwm_period_us;
        time_range = pwm_period_us - pwm_duty_min_us;
        pwm.duty_en = (magnitude * time_range) / MAX_FF_MAGNITUDE + pwm_duty_min_us;

        if (total_us < pwm.period)
                total_us = pwm.period;

        vib_signal_config(&vib->ctrl.vib_en, total_us,
                                pwm.period, pwm.duty_en);
        of = &vib->ctrl.vib_dir.of;
        gpio_set_value(of->gpio, of->active_level);
        vib_signal_activate(&vib->ctrl.vib_en);
}

static int vibrator_power_off(void)
{
        vib_signal_deactivate(&vib->ctrl.vib_en);
        vib_signal_deactivate(&vib->ctrl.vib_dir);
        vibrator_regulator_disable();
        wake_unlock(&vib->wakelock);
        return 0;
}

void mmi_vibrator_enable(unsigned int magnitude)
{
        if (magnitude > MAX_FF_MAGNITUDE)
            magnitude = MAX_FF_MAGNITUDE ;
        vibrator_power_on(magnitude);
}

EXPORT_SYMBOL(mmi_vibrator_enable);

void mmi_vibrator_disable(void)
{
        vibrator_power_off();
}
EXPORT_SYMBOL(mmi_vibrator_disable);

void mmi_vibrator_set_magnitude(unsigned int magnitude)
{
        struct vib_ctrl_gpio *gpioc = &vib->ctrl.vib_en.gpioc;
        int time_range;

        time_range = pwm_period_us - pwm_duty_min_us;
        gpioc->active_us = (magnitude * time_range) / MAX_FF_MAGNITUDE + pwm_duty_min_us;
        gpioc->inactive_us = pwm_period_us - gpioc->active_us;
}

EXPORT_SYMBOL(mmi_vibrator_set_magnitude);

static int vib_signal_init(struct vib_signal *vibs)
{
        struct vib_of_signal *of = &vibs->of;
        int ret;

        switch (of->type) {
        case SIGNAL_GPIO:
                ret = vib_ctrl_gpio_init(vibs);
                vibs->ops = &vib_ctrl_gpio_ops;
                break;
        default:
                zfprintk("%s unknown signal type %d\n", vibs->name, of->type);
                of->type = 0;
                ret = -1;
        }
        return ret;
}

static int vib_of_init_default(struct vibrator *vib, int vib_nr)
{
        vib->type = VIB_TYPE_GENENIC_ROTARY;
        vib->ctrl.vib_en.name = VIB_EN;
        vib->ctrl.vib_en.signal_type = SIGNAL_ENABLE;
        vib->ctrl.vib_en.of.type = SIGNAL_GPIO;
        vib->ctrl.vib_en.of.active_level = ACTIVE_HIGH;
        vib->ctrl.vib_en.of.gpio = GPIO_VIB_ENABLE;

        if (vib_signal_init(&vib->ctrl.vib_en)) {
                zfprintk("vib_en init failed\n");
                return -ENODEV;
                }

        vib->ctrl.vib_dir.name = VIB_DIR;
        vib->ctrl.vib_dir.signal_type = SIGNAL_DIRECTION;
        vib->ctrl.vib_dir.of.type = SIGNAL_GPIO;
        vib->ctrl.vib_dir.of.active_level = ACTIVE_HIGH;
        vib->ctrl.vib_dir.of.gpio = GPIO_VIB_DIRECTION;

        if (vib_signal_init(&vib->ctrl.vib_dir)) {
                zfprintk("vib_dir init failed\n");
                return -ENODEV;
        }

        strcpy(vib->reg.name,VIB_REGULATOR_NAME);
        vib->reg.volt[0].time =  MAX_TIMEOUT;
        vib->reg.volt[0].min_uV = 2800000;
        vib->reg.volt[0].max_uV = 2800000;

        vib->min_us = MIN_TIMEOUT;
        vib->max_us = MAX_TIMEOUT;
        return 0;
}

static int vib_of_init(struct vibrator *vib, int vib_nr)
{
        struct device_node *node;
        const void *prop = NULL;
        int len = 0;
        char dt_path_vib[sizeof(DT_PATH_VIB) + 3];

        snprintf(dt_path_vib, sizeof(DT_PATH_VIB) + 2, "%s%1d",
                DT_PATH_VIB, vib_nr % MAX_VIBS);
        node = of_find_node_by_path(dt_path_vib);
        if (!node)
                return -ENODEV;

        prop = of_get_property(node, DT_PROP_VIB_TYPE, &len);
        if (prop && len)
                vib->type = *((int *)prop);
        else
                return -ENODEV;

        if ((vib->type != VIB_TYPE_GENENIC_ROTARY)
                && (vib->type != VIB_TYPE_GENENIC_LINEAR))
                return -ENODEV;

        prop = of_get_property(node, VIB_EN, &len);
        if (prop && len) {
                vib->ctrl.vib_en.of = *((struct vib_of_signal *)prop);
                vib->ctrl.vib_en.name = VIB_EN;
                vib->ctrl.vib_en.signal_type = SIGNAL_ENABLE;
                if (vib_signal_init(&vib->ctrl.vib_en)) {
                        zfprintk("vib_en init failed\n");
                        return -ENODEV;
                }
        } else {
                zfprintk("vib_en not found in %s\n", dt_path_vib);
                return -ENODEV;
        }

        prop = of_get_property(node, VIB_DIR, &len);
        if (prop && len) {
                vib->ctrl.vib_dir.of = *((struct vib_of_signal *)prop);
                vib->ctrl.vib_dir.name = VIB_DIR;
                vib->ctrl.vib_dir.signal_type = SIGNAL_DIRECTION;
                if (vib_signal_init(&vib->ctrl.vib_dir)) {
                        zfprintk("vib_dir init failed\n");
                        return -ENODEV;
                }
        } else {
                if (vib->type == VIB_TYPE_GENENIC_LINEAR) {
                        zfprintk("vib_dir not found in %s\n", dt_path_vib);
                        return -ENODEV;
                }
        }

        prop = of_get_property(node, "regulator", &len);
        if (prop && len) {
                strncpy(vib->reg.name, (char *)prop,
                                REGULATOR_NAME_SIZE - 1);
                vib->reg.name[REGULATOR_NAME_SIZE - 1] = '\0';

                prop = of_get_property(node, "deferred_off", &len);
                if (prop && len) {
                        vib->reg.deferred_off = *(u32 *)prop;
                        zfprintk("deferred_off %u\n", vib->reg.deferred_off);
                }
                vib->reg.volt[0].time =  MAX_TIMEOUT;
                vib->reg.volt[0].min_uV = 2800000;
                vib->reg.volt[0].max_uV = 2800000;

                prop = of_get_property(node, "voltage", &len);
                if (prop && len) {
                        int i, j = len / sizeof(struct vib_voltage);
                        dprintk("voltage len %d size %d\n", len,
                                len/sizeof(struct vib_voltage));
                        if (j > MAX_VOLT)
                                j = MAX_VOLT;
                        for (i = 0; i < j; i++)
                                vib->reg.volt[i] =
                                        *(((struct vib_voltage *)prop) + i);
                }
        }

        prop = of_get_property(node, "min", &len);
        if (prop && len)
                vib->min_us = *((unsigned int *)prop);
        else
                vib->min_us = MIN_TIMEOUT;

        prop = of_get_property(node, "max", &len);
        if (prop && len)
                vib->max_us = *((unsigned int *)prop);
        else
                vib->max_us = MAX_TIMEOUT;

        of_node_put(node);
        return 0;
}


void __init mmi_vibrator_init(void)
{
        vib = kzalloc(sizeof(*vib), GFP_KERNEL);
        if (!vib)
            return;

        if (vib_of_init(vib, 0)) {
                zfprintk("DT vibrator settings not found,"
                                        " using defaults\n");
                return;
                if (!vib_of_init_default(vib, 0))
                        return;
        }

        wake_lock_init(&vib->wakelock, WAKE_LOCK_SUSPEND, vib_name);
        dprintk("type 0x%x\n", vib->type);

        vibrator_init();
        vibrator_dump();
        pwm_period_us = 10000; // 10ms
        pwm_duty_min_us = 2100;

        return;
}

