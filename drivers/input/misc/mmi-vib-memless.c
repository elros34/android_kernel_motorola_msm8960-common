#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/slab.h>


struct mmi_vib {
        struct input_dev *vib_input_dev;
        struct work_struct work;
        int magnitude;
        bool active;
};

static struct mmi_vib *mmi_vib;

extern void mmi_vibrator_enable(unsigned int magnitude);
extern void mmi_vibrator_disable(void);
extern void mmi_vibrator_set_magnitude(unsigned int magnitude);


static void mmi_vib_close(struct input_dev *dev)
{
    cancel_work_sync(&mmi_vib->work);
    if (mmi_vib->active) {
        mmi_vib->active = false;
        mmi_vibrator_disable();
    }
}

static void mmi_vib_work_handler(struct work_struct *work)
{
    if (mmi_vib->magnitude) {
        if (!mmi_vib->active) {
            mmi_vibrator_enable(mmi_vib->magnitude);
        } else {
            mmi_vibrator_set_magnitude(mmi_vib->magnitude);
        }
        mmi_vib->active = true;

    } else {
        mmi_vib->active = false;
        mmi_vibrator_disable();
    }

}

static int mmi_vib_play_effect(struct input_dev *dev, void *data,
                                  struct ff_effect *effect)
{
    mmi_vib->magnitude = effect->u.rumble.strong_magnitude >> 8;
    if (!mmi_vib->magnitude)
        mmi_vib->magnitude = effect->u.rumble.weak_magnitude >> 9;

    schedule_work(&mmi_vib->work);

    return 0;
}

static int mmi_vib_init(void)
{
    int error;
    struct input_dev *input_dev;

    input_dev = input_allocate_device();
    if (!input_dev) {
            pr_err("%s: couldn't allocate memory\n", __func__);
            error = -ENOMEM;
            goto err_free_mem;
    }

    mmi_vib = kzalloc(sizeof(mmi_vib), GFP_KERNEL);
    if (!mmi_vib) {
        error = -ENOMEM;
        goto err_free_mem;
    }

    INIT_WORK(&mmi_vib->work, mmi_vib_work_handler);
    mmi_vib->vib_input_dev = input_dev;

    input_dev->name = "mmi_vib_memless";
    input_dev->close = mmi_vib_close;
    input_set_capability(input_dev, EV_FF, FF_RUMBLE);

    error = input_ff_create_memless(input_dev, NULL, mmi_vib_play_effect);
    if (error) {
            pr_err("%s: couldn't register vibrator as FF device\n", __func__);
            goto err_free_mem;
    }

    error = input_register_device(input_dev);
    if (error) {
            pr_err("%s: failed to register device\n", __func__);
            goto err_destroy_memless;
    }

    return 0;

    err_destroy_memless:
        input_ff_destroy(input_dev);
    err_free_mem:
        input_free_device(input_dev);

    return error;
}


static int mmi_vib_memless_init(void)
{
    return mmi_vib_init();
}

static void mmi_vib_memless_exit(void)
{
    input_unregister_device(mmi_vib->vib_input_dev);
    kfree(mmi_vib);
}


module_init(mmi_vib_memless_init);
module_exit(mmi_vib_memless_exit);

MODULE_LICENSE("GPL");
