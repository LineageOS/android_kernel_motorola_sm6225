#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <net/sock.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>

#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif

#include "fpsensor_spi_tee.h"
#include "fpsensor_wakelock.h"
#define FPSENSOR_FB_DRM         1
#define FPSENSOR_FB_OLD         2

#define FPSENSOR_WAKEUP_SOURCE  1
#define FPSENSOR_WAKEUP_OLD     2

#define FPSENSOR_SPI_VERSION              "fpsensor_spi_tee_qcomm_v1.23.1"
#ifdef SUPPORT_POWER_GPIO
#define FPSENSOR_USE_QCOM_POWER_GPIO     1
#else
#define FPSENSOR_USE_QCOM_POWER_GPIO     0
#endif
#define FPSENSOR_WAKEUP_TYPE              FPSENSOR_WAKEUP_SOURCE
#define FP_NOTIFY                         1
#define FP_NOTIFY_TYPE                    FPSENSOR_FB_DRM

#if FP_NOTIFY
#if FP_NOTIFY_TYPE == FPSENSOR_FB_DRM
#include <linux/msm_drm_notify.h>
#define FP_NOTIFY_ON                            MSM_DRM_BLANK_UNBLANK
#define FP_NOTIFY_OFF                           MSM_DRM_BLANK_POWERDOWN
#define FP_NOTIFY_EVENT_BLANK                   MSM_DRM_EARLY_EVENT_BLANK    //MSM_DRM_EVENT_BLANK
#define fpsensor_fb_register_client(client)     msm_drm_register_client(client);
#define fpsensor_fb_unregister_client(client)   msm_drm_unregister_client(client);
#else
#define FP_NOTIFY_ON                            FB_BLANK_UNBLANK
#define FP_NOTIFY_OFF                           FB_BLANK_POWERDOWN
#define FP_NOTIFY_EVENT_BLANK                   FB_EVENT_BLANK
#define fpsensor_fb_register_client(client)     fb_register_client(client);
#define fpsensor_fb_unregister_client(client)   fb_unregister_client(client)
#endif
#endif

/* debug log setting */
static u8 fpsensor_debug_level = INFO_LOG;
/* global variables */
static fpsensor_data_t *g_fpsensor = NULL;
uint32_t g_cmd_sn = 0;

#if FPSENSOR_WAKEUP_TYPE == FPSENSOR_WAKEUP_SOURCE
#include <linux/ktime.h>
#include <linux/device.h>
struct wakeup_source g_ttw_wl;
#else
#include <linux/wakelock.h>
struct wake_lock g_ttw_wl;
#endif

/* -------------------------------------------------------------------- */
/* fingerprint chip hardware configuration                              */
/* -------------------------------------------------------------------- */
static void fpsensor_gpio_free(fpsensor_data_t *fpsensor)
{
    struct device *dev = &fpsensor->spi->dev;

    if (fpsensor->irq_gpio != 0 ) {
        devm_gpio_free(dev, fpsensor->irq_gpio);
        fpsensor->irq_gpio = 0;
    }
    if (fpsensor->reset_gpio != 0) {
        devm_gpio_free(dev, fpsensor->reset_gpio);
        fpsensor->reset_gpio = 0;
    }
#if FPSENSOR_USE_QCOM_POWER_GPIO
    if (fpsensor->power_gpio != 0) {
        devm_gpio_free(dev, fpsensor->power_gpio);
        fpsensor->power_gpio = 0;
    }
#endif
}

#if FPSENSOR_USE_QCOM_POWER_GPIO
static void fpsensor_power_on(fpsensor_data_t *fpsensor)
{
//    struct device *dev = &fpsensor->spi->dev;

    fpsensor_debug(ERR_LOG, "fpsensor power_gpio=%d\n", fpsensor->power_gpio);
    if (fpsensor->power_gpio != 0) {
      // set power on
      gpio_direction_output(fpsensor->power_gpio, 1);
      gpio_set_value(fpsensor->power_gpio, 1);
      fpsensor_debug(ERR_LOG, "fpsensor fingerprint power on\n");
    }
	if (fpsensor->reset_gpio != 0) {
      // set reset_gpio high
      gpio_direction_output(fpsensor->reset_gpio, 1);
      gpio_set_value(fpsensor->reset_gpio, 1);
      fpsensor_debug(ERR_LOG, "fpsensor fingerprint reset_gpio high\n");
    }
}
static void fpsensor_power_off(fpsensor_data_t *fpsensor)
{
//    struct device *dev = &fpsensor->spi->dev;

    fpsensor_debug(ERR_LOG, "fpsensor power_gpio=%d\n", fpsensor->power_gpio);
    if (fpsensor->power_gpio != 0) {
      // set power off
      gpio_direction_output(fpsensor->power_gpio, 0);
      gpio_set_value(fpsensor->power_gpio, 0);
      fpsensor_debug(ERR_LOG, "fpsensor fingerprint power off\n");
    }
	if (fpsensor->reset_gpio != 0) {
      // set reset_gpio low
      gpio_direction_output(fpsensor->reset_gpio, 0);
      gpio_set_value(fpsensor->reset_gpio, 0);
      fpsensor_debug(ERR_LOG, "fpsensor fingerprint reset_gpio low\n");
    }
}
#endif	

static void fpsensor_irq_gpio_cfg(fpsensor_data_t *fpsensor)
{
    int error = 0;

    error = gpio_direction_input(fpsensor->irq_gpio);
    if (error) {
        fpsensor_debug(ERR_LOG, "setup fpsensor irq gpio for input failed!error[%d]\n", error);
        return ;
    }

    fpsensor->irq = gpio_to_irq(fpsensor->irq_gpio);
    fpsensor_debug(DEBUG_LOG, "fpsensor irq number[%d]\n", fpsensor->irq);
    if (fpsensor->irq <= 0) {
        fpsensor_debug(ERR_LOG, "fpsensor irq gpio to irq failed!\n");
        return ;
    }

    return;
}

static int fpsensor_request_named_gpio(fpsensor_data_t *fpsensor_dev, const char *label, int *gpio)
{
    struct device *dev = &fpsensor_dev->spi->dev;
    struct device_node *np = dev->of_node;
    int ret = of_get_named_gpio(np, label, 0);

    if (ret < 0) {
        fpsensor_debug(ERR_LOG, "failed to get '%s'\n", label);
        return ret;
    }
    *gpio = ret;
    ret = devm_gpio_request(dev, *gpio, label);
    if (ret) {
        fpsensor_debug(ERR_LOG, "failed to request gpio %d\n", *gpio);
        return ret;
    }

    fpsensor_debug(ERR_LOG, "%s %d\n", label, *gpio);
    return ret;
}

/* delay us after reset */
static void fpsensor_hw_reset(int delay)
{
    FUNC_ENTRY();
    gpio_set_value(g_fpsensor->reset_gpio, 1);

    udelay(100);
    gpio_set_value(g_fpsensor->reset_gpio, 0);

    udelay(1000);
    gpio_set_value(g_fpsensor->reset_gpio, 1);

    if (delay) {
        udelay(delay);
    }

    FUNC_EXIT();
    return;
}

static int fpsensor_get_gpio_dts_info(fpsensor_data_t *fpsensor)
{
    int ret = 0;

    FUNC_ENTRY();
    // get interrupt gpio resource
    ret = fpsensor_request_named_gpio(fpsensor, "fp-gpio-int", &fpsensor->irq_gpio);
    if (ret) {
        fpsensor_debug(ERR_LOG, "Failed to request irq GPIO. ret = %d\n", ret);
        return -1;
    }

    // get reest gpio resourece
    ret = fpsensor_request_named_gpio(fpsensor, "fp-gpio-reset", &fpsensor->reset_gpio);
    if (ret) {
        fpsensor_debug(ERR_LOG, "Failed to request reset GPIO. ret = %d\n", ret);
        return -1;
    }
#if FPSENSOR_USE_QCOM_POWER_GPIO
    // get power gpio resourece
    ret = fpsensor_request_named_gpio(fpsensor, "fp-gpio-power", &fpsensor->power_gpio);
    if (ret) {
        fpsensor_debug(ERR_LOG, "Failed to request power GPIO. ret = %d\n", ret);
        return -1;
    }
    // set power direction output
    gpio_direction_output(fpsensor->power_gpio, 1);
    gpio_set_value(fpsensor->power_gpio, 1);
#endif
    // set reset direction output
#if FPSENSOR_USE_QCOM_POWER_GPIO
    gpio_direction_output(fpsensor->reset_gpio, 1);
    gpio_set_value(g_fpsensor->reset_gpio, 1);
    mdelay(4);
    //fpsensor_hw_reset(1250);
#else
    // set reset direction output
    gpio_direction_output(fpsensor->reset_gpio, 1);
    fpsensor_hw_reset(1250);
#endif
    return ret;
}

static void setRcvIRQ(int val)
{
    fpsensor_data_t *fpsensor_dev = g_fpsensor;
    fpsensor_dev->RcvIRQ = val;
}

static void fpsensor_enable_irq(fpsensor_data_t *fpsensor_dev)
{
    FUNC_ENTRY();
    setRcvIRQ(0);
    /* Request that the interrupt should be wakeable */
    if (fpsensor_dev->irq_enabled == 0) {
        enable_irq(fpsensor_dev->irq);
        fpsensor_dev->irq_enabled = 1;
    }
    FUNC_EXIT();
    return;
}

static void fpsensor_disable_irq(fpsensor_data_t *fpsensor_dev)
{
    FUNC_ENTRY();

    if (0 == fpsensor_dev->device_available) {
        fpsensor_debug(ERR_LOG, "%s, devices not available\n", __func__);
        goto out;
    }

    if (0 == fpsensor_dev->irq_enabled) {
        fpsensor_debug(ERR_LOG, "%s, irq already disabled\n", __func__);
        goto out;
    }

    if (fpsensor_dev->irq) {
        disable_irq_nosync(fpsensor_dev->irq);
        fpsensor_debug(DEBUG_LOG, "%s disable interrupt!\n", __func__);
    }
    fpsensor_dev->irq_enabled = 0;

out:
    setRcvIRQ(0);
    FUNC_EXIT();
    return;
}

static irqreturn_t fpsensor_irq(int irq, void *handle)
{
    fpsensor_data_t *fpsensor_dev = (fpsensor_data_t *)handle;

    /* Make sure 'wakeup_enabled' is updated before using it
    ** since this is interrupt context (other thread...) */
    smp_rmb();
#if FPSENSOR_WAKEUP_TYPE == FPSENSOR_WAKEUP_SOURCE
    __pm_wakeup_event(&g_ttw_wl, 1000);
#else
    wake_lock_timeout(&g_ttw_wl, msecs_to_jiffies(1000));
#endif
    setRcvIRQ(1);
    wake_up_interruptible(&fpsensor_dev->wq_irq_return);

    return IRQ_HANDLED;
}

// release and cleanup fpsensor char device
static void fpsensor_dev_cleanup(fpsensor_data_t *fpsensor)
{
    FUNC_ENTRY();

    if (fpsensor->class == NULL) {
        fpsensor_debug(INFO_LOG, "Cleanup with fpsensor = NULL\n");
        goto out;
    }
    cdev_del(&fpsensor->cdev);
    unregister_chrdev_region(fpsensor->devno, FPSENSOR_NR_DEVS);
    device_destroy(fpsensor->class, fpsensor->devno);
    class_destroy(fpsensor->class);
    fpsensor->class = NULL;

out:
    FUNC_EXIT();
}

static long fpsensor_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    fpsensor_data_t *fpsensor_dev = NULL;
    int retval = 0;
    uint32_t val = 0;
    int irqf;

    fpsensor_debug(INFO_LOG, "[rickon]: fpsensor ioctl cmd : 0x%x \n", cmd );
    fpsensor_dev = (fpsensor_data_t *)filp->private_data;
    fpsensor_dev->cancel = 0 ;
    switch (cmd) {
    case FPSENSOR_IOC_INIT:
        fpsensor_debug(INFO_LOG, "%s: fpsensor init started======\n", __func__);
        retval = fpsensor_get_gpio_dts_info(fpsensor_dev);
        if (retval) {
            break;
        }
        fpsensor_irq_gpio_cfg(fpsensor_dev);
        //regist irq
        irqf = IRQF_TRIGGER_RISING | IRQF_ONESHOT;
        retval = devm_request_threaded_irq(&g_fpsensor->spi->dev, g_fpsensor->irq, fpsensor_irq,
                                           NULL, irqf, dev_name(&g_fpsensor->spi->dev), g_fpsensor);
        if (retval == 0) {
            fpsensor_debug(INFO_LOG, " irq thread reqquest success!\n");
        } else {
            fpsensor_debug(INFO_LOG, " irq thread request failed , retval =%d \n", retval);
            break;
        }
        enable_irq_wake(g_fpsensor->irq);
        fpsensor_dev->device_available = 1;
        // fix Unbalanced enable for IRQ, disable irq at first
        fpsensor_dev->irq_enabled = 1;
        fpsensor_disable_irq(fpsensor_dev);
        fpsensor_debug(INFO_LOG, "%s: fpsensor init finished======\n", __func__);
        break;

    case FPSENSOR_IOC_EXIT:
        if(fpsensor_dev->device_available) {
            fpsensor_disable_irq(fpsensor_dev);
            if (fpsensor_dev->irq) {
                devm_free_irq(&g_fpsensor->spi->dev, fpsensor_dev->irq, fpsensor_dev);
                fpsensor_dev->irq_enabled = 0;
            }
            fpsensor_dev->device_available = 0;
            fpsensor_gpio_free(fpsensor_dev);
            fpsensor_debug(INFO_LOG, "%s: fpsensor exit finished======\n", __func__);
        } else {
            fpsensor_debug(INFO_LOG, "%s: IOC_EXIT with device_available = 0 ======\n", __func__);
        }
        break;

    case FPSENSOR_IOC_RESET:
        fpsensor_debug(INFO_LOG, "%s: chip reset command\n", __func__);
#if FPSENSOR_USE_QCOM_POWER_GPIO
        fpsensor_hw_reset(4000);
#else
        fpsensor_hw_reset(1250);
#endif
        break;

    case FPSENSOR_IOC_ENABLE_IRQ:
        fpsensor_debug(INFO_LOG, "%s: chip ENable IRQ command\n", __func__);
        fpsensor_enable_irq(fpsensor_dev);
        break;

    case FPSENSOR_IOC_DISABLE_IRQ:
        fpsensor_debug(INFO_LOG, "%s: chip disable IRQ command\n", __func__);
        fpsensor_disable_irq(fpsensor_dev);
        break;
    case FPSENSOR_IOC_GET_INT_VAL:
        val = gpio_get_value(fpsensor_dev->irq_gpio);
        if (copy_to_user((void __user *)arg, (void *)&val, sizeof(uint32_t))) {
            fpsensor_debug(ERR_LOG, "Failed to copy data to user\n");
            retval = -EFAULT;
            break;
        }
        retval = 0;
        break;
    case FPSENSOR_IOC_ENABLE_SPI_CLK:
        fpsensor_debug(INFO_LOG, "%s: ENABLE_SPI_CLK ======\n", __func__);
        break;
    case FPSENSOR_IOC_DISABLE_SPI_CLK:
        fpsensor_debug(INFO_LOG, "%s: DISABLE_SPI_CLK ======\n", __func__);
        break;
    case FPSENSOR_IOC_ENABLE_POWER:
        fpsensor_debug(INFO_LOG, "%s: FPSENSOR_IOC_ENABLE_POWER ======\n", __func__);
#if FPSENSOR_USE_QCOM_POWER_GPIO
	fpsensor_power_on(fpsensor_dev);
#endif
        break;
    case FPSENSOR_IOC_DISABLE_POWER:
        fpsensor_debug(INFO_LOG, "%s: FPSENSOR_IOC_DISABLE_POWER ======\n", __func__);
#if FPSENSOR_USE_QCOM_POWER_GPIO
	fpsensor_power_off(fpsensor_dev);
#endif
        break;
    case FPSENSOR_IOC_REMOVE:
        if(fpsensor_dev->device_available) {
            fpsensor_disable_irq(fpsensor_dev);
            if (fpsensor_dev->irq) {
                devm_free_irq(&g_fpsensor->spi->dev, fpsensor_dev->irq, fpsensor_dev);
                fpsensor_dev->irq_enabled = 0;
            }
            fpsensor_gpio_free(fpsensor_dev);
            fpsensor_dev->device_available = 0;
        } else {
            fpsensor_debug(INFO_LOG, "%s: IOC_REMOVE with device_available = 0 ======\n", __func__);
        }
        fpsensor_dev_cleanup(fpsensor_dev);
#if FP_NOTIFY
        fpsensor_fb_unregister_client(&fpsensor_dev->notifier);
#endif
        fpsensor_dev->free_flag = 1;
        fpsensor_debug(INFO_LOG, "%s remove finished\n", __func__);
        break;
    case FPSENSOR_IOC_CANCEL_WAIT:
        fpsensor_debug(INFO_LOG, "%s: FPSENSOR CANCEL WAIT\n", __func__);
        wake_up_interruptible(&fpsensor_dev->wq_irq_return);
        fpsensor_dev->cancel = 1;
        break;
#if FP_NOTIFY
    case FPSENSOR_IOC_GET_FP_STATUS :
        val = fpsensor_dev->fb_status;
        fpsensor_debug(INFO_LOG, "%s: FPSENSOR_IOC_GET_FP_STATUS  %d \n",__func__, fpsensor_dev->fb_status);
        if (copy_to_user((void __user *)arg, (void *)&val, sizeof(uint32_t))) {
            fpsensor_debug(ERR_LOG, "Failed to copy data to user\n");
            retval = -EFAULT;
            break;
        }
        retval = 0;
        break;
#endif
    case FPSENSOR_IOC_ENABLE_REPORT_BLANKON:
        if (copy_from_user(&val, (void __user *)arg, sizeof(uint32_t))) {
            retval = -EFAULT;
            break;
        }
        fpsensor_dev->enable_report_blankon = val;
        fpsensor_debug(INFO_LOG, "%s: FPSENSOR_IOC_ENABLE_REPORT_BLANKON: %d\n", __func__, val);
        break;
    case FPSENSOR_IOC_UPDATE_DRIVER_SN:
        if (copy_from_user(&g_cmd_sn, (void __user *)arg, sizeof(uint32_t))) {
            fpsensor_debug(ERR_LOG, "Failed to copy g_cmd_sn from user to kernel\n");
            retval = -EFAULT;
            break;
        }
        //fpsensor_debug(INFO_LOG, "%s: FPSENSOR_IOC_UPDATE_DRIVER_SN: %d\n", __func__, g_cmd_sn);
        break;
    default:
        fpsensor_debug(ERR_LOG, "fpsensor doesn't support this command(0x%x)\n", cmd);
        break;
    }

    //FUNC_EXIT();
    return retval;
}

static long fpsensor_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    return fpsensor_ioctl(filp, cmd, (unsigned long)(arg));
}

static unsigned int fpsensor_poll(struct file *filp, struct poll_table_struct *wait)
{
    unsigned int ret = 0;

    ret |= POLLIN;
    poll_wait(filp, &g_fpsensor->wq_irq_return, wait);
    if (g_fpsensor->cancel == 1) {
        fpsensor_debug(ERR_LOG, " cancle\n");
        ret =  POLLERR;
        g_fpsensor->cancel = 0;
        return ret;
    }

    if ( g_fpsensor->RcvIRQ) {
        if (g_fpsensor->RcvIRQ == 2) {
            fpsensor_debug(ERR_LOG, " get fp on notify\n");
            ret |= POLLHUP;
        } else {
            fpsensor_debug(ERR_LOG, " get irq\n");
            ret |= POLLRDNORM;
        }
    } else {
        ret = 0;
    }
    return ret;
}

static int fpsensor_open(struct inode *inode, struct file *filp)
{
    fpsensor_data_t *fpsensor_dev;

    FUNC_ENTRY();
    fpsensor_dev = container_of(inode->i_cdev, fpsensor_data_t, cdev);
    fpsensor_dev->users++;
    fpsensor_dev->device_available = 1;
    filp->private_data = fpsensor_dev;
    FUNC_EXIT();
    return 0;
}

static int fpsensor_release(struct inode *inode, struct file *filp)
{
    fpsensor_data_t *fpsensor_dev;
    int    status = 0;

    FUNC_ENTRY();
    fpsensor_dev = filp->private_data;
    filp->private_data = NULL;

    /*last close??*/
    fpsensor_dev->users--;
    if (fpsensor_dev->users <= 0) {
        fpsensor_debug(INFO_LOG, "%s, disble_irq. irq = %d\n", __func__, fpsensor_dev->irq);
        fpsensor_disable_irq(fpsensor_dev);
    }
    fpsensor_dev->device_available = 0;
    FUNC_EXIT();
    return status;
}

static ssize_t fpsensor_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
    fpsensor_debug(ERR_LOG, "Not support read opertion in TEE version\n");
    return -EFAULT;
}

static ssize_t fpsensor_write(struct file *filp, const char __user *buf, size_t count,
                              loff_t *f_pos)
{
    fpsensor_debug(ERR_LOG, "Not support write opertion in TEE version\n");
    return -EFAULT;
}

static const struct file_operations fpsensor_fops = {
    .owner          = THIS_MODULE,
    .write          = fpsensor_write,
    .read           = fpsensor_read,
    .unlocked_ioctl = fpsensor_ioctl,
    .compat_ioctl   = fpsensor_compat_ioctl,
    .open           = fpsensor_open,
    .release        = fpsensor_release,
    .poll           = fpsensor_poll,

};

// create and register a char device for fpsensor
static int fpsensor_dev_setup(fpsensor_data_t *fpsensor)
{
    int ret = 0;
    dev_t dev_no = 0;
    struct device *dev = NULL;
    int fpsensor_dev_major = FPSENSOR_DEV_MAJOR;
    int fpsensor_dev_minor = 0;

    FUNC_ENTRY();

    if (fpsensor_dev_major) {
        dev_no = MKDEV(fpsensor_dev_major, fpsensor_dev_minor);
        ret = register_chrdev_region(dev_no, FPSENSOR_NR_DEVS, FPSENSOR_DEV_NAME);
    } else {
        ret = alloc_chrdev_region(&dev_no, fpsensor_dev_minor, FPSENSOR_NR_DEVS, FPSENSOR_DEV_NAME);
        fpsensor_dev_major = MAJOR(dev_no);
        fpsensor_dev_minor = MINOR(dev_no);
        fpsensor_debug(INFO_LOG, "fpsensor device major is %d, minor is %d\n",
                       fpsensor_dev_major, fpsensor_dev_minor);
    }

    if (ret < 0) {
        fpsensor_debug(ERR_LOG, "can not get device major number %d\n", fpsensor_dev_major);
        goto out;
    }

    cdev_init(&fpsensor->cdev, &fpsensor_fops);
    fpsensor->cdev.owner = THIS_MODULE;
    fpsensor->cdev.ops   = &fpsensor_fops;
    fpsensor->devno      = dev_no;
    ret = cdev_add(&fpsensor->cdev, dev_no, FPSENSOR_NR_DEVS);
    if (ret) {
        fpsensor_debug(ERR_LOG, "add char dev for fpsensor failed\n");
        goto release_region;
    }

    fpsensor->class = class_create(THIS_MODULE, FPSENSOR_CLASS_NAME);
    if (IS_ERR(fpsensor->class)) {
        fpsensor_debug(ERR_LOG, "create fpsensor class failed\n");
        ret = PTR_ERR(fpsensor->class);
        goto release_cdev;
    }

    dev = device_create(fpsensor->class, &fpsensor->spi->dev, dev_no, fpsensor, FPSENSOR_DEV_NAME);
    if (IS_ERR(dev)) {
        fpsensor_debug(ERR_LOG, "create device for fpsensor failed\n");
        ret = PTR_ERR(dev);
        goto release_class;
    }
    FUNC_EXIT();
    return ret;

release_class:
    class_destroy(fpsensor->class);
    fpsensor->class = NULL;
release_cdev:
    cdev_del(&fpsensor->cdev);
release_region:
    unregister_chrdev_region(dev_no, FPSENSOR_NR_DEVS);
out:
    FUNC_EXIT();
    return ret;
}

#if FP_NOTIFY
static int fpsensor_fb_notifier_callback(struct notifier_block* self, unsigned long event, void* data)
{
    int retval = 0;
    static char screen_status[64] = { '\0' };
    struct fb_event* evdata = data;
    unsigned int blank;
    fpsensor_data_t *fpsensor_dev = g_fpsensor;

    fpsensor_debug(INFO_LOG,"%s enter, event: 0x%x\n", __func__, (unsigned)event);
    if (event != FP_NOTIFY_EVENT_BLANK) {
        return 0;
    }

    blank = *(int*)evdata->data;
    fpsensor_debug(INFO_LOG,"%s enter, blank=0x%x\n", __func__, blank);

    switch (blank) {
    case FP_NOTIFY_ON:
        fpsensor_debug(INFO_LOG,"%s: lcd on notify\n", __func__);
        sprintf(screen_status, "SCREEN_STATUS=%s", "ON");
        fpsensor_dev->fb_status = 1;
        if( fpsensor_dev->enable_report_blankon) {
            fpsensor_dev->RcvIRQ = 2;
            wake_up_interruptible(&fpsensor_dev->wq_irq_return);
        }
        break;

    case FP_NOTIFY_OFF:
        fpsensor_debug(INFO_LOG,"%s: lcd off notify\n", __func__);
        sprintf(screen_status, "SCREEN_STATUS=%s", "OFF");
        fpsensor_dev->fb_status = 0;
        break;

    default:
        fpsensor_debug(INFO_LOG,"%s: other notifier, ignore\n", __func__);
        break;
    }

    fpsensor_debug(INFO_LOG,"%s %s leave.\n", screen_status, __func__);
    return retval;
}
#endif

static int fpsensor_probe(struct platform_device *pdev)
{
    int status = 0;
    fpsensor_data_t *fpsensor_dev = NULL;

    FUNC_ENTRY();

    /* Allocate driver data */
    fpsensor_dev = kzalloc(sizeof(*fpsensor_dev), GFP_KERNEL);
    if (!fpsensor_dev) {
        status = -ENOMEM;
        fpsensor_debug(ERR_LOG, "%s, Failed to alloc memory for fpsensor device.\n", __func__);
        goto out;
    }

    /* Initialize the driver data */
    g_fpsensor = fpsensor_dev;
    fpsensor_dev->spi               = pdev ;
    fpsensor_dev->device_available  = 0;
    fpsensor_dev->users             = 0;
    fpsensor_dev->irq               = 0;
    fpsensor_dev->power_gpio        = 0;
    fpsensor_dev->reset_gpio        = 0;
    fpsensor_dev->irq_gpio          = 0;
    fpsensor_dev->irq_enabled       = 0;
    fpsensor_dev->free_flag         = 0;
    fpsensor_dev->fb_status         = 1;
    /* setup a char device for fpsensor */
    status = fpsensor_dev_setup(fpsensor_dev);
    if (status) {
        fpsensor_debug(ERR_LOG, "fpsensor setup char device failed, %d", status);
        goto release_drv_data;
    }
    init_waitqueue_head(&fpsensor_dev->wq_irq_return);
#if FPSENSOR_WAKEUP_TYPE == FPSENSOR_WAKEUP_SOURCE
    wakeup_source_init_internal(&g_ttw_wl, "fpsensor_ttw_wl");
#else
    wake_lock_init(&g_ttw_wl, WAKE_LOCK_SUSPEND, "fpsensor_ttw_wl");
#endif
    fpsensor_dev->device_available = 1;
#if FP_NOTIFY
    fpsensor_dev->notifier.notifier_call = fpsensor_fb_notifier_callback;
    fpsensor_fb_register_client(&fpsensor_dev->notifier);
#endif

    fpsensor_debug(INFO_LOG, "%s finished, driver version: %s\n", __func__, FPSENSOR_SPI_VERSION);
    goto out;

release_drv_data:
    kfree(fpsensor_dev);
    fpsensor_dev = NULL;
out:
    FUNC_EXIT();
    return status;
}

static int fpsensor_remove(struct platform_device *pdev)
{
    fpsensor_data_t *fpsensor_dev = g_fpsensor;
    int ret = 0;

    FUNC_ENTRY();

    if (g_fpsensor == NULL) {
        fpsensor_debug(ERR_LOG, "%s with g_fpsensor = NULL\n", __func__);
        ret = -EINVAL;
        goto out;
    }

    if(fpsensor_dev->device_available) {
        fpsensor_disable_irq(fpsensor_dev);
        if (fpsensor_dev->irq)
            devm_free_irq(&g_fpsensor->spi->dev, fpsensor_dev->irq, fpsensor_dev);

        fpsensor_gpio_free(fpsensor_dev);
        fpsensor_dev->device_available = 0;
    }

#if FP_NOTIFY
    fpsensor_fb_unregister_client(&fpsensor_dev->notifier);
#endif

    fpsensor_dev_cleanup(fpsensor_dev);

#if FPSENSOR_WAKEUP_TYPE == FPSENSOR_WAKEUP_SOURCE
    wakeup_source_trash_internal(&g_ttw_wl);
#else
    wake_lock_destroy(&g_ttw_wl);
#endif

    kfree(fpsensor_dev);
    g_fpsensor = NULL;

out:
    FUNC_EXIT();
    return ret;
}

static int fpsensor_suspend(struct platform_device *pdev, pm_message_t state)
{
    return 0;
}

static int fpsensor_resume( struct platform_device *pdev)
{
    return 0;
}

#ifdef CONFIG_OF
static struct of_device_id fpsensor_of_match[] = {
    { .compatible = "qcom,fingerprint-gpio" },
    {}
};
//MODULE_DEVICE_TABLE(of, fpsensor_of_match);
#endif

static struct platform_driver fpsensor_driver = {
    .driver = {
        .name = FPSENSOR_DEV_NAME,
        .owner = THIS_MODULE,
#ifdef CONFIG_OF
        .of_match_table = fpsensor_of_match,
#endif
    },
    .probe = fpsensor_probe,
    .remove = fpsensor_remove,
    .suspend = fpsensor_suspend,
    .resume = fpsensor_resume,
};

static int __init fpsensor_init(void)
{
    int status;

    status = platform_driver_register(&fpsensor_driver);
    if (status < 0) {
        fpsensor_debug(ERR_LOG, "%s, Failed to register SPI driver.\n", __func__);
    }

    return status;
}
module_init(fpsensor_init);

static void __exit fpsensor_exit(void)
{
    platform_driver_unregister(&fpsensor_driver);
}
module_exit(fpsensor_exit);

MODULE_AUTHOR("xhli");
MODULE_DESCRIPTION(" Fingerprint chip TEE driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:fpsensor-drivers");
