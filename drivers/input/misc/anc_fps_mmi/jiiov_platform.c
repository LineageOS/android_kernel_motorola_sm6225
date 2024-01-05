/**
 * The device control driver for JIIOV's fingerprint sensor.
 *
 * Copyright (C) 2020 JIIOV Corporation. <http://www.jiiov.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
 * Public License for more details.
 **/
#include <linux/atomic.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <net/sock.h>

// clang-format off
#include "jiiov_config.h"
#include "jiiov_log.h"
#include "jiiov_netlink.h"
#include "jiiov_platform.h"
// clang-format on

#ifdef SAMSUNG_PLATFORM
#include <soc/samsung/exynos_pm_qos.h>
#include <soc/samsung/exynos-devfreq.h>
#endif

#ifdef CONFIG_PM_WAKELOCKS
#include <linux/device.h>
#include <linux/pm_wakeup.h>
#else
#include "../include/wakelock.h"
#endif

#if defined(ANC_USE_REE_SPI) || defined(MTK_PLATFORM)
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#endif

#ifdef ANC_SUPPORT_NAVIGATION_EVENT
#include <linux/input.h>
#define ANC_INPUT_NAME "jiiov-keys"
#endif

typedef enum {
    ANC_KEY_NONE = 0,
    ANC_KEY_HOME,
    ANC_KEY_MENU,
    ANC_KEY_BACK,
    ANC_KEY_UP,
    ANC_KEY_LEFT,
    ANC_KEY_RIGHT,
    ANC_KEY_DOWN,
    ANC_KEY_POWER,
    ANC_KEY_WAKEUP,
    ANC_KEY_CAMERA,
} ANC_KEY_TYPE;

typedef struct {
    ANC_KEY_TYPE key;
    int value; /* 0 = key up, 1 = key down */
} ANC_KEY_EVENT;

typedef struct {
    char product_id[16];
} __attribute__((packed)) ANC_SENSOR_PRODUCT_INFO;

#define ANC_IOC_MAGIC 'a'

#define ANC_IOC_RESET _IO(ANC_IOC_MAGIC, 0)
#define ANC_IOC_REQUEST_RESOURCE _IO(ANC_IOC_MAGIC, 1)
#define ANC_IOC_RELEASE_RESOURCE _IO(ANC_IOC_MAGIC, 2)
#define ANC_IOC_ENABLE_IRQ _IO(ANC_IOC_MAGIC, 3)
#define ANC_IOC_DISABLE_IRQ _IO(ANC_IOC_MAGIC, 4)
#define ANC_IOC_INIT_IRQ _IO(ANC_IOC_MAGIC, 5)
#define ANC_IOC_DEINIT_IRQ _IO(ANC_IOC_MAGIC, 6)
#define ANC_IOC_SET_IRQ_FLAG_MASK _IO(ANC_IOC_MAGIC, 7)
#define ANC_IOC_CLEAR_IRQ_FLAG_MASK _IO(ANC_IOC_MAGIC, 8)
#define ANC_IOC_ENABLE_SPI_CLK _IO(ANC_IOC_MAGIC, 9)
#define ANC_IOC_DISABLE_SPI_CLK _IO(ANC_IOC_MAGIC, 10)
#define ANC_IOC_ENABLE_POWER _IO(ANC_IOC_MAGIC, 11)
#define ANC_IOC_DISABLE_POWER _IO(ANC_IOC_MAGIC, 12)
#define ANC_IOC_SPI_SPEED _IOW(ANC_IOC_MAGIC, 13, uint32_t)
#define ANC_IOC_CLEAR_FLAG _IO(ANC_IOC_MAGIC, 14)
#define ANC_IOC_WAKE_LOCK _IO(ANC_IOC_MAGIC, 15)
#define ANC_IOC_WAKE_UNLOCK _IO(ANC_IOC_MAGIC, 16)
#define ANC_IOC_REPORT_KEY_EVENT _IOW(ANC_IOC_MAGIC, 17, ANC_KEY_EVENT)
#define ANC_IOC_SET_PRODUCT_ID _IOW(ANC_IOC_MAGIC, 18, ANC_SENSOR_PRODUCT_INFO)
#define ANC_IOC_GET_IRQ_PIN_LEVEL _IOR(ANC_IOC_MAGIC, 19, uint8_t)

#define ANC_COMPATIBLE_SW_FP "jiiov,fingerprint"
#define ANC_DEVICE_NAME "jiiov_fp"

#ifdef SAMSUNG_PLATFORM
enum anc_spi_speed {
    ANC_LOW_SPEED = 0,
    ANC_HIGH_SPEED = 1,
    ANC_SUPER_SPEED = 2,
};

#define SPI_LOW_SPEED (1000000 * 4 * 6)    /* 6M */
#define SPI_HIGH_SPEED (1000000 * 4 * 20)  /* 20M */
#define SPI_SUPER_SPEED (1000000 * 4 * 25) /* 25M */
#endif

#if defined(ANC_USE_REE_SPI) || defined(MTK_PLATFORM)
#ifdef ANC_USE_REE_SPI
#define SPI_BUFFER_SIZE (50 * 1024)
#define ANC_DEFAULT_SPI_SPEED (18 * 1000 * 1000)
static uint8_t *spi_buffer = NULL;
#endif
typedef struct spi_device anc_device_t;
typedef struct spi_driver anc_driver_t;
#else
typedef struct platform_device anc_device_t;
typedef struct platform_driver anc_driver_t;
#endif

static anc_device_t *g_anc_spi_device = NULL;

struct vreg_config {
    char *name;
    unsigned int vmin;
    unsigned int vmax;
    int ua_load;
};

#define ANC_VREG_VDD_NAME "vdd"
#define ANC_VDD_CONFIG_NAME "anc,vdd_config"
static struct vreg_config vreg_conf;

static const char *const pctl_names[] = {
    "anc_reset_low",
    "anc_reset_high",
    "anc_irq_default",
};

struct anc_data {
    struct device *dev;
    struct class *dev_class;
    dev_t dev_num;
    struct cdev cdev;
    ANC_SENSOR_PRODUCT_INFO product_info;
#ifdef ANC_SUPPORT_NAVIGATION_EVENT
    struct input_dev *input;
    ANC_KEY_EVENT key_event;
#endif

    struct pinctrl *fingerprint_pinctrl;
    struct pinctrl_state *pinctrl_state[ARRAY_SIZE(pctl_names)];
    struct regulator *vreg;
#ifdef CONFIG_PM_WAKELOCKS
    struct wakeup_source *fp_wakelock;
#else
    struct wake_lock fp_wakelock;
#endif
    struct work_struct work_queue;
    int irq_gpio;
    int irq;
    atomic_t irq_enabled;
    bool irq_mask_flag;
    int irq_init;
    int pwr_gpio;
    int rst_gpio;
    struct mutex lock;
    bool resource_requested;

    bool vdd_use_gpio;
    bool vdd_use_pmic;
    bool use_gpio_init;

#ifdef SAMSUNG_PLATFORM
    struct clk *core_clk;
    struct clk *iface_clk;
    int clk_enabled;
#endif
};

static int vreg_setup(struct anc_data *p_data, const char *name, bool enable) {
    int ret_val = 0;

    if (p_data == NULL) {
        ANC_LOGE("p_data is NULL");
        return -EINVAL;
    }
    ret_val = strncmp(vreg_conf.name, name, strlen(vreg_conf.name));
    if (ret_val != 0) {
        ANC_LOGE("Regulator %s not found", name);
        return -EINVAL;
    }

    if (enable) {
        if (p_data->vreg == NULL) {
            p_data->vreg = regulator_get(p_data->dev, name);
            if (IS_ERR(p_data->vreg)) {
                ANC_LOGE("Unable to get %s", name);
                return PTR_ERR(p_data->vreg);
            }
        }

        if (regulator_count_voltages(p_data->vreg) > 0) {
            ret_val = regulator_set_voltage(p_data->vreg, vreg_conf.vmin, vreg_conf.vmax);
            if (ret_val) {
                ANC_LOGE("Unable to set voltage on %s, %d", name, ret_val);
            }
        }

        ret_val = regulator_set_load(p_data->vreg, vreg_conf.ua_load);
        if (ret_val < 0) {
            ANC_LOGE("Unable to set current on %s, %d", name, ret_val);
        }

        ret_val = regulator_enable(p_data->vreg);
        if (ret_val) {
            ANC_LOGE("error enabling %s: %d", name, ret_val);
            regulator_put(p_data->vreg);
            p_data->vreg = NULL;
        }
    } else {
        if (p_data->vreg != NULL) {
            ret_val = regulator_is_enabled(p_data->vreg);
            if (ret_val != 0) {
                regulator_disable(p_data->vreg);
                ANC_LOGD("disabled %s", name);
            }
            regulator_put(p_data->vreg);
            p_data->vreg = NULL;
        }
    }

    return ret_val;
}

/**
 * sysfs node to forward netlink event
 */
static ssize_t forward_netlink_event_set(struct device *p_dev, struct device_attribute *p_attr,
                                         const char *p_buffer, size_t count) {
    char netlink_msg = (char)ANC_NETLINK_EVENT_INVALID;

    ANC_LOGD("forward netlink event: %s", p_buffer);
    if (!strncmp(p_buffer, "test", strlen("test"))) {
        netlink_msg = (char)ANC_NETLINK_EVENT_TEST;
    } else if (!strncmp(p_buffer, "irq", strlen("irq"))) {
        netlink_msg = (char)ANC_NETLINK_EVENT_IRQ;
    } else if (!strncmp(p_buffer, "screen_off", strlen("screen_off"))) {
        netlink_msg = (char)ANC_NETLINK_EVENT_SCR_OFF;
    } else if (!strncmp(p_buffer, "screen_on", strlen("screen_on"))) {
        netlink_msg = (char)ANC_NETLINK_EVENT_SCR_ON;
    } else if (!strncmp(p_buffer, "touch_down", strlen("touch_down"))) {
        netlink_msg = (char)ANC_NETLINK_EVENT_TOUCH_DOWN;
    } else if (!strncmp(p_buffer, "touch_up", strlen("touch_up"))) {
        netlink_msg = (char)ANC_NETLINK_EVENT_TOUCH_UP;
    } else if (!strncmp(p_buffer, "ui_ready", strlen("ui_ready"))) {
        netlink_msg = (char)ANC_NETLINK_EVENT_UI_READY;
    } else if (!strncmp(p_buffer, "exit", strlen("exit"))) {
        netlink_msg = (char)ANC_NETLINK_EVENT_EXIT;
    } else {
        ANC_LOGE("don't support the netlink evnet: %s", p_buffer);
        return -EINVAL;
    }

    return netlink_send_message_to_user(&netlink_msg, sizeof(netlink_msg));
}
static DEVICE_ATTR(netlink_event, S_IWUSR, NULL, forward_netlink_event_set);

/**
 * sysfs node to select the set of pins (GPIOS) defined in a pin control node of
 * the device tree
 */
static int select_pin_ctl(struct anc_data *p_data, const char *name) {
    int ret_val = 0;
    size_t i = 0;

    if ((p_data == NULL) || (name == NULL)) {
        ANC_LOGE("parameter is NULL, p_data:%p, name:%p", p_data, name);
        return -EINVAL;
    }
    for (i = 0; i < ARRAY_SIZE(p_data->pinctrl_state); i++) {
        const char *n = pctl_names[i];

        if (!strncmp(n, name, strlen(n))) {
            ret_val = pinctrl_select_state(p_data->fingerprint_pinctrl, p_data->pinctrl_state[i]);
            if (ret_val) {
                ANC_LOGE("cannot select %s, ret_val: %d", name, ret_val);
            } else {
                ANC_LOGD("Selected %s success", name);
            }
            goto exit;
        }
    }

    ret_val = -EINVAL;
    ANC_LOGE("%s not found", name);

exit:
    return ret_val;
}

static ssize_t pinctl_set(struct device *dev, struct device_attribute *attr, const char *buf,
                          size_t count) {
    int ret_val = 0;
    struct anc_data *p_data = dev_get_drvdata(dev);
    if (p_data == NULL) {
        ANC_LOGE("dev get data failed");
        return -EINVAL;
    }

    mutex_lock(&p_data->lock);
    ANC_LOGD("pinctl select %s", buf);
    ret_val = select_pin_ctl(p_data, buf);
    mutex_unlock(&p_data->lock);

    return ret_val ? ret_val : count;
}
static DEVICE_ATTR(pinctl_set, S_IWUSR, NULL, pinctl_set);

static int set_reset_level(struct anc_data *p_data, int level) {
    int ret_val = 0;

    if (p_data == NULL) {
        ANC_LOGE("p_data is NULL");
        return -EINVAL;
    }
    if (p_data->use_gpio_init) {
        gpio_set_value(p_data->rst_gpio, level);
    } else {
        if (level == 0) {
            ret_val = select_pin_ctl(p_data, "anc_reset_low");
        } else {
            ret_val = select_pin_ctl(p_data, "anc_reset_high");
        }
    }

    if (ret_val != 0) {
        ANC_LOGE("set reset level:%d failed, ret_val: %d", level, ret_val);
    }

    return ret_val;
}

static int anc_reset(struct anc_data *p_data) {
    int ret_val = 0;

    if (p_data == NULL) {
        ANC_LOGE("p_data is NULL");
        return -EINVAL;
    }
    ANC_LOGD("hardware reset");
    mutex_lock(&p_data->lock);
    do {
        ret_val = set_reset_level(p_data, 0);
        if (ret_val != 0) {
            break;
        }
        // T2 >= 10ms
        mdelay(10);
        ret_val = set_reset_level(p_data, 1);
        if (ret_val != 0) {
            break;
        }
        mdelay(10);
    } while (0);
    mutex_unlock(&p_data->lock);

    return ret_val;
}

static ssize_t hw_reset_set(struct device *dev, struct device_attribute *attr, const char *buf,
                            size_t count) {
    int ret_val = 0;
    struct anc_data *p_data = dev_get_drvdata(dev);
    if (p_data == NULL) {
        ANC_LOGE("dev get data failed");
        return -EINVAL;
    }

    if (!strncmp(buf, "reset", strlen("reset"))) {
        ret_val = anc_reset(p_data);
        if (ret_val != 0) {
            ANC_LOGE("hardware reset failed, ret_val: %d", ret_val);
        }
    } else {
        ret_val = -EINVAL;
        ANC_LOGE("the reset name is not matched, name: %s", buf);
    }

    return ret_val ? ret_val : count;
}
static DEVICE_ATTR(hw_reset, S_IWUSR, NULL, hw_reset_set);

static void anc_power_onoff(struct anc_data *p_data, bool power_onoff) {
    if (p_data == NULL) {
        ANC_LOGE("p_data is NULL");
        return;
    }

    ANC_LOGD("power_onoff = %d ", power_onoff);
    if (!p_data->resource_requested) {
        ANC_LOGW("warning: resource is not requested, return!");
        return;
    }

    if (p_data->vdd_use_gpio) {
        gpio_set_value(p_data->pwr_gpio, power_onoff);
    }

    if (p_data->vdd_use_pmic) {
        int ret_val = vreg_setup(p_data, ANC_VREG_VDD_NAME, power_onoff);
        if (ret_val != 0) {
            ANC_LOGE("use pmic to contrl power failed, ret_val: %d", ret_val);
        }
    }
}

static void device_power_up(struct anc_data *p_data) {
    if (p_data == NULL) {
        ANC_LOGE("p_data is NULL");
        return;
    }

    ANC_LOGD("device power up");
    anc_power_onoff(p_data, true);
}

/**
 * sysfs node to power on/power off the sensor
 */
static ssize_t device_power_set(struct device *dev, struct device_attribute *attr, const char *buf,
                                size_t count) {
    ssize_t ret_val = count;
    struct anc_data *p_data = dev_get_drvdata(dev);
    if (p_data == NULL) {
        ANC_LOGE("dev get data failed");
        return -EINVAL;
    }

    mutex_lock(&p_data->lock);
    if (!strncmp(buf, "on", strlen("on"))) {
        ANC_LOGD("device power on");
        anc_power_onoff(p_data, true);
    } else if (!strncmp(buf, "off", strlen("off"))) {
        ANC_LOGD("device power off");
        anc_power_onoff(p_data, false);
    } else {
        ret_val = -EINVAL;
        ANC_LOGE("the power name is not matched, name: %s", buf);
    }
    mutex_unlock(&p_data->lock);

    return ret_val;
}
static DEVICE_ATTR(device_power, S_IWUSR, NULL, device_power_set);

#ifdef ANC_USE_REE_SPI
static int anc_spi_transfer(const uint8_t *txbuf, uint8_t *rxbuf, int len) {
    struct spi_transfer t;
    struct spi_message m;

    memset(&t, 0, sizeof(t));
    spi_message_init(&m);
    t.tx_buf = txbuf;
    t.rx_buf = rxbuf;
    t.bits_per_word = 8;
    t.len = len;
    t.speed_hz = g_anc_spi_device->max_speed_hz;
    spi_message_add_tail(&t, &m);
    return spi_sync(g_anc_spi_device, &m);
}

static uint32_t anc_read_sensor_id(struct anc_data *p_data) {
    int ret_val = -1;
    int trytimes = 3;
    uint32_t sensor_chip_id = 0;

    if (p_data == NULL) {
        ANC_LOGE("p_data is NULL");
        return -EINVAL;
    }
    do {
        memset(spi_buffer, 0, 4);
        spi_buffer[0] = 0x30;
        spi_buffer[1] = (char)(~0x30);

        anc_reset(p_data);

        ret_val = anc_spi_transfer(spi_buffer, spi_buffer, 4);
        if (ret_val != 0) {
            ANC_LOGE("spi_transfer failed");
            continue;
        }

        sensor_chip_id = (uint32_t)((spi_buffer[3] & 0x00FF) | ((spi_buffer[2] << 8) & 0xFF00));
        ANC_LOGD("sensor chip_id = %#x", sensor_chip_id);

        if (sensor_chip_id == 0x6311) {
            ANC_LOGD("Read Sensor Id Success");
            return 0;
        } else {
            ANC_LOGE("Read Sensor Id Fail");
        }
    } while (trytimes--);

    return sensor_chip_id;
}

/**
 * sysfs node to read sensor id
 */
static ssize_t sensor_id_show(struct device *dev, struct device_attribute *attr, char *buf) {
    struct anc_data *p_data = dev_get_drvdata(dev);
    uint32_t sensor_chip_id = anc_read_sensor_id(p_data);

    return scnprintf(buf, PAGE_SIZE, "0x%04x", sensor_chip_id);
}
static DEVICE_ATTR(sensor_id, S_IRUSR, sensor_id_show, NULL);

static int set_spi_speed(uint32_t speed) {
    ANC_LOGD("set spi speed : %dhz", speed);
    g_anc_spi_device->max_speed_hz = speed;

    return spi_setup(g_anc_spi_device);
}

static ssize_t anc_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos) {
    ssize_t status = 0;
    // struct anc_data *dev_data = filp->private_data;

    ANC_LOGD("count = %zu", count);

    if (count > SPI_BUFFER_SIZE) {
        return (-EMSGSIZE);
    }

    if (copy_from_user(spi_buffer, buf, count)) {
        ANC_LOGE("copy_from_user failed");
        return (-EMSGSIZE);
    }

    status = anc_spi_transfer(spi_buffer, spi_buffer, count);

    if (status == 0) {
        status = copy_to_user(buf, spi_buffer, count);

        if (status != 0) {
            status = -EFAULT;
        } else {
            status = count;
        }
    } else {
        ANC_LOGE("spi_transfer failed");
    }

    return status;
}

static ssize_t anc_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos) {
    ssize_t status = 0;
    // struct anc_data *dev_data = filp->private_data;

    ANC_LOGD("count = %zu", count);

    if (count > SPI_BUFFER_SIZE) {
        return (-EMSGSIZE);
    }

    if (copy_from_user(spi_buffer, buf, count)) {
        ANC_LOGE("copy_from_user failed");
        return (-EMSGSIZE);
    }

    status = anc_spi_transfer(spi_buffer, NULL, count);

    if (status == 0) {
        status = count;
    } else {
        ANC_LOGE("spi_transfer failed");
    }

    return status;
}

static int anc_config_spi(void) {
    g_anc_spi_device->mode = SPI_MODE_0;
    g_anc_spi_device->bits_per_word = 8;
    g_anc_spi_device->max_speed_hz = ANC_DEFAULT_SPI_SPEED;

    return spi_setup(g_anc_spi_device);
}
#endif

static void anc_enable_irq(struct anc_data *p_data) {
    if (p_data == NULL) {
        ANC_LOGE("p_data is NULL");
        return;
    }
    ANC_LOGD("enable irq");

    if (!p_data->resource_requested) {
        ANC_LOGW("warning: resource is not requested, return!");
        return;
    }

    if (atomic_read(&p_data->irq_enabled)) {
        ANC_LOGW("IRQ has been enabled");
    } else {
        enable_irq(p_data->irq);
        atomic_set(&p_data->irq_enabled, 1);
    }
}

static void anc_disable_irq(struct anc_data *p_data) {
    if (p_data == NULL) {
        ANC_LOGE("p_data is NULL");
        return;
    }
    ANC_LOGD("disable irq");

    if (!p_data->resource_requested) {
        ANC_LOGW("warning: resource is not requested, return!");
        return;
    }

    if (atomic_read(&p_data->irq_enabled)) {
        disable_irq(p_data->irq);
        atomic_set(&p_data->irq_enabled, 0);
    } else {
        ANC_LOGW("IRQ has been disabled");
    }
}

static void anc_wake_lock(struct anc_data *p_data) {
    if (p_data == NULL) {
        ANC_LOGE("p_data is NULL");
        return;
    }
    ANC_LOGD("wake lock");
#ifdef CONFIG_PM_WAKELOCKS
    __pm_stay_awake(p_data->fp_wakelock);
#else
    wake_lock(&p_data->fp_wakelock);
#endif
}

static void anc_wake_unlock(struct anc_data *p_data) {
    if (p_data == NULL) {
        ANC_LOGE("p_data is NULL");
        return;
    }
    ANC_LOGD("wake unlock");
#ifdef CONFIG_PM_WAKELOCKS
    __pm_relax(p_data->fp_wakelock);
    __pm_wakeup_event(p_data->fp_wakelock, msecs_to_jiffies(ANC_WAKELOCK_HOLD_TIME));
#else
    wake_unlock(&p_data->fp_wakelock);
    wake_lock_timeout(&p_data->fp_wakelock, msecs_to_jiffies(ANC_WAKELOCK_HOLD_TIME));
#endif
}

/**
 * sysfs node for controlling whether the driver is allowed
 * to wake up the platform on interrupt.
 */
static ssize_t irq_control_set(struct device *dev, struct device_attribute *attr, const char *buf,
                               size_t count) {
    ssize_t ret_val = count;
    struct anc_data *p_data = dev_get_drvdata(dev);

    mutex_lock(&p_data->lock);
    if (!strncmp(buf, "enable", strlen("enable"))) {
        anc_enable_irq(p_data);
    } else if (!strncmp(buf, "disable", strlen("disable"))) {
        anc_disable_irq(p_data);
    } else {
        ret_val = -EINVAL;
        ANC_LOGE("the irq name is not matched, name: %s", buf);
    }
    mutex_unlock(&p_data->lock);

    return ret_val ? ret_val : count;
}
static DEVICE_ATTR(irq_set, S_IWUSR, NULL, irq_control_set);

static int anc_platform_init(struct anc_data *p_data);
static int anc_platform_free(struct anc_data *p_data);
/**
 * sysfs node to request/release resource
 */
static ssize_t resource_control_set(struct device *dev, struct device_attribute *attr,
                                    const char *buf, size_t count) {
    ssize_t ret_val = count;
    struct anc_data *p_data = dev_get_drvdata(dev);
    if (p_data == NULL) {
        ANC_LOGE("dev get data failed");
        return -EINVAL;
    }

    mutex_lock(&p_data->lock);
    if (!strncmp(buf, "request", strlen("request"))) {
        ANC_LOGD("request resource");
        anc_platform_init(p_data);

    } else if (!strncmp(buf, "release", strlen("release"))) {
        ANC_LOGD("release resource");
        anc_platform_free(p_data);
    } else {
        ret_val = -EINVAL;
        ANC_LOGE("the resource name is not matched, name: %s", buf);
    }
    mutex_unlock(&p_data->lock);

    return ret_val;
}
static DEVICE_ATTR(resource_set, S_IWUSR, NULL, resource_control_set);

static struct attribute *attributes[] = {&dev_attr_pinctl_set.attr,
                                         &dev_attr_device_power.attr,
                                         &dev_attr_hw_reset.attr,
                                         &dev_attr_irq_set.attr,
                                         &dev_attr_netlink_event.attr,
                                         &dev_attr_resource_set.attr,
#ifdef ANC_USE_REE_SPI
                                         &dev_attr_sensor_id.attr,
#endif
                                         NULL};

static const struct attribute_group attribute_group = {
    .attrs = attributes,
};

static void anc_do_irq_work(struct work_struct *ws) {
    char netlink_msg = (char)ANC_NETLINK_EVENT_IRQ;

    netlink_send_message_to_user(&netlink_msg, sizeof(netlink_msg));
}

static irqreturn_t anc_irq_handler(int irq, void *handle) {
    struct anc_data *p_data = handle;

    ANC_LOGD("irq handler, %d", p_data->irq_mask_flag);
    if (p_data->irq_mask_flag) {
        return IRQ_HANDLED;
    }
#ifdef CONFIG_PM_WAKELOCKS
    __pm_wakeup_event(p_data->fp_wakelock, msecs_to_jiffies(ANC_WAKELOCK_HOLD_TIME));
#else
    wake_lock_timeout(&p_data->fp_wakelock, msecs_to_jiffies(ANC_WAKELOCK_HOLD_TIME));
#endif
    schedule_work(&p_data->work_queue);

    return IRQ_HANDLED;
}

static int anc_request_named_gpio(struct anc_data *p_data, const char *label, int *gpio) {
    int ret_val = 0;

    if ((p_data == NULL) || (label == NULL) || (gpio == NULL)) {
        ANC_LOGE("parameter is NULL, p_data:%p, label:%p, gpio:%p", p_data, label, gpio);
        return -EINVAL;
    }
    ret_val = of_get_named_gpio(p_data->dev->of_node, label, 0);
    if (ret_val < 0) {
        ANC_LOGE("failed to get '%s'", label);
        return ret_val;
    }
    *gpio = ret_val;

    ret_val = devm_gpio_request(p_data->dev, *gpio, label);
    if (ret_val) {
        ANC_LOGE("failed to request gpio %d", *gpio);
        return ret_val;
    }
    ANC_LOGD("%s %d", label, *gpio);

    return 0;
}

static int anc_irq_init(struct anc_data *p_data) {
    int ret_val = 0;
    int irqf = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;  // IRQF_TRIGGER_FALLING or IRQF_TRIGGER_RISING

    if (p_data == NULL) {
        ANC_LOGE("p_data is NULL");
        return -EINVAL;
    }
    p_data->irq = gpio_to_irq(p_data->irq_gpio);
    if (p_data->irq_init == 0) {
        ret_val = devm_request_threaded_irq(
            p_data->dev, p_data->irq, NULL, anc_irq_handler, irqf, dev_name(p_data->dev), p_data);
        if (ret_val != 0) {
            ANC_LOGE("Could not request irq %d, ret_val: %d", p_data->irq, ret_val);
            return ret_val;
        }
        p_data->irq_init = 1;

        /* Request that the interrupt should be wakeable */
        enable_irq_wake(p_data->irq);
        atomic_set(&p_data->irq_enabled, 1);
    }

    return ret_val;
}

static int anc_irq_deinit(struct anc_data *p_data) {
    if (p_data == NULL) {
        ANC_LOGE("p_data is NULL");
        return -EINVAL;
    }

    if (p_data->irq_init == 1) {
        disable_irq_wake(p_data->irq);
        devm_free_irq(p_data->dev, p_data->irq, p_data);
        p_data->irq_init = 0;
    }

    return 0;
}

static int anc_copy_from_user(void *to, const void *from, unsigned long n) {
    int ret_val = 0;

    if (!access_ok(from, n)) {
        ANC_LOGE("the parameter is invalid");
        return -EFAULT;
    }

    ret_val = copy_from_user(to, from, n);
    if (ret_val != 0) {
        ANC_LOGE("copy_from_user failed, %d", ret_val);
        ret_val = -EFAULT;
    }

    return ret_val;
}

#ifdef ANC_SUPPORT_NAVIGATION_EVENT
static int anc_report_key_event(struct anc_data *p_data, const void *arg) {
    int ret_val = 0;
    unsigned int key_code = KEY_UNKNOWN;

    if (p_data == NULL) {
        ANC_LOGE("p_data is NULL");
        return -EINVAL;
    }

    ret_val = anc_copy_from_user(&(p_data->key_event), arg, sizeof(ANC_KEY_EVENT));
    if (ret_val != 0) {
        ANC_LOGE("Failed to copy input key event from user to kernel, ret_val: %d", ret_val);
        return -EFAULT;
    }

    ANC_LOGD("key = %d, value = %d", p_data->key_event.key, p_data->key_event.value);
    switch (p_data->key_event.key) {
        case ANC_KEY_HOME:
            key_code = KEY_HOME;
            break;
        case ANC_KEY_MENU:
            key_code = KEY_MENU;
            break;
        case ANC_KEY_BACK:
            key_code = KEY_BACK;
            break;
        case ANC_KEY_UP:
            key_code = KEY_UP;
            break;
        case ANC_KEY_DOWN:
            key_code = KEY_DOWN;
            break;
        case ANC_KEY_LEFT:
            key_code = KEY_LEFT;
            break;
        case ANC_KEY_RIGHT:
            key_code = KEY_RIGHT;
            break;
        case ANC_KEY_POWER:
            key_code = KEY_POWER;
            break;
        case ANC_KEY_WAKEUP:
            key_code = KEY_WAKEUP;
            break;
        case ANC_KEY_CAMERA:
            key_code = KEY_CAMERA;
            break;
        default:
            key_code = (unsigned int)(p_data->key_event.key);
            break;
    }

    input_report_key(p_data->input, key_code, p_data->key_event.value);
    input_sync(p_data->input);

    return ret_val;
}
#endif

static int use_pinctrl_init(struct anc_data *p_data) {
    int ret_val = 0;
    size_t i = 0;

    if (p_data == NULL) {
        ANC_LOGE("p_data is NULL");
        return -EINVAL;
    }
    p_data->fingerprint_pinctrl = devm_pinctrl_get(p_data->dev);
    if (IS_ERR(p_data->fingerprint_pinctrl)) {
        if (PTR_ERR(p_data->fingerprint_pinctrl) == -EPROBE_DEFER) {
            ANC_LOGD("pinctrl not ready");
            return -EPROBE_DEFER;
        }
        ANC_LOGE("Target does not use pinctrl");
        p_data->fingerprint_pinctrl = NULL;
        return -EINVAL;
    }

    for (i = 0; i < ARRAY_SIZE(p_data->pinctrl_state); i++) {
        struct pinctrl_state *state =
            pinctrl_lookup_state(p_data->fingerprint_pinctrl, pctl_names[i]);
        if (IS_ERR(state)) {
            ANC_LOGE("cannot find '%s'", pctl_names[i]);
            return -EINVAL;
        }
        ANC_LOGD("found pin control %s", pctl_names[i]);
        p_data->pinctrl_state[i] = state;
    }

    ret_val = select_pin_ctl(p_data, "anc_irq_default");
    if (ret_val) {
        ANC_LOGD("set default irq failed, ret_val:%d", ret_val);
        return ret_val;
    }

    ret_val = select_pin_ctl(p_data, "anc_reset_high");
    if (ret_val) {
        ANC_LOGD("set reset low failed, ret_val:%d", ret_val);
        return ret_val;
    }

    return ret_val;
}

static int use_gpio_init(struct anc_data *p_data) {
    int ret_val = 0;

    if (p_data == NULL) {
        ANC_LOGE("p_data is NULL");
        return -EINVAL;
    }
    ret_val = anc_request_named_gpio(p_data, "anc,gpio_rst", &p_data->rst_gpio);
    if (ret_val) {
        ANC_LOGD("request reset gpio failed, ret_val:%d", ret_val);
        return ret_val;
    }

    ret_val = gpio_direction_output(p_data->rst_gpio, 1);
    if (ret_val) {
        ANC_LOGD("reset gpio direction output failed, ret_val:%d", ret_val);
        return ret_val;
    }

    ret_val = anc_request_named_gpio(p_data, "anc,gpio_irq", &p_data->irq_gpio);
    if (ret_val) {
        ANC_LOGD("request irq gpio failed, ret_val:%d", ret_val);
        return ret_val;
    }

    ret_val = gpio_direction_input(p_data->irq_gpio);
    if (ret_val) {
        ANC_LOGD("irq gpio direction input failed, ret_val:%d", ret_val);
        return ret_val;
    }

    return ret_val;
}

static int anc_gpio_init(struct anc_data *p_data) {
    int ret_val = 0;

    if (p_data == NULL) {
        ANC_LOGE("p_data is NULL");
        return -EINVAL;
    }
    ANC_LOGD("init gpio");
    if (p_data->resource_requested) {
        ANC_LOGW("warning: resource is already requested, return!");
        return 0;
    }

    if (p_data->vdd_use_gpio) {
        ret_val = anc_request_named_gpio(p_data, "anc,gpio_pwr", &p_data->pwr_gpio);
        if (ret_val) {
            ANC_LOGE("request power gpio failed, ret_val:%d", ret_val);
            return ret_val;
        }

        ret_val = gpio_direction_output(p_data->pwr_gpio, 0);
        if (ret_val) {
            ANC_LOGE("power gpio direction output failed, ret_val:%d", ret_val);
            return ret_val;
        }
    }

    if (p_data->use_gpio_init == false) {
        ret_val = use_pinctrl_init(p_data);
        if (ret_val) {
            ANC_LOGE("use pinctrl init failed, ret_val:%d", ret_val);
            return ret_val;
        }
    }

    ret_val = use_gpio_init(p_data);
    if (ret_val) {
        ANC_LOGE("use gpio init failed, ret_val:%d", ret_val);
        return ret_val;
    }

#ifdef SAMSUNG_PLATFORM
    /* Init spi clock */
    ret_val = anc_spi_clk_init(p_data);
    if (ret_val) {
        ANC_LOGE("spi init failed, ret_val:%d", ret_val);
        return ret_val;
    }
#endif

    if (of_property_read_bool(p_data->dev->of_node, "anc,enable-on-boot")) {
        ANC_LOGD("Enabling hardware");
        device_power_up(p_data);
    }

    p_data->resource_requested = true;
    ANC_LOGD("success");

    return 0;
}

static int anc_gpio_deinit(struct anc_data *p_data) {
    if (p_data == NULL) {
        ANC_LOGE("p_data is NULL");
        return -EINVAL;
    }

    ANC_LOGD("deinit gpio");
    if (!p_data->resource_requested) {
        ANC_LOGW("warning: resource is already released, return!");
        return 0;
    }

    if (p_data->vdd_use_gpio) {
        if (gpio_is_valid(p_data->pwr_gpio)) {
            gpio_free(p_data->pwr_gpio);
        }
    }

    /* Release pinctl */
    if (p_data->fingerprint_pinctrl) {
        ANC_LOGD("devm_pinctrl_put");
        devm_pinctrl_put(p_data->fingerprint_pinctrl);
        p_data->fingerprint_pinctrl = NULL;
    }

    if (gpio_is_valid(p_data->rst_gpio)) {
        gpio_free(p_data->rst_gpio);
    }

    if (gpio_is_valid(p_data->irq_gpio)) {
        gpio_free(p_data->irq_gpio);
    }

#ifdef SAMSUNG_PLATFORM
    /* Deinit spi clock */
    anc_spi_clk_deinit(p_data);
#endif

    p_data->resource_requested = false;
    ANC_LOGD("success");

    return 0;
}

static int anc_open(struct inode *inode, struct file *filp) {
    struct anc_data *p_data = NULL;
    p_data = container_of(inode->i_cdev, struct anc_data, cdev);
    if (p_data == NULL) {
        ANC_LOGE("get anc date failed");
        return -EINVAL;
    }
    filp->private_data = p_data;

    return 0;
}

static int anc_platform_init(struct anc_data *p_data) {
    int ret_val = 0;

    if (p_data == NULL) {
        ANC_LOGE("p_data is NULL");
        return -EINVAL;
    }
    ret_val = anc_gpio_init(p_data);
    if (ret_val) {
        ANC_LOGE("Failed to init gpio, ret_val: %d", ret_val);
        return ret_val;
    }

    return ret_val;
}

static int anc_platform_free(struct anc_data *p_data) {
    int ret_val = 0;

    if (p_data == NULL) {
        ANC_LOGE("p_data is NULL");
        return -EINVAL;
    }
    ret_val = anc_gpio_deinit(p_data);
    if (ret_val) {
        ANC_LOGE("Failed to deinit gpio");
        return ret_val;
    }

    return ret_val;
}

#ifdef MTK_PLATFORM
extern void mt_spi_enable_master_clk(struct spi_device *spidev);
extern void mt_spi_disable_master_clk(struct spi_device *spidev);

void spi_clk_enable(bool enable_flag) {
    static bool is_spi_clk_open = false;

    if (enable_flag) {
        if (!is_spi_clk_open) {
            ANC_LOGD("enable spi clk");
            mt_spi_enable_master_clk(g_anc_spi_device);
            is_spi_clk_open = true;
        } else {
            ANC_LOGD("spi clk already enable");
        }
    } else {
        if (is_spi_clk_open) {
            ANC_LOGD("disable spi clk ");
            mt_spi_disable_master_clk(g_anc_spi_device);
            is_spi_clk_open = false;
        } else {
            ANC_LOGD("spi clk already disable ");
        }
    }
}
#endif

#ifdef SAMSUNG_PLATFORM
static void spi_clock_set(struct anc_data *data, int speed) {
#if 0
    unsigned long rate_core;
    unsigned long rate_iface;

    rate_core = clk_get_rate(data->core_clk);
    rate_iface = clk_get_rate(data->iface_clk);
    ANC_LOGD("%s, spi clock speed (%lld, %lld)", __func__, rate_core, rate_iface);
#endif
    clk_set_rate(data->core_clk, speed);
    clk_set_rate(data->iface_clk, speed);
}

static int anc_spi_clk_init(struct anc_data *data) {
    ANC_LOGD("%s, enter", __func__);

    data->clk_enabled = 0;
    data->core_clk = clk_get(data->dev, "gate_spi_clk" /*"spi"*/);
    if (IS_ERR_OR_NULL(data->core_clk)) {
        ANC_LOGE("%s: fail to get core_clk", __func__);
        return -EPERM;
    }
    data->iface_clk = clk_get(data->dev, "ipclk_spi" /*"spi_busclk0"*/);
    if (IS_ERR_OR_NULL(data->iface_clk)) {
        ANC_LOGE("%s: fail to get iface_clk", __func__);
        clk_put(data->core_clk);
        data->core_clk = NULL;
        return -ENOENT;
    }
    return 0;
}

static int anc_spi_clk_enable(struct anc_data *data, uint32_t flag) {
    int ret_val = 0;

    ANC_LOGD("%s, enter", __func__);

    if (data->clk_enabled) {
        ANC_LOGW("%s warning: spi clock is already enabled, return!", __func__);
        return 0;
    }

    ret_val = clk_prepare_enable(data->core_clk);
    if (ret_val) {
        ANC_LOGE("%s: fail to enable core_clk", __func__);
        return -EPERM;
    }

    ret_val = clk_prepare_enable(data->iface_clk);
    if (ret_val) {
        ANC_LOGE("%s: fail to enable iface_clk", __func__);
        clk_disable_unprepare(data->core_clk);
        return -ENOENT;
    }

    // anc_pm_qos_opp_hold_int_mif();

    if (ANC_LOW_SPEED == flag) {
        spi_clock_set(data, SPI_LOW_SPEED);
    } else if (ANC_HIGH_SPEED == flag) {
        spi_clock_set(data, SPI_HIGH_SPEED);
    } else if (ANC_SUPER_SPEED == flag) {
        spi_clock_set(data, SPI_SUPER_SPEED);
    } else {
        ANC_LOGE("%s: fail to enable clock, flag(%d) is not in range[0-2]", __func__, flag);
    }

    data->clk_enabled = 1;

    return 0;
}

static int anc_spi_clk_disable(struct anc_data *data) {
    ANC_LOGD("%s, enter", __func__);

    if (!data->clk_enabled) {
        ANC_LOGW("%s warning: spi clock is disabled, return!", __func__);
        return 0;
    }

    clk_disable_unprepare(data->core_clk);
    clk_disable_unprepare(data->iface_clk);
    data->clk_enabled = 0;

    // anc_pm_qos_opp_release_int_mif();

    return 0;
}

static int anc_spi_clk_deinit(struct anc_data *data) {
    ANC_LOGD("%s, enter", __func__);

    if (data->clk_enabled) {
        anc_spi_clk_disable(data);
    }

    if (!IS_ERR_OR_NULL(data->core_clk)) {
        clk_put(data->core_clk);
        data->core_clk = NULL;
    }

    if (!IS_ERR_OR_NULL(data->iface_clk)) {
        clk_put(data->iface_clk);
        data->iface_clk = NULL;
    }

    return 0;
}

#if defined(CONFIG_EXYNOS_PM_QOS) || defined(CONFIG_EXYNOS_PM_QOS_MODULE)
static struct exynos_pm_qos_request pm_qos_req_cpu[2];
static struct exynos_pm_qos_request pm_qos_req_mif;
static struct exynos_pm_qos_request pm_qos_req_int;
#endif

static int anc_pm_qos_opp_init(void) {
    ANC_LOGD("%s: add qos class", __func__);

#if IS_ENABLED(CONFIG_EXYNOS_PM_QOS) || IS_ENABLED(CONFIG_EXYNOS_PM_QOS_MODULE)
    exynos_pm_qos_add_request(&pm_qos_req_cpu[0], PM_QOS_CLUSTER0_FREQ_MIN, 0);
    exynos_pm_qos_add_request(&pm_qos_req_cpu[1], PM_QOS_CLUSTER1_FREQ_MIN, 0);
    exynos_pm_qos_add_request(&pm_qos_req_mif, PM_QOS_BUS_THROUGHPUT, 0);
    exynos_pm_qos_add_request(&pm_qos_req_int, PM_QOS_DEVICE_THROUGHPUT, 0);
#endif
    return 0;
}

static int anc_pm_qos_opp_deinit(void) {
    ANC_LOGD("%s: remove qos class", __func__);

#if IS_ENABLED(CONFIG_EXYNOS_PM_QOS) || IS_ENABLED(CONFIG_EXYNOS_PM_QOS_MODULE)
    exynos_pm_qos_remove_request(&pm_qos_req_cpu[0]);
    exynos_pm_qos_remove_request(&pm_qos_req_cpu[1]);
    exynos_pm_qos_remove_request(&pm_qos_req_mif);
    exynos_pm_qos_remove_request(&pm_qos_req_int);
#endif
    return 0;
}

static int anc_pm_qos_opp_hold(void) {
    ANC_LOGD("%s: Request MAX QOS", __func__);

#if IS_ENABLED(CONFIG_EXYNOS_PM_QOS) || IS_ENABLED(CONFIG_EXYNOS_PM_QOS_MODULE)
    if (exynos_pm_qos_request_active(&pm_qos_req_int)) {
        exynos_pm_qos_update_request(&pm_qos_req_int, INT_MAX);
    }

    if (exynos_pm_qos_request_active(&pm_qos_req_mif)) {
        exynos_pm_qos_update_request(&pm_qos_req_mif, INT_MAX);
    }

    if (exynos_pm_qos_request_active(&pm_qos_req_cpu[0])) {
        exynos_pm_qos_update_request(&pm_qos_req_cpu[0], INT_MAX);
    }

    if (exynos_pm_qos_request_active(&pm_qos_req_cpu[1])) {
        exynos_pm_qos_update_request(&pm_qos_req_cpu[1], INT_MAX);
    }
#endif
    return 0;
}

static int anc_pm_qos_opp_release(void) {
    ANC_LOGD("%s: Release normal QOS", __func__);

#if IS_ENABLED(CONFIG_EXYNOS_PM_QOS) || IS_ENABLED(CONFIG_EXYNOS_PM_QOS_MODULE)
    if (exynos_pm_qos_request_active(&pm_qos_req_int)) {
        exynos_pm_qos_update_request(&pm_qos_req_int, 0);
    }

    if (exynos_pm_qos_request_active(&pm_qos_req_mif)) {
        exynos_pm_qos_update_request(&pm_qos_req_mif, 0);
    }

    if (exynos_pm_qos_request_active(&pm_qos_req_cpu[0])) {
        exynos_pm_qos_update_request(&pm_qos_req_cpu[0], 0);
    }

    if (exynos_pm_qos_request_active(&pm_qos_req_cpu[1])) {
        exynos_pm_qos_update_request(&pm_qos_req_cpu[1], 0);
    }
#endif
    return 0;
}

static int anc_pm_qos_opp_hold_int_mif(void) {
    ANC_LOGD("%s: Request int and mif MAX QOS", __func__);

// dma memory frequency update to max
#if IS_ENABLED(CONFIG_EXYNOS_PM_QOS) || IS_ENABLED(CONFIG_EXYNOS_PM_QOS_MODULE)
    if (exynos_pm_qos_request_active(&pm_qos_req_int)) {
        // exynos_pm_qos_update_request(&pm_qos_req_int, INT_MAX);
        exynos_pm_qos_update_request(&pm_qos_req_int, 400000);
    }

    if (exynos_pm_qos_request_active(&pm_qos_req_mif)) {
        // exynos_pm_qos_update_request(&pm_qos_req_mif, INT_MAX);
        exynos_pm_qos_update_request(&pm_qos_req_mif, 2093000);
    }
#endif
    return 0;
}

static int anc_pm_qos_opp_release_int_mif(void) {
    ANC_LOGD("%s: Release int and mif normal QOS", __func__);

#if IS_ENABLED(CONFIG_EXYNOS_PM_QOS) || IS_ENABLED(CONFIG_EXYNOS_PM_QOS_MODULE)
    if (exynos_pm_qos_request_active(&pm_qos_req_int)) {
        exynos_pm_qos_update_request(&pm_qos_req_int, 0);
    }

    if (exynos_pm_qos_request_active(&pm_qos_req_mif)) {
        exynos_pm_qos_update_request(&pm_qos_req_mif, 0);
    }
#endif
    return 0;
}
#endif

static void set_irq_flag(struct anc_data *p_data, bool flag) {
    if (p_data == NULL) {
        ANC_LOGE("p_data is NULL");
        return;
    }
    ANC_LOGD("set irq flag mask : %d", flag);
    p_data->irq_mask_flag = flag;
}

static int init_irq(struct anc_data *p_data) {
    int ret_val = 0;

    if (p_data == NULL) {
        ANC_LOGE("p_data is NULL");
        return -EINVAL;
    }
    ANC_LOGD("init irq");
    ret_val = anc_irq_init(p_data);
    INIT_WORK(&p_data->work_queue, anc_do_irq_work);

    return ret_val;
}

static void deinit_irq(struct anc_data *p_data) {
    if (p_data == NULL) {
        ANC_LOGE("p_data is NULL");
        return;
    }

    ANC_LOGD("deinit irq");
    anc_irq_deinit(p_data);
    cancel_work_sync(&p_data->work_queue);
}

static int set_product_id(struct anc_data *p_data, const void *arg) {
    int ret_val = 0;

    ret_val = anc_copy_from_user(&(p_data->product_info), arg, sizeof(ANC_SENSOR_PRODUCT_INFO));
    if (ret_val != 0) {
        ANC_LOGE("Failed to copy product info from user to kernel");
        return ret_val;
    }
    ANC_LOGD("set product id: %s", p_data->product_info.product_id);
    custom_send_command(CUSTOM_COMMAND_SET_PRODUCT_ID, p_data->product_info.product_id);

    return ret_val;
}

static int get_irq_pin_level(struct anc_data *p_data, uint8_t *pin_level) {
    int ret_val = 0;
    int irq_pin_levl = 0;
    uint8_t irq_pin_levl_u8 = 0;

    irq_pin_levl = gpio_get_value(p_data->irq_gpio);
    if (irq_pin_levl >= 0) {
        ANC_LOGD("read irq pin level : %d", irq_pin_levl);
    } else {
        ANC_LOGE("read irq pin level fail, level:%d", irq_pin_levl);
        return -EINVAL;
    }

    irq_pin_levl_u8 = irq_pin_levl;
    ret_val = copy_to_user(pin_level, &irq_pin_levl_u8, sizeof(uint8_t));
    if (ret_val != 0) {
        ANC_LOGE("Failed to copy irq pin level from kernel to user");
        return ret_val;
    }
    ANC_LOGD("get irq pin level:%d", *pin_level);

    return ret_val;
}

static long anc_ioctl(struct file *filp, unsigned int cmd, unsigned long arg) {
    int ret_val = 0;
    struct anc_data *p_data = filp->private_data;
    if (p_data == NULL) {
        ANC_LOGE("get anc data failed");
        return -EINVAL;
    }

    if (_IOC_TYPE(cmd) != ANC_IOC_MAGIC) {
        ANC_LOGE("magic number does not match");
        return -ENOTTY;
    }

    ANC_LOGD("cmd = %d", _IOC_NR(cmd));

    switch (cmd) {
        case ANC_IOC_RESET:
            ret_val = anc_reset(p_data);
            break;
        case ANC_IOC_ENABLE_POWER:
            anc_power_onoff(p_data, true);
            break;
        case ANC_IOC_DISABLE_POWER:
            anc_power_onoff(p_data, false);
            break;
        case ANC_IOC_CLEAR_FLAG:
            custom_send_command(CUSTOM_COMMAND_CLEAR_TOUCH_STATE, NULL);
            break;
        case ANC_IOC_ENABLE_IRQ:
            anc_enable_irq(p_data);
            break;
        case ANC_IOC_DISABLE_IRQ:
            anc_disable_irq(p_data);
            break;
        case ANC_IOC_SET_IRQ_FLAG_MASK:
            set_irq_flag(p_data, true);
            break;
        case ANC_IOC_CLEAR_IRQ_FLAG_MASK:
            set_irq_flag(p_data, false);
            break;
#ifdef ANC_USE_REE_SPI
        case ANC_IOC_SPI_SPEED:
            ret_val = set_spi_speed(arg);
            break;
#endif
        case ANC_IOC_WAKE_LOCK:
            anc_wake_lock(p_data);
            break;
        case ANC_IOC_WAKE_UNLOCK:
            anc_wake_unlock(p_data);
            break;

#ifdef MTK_PLATFORM
        case ANC_IOC_ENABLE_SPI_CLK:
            spi_clk_enable(true);
            break;
        case ANC_IOC_DISABLE_SPI_CLK:
            spi_clk_enable(false);
            break;
#elif defined SAMSUNG_PLATFORM
        case ANC_IOC_ENABLE_SPI_CLK:
            anc_spi_clk_enable(p_data, ANC_LOW_SPEED);
            break;
        case ANC_IOC_DISABLE_SPI_CLK:
            anc_spi_clk_disable(p_data);
            break;
#endif
        case ANC_IOC_INIT_IRQ:
            ret_val = init_irq(p_data);
            break;
        case ANC_IOC_DEINIT_IRQ:
            deinit_irq(p_data);
            break;
        case ANC_IOC_REQUEST_RESOURCE:
            ret_val = anc_platform_init(p_data);
            break;
        case ANC_IOC_RELEASE_RESOURCE:
            ret_val = anc_platform_free(p_data);
            break;
#ifdef ANC_SUPPORT_NAVIGATION_EVENT
        case ANC_IOC_REPORT_KEY_EVENT:
            ret_val = anc_report_key_event(p_data, (const void *)arg);
            break;
#endif
        case ANC_IOC_SET_PRODUCT_ID:
            ret_val = set_product_id(p_data, (const void *)arg);
            break;
        case ANC_IOC_GET_IRQ_PIN_LEVEL:
            ret_val = get_irq_pin_level(p_data, (uint8_t *)arg);
            break;
        default:
            ret_val = -EINVAL;
            break;
    }

    return ret_val;
}

#ifdef CONFIG_COMPAT
static long anc_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg) {
    return anc_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#endif

static const struct file_operations anc_fops = {
    .owner = THIS_MODULE,
    .open = anc_open,
    .unlocked_ioctl = anc_ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl = anc_compat_ioctl,
#endif
#ifdef ANC_USE_REE_SPI
    .write = anc_write,
    .read = anc_read,
#endif
};

static int anc_malloc(struct anc_data **pp_data, struct device *dev) {
    if ((pp_data == NULL) || (dev == NULL)) {
        ANC_LOGE("parameter is NULL, pp_data:%p, dev:%p", pp_data, dev);
        return -EINVAL;
    }

    /* Allocate device data */
    *pp_data = devm_kzalloc(dev, sizeof(struct anc_data), GFP_KERNEL);
    if (*pp_data == NULL) {
        ANC_LOGE("Failed to allocate memory for device data");
        return -ENOMEM;
    }

#ifdef ANC_USE_REE_SPI
    /* Allocate SPI transfer DMA buffer */
    spi_buffer = devm_kzalloc(dev, SPI_BUFFER_SIZE, GFP_KERNEL | GFP_DMA);
    if (!spi_buffer) {
        ANC_LOGE("Failed to allocate memory for spi buffer");
        return -ENOMEM;
    }
#endif

    return 0;
}

static void anc_free(struct anc_data *p_data, struct device *dev) {
    if (dev == NULL) {
        ANC_LOGE("dev is NULL");
        return;
    }

    if (p_data) {
        devm_kfree(dev, p_data);
        p_data = NULL;
    }

#ifdef ANC_USE_REE_SPI
    if (spi_buffer) {
        devm_kfree(dev, spi_buffer);
        spi_buffer = NULL;
    }
#endif
}

static int anc_create_device(struct anc_data *p_data) {
    int ret_val = 0;
    struct device *device_ptr = NULL;

    if (p_data == NULL) {
        ANC_LOGE("p_data is NULL");
        return -EINVAL;
    }
    p_data->dev_class = class_create(THIS_MODULE, ANC_DEVICE_NAME);
    if (IS_ERR(p_data->dev_class)) {
        ANC_LOGE("class_create failed");
        return -ENODEV;
    }

    ret_val = alloc_chrdev_region(&p_data->dev_num, 0, 1, ANC_DEVICE_NAME);
    if (ret_val < 0) {
        ANC_LOGE("Failed to alloc char device region, ret_val: %d", ret_val);
        goto device_region_err;
    }
    ANC_LOGD("Major number of device = %d", MAJOR(p_data->dev_num));

    device_ptr = device_create(p_data->dev_class, NULL, p_data->dev_num, p_data, ANC_DEVICE_NAME);
    if (IS_ERR(device_ptr)) {
        ANC_LOGE("Failed to create char device");
        ret_val = -ENODEV;
        goto device_create_err;
    }

    cdev_init(&p_data->cdev, &anc_fops);
    p_data->cdev.owner = THIS_MODULE;
    ret_val = cdev_add(&p_data->cdev, p_data->dev_num, 1);
    if (ret_val < 0) {
        ANC_LOGE("Failed to add char device");
        goto cdev_add_err;
    }

    return 0;

cdev_add_err:
    device_destroy(p_data->dev_class, p_data->dev_num);
device_create_err:
    unregister_chrdev_region(p_data->dev_num, 1);
device_region_err:
    class_destroy(p_data->dev_class);
    return ret_val;
}

static void anc_destroy_device(struct anc_data *p_data) {
    if (p_data == NULL) {
        ANC_LOGE("p_data is NULL");
        return;
    }

    cdev_del(&p_data->cdev);
    device_destroy(p_data->dev_class, p_data->dev_num);
    unregister_chrdev_region(p_data->dev_num, 1);
    class_destroy(p_data->dev_class);
}

#ifdef ANC_SUPPORT_NAVIGATION_EVENT
static int anc_input_init(struct anc_data *p_data) {
    int rc = 0;

    if (p_data == NULL) {
        ANC_LOGE("p_data is NULL");
        return -EINVAL;
    }
    p_data->input = input_allocate_device();
    if (!p_data->input) {
        ANC_LOGE("Failed to allocate input device");
        return -ENOMEM;
    }

    p_data->input->name = ANC_INPUT_NAME;
    __set_bit(EV_KEY, p_data->input->evbit);
    __set_bit(KEY_HOME, p_data->input->keybit);
    __set_bit(KEY_MENU, p_data->input->keybit);
    __set_bit(KEY_BACK, p_data->input->keybit);
    __set_bit(KEY_UP, p_data->input->keybit);
    __set_bit(KEY_DOWN, p_data->input->keybit);
    __set_bit(KEY_LEFT, p_data->input->keybit);
    __set_bit(KEY_RIGHT, p_data->input->keybit);
    __set_bit(KEY_POWER, p_data->input->keybit);
    __set_bit(KEY_WAKEUP, p_data->input->keybit);
    __set_bit(KEY_CAMERA, p_data->input->keybit);

    rc = input_register_device(p_data->input);
    if (rc) {
        ANC_LOGE("Failed to register input device");
        input_free_device(p_data->input);
        p_data->input = NULL;
        return -ENODEV;
    }

    return rc;
}
#endif

static int anc_parse_dts(struct anc_data *p_data) {
    int ret_val = 0;

    if (p_data == NULL) {
        ANC_LOGE("p_data is NULL");
        return -EINVAL;
    }
    p_data->vdd_use_gpio = of_property_read_bool(p_data->dev->of_node, "anc,vdd_use_gpio");
    p_data->vdd_use_pmic = of_property_read_bool(p_data->dev->of_node, "anc,vdd_use_pmic");
    p_data->use_gpio_init = of_property_read_bool(p_data->dev->of_node, "anc,use_gpio_init");
    ANC_LOGD("vdd_use_gpio = %d, vdd_use_pmic = %d, use_gpio_init = %d",
             p_data->vdd_use_gpio,
             p_data->vdd_use_pmic,
             p_data->use_gpio_init);

    if (p_data->vdd_use_pmic) {
        vreg_conf.name = ANC_VREG_VDD_NAME;
        ret_val = of_property_read_u32_index(
            p_data->dev->of_node, ANC_VDD_CONFIG_NAME, 0, &vreg_conf.vmin);
        if (ret_val != 0) {
            ANC_LOGE("read anc,vdd_config index 0 failed");
            return ret_val;
        }

        ret_val = of_property_read_u32_index(
            p_data->dev->of_node, ANC_VDD_CONFIG_NAME, 1, &vreg_conf.vmax);
        if (ret_val != 0) {
            ANC_LOGE("read anc,vdd_config index 1 failed");
            return ret_val;
        }

        ret_val = of_property_read_u32_index(
            p_data->dev->of_node, ANC_VDD_CONFIG_NAME, 2, &vreg_conf.ua_load);
        if (ret_val != 0) {
            ANC_LOGE("read anc,vdd_config index 2 failed");
            return ret_val;
        }
        ANC_LOGD("vreg config: name = %s, vmin = %d, vmax = %d, ua_load = %d",
                 vreg_conf.name,
                 vreg_conf.vmin,
                 vreg_conf.vmax,
                 vreg_conf.ua_load);
    }

    return ret_val;
}

static int anc_probe(anc_device_t *pdev) {
    struct device *dev = &pdev->dev;
    struct anc_data *p_data = NULL;
    int ret_val = 0;

    ANC_LOGD("entry");

    ret_val = anc_malloc(&p_data, dev);
    if (ret_val != 0) {
        goto out_free;
    }
    p_data->dev = dev;
    p_data->resource_requested = false;
    g_anc_spi_device = pdev;
    p_data->irq_mask_flag = false;
    p_data->irq_init = 0;
    dev_set_drvdata(dev, p_data);

    ret_val = custom_send_command(CUSTOM_COMMAND_INIT, p_data);
    if (ret_val != 0) {
        ANC_LOGE("custom init failed");
        goto out_free;
    }

    ret_val = anc_parse_dts(p_data);
    if (ret_val != 0) {
        ANC_LOGE("parse dts failed");
        goto out_deinit_custom;
    }

    ret_val = anc_create_device(p_data);
    if (ret_val != 0) {
        goto out_deinit_custom;
    }

    mutex_init(&p_data->lock);

#ifdef CONFIG_PM_WAKELOCKS
    p_data->fp_wakelock = wakeup_source_register(dev, "anc_fp_wakelock");
#else
    wake_lock_init(&p_data->fp_wakelock, WAKE_LOCK_SUSPEND, "anc_fp_wakelock");
#endif

#ifdef ANC_SUPPORT_NAVIGATION_EVENT
    ret_val = anc_input_init(p_data);
    if (ret_val != 0) {
        ANC_LOGE("Failed to init input dev");
        goto out_destroy_device;
    }
#endif

#ifdef ANC_USE_REE_SPI
    ret_val = anc_config_spi();
    if (ret_val != 0) {
        ANC_LOGE("config spi failed");
        goto out;
    }
#endif

    ANC_LOGD("Create sysfs path = %s", (&dev->kobj)->name);
    ret_val = sysfs_create_group(&dev->kobj, &attribute_group);
    if (ret_val != 0) {
        ANC_LOGE("Could not create sysfs");
        goto out;
    }

    ANC_LOGD("Success");
    return 0;

out:
#ifdef ANC_SUPPORT_NAVIGATION_EVENT
    input_unregister_device(p_data->input);
out_destroy_device:
#endif
    anc_destroy_device(p_data);
out_deinit_custom:
    custom_send_command(CUSTOM_COMMAND_DEINIT, NULL);
out_free:
    anc_free(p_data, dev);
    ANC_LOGE("Probe Failed, ret_val = %d", ret_val);
    return ret_val;
}

static int anc_remove(anc_device_t *pdev) {
    struct anc_data *p_data = dev_get_drvdata(&pdev->dev);
    if (p_data == NULL) {
        ANC_LOGE("get data handle failed");
        return -EINVAL;
    }

    sysfs_remove_group(&pdev->dev.kobj, &attribute_group);
    mutex_destroy(&p_data->lock);

#ifdef CONFIG_PM_WAKELOCKS
    wakeup_source_unregister(p_data->fp_wakelock);
#else
    wake_lock_destroy(&p_data->fp_wakelock);
#endif

    custom_send_command(CUSTOM_COMMAND_DEINIT, NULL);
    anc_destroy_device(p_data);
    anc_free(p_data, &pdev->dev);
    return 0;
}

static void anc_shutdown(anc_device_t *pdev) {
    struct anc_data *p_data = NULL;

    ANC_LOGD("entry");
    p_data = dev_get_drvdata(&pdev->dev);
    if (p_data == NULL) {
        ANC_LOGE("get data handle failed");
        return;
    }

    anc_power_onoff(p_data, false);
}

static struct of_device_id anc_of_match[] = {
    {
        .compatible = ANC_COMPATIBLE_SW_FP,
    },
    {},
};
MODULE_DEVICE_TABLE(of, anc_of_match);

static anc_driver_t anc_driver = {
    .driver =
        {
            .name = ANC_DEVICE_NAME,
            .owner = THIS_MODULE,
            .of_match_table = anc_of_match,
#if defined(MTK_PLATFORM)
            .bus = &spi_bus_type,
#endif
        },
    .probe = anc_probe,
    .remove = anc_remove,
    .shutdown = anc_shutdown,
};

static int __init ancfp_init(void) {
    int ret_val = 0;

    ANC_LOGD("entry");

#if defined(ANC_USE_REE_SPI) || defined(MTK_PLATFORM)
    ret_val = spi_register_driver(&anc_driver);
#else
    ret_val = platform_driver_register(&anc_driver);
#endif
    if (!ret_val) {
        ANC_LOGD("success");
    } else {
        ANC_LOGE("register spi driver failed, ret_val:%d", ret_val);
        return ret_val;
    }

    anc_netlink_init();
    return ret_val;
}

static void __exit ancfp_exit(void) {
    ANC_LOGD("entry");

    anc_netlink_exit();
#if defined(ANC_USE_REE_SPI) || defined(MTK_PLATFORM)
    spi_unregister_driver(&anc_driver);
#else
    platform_driver_unregister(&anc_driver);
#endif
}

module_init(ancfp_init);
module_exit(ancfp_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("JIIOV");
MODULE_DESCRIPTION("JIIOV fingerprint sensor device driver");
