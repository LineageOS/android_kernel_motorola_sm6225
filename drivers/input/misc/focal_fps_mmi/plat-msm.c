/**
 * plat-msm.c
 *
**/

#include <linux/stddef.h>
#include <linux/bug.h>
#include <linux/delay.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>

#include "ff_log.h"
#include "ff_ctl.h"

# undef LOG_TAG
#define LOG_TAG "msm"

#define FF_COMPATIBLE_NODE "focaltech,fingerprint"

/* Define pinctrl state types. */
typedef enum {
    FF_PINCTRL_STATE_PWR_ACT,
    FF_PINCTRL_STATE_PWR_CLR,
    FF_PINCTRL_STATE_RST_CLR,
    FF_PINCTRL_STATE_RST_ACT,
    FF_PINCTRL_STATE_INT_ACT,
    FF_PINCTRL_STATE_MAXIMUM /* Array size */
} ff_pinctrl_state_t;

/* Define pinctrl state names. */
static const char *g_pinctrl_state_names[FF_PINCTRL_STATE_MAXIMUM] = {
    "fpsensor_finger_power_high","fpsensor_finger_power_low","fpsensor_finger_rst_low","fpsensor_finger_rst_high","fpsensor_eint_as_int"
};

/* Native context and its singleton instance. */
typedef struct {
    struct pinctrl *pinctrl;
    struct pinctrl_state *pin_states[FF_PINCTRL_STATE_MAXIMUM];
} ff_msm_context_t;
static ff_msm_context_t ff_msm_context, *g_context = &ff_msm_context;

int ff_ctl_enable_power(bool on);

int ff_ctl_init_pins(int *irq_num)
{
    int err = 0, i;
    int irq_num1 = 0;
    struct device_node *dev_node = NULL;
    struct platform_device *pdev = NULL;

    printk("'%s' enter.", __func__);

    /* Find device tree node. */
    dev_node = of_find_compatible_node(NULL, NULL, FF_COMPATIBLE_NODE);
    if (!dev_node) {
        printk("of_find_compatible_node(.., '%s') failed.", FF_COMPATIBLE_NODE);
        return (-ENODEV);
    }
    printk("dev_node :%s",dev_node->name);

    irq_num1 = irq_of_parse_and_map(dev_node, 0);
    *irq_num = irq_num1;
    printk("irq number is %d.", irq_num1);

    /* Convert to platform device. */
    pdev = of_find_device_by_node(dev_node);
    if (!pdev) {
        printk("of_find_device_by_node(..) failed.");
        return (-ENODEV);
    }

    /* Retrieve the pinctrl handler. */
    g_context->pinctrl = devm_pinctrl_get(&pdev->dev);
    if (!g_context->pinctrl) {
        printk("devm_pinctrl_get(..) failed.");
        return (-ENODEV);
    }

    printk("register pins.");
    /* Register all pins. */
    for (i = 0; i < FF_PINCTRL_STATE_MAXIMUM; ++i) {
        g_context->pin_states[i] = pinctrl_lookup_state(g_context->pinctrl, g_pinctrl_state_names[i]);
        if (!g_context->pin_states[i]) {
            printk("can't find pinctrl state for '%s'.", g_pinctrl_state_names[i]);
            err = (-ENODEV);
            break;
        }
    }
    if (i < FF_PINCTRL_STATE_MAXIMUM) {
        return (-ENODEV);
    }

    /* Initialize the INT pin. */
    printk("init int pin.");
    err = pinctrl_select_state(g_context->pinctrl, g_context->pin_states[FF_PINCTRL_STATE_INT_ACT]);

    /* Initialize the RST pin. */
    pinctrl_select_state(g_context->pinctrl, g_context->pin_states[FF_PINCTRL_STATE_RST_ACT]);

    ff_ctl_enable_power(true);

    printk("'%s' leave.", __func__);
    return err;
}

int ff_ctl_free_pins(void)
{
    int err = 0;
    printk("'%s' enter.", __func__);

    // TODO:
	if (g_context->pinctrl) {
        pinctrl_put(g_context->pinctrl);
        g_context->pinctrl = NULL;
    }
    printk("'%s' leave.", __func__);
    return err;
}

int ff_ctl_enable_spiclk(bool on)
{
    int err = 0;
    FF_LOGV("'%s' enter.", __func__);
    FF_LOGD("clock: '%s'.", on ? "enable" : "disabled");

    if (on) {
        // TODO:
    } else {
        // TODO:
    }

    FF_LOGV("'%s' leave.", __func__);
    return err;
}

int ff_ctl_enable_power(bool on)
{
    int err = 0;
    printk("'%s' enter.", __func__);
    printk("power: '%s'.", on ? "on" : "off");

    if (unlikely(!g_context->pinctrl)) {
        return (-ENOSYS);
    }

    if (on) {
        err = pinctrl_select_state(g_context->pinctrl, g_context->pin_states[FF_PINCTRL_STATE_PWR_ACT]);
    } else {
        err = pinctrl_select_state(g_context->pinctrl, g_context->pin_states[FF_PINCTRL_STATE_PWR_CLR]);
    }

    printk("'%s' leave.", __func__);
    return err;
}

int ff_ctl_reset_device(void)
{
    int err = 0;
    printk("'%s' enter.", __func__);

    if (unlikely(!g_context->pinctrl)) {
        return (-ENOSYS);
    }

	err = pinctrl_select_state(g_context->pinctrl, g_context->pin_states[FF_PINCTRL_STATE_RST_ACT]);
	mdelay(1);
    /* 3-1: Pull down RST pin. */
	err = pinctrl_select_state(g_context->pinctrl, g_context->pin_states[FF_PINCTRL_STATE_RST_CLR]);

    /* 3-2: Delay for 10ms. */
    mdelay(10);

    /* Pull up RST pin. */
    err = pinctrl_select_state(g_context->pinctrl, g_context->pin_states[FF_PINCTRL_STATE_RST_ACT]);

    printk("'%s' leave.", __func__);
    return err;
}

const char *ff_ctl_arch_str(void)
{
    return "msm";
}

