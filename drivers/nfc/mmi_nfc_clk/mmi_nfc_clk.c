/*
 * Copyright (C) 2019 Motorola Mobility LLC
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/version.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/poll.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/of_gpio.h>
#include <linux/clk.h>

#define DRIVER_VERSION "0.0.1"

struct mmi_nfc_clk_drvdata {
	struct device	*dev;
	unsigned int clkreq_gpio;
	unsigned int clkreq_irq;
};

static irqreturn_t mmi_nfc_clk_gpio_isr(int irq, void *dev_id)
{
	// 0x00005346 LN_BB_CLK3_EN_CTL is set to 0x01
	//	Software control is off
	//	PC_POLARITY - pin control polarity - logic high enables clock
	//	FOLLOW_PC_EN - pin control enable is on
	//
	// PM660 GPIO 04 will trigger this ISR. As soon as MSM and PM660 is
	// awake BB_CLK3 is enabled. No additional action is needed here.
	return IRQ_HANDLED;
}

static int read_setup_clkreq_irq(struct platform_device *pdev,
	struct mmi_nfc_clk_drvdata *drvdata)
{
	int rc = 0;
	int gpio;
	int irq;
	const char *desc = "nfc_clkreq";

	gpio = of_get_named_gpio(pdev->dev.of_node, "mmi,clkreq_gpio", 0);
	if (gpio < 0) {
		rc =  gpio;
		dev_err(&pdev->dev, "probe: clkreq gpio not found, error=%d\n", rc);
		goto end;
	}
	drvdata->clkreq_gpio = gpio;

	rc = devm_gpio_request_one(&pdev->dev, drvdata->clkreq_gpio,
		GPIOF_IN, desc);

	if (rc < 0) {
		dev_err(&pdev->dev, "probe: gpio (%d) request failed, error %d\n",
			drvdata->clkreq_gpio, rc);
		goto end;
	}

	irq = gpio_to_irq(drvdata->clkreq_gpio);
	if (irq < 0) {
		rc = irq;
		dev_err(&pdev->dev, "probe: get irq failed for gpio %d, error %d\n",
			drvdata->clkreq_gpio, rc);
		goto end;
	}
	dev_dbg(&pdev->dev, "probe: gpio %d, irq %d\n", drvdata->clkreq_gpio, irq);

	drvdata->clkreq_irq = irq;
	rc = devm_request_irq(&pdev->dev, drvdata->clkreq_irq,
		mmi_nfc_clk_gpio_isr, IRQF_TRIGGER_RISING,
		desc, drvdata);

	if (rc < 0) {
		dev_err(&pdev->dev, "probe: unable to claim irq %d; error %d\n",
			drvdata->clkreq_irq, rc);
		goto end;
	}

	enable_irq_wake(drvdata->clkreq_irq);

end:
	return rc;
}


static int mmi_nfc_clk_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mmi_nfc_clk_drvdata *drvdata;
	int rc = 0;

	dev_dbg(&pdev->dev, "%s begin\n", __func__);
	drvdata = devm_kzalloc(dev, sizeof(*drvdata), GFP_KERNEL);
	if (!drvdata)
		return -ENOMEM;

	drvdata->dev = &pdev->dev;
	platform_set_drvdata(pdev, drvdata);

	rc = read_setup_clkreq_irq(pdev, drvdata);
	if (rc < 0) {
		dev_err(&pdev->dev, "probe: failed rc=%d\n", rc);
		return rc;
	}

	device_init_wakeup(&pdev->dev, 1);
	dev_info(&pdev->dev, "probe: success !\n");
	return rc;
}

static int mmi_nfc_clk_remove(struct platform_device *pdev)
{
	device_init_wakeup(&pdev->dev, 0);
	return 0;
}


static const struct of_device_id mmi_nfc_clk_match[] = {
	{ .compatible = "mmi,nfc-clk" },
	{}
};

static struct platform_driver mmi_nfc_clk_plat_driver = {
	.probe = mmi_nfc_clk_probe,
	.remove = mmi_nfc_clk_remove,
	.driver = {
		.name = "mmi_nfc_clk",
		.owner = THIS_MODULE,
		.of_match_table = mmi_nfc_clk_match,
	},
};

module_platform_driver(mmi_nfc_clk_plat_driver);


MODULE_AUTHOR("Motorola Mobiity");
MODULE_DESCRIPTION("NFC Clock Enable/Disable driver");
MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPL");
