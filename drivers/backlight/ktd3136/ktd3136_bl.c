/*
 * KTD3136_BL Driver
 *
 * SiliconMitus KTD3136 Backlight driver chip
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/types.h>
#include <linux/regulator/consumer.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/backlight.h>
#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#endif

#include "ktd3136_bl.h"

#define KTD3136_LED_DEV 	"ktd3136-BL"
#define KTD3136_NAME 		"ktd3136-bl"
#define KTD3136_CHIP_ID		0x18

static int platform_read_i2c_block(struct i2c_client *client, char *writebuf,
			   int writelen, char *readbuf, int readlen)
{
	int ret;

	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
				 .addr = client->addr,
				 .flags = 0,
				 .len = writelen,
				 .buf = writebuf,
			 },
			{
				 .addr = client->addr,
				 .flags = I2C_M_RD,
				 .len = readlen,
				 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret < 0)
			dev_err(&client->dev, "%s: i2c read error.\n",
				__func__);
	} else {
		struct i2c_msg msgs[] = {
			{
				 .addr = client->addr,
				 .flags = I2C_M_RD,
				 .len = readlen,
				 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0)
			dev_err(&client->dev, "%s:i2c read error.\n", __func__);
	}
	return ret;
}

static int ktd3136_read_reg(struct i2c_client *client, u8 addr, u8 *val)
{
	return platform_read_i2c_block(client, &addr, 1, val, 1);
}

static int platform_write_i2c_block(struct i2c_client *client, char *writebuf, int writelen)
{
	int ret;

	struct i2c_msg msgs[] = {
		{
			 .addr = client->addr,
			 .flags = 0,
			 .len = writelen,
			 .buf = writebuf,
		 },
	};
	ret = i2c_transfer(client->adapter, msgs, 1);
	if (ret < 0)
		dev_err(&client->dev, "%s: i2c write error.\n", __func__);

	return ret;
}

static int ktd3136_write_reg(struct i2c_client *client, u8 addr, const u8 val)
{
	u8 buf[2] = {0};

	buf[0] = addr;
	buf[1] = val;
	return platform_write_i2c_block(client, buf, sizeof(buf));
}
static void ktd3136_hwen_pin_ctrl(struct ktd3136_data *drvdata, int en)
{
	if (gpio_is_valid(drvdata->hwen_gpio)) {
		if (en) {
			pr_debug("hwen pin is going to be high!---<%d>\n", en);
			gpio_set_value(drvdata->hwen_gpio, true);
		} else {
			pr_debug("hwen pin is going to be low!---<%d>\n", en);
			gpio_set_value(drvdata->hwen_gpio, false);
		}
		msleep(1);
	}
}

static int ktd3136_gpio_init(struct ktd3136_data *drvdata)
{
	int ret;

	pr_info("%s enter\n", __func__);
	if (gpio_is_valid(drvdata->hwen_gpio)) {
		ret = gpio_request(drvdata->hwen_gpio, "ktd_hwen_gpio");
		if (ret<0) {
			pr_err("failed to request gpio\n");
			return -1;
		}
		ret = gpio_direction_output(drvdata->hwen_gpio, 0);
		pr_debug(" request gpio init\n");
		if (ret<0) {
			pr_err("failed to set output");
			gpio_free(drvdata->hwen_gpio);
			return ret;
		}
		pr_debug("gpio is valid!\n");
		ktd3136_hwen_pin_ctrl(drvdata, 1);
	}

	pr_info("%s exit\n", __func__);
	return 0;
}
static int ktd3136_masked_write(struct i2c_client *client, int reg, u8 mask, u8 val)
{
	int rc;
	u8 temp;

	rc = ktd3136_read_reg(client, reg, &temp);
	if (rc < 0) {
		pr_err("failed to read reg=0x%x, rc=%d\n", reg, rc);
	} else {
		temp &= ~mask;
		temp |= val & mask;
		rc = ktd3136_write_reg(client, reg, temp);
		if (rc<0) {
			pr_err( "failed to write masked data. reg=%03x, rc=%d\n", reg, rc);
		}
	}

	ktd3136_read_reg(client, reg, &temp);
	return rc;
}

static int ktd3136_bl_enable_channel(struct ktd3136_data *drvdata)
{
	int ret = -1;

	if (drvdata->channel == 0) {
		//default value for mode Register, all channel disabled.
		pr_debug("all channels are going to be disabled\n");
		ret = ktd3136_write_reg(drvdata->client, REG_PWM, 0x18);//b0001 1000
	} else if (drvdata->channel == 3) {
		pr_debug("turn all channel on!\n");
		ret = ktd3136_masked_write(drvdata->client, REG_PWM, 0x07, 0x07);
	} else if (drvdata->channel == 2) {
		ret = ktd3136_masked_write(drvdata->client, REG_PWM, 0x07, 0x03);
	}

	return ret;
}
static void ktd3136_pwm_mode_enable(struct ktd3136_data *drvdata, bool en)
{
	u8 value;

	if (en) {
		if (drvdata->pwm_mode) {
			pr_debug("already activated!\n");
		} else {
			drvdata->pwm_mode = en;
		}
		ktd3136_masked_write(drvdata->client, REG_PWM, 0x80, 0x80);
	} else {
		if (drvdata->pwm_mode) {
			drvdata->pwm_mode = en;
		}
		ktd3136_masked_write(drvdata->client, REG_PWM, 0x80, 0x00);
	}

	ktd3136_read_reg(drvdata->client, REG_PWM, &value);
	pr_debug("current pwm_mode is --<%x>\n", value);
}
static int ktd_find_bit(int x)
{
	int i = 0;

	while ((x = x >> 1))
		i++;

	return i+1;
}

static void ktd3136_ramp_setting(struct ktd3136_data *drvdata)
{
	unsigned int max_time = 16384;
	int temp = 0;

	if (drvdata->ramp_on_time == 0) {//512us
		ktd3136_masked_write(drvdata->client, REG_RAMP_ON, 0xf0, 0x00);
		pr_debug("rampon time is 0 \n");
	} else if (drvdata->ramp_on_time > max_time) {
		ktd3136_masked_write(drvdata->client, REG_RAMP_ON, 0xf0, 0xf0);
		pr_debug("rampon time is max \n");
	} else {
		temp = ktd_find_bit(drvdata->ramp_on_time);
		ktd3136_masked_write(drvdata->client, REG_RAMP_ON, 0xf0, temp<<4);
		pr_debug("temp is %d\n", temp);
	}

	if (drvdata->ramp_off_time == 0) {//512us
		ktd3136_masked_write(drvdata->client, REG_RAMP_ON, 0x0f, 0x00);
		pr_debug("rampoff time is 0 \n");
	} else if (drvdata->ramp_off_time > max_time) {
		ktd3136_masked_write(drvdata->client, REG_RAMP_ON, 0x0f, 0x0f);
		pr_debug("rampoff time is max \n");
	} else {
		temp = ktd_find_bit(drvdata->ramp_off_time);
		ktd3136_masked_write(drvdata->client, REG_RAMP_ON, 0x0f, temp);
		pr_debug("temp is %d\n", temp);
	}

}
static void ktd3136_transition_ramp(struct ktd3136_data *drvdata)
{
	int reg_i2c, reg_pwm, temp;

	if (drvdata->i2c_trans_dim >= 1024) {
		reg_i2c = 0xf;
	} else if (drvdata->i2c_trans_dim < 128) {
		reg_i2c = 0x0;
	} else {
		temp =drvdata->i2c_trans_dim/64;
		reg_i2c = temp-1;
		pr_debug("reg_i2c is --<0x%x>\n", reg_i2c);
	}

	if(drvdata->pwm_trans_dim >= 256){
		reg_pwm = 0x7;
	}else if(drvdata->pwm_trans_dim < 4){
		reg_pwm = 0x0;
	}else{
		temp = ktd_find_bit(drvdata->pwm_trans_dim);
		reg_pwm = temp -2;
		pr_debug("temp is %d\n", temp);
	}

	ktd3136_masked_write(drvdata->client, REG_TRANS_RAMP, 0x70, reg_pwm);
	ktd3136_masked_write(drvdata->client, REG_TRANS_RAMP, 0x0f, reg_i2c);

}

static int ktd3136_backlight_init(struct ktd3136_data *drvdata)
{
	int err = 0;
	u8 value;
	u8 update_value;
	pr_info("%s enter.\n", __func__);
	update_value = (drvdata->ovp_level == 32) ? 0x20 : 0x00;
	(drvdata->induct_current == 2600) ? update_value |=0x08 : update_value;
	(drvdata->frequency == 1000) ? update_value |=0x40: update_value;

	ktd3136_write_reg(drvdata->client, REG_CONTROL, update_value | 0x06); /* Linear default*/
	ktd3136_bl_enable_channel(drvdata);
		if (drvdata->pwm_mode) {
			ktd3136_pwm_mode_enable(drvdata, true);
		} else {
			ktd3136_pwm_mode_enable(drvdata, false);
			}
	ktd3136_ramp_setting(drvdata);
	ktd3136_transition_ramp(drvdata);
	ktd3136_read_reg(drvdata->client, REG_CONTROL, &value);
	pr_debug("read control register -before--<0x%x> -after--<0x%x> \n",
					update_value, value);

	pr_info("%s exit\n", __func__);
	return err;
}

static int ktd3136_backlight_enable(struct ktd3136_data *drvdata)
{
	int err = 0;
	pr_info("%s enter.\n", __func__);
	ktd3136_masked_write(drvdata->client, REG_MODE, 0xf8, drvdata->full_scale_led);
	drvdata->enable = true;

	return err;
}

int ktd3136_set_brightness(struct ktd3136_data *drvdata, int brt_val)
{
	pr_info("%s brt_val is %d\n", __func__, brt_val);

	if (drvdata->enable == false)
		ktd3136_backlight_init(drvdata);

	if (brt_val>0) {
		ktd3136_masked_write(drvdata->client, REG_MODE, 0x01, 0x01); //enalbe bl mode
	} else {
		ktd3136_masked_write(drvdata->client, REG_MODE, 0x01, 0x00); //disable bl mode
	}
	if (drvdata->using_lsb) {
		ktd3136_masked_write(drvdata->client, REG_RATIO_LSB, 0x07, brt_val);
		ktd3136_masked_write(drvdata->client, REG_RATIO_MSB, 0xff, brt_val>>3);
	} else {
		ktd3136_masked_write(drvdata->client, REG_RATIO_LSB, 0x07, ktd3136_brightness_table_reg4[brt_val]);
		ktd3136_masked_write(drvdata->client, REG_RATIO_MSB, 0xff, ktd3136_brightness_table_reg5[brt_val]);
	}

	if (drvdata->enable == false)
		ktd3136_backlight_enable(drvdata);

	drvdata->brightness = brt_val;

	if (drvdata->brightness == 0)
		drvdata->enable = false;

	return 0;
}

#ifdef KERNEL_ABOVE_4_14
static int ktd3136_bl_get_brightness(struct backlight_device *bl_dev)
{
		return bl_dev->props.brightness;
}

static int ktd3136_bl_update_status(struct backlight_device *bl_dev)
{
		struct ktd3136_data *drvdata = bl_get_data(bl_dev);
		int brt;

		if (bl_dev->props.state & BL_CORE_SUSPENDED)
				bl_dev->props.brightness = 0;

		brt = bl_dev->props.brightness;
		/*
		 * Brightness register should always be written
		 * not only register based mode but also in PWM mode.
		 */
		return ktd3136_set_brightness(drvdata, brt);
}

static const struct backlight_ops ktd3136_bl_ops = {
		.update_status = ktd3136_bl_update_status,
		.get_brightness = ktd3136_bl_get_brightness,
};
#endif

static int ktd3136_backlight_reset(struct ktd3136_data *drvdata)
{
	int err = 0;
	ktd3136_masked_write(drvdata->client, REG_SW_RESET, 0x01, 0x01);
	return err;
}

static int ktd3136_check_id(struct ktd3136_data *drvdata)
{
	u8 value=0;
	int err = 0;
	ktd3136_read_reg(drvdata->client, 0x00, &value);
	pr_info("%s: ID check: %x0x\n", __func__, value);
	if (value != KTD3136_CHIP_ID) {
		pr_err("%s : ID check err\n", __func__);
		err = -EINVAL;
		return err;
	}

	return err;
}

static void ktd3136_check_status(struct ktd3136_data *drvdata)
{
	u8 value=0;

	ktd3136_read_reg(drvdata->client, REG_STATUS, &value);
	if (value) {
		pr_err("status bit has been change! <%x>", value);

		if (value & RESET_CONDITION_BITS) {
			ktd3136_backlight_reset(drvdata);
			ktd3136_backlight_init(drvdata);
			ktd3136_backlight_enable(drvdata);
		}

	}
	return ;
}

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
				       unsigned long event, void *data)
{
	int *blank;
	struct fb_event *evdata = data;
	struct ktd3136_data *drvdata =
		container_of(self, struct ktd3136_data, fb_notif);

	/*
	 *  FB_EVENT_BLANK(0x09): A hardware display blank change occurred.
	 *  FB_EARLY_EVENT_BLANK(0x10): A hardware display blank early change
	 * occurred.
	 */
	if (evdata && evdata->data && (event == FB_EARLY_EVENT_BLANK)) {
		blank = evdata->data;
		if (*blank == FB_BLANK_POWERDOWN)
			drvdata->enable = false;
	}

	return NOTIFY_OK;
}
#endif

static void __ktd3136_work(struct ktd3136_data *led,
				enum led_brightness value)
{
	mutex_lock(&led->lock);
	ktd3136_set_brightness(led, value);
	mutex_unlock(&led->lock);
}

static void ktd3136_work(struct work_struct *work)
{
	struct ktd3136_data *drvdata = container_of(work,
					struct ktd3136_data, work);

	__ktd3136_work(drvdata, drvdata->led_dev.brightness);

	return;
}


static void ktd3136_brightness_set(struct led_classdev *led_cdev,
			enum led_brightness brt_val)
{
	struct ktd3136_data *drvdata;

	drvdata = container_of(led_cdev, struct ktd3136_data, led_dev);

	schedule_work(&drvdata->work);
}

static void ktd3136_get_dt_data(struct device *dev, struct ktd3136_data *drvdata)
{
	int rc;
	struct device_node *np = dev->of_node;
	u32 bl_channel, temp;
	drvdata->hwen_gpio = of_get_named_gpio(np, "ktd,hwen-gpio", 0);
	pr_info("%s hwen --<%d>\n", __func__, drvdata->hwen_gpio);

	drvdata->pwm_mode = of_property_read_bool(np,"ktd,pwm-mode");
	pr_debug("pwmmode --<%d> \n", drvdata->pwm_mode);

	drvdata->using_lsb = of_property_read_bool(np, "ktd,using-lsb");
	pr_info("%s using_lsb --<%d>\n", __func__, drvdata->using_lsb);

	if (drvdata->using_lsb) {
		drvdata->default_brightness = 0x7ff;
		drvdata->max_brightness = 2047;
	} else {
		drvdata->default_brightness = 0xff;
		drvdata->max_brightness = 255;
	}
	rc = of_property_read_u32(np, "ktd,pwm-frequency", &temp);
	if (rc) {
		pr_err("Invalid pwm-frequency!\n");
	} else {
		drvdata->pwm_period = temp;
		pr_debug("pwm-frequency --<%d> \n", drvdata->pwm_period);
	}

	rc = of_property_read_u32(np, "ktd,bl-fscal-led", &temp);
	if (rc) {
		pr_err("Invalid backlight full-scale led current!\n");
	} else {
		drvdata->full_scale_led = temp;
		pr_debug("full-scale led current --<%d mA> \n", drvdata->full_scale_led);
	}

	rc = of_property_read_u32(np, "ktd,turn-on-ramp", &temp);
	if (rc) {
		pr_err("Invalid ramp timing ,,turnon!\n");
	} else {
		drvdata->ramp_on_time = temp;
		pr_debug("ramp on time --<%d ms> \n", drvdata->ramp_on_time);
	}

	rc = of_property_read_u32(np, "ktd,turn-off-ramp", &temp);
	if (rc) {
		pr_err("Invalid ramp timing ,,turnoff!\n");
	} else {
		drvdata->ramp_off_time = temp;
		pr_debug("ramp off time --<%d ms> \n", drvdata->ramp_off_time);
	}

	rc = of_property_read_u32(np, "ktd,pwm-trans-dim", &temp);
	if (rc) {
		pr_err("Invalid pwm-tarns-dim value!\n");
	}
	else {
		drvdata->pwm_trans_dim = temp;
		pr_debug("pwm trnasition dimming	--<%d ms> \n", drvdata->pwm_trans_dim);
	}

	rc = of_property_read_u32(np, "ktd,i2c-trans-dim", &temp);
	if (rc) {
		pr_err("Invalid i2c-trans-dim value !\n");
	} else {
		drvdata->i2c_trans_dim = temp;
		pr_debug("i2c transition dimming --<%d ms>\n", drvdata->i2c_trans_dim);
	}

	rc = of_property_read_u32(np, "ktd,bl-channel", &bl_channel);
	if (rc) {
		pr_err("Invalid channel setup\n");
	} else {
		drvdata->channel = bl_channel;
		pr_debug("bl-channel --<%x> \n", drvdata->channel);
	}

	rc = of_property_read_u32(np, "ktd,ovp-level", &temp);
	if (!rc) {
		drvdata->ovp_level = temp;
		pr_debug("ovp-level --<%d> --temp <%d>\n", drvdata->ovp_level, temp);
	}else
		pr_err("Invalid OVP level!\n");

	rc = of_property_read_u32(np, "ktd,switching-frequency", &temp);
	if (!rc) {
		drvdata->frequency = temp;
		pr_debug("switching frequency --<%d> \n", drvdata->frequency);
	} else {
		pr_err("Invalid Frequency value!\n");
	}

	rc = of_property_read_u32(np, "ktd,inductor-current", &temp);
	if (!rc) {
		drvdata->induct_current = temp;
		pr_debug("inductor current limit --<%d> \n", drvdata->induct_current);
	} else
		pr_err("invalid induct_current limit\n");

	rc = of_property_read_u32(np, "ktd,flash-timeout", &temp);
	if (!rc) {
		drvdata->flash_timeout = temp;
		pr_debug("flash timeout --<%d> \n", drvdata->flash_timeout);
	} else {
		pr_err("invalid flash-time value!\n");
	}

	rc = of_property_read_u32(np, "ktd,flash-current", &temp);
	if (!rc) {
		drvdata->flash_current = temp;
		pr_debug("flash current --<0x%x> \n", drvdata->flash_current);
	} else {
		pr_err("invalid flash current value!\n");
	}
}

static int ktd3136_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct ktd3136_data *drvdata;
#ifdef KERNEL_ABOVE_4_14
	struct backlight_device *bl_dev;
	struct backlight_properties props;
#endif
	int err = 0;

	pr_info("%s enter!\n", __func__);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s : I2C_FUNC_I2C not supported\n", __func__);
		err = -EIO;
		goto err_out;
	}

	if (!client->dev.of_node) {
		pr_err("%s : no device node\n", __func__);
		err = -ENOMEM;
		goto err_out;
	}

	drvdata = kzalloc(sizeof(struct ktd3136_data), GFP_KERNEL);
	if (drvdata == NULL) {
		pr_err("%s : kzalloc failed\n", __func__);
		err = -ENOMEM;
		goto err_out;
	}

	drvdata->client = client;
	drvdata->adapter = client->adapter;
#ifdef KTD3136_REG_CONFILCT
	pr_info("%s: client->addr=0x%x\n", __func__, client->addr);
	client->addr = KTD3136_REG_REAL;
	pr_info("%s: confilct, reset client->addr to 0x%x\n", __func__, client->addr);
#endif
	drvdata->addr = client->addr;
	drvdata->brightness = LED_OFF;
	drvdata->enable = true;
	drvdata->led_dev.default_trigger = "bkl-trigger";
	drvdata->led_dev.name = KTD3136_LED_DEV;
	drvdata->led_dev.brightness_set = ktd3136_brightness_set;
	drvdata->led_dev.max_brightness = MAX_BRIGHTNESS;
	ktd3136_get_dt_data(&client->dev, drvdata);
	i2c_set_clientdata(client, drvdata);
	err =ktd3136_check_id(drvdata);
	if (err <0) {
		pr_err("%s : ID idenfy failed\n", __func__);
		goto err_init;
	}

	mutex_init(&drvdata->lock);
	INIT_WORK(&drvdata->work, ktd3136_work);
	err = led_classdev_register(&client->dev, &drvdata->led_dev);
	if (err < 0) {
		pr_err("%s : Register led class failed\n", __func__);
		err = -ENODEV;
		goto err_init;
	} else {
		pr_info("%s: Register led class successful\n", __func__);
	}
	ktd3136_gpio_init(drvdata);

#ifdef KERNEL_ABOVE_4_14
	memset(&props, 0, sizeof(struct backlight_properties));
	props.type = BACKLIGHT_PLATFORM;
	props.brightness = MAX_BRIGHTNESS;
	props.max_brightness = MAX_BRIGHTNESS;
	bl_dev = backlight_device_register(KTD3136_NAME, &client->dev,
					drvdata, &ktd3136_bl_ops, &props);
#endif
	ktd3136_backlight_init(drvdata);
	ktd3136_backlight_enable(drvdata);
	ktd3136_check_status(drvdata);

#if defined(CONFIG_FB)
	drvdata->fb_notif.notifier_call = fb_notifier_callback;
	err = fb_register_client(&drvdata->fb_notif);
	if (err)
		pr_err("%s : Unable to register fb_notifier: %d\n", __func__, err);
#endif

	pr_info("%s exit\n", __func__);
	return 0;

err_init:
	pr_info("%s err_init\n", __func__);
	kfree(drvdata);
err_out:
	pr_info("%s err_out\n", __func__);
	return err;
}

static int ktd3136_remove(struct i2c_client *client)
{
	struct ktd3136_data *drvdata = i2c_get_clientdata(client);

	led_classdev_unregister(&drvdata->led_dev);

	kfree(drvdata);
	return 0;
}

static const struct i2c_device_id ktd3136_id[] = {
	{KTD3136_NAME, 0},
	{}
};
static struct of_device_id match_table[] = {
		{.compatible = "ktd,ktd3136",}
};

MODULE_DEVICE_TABLE(i2c, ktd3136_id);

static struct i2c_driver ktd3136_i2c_driver = {
	.probe = ktd3136_probe,
	.remove = ktd3136_remove,
	.id_table = ktd3136_id,
	.driver = {
		.name = KTD3136_NAME,
		.owner = THIS_MODULE,
		.of_match_table = match_table,
	},
};

module_i2c_driver(ktd3136_i2c_driver);
MODULE_DESCRIPTION("Backlight driver for ktd3136");
MODULE_LICENSE("GPL v2");
