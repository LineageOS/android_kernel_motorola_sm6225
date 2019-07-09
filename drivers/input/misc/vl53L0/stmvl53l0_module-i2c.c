/*
 *  stmvl53l0_module-i2c.c - Linux kernel modules for STM VL53L0 FlightSense TOF
 *							sensor
 *
 *  Copyright (C) 2016 STMicroelectronics Imaging Division.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 */
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/miscdevice.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/time.h>
#include <linux/platform_device.h>
/*
 * power specific includes
 */
#include <linux/pwm.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/clk.h>
#include <linux/of_gpio.h>
/*
 * API includes
 */
#include "vl53l0_api.h"
#include "vl53l0_def.h"
#include "vl53l0_platform.h"
#include "stmvl53l0-i2c.h"
#include "stmvl53l0.h"

#define LASER_SENSOR_PINCTRL_STATE_SLEEP "laser_suspend"
#define LASER_SENSOR_PINCTRL_STATE_DEFAULT "laser_default"

/*
 * Global data
 */
static int stmvl53l0_parse_vdd(struct device *dev, struct i2c_data *data);

/*
 * QCOM specific functions
 */
static int stmvl53l0_parse_vdd(struct device *dev, struct i2c_data *data)
{
	int ret = 0;

	vl53l0_dbgmsg("Enter\n");

	if (dev->of_node) {
		data->vana = regulator_get_optional(dev, "vdd");
		if (IS_ERR(data->vana)) {
			vl53l0_errmsg("vdd supply is not provided\n");
			ret = -1;
		}
	}
	vl53l0_dbgmsg("End\n");

	return ret;
}
int get_dt_xtalk_data(struct device_node *of_node, int *xtalk)
{
	int rc = 0;
	uint32_t count = 0;
	uint32_t v_array[1];

	count = of_property_count_strings(of_node, "st,xtalkval");

	if (!count)
		return 0;

	rc = of_property_read_u32_array(of_node, "st,xtalkval",
		v_array, 1);

	if (rc != -EINVAL) {
		if (rc < 0)
			pr_err("%s failed %d\n", __func__, __LINE__);
		else
			*xtalk = v_array[0];
	} else
		rc = 0;

	return rc;
}

static int stmvl53l0_get_dt_gpio_req_tbl(struct device_node *of_node,
	struct msm_camera_gpio_conf *gconf, uint16_t *gpio_array,
	uint16_t gpio_array_size)
{
	int rc = 0, i = 0;
	uint32_t count = 0;
	uint32_t *val_array = NULL;

	if (!of_get_property(of_node, "qcom,gpio-req-tbl-num", &count))
		return 0;

	count /= sizeof(uint32_t);
	if (!count) {
		pr_err("%s qcom,gpio-req-tbl-num 0\n", __func__);
		return 0;
	}

	val_array = kcalloc(count, sizeof(uint32_t), GFP_KERNEL);
	if (!val_array)
		return -ENOMEM;

	gconf->cam_gpio_req_tbl = kcalloc(count, sizeof(struct gpio),
		GFP_KERNEL);
	if (!gconf->cam_gpio_req_tbl) {
		rc = -ENOMEM;
		goto ERROR1;
	}
	gconf->cam_gpio_req_tbl_size = count;

	rc = of_property_read_u32_array(of_node, "qcom,gpio-req-tbl-num",
		val_array, count);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto ERROR2;
	}
	for (i = 0; i < count; i++) {
		if (val_array[i] >= gpio_array_size) {
			pr_err("%s gpio req tbl index %d invalid\n",
				__func__, val_array[i]);
			return -EINVAL;
		}
		gconf->cam_gpio_req_tbl[i].gpio = gpio_array[val_array[i]];
		vl53l0_dbgmsg("cam_gpio_req_tbl[%d].gpio = %d\n", i,
			gconf->cam_gpio_req_tbl[i].gpio);
	}

	rc = of_property_read_u32_array(of_node, "qcom,gpio-req-tbl-flags",
		val_array, count);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto ERROR2;
	}
	for (i = 0; i < count; i++) {
		gconf->cam_gpio_req_tbl[i].flags = val_array[i];
		vl53l0_dbgmsg("cam_gpio_req_tbl[%d].flags = %ld\n", i,
			gconf->cam_gpio_req_tbl[i].flags);
	}

	for (i = 0; i < count; i++) {
		rc = of_property_read_string_index(of_node,
			"qcom,gpio-req-tbl-label", i,
			&gconf->cam_gpio_req_tbl[i].label);
		vl53l0_dbgmsg("cam_gpio_req_tbl[%d].label = %s\n", i,
			gconf->cam_gpio_req_tbl[i].label);
		if (rc < 0) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			goto ERROR2;
		}
	}

	kfree(val_array);
	return rc;

ERROR2:
	kfree(gconf->cam_gpio_req_tbl);
ERROR1:
	kfree(val_array);
	gconf->cam_gpio_req_tbl_size = 0;
	return rc;
}

static int stmvl53l0_init_gpio_pin_tbl(struct device_node *of_node,
	struct msm_camera_gpio_conf *gconf, uint16_t *gpio_array,
	uint16_t gpio_array_size)
{
	int rc = 0, val = 0;

	gconf->gpio_num_info = kzalloc(sizeof(struct msm_camera_gpio_num_info),
		GFP_KERNEL);
	if (!gconf->gpio_num_info) {
		rc = -ENOMEM;
		return rc;
	}

	rc = of_property_read_u32(of_node, "qcom,gpio-vana", &val);
	if (rc != -EINVAL) {
		if (rc < 0) {
			pr_err("%s:%d read qcom,gpio-vana failed rc %d\n",
				__func__, __LINE__, rc);
			goto ERROR;
		} else if (val >= gpio_array_size) {
			pr_err("%s:%d qcom,gpio-vana invalid %d\n",
				__func__, __LINE__, val);
			rc = -EINVAL;
			goto ERROR;
		}
		gconf->gpio_num_info->gpio_num[1] =
			gpio_array[val];
		gconf->gpio_num_info->valid[1] = 1;
		vl53l0_dbgmsg("qcom,gpio-vana %d\n",
			gconf->gpio_num_info->gpio_num[1]);
	} else {
		rc = 0;
	}

	rc = of_property_read_u32(of_node, "qcom,gpio-reset", &val);
	if (rc != -EINVAL) {
		if (rc < 0) {
			pr_err("%s:%d read qcom,gpio-reset failed rc %d\n",
				__func__, __LINE__, rc);
			goto ERROR;
		} else if (val >= gpio_array_size) {
			pr_err("%s:%d qcom,gpio-reset invalid %d\n",
				__func__, __LINE__, val);
			rc = -EINVAL;
			goto ERROR;
		}
		gconf->gpio_num_info->gpio_num[0] =
			gpio_array[val];
		gconf->gpio_num_info->valid[0] = 1;
		vl53l0_dbgmsg("qcom,gpio-reset %d\n",
			gconf->gpio_num_info->gpio_num[0]);
	} else {
		rc = 0;
	}

	return rc;

ERROR:
	kfree(gconf->gpio_num_info);
	gconf->gpio_num_info = NULL;
	return rc;
}


static void stmvl53l0_request_gpio_table(struct gpio *gpio_tbl, uint8_t size,
	int gpio_en)
{
	int i = 0, err = 1, retry_count = 0;

	if (!gpio_tbl || !size) {
		pr_err("%s:%d invalid gpio_tbl %pK / size %d\n", __func__,
			__LINE__, gpio_tbl, size);
	}
	for (i = 0; i < size; i++) {
		vl53l0_dbgmsg("%d i %d, gpio %d dir %ld\n", __LINE__, i,
			gpio_tbl[i].gpio, gpio_tbl[i].flags);
	}
	if (gpio_en) {
		while (err && retry_count < 10) {
			for (i = 0; i < size; i++) {
				err = gpio_request_one(gpio_tbl[i].gpio,
					gpio_tbl[i].flags, gpio_tbl[i].label);
				if (err) {
					/*
					* After GPIO request fails, contine to
					* apply new gpios, outout a error message
					* for driver bringup debug
					*/
					pr_err("%s:%d gpio %d:%s request fails, retry_count %d\n",
						__func__, __LINE__,
						gpio_tbl[i].gpio, gpio_tbl[i].label, retry_count);
					msleep(30);
					retry_count++;
					break;
				}
			}
		}
	} else {
		gpio_free_array(gpio_tbl, size);
	}
}

static int stmvl53l0_get_dt_data(struct device *dev, struct i2c_data *data)
{
	int rc = 0;
	struct msm_camera_gpio_conf *gconf;
	uint16_t *gpio_array;
	uint16_t gpio_array_size;
	int i;
	struct msm_pinctrl_info *sensor_pctrl;

	vl53l0_dbgmsg("Enter\n");

	sensor_pctrl = &data->pinctrl_info;
	sensor_pctrl->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(sensor_pctrl->pinctrl)) {
		pr_err("%s:%d Getting pinctrl handle failed\n",
			__func__, __LINE__);
		return -EINVAL;
	}
	sensor_pctrl->gpio_state_active =
		pinctrl_lookup_state(sensor_pctrl->pinctrl,
		LASER_SENSOR_PINCTRL_STATE_DEFAULT);

	sensor_pctrl->gpio_state_suspend
		= pinctrl_lookup_state(sensor_pctrl->pinctrl,
		LASER_SENSOR_PINCTRL_STATE_SLEEP);

	if (dev->of_node) {
		struct device_node *of_node = dev->of_node;

		gpio_array_size = of_gpio_count(of_node);
		gconf = &data->gconf;

		if (gpio_array_size) {
			gpio_array = kcalloc(gpio_array_size, sizeof(uint16_t),
				GFP_KERNEL);
			if (!gpio_array)
				return -ENOMEM;

			for (i = 0; i < gpio_array_size; i++) {
				gpio_array[i] = of_get_gpio(of_node, i);
				pr_err("%s gpio_array[%d] = %d\n", __func__, i,
					gpio_array[i]);
			}

			rc = stmvl53l0_get_dt_gpio_req_tbl(of_node, gconf,
				gpio_array, gpio_array_size);
			if (rc < 0) {
				pr_err("%s failed %d\n", __func__, __LINE__);
				kfree(gpio_array);
				return rc;
			}

			rc = stmvl53l0_init_gpio_pin_tbl(of_node, gconf,
				gpio_array, gpio_array_size);
			if (rc < 0) {
				pr_err("%s failed %d\n", __func__, __LINE__);
				kfree(gpio_array);
				return rc;
			}
			kfree(gpio_array);
		}
		rc = get_dt_xtalk_data(of_node,
			&(data->xtalk));
	}
	vl53l0_dbgmsg("End rc =%d\n", rc);

	return rc;
}
static int stmvl53l0_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	int rc = 0, i = 0;
	struct stmvl53l0_data *vl53l0_data = NULL;
	struct i2c_data *i2c_object = NULL;
	int present;

	vl53l0_errmsg("Enter\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE)) {
		rc = -EIO;
		return rc;
	}

	vl53l0_data = kzalloc(sizeof(struct stmvl53l0_data), GFP_KERNEL);
	if (!vl53l0_data) {
		rc = -ENOMEM;
		return rc;
	}
	if (vl53l0_data) {
		vl53l0_data->client_object =
		    kzalloc(sizeof(struct i2c_data), GFP_KERNEL);
		i2c_object = (struct i2c_data *)vl53l0_data->client_object;
	}
	i2c_object->client = client;

	/* setup bus type */
	vl53l0_data->bus_type = I2C_BUS;

	/* setup regulator */
	stmvl53l0_parse_vdd(&i2c_object->client->dev, i2c_object);

	/* setup device name */
	vl53l0_data->dev_name = dev_name(&client->dev);

	/* setup device data */
	dev_set_drvdata(&client->dev, vl53l0_data);

	/* setup client data */
	i2c_set_clientdata(client, vl53l0_data);

	stmvl53l0_get_dt_data(&client->dev, i2c_object);

	/* fill xtalk data */
	vl53l0_data->xtalk= i2c_object->xtalk;

	rc = stmvl53l0_power_up_i2c(i2c_object, &present);
	if (rc) {
		vl53l0_errmsg("%d,error rc %d\n", __LINE__, rc);
		return rc;
	}
	for (i = 0; i < 5; i++) {
		rc = stmvl53l0_checkmoduleid
			(vl53l0_data, i2c_object->client, I2C_BUS);
		if (!rc)
			break;
		msleep(20);
	}
	if (rc != 0) {
		stmvl53l0_power_down_i2c(i2c_object);
		msleep(20);
		stmvl53l0_power_up_i2c(i2c_object, &present);
		for (i = 0; i < 10; i++) {
			rc = stmvl53l0_checkmoduleid
				(vl53l0_data, i2c_object->client, I2C_BUS);
			if (!rc)
				break;
			msleep(20);
		}
	}
	if (rc != 0) {
		vl53l0_errmsg("%d,error rc %d\n", __LINE__, rc);
		stmvl53l0_power_down_i2c(i2c_object);
	}

	stmvl53l0_power_down_i2c(i2c_object);

	/* setup other stuff */
	rc = stmvl53l0_setup(vl53l0_data);

	/* init default value */
	i2c_object->power_up = 0;

	vl53l0_errmsg("End\n");
	return rc;
}

static int stmvl53l0_remove(struct i2c_client *client)
{
	struct stmvl53l0_data *data = i2c_get_clientdata(client);

	vl53l0_dbgmsg("Enter\n");

	/* Power down the device */
	stmvl53l0_power_down_i2c(data->client_object);
	stmvl53l0_cleanup(data);
	kfree(data->client_object);
	kfree(data);
	vl53l0_dbgmsg("End\n");
	return 0;
}

static const struct i2c_device_id stmvl53l0_id[] = {
	{STMVL53L0_DRV_NAME, 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, stmvl53l0_id);

static const struct of_device_id st_stmvl53l0_dt_match[] = {
	{ .compatible = "st,stmvl53l0_i2c", },
	{ },
};

static struct i2c_driver stmvl53l0_driver = {
	.driver = {
		.name = STMVL53L0_DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = st_stmvl53l0_dt_match,
	},
	.probe = stmvl53l0_probe,
	.remove = stmvl53l0_remove,
	.id_table = stmvl53l0_id,

};

int stmvl53l0_power_up_i2c(void *i2c_object, unsigned int *preset_flag)
{
	int ret = 0;

	struct i2c_data *data = (struct i2c_data *)i2c_object;

	vl53l0_dbgmsg("Enter i2c powerup\n");

	if (!IS_ERR(data->vana)) {
		ret = regulator_set_voltage(data->vana, VL53L0_VDD_MIN,
				VL53L0_VDD_MAX);
		if (ret < 0) {
			vl53l0_errmsg("set_vol(%p) fail %d\n", data->vana, ret);
			return ret;
		}
		ret = regulator_enable(data->vana);

		if (ret < 0) {
			vl53l0_errmsg("reg enable(%p) failed.rc=%d\n",
					data->vana, ret);
			return ret;
		}
	}

	stmvl53l0_request_gpio_table(
		data->gconf.cam_gpio_req_tbl,
		data->gconf.cam_gpio_req_tbl_size, 1);

	pinctrl_select_state(data->pinctrl_info.pinctrl,
		data->pinctrl_info.gpio_state_active);

	gpio_set_value_cansleep(data->gconf.cam_gpio_req_tbl[1].gpio, 1);
	vl53l0_dbgmsg("Enable gpio%d\n",data->gconf.cam_gpio_req_tbl[1].gpio );
	msleep(20);
	gpio_set_value_cansleep(data->gconf.cam_gpio_req_tbl[0].gpio, 1);
	vl53l0_dbgmsg("Enable gpio%d\n",data->gconf.cam_gpio_req_tbl[0].gpio );

	stmvl53l0_request_gpio_table(
		data->gconf.cam_gpio_req_tbl,
		data->gconf.cam_gpio_req_tbl_size, 0);

	data->power_up = 1;
	*preset_flag = 1;

	vl53l0_dbgmsg("End\n");
	return ret;
}

int stmvl53l0_power_down_i2c(void *i2c_object)
{
	int ret = 0;

	struct i2c_data *data = (struct i2c_data *)i2c_object;

	vl53l0_dbgmsg("Enter\n");
	if (data->power_up) {
		stmvl53l0_request_gpio_table(
			data->gconf.cam_gpio_req_tbl,
			data->gconf.cam_gpio_req_tbl_size, 1);

		pinctrl_select_state(data->pinctrl_info.pinctrl,
			data->pinctrl_info.gpio_state_suspend);

		gpio_set_value_cansleep(
			data->gconf.cam_gpio_req_tbl[0].gpio, 0);
		vl53l0_dbgmsg("Disable gpio%d\n",data->gconf.cam_gpio_req_tbl[0].gpio );

		/* HACK, no disable TOF_VANA as GPIO 70 is common for Prox and TOF*/
		//gpio_set_value_cansleep(
		//	data->gconf.cam_gpio_req_tbl[1].gpio, 0);
		//vl53l0_dbgmsg("Disable gpio%d\n",data->gconf.cam_gpio_req_tbl[1].gpio );

		stmvl53l0_request_gpio_table(
			data->gconf.cam_gpio_req_tbl,
			data->gconf.cam_gpio_req_tbl_size, 0);

		if (!IS_ERR(data->vana)) {
			ret = regulator_disable(data->vana);
			if (ret < 0)
				vl53l0_errmsg("reg disable(%p) failed.rc=%d\n",
				data->vana, ret);
		}

		data->power_up = 0;
	}

	vl53l0_dbgmsg("End\n");
	return ret;
}

int stmvl53l0_init_i2c(void)
{
	int ret = 0;

	vl53l0_dbgmsg("Enter\n");

	/* register as a i2c client device */
	ret = i2c_add_driver(&stmvl53l0_driver);
	if (ret)
		vl53l0_errmsg("%d erro ret:%d\n", __LINE__, ret);

	vl53l0_errmsg("End with rc:%d\n", ret);

	return ret;
}

void stmvl53l0_exit_i2c(void *i2c_object)
{
	vl53l0_dbgmsg("Enter\n");
	i2c_del_driver(&stmvl53l0_driver);

	vl53l0_dbgmsg("End\n");
}
