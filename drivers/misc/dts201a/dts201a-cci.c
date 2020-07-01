/*
 *  dts201a_module-cci.c - Linux kernel modules for Partron
 *							sensor
 *
 *  Copyright (C) 2020 Motorola mobility
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
#ifdef CAMERA_CCI
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
#include "dts201a-cci.h"

#define SENSOR_NAME       "dts201a"
#define DRV_NAME      "dts201a_cci"


#define 			OTHER_DEVICE_TYPE         0x000200ff
struct cam_subdev  *g_v4l2_dev_str = NULL;



static int dts201a_request_pwren(struct cci_ctrl_t *cci_ctrl)
{
	int rc = 0;

	if (cci_ctrl->pwren_gpio == -1) {
		dts201a_wanrmsg("pwren gpio disable");
		goto no_gpio;
	}

	dts201a_dbgmsg("request pwren_gpio %d", cci_ctrl->pwren_gpio);
	rc = gpio_request(cci_ctrl->pwren_gpio, "dts201a_pwren");
	if (rc) {
		dts201a_errmsg("fail to acquire pwren %d", rc);
		goto no_gpio;
	}

	rc = gpio_direction_output(cci_ctrl->pwren_gpio, 0);
	if (rc) {
		dts201a_errmsg("fail to configure pwren as output %d", rc);
	}

	return rc;

no_gpio:
	rc =-1;
	return rc;
}

static void dts201a_release_pwren(struct cci_ctrl_t *cci_ctrl)
{
	if (cci_ctrl->pwren_gpio > 0) {
		dts201a_dbgmsg("release pwren_gpio %d", cci_ctrl->pwren_gpio);
		gpio_free(cci_ctrl->pwren_gpio);
	}
	cci_ctrl->pwren_gpio = -1;
}

static void dts201a_release_gpios_cci(struct cci_ctrl_t *t_ctrl)
{
	if (t_ctrl->power_supply) {
		regulator_put(t_ctrl->power_supply);
		t_ctrl->power_supply = NULL;
	}
	if (t_ctrl->cci_supply) {
		regulator_put(t_ctrl->cci_supply);
		t_ctrl->cci_supply = NULL;
	}
	dts201a_release_pwren(t_ctrl);
}

static int dts201a_get_dt_info(struct device *dev, struct cci_ctrl_t *t_ctrl)
{
	int rc = 0;
	struct device_node   *of_node  = NULL;

	if (!dev || !t_ctrl)
		return -EINVAL;

	of_node  = dev->of_node;
	if (!of_node) {
		dts201a_errmsg("of_node is NULL %d\n", __LINE__);
		return -EINVAL;
	}

	t_ctrl->pwren_gpio = -1;
	rc = of_property_read_u32(of_node, "cell-index", &t_ctrl->pdev->id);
	if (rc < 0) {
		dts201a_errmsg("failed to read cell index %d\n", __LINE__);
		return rc;
	}

	rc = of_property_read_u32(of_node, "cci-master", &t_ctrl->cci_master);
	if (rc < 0) {
		dts201a_errmsg("failed to get the cci master %d\n", __LINE__);
		return rc;
	}
	rc = of_property_read_u32(of_node, "cci-device", &t_ctrl->cci_num);
	if (rc < 0) {
		/* Set default master 0 */
		t_ctrl->cci_num = CCI_DEVICE_0;
		rc = 0;
	}
	t_ctrl->io_master_info.cci_client->cci_device = t_ctrl->cci_num;

	t_ctrl->power_supply = regulator_get(dev, "laser");
	if (IS_ERR(t_ctrl->power_supply) || t_ctrl->power_supply == NULL) {
		t_ctrl->power_supply = NULL;

		dts201a_wanrmsg("no regulator, laser power_supply");
	}
	t_ctrl->cci_supply = regulator_get(dev, "cci");
	if (IS_ERR(t_ctrl->cci_supply)) {
		t_ctrl->cci_supply = NULL;
		/* try gpio */
		rc = of_property_read_u32_array(dev->of_node, "pwren-gpio", &t_ctrl->pwren_gpio, 1);
		if (rc) {
			t_ctrl->pwren_gpio = -1;
			dts201a_wanrmsg("no regulator, nor power gpio => power ctrl disabled");
		}
		if (gpio_is_valid(t_ctrl->pwren_gpio)) {
			rc = devm_gpio_request(dev, gf_dev->pwr_gpio, "fir_pwr");
		if (rc) {
			dts201a_wanrmsg("failed to request pwr gpio, rc = %d\n", rc);
		}
		gpio_direction_output(gf_dev->pwr_gpio, 1);
		dts201a_wanrmsg("Unable to get cci power supply");
	}
	if (t_ctrl->pwren_gpio != -1)
		rc = dts201a_request_pwren(t_ctrl);
	return rc;


}
static int msm_tof_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	int rc = 0;
	return rc;
}

static const struct v4l2_subdev_internal_ops msm_tof_internal_ops = {
	.close = msm_tof_close,
};

static long msm_tof_subdev_ioctl(struct v4l2_subdev *sd,
				 unsigned int cmd, void *arg)
{
	int32_t rc = 0;
	return rc;
}

static int32_t msm_tof_power(struct v4l2_subdev *sd, int on)
{
	dts201a_dbgmsg("TOF power called\n");
	return 0;
}

static struct v4l2_subdev_core_ops msm_tof_subdev_core_ops = {
	.ioctl = msm_tof_subdev_ioctl,
	.s_power = msm_tof_power,
};

static struct v4l2_subdev_ops msm_tof_subdev_ops = {
	.core = &msm_tof_subdev_core_ops,
};

static int32_t dts201a_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	struct dts201a_data *dts201a_data  = NULL;
	struct cci_ctrl_t *cci_ctrl         = NULL;
	struct cam_sensor_cci_client *cci_client = NULL;

	dts201a_errmsg("Enter\n");

	if (!cam_cci_get_subdev(0)){
		dts201a_errmsg("Get cam_cci_get_subdev error\n");
		return -EPROBE_DEFER;
	}
	dts201a_info("Get cam_cci_get_subdev  success\n");
	dts201a_data = kzalloc(sizeof(struct dts201a_data), GFP_KERNEL);
	if (!dts201a_data) {
		rc = -ENOMEM;
		dts201a_errmsg("memory failed\n");
		return rc;
	}

	if (dts201a_data) {
		dts201a_data->client_object = kzalloc(sizeof(struct cci_ctrl_t), GFP_KERNEL);
		if (!dts201a_data->client_object) {
			rc = -ENOMEM;
			dts201a_errmsg("alloc client_object  failed\n");
			goto free_dts201a_data;
		}
		cci_ctrl = (struct cci_ctrl_t *)dts201a_data->client_object;
	}
	mutex_init(&dts201a_data->lock);
	mutex_lock(&dts201a_data->lock);
	cci_ctrl->pdev = pdev;

	cci_ctrl->dts201a_data = dts201a_data;
	cci_ctrl->device_type = MSM_CAMERA_PLATFORM_DEVICE;
	cci_ctrl->cam_pinctrl_status = 0;
	cci_ctrl->io_master_info.master_type = CCI_MASTER;
	cci_ctrl->io_master_info.cci_client = kzalloc(
		sizeof(struct cam_sensor_cci_client), GFP_KERNEL);
	if (!cci_ctrl->io_master_info.cci_client) {
		dts201a_errmsg("alloc cci_client failed\n");
		goto free_tof_ctrl;
	}

	rc = dts201a_get_dt_info(&pdev->dev, cci_ctrl);
	if (rc < 0) {
		dts201a_errmsg("%d, failed to get dt info rc %d\n", __LINE__, rc);
		goto free_cci_client;
	}


	cci_client = cci_ctrl->io_master_info.cci_client;
	cci_client->cci_i2c_master = cci_ctrl->cci_master;
	//i2c slave address
	cci_client->sid = 0x3A;
	cci_client->retries = 3;
	cci_client->id_map = 0;
	//i2c speed 400k fast, 100k standard I2C_STANDARD_MODE
	cci_client->i2c_freq_mode = I2C_FAST_MODE;

	cci_ctrl->v4l2_dev_str.internal_ops = &msm_tof_internal_ops;
	cci_ctrl->v4l2_dev_str.ops = &msm_tof_subdev_ops;
	strlcpy(cci_ctrl->device_name, SENSOR_NAME,
		sizeof(cci_ctrl->device_name));
	cci_ctrl->v4l2_dev_str.name = cci_ctrl->device_name;
	cci_ctrl->v4l2_dev_str.sd_flags =		(V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS);
	cci_ctrl->v4l2_dev_str.sd_flags =		(V4L2_SUBDEV_FL_HAS_EVENTS);
	cci_ctrl->v4l2_dev_str.ent_function = OTHER_DEVICE_TYPE;
	cci_ctrl->v4l2_dev_str.token = cci_ctrl;

	rc = cam_register_subdev(&(cci_ctrl->v4l2_dev_str));
	if (rc) {
		dts201a_errmsg("fail to create subdev");
		goto unregister_subdev;
	}
	dts201a_info("cam_register_subdev done \n");
	g_v4l2_dev_str = &cci_ctrl->v4l2_dev_str;

	/* setup device data */
	dev_set_drvdata(&pdev->dev, dts201a_data);

	dts201a_power_up_cci((void *)cci_ctrl);
	msleep(10);
	/* setup other stuff */
	rc = dts201a_setup(&pdev->dev, dts201a_data);
	if (rc) {
		dts201a_errmsg("fail to dts201a_palfrform_probe");
		goto release_gpios;
	}
	kref_init(&cci_ctrl->ref);

	dts201a_info("End = %d\n", rc);
	mutex_unlock(&dts201a_data->lock);
	return rc;

release_gpios:
	camera_io_release(&cci_ctrl->io_master_info);
	dts201a_release_gpios_cci(cci_ctrl);
unregister_subdev:
	cam_unregister_subdev(&(cci_ctrl->v4l2_dev_str));
free_cci_client:
	kfree(cci_ctrl->io_master_info.cci_client);
free_tof_ctrl:
	kfree(cci_ctrl);
free_dts201a_data:
	mutex_unlock(&dts201a_data->lock);
	kfree(dts201a_data);

	return rc;
}

static int32_t dts201a_platform_remove(struct platform_device *pdev)
{
	int ret = 0;
	struct dts201a_data *dts201a_data = platform_get_drvdata(pdev);
	struct cci_ctrl_t *cci_ctrl = (struct cci_ctrl_t *)dts201a_data->client_object;
	dts201a_cleanup(dts201a_data);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static const struct of_device_id dts201a_cci_dt_match[] = {
	{.compatible = "partron,dts201a",},
	{},
};


#ifdef CONFIG_PM
/**
 * dts201a_pm_suspend - PM suspend function
 * Called by kernel during system suspend phrase
 */
static int dts201a_pm_suspend(struct device *dev)
{
	return 0;
}
/**
 * dts201a_pm_resume - PM resume function
 * Called by kernel during system wakeup
 */
static int dts201a_pm_resume(struct device *dev)
{
	return 0;
}

#endif

#ifdef CONFIG_PM
static const struct dev_pm_ops dev_pm_ops = {
	.suspend = dts201a_pm_suspend,
	.resume = dts201a_pm_resume,
};
#endif

static struct platform_driver dts201a_platform_driver = {
	.probe = dts201a_platform_probe,
	.remove = dts201a_platform_remove,
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = dts201a_cci_dt_match,
#ifdef CONFIG_PM
		.pm = &dev_pm_ops,
#endif

	},
};

int dts201a_power_up_cci(void *object)
{
	int rc = 0;
	struct cci_ctrl_t *cci_ctrl = (struct cci_ctrl_t *)object;

	dts201a_dbgmsg("Enter");

	if (!cci_ctrl) {
		dts201a_errmsg("dts201a_power_up_cci failed %d\n", __LINE__);
		return -EINVAL;
	}
	/* turn on power */
	if (cci_ctrl->power_supply) {
		rc = cam_soc_util_regulator_enable(cci_ctrl->power_supply, "laser", 2800000, 2800000, 80000, 0);
		if (rc) {
			dts201a_errmsg("fail to turn on avdd regulator");
			return rc;
		}
		dts201a_dbgmsg("enable laser 28\n");
	} else
		dts201a_wanrmsg("no power control");
	if (cci_ctrl->cci_supply) {
		rc = cam_soc_util_regulator_enable(cci_ctrl->cci_supply,"cci", 1800000, 1800000, 0, 0);
		if (rc) {
			dts201a_errmsg("fail to turn on iovdd regulator");
			return rc;
		}
		dts201a_dbgmsg("enable laser 18\n");
	} else {
		if (cci_ctrl->pwren_gpio != -1) {
			gpio_set_value_cansleep(cci_ctrl->pwren_gpio, 1);
		}
	}
	msleep(10);
	rc = camera_io_init(&cci_ctrl->io_master_info);
	if (rc < 0)
		dts201a_errmsg("cci init failed: rc: %d", rc);

	dts201a_dbgmsg("End\n");

	return rc;
}

int dts201a_power_down_cci(void *cci_object)
{
	int rc = 0;
	struct cci_ctrl_t *cci_ctrl = (struct cci_ctrl_t *)cci_object;

	if (!cci_ctrl) {
		dts201a_errmsg("dts201a_power_down_cci failed %d\n", __LINE__);
		return -EINVAL;
	}

	dts201a_dbgmsg("Enter\n");
	/* turn off power */
	if (cci_ctrl->power_supply) {
		rc = cam_soc_util_regulator_disable(cci_ctrl->power_supply, "laser", 2800000, 2800000, 80000, 0);
		rc = cam_soc_util_regulator_disable(cci_ctrl->cci_supply,"cci", 1800000, 1800000, 0, 0);
		if (rc)
			dts201a_errmsg("reg disable failed. rc=%d\n",
				rc);
	} else if (cci_ctrl->pwren_gpio != -1) {
		gpio_set_value_cansleep(cci_ctrl->pwren_gpio, 0);
	}
	camera_io_release(&cci_ctrl->io_master_info);

	dts201a_dbgmsg("power off");

	return rc;
}

void dts201a_clean_up_cci(void)
{
	int rc = 0;
	rc = cam_unregister_subdev(g_v4l2_dev_str);
}

int dts201a_init_cci(void)
{
	int ret = 0;

	dts201a_errmsg("Enter\n");

	/* register as a platform device */
	ret = platform_driver_register(&dts201a_platform_driver);
	if (ret)
		dts201a_errmsg("%d, error ret:%d\n", __LINE__, ret);

	dts201a_dbgmsg("End\n");

	return ret;
}

void dts201a_exit_cci(void *object)
{
	struct cci_ctrl_t *cci_ctrl = (struct cci_ctrl_t *)object;

	dts201a_dbgmsg("Enter\n");
	platform_driver_unregister(&dts201a_platform_driver);
	dts201a_dbgmsg("End\n");
}
#endif				/* end of CAMERA_CCI */
