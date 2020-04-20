/*
 * Copyright (C) 2019 Motorola Mobility LLC
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/gpio.h>
#include <linux/touchscreen_mmi.h>
#include <linux/regulator/consumer.h>
#include "synaptics_core.h"

#define GET_SYNA_DATA(dev) { \
	pdev = dev_get_drvdata(dev); \
	if (!pdev) { \
		LOGE(dev, "Failed to get platform device");\
		return -ENODEV; \
	} \
	tcm_hcd = platform_get_drvdata(pdev); \
	if (!tcm_hcd) { \
		LOGE(dev, "Failed to get driver data"); \
		return -ENODEV; \
	} \
}

static ssize_t syna_ts_pill_region_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t syna_ts_pill_region_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t syna_ts_gs_distance_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t syna_ts_gs_distance_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static DEVICE_ATTR(pill_region, (S_IRUGO | S_IWUSR | S_IWGRP),
		syna_ts_pill_region_show, syna_ts_pill_region_store);
static DEVICE_ATTR(gs_distance, (S_IRUGO | S_IWUSR | S_IWGRP),
		syna_ts_gs_distance_show, syna_ts_gs_distance_store);

#define MAX_ATTRS_ENTRIES 10
#define ADD_ATTR(name) { \
	if (idx < MAX_ATTRS_ENTRIES)  { \
		dev_info(dev, "%s: [%d] adding %p\n", __func__, idx, &dev_attr_##name.attr); \
		ext_attributes[idx] = &dev_attr_##name.attr; \
		idx++; \
	} else { \
		dev_err(dev, "%s: cannot add attribute '%s'\n", __func__, #name); \
	} \
}

static struct attribute *ext_attributes[MAX_ATTRS_ENTRIES];
static struct attribute_group ext_attr_group = {
	.attrs = ext_attributes,
};

static int syna_ts_extend_attribute_group(struct device *dev, struct attribute_group **group)
{
	int idx = 0;
	const struct syna_tcm_hw_interface *hw_if;
	const struct syna_tcm_board_data *bdata;
	struct syna_tcm_hcd *tcm_hcd;
	struct platform_device *pdev;

	LOGN(dev, "syna_ts_extend_attribute_group\n");
	GET_SYNA_DATA(dev);
	hw_if = tcm_hcd->hw_if;
	if (!hw_if) {
		LOGE(dev, "Hardware interface not found\n");
		return -ENODEV;
	}
	bdata = hw_if->bdata;

	if (bdata->pill_region_ctrl)
		ADD_ATTR(pill_region);

	if (bdata->gs_distance_ctrl)
		ADD_ATTR(gs_distance);

	if (idx) {
		ext_attributes[idx] = NULL;
		*group = &ext_attr_group;
	} else
		*group = NULL;

	return 0;
}

static ssize_t syna_ts_gs_distance_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int retval;
	unsigned long value;
	unsigned short buffer;
	struct syna_tcm_hcd *tcm_hcd;
	struct platform_device *pdev;

	dev = MMI_DEV_TO_TS_DEV(dev);
	GET_SYNA_DATA(dev);

	retval = kstrtoul(buf, 0, &value);
	if (retval)
		return -EINVAL;

	mutex_lock(&tcm_hcd->extif_mutex);
	buffer = (unsigned short)value;
	dev_info(dev, "%s: program value 0x%02x\n", __func__, (unsigned int)value);

	if (tcm_hcd->gs_distance_data == buffer) {
		dev_dbg(dev, "%s: value is same,so not write.\n", __func__);
		retval = size;
		goto exit;
	}

	tcm_hcd->gs_distance_data = buffer;
	retval = tcm_hcd->set_dynamic_config(tcm_hcd,
		DC_SUPP_X_WIDTH,
		buffer);

	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
			"Failed to enable wakeup gesture mode\n");
		goto exit;
	}
	retval = size;

exit:
	mutex_unlock(&tcm_hcd->extif_mutex);
	return retval;

}

static ssize_t syna_ts_gs_distance_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int retval;
	unsigned short value;
	ssize_t blen = 0;
	struct syna_tcm_hcd *tcm_hcd;
	struct platform_device *pdev;

	dev = MMI_DEV_TO_TS_DEV(dev);
	GET_SYNA_DATA(dev);

	mutex_lock(&tcm_hcd->extif_mutex);
	retval = tcm_hcd->get_dynamic_config(tcm_hcd, DC_SUPP_X_WIDTH, &value);
	if (retval < 0)
		LOGE(tcm_hcd->pdev->dev.parent,
			"Failed to read gs_distance (%d)\n",retval );
	else
		blen += scnprintf(buf, PAGE_SIZE, "0x%02x", (unsigned int)value);

	mutex_unlock(&tcm_hcd->extif_mutex);
	return blen;
}

static int syna_ts_get_pill_region(struct syna_tcm_hcd *tcm_hcd)
{
	int retval;
	struct pill_region_data *region_data;
	region_data = &tcm_hcd->region_data;

	retval = tcm_hcd->get_dynamic_config(tcm_hcd,
			DC_SUPP_Y_START_L, &region_data->y_start_l);
	if (retval < 0)
		LOGD(tcm_hcd->pdev->dev.parent,
			"Failed to read LEFT_Y_START (%d)\n", retval);

	retval = tcm_hcd->get_dynamic_config(tcm_hcd,
			DC_SUPP_Y_END_L, &region_data->y_end_l);
	if (retval < 0)
		LOGD(tcm_hcd->pdev->dev.parent,
			"Failed to read LEFT_Y_END (%d)\n", retval);

	retval = tcm_hcd->get_dynamic_config(tcm_hcd,
			DC_SUPP_Y_START_R, &region_data->y_start_r);
	if (retval < 0)
		LOGD(tcm_hcd->pdev->dev.parent,
			"Failed to read RIGHT_Y_START (%d)\n", retval);

	retval = tcm_hcd->get_dynamic_config(tcm_hcd,
			DC_SUPP_Y_END_R, &region_data->y_end_r);
	if (retval < 0)
		LOGD(tcm_hcd->pdev->dev.parent,
			"Failed to read RIGHT_Y_END (%d)\n", retval);

	return retval;
}

static int syna_ts_set_pill_region(struct syna_tcm_hcd *tcm_hcd)
{
	int retval;
	struct pill_region_data *region_data;
	region_data = &tcm_hcd->region_data;

	retval = tcm_hcd->set_dynamic_config(tcm_hcd,
		DC_SUPP_Y_START_L, region_data->y_start_l);
	if (retval < 0)
		LOGD(tcm_hcd->pdev->dev.parent,
			"Failed to set LEFT_Y_START of pill_region\n");

	retval = tcm_hcd->set_dynamic_config(tcm_hcd,
		DC_SUPP_Y_END_L, region_data->y_end_l);
	if (retval < 0)
		LOGD(tcm_hcd->pdev->dev.parent,
			"Failed to set LEFT_Y_END of pill_region\n");

	retval = tcm_hcd->set_dynamic_config(tcm_hcd,
		DC_SUPP_Y_START_R, region_data->y_start_r);
	if (retval < 0)
		LOGD(tcm_hcd->pdev->dev.parent,
			"Failed to set RIGHT_Y_START of pill_region\n");

	retval = tcm_hcd->set_dynamic_config(tcm_hcd,
		DC_SUPP_Y_END_R, region_data->y_end_r);
	if (retval < 0)
		LOGD(tcm_hcd->pdev->dev.parent,
			"Failed to set RIGHT_Y_END of pill_region\n");

	return retval;
}

#define REQ_ARGS_NUM 3
static ssize_t syna_ts_pill_region_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int retval;
	unsigned short args[3] = {0};
	struct syna_tcm_hcd *tcm_hcd;
	struct platform_device *pdev;
	struct pill_region_data *region_data;

	dev = MMI_DEV_TO_TS_DEV(dev);
	GET_SYNA_DATA(dev);

	mutex_lock(&tcm_hcd->extif_mutex);
	region_data = &tcm_hcd->region_data;

	retval = sscanf(buf, "0x%hx 0x%hx 0x%hx", &args[0], &args[1], &args[2]);
	if (retval < REQ_ARGS_NUM) {
		retval = -EINVAL;
		goto exit;
	}

	dev_info(dev, "%s: program pill region:0x%02hx 0x%04hx 0x%04hx\n",
		__func__, args[0], args[1], args[2]);

	region_data->region_side = args[0];
	switch (region_data->region_side) {
		case 0:
			if (!region_data->y_start_l && !region_data->y_end_l
					&& !region_data->y_start_r && !region_data->y_end_r) {
				dev_info(dev, "%s: The value of region side is same, so not write.\n",
					__func__);
				retval = size;
				goto exit;
			}
			region_data->y_start_l = 0x00;
			region_data->y_end_l = 0x00;
			region_data->y_start_r = 0x00;
			region_data->y_end_r = 0x00;
			break;

		case 1:
			if (region_data->y_start_l ==  args[1] && region_data->y_end_l == args[2]) {
				dev_info(dev, "%s: The values of left region are same, so not write.\n", __func__);
				retval = size;
				goto exit;
			}
			region_data->y_start_l = args[1];
			region_data->y_end_l = args[2];
			region_data->y_start_r = 0x00;
			region_data->y_end_r = 0x00;
			break;

		case 2:
			if (region_data->y_start_r ==  args[1] && region_data->y_end_r == args[2]) {
				dev_info(dev, "%s: The values of right region are same, so not write.\n", __func__);
				retval = size;
				goto exit;
			}
			region_data->y_start_l = 0x00;
			region_data->y_end_l = 0x00;
			region_data->y_start_r = args[1];
			region_data->y_end_r = args[2];
			break;

		default:
			LOGE(dev, "The commond is not support!\n");
			retval = -EINVAL;
			goto exit;
	}

	retval = syna_ts_set_pill_region(tcm_hcd);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
			"Failed to set pill_region none\n");
		goto exit;
	}
	retval = size;

exit:
	mutex_unlock(&tcm_hcd->extif_mutex);
	return retval;
}

static ssize_t syna_ts_pill_region_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int retval;
	ssize_t blen = 0;
	struct syna_tcm_hcd *tcm_hcd;
	struct platform_device *pdev;
	struct pill_region_data *region_data;

	dev = MMI_DEV_TO_TS_DEV(dev);
	GET_SYNA_DATA(dev);
	dev_info(dev, "%s: program pill region show",__func__);

	mutex_lock(&tcm_hcd->extif_mutex);
	region_data = &tcm_hcd->region_data;

	retval = syna_ts_get_pill_region(tcm_hcd);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
			"Failed to get pill_region value (%d)\n", retval);
		goto exit;
	}

	switch (region_data->region_side) {
		case 0:
			blen += scnprintf(buf+blen, PAGE_SIZE, "0x%02x ", 0);
			blen += scnprintf(buf+blen, PAGE_SIZE, "0x%hx ", region_data->y_start_r);
			blen += scnprintf(buf+blen, PAGE_SIZE, "0x%hx ", region_data->y_end_r);
			break;

		case 1:
			blen += scnprintf(buf+blen, PAGE_SIZE, "0x%02x ", 1);
			blen += scnprintf(buf+blen, PAGE_SIZE, "0x%hx ", region_data->y_start_l);
			blen += scnprintf(buf+blen, PAGE_SIZE, "0x%hx ", region_data->y_end_l);
			break;

		case 2:
			blen += scnprintf(buf+blen, PAGE_SIZE, "0x%02x ", 2);
			blen += scnprintf(buf+blen, PAGE_SIZE, "0x%hx ", region_data->y_start_r);
			blen += scnprintf(buf+blen, PAGE_SIZE, "0x%hx ", region_data->y_end_r);
			break;

		default:
			LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to read the pill_region (%d)\n", retval);
	}
	retval = blen;

exit:
	mutex_unlock(&tcm_hcd->extif_mutex);
	return retval;
}

static int syna_ts_mmi_methods_get_vendor(struct device *dev, void *cdata)
{
	return scnprintf(TO_CHARP(cdata), TS_MMI_MAX_VENDOR_LEN, "%s", "synaptics");
}

static int syna_ts_mmi_methods_get_productinfo(struct device *dev, void *cdata)
{
	struct syna_tcm_hcd *tcm_hcd;
	struct platform_device *pdev;

	GET_SYNA_DATA(dev);
	return scnprintf(TO_CHARP(cdata), 6, "%s", tcm_hcd->id_info.part_number);
}

static int syna_ts_mmi_methods_get_build_id(struct device *dev, void *cdata)
{
	unsigned int buildid;
	struct syna_tcm_hcd *tcm_hcd;
	struct platform_device *pdev;

	GET_SYNA_DATA(dev);
	buildid = tcm_hcd->packrat_number;
	return scnprintf(TO_CHARP(cdata), TS_MMI_MAX_ID_LEN, "%06x", buildid);
}

/*return firmware version*/
static int syna_ts_mmi_methods_get_config_id(struct device *dev, void *cdata)
{
	struct syna_tcm_hcd *tcm_hcd;
	struct platform_device *pdev;

	GET_SYNA_DATA(dev);
	return scnprintf(TO_CHARP(cdata), TS_MMI_MAX_ID_LEN, "%s", tcm_hcd->app_info.customer_config_id);
}

static int syna_ts_mmi_methods_get_bus_type(struct device *dev, void *idata)
{
	const struct syna_tcm_hw_interface *hw_if;
	struct syna_tcm_hcd *tcm_hcd;
	struct platform_device *pdev;

	GET_SYNA_DATA(dev);
	hw_if = tcm_hcd->hw_if;
	if (!hw_if) {
		LOGE(dev, "Hardware interface not found\n");
		return -ENODEV;
	}
	TO_INT(idata) = hw_if->bus_io->type;
	return 0;
}

static int syna_ts_mmi_methods_get_irq_status(struct device *dev, void *idata)
{
	const struct syna_tcm_hw_interface *hw_if;
	struct syna_tcm_board_data *bdata;
	struct syna_tcm_hcd *tcm_hcd;
	struct platform_device *pdev;

	GET_SYNA_DATA(dev);
	hw_if = tcm_hcd->hw_if;
	if (!hw_if) {
		LOGE(dev, "Hardware interface not found\n");
		return -ENODEV;
	}
	bdata = hw_if->bdata;
	if (!bdata) {
		LOGE(dev, "Failed to get ts board data");
		return -ENODEV;
	}
	TO_INT(idata) = gpio_get_value(bdata->irq_gpio);
	return 0;
}

static int syna_ts_mmi_methods_get_drv_irq(struct device *dev, void *idata)
{
	struct syna_tcm_board_data *bdata;
	struct syna_tcm_hcd *tcm_hcd;
	struct platform_device *pdev;

	GET_SYNA_DATA(dev);
	bdata = tcm_hcd->hw_if->bdata;
	if (!bdata) {
		LOGE(dev, "Failed to get ts board data");
		return -ENODEV;
	}
	TO_INT(idata) = tcm_hcd->irq_enabled ? 1 : 0;
	return 0;
}

static int syna_ts_mmi_methods_get_poweron(struct device *dev, void *idata)
{
	struct syna_tcm_hcd *tcm_hcd;
	struct platform_device *pdev;

	GET_SYNA_DATA(dev);
	TO_INT(idata) = tcm_hcd->in_suspend == 0 ? 1 : 0;
	return 0;
}

static int syna_ts_mmi_methods_get_flashprog(struct device *dev, void *idata)
{
	struct syna_tcm_hcd *tcm_hcd;
	struct platform_device *pdev;

	GET_SYNA_DATA(dev);
	if (IS_NOT_FW_MODE(tcm_hcd->id_info.mode) ||
			tcm_hcd->app_status != APP_STATUS_OK)
		TO_INT(idata) = 1;
	else
		TO_INT(idata) = 0;

	return 0;
}

static int syna_ts_mmi_methods_drv_irq(struct device *dev, int state)
{
	int retval;
	struct syna_tcm_hcd *tcm_hcd;
	struct platform_device *pdev;

	GET_SYNA_DATA(dev);
	mutex_lock(&tcm_hcd->extif_mutex);
	if (state == 0) {
		retval = tcm_hcd->enable_irq(tcm_hcd, false, true);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to disable interrupt\n");
			goto exit;
		}
	} else if (state == 1) {
		retval = tcm_hcd->enable_irq(tcm_hcd, true, NULL);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to enable interrupt\n");
			goto exit;
		}
	} else {
		retval = -EINVAL;
		goto exit;
	}

exit:
	mutex_unlock(&tcm_hcd->extif_mutex);
	return retval;
}

static int syna_ts_mmi_methods_reset(struct device *dev, int type)
{
	bool hw_reset = !!type;
	struct syna_tcm_hcd *tcm_hcd;
	struct platform_device *pdev;

	GET_SYNA_DATA(dev);
	if (tcm_hcd->reset)
		return tcm_hcd->reset_n_reinit(tcm_hcd, hw_reset, true);
	return -EFAULT;
}

static int syna_ts_firmware_update(struct device *dev, char *fwname) {
	struct syna_tcm_hcd *tcm_hcd;
	struct platform_device *pdev;

	GET_SYNA_DATA(dev);
	LOGI(dev, "HW request update fw");
	/* set firmware image name */
	if (tcm_hcd->set_fw_name)
		tcm_hcd->set_fw_name(fwname);
	else {
		LOGE(dev, "fw update handler not ready, can't set fw_name");
		return -EFAULT;
	}
	if (tcm_hcd->do_fw_update) {
		tcm_hcd->do_fw_update();
	} else {
		LOGE(dev, "fw update handler not ready.");
		return -EFAULT;
	}
	return 0;
}

static int syna_ts_mmi_methods_power(struct device *dev, int on)
{
	int retval;
	const struct syna_tcm_board_data *bdata;
	struct syna_tcm_hcd *tcm_hcd;
	struct platform_device *pdev;

	GET_SYNA_DATA(dev);
	bdata = tcm_hcd->hw_if->bdata;
	if (!bdata) {
		LOGE(dev, "Failed to get ts board data");
		return -ENODEV;
	}
	if (!on) {
		retval = 0;
		goto disable_pwr_reg;
	}

	if (tcm_hcd->bus_reg) {
		retval = regulator_enable(tcm_hcd->bus_reg);
		if (retval < 0) {
			LOGE(dev, "Failed to enable bus regulator\n");
			goto exit;
		}
	}

	if (tcm_hcd->pwr_reg) {
		retval = regulator_enable(tcm_hcd->pwr_reg);
		if (retval < 0) {
			LOGE(dev, "Failed to enable power regulator\n");
			goto disable_bus_reg;
		}
		msleep(bdata->power_delay_ms);
	}
	return 0;

disable_pwr_reg:
	if (tcm_hcd->pwr_reg)
		regulator_disable(tcm_hcd->pwr_reg);
disable_bus_reg:
	if (tcm_hcd->bus_reg)
		regulator_disable(tcm_hcd->bus_reg);
exit:
	return retval;
}

static int syna_ts_mmi_post_resume(struct device *dev)
{
	struct platform_device *pdev;
	pdev = dev_get_drvdata(dev);

	if (!pdev) {
		LOGE(dev, "Failed to get platform device");
		return -ENODEV;
	}

	return syna_tcm_resume(&pdev->dev);
}

static int syna_ts_mmi_panel_state(struct device *dev,
	enum ts_mmi_pm_mode from, enum ts_mmi_pm_mode to)
{
	struct syna_tcm_hcd *tcm_hcd;
	struct platform_device *pdev;

	GET_SYNA_DATA(dev);

	switch (to) {
	case TS_MMI_PM_DEEPSLEEP:
		tcm_hcd->wakeup_gesture_enabled = false;
		syna_tcm_early_suspend(&pdev->dev);
		syna_tcm_suspend(&pdev->dev);
		break;

	case TS_MMI_PM_GESTURE:
		tcm_hcd->wakeup_gesture_enabled = true;
		syna_tcm_early_suspend(&pdev->dev);
		syna_tcm_suspend(&pdev->dev);
		break;

	case TS_MMI_PM_ACTIVE:
		break;

	default:
		dev_warn(dev, "panel mode %d is invalid.\n", to);
		return -EINVAL;
	}
	return 0;
}

static struct ts_mmi_methods syna_ts_mmi_methods = {
	.get_vendor = syna_ts_mmi_methods_get_vendor,
	.get_productinfo = syna_ts_mmi_methods_get_productinfo,
	.get_build_id = syna_ts_mmi_methods_get_build_id,
	.get_config_id = syna_ts_mmi_methods_get_config_id,
	.get_bus_type = syna_ts_mmi_methods_get_bus_type,
	.get_irq_status = syna_ts_mmi_methods_get_irq_status,
	.get_drv_irq = syna_ts_mmi_methods_get_drv_irq,
	.get_flashprog = syna_ts_mmi_methods_get_flashprog,
	.get_poweron = syna_ts_mmi_methods_get_poweron,
	/* SET methods */
	.reset =  syna_ts_mmi_methods_reset,
	.drv_irq = syna_ts_mmi_methods_drv_irq,
	.power = syna_ts_mmi_methods_power,
	/* Firmware */
	.firmware_update = syna_ts_firmware_update,
	 /* vendor specific attribute group */
	.extend_attribute_group = syna_ts_extend_attribute_group,
	/* PM callback */
	.panel_state = syna_ts_mmi_panel_state,
	.post_resume = syna_ts_mmi_post_resume,
};

int syna_ts_mmi_dev_register(struct syna_tcm_hcd *tcm_hcd) {
	int ret;
	const struct syna_tcm_board_data *bdata = tcm_hcd->hw_if->bdata;

	ret = ts_mmi_dev_register(tcm_hcd->pdev->dev.parent, &syna_ts_mmi_methods);
	if (ret) {
		dev_err(tcm_hcd->pdev->dev.parent, "Failed to register ts mmi\n");
		return ret;
	}

	/* initialize class imported methods */
	tcm_hcd->imports = &syna_ts_mmi_methods.exports;

	if(bdata->pill_region_ctrl) {
		ret = syna_ts_get_pill_region(tcm_hcd);
		if (ret < 0)
			dev_err(tcm_hcd->pdev->dev.parent, "%s: failed to read pill region (%d)\n",
					__func__, ret);
			dev_err(tcm_hcd->pdev->dev.parent, "%s: pill region is set to default value\n", __func__);
			tcm_hcd->region_data.region_side = 0x00;
			tcm_hcd->region_data.y_start_l = 0x00;
			tcm_hcd->region_data.y_end_l = 0x00;
			tcm_hcd->region_data.y_start_r = 0x00;
			tcm_hcd->region_data.y_end_r = 0x00;
	}

	if(bdata->gs_distance_ctrl) {
		ret = tcm_hcd->get_dynamic_config(tcm_hcd, DC_SUPP_X_WIDTH,
						&tcm_hcd->gs_distance_data);
		if (ret < 0) {
			dev_err(tcm_hcd->pdev->dev.parent, "%s: failed to read grip supp distance (%d)\n",
					__func__, ret);
			dev_err(tcm_hcd->pdev->dev.parent, "%s: gs-distance is set to default value\n", __func__);
			tcm_hcd->gs_distance_data = 0x1E;
		}
	}

	return 0;
}

void syna_ts_mmi_dev_unregister(struct syna_tcm_hcd *tcm_hcd) {
	ts_mmi_dev_unregister(tcm_hcd->pdev->dev.parent);
}
