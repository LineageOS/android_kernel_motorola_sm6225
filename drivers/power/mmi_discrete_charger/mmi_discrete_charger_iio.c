/* Copyright (c) 2020, 2021 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/power_supply.h>
#include "mmi_discrete_charger_iio.h"

static int mmi_discrete_iio_write_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan, int val1,
		int val2, long mask)
{
	struct mmi_discrete_charger *chip = iio_priv(indio_dev);
	int rc = 0;

	switch (chan->channel) {
	case PSY_IIO_MMI_OTG_ENABLE:
		rc = mmi_discrete_otg_enable(chip, !!val1);
		break;
	case PSY_IIO_TYPEC_MODE:
		rc = mmi_discrete_config_typec_mode(chip, val1);
		break;
	case PSY_IIO_PD_ACTIVE:
		rc = mmi_discrete_config_pd_active(chip, val1);
		break;
	case PSY_IIO_USB_CHARGING_ENABLED:
		rc = mmi_discrete_config_charging_enabled(chip, val1);
		break;
	case PSY_IIO_INPUT_CURRENT_SETTLED:
		rc = mmi_discrete_config_input_current_settled(chip, val1);
		break;
	case PSY_IIO_USB_TERMINATION_ENABLED:
		rc = mmi_discrete_config_termination_enabled(chip, val1);
		break;
	case PSY_IIO_DP_DM:
		if (chip->real_charger_type == POWER_SUPPLY_TYPE_USB_HVDCP_3
			|| chip->real_charger_type == POWER_SUPPLY_TYPE_USB_HVDCP_3P5) {
			rc = mmi_discrete_set_dp_dm(chip, val1);
		}
		break;
	case PSY_IIO_CP_ENABLE:
		chip->cp_active = !!val1;
		break;
	default:
		pr_err("Unsupported mmi_discrete IIO chan %d\n", chan->channel);
		rc = -EINVAL;
		break;
	}

	if (rc < 0)
		pr_err("Couldn't write IIO channel %d, rc = %d\n",
			chan->channel, rc);

	return rc;
}

static int mmi_discrete_iio_read_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan, int *val1,
		int *val2, long mask)
{
	struct mmi_discrete_charger *chip = iio_priv(indio_dev);
	int rc = 0;
	bool val_bool = false, val_suspend = false;
	*val1 = 0;

	switch (chan->channel) {
	case PSY_IIO_USB_REAL_TYPE:
		*val1 = chip->real_charger_type;
		break;
	case PSY_IIO_HW_CURRENT_MAX:
		rc = mmi_discrete_get_hw_current_max(chip, val1);
		break;
	case PSY_IIO_USB_CHARGING_ENABLED:
		mmi_discrete_get_charging_enabled(chip, &val_bool);
		mmi_discrete_get_charger_suspend(chip, &val_suspend);
		if(val_bool && !val_suspend)
			*val1 = 1;
		else
			*val1 = 0;
		break;
	case PSY_IIO_MMI_QC3P_POWER:
		rc = mmi_discrete_get_qc3p_power(chip, val1);
		break;
	case PSY_IIO_DP_DM:
		rc = mmi_discrete_get_pulse_cnt(chip, val1);
		break;
	case PSY_IIO_TYPEC_ACCESSORY_MODE:
		rc = mmi_discrete_get_typec_accessory_mode(chip, val1);
		break;
	case PSY_IIO_PD_ACTIVE:
		*val1 = chip->pd_active;
		break;
	case PSY_IIO_CP_ENABLE:
		*val1 = chip->cp_active;
		break;
	default:
		pr_err("Unsupported mmi_discrete IIO chan %d\n", chan->channel);
		rc = -EINVAL;
		break;
	}

	if (rc < 0) {
		pr_err("Couldn't read IIO channel %d, rc = %d\n",
			chan->channel, rc);
		return rc;
	}

	return IIO_VAL_INT;
}

static int mmi_discrete_iio_of_xlate(struct iio_dev *indio_dev,
				const struct of_phandle_args *iiospec)
{
	struct mmi_discrete_charger *chip = iio_priv(indio_dev);
	struct iio_chan_spec *iio_chan = chip->iio_chan;
	int i;

	for (i = 0; i < ARRAY_SIZE(mmi_discrete_iio_psy_channels);
					i++, iio_chan++)
		if (iio_chan->channel == iiospec->args[0])
			return i;

	return -EINVAL;
}

static const struct iio_info mmi_discrete_iio_info = {
	.read_raw	= mmi_discrete_iio_read_raw,
	.write_raw	= mmi_discrete_iio_write_raw,
	.of_xlate	= mmi_discrete_iio_of_xlate,
};

int mmi_discrete_init_iio_psy(struct mmi_discrete_charger *chip,
				struct platform_device *pdev)
{
	struct iio_dev *indio_dev = chip->indio_dev;
	struct iio_chan_spec *chan;
	int mmi_discrete_num_iio_channels = ARRAY_SIZE(mmi_discrete_iio_psy_channels);
	int rc, i;

	chip->iio_chan = devm_kcalloc(chip->dev, mmi_discrete_num_iio_channels,
				sizeof(*chip->iio_chan), GFP_KERNEL);
	if (!chip->iio_chan)
		return -ENOMEM;

	chip->int_iio_chans = devm_kcalloc(chip->dev,
				mmi_discrete_num_iio_channels,
				sizeof(*chip->int_iio_chans),
				GFP_KERNEL);
	if (!chip->int_iio_chans)
		return -ENOMEM;

	indio_dev->info = &mmi_discrete_iio_info;
	indio_dev->dev.parent = chip->dev;
	indio_dev->dev.of_node = chip->dev->of_node;
	indio_dev->name = "mmi-discrete-iio";
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = chip->iio_chan;
	indio_dev->num_channels = mmi_discrete_num_iio_channels;

	for (i = 0; i < mmi_discrete_num_iio_channels; i++) {
		chip->int_iio_chans[i].indio_dev = indio_dev;
		chan = &chip->iio_chan[i];
		chip->int_iio_chans[i].channel = chan;
		chan->address = i;
		chan->channel = mmi_discrete_iio_psy_channels[i].channel_num;
		chan->type = mmi_discrete_iio_psy_channels[i].type;
		chan->datasheet_name =
			mmi_discrete_iio_psy_channels[i].datasheet_name;
		chan->extend_name =
			mmi_discrete_iio_psy_channels[i].datasheet_name;
		chan->info_mask_separate =
			mmi_discrete_iio_psy_channels[i].info_mask;
	}

	rc = devm_iio_device_register(chip->dev, indio_dev);
	if (rc) {
		pr_err("Failed to register mmi-discrete-iio IIO device, rc=%d\n", rc);
		return rc;
	}

	return rc;
}
