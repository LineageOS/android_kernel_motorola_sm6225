#include "aw9620x.h"
#include "aw_update_fw_interface.h"

#define AW9620X_DRIVER_VERSION 		"v0.0.8"

static uint32_t aw9620x_rc_irqscr(void *i2c);

static void aw9620x_irq_handle_func(uint32_t irq_status, void *data)
{
	int32_t ret = 0;
	uint32_t curr_status_val = 0;
	uint32_t curr_status = 0;
	uint8_t i = 0;
	struct aw_sar *p_sar = (struct aw_sar *)data;

	AWLOGD(p_sar->dev, "IRQSRC = 0x%x", irq_status);

	ret = aw_sar_i2c_read(p_sar->i2c, REG_STAT0, &curr_status_val);
	if (ret < 0) {
		AWLOGE(p_sar->dev, "i2c IO error");
		return;
	}
	AWLOGD(p_sar->dev, "STAT0 = 0x%x", curr_status_val);

	for (i = 0; i < AW9620X_CHANNEL_NUM_MAX; i++) {
		curr_status = (((curr_status_val >> (OFFSET_BIT_8 + i)) & 0x1) << 1) |
				(((curr_status_val >> (i)) & 0x1));
		if (p_sar->channels_arr[i].used == AW_FALSE) {
			continue;
		}
		if (p_sar->channels_arr[i].last_channel_info == curr_status) {
			continue;
		}

		switch (curr_status) {
		case AW9620X_TRIGGER_FAR:
			input_report_abs(p_sar->channels_arr[i].input, ABS_DISTANCE, AW9620X_TRIGGER_FAR);
			break;
		case AW9620X_TRIGGER_TH0:
			input_report_abs(p_sar->channels_arr[i].input, ABS_DISTANCE, AW9620X_TRIGGER_TH0);
			break;
		case AW9620X_TRIGGER_TH1:
			input_report_abs(p_sar->channels_arr[i].input, ABS_DISTANCE, AW9620X_TRIGGER_TH1);
			break;
		default:
			AWLOGD(p_sar->dev, "error abs distance");
			return;
		}
		input_sync(p_sar->channels_arr[i].input);

		p_sar->channels_arr[i].last_channel_info = curr_status;
	}
}

int32_t aw9620x_check_chipid(void *data)
{
	int32_t ret = -AW_ERR;
	uint32_t reg_val = 0;
	struct aw_sar *p_sar = (struct aw_sar *)data;

	if ((p_sar == NULL) && ( p_sar->priv_data == NULL)) {
		return -AW_BIN_PARA_INVALID;
	}

	AWLOGD(p_sar->dev, "enter");

	ret = aw_sar_i2c_read(p_sar->i2c, REG_CHIP_ID, &reg_val);
	if (ret < 0) {
		AWLOGE(p_sar->dev, "read CHIP ID failed: %d", ret);
		return -AW_ERR;
	}

	switch (reg_val)
	{
		case AW96203CSR_CHIP_ID:
			memcpy(p_sar->chip_name, "aw96203", 8);
			AWLOGD(p_sar->dev, "AW96203CSR CHIP ID : 0x%x", reg_val);
		break;
		case AW96205DNR_CHIP_ID:
			memcpy(p_sar->chip_name, "aw96205", 8);
			AWLOGD(p_sar->dev, "AW96205DNR CHIP ID : 0x%x", reg_val);
		break;
		case AW96208CSR_CHIP_ID:
			memcpy(p_sar->chip_name, "aw96208", 8);
			AWLOGD(p_sar->dev, "AW96208CSR CHIP ID : 0x%x", reg_val);
		break;
		default:
			AWLOGD(p_sar->dev, "no chipid,need update root and frimware,CHIP ID : 0x%x", reg_val);
			return -AW_ERR_CHIPID;
		break;
	}

	p_sar->chip_type = AW_SAR_NONE_CHECK_CHIP;

	return AW_OK;
}

static int32_t aw9620x_soft_reset(void *data)
{
	int32_t ret = 0;
	struct aw_sar *p_sar = (struct aw_sar *)data;

	AWLOGD(p_sar->dev, "enter");

	ret = aw_sar_i2c_write(p_sar->i2c, REG_ACCESSEN, REG_OPEN_APB_ACCESS_EN);
	if (ret < 0) {
		AWLOGE(p_sar->dev, "read REG_APB_ACCESS_EN err: %d", ret);
		return -AW_ERR;
	}

	ret = aw_sar_i2c_write(p_sar->i2c, AW_REG_FLASH_WAKE_UP, AW_REG_FLASH_WAKE_UP_ON);
	if (ret < 0) {
		AWLOGE(p_sar->dev, "read REG_APB_ACCESS_EN err: %d", ret);
		return -AW_ERR;
	}
	msleep(1);

	ret = aw_sar_i2c_write(p_sar->i2c, REG_MCFG, REG_SET_MCFG00);
	if (ret != AW_OK) {
		AWLOGD(p_sar->dev, "REG_MCFG err");
		return ret;
	}
	msleep(AW9620X_REG_MCFG_DELAY_MS);

	ret = aw_sar_i2c_write(p_sar->i2c, REG_RSTNALL, AW9620X_REG_RSTNALL_VAL);
	if (ret < 0) {
		AWLOGE(p_sar->dev, "read soft_reset err: %d", ret);
		return -AW_ERR;
	}

	msleep(AW9620X_POWER_ON_DELAY_MS);

	ret = aw_sar_i2c_write(p_sar->i2c, AW_REG_FLASH_WAKE_UP, AW_REG_FLASH_WAKE_UP_OFF);
	if (ret < 0) {
		AWLOGE(p_sar->dev, "read REG_APB_ACCESS_EN err: %d", ret);
		return -AW_ERR;
	}

	return AW_OK;
}

static int32_t aw9620x_load_reg_bin(struct aw_bin *aw_bin, void *load_bin_para)
{
	int32_t ret = 0;
	struct aw_sar *p_sar = (struct aw_sar *)load_bin_para;
/*
	if (p_sar->chip_name == NULL) {
		AWLOGE(p_sar->dev, "p_sar->chip_name is NULL, error!");
		return -AW_ERR;
	}
*/
	AWLOGE(p_sar->dev, "reg chip name: %s, soc chip name: %s, len = %d",
				p_sar->chip_name, aw_bin->header_info[0].chip_type, aw_bin->info.len);

	ret = strncmp(p_sar->chip_name, aw_bin->header_info[0].chip_type, sizeof(aw_bin->header_info[0].chip_type));
	if (ret != 0) {
		AWLOGE(p_sar->dev,
			"load_binname(%s) incompatible with chip type(%s)",
			p_sar->chip_name, aw_bin->header_info[0].chip_type);
		//return -AW_ERR;
	}
	ret = aw_sar_load_reg(aw_bin, p_sar->i2c);

	return ret;
}

/*************************fw update start************************/
static int32_t aw9620x_reg_update_firmware(struct aw_bin *aw_bin, void *load_bin_para)
{
	uint32_t start_index = aw_bin->header_info[0].valid_data_addr;
	int32_t ret = 0;
	struct aw_sar *p_sar = (struct aw_sar *)load_bin_para;

	ret = aw9620x_reg_update_fw(p_sar->i2c,
				&(aw_bin->info.data[start_index]), aw_bin->header_info[0].valid_data_len);
	if (ret != AW_OK) {
		AWLOGE(p_sar->dev, "reg update fw error");
	}

	ret = aw9620x_soft_reset(p_sar);
	aw9620x_rc_irqscr(p_sar->i2c);

	return ret;
}

//BOOT update
static int32_t aw9620x_reg_update_bt(struct aw_bin *aw_bin, void *load_bin_para)
{
	uint32_t start_index = aw_bin->header_info[0].valid_data_addr;
	struct aw_sar *p_sar = (struct aw_sar *)load_bin_para;

	return aw9620x_reg_update_boot(p_sar->i2c,
				&(aw_bin->info.data[start_index]), aw_bin->header_info[0].valid_data_len);
}

static int32_t aw9620x_prot_update_firmware(struct aw_bin *aw_bin,void *load_bin_para)
{
	uint32_t start_index = aw_bin->header_info[0].valid_data_addr;
	uint32_t fw_bin_version = aw_bin->header_info[0].app_version;
	struct aw_sar *p_sar = (struct aw_sar *)load_bin_para;
	//If the verification of flash firmware area fails, the version number will not be compared, and it will be loaded directly
	uint8_t direct_updata_flag = p_sar->fw_fail_flag;

	return aw9620x_prot_update_fw(p_sar->i2c, direct_updata_flag,
				&(aw_bin->info.data[start_index]), aw_bin->header_info[0].valid_data_len, fw_bin_version);
}

static int32_t aw9620x_prox_direct_upgrade_firmware(struct aw_bin *aw_bin, void *load_bin_para)
{
	uint32_t start_index = aw_bin->header_info[0].valid_data_addr;
	uint32_t fw_bin_version = aw_bin->header_info[0].app_version;
	struct aw_sar *p_sar = (struct aw_sar *)load_bin_para;

	return aw9620x_prot_update_fw(p_sar->i2c, AW_TRUE,
				&(aw_bin->info.data[start_index]), aw_bin->header_info[0].valid_data_len, fw_bin_version);
}

/*************************fw update end************************/

/**********************mode operation start*******************************/
static void aw9620x_enable_clock(void *i2c)
{
	aw_sar_i2c_write(i2c, REG_HOSTCTRL, REG_HOSTCTRL_EN);
}

static uint32_t aw9620x_rc_irqscr(void *i2c)
{
	uint32_t val = 0;
	aw_sar_i2c_read(i2c, REG_IRQSRC, &val);
	return val;
}

static void aw9620x_set_active_cmd(void *i2c)
{
	aw_sar_i2c_write(i2c, REG_CMD, AW9620X_ACTIVE_MODE);
}

static void aw9620x_set_sleep_cmd(void *i2c)
{
	aw_sar_i2c_write(i2c, REG_CMD, AW9620X_SLEEP_MODE);
}

static void aw9620x_set_deepsleep_cmd(void *i2c)
{
	aw_sar_i2c_write(i2c, REG_CMD, AW9620X_DEEPSLEEP_MODE);
}

static ssize_t aw9620x_mode_operation_get(void *data, char *buf)
{
	uint32_t reg_val = 0;
	struct aw_sar *p_sar = (struct aw_sar *)data;

	aw_sar_i2c_read(p_sar->i2c, REG_WST, &reg_val);

	return snprintf(buf, PAGE_SIZE, "mode : %d, REG_WST :0x%x \n",
				p_sar->last_mode, reg_val);
}

static void aw9620x_mul_cap_parasitic_data_get(struct aw_sar *p_sar,
				char *buf, uint32_t ch, uint32_t *p_len, uint32_t reg_data)
{
	uint32_t j = 0;
	uint32_t reg_data_cfg_val = 0;
	uint32_t off_m = 0;
	uint32_t fine_off_f_temp = 0;
	uint32_t off_f = 0;
	uint32_t off_c = 0;
	int32_t s_off_c = 0;
	int32_t parasitic_data_temp = 0;
	int32_t parasitic_data_int = 0;
	int32_t parasitic_data_float = 0;

	if (((reg_data >> AW9620X_AFECFG3_CVOFF2X) & 0x1) == 0) {
		aw_sar_i2c_read(p_sar->i2c, REG_AFECFG1_CH0 + AW9620X_CFG_REG_STEP * ch, &reg_data_cfg_val);
		off_m = (reg_data_cfg_val >> 16) & 0xff;
		off_f = reg_data_cfg_val & 0xff;
		fine_off_f_temp = off_f * AW9620X_FINE_ADJUST_STEP0;
		off_c = (reg_data_cfg_val >> 8) & 0xff;
		for (j = 0; j < 8; j++) {
			s_off_c += ((1 - 2 * ((off_m >> j) & 0x1)) * ((off_c >> j) & 0x1) * aw_sar_pow2(j)) * AW9620X_COARSE_ADJUST_STEP0;
		}

		parasitic_data_temp = AW_SAR_ABS((int32_t)fine_off_f_temp + s_off_c);
		parasitic_data_int = parasitic_data_temp / AW9620X_PARA_TIMES;
		parasitic_data_float = parasitic_data_temp % AW9620X_PARA_TIMES;
		*p_len += snprintf(buf + *p_len, PAGE_SIZE - *p_len, "ch%d parasitic_data %d.%d pf\n", ch, parasitic_data_int, parasitic_data_float);
	} else {
		*p_len += snprintf(buf + *p_len, PAGE_SIZE - *p_len, "ch%d Mutual tolerance mode does not allow 2x to be configured", ch);
	}
}

static void aw9620x_self_cap_parasitic_data_get(struct aw_sar *p_sar,
					char *buf, uint32_t ch, uint32_t *p_len, uint32_t reg_data)
{
	uint32_t reg_data_cfg = 0;
	uint32_t reg_data_cfg_val = 0;
	uint32_t reg_data_cfg_tmp0 = 0;
	uint32_t reg_data_cfg_tmp1 = 0;
	uint32_t cfg_int = 0;
	uint32_t cfg_float = 0;

	aw_sar_i2c_read(p_sar->i2c, REG_AFECFG1_CH0 + AW9620X_CFG_REG_STEP * ch, &reg_data_cfg_val);

	if (((reg_data >> AW9620X_AFECFG3_CVOFF2X) & 0x1) == 0) {
		reg_data_cfg_tmp0 = (reg_data_cfg_val & 0xff) * AW9620X_FINE_ADJUST_STEP0;
		reg_data_cfg_tmp1 = ((reg_data_cfg_val >> 8) & 0xff) * AW9620X_COARSE_ADJUST_STEP0;
	} else {
		reg_data_cfg_tmp0 = (reg_data_cfg_val & 0xff) * AW9620X_FINE_ADJUST_STEP1;
		reg_data_cfg_tmp1 = ((reg_data_cfg_val >> 8) & 0xff) * AW9620X_COARSE_ADJUST_STEP1;
	}
	reg_data_cfg = reg_data_cfg_tmp0 + reg_data_cfg_tmp1;
	cfg_int = reg_data_cfg / AW9620X_PARA_TIMES;
	cfg_float = reg_data_cfg % AW9620X_PARA_TIMES;
	*p_len+= snprintf(buf + *p_len, PAGE_SIZE - *p_len, "ch%d parasitic_data %d.%d pf\n", ch, cfg_int, cfg_float);
}

static ssize_t aw9620x_offset_get(void *data, char *buf)
{
	uint32_t len = 0;
	uint32_t i = 0;
	uint32_t reg_data = 0;
	struct aw_sar *p_sar = (struct aw_sar *)data;

	for (i = 0; i < AW9620X_CHANNEL_NUM_MAX; i++) {
		aw_sar_i2c_read(p_sar->i2c, REG_AFECFG3_CH0 + AW9620X_CFG_REG_STEP * i, &reg_data);
		if (((reg_data >> AW9620X_AFECFG3_CVMULTUALMOD) & 0x1) != 1) {
			//self contained
			aw9620x_self_cap_parasitic_data_get(p_sar, buf, i, &len, reg_data);
		} else {
			//mutual compatibility
			aw9620x_mul_cap_parasitic_data_get(p_sar, buf, i, &len, reg_data);
		}
	}

	return len;
}

static void aw9620x_sar_chip_info_get(void *data, char *buf, ssize_t *p_len)
{
	uint32_t reg_data = 0;
	struct aw_sar *p_sar = (struct aw_sar *)data;

	*p_len += snprintf(buf + *p_len, PAGE_SIZE - *p_len, "sar%d, aw9620x chip driver version:%s\n",
							p_sar->dts_info.sar_num, AW9620X_DRIVER_VERSION);
	*p_len += snprintf(buf + *p_len, PAGE_SIZE - *p_len, "The driver supports UI\n");

	aw_sar_i2c_read(p_sar->i2c, REG_CHIP_ID, &reg_data);
	*p_len += snprintf(buf + *p_len, PAGE_SIZE - *p_len, "chipid is 0x%08x\n", reg_data);

	aw_sar_i2c_read(p_sar->i2c, REG_IRQEN, &reg_data);
	*p_len += snprintf(buf + *p_len, PAGE_SIZE - *p_len, "REG_HOSTIRQEN is 0x%08x\n", reg_data);
}

static void aw9620x_get_firmware_version(void *data, char *buf, ssize_t *p_len)
{
	uint32_t fw_ver = 0;
	struct aw_sar *p_sar = (struct aw_sar *)data;

	aw_sar_i2c_read(p_sar->i2c, REG_FWVER, &fw_ver);

	*p_len += snprintf(buf + *p_len, PAGE_SIZE - *p_len, "current firmware version is 0x%08x\n", fw_ver);
}

static int32_t aw9320x_get_err_info(void *data)
{
	uint32_t err_code = 0;
	uint32_t boot_mode = 0;
	uint32_t jump_info = 0;
	struct aw_sar *p_sar = (struct aw_sar *)data;

	aw_sar_i2c_write(p_sar->i2c, REG_ACCESSEN, REG_ACCESSEN_OPEN);

	//Get error check code,(0x09: Firmware verification failed.0x7:boot verification failed) 
	aw_sar_i2c_read(p_sar->i2c, AW9320X_SRAM_ERROR_CODE, &err_code);

	//The bit[31:24] must be 0, otherwise the boot cannot be jumped out actively
	aw_sar_i2c_read(p_sar->i2c, REG_UPDATA_DIS, &jump_info);

	//bit8 0:Boot from ROM, 1: Boot from RAM
	aw_sar_i2c_read(p_sar->i2c, REG_MCFG, &boot_mode);

	AWLOGE(p_sar->dev, "0x1c00:0x%x, 0x4744:0x%x, 0x4444:0x%x", err_code, jump_info, boot_mode);

	aw_sar_i2c_write(p_sar->i2c, REG_ACCESSEN, REG_ACCESSEN_CLOSE);

	return AW_OK;
}


static const struct aw_sar_mode_set_t g_aw9620x_mode_set[] = {
	//0
	{
		.chip_id = AW_SAR_NONE_CHECK_CHIP,
		.chip_mode = {
			.curr_mode = AW9620X_ACTIVE_MODE,
			.last_mode = AW9620X_DEEPSLEEP_MODE,
		},
		.mode_switch_ops = {
			.enable_clock = aw9620x_enable_clock,
			.rc_irqscr = NULL,
			.mode_update = aw9620x_set_active_cmd,
		},
	},
	//1
	{
		.chip_id = AW_SAR_NONE_CHECK_CHIP,
		.chip_mode = {
			.curr_mode = AW9620X_ACTIVE_MODE,
			.last_mode = AW9620X_SLEEP_MODE,
		},
		.mode_switch_ops = {
			.enable_clock = NULL,
			.rc_irqscr = NULL,
			.mode_update = aw9620x_set_active_cmd,
		},
	},
	//1
	{
		.chip_id = AW_SAR_NONE_CHECK_CHIP,
		.chip_mode = {
			.curr_mode = AW9620X_ACTIVE_MODE,
			.last_mode = AW9620X_ACTIVE_MODE,
		},
		.mode_switch_ops = {
			.enable_clock = NULL,
			.rc_irqscr = NULL,
			.mode_update = aw9620x_set_active_cmd,
		},
	},
	//2
	{
		.chip_id = AW_SAR_NONE_CHECK_CHIP,
		.chip_mode = {
			.curr_mode = AW9620X_SLEEP_MODE,
			.last_mode = AW9620X_DEEPSLEEP_MODE,
		},
		.mode_switch_ops = {
			.enable_clock = aw9620x_enable_clock,
			.rc_irqscr = NULL,
			.mode_update = aw9620x_set_sleep_cmd,
		},
	},
	//3
	{
		.chip_id = AW_SAR_NONE_CHECK_CHIP,
		.chip_mode = {
			.curr_mode = AW9620X_SLEEP_MODE,
			.last_mode = AW9620X_ACTIVE_MODE,
		},
		.mode_switch_ops = {
			.enable_clock = NULL,
			.rc_irqscr = NULL,
			.mode_update = aw9620x_set_sleep_cmd,
		},
	},
	//4
	{
		.chip_id = AW_SAR_NONE_CHECK_CHIP,
		.chip_mode = {
			.curr_mode = AW9620X_DEEPSLEEP_MODE,
			.last_mode = AW9620X_SLEEP_MODE,
		},
		.mode_switch_ops = {
			.enable_clock = NULL,
			.rc_irqscr = NULL,
			.mode_update = aw9620x_set_deepsleep_cmd,
		},
	},
	//5
	{
		.chip_id = AW_SAR_NONE_CHECK_CHIP,
		.chip_mode = {
			.curr_mode = AW9620X_DEEPSLEEP_MODE,
			.last_mode = AW9620X_ACTIVE_MODE,
		},
		.mode_switch_ops = {
			.enable_clock = NULL,
			.rc_irqscr = aw9620x_rc_irqscr,
			.mode_update = aw9620x_set_deepsleep_cmd,
		},
	},
};

static const struct aw_sar_check_chipid_t g_aw9620x_check_chipid = {
	.p_check_chipid_fn = aw9620x_check_chipid,
};

static const struct aw_sar_soft_rst_t g_aw9620x_soft_rst = {
	.p_soft_reset_fn = aw9620x_soft_reset,
};

static const struct aw_sar_get_chip_info_t g_aw9620x_get_chip_info = {
	.p_get_chip_info_node_fn = aw9620x_sar_chip_info_get,
};

#define AW9620X_MODE_SET_ARR_LEN  (sizeof(g_aw9620x_mode_set) / sizeof(g_aw9620x_mode_set[0]))

static const struct aw_sar_irq_init_t g_aw9620x_irq_init = {
	.flags = GPIOF_DIR_IN | GPIOF_INIT_HIGH,
	.irq_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
	.handler = NULL,
	.thread_fn = NULL,
	.rc_irq_fn = aw9620x_rc_irqscr,
	.irq_spec_handler_fn = aw9620x_irq_handle_func,
};

static const struct aw_sar_init_over_irq_t g_aw9620x_init_over_irq = {
	.wait_times = 100,
	.daley_step = 1,
	.reg_irqsrc = REG_IRQSRC,
	.irq_offset_bit = 0,
	.irq_mask = 0x1,
	.irq_flag = 0x1,
	.p_check_init_over_irq_fn = NULL,
	.p_get_err_type_fn = aw9320x_get_err_info,
};

static const struct aw_sar_load_bin_t g_aw9620x_load_reg_bin = {
	.bin_name = "aw9620x_reg",
	.bin_opera_func = aw9620x_load_reg_bin,
	.bin_load_fail_opera = NULL,
	.p_update_fn = NULL,
};

static const struct aw_sar_load_bin_t g_aw9620x_load_boot_bin = {
	.bin_name = "aw9620x_boot",
	.bin_opera_func = aw9620x_reg_update_bt,
	.bin_load_fail_opera = NULL,
};

static const struct aw_sar_load_bin_t g_aw9620x_load_fw_bin = {
	.bin_name = "aw9620x_fw",
	.bin_opera_func = aw9620x_prot_update_firmware,
	.bin_load_fail_opera = aw9620x_reg_update_firmware,
};

//Node prox upgrade firmware
static const struct aw_sar_load_bin_t g_aw9620x_node_prox_load_fw_bin = {
	.bin_name = "aw9620x_fw",
	.bin_opera_func = aw9620x_prox_direct_upgrade_firmware,
	.p_get_prot_update_fw_node_fn = aw9620x_get_firmware_version,
	.bin_load_fail_opera = NULL,
};

//Node reg upgrade firmware
static const struct aw_sar_load_bin_t g_aw9620x_node_reg_load_fw_bin = {
	.bin_name = "aw9620x_fw",
	.bin_opera_func = aw9620x_reg_update_firmware,
	.bin_load_fail_opera = NULL,
};

static const struct aw_sar_para_load_t g_aw9620x_reg_arr_para = {
	.reg_arr = aw9620x_reg_default,
	.reg_arr_len = sizeof(aw9620x_reg_default) / sizeof(aw9620x_reg_default[0]),
};

static const struct aw_sar_aot_t g_aw9620x_aot = {
	.aot_reg = REG_SCANCTRL0,
	.aot_mask = ~(0xff << 8),
	.aot_flag = 0xff << 8,
};

static const struct aw_sar_diff_t g_aw9620x_diff = {
	.diff0_reg = REG_DIFF_CH0,
	.diff_step = AW9620X_REG_STEP,
	.rm_float = AW9620X_REMOVE_FLOAT_COEF,
};

static const struct aw_sar_offset_t g_aw9620x_offset = {
	.p_get_offset_node_fn = aw9620x_offset_get,
};

static const struct aw_sar_mode_t g_aw9620x_mode = {
	.mode_set_arr = &g_aw9620x_mode_set[0],
	.mode_set_arr_len = AW9620X_MODE_SET_ARR_LEN,
	.p_get_mode_node_fn = aw9620x_mode_operation_get,
};

static const struct aw_sar_reg_list_t g_aw9620x_reg_list = {
	.reg_none_access = REG_NONE_ACCESS,
	.reg_rd_access = REG_RD_ACCESS,
	.reg_wd_access = REG_WR_ACCESS,
	.reg_perm = (struct aw_sar_reg_data *)&g_aw9620x_reg_access[0],
	.reg_num = sizeof(g_aw9620x_reg_access) / sizeof(g_aw9620x_reg_access[0]),
};

static const struct aw_sar_pm_t g_aw9620x_pm_chip_mode = {
	.suspend_set_mode = AW9620X_SLEEP_MODE,
	.resume_set_mode = AW9620X_ACTIVE_MODE,
	.shutdown_set_mode = AW9620X_SLEEP_MODE,
};

static const struct aw_sar_chip_mode_t g_aw9620x_chip_mode = {
	.init_mode = AW9620X_ACTIVE_MODE,
	.active = AW9620X_ACTIVE_MODE,
	.pre_init_mode = AW9620X_SLEEP_MODE,
};

static const struct aw_sar_regulator_config_t g_aw9620x_regulator_config = {
	.vcc_name = "vcc",
	.min_uV = AW9620X_SAR_VCC_MIN_UV,
	.max_uV = AW9620X_SAR_VCC_MAX_UV,
};

static const struct aw_sar_platform_config g_aw9620x_platform_config = {
	.p_regulator_config = &g_aw9620x_regulator_config,
	.p_irq_init = &g_aw9620x_irq_init,
	.p_pm_chip_mode = &g_aw9620x_pm_chip_mode,
};

static const struct aw_sar_chip_config g_aw9620x_chip_config = {
	.ch_num_max = AW9620X_CHANNEL_NUM_MAX,

	.p_platform_config = &g_aw9620x_platform_config,

	.p_check_chipid = &g_aw9620x_check_chipid,
	.p_soft_rst = &g_aw9620x_soft_rst,
	.p_init_over_irq = &g_aw9620x_init_over_irq,
	.p_fw_bin = &g_aw9620x_load_fw_bin,
	.p_reg_bin = &g_aw9620x_load_reg_bin,
	.p_chip_mode = &g_aw9620x_chip_mode,

	//Node usage parameters
	.p_reg_list = &g_aw9620x_reg_list,
	.p_reg_arr = &g_aw9620x_reg_arr_para,
	.p_aot = &g_aw9620x_aot,
	.p_diff = &g_aw9620x_diff,
	.p_offset = &g_aw9620x_offset,
	.p_mode = &g_aw9620x_mode,
	.p_prox_fw = &g_aw9620x_node_prox_load_fw_bin,
	.p_reg_fw = &g_aw9620x_node_reg_load_fw_bin,
	.p_get_chip_info = &g_aw9620x_get_chip_info,
	.p_aw_sar_awrw = NULL,
	.p_boot_bin = &g_aw9620x_load_boot_bin,
};

int32_t aw9620x_init(struct aw_sar *p_sar)
{
	if (p_sar == NULL) {
		AWLOGE(p_sar->dev, "para is NULL, error!");
		return -AW_ERR;
	}

	//Chip private function operation
	p_sar->p_sar_para = &g_aw9620x_chip_config;

	return AW_OK;
}

void aw9620x_deinit(struct aw_sar *p_sar)
{
	if (p_sar->priv_data != NULL) {
		devm_kfree(p_sar->dev, p_sar->priv_data);
	}
}
