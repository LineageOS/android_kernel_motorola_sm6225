#include <aw_update_fw.h>
#include <aw_update_fw_interface.h>
/*********************protocol update frimware start****************** *****/
//#define AW_FW_INFO_LOG

//If it is a bin file, the size side conversion is required
#define AW_SAR_BIN_TR

static uint8_t g_prot_update_fw_flag = 0;

static int32_t aw_check_isp_go_reg(AW_SAR_I2C_TYPE i2c)
{
	int32_t delay_cnt = 100;
	uint32_t r_isp_go_reg = 0;
	int32_t ret = 0;

	do {
		ret = aw_sar_i2c_read(i2c, REG_ISPGO, &r_isp_go_reg);
		if (ret != AW_FW_OK) {
			AWLOGE_FW("read REG_ISPGO");
			return ret;
		}
		if (r_isp_go_reg == 0) {
			break;
		}
		aw_sar_delay_ms(1);
	} while (delay_cnt--);

	if (delay_cnt < 0) {
		AWLOGE_FW("check_isp_go_reg err!");
		return -AW_FW_ERR;
	}

	return AW_FW_OK;
}

static int32_t aw_reg_read_val(AW_SAR_I2C_TYPE i2c, uint32_t *p_read_data, uint16_t start_addr)
{
	int32_t ret = 0;

	//3.config ISPADR reg
	ret = aw_sar_i2c_write(i2c, REG_ISPADR, start_addr);
	if (ret != AW_FW_OK) {
		AWLOGE_FW("write REG_ISPADR");
		return -AW_FW_ERR;
	}
	//4.config ISP_CMD reg
	ret = aw_sar_i2c_write(i2c, REG_ISPCMD, REG_ISP_CMD_MAIN_ARR);
	if (ret != AW_FW_OK) {
		AWLOGE_FW("write REG_ISPCMD");
		return -AW_FW_ERR;
	}
	//5.config ISP_GO reg
	ret = aw_sar_i2c_write(i2c, REG_ISPGO, REG_SET_ISP_GO);
	if (ret != AW_FW_OK) {
		AWLOGE_FW("write ISP_CMD ");
		return -AW_FW_ERR;
	}
	//6.check isp_go reg
	ret = aw_check_isp_go_reg(i2c);
	if (ret != AW_FW_OK) {
		AWLOGE_FW("check_isp_go_reg");
		return -AW_FW_ERR;
	}
	//7 read data
	ret = aw_sar_i2c_read(i2c, REG_ISPRDATA, p_read_data);
	if (ret != AW_FW_OK) {
		AWLOGE_FW("read REG_ISPRDATA p_read_data = 0x%08x", *p_read_data);
		return -AW_FW_ERR;
	}

	return AW_FW_OK;
}

static uint32_t get_pack_checksum(uint8_t *data, uint16_t length,
			   				uint8_t module, uint8_t command)
{
	uint32_t i = 0;
	uint32_t check_sum = 0;

	check_sum = module + command + length;
	for (i = 0; i < length; i += 4) {
		check_sum += AW_SAR_GET_32_DATA(data[i + 0], data[i + 1],
					data[i + 2], data[i + 3]);
	}

	return (~check_sum + 1);
}

static int32_t dri_to_soc_pack_send(AW_SAR_I2C_TYPE i2c, uint8_t module, uint8_t command,
			    			 uint16_t length, uint8_t *data)
{
	int8_t cnt = AW_RETRIES;
#ifdef AW_FW_INFO_LOG
	uint32_t i = 0;
#endif
	int32_t ret = -1;
	uint32_t checksum = 0;
	uint8_t *prot_pack_w = (uint8_t *)malloc(AW_PACK_FIXED_SIZE + length + SEND_ADDR_LEN);
	if (prot_pack_w == NULL) {
		AWLOGE_FW("malloc application failed!");
		return -1;
	}

	prot_pack_w[0] = ((uint16_t)PROT_SEND_ADDR & GET_BITS_15_8) >> OFFSET_BIT_8;
	prot_pack_w[1] = (uint16_t)PROT_SEND_ADDR & GET_BITS_7_0;

	//header
	prot_pack_w[2] = ((uint16_t)AW_HEADER_VAL & GET_BITS_15_8) >> OFFSET_BIT_8;
	prot_pack_w[3] = (uint16_t)AW_HEADER_VAL & GET_BITS_7_0;

	//size
	prot_pack_w[4] = ((uint16_t)(AW_PACK_FIXED_SIZE + length) & GET_BITS_15_8) >> OFFSET_BIT_8;
	prot_pack_w[5] = (uint16_t)(AW_PACK_FIXED_SIZE + length) & GET_BITS_7_0;

	//checksum
	checksum = get_pack_checksum(data, length, module, command);
	prot_pack_w[6] = ((uint32_t)checksum & GET_BITS_31_25) >> OFFSET_BIT_24;
	prot_pack_w[7] = ((uint32_t)checksum & GET_BITS_24_16) >> OFFSET_BIT_16;
	prot_pack_w[8] = ((uint32_t)checksum & GET_BITS_15_8) >> OFFSET_BIT_8;
	prot_pack_w[9] = (uint32_t)checksum & GET_BITS_7_0;

	//module
	prot_pack_w[10] = module;

	//command
	prot_pack_w[11] = command;

	//length
	prot_pack_w[12] = ((uint16_t)length & 0xff00) >> OFFSET_BIT_8;
	prot_pack_w[13] = (uint16_t)length & 0x00ff;

	if (length != 0 && data != NULL) {
		memcpy(prot_pack_w + AW_PACK_FIXED_SIZE + SEND_ADDR_LEN, data, length);
	}

#ifdef AW_FW_INFO_LOG
	//Protocol sending content
	AWLOGD_FW("Protocol sending content PROT_SEND_ADDR:0x%x", PROT_SEND_ADDR);
	for (i = 2; i < AW_PACK_FIXED_SIZE + length + AW_ADDR_SIZE; i += 4) {
		AWLOGD_FW("i = %d, 0x%02x 0x%02x 0x%02x 0x%02x",
			i, prot_pack_w[i + 0], prot_pack_w[i + 1],
			prot_pack_w[i + 2], prot_pack_w[i + 3]);
	}
#endif

	do {
		ret = aw_sar_i2c_write_seq(i2c, prot_pack_w, AW_PACK_FIXED_SIZE + AW_ADDR_SIZE + length);
		if (ret < 0) {
				AWLOGE_FW("aw_sar_i2c_write_seq err cnt = %d ret = %d", cnt, ret);
		} else {
			break;
		}
	} while(cnt--);

	free(prot_pack_w);

	if (cnt < 0) {
		AWLOGE_FW("aw_sar_i2c_write_seq err! ret = %d", ret);
		return -AW_FW_ERR;
	}

	return AW_FW_OK;
}

static int32_t soc_to_dri_pack_recv(AW_SAR_I2C_TYPE i2c, struct aw_soc_protocol *prot_pack,
							 uint32_t pack_len, uint8_t *addr)
{
	int8_t cnt = AW_RETRIES;
	int32_t ret = -1;

	if (prot_pack == NULL || pack_len == 0) {
		return -1;
	}

	do {
		ret = aw_sar_i2c_read_seq(i2c, addr, 2, (uint8_t *)prot_pack, pack_len);
		if (ret < 0) {
			AWLOGE_FW("aw_sar_i2c_read_seq cnt = %d ret = %d", cnt, ret);
		} else {
			break;
		}
	} while(cnt--);

	if (cnt < 0) {
		AWLOGE_FW("aw_sar_i2c_read_seq! ret = %d", ret);
		return -AW_FW_ERR;
	}

	return AW_FW_OK;
}

/**
  * @brief flash init
  * @param parse pack value
  * @retval err code
  */
#define AW_SAR_FLIP_U16(value) ((((value) & (0x00FF)) << (8)) | (((value) & (0xFF00)) >> (8)))
#define AW_SAR_FLIP_U32(value) ((((value) & (0x000000FF)) << (24)) | ((((value) & (0x0000FF00))) << (8)) | (((value) & (0x00FF0000)) >> (8)) | (((value) & (0xFF000000)) >> (24)))
static int32_t soc_to_dri_pack_parse(AW_SAR_I2C_TYPE i2c, uint32_t length,
							  uint8_t module, uint8_t command)
{
	int32_t ret = -1;
	uint32_t pack_len = AW_PACK_FIXED_SIZE + length;
	uint8_t ack_addr[2] = { 0 };
	uint32_t cmd_status = 0;

	struct aw_soc_protocol *prot_pack_r = (struct aw_soc_protocol *)malloc(pack_len);
	if (prot_pack_r == NULL) {
		AWLOGE_FW("malloc application failed!");
		return -AW_FW_ERR;
	}

	ack_addr[0] = (uint8_t)(AW_ACK_ADDR >> OFFSET_BIT_8);
	ack_addr[1] = (uint8_t)AW_ACK_ADDR;

	ret = soc_to_dri_pack_recv(i2c, prot_pack_r, pack_len, ack_addr);
	if (ret != AW_FW_OK) {
		AWLOGE_FW("soc_to_dri_pack_recv failed!");
		goto err_pack_parse;
	}

	prot_pack_r->header = AW_SAR_FLIP_U16(prot_pack_r->header);
	prot_pack_r->size = AW_SAR_FLIP_U16(prot_pack_r->size);
	prot_pack_r->length = AW_SAR_FLIP_U16(prot_pack_r->length);
	prot_pack_r->checksum = AW_SAR_FLIP_U32(prot_pack_r->checksum);

	cmd_status = AW_SAR_GET_32_DATA(prot_pack_r->value[3], prot_pack_r->value[2],
				    prot_pack_r->value[1], prot_pack_r->value[0]);

	//parse soc to dri pack
/*	AWLOGD_FW("header= 0x%x, size= 0x%x, length= 0x%x, checksum= 0x%x,",
			prot_pack_r->header, prot_pack_r->size,
			prot_pack_r->length, prot_pack_r->checksum);
	AWLOGD_FW("module= 0x%x, command= 0x%x, length=0x%x, cmd_status = %d",
			prot_pack_r->module, prot_pack_r->command,
			prot_pack_r->length, cmd_status);*/

	if ((module == prot_pack_r->module) && (command == prot_pack_r->command) && (cmd_status == 0)) {
	} else {
		AWLOGE_FW("soc to dri send pack parse failed!");
		return -AW_FW_ERR;
	}

	free(prot_pack_r);

	return AW_FW_OK;

err_pack_parse:
	free(prot_pack_r);
	return -AW_FW_ERR;
}

static uint32_t aw_get_bin_checksum(const uint8_t *w_bin_offset,
					uint32_t update_data_len, uint32_t check_len)
{
	uint32_t i = 0;
	uint32_t check_sum = 0;
	uint32_t tmp = 0;
	uint32_t index = 0;

	AWLOGD_FW("update_data_len:0x%x, check_len:0x%x", update_data_len, check_len);

	for (i = 0; i < check_len; i += WORD_LEN) {
		if (i < update_data_len) {
#ifdef AW_SAR_BIN_TR
			tmp = AW_SAR_GET_32_DATA(w_bin_offset[index + 3],
				w_bin_offset[index + 2],
				w_bin_offset[index + 1],
				w_bin_offset[index + 0]);
#else
			tmp = AW_SAR_GET_32_DATA(w_bin_offset[index + 0],
				w_bin_offset[index + 1],
				w_bin_offset[index + 2],
				w_bin_offset[index + 3]);
#endif
			index  += WORD_LEN;
		} else {
			tmp = AW_FLASH_DEFAULT_VAL;
		}
		check_sum += tmp;
	}
	check_sum = ~check_sum + 1;

	return check_sum;
}

static int32_t aw_read_ack_irq(AW_SAR_I2C_TYPE i2c)
{
	uint32_t irq_stat = 0;
	int32_t cnt = AW_WAIT_IRQ_CYCLES;
	int32_t ret = 0;

	if (g_prot_update_fw_flag == SEND_UPDATE_FW_CMD) {
		cnt = AW_PROT_STOP_WAIT_IRQ_CYCLES;
	}

	do {
		ret = aw_sar_i2c_read(i2c, REG_IRQSRC, &irq_stat);
		if (ret != AW_FW_OK) {
			AWLOGE_FW("read REG_IRQSRC fail");
			return -AW_FW_ERR;
		}
		if (((irq_stat >> 29) & 0x01) == 1) {
			break;
		} else {
			//AWLOGE_FW("REG_IRQSRC val: 0x%x cnt: %d", irq_stat, cnt);
		}
		aw_sar_delay_ms(1);
	} while (cnt--);

	if (cnt == -1) {
		AWLOGE_FW("read irqsrc failed!, REG_IRQSRC val: 0x%x", irq_stat);
		return -AW_FW_ERR;
	}

	return AW_FW_OK;
}

static int32_t aw_read_init_comp_irq(AW_SAR_I2C_TYPE i2c)
{
	uint32_t irq_stat = 0;
	int32_t cnt = 10;
	int32_t ret = 0;

	do {
		ret = aw_sar_i2c_read(i2c, REG_IRQSRC, &irq_stat);
		if (ret != AW_FW_OK) {
			AWLOGD_FW("read REG_HOSTIRQSRC err");
			return -AW_FW_ERR;
		}

		if (irq_stat == INIT_OVER_IRQ_OK) {
			AWLOGD_FW("read init over irq ok success!,stop cmd success!");
			break;
		} else {
			AWLOGE_FW("stop cmd failed, REG_IRQSRC val: 0x%x cnt: %d", irq_stat, cnt);
		}

		aw_sar_delay_ms(1);
	} while (cnt--);

	if (cnt == -1) {
		AWLOGD_FW("stop cmd failed!");
		return -AW_FW_ERR;
	}

	return AW_FW_OK;
}

static int32_t aw_send_once_cmd(AW_SAR_I2C_TYPE i2c, uint8_t module, uint8_t command,
							uint8_t* send_value, uint16_t send_val_len)
{
	int32_t ret = -AW_FW_ERR;
	uint8_t recv_len = 0;
	uint32_t delay_ms_cnt = 0;

	//1.send cmd
	ret = dri_to_soc_pack_send(i2c, module, command,
				send_val_len, send_value);
	if (ret != AW_FW_OK) {
		AWLOGE_FW("UPDATE_START_CMD err");
		return -AW_FW_ERR;
	}

	ret = aw_sar_i2c_write(i2c, AW_BT_PROT_CMD_PACK_ADDR, AW_SRAM_FIRST_DETECT);
	if (ret != AW_FW_OK) {
		AWLOGE_FW("REG_CMD_SEND_TIRG err");
		return -AW_FW_ERR;
	}

	//2.send trig
	ret = aw_sar_i2c_write(i2c, REG_CMD, REG_H2C_TRIG_PARSE_CMD);
	if (ret != AW_FW_OK) {
		AWLOGE_FW("REG_CMD_SEND_TIRG err");
		return -AW_FW_ERR;
	}

	switch (g_prot_update_fw_flag) {
	case SEND_START_CMD:
		recv_len = SEND_START_CMD_RECV_LEN;
		delay_ms_cnt = SEND_START_CMD_DELAY_MS;
		break;
	case SEND_ERASE_SECTOR_CMD:
		recv_len = SEND_ERASE_CHIP_CMD_RECV_LEN;
		delay_ms_cnt = SEND_ERASE_SECTOR_CMD_DELAY_MS;
		break;
	case SEND_UPDATE_FW_CMD:
		recv_len = SEND_UPDATE_FW_CMD_RECV_LEN;
		delay_ms_cnt = 0;
		break;
	case SEND_UPDATE_CHECK_CODE_CMD:
		recv_len = SEND_UPDATE_CHECK_CODE_CMD_RECV_LEN;
		delay_ms_cnt = SEND_UPDATE_CHECK_CODE_CMD_DELAY_MS;
		break;
	case SEND_RESTORE_CMD:
		recv_len = SEND_RESTORE_CMD_RECV_LEN;
		delay_ms_cnt = SEND_RESTORE_CMD_DELAY_MS;
		break;
	default:
		recv_len = 0;
		delay_ms_cnt = 0;
		break;
	}

	aw_sar_delay_ms(delay_ms_cnt);

	//3.Read interrupt information, wait 100ms
	ret = aw_read_ack_irq(i2c);
	if (ret != AW_FW_OK) {
		AWLOGE_FW("read_ack_irq err");
		return -AW_FW_ERR;
	}

	//4.read start ack and pare pack
	ret = soc_to_dri_pack_parse(i2c, recv_len, module, command);
	if (ret != AW_FW_OK) {
		AWLOGE_FW("soc_to_dri_pack_parse err");
		return -AW_FW_ERR;
	}

	return AW_FW_OK;
}

static int32_t aw_send_stop_cmd(AW_SAR_I2C_TYPE i2c)
{
	int32_t ret = -AW_FW_ERR;

	AWLOGD_FW("enter");

	aw_sar_i2c_write(i2c, AW_REG_MCFG, AW_CPU_HALT);
	aw_sar_i2c_write(i2c, AW_REG_ACESS_EN, AW_ACC_PERI);
	aw_sar_i2c_write_bits(i2c, REG_UPDATA_DIS, AW_REG_UPDATA_DIS_MASK, 0);
	aw_sar_i2c_write(i2c, REG_ACCESSEN, REG_ACCESSEN_CLOSE);
	aw_sar_i2c_write(i2c, AW_REG_MCFG, AW_CPU_RUN);

	aw_sar_delay_ms(SEND_STOP_CMD_DELAY_MS);

	//Read interrupt information, wait 10ms
	ret = aw_read_init_comp_irq(i2c);
	if (ret != AW_FW_OK) {
		AWLOGE_FW("stop read_ack_irq err");
		return -AW_FW_ERR;
	}

	return AW_FW_OK;
}

static int32_t aw_reg_read_flash_val(AW_SAR_I2C_TYPE i2c, uint32_t *read_data, uint16_t start_addr)
{
	int32_t ret = 0;

	//1.config FMC reg
	ret = aw_sar_i2c_write(i2c, REG_ACCESSEN, REG_OPEN_APB_ACCESS_EN);
	if (ret != AW_FW_OK) {
		AWLOGE_FW("write REG_ACCESSEN failed!");
		return -AW_FW_ERR;
	}
	//2.config PMU_CFG reg
	ret = aw_sar_i2c_write(i2c, REG_PMU_CFG, REG_ENSET_PMU_CFG);
	if (ret != AW_FW_OK) {
		AWLOGE_FW("config PMU_CFG failed");
		return -AW_FW_ERR;
	}

	ret = aw_reg_read_val(i2c, read_data, start_addr);
	if (ret != AW_FW_OK) {
			AWLOGD_FW("read addr:0x%x failed", start_addr);
			return -AW_FW_ERR;
	}

	return AW_FW_OK;
}


#ifdef AW_SAR_BIN_TR
static void aw_swap(uint8_t *a, uint8_t *b)
{
	uint8_t tmp = 0;

	tmp = *a;
	*a = *b;
	*b = tmp;
}

static void aw_bin_conver(uint8_t *data, uint32_t len)
{
	uint32_t i = 0;

	for (i = 0; i < len; i += WORD_LEN) {
		aw_swap(&(data[i + 0]), &(data[i + 3]));
		aw_swap(&(data[i + 1]), &(data[i + 2]));
	}
}
#endif

static int32_t aw_cycle_write_firmware(AW_SAR_I2C_TYPE i2c,
						const uint8_t *fw_data, uint32_t firmware_len,
						uint32_t flash_addr, uint32_t single_wr_len)
{
	uint8_t value_head_len = TRANSFER_SEQ_LEN + TRANSFER_DTS_ADDR_LEN;
	uint32_t seq = 1;
	int32_t ret = -AW_FW_ERR;
	uint32_t i = 0;
	uint32_t start_addr = 0;
	uint32_t word_comp_len = 0;
	uint32_t cycle_cnt = 0;
	uint32_t cycle_cnt_last_len = 0;
	uint8_t *firmware_info = NULL;

	AWLOGD_FW("enter single_wr_len = %d", single_wr_len);

	//If firmware len is not a multiple of 4,
	//let it be equal to the multiple of 4 over there,
	//and the less part is supplemented by 1
	if (firmware_len % WORD_LEN != 0) {
		word_comp_len = WORD_LEN - firmware_len % WORD_LEN;
		AWLOGE_FW("word_comp_len = %d", word_comp_len);
	}

	firmware_len = firmware_len + word_comp_len;
	cycle_cnt = firmware_len / single_wr_len;
	cycle_cnt_last_len = firmware_len % single_wr_len;

	for (i = 0; i < cycle_cnt; i++) {
		firmware_info = (uint8_t *)malloc(single_wr_len + value_head_len);
		if (firmware_info == NULL) {
			AWLOGE_FW("malloc application failed");
			return -AW_FW_ERR;
		}

		//Insufficient word makes up for 0xff
		memset(firmware_info, 1, single_wr_len + value_head_len);

		firmware_info[0] = (uint8_t)(seq >> OFFSET_BIT_24);
		firmware_info[1] = (uint8_t)(seq >> OFFSET_BIT_16);
		firmware_info[2] = (uint8_t)(seq >> OFFSET_BIT_8);
		firmware_info[3] = (uint8_t)(seq);

		firmware_info[4] = (uint8_t)(flash_addr >> OFFSET_BIT_24);
		firmware_info[5] = (uint8_t)(flash_addr >> OFFSET_BIT_16);
		firmware_info[6] = (uint8_t)(flash_addr >> OFFSET_BIT_8);
		firmware_info[7] = (uint8_t)(flash_addr);

		AWLOGD_FW("cnt = %d firmware addr written to flash 0x%02x 0x%02x 0x%02x 0x%02x",
				i, firmware_info[4], firmware_info[5],
				   firmware_info[6], firmware_info[7]);

		memcpy(firmware_info + value_head_len,
				&(fw_data[start_addr + single_wr_len * i]), single_wr_len);
#ifdef AW_SAR_BIN_TR
		aw_bin_conver(firmware_info + value_head_len, single_wr_len);
#endif
		ret = aw_send_once_cmd(i2c, UPDATE_MODULE, UPDATE_TRANSFER_CMD,
					firmware_info, single_wr_len + value_head_len);
		if (ret != AW_FW_OK) {
			AWLOGE_FW("aw_send_once_cmd failed");
			goto err_hand;
		}

		flash_addr += single_wr_len;
		seq++;

		if (firmware_info != NULL) {
			free(firmware_info);
			firmware_info = NULL;
		}
	}

	if (cycle_cnt_last_len != 0) {
		firmware_info = (uint8_t *)malloc(cycle_cnt_last_len + value_head_len);
		if (firmware_info == NULL) {
			AWLOGE_FW("malloc application failed");
			goto err_hand;
		}
		//Insufficient word makes up for 0xff
		memset(firmware_info, 1, cycle_cnt_last_len + value_head_len);

		firmware_info[0] = (uint8_t)(seq >> OFFSET_BIT_24);
		firmware_info[1] = (uint8_t)(seq >> OFFSET_BIT_16);
		firmware_info[2] = (uint8_t)(seq >> OFFSET_BIT_8);
		firmware_info[3] = (uint8_t)(seq);
		firmware_info[4] = (uint8_t)(flash_addr >> OFFSET_BIT_24);
		firmware_info[5] = (uint8_t)(flash_addr >> OFFSET_BIT_16);
		firmware_info[6] = (uint8_t)(flash_addr >> OFFSET_BIT_8);
		firmware_info[7] = (uint8_t)(flash_addr);

		AWLOGD_FW("last transfer, len = %d ,firmware addr written to flash 0x%02x 0x%02x 0x%02x 0x%02x",
				cycle_cnt_last_len, firmware_info[4], firmware_info[5],
				 firmware_info[6], firmware_info[7]);

		memcpy(firmware_info + value_head_len,
				&(fw_data[start_addr + cycle_cnt * single_wr_len]),
				cycle_cnt_last_len);
#ifdef AW_SAR_BIN_TR
		
		aw_bin_conver(firmware_info + value_head_len, cycle_cnt_last_len);
#endif

		ret = aw_send_once_cmd(i2c, UPDATE_MODULE, UPDATE_TRANSFER_CMD,
					firmware_info, cycle_cnt_last_len + value_head_len);
		if (ret != AW_FW_OK) {
			AWLOGD_FW("aw_send_once_cmd fail");
			goto err_hand;
		}
		if (firmware_info != NULL) {
			free(firmware_info);
			firmware_info = NULL;
		}
	}

	return AW_FW_OK;
err_hand:
	if (firmware_info != NULL) {
		free(firmware_info);
		firmware_info = NULL;
	}
	return -AW_FW_ERR;
}

static int32_t aw_write_firmware_checksum(AW_SAR_I2C_TYPE i2c, uint8_t *p_checksum,
								uint32_t fw_check_en_addr,
								uint32_t check_en_val,
								uint32_t fw_check_code_addr)
{
	uint8_t i = 0;
	uint8_t j = 0;
	uint8_t transfer_value_w[AW_EN_TR_CHECK_VALUE_LEN] = { 0 };
	int32_t ret = 0;
	uint32_t r_check_en = 0;
	uint32_t r_checksum = 0;
	uint32_t w_checksum = 0;

	for (i = 0; i < 4; i++) {
		transfer_value_w[i] = (uint8_t)(0 >> ((3 - i) * 8));
	}

	for (i = 4; i < 8; i++) {
		transfer_value_w[i] = (uint8_t)(fw_check_en_addr >> ((3 - j) * 8));
		j++;
	}

	j = 0;
	for (i = 8; i < 12; i++) {
		transfer_value_w[i] = (uint8_t)(check_en_val >> ((3 - j) * 8));
		j++;
	}

	j = 0;
	for (i = 12; i < 16; i++) {
		transfer_value_w[i] = p_checksum[j];
		j++;
	}

	AWLOGD_FW("fw_check_en_addr:0x%8x, check_en_val:0x%8x", fw_check_en_addr, check_en_val);

	ret =  aw_send_once_cmd(i2c, UPDATE_MODULE, UPDATE_TRANSFER_CMD,
						transfer_value_w, AW_EN_TR_CHECK_VALUE_LEN);
		if (ret != AW_FW_OK) {
		AWLOGD_FW("aw_write_firmware_checksum err");
		return -AW_FW_ERR;
	}
	ret = aw_reg_read_flash_val(i2c, &r_check_en, (uint16_t)fw_check_en_addr);
	if (ret != AW_FW_OK) {
		AWLOGD_FW("aw_read check_en err");
		return -AW_FW_ERR;
	}
	ret = aw_reg_read_flash_val(i2c, &r_checksum, (uint16_t)fw_check_code_addr);
	if (ret != AW_FW_OK) {
		AWLOGD_FW("aw_read check_en err");
		return -AW_FW_ERR;
	}
	w_checksum = AW_SAR_GET_32_DATA(p_checksum[0], p_checksum[1], p_checksum[2], p_checksum[3]);
	if ((r_check_en == check_en_val) && (r_checksum == w_checksum)) {
		return AW_FW_OK;
	} else {
		AWLOGE_FW("r_check_en:0x%08x, check_en_val:0x%08x, r_checksum:0x%08x, w_checksum:0x%08x",
				r_check_en, check_en_val, r_checksum, w_checksum);
		return -AW_FW_ERR;
	}

	return AW_FW_OK;
}

static int32_t aw_get_fw_and_bt_info(AW_SAR_I2C_TYPE i2c,
				const uint8_t *fw_data, uint32_t fw_len,
				uint8_t *p_fw_check_sum, uint32_t check_len)
{
	uint32_t fw_checksum = 0;
	int32_t ret = 0;
	uint32_t bt_version = 0;
	uint32_t bt_date = 0;
	uint32_t bt_checksum = 0;

	ret = aw_sar_i2c_read(i2c, AW_BT_VER_INF_VERSION, &bt_version);
	if (ret != AW_FW_OK) {
		AWLOGE_FW("read AW_BT_VER_INF_VERSION  fail!");
		return -AW_FW_ERR;
	}

	ret = aw_sar_i2c_read(i2c, AW_BT_VER_INF_DATE, &bt_date);
	if (ret != AW_FW_OK) {
		AWLOGE_FW("read AW_BT_VER_INF_DATE fail");
		return -AW_FW_ERR;
	}

	ret = aw_reg_read_flash_val(i2c, &bt_checksum, AW_BT_CHECK_SUM_ADDR);
	if (ret != AW_FW_OK) {
		AWLOGE_FW("read bt_checksum fail");
		return -AW_FW_ERR;
	}
	AWLOGD_FW("boot version:0x%08x, date:0x%08x checksum:0x%08x", bt_version, bt_date, bt_checksum);

	fw_checksum = aw_get_bin_checksum(fw_data, fw_len, check_len);
	p_fw_check_sum[0] = (uint8_t)(fw_checksum >> 24);
	p_fw_check_sum[1] = (uint8_t)(fw_checksum >> 16);
	p_fw_check_sum[2] = (uint8_t)(fw_checksum >> 8);
	p_fw_check_sum[3] = (uint8_t)(fw_checksum >> 0);

	AWLOGD_FW("firmware checksum is 0x%08x", fw_checksum);

	return AW_FW_OK;
}

static int32_t aw_flash_erase_sector(AW_SAR_I2C_TYPE i2c, uint32_t erase_addr, uint32_t erase_sector_cnt, uint32_t sector_size)
{
	uint8_t i = 0;
	uint8_t addr_buf[4] = { 0 };
	int32_t ret = 0;

	for (i = 0; i < erase_sector_cnt; i++) {
		addr_buf[0] = (uint8_t)(erase_addr >> OFFSET_BIT_24);
		addr_buf[1] = (uint8_t)(erase_addr >> OFFSET_BIT_16);
		addr_buf[2] = (uint8_t)(erase_addr >> OFFSET_BIT_8);
		addr_buf[3] = (uint8_t)(erase_addr);
		g_prot_update_fw_flag = SEND_ERASE_SECTOR_CMD;
		ret = aw_send_once_cmd(i2c, FLASH_MODULE, FLASH_ERASE_SECTOR_CMD,
							addr_buf, sizeof(addr_buf));
		if (ret != AW_FW_OK) {
			AWLOGE_FW("send erase sector once cmd i = %d", i);
			break;
		}
		erase_addr += sector_size;
	}
	AWLOGD_FW("erase sector success!");

	return ret;
}

static int32_t aw_flash_erase_last_sector(AW_SAR_I2C_TYPE i2c, uint32_t erase_addr)
{
	uint8_t addr_buf[4] = { 0 };
	int32_t ret = 0;

	addr_buf[0] = (uint8_t)(erase_addr >> OFFSET_BIT_24);
	addr_buf[1] = (uint8_t)(erase_addr >> OFFSET_BIT_16);
	addr_buf[2] = (uint8_t)(erase_addr >> OFFSET_BIT_8);
	addr_buf[3] = (uint8_t)(erase_addr);
	g_prot_update_fw_flag = SEND_ERASE_SECTOR_CMD;
	ret = aw_send_once_cmd(i2c, FLASH_MODULE, FLASH_ERASE_SECTOR_CMD,
							addr_buf, sizeof(addr_buf));
	if (ret != AW_FW_OK) {
		AWLOGE_FW("flash_erase_last_sector fail");
		return -AW_FW_ERR;
	}
	AWLOGD_FW("erase last sector addr = 0x%x", erase_addr);

	return AW_FW_OK;
}

static int32_t aw_send_online_cmd(AW_SAR_I2C_TYPE i2c)
{
	int32_t ret = 0;
	int32_t cnt = 200;
	uint32_t irq_stat = 0;

	ret = aw_sar_i2c_write(i2c, AW_BT_HOST2CPU_TRIG, AW_BT_HOST2CPU_TRIG_ONLINE_UPGRADE_CMD);
	if (ret != AW_FW_OK) {
		AWLOGE_FW("write AW_BT_HOST2CPU_TRIG failed");
		return -AW_FW_ERR;
	}

	aw_sar_delay_ms(1);

	do {
		ret = aw_sar_i2c_read(i2c, REG_IRQSRC, &irq_stat);
		if (ret != AW_FW_OK) {
			AWLOGE_FW("read REG_IRQSRC fail");
			return -AW_FW_ERR;
		}
		if (((irq_stat >> 29) & 0x01) == 1) {
			break;
		}
		aw_sar_delay_ms(1);
	} while (cnt--);

	if (cnt == -1) {
		AWLOGE_FW("read irqsrc failed!, REG_IRQSRC val: 0x%x", irq_stat);
		return -AW_FW_ERR;
	}

	return AW_FW_OK;
}

static int32_t aw_send_all_update_cmd(AW_SAR_I2C_TYPE i2c, struct aw_update_fw *update_fw)
{
	int8_t update_flag = AW_TRUE;
	int32_t ret = -AW_FW_ERR;
	uint32_t data_tmp = 0;
	uint8_t fw_check_sum[4] = { 0 };
	uint32_t reg_boot_loader_active_val = 0;
	const uint8_t *fw_data = update_fw->fw_data_info->data_start_offset;
	uint32_t fw_data_len = update_fw->fw_data_info->update_data_len;
	const struct aw_flash_update_info *flash_update_info = update_fw->fw_info->flash_info;
	uint8_t direct_update_flag = update_fw->update_para->direct_update_flag;
	uint32_t fw_check_len = flash_update_info->check_len;

	do {
		//1.Send online upgrade command
		AWLOGD_FW("1.Send online upgrade command");
		ret = aw_send_online_cmd(i2c);
		if (ret != AW_FW_OK) {
			AWLOGE_FW("1.Send online upgrade command failed!");
			update_flag = AW_FALSE;
			break;
		}

		ret = aw_get_fw_and_bt_info(i2c, fw_data, fw_data_len, fw_check_sum, fw_check_len);
		if (ret != AW_FW_OK) {
			AWLOGE_FW("aw_get_fw_and_bt_info failed!");
			update_flag = AW_FALSE;
			break;
		}

		//2.send start cmd
		AWLOGD_FW("2.send start cmd");
		g_prot_update_fw_flag = SEND_START_CMD;
		ret = aw_send_once_cmd(i2c, UPDATE_MODULE, UPDATE_START_CMD,
				P_AW_START_CMD_SEND_VALUE, AW_START_CMD_SEND_VALUE_LEN);
		if (ret != AW_FW_OK) {
			AWLOGE_FW("send start cmd failed!");
			update_flag = AW_FALSE;
			break;
		}

		//3.a en fw check erase_last_sector"
		AWLOGD_FW("3.a en fw check erase_last_sector");
		ret = aw_flash_erase_last_sector(i2c, flash_update_info->erase_start_addr +
					(flash_update_info->erase_sector_cnt - 1) * flash_update_info->sector_size);
		if (ret != AW_FW_OK) {
			AWLOGD_FW("send erase_last_sector failed!");
			update_flag = AW_FALSE;
			break;
		}

		//3.b en fw check
		AWLOGD_FW("3.b en fw check");
		g_prot_update_fw_flag = SEND_UPDATE_CHECK_CODE_CMD;
		ret = aw_write_firmware_checksum(i2c, fw_check_sum, flash_update_info->check_en_addr,
								flash_update_info->check_en_val,
								flash_update_info->check_code_addr);
		if (ret != AW_FW_OK) {
			AWLOGD_FW("aw_write_firmware_checksum err");
				update_flag = AW_FALSE;
				break;
		}

		//4.send Erase flash firmware area cmd, except for the last sector
		AWLOGD_FW("4.send Erase Chip Cmd");
		ret = aw_flash_erase_sector(i2c, flash_update_info->erase_start_addr,
								   (flash_update_info->erase_sector_cnt - 1),
									flash_update_info->sector_size);
		if (ret != AW_FW_OK) {
			AWLOGD_FW("send Erase flash firmware area cmd failed!");
			update_flag = AW_FALSE;
			break;
		}
		//5.Cycle write firmware
		AWLOGD_FW("5. Cycle write firmware");
		g_prot_update_fw_flag = SEND_UPDATE_FW_CMD;
		ret = aw_cycle_write_firmware(i2c, fw_data, fw_data_len, flash_update_info->tr_start_addr,
										update_fw->fw_info->single_wr_len);
		if (ret != AW_FW_OK) {
			AWLOGD_FW("Cycle write firmware");
			update_flag = AW_FALSE;
			break;
		}
		//6.send stop cmd
		AWLOGD_FW("6.send stop cmd");
		g_prot_update_fw_flag = SEND_STOP_CMD;
		ret = aw_send_stop_cmd(i2c);
		if (ret != AW_FW_OK) {
			AWLOGD_FW("stop cmd failed, protocol update firmware failed");
			update_flag = AW_FALSE;
			break;
		} else {
			AWLOGI_FW("protocol update firmware succeeded!");
		}
	} while(0);

	if (update_flag == AW_FALSE) {
		AWLOGD_FW("protocol update firmware failed, recovery reg");
		ret = aw_sar_i2c_write(i2c, REG_ACCESSEN, REG_ACCESSEN_OPEN);
		if (ret != AW_FW_OK) {
			AWLOGE_FW("write REG_ACCESSEN failed!");
			return -AW_FW_ERR;
		}

		ret = aw_sar_i2c_read(i2c, REG_BOOT_LOADER_ACTIVE, &reg_boot_loader_active_val);
		if (ret != AW_FW_OK) {
			AWLOGE_FW("read REG_BOOT_LOADER_ACTIVE failed!");
			return -AW_FW_ERR;
		}
		if (reg_boot_loader_active_val != 0) {
			ret = aw_sar_i2c_write(i2c, REG_BOOT_LOADER_ACTIVE, 0);
				if (ret != AW_FW_OK) {
				AWLOGE_FW("write REG_BOOT_LOADER_ACTIVE failed!");
				return -AW_FW_ERR;
			}
		}

		ret = aw_sar_i2c_read(i2c, REG_UPDATA_DIS, &data_tmp);
		if (ret != AW_FW_OK) {
			AWLOGE_FW("read REG_UPDATA_DIS failed!");
			return -AW_FW_ERR;
		}
		if (((data_tmp >> 24) & 0xff) != 0) {
			ret = aw_sar_i2c_write(i2c, REG_UPDATA_DIS, data_tmp & AW_REG_UPDATA_DIS_MASK);
			if (ret != AW_FW_OK) {
				AWLOGE_FW("write REG_UPDATA_DIS failed!");
				return -AW_FW_ERR;
			}
		}

		ret = aw_sar_i2c_write(i2c, REG_ACCESSEN, REG_ACCESSEN_CLOSE);
		if (ret != AW_FW_OK) {
			AWLOGE_FW("write REG_ACCESSEN failed!");
			return -AW_FW_ERR;
		}
		if (direct_update_flag == AW_TRUE) {
			AWLOGI_FW("protocol update firmware failed!, exit load");
			return -AW_FW_ERR;
		}
		AWLOGI_FW("protocol update firmware failed, start reg mode upgrade");
		return -AW_PROT_UPDATE_ERR;
	}

	return AW_FW_OK;
}

static int32_t aw_fw_version_cmp(AW_SAR_I2C_TYPE i2c, int8_t *cmp_val, uint32_t flash_app_version, uint16_t fwver_reg)
{
	uint32_t firmware_version = 0;
	int32_t ret = -AW_FW_ERR;

	ret = aw_sar_i2c_read(i2c, fwver_reg, &firmware_version);
	if (ret < 0) {
		AWLOGD_FW("read fwver_reg failed!");
		return -AW_FW_ERR;
	}
	AWLOGI_FW("chip_firmware version :0x%08x bin_irmware version :0x%08x!",
			firmware_version, flash_app_version);

	if (flash_app_version != firmware_version) {
		*cmp_val = BIN_VER_NOT_EQUALS_SOC_VER;
	} else {
		*cmp_val = BIN_VER_EQUALS_SOC_VER;
	}

	return AW_FW_OK;
}

static int32_t aw_prox_update_para_check(struct aw_update_fw *update_fw)
{
	if ((update_fw == NULL) ||
		(update_fw->fw_info == NULL) ||
		(update_fw->update_para == NULL) ||
		(update_fw->fw_data_info == NULL)) {
		AWLOGE_FW("para1 is NULL");
		return -AW_FW_ERR;
	}

	if (update_fw->fw_info->flash_info == NULL) {
		AWLOGE_FW("para3 is NULL");
		return -AW_FW_ERR;
	}
	if (update_fw->update_para->aw_update_diff == NULL) {
		AWLOGE_FW("para4 is NULL");
		return -AW_FW_ERR;
	}
	if (update_fw->fw_data_info->data_start_offset == NULL) {
		AWLOGE_FW("para5 is NULL");
		return -AW_FW_ERR;
	}

	return AW_FW_OK;
}

int32_t aw_prot_update(AW_SAR_I2C_TYPE i2c, struct aw_update_fw *update_fw)
{
	int8_t cmp_val = 0;
	int32_t ret = 0;

	ret = aw_prox_update_para_check(update_fw);
	if (ret != AW_FW_OK) {
		AWLOGE_FW("para check failed!");
		return -AW_FW_ERR;
	}

	if (update_fw->update_para->direct_update_flag == AW_FALSE) {
		aw_fw_version_cmp(i2c, &cmp_val, update_fw->fw_data_info->fw_version,
								update_fw->update_para->aw_update_diff->fwver_reg_addr);
		if (cmp_val == BIN_VER_EQUALS_SOC_VER) {
			AWLOGI_FW("Same version number, no update!");
			return AW_FW_OK;
		} else {
			AWLOGD_FW("firmware Version number is different, update!");
		}
	}

	return aw_send_all_update_cmd(i2c, update_fw);
}

/********aw_reg_mode_update start********/

static int32_t aw_close_write_flash_protect(AW_SAR_I2C_TYPE i2c, uint32_t flash_addr_flag)
{
	int32_t ret = 0;

	//Open host read/write FMC protection
	ret = aw_sar_i2c_write(i2c, REG_ACCESSEN, REG_OPEN_APB_ACCESS_EN);
	if (ret != AW_FW_OK) {
		AWLOGE_FW("write REG_ACCESSEN failed");
		return -AW_FW_ERR;
	}

	//Configure PMC_ CFG register
	ret = aw_sar_i2c_write(i2c, REG_PMU_CFG, REG_SET_PMU_CFG);
	if (ret != AW_FW_OK) {
		AWLOGE_FW("write REG_PMU_CFG failed!");
		return -AW_FW_ERR;
	}

	//Turn on flash write protection
	if (flash_addr_flag == BOOT_UPDATE) {
		ret = aw_sar_i2c_write(i2c, REG_ARRAY2_EW_EN, REG_SET_BTROM_EW_EN);
		if (ret != AW_FW_OK) {
			AWLOGE_FW("write REG_ARRAY2_EW_EN failed");
			return -AW_FW_ERR;
		}
	}

	return AW_FW_OK;
}

static int32_t aw_reg_write_to_flash_once(AW_SAR_I2C_TYPE i2c, uint16_t addr, uint32_t w_data)
{
	int32_t ret = 0;

	//Write access address
	ret = aw_sar_i2c_write(i2c, REG_ISPADR, addr);
	if (ret != AW_FW_OK) {
		AWLOGE_FW("write 0xff20 err");
		return -AW_FW_ERR;
	}

	//Write data
	ret = aw_sar_i2c_write(i2c, REG_ISPWDATA, w_data);
	if (ret != AW_FW_OK) {
		AWLOGE_FW("write 0xff20 err");
		return -AW_FW_ERR;
	}

	//Configure ISP_CMD reg
	ret = aw_sar_i2c_write(i2c, REG_ISPCMD, REG_ISP_CMD_CONFIG);
	if (ret != AW_FW_OK) {
		AWLOGE_FW("write 0xff20 err");
		return -AW_FW_ERR;
	}

	//Configure ISP_GO reg
	ret = aw_sar_i2c_write(i2c, REG_ISPGO, REG_SET_ISP_GO);
	if (ret != AW_FW_OK) {
		AWLOGE_FW("write 0xff20 err");
		return -AW_FW_ERR;
	}

	ret = aw_check_isp_go_reg(i2c);
	if (ret != AW_FW_OK) {
		AWLOGE_FW("check_isp_go_reg err");
		return -AW_FW_ERR;
	}

	return AW_FW_OK;
}


static int32_t aw_reg_write_val_to_flash(AW_SAR_I2C_TYPE i2c, uint8_t update_flag, uint16_t addr, uint32_t val)
{
	int32_t ret = 0;

	AWLOGD_FW("enter");

	ret = aw_close_write_flash_protect(i2c, update_flag);
	if (ret != AW_FW_OK) {
		AWLOGE_FW("close_write_flash_protect failed!");
		return -AW_FW_ERR;
	}

	ret = aw_reg_write_to_flash_once(i2c, addr, val);
	if (ret != AW_FW_OK) {
		AWLOGE_FW("write aw_reg_write_bin_once failed!");
		return -AW_FW_ERR;
	}

	//Configure PMU_ CFG register
	ret = aw_sar_i2c_write(i2c, REG_PMU_CFG, REG_ENSET_PMU_CFG);
	if (ret != AW_FW_OK) {
		AWLOGE_FW("write REG_PMU_CFG failed!");
		return -AW_FW_ERR;
	}

	return AW_FW_OK;
}

static int32_t aw_rd_or_wi_cmp(AW_SAR_I2C_TYPE i2c, struct aw_update_common *update_info,
				const uint8_t *w_bin_offset, uint32_t update_data_len)
{
	int32_t ret = 0;
	uint32_t i = 0;
	uint32_t r_data = 0;
	uint32_t w_data = 0;
	uint32_t read_cnt = update_info->update_data_len;

	AWLOGD_FW("enter");

	//1.config FMC reg
	ret = aw_sar_i2c_write(i2c, REG_ACCESSEN, REG_OPEN_APB_ACCESS_EN);
	if (ret != AW_FW_OK) {
		AWLOGE_FW("write REG_ACCESSEN failed!");
		return -AW_FW_ERR;
	}
	//2.config PMU_CFG reg
	ret = aw_sar_i2c_write(i2c, REG_PMU_CFG, REG_ENSET_PMU_CFG);
	if (ret != AW_FW_OK) {
		AWLOGE_FW("config PMU_CFG reg failed");
		return -AW_FW_ERR;
	}

	AWLOGE_FW("read_cnt = %d", read_cnt);
	for (i = 0; i < read_cnt; i += WORD_LEN) {
		ret = aw_reg_read_val(i2c, &r_data,
					update_info->flash_info->tr_start_addr + i);
		if (ret != AW_FW_OK) {
			AWLOGE_FW("reg_read_bin failed");
			return -AW_FW_ERR;
		}
#ifdef AW_SAR_BIN_TR
		w_data = AW_SAR_GET_32_DATA(w_bin_offset[i + 3],
					w_bin_offset[i + 2],
					w_bin_offset[i + 1],
					w_bin_offset[i + 0]);

#else
		w_data = AW_SAR_GET_32_DATA(w_bin_offset[i + 0],
					w_bin_offset[i + 1],
					w_bin_offset[i + 2],
					w_bin_offset[i + 3]);

#endif
#ifdef AW_FW_INFO_LOG
		AWLOGD_FW("i= %d, addr= 0x%08x, W_DATA= 0x%08x, R_DATA= 0x%08x",
			i, update_info->flash_info->tr_start_addr + i, w_data, r_data);
#endif
		if (w_data != r_data) {
			AWLOGE_FW("read and write check failed! i=%d, w_data=0x%08x, r_data=0x%08x",
					i, w_data, r_data);
			return -AW_FW_ERR;
		}
	}

	return AW_FW_OK;
}

static int32_t aw_reg_write_bin_to_flash(AW_SAR_I2C_TYPE i2c, struct aw_update_common *update_info)
{
	int32_t ret = 0;
	uint32_t i = 0;
	uint32_t w_data = 0;
	uint32_t index = 0;
	const uint8_t *p_data = update_info->w_bin_offset;
	uint32_t len = update_info->update_data_len;
	uint16_t flash_addr = update_info->flash_info->tr_start_addr;

	AWLOGD_FW("enter");

	ret = aw_close_write_flash_protect(i2c, update_info->update_flag);
	if (ret != AW_FW_OK) {
		AWLOGE_FW("close_write_flash_protect failed!");
		return -AW_FW_ERR;

	}
	for (i = 0; i < len; i += WORD_LEN, index += WORD_LEN) {
#ifdef AW_SAR_BIN_TR
		w_data = AW_SAR_GET_32_DATA(p_data[index + 3], p_data[index + 2],
				p_data[index + 1], p_data[index + 0]);
#else
		w_data = AW_SAR_GET_32_DATA(p_data[index + 0], p_data[index + 1],
				p_data[index + 2], p_data[index + 3]);
#endif
		ret = aw_reg_write_to_flash_once(i2c, flash_addr + i, w_data);
		if (ret != AW_FW_OK) {
			AWLOGE_FW("write aw_reg_write_bin_once failed!");
			return -AW_FW_ERR;
		}
	}

	//Configure PMU_ CFG register
	ret = aw_sar_i2c_write(i2c, REG_PMU_CFG, REG_ENSET_PMU_CFG);
	if (ret != AW_FW_OK) {
		AWLOGE_FW("write REG_PMU_CFG failed!");
		return -AW_FW_ERR;
	}

	return AW_FW_OK;
}

static int32_t aw_erase_sector(AW_SAR_I2C_TYPE i2c, struct aw_update_common *update_info)
{
	int32_t ret = 0;
	uint32_t i = 0;
	uint32_t erase_len = update_info->flash_info->check_len;
	uint32_t erase_cnt = update_info->flash_info->erase_sector_cnt;
	uint32_t erase_addr = update_info->flash_info->erase_start_addr;

	AWLOGD_FW("erase_addr = 0x%08x, erase_len = %d, erase_cnt = %d", erase_addr, erase_len, erase_cnt);
	erase_len = erase_len * 1;

	//1.close write protect
	AWLOGD_FW("1.open host write FMC protect");
	ret = aw_sar_i2c_write(i2c, REG_ACCESSEN, REG_OPEN_APB_ACCESS_EN);
	if (ret != AW_FW_OK) {
		AWLOGE_FW("write REG_ACCESSEN err");
		return ret;
	}

	ret = aw_sar_i2c_write(i2c, REG_PMU_CFG, REG_SET_PMU_CFG);
	if (ret != AW_FW_OK) {
		AWLOGE_FW("write REG_PMU_CFG err");
		return ret;
	}

	if (update_info->update_flag == BOOT_UPDATE) {
		ret = aw_sar_i2c_write(i2c, REG_ARRAY2_EW_EN, REG_SET_BTROM_EW_EN);
		if (ret != AW_FW_OK) {
			AWLOGE_FW("write REG_ARRAY2_EW_EN failed!");
			return ret;
		}
	}

	for (i = 0; i < erase_cnt; i++)	{
		//2.Erase one sector at a time
		ret = aw_sar_i2c_write(i2c, REG_ISPADR, erase_addr + i * update_info->flash_info->sector_size);
		if (ret != AW_FW_OK) {
			AWLOGE_FW("write REG_ISPADR failed!");
			return ret;
		}

		ret = aw_sar_i2c_write(i2c, REG_ISPCMD, REG_ACCESS_MAIN_ARR);
		if (ret != AW_FW_OK) {
			AWLOGE_FW("write REG_ISPCMD failed!");
			return ret;
		}

		ret = aw_sar_i2c_write(i2c, REG_T_RCV, REG_SET_T_RCV);
		if (ret != AW_FW_OK) {
			AWLOGE_FW("write REG_T_RCV failed!");
			return ret;
		}

		ret = aw_sar_i2c_write(i2c, REG_ISPGO, REG_SET_ISP_GO);
		if (ret != AW_FW_OK) {
			AWLOGE_FW("write REG_ISPGO failed!");
			return ret;
		}

		ret = aw_check_isp_go_reg(i2c);
		if (ret != AW_FW_OK) {
			AWLOGE_FW("check_isp_go_reg failed!");
			return ret;
		}
	}
	ret = aw_sar_i2c_write(i2c, REG_T_RCV, REG_SET_T_RCV_EN);
	if (ret != AW_FW_OK) {
		AWLOGE_FW("write REG_T_RCV failed!");
		return ret;
	}

	ret = aw_sar_i2c_write(i2c, REG_PMU_CFG, REG_ENSET_PMU_CFG);
	if (ret != AW_FW_OK) {
		AWLOGE_FW("write REG_PMU_CFG failed!");
		return ret;
	}

	return AW_FW_OK;
}

static int32_t aw_flash_update(AW_SAR_I2C_TYPE i2c, struct aw_update_common *update_info)
{
	int32_t ret = 0;
	uint32_t check_sum = 0;

	AWLOGD_FW("enter read_len = %d", update_info->update_data_len);

	//1.open register access enable
	AWLOGE_FW("1.opne register access enable");
	ret = aw_sar_i2c_write(i2c, REG_ACCESSEN, REG_OPEN_REG_ACCESS_EN);
	if (ret != AW_FW_OK) {
		AWLOGE_FW("write REG_ACCESSEN failed!");
		return ret;
	}
	ret = aw_sar_i2c_write(i2c, REG_MCFG, REG_OPEN_MCFG_EN);
	if (ret != AW_FW_OK) {
		AWLOGE_FW("write REG_MCFG failed!");
		return ret;
	}
	ret = aw_sar_i2c_write(i2c, REG_PMU_CFG, REG_SET_PMU_CFG);
	if (ret != AW_FW_OK) {
		AWLOGE_FW("write REG_PMU_CFG failed!");
		return ret;
	}
	if (update_info->update_flag == FIRMWARE_UPDATE) {
		ret = aw_sar_i2c_write(i2c, REG_ARRAY2_EW_EN, REG_SET_BTROM_EW_EN);
		if (ret != AW_FW_OK) {
			AWLOGE_FW("write REG_ARRAY2_EW_EN failed!");
			return ret;
		}
	}

	//2.Erase sector
	AWLOGD_FW("2.Erase sector, flash_erase_start_addr:0x%08x, erase_len:0x%08x",
		update_info->flash_info->erase_start_addr, update_info->flash_info->check_len);
	ret = aw_erase_sector(i2c, update_info);
	if (ret != AW_FW_OK) {
		AWLOGE_FW("2. Erase sector failed!");
		return ret;
	}

	//3.write flash
	AWLOGD_FW("3.write flash");
	ret = aw_reg_write_bin_to_flash(i2c, update_info);
	if (ret != AW_FW_OK) {
		AWLOGE_FW("3. write flash failed!");
		return ret;
	}

	//4.read data check
	AWLOGD_FW("4.read data check");
	ret = aw_rd_or_wi_cmp(i2c, update_info,
				   update_info->w_bin_offset,
				   update_info->update_data_len);
	if (ret != AW_FW_OK) {
		AWLOGE_FW("4. read data check failed!");
		return -AW_FW_ERR;
	}

	//5.write checksum
	AWLOGD_FW("5.write checksum");
	//Turn on verification enable
	ret = aw_reg_write_val_to_flash(i2c, update_info->update_flag,
							update_info->flash_info->check_en_addr,
							update_info->flash_info->check_en_val);
	if (ret != AW_FW_OK) {
		AWLOGE_FW("5.write check enable failed!");
		return -AW_FW_ERR;
	}

	check_sum = aw_get_bin_checksum(update_info->w_bin_offset,
							update_info->update_data_len,
							update_info->flash_info->check_len);
	AWLOGI_FW("check_sum = 0x%x", check_sum);
	ret = aw_reg_write_val_to_flash(i2c, update_info->update_flag,
							update_info->flash_info->check_code_addr,
							check_sum);
	if (ret != AW_FW_OK) {
		AWLOGE_FW("5.write checksum failed!");
		return -AW_FW_ERR;
	}

	//6.open flash protect
	AWLOGD_FW("6.open flash protect");
	if (update_info->update_flag == FIRMWARE_UPDATE) {
		ret = aw_sar_i2c_write(i2c, REG_ARRAY2_EW_EN, REG_ENSET_BTROM_EW_EN);
		if (ret != AW_FW_OK) {
			AWLOGE_FW("6.open flash protect failed");
			return ret;
		}
	}

	AWLOGI_FW("%s reg mode update succeeded!!!",
		((update_info->update_flag == BOOT_UPDATE)? "boot":"firmware"));

	return AW_FW_OK;
}

int32_t aw_reg_update_para_check(struct aw_update_common *update_info)
{
	if ((update_info == NULL) || (update_info->flash_info == NULL)) {
		return -AW_FW_ERR;
	}

	if (update_info->w_bin_offset == NULL) {
		return -AW_FW_ERR;
	}

	return AW_FW_OK;
}

int32_t aw_reg_update(AW_SAR_I2C_TYPE i2c, struct aw_update_common *update_info)
{
	int32_t ret = 0;

	ret = aw_reg_update_para_check(update_info);
	if (ret != AW_FW_OK) {
		return -AW_FW_ERR;
	}

	return aw_flash_update(i2c, update_info);
}

#define AW9620X_STAT_BASE_ADDR			(0x1A00)
#define AW9620X_REG_FWVER			((0x000C) + AW9620X_STAT_BASE_ADDR)
#define AW9620X_REG_WST				((0x0010) + AW9620X_STAT_BASE_ADDR)

#define AW_WORD_SZIE					(4)

#define AW9620X_BT_STAET_ADDR			(0x10000000)
#define AW9620X_BT_END_ADDR			(0x100007F4)
//Check the firmware area except the check code and check enable position
#define AW9620X_BT_CHECK_LEN			(AW9620X_BT_END_ADDR - AW9620X_BT_STAET_ADDR + AW_WORD_SZIE)
#define AW9620X_BT_CHECK_EN_ADDR		(0x100007F8)
#define AW9620X_BT_CHECK_CODE_ADDR		(0x100007FC)
#define AW9620X_BT_CHECK_CODE_VAL		(0x20222022)
#define AW9620X_BT_ERASE_START_ADDR		(0x10000000)
#define AW9620X_BT_SECTOR_SIZE			(128)
#define AW9620X_BT_ERASE_CNT			((AW9620X_BT_CHECK_CODE_ADDR - AW9620X_BT_ERASE_START_ADDR + AW_WORD_SZIE) / AW9620X_BT_SECTOR_SIZE)

#define AW9620X_FW_START_ADDR			(0x10002000)
#define AW9620X_FW_END_ADDR			(0x10003FF4)
#define AW9620X_FW_CHECK_EN_ADDR		(0x10003FF8)
#define AW9620X_FW_CHECK_CODE_ADDR		(0x10003FFC)

//Check the firmware area except the check code and check enable position
#define AW9620X_FW_CHECK_LEN			(AW9620X_FW_END_ADDR - AW9620X_BT_CHECK_CODE_ADDR)

#define AW9620X_FW_CHECK_CODE_VAL		(0x20222022)
#define AW9620X_FW_ERASE_START_ADDR		(0x10002000)
#define AW9620X_FW_SECTOR_SIZE			(128)
//The number of sectors in the flash FW area is reduced by one
#define AW9620X_FW_ERASE_CNT			((AW9620X_FW_CHECK_CODE_ADDR - AW9620X_FW_ERASE_START_ADDR + AW_WORD_SZIE) / AW9620X_FW_SECTOR_SIZE)
#define AW9620X_CACHE_LEN				(0x1000)

static const struct aw_flash_update_info g_aw_prot_update_fw_info = {
	.tr_start_addr = AW9620X_FW_START_ADDR,
	.check_en_addr = AW9620X_FW_CHECK_EN_ADDR,
	.check_code_addr = AW9620X_FW_CHECK_CODE_ADDR,
	.check_en_val = AW9620X_FW_CHECK_CODE_VAL,
	.check_len = AW9620X_FW_CHECK_LEN,
	.erase_start_addr = AW9620X_FW_ERASE_START_ADDR,
	.erase_sector_cnt = AW9620X_FW_ERASE_CNT,
	.sector_size = AW9620X_FW_SECTOR_SIZE,
};

static const struct aw_update_chip_diff update_chip_diff = {
	.fwver_reg_addr = AW9620X_REG_FWVER,
	.chip_flag = CHIP_AW9620X,
};

int32_t aw9620x_prot_update_fw(AW_SAR_I2C_TYPE i2c, uint8_t direct_update_flag,
						const uint8_t *data_start_offset, uint32_t update_data_len ,uint32_t fw_version)
{
	struct aw_update_para update_para = {
		.direct_update_flag = direct_update_flag,			//Upgrade without comparing version numbers
		.aw_update_diff = &update_chip_diff,				//Differences in chip updates
	};
		//.cache_len = AW9620X_CACHE_LEN,
	struct aw_fw_data_info fw_data_info = {
		.data_start_offset = data_start_offset,
		.update_data_len = update_data_len,
		.fw_version = fw_version,
	};

	struct aw_fw_info fw_info = {
		//.single_wr_len = AW9620X_CACHE_LEN / 4,
		.single_wr_len = AW9620X_CACHE_LEN - 0x40,
		.flash_info = &g_aw_prot_update_fw_info,
	};

	struct aw_update_fw update_fw = {
		.fw_info = &fw_info,
		.update_para = &update_para,
		.fw_data_info = &fw_data_info,
	};

	return aw_prot_update(i2c, &update_fw);
}

static const struct aw_flash_update_info g_aw_reg_fw_flash_info = {
		.tr_start_addr = AW9620X_FW_START_ADDR,
		.check_len = AW9620X_FW_CHECK_LEN, //37f8
		.check_en_addr = AW9620X_FW_CHECK_EN_ADDR,
		.check_code_addr = AW9620X_FW_CHECK_CODE_ADDR,
		.check_en_val = AW9620X_FW_CHECK_CODE_VAL,

		.erase_start_addr = AW9620X_FW_ERASE_START_ADDR,
		.erase_sector_cnt = AW9620X_FW_ERASE_CNT,
		.sector_size = AW9620X_FW_SECTOR_SIZE,
	};

int32_t aw9620x_reg_update_fw(AW_SAR_I2C_TYPE i2c,
					const uint8_t *data_start_offset, uint32_t update_data_len)
{
	struct aw_update_common update_info = {
		.update_flag = FIRMWARE_UPDATE,
		.w_bin_offset = data_start_offset,
		.update_data_len = update_data_len,
		.flash_info = &g_aw_reg_fw_flash_info,
	};

	return aw_reg_update(i2c, &update_info);
}

static const struct aw_flash_update_info g_aw_reg_boot_flash_info = {
		.tr_start_addr = AW9620X_BT_STAET_ADDR,
		.check_len = AW9620X_BT_CHECK_LEN, //7f8
		.check_en_addr = AW9620X_BT_CHECK_EN_ADDR,
		.check_code_addr = AW9620X_BT_CHECK_CODE_ADDR,
		.check_en_val = AW9620X_BT_CHECK_CODE_VAL,

		.erase_start_addr = AW9620X_BT_STAET_ADDR,
		.erase_sector_cnt = AW9620X_BT_ERASE_CNT,
		.sector_size = AW9620X_BT_SECTOR_SIZE,
};

int32_t aw9620x_reg_update_boot(AW_SAR_I2C_TYPE i2c,
					const uint8_t *data_start_offset, uint32_t update_data_len)
{
	struct aw_update_common update_info = {
		.update_flag = BOOT_UPDATE,
		.w_bin_offset = data_start_offset,
		.update_data_len = update_data_len,
		.flash_info = &g_aw_reg_boot_flash_info,
	};

	return aw_reg_update(i2c, &update_info);
}

