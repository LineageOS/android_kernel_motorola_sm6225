#ifndef __AW_UPDATE_FW_INTERFAACE_H_
#define __AW_UPDATE_FW_INTERFAACE_H_

#define  AW_SAR_I2C_TYPE	struct i2c_client *

int32_t aw9620x_prot_update_fw(AW_SAR_I2C_TYPE i2c, uint8_t direct_update_flag,
					const uint8_t *data_start_offset, uint32_t update_data_len ,uint32_t fw_version);

int32_t aw9620x_reg_update_fw(AW_SAR_I2C_TYPE i2c,
					const uint8_t *data_start_offset, uint32_t update_data_len);

int32_t aw9620x_reg_update_boot(AW_SAR_I2C_TYPE i2c,
					const uint8_t *data_start_offset, uint32_t update_data_len);
#endif