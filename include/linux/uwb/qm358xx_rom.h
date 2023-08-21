// SPDX-License-Identifier: GPL-2.0 OR Apache-2.0
/*
 * Copyright 2021 Qorvo US, Inc.
 *
 */

#ifndef __QM358XX_ROM_H__
#define __QM358XX_ROM_H__

#define QM358XX_ROM_UUID_LEN 0x10

/* Life cycle state definitions. */
#define LCS_TEST (0x00) /* Previously: CC_BSV_CHIP_MANUFACTURE_LCS */
#define LCS_OEM (0x01) /* Previously: CC_BSV_DEVICE_MANUFACTURE_LCS */
#define LCS_USER (0x02) /* Previously: CC_BSV_SECURE_LCS */
#define LCS_RMA (0x03) /* Previously: CC_BSV_RMA_LCS */

typedef int (*gen_secrets_fn)(struct qmrom_handle *handle);
typedef int (*load_asset_pkg_fn)(struct qmrom_handle *handle,
				 const struct firmware *asset_pkg);
typedef int (*load_fw_pkg_fn)(struct qmrom_handle *handle,
			      const struct firmware *fw_pkg);
typedef int (*load_secure_dbg_pkg_fn)(struct qmrom_handle *handle,
				      struct firmware *dbg_pkg);
typedef int (*erase_secure_dbg_pkg_fn)(struct qmrom_handle *handle);
typedef int (*run_test_mode_fn)(struct qmrom_handle *handle);
typedef int (*load_sram_fw_fn)(struct qmrom_handle *handle,
			       const struct firmware *sram_fw);

struct qm358xx_rom_code_ops {
	gen_secrets_fn gen_secrets;
	load_asset_pkg_fn load_asset_pkg;
	load_fw_pkg_fn load_fw_pkg;
	load_secure_dbg_pkg_fn load_secure_dbg_pkg;
	erase_secure_dbg_pkg_fn erase_secure_dbg_pkg;
	run_test_mode_fn run_test_mode;
	load_sram_fw_fn load_sram_fw;
};

struct qm358xx_soc_infos {
	uint8_t uuid[QM358XX_ROM_UUID_LEN];
	uint32_t customer_id;
	uint32_t product_id;
	uint32_t arb_ver_icv;
	uint32_t arb_ver_oem;
	uint8_t lcs_state;
	uint8_t working_model;
	uint8_t business_ctx;
	uint8_t secure_debug;
	uint8_t secure_boot_oem;
	uint8_t fw_encryption_oem;
	uint8_t lock_debug;
	uint8_t sec_ver_icv;
	uint8_t sec_ver_oem;
	uint8_t enc_l2_lock;
};

int qm358xx_rom_gen_secrets(struct qmrom_handle *handle);
int qm358xx_rom_load_asset_pkg(struct qmrom_handle *handle,
			       struct firmware *asset_pkg);
int qm358xx_rom_load_fw_pkg(struct qmrom_handle *handle,
			    struct firmware *fw_pkg);
int qm358xx_rom_load_secure_dbg_pkg(struct qmrom_handle *handle,
				    struct firmware *dbg_pkg);
int qm358xx_rom_erase_secure_dbg_pkg(struct qmrom_handle *handle);
int qm358xx_rom_run_test_mode(struct qmrom_handle *handle);
int qm358xx_rom_load_sram_fw(struct qmrom_handle *handle,
			     struct firmware *sram_fw);

#endif /* __QM358XX_ROM_H__ */
