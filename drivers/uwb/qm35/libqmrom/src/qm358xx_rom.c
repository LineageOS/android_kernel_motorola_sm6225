// SPDX-License-Identifier: GPL-2.0 OR Apache-2.0
/*
 * Copyright 2022 Qorvo US, Inc.
 *
 */

#include <qmrom.h>
#include <qmrom_spi.h>
#include <qmrom_log.h>
#include <qmrom_utils.h>
#include <spi_rom_protocol.h>
#include <qm358xx_firmware_pack_handling.h>

#define DEFAULT_SPI_CLOCKRATE 5000000
#define CHIP_VERSION_CHIP_REV_PAYLOAD_OFFSET (sizeof(resp_t))
#define CHIP_VERSION_DEV_REV_PAYLOAD_OFFSET \
	(CHIP_VERSION_CHIP_REV_PAYLOAD_OFFSET + 2)
#define CHUNK_SIZE 2036
#define CHUNK_PAYLOAD_MAX_SIZE (CHUNK_SIZE - CRYPTO_FIRMWARE_CHUNK_HDR_SIZE)
#define QM358_SPI_READY_TIMEOUT_MS 200

#ifdef QM358XX_WRITE_STATS
#include <linux/ktime.h>
#endif

#define QM358_SPI_SH_READY_CMD_BIT_MASK \
	(SPI_SH_READY_CMD_BIT_MASK >> 4 | SPI_SH_READY_CMD_BIT_MASK)

#define SB_OK 0x3E9

enum qm358xx_rom_cmd_e {
	/*! Defines the null placeholder for no command code. */
	CMD_NONE = 0x0u,
	/*! Defines the get chip version command code. */
	CMD_GET_CHIP_VERSION = 0x2u,
	/*! Defines the get soc info command code. */
	CMD_GET_SOC_INFO,
	/*! Defines the load firmware package command code. */
	CMD_LOAD_FW_PKG,
	/*! Defines the load asset package command code. */
	CMD_LOAD_ASSET_PKG,
	/*! Defines the load secure debug package command code. */
	CMD_LOAD_SECURE_DEBUG_PKG,
	/*! Defines the erase secure debug package command code. */
	CMD_ERASE_SECURE_DEBUG_PKG,
	/*! Defines the load SRAM firmware command code. */
	CMD_LOAD_SRAM,
	/*! Defines the run test mode command code. */
	CMD_RUN_TEST_MODE,
	/*! Defines the generate master secret and uuid command code. */
	CMD_GENERATE_MASTER_SECRET_AND_UUID,
};

/*! Defines the null placeholder for no sub-command code. */
#define SUB_CMD_NONE 0x0
/*! Defines the sub-command code base value. */
#define SUB_CMD_BASE 0x0

enum qm358xx_rom_sub_cmd_load_fw_pkg_e {
	/*! Defines the load firmware package header sub-command code. */
	SUB_CMD_LOAD_FW_PKG_HDR = SUB_CMD_BASE,
	/*! Defines the load firmware package payload chunk sub-command code. */
	SUB_CMD_LOAD_FW_PKG_PAYLOAD_CHUNK
};

enum qm358xx_rom_sub_cmd_load_sram_e {
	/*! Defines the load SRAM file size sub-command code. */
	SUB_CMD_LOAD_SRAM_FILE_SIZE = SUB_CMD_BASE,
	/*! Defines the load SRAM image data sub-command code. */
	SUB_CMD_LOAD_SRAM_IMG_DATA
};

/*! Defines the null placeholder for no response code. */
#define RESP_NONE 0x0000
/*! Defines the response code base value. */
#define RESP_OK 0xA000
/*! Defines the response code error base value. */
#define RESP_ERR_BASE 0xE000

/*! Defines the command mode ready response code. */
#define RESP_CMD_MODE_READY RESP_OK

/*! Defines the invalid command response code. */
#define RESP_INVALID_CMD_BASE (RESP_ERR_BASE + 0x100)

enum qm358xx_rom_invalid_cmd_rsp_e {
	/*! Defines the invalid command response code. */
	RESP_INVALID_CMD = (RESP_INVALID_CMD_BASE + 0x1u),
	/*! Defines the invalid I3C command response code. */
	RESP_INVALID_I3C_CMD
};

/*! Defines the soc info error response code base. */
#define RESP_GET_SOC_INFO_ERR_BASE (RESP_ERR_BASE + 0x200)
/*! Defines the soc info failed response code. */
#define RESP_GET_SOC_INFO_FAILED (RESP_GET_SOC_INFO_ERR_BASE + 0x1)

/*! Defines the load asset package error base code. */
#define RESP_LOAD_ASSET_PKG_ERR_BASE (RESP_ERR_BASE + 0x300)

enum qm358xx_rom_load_asset_pkg_rsp_e {
	/*! Defines the load asset package failed response code. */
	RESP_LOAD_ASSET_PKG_FAILED = (RESP_LOAD_ASSET_PKG_ERR_BASE + 0x1),
	/*! Defines the load asset package failed response code. Cmd not allowed for this LCS */
	RESP_LOAD_ASSET_PKG_CMD_NOT_ALLOWED_FOR_CURRENT_LCS,
	/*! Defines the load asset package invalid size response code. */
	RESP_LOAD_ASSET_PKG_INVALID_ASSET_SIZE
};

/*! Defines the load asset package error base code. */
#define RESP_LOAD_FW_PKG_ERR_BASE (RESP_ERR_BASE + 0x400)

enum qm358xx_rom_load_fw_pkg_rsp_e {
	/*! Defines the load firmware package invalid command response code. */
	RESP_LOAD_FW_PKG_INVALID_CMD = (RESP_LOAD_FW_PKG_ERR_BASE + 0x1),
	/*! Defines the load firmware package invalid sub-command response code. */
	RESP_LOAD_FW_PKG_INVALID_SUB_CMD,
	/*! Defines the load firmware package invalid certificate size response code. */
	RESP_LOAD_FW_PKG_INVALID_CERT_SIZE,
	/*! Defines the load firmware package header verification failed response code. */
	RESP_LOAD_FW_PKG_HDR_VERIFY_FAILED,
	/*! Defines the load firmware package payload chunk verification failed response code. */
	RESP_LOAD_FW_PKG_PAYLOAD_CHUNK_VERIFY_FAILED,
	/*! Defines the load firmware package set IV failed response code. */
	RESP_LOAD_FW_PKG_SET_IV_FAILED,
	/*! Defines the load firmware package decrypt payload failed response code. */
	RESP_LOAD_FW_PKG_DECRYPT_PAYLOAD_FAILED,
	/*! Defines the load firmware package image header verification failed response code. */
	RESP_LOAD_FW_PKG_VERIFY_IMG_HDR_FAILED,
	/*! Defines the load firmware package secure boot package verification failed response code. */
	RESP_LOAD_FW_PKG_VERIFY_SB_PKG_FAILED,
	/*! Defines the load firmware package secure boot package storing failed response code. */
	RESP_LOAD_FW_PKG_STORE_SB_PKG_FAILED,
	/*! Defines the load firmware package image storing failed response code. */
	RESP_LOAD_FW_PKG_STORE_IMG_FAILED,
	/*! Defines the load firmware package image verification failed response code. */
	RESP_LOAD_FW_PKG_VERIFY_IMG_FAILED,
	/*! Defines the load firmware package invalid chunk size response code. */
	RESP_LOAD_FW_PKG_INVALID_CHUNK_SIZE,
	/*! Defines the load firmware package secure boot package L2 encryption lock check failed response code. */
	RESP_LOAD_FW_PKG_LOCK_L2_ENC_CHECK_FAILED
};

/*! Defines the load to SRAM response code. Size is 0*/
#define RESP_LOAD_SRAM_ERR_BASE (RESP_ERR_BASE + 0x500)

enum qm358xx_rom_load_sram_rsp_e {
	/*! Defines the load to SRAM response code. Size is 0*/
	RESP_LOAD_SRAM_SIZE_0 = (RESP_LOAD_SRAM_ERR_BASE + 1),
	/*! Defines the load to SRAM response code. Size is too big */
	RESP_LOAD_SRAM_SIZE_TOO_BIG,
	/*! Defines the load to SRAM response code. Wrong sub command */
	RESP_LOAD_SRAM_INVALID_SUB_CMD,
	/*! Defines the load to SRAM response code. Cmd not allowed for this LCS */
	RESP_LOAD_SRAM_CMD_NOT_ALLOWED_FOR_CURRENT_LCS,
	/*! Defines the load to SRAM got data more data than expected */
	RESP_LOAD_SRAM_GOT_DATA_MORE_THAN_EXPECTED,
	/*! Defines the load to SRAM got wrong command */
	RESP_LOAD_SRAM_INVALID_CMD
};
/*! Defines the run test error base code. */
#define RESP_RUN_TEST_ERR_BASE (RESP_ERR_BASE + 0x600)

enum qm358xx_rom_run_test_mode_rsp_e {
	/*! Defines run test failed response code. */
	RESP_RUN_TEST_FAILED = (RESP_RUN_TEST_ERR_BASE + 0x1),
	/*! Defines run test failed response code. Cmd not allowed for this LCS */
	RESP_RUN_TEST_CMD_NOT_ALLOWED_FOR_CURRENT_LCS
};

/*! Defines the secure debug package processing error base code. */
#define RESP_LOAD_DEBUG_PKG_ERR_BASE (RESP_ERR_BASE + 0x700)

enum qm358xx_rom_load_secure_dbg_pkg_rsp_e {
	/*! Defines the load secure debug package failed response code. */
	RESP_LOAD_SEC_DEBUG_PKG_FAILED = (RESP_LOAD_DEBUG_PKG_ERR_BASE + 0x1),
	/*! Defines the load secure debug package storing failed response code. */
	RESP_LOAD_SEC_DEBUG_PKG_STORE_FAILED,
	/*! Defines the load secure debug package command not allowed for this LCS. */
	RESP_LOAD_SEC_DEBUG_PKG_NOT_ALLOWED_FOR_CURRENT_LCS,
	/*! Defines the load secure debug package invalid size response code. */
	RESP_LOAD_SEC_DEBUG_PKG_INVALID_SIZE
};

/*! Defines the secure debug package erasing error base code. */
#define RESP_ERASE_DEBUG_PKG_ERR_BASE (RESP_ERR_BASE + 0x800)
/*! Defines the erase secure debug package wiping failed response code. */
#define RESP_ERASE_SEC_DEBUG_PKG_WIPE_FAILED \
	(RESP_ERASE_DEBUG_PKG_ERR_BASE + 0x1)

/*! Defines the generate master secret and UUID error base code. */
#define RESP_GENERATE_MASTER_SECRET_AND_UUID_ERR_BASE (RESP_ERR_BASE + 0xA00)

enum qm358xx_rom_gen_secrets_rsp_e {
	/*! Defines the generate master secret and UUID error during master secret generation. */
	RESP_GENERATE_MASTER_SECRET_ERROR =
		(RESP_GENERATE_MASTER_SECRET_AND_UUID_ERR_BASE + 0x1),
	/*! Defines the generate master secret and UUID error during UUID loading. */
	RESP_GENERATE_UUID_ERROR,
	/*! Defines the generate master secret and UUID return code for when the UUID has already been generated. */
	RESP_GENERATE_UUID_ALREADY_GENERATED,
	/*! Defines the generate master secret and UUID return code for when the current LCS does not allow the command. */
	RESP_GENERATE_MASTER_SECRETS_NOT_ALLOWED_LCS
};

/*! Defines the get chip version success response code. */
#define RESP_GET_CHIP_VERSION_SUCCESS (RESP_OK + CMD_GET_CHIP_VERSION)
/*! Defines the soc info success response code. */
#define RESP_GET_SOC_INFO_SUCCESS (RESP_OK + CMD_GET_SOC_INFO)
/*! Defines the load asset package completed successfully response code */
#define RESP_LOAD_ASSET_PKG_SUCCESS (RESP_OK + CMD_LOAD_ASSET_PKG)
/*! Defines the load firmware package all images verified response code. */
#define RESP_LOAD_FW_PKG_VERIFIED_IMGS_COMPLETED (RESP_OK + CMD_LOAD_FW_PKG)
/*! Defines the load secure debug package success response code. */
#define RESP_LOAD_SEC_DEBUG_PKG_SUCCESS (RESP_OK + CMD_LOAD_SECURE_DEBUG_PKG)
/*! Defines the load secure debug package storing failed response code. */
#define RESP_ERASE_SEC_DEBUG_PKG_SUCCESS (RESP_OK + CMD_ERASE_SECURE_DEBUG_PKG)
/*! Defines the generation of master secret and UUID success response code. */
#define RESP_GENERATE_MASTER_SECRET_AND_UUID_SUCCESS \
	(RESP_OK + CMD_GENERATE_MASTER_SECRET_AND_UUID)

/*! Command header structure typedef. */
typedef struct {
	union {
		struct {
			/*! Command code. */
			uint8_t cmd;
			/*! Sub-command code. */
			uint8_t sub_cmd;
			/*! Response code. */
			uint16_t resp;
		};
		uint32_t all;
	};
} cmd_hdr_t;

#define MAKE_CMD_HDR(c, sc, r)                           \
	(cmd_hdr_t)                                      \
	{                                                \
		.cmd = (c), .sub_cmd = (sc), .resp = (r) \
	}

/*! Response header structure typedef. */
typedef struct {
	/*! Command header. */
	cmd_hdr_t cmd_hdr;
	/*! Error code. */
	uint32_t err_code;
} resp_t;

#define MAKE_RSP_HDR(ch, ec)                      \
	(resp_t)                                  \
	{                                         \
		.cmd_hdr = (ch), .err_code = (ec) \
	}

#define CMD_MODE_READY_RSP_HDR                            \
	MAKE_RSP_HDR(MAKE_CMD_HDR(CMD_NONE, SUB_CMD_NONE, \
				  RESP_CMD_MODE_READY),   \
		     SB_OK)

#define CMD_GET_CHIP_VERSION_RSP_HDR                                  \
	MAKE_RSP_HDR(MAKE_CMD_HDR(CMD_GET_CHIP_VERSION, SUB_CMD_NONE, \
				  RESP_GET_CHIP_VERSION_SUCCESS),     \
		     SB_OK)

#define CMD_GET_SOC_INFO_RSP_HDR                                  \
	MAKE_RSP_HDR(MAKE_CMD_HDR(CMD_GET_SOC_INFO, SUB_CMD_NONE, \
				  RESP_GET_SOC_INFO_SUCCESS),     \
		     SB_OK)

#define CMD_GENERATE_MASTER_SECRET_AND_UUID_RSP_HDR                         \
	MAKE_RSP_HDR(                                                       \
		MAKE_CMD_HDR(CMD_GENERATE_MASTER_SECRET_AND_UUID,           \
			     SUB_CMD_NONE,                                  \
			     RESP_GENERATE_MASTER_SECRET_AND_UUID_SUCCESS), \
		SB_OK)

#define CMD_LOAD_ASSET_PKG_RSP_HDR                                  \
	MAKE_RSP_HDR(MAKE_CMD_HDR(CMD_LOAD_ASSET_PKG, SUB_CMD_NONE, \
				  RESP_LOAD_ASSET_PKG_SUCCESS),     \
		     SB_OK)

#define CMD_LOAD_FW_PKG_LOAD_PAYLOAD_CHUNK_RSP_HDR                             \
	MAKE_RSP_HDR(MAKE_CMD_HDR(CMD_LOAD_FW_PKG,                             \
				  SUB_CMD_LOAD_FW_PKG_PAYLOAD_CHUNK, RESP_OK), \
		     SB_OK)

#define CMD_LOAD_FW_PKG_LOAD_PAYLOAD_CHUNK_COMPLETE_RSP_HDR                  \
	MAKE_RSP_HDR(MAKE_CMD_HDR(CMD_LOAD_FW_PKG,                           \
				  SUB_CMD_LOAD_FW_PKG_PAYLOAD_CHUNK,         \
				  RESP_LOAD_FW_PKG_VERIFIED_IMGS_COMPLETED), \
		     SB_OK)

#define CMD_LOAD_SECURE_DEBUG_PKG_RSP_HDR                                  \
	MAKE_RSP_HDR(MAKE_CMD_HDR(CMD_LOAD_SECURE_DEBUG_PKG, SUB_CMD_NONE, \
				  RESP_LOAD_SEC_DEBUG_PKG_SUCCESS),        \
		     SB_OK)

#define CMD_ERASE_SECURE_DEBUG_PKG_RSP_HDR                                  \
	MAKE_RSP_HDR(MAKE_CMD_HDR(CMD_ERASE_SECURE_DEBUG_PKG, SUB_CMD_NONE, \
				  RESP_ERASE_SEC_DEBUG_PKG_SUCCESS),        \
		     SB_OK)

#define CMD_LOAD_SRAM_RSP_HDR                                                \
	MAKE_RSP_HDR(MAKE_CMD_HDR(CMD_LOAD_SRAM, SUB_CMD_LOAD_SRAM_IMG_DATA, \
				  RESP_OK),                                  \
		     SB_OK)

#define COMPARE_RSP(act_rsp, exp_rsp) memcmp(act_rsp, exp_rsp, sizeof(resp_t))

#define RSP_HDR_FMT_STR "{ .cmd = %#x, .sub_cmd = %#x, .resp = %#x, err = %#x }"
#define RSP_HDR_EXP(r) \
	(r)->cmd_hdr.cmd, (r)->cmd_hdr.sub_cmd, (r)->cmd_hdr.resp, (r)->err_code

static int gen_secrets(struct qmrom_handle *handle);
static int load_asset_pkg(struct qmrom_handle *handle,
			  const struct firmware *asset_pkg);
static int load_fw_pkg(struct qmrom_handle *handle,
		       const struct firmware *fw_pkg);
static int load_secure_debug_pkg(struct qmrom_handle *handle,
				 struct firmware *dbg_pkg);
static int erase_secure_debug_pkg(struct qmrom_handle *handle);
static int run_test_mode(struct qmrom_handle *handle);
static int load_sram_fw(struct qmrom_handle *handle,
			const struct firmware *sram_fw);

#define qm358xx_rom_pre_read(h)                                            \
	({                                                                 \
		int rc;                                                    \
		qmrom_spi_wait_for_ready_line((h)->ss_rdy_handle,          \
					      QM358_SPI_READY_TIMEOUT_MS); \
		rc = qmrom_pre_read((h));                                  \
		rc;                                                        \
	})
#define qm358xx_rom_read(h)                                                \
	({                                                                 \
		int rc;                                                    \
		qmrom_spi_wait_for_ready_line((h)->ss_rdy_handle,          \
					      QM358_SPI_READY_TIMEOUT_MS); \
		rc = qmrom_read((h));                                      \
		rc;                                                        \
	})
#define qm358xx_rom_write_cmd(h, cmd)                                      \
	({                                                                 \
		int rc;                                                    \
		qmrom_spi_wait_for_ready_line((h)->ss_rdy_handle,          \
					      QM358_SPI_READY_TIMEOUT_MS); \
		rc = _qm358xx_rom_write_cmd((h), (cmd));                   \
		rc;                                                        \
	})
#define qm358xx_rom_write_size_cmd(h, cmd, ds, d)                          \
	({                                                                 \
		int rc;                                                    \
		qmrom_spi_wait_for_ready_line((h)->ss_rdy_handle,          \
					      QM358_SPI_READY_TIMEOUT_MS); \
		rc = _qm358xx_rom_write_size_cmd((h), (cmd), (ds), (d));   \
		rc;                                                        \
	})

static int _qm358xx_rom_write_cmd(struct qmrom_handle *handle, cmd_hdr_t *cmd)
{
	handle->hstc->all = 0;
	handle->hstc->host_flags.write = 1;
	handle->hstc->ul = 1;
	handle->hstc->len = sizeof(cmd_hdr_t);
	memcpy(&handle->hstc->payload[0], cmd, sizeof(cmd_hdr_t));

	return qmrom_spi_transfer(handle->spi_handle, (char *)handle->sstc,
				  (const char *)handle->hstc,
				  sizeof(struct stc) + handle->hstc->len);
}

static int _qm358xx_rom_write_size_cmd(struct qmrom_handle *handle,
				       cmd_hdr_t *cmd, uint16_t data_size,
				       const char *data)
{
	handle->hstc->all = 0;
	handle->hstc->host_flags.write = 1;
	handle->hstc->ul = 1;
	handle->hstc->len = sizeof(cmd_hdr_t) + data_size;
	memcpy(&handle->hstc->payload[0], cmd, sizeof(cmd_hdr_t));
	memcpy(&handle->hstc->payload[sizeof(cmd_hdr_t)], data, data_size);

	return qmrom_spi_transfer(handle->spi_handle, (char *)handle->sstc,
				  (const char *)handle->hstc,
				  sizeof(struct stc) + handle->hstc->len);
}

static void qm358xx_rom_poll_soc(struct qmrom_handle *handle)
{
	int retries = handle->comms_retries;
	memset(handle->hstc, 0, sizeof(struct stc));
	handle->sstc->raw_flags = 0;
	do {
		int rc =
			qmrom_spi_wait_for_ready_line(handle->ss_rdy_handle, 0);
		if (rc) {
			LOG_ERR("%s qmrom_spi_wait_for_ready_line failed\n",
				__func__);
			continue;
		}
		qmrom_spi_transfer(handle->spi_handle, (char *)handle->sstc,
				   (const char *)handle->hstc,
				   sizeof(struct stc) + handle->hstc->len);
	} while (retries-- && handle->sstc->raw_flags == 0);
}

static int qm358xx_rom_wait_ready(struct qmrom_handle *handle)
{
	int retries = handle->comms_retries;

	qm358xx_rom_poll_soc(handle);

	/* handle->sstc has been updated */
	while (retries-- &&
	       !(handle->sstc->raw_flags & QM358_SPI_SH_READY_CMD_BIT_MASK)) {
		if (handle->sstc->soc_flags.out_waiting) {
			qm358xx_rom_pre_read(handle);
			qm358xx_rom_read(handle);
		} else if (handle->sstc->soc_flags.out_active) {
			return qm358xx_rom_read(handle);
		} else
			qm358xx_rom_poll_soc(handle);
	}

	return handle->sstc->raw_flags & QM358_SPI_SH_READY_CMD_BIT_MASK ?
		       0 :
		       SPI_ERR_WAIT_READY_TIMEOUT;
}

static int qm358xx_rom_poll_cmd_resp(struct qmrom_handle *handle)
{
	int retries = handle->comms_retries;

	qm358xx_rom_poll_soc(handle);
	do {
		if (handle->sstc->soc_flags.out_waiting) {
			qm358xx_rom_pre_read(handle);
			return qm358xx_rom_read(handle);
		} else
			qm358xx_rom_poll_soc(handle);
	} while (retries--);
	if (retries <= 0)
		LOG_ERR("%s failed after %d replies\n", __func__,
			handle->comms_retries);

	return retries > 0 ? 0 : -1;
}

int qm358xx_rom_probe_device(struct qmrom_handle *handle)
{
	int rc;

	handle->is_be = false;

	if (handle->spi_speed == 0)
		qmrom_spi_set_freq(DEFAULT_SPI_CLOCKRATE);
	else
		qmrom_spi_set_freq(handle->spi_speed);

	rc = qmrom_reboot_bootloader(handle);
	if (rc) {
		LOG_ERR("%s: cannot reset the device...\n", __func__);
		return rc;
	}

	rc = qm358xx_rom_wait_ready(handle);
	if (rc) {
		LOG_INFO("%s: maybe not a QM358xx device\n", __func__);
		return rc;
	}

	LOG_DBG("%s: checking for command mode ready response\n", __func__);
	qm358xx_rom_poll_cmd_resp(handle);
	if (COMPARE_RSP(&handle->sstc->payload[0], &CMD_MODE_READY_RSP_HDR) !=
	    0) {
		LOG_ERR("%s: Expected: " RSP_HDR_FMT_STR
			" Received: " RSP_HDR_FMT_STR "\n",
			__func__, RSP_HDR_EXP(&CMD_MODE_READY_RSP_HDR),
			RSP_HDR_EXP((resp_t *)&handle->sstc->payload[0]));
		return SPI_PROTO_WRONG_RESP;
	}

	LOG_DBG("%s: sending CMD_GET_CHIP_VERSION command\n", __func__);
	rc = qm358xx_rom_write_cmd(handle,
				   &MAKE_CMD_HDR(CMD_GET_CHIP_VERSION,
						 SUB_CMD_NONE, RESP_NONE));
	qm358xx_rom_poll_cmd_resp(handle);
	if (COMPARE_RSP(&handle->sstc->payload[0],
			&CMD_GET_CHIP_VERSION_RSP_HDR) != 0) {
		LOG_ERR("%s: Expected: " RSP_HDR_FMT_STR
			" Received: " RSP_HDR_FMT_STR "\n",
			__func__, RSP_HDR_EXP(&CMD_GET_CHIP_VERSION_RSP_HDR),
			RSP_HDR_EXP((resp_t *)&handle->sstc->payload[0]));
		return SPI_PROTO_WRONG_RESP;
	}

	handle->chip_rev =
		bswap_16(SSTC2UINT16(handle,
				     CHIP_VERSION_CHIP_REV_PAYLOAD_OFFSET)) &
		0xFF;
	handle->device_version = bswap_16(
		SSTC2UINT16(handle, CHIP_VERSION_DEV_REV_PAYLOAD_OFFSET));
	if (handle->chip_rev != CHIP_REVISION_A0) {
		LOG_ERR("%s: wrong chip revision %#x\n", __func__,
			handle->chip_rev);
		handle->chip_rev = -1;
		return -1;
	}

	rc = qm358xx_rom_wait_ready(handle);
	if (rc) {
		LOG_ERR("%s: hmm something went wrong!!!\n", __func__);
		return rc;
	}

	rc = qm358xx_rom_write_cmd(handle,
				   &MAKE_CMD_HDR(CMD_GET_SOC_INFO, SUB_CMD_NONE,
						 RESP_NONE));
	qm358xx_rom_poll_cmd_resp(handle);
	if (COMPARE_RSP(&handle->sstc->payload[0], &CMD_GET_SOC_INFO_RSP_HDR) !=
	    0) {
		LOG_ERR("%s: Expected: " RSP_HDR_FMT_STR
			" Received: " RSP_HDR_FMT_STR "\n",
			__func__, RSP_HDR_EXP(&CMD_GET_SOC_INFO_RSP_HDR),
			RSP_HDR_EXP((resp_t *)&handle->sstc->payload[0]));
		return SPI_PROTO_WRONG_RESP;
	}

	/* skip the resp header bytes */
	memcpy(&handle->qm358xx_soc_info,
	       &handle->sstc->payload[sizeof(resp_t)],
	       sizeof(handle->qm358xx_soc_info));

	/* Set device type */
	handle->dev_gen = DEVICE_GEN_QM358XX;
	/* Set rom ops */
	handle->qm358xx_rom_ops.gen_secrets = gen_secrets;
	handle->qm358xx_rom_ops.load_asset_pkg = load_asset_pkg;
	handle->qm358xx_rom_ops.load_fw_pkg = load_fw_pkg;
	handle->qm358xx_rom_ops.load_secure_dbg_pkg = load_secure_debug_pkg;
	handle->qm358xx_rom_ops.erase_secure_dbg_pkg = erase_secure_debug_pkg;
	handle->qm358xx_rom_ops.load_sram_fw = load_sram_fw;
	handle->qm358xx_rom_ops.run_test_mode = run_test_mode;

	return 0;
}

#ifdef QM358XX_WRITE_STATS
static uint64_t total_bytes, total_time_ns;
static uint32_t max_write_time_ns, min_write_time_ns = ~0;

static void update_write_max_chunk_stats(ktime_t start_time)
{
	uint64_t elapsed_time_ns;

	total_bytes += CHUNK_SIZE;
	elapsed_time_ns = ktime_to_ns(ktime_sub(ktime_get(), start_time));
	total_time_ns += elapsed_time_ns;
	if (elapsed_time_ns > max_write_time_ns)
		max_write_time_ns = elapsed_time_ns;
	if (elapsed_time_ns < min_write_time_ns)
		min_write_time_ns = elapsed_time_ns;
}

static void dump_stats(void)
{
	uint32_t nb_chunks = total_bytes / CHUNK_SIZE;
	LOG_INFO(
		"QM358XX flashing time stats: %llu bytes over %llu us (chunk size %u, write timings: mean %u us, min %u us, max %u us)\n",
		total_bytes, total_time_ns / 1000, CHUNK_SIZE,
		(uint32_t)((total_time_ns / nb_chunks) / 1000),
		min_write_time_ns / 1000, max_write_time_ns / 1000);
}
#endif

static int gen_secrets(struct qmrom_handle *handle)
{
	int rc;

	LOG_DBG("%s: starting...\n", __func__);

	rc = qm358xx_rom_wait_ready(handle);
	if (rc)
		return rc;

	LOG_DBG("%s: sending CMD_GENERATE_MASTER_SECRET_AND_UUID command\n",
		__func__);
	rc = qm358xx_rom_write_cmd(
		handle, &MAKE_CMD_HDR(CMD_GENERATE_MASTER_SECRET_AND_UUID,
				      SUB_CMD_NONE, RESP_NONE));
	qm358xx_rom_poll_cmd_resp(handle);
	if (COMPARE_RSP(&handle->sstc->payload[0],
			&CMD_GENERATE_MASTER_SECRET_AND_UUID_RSP_HDR) != 0) {
		LOG_ERR("%s: Expected: " RSP_HDR_FMT_STR
			" Received: " RSP_HDR_FMT_STR "\n",
			__func__,
			RSP_HDR_EXP(
				&CMD_GENERATE_MASTER_SECRET_AND_UUID_RSP_HDR),
			RSP_HDR_EXP((resp_t *)&handle->sstc->payload[0]));
		return SPI_PROTO_WRONG_RESP;
	}

	return 0;
}

bool verify_fw_pkg_img_chunk(const char *data)
{
	fw_pkg_payload_chunk_hdr_t *hdr = (fw_pkg_payload_chunk_hdr_t *)data;
	if ((hdr->magic != CRYPTO_FIRMWARE_CHUNK_MAGIC_VALUE) ||
	    (hdr->version != CRYPTO_FIRMWARE_CHUNK_VERSION) ||
	    (hdr->length > CHUNK_PAYLOAD_MAX_SIZE)) {
		return false;
	}
	LOG_DBG("%s, image chunk: hdr->magic: %4s hdr->version: %#x hdr->length: %#x\n",
		__func__, (char *)&hdr->magic, hdr->version, hdr->length);

	return true;
}

static int load_fw_pkg(struct qmrom_handle *handle,
		       const struct firmware *fw_pkg)
{
	int rc, sent = 0, init_tx = 0;
	const char *bin_data = (const char *)fw_pkg->data;
	char *img_chunk = NULL;
	bool bin_data_pre_chunked = false;
#ifdef QM358XX_WRITE_STATS
	ktime_t start_time;
#endif
	LOG_DBG("%s: starting...\n", __func__);

	if (((uint32_t) * (uint32_t *)bin_data !=
	     CRYPTO_FIRMWARE_PACK_MAGIC_VALUE) ||
	    (fw_pkg->size <
	     (CRYPTO_FIRMWARE_PACK_HEADER_SIZE +
	      CRYPTO_FIRMWARE_IMAGE_CERT_SIZE + CRYPTO_IMAGES_CERT_PKG_SIZE +
	      CRYPTO_FIRMWARE_CHUNK_MIN_SIZE))) {
		LOG_ERR("%s: Invalid firmware package.\n", __func__);
		rc = -EINVAL;
		goto exit;
	}

	rc = qm358xx_rom_wait_ready(handle);
	if (rc)
		goto exit;

	LOG_DBG("%s: sending CMD_LOAD_FW_PKG:HEADER command\n", __func__);
	rc = qm358xx_rom_write_size_cmd(
		handle,
		&MAKE_CMD_HDR(CMD_LOAD_FW_PKG, SUB_CMD_LOAD_FW_PKG_HDR,
			      RESP_NONE),
		CRYPTO_FIRMWARE_PACK_HEADER_SIZE, bin_data);
	qm358xx_rom_poll_cmd_resp(handle);
	if (COMPARE_RSP(&handle->sstc->payload[0],
			&CMD_LOAD_FW_PKG_LOAD_PAYLOAD_CHUNK_RSP_HDR) != 0) {
		LOG_ERR("%s: Expected: " RSP_HDR_FMT_STR
			" Received: " RSP_HDR_FMT_STR "\n",
			__func__,
			RSP_HDR_EXP(
				&CMD_LOAD_FW_PKG_LOAD_PAYLOAD_CHUNK_RSP_HDR),
			RSP_HDR_EXP((resp_t *)&handle->sstc->payload[0]));
		rc = SPI_PROTO_WRONG_RESP;
		goto exit;
	}

	sent += CRYPTO_FIRMWARE_PACK_HEADER_SIZE;
	bin_data += CRYPTO_FIRMWARE_PACK_HEADER_SIZE;

	qmrom_alloc(img_chunk, CHUNK_SIZE);
	if (!img_chunk) {
		LOG_ERR("Image chunk buffer allocation failure\n");
		rc = -1;
		goto exit;
	}
	((fw_pkg_payload_chunk_hdr_t *)img_chunk)->magic =
		CRYPTO_FIRMWARE_CHUNK_MAGIC_VALUE;
	((fw_pkg_payload_chunk_hdr_t *)img_chunk)->version =
		CRYPTO_FIRMWARE_CHUNK_VERSION;

	while (sent < fw_pkg->size) {
		uint32_t tx_bytes = 0;
		const char *payload = NULL;
		if (verify_fw_pkg_img_chunk(bin_data)) {
			bin_data_pre_chunked = true;
			tx_bytes =
				((fw_pkg_payload_chunk_hdr_t *)bin_data)->length;
			if (tx_bytes > CHUNK_SIZE) {
				LOG_ERR("Invalid payload chunk size\n");
				rc = -1;
				goto exit;
			}
			payload = bin_data;
		} else {
			if (bin_data_pre_chunked) {
				LOG_ERR("Invalid pre-chunked package\n");
				rc = -1;
				goto exit;
			}
			tx_bytes = fw_pkg->size - sent;
			if (tx_bytes > CHUNK_PAYLOAD_MAX_SIZE)
				tx_bytes = CHUNK_PAYLOAD_MAX_SIZE;
			((fw_pkg_payload_chunk_hdr_t *)img_chunk)->length =
				tx_bytes;
			memcpy(&img_chunk[CRYPTO_FIRMWARE_CHUNK_HDR_SIZE],
			       bin_data, tx_bytes);
			payload = img_chunk;
		}

		LOG_DBG("%s: sending CMD_LOAD_FW_PKG:PAYLOAD_CHUNK command with %" PRIu32
			" bytes\n",
			__func__, tx_bytes);
#ifdef QM358XX_WRITE_STATS
		start_time = ktime_get();
#endif
		rc = qm358xx_rom_write_size_cmd(
			handle,
			&MAKE_CMD_HDR(CMD_LOAD_FW_PKG,
				      SUB_CMD_LOAD_FW_PKG_PAYLOAD_CHUNK,
				      RESP_NONE),
			CRYPTO_FIRMWARE_CHUNK_HDR_SIZE + tx_bytes, payload);
		if (rc)
			goto exit;
		sent += tx_bytes;
		bin_data += tx_bytes;
		if (bin_data_pre_chunked) {
			sent += CRYPTO_FIRMWARE_CHUNK_HDR_SIZE;
			bin_data += CRYPTO_FIRMWARE_CHUNK_HDR_SIZE;
		}
		if (!init_tx) {
			init_tx = 1;
			continue;
		}
		qm358xx_rom_poll_cmd_resp(handle);
#ifdef QM358XX_WRITE_STATS
		if (tx_bytes == CHUNK_SIZE)
			update_write_max_chunk_stats(start_time);
#endif
		if (COMPARE_RSP(&handle->sstc->payload[0],
				&CMD_LOAD_FW_PKG_LOAD_PAYLOAD_CHUNK_RSP_HDR) ==
		    0) {
			continue;
		} else if (COMPARE_RSP(
				   &handle->sstc->payload[0],
				   &CMD_LOAD_FW_PKG_LOAD_PAYLOAD_CHUNK_COMPLETE_RSP_HDR) ==
			   0) {
			LOG_INFO("%s: flashing done, quitting now\n", __func__);
			rc = 0;
			goto exit;
		} else {
			LOG_ERR("%s: Expected: " RSP_HDR_FMT_STR
				" OR " RSP_HDR_FMT_STR
				" Received: " RSP_HDR_FMT_STR "\n",
				__func__,
				RSP_HDR_EXP(
					&CMD_LOAD_FW_PKG_LOAD_PAYLOAD_CHUNK_RSP_HDR),
				RSP_HDR_EXP(
					&CMD_LOAD_FW_PKG_LOAD_PAYLOAD_CHUNK_COMPLETE_RSP_HDR),
				RSP_HDR_EXP(
					(resp_t *)&handle->sstc->payload[0]));
			rc = SPI_PROTO_WRONG_RESP;
			goto exit;
		}
	}
	qm358xx_rom_poll_cmd_resp(handle);
	if (COMPARE_RSP(&handle->sstc->payload[0],
			&CMD_LOAD_FW_PKG_LOAD_PAYLOAD_CHUNK_COMPLETE_RSP_HDR) !=
	    0) {
		LOG_ERR("%s: Expected: " RSP_HDR_FMT_STR
			" Received: " RSP_HDR_FMT_STR "\n",
			__func__,
			RSP_HDR_EXP(
				&CMD_LOAD_FW_PKG_LOAD_PAYLOAD_CHUNK_COMPLETE_RSP_HDR),
			RSP_HDR_EXP((resp_t *)&handle->sstc->payload[0]));
		rc = SPI_PROTO_WRONG_RESP;
	}
exit:
	if (img_chunk)
		qmrom_free(img_chunk);
	return rc;
}

static int load_asset_pkg(struct qmrom_handle *handle,
			  const struct firmware *asset_pkg)
{
	int rc;

	LOG_DBG("%s: starting...\n", __func__);
	rc = qm358xx_rom_wait_ready(handle);
	if (rc)
		return rc;

	LOG_DBG("%s: sending CMD_LOAD_ASSET_PKG command\n", __func__);
	rc = qm358xx_rom_write_size_cmd(
		handle,
		&MAKE_CMD_HDR(CMD_LOAD_ASSET_PKG, SUB_CMD_NONE, RESP_NONE),
		asset_pkg->size, (const char *)asset_pkg->data);
	qm358xx_rom_poll_cmd_resp(handle);
	if (COMPARE_RSP(&handle->sstc->payload[0],
			&CMD_LOAD_ASSET_PKG_RSP_HDR) != 0) {
		LOG_ERR("%s: Expected: " RSP_HDR_FMT_STR
			" Received: " RSP_HDR_FMT_STR "\n",
			__func__, RSP_HDR_EXP(&CMD_LOAD_ASSET_PKG_RSP_HDR),
			RSP_HDR_EXP((resp_t *)&handle->sstc->payload[0]));
		return SPI_PROTO_WRONG_RESP;
	}

	return 0;
}

static int load_secure_debug_pkg(struct qmrom_handle *handle,
				 struct firmware *dbg_pkg)
{
	int rc;

	LOG_DBG("%s: starting...\n", __func__);
	rc = qm358xx_rom_wait_ready(handle);
	if (rc)
		return rc;

	LOG_DBG("%s: sending CMD_LOAD_SECURE_DEBUG_PKG command\n", __func__);
	rc = qm358xx_rom_write_size_cmd(handle,
					&MAKE_CMD_HDR(CMD_LOAD_SECURE_DEBUG_PKG,
						      SUB_CMD_NONE, RESP_NONE),
					dbg_pkg->size,
					(const char *)dbg_pkg->data);
	qm358xx_rom_poll_cmd_resp(handle);
	if (COMPARE_RSP(&handle->sstc->payload[0],
			&CMD_LOAD_SECURE_DEBUG_PKG_RSP_HDR) != 0) {
		LOG_ERR("%s: Expected: " RSP_HDR_FMT_STR
			" Received: " RSP_HDR_FMT_STR "\n",
			__func__,
			RSP_HDR_EXP(&CMD_LOAD_SECURE_DEBUG_PKG_RSP_HDR),
			RSP_HDR_EXP((resp_t *)&handle->sstc->payload[0]));
		return SPI_PROTO_WRONG_RESP;
	}

	return 0;
}

static int erase_secure_debug_pkg(struct qmrom_handle *handle)
{
	int rc;

	LOG_DBG("%s: starting...\n", __func__);

	rc = qm358xx_rom_wait_ready(handle);
	if (rc)
		return rc;

	LOG_DBG("%s: sending CMD_ERASE_SECURE_DEBUG_PKG command\n", __func__);
	rc = qm358xx_rom_write_cmd(handle,
				   &MAKE_CMD_HDR(CMD_ERASE_SECURE_DEBUG_PKG,
						 SUB_CMD_NONE, RESP_NONE));
	qm358xx_rom_poll_cmd_resp(handle);
	if (COMPARE_RSP(&handle->sstc->payload[0],
			&CMD_ERASE_SECURE_DEBUG_PKG_RSP_HDR) != 0) {
		LOG_ERR("%s: Expected: " RSP_HDR_FMT_STR
			" Received: " RSP_HDR_FMT_STR "\n",
			__func__,
			RSP_HDR_EXP(&CMD_ERASE_SECURE_DEBUG_PKG_RSP_HDR),
			RSP_HDR_EXP((resp_t *)&handle->sstc->payload[0]));
		return SPI_PROTO_WRONG_RESP;
	}

	return 0;
}

static int run_test_mode(struct qmrom_handle *handle)
{
	int rc;

	LOG_DBG("%s: starting...\n", __func__);

	if ((handle->qm358xx_soc_info.lcs_state != LCS_TEST) &&
	    (handle->qm358xx_soc_info.lcs_state != LCS_RMA)) {
		LOG_ERR("%s: Test mode only available in TEST and RMA LCS.\n",
			__func__);
		return -1;
	}

	rc = qm358xx_rom_wait_ready(handle);
	if (rc)
		return rc;

	LOG_DBG("%s: sending CMD_RUN_TEST_MODE command\n", __func__);
	rc = qm358xx_rom_write_cmd(handle,
				   &MAKE_CMD_HDR(CMD_RUN_TEST_MODE,
						 SUB_CMD_NONE, RESP_NONE));

	return 0;
}

static int load_sram_fw(struct qmrom_handle *handle,
			const struct firmware *sram_fw)
{
	int rc, sent = 0;
	const char *bin_data = (const char *)sram_fw->data;
#ifdef QM358XX_WRITE_STATS
	ktime_t start_time;
#endif
	LOG_DBG("%s: starting...\n", __func__);
	rc = qm358xx_rom_wait_ready(handle);
	if (rc)
		return rc;

	LOG_DBG("%s: sending CMD_LOAD_SRAM:FILE_SIZE command\n", __func__);
	rc = qm358xx_rom_write_size_cmd(
		handle,
		&MAKE_CMD_HDR(CMD_LOAD_SRAM, SUB_CMD_LOAD_SRAM_FILE_SIZE,
			      RESP_NONE),
		sizeof(uint32_t), (const char *)&sram_fw->size);
	qm358xx_rom_poll_cmd_resp(handle);
	if (COMPARE_RSP(&handle->sstc->payload[0], &CMD_LOAD_SRAM_RSP_HDR) !=
	    0) {
		LOG_ERR("%s: Expected: " RSP_HDR_FMT_STR
			" Received: " RSP_HDR_FMT_STR "\n",
			__func__, RSP_HDR_EXP(&CMD_LOAD_SRAM_RSP_HDR),
			RSP_HDR_EXP((resp_t *)&handle->sstc->payload[0]));
		return SPI_PROTO_WRONG_RESP;
	}

	while (sent < sram_fw->size) {
		uint32_t tx_bytes = sram_fw->size - sent;
		if (tx_bytes > CHUNK_SIZE)
			tx_bytes = CHUNK_SIZE;

		LOG_DBG("%s: sending CMD_LOAD_SRAM:IMG_DATA command with %" PRIu32
			" bytes\n",
			__func__, tx_bytes);
#ifdef QM358XX_WRITE_STATS
		start_time = ktime_get();
#endif
		rc = qm358xx_rom_write_size_cmd(
			handle,
			&MAKE_CMD_HDR(CMD_LOAD_SRAM, SUB_CMD_LOAD_SRAM_IMG_DATA,
				      RESP_NONE),
			tx_bytes, bin_data);
		if (rc)
			return rc;
		sent += tx_bytes;
		bin_data += tx_bytes;
		if (sent == sram_fw->size) {
			LOG_INFO("%s: flashing done, quitting now\n", __func__);
			break;
		}
		qm358xx_rom_poll_cmd_resp(handle);
#ifdef QM358XX_WRITE_STATS
		if (tx_bytes == CHUNK_SIZE)
			update_write_max_chunk_stats(start_time);
#endif
		if (COMPARE_RSP(&handle->sstc->payload[0],
				&CMD_LOAD_SRAM_RSP_HDR) != 0) {
			LOG_ERR("%s: Expected: " RSP_HDR_FMT_STR
				" Received: " RSP_HDR_FMT_STR "\n",
				__func__, RSP_HDR_EXP(&CMD_LOAD_SRAM_RSP_HDR),
				RSP_HDR_EXP(
					(resp_t *)&handle->sstc->payload[0]));
			return SPI_PROTO_WRONG_RESP;
		}
	}

	return 0;
}

int qm358xx_rom_gen_secrets(struct qmrom_handle *handle)
{
	if (!handle->qm358xx_rom_ops.gen_secrets) {
		LOG_ERR("%s: generate secrets not supported on this device\n",
			__func__);
		return -EINVAL;
	}
	return handle->qm358xx_rom_ops.gen_secrets(handle);
}

int qm358xx_rom_load_asset_pkg(struct qmrom_handle *handle,
			       struct firmware *asset_pkg)
{
	if (!handle->qm358xx_rom_ops.load_asset_pkg) {
		LOG_ERR("%s: loading asset packages not supported on this device\n",
			__func__);
		return -EINVAL;
	}
	return handle->qm358xx_rom_ops.load_asset_pkg(handle, asset_pkg);
}

int qm358xx_rom_load_fw_pkg(struct qmrom_handle *handle,
			    struct firmware *fw_pkg)
{
	if (!handle->qm358xx_rom_ops.load_fw_pkg) {
		LOG_ERR("%s: loading firmware packages not supported on this device\n",
			__func__);
		return -EINVAL;
	}
	return handle->qm358xx_rom_ops.load_fw_pkg(handle, fw_pkg);
}

int qm358xx_rom_load_secure_dbg_pkg(struct qmrom_handle *handle,
				    struct firmware *dbg_pkg)
{
	if (!handle->qm358xx_rom_ops.load_secure_dbg_pkg) {
		LOG_ERR("%s: loading secure debug packages not supported on this device\n",
			__func__);
		return -EINVAL;
	}
	return handle->qm358xx_rom_ops.load_secure_dbg_pkg(handle, dbg_pkg);
}

int qm358xx_rom_erase_secure_dbg_pkg(struct qmrom_handle *handle)
{
	if (!handle->qm358xx_rom_ops.erase_secure_dbg_pkg) {
		LOG_ERR("%s: erasing secure debug packages not supported on this device\n",
			__func__);
		return -EINVAL;
	}
	return handle->qm358xx_rom_ops.erase_secure_dbg_pkg(handle);
}

int qm358xx_rom_run_test_mode(struct qmrom_handle *handle)
{
	if (!handle->qm358xx_rom_ops.run_test_mode) {
		LOG_ERR("%s: test mode not supported on this device\n",
			__func__);
		return -EINVAL;
	}
	return handle->qm358xx_rom_ops.run_test_mode(handle);
}

int qm358xx_rom_load_sram_fw(struct qmrom_handle *handle,
			     struct firmware *sram_fw)
{
	if (!handle->qm358xx_rom_ops.load_sram_fw) {
		LOG_ERR("%s: loading SRAM firmware not supported on this device\n",
			__func__);
		return -EINVAL;
	}
	return handle->qm358xx_rom_ops.load_sram_fw(handle, sram_fw);
}
