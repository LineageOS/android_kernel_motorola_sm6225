#ifndef __AW_UPDATE_H_
#define __AW_UPDATE_H_
#include "aw_sar_comm_interface.h"
#include "aw_sar_type.h"
#include "aw_update_fw_interface.h"
#ifdef __cplusplus
extern "C" {
#endif

#define AW_SAR_LOG_TAG "awinic sar "
#define AW_FW_UPDATE_DEBUG_LOG
#ifdef AW_FW_UPDATE_DEBUG_LOG
#define AWLOGD_FW(format, arg...) \
	do {\
		 pr_info(AW_SAR_LOG_TAG "[%s:%d] "format"\n", __func__, __LINE__, ##arg);\
	} while (0)
#else
#define	AWLOGD_FW(format, arg...)
#endif

#define AW_FW_UPDATE_DEBUG_LOG
#ifdef AW_FW_UPDATE_DEBUG_LOG
#define AWLOGI_FW(format, arg...) \
	do {\
		pr_info(AW_SAR_LOG_TAG "[%s:%d] "format"\n", __func__, __LINE__, ##arg);\
	} while (0)
#else
#define	AWLOGI_FW(format, arg...)
#endif

#define AW_FW_UPDATE_DEBUG_LOG
#ifdef AW_FW_UPDATE_DEBUG_LOG
#define AWLOGE_FW(format, arg...) \
	do {\
		pr_err(AW_SAR_LOG_TAG "[%s:%d] "format"\n", __func__, __LINE__, ##arg);\
	} while (0)
#else
#define	AWLOGE_FW(format, arg...)
#endif

#define REG_IRQSRC					(0x4410)

//driver user
#define REG_ACCESSEN					(0xFF20)
#define REG_MCFG					(0x4444)
#define REG_ISPGO					(0x4714)
#define REG_PMU_CFG					(0x4820)
#define REG_ARRAY2_EW_EN				(0x4794)
#define REG_ISPADR					(0x4704)
#define REG_ISPWDATA					(0x4708)
#define REG_ISPRDATA					(0x470C)
#define REG_ISPCMD					(0x4710)
#define REG_T_RCV					(0x472C)
#define REG_CMD						(0x4408)
#define REG_BOOT_LOADER_ACTIVE				(0x4748)

#define REG_OPEN_APB_ACCESS_EN			(0x3c00f091)
#define REG_SET_PMU_CFG				(0x6)
#define REG_SET_BTROM_EW_EN			(0x5a637955)
#define REG_ISP_CMD_CONFIG			(0x0c)
#define REG_SET_ISP_GO				(0x1)
#define REG_ENSET_PMU_CFG			(0x4)
#define REG_ACCESS_MAIN_ARR			(0x05)
#define REG_SET_T_RCV				(0xf0)
#define REG_SET_T_RCV_EN			(0x16)
#define REG_OPEN_REG_ACCESS_EN			(0x3c00ffff)
#define REG_OPEN_MCFG_EN			(0x10000)
#define REG_ENSET_BTROM_EW_EN			(0x0)
#define REG_SET_MCFG00				(0x0)
#define REG_ISP_CMD_MAIN_ARR			(0X03)
#define REG_HOSTCTRL_EN				(0x1)
#define REG_ACCESSEN_OPEN			(0x3c00F091)
#define REG_ACCESSEN_CLOSE			(0x3c00f011)
#define AW_REG_FLASH_WAKE_UP			(0x4700)
#define REG_UPDATA_DIS				(0x4744)
#define AW_REG_FLASH_WAKE_UP_ON			(0x110)
#define AW_REG_FLASH_WAKE_UP_OFF		(0x102)

#define AW_BT_HOST2CPU_TRIG_ONLINE_UPGRADE_CMD		(0x14)

#define AW_BT_STATE_INFO_ERR			(0x1c00)
#define AW_BT_STATE_INFO_PST			(0x1C04)
#define AW_BT_STATE_INFO_FLAG			(0x1C08)
#define AW_BT_PROT_CMD_PACK_ADDR		(0x1C10)
#define AW_BT_PROT_ACK_PACK_ADDR		(0x1C14)
#define AW_BT_MESSAGE_MSG0			(0x1C20)
#define AW_BT_MESSAGE_MSG1			(0x1C24)
#define AW_BT_MESSAGE_MSG_FLAG			(0x1C28)
#define	AW_BT_VER_INF_VERSION			(0x1C30)
#define	AW_BT_VER_INF_DATE			(0x1C34)
#define	AW_BT_HOST2CPU_TRIG			(0x4408)
#define AW_BTCPU2HOST_TRIG			(0x4410)

#define AW_SRAM_FIRST_DETECT			(0x20000800)

#define AW_POWER_ON_DELAY_MS			(25)
#define REG_MCFG_DELAY_MS			(20)
#define AW_IRQ_DELAY_MS				(100)
#define REG_RSTNALL_VAL				(0x3c)
#define INIT_OVER_IRQ				(1)
#define CMD_SLEEP_MODE				(2)

#define AW_WORD_LEN				(4)
#define AW_REG_DATA_LEN				(4)
#define AW_REG_ADDR_LEN				(2)
#define AW_TX_BUF_LEN				(AW_REG_DATA_LEN +  AW_REG_ADDR_LEN)

#define AW_RETRIES				(5)
#define TRANSFER_SEQ_LEN			    (4)
#define TRANSFER_DTS_ADDR_LEN			(4)
#define AW_FLASH_DEFAULT_VAL			(0xffffffff)

#define AW_PROT_STOP_WAIT_IRQ_CYCLES		(100)
#define AW_REG_UPDATA_DIS_MASK			(0x00ffffff)
#define AW_AFECFG3_CVMULTUALMOD			(11)
#define AW_AFECFG3_CVOFF2X			(10)

#define AW_REG_MCFG				(0x4444)
#define AW_REG_ACESS_EN				(0xff20)
#define AW_REG_BOOTLOADER_ACTIVER		(0x4748)
#define AW_REG_MCFG				    (0x4444)
#define AW_REG_RSTNALL				(0xff18)
#define AW_CPU_HALT				    (0x00010000)
#define AW_ACC_PERI				    (0x3c00ffff)
#define AWDIS_HARD_BT_MODE			(0x00000000)
#define AW_CPU_RUN				    (0x00000000)
#define AW_RSTNALL				    (0x0000003c)

enum AW_UPDATE_RET {
	AW_FW_OK,
	AW_FW_ERR,
};
enum AW9620X_UPDATE_MODE {
	BOOT_UPDATE,
	FIRMWARE_UPDATE,
};

#define PROT_SEND_ADDR	            (0x0800)
#define	SEND_ADDR_LEN	            (2)
#define AW_ACK_ADDR				    (0x1800)


#define AW_PACK_FIXED_SIZE 			    (12)
#define AW_ADDR_SIZE 				    (2)

//hecksum_en host->slave
#define AW_HEADER_VAL				    (0x02)

#define REG_H2C_TRIG_PARSE_CMD		    (0x15)

#define AW_WAIT_IRQ_CYCLES				(10)
#define AW_WAIT_ENTER_SLEEP_MODE		(100)
#define AW_WAIT_ENTER_TR_MODE			(100)

#define INIT_OVER_IRQ_OK			    (1)

#define TRANSFER_SEQ_LEN			    (4)
#define TRANSFER_DTS_ADDR_LEN			(4)

#define AW_BT_CHECK_SUM_ADDR			(0x07fc)

#define AW_START_CMD_ACK_STATUS			(2)
#define AW_START_CMD_ACK_ADDR_LEN		(2)

#define P_AW_START_CMD_SEND_VALUE			(NULL)
#define AW_START_CMD_SEND_VALUE_LEN			(0)
#define P_AW_ERASE_CHIP_CMD_SEND_VALUE		(NULL)
#define AW_ERASE_CHIP_CMD_SEND_VALUE_LEN	(0)
#define P_AW_RESTORE_CMD_SEND_VALUE		    (NULL)
#define AW_RESTORE_CMD_SEND_VALUE_LEN		(0)

#define AW_EN_TR_CHECK_VALUE_LEN		    (16)
#define	AW_FLASH_ERASE_CHIO_CMD_ACK_LEN		(4)

#define AW_REG_WST_MODE_SLEEP			    (0x03000000)


enum AW9620X_FLASH_ADDR {
	FLASH_ADDR_BOOT,
	FLASH_ADDR_FW,
};

enum AW9620X_CMP_VAL {
	BIN_VER_EQUALS_SOC_VER,
	BIN_VER_NOT_EQUALS_SOC_VER,
};

enum AW_MODULE {
	UPDATE_MODULE = 0x01,
	FLASH_MODULE,
	QUERY_MODULE,
};

enum AW_UPDATE_COMMAND {
	UPDATE_START_CMD = 0X01,
	UPDATE_TRANSFER_CMD,
	UPDATE_STOP_CMD,
	UPDATE_RESTORE_FLASHBT_CMD,
};

enum AW_UPDATE_COMMAND_ACK {
	UPDATE_START_CMD_ACK = 0X01,
	UPDATE_RANSFER_CMD_ACK,
	UPDATE_STOP_CMD_ACK,
	UPDATE_RESTORE_FLASHBT_CMD_ACK,
};

enum AW_FLASH_COMMAND {
	FLASH_ERASE_CHIP_CMD = 0X01,
	FLASH_ERASE_SECTOR_CMD,
};

enum AW_FLASH_COMMAND_ACK {
	FLASH_ERASE_CHIP_CMD_ACK = 0X01,
	FLASH_ERASE_SECTOR_CMD_ACK,
};

enum AW_QUERY_COMMAND {
	QUERY_BT_VER_CMD = 0X01,
	QUERY_BT_DATE_CMD,
	QUERY_ERR_CODE_CMD,
	QUERY_PST_CMD,
	QUERY_CACHE_CMD,
};

enum AW_QUERY_COMMAND_ACK {
	QUERY_BT_VER_CMD_ACK = 0X01,
	QUERY_BT_DATE_CMD_ACK,
	QUERY_ERR_CODE_CMD_ACK,
	QUERY_PST_CMD_ACK,
	QUERY_CACHE_CMD_ACK,
};

struct aw_soc_protocol {
	uint16_t header;
	uint16_t size;
	uint32_t checksum;
	uint8_t module;
	uint8_t command;
	uint16_t length;
	uint8_t value[0];
};

enum AW_UPDATE_FW_STATE {
	SEND_UPDTAE_CMD,
	SEND_START_CMD,
	SEND_ERASE_SECTOR_CMD,
	SEND_UPDATE_FW_CMD,
	SEND_UPDATE_CHECK_CODE_CMD,
	SEND_RESTORE_CMD,
	SEND_STOP_CMD,
};

enum AW_PROT_UPDATE_FW_DELAY_TIME{
	SEND_UPDTAE_CMD_DELAY_MS = 1,
	SEND_START_CMD_DELAY_MS = 3,
	SEND_ERASE_SECTOR_CMD_DELAY_MS = 5,
	SEND_UPDATE_FW_CMD_DELAY_MS = 100,
	SEND_UPDATE_CHECK_CODE_CMD_DELAY_MS = 1,
	SEND_RESTORE_CMD_DELAY_MS = 27,
	SEND_STOP_CMD_DELAY_MS = 25,
};

enum AW_PROT_RECV_LEN {
	SEND_START_CMD_RECV_LEN = 8,
	SEND_ERASE_CHIP_CMD_RECV_LEN = 4,
	SEND_UPDATE_FW_CMD_RECV_LEN = 8,
	SEND_UPDATE_CHECK_CODE_CMD_RECV_LEN = 4,
	SEND_RESTORE_CMD_RECV_LEN = 4,
};
/********************reg update end****************/
struct aw_flash_update_info {
	uint32_t tr_start_addr;
	uint32_t check_len;//The verification code is calculated by the verification length and the firmware chip
	uint32_t check_en_addr;
	uint32_t check_code_addr;
	uint32_t check_en_val;

	uint32_t erase_start_addr;
	uint32_t erase_sector_cnt;
	uint32_t sector_size;
};

enum chip_select_flag {
	CHIP_AW9620X,
	//CHIP_AW9320X,
	//CHIP_AW9328XQNR,
};

struct aw_fw_data_info {
	const uint8_t *data_start_offset;
	uint32_t update_data_len;
	uint32_t fw_version;
};

struct aw_update_chip_diff {
	uint16_t fwver_reg_addr;
	//uint16_t wst_reg_addr;

	uint16_t cmd_reg_addr;
	uint32_t cmd_sleep_mode;
	enum chip_select_flag chip_flag;
};

struct aw_fw_info {
	uint32_t single_wr_len;
	const struct aw_flash_update_info *flash_info;
};

struct aw_update_para {
	uint8_t direct_update_flag;				//Upgrade without comparing version numbers
	const struct aw_update_chip_diff *aw_update_diff;	//Differences in chip updates
};

struct aw_update_fw {
	struct aw_fw_info *fw_info;
	struct aw_update_para *update_para;
	struct aw_fw_data_info *fw_data_info; //Firmware information
};


struct aw_update_common {
	uint8_t update_flag;
	const uint8_t *w_bin_offset;
	uint32_t update_data_len;
	const struct aw_flash_update_info *flash_info;
};


#ifdef __cplusplus
}
#endif

#endif

