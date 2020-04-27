#ifndef _AW882XX_H_
#define _AW882XX_H_

//#define AW_DEBUG
/*
 * i2c transaction on Linux limited to 64k
 * (See Linux kernel documentation: Documentation/i2c/writing-clients)
*/
#define MAX_I2C_BUFFER_SIZE					65536
#define AW882XX_I2C_READ_MSG_NUM		2

#define AW882XX_FLAG_START_ON_MUTE			(1 << 0)
#define AW882XX_FLAG_SKIP_INTERRUPTS		(1 << 1)
#define AW882XX_FLAG_SAAM_AVAILABLE			(1 << 2)
#define AW882XX_FLAG_STEREO_DEVICE			(1 << 3)
#define AW882XX_FLAG_MULTI_MIC_INPUTS		(1 << 4)

#define AW882XX_NUM_RATES					9
#define AW882XX_SYSST_CHECK_MAX				10

#define AW882XX_MODE_SHIFT_MAX				2

#define AFE_RX_PROT_ID  0x1000				/*AFE_PORT_ID_PRIMARY_MI2S_RX*/
#define AW_MODULE_ID_COPP (0X10013D02)			/*SKT module id*/
#define AW_MODULE_PARAMS_ID_COPP_ENABLE (0X10013D14)	/*SKT enable param id*/


#define DEFAULT_CALI_VALUE (7)
#define ERRO_CALI_VALUE (0)
#define AFE_PARAM_ID_AWDSP_RX_SET_ENABLE        (0x10013D11)
#define AFE_PARAM_ID_AWDSP_RX_PARAMS            (0x10013D12)
#define AFE_PARAM_ID_AWDSP_TX_SET_ENABLE        (0x10013D13)
#define AFE_PARAM_ID_AWDSP_RX_VMAX_L            (0X10013D17)
#define AFE_PARAM_ID_AWDSP_RX_VMAX_R            (0X10013D18)
#define AFE_PARAM_ID_AWDSP_RX_CALI_CFG_L        (0X10013D19)
#define AFE_PARAM_ID_AWDSP_RX_CALI_CFG_R        (0x10013d1A)
#define AFE_PARAM_ID_AWDSP_RX_RE_L              (0x10013d1B)
#define AFE_PARAM_ID_AWDSP_RX_RE_R              (0X10013D1C)
#define AFE_PARAM_ID_AWDSP_RX_NOISE_L           (0X10013D1D)
#define AFE_PARAM_ID_AWDSP_RX_NOISE_R           (0X10013D1E)
#define AFE_PARAM_ID_AWDSP_RX_F0_L              (0X10013D1F)
#define AFE_PARAM_ID_AWDSP_RX_F0_R              (0X10013D20)
#define AFE_PARAM_ID_AWDSP_RX_REAL_DATA_L       (0X10013D21)
#define AFE_PARAM_ID_AWDSP_RX_REAL_DATA_R       (0X10013D22)


enum AWINIC_CALI_CMD{
	AW_CALI_CMD_NONE = 0,
	AW_CALI_CMD_START,
	AW_CALI_CMD_END,
	AW_CALI_CMD_SET_CALI_CFG,
	AW_CALI_CMD_GET_CALI_CFG,
	AW_CALI_CMD_GET_CALI_DATA,
	AW_CALI_CMD_SET_NOISE,
	AW_CALI_CMD_GET_F0,
	AW_CALI_CMD_SET_CALI_RE,
	AW_CALI_CMD_GET_CALI_RE,
	AW_CALI_CMD_SET_VMAX,
	AW_CALI_CMD_GET_VMAX,
	AW_CALI_CMD_MAX,
};

#define AW882XX_CALI_CFG_NUM 3
#define AW882XX_CALI_DATA_NUM 6
#define AW882XX_PARAMS_NUM 400
struct cali_cfg{
	int32_t data[AW882XX_CALI_CFG_NUM];
};
struct cali_data{
	int32_t data[AW882XX_CALI_DATA_NUM];
};
struct params_data{
	int32_t data[AW882XX_PARAMS_NUM];
};

#define AW882XX_IOCTL_MAGIC                'a'
#define AW882XX_IOCTL_SET_CALI_CFG         _IOWR(AW882XX_IOCTL_MAGIC, 1, struct cali_cfg)
#define AW882XX_IOCTL_GET_CALI_CFG         _IOWR(AW882XX_IOCTL_MAGIC, 2, struct cali_cfg)
#define AW882XX_IOCTL_GET_CALI_DATA        _IOWR(AW882XX_IOCTL_MAGIC, 3, struct cali_data)
#define AW882XX_IOCTL_SET_NOISE            _IOWR(AW882XX_IOCTL_MAGIC, 4, int32_t)
#define AW882XX_IOCTL_GET_F0               _IOWR(AW882XX_IOCTL_MAGIC, 5, int32_t)
#define AW882XX_IOCTL_SET_CALI_RE          _IOWR(AW882XX_IOCTL_MAGIC, 6, int32_t)
#define AW882XX_IOCTL_GET_CALI_RE          _IOWR(AW882XX_IOCTL_MAGIC, 7, int32_t)
#define AW882XX_IOCTL_SET_VMAX             _IOWR(AW882XX_IOCTL_MAGIC, 8, int32_t)
#define AW882XX_IOCTL_GET_VMAX             _IOWR(AW882XX_IOCTL_MAGIC, 9, int32_t)
#define AW882XX_IOCTL_SET_PARAM            _IOWR(AW882XX_IOCTL_MAGIC, 10,struct params_data)
#define AW882XX_IOCTL_ENABLE_CALI          _IOWR(AW882XX_IOCTL_MAGIC, 11,int8_t)

enum aw882xx_init {
	AW882XX_INIT_ST = 0,
	AW882XX_INIT_OK = 1,
	AW882XX_INIT_NG = 2,
};

enum aw882xx_chipid {
	AW882XX_ID = 0x1852,
};

enum aw882xx_modeshift {
	AW882XX_MODE_SPK_SHIFT = 0,
	AW882XX_MODE_RCV_SHIFT = 1,
};

enum aw882xx_mode_spk_rcv {
	AW882XX_SPEAKER_MODE = 0,
	AW882XX_RECEIVER_MODE = 1,
};
//smartpa monitor
enum aw882xx_ipeak {
	IPEAK_3P50_A = 0x08,
	IPEAK_3P00_A = 0x06,
	IPEAK_2P75_A = 0x05,
	IPEAK_2P50_A = 0x04,
	IPEAK_NONE   = 0xFF,
};

enum aw882xx_gain {
	GAIN_NEG_0P0_DB = 0x00,
	GAIN_NEG_0P5_DB = 0x01,
	GAIN_NEG_1P0_DB = 0x02,
	GAIN_NEG_1P5_DB = 0x03,
	GAIN_NEG_3P0_DB = 0x06,
	GAIN_NEG_4P5_DB = 0x09,
	GAIN_NEG_6P0_DB = 0x10,
	GAIN_NONE       = 0xFF,
};

enum aw882xx_vmax_percentage {
	VMAX_100_PERCENTAGE  = 0x00000000,
	VMAX_086_PERCENTAGE  = 0xFFED714D,
	VMAX_075_PERCENTAGE  = 0xFFD80505,
	VMAX_063_PERCENTAGE  = 0xFFBEAE7E,
	VMAX_NONE            = 0xFFFFFFFF,
};

#define AW882XX_MONITOR_DEFAULT_FLAG 0
#define AW882XX_MONITOR_SYSCTRL 0
#define AW882XX_MONITOR_DEFAULT_TIMER_VAL 30000
#define AW882XX_MONITOR_VBAT_RANGE 6025
#define AW882XX_MONITOR_INT_10BIT 1023
#define AW882XX_MONITOR_TEMP_SIGN_MASK (1<<9)
#define AW882XX_MONITOR_TEMP_NEG_MASK (0XFC00)
#define AW882XX_BIT_SYSCTRL2_BST_IPEAK_MASK         ( 15<< 0)
#define AW882XX_BIT_HAGCCFG4_GAIN_SHIFT (8)
#define AW882XX_BIT_HAGCCFG4_GAIN_MASK (0x00ff)

struct aw882xx_low_vol {
	uint32_t vol;
	uint8_t ipeak;
	uint8_t gain;
};

struct aw882xx_low_temp {
	int16_t temp;
	uint8_t ipeak;
	uint8_t gain;
	uint32_t vmax;
};

struct aw882xx_monitor{
	struct hrtimer timer;
	uint32_t timer_val;
	struct work_struct work;
	uint32_t is_enable;
	uint32_t sysctrl;
	uint16_t pre_vol;
	int16_t pre_temp;
#ifdef AW_DEBUG
	uint16_t test_vol;
	int16_t test_temp;
#endif
};

enum AWINIC_PROFILE{
	AW_PROFILE_MUSIC = 0,
	AW_PROFILE_RINGTONE,
	AW_PROFILE_NOTIFICATION,
	AW_PROFILE_VOICE,
	AW_PROFILE_MAX,
};

#define VERSION_MAX 4
#define PROJECT_NAME_MAX 24
#define VOLUME_STEP_DB  (6)
#define VOLUME_MIN_NEG_90_DB  (90)
#define FADE_STEP_DB   (6)


typedef struct awinic_afe_param_header{
	uint8_t fw[VERSION_MAX];
	uint8_t cfg[VERSION_MAX];
	uint8_t project[PROJECT_NAME_MAX];
	uint32_t start;
	uint32_t params_len;
	uint8_t check_sum;
	uint8_t profile_num;
	uint8_t reserve[2];
}aw_afe_params_hdr_t;

struct  profile_info {
	struct mutex lock;
	int cur_profile;
	int status;
	int len;
	char* data[AW_PROFILE_MAX];
};

struct aw882xx {
	struct regmap *regmap;
	struct i2c_client *i2c;
	struct snd_soc_codec *codec;
	struct device *dev;
	struct mutex lock;

	struct aw882xx_monitor monitor;
	int sysclk;
	int rate;
	int pstream;
	int cstream;

	int reset_gpio;
	int irq_gpio;

	unsigned char reg_addr;

	unsigned int flags;
	unsigned int chipid;
	unsigned int init;
	unsigned int spk_rcv_mode;
	int32_t cali_re;
	unsigned int cfg_num;
	struct  profile_info profile;
};

struct aw882xx_container {
	int len;
	unsigned char data[];
};
#endif
