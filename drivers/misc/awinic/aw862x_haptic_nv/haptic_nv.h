#ifndef _HAPTIC_NV_H_
#define _HAPTIC_NV_H_
/*********************************************************
 *
 * Macro Control
 *
 *********************************************************/
#define AW_CHECK_RAM_DATA
#define AW_READ_BIN_FLEXBALLY
#define AW_INPUT_FRAMEWORK
/* #define AW_DURATION_DECIDE_WAVEFORM */
/* #define AW_ENABLE_RTP_PRINT_LOG */
/* #define AW8624_MUL_GET_F0 */

#define AW8624_DRIVER
#define AW8622X_DRIVER
#define AW86214_DRIVER
#define AW8623X_DRIVER
/*********************************************************
 *
 * Haptic_NV CHIPID
 *
 *********************************************************/
#define AW8624_CHIP_ID				(0x24)
#define AW8622X_CHIP_ID				(0x00)
#define AW86214_CHIP_ID				(0x01)
#define AW8623X_CHIP_ID_H			(0x23)
#define AW86233_CHIP_ID_L			(0x30)
#define AW86234_CHIP_ID_L			(0x40)
#define AW86235_CHIP_ID_L			(0x50)
/*********************************************************
 *
 * Marco
 *
 *********************************************************/
#ifdef AW8622X_DRIVER
#define AW862XX_COMMON
#endif

#ifdef AW86214_DRIVER
#define AW862XX_COMMON
#endif

#define AW_I2C_NAME				"haptic_nv"
#define AW_I2C_RETRIES				(5)
#define AW_I2C_READ_MSG_NUM			(2)
#define AW_I2C_BYTE_ONE				(1)
#define AW_I2C_BYTE_TWO				(2)
#define AW_I2C_BYTE_THREE			(3)
#define AW_I2C_BYTE_FOUR			(4)
#define AW_I2C_BYTE_FIVE			(5)
#define AW_I2C_BYTE_SIX				(6)
#define AW_I2C_BYTE_SEVEN			(7)
#define AW_I2C_BYTE_EIGHT			(8)

#define AWRW_SIZE				(220)
#define AW_REG_MAX				(0xFF)
#define AW_NAME_MAX				(64)
#define AW_VBAT_MIN				(3000)
#define AW_VBAT_MAX				(4500)
#define AW_TRIG_NUM				(3)
#define AWRW_CMD_UNIT				(5)
#define AW_VBAT_REFER				(4200)
#define AW_DEFAULT_GAIN				(0x80)
#define AW_SEQUENCER_SIZE			(8)
#define AW_PM_QOS_VALUE_VB			(400)
#define AW_GUN_TYPE_DEF_VAL			(0xFF)
#define AW_DTS_NAME_NUM_MAX			(40)
#define AW_F0_CALI_DATA_MIN			(0x20)
#define AW_F0_CALI_DATA_MAX			(0x1F)
#define AW_BULLET_NR_DEF_VAL			(0)
#define CPU_LATENCY_QOC_VALUE 			(0)
#define AW_SEQUENCER_LOOP_SIZE			(4)
#define AW_RAMDATA_SHOW_COLUMN			(16)
#define AW_READ_CHIPID_RETRIES			(5)
#define AW_RAMDATA_RD_BUFFER_SIZE		(1024)
#define AW_RAMDATA_WR_BUFFER_SIZE		(2048)
#define AW_RAMDATA_SHOW_UINT_SIZE		(6)
#define AW_READ_CHIPID_RETRY_DELAY		(2)
#define AW_EFFECT_NUMBER			(3)
#define AW_RAM_WORK_DELAY_INTERVAL		(8000)
#define AW_RAMDATA_SHOW_LINE_BUFFER_SZIE	(100)

#define AW_RL_DELAY_MIN				(3000)
#define AW_RL_DELAY_MAX				(3500)
#define AW_F0_DELAY_MIN				(10000)
#define AW_F0_DELAY_MAX				(10500)
#define AW_RTP_DELAY_MIN			(2000)
#define AW_RTP_DELAY_MAX			(2500)
#define AW_PLAY_DELAY_MIN			(2000)
#define AW_PLAY_DELAY_MAX			(2500)
#define AW_STOP_DELAY_MIN			(2000)
#define AW_STOP_DELAY_MAX			(2500)
#define AW_VBAT_DELAY_MIN			(2000)
#define AW_VBAT_DELAY_MAX			(2500)
#define AW_OSC_TRIM_PARAM			(50)
#define AW_OSC_CALI_MAX_LENGTH			(5100000)

#define AW_SET_RAMADDR_H(addr)			((addr) >> 8)
#define AW_SET_RAMADDR_L(addr)			((addr) & 0x00FF)
#define AW_SET_BASEADDR_H(addr)			((addr) >> 8)
#define AW_SET_BASEADDR_L(addr)			((addr) & 0x00FF)
/*********************************************************
 *
 * aw8624 marco
 *
 ********************************************************/
#define AW8624_F0_CALI_ACCURACY			(25)
#define AW8624_MUL_GET_F0_RANGE			(150)
#define AW8624_MUL_GET_F0_NUM			(3)

#define AW8624_VBAT_FORMULA(code)		(6100 * (code) / 256)
#define AW8624_F0_FORMULA(reg, coeff)		(1000000000 / ((reg) * (coeff)))
#define AW8624_RL_FORMULA(reg_val)		(298 * (reg_val))
#define AW8624_SET_AEADDR_H(addr)		((((addr) >> 1) >> 8))
#define AW8624_SET_AEADDR_L(addr)		(((addr) >> 1) & 0x00FF)
#define AW8624_SET_AFADDR_H(addr)		(((addr) - ((addr) >> 2)) >> 8)
#define AW8624_SET_AFADDR_L(addr)		(((addr) - ((addr) >> 2)) & 0x00FF)
/*********************************************************
 *
 * aw862xx marco
 *
 ********************************************************/
#define AW862XX_DRV2_LVL_MAX			(127)
#define AW862XX_DRV_WIDTH_MIN			(40)
#define AW862XX_DRV_WIDTH_MAX			(255)
#define AW862XX_F0_CALI_ACCURACY		(24)

#define AW862XX_VBAT_FORMULA(code)		(6100 * (code) / 1024)
#define AW862XX_F0_FORMULA(code)		(384000 * 10 / (code))
#define AW862XX_RL_FORMULA(code, d2s_gain)	(((code) * 678 * 100) / (1024 * (d2s_gain)))
#define AW862XX_SET_AEADDR_H(addr)		((((addr) >> 1) >> 4) & 0xF0)
#define AW862XX_SET_AEADDR_L(addr)		(((addr) >> 1) & 0x00FF)
#define AW862XX_SET_AFADDR_H(addr)		((((addr) - ((addr) >> 2)) >> 8) & 0x0F)
#define AW862XX_SET_AFADDR_L(addr)		(((addr) - ((addr) >> 2)) & 0x00FF)
#define AW862XX_DRV2_LVL_FORMULA(f0, vrms)	((((f0) < 1800) ? 1809920 : 1990912) / 1000 * (vrms) / 61000)
#define AW862XX_DRV_WIDTH_FORMULA(f0, margin, brk_gain) \
						((240000 / (f0)) - (margin) - (brk_gain) - 8)

/*********************************************************
 *
 * aw8623x marco
 *
 ********************************************************/
#define AW8623X_DRV2_LVL_MAX			(127)
#define AW8623X_DRV_WIDTH_MIN			(40)
#define AW8623X_DRV_WIDTH_MAX			(255)
#define AW8623X_F0_CALI_ACCURACY		(24)

#define AW8623X_VBAT_FORMULA(code)		(6100 * (code) / 1023)
#define AW8623X_F0_FORMULA(code)		(384000 * 10 / (code))
#define AW8623X_RL_FORMULA(code, d2s_gain)	(((code) * 678 * 1000) / (1023 * (d2s_gain)))
#define AW8623X_OS_FORMULA(code, d2s_gain)	(2440 * ((code) - 512) / (1023 * (1 + d2s_gain)))
#define AW8623X_SET_AEADDR_H(addr)		((((addr) >> 1) >> 4) & 0xF0)
#define AW8623X_SET_AEADDR_L(addr)		(((addr) >> 1) & 0x00FF)
#define AW8623X_SET_AFADDR_H(addr)		((((addr) - ((addr) >> 2)) >> 8) & 0x0F)
#define AW8623X_SET_AFADDR_L(addr)		(((addr) - ((addr) >> 2)) & 0x00FF)
#define AW8623X_DRV2_LVL_FORMULA(f0, vrms)	((((f0) < 1800) ? 1809920 : 1990912) / 1000 * (vrms) / 61000)
#define AW8623X_DRV_WIDTH_FORMULA(f0, margin, brk_gain) \
						((240000 / (f0)) - (margin) - (brk_gain) - 8)

/*********************************************************
 *
 * Conditional Marco
 *
 *********************************************************/
#if LINUX_VERSION_CODE <= KERNEL_VERSION(4, 4, 1)
#define TIMED_OUTPUT
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0)
#define KERNEL_OVER_5_10
#endif

#ifdef TIMED_OUTPUT
#include <../../../drivers/staging/android/timed_output.h>
typedef struct timed_output_dev cdev_t;
#else
typedef struct led_classdev cdev_t;
#endif
/*********************************************************
 *
 * Log Format
 *
 *********************************************************/
#define aw_err(format, ...) \
	pr_err("[%s][%04d]%s: " format "\n", AW_I2C_NAME, __LINE__, __func__, ##__VA_ARGS__)

#define aw_info(format, ...) \
	pr_info("[%s][%04d]%s: " format "\n", AW_I2C_NAME, __LINE__, __func__, ##__VA_ARGS__)

#define aw_dbg(format, ...) \
	pr_debug("[%s][%04d]%s: " format "\n", AW_I2C_NAME, __LINE__, __func__, ##__VA_ARGS__)

/*********************************************************
 *
 * Enum Define
 *
 *********************************************************/
enum awrw_flag {
	AW_WRITE = 0,
	AW_READ = 1,
};

enum aw_haptic_flags {
	AW_FLAG_NONR = 0,
	AW_FLAG_SKIP_INTERRUPTS = 1,
};

enum aw_haptic_work_mode {
	AW_STANDBY_MODE = 0,
	AW_RAM_MODE = 1,
	AW_RAM_LOOP_MODE = 2,
	AW_CONT_MODE = 3,
	AW_RTP_MODE = 4,
	AW_TRIG_MODE = 5,
	AW_NULL = 6,
};

enum aw_haptic_cont_vbat_comp_mode {
	AW_CONT_VBAT_SW_COMP_MODE = 0,
	AW_CONT_VBAT_HW_COMP_MODE = 1,
};

enum aw_haptic_ram_vbat_comp_mode {
	AW_RAM_VBAT_COMP_DISABLE = 0,
	AW_RAM_VBAT_COMP_ENABLE = 1,
};

enum aw_haptic_pwm_mode {
	AW_PWM_48K = 0,
	AW_PWM_24K = 1,
	AW_PWM_12K = 2,
};

enum aw_haptic_play {
	AW_PLAY_NULL = 0,
	AW_PLAY_ENABLE = 1,
	AW_PLAY_STOP = 2,
	AW_PLAY_GAIN = 8,
};

enum aw_haptic_cmd {
	AW_CMD_NULL = 0,
	AW_CMD_ENABLE = 1,
	AW_CMD_HAPTIC = 0x0F,
	AW_CMD_TP = 0x10,
	AW_CMD_SYS = 0xF0,
	AW_CMD_STOP = 255,
};

enum aw_haptic_cali_lra {
	AW_WRITE_ZERO = 0,
	AW_F0_CALI_LRA = 1,
	AW_OSC_CALI_LRA = 2,
};

enum aw_haptic_awrw_flag {
	AW_SEQ_WRITE = 0,
	AW_SEQ_READ = 1,
};

enum aw_haptic_read_chip_type {
	AW_FIRST_TRY = 0,
	AW_LAST_TRY = 1,
};

enum aw_haptic_chip_name {
	AW_CHIP_NULL = 0,
	AW86223 = 1,
	AW86224 = 2,
	AW86225 = 3,
	AW86214 = 4,
	AW8624 = 5,
	AW86233 = 6,
	AW86234 = 7,
	AW86235 = 8,
};

enum aw_haptic_irq_state {
	AW_IRQ_ALMOST_EMPTY = 1,
};

enum aw_haptic_dts_data_type {
	AW_DTS_NULL = 0,
	AW_DTS_DATA_U32 = 1,
	AW_DTS_DATA_ARRAY = 2,
	AW_DTS_DATA_BOOL = 3,
};

enum aw_haptic_protect_config {
	AW_PROTECT_OFF = 0,
	AW_PROTECT_EN = 1,
	AW_PROTECT_CFG_1 = 0x2D,
	AW_PROTECT_CFG_2 = 0x3E,
	AW_PROTECT_CFG_3 = 0X3F,
};

enum aw_haptic_ram_file_num {
	AW8624_RAM_FILE = 0,
	AW8622X_RAM_FILE = 1,
	AW86214_RAM_FILE = 2,
	AW8623X_RAM_FILE = 3,
};

enum aw_haptic_pin {
	AW_TRIG1 = 0,
	AW_TRIG2 = 1,
	AW_TRIG3 = 2,
	AW_IRQ = 3,
};

enum aw_haptic_dts_name_flag {
	AW_MODE = 0,
	AW_F0_PRE = 1,
	AW_CONT_TD = 2,
	AW_F0_COEFF = 3,
	AW_D2S_GAIN = 4,
	AW_CONT_TSET = 5,
	AW_LK_F0_CALI = 6,
	AW_CONT_ZC_THR = 7,
	AW_CONT_NUM_BRK = 8,
	AW_CONT_DRV1_LVL = 9,
	AW_CONT_DRV2_LVL = 10,
	AW_CONT_WAIT_NUM = 11,
	AW_CONT_BRK_GAIN = 12,
	AW_CONT_BEMF_SET = 13,
	AW_CONT_BRK_TIME = 14,
	AW_CONT_DRV1_TIME = 15,
	AW_CONT_DRV2_TIME = 16,
	AW_F0_CALI_PERCENT = 17,
	AW_CONT_TRACK_MARGIN = 18,
	AW_LRA_VRMS = 19,
/*------------bool---------------*/
	AW_ENABLE_AUTO_BRK = 25,
	AW_ENABLE_TRACK_EN = 26,
/*------------array---------------*/
	AW_SW_BRAKE = 31,
	AW_SINE_ARRAY = 32,
	AW_TRIG_CONFIG = 33,
	AW_BEMF_CONFIG = 34,
	AW_DURATION_TIME = 35,
	AW_F0_TRACE_PARAM = 36,
	AW_BRAKE_CONT_CONFIG = 37,
};

/*********************************************************
 *
 * Enum aw8623x
 *
 *********************************************************/
enum aw8623x_sram_size_flag {
	AW8623X_HAPTIC_SRAM_1K = 0,
	AW8623X_HAPTIC_SRAM_2K = 1,
	AW8623X_HAPTIC_SRAM_3K = 2,
};

/*********************************************************
 *
 * Enum aw862xx
 *
 *********************************************************/
enum aw862xx_ef_id {
	AW86223_EF_ID = 0x01,
	AW86224_EF_ID = 0x00,
	AW86225_EF_ID = 0x00,
	AW86214_EF_ID = 0x41,
};

enum aw862xx_sram_size_flag {
	AW862XX_HAPTIC_SRAM_1K = 0,
	AW862XX_HAPTIC_SRAM_2K = 1,
	AW862XX_HAPTIC_SRAM_3K = 2,
};
/*********************************************************
 *
 * Enum aw8624
 *
 *********************************************************/
enum aw8624_pwm_clk {
	AW8624_CLK_24K = 2,
	AW8624_CLK_12K = 3,
};
/*********************************************************
 *
 * Struct Define
 *
 *********************************************************/
struct trig {
	uint8_t enable;
	uint8_t trig_edge;
	uint8_t trig_brk;
	uint8_t trig_level;
	uint8_t trig_polar;
	uint8_t pos_enable;
	uint8_t neg_enable;
	uint8_t pos_sequence;
	uint8_t neg_sequence;
};

struct aw_haptic_ram {
	uint8_t ram_num;
	uint8_t version;
	uint8_t ram_shift;
	uint8_t baseaddr_shift;

	uint32_t len;
	uint32_t check_sum;
	uint32_t base_addr;
};

struct aw_haptic_ctr {
	uint8_t cnt;
	uint8_t cmd;
	uint8_t play;
	uint8_t loop;
	uint8_t gain;
	uint8_t wavseq;
	struct list_head list;
};

struct aw_i2c_info {
	uint8_t flag;
	uint8_t reg_num;
	uint8_t first_addr;
	uint8_t reg_data[AWRW_SIZE];
};

struct aw_haptic_audio {
	int delay_val;
	int timer_val;
	struct mutex lock;
	struct hrtimer timer;
	struct list_head list;
	struct work_struct work;
	struct aw_haptic_ctr ctr;
	struct list_head ctr_list;
};

struct aw_haptic_dts_data {
	void *dts_val;
	uint8_t type;
	uint32_t len;
};


struct aw_haptic_dts_name {
	uint8_t *name;
	uint8_t flag;
};

struct aw_haptic_dts_info {
	bool is_enabled_auto_brk;
	bool is_enabled_track_en;

	uint32_t mode;
	uint32_t f0_cali_percent;
	uint32_t f0_pre;
	uint32_t lra_vrms;
	uint32_t lk_f0_cali;
	uint32_t cont_tset;
	uint32_t cont_drv1_lvl;
	uint32_t cont_drv2_lvl;
	uint32_t duration_time[3];
	uint32_t trig_cfg[21];
	/* aw8624 */
	uint32_t cont_td;
	uint32_t cont_zc_thr;
	uint32_t cont_num_brk;
	uint32_t f0_coeff;
	uint32_t cont_brake[24];
	uint32_t bemf_config[4];
	uint32_t sw_brake[2];
	uint32_t wavseq[16];
	uint32_t wavloop[10];
	uint32_t td_brake[3];
	uint32_t f0_trace_parameter[4];
	/* aw862xx */
	uint32_t cont_drv1_time;
	uint32_t cont_drv2_time;
	uint32_t cont_wait_num;
	uint32_t cont_brk_time;
	uint32_t cont_track_margin;
	uint32_t cont_bemf_set;
	uint32_t cont_brk_gain;
	uint32_t d2s_gain;
	uint32_t prctmode[3];
	uint32_t sine_array[4];
};

struct aw_haptic {
	bool rtp_init;
	bool rtp_loop;
	bool ram_init;
	bool is_used_irq_pin;
	bool is_supported_rtp;
	bool is_supported_trig;

	uint8_t flags;
	uint8_t play_mode;
	uint8_t chipid_flag;
	uint8_t dts_name_len;
	uint8_t max_pos_beme;
	uint8_t max_neg_beme;
	uint8_t activate_mode;
	uint8_t ram_vbat_comp;
	uint8_t seq[AW_SEQUENCER_SIZE];
	uint8_t loop[AW_SEQUENCER_SIZE];

	int name;
	int gain;
	int state;
	int index;
	int irq_gpio;
	int duration;
	int effect_id;
	int amplitude;
	int reset_gpio;

	uint32_t f0;
	uint32_t lra;
	uint32_t vbat;
	uint32_t level;
	uint32_t f0_pre;
	uint32_t cont_f0;
	uint32_t rtp_cnt;
	uint32_t rtp_len;
	uint32_t gun_type;
	uint32_t bullet_nr;
	uint32_t theory_time;
	uint32_t interval_us;
	uint32_t f0_cali_data;
	uint32_t rtp_file_num;
	uint32_t ram_file_num;
	uint32_t timeval_flags;
	uint32_t osc_cali_data;
	uint32_t aw862xx_i2c_addr;
	uint64_t microsecond;

	cdev_t vib_dev;

	ktime_t kend;
	ktime_t kstart;
	ktime_t current_t;
	ktime_t pre_enter_t;

	struct mutex lock;
	struct device *dev;
	struct hrtimer timer;
	struct mutex rtp_lock;
	struct i2c_client *i2c;
	struct aw_haptic_ram ram;
	struct aw_haptic_func *func;
	struct aw_i2c_info i2c_info;
	struct input_dev *input_dev;
	struct work_struct rtp_work;
	struct delayed_work ram_work;
	struct trig trig[AW_TRIG_NUM];
	struct aw_haptic_dts_info info;
	struct work_struct vibrator_work;
	struct work_struct input_gain_work;
	struct work_struct input_vib_work;
	struct aw_haptic_container *aw_rtp;
	struct aw_haptic_audio haptic_audio;
	struct workqueue_struct *work_queue;
	struct aw_haptic_dts_name *dts_name_array;
	struct pm_qos_request aw_pm_qos_req_vb;
};

struct aw_haptic_container {
	int len;
	uint8_t data[];
};

struct aw_haptic_func {
	int (*check_qualify)(struct aw_haptic *);
	int (*container_update)(struct aw_haptic *,
				struct aw_haptic_container *);
	int (*get_irq_state)(struct aw_haptic *);
	int (*get_f0)(struct aw_haptic *);
	void (*creat_node)(struct aw_haptic *);
	void (*read_cont_f0)(struct aw_haptic *);
	void (*trig_init)(struct aw_haptic *);
	void (*irq_clear)(struct aw_haptic *);
	void (*haptic_start)(struct aw_haptic *);
	void (*play_stop)(struct aw_haptic *);
	void (*play_mode)(struct aw_haptic *, uint8_t);
	void (*cont_config)(struct aw_haptic *);
	void (*offset_cali)(struct aw_haptic *);
	void (*ram_init)(struct aw_haptic *, bool);
	void (*misc_para_init)(struct aw_haptic *);
	void (*interrupt_setup)(struct aw_haptic *);
	void (*vbat_mode_config)(struct aw_haptic *, uint8_t);
	void (*protect_config)(struct aw_haptic *, uint8_t, uint8_t);
	void (*calculate_cali_data)(struct aw_haptic *);
	void (*set_gain)(struct aw_haptic *, uint8_t);
	void (*get_gain)(struct aw_haptic *, uint8_t *);
	void (*set_wav_seq)(struct aw_haptic *, uint8_t, uint8_t);
	void (*get_wav_seq)(struct aw_haptic *, uint32_t);
	void (*set_wav_loop)(struct aw_haptic *, uint8_t, uint8_t);
	void (*get_wav_loop)(struct aw_haptic *, uint8_t *);
	void (*set_rtp_data)(struct aw_haptic *, uint8_t *, uint32_t);
	void (*set_fifo_addr)(struct aw_haptic *);
	void (*get_fifo_addr)(struct aw_haptic *);
	void (*set_ram_data)(struct aw_haptic *, uint8_t *, int);
	void (*get_ram_data)(struct aw_haptic *, uint8_t *, int);
	void (*set_ram_addr)(struct aw_haptic *);
	void (*set_repeat_seq)(struct aw_haptic *, uint8_t);
	void (*set_base_addr)(struct aw_haptic *);
	void (*set_trim_lra)(struct aw_haptic *, uint8_t);
	void (*set_rtp_aei)(struct aw_haptic *, bool);
	void (*get_vbat)(struct aw_haptic *);
	void (*get_lra_resistance)(struct aw_haptic *);
	void (*get_first_wave_addr)(struct aw_haptic *, uint32_t *);
	ssize_t (*get_reg)(struct aw_haptic *, ssize_t, char *);
	uint8_t (*get_prctmode)(struct aw_haptic *);
	uint8_t (*get_glb_state)(struct aw_haptic *);
	uint8_t (*get_osc_status)(struct aw_haptic *);
	uint8_t (*judge_rtp_going)(struct aw_haptic *);
	uint8_t (*rtp_get_fifo_afs)(struct aw_haptic *);
	uint64_t (*get_theory_time)(struct aw_haptic *);
};
/*********************************************************
 *
 * Function Call
 *
 *********************************************************/
#ifdef AW8624_DRIVER
extern struct aw_haptic_func aw8624_func_list;
#endif

#ifdef AW862XX_COMMON
extern struct aw_haptic_func aw862xx_func_list;
#endif

#ifdef AW8623X_DRIVER
extern struct aw_haptic_func aw8623x_func_list;
#endif

extern int haptic_nv_i2c_reads(struct aw_haptic *, uint8_t, uint8_t *, uint32_t);
extern int haptic_nv_i2c_writes(struct aw_haptic *, uint8_t, uint8_t *, uint32_t);
extern int haptic_nv_i2c_write_bits(struct aw_haptic *, uint8_t, uint32_t, uint8_t);
extern ssize_t haptic_nv_read_reg_array(struct aw_haptic *, char *, ssize_t,
					uint8_t, uint8_t);
#endif
