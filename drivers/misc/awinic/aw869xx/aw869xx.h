/*********************************************************
 *
 * aw869xx.h
 *
 ********************************************************/
#ifndef _AW869XX_H_
#define _AW869XX_H_

#include <linux/regmap.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/hrtimer.h>
#include <linux/mutex.h>
#include <linux/cdev.h>
#include <linux/version.h>
#include <sound/control.h>
#include <sound/soc.h>
/*********************************************************
 *
 * Conditional Compilation Marco
 *
 ********************************************************/
#if LINUX_VERSION_CODE <= KERNEL_VERSION(4, 4, 1)
#define TIMED_OUTPUT
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 1)
#define AW_KERNEL_VER_OVER_4_19
#endif

#ifdef AW_KERNEL_VER_OVER_4_19
typedef struct snd_soc_component aw_snd_soc_codec_t;
typedef struct snd_soc_component_driver aw_snd_soc_codec_driver_t;
#else
typedef struct snd_soc_codec aw_snd_soc_codec_t;
typedef struct snd_soc_codec_driver aw_snd_soc_codec_driver_t;
#endif

struct aw_componet_codec_ops {
	aw_snd_soc_codec_t *(*aw_snd_soc_kcontrol_codec) (struct snd_kcontrol *kcontrol);
	void *(*aw_snd_soc_codec_get_drvdata) (aw_snd_soc_codec_t *codec);
	int (*aw_snd_soc_add_codec_controls) (aw_snd_soc_codec_t *codec,
					      const struct snd_kcontrol_new *controls,
					      unsigned int num_controls);
	void (*aw_snd_soc_unregister_codec) (struct device *dev);
	int (*aw_snd_soc_register_codec) (struct device *dev,
					  const aw_snd_soc_codec_driver_t *codec_drv,
					  struct snd_soc_dai_driver *dai_drv,
					  int num_dai);
};

#ifdef TIMED_OUTPUT
#include <../../../drivers/staging/android/timed_output.h>
typedef struct timed_output_dev cdev_t;
#else
typedef struct led_classdev cdev_t;
#endif

/* #define AW_F0_COARSE_CALI */
/* #define AW_OSC_COARSE_CALI */
/* #define AW_RAM_STATE_OUTPUT */
/* #define AW_CHECK_RAM_DATA */
#define AW_READ_BIN_FLEXBALLY
/*********************************************************
 *
 * Normal Marco
 *
 ********************************************************/
#define AW86905_CHIPID			0x05
#define AW86907_CHIPID			0x04
#define AW86915_CHIPID			0x07
#define AW86917_CHIPID			0x06
#define AW869XX_I2C_NAME		"awinic_haptic"
#define AW869XX_DEV_NAME		"aw869xx_smartpa"
#define AW869XX_RATES			SNDRV_PCM_RATE_8000_48000
#define AW869XX_FORMATS			(SNDRV_PCM_FMTBIT_S16_LE | \
					SNDRV_PCM_FMTBIT_S24_LE | \
					SNDRV_PCM_FMTBIT_S32_LE)
#define AW_I2C_RETRIES			5
#define AW_I2C_RETRY_DELAY		2
#define AW_READ_CHIPID_RETRIES		5
#define AW_READ_CHIPID_RETRY_DELAY	2
#define OSC_CALI_MAX_LENGTH		5100000
#define PM_QOS_VALUE_VB			400
#define AW869XX_RTP_NAME_MAX		64
#define AW869XX_SEQUENCER_SIZE		8
#define AW869XX_SEQUENCER_LOOP_SIZE	4
#define AW869XX_VBAT_REFER		4200
#define AW869XX_VBAT_MIN		3000
#define AW869XX_VBAT_MAX		5500
#define AW869XX_FIFO_SIZE		2048
#define AE_THRESHOLD			1024
#define AF_THRESHOLD			1536
#define AW869XX_TRIG_NUM		3

/********************************************
 * print information control
 *******************************************/
#define aw_dev_err(dev, format, ...) \
			pr_err("[%s]" format, dev_name(dev), ##__VA_ARGS__)

#define aw_dev_info(dev, format, ...) \
			pr_info("[%s]" format, dev_name(dev), ##__VA_ARGS__)

#define aw_dev_dbg(dev, format, ...) \
			pr_debug("[%s]" format, dev_name(dev), ##__VA_ARGS__)

/*********************************************************
 *
 * Enum Define
 *
 ********************************************************/
enum aw869xx_flags {
	AW869XX_FLAG_NONR = 0,
	AW869XX_FLAG_SKIP_INTERRUPTS = 1,
};

enum aw869xx_haptic_read_write {
	AW869XX_HAPTIC_CMD_READ_REG = 0,
	AW869XX_HAPTIC_CMD_WRITE_REG = 1,
};

enum aw869xx_haptic_work_mode {
	AW869XX_HAPTIC_STANDBY_MODE = 0,
	AW869XX_HAPTIC_RAM_MODE = 1,
	AW869XX_HAPTIC_RTP_MODE = 2,
	AW869XX_HAPTIC_TRIG_MODE = 3,
	AW869XX_HAPTIC_CONT_MODE = 4,
	AW869XX_HAPTIC_RAM_LOOP_MODE = 5,
};

enum aw869xx_haptic_bst_pc {
	AW869XX_HAPTIC_BST_PC_L1 = 0,
	AW869XX_HAPTIC_BST_PC_L2 = 1,
};

enum aw869xx_haptic_activate_mode {
	AW869XX_HAPTIC_ACTIVATE_RAM_MODE = 0,
	AW869XX_HAPTIC_ACTIVATE_CONT_MODE = 1,
};

enum aw869xx_haptic_cont_vbat_comp_mode {
	AW869XX_HAPTIC_CONT_VBAT_SW_ADJUST_MODE = 0,
	AW869XX_HAPTIC_CONT_VBAT_HW_ADJUST_MODE = 1,
};

enum aw869xx_haptic_ram_vbat_compensate_mode {
	AW869XX_HAPTIC_RAM_VBAT_COMP_DISABLE = 0,
	AW869XX_HAPTIC_RAM_VBAT_COMP_ENABLE = 1,
};

enum aw869xx_haptic_f0_flag {
	AW869XX_HAPTIC_LRA_F0 = 0,
	AW869XX_HAPTIC_CALI_F0 = 1,
};

enum aw869xx_haptic_pwm_mode {
	AW869XX_PWM_48K = 0,
	AW869XX_PWM_24K = 1,
	AW869XX_PWM_12K = 2,
};

enum aw869xx_haptic_play {
	AW869XX_HAPTIC_PLAY_NULL = 0,
	AW869XX_HAPTIC_PLAY_ENABLE = 1,
	AW869XX_HAPTIC_PLAY_STOP = 2,
	AW869XX_HAPTIC_PLAY_GAIN = 8,
};

enum aw869xx_haptic_cmd {
	AW869XX_HAPTIC_CMD_NULL = 0,
	AW869XX_HAPTIC_CMD_ENABLE = 1,
	AW869XX_HAPTIC_CMD_HAPTIC = 0x0f,
	AW869XX_HAPTIC_CMD_TP = 0x10,
	AW869XX_HAPTIC_CMD_SYS = 0xf0,
	AW869XX_HAPTIC_CMD_STOP = 255,
};

enum aw869xx_haptic_bst_mode {
	AW869XX_HAPTIC_BST_MODE_BYPASS = 0,
	AW869XX_HAPTIC_BST_MODE_BOOST = 1,
};

enum aw869xx_haptic_cali_lra {
	WRITE_ZERO = 0,
	F0_CALI = 1,
	OSC_CALI = 2,
};

/*********************************************************
 *
 * Struct Define
 *
 ********************************************************/
struct ram {
	unsigned int len;
	unsigned int check_sum;
	unsigned int base_addr;
	unsigned char version;
	unsigned char ram_shift;
	unsigned char baseaddr_shift;
};

struct haptic_ctr {
	unsigned char cnt;
	unsigned char cmd;
	unsigned char play;
	unsigned char wavseq;
	unsigned char loop;
	unsigned char gain;
	struct list_head list;
};

/* trig_config
 * trig default high level
 * ___________           ___________
 *           |           |
 *           |           |
 *           |___________|
 *        first edge
 *                   second edge
 *
 * trig default low level
 *            ___________
 *           |           |
 *           |           |
 * __________|           |__________
 *        first edge
 *                   second edge
 ******************** vib_trig_config *********************
 *     level polar pos_en pos_seq neg_en neg_seq brk bst
 trig1*  1     0     1       1       1      2     0   0
 trig2*  1     0     0       1       0      2     0   0
 trig3*  1     0     0       1       0      2     0   0
*/
struct trig {
	unsigned char trig_level;
	unsigned char trig_polar;
	unsigned char pos_enable;
	unsigned char pos_sequence;
	unsigned char neg_enable;
	unsigned char neg_sequence;
	unsigned char trig_brk;
	unsigned char trig_bst;
};

struct aw869xx_dts_info {
	unsigned int mode;
	unsigned int f0_ref;
	unsigned int f0_cali_percent;
	unsigned int cont_drv1_lvl_dt;
	unsigned int cont_drv2_lvl_dt;
	unsigned int cont_drv1_time_dt;
	unsigned int cont_drv2_time_dt;
	unsigned int cont_wait_num_dt;
	unsigned int cont_brk_time_dt;
	unsigned int cont_track_margin;
	unsigned int cont_tset;
	unsigned int cont_drv_width;
	unsigned int cont_bemf_set;
	unsigned int cont_brk_gain;
	unsigned int cont_bst_brk_gain;
	unsigned int d2s_gain;
	unsigned int bstcfg[5];
	unsigned int prctmode[3];
	unsigned int sine_array[4];
	unsigned int trig_config[24];
	bool is_enabled_auto_bst;
	bool is_enabled_i2s;
	bool is_enabled_powerup_f0_cali;
};

struct aw869xx {
	struct regmap *regmap;
	struct i2c_client *i2c;
	/*struct snd_soc_codec *codec; */
	aw_snd_soc_codec_t *codec;
	struct device *dev;
	struct input_dev *input;
	struct mutex lock;
	struct mutex rtp_lock;
	struct hrtimer timer;
	struct work_struct long_vibrate_work;
	struct work_struct rtp_work;
	struct delayed_work ram_work;
	struct trig trig[AW869XX_TRIG_NUM];
	struct aw869xx_dts_info dts_info;
	struct ram ram;
	struct timeval start, end;

	cdev_t vib_dev;

	bool haptic_ready;
	bool audio_ready;

	unsigned char seq[AW869XX_SEQUENCER_SIZE];
	unsigned char loop[AW869XX_SEQUENCER_SIZE];
	unsigned char rtp_init;
	unsigned char ram_init;
	unsigned char rtp_routine_on;
	unsigned char max_pos_beme;
	unsigned char max_neg_beme;
	unsigned char f0_cali_flag;
	unsigned char ram_vbat_compensate;
	unsigned char hwen_flag;
	unsigned char flags;
	unsigned char chipid;
	unsigned char play_mode;
	unsigned char activate_mode;
	unsigned char auto_boost;
	unsigned char boost_mode;
	unsigned char bst_pc;

	int reset_gpio;
	int irq_gpio;
	int state;
	int duration;
	int amplitude;
	int index;
	int vmax;
	int gain;
	int sysclk;
	int rate;
	int width;
	int pstream;
	int cstream;

	unsigned int rtp_cnt;
	unsigned int rtp_file_num;
	unsigned int f0;
	unsigned int cont_f0;
	unsigned int cont_drv1_lvl;
	unsigned int cont_drv2_lvl;
	unsigned int cont_brk_time;
	unsigned int cont_wait_num;
	unsigned int cont_drv1_time;
	unsigned int cont_drv2_time;
	unsigned int theory_time;
	unsigned int vbat;
	unsigned int lra;
	unsigned int ram_update_flag;
	unsigned int rtp_update_flag;
	unsigned int osc_cali_data;
	unsigned int f0_cali_data;
	unsigned int timeval_flags;
	unsigned int osc_cali_flag;
	unsigned int sys_frequency;
	unsigned int rtp_len;
	unsigned long int microsecond;
};

struct aw869xx_container {
	int len;
	unsigned char data[];
};

#endif
