#define pr_fmt(fmt) "drv8424: %s: " fmt, __func__

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/version.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/debugfs.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/wait.h>
#include <linux/semaphore.h>
#include <uapi/linux/sched/types.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#include <linux/kfifo.h>
#include <linux/clk-provider.h>

#define DEFAULT_STEP_FREQ 2400
#define MOTOR_CLASS_NAME  "drv8424"
#define MOTOR_CONTROL  "control"
#define MOTOR_HW_CLK         9600 //9.6KHz clock
#define MOTOR_HW_CLK_NAME "gcc_gp3"

#define MOTOR_DEFAULT_EXPIRE 5000000 //5s timeout
#define MOTOR_DETECT_EXPIRE 3000000 //3s timeout
#define INT_20MS 20

#define MOTOR_MODE_SPEED 0
#define MOTOR_MODE_STEP 1

#define DIR_EXTEND 1
#define DIR_WITHDRAW 0

#define RESET_TIME 1000

#define UNINITIALIZED -10000
#define OUT_OF_RANGE 10000

#define LOGD(fmt, args...) pr_err(fmt, ##args)
#define PARANOIC(fmt, args...) pr_debug(fmt, ##args)
#undef MOTOR_SLOT_DBG

enum position_id {
	POS_UNKNOWN = 0,
	POS_COMPACT,
	POS_EXPANDED,
	POS_PEEK,
	POS_MAX,
};

static const char *position_labels[] = {
	"UNKNOWN",
	"COMPACT",
	"EXPANDED",
	"PEEK",
};

#define ARGS_NUM_MIN 2
typedef struct {
	int index;
	int value;
} sensor_scan_t;

/*
 * Sensor data defined in the range from -5000 to 5000 that corresponds
 * to the range of angles from -50 degrees to 50 degrees.
 * Each value represented by float with 2 digits precision and then
 * multiplied by 100 to fit decimal range above.
 * Everything is just for the reference purpose and will be
 * adjusted as more details become available
 */
static sensor_scan_t scanner[POS_MAX] = {
	{-1, -1},
	{1, 0},	/* compact position: magnet[1], value 0 degrees */
	{0, 0},	/* expanded position: magnet[0], value 0 degrees */
	{2, 0},	/* peek position: magnet[2], value 0 degrees */
};

enum status_id {
	STATUS_UNKNOWN,
	STATUS_STOPPED_COMPACT,
	STATUS_STOPPED_EXPANDED,
	STATUS_STOPPED_PEEK,
	STATUS_MOVING_OUT,
	STATUS_MOVING_IN,
	STATUS_QUERYING_POS,
	STATUS_FAULTING,
};

static const char *status_labels[] = {
	"UNKNOWN",
	"STOPPED_COMPACT",
	"STOPPED_EXPANDED",
	"STOPPED_PEEK",
	"EXPANDING",
	"WITHDRAWING",
	"QUERYING_POSITION",
	"FAULTING",
};

enum gpios_index {
	MOTOR_POWER_EN = 0, /* BOOST */
	MOTOR_FAULT1_INT,	/* nFAULT1 INT */
	MOTOR_FAULT2_INT,	/* nFAULT2 INT */
	MOTOR_STEP,			/* STEP */
	MOTOR_DIR,			/* DIR */
	MOTOR_MODE1,		/* set to 0 in HW */
	MOTOR_MODE2,		/* set to 0 in HW */
	MOTOR_EN,			/* set to 1 in HW */
	MOTOR_SLEEP,		/* nSLEEP */
	MOTOR_T0,			/* set to 0 in HW */
	MOTOR_T1,			/* set to 0 in HW */
	MOTOR_ACTIVE,		/* signal GPIO */
	MOTOR_MAX_GPIO,
	MOTOR_UNKNOWN = MOTOR_MAX_GPIO
};

static const char* const gpios_labels[] = {
	"MOTOR_POWER_EN",
	"MOTOR_FAULT1_INT",
	"MOTOR_FAULT2_INT",
	"MOTOR_STEP",
	"MOTOR_DIR",
	"MOTOR_MODE1",
	"MOTOR_MODE2",
	"MOTOR_EN",
	"MOTOR_SLEEP",
	"MOTOR_T0",
	"MOTOR_T1",
	"MOTOR_ACTIVE",
	"MOTOR_UNKNOWN"
};

static const unsigned long hw_clocks[] = {
	DEFAULT_STEP_FREQ,      //0,0        software clock
	4800,                   //0,Z        2*2400
	9600,                   //0,1        4*2400
	19200,                  //Z,0        8*2400
	9600,                   //Z,Z        4*2400
	38400,                  //Z,1        16*2400
	19200,                  //1,0        8*2400
	76800,                  //1,Z        32*2400
	38400,                  //1,1        16*2400
};

enum motor_step_mode {
	FULL_STEP = 0,          //0,0
	STEP_2_RISING,          //0,Z
	STEP_4_RISING,          //0,1
	STEP_8_RISING,          //Z,0
	STEP_8_BOTH_EDEG,       //Z,Z
	STEP_16_RISING,         //Z,1
	STEP_16_BOTH_EDGE,      //1,0
	STEP_32_RISING,         //1,Z
	STEP_32_BOTH_EDEG,      //1,1
};

enum motor_torque {
	TORQUE_FULL = 0,        //0,0
	TORQUE_87,              //0,Z
	TORQUE_75,              //0,1
	TORQUE_62,              //Z,0
	TORQUE_50,              //Z,Z
	TORQUE_37,              //Z,1
	TORQUE_25,              //1,0
	TORQUE_12,              //1,Z
	TORQUE_DISABLE,         //1,1
};

//Keep in sequence with dtsi.
enum motor_pins_mode {
	MODE0_LOW = 0,
	MODE0_HIGH,
	MODE0_DISABLE,
	MODE1_LOW,
	MODE1_HIGH,
	MODE1_DISABLE,
	T0_LOW,
	T0_HIGH,
	T0_DISABLE,
	T1_LOW,
	T1_HIGH,
	T1_DISABLE,
	INT_DEFAULT,
	CLK_ACTIVE,
	CLK_SLEEP,
	INIT_STATE,
	PINS_END
};

//Keep in same name with the pinctrl-names of dtsi
static const char* const pins_state[] = {
	"m0_low", "m0_high", "m0_disable",
	"m1_low", "m1_high", "m1_disable",
	"t0_low", "t0_high", "t0_disable",
	"t1_low", "t1_high", "t1_disable",
	"drv8424_int_default",
	"drv8424_clk_active", "drv8424_clk_sleep",
	"init_state",
	NULL
};

enum regime_idx {
	SQ_FULL,
	SQ_PROLONGED,
	SQ_SHORT,
	SQ_TINY,
	SQ_SHOW_MAX,
	SQ_SLOWMO,
	SQ_MAX,
};

static const char *regime_names[] = {
	"FULL",
	"PROLONGED",
	"SHORT",
	"TINY",
	"dummy",
	"SLOWMO"
};

/*             PPS Pulses
	stage 1:   400   24
	stage 2:  1400   56
	stage 3:  3000  6252
	stage 4:  2000   12
*/
#define MAX_STAGE_LEGS 10
typedef struct {
	unsigned freq, ceiling;
} motor_stage;

/* Default sequences for 2.3pitch */
static motor_stage initial_data_2p3[SQ_MAX][MAX_STAGE_LEGS] = {
	{ /* FULL 39mm */
		{600,60}, {1000,60}, {1400,60}, {1800,4680},
	},
	{ /* PROLONGED 44mm */
		{600,60}, {1000,60}, {1400,60}, {1800,5680},
	},
	{ /* SHORT 5mm */
		{600,60}, {1000,770},
	},
	{ /* TINY 1mm */
		{600,60}, {1000,68},
	},
	{ /* dummy to continue beyond SQ_SHOW_MAX */
		{0, 0},
	},
	{ /* SLOWMO */
		{600,4900},
	},
};

/* Sequences for 2.8pitch */
static motor_stage initial_data_2p8[SQ_MAX][MAX_STAGE_LEGS] = {
	{ /* FULL 39mm */
		{600,60}, {1000,60}, {1400,60}, {1800,3840},
	},
	{ /* PROLONGED 44mm */
		{600,60}, {1000,60}, {1400,60}, {1800,4460},
	},
	{ /* SHORT 5mm */
		{600,60}, {1000,600},
	},
	{ /* TINY 1mm */
		{600,108},
	},
	{ /* dummy to continue beyond SQ_SHOW_MAX */
		{0, 0},
	},
	{ /* SLOWMO */
		{600,4100},
	},
};
/*                                10,000,000 10,000,000 4,000,000 2,000,000   15,000,000 */
//static unsigned timeoutMicroS[] = { 10000000,  13000000,  4000000,  2000000, 0, 15000000 };

enum cmds {
	CMD_FAULT,
	CMD_STATUS,
	CMD_POSITION,
	CMD_POLL,
	CMD_TIMEOUT,
	CMD_RECOVERY,
	CMD_TIMER_START,
	CMD_TIMER_STOP,
};

#define MAX_GPIOS MOTOR_UNKNOWN
typedef struct motor_control {
	struct regulator    *vdd;
	int32_t ptable[MOTOR_MAX_GPIO];
	size_t tab_cells;
	struct pinctrl* pins;
	struct pinctrl_state *pin_state[PINS_END];
	const char* const * plabels;
} motor_control;

#define CLOCK_NAME_LEN 16
typedef struct motor_device {
	char clock_name[CLOCK_NAME_LEN];
	struct device*  dev;
	struct device*  sysfs_dev;
	struct class*   drv8424_class;
	struct workqueue_struct* motor_wq;
	struct clk * pwm_clk;
	struct delayed_work motor_work;
	struct task_struct *motor_task;
	wait_queue_head_t sync_complete;
	wait_queue_head_t status_wait;
	wait_queue_head_t position_wait;
	struct hrtimer stepping_timer;
	struct hrtimer timeout_timer;
	motor_control mc;
	spinlock_t mlock;
	struct mutex mx_lock;
	atomic_t fault_irq;
	bool faulting;
	bool use_2p8_pitch;
	unsigned step_freq;
	unsigned long step_period;
	unsigned long step_ceiling;
	unsigned long cur_clk;
	atomic_t step_count;
	atomic_t stepping;
	atomic_t status;
	atomic_t position;
	atomic_t destination;
	unsigned slot;
	unsigned mode, cur_mode;
	unsigned torque;
	unsigned time_out;
	unsigned half;
	unsigned stage, max_stages;
	unsigned regime;
	motor_stage sequencer[SQ_MAX][MAX_STAGE_LEGS];
	bool     double_edge;
	bool     hw_clock;
	bool     power_default_off;
	bool     support_mode;
	bool     support_torque;
	bool     sensors_off;
	bool     irq_enabled;
	bool     do_not_arm_irq;
	int      level:1;
	unsigned power_en:1;
	unsigned nsleep:1;
	unsigned nEN:1;
	unsigned dir:1;
	unsigned user_sync_complete:1;
	unsigned status_update_ready:1;
	unsigned position_update_ready:1;
	struct kfifo cmd_pipe;
	int volatile sensor_data[3];
	atomic_t samples;
	struct semaphore data_ready;
	wait_queue_head_t data_wait;
	unsigned data_update_ready:1;
	struct task_struct *detection_task;
	bool     cancel_thread;
	bool     detect_incomplete;
	bool     ready;
} motor_device;

#define RESET_SENSOR_DATA(data) do { \
		md->sensor_data[0] = md->sensor_data[1] = \
		md->sensor_data[2] = data; \
	} while(0)

#define GPIO_OUTPUT_DIR(g, p) do { \
		if (gpio_is_valid(g)) \
			gpio_direction_output(g, p); \
	} while(0)

static int set_pinctrl_state(motor_device* md, unsigned state_index);
static void moto_drv8424_set_step_freq(motor_device* md, unsigned freq);
static void motor_set_sequencer(motor_device* md);

#ifdef MOTOR_SLOT_DBG
#define RECORD_TIME(t) \
	do { \
		/* record earliest time only */ \
		if (!t) \
			t = ktime_get(); \
	} while(0)

typedef struct {
	ktime_t kTime;
	void *tData;
} timed_data;

static timed_data ArrayData[200];
static timed_data ArrayOut[200];
static unsigned	max_slot = 200;
static unsigned slotD, slotP;
static DEFINE_MUTEX(logtime_mutex);
static ktime_t tZero, kTerm, kKill, kClean, kDetect, kSensorUp, kSensorDown;
static ktime_t kStepping[15000];
static int steppingIdx;

static void logtime_alloc(int esize)
{
	int i;
	for (i = 0; i < max_slot; i++) {
		ArrayData[i].tData = kzalloc(esize, GFP_KERNEL);
		ArrayOut[i].tData = kzalloc(esize, GFP_KERNEL);
		if (ArrayData[i].tData == NULL ||
			ArrayOut[i].tData == NULL) {
			max_slot = i;
			break;
		}
	}
	pr_err("max slot set: %u\n", max_slot);
}


static void inline logtime_stepping_store(void)
{

	kStepping[steppingIdx++] = ktime_get();
	if (steppingIdx >= ARRAY_SIZE(kStepping))
		steppingIdx--;
}

static void logtime_store(void *ptr, int esize, int t)
{
	timed_data *dptr;
	unsigned *cptr;

	if (!t) {
		cptr = &slotD;
		dptr = &ArrayData[*cptr];
	} else {
		cptr = &slotP;
		dptr = &ArrayOut[*cptr];
	}

	if ((*cptr + 1) < max_slot) {
		dptr->kTime = ktime_get();
		memcpy(dptr->tData, ptr, esize);
		*cptr = *cptr + 1;
	}
}

static void logtime_reset(void)
{
	int i;
	size_t esize = sizeof(int)*3;

	slotD = slotP = 0U;
	for (i = 0; i < max_slot; i++) {
		ArrayData[i].kTime = ArrayOut[i].kTime = (ktime_t)0;
		memset(ArrayData[i].tData, 0, esize);
		memset(ArrayOut[i].tData, 0, esize);
	}
	tZero = kDetect = kTerm = kKill = kClean = kSensorUp = kSensorDown = (ktime_t)0;

	for (i = 0; i < ARRAY_SIZE(kStepping); i++)
		kStepping[i] = (ktime_t)0;
	steppingIdx = 0;
}

#define DETECT 0
#define TERMINATE 1
#define KILL 2
#define CLEAN 3
#define SENS_UP 4
#define SENS_DOWN 5

#define ZERO_CHK(t) (!t ? t : ktime_ms_delta(t, tZero))

#define LAST_STEPPING_TIMES 20

static void logtime_show(const char *f)
{
	int *iptr, *optr, i, o, endIdx;
	ktime_t tdiff;
	ktime_t td[10];

	mutex_lock(&logtime_mutex);
	endIdx = steppingIdx;
	pr_err("Called from: %s; Available %u sample(s) zero time %llu\n", f, slotD, tZero);
	if (!slotD)
		return;
	for (i = 0; i < slotD; i++) {
		iptr = (int *)ArrayData[i].tData;
		o = i;
ff:
		optr = (int *)ArrayOut[o].tData;
		if (*optr != 0) {
			if (!(*iptr == *optr &&
				*(iptr+1) == *(optr+1) &&
				*(iptr+2) == *(optr+2))) {
				if (++o > (slotD + 1))
					break;
				goto ff;
			}
		}
		tdiff = ktime_us_delta(ArrayOut[o].kTime, ArrayData[i].kTime);
		pr_err("%d|%d) delta:%lldus <%d %d %d> [%d %d %d] {%llu-%llu}\n", i, o, tdiff,
			*iptr, *(iptr+1), *(iptr+2),
			*optr, *(optr+1), *(optr+2),
			ArrayData[i].kTime,
			ArrayOut[o].kTime);
	}
	td[SENS_UP] = ZERO_CHK(kSensorUp);
	td[DETECT] = ZERO_CHK(kDetect);
	td[SENS_DOWN] = ZERO_CHK(kSensorDown);
	td[TERMINATE] = ZERO_CHK(kTerm);
	td[KILL] = ZERO_CHK(kKill);
	td[CLEAN] = ZERO_CHK(kClean);
	pr_err("TZ=%llu SensUp=%llu(%lldms), Detect=%llu(%lldms), SensDown=%llu(%lldms), Term=%llu(%lldms), Kill=%llu(%lldms), Clean=%llu(%lldms)\n",
		tZero,
		kSensorUp, td[SENS_UP],
		kDetect, td[DETECT],
		kSensorDown, td[SENS_DOWN],
		kTerm, td[TERMINATE],
		kKill, td[KILL],
		kClean, td[CLEAN]);
	/* print the last 2o stepping times */
	pr_err("logged %d steppings; the last sample @%llu\n", endIdx, ArrayData[slotD-1].kTime);
	if (endIdx > LAST_STEPPING_TIMES) {
		for (i = endIdx - LAST_STEPPING_TIMES; i < endIdx; i++) {
			/* print only if pulse happened after the last
			 * registered sensors data arrived */
			if (kStepping[i] > ArrayData[slotD-1].kTime)
				pr_err("%llu\n", kStepping[i]);
		}
	}
	logtime_reset();
	mutex_unlock(&logtime_mutex);
}

#else
#define RECORD_TIME(...)
#define logtime_alloc(...)
#define logtime_store(...)
#define logtime_stepping_store(...)
#define logtime_show(...)
#define logtime_reset()
#endif

#define POSITION_DETECT_INIT(_Pos, _Sta) do { \
		if (!md->sensors_off) { \
			atomic_set(&md->position, POS_UNKNOWN); \
			atomic_set(&md->status, STATUS_UNKNOWN); \
			atomic_set(&md->destination, POS_UNKNOWN); \
			RECORD_TIME(tZero); \
			/* run position detection */ \
			moto_drv8424_set_sensing(md, true); \
			dev_info(md->dev, "Start position detection\n"); \
		} else { \
			atomic_set(&md->position, _Pos); \
			moto_drv8424_cmd_push(md, CMD_POSITION, 0); \
			atomic_set(&md->status, _Sta); \
			moto_drv8424_cmd_push(md, CMD_STATUS, 0); \
			md->ready = true; \
			dev_info(md->dev, "Assume %s position\n", position_labels[_Pos]); \
		} \
	} while(0)

#define POSITION_RANGE_CHK(n) \
	((n >= POS_UNKNOWN) && (n < POS_MAX))

#define STOP_STEPPING_SIGNAL \
	do { \
		atomic_set(&md->stepping, 0); \
		atomic_set(&md->step_count, 0); \
		RECORD_TIME(kKill); \
		if (hrtimer_try_to_cancel(&md->stepping_timer) < 0) \
			hrtimer_cancel_wait_running(&md->stepping_timer); \
	} while(0)

#define SIMPLE_MOTOR_STOP do { \
		char *msg = NULL; \
		msg = motor_stop(__func__, true, md, true); \
		if (msg) \
			LOGD("%s", msg); \
	} while(0)


static ktime_t adapt_time_helper(ktime_t usec)
{
	return ns_to_ktime(usec * 1000);
}
#if 0
static inline bool is_hw_clk(motor_device* md)
{
	return (md->hw_clock && (md->cur_clk != hw_clocks[0]));
}
#endif
void sleep_helper(unsigned usec)
{
	if (usec < 500) { //<500us, udelay
		udelay(usec);
	} else if (usec >= 500 && usec < 20000) { //500us - 20ms, usleep_range
		usleep_range(usec, usec + 10);
	} else { //>= 20ms, msleep
		msleep(usec);
	}
}

static int moto_drv8424_set_regulator_power(motor_device* md, bool en)
{
	motor_control * mc = &md->mc;
	int err = 0;

	if (en) {
		err = regulator_enable(mc->vdd);
		if (err) {
			dev_err(md->dev, "Failed to enable VDD ret=%d\n", err);
			goto exit;
		}
		LOGD("regulator enabled\n");
	} else {
		err = regulator_disable(mc->vdd);
		if (err) {
			dev_err(md->dev, "Failed to disable VDD ret=%d\n", err);
			goto exit;
		}
		LOGD("regulator disabled\n");
	}
	return 0;
exit:
	return err;
}
#if 0
static int set_motor_clk(motor_device* md, bool en)
{
	int ret = 0;
	if (en) {
		set_pinctrl_state(md, CLK_ACTIVE);
		ret = clk_set_rate(md->pwm_clk, md->cur_clk);
		if (ret < 0) {
			dev_err(md->dev, "Failed to set clk rate to %ld\n", md->cur_clk);
			ret = -ENODEV;
			goto soft_clk;
		}
		ret = clk_prepare_enable(md->pwm_clk);
		if (ret < 0) {
			dev_err(md->dev, "Failed to clk_prepare_enable\n");
			ret = -ENODEV;
			goto soft_clk;
		}
	} else {
		clk_disable_unprepare(md->pwm_clk);
		dev_info(md->dev, "disable clock");
		set_pinctrl_state(md, CLK_SLEEP);
	}
	 return 0;

soft_clk:
	md->hw_clock = false;
	return ret;
}
#endif
static int init_motor_clk(motor_device* md)
{
	int ret = 0;

	md->pwm_clk = devm_clk_get(md->dev, md->clock_name);
	if (IS_ERR(md->pwm_clk)) {
		dev_err(md->dev, "Get clk error, motor is not drived\n");
		ret = -ENODEV;
		goto soft_clk;
	}
	return 0;
soft_clk:
	md->hw_clock = false;
	return ret;
}

static int set_pinctrl_state(motor_device* md, unsigned state_index)
{
	motor_control* mc = &md->mc;
	int ret = 0;

	if (state_index >= PINS_END) {
		dev_err(md->dev, "Illegal pin index\n");
		ret = -EINVAL;
		goto err;
	}
	dev_dbg(md->dev, "setting punctrl for state '%s'\n", pins_state[state_index]);
	mc->pins = devm_pinctrl_get(md->dev);
	if (IS_ERR_OR_NULL(mc->pins)) {
		ret = PTR_ERR(mc->pins);
		dev_err(md->dev, "Failed to get pinctrl %d\n", ret);
		goto err;
	}
	mc->pin_state[state_index] = pinctrl_lookup_state(mc->pins, pins_state[state_index]);
	if (IS_ERR_OR_NULL(mc->pin_state[state_index])) {
		ret = PTR_ERR(mc->pin_state[state_index]);
		dev_err(md->dev, "Cannot find pin_state '%s' %d\n", pins_state[state_index], ret);
		goto err_pin_state;
	}
	ret = pinctrl_select_state(mc->pins, mc->pin_state[state_index]);
	if (ret) {
		dev_err(md->dev, "Cannot set pin_state '%s' %d\n", pins_state[state_index], ret);
	}

err_pin_state:
	pinctrl_put(mc->pins);
err:
	return ret;
}

static void set_irq_state(motor_device *md, bool state)
{
	if (md->do_not_arm_irq) {
		LOGD("setting irq %d ignored\n", state);
		return;
	}
	if (state) {
		if (md->irq_enabled == true)
			return;
		if (gpio_is_valid(md->mc.ptable[MOTOR_FAULT1_INT]))
			enable_irq(gpio_to_irq(md->mc.ptable[MOTOR_FAULT1_INT]));
		if (gpio_is_valid(md->mc.ptable[MOTOR_FAULT2_INT]))
			enable_irq(gpio_to_irq(md->mc.ptable[MOTOR_FAULT2_INT]));
		md->irq_enabled = true;
	} else {
		if (md->irq_enabled == false)
			return;
		if (gpio_is_valid(md->mc.ptable[MOTOR_FAULT1_INT]))
			disable_irq(gpio_to_irq(md->mc.ptable[MOTOR_FAULT1_INT]));
		if (gpio_is_valid(md->mc.ptable[MOTOR_FAULT2_INT]))
			disable_irq(gpio_to_irq(md->mc.ptable[MOTOR_FAULT2_INT]));
		md->irq_enabled = false;

	}
}

static void moto_drv8424_set_motor_torque(motor_device* md)
{
	if (!md->support_torque)
		return;
	switch (md->torque) {
	case TORQUE_FULL:
		set_pinctrl_state(md, T0_LOW);
		set_pinctrl_state(md, T1_LOW);
			break;
	case TORQUE_87:
		set_pinctrl_state(md, T0_DISABLE);
		set_pinctrl_state(md, T1_LOW);
			break;
	case TORQUE_75:
		set_pinctrl_state(md, T0_HIGH);
		set_pinctrl_state(md, T1_LOW);
			break;
	case TORQUE_62:
		set_pinctrl_state(md, T0_LOW);
		set_pinctrl_state(md, T1_DISABLE);
			break;
	case TORQUE_50:
		set_pinctrl_state(md, T0_DISABLE);
		set_pinctrl_state(md, T1_DISABLE);
			break;
	case TORQUE_37:
		set_pinctrl_state(md, T0_HIGH);
		set_pinctrl_state(md, T1_DISABLE);
			break;
	case TORQUE_25:
		set_pinctrl_state(md, T0_LOW);
		set_pinctrl_state(md, T1_HIGH);
			break;
	case TORQUE_12:
		set_pinctrl_state(md, T0_DISABLE);
		set_pinctrl_state(md, T1_HIGH);
			break;
	case TORQUE_DISABLE:
		set_pinctrl_state(md, T0_HIGH);
		set_pinctrl_state(md, T1_HIGH);
			break;
	default:
		pr_err("Unsupported torque %d\n", md->torque);
			return;
	}
	LOGD("pins T0 & T1 updated\n");
}

static void moto_drv8424_set_motor_mode(motor_device* md)
{
	if (!md->support_mode) {
		md->cur_mode = md->mode;
		return;
	}
	switch (md->mode) {
	case FULL_STEP:
		set_pinctrl_state(md, MODE0_LOW);
		set_pinctrl_state(md, MODE1_LOW);
			break;
	case STEP_2_RISING:
		set_pinctrl_state(md, MODE0_DISABLE);
		set_pinctrl_state(md, MODE1_LOW);
			break;
	case STEP_4_RISING:
		set_pinctrl_state(md, MODE0_HIGH);
		set_pinctrl_state(md, MODE1_LOW);
			break;
	case STEP_8_RISING:
		set_pinctrl_state(md, MODE0_LOW);
		set_pinctrl_state(md, MODE1_DISABLE);
			break;
	case STEP_8_BOTH_EDEG:
		set_pinctrl_state(md, MODE0_DISABLE);
		set_pinctrl_state(md, MODE1_DISABLE);
			break;
	case STEP_16_RISING:
		set_pinctrl_state(md, MODE0_HIGH);
		set_pinctrl_state(md, MODE1_DISABLE);
			break;
	case STEP_16_BOTH_EDGE:
		set_pinctrl_state(md, MODE0_LOW);
		set_pinctrl_state(md, MODE1_HIGH);
			break;
	case STEP_32_RISING:
		set_pinctrl_state(md, MODE0_DISABLE);
		set_pinctrl_state(md, MODE1_HIGH);
			break;
	case STEP_32_BOTH_EDEG:
		set_pinctrl_state(md, MODE0_HIGH);
		set_pinctrl_state(md, MODE1_HIGH);
			break;
	default:
		pr_err("Unsupported mode %d\n", md->mode);
			return;
	}
	if (md->cur_mode != md->mode) {
		if (md->hw_clock) {
			if (md->mode == FULL_STEP) {
			    md->cur_clk = hw_clocks[md->mode];
			    set_pinctrl_state(md, CLK_SLEEP);
			    dev_info(md->dev, "md->mode is FULL_STEP\n");
			} else {
			    md->cur_clk = hw_clocks[md->mode];
			    dev_info(md->dev, "Switch hw clock\n");
			}
		}
		md->cur_mode = md->mode;
		md->cur_mode = md->mode;
	}
	usleep_range(800, 900);
	LOGD("pins MODE0 & MODE1 updated\n");
}

static void moto_drv8424_set_motor_dir(motor_device* md)
{
	motor_control* mc = &md->mc;

	GPIO_OUTPUT_DIR(mc->ptable[MOTOR_DIR], md->dir);
	usleep_range(800, 900);
	PARANOIC("pin DIR set %d\n", md->dir);
}

/* Set operating modes, only control nSleep
 * Set nSleep & nEn  | H-Bridge | Vreg | Sequencer
 *       1       0   |   Op     |  Op  |   Op
 *       1       1   |   Dis    |  Op  |   Op
 *       0       X   |   Dis    |  Dis |   Dis
 *         Fault     |   Dis    |Depends on Fault
 */
static void moto_drv8424_set_motor_opmode(motor_device* md)
{
	motor_control* mc = &md->mc;

	GPIO_OUTPUT_DIR(mc->ptable[MOTOR_SLEEP], md->nsleep);
	//Twake 0.5ms Tsleep 0.7ms
	usleep_range(800, 1000);
	GPIO_OUTPUT_DIR(mc->ptable[MOTOR_EN], md->nEN);
	PARANOIC("pin SLEEP set %d\n", md->nsleep);
}

static int moto_drv8424_set_opmode(motor_device* md, unsigned opmode)
{
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&md->mlock, flags);
	if (md->nsleep == opmode) {
		ret = -EINVAL;
		goto exit;
	}
	md->nsleep = !!opmode;
	md->nEN = !md->nsleep;
	spin_unlock_irqrestore(&md->mlock, flags);

	PARANOIC("nsleep=%d, en=%d\n", md->nsleep, md->nEN);
	return 0;
exit:
	spin_unlock_irqrestore(&md->mlock, flags);
	if (ret < 0)
		dev_info(md->dev, "Unchanged the opmode status, ignore\n");
	return ret;
}

static void inline moto_drv8424_cmd_push(motor_device* md,
	int cmd, unsigned long delay)
{
	kfifo_put(&md->cmd_pipe, cmd);
	queue_delayed_work(md->motor_wq, &md->motor_work, delay);
}

static void moto_drv8424_set_sensing(motor_device* md, bool en)
{
	motor_control* mc = &md->mc;

	if (md->sensors_off)
		return;
	if (en) {
		md->detect_incomplete = true;
		/* signal sensor hub to start/stop sampling hall effect */
		GPIO_OUTPUT_DIR(mc->ptable[MOTOR_ACTIVE], 1);
		RECORD_TIME(kSensorUp);
		moto_drv8424_cmd_push(md, CMD_POLL, 0);
		if (!md->power_en && !md->sensors_off) {
			/* declare sensor querying stage when powered off */
			/* when motor powered status will indicate direction */
			atomic_set(&md->status, STATUS_QUERYING_POS);
			moto_drv8424_cmd_push(md, CMD_STATUS, msecs_to_jiffies(INT_20MS));
		}
	} else {
		GPIO_OUTPUT_DIR(mc->ptable[MOTOR_ACTIVE], 0);
		RECORD_TIME(kSensorDown);
	}
	PARANOIC("gpio ACTIVE set: %u\n", en ? 1 : 0);
}

//Set VDD
static void moto_drv8424_set_power_en(motor_device* md)
{
	motor_control* mc = &md->mc;
	GPIO_OUTPUT_DIR(mc->ptable[MOTOR_POWER_EN], md->power_en);
	//Tpower 0.5ms
	usleep_range(500, 1000);
	PARANOIC("BOOST set %d\n", md->power_en);
}

//Power sequence: PowerEn On->nSleep On->nSleep Off->PowerEn Off
static int moto_drv8424_set_power(motor_device* md, unsigned power)
{
	unsigned long flags;
	int ret = 0;

	if (md->power_en == power) {
		dev_dbg(md->dev, "Unchanged the power status, ignore\n");
		ret = -EINVAL;
		goto exit;
	}
	PARANOIC("power_en=%u\n", !!power);
	spin_lock_irqsave(&md->mlock, flags);
	md->power_en = !!power;
	spin_unlock_irqrestore(&md->mlock, flags);
	ret = moto_drv8424_set_opmode(md, md->power_en);
	if (ret < 0 ) {
		goto exit;
	}
	if (!md->power_en) {
		moto_drv8424_set_motor_opmode(md);
	}
	moto_drv8424_set_power_en(md);
exit:
	if (md->power_en) {
		logtime_reset();
		/* make sure zero time set before sensing status changed */
		RECORD_TIME(tZero);
	}
	/* Sensing can be On w/o motor powered */
	moto_drv8424_set_sensing(md, md->power_en ? true : false);

	return ret;
}

static void moto_drv8424_set_regime(motor_device *md, unsigned regime)
{
	unsigned msn;
	unsigned long flags;

	for (msn = 0; msn < MAX_STAGE_LEGS; msn++) {
		if (md->sequencer[regime][msn].freq && md->sequencer[regime][msn].ceiling)
			continue;
		break;
	}
	spin_lock_irqsave(&md->mlock, flags);
	md->regime = regime;
	md->stage = 0;
	md->max_stages = msn; /* msn value is 0-based */
	spin_unlock_irqrestore(&md->mlock, flags);
	dev_info(md->dev, "Set active regime: %s, stages # %u\n",
			regime_names[md->regime], msn+1);
}

#if 0
static int moto_drv8424_enable_clk(motor_device* md, bool en)
{
	return set_motor_clk(md, en);
}
#endif

static void moto_drv8424_drive_stage_init(motor_device* md)
{
	atomic_set(&md->stepping, 1);
	atomic_set(&md->step_count, 1);
	md->double_edge = false;
	md->half = md->step_period >> 1;
	md->level = 1;
	if (md->mode == STEP_8_BOTH_EDEG
		|| md->mode == STEP_16_BOTH_EDGE
		|| md->mode == STEP_32_BOTH_EDEG) {
			md->double_edge = true;
	}
}

static bool moto_drv8424_next_stage(motor_device* md)
{
	unsigned freq;
	motor_stage *ms;

	if (++md->stage > md->max_stages) {
		md->max_stages = 0;
		md->stage = 0;
		return false;
	}

	ms = &md->sequencer[md->regime][md->stage - 1];
	freq = ms->freq;
	md->step_ceiling = ms->ceiling;
	moto_drv8424_set_step_freq(md, freq);
	moto_drv8424_drive_stage_init(md);

	return true;
}

static int moto_drv8424_drive_sequencer(motor_device* md)
{
	if (md->faulting) {
		dev_warn(md->dev, "Motor faulting: action cancelled!!!\n");
		return -EBUSY;
	}

	md->ready = false;
	moto_drv8424_next_stage(md);
	moto_drv8424_set_motor_torque(md);
	moto_drv8424_set_motor_dir(md);
	moto_drv8424_set_motor_mode(md);
	moto_drv8424_set_motor_opmode(md);
	/* can judge about moving direction based on DIR or destination position */
	atomic_set(&md->status, md->dir ? STATUS_MOVING_OUT : STATUS_MOVING_IN);
	moto_drv8424_cmd_push(md, CMD_STATUS, 0);
	PARANOIC("sequencer init: stages %u ceiling %lu, freq %uHz period %luns, half %uns\n",
			md->max_stages, md->step_ceiling, md->step_freq, md->step_period, md->half);
	if (atomic_read(&md->stepping)) {
		motor_control* mc = &md->mc;

		PARANOIC("Status updated: status %d\n", atomic_read(&md->status));
		GPIO_OUTPUT_DIR(mc->ptable[MOTOR_STEP], md->level);
		atomic_inc(&md->step_count);
		hrtimer_start(&md->stepping_timer, adapt_time_helper(md->half), HRTIMER_MODE_REL);
	}

	return 0;
}

/* this function is called with spinlock running!!! */
static char *motor_stop(const char *f, bool lock, motor_device* md, bool clean)
{
	unsigned long flags;
	char *retptr = NULL;
	static char msg[128];

	PARANOIC("step count %d\n", atomic_read(&md->step_count));
	if (clean) {
		STOP_STEPPING_SIGNAL;
		RECORD_TIME(kClean);
		PARANOIC("step_count & stepping reset\n");
	}
	set_irq_state(md, false);

	if (lock)
		spin_lock_irqsave(&md->mlock, flags);
	if (!md->user_sync_complete) {
		md->user_sync_complete = true;
		wake_up(&md->sync_complete);
		snprintf(msg, 128, "called from: %s; waking up motor thread\n", f);
		retptr = msg;
	}
	if (lock)
		spin_unlock_irqrestore(&md->mlock, flags);
	return retptr;
}

static __ref int motor_kthread(void *arg)
{
	motor_device* md = (motor_device*)arg;
	struct sched_param param = {.sched_priority = MAX_USER_RT_PRIO - 1};
	int value, ret = 0;

	sched_setscheduler(current, SCHED_FIFO, &param);
	while (!kthread_should_stop()) {
		md->user_sync_complete = false;
		LOGD("wait for motor event\n");
		do {
			ret = wait_event_interruptible(md->sync_complete,
			            md->user_sync_complete || kthread_should_stop());
		} while (ret != 0);
		if (kthread_should_stop())
			break;
		if (md->faulting) {
			value = atomic_read(&md->fault_irq);
			if (value != 0) {
				moto_drv8424_cmd_push(md, CMD_FAULT, 0);
				atomic_set(&md->status, STATUS_FAULTING);
				moto_drv8424_cmd_push(md, CMD_STATUS, 0);
				dev_warn(md->dev, "Motor #%d failure reported!!!\n",
					irq_to_gpio(value) == md->mc.ptable[MOTOR_FAULT1_INT] ? 1 : 2);
			}
			goto show_stats;
		}
		/* it's safe to call the following functions even if motor was not running */
		moto_drv8424_set_power(md, 0);
		if (md->power_default_off) {
			dev_info(md->dev, "vdd power off\n");
			moto_drv8424_set_regulator_power(md, false);
		}
		/*
		 * In sensor driven position detection we need to handle
		 * motor slippage. When this happens, position will still
		 * be unknown even though stepper sequence completed.
		 */
		if (!md->sensors_off && md->detect_incomplete) {
			if (md->regime == SQ_SLOWMO) {
				/* make sure position is not unknown */
				atomic_set(&md->position, POS_COMPACT);
				atomic_set(&md->status, STATUS_FAULTING);
				moto_drv8424_cmd_push(md, CMD_STATUS, 0);
				/* Unrecoverable motor fault */
				SIMPLE_MOTOR_STOP;
				md->faulting = true;
				/* make sure timeout is not stuck */
				md->time_out = 0;
				dev_warn(md->dev, "Motor faulting state set!!!\n");
				goto show_stats;
			}
			/* Recovery from incomplete transition */
			moto_drv8424_cmd_push(md, CMD_RECOVERY, 0);
			dev_warn(md->dev, "position recovery!!!\n");
			goto show_stats;
		}

		/* Need to stop stepping to avoid overshoot, But only can
		 * do that when detection completed successfully. Otherwise
		 * there is noticeable delay between main transition and
		 * recovery attempt
		 */
		STOP_STEPPING_SIGNAL;

		/* if position detected based on sensor data,
		 * current position will be set to destination
		 * to follow the same logic flow
		 */
		atomic_set(&md->position, atomic_read(&md->destination));
		moto_drv8424_cmd_push(md, CMD_POSITION, 0);

		/* this will stop polling */
		atomic_set(&md->destination, 0);

		switch (atomic_read(&md->position)) {
		case POS_COMPACT:
			value = STATUS_STOPPED_COMPACT;
				break;
		case POS_EXPANDED:
			value = STATUS_STOPPED_EXPANDED;
				break;
		case POS_PEEK:
			value = STATUS_STOPPED_PEEK;
				break;
		default: value = STATUS_UNKNOWN;
			 /* this should never happen */
			 dev_err(md->dev, "Undetermined position!!!");
		}

		atomic_set(&md->status, value);
		moto_drv8424_cmd_push(md, CMD_STATUS, 0);
		/* At this point it's safe to allow next position change */
		md->ready = true;

		PARANOIC("update: status %d, position %d\n",
		atomic_read(&md->status), atomic_read(&md->position));

		/* need to show statistics on completion */
		logtime_show(__func__);
show_stats:
		PARANOIC("semaphore=%u samples=%d\n", md->data_ready.count, atomic_read(&md->samples));
		atomic_set(&md->samples, 0);
	}

	return 0;
}

static int disable_motor(struct device* dev)
{
	motor_device* md = (motor_device*)dev_get_drvdata(dev);
	STOP_STEPPING_SIGNAL;
	return 0;
}

static int motor_set_enable(struct device* dev, bool enable)
{
	motor_device* md = (motor_device*)dev_get_drvdata(dev);

	if (atomic_read(&md->stepping)) {
		disable_motor(dev);
	}
	if (enable) {
		/* reset sensor data before motor started moving */
		RESET_SENSOR_DATA(OUT_OF_RANGE);
		moto_drv8424_drive_sequencer(md);
		set_irq_state(md, true);
	}

	return 0;
}


static void moto_drv8424_set_enable_with_power(motor_device *md, bool enable)
{
	if (md->power_default_off && enable) {
		dev_info(md->dev, "vdd power on\n");
		moto_drv8424_set_regulator_power(md, true);
		msleep(1);
	}
	if (!moto_drv8424_set_power(md, enable)) {
		motor_set_enable(md->dev, enable);
	}
}

static void motor_set_motion_params(motor_device *md, int start, int end)
{
	int regime = SQ_FULL;

	if (start == POS_UNKNOWN || end == POS_UNKNOWN) {
		dev_err(md->dev, "unknown src | dest!!!\n");
	}

	if (start == POS_EXPANDED) {
		md->dir = DIR_WITHDRAW;
		if (end == POS_COMPACT)
			regime = SQ_FULL;
		else // end == PEEK
			regime = SQ_PROLONGED;
	} else if (start == POS_COMPACT) {
		if (end == POS_EXPANDED) {
			md->dir = DIR_EXTEND;
			regime = SQ_FULL;
		} else { // end == PEEK
			md->dir = DIR_WITHDRAW;
			regime = SQ_SHORT;
		}
	} else { // start == PEEK
		md->dir = DIR_EXTEND;
		if (end == POS_EXPANDED)
			regime = SQ_PROLONGED;
		else // end == COMPACT
			regime = SQ_SHORT;
	}

	if (!md->sensors_off && md->detect_incomplete) {
		regime = SQ_SLOWMO;
		PARANOIC("engage SLOWMO in recovery\n");
	}

	moto_drv8424_set_regime(md, regime);
}

static enum hrtimer_restart motor_stepping_timer_action(struct hrtimer *h)
{
	motor_device * md = container_of(h, motor_device, stepping_timer);
	unsigned long flags;
	bool showlog = false;
	enum hrtimer_restart ret = HRTIMER_RESTART;

	spin_lock_irqsave(&md->mlock, flags);
	logtime_stepping_store();
	if (!atomic_read(&md->stepping)) {
		goto next_stage;
	}
	md->level = !md->level;
	GPIO_OUTPUT_DIR(md->mc.ptable[MOTOR_STEP], md->level);
	if (md->double_edge) {
		atomic_inc(&md->step_count);
	} else if (md->level) {
		atomic_inc(&md->step_count);
	}

	if (md->step_ceiling && atomic_read(&md->step_count) > md->step_ceiling) {
		/* multi stage sequence */
		if (!moto_drv8424_next_stage(md)) {
			char *msg = NULL;
next_stage:
			/* wake up motor thread at the end of sequence */
			/* when sensors unavailable */
			if (md->sensors_off || md->detect_incomplete) {
				msg = motor_stop(__func__, false, md, true);
				showlog = true;
			}
			GPIO_OUTPUT_DIR(md->mc.ptable[MOTOR_STEP], 0);
			spin_unlock_irqrestore(&md->mlock, flags);
			RECORD_TIME(kTerm);
			if (showlog) {
				if (msg)
					LOGD("%s", msg);
				LOGD("sequence ended!!! last data: %d %d %d\n",
					md->sensor_data[0],
					md->sensor_data[1],
					md->sensor_data[2]);
			}
			return HRTIMER_NORESTART;
		}
	}
	hrtimer_forward_now(h, adapt_time_helper(md->half));
	spin_unlock_irqrestore(&md->mlock, flags);

	return ret;
}

static void inline motor_cancel_detection(motor_device *md)
{
	md->cancel_thread = true;
	up(&md->data_ready);
}

static void inline motor_cancel_motion(motor_device *md)
{
	md->cancel_thread = true;
	SIMPLE_MOTOR_STOP;
}

/*
 * Timeout can happen either if destination position never reached or
 * driver cannot detect current position. In either way timeout handler
 * will try enforcing COMPACT state
 */
static enum hrtimer_restart motor_timeout_timer_action(struct hrtimer *h)
{
	motor_device * md = container_of(h, motor_device, timeout_timer);
	int original = atomic_read(&md->position);
	int destination = atomic_read(&md->destination);
	enum hrtimer_restart ret = HRTIMER_NORESTART;

	if (md->sensors_off) {
		dev_warn(md->dev, "time out in no sensors mode!\n");
		goto exit;
	}

	if (POSITION_RANGE_CHK(original) &&
	    POSITION_RANGE_CHK(destination)) {
		LOGD("Timeout: position=%s, detection=%s\n",
			position_labels[original], position_labels[destination]);
	} else {
		LOGD("Timeout: position=%d, detection=%d\n", original, destination);
	}

	/* Timed out arriving to destination from known starting position.
	 * Solution would be to go back to starting position
	 */
	if (original != POS_UNKNOWN &&
	    original != destination) {
		/* repeat the same motion */
		motor_cancel_motion(md);
		LOGD("Timeout driving motor\n");
	} else { /* original & destination ==  POS_UNKNOWN */
		motor_cancel_detection(md);
		LOGD("Timeout position detection\n");
	}
	moto_drv8424_cmd_push(md, CMD_TIMEOUT, msecs_to_jiffies(INT_20MS));
exit:
	return ret;
}

#define PROXIMITY_HD 350 /* 3.5 degrees multiplied by 100 to get rid of float */
#define PROXIMITY_SD 750 /* 7.5 degrees multiplied by 100 to get rid of float */
static int sensProximity = PROXIMITY_HD;

static inline bool IN_RANGE(int valMeas, int valSet)
{
	return (abs(valMeas) < sensProximity) ? true : false;
}

static void motor_reset(motor_device *md)
{
	if (atomic_read(&md->stepping))
		disable_motor(md->dev);

	moto_drv8424_set_power(md, 1);
	msleep(RESET_TIME);
	moto_drv8424_set_power(md, 0);
}

static int moto_drv8424_detect_position(motor_device *md)
{
	int i, start, end, ret = POS_UNKNOWN;

	if (atomic_read(&md->destination) > 0) {
		/* check only destination position */
		start = atomic_read(&md->destination);
		end = start + 1;
	} else {
		/* detection check all */
		start = POS_COMPACT;
		end = POS_MAX;
	}

	mutex_lock(&md->mx_lock);
	logtime_store((void *)md->sensor_data, sizeof(md->sensor_data), 1);
	for (i = start; i < end; i++) {
		if (IN_RANGE(md->sensor_data[scanner[i].index], scanner[i].value)) {
			RECORD_TIME(kDetect);
			/* this is supposed to disable motor output immediately!!! */
			moto_drv8424_set_power(md, 0);
			md->detect_incomplete = false;
			STOP_STEPPING_SIGNAL;
			PARANOIC("sensor within range: (%d) < %d < (%d)\n",
				scanner[i].value - sensProximity,
				md->sensor_data[scanner[i].index],
				scanner[i].value + sensProximity);
			if (sensProximity == PROXIMITY_SD) {
				sensProximity = PROXIMITY_HD;
				LOGD("set tighter proximity threshold\n");
			}
			ret = i;
			break;
		}
	}
	mutex_unlock(&md->mx_lock);

	return ret;
}

static __ref int detection_kthread(void *arg)
{
	motor_device* md = (motor_device*)arg;
	struct sched_param param = {.sched_priority = MAX_USER_RT_PRIO - 1};
	int position, pending, samples, ret = 0;

	sched_setscheduler(current, SCHED_FIFO, &param);
	while (!kthread_should_stop()) {
		mutex_lock(&md->mx_lock);
		pending = md->data_ready.count;
		if (pending != 0) {
			dev_warn(md->dev, "Unprocessed data detected (%u)!!!\n", pending);
			while (pending-- > 0) {
				down(&md->data_ready);
			}
			logtime_show(__func__);
		}
		mutex_unlock(&md->mx_lock);
		LOGD("wait for data update\n");
		do {
			ret = wait_event_interruptible(md->data_wait,
			            md->data_update_ready || kthread_should_stop());
		} while (ret != 0);
		if (kthread_should_stop())
			break;
		md->data_update_ready = false;
		md->cancel_thread = false;
		samples = 0;
		do {
			down(&md->data_ready);
			position = moto_drv8424_detect_position(md);
			samples++;
			if (md->cancel_thread) {
				dev_warn(md->dev, "Sensors scan terminated\n");
				break;
			}
		} while (position == POS_UNKNOWN);

		if (!md->cancel_thread) {
			/* polling might occur without driving motor
			 * we need to release kthread that will trigger position change
			 * and set destination to properly complete detection
			 */
			atomic_set(&md->destination, position);
			SIMPLE_MOTOR_STOP;
			/* in case of successful initial position detection */
			/* timeout timer still running, thus need to cancel */
			/* assuming it does not hurt to attempt cancelling  */
			/* the timer that has not been armed */
			moto_drv8424_cmd_push(md, CMD_TIMER_STOP, 0);
		}

		if (position != POS_UNKNOWN) {
			/*
			 * In case position was detected while permanent faulty
			 * state has been set already, we need to drop it as mishap
			 */
			if (md->faulting) {
				md->faulting = false;
				dev_warn(md->dev, "Motor faulting state dropped!!!");
			}
			dev_info(md->dev, "detected position: %s\n", position_labels[position]);
		}
		LOGD("samples %d; last data: %d %d %d\n", samples,
			md->sensor_data[0], md->sensor_data[1], md->sensor_data[2]);
	}

	return 0;
}

static void motor_fault_handler(motor_device *md)
{
	int fault_irq = atomic_read(&md->fault_irq);
	int position, destination, status;

	if (!fault_irq) {
		dev_warn(md->dev, "Called w/o IRQ!!!\n");
		return;
	}

	motor_cancel_detection(md);
	motor_reset(md);

	position = atomic_read(&md->position);
	status = atomic_read(&md->status);
	destination = atomic_read(&md->destination);
	POSITION_DETECT_INIT(destination, status);
	/* TODO need to reinstate sequence, since */
	/* motor has stopped already!!! */

	md->faulting = false;
	atomic_set(&md->fault_irq, 0);

	dev_warn(md->dev, "Device reset due to motor #%d fault\n",
		irq_to_gpio(fault_irq) == md->mc.ptable[MOTOR_FAULT1_INT] ? 1 : 2);
	LOGD("Pos/dest: %s/%s, status: %s\n",
		position_labels[position],
		position_labels[destination],
		status_labels[status]);
}

#define PEEK_ID 2
#define COMPACT_ID 1
#define EXPANDED_ID 0

#define PROX_THRES 4500

static void motor_cmd_work(struct work_struct *work)
{
	struct delayed_work *dw = container_of(work, struct delayed_work, work);
	motor_device* md = container_of(dw, motor_device, motor_work);
	int cpos, dest, cmd = 0;

	while (kfifo_get(&md->cmd_pipe, &cmd)) {
		switch (cmd) {
		case CMD_FAULT:
			motor_fault_handler(md);
					break;
		case CMD_STATUS:
			sysfs_notify(&md->dev->kobj, NULL, "status");
			md->status_update_ready = true;
			wake_up(&md->status_wait);
					break;
		case CMD_POSITION:
			sysfs_notify(&md->dev->kobj, NULL, "position");
			md->position_update_ready = true;
			wake_up(&md->position_wait);
					break;
		case CMD_POLL:
			md->data_update_ready = true;
			wake_up(&md->data_wait);
					break;
		case CMD_TIMER_START:
			hrtimer_start(&md->timeout_timer,
				adapt_time_helper(md->time_out), HRTIMER_MODE_REL);
			LOGD("timer armed\n");
					break;
		case CMD_TIMER_STOP:
			md->time_out = 0;
			if (hrtimer_active(&md->timeout_timer)) {
				hrtimer_try_to_cancel(&md->timeout_timer);
				LOGD("timer stopped\n");
			}
					break;
		case CMD_RECOVERY:
			/* pos & dest depends on recovery cause */
			dest = atomic_read(&md->destination);
			md->power_en = 0;
			motor_set_motion_params(md,
				atomic_read(&md->position), dest);
			if (dest == POS_COMPACT) {
			/* Recovery invoked when motor failed to arrive to
			 * destination. If the destination is COMPACT, there
			 * is a chance motor will "overshoot" and go beyond
			 * COMPACT position w/o properly detecting it.
			 * In this case slider will go all the way up/down
			 * to its physical limit. So we need to allow
			 * detecting those.
			 * Thus setting destination to UNKNOWN to allow
			 * moto_drv8424_detect_position() scanning for all
			 * detectable positions
			 */
				dest = POS_UNKNOWN;
				atomic_set(&md->destination, dest);
			}
			moto_drv8424_set_enable_with_power(md, true);
			if (dest == POS_UNKNOWN)
				LOGD("Enabled limits in recovery!!!\n");
					break;
		case CMD_TIMEOUT:
			/* We want to be able detecting ANY valid position, so */
			/* md->position and md->destination must stay unknown */
			/* Motion parameters will be set based on discovered */
			/* position or set to transition from EXPANEDE to */
			/* COMPACT if current position cannot be determined */
			md->power_en = 0;
			cpos = POS_EXPANDED;
			dest = POS_COMPACT;
			if (md->sensor_data[PEEK_ID] < PROX_THRES ||
				(md->sensor_data[COMPACT_ID] < 0 &&
				md->sensor_data[COMPACT_ID] > -PROX_THRES)) {
				/* near PEEK or between PEEK&COMPACT */
				/* move toward EXPANDED position */
				cpos = POS_PEEK;
			}
			motor_set_motion_params(md, cpos, dest);
			moto_drv8424_set_enable_with_power(md, true);
					break;
		default:
			dev_err(md->dev, "Unsupported command %d\n", cmd);
					break;
		}
	}
}

static irqreturn_t motor_fault_irq(int irq, void *pdata)
{
	motor_device * md = (motor_device*) pdata;

	set_irq_state(md, false);
	atomic_set(&md->fault_irq, irq);
	/* no need to wake up twice if device is faulting */
	if (!md->faulting) {
		md->faulting = true;
		md->user_sync_complete = true;
		wake_up(&md->sync_complete);
	}

	return IRQ_HANDLED;
}

//module node interface
static void moto_drv8424_set_torque(motor_device* md, unsigned torque)
{
	unsigned long flags;
	bool showlog = false;

	spin_lock_irqsave(&md->mlock, flags);
	if(md->torque == torque) {
		showlog = true;
		goto exit;
	}
	md->torque = torque;
exit:
	spin_unlock_irqrestore(&md->mlock, flags);
	if (showlog)
		dev_info(md->dev, "Unchanged the torque, ignore\n");
}

static int moto_drv8424_set_mode(motor_device* md, unsigned mode)
{
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&md->mlock, flags);
	if (md->mode == mode) {
		ret = -EINVAL;
		goto exit;
	}
	md->mode = mode;
	spin_unlock_irqrestore(&md->mlock, flags);
	return 0;
exit:
	spin_unlock_irqrestore(&md->mlock, flags);
	if (ret < 0)
		dev_info(md->dev, "Unchanged the mode, ignore\n");
	return ret;
}

static int moto_drv8424_set_dir(motor_device* md, unsigned dir)
{
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&md->mlock, flags);
	if (md->dir == dir) {
		ret = -EINVAL;
		goto exit;
	}
	md->dir = dir;
	spin_unlock_irqrestore(&md->mlock, flags);
	return 0;
exit:
	spin_unlock_irqrestore(&md->mlock, flags);
	if (ret < 0)
		dev_dbg(md->dev, "Unchanged the dir, ignore\n");
	return ret;
}

//Spec step max frequency 250KHz
#define STEP_MAX_FREQ 250000
static void moto_drv8424_set_step_freq(motor_device* md, unsigned freq)
{
	if (md->step_freq == freq) {
		dev_dbg(md->dev, "Unchanged the freq, ignore\n");
		return;
	} else if(freq == 0) {
		dev_err(md->dev, "Invalid frequency, ignore\n");
		return;
	}
	if(freq > STEP_MAX_FREQ)
		freq = STEP_MAX_FREQ;
	md->step_freq = freq;
	md->step_period = 1000000 / md->step_freq;
}

static int moto_drv8424_set_ceiling(motor_device* md, unsigned ceiling)
{
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&md->mlock, flags);
	if (md->step_ceiling == ceiling) {
		ret = -EINVAL;
		goto exit;
	}
	md->step_ceiling = ceiling;
	spin_unlock_irqrestore(&md->mlock, flags);
	return 0;
exit:
	spin_unlock_irqrestore(&md->mlock, flags);
	if (ret < 0)
		dev_dbg(md->dev, "Unchanged the ceiling, ignore\n");
	return ret;
}

static ssize_t motor_flags_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	motor_device* md = (motor_device*)dev_get_drvdata(dev);
	ssize_t blen = 0;

	blen += snprintf(buf+blen, PAGE_SIZE-blen, "===> Motor params:\n");
	blen += snprintf(buf+blen, PAGE_SIZE-blen, "Position: %s\n", position_labels[atomic_read(&md->position)]);
	blen += snprintf(buf+blen, PAGE_SIZE-blen, "Destination: %s\n", position_labels[atomic_read(&md->destination)]);
	blen += snprintf(buf+blen, PAGE_SIZE-blen, "Status: %s\n", status_labels[atomic_read(&md->status)]);
	blen += snprintf(buf+blen, PAGE_SIZE-blen, "Stepping/count/ceiling: %d/%d/%lu\n",
		atomic_read(&md->stepping), atomic_read(&md->step_count), md->step_ceiling);
	blen += snprintf(buf+blen, PAGE_SIZE-blen, "Stage/max: %d/%d\n", md->stage, md->max_stages);
	blen += snprintf(buf+blen, PAGE_SIZE-blen, "Faulting/ready: %d/%d\n", md->faulting, md->ready);
	blen += snprintf(buf+blen, PAGE_SIZE-blen, "Timeout: %u\n", md->time_out);
	blen += snprintf(buf+blen, PAGE_SIZE-blen, "Pitch: %s\n", md->use_2p8_pitch ? "2.8" : "2.3");
	blen += snprintf(buf+blen, PAGE_SIZE-blen, "===> Sensor params:\n");
	blen += snprintf(buf+blen, PAGE_SIZE-blen, "Sensors: %s\n", md->sensors_off ? "Off" : "On");
	blen += snprintf(buf+blen, PAGE_SIZE-blen, "Samples: %d\n", atomic_read(&md->samples));
	mutex_lock(&md->mx_lock);
	blen += snprintf(buf+blen, PAGE_SIZE-blen, "Semaphore: %d\n", md->data_ready.count);
	mutex_unlock(&md->mx_lock);

	return blen;
}

static ssize_t motor_enable_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	motor_device* md = (motor_device*)dev_get_drvdata(dev);
	return snprintf(buf, 20, "%d\n", md->power_en);
}

static ssize_t motor_enable_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t len)
{
	motor_device* md = (motor_device*)dev_get_drvdata(dev);
	unsigned enable = 0;

	if (md->faulting || md->ready == false) {
		dev_err(dev, "%s: Device not ready or faulting\n", __func__);
		return -EBUSY;
	}
	if (kstrtouint(buf, 10, &enable)) {
		dev_err(dev, "Error value: %s\n", buf);
		goto exit;
	}
	if (!!enable) {
		if (atomic_read(&md->position) == POS_UNKNOWN) {
			dev_warn(md->dev, "Cannot move from unknown position!!!\n");
			goto exit;
		}
	}
	moto_drv8424_set_enable_with_power(md, !!enable);
exit:
	return len;
}

static ssize_t motor_dir_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	motor_device* md = (motor_device*)dev_get_drvdata(dev);
	return snprintf(buf, 20, "%d\n", md->dir);
}

static ssize_t motor_dir_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t len)
{
	motor_device* md = (motor_device*)dev_get_drvdata(dev);
	unsigned value = 0;

	if (md->faulting || md->ready == false) {
		dev_err(dev, "%s: Device not ready or faulting\n", __func__);
		return -EBUSY;
	}
	if (kstrtouint(buf, 10, &value)) {
		dev_err(dev, "Error value: %s\n", buf);
		goto exit;
	}
	moto_drv8424_set_dir(md, value);
exit:
	return len;
}

static ssize_t motor_step_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	motor_device* md = (motor_device*)dev_get_drvdata(dev);
	return snprintf(buf, 20, "%d\n", md->step_freq);
}

static ssize_t motor_step_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t len)
{
	motor_device* md = (motor_device*)dev_get_drvdata(dev);
	unsigned long flags;
	unsigned value = 0;

	if (md->faulting || md->ready == false) {
		dev_err(dev, "%s: Device not ready or faulting\n", __func__);
		return -EBUSY;
	}
	if (kstrtouint(buf, 10, &value)) {
		dev_err(dev, "Error value: %s\n", buf);
		return -EINVAL;
	}
	spin_lock_irqsave(&md->mlock, flags);
	moto_drv8424_set_step_freq(md, value);
	spin_unlock_irqrestore(&md->mlock, flags);
	dev_info(md->dev, "freq %uHz period %ldus\n", md->step_freq, md->step_period);

	return len;
}

static ssize_t motor_prox_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	return snprintf(buf, 20, "%d\n", sensProximity);
}

static ssize_t motor_prox_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t len)
{
	motor_device* md = (motor_device*)dev_get_drvdata(dev);
	unsigned value = 0;

	if (kstrtouint(buf, 10, &value)) {
		dev_err(dev, "Error value: %s\n", buf);
		return -EINVAL;
	}
	sensProximity = value;
	dev_info(md->dev, "proximity value %d\n", sensProximity);
	return len;
}

static ssize_t motor_pitch_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	motor_device* md = (motor_device*)dev_get_drvdata(dev);
	return snprintf(buf, 20, "%s\n", md->use_2p8_pitch ? "2.8" : "2.3");
}

static ssize_t motor_pitch_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t len)
{
	motor_device* md = (motor_device*)dev_get_drvdata(dev);
	bool use_2p8_pitch = false;

	if (!strncmp(buf, "2.8", 3)) {
		use_2p8_pitch = true;
	} else if (!strncmp(buf, "2.8", 3)) {
		use_2p8_pitch = false;
	} else {
		dev_err(dev, "Error value: %s\n", buf);
		return -EINVAL;
	}
	if (use_2p8_pitch != md->use_2p8_pitch) {
		md->use_2p8_pitch = use_2p8_pitch;
		/* update sequencer */
		motor_set_sequencer(md);
		dev_info(md->dev, "pitch value %s\n", md->use_2p8_pitch ? "2.8" : "2.3");
	}

	return len;
}

static ssize_t motor_sensing_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	motor_device* md = (motor_device*)dev_get_drvdata(dev);
	dev_info(md->dev, "Motor faulting state support\n");
	return snprintf(buf, 20, "%d\n", md->sensors_off ? 0 : 1);
}

static ssize_t motor_sensing_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t len)
{
	motor_device* md = (motor_device*)dev_get_drvdata(dev);
	unsigned value = 0;

	if (kstrtouint(buf, 10, &value)) {
		dev_err(dev, "Error value: %s\n", buf);
		return -EINVAL;
	}
	md->sensors_off = !!value ? false : true;
	dev_info(md->dev, "sensing %d\n", md->sensors_off ? 0 : 1);
	/* backdoor to recover from bad sensors calibration */
	if (md->sensors_off &&
		(md->faulting || md->ready == false)) {
		md->faulting = false;
		md->ready = true;
		dev_info(md->dev, "Motor faulting state dropped!!!\n");
	}

	return len;
}

static ssize_t motor_mode_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	motor_device* md = (motor_device*)dev_get_drvdata(dev);
	return snprintf(buf, 20, "%d\n", md->mode);
}

static ssize_t motor_mode_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t len)
{
	motor_device* md = (motor_device*)dev_get_drvdata(dev);
	unsigned value = 0;

	if (md->faulting) {
		dev_err(dev, "Device faulting\n");
		return -EBUSY;
	}
	if (kstrtouint(buf, 10, &value)) {
		dev_err(dev, "Error value: %s\n", buf);
		goto exit;
	}
	moto_drv8424_set_mode(md, value);
exit:
	return len;
}

static ssize_t motor_torque_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	motor_device* md = (motor_device*)dev_get_drvdata(dev);
	return snprintf(buf, 20, "%d\n", md->torque);
}

static ssize_t motor_torque_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t len)
{
	motor_device* md = (motor_device*)dev_get_drvdata(dev);
	unsigned value = 0;

	if (md->faulting) {
		dev_err(dev, "Device faulting\n");
		return -EBUSY;
	}
	if (kstrtouint(buf, 10, &value)) {
		dev_err(dev, "Error value: %s\n", buf);
		goto exit;
	}
	moto_drv8424_set_torque(md, value);
exit:
	return len;
}

static ssize_t motor_ceiling_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	motor_device* md = (motor_device*)dev_get_drvdata(dev);
	return snprintf(buf, 20, "%ld\n", md->step_ceiling);
}

static ssize_t motor_ceiling_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t len)
{
	motor_device* md = (motor_device*)dev_get_drvdata(dev);
	unsigned value = 0;

	if (md->faulting) {
		dev_err(dev, "Device faulting\n");
		return -EBUSY;
	}
	if (kstrtouint(buf, 10, &value)) {
		dev_err(dev, "Error value: %s\n", buf);
		goto exit;
	}
	moto_drv8424_set_ceiling(md, value);
exit:
	return len;
}

static unsigned motor_get_max_pps(motor_stage *stage)
{
	unsigned mpps = 0;
	int i;

	for (i = 0; i < MAX_STAGE_LEGS; i++, stage++) {
		if (!(stage->freq | stage->ceiling))
			break;
		if (stage->freq > mpps)
			mpps = stage->freq;
	}

	return mpps;
}

static int motor_set_max_pps(motor_stage *stage, unsigned value)
{
	int i;

	for (i = 0; i < MAX_STAGE_LEGS; i++, stage++) {
		if (!(stage->freq | stage->ceiling))
			break;
		if (stage->freq > (stage + 1)->freq) {
			stage->freq = value;
			break;
		}
	}

	return 0;
}

static ssize_t motor_maxpps_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	motor_device* md = (motor_device*)dev_get_drvdata(dev);
	return snprintf(buf, 20, "%u\n", motor_get_max_pps(md->sequencer[SQ_FULL]));
}

static ssize_t motor_maxpps_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t len)
{
	motor_device* md = (motor_device*)dev_get_drvdata(dev);
	unsigned value = 0;

	if (kstrtouint(buf, 10, &value)) {
		dev_err(dev, "Error value: %s\n", buf);
		goto exit;
	}
	motor_set_max_pps(md->sequencer[SQ_FULL], value);
	motor_set_max_pps(md->sequencer[SQ_PROLONGED], value);
	dev_info(dev, "Set max PPS: %u\n", value);
exit:
	return len;
}

static ssize_t motor_time_out_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	motor_device* md = (motor_device*)dev_get_drvdata(dev);
	return snprintf(buf, 20, "%d\n", md->time_out);
}

static ssize_t motor_time_out_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t len)
{
	motor_device* md = (motor_device*)dev_get_drvdata(dev);
	unsigned value = 0;
	unsigned long flags;

	if (md->faulting) {
		dev_err(dev, "Device faulting\n");
		return -EBUSY;
	}
	if (kstrtouint(buf, 10, &value)) {
		dev_err(dev, "Error value: %s\n", buf);
		goto exit;
	}
	spin_lock_irqsave(&md->mlock, flags);
	if (value > MOTOR_DEFAULT_EXPIRE) {
		value = MOTOR_DEFAULT_EXPIRE;
	}
	md->time_out = value;
	spin_unlock_irqrestore(&md->mlock, flags);
exit:
	return len;
}

static ssize_t motor_reset_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t len)
{
	motor_device* md = (motor_device*)dev_get_drvdata(dev);
	unsigned value = 0;

	if (kstrtouint(buf, 10, &value)) {
		dev_err(dev, "Error value: %s\n", buf);
	} else {
		mutex_lock(&md->mx_lock);
		motor_reset(md);
		mutex_unlock(&md->mx_lock);
		dev_info(dev, "Device reset\n");
	}

	return len;
}

/* store new motor sequencer */
/* Format: index freq0:ceiling0 [... freqN:ceilingN] */
/* Example: 0 400:24 1400:56 3000:6252 2000:12 */
static ssize_t motor_sequencer_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t len)
{
	motor_device* md = (motor_device*)dev_get_drvdata(dev);
	char *buffer, *next, *pair;
	unsigned nstages = 0;
	unsigned msn, value, idx, freq, ceiling;

	buffer = kmalloc(len + 1, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;
	strlcpy(buffer, buf, len);
	buffer[len] = '\0';

	next = strnchr(buffer, len, ' ');
	if (!next) {
		dev_err(dev, "Invalid input data format: %s\n", buf);
		len = -EINVAL;
		goto exit;
	}
	*next++ = '\0';
	if (kstrtouint(buffer, 10, &value) ||
		(value < 0 || value >= SQ_SHOW_MAX)) {
		dev_err(dev, "Invalid index: %s\n", buffer);
		goto exit;
	}
	idx = value;
	for (msn = 0, pair = next; msn < MAX_STAGE_LEGS; msn++) {
		LOGD("old values [%u][%u]: freq=%u, ceiling=%u\n",
			idx, msn, md->sequencer[idx][msn].freq,
			md->sequencer[idx][msn].ceiling);
		if (!pair) {
			LOGD("zero-ing: [%u][%u]\n", idx, msn);
			md->sequencer[idx][msn].freq =
			md->sequencer[idx][msn].ceiling = 0;
			continue;
		}
		LOGD("pair string: [%s]\n", pair);
		next = strnchr(pair, len, ' ');
		if (next) {
			*next++ = '\0';
			len -= strlen(pair);
		}
		LOGD("remaining string: [%s]\n", next);
		if (sscanf(pair, "%u:%u", &freq, &ceiling) == 2) {
			nstages++;
			md->sequencer[idx][msn].freq = freq;
			md->sequencer[idx][msn].ceiling = ceiling;
			dev_info(md->dev, "[%u][%u]: freq=%u, ceiling=%u\n",
				idx, msn, freq, ceiling);
		}
		pair = next;
	}

	if (nstages > 0) {
		md->max_stages = nstages;
		LOGD("# of stages: %u\n", md->max_stages);
	} else {
		dev_err(dev, "Error value: %s\n", buf);
		len = -EINVAL;
	}
exit:
	kfree(buffer);

	return len;
}

static ssize_t motor_sequencer_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	motor_device* md = (motor_device*)dev_get_drvdata(dev);
	ssize_t blen = 0;
	unsigned idx, msn;

	for (idx = 0; idx < SQ_SHOW_MAX; idx++) {
		blen += snprintf(buf + blen, PAGE_SIZE - blen, "regime %s: ",
					regime_names[idx]);
		for (msn = 0; msn < MAX_STAGE_LEGS; msn++) {
			if (md->sequencer[idx][msn].freq == 0 &&
				md->sequencer[idx][msn].ceiling == 0)
				continue;
			blen += snprintf(buf + blen, PAGE_SIZE - blen, "%u:%u ",
			md->sequencer[idx][msn].freq,
			md->sequencer[idx][msn].ceiling);
		}
		blen += snprintf(buf + blen, PAGE_SIZE - blen, "\n");
	}

	return blen;
}

static ssize_t motor_regime_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t len)
{
	motor_device* md = (motor_device*)dev_get_drvdata(dev);
	unsigned value;

	if (md->faulting || md->ready == false) {
		dev_err(dev, "%s: Device not ready or faulting\n", __func__);
		return -EBUSY;
	}
	if (kstrtouint(buf, 10, &value) ||
		(value < SQ_FULL || value >= SQ_SHOW_MAX)) {
		dev_err(dev, "Error value: %s\n", buf);
		return -EINVAL;
	}
	moto_drv8424_set_regime(md, value);

	return len;
}

static ssize_t motor_regime_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	motor_device* md = (motor_device*)dev_get_drvdata(dev);
	return snprintf(buf, 20, "%s\n", regime_names[md->regime]);
}

static ssize_t motor_irq_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t len)
{
	motor_device* md = (motor_device*)dev_get_drvdata(dev);
	unsigned value;

	if (kstrtouint(buf, 10, &value)) {
		dev_err(dev, "Error value: %s\n", buf);
		return -EINVAL;
	}
	if (!!value) {
		md->do_not_arm_irq = false;
		dev_info(dev, "Enabled fault IRQ\n");
	} else {
		md->do_not_arm_irq = true;
		dev_info(dev, "Disabled fault IRQ\n");
	}

	return len;
}

static ssize_t motor_irq_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	motor_device* md = (motor_device*)dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", md->irq_enabled ? 1 : 0);
}

static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP, motor_enable_show, motor_enable_store);
static DEVICE_ATTR(dir, S_IRUGO|S_IWUSR|S_IWGRP, motor_dir_show, motor_dir_store);
static DEVICE_ATTR(step, S_IRUGO|S_IWUSR|S_IWGRP, motor_step_show, motor_step_store);
static DEVICE_ATTR(ceiling, S_IRUGO|S_IWUSR|S_IWGRP, motor_ceiling_show, motor_ceiling_store);
static DEVICE_ATTR(maxpps, S_IRUGO|S_IWUSR|S_IWGRP, motor_maxpps_show, motor_maxpps_store);
static DEVICE_ATTR(mode, S_IRUGO|S_IWUSR|S_IWGRP, motor_mode_show, motor_mode_store);
static DEVICE_ATTR(sequencer, S_IRUGO|S_IWUSR|S_IWGRP, motor_sequencer_show, motor_sequencer_store);
static DEVICE_ATTR(torque, S_IRUGO|S_IWUSR|S_IWGRP, motor_torque_show, motor_torque_store);
static DEVICE_ATTR(time_out, S_IRUGO|S_IWUSR|S_IWGRP, motor_time_out_show, motor_time_out_store);
static DEVICE_ATTR(prox, S_IRUGO|S_IWUSR|S_IWGRP, motor_prox_show, motor_prox_store);
static DEVICE_ATTR(pitch, S_IRUGO|S_IWUSR|S_IWGRP, motor_pitch_show, motor_pitch_store);
static DEVICE_ATTR(sensing, S_IRUGO|S_IWUSR|S_IWGRP, motor_sensing_show, motor_sensing_store);
static DEVICE_ATTR(reset, S_IWUSR|S_IWGRP, NULL, motor_reset_store);
static DEVICE_ATTR(flags, S_IRUGO, motor_flags_show, NULL);
static DEVICE_ATTR(regime, S_IRUGO|S_IWUSR|S_IWGRP, motor_regime_show, motor_regime_store);
static DEVICE_ATTR(irq, S_IRUGO|S_IWUSR|S_IWGRP, motor_irq_show, motor_irq_store);

#define ATTRS_STATIC 8
#define ATTRS_MAX 14

static int last_idx = ATTRS_STATIC;

static struct attribute *motor_attributes[ATTRS_MAX + 1] = {
	&dev_attr_dir.attr,
	&dev_attr_enable.attr,
	&dev_attr_step.attr,
	&dev_attr_ceiling.attr,
	&dev_attr_maxpps.attr,
	&dev_attr_reset.attr,
	&dev_attr_prox.attr,
	&dev_attr_pitch.attr,
	&dev_attr_sensing.attr,
	&dev_attr_flags.attr,
	&dev_attr_time_out.attr,
	&dev_attr_regime.attr,
	&dev_attr_sequencer.attr,
	&dev_attr_irq.attr,
	NULL
};

#define ATTR_ADD(name) { \
	if (last_idx < ATTRS_MAX) { \
		dev_info(md->dev, "[%d] adding attribute '%s'\n", last_idx, #name); \
		motor_attributes[last_idx++] = &dev_attr_##name.attr; \
	} else { \
		dev_err(md->dev, "cannot add attribute '%s'\n", #name); \
	} \
}

static struct attribute_group motor_attribute_group = {
	.attrs = motor_attributes
};

static const struct attribute_group * motor_attr_groups[] = {
	&motor_attribute_group,
	NULL
};

/* method to report/enforce position detection */
static ssize_t motor_detect_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t len)
{
	motor_device* md = (motor_device*)dev_get_drvdata(dev);
	POSITION_DETECT_INIT(POS_COMPACT, STATUS_STOPPED_COMPACT);
	return len;
}

static ssize_t motor_detect_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	motor_device* md = (motor_device*)dev_get_drvdata(dev);
	return snprintf(buf, 20, "%s\n", position_labels[atomic_read(&md->position)]);
}

/* HAL communicates desired position through this sysfs */
static ssize_t motor_position_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t len)
{
	motor_device* md = (motor_device*)dev_get_drvdata(dev);
	unsigned long flags;
	unsigned value = 0;

	if (kstrtouint(buf, 10, &value) ||
		/* at this point it's well known position */
		(value <= POS_UNKNOWN || value > POS_PEEK)) {
		dev_err(dev, "Error value: %s\n", buf);
		return -EINVAL;
	}
	spin_lock_irqsave(&md->mlock, flags);
	atomic_set(&md->destination, value);
	spin_unlock_irqrestore(&md->mlock, flags);
	dev_info(md->dev, "expected position %s\n", position_labels[value]);

	return len;
}

static ssize_t motor_position_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	motor_device* md = (motor_device*)dev_get_drvdata(dev);
	int ret;

	ret = wait_event_interruptible(md->position_wait, md->position_update_ready);
	if (ret)
		return -EFAULT;
	md->position_update_ready = false;
	LOGD("Position update: %s\n", position_labels[atomic_read(&md->position)]);

	return snprintf(buf, 20, "%d", atomic_read(&md->position));
}

static ssize_t motor_status_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	motor_device* md = (motor_device*)dev_get_drvdata(dev);
	int ret;

	ret = wait_event_interruptible(md->status_wait, md->status_update_ready);
	if (ret)
		return -EFAULT;
	md->status_update_ready = false;
	LOGD("Status update: %s\n", status_labels[atomic_read(&md->status)]);

	return snprintf(buf, 20, "%d", atomic_read(&md->status));
}
/*
 * Input data expected in the following format:
 * Decimal_1 Decimal_2 [Decimal_3]
 * where Decimal_3 is optional
 */
static ssize_t motor_sensor_data_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t len)
{
	motor_device* md = (motor_device*)dev_get_drvdata(dev);
	int argnum = 0;
	int volatile args[3] = {OUT_OF_RANGE};
	static bool inited;

	argnum = sscanf(buf, "%d %d %d", &args[0], &args[1], &args[2]);
	PARANOIC("sensor_data (%d): %d %d %d\n", argnum, args[0], args[1], args[2]);
	if (argnum > 0 && argnum >= ARGS_NUM_MIN) {
		mutex_lock(&md->mx_lock);
		if (!inited) {
			/* First sample arrives when sensors HAL gets loaded
			 * This will be a good point to synchronize intial
			 * position detection with sensors data availability
			 */
			if (md->sensor_data[0] == UNINITIALIZED) {
				inited = true;
				if (!md->sensors_off) {
					md->time_out = MOTOR_DETECT_EXPIRE;
					moto_drv8424_cmd_push(md, CMD_TIMER_START, 0);
				}
			}
		}
		/* store necessary sensor info */
		memcpy((void *)md->sensor_data, (void *)args, sizeof(md->sensor_data));
		logtime_store((void *)args, sizeof(args), 0);
		atomic_inc(&md->samples);
		mutex_unlock(&md->mx_lock);
		up(&md->data_ready);
	}

	return len;
}

/* sysfs polled from  HAL */
static DEVICE_ATTR(position, S_IRUGO|S_IWUSR|S_IWGRP, motor_position_show, motor_position_store);
static DEVICE_ATTR(detect, S_IRUGO|S_IWUSR|S_IWGRP, motor_detect_show, motor_detect_store);
static DEVICE_ATTR(status, S_IRUGO, motor_status_show, NULL);
static DEVICE_ATTR(sensor_data, S_IWUSR|S_IWGRP, NULL, motor_sensor_data_store);

static struct attribute *status_attributes[] = {
	&dev_attr_status.attr,
	&dev_attr_position.attr,
	&dev_attr_detect.attr,
	&dev_attr_sensor_data.attr,
	NULL
};

static struct attribute_group status_attribute_group = {
	.attrs = status_attributes
};

static int moto_drv8424_init_from_dt(motor_device* md)
{
	struct device* pdev = md->dev;
	struct device_node *np = pdev->of_node;
	motor_control* mc = &md->mc;
	int i, rc = 0;
	char gpio_name[32];
	const char *clock_name;

	mc->tab_cells = MOTOR_MAX_GPIO;
	for (i = 0; i < mc->tab_cells; i++) {
		snprintf(gpio_name, sizeof(gpio_name)-1, "%s-gpio", gpios_labels[i]);
		md->mc.ptable[i] = of_get_named_gpio(np, gpio_name, 0);
		if (gpio_is_valid(md->mc.ptable[i])) {
			dev_info(pdev, "gpio %s = %d\n", gpios_labels[i], md->mc.ptable[i]);
		}
	}
	if (!of_property_read_string(np, "clock-names", &clock_name))
		strlcpy(md->clock_name, clock_name, CLOCK_NAME_LEN);
	else
		strlcpy(md->clock_name, MOTOR_HW_CLK_NAME, CLOCK_NAME_LEN);
	dev_dbg(pdev, "Clock name: %s\n", md->clock_name);
	md->hw_clock = of_property_read_bool(np, "enable-hw-clock");
	dev_dbg(pdev, "Enable hw clock %d\n", md->hw_clock);
	md->support_mode = of_property_read_bool(np, "support-mode");
	if (md->support_mode) {
		ATTR_ADD(mode);
		dev_info(pdev, "Support mode %d\n", md->support_mode);
	}
	md->support_torque = of_property_read_bool(np, "support-torque");
	if (md->support_torque) {
		ATTR_ADD(torque);
		dev_info(pdev, "Support torque %d\n", md->support_torque);
	};
	md->sensors_off = of_property_read_bool(np, "no-sensors");
	if (md->sensors_off)
		dev_info(pdev, "Ignore sensors data\n");
	md->power_default_off = of_property_read_bool(np, "power-default-off");
	dev_info(pdev, "power is default off: %d\n", md->power_default_off);
	md->use_2p8_pitch = of_property_read_bool(np, "use-2p8-pitch");
	if (md->use_2p8_pitch)
		dev_info(pdev, "use 2.8 pitch\n");

	return rc;
}

static int moto_drv8424_irq_setup(motor_device *md, unsigned gidx)
{
	static int id;
	int ret, irq_gpio = md->mc.ptable[gidx];
	static char *irq_name[2];

	if (!gpio_is_valid(irq_gpio))
		return ENOTCONN;
	irq_name[id] = devm_kzalloc(md->dev, 32, GFP_KERNEL);
	if (!irq_name[id])
		return -ENOMEM;
	snprintf(irq_name[id], 31, "fault_motor%d", id+1);
	ret = devm_request_threaded_irq(md->dev,
			gpio_to_irq(irq_gpio), motor_fault_irq, NULL,
			IRQF_TRIGGER_LOW | IRQF_ONESHOT, irq_name[id], md);
	if (ret) {
		dev_err(md->dev, "Failed to register \'%s\': %d\n",
			irq_name[id], ret);
	} else {
		dev_info(md->dev, "Registered IRQ %d as '%s'\n",
			gpio_to_irq(irq_gpio), irq_name[id]);
		/* disable immediately */
		disable_irq(gpio_to_irq(irq_gpio));
	}
	id++;

	return ret;
}

static void motor_set_sequencer(motor_device* md)
{
	motor_stage *stages;
	int s, i;

	for (s = SQ_FULL; s < SQ_MAX; s++) {
		PARANOIC("regime %s\n", regime_names[s]);
		/* skip dummy */
		if (s == SQ_SHOW_MAX)
			continue;
		if (md->use_2p8_pitch)
			stages = initial_data_2p8[s];
		else
			stages = initial_data_2p3[s];
	        for (i = 0; i < MAX_STAGE_LEGS; i++) {
	                if (!((stages+i)->freq | (stages+i)->ceiling))
	                        break;
	                md->sequencer[s][i].freq = (stages+i)->freq;
	                md->sequencer[s][i].ceiling = (stages+i)->ceiling;
	        }
		PARANOIC("has %d pairs\n", i);
#if 0
		/* skip adding slow down stage if running in no */
		/* sensors mode and for TINY & SLOWMO regimes */
		if (md->sensors_off || s == SQ_TINY || s == SQ_SLOWMO)
			continue;
		if (++i < MAX_STAGE_LEGS) {
			md->sequencer[s][i].freq = 800;
			md->sequencer[s][i].ceiling = 60;
			LOGD("added slow down stage to %s\n", regime_names[s]);
		}
#endif
	}
}

static int moto_drv8424_probe(struct platform_device *pdev)
{
	struct device* dev = &pdev->dev;
	motor_device* md;
	int i;
	int ret = 0;

	md = kzalloc(sizeof(motor_device), GFP_KERNEL);
	if (!md) {
		dev_err(dev, "probe: Out of memory\n");
		return -ENOMEM;
	}
	md->dev = dev;
	md->mc.plabels = gpios_labels;
	spin_lock_init(&md->mlock);
	mutex_init(&md->mx_lock);
	sema_init(&md->data_ready, 0);
	platform_set_drvdata(pdev, md);
	moto_drv8424_set_step_freq(md, DEFAULT_STEP_FREQ);
	/* assign default values for optional parameters */
	md->mode = FULL_STEP;
	md->torque = TORQUE_FULL;
	moto_drv8424_init_from_dt(md);
	/* populate sequencer with initial values */
	motor_set_sequencer(md);

	md->mc.vdd = devm_regulator_get(md->dev, "vdd");
	if (IS_ERR(md->mc.vdd)) {
		ret = PTR_ERR(md->mc.vdd);
		dev_err(md->dev, "Failed to get VDD ret=%d\n", ret);
		goto failed_mem;
	}
	if (!md->power_default_off) {
		ret = moto_drv8424_set_regulator_power(md, true);
		if (ret) {
			regulator_put(md->mc.vdd);
			dev_err(dev, "Failed enable regulator\n");
			goto failed_mem;
		}
	}
	md->motor_wq = alloc_workqueue("motor_wq", WQ_HIGHPRI, 0);
	if(!md->motor_wq) {
		dev_err(dev, "Out of memory for work queue\n");
		goto failed_mem;
	}
	INIT_DELAYED_WORK(&md->motor_work, motor_cmd_work);
	ret = kfifo_alloc(&md->cmd_pipe, sizeof(unsigned int)* 10, GFP_KERNEL);
	if (ret) {
		dev_err(dev, "Failed create fifo\n");
		goto failed_work;
	}
	md->status_update_ready = false;
	init_waitqueue_head(&md->status_wait);
	md->position_update_ready = false;
	init_waitqueue_head(&md->position_wait);
	md->user_sync_complete = false;
	init_waitqueue_head(&md->sync_complete);
	md->motor_task = kthread_run(motor_kthread, md, "motor_task");
	if (IS_ERR(md->motor_task)) {
		ret = PTR_ERR(md->motor_task);
		dev_err(dev, "Failed create motor kthread\n");
		goto failed_kfifo;
	}
	md->data_update_ready = false;
	init_waitqueue_head(&md->data_wait);
	md->detection_task = kthread_run(detection_kthread, md, "detection_task");
	if (IS_ERR(md->detection_task)) {
		ret = PTR_ERR(md->detection_task);
		dev_err(dev, "Failed create position detection kthread\n");
		goto failed_kthread;
	}
	hrtimer_init(&md->stepping_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	md->stepping_timer.function = motor_stepping_timer_action;
	hrtimer_init(&md->timeout_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	md->timeout_timer.function = motor_timeout_timer_action;
	md->drv8424_class = class_create(THIS_MODULE, MOTOR_CLASS_NAME);
	if(IS_ERR(md->drv8424_class)) {
		dev_err(dev, "Failed to create class\n");
		goto failed_sys;
	}
	md->sysfs_dev = device_create_with_groups(md->drv8424_class,
			     dev, MKDEV(0, 0), md, motor_attr_groups,
			     "%s", MOTOR_CONTROL);
	if (IS_ERR(md->sysfs_dev)) {
		dev_err(dev, "Failed to create device\n");
		goto failed_sys_dev;
	}
	if (sysfs_create_group(&md->dev->kobj, &status_attribute_group)) {
		dev_err(dev, "Failed to create status attribute");
		goto failed_device;
	}
	if (md->hw_clock) {
		if (init_motor_clk(md) < 0) {
			//Should get this gpio from DTB
			dev_err(dev, "HW clock is not setup\n");
		} else {
			md->cur_clk = hw_clocks[0];
			dev_info(dev, "HW clock is setup\n");
		}
	}
	/* set initial pinctrl */
	set_pinctrl_state(md, INIT_STATE);
	//Init software clock pin.
	set_pinctrl_state(md, CLK_SLEEP);
	for (i = 0; i < MOTOR_UNKNOWN; i++) {
		if (!gpio_is_valid(md->mc.ptable[i]))
			continue;
		ret = devm_gpio_request(dev, md->mc.ptable[i], gpios_labels[i]);
		if(ret < 0) {
			pr_err("Failed to request %s, errno %d\n", gpios_labels[i--], ret);
			goto failed_gpio;
		}
	}
	if (!set_pinctrl_state(md, INT_DEFAULT)) {
		ret = moto_drv8424_irq_setup(md, MOTOR_FAULT1_INT);
			if (ret < 0)
				goto failed_gpio;
		ret = moto_drv8424_irq_setup(md, MOTOR_FAULT2_INT);
			if (ret < 0)
				goto failed_gpio;
	} else {
		/* motor can work, but have no fault irq */
		dev_info(dev, "Device has no fault irq\n");
	}
	/* get ready for initial position detection */
	RESET_SENSOR_DATA(UNINITIALIZED);
	POSITION_DETECT_INIT(POS_COMPACT, STATUS_STOPPED_COMPACT);
	dev_info(dev, "Success init device\n");
	if (!md->sensors_off)
		dev_info(dev, "running position detection...\n");

	logtime_alloc(sizeof(int)*3);

	return 0;

failed_gpio:
	while (i >= MOTOR_POWER_EN) {
		if (gpio_is_valid(md->mc.ptable[i]))
			devm_gpio_free(dev, md->mc.ptable[i]);
		i--;
	}
	sysfs_remove_group(&md->dev->kobj, &status_attribute_group);
failed_device:
	device_destroy(md->drv8424_class, MKDEV(0, 0));
failed_sys_dev:
	class_destroy(md->drv8424_class);
failed_sys:
	kthread_stop(md->detection_task);
failed_kthread:
	kthread_stop(md->motor_task);
failed_work:
	destroy_workqueue(md->motor_wq);
failed_kfifo:
	kfifo_free(&md->cmd_pipe);
failed_mem:
	kfree(md);

	return ret;
}

static int moto_drv8424_remove(struct platform_device *pdev)
{
	motor_device* md = (motor_device*)platform_get_drvdata(pdev);

	set_irq_state(md, false);
	moto_drv8424_set_power(md, 0);
	moto_drv8424_set_regulator_power(md, false);
	devm_regulator_put(md->mc.vdd);
	device_destroy(md->drv8424_class, MKDEV(0, 0));
	class_destroy(md->drv8424_class);
	sysfs_remove_group(&md->dev->kobj, &status_attribute_group);
	disable_motor(md->dev);
	kthread_stop(md->motor_task);
	kthread_stop(md->detection_task);
	destroy_workqueue(md->motor_wq);
	kfifo_free(&md->cmd_pipe);
	kfree(md);

	return 0;
}

void moto_drv8424_platform_shutdown(struct platform_device *pdev)
{
	motor_device* md = (motor_device*)platform_get_drvdata(pdev);
	int curpos = atomic_read(&md->position);

	LOGD("shutdown handler\n");
	if (curpos != POS_COMPACT && curpos != POS_UNKNOWN) {
		if (!md->sensors_off) {
			/* unable to guarantee sensors will stay up */
			/* all the way until slider reached destination */
			/* thus not relying on sensors for this transition */
			md->sensors_off = true;
			LOGD("turned off sensors\n");
		}
		md->power_en = 0;
		motor_set_motion_params(md, curpos, POS_COMPACT);
		atomic_set(&md->destination, POS_COMPACT);
		moto_drv8424_set_enable_with_power(md, true);
		msleep(5000);
	}
}

static const struct of_device_id moto_drv8424_match_table[] = {
	{.compatible = "moto,drv8424"},
	{},
};
MODULE_DEVICE_TABLE(of, moto_drv8424_match_table);

static struct platform_driver moto_drv8424_driver = {
	.probe = moto_drv8424_probe,
	.remove = moto_drv8424_remove,
	.shutdown = moto_drv8424_platform_shutdown,
	.driver = {
		.name = "moto,drv8424",
		.owner = THIS_MODULE,
		.of_match_table = moto_drv8424_match_table,
	},
};

static int __init moto_drv8424_motor_init(void)
{
	return platform_driver_register(&moto_drv8424_driver);
}

module_init(moto_drv8424_motor_init);

static void __exit moto_drv8424_motor_exit(void)
{
	platform_driver_unregister(&moto_drv8424_driver);
}
module_exit(moto_drv8424_motor_exit);

MODULE_DESCRIPTION("Motorola TI DRV8424 Driver");
MODULE_LICENSE("GPL v2");
