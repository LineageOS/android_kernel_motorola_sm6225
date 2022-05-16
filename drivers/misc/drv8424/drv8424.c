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
#include <uapi/linux/sched/types.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#include <linux/kfifo.h>
#include <linux/clk-provider.h>

#define DRV8424_INIT_DETECTION 1

#define DEFAULT_STEP_FREQ 2400
#define MOTOR_CLASS_NAME  "drv8424"
#define MOTOR_CONTROL  "control"
#define MOTOR_HW_CLK         9600 //9.6KHz clock
#define MOTOR_HW_CLK_NAME "gcc_gp3"

#define MOTOR_DEFAULT_EXPIRE 3000 //3000ms timeout

#if defined(DRV8424_INIT_DETECTION)
#define MOTOR_DETECT_EXPIRE 600000 //10min timeout
#define POLL_INIT_INT 5000 //start polling sensor data in Xms
#define POLL_NEXT_INT 1000
#else
#define MOTOR_DETECT_EXPIRE 500 //500ms timeout
#define POLL_INIT_INT 100 //start polling sensor data in Xms
#define POLL_NEXT_INT 50
#endif

#define MOTOR_MODE_SPEED 0
#define MOTOR_MODE_STEP 1

#define DIR_EXTEND 1
#define DIR_WITHDRAW 0

#define LOGD(fmt, args...) pr_err(fmt, ##args)
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
 * Assumption is that in PEEK position sensor will not be aligned exactly
 * above the magnet and thus value of 1500 corresponds to the angle of
 * 15 degrees. Everything is just for the reference purpose and will be
 * adjusted as more details become available
 */
static sensor_scan_t scanner[POS_MAX] = {
	{-1, -1},
	{0, 0},		/* compact position: sensor[0], value 0 degrees */
	{1, 0},		/* expanded position: sensor[1], value 0 degrees */
	{1, 1500},	/* peek position: sensor[1], value 15 degrees */
};

enum status_id {
	STATUS_UNKNOWN,
	STATUS_STOPPED_COMPACT,
	STATUS_STOPPED_EXPANDED,
	STATUS_STOPPED_PEEK,
	STATUS_MOVING_OUT,
	STATUS_MOVING_IN,
	STATUS_QUERYING_POS,
};

static const char *status_labels[] = {
    "UNKNOWN",
    "STOPPED_COMPACT",
    "STOPPED_EXPANDED",
	"STOPPED_PEEK",
	"EXPANDING",
	"WITHDRAWING",
	"QUERYING_POSITION",
};

enum gpios_index {
    MOTOR_POWER_EN = 0, /* BOOST */
    MOTOR_FAULT_INT,	/* nFAULT INT */
    MOTOR_STEP,			/* STEP */
    MOTOR_DIR,			/* DIR */
    MOTOR_MODE1,		/* set to 0 in HW */
    MOTOR_MODE2,		/* set to 0 in HW */
    MOTOR_EN,			/* set to 1 in HW */
    MOTOR_SLEEP,		/* nSLEEP */
    MOTOR_T0,			/* set to 0 in HW */
    MOTOR_T1,			/* set to 0 in HW */
	MOTOR_ACTIVE,		/* signal GPIO */
    MOTOR_UNKNOWN
};

static const char* const gpios_labels[] = {
    "MOTOR_POWER_EN",
    "MOTOR_FAULT_INT",
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

static const int def_gpios_table[] = {
    0, 0, 136, 0, 0, 0, 0, 0, 0, 0, 0
//    52, 57, 21, 32, 30, 51, 29, 50, 49, 33, -1
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
	SQ_SHORTENED,
	SQ_SHORT,
	SQ_TINY,
	SQ_MAX,
	SQ_SLOWMO,
};

static const char *regime_names[] = {
	"FULL",
	"SHORTENED",
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

static motor_stage initial_data[SQ_SLOWMO + 1][MAX_STAGE_LEGS] = {
	{ /* FULL 44mm */
		{400,24}, {1400,56}, {3000,6252}, {2000,12}
	},
	{ /* SHORTENED 39mm */
		{400,24}, {1400,56}, {3000,5530}, {2000,12}
	},
	{ /* SHORT 5mm */
		{400,24}, {1400,56}, {3000,630}, {2000,12}
	},
	{ /* TINY 1mm */
		{400,24}, {1400,56}, {2000,65}
	},
	{ /* dummy to continue beyond SQ_MAX */
		{0, 0},
	},
	{ /* SLOWMO */
		{400,100}, {400,200}, {400,400}, {400,800}, {400,1600},
	},
};

static unsigned timeoutMs[] = { 3000, 2500, 500, 100, 0, 5000 };

enum cmds {
	CMD_FAULT,
	CMD_STATUS,
	CMD_POSITION,
	CMD_POLL,
};

#define MAX_GPIOS MOTOR_UNKNOWN
typedef struct motor_control {
    struct regulator    *vdd;
    int32_t ptable[MAX_GPIOS];
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
    int fault_irq;
    bool faulting;
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
    int      level:1;
    unsigned power_en:1;
    unsigned nsleep:1;
    unsigned nEN:1;
    unsigned dir:1;
    unsigned user_sync_complete:1;
	unsigned status_update_ready:1;
	unsigned position_update_ready:1;
	struct kfifo cmd_pipe;
    int sensor_data[3];
} motor_device;

#define RESET_SENSOR_DATA do { \
		md->sensor_data[0] = md->sensor_data[1] = \
		md->sensor_data[2] = OUT_OF_RANGE; \
	} while(0)

#define GPIO_OUTPUT_DIR(g, p) do { \
		if (gpio_is_valid(g)) \
			gpio_direction_output(g, p); \
	} while(0)

static int set_pinctrl_state(motor_device* md, unsigned state_index);
static void moto_drv8424_set_step_freq(motor_device* md, unsigned freq);

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
    if(usec < 500) { //<500us, udelay
        udelay(usec);
    } else if(usec >= 500 && usec < 20000) { //500us - 20ms, usleep_range
        usleep_range(usec, usec + 10);
    } else { //>= 20ms, msleep
        msleep(usec);
    }
}

static int moto_drv8424_set_regulator_power(motor_device* md, bool en)
{
    motor_control * mc = &md->mc;
    int err = 0;
//FIXME
return 0;
    if(en) {
        err = regulator_enable(mc->vdd);
        if (err) {
            dev_err(md->dev, "Failed to enable VDD ret=%d\n", err);
            goto exit;
        }
    } else {
        err = regulator_disable(mc->vdd);
        if (err) {
            dev_err(md->dev, "Failed to disable VDD ret=%d\n", err);
            goto exit;
        }
    }

    return 0;

exit:
    return err;
}
#if 0
static int set_motor_clk(motor_device* md, bool en)
{
    int ret = 0;

    if(en) {
        set_pinctrl_state(md, CLK_ACTIVE);
        ret = clk_set_rate(md->pwm_clk, md->cur_clk);
        if(ret < 0) {
            dev_err(md->dev, "Failed to set clk rate to %ld\n", md->cur_clk);
            ret = -ENODEV;
            goto soft_clk;
        }
        ret = clk_prepare_enable(md->pwm_clk);
        if(ret < 0) {
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
    if(IS_ERR(md->pwm_clk)) {
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

    if(state_index >= PINS_END) {
        dev_err(md->dev, "Illegal pin index\n");
        goto err;
    }

    mc->pins = devm_pinctrl_get(md->dev);
    if(IS_ERR_OR_NULL(mc->pins)) {
        ret = PTR_ERR(mc->pins);
        dev_err(md->dev, "Failed to get pinctrl %d\n", ret);
        goto err;
    }

    mc->pin_state[state_index] = pinctrl_lookup_state(mc->pins, pins_state[state_index]);
    if (IS_ERR_OR_NULL(mc->pin_state[state_index])) {
        ret = PTR_ERR(mc->pin_state[state_index]);
        dev_err(md->dev, "Failed to lookup pin_state[%d] %d\n", state_index, ret);
        goto err_pin_state;
    }

    ret = pinctrl_select_state(mc->pins, mc->pin_state[state_index]);
    if(ret) {
        dev_err(md->dev, "Failed to set pin_state[%d] %d\n", state_index, ret);
    }

err_pin_state:
    pinctrl_put(mc->pins);
err:
    return ret;
}

static void moto_drv8424_set_motor_torque(motor_device* md)
{
	if (!md->support_torque)
		return;

    switch(md->torque) {
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

    switch(md->mode) {
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

    if(md->cur_mode != md->mode) {
        if(md->hw_clock) {
            if(md->mode == FULL_STEP) {
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
	LOGD("pin DIR updated\n");
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
	LOGD("pins nSLEEP & ENABLE updated\n");
}

static int moto_drv8424_set_opmode(motor_device* md, unsigned opmode)
{
    unsigned long flags;
    int ret = 0;

    spin_lock_irqsave(&md->mlock, flags);

    if(md->nsleep == opmode) {
        dev_info(md->dev, "Unchanged the opmode status, ignore\n");
        ret = -EINVAL;
        goto exit;
    }
    md->nsleep = !!opmode;
    md->nEN = !md->nsleep;
    spin_unlock_irqrestore(&md->mlock, flags);

	LOGD("opmode: nsleep=%d, en=%d\n", md->nsleep, md->nEN);

    return 0;

exit:
    spin_unlock_irqrestore(&md->mlock, flags);
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

	if (en) {
		/* signal sensor hub to start/stop sampling hall effect */
		GPIO_OUTPUT_DIR(mc->ptable[MOTOR_ACTIVE], 1);
		hrtimer_start(&md->timeout_timer, adapt_time_helper(md->time_out), HRTIMER_MODE_REL);
		moto_drv8424_cmd_push(md, CMD_POLL, msecs_to_jiffies(POLL_INIT_INT));
		/* declare sensor querying stage */
		atomic_set(&md->status, STATUS_QUERYING_POS);
		moto_drv8424_cmd_push(md, CMD_STATUS, msecs_to_jiffies(POLL_INIT_INT));
	} else {
		hrtimer_try_to_cancel(&md->timeout_timer);
		GPIO_OUTPUT_DIR(mc->ptable[MOTOR_ACTIVE], 0);
	}
	LOGD("gpio ACTIVE set: %u\n", en ? 1 : 0);
}

//Set VDD
static void moto_drv8424_set_power_en(motor_device* md)
{
    motor_control* mc = &md->mc;

    GPIO_OUTPUT_DIR(mc->ptable[MOTOR_POWER_EN], md->power_en);
    //Tpower 0.5ms
    usleep_range(500, 1000);
	LOGD("gpio BOOST set: %u\n", md->power_en);
	moto_drv8424_set_sensing(md, md->power_en ? true : false);
}

//Power sequence: PowerEn On->nSleep On->nSleep Off->PowerEn Off
static int moto_drv8424_set_power(motor_device* md, unsigned power)
{
    unsigned long flags;
    int ret = 0;

    if(md->power_en == power) {
        dev_info(md->dev, "Unchanged the power status, ignore\n");
        ret = -EINVAL;
        goto exit;
    }
	LOGD("power_en=%u\n", !!power);
    spin_lock_irqsave(&md->mlock, flags);
    md->power_en = !!power;
    spin_unlock_irqrestore(&md->mlock, flags);

    ret = moto_drv8424_set_opmode(md, md->power_en);
    if(ret < 0 ) {
        goto exit;
    }
    if(!md->power_en) {
        moto_drv8424_set_motor_opmode(md);
    }
    moto_drv8424_set_power_en(md);

    return 0;

exit:
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
    md->max_stages = msn;
	md->time_out = timeoutMs[regime];
    spin_unlock_irqrestore(&md->mlock, flags);

	dev_info(md->dev, "Set active regime: %s, stages # %u\n",
			regime_names[md->regime], msn);
}

#if 0
static int moto_drv8424_enable_clk(motor_device* md, bool en)
{
    return set_motor_clk(md, en);
}
#endif
#ifdef MOTOR_SLOT_DBG
static ktime_t timesA[20];
static unsigned	slot;

static void inline logtime_store(void)
{
	if ((slot + 1) < sizeof(timesA)/sizeof(timesA[0])) {
		timesA[slot++] = ktime_get();
	}
}

static void logtime_show(void)
{
	int i;
	ktime_t diff;

	if (!slot)
		return;

	for (i = 1; i < slot; i++) {
		diff = ktime_sub(timesA[i], timesA[i-1]);
		pr_err("step[%d]=%lldus\n", i, ktime_to_us(diff));
	}
}

static void inline logtime_reset(void)
{
	slot = 0U;
}
#else
#define logtime_store()
#define logtime_show()
#define logtime_reset()
#endif

static void moto_drv8424_drive_stage_init(motor_device* md)
{
	logtime_reset();

    atomic_set(&md->stepping, 1);
    atomic_set(&md->step_count, 1);
    md->double_edge = false;
    md->half = md->step_period >> 1;
    md->level = 1;
    if(md->mode == STEP_8_BOTH_EDEG
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

	logtime_show();

	ms = &md->sequencer[md->regime][md->stage - 1];
	freq = ms->freq;
	md->step_ceiling = ms->ceiling;
	moto_drv8424_set_step_freq(md, freq);
	moto_drv8424_drive_stage_init(md);

	return true;
}

static int moto_drv8424_drive_sequencer(motor_device* md)
{
	moto_drv8424_next_stage(md);
    moto_drv8424_set_motor_torque(md);
    moto_drv8424_set_motor_dir(md);
    moto_drv8424_set_motor_mode(md);
    moto_drv8424_set_motor_opmode(md);
	/* can judge about moving direction based on DIR or destination position */
	atomic_set(&md->status, md->dir ? STATUS_MOVING_OUT : STATUS_MOVING_IN);
	moto_drv8424_cmd_push(md, CMD_STATUS, 0);
	LOGD("sequencer init: stages %u ceiling %lu, freq %uHz period %luns, half %uns\n",
			md->max_stages, md->step_ceiling, md->step_freq, md->step_period, md->half);

    if(atomic_read(&md->stepping)) {
        motor_control* mc = &md->mc;

		LOGD("Status updated: status %d\n", atomic_read(&md->status));
        GPIO_OUTPUT_DIR(mc->ptable[MOTOR_STEP], md->level);
        atomic_inc(&md->step_count);
        hrtimer_start(&md->stepping_timer, adapt_time_helper(md->half), HRTIMER_MODE_REL);
		logtime_store();
    }

    return 0;
}

/* this function is called with spinlock running!!! */
static void inline motor_stop(motor_device* md, bool clean)
{
	//LOGD("step count %d\n", atomic_read(&md->step_count));
	if (clean) {
        atomic_set(&md->step_count, 0);
        atomic_set(&md->stepping, 0);
		//LOGD("step_count & stepping reset\n");
	}
	//LOGD("waking up kthread\n");
    md->user_sync_complete = true;
    wake_up(&md->sync_complete);
}

static __ref int motor_kthread(void *arg)
{
    motor_device* md = (motor_device*)arg;
    struct sched_param param = {.sched_priority = MAX_USER_RT_PRIO - 1};
    int value, ret = 0;

    sched_setscheduler(current, SCHED_FIFO, &param);
    while (!kthread_should_stop()) {
        LOGD("wait for event\n");
        do {
            ret = wait_event_interruptible(md->sync_complete,
                        md->user_sync_complete || kthread_should_stop());
        } while (ret != 0);

        if(kthread_should_stop())
            break;

        md->user_sync_complete = false;
		/* it's safe to call the following functions even if motor was not running */
        moto_drv8424_set_power(md, 0);
        if (md->power_default_off) {
            dev_info(md->dev, "vdd power off\n");
            moto_drv8424_set_regulator_power(md, false);
        }
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
		}

		atomic_set(&md->status, value);
		moto_drv8424_cmd_push(md, CMD_STATUS, 0);

		LOGD("Status updated: status %d, position %d\n",
				atomic_read(&md->status), atomic_read(&md->position));
		logtime_show();
    }

    return 0;
}

static int disable_motor(struct device* dev)
{
    motor_device* md = (motor_device*)dev_get_drvdata(dev);
    ktime_t time_rem;
    int ret = 0;

    if(atomic_read(&md->stepping)) {
        atomic_set(&md->stepping, 0);
        atomic_set(&md->step_count, 0);

        time_rem = hrtimer_get_remaining(&md->stepping_timer);
        if(ktime_to_us(time_rem) > 0) {
            hrtimer_try_to_cancel(&md->stepping_timer);
        }
    }

    return ret;
}

static int motor_set_enable(struct device* dev, bool enable)
{
    motor_device* md = (motor_device*)dev_get_drvdata(dev);

    if(atomic_read(&md->stepping)) {
        disable_motor(dev);
    }

    if(enable) {
       moto_drv8424_drive_sequencer(md);
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

    if(!moto_drv8424_set_power(md, enable)) {
        motor_set_enable(md->dev, enable);
    }
}

static void motor_set_motion_params(motor_device *md, int start, int end)
{
	int regime = SQ_FULL;

	if (start == POS_EXPANDED) {
		md->dir = DIR_WITHDRAW;
		if (end == POS_COMPACT)
			regime = SQ_FULL;
		else // end == PEEK
			regime = SQ_SHORTENED;
	} else if (start == POS_COMPACT) {
		md->dir = DIR_EXTEND;
		if (end == POS_EXPANDED)
			regime = SQ_FULL;
		else // end == PEEK
			regime = SQ_SHORT;
	} else { // start == PEEK
		if (end == POS_EXPANDED) {
			md->dir = DIR_EXTEND;
			regime = SQ_SHORTENED;
		} else { // end == COMPACT
			md->dir = DIR_WITHDRAW;
			regime = SQ_SHORT;
		}
	}

	moto_drv8424_set_motor_dir(md);
	moto_drv8424_set_regime(md, regime);
}

static enum hrtimer_restart motor_stepping_timer_action(struct hrtimer *h)
{
    motor_device * md = container_of(h, motor_device, stepping_timer);
    unsigned long flags;
    enum hrtimer_restart ret = HRTIMER_RESTART;

    spin_lock_irqsave(&md->mlock, flags);

    if(!atomic_read(&md->stepping))
    	goto next_stage;

    md->level = !md->level;
    GPIO_OUTPUT_DIR(md->mc.ptable[MOTOR_STEP], md->level);

    if(md->double_edge) {
        atomic_inc(&md->step_count);
    } else if(md->level) {
        atomic_inc(&md->step_count);
    }

	logtime_store();

    if(md->step_ceiling && atomic_read(&md->step_count) > md->step_ceiling) {
next_stage:
		/* multi stage sequence */
		if (!moto_drv8424_next_stage(md)) {
			motor_stop(md, true);
			GPIO_OUTPUT_DIR(md->mc.ptable[MOTOR_STEP], 0);
			spin_unlock_irqrestore(&md->mlock, flags);
			return HRTIMER_NORESTART;
		}
    }

    hrtimer_forward_now(h, adapt_time_helper(md->half));
    spin_unlock_irqrestore(&md->mlock, flags);

    return ret;
}

/*
 * Timeout can happen either if destination position never reached or
 * driver cannot detect current position. In either way timeout handler
 * will try enforcing COMPACT state
 */
static enum hrtimer_restart motor_timeout_timer_action(struct hrtimer *h)
{
    motor_device * md = container_of(h, motor_device, stepping_timer);
	int original = atomic_read(&md->position);
	int destination = atomic_read(&md->destination);
    enum hrtimer_restart ret = HRTIMER_NORESTART;

	LOGD("Timeout: position=%d, detection=%d\n", original, destination);
#if 0
	/* Timed out arriving to destination from known starting position.
	 * Solution would be to go back to starting position
	 */
	if (original != POS_UNKNOWN) {
		LOGD("Timeout driving motor\n");
		/* move back to the original position */
		atomic_set(&md->destination, original);
		atomic_set(&md->position, POS_UNKNOWN);
	}
#endif
	/* timed out position detection cycle */
	if (original == POS_UNKNOWN && destination == POS_UNKNOWN) {
		LOGD("Timeout position detection\n");
		md->power_en = 0;
		//FIXME: assuming worst case might not be the best solution here!!!
		motor_set_motion_params(md, POS_EXPANDED, POS_COMPACT);
		atomic_set(&md->position, POS_EXPANDED);
		atomic_set(&md->destination, POS_COMPACT);
		moto_drv8424_set_enable_with_power(md, true);
	}

    return ret;
}

#define OUT_OF_RANGE 10000
#define PROXIMITY 10
static inline bool IN_RANGE(int valMeas, int valSet)
{
	return (valMeas >= (valSet - PROXIMITY) &&
		valMeas <= (valSet + PROXIMITY)) ? true : false;
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
	LOGD("scan sensor(s): %d - %d\n", start, end);
	mutex_lock(&md->mx_lock);
	for (i = start; i < end; i++) {
		if (IN_RANGE(md->sensor_data[scanner[i].index], scanner[i].value)) {
			ret = i;
			atomic_set(&md->stepping, 0);
			LOGD("sensor within range: (%d) < %d < (%d)\n",
					scanner[i].value - PROXIMITY,
					md->sensor_data[scanner[i].index],
					scanner[i].value + PROXIMITY);
			break; // assuming position is exclusive!!!
		}
	}
	mutex_unlock(&md->mx_lock);
	if (ret)
		LOGD("detected position: %d\n", ret);

	return ret;
}

#define RESET_TIME 1000
static void motor_cmd_work(struct work_struct *work)
{
	struct delayed_work *dw = container_of(work, struct delayed_work, work);
    motor_device* md = container_of(dw, motor_device, motor_work);
	bool polling = false;
	int ret, cmd = 0;

	while (kfifo_get(&md->cmd_pipe, &cmd)) {
		switch (cmd) {
		case CMD_FAULT:
			disable_irq(md->fault_irq);
			if(atomic_read(&md->stepping)) {
				disable_motor(md->dev);
			}
			moto_drv8424_set_power(md, 0);
			msleep(RESET_TIME);
			moto_drv8424_set_power(md, 1);
			md->faulting = false;
			enable_irq(md->fault_irq);
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
			ret = moto_drv8424_detect_position(md);
			if (ret == POS_UNKNOWN) {
				polling = true;
			} else {
				RESET_SENSOR_DATA;
				if (atomic_read(&md->stepping) != 0) {
					atomic_set(&md->stepping, 0);
				} else {
					/* polling might occur without driving motor
					 * we need to release kthread that will trigger position change
					 * and set destination to properly complete detection
					 */
					atomic_set(&md->destination, ret);
					motor_stop(md, false);
				}
			}
				break;
		default:
			dev_err(md->dev, "Unsupported command %d\n", cmd);
				break;
		}
	}

	if (polling) {
		moto_drv8424_cmd_push(md, CMD_POLL, msecs_to_jiffies(POLL_NEXT_INT));
	}
}

static irqreturn_t motor_fault_irq(int irq, void *pdata)
{
    motor_device * md = (motor_device*) pdata;
    int value = gpio_get_value(md->mc.ptable[MOTOR_FAULT_INT]);

    if(value) {
        dev_info(md->dev, "dummy motor irq event\n");
    }
#if 0
    md->faulting = true;
    dev_err(md->dev, "Motor fault irq is happened\n");
	moto_drv8424_cmd_push(md, CMD_FAULT, 0);
#endif
    return IRQ_HANDLED;
}

//module node interface
static void moto_drv8424_set_torque(motor_device* md, unsigned torque)
{
    unsigned long flags;

    spin_lock_irqsave(&md->mlock, flags);
    if(md->torque == torque) {
        dev_info(md->dev, "Unchanged the torque, ignore\n");
        goto exit;
    }

    md->torque = torque;
exit:
    spin_unlock_irqrestore(&md->mlock, flags);
}

static int moto_drv8424_set_mode(motor_device* md, unsigned mode)
{
    unsigned long flags;
    int ret = 0;

    spin_lock_irqsave(&md->mlock, flags);
    if(md->mode == mode) {
        dev_info(md->dev, "Unchanged the mode, ignore\n");
        ret = -EINVAL;
        goto exit;
    }

    md->mode = mode;
    spin_unlock_irqrestore(&md->mlock, flags);

    return 0;

exit:
    spin_unlock_irqrestore(&md->mlock, flags);
    return ret;
}

static int moto_drv8424_set_dir(motor_device* md, unsigned dir)
{
    unsigned long flags;
    int ret = 0;

    spin_lock_irqsave(&md->mlock, flags);
    if(md->dir == dir) {
        dev_info(md->dev, "Unchanged the dir, ignore\n");
        ret = -EINVAL;
        goto exit;
    }

    md->dir = dir;
    spin_unlock_irqrestore(&md->mlock, flags);

    return 0;

exit:
    spin_unlock_irqrestore(&md->mlock, flags);
    return ret;
}

//Spec step max frequency 250KHz
#define STEP_MAX_FREQ 250000
static void moto_drv8424_set_step_freq(motor_device* md, unsigned freq)
{
    if(md->step_freq == freq) {
        dev_err(md->dev, "Unchanged the freq, ignore\n");
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
    if(md->step_ceiling == ceiling) {
        dev_info(md->dev, "Unchanged the ceiling, ignore\n");
        ret = -EINVAL;
        goto exit;
    }

    md->step_ceiling = ceiling;
    spin_unlock_irqrestore(&md->mlock, flags);

    return 0;

exit:
    spin_unlock_irqrestore(&md->mlock, flags);
    return ret;
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

    if(md->faulting) {
        dev_err(dev, "Device faulting\n");
        return -EBUSY;
    }

    if(kstrtouint(buf, 10, &enable)) {
        dev_err(dev, "Error value: %s\n", buf);
        goto exit;
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

    if(md->faulting) {
        dev_err(dev, "Device faulting\n");
        return -EBUSY;
    }

    if(kstrtouint(buf, 10, &value)) {
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

    if(md->faulting) {
        dev_err(dev, "Device faulting\n");
        return -EBUSY;
    }

    if(kstrtouint(buf, 10, &value)) {
        dev_err(dev, "Error value: %s\n", buf);
        return -EINVAL;
    }
    spin_lock_irqsave(&md->mlock, flags);
    moto_drv8424_set_step_freq(md, value);
    spin_unlock_irqrestore(&md->mlock, flags);
    dev_info(md->dev, "freq %uHz period %ldus\n", md->step_freq, md->step_period);

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

    if(md->faulting) {
        dev_err(dev, "Device faulting\n");
        return -EBUSY;
    }

    if(kstrtouint(buf, 10, &value)) {
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

    if(md->faulting) {
        dev_err(dev, "Device faulting\n");
        return -EBUSY;
    }

    if(kstrtouint(buf, 10, &value)) {
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

    if(md->faulting) {
        dev_err(dev, "Device faulting\n");
        return -EBUSY;
    }

    if(kstrtouint(buf, 10, &value)) {
        dev_err(dev, "Error value: %s\n", buf);
        goto exit;
    }

    moto_drv8424_set_ceiling(md, value);

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

    if(md->faulting) {
        dev_err(dev, "Device faulting\n");
        return -EBUSY;
    }

    if(kstrtouint(buf, 10, &value)) {
        dev_err(dev, "Error value: %s\n", buf);
        goto exit;
    }

    spin_lock_irqsave(&md->mlock, flags);
    if(value > MOTOR_DEFAULT_EXPIRE) {
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

    mutex_lock(&md->mx_lock);

    if(kstrtouint(buf, 10, &value)) {
        dev_err(dev, "Error value: %s\n", buf);
        goto exit;
    }
    disable_motor(md->dev);
    moto_drv8424_set_power(md, 0);
    moto_drv8424_set_regulator_power(md, false);
    msleep(10);
    moto_drv8424_set_regulator_power(md, true);
    msleep(10);
    moto_drv8424_set_power(md, 0);
exit:
    mutex_unlock(&md->mx_lock);
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
    if(kstrtouint(buffer, 10, &value) ||
    	(value < 0 || value >= SQ_MAX)) {
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

	for (idx = 0; idx < SQ_MAX; idx++) {
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

    if(kstrtouint(buf, 10, &value) ||
		(value < SQ_FULL || value >= SQ_MAX)) {
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

static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP, motor_enable_show, motor_enable_store);
static DEVICE_ATTR(dir, S_IRUGO|S_IWUSR|S_IWGRP, motor_dir_show, motor_dir_store);
static DEVICE_ATTR(step, S_IRUGO|S_IWUSR|S_IWGRP, motor_step_show, motor_step_store);
static DEVICE_ATTR(ceiling, S_IRUGO|S_IWUSR|S_IWGRP, motor_ceiling_show, motor_ceiling_store);
static DEVICE_ATTR(mode, S_IRUGO|S_IWUSR|S_IWGRP, motor_mode_show, motor_mode_store);
static DEVICE_ATTR(sequencer, S_IRUGO|S_IWUSR|S_IWGRP, motor_sequencer_show, motor_sequencer_store);
static DEVICE_ATTR(torque, S_IRUGO|S_IWUSR|S_IWGRP, motor_torque_show, motor_torque_store);
static DEVICE_ATTR(time_out, S_IRUGO|S_IWUSR|S_IWGRP, motor_time_out_show, motor_time_out_store);
static DEVICE_ATTR(reset, S_IRUGO|S_IWUSR|S_IWGRP, NULL, motor_reset_store);
static DEVICE_ATTR(regime, S_IRUGO|S_IWUSR|S_IWGRP, motor_regime_show, motor_regime_store);

#define ATTRS_STATIC 8
#define ATTRS_MAX 11

static int last_idx = ATTRS_STATIC;

static struct attribute *motor_attributes[ATTRS_MAX + 1] = {
    &dev_attr_dir.attr,
    &dev_attr_enable.attr,
    &dev_attr_step.attr,
    &dev_attr_ceiling.attr,
    &dev_attr_reset.attr,
    &dev_attr_time_out.attr,
    &dev_attr_regime.attr,
    &dev_attr_sequencer.attr,
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

/* HAL communicates desired position through this sysfs */
static ssize_t motor_position_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t len)
{
    motor_device* md = (motor_device*)dev_get_drvdata(dev);
    unsigned long flags;
    unsigned value = 0;

	if(kstrtouint(buf, 10, &value) ||
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
	int args[3] = {OUT_OF_RANGE};
#if 0
	if (atmic_read(&md->destination) == 0) {
		dev_err(dev, "Scan is not enabled\n");
		goto just_exit;
	}
#endif
	argnum = sscanf(buf, "%d %d %d", &args[0], &args[1], &args[2]);
	LOGD("sensor_data (%d): %d %d %d\n", argnum, args[0], args[1], args[2]);
	if (argnum > 0 && argnum >= ARGS_NUM_MIN) {
		mutex_lock(&md->mx_lock);
		/* store necessary sensor info */
		memcpy(md->sensor_data, args, sizeof(md->sensor_data));
		mutex_unlock(&md->mx_lock);
	}

//just_exit:
    return len;
}

/* sysfs polled from  HAL */
static DEVICE_ATTR(position, S_IRUGO|S_IWUSR|S_IWGRP, motor_position_show, motor_position_store);
static DEVICE_ATTR(status, S_IRUGO, motor_status_show, NULL);
static DEVICE_ATTR(sensor_data, S_IWUSR|S_IWGRP, NULL, motor_sensor_data_store);

static struct attribute *status_attributes[] = {
    &dev_attr_status.attr,
    &dev_attr_position.attr,
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
    uint32_t temp_value;
    int i, gpio, rc = 0;
	char gpio_name[32];
    const char *clock_name;

    rc = of_property_read_u32(np, "drv8424-gpios-cells", &temp_value);
    if (rc) {
        dev_err(pdev, "%d:Failed to get gpios cells\n", rc);
        goto exit;
    }
    mc->tab_cells = temp_value;

    if(mc->tab_cells > MAX_GPIOS) {
        dev_err(pdev, "Occupied too many gpios, max limited is %d\n", MAX_GPIOS);
        mc->tab_cells = MAX_GPIOS;
    }

	for (i = 0; i < mc->tab_cells; i++) {
		snprintf(gpio_name, sizeof(gpio_name)-1, "%s-gpio", gpios_labels[i]);
		gpio = of_get_named_gpio(np, gpio_name, 0);
		md->mc.ptable[i] = gpio_is_valid(gpio) ? gpio : -EINVAL;
		dev_info(pdev, "gpio %s = %d\n", gpios_labels[i], md->mc.ptable[i]);
    }

    if (!of_property_read_string(np, "clock-names", &clock_name))
        strlcpy(md->clock_name, clock_name, CLOCK_NAME_LEN);
    else
        strlcpy(md->clock_name, MOTOR_HW_CLK_NAME, CLOCK_NAME_LEN);
    dev_info(pdev, "hw clock name: %s\n", md->clock_name);

    md->hw_clock = of_property_read_bool(np, "enable-hw-clock");
    dev_info(pdev, "Enable hw clock %d\n", md->hw_clock);
	
    md->support_mode = of_property_read_bool(np, "support-mode");
    dev_info(pdev, "Enable hw clock %d\n", md->support_mode);
	if (md->support_mode)
		ATTR_ADD(mode);

	md->support_torque = of_property_read_bool(np, "support-torque");
    dev_info(pdev, "Enable hw clock %d\n", md->support_torque);
	if (md->support_torque)
		ATTR_ADD(torque);

    md->power_default_off = of_property_read_bool(np, "power-default-off");
    dev_info(pdev, "power is default off:  %d\n", md->power_default_off);

exit:
    return rc;
}

static int moto_drv8424_probe(struct platform_device *pdev)
{
    struct device* dev = &pdev->dev;
    motor_device* md;
    int i;
    int ret = 0;

    md = kzalloc(sizeof(motor_device), GFP_KERNEL);
    if(!md) {
        dev_err(dev, "probe: Out of memory\n");
        return -ENOMEM;
    }

    md->dev = dev;
    md->mc.plabels = gpios_labels;
    spin_lock_init(&md->mlock);
    mutex_init(&md->mx_lock);
    platform_set_drvdata(pdev, md);
    moto_drv8424_set_step_freq(md, DEFAULT_STEP_FREQ);

	/* populate sequencer with initial values */
	for (i = SQ_FULL; i < SQ_MAX; i++)
		memcpy(md->sequencer[i], initial_data[i], sizeof(initial_data[i]));

	/* assign default values for optional parameters */
	md->mode = FULL_STEP;
	md->torque = TORQUE_FULL;

    ret = moto_drv8424_init_from_dt(md);
    if (ret)
        memcpy((void*)md->mc.ptable, def_gpios_table, sizeof(def_gpios_table));
#if 0
    md->mc.vdd = devm_regulator_get(md->dev, "vdd");
    if (IS_ERR(md->mc.vdd)) {
        ret = PTR_ERR(md->mc.vdd);
        dev_err(md->dev, "Failed to get VDD ret=%d\n", ret);
        goto failed_mem;
    }
#endif
    if (!md->power_default_off) {
        ret = moto_drv8424_set_regulator_power(md, true);
        if(ret) {
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

    md->user_sync_complete = false;
    init_waitqueue_head(&md->sync_complete);
	md->status_update_ready = false;
    init_waitqueue_head(&md->status_wait);
	md->position_update_ready = false;
    init_waitqueue_head(&md->position_wait);

    md->motor_task = kthread_create(motor_kthread, md, "motor_task");
    if (IS_ERR(md->motor_task)) {
        ret = PTR_ERR(md->motor_task);
        dev_err(dev, "Failed create motor kthread\n");
        goto failed_kfifo;
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
    if(IS_ERR(md->sysfs_dev)) {
        dev_err(dev, "Failed to create device\n");
        goto failed_sys_dev;
    }

    if(sysfs_create_group(&md->dev->kobj, &status_attribute_group)) {
        dev_err(dev, "Failed to create status attribute");
        goto failed_device;
    }

    if(md->hw_clock) {
        if(init_motor_clk(md) < 0) {
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
        gpio_direction_output(md->mc.ptable[i], 0);
    }

    wake_up_process(md->motor_task);

    if(!set_pinctrl_state(md, INT_DEFAULT)) {
        md->fault_irq = gpio_to_irq(md->mc.ptable[MOTOR_FAULT_INT]);
        ret = devm_request_threaded_irq(&pdev->dev, md->fault_irq, motor_fault_irq,
                motor_fault_irq, IRQF_TRIGGER_FALLING, "motor_irq", md);
        if(ret < 0) {
            dev_err(dev, "Failed to request irq %d\n", ret);
            goto failed_gpio;
        }
    } else {
        /*Here motor can work,  but have not irq*/
        dev_info(dev, "Failed to set device irq\n");
    }

	RESET_SENSOR_DATA;
#if defined(DRV8424_INIT_DETECTION)
    md->time_out = MOTOR_DETECT_EXPIRE;
	atomic_set(&md->position, POS_UNKNOWN);
	atomic_set(&md->destination, POS_UNKNOWN);
	atomic_set(&md->status, STATUS_UNKNOWN);
	/* run position detection */
	moto_drv8424_set_sensing(md, true);
#else
    md->time_out = MOTOR_DEFAULT_EXPIRE;
	atomic_set(&md->position, POS_COMPACT);
	atomic_set(&md->status, STATUS_STOPPED_COMPACT);
#endif
    dev_info(dev, "Success init device; running position detection...\n");
    return 0;

failed_gpio:
    while(i >= MOTOR_POWER_EN) {
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

    disable_irq(md->fault_irq);
    moto_drv8424_set_power(md, 0);
    moto_drv8424_set_regulator_power(md, false);
    devm_regulator_put(md->mc.vdd);
    device_destroy(md->drv8424_class, MKDEV(0, 0));
    class_destroy(md->drv8424_class);
    sysfs_remove_group(&md->dev->kobj, &status_attribute_group);
    disable_motor(md->dev);
    kthread_stop(md->motor_task);
    destroy_workqueue(md->motor_wq);
	kfifo_free(&md->cmd_pipe);
    kfree(md);

    return 0;
}

void moto_drv8424_platform_shutdown(struct platform_device *pdev)
{
    motor_device* md = (motor_device*)platform_get_drvdata(pdev);
	int curpos = atomic_read(&md->position);

    if (curpos != POS_COMPACT) {
		md->power_en = 0;
		motor_set_motion_params(md, curpos, POS_COMPACT);
		atomic_set(&md->destination, POS_COMPACT);
		moto_drv8424_set_enable_with_power(md, true);
    }
    msleep(timeoutMs[md->regime]);
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
