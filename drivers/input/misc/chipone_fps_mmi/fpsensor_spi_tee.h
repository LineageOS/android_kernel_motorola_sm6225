#ifndef __FPSENSOR_SPI_TEE_H
#define __FPSENSOR_SPI_TEE_H

#include <linux/types.h>
#include <linux/cdev.h>
#include <linux/wait.h>
#include <linux/fb.h>
#include <linux/notifier.h>

#define FPSENSOR_DEV_NAME           "fpsensor"
#define FPSENSOR_CLASS_NAME         "fpsensor"
#define FPSENSOR_DEV_MAJOR          0
#define N_SPI_MINORS                32    /* ... up to 256 */
#define FPSENSOR_NR_DEVS            1

#define ERR_LOG     (0)
#define INFO_LOG    (1)
#define DEBUG_LOG   (2)
#define fpsensor_debug(level, fmt, args...) do { \
        if (fpsensor_debug_level >= level) {\
            printk("[fpCoreDriver][SN=%d] " fmt, g_cmd_sn, ##args); \
        } \
    } while (0)
#define FUNC_ENTRY()  fpsensor_debug(DEBUG_LOG, "%s, %d, entry\n", __func__, __LINE__)
#define FUNC_EXIT()   fpsensor_debug(DEBUG_LOG, "%s, %d, exit\n", __func__, __LINE__)

/**********************IO Magic**********************/
#define FPSENSOR_IOC_MAGIC    0xf0    //CHIP

/* define commands */
#define FPSENSOR_IOC_INIT                       _IOWR(FPSENSOR_IOC_MAGIC,0,uint32_t)
#define FPSENSOR_IOC_EXIT                       _IOWR(FPSENSOR_IOC_MAGIC,1,uint32_t)
#define FPSENSOR_IOC_RESET                      _IOWR(FPSENSOR_IOC_MAGIC,2,uint32_t)
#define FPSENSOR_IOC_ENABLE_IRQ                 _IOWR(FPSENSOR_IOC_MAGIC,3,uint32_t)
#define FPSENSOR_IOC_DISABLE_IRQ                _IOWR(FPSENSOR_IOC_MAGIC,4,uint32_t)
#define FPSENSOR_IOC_GET_INT_VAL                _IOWR(FPSENSOR_IOC_MAGIC,5,uint32_t)
#define FPSENSOR_IOC_DISABLE_SPI_CLK            _IOWR(FPSENSOR_IOC_MAGIC,6,uint32_t)
#define FPSENSOR_IOC_ENABLE_SPI_CLK             _IOWR(FPSENSOR_IOC_MAGIC,7,uint32_t)
#define FPSENSOR_IOC_ENABLE_POWER               _IOWR(FPSENSOR_IOC_MAGIC,8,uint32_t)
#define FPSENSOR_IOC_DISABLE_POWER              _IOWR(FPSENSOR_IOC_MAGIC,9,uint32_t)
/* fp sensor has change to sleep mode while screen off */
#define FPSENSOR_IOC_ENTER_SLEEP_MODE           _IOWR(FPSENSOR_IOC_MAGIC,11,uint32_t)
#define FPSENSOR_IOC_REMOVE                     _IOWR(FPSENSOR_IOC_MAGIC,12,uint32_t)
#define FPSENSOR_IOC_CANCEL_WAIT                _IOWR(FPSENSOR_IOC_MAGIC,13,uint32_t)
#define FPSENSOR_IOC_GET_FP_STATUS              _IOWR(FPSENSOR_IOC_MAGIC,19,uint32_t)
#define FPSENSOR_IOC_ENABLE_REPORT_BLANKON      _IOWR(FPSENSOR_IOC_MAGIC,21,uint32_t)
#define FPSENSOR_IOC_UPDATE_DRIVER_SN           _IOWR(FPSENSOR_IOC_MAGIC,22,uint32_t)
typedef struct {
    dev_t devno;
    struct class *class;
    struct cdev cdev;
    struct platform_device *spi;
    unsigned int users;
    u8 device_available;    /* changed during fingerprint chip sleep and wakeup phase */
    u8 irq_enabled;
    volatile unsigned int RcvIRQ;
    int irq;
    int irq_gpio;
    int reset_gpio;
    int power_gpio;
    wait_queue_head_t wq_irq_return;
    int cancel;
    struct notifier_block notifier;
    u8 fb_status;
    int enable_report_blankon;
    int free_flag;
    struct pinctrl *pinctrl;
    struct pinctrl_state *reset_high;
    struct pinctrl_state *reset_low;
    struct pinctrl_state *vcc_high;
    struct pinctrl_state *vcc_low;
} fpsensor_data_t;

#endif    /* __FPSENSOR_SPI_TEE_H */
