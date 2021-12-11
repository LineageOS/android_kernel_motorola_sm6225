#ifndef _PM6125_FLASH_GPIO_H_
#define _PM6125_FLASH_GPIO_H_

#include "../cam_sensor_utils/cam_sensor_cmn_header.h"

#define PM6125_FLASH_PRINT pr_info //printk

/* DTS state */
typedef enum {
	PM6125_FLASH_GPIO_STATE_ACTIVE,
	PM6125_FLASH_GPIO_STATE_SUSPEND,
	PM6125_FLASH_GPIO_STATE_MAX,	/* for array size */
} PM6125_FLASH_GPIO_STATE;

#ifdef CONFIG_CAMERA_FLASH_PWM
enum{
	CAMERA_SENSOR_FLASH_STATUS_OFF,
	CAMERA_SENSOR_FLASH_STATUS_LOW,
	CAMERA_SENSOR_FLASH_STATUS_HIGH
};
#endif

#define PM6125_PWM_PERIOD 50000
#define FLASH_FIRE_HIGH_MAXCURRENT 1200
#define FLASH_FIRE_LOW_MAXCURRENT 150

/*****************************************************************************
 * Function Prototype
 *****************************************************************************/
extern void pm6125_flash_gpio_select_state(PM6125_FLASH_GPIO_STATE s, enum camera_flash_opcode opcode, u64 flash_current);

#ifdef CONFIG_CAMERA_FLASH_PWM
extern int pm6125_flash_control_create_device(struct device* dev);
extern int pm6125_flash_control_remove_device(struct device* dev);
#endif
#endif /* _PM6125_FLASH_GPIO_H_*/
