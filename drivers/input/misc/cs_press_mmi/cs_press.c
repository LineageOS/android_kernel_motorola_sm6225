/*
 *-----------------------------------------------------------------------------
 * The confidential and proprietary information contained in this file may
 * only be used by a person authorised under and to the extent permitted
 * by a subsisting licensing agreement from  CHIPSEA.
 *
 *              (C) COPYRIGHT 2020 SHENZHEN CHIPSEA TECHNOLOG_ERRIES CO.,LTD.
 *                  ALL RIGHTS RESERVED
 *
 * This entire notice must be reproduced on all copies of this file
 * and copies of this file may only be made by a person if such person is
 * permitted to do so under the terms of a subsisting license agreement
 * from CHIPSEA.
 *
 *        Release Information : cs_press chip forcetouch fw linux driver source file
 *        version : v1.x
 *-----------------------------------------------------------------------------
 */

#include "cs_press.h"

//#define KERNEL_4_1_5

const char *cs_driver_ver = "1.1";

#define PROC_FOPS_NUM  9
#define PROC_NAME_LEN  50

#ifndef KERNEL_4_1_5
#define FOPS_ARRAY(_open, _write) \
{\
    .proc_open = _open,\
    .proc_read = seq_read,\
    .proc_release = single_release,\
    .proc_write = _write,\
}
#else
#define FOPS_ARRAY(_open, _write) \
{\
    .owner = THIS_MODULE,\
    .open = _open,\
    .read = seq_read,\
    .llseek = seq_lseek,\
    .release = single_release,\
    .write = _write,\
}
#endif

static struct mutex    i2c_rw_lock;
static DEFINE_MUTEX(i2c_rw_lock);

static struct cs_press_t g_cs_press;
static struct cs_vtp_scroll vtp_scroll;
static struct cs_vtp_zoom vtp_zoom;
struct class *cs_press_class;


#ifdef INT_SET_EN
static DECLARE_WAIT_QUEUE_HEAD(cs_press_waiter);
static int cs_press_int_flag;
static int cs_press_stop_read;
int cs_press_irq_gpio;
int cs_press_irq_num;

/*1 enable,0 disable,  need to confirm after register eint*/
static int cs_irq_flag = 1;
struct input_dev *cs_input_dev;
#endif
/******* fuction definition start **********/
#ifdef INT_SET_EN
static void cs_irq_enable(void);
static void cs_irq_disable(void);
#endif


void cs_press_struct_init(void)
{
    g_cs_press.update_type = HIGH_VER_FILE_UPDATE;/*1:force update, 0:to higher ver update*/
    g_cs_press.updating_flag = 0;
}

/**@brief     read n reg datas from reg_addr.
 * @param[in]  dev:      cs_press device struct.
 * @param[in]  reg_addr: register addr
 * @param[in]  len:      read data lenth
 * @param[out] buf:     read data buffer addr.
 * @return     i2c_transfer status
 *             OK:      return num of send data
 *             err:     return err code
 */
static int cs_i2c_read_bytes(struct i2c_client *client, unsigned char reg_addr, void *buf, unsigned int len)
{
    int ret = 0;
    struct i2c_msg msg[2];
    struct i2c_adapter *adapter = client->adapter;
    int i = 5;

    if(buf == NULL){
        LOG_ERR("input buf is NULL\n");
        return -1;
    }
    if(len == 0){
        LOG_ERR("input len is invalid,len = 0\n");
        return -1;
    }
    /* msg[0] send first addr for read */
    msg[0].addr = client->addr;            /* cs_press addr */
    msg[0].flags = 0;                      /* send flag */
    msg[0].buf = &reg_addr;                /* reg addr */
    msg[0].len = 1;                        /* reg lenth */

    /* msg[1]read data */
    msg[1].addr = client->addr;
    msg[1].flags = I2C_M_RD;               /* read flag */
    msg[1].buf = buf;
    msg[1].len = len;
    mutex_lock(&i2c_rw_lock);
    do{
        ret = i2c_transfer(adapter, msg, sizeof(msg) / sizeof(struct i2c_msg));
        if(ret <= 0){
            LOG_ERR("i2c read failed! err_code:%d\n",ret);
        }else{
            break;
        }
        i--;
    }while(i > 0);
    mutex_unlock(&i2c_rw_lock);
    return ret;
}

 /**@brief    send multi datas to reg_addr.
 * @param[in]  dev:      cs_press device struct.
 * @param[in]  reg_addr: register addr
 * @param[in]  len:      send data lenth
 * @param[out] buf:     send data buffer addr.
 * @return     i2c_transfer status
 *             OK:      return num of send data
 *             err:     return err code
 */
static int cs_i2c_write_bytes(struct i2c_client *client, unsigned char reg_addr, unsigned char *buf, unsigned char len)
{
    int ret = 0;
    unsigned char *t_buf = NULL;
    struct i2c_msg msg;
    struct i2c_adapter *adapter = client->adapter;
    int i= 5;

    if(buf == NULL){
        LOG_ERR("input buf is NULL\n");
        return -1;
    }
    if(len == 0){
        LOG_ERR("input len is invalid,len = 0\n");
        return -1;
    }
    t_buf = (unsigned char *)kmalloc(len + sizeof(reg_addr), GFP_KERNEL );
    if (!t_buf){
        LOG_ERR("kmalloc  failed\n");
        return -1;
    }
    t_buf[0] = reg_addr;            /* register first addr */
    memcpy(&t_buf[1],buf,len);      /* copy send data to b[256]*/

    msg.addr = client->addr;
    msg.flags = 0;                  /* write flag*/
    msg.buf = t_buf;
    msg.len = len + 1;
    mutex_lock(&i2c_rw_lock);
    do{
        ret = i2c_transfer(adapter, &msg, 1);
        if(ret <= 0){
            LOG_ERR("i2c write failed! err_code:%d\n", ret);
        }else if(ret > 0){
            break;
        }
        i--;
    }while(i > 0);
    mutex_unlock(&i2c_rw_lock);
    kfree(t_buf);
    return ret;
}

/**@brief      read n reg datas from reg_addr.
 * @param[in]  dev:         cs_press device struct.
 * @param[in]  reg_addr:    register addr,addr type is word type
 * @param[in]  len:         read data lenth
 * @param[out] buf:         read data buffer addr.
 * @return     i2c_transfer status
 *             OK:      return num of send data
 *             err:     return err code
 */
static int cs_i2c_read_bytes_by_u16_addr(struct i2c_client *client, unsigned short reg_addr, void *buf, int len)
{
    int ret = 0;
    unsigned char reg16[2];
    struct i2c_msg msg[2];
    struct i2c_adapter *adapter = client->adapter;

    if(buf == NULL){
        LOG_ERR("input buf is NULL\n");
        return -1;
    }
    if(len == 0){
        LOG_ERR("input len is invalid,len = 0\n");
        return -1;
    }

    reg16[0] = (reg_addr >> 8)&0xff;
    reg16[1] = reg_addr & 0xff;
    /* msg[0] send first addr for read */
    msg[0].addr = client->addr;            /* cs_press addr */
    msg[0].flags = 0;                      /* send flag */
    msg[0].buf = reg16;                    /* reg addr */
    msg[0].len = sizeof(reg16);            /* reg lenth:2*byte */

    /* msg[1]read data */
    msg[1].addr = client->addr;
    msg[1].flags = I2C_M_RD;               /* read flag*/
    msg[1].buf = buf;
    msg[1].len = len;

    ret = i2c_transfer(adapter, msg, 2);
    if(ret == 2) {
        ret = 0;
    } else {
        LOG_ERR("i2c read failed! err_code:%d\n",ret);
    }
    return ret;
}

 /**@brief    send multi datas to reg_addr.
 * @param[in] dev:      cs_press device struct.
 * @param[in] reg_addr: register addr
 * @param[in] len:      send data lenth
 * @param[out] buf:     send data buffer addr.
 * @return     i2c_transfer status
 *             OK:   return num of send data
 *             err:  return err code
 */
static int cs_i2c_write_bytes_by_u16_addr(struct i2c_client *client, unsigned short reg_addr, unsigned char *buf, unsigned char len)
{
    int ret = 0;
    unsigned char *t_buf = NULL;
    struct i2c_msg msg;
    struct i2c_adapter *adapter = client->adapter;

    if(buf == NULL){
        LOG_ERR("input buf is NULL\n");
        return -1;
    }
    if(len == 0){
        LOG_ERR("input len is invalid,len = 0\n");
        return -1;
    }
    t_buf = (unsigned char *)kmalloc(len + sizeof(reg_addr), GFP_KERNEL );
    if (!t_buf){
        LOG_ERR("kmalloc  failed\n");
        return -1;
    }

    t_buf[0] = (reg_addr >> 8)&0xff; /* register first addr */
    t_buf[1] = reg_addr & 0xff;
    memcpy(&t_buf[2],buf,len);       /* copy send data to b[256]*/

    msg.addr = client->addr;
    msg.flags = 0;                   /* write flag*/

    msg.buf = t_buf;
    msg.len = len + 2;
    ret = i2c_transfer(adapter, &msg, 1);
    if(ret < 0)
    {
        LOG_ERR("i2c write failed! err_code:%d\n",ret);
    }
    kfree(t_buf);
    return ret;
}

/**
  * @brief      iic write funciton
  * @param[in]  regAddress: reg data
  * @param[in]  dat:        point to data to write
  * @param[in]  length:     write data length
  * @retval     0:success, < 0: fail
  */
static int cs_press_iic_write(unsigned char regAddress, unsigned char *dat, unsigned int length)
{
    int ret = 0;

    if(dat == NULL){
        LOG_ERR("input buf is NULL\n");
        return -1;
    }
    if(length == 0){
        LOG_ERR("input len is invalid,len = 0\n");
        return -1;
    }
    ret = cs_i2c_write_bytes(g_cs_press.client, regAddress, dat, length);
    if(ret < 0){
        return -1;
    }
    return 0;
}

/**
  * @brief       iic read funciton
  * @param[in]   regAddress: reg data,
  * @param[out]  dat:        read data buffer,
  * @param[in]   length:     read data length
  * @retval      0:success, -1: fail
  */
static int cs_press_iic_read(unsigned char regAddress, unsigned char *dat, unsigned int length)
{
    int ret = 0;

    if(dat == NULL){
        LOG_ERR("input buf is NULL\n");
        return -1;
    }
    if(length == 0){
        LOG_ERR("input len is invalid,len = 0\n");
        return -1;
    }
    /* user program */
    ret = cs_i2c_read_bytes(g_cs_press.client, regAddress, dat, length);
    if(ret < 0){
        LOG_ERR("cs_i2c_read_bytes err\n");
        return -1;
    }
    return 0;
}

/**
  * @brief  iic write funciton
  * @param[in]  regAddress: reg data,
  * @param[in]  *dat:       point to data to write,
  * @param[in]  length:     write data length
  * @retval 0:success, -1: fail
  */
static int cs_press_iic_write_double_reg(unsigned short regAddress, unsigned char *dat, unsigned int length)
{
    int ret = 0;

    if(dat == NULL){
        LOG_ERR("input buf is NULL\n");
        return -1;
    }
    if(length == 0){
        LOG_ERR("input len is invalid,len = 0\n");
        return -1;
    }
    /* user program*/
    ret = cs_i2c_write_bytes_by_u16_addr(g_cs_press.client, regAddress, dat,length);
    return ret;
}

/**
  * @brief      iic read funciton
  * @param[in]  regAddress: reg data,
  * @param[out] *dat:       read data buffer,
  * @param[in]  length:     read data length
  * @retval 0:success, -1: fail
  */
static int cs_press_iic_read_double_reg(unsigned short regAddress, unsigned char *dat, unsigned int length)
{
    int ret = 0;

    if(dat == NULL){
        LOG_ERR("input buf is NULL\n");
        return -1;
    }
    if(length == 0){
        LOG_ERR("input len is invalid,len = 0\n");
        return -1;
    }
    /* user program*/
    ret = cs_i2c_read_bytes_by_u16_addr(g_cs_press.client, regAddress, dat, length);
    return ret;
}

/**
  * @brief      delay function
  * @param[in]  time_ms: delay time, unit:ms
  * @retval     None
  */
static void cs_press_delay_ms(unsigned int time_ms)
{
    msleep(time_ms);
}

#if !RSTPIN_RESET_ENABLE
/**
  * @brief  cs_press_power_up
  * @param  None
  * @retval None
  */
static void cs_press_power_up(void)
{
    // user program
    #if 0
    if(gpio_is_valid(g_cs_press.power_gpio)){
        gpio_set_value(g_cs_press.power_gpio, RST_GPIO_HIGH);
    }else{
        LOG_ERR("gpio rst is invalid\n");
    }
    #endif
}

/**
  * @brief  ic power down function
  * @param  None
  * @retval None
  */
static void cs_press_power_down(void)
{
    // user program
    #if 0
    if(gpio_is_valid(g_cs_press.power_gpio)){
        gpio_set_value(g_cs_press.power_gpio, RST_GPIO_LOW);
    }else{
        LOG_ERR("gpio rst is invalid\n");
    }
    #endif
}
#endif

#if RSTPIN_RESET_ENABLE
/**
  * @brief  ic rst pin set high
  * @param  None
  * @retval None
  */
static void cs_press_rstpin_high(void)
{
    /* user program*/
    #if 1
    if(gpio_is_valid(g_cs_press.rst_gpio)){
        gpio_set_value(g_cs_press.rst_gpio, RST_GPIO_HIGH);
    }else{
        LOG_ERR("gpio rst is invalid\n");
    }
    #endif
}

/**
  * @brief  ic rst pin set low
  * @param  None
  * @retval None
  */
static void cs_press_rstpin_low(void)
{
    /* user program*/
    #if 1
    if(gpio_is_valid(g_cs_press.rst_gpio)){
        gpio_set_value(g_cs_press.rst_gpio, RST_GPIO_LOW);
    }else{
        LOG_ERR("gpio rst is invalid\n");
    }
    #endif
}
#endif
/**
  * @brief      iic function test
  * @param[in]  test_data: test data
  * @retval     0:success, -1: fail
  */
int cs_press_iic_rw_test(unsigned char test_data)
{
    int ret = 0;
    unsigned char retry = RETRY_NUM;
    unsigned char read_data = 0;
    unsigned char write_data = test_data;

    do
    {
        cs_press_iic_write(AP_RW_TEST_REG, &write_data, 1);
        cs_press_iic_read(AP_RW_TEST_REG, &read_data, 1);
        ret = 0;
        retry--;
        if(read_data != write_data)
        {
            ret = -1;
            LOG_ERR("iic test failed,w:%d,rd:%d  %d\n",write_data,read_data,ret);
            cs_press_delay_ms(1);
        }else{
            LOG_INFO("iic test ok,w:%d,rd:%d\n",write_data,read_data);
            retry = 0;
        }

    }while(retry > 0);
    return ret;
}

/**
  * @brief  wakeup iic
  * @param  None
  * @retval 0:success, -1: fail
  */
static char cs_press_wakeup_iic(void)
{
    int ret = 0;

    ret = cs_press_iic_rw_test(0x67);
    return (char)ret;
}

#ifdef INT_SET_EN
/**
  * @brief    input system register
  * @param
  * @retval none
  */
void fml_input_dev_init(void){
    int ret = 0;

    cs_input_dev = input_allocate_device();
    if (cs_input_dev != NULL) {
        cs_input_dev->name = CS_PRESS_NAME;
        cs_input_dev->phys = CS_INPUT_PHYS;
        cs_input_dev->id.product = 0x1357;
        cs_input_dev->id.vendor = 0x2468;
        cs_input_dev->id.version = 10428;

        __set_bit(EV_KEY, cs_input_dev->evbit);
        __set_bit(KEY_HOME, cs_input_dev->keybit);
        __set_bit(KEY_VOLUMEUP, cs_input_dev->keybit);
        __set_bit(KEY_VOLUMEDOWN, cs_input_dev->keybit);
        __set_bit(KEY_POWER, cs_input_dev->keybit);

        __set_bit(EV_SYN, cs_input_dev->keybit);
        __set_bit(EV_ABS, cs_input_dev->keybit);
        __set_bit(BTN_TOUCH, cs_input_dev->keybit);
        __set_bit(INPUT_PROP_DIRECT, cs_input_dev->propbit);
        __set_bit(BTN_TOOL_FINGER, cs_input_dev->keybit);

        /* set input parameters */
        input_set_abs_params(cs_input_dev, ABS_MT_POSITION_X,
                     0, g_cs_press.panel_max_x, 0, 0);
        input_set_abs_params(cs_input_dev, ABS_MT_POSITION_Y,
                     0, g_cs_press.panel_max_y, 0, 0);
        input_set_abs_params(cs_input_dev, ABS_MT_TOUCH_MAJOR,
                     0, g_cs_press.panel_max_w, 0, 0);

#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 7, 0)
        input_mt_init_slots(cs_input_dev, CS_MAX_TOUCH,
                INPUT_MT_DIRECT);
#else
        input_mt_init_slots(cs_input_dev, CS_MAX_TOUCH);
#endif

        ret = input_register_device(cs_input_dev);
        if (ret != 0) {
            LOG_ERR("input register device error = %d\n", ret);
            input_free_device(cs_input_dev);
        }
    }
}

/**
  * @brief    input system unregister
  * @param
  * @retval none
  */
void fml_input_dev_exit(void){
    /* release input_dev */
    input_unregister_device(cs_input_dev);
    input_free_device(cs_input_dev);
}

/**
  * @brief    cs_irq_enable
  * @param
  * @retval
  */
static void cs_irq_enable(void)
{
    if (cs_irq_flag == 0) {
        cs_irq_flag++;
        enable_irq(cs_press_irq_num);
    } else {
        LOG_ERR("cs_press Eint already enabled!\n");
    }
    LOG_ERR("Enable irq_flag=%d\n", cs_irq_flag);
}

/**
  * @brief    cs_irq_disable
  * @param
  * @retval
*/
static void cs_irq_disable(void)
{
    if (cs_irq_flag == 1) {
        cs_irq_flag--;
        disable_irq_nosync(cs_press_irq_num);
    } else {
        LOG_ERR("cs_press Eint already disabled!\n");
    }
    LOG_ERR("Disable irq_flag=%d\n", cs_irq_flag);
}

int cs_irq_stat(void)
{
    return gpio_get_value(cs_press_irq_gpio);
}

static void* vtp_scroll_up_down(void *arg, int scroll_direction)
{
    struct cs_vtp_scroll *scroll = arg;
    int i;
    LOG_INFO("nav-gesture: start, p0=(%d,%d) L=%d s=%dpx t=%dms", scroll->x0, scroll->y0,
            scroll->range, scroll->step, scroll->interval);

    for(i = 0; i < scroll->range; i++) {
        input_mt_slot(cs_input_dev, 0);
        input_mt_report_slot_state(cs_input_dev, MT_TOOL_FINGER, true);

        if(i == 0) {
            input_report_abs(cs_input_dev, ABS_MT_POSITION_X, scroll->x0 - 1);
        } else {
            input_report_abs(cs_input_dev, ABS_MT_POSITION_X, scroll->x0);
	}

        if(scroll_direction == CS_NAV_MODE_SCROLL_UP) {
            input_report_abs(cs_input_dev, ABS_MT_POSITION_Y, scroll->y0 - (i * scroll->step));
        } else {
            input_report_abs(cs_input_dev, ABS_MT_POSITION_Y, scroll->y0 + (i * scroll->step));
        }
        input_report_abs(cs_input_dev, ABS_MT_TOUCH_MAJOR, 45);
        input_report_key(cs_input_dev, BTN_TOUCH, 1);
        input_sync(cs_input_dev);
        udelay(scroll->interval * 1000);
    }
    //input_mt_slot(cs_input_dev, 0);
    //input_mt_report_slot_state(cs_input_dev, MT_TOOL_FINGER, false);
    //input_report_key(cs_input_dev, BTN_TOUCH, 0);
    //input_sync(cs_input_dev);
    LOG_INFO("nav-gesture scroll %d done", scroll_direction);

    return NULL;
}

static void* vtp_zoom_in_out(void* arg, int zoom_direction)
{
    struct cs_vtp_zoom *zoom = arg;
    int i;
    LOG_INFO("nav-gesture: start, p1=(%d,%d) p2=(%d,%d) L=%d s=%dpx t=%dms", zoom->x0,
        zoom->p1_y0, zoom->x0, zoom->p2_y0, zoom->range, zoom->step, zoom->interval);

    for(i = 0; i < zoom->range; i++) {
        //finger 1
        input_mt_slot(cs_input_dev, 0);
        input_mt_report_slot_state(cs_input_dev, MT_TOOL_FINGER, true);

        if(i == 0) {
            input_report_abs(cs_input_dev, ABS_MT_POSITION_X, zoom->x0 - 1);
        } else {
            input_report_abs(cs_input_dev, ABS_MT_POSITION_X, zoom->x0);
	}

        if(zoom_direction == CS_NAV_MODE_ZOOM_IN) {
            input_report_abs(cs_input_dev, ABS_MT_POSITION_Y, zoom->p1_y0 - (i * zoom->step));
        } else {
            input_report_abs(cs_input_dev, ABS_MT_POSITION_Y, zoom->p1_y0 + (i * zoom->step));
        }
        input_report_abs(cs_input_dev, ABS_MT_TOUCH_MAJOR, 45);
        input_report_key(cs_input_dev, BTN_TOUCH, 1);
        input_sync(cs_input_dev);

        //finger 2
        input_mt_slot(cs_input_dev, 1);
        input_mt_report_slot_state(cs_input_dev, MT_TOOL_FINGER, true);

        if(i == 0) {
            input_report_abs(cs_input_dev, ABS_MT_POSITION_X, zoom->x0 - 1);
        } else {
            input_report_abs(cs_input_dev, ABS_MT_POSITION_X, zoom->x0);
	}

        if(zoom_direction == CS_NAV_MODE_ZOOM_IN) {
            input_report_abs(cs_input_dev, ABS_MT_POSITION_Y, zoom->p2_y0 + (i * zoom->step));
        } else {
            input_report_abs(cs_input_dev, ABS_MT_POSITION_Y, zoom->p2_y0 - (i * zoom->step));
        }
        input_report_abs(cs_input_dev, ABS_MT_TOUCH_MAJOR, 45);
        input_report_key(cs_input_dev, BTN_TOUCH, 1);
        input_sync(cs_input_dev);

        udelay(zoom->interval * 1000);
    }

    LOG_INFO("nav-gesture zoom done");

    return NULL;
}

static void cs_press_worker_func(struct work_struct *w)
{
    int ret, i;
    struct cs_press_coords raw_data = {0};
    int last_coord = -1;
    int diff_coord;

    switch(g_cs_press.nav_mode) {
    case CS_NAV_MODE_SCROLL:
        LOG_INFO("CS_NAV_MODE_SCROLL mode\n");
        vtp_scroll.x0 = (g_cs_press.panel_max_x/2);
        vtp_scroll.y0 = (g_cs_press.panel_max_y/2);
        //vtp_scroll.range = g_nav_vtp_range_point;
        vtp_scroll.step = 5;
        vtp_scroll.interval = 4;
        while(1) {
            if(!kfifo_len(&g_cs_press.data_queue) && cs_irq_stat()) {
                input_mt_slot(cs_input_dev, 0);
                input_mt_report_slot_state(cs_input_dev, MT_TOOL_FINGER, false);
                input_report_key(cs_input_dev, BTN_TOUCH, 0);
                input_sync(cs_input_dev);
                break;
            }

            if (kfifo_len(&g_cs_press.data_queue)) {
                ret = kfifo_out(&g_cs_press.data_queue, &raw_data, sizeof(struct cs_press_coords));
                if (!ret) {
                    LOG_ERR(" kfifo_out failed, it seems empty, ret=%d\n", ret);
                }
                LOG_INFO("kfifo_out coord: %d, pressure: %d\n", raw_data.coord, raw_data.pressure);
                //simulate scroll touch events

                if (last_coord == -1) {
                     last_coord = raw_data.coord;
                     continue;
                }
                diff_coord = raw_data.coord - last_coord;
                if(diff_coord < 0) {
                    //slide up
                    vtp_scroll.range = abs(diff_coord);
                    vtp_scroll_up_down(&vtp_scroll, CS_NAV_MODE_SCROLL_UP);
                    vtp_scroll.y0 -= vtp_scroll.range * vtp_scroll.step;
                } else if(diff_coord > 0) {
                    //slide down
                    vtp_scroll.range = abs(diff_coord);
                    vtp_scroll_up_down(&vtp_scroll, CS_NAV_MODE_SCROLL_DOWN);
                    vtp_scroll.y0 += vtp_scroll.range * vtp_scroll.step;
                }
                last_coord = raw_data.coord;
            } else {
                    //LOG_INFO("left raw_data is nothing\n");
            }
        }
    break;

    case CS_NAV_MODE_ZOOM:
        LOG_INFO("CS_NAV_MODE_ZOOM mode\n");
        vtp_zoom.x0 = (g_cs_press.panel_max_x/2);
        //vtp_zoom.p1_y0 = (g_cs_press.panel_max_y/2) - 100;
        //vtp_zoom.p2_y0 = (g_cs_press.panel_max_y/2) + 100;
        vtp_zoom.p1_y0 = (g_cs_press.panel_max_y/2) - 400;
        vtp_zoom.p2_y0 = (g_cs_press.panel_max_y/2) + 400;
        //vtp_zoom.range = g_nav_vtp_range_point;
        vtp_zoom.step = 4;
        vtp_zoom.interval = 5;

        while(1) {
            if(!kfifo_len(&g_cs_press.data_queue) && cs_irq_stat()) {
                for(i = 0; i < 2; i++) {
                    input_mt_slot(cs_input_dev, i);
                    input_mt_report_slot_state(cs_input_dev, MT_TOOL_FINGER, false);
                    input_report_key(cs_input_dev, BTN_TOUCH, 0);
                    input_sync(cs_input_dev);
                }
                break;
            }

            if (kfifo_len(&g_cs_press.data_queue)) {
                ret = kfifo_out(&g_cs_press.data_queue, &raw_data, sizeof(struct cs_press_coords));
                if (!ret) {
                    LOG_ERR(" kfifo_out failed, it seems empty, ret=%d\n", ret);
                }
                LOG_INFO("kfifo_out coord: %d, pressure: %d\n", raw_data.coord, raw_data.pressure);
                //simulate zoom touch events

                if (last_coord == -1) {
                     last_coord = raw_data.coord;
                     continue;
                }
                diff_coord = raw_data.coord - last_coord;
                if(diff_coord < 0) {
                    //slide up -> zoom in
                    vtp_zoom.range = abs(diff_coord);
                    vtp_zoom_in_out(&vtp_zoom, CS_NAV_MODE_ZOOM_IN);
                    vtp_zoom.p1_y0 -= vtp_zoom.range * vtp_zoom.step;
                    vtp_zoom.p2_y0 += vtp_zoom.range * vtp_zoom.step;
                } else if(diff_coord > 0) {
                    //slide down -> zoom out
                    vtp_zoom.range = abs(diff_coord);
                    vtp_zoom_in_out(&vtp_zoom, CS_NAV_MODE_ZOOM_OUT);
                    vtp_zoom.p1_y0 += vtp_zoom.range * vtp_zoom.step;
                    vtp_zoom.p2_y0 -= vtp_zoom.range * vtp_zoom.step;
                }
                last_coord = raw_data.coord;
            } else {
                    //LOG_INFO("left raw_data is nothing\n");
            }
        }
    break;

    default:
        LOG_INFO("not supported navigation mode\n");
    }

}

/**
  * @brief    cs_press_interrupt_handler
  * @param
  * @retval
*/
static irqreturn_t cs_press_interrupt_handler(int irq, void *dev_id)
{
    int irq_state = 0;

    printk("cs_press entry irq ok.\n");
    cs_press_int_flag = 1;

    irq_state = cs_irq_stat();

    //cs_irq_disable();
    if (irq_state == IRQ_TYPE_EDGE_FALLING) {
        cs_press_stop_read = 0; //clear flag
        schedule_delayed_work(&g_cs_press.work, 0); //start process data on buffer
        wake_up_interruptible(&cs_press_waiter); //start put data to buffer
    } else if (irq_state == IRQ_TYPE_EDGE_RISING) {
        cs_press_stop_read = 1; //need stop read data from IC
    }

    return IRQ_HANDLED;
}
/**
  * @brief    get_key_event
  * @param
  * @retval
*/
unsigned char get_key_event(void)
{
    unsigned char rbuf[1];
    unsigned char addr;
    int len = 0;

    addr = IIC_KEY_EVENT;
    len = 1;
    rbuf[0] = 0x0;
    if (cs_press_iic_read(addr, rbuf, len) <= 0) {
        LOG_ERR("reg=%d,buf[0]=%d,len=%d,err\n", addr, rbuf[0], len);
    }
    return rbuf[0];
}

/**
  * @brief    key event  handler
  * @param
  * @retval
*/
void fml_key_report(void)
{
#ifdef SIDE_KEY_MMI
    unsigned char rbuf[6];
    unsigned char addr;
    int len = 0;

    cs_press_wakeup_iic();

    addr = IIC_KEY_EVENT_MMI;
    len = 6;
    memset(rbuf, 0x0, len);
    if (cs_press_iic_read(addr, rbuf, len) != 0) {
        LOG_ERR("reg=%02x, buf[0]=%02x, buf[1]=%02x, force=%d len=%d,err\n",
            addr, rbuf[0], rbuf[1], (int16_t)((rbuf[5] << 8) + rbuf[4]), len);
        return;
    } else {
        LOG_ERR("reg=%02x, buf[0]=%02x, buf[1]=%02x, force=%d len=%d,success\n",
            addr, rbuf[0], rbuf[1], (int16_t)((rbuf[5] << 8) + rbuf[4]), len);
    }

    if(rbuf[0] == 1) {
        LOG_ERR("vol down key.\n");
        input_report_key(cs_input_dev, KEY_VOLUMEUP, 1);
        input_sync(cs_input_dev);
        input_report_key(cs_input_dev, KEY_VOLUMEUP, 0);
        input_sync(cs_input_dev);
    } else if(rbuf[1] == 1) {
        LOG_ERR("vol up key.\n");
        input_report_key(cs_input_dev, KEY_VOLUMEDOWN, 1);
        input_sync(cs_input_dev);
        input_report_key(cs_input_dev, KEY_VOLUMEDOWN, 0);
        input_sync(cs_input_dev);
    }
#endif

#ifdef SIDE_KEY
    unsigned char event = 0;
    event = get_key_event();
    if((event & 0x01) == 0x01){
        LOG_ERR("vol down key.\n");
        input_report_key(cs_input_dev, KEY_VOLUMEDOWN, 1);
        input_sync(cs_input_dev);
        input_report_key(cs_input_dev, KEY_VOLUMEDOWN, 0);
        input_sync(cs_input_dev);
    }else if((event & 0x02) == 0x02){
        LOG_ERR("vol up key.\n");
        input_report_key(cs_input_dev, KEY_VOLUMEUP, 1);
        input_sync(cs_input_dev);
        input_report_key(cs_input_dev, KEY_VOLUMEUP, 0);
        input_sync(cs_input_dev);
    }else if((event & 0x04) == 0x04){
        LOG_ERR("power key .\n");
        input_report_key(cs_input_dev, KEY_POWER, 1);
        input_sync(cs_input_dev);
        input_report_key(cs_input_dev, KEY_POWER, 0);
        input_sync(cs_input_dev);
    }
#endif

#ifdef HOME_KEY
    if(gpio_get_value(cs_press_irq_gpio) == 0){
        LOG_ERR("Home key down.\n");
        input_report_key(cs_input_dev, KEY_HOME, 1);
        input_sync(cs_input_dev);

    }else if(gpio_get_value(cs_press_irq_gpio) == 1){
        LOG_ERR("Home key up.\n");
        input_report_key(cs_input_dev, KEY_HOME, 0);
        input_sync(cs_input_dev);
    }
#endif

}

/**
  * @brief    cs_press_event_handler
  * @param
  * @retval
*/
static int cs_press_event_handler(void *unused)
{
    unsigned char read_data = 0;
    unsigned char read_temp[FW_ONE_BLOCK_LENGTH_R] = {0};
    unsigned char read_data_cmd = 0x24;
    unsigned char new_frame_cmd = 0x0;
    unsigned char exit_read_cmd = 0x0;
    struct cs_press_coords raw_data = {0};
    int queue_size;

    do {
        LOG_ERR("cs_press_event_handler do wait\n");
        wait_event_interruptible(cs_press_waiter,
            cs_press_int_flag != 0);
        LOG_ERR("cs_press_event_handler enter wait\n");
        cs_press_int_flag = 0;

        cs_press_wakeup_iic();

        cs_press_iic_write(DATA_MODE_REG, &read_data_cmd, 1);

        while(!cs_press_stop_read) {
            //read raw press data from IC
            cs_press_iic_write(DATA_READY_REG, &new_frame_cmd, 1);

            while(1) {
                cs_press_iic_read(DATA_READY_REG, &read_data, 1);
                if(read_data)
                    break;
                cs_press_delay_ms(1);
            }

            memset(read_temp, 0x0, sizeof(read_temp));
            cs_press_iic_read(DATA_READ_REG, read_temp, read_data);

            //put valid raw data to buffer
            raw_data.coord = (read_temp[1] << 8)|read_temp[0];
            raw_data.pressure = (read_temp[3] << 8)|read_temp[2];
            LOG_INFO("coord: %d, press: %d", raw_data.coord, raw_data.pressure);

            if(raw_data.coord >= 0 && raw_data.coord <= 100) {
                queue_size = (kfifo_len(&g_cs_press.data_queue) / sizeof(struct cs_press_coords));
                if (kfifo_avail(&g_cs_press.data_queue) && (queue_size < CS_DATA_MAX_QUEUE)) {
                    kfifo_in(&g_cs_press.data_queue, &raw_data, sizeof(struct cs_press_coords));
                    LOG_INFO("push raw data: %d\n", raw_data.coord);
                } else {
                    LOG_ERR("data_queue is full!!\n");
                    kfifo_reset(&g_cs_press.data_queue);
                    LOG_ERR("data_queue is reset!!\n");
                }
            }
        }

        cs_press_iic_write(DATA_MODE_REG, &exit_read_cmd, 1);
        //fml_key_report();
        //cs_irq_enable();
    } while (!kthread_should_stop());

    return 0;
}

/**
  * @brief
  * @param
  * @retval
*/
void eint_init(void)
{
    init_waitqueue_head(&cs_press_waiter);
    INIT_DELAYED_WORK(&g_cs_press.work, cs_press_worker_func);

    kthread_run(cs_press_event_handler, 0, CS_PRESS_NAME);
    cs_irq_disable();
    cs_irq_enable();
    LOG_ERR("init_irq ok");
    fml_input_dev_init();
}
void eint_exit(void)
{
    fml_input_dev_exit();
}
#endif
/**
  * @brief  clean debug mode reg, debug ready reg
  * @param  None
  * @retval 0:success, -1: fail
  */
static char cs_press_clean_debugmode(void)
{
    char ret = 0;
    unsigned char temp_data = 0;

    ret = cs_press_iic_write(DEBUG_MODE_REG, &temp_data, 1);
    if(ret < 0){
        LOG_ERR("DEBUG_MODE_REG write failed\n");
        return -1;
    }
    ret = cs_press_iic_write(DEBUG_READY_REG, &temp_data, 1);
    if(ret < 0){
        LOG_ERR("DEBUG_READY_REG write failed\n");
        return -1;
    }
    return 0;
}

/**
  * @brief  set debug mode reg
  * @param  mode_num: debug mode num data
  * @retval 0:success, -1: fail
  */
static char cs_press_set_debugmode(unsigned char mode_num)
{
    int ret = 0;

    ret = cs_press_iic_write(DEBUG_MODE_REG, &mode_num, 1);
    if(ret < 0){
        LOG_ERR("DEBUG_MODE_REG write failed\n");
        return -1;
    }
    return 0;
}

/**
  * @brief  set debug ready reg
  * @param  ready_num: debug ready num data
  * @retval 0:success, -1: fail
  */
static char cs_press_set_debugready(unsigned char ready_num)
{
    int ret = 0;

    ret = cs_press_iic_write(DEBUG_READY_REG, &ready_num, 1);
    if(ret < 0){
        LOG_ERR("DEBUG_READY_REG write failed\n");
        return -1;
    }
    return 0;
}

/**
  * @brief  get debug ready reg data
  * @param  None
  * @retval ready reg data
  */
static unsigned char cs_press_get_debugready(void)
{
    int ret = 0;
    unsigned char ready_num = 0;

    ret = cs_press_iic_read(DEBUG_READY_REG, &ready_num, 1);
    if(ret < 0)
    {
        ready_num = 0;
    }
    return ready_num;
}

/**
  * @brief      write debug data
  * @param[in]  *debugdata: point to data buffer,
  * @param[in]  length:     write data length
  * @retval     0:success, -1: fail
  */
static char cs_press_write_debugdata(unsigned char *debugdata, unsigned char length)
{
    int ret = 0;

    ret = cs_press_iic_write(DEBUG_DATA_REG, debugdata, length);
    if(ret < 0){
        LOG_ERR("DEBUG_DATA_REG write failed\n");
        return (char)-1;
    }
    return 0;
}

/**
  * @brief  read debug data
  * @param  *debugdata: point to data buffer, length: write data length
  * @retval 0:success, -1: fail
  */
static char cs_press_read_debugdata(unsigned char *debugdata, unsigned char length)
{
    int ret = 0;

    if(debugdata == NULL){
        LOG_ERR("ERR:buf is NULL\n");
        return (char)-1;
    }
    if(length == 0){
        LOG_ERR("ERR:read lenth = 0\n");
        return (char)-1;
    }
    ret = cs_press_iic_read(DEBUG_DATA_REG, debugdata, length);
    if(ret < 0){
        LOG_ERR("DEBUG_DATA_REG read failed\n");
        return (char)-1;
    }
    return 0;
}

#if 0
/**
  * @brief  soft_reset the device
  * @param  None
  * @retval 0:success, -1: fail
  */
static char cs_press_soft_reset_device(void)
{
    char ret = 0;
    unsigned char retry = RETRY_NUM;
    // M series required write value 0xcc
    unsigned char temp_data = 0xcc;

    // repeatly reset including first time i2c wake up
    do
    {
        if(ret!=0)
        {
            cs_press_delay_ms(1);
        }
        ret = cs_press_iic_write(AP_RESET_MCU_REG, &temp_data, 1);
    }while((ret != 0 ) && (retry--));

    return ret;
}
#endif

/**
  * @brief  reset ic
  * @param  None
  * @retval 0:success, -1: fail
  */
char cs_press_reset_ic(void)
{
    char ret = 0;

    rt_mutex_lock(&(g_cs_press.client->adapter->bus_lock));
    #if RSTPIN_RESET_ENABLE
        cs_press_rstpin_high();
        cs_press_delay_ms(10);
        cs_press_rstpin_low();
        cs_press_delay_ms(80);

    #else/* hw reset ic*/
        cs_press_power_down();
        cs_press_delay_ms(50);
        cs_press_power_up();
        cs_press_delay_ms(80);

    #endif
    rt_mutex_unlock(&(g_cs_press.client->adapter->bus_lock));
    return ret;
}

void set_device_updating_flag(unsigned char val)
{
    g_cs_press.updating_flag = val;
}

unsigned char  get_device_updating_flag(void)
{
    return g_cs_press.updating_flag;
}
/**
  * @brief      forced firmware update
  * @param[in]  *fw_array: point to fw hex array
  * @retval     0:success, -1: fail
  */
char cs_press_fw_force_update(const unsigned char *fw_array)
{
    unsigned int i = 0;
    unsigned int j = 0;
    int ret = 0;
    char result = 0;
    unsigned int fw_code_length = 0;
    unsigned int fw_block_num_w = 0;
    unsigned int fw_block_num_r = 0;
    const unsigned char *fw_code_start = NULL;
    unsigned int fw_count = 0;
    unsigned char fw_read_code[FW_ONE_BLOCK_LENGTH_R] = {0};
    unsigned short fw_default_version = 0;
    unsigned short fw_read_version = 0;
    unsigned char boot_fw_write_cmd[BOOT_CMD_LENGTH] = BOOT_FW_WRITE_CMD;
    unsigned char boot_fw_wflag_cmd[BOOT_CMD_LENGTH] = BOOT_FW_WFLAG_CMD;
    unsigned char page_end = 0;

#ifdef INT_SET_EN
    cs_irq_disable(); /*close enit irq.*/
#endif
    /* fw init */
    fw_code_length = ((((unsigned short)fw_array[FW_ADDR_CODE_LENGTH+0]<<8)&0xff00)|fw_array[FW_ADDR_CODE_LENGTH+1]);
    fw_code_start = &fw_array[FW_ADDR_CODE_START];
    fw_block_num_w = fw_code_length/FW_ONE_BLOCK_LENGTH_W;
    fw_block_num_r = fw_code_length/FW_ONE_BLOCK_LENGTH_R;
    fw_default_version = ((((unsigned short)fw_array[FW_ADDR_VERSION+0]<<8)&0xff00)|fw_array[FW_ADDR_VERSION+1]);

    if(fw_code_length % 128){
        LOG_INFO("fw is not 128*\n");
        goto FLAG_FW_FAIL;
    }
    page_end = fw_code_length % 256;
    cs_press_reset_ic();
    /* send fw write cmd */
    cs_press_iic_write_double_reg(BOOT_CMD_REG ,boot_fw_write_cmd, BOOT_CMD_LENGTH);
    cs_press_delay_ms(1500);    /* waiting flash erase*/
    /* send fw code */
    fw_count = 0;
    for(i = 0; i < fw_block_num_w; i++)
    {
        ret = cs_press_iic_write_double_reg(i*FW_ONE_BLOCK_LENGTH_W, (unsigned char*)fw_code_start+fw_count, FW_ONE_BLOCK_LENGTH_W);
        fw_count += FW_ONE_BLOCK_LENGTH_W;
        if(ret < 0)
        {
            LOG_ERR("ERR:iic write failed\n");
            result = (char)-1;
            goto FLAG_FW_FAIL;
        }
        cs_press_delay_ms(10);
    }
    /* read & check fw code */
    fw_count = 0;
    for(i = 0; i < fw_block_num_r; i++)
    {
        /* read code data */
        ret = cs_press_iic_read_double_reg(i*FW_ONE_BLOCK_LENGTH_R, fw_read_code, FW_ONE_BLOCK_LENGTH_R);
        if(ret < 0)
        {
            LOG_ERR("ERR:iic write failed\n");
            result = (char)-1;
            goto FLAG_FW_FAIL;
        }
        /* check code data */
        for(j = 0; j < FW_ONE_BLOCK_LENGTH_R; j++)
        {
            if(fw_read_code[j] != fw_code_start[fw_count+j])
            {
                LOG_ERR("ERR:check code data failed\n");
                result = (char)-1;
                goto FLAG_FW_FAIL;
            }
        }
        fw_count += FW_ONE_BLOCK_LENGTH_R;
        cs_press_delay_ms(5);
    }
    if(page_end > 0){
        /* read code data */
        ret = cs_press_iic_read_double_reg(fw_block_num_r*FW_ONE_BLOCK_LENGTH_R, fw_read_code, 128);
        if(ret < 0)
        {
            LOG_ERR("ERR:iic write failed\n");
            result = (char)-1;
            goto FLAG_FW_FAIL;
        }
        /* check code data */
        for(j = 0; j < 128; j++)
        {
            if(fw_read_code[j] != fw_code_start[fw_count+j])
            {
                LOG_ERR("ERR:check code data failed\n");
                result = (char)-1;
                goto FLAG_FW_FAIL;
            }
        }
        cs_press_delay_ms(5);
    }

    /* send fw flag cmd */
    cs_press_iic_write_double_reg(BOOT_CMD_REG ,boot_fw_wflag_cmd, BOOT_CMD_LENGTH);
    cs_press_delay_ms(50);
    /* reset */
    cs_press_reset_ic();
    /* check fw version */
    cs_press_delay_ms(300); /* skip boot */
    ret = cs_press_iic_read(AP_VERSION_REG, fw_read_code, CS_FW_VERSION_LENGTH);
    fw_read_version = 0;

    if(ret >= 0){
        fw_read_version = ((((unsigned short)fw_read_code[2]<<8)&0xff00)|fw_read_code[3]);
    }
    LOG_INFO("update version:0x%04x,read version:0x%04x\n", fw_default_version, fw_read_version);
    if(fw_read_version != fw_default_version){
        LOG_ERR("ERR:fw_read_version != fw_default_version\n");
        result = (char)-1;
        goto FLAG_FW_FAIL;
    }
FLAG_FW_FAIL:
#ifdef INT_SET_EN
    cs_irq_enable(); /*open enit irq.*/
#endif
    return result;
}


/**
  * @brief      firmware high version update
  * @param[in]  *fw_array: point to fw hex array
  * @retval     0:success, -1: fail, 1: no need update
  */
char cs_press_fw_high_version_update(const unsigned char *fw_array)
{
    int ret = 0;
    char result = 0;
    unsigned char read_temp[FW_ONE_BLOCK_LENGTH_R] = {0};
    unsigned short read_version = 0;
    unsigned short default_version = 0;
    char flag_update = 0;   /* 0: no need update fw, 1: need update fw */
    unsigned char retry = RETRY_NUM;

    cs_press_delay_ms(300); /* skip boot jump time */
    /* read ap version */
    ret = cs_press_iic_read(AP_VERSION_REG, read_temp, CS_FW_VERSION_LENGTH);

    if(ret >= 0)
    {
        /* get driver ap version */
        default_version = ((((unsigned short)fw_array[FW_ADDR_VERSION+0]<<8)&0xff00)|fw_array[FW_ADDR_VERSION+1]);
        /* get ic ap version */
        read_version = ((((unsigned short)read_temp[2]<<8)&0xff00)|read_temp[3]);
        /* compare */
        if(read_version != default_version)
        {
            flag_update = 1;
        }
    }
    else
    {
        flag_update = 1;

        LOG_ERR("read AP_VERSION_REG failed\n");
    }

    LOG_INFO("FW on file version:0x%04x, FW on IC version:0x%04x, need update flag:%d\n",
                default_version, read_version, flag_update);
    if(flag_update == 0)
    {
        LOG_INFO("no need update\n");
        return 1;   /* no need update */
    }
    /* update fw */
    retry = RETRY_NUM;
    do
    {
        result = cs_press_fw_force_update(fw_array);
        if(result < 0 || result >= 0x80){
            LOG_ERR("update failed,ret:%d, retry count:%d\n",result, retry);
        }
    }while((result!=0)&&(retry--));

    return result;
}

/**
  * @brief    fml_firmware_send_data
  * @param
  * @retval   none
  */
int fml_firmware_send_data(unsigned char *data, int len)
{
    int fw_body_len = 0;
    int fw_len = 0;
    int ret = 0;
    char result = 0;

    if(data == NULL){
        ret = -1;
        LOG_ERR("data buffer null:\n");
        goto exit_fw_buf;
    }
    fw_body_len = ((int)data[FW_ADDR_CODE_LENGTH]<<8)|((int)data[FW_ADDR_CODE_LENGTH+1]);
    fw_len = fw_body_len + 256;
    LOG_INFO("[fw file len:%d,fw body len: %d]\n", len, fw_body_len);
    if(fw_body_len <= 0 || fw_body_len > FW_UPDATA_MAX_LEN)
    {
        LOG_ERR("[err!fw body len err!len=%d]\n", len);
        ret = -1;
        goto exit_fw_buf;
    }
    if(fw_len != len)
    {
        LOG_ERR("[err!fw file len err! len=%d]\n", fw_len);
        ret = -1;
        goto exit_fw_buf;
    }
    set_device_updating_flag(1);
    if(g_cs_press.update_type == FORCE_FILE_UPDATE)
    {
        result = cs_press_fw_force_update(data);
        g_cs_press.update_type = HIGH_VER_FILE_UPDATE;
    }else{
        result = cs_press_fw_high_version_update(data);
    }
    set_device_updating_flag(0);
    if (result < 0 || result >= 0x80)
    {
        ret  = -1;
        LOG_ERR("Burning firmware fails\n");
    } else {
        LOG_ERR("Burning firmware success\n");
    }
exit_fw_buf:
    LOG_INFO("end\n");
    return ret;
}

/**
  * @brief    fml_firmware_config_cb
  * @param
  * @retval   none
  */
static void fml_firmware_config_cb(const struct firmware *cfg, void *ctx)
{
    int fw_error = 0;

    if(cfg)
    {
        fw_error = fml_firmware_send_data((unsigned char*)cfg->data, cfg->size);
        if(fw_error)
        {
            LOG_ERR("firmware send data err:%d\n", fw_error);
            goto err_release_cfg;
        }
    }
err_release_cfg:
    release_firmware(cfg);
    cs_press_delay_ms(10);
    LOG_INFO("end\n");
}


/**
  * @brief    firmware update by file
  * @param
  * @retval 0:success, -1: fail,1:no need update
  */
int fml_fw_update_by_file(void)
{
    int ret_error = 0;
    struct cs_press_t *fw_st = NULL;

    if (strlen(g_cs_press.cs_fw_name) == 0) {
        LOG_ERR("cs FW name is not specified\n");
        return -1;
    }

    fw_st = devm_kzalloc(&(g_cs_press.client->dev), sizeof(*fw_st), GFP_KERNEL);
    if(!fw_st)
    {
        LOG_ERR("devm_kzalloc failed\n");
        return -1;
    }
    fw_st->client = g_cs_press.client;
    ret_error = request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG, g_cs_press.cs_fw_name,
                &(g_cs_press.client->dev), GFP_KERNEL, fw_st, fml_firmware_config_cb);
    devm_kfree(&(g_cs_press.client->dev), fw_st);
    if(ret_error)
    {
        LOG_ERR("request_firmware_nowait failed:%d\n",ret_error);
        return -1;
    }
    LOG_INFO("end\n");
    return 0;
}

#if 0
/**
  * @brief  fml_fw_update_by_array
  * @param
  * @retval 0:success, -1: fail
  */
int fml_fw_update_by_array(void)
{
    char ret = 0;
    char retry = RETRY_NUM;

    cs_press_wakeup_iic();
    do{
        ret = cs_press_fw_high_version_update(cs_default_fw_array);
        if(ret < 0){
            LOG_ERR("update failed,ret:%d\n",ret);
        }
    }while((ret!=0)&&(retry--));

    return ret;
}
#endif

/**
  * @brief       read fw info
  * @param[out]  *fw_info: point to info struct
  * @retval      0:success, -1: fail
  */
char cs_press_read_fw_info(CS_FW_INFO_Def *fw_info)
{
    char ret = 0;
    unsigned char read_temp[FW_ONE_BLOCK_LENGTH_R] = {0};

    if(fw_info == NULL){
        LOG_ERR("ERR:point fw_info NULL\n");
    }
    cs_press_wakeup_iic();
    /* read fw info*/
#if 0
    ret |= cs_press_iic_read(AP_MANUFACTURER_ID_REG, read_temp, CS_MANUFACTURER_ID_LENGTH);  // Manufacturer id
    if(ret >= 0)
    {
        LOG_ERR("%x,%x\n",read_temp[0],read_temp[1]);
        fw_info->manufacturer_id = ((unsigned short)read_temp[1]<<8) | read_temp[0];
        LOG_ERR("%x\n",fw_info->manufacturer_id);
    }
    ret |= cs_press_iic_read(AP_MODULE_ID_REG, read_temp, CS_MODULE_ID_LENGTH);  // Module id
    if(ret >= 0)
    {
        fw_info->module_id = ((unsigned short)read_temp[1]<<8) | read_temp[0];
    }
#endif
    ret |= cs_press_iic_read(AP_VERSION_REG, read_temp, CS_FW_VERSION_LENGTH);  /* FW Version*/
    if(ret==0)
    {
        fw_info->fw_version = ((unsigned short)read_temp[2]<<8) | read_temp[3];
    }
    return ret;
}

/**
  * @brief  wake up device
  * @param  None
  * @retval 0:success, -1: fail
  */
char cs_press_set_device_wakeup(void)
{
    char ret = 0;
    unsigned char retry = RETRY_NUM;
    unsigned char temp_data = 1;

    do
    {
        if(ret!=0)
        {
            cs_press_delay_ms(1);
        }
        ret = cs_press_iic_write(AP_WAKEUP_REG, &temp_data, 1);
    }while((ret!=0)&&(retry--));
    return ret;
}

/**
  * @brief  put the device to sleep
  * @param  None
  * @retval 0:success, -1: fail
  */
char cs_press_set_device_sleep(void)
{
    char ret = 0;
    unsigned char retry = 5;
    char i;
    /*0=FTM 0x01=write flag 0xFF=check sum*/
    unsigned char sleep_cmd[3] = {0x00, 0x01, 0xFF};
    unsigned char read_data[3] = {0x00, 0xFF, 0xFF};

    for(i = 0; i < retry; i++)
    {
        ret = cs_press_iic_write(AP_RW_WORK_MODE_REG, sleep_cmd, 3);
        if(ret == 0)
        {
            ret = -1;
            cs_press_delay_ms(1);
            if(cs_press_iic_read(AP_RW_WORK_MODE_REG, read_data, 3) == 0)
            {
                if((read_data[0]==0)&&((read_data[1]+read_data[2]) == 0))
                {
                    ret = 0;
                    break;
                }
            }
        }
        cs_press_delay_ms(1);
    }
    return ret;
}

/**
  * @brief  init read rawdata
  * @param  None
  * @retval 0:success, -1: fail
  */
char cs_press_read_rawdata_init(void)
{
    char ret = 0;

    cs_press_wakeup_iic();

    cs_press_clean_debugmode();

    ret = cs_press_set_debugmode(AP_R_RAWDATA_DEBUG_MODE);

    return ret;
}

/**
  * @brief      read rawdata
  * @param[in] *rawdata: point to sensor data strcut
  * @retval    0:none, -1: fail, >0:vaild data num
  */
char cs_press_read_rawdata(CS_RAWDATA_Def *rawdata)
{
    char ret = 0;
    unsigned char i = 0;
    unsigned char data_temp[AFE_MAX_CH*2] = {0};
    unsigned char byte_num;

    ret = (char)-1;

    byte_num = cs_press_get_debugready();
    if((byte_num <= (AFE_MAX_CH*2))&&(byte_num > 0))
    {
        if((byte_num%2) == 0)
        {
            ret = cs_press_read_debugdata(data_temp, byte_num);
            if(ret == 0)
            {
                for(i=0;i<(byte_num/2);i++)
                {
                    rawdata->rawdata[i] = ((((unsigned short)data_temp[i*2+1]<<8)&0xff00)|data_temp[i*2]);
                }
                ret = byte_num/2;
            }
        }
        else{
            ret = (char)-1;
        }
    }else{
        LOG_ERR("byte_num invalid:%d\n",byte_num);
    }
    cs_press_set_debugready(0);
    return ret;
}

/**
  * @brief  init read processed data
  * @param  None
  * @retval 0:success, -1: fail
  */
char cs_press_read_processed_data_init(void)
{
    char ret = 0;

    cs_press_wakeup_iic();

    cs_press_clean_debugmode();

    ret = cs_press_set_debugmode(AP_R_PROCESSED_DEBUG_MODE);

    return ret;
}

/**
  * @brief      read processeddata
  * @param[in]  *proce_data: point to processed data strcut
  * @retval     0:success, -1: fail
  */
char cs_press_read_processed_data(CS_PROCESSED_DATA_Def *proce_data)
{
    char ret = 0;
    unsigned char i = 0;
    unsigned char data_temp[(AFE_USE_CH*4*2)+2]; /* 1 ch have 4 types of data*/
    short data_temp_s16[(AFE_USE_CH*4)+1];
    unsigned char byte_num;
    short checksum;

    ret = (char)-1;

    byte_num = cs_press_get_debugready();

    if((byte_num == ((AFE_USE_CH*4*2)+2))&&(byte_num > 0))
    {
        ret = cs_press_read_debugdata(data_temp, byte_num);
        if(ret == 0)
        {
            for(i=0;i<(AFE_USE_CH*4)+1;i++)
            {
                data_temp_s16[i] = (short)((((unsigned short)data_temp[i*2+1]<<8)&0xff00)|data_temp[i*2]);
            }
            checksum = 0;
            for(i=0;i<(AFE_USE_CH*4);i++)
            {
                checksum += data_temp_s16[i];
            }
            if(checksum == data_temp_s16[(AFE_USE_CH*4)])    /* check right*/
            {
                for(i=0;i<AFE_USE_CH;i++)
                {
                    proce_data->arith_rawdata[i] = data_temp_s16[i];
                    proce_data->baseline[i] = data_temp_s16[AFE_USE_CH+i];
                    proce_data->diffdata[i] = data_temp_s16[AFE_USE_CH*2+i];
                    proce_data->energy_data[i] = data_temp_s16[AFE_USE_CH*3+i];
                }
                ret = 0;
            }
        }else{
            LOG_ERR("byte_num invalid:%d\n",byte_num);
        }
    }
    cs_press_set_debugready(0);
    return ret;
}

/**
  * @brief  init read processed data
  * @param  None
  * @retval 0:success, -1: fail
  */
char cs_press_read_ndt_debug_init(void)
{
    char ret = 0;
    unsigned char retry = RETRY_NUM;
    cs_press_wakeup_iic();
    cs_press_clean_debugmode();
    do
    {
        ret = cs_press_set_debugmode(AP_R_NDT_DEBUG_MODE);
    }while((ret != 0 ) && (retry--));

    LOG_ERR("debug init %d\n",ret);
    return ret;
}


/**
  * @brief        read calibration factor
  * @param[in]    read channel num
  * @param[out]   *cal_factor: point to calibration factor
  * @retval       0:success, -1: fail
  */
char cs_press_read_calibration_factor(unsigned char ch, unsigned short *cal_factor)
{
    char ret = (char)-1;
    unsigned char data_temp[4]={0,0,0,0};
    unsigned char num;

    if(cal_factor == NULL){
        LOG_ERR("point cal_factor if NULL\n");
        return -1;
    }
    cs_press_wakeup_iic();
    cs_press_clean_debugmode();
    cs_press_set_debugmode(AP_R_CAL_FACTOR_DEBUG_MODE);
    data_temp[0] = ch;
    cs_press_write_debugdata(data_temp, 2);
    cs_press_set_debugready(2);
    cs_press_delay_ms(DEBUG_MODE_DELAY_TIME);
    num = cs_press_get_debugready();
    if(num == 4)
    {
        ret = cs_press_read_debugdata(data_temp, 4);
        if(ret == 0)
        {
            *cal_factor = ((((unsigned short)data_temp[1]<<8)&0xff00)|data_temp[0]);
        }
    }
    return ret;
}

/**
  * @brief      write calibration factor
  * @param[in]  channel num
  * @param[in]  cal_factor: calibration factor data
  * @retval     0:success, -1: fail
  */
char cs_press_write_calibration_factor(unsigned char ch, unsigned short cal_factor)
{
    char ret = 0;
    unsigned char data_temp[7] = {0};
    unsigned short read_cal_factor = 0;

    cs_press_wakeup_iic();
    cs_press_clean_debugmode();
    cs_press_set_debugmode(AP_W_CAL_FACTOR_DEBUG_MODE);

    data_temp[0] = ch;
    data_temp[1] = 0;
    data_temp[2] = cal_factor;
    data_temp[3] = cal_factor>>8;
    data_temp[4] = 0;
    data_temp[5] = 0;

    cs_press_write_debugdata(data_temp, 6);
    cs_press_set_debugready(6);
    cs_press_delay_ms(DEBUG_MODE_DELAY_TIME*3);
    cs_press_read_calibration_factor(ch, &read_cal_factor);
    if(read_cal_factor != cal_factor)
    {
        LOG_ERR("set calibration factor failed\n");
        ret = (char)-1;
    }
    return ret;
}
/**
  * @brief      enable calibration function
  * @param[in]  channel num
  * @retval 0:success, -1: fail
  */
char cs_press_calibration_enable(unsigned char ch_num)
{
    char ret = 0;
    unsigned char temp_data = 1 + ch_num;

    cs_press_wakeup_iic();
    cs_press_delay_ms(1);
    ret = cs_press_iic_write(AP_CALIBRATION_REG, &temp_data, 1);
    return ret;
}

/**
  * @brief  disable calibration function
  * @param  None
  * @retval 0:success, -1: fail
  */
char cs_press_calibration_disable(void)
{
    char ret = 0;
    unsigned char temp_data = 0;

    cs_press_wakeup_iic();
    cs_press_delay_ms(1);
    ret = cs_press_iic_write(AP_CALIBRATION_REG, &temp_data, 1);
    return ret;

}
/**
  * @brief       read press level
  * @param[out]  *press_level: point to press level buffer
  * @retval      0:success, -1: fail
  */
char cs_press_read_press_level(unsigned short *press_level)
{
    char ret = (char)-1;
    unsigned char data_temp[4]={0,0,0,0};
    unsigned char num;

    if(press_level == NULL){
        LOG_ERR("point press_level is NULL\n");
        return -1;
    }
    cs_press_wakeup_iic();
    cs_press_clean_debugmode();
    cs_press_set_debugmode(AP_R_PRESS_LEVEL_DEBUG_MODE);
    cs_press_write_debugdata(data_temp, 2);
    cs_press_set_debugready(2);
    cs_press_delay_ms(DEBUG_MODE_DELAY_TIME);
    num = cs_press_get_debugready();
    if(num == 4)
    {
        ret = cs_press_read_debugdata(data_temp, num);
        if(ret == 0)
        {
            *press_level = ((((unsigned short)data_temp[1]<<8)&0xff00)|data_temp[0]);
        }
    }
    return ret;
}

/**
  * @brief      write press level
  * @param[in]  press_level: press level data
  * @retval     :success, -1: fail
  */
char cs_press_write_press_level(unsigned short press_level)
{
    char ret;
    unsigned char data_temp[6]={0};
    unsigned short read_press_level = 0;

    cs_press_wakeup_iic();

    cs_press_clean_debugmode();
    cs_press_set_debugmode(AP_W_PRESS_LEVEL_DEBUG_MODE);

    data_temp[0] = 0;
    data_temp[1] = 0;
    data_temp[2] = press_level;
    data_temp[3] = press_level>>8;
    data_temp[4] = 0;
    data_temp[5] = 0;

    cs_press_write_debugdata(data_temp, 6);
    cs_press_set_debugready(6);
    cs_press_delay_ms(DEBUG_MODE_DELAY_TIME*3);
    cs_press_read_press_level(&read_press_level);

    ret = 0;
    if(read_press_level != press_level)
    {
        ret = (char)-1;
    }

    return ret;
}
/**
  * @brief      check calibration result
  * @param[out] check result data buf
  * @retval     0:running, -1: error, 1: success, 2: fail, 3: overtime
  */
char cs_press_calibration_check(CS_CALIBRATION_RESULT_Def *calibration_result)
{
    char ret;
    unsigned char num;
    unsigned char data_temp[10];

    if(calibration_result == NULL){
        LOG_ERR("point calibration_result is NULL\n");
        return -1;
    }
    ret = cs_press_set_debugmode(AP_CALIBRATION_DEBUG_MODE);
    num = cs_press_get_debugready();
    if(num == 10)
    {
        ret = cs_press_read_debugdata(data_temp, 10);

        if(ret == 0)
        {
            calibration_result->calibration_channel = data_temp[0];
            calibration_result->calibration_progress = data_temp[1];
            calibration_result->calibration_factor = ((((unsigned short)data_temp[3]<<8)&0xff00)|data_temp[2]);
            calibration_result->press_adc_1st = ((((short)data_temp[5]<<8)&0xff00)|data_temp[4]);
            calibration_result->press_adc_2nd = ((((short)data_temp[7]<<8)&0xff00)|data_temp[6]);
            calibration_result->press_adc_3rd = ((((short)data_temp[9]<<8)&0xff00)|data_temp[8]);

            if(calibration_result->calibration_progress >= CALIBRATION_SUCCESS_FALG)
            {
                ret = 1;
            }
            if(calibration_result->calibration_progress == CALIBRATION_FAIL_FALG)
            {
                ret = 2;
            }
            if(calibration_result->calibration_progress == CALIBRATION_OVERTIME_FALG)
            {
                ret = 3;
            }
        }
    }

    return ret;
}

/**
  * @brief      init read noise data
  * @param[in]  period_num: period num of calculation the noise
  * @retval     0:success, -1: fail
  */
char cs_press_read_noise_init(unsigned short period_num)
{
    char ret = 0;
    unsigned char data_temp[2];

    cs_press_wakeup_iic();

    ret |= cs_press_clean_debugmode();

    ret |= cs_press_set_debugmode(AP_R_NOISE_DEBUG_MODE);

    data_temp[0] = period_num;
    data_temp[1] = period_num>>8;
    ret |= cs_press_write_debugdata(data_temp, 2);

    ret |= cs_press_set_debugready(2);

    return ret;
}

/**
  * @brief      read noise data
  * @param[out] noise_data:read data buf
  * @retval     0:success, -1: fail
  */
char cs_press_read_noise(CS_NOISE_DATA_Def *noise_data)
{
    char ret = 0;
    unsigned char i = 0;
    unsigned char num = 0;
    unsigned char data_temp[AFE_USE_CH*6];
    unsigned int dat_temp;

    if(noise_data == NULL){
        LOG_ERR("point noise_data is NULL\n");
        return -1;
    }
    ret = (char)-1;

    cs_press_wakeup_iic();

    num = cs_press_get_debugready();

    if(num == AFE_USE_CH*6)
    {
        cs_press_read_debugdata(data_temp, num);

        dat_temp = 0;
        for(i=0;i<AFE_USE_CH;i++)
        {
            noise_data->noise_peak[i] = ((((short)data_temp[1+i*6]<<8)&0xff00)|data_temp[0+i*6]);

            dat_temp = data_temp[5+i*6];
            dat_temp <<= 8;
            dat_temp |= data_temp[4+i*6];
            dat_temp <<= 8;
            dat_temp |= data_temp[3+i*6];
            dat_temp <<= 8;
            dat_temp |= data_temp[2+i*6];

            noise_data->noise_std[i] = dat_temp;
        }
        ret = 0;
    }
    return ret;
}
/**
  * @brief  init read offset data
  * @param  None
  * @retval 0:success, -1: fail
  */
static char cs_press_read_offset_init(void)
{
    char ret = 0;

    cs_press_wakeup_iic();
    ret |= cs_press_clean_debugmode();
    ret |= cs_press_set_debugmode(AP_R_OFFSET_DEBUG_MODE);
    ret |= cs_press_set_debugready(0);
    return ret;
}

/**
  * @brief          read offset data
  * @param[out]      offset_data
  * @retval         0:success, -1: fail
  */
char cs_press_read_offset(CS_OFFSET_DATA_Def *offset_data)
{
    char ret;
    unsigned char i;
    unsigned char num;
    unsigned char data_temp[AFE_USE_CH*2]={0};

    if(offset_data == NULL){
        LOG_ERR("point offset_data is NULL\n");
        return -1;
    }
    ret = (char)-1;
    cs_press_read_offset_init();

    cs_press_delay_ms(50);                    /* waitting for data ready*/

    num = cs_press_get_debugready();

    if(num == (AFE_USE_CH*2))
    {
        cs_press_read_debugdata(data_temp, num);
        for(i = 0; i < AFE_USE_CH; i++)
        {
            offset_data->offset[i] = ((((short)data_temp[i*2+1]<<8)&0xff00)|(short)data_temp[i*2]);
        }
        ret = 0;
    }
    return ret;
}
/**
  * @brief  init read sensor status
  * @param  None
  * @retval 0:success, -1: fail
  */
static char cs_press_read_sensor_status_init(void)
{
    char ret = 0;

    cs_press_wakeup_iic();
    ret |= cs_press_clean_debugmode();
    ret |= cs_press_set_debugready(0);
    ret |= cs_press_set_debugmode(AP_R_SENSOR_STATUS_DEBUG_MODE);

    return ret;
}

/**
  * @brief      read noise data
  * @param[out] sensor_status
  * @retval     0:success, -1: fail
  */
char cs_press_read_sensor_status(CS_SENSOR_STATUS_Def *sensor_status)
{
    char ret;
    unsigned char i;
    unsigned char num;
    unsigned char data_temp[AFE_USE_CH]={0};

    if(sensor_status == NULL){
        LOG_ERR("point sensor_status is NULL\n");
        return -1;
    }
    ret = (char)-1;

    cs_press_read_sensor_status_init();

    for(i=0;i<AFE_USE_CH;i++)
    {
        cs_press_delay_ms(50);                  /* waitting for data ready */
    }

    num = cs_press_get_debugready();

    if(num == (AFE_USE_CH))
    {
        cs_press_read_debugdata(data_temp, num);

        for(i=0;i<AFE_USE_CH;i++)
        {
            sensor_status->status[i] = (short)data_temp[i];
        }
        ret = 0;
    }

    return ret;
}

char cs_write_calibration_command(unsigned char *data)
{
    char ret = 0;

    if(data == NULL){
        LOG_ERR("point data is NULL\n");
        return -1;
    }
    ret |= cs_press_wakeup_iic();
    ret |= cs_press_clean_debugmode();
    ret |= cs_press_set_debugmode(0x14);
    ret |= cs_press_read_debugdata(data, 2);
    return ret;
}


char cs_read_boot_version(unsigned char *version_data)
{
    char ret = 0 ;

    if(version_data == NULL){
        LOG_ERR("point version_data is NULL\n");
        return -1;
    }
    cs_press_reset_ic();
    ret = cs_press_iic_read_double_reg(0xF100, version_data, 4);
    return ret;
}

char cs_set_press_threshold(int left_val, int right_val)
{
    int i;
    char ret = 0 ;
    unsigned char retry = RETRY_NUM;
    unsigned char temp_data[CS_FORCE_TRIG_LENGTH];

    temp_data[0] = left_val&0xff;
    temp_data[1] = (left_val>>8)&0xff;
    temp_data[2] = right_val&0xff;
    temp_data[3] = (right_val>>8)&0xff;
    temp_data[4] = 0x03;/*set left & right*/
    temp_data[5] = 0;
    for(i = 0; i < 5; i++)
    {
        temp_data[5] += temp_data[i];
    }
    temp_data[5] = (0xFF - temp_data[5]) + 1;
    do
    {
        if(ret!=0)
        {
            cs_press_delay_ms(1);
        }
        ret = cs_press_iic_write(AP_RW_BUTTON_FORCE_THR_REG, temp_data, CS_FORCE_TRIG_LENGTH);
    }while((ret != 0 ) && (retry--));
    if(ret < 0)
    {
        LOG_ERR("err write press trig\n");
    }else{
        LOG_DEBUG("write press trig:ok %d %d\n",left_val, right_val);
    }
    return ret;
}

#if 0
/**
  * @brief  init
  * @param  None
  * @retval 0:success, -1: fail
  */
char cs_press_init(void)
{
    char ret = 0;
    char i;
    unsigned char boot_ver_buf[4];
    LOG_DEBUG("cs driver ver %s\n", cs_driver_ver);
    for(i = 0; i < 3; i++)
    {
        if(cs_read_boot_version(boot_ver_buf) >= 0)
        {
            if(boot_ver_buf[0] == 0x71)
            {
                LOG_INFO("boot version %02x %02x %02x %02x\n", boot_ver_buf[0], boot_ver_buf[1],
                                                            boot_ver_buf[2], boot_ver_buf[3]);
                break;
            }
        }else{
            LOG_ERR("read boot version err %d\n",i);
        }
    }
    if(i >= 3)
    {
        LOG_ERR("chipid err return\n");
        return ret;
    }
    /* reset ic */
    /* check whether need to update ap fw */
    ret = fml_fw_update_by_file();
    /*ret = fml_fw_update_by_array();*/

    if(ret >= 0)
    {
        return 0;
    }
    return ret;
}
#endif
#if 0
static ssize_t fml_fw_update_by_array_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
    ssize_t ret = 0;
    int result = 0;

    result = fml_fw_update_by_array();
    if (result >= 0)
        ret = sprintf(buf, "%d,pass!\n", result);
    else
        ret = sprintf(buf, "%d,failed!\n", result);

    LOG_ERR("%s\n", buf);
    return ret+1;
}
#endif

static int cs_proc_rw_reg_show(struct seq_file *m,void *v)
{
    int i = 0;

    if ((g_cs_press.read_reg_len != 0)&&(g_cs_press.read_reg_len < 255)) {
        for (i = 0; i < g_cs_press.read_reg_len; i++)
        {
            seq_printf(m, "%02x ", g_cs_press.read_reg_data[i]);
        }
        seq_printf(m, "\n");
    }else{
        seq_printf(m,"rw reg failed\n");
    }
    return 0;
}

static int cs_proc_rw_reg_open(struct inode *inode, struct file *filp)
{
    LOG_INFO("reg open start\n");
    return single_open(filp,cs_proc_rw_reg_show,NULL);
}

static ssize_t cs_proc_rw_reg_write(struct file *file, const char __user *buf,
        size_t count, loff_t *offset)
{
    char *kbuf = NULL;
    const char *startpos = NULL;
    unsigned int tempdata = 0;
    const char *lastc = NULL;
    char *firstc = NULL;
    int idx = 0;
    int ret = 0;
    int set_mode = 0;
    unsigned char change_val[32] = {0};

    LOG_INFO("reg write start\n");
    if(count <= 0){
        LOG_ERR("PARAM invalid\n");
        goto exit;
    }
    kbuf = kzalloc(count, GFP_KERNEL );
    if (!kbuf) {
        goto exit;
    }
    if (copy_from_user(kbuf, buf, count)) {
        goto exit_kfree;
    }
    startpos = kbuf;
    lastc = kbuf + count;
    while (startpos < lastc)
    {
        LOG_DEBUG("idx:%d\n", idx);
        firstc = strstr(startpos, "0x");
        if (!firstc) {
            LOG_ERR("cant find firstc\n");
            goto exit_kfree;
        }
        firstc[4] = 0;
        ret = kstrtouint(startpos, 0, &tempdata);
        if (ret) {
            LOG_ERR("fail to covert digit\n");
            goto exit_kfree;
        }
        if (idx == 0) {
            set_mode = tempdata;
            LOG_ERR("set_mode:%d\n", set_mode);
        } else {
            change_val[idx - 1] = tempdata;
            LOG_DEBUG("tempdata:%d\n", tempdata);
        }
        startpos = firstc + 5;
        idx++;
        if (set_mode == 0 && idx > 3 && idx >= change_val[0] + 3)
            break;
        else if (set_mode == 1 && idx > 3)
            break;
        else if (set_mode == 2 && idx > 4 && idx >= change_val[0] + 4)
            break;
        else if (set_mode == 3 && idx > 4)
            break;
    }

    cs_press_set_device_wakeup();
    if (set_mode == 0) {
        cs_press_iic_write(change_val[1],
            &change_val[2], (unsigned int)change_val[0]);
        g_cs_press.read_reg_len = 0;
    } else if (set_mode == 1) {
        cs_press_iic_read(change_val[1],
            &g_cs_press.read_reg_data[0], (unsigned int)change_val[0]);
        g_cs_press.read_reg_len = change_val[0];
    } else if (set_mode == 2) {
        cs_press_iic_write_double_reg(((change_val[1]<<8) | change_val[2]),
            &change_val[3], (unsigned int)change_val[0]);
        g_cs_press.read_reg_len = 0;
    } else if (set_mode == 3) {
        cs_press_iic_read_double_reg(((change_val[1]<<8) | change_val[2]),
            &g_cs_press.read_reg_data[0], (unsigned int)change_val[0]);
        g_cs_press.read_reg_len = change_val[0];
    } else {
        g_cs_press.read_reg_len = 0;
    }
exit_kfree:
    kfree(kbuf);
exit:
    return count;
}

static ssize_t cs_proc_iic_rw_test_write(struct file *file, const char __user *buf,
    size_t count, loff_t *offset)
{
    char *kbuf = NULL;
    int ret = 0;
    const char *startpos = NULL;
    char *firstc = NULL;
    unsigned int tempdata = 0;

    if(count <= 0){
        LOG_ERR("argument err\n");
        goto exit;
    }
    kbuf = kzalloc(count, GFP_KERNEL );
    if (!kbuf) {
        goto exit;
    }
    startpos = kbuf;
    if (copy_from_user(kbuf, buf, count)) {
        goto exit_kfree;
    }
    firstc = strstr(startpos, "0x");
    if (!firstc) {
        LOG_ERR("param format invalid\n");
        goto exit_kfree;
    }
    firstc[4] = 0;
    ret = kstrtouint(startpos, 0, &tempdata);
    if (ret) {
        LOG_ERR("param convert to int failed\n");
        goto exit_kfree;
    }
    ret = cs_press_iic_rw_test(tempdata);
    if(ret < 0){
        LOG_ERR("iic_rw_test:err:%d\n",ret);
    }else{
        LOG_ERR("iic_rw_test:ok:%d\n",ret);
    }
exit_kfree:
    kfree(kbuf);
exit:
    return count;
}

static ssize_t cs_proc_nav_mode_write(struct file *file, const char __user *buf,
    size_t count, loff_t *offset)
{
    char *kbuf = NULL;
    int ret = 0;
    const char *startpos = NULL;
    char *firstc = NULL;
    unsigned int tempdata = 0;

    if(count <= 0){
        LOG_ERR("argument err\n");
        goto exit;
    }
    kbuf = kzalloc(count, GFP_KERNEL );
    if (!kbuf) {
        goto exit;
    }
    startpos = kbuf;
    if (copy_from_user(kbuf, buf, count)) {
        goto exit_kfree;
    }
    firstc = strstr(startpos, "0x");
    if (!firstc) {
        LOG_ERR("param format invalid\n");
        goto exit_kfree;
    }
    firstc[4] = 0;
    ret = kstrtouint(startpos, 0, &tempdata);
    if (ret) {
        LOG_ERR("param convert to int failed\n");
        goto exit_kfree;
    }

    g_cs_press.nav_mode = tempdata;
    LOG_INFO("navigation mode: %d\n", g_cs_press.nav_mode);

exit_kfree:
    kfree(kbuf);
exit:
    return count;
}

/**
  * @brief      set_debug_mode
  * @param[in]  reg addr, debug mode
  * @retval     0: success, -1: fail
  */
int set_debug_mode(unsigned char addr, unsigned char data)
{
    int ret = 0;
    int len = 2;
    unsigned char reg_addr;
    unsigned char reg_data[2];

    reg_addr = addr;
    len = 1;
    reg_data[0] = data;
    ret |= cs_press_iic_write(reg_addr, reg_data, len);
    if (ret == 0)
    {
        LOG_DEBUG("reg = 0x%x, data = 0x%x, len = %d\n",
            reg_addr, reg_data[0], len);
    }

    reg_addr = DEBUG_READY_REG;
    len = 1;
    reg_data[0] = 0; /* clear ready num */
    ret |= cs_press_iic_write(reg_addr, reg_data, len);
    if (ret == 0) {
        LOG_DEBUG("reg = 0x%x, data = 0x%x, len = %d\n",
            reg_addr, reg_data[0], len);
    }
    return ret;
}

/**
  * @brief      get_debug_data_ready
  * @param[in]  reg addr
  * @retval     ready data num
  */
int get_debug_data_ready(unsigned char addr)
{
    int ret = 0;
    int len = 1;
    unsigned char reg_addr;
    unsigned char reg_data[1];

    reg_addr = addr;
    len = 1;
    reg_data[0] = 0;
    ret |= cs_press_iic_read(reg_addr, reg_data, len);
    if (ret == 0) {
        LOG_DEBUG("reg = 0x%x, data = 0x%x, len = %d\n", reg_addr, reg_data[0], len);
    }else{
        LOG_ERR("reg = 0x%x, data = 0x%x, len = %d, err\n", reg_addr, reg_data[0], len);
    }
    return (int)reg_data[0];
}

/**
  * @brief      get_debug_data
  * @param[in]  reg  len
  * @param[out] ready data
  * @retval     0: success, -1: fail
  */
int get_debug_data(unsigned char addr, unsigned char* data, int len)
{
    int ret = 0;
    unsigned char reg_addr;
    unsigned char *reg_data;
    int i;

    reg_data = (unsigned char *)kmalloc(len, GFP_KERNEL );
    if (NULL == reg_data) {
        LOG_ERR("kmalloc fails. line : %d\n", __LINE__);
        return -1;
    }
    memset(reg_data, 0, len);
    reg_addr = addr;
    reg_data[0] = 0;
    ret = cs_press_iic_read(reg_addr, reg_data, len);
    if (ret == 0) {
        LOG_DEBUG("reg = 0x%x, data = 0x%x, len = %d, err\n",
            reg_addr, reg_data[0], len);
    }
    for(i = 0 ; i < len ; i++)
        data[i] = reg_data[i];

    if (NULL != reg_data) {
        kfree(reg_data);
    }
    return ret;
}

/**
  * @brief      check calibration result
  * @param[out] check result data buf
  * @retval     0:running, -1: error, 1: success, 2: fail, 3: overtime
  */
static int cs_proc_get_rawdata_show(struct seq_file *m,void *v)
{
    int ret = 0;
    int reg_len = 64;
    unsigned char reg_addr;
    unsigned char reg_data[64];
    int each_raw_size = 2;
    int len = 16;
    short raw_data[16];
    int i;
    int retry = 50;

    cs_press_wakeup_iic();
    set_debug_mode(DEBUG_MODE_REG, DEBUG_RAW_MODE);
    do {
        msleep(5);
        len = get_debug_data_ready((unsigned char)DEBUG_READY_REG);
        if (len > 0) {
            ret = get_debug_data(DEBUG_DATA_REG, reg_data, len);
            seq_printf(m,"D0 : %d, D1 : %d, D2 : %d, len : %d\n", reg_data[0], reg_data[1], reg_data[2], len);
        }
        retry--;
    } while ((len == 0) && retry > 0);

    if (len > reg_len)
    {
        len = reg_len;
    }
    if (len > 0) {
        for (i = 0 ; i < CH_NUM ; i++) {
            raw_data[i] = ((short)(reg_data[i * each_raw_size] & 0xff)
                | (short)((reg_data[i * each_raw_size + 1] << 8) & 0xff00));
            seq_printf(m,"raw_data %d = %d\n", i, raw_data[i]);
        }
        seq_printf(m,"\n");
    } else {
        seq_printf(m,"Data is not ready yet!\n");
    }
    set_debug_mode(DEBUG_MODE_REG, DEBUG_CLEAR_MODE);
    reg_addr = DEBUG_READY_REG;
    len = 1;
    reg_data[0] = 0x0;
    cs_press_iic_write(reg_addr, reg_data, len);
    return ret;
}

static int cs_proc_get_rawdata_open(struct inode *inode, struct file *filp)
{
    if(filp == NULL){
        return -1;
    }
    return single_open(filp, cs_proc_get_rawdata_show, NULL);
}
/**
  * @brief      check calibration result
  * @param[out] check result data buf
  * @retval     0:running, -1: error, 1: success, 2: fail, 3: overtime
  */
static int cs_proc_get_forcedata_show(struct seq_file *m,void *v)
{
    int ret;
    unsigned char reg_addr;
    unsigned char reg_data[64];
    int each_raw_size = 2;
    int len = 16;
    short raw_data[16];
    int i;
    int retry = 50;

    cs_press_wakeup_iic();

    set_debug_mode(DEBUG_MODE_REG, DEBUG_FORCE_DATA_MODE);

    do {
        msleep(5);
        len = get_debug_data_ready((unsigned char)DEBUG_READY_REG);
        if (len > 0) {
            ret = get_debug_data(DEBUG_DATA_REG, reg_data, len);
            seq_printf(m,"D0 : %d, D1 : %d, D2 : %d, len : %d\n", reg_data[0], reg_data[1], reg_data[2], len);
        }
        retry--;
    } while (len == 0 && retry > 0);

    ret = 0;
    if (len > 0)
    {
        for (i = 0 ; i < CH_NUM ; i++)
        {
            raw_data[i] = ((short)(reg_data[i * each_raw_size] & 0xff)
                | ((short)(reg_data[i * each_raw_size + 1] & 0xff) << 8));

            seq_printf(m, "diff_data:%05d ", raw_data[i]);
        }
        seq_printf(m, "\n");
    } else {
        seq_printf(m, "Data is not ready yet!\n");
    }
    set_debug_mode(DEBUG_MODE_REG, DEBUG_CLEAR_MODE);
    reg_addr = DEBUG_READY_REG;
    len = 1;
    reg_data[0] = 0x0;
    cs_press_iic_write(reg_addr, reg_data, len);
    return 0;
}

static int cs_proc_get_forcedata_open(struct inode *inode, struct file *filp)
{
    if(filp == NULL){
        LOG_ERR("filp is null\n");
        return -1;
    }
    return single_open(filp,cs_proc_get_forcedata_show,NULL);
}


static int cs_proc_calibration_param_show(struct seq_file *m,void *v)
{
    int ret = 0;
    int read_coeffs[CH_NUM];
    int i;
    unsigned char read_temp[FW_ONE_BLOCK_LENGTH_R] = {0};

    cs_press_wakeup_iic();
    ret |= cs_press_iic_read(AP_RD_CAIL_REG, read_temp, CS_CALI_PARA_LENGTH);  /* calibrate coef */

    if (ret == 0)
    {
        for (i = 0 ; i < CH_NUM ; i++)
        {
            read_coeffs[i] = ((unsigned short)read_temp[i*2+0]<<8) | read_temp[i*2+1];
            seq_printf(m, "%05d ", read_coeffs[i]);
        }
        seq_printf(m, "\n");
    } else {
        seq_printf(m, "read coefs fail!\n");
    }
    return 0;
}

static int cs_proc_calicoef_open(struct inode *inode, struct file *filp)
{
    if(filp == NULL){
        return -1;
    }
    return single_open(filp, cs_proc_calibration_param_show, NULL);
}


/**
  * @brief    check iic function
  * @param
  * @retval
*/
int cs_check_i2c(void)
{
    int retry = 3;
    unsigned char rbuf[2];
    unsigned char addr;
    int len = 0;

    addr = 0x03;
    len = 1;
    rbuf[0] = 0;
    rbuf[1] = 0;

    do {
        if (cs_press_iic_rw_test(0x67) >= 0)
            return 1;
        msleep(50);
        retry--;
        LOG_INFO("read fw ver fail!retry:%d\n", retry);
    } while (retry > 0);

    retry = 3;
    do {
        cs_press_reset_ic();
        msleep(300);
        if (cs_press_iic_rw_test(0x67) >= 0)
            return 1;
        retry--;
        LOG_INFO("reset fw fail!retry:%d\n", retry);
    } while (retry > 0);

    return -1;
}

/**
  * @brief    update function
  * @param
  * @retval
*/

static void __maybe_unused update_work_func(struct work_struct *worker)
{
    /*
    int ret;
    ret = cs_press_init();
    if (ret < 0){
        LOG_ERR("press init err\n");
    }else{
        LOG_DEBUG("press init ok\n");
    }
    */
    cancel_delayed_work(&g_cs_press.update_worker);
}
/*
#ifdef I2C_CHECK_SCHEDULE

static void  i2c_check_func(struct work_struct *worker)
{
    int ret = 0;
    ret = cs_check_i2c();
    if(ret != -1){
        schedule_delayed_work(&g_cs_press.i2c_check_worker, msecs_to_jiffies(CHECK_DELAY_TIME));
        LOG_ERR("i2c_check_func start,delay 20s.\n");
    }

}
#endif
*/

/* procfs api*/
static int cs_proc_fw_info_show(struct seq_file *m,void *v)
{
    char ret = 0;

    unsigned char read_temp[FW_ONE_BLOCK_LENGTH_R] = {0};
    cs_press_wakeup_iic();
    ret |= cs_press_iic_read(AP_VERSION_REG, read_temp, CS_FW_VERSION_LENGTH);  /*FW Version*/
    if(ret==0)
    {
        seq_printf(m,"fw_ver: %d %d %d %d\n",
             read_temp[0], read_temp[1], read_temp[2], read_temp[3]);
    }else{
        seq_printf(m,"read fw info err\n");
    }
    return 0;
}

static int cs_proc_fw_info_open(struct inode *inode, struct file *filp)
{
    if(filp == NULL){

        return -1;
    }
    return single_open(filp,cs_proc_fw_info_show,NULL);
}

static ssize_t cs_proc_fw_file_update_write(struct file *file, const char __user *buf,
                                            size_t count, loff_t *offset)
{
    char *kbuf = NULL;
    int err = 0;

    kbuf = kzalloc(count, GFP_KERNEL );
    if (!kbuf) {
        goto exit;
    }
    if (copy_from_user(kbuf, buf, count)) {
        goto exit_kfree;
    }

    if(kbuf[0] == '2')
    {
        g_cs_press.update_type = FORCE_FILE_UPDATE;
    }else
    {
        g_cs_press.update_type = HIGH_VER_FILE_UPDATE;
    }
    err = fml_fw_update_by_file();
    if (err == 0)
        LOG_DEBUG("pass!\n");
    else
        LOG_ERR("%d,failed!\n", err);

exit_kfree:
    kfree(kbuf);
exit:
    return count;
}

static int cs_read_boot_version_show(struct seq_file *m, void *v)
{
    char result = 0;
    unsigned char boot_ver_buf[4];

    result = cs_read_boot_version(boot_ver_buf);
    if (result == 0){
        seq_printf(m,"%02X %02X %02X %02X\n", boot_ver_buf[0],boot_ver_buf[1],boot_ver_buf[2],boot_ver_buf[3]);
    }else{
        seq_printf(m,"ERR\n");
    }
    return 0;
}

static int cs_proc_read_boot_version_open(struct inode *inode, struct file *filp)
{
    return single_open(filp,cs_read_boot_version_show,NULL);
}


static int cs_proc_show(struct seq_file *m,void *v)
{
    return 0;
}

static int cs_proc_open(struct inode *inode, struct file *filp)
{
    return single_open(filp,cs_proc_show,NULL);
}

static ssize_t cs_proc_write(struct file *file, const char __user *buf,
                             size_t count, loff_t *offset)
{
    char *kbuf = NULL;

    kbuf = kzalloc(count, GFP_KERNEL );
    if (!kbuf) {
        goto exit;
    }
    if (copy_from_user(kbuf, buf, count)) {
        goto exit_kfree;
    }
exit_kfree:
    kfree(kbuf);
exit:
    return count;
}

/**
  * @brief    dts parse
  * @param
  * @retval
*/
int cs_parse_dts(struct i2c_client *pdev)
{
    int ret = -1;
    struct regulator *power_3v3 = NULL;
    const char *name_tmp;

#ifdef INT_SET_EN

    cs_press_irq_gpio = of_get_named_gpio((pdev->dev).of_node,"irq-gpio",0);
    if(!gpio_is_valid(cs_press_irq_gpio))
    {
        dev_err(&pdev->dev, "cs_press request_irq IRQ fail");
    }
    else
    {
        ret = gpio_request(cs_press_irq_gpio, "irq-gpio");
        if(ret)
        {
            dev_err(&pdev->dev, "cspress request_irq IRQ fail !,ret=%d.\n", ret);
        }
        ret = gpio_direction_input(cs_press_irq_gpio);
        msleep(50);
        cs_press_irq_num = gpio_to_irq(cs_press_irq_gpio);
        ret = request_irq(cs_press_irq_num,
          (irq_handler_t)cs_press_interrupt_handler,
          IRQF_TRIGGER_FALLING |IRQF_TRIGGER_RISING |IRQF_ONESHOT,
          "CS_PRESS-eint", g_cs_press.device_irq);
        if (ret > 0) {
            ret = -1;
            dev_err(&pdev->dev, "cs_press request_irq IRQ fail !,ret=%d.\n", ret);
        }
    }
#endif
    g_cs_press.rst_gpio = of_get_named_gpio((pdev->dev).of_node, "rst-gpio", 0);
    if(!gpio_is_valid(g_cs_press.rst_gpio))
    {
        dev_err(&pdev->dev, "cs_press rst-gpio fail");
    } else {
        ret = gpio_request(g_cs_press.rst_gpio, "rst-gpio");
            if(ret)
            {
                dev_err(&pdev->dev, "cs_press request rst fail !,ret=%d.\n", ret);
            }
            ret = gpio_direction_output(g_cs_press.rst_gpio,0);
            msleep(50);
        gpio_set_value(g_cs_press.rst_gpio, 0);
    }

    g_cs_press.power_gpio = of_get_named_gpio((pdev->dev).of_node, "power-gpio", 0);
    if(!gpio_is_valid(g_cs_press.power_gpio))
    {
        dev_err(&pdev->dev, "cs_press power-gpio fail");
    } else {
        ret = gpio_request(g_cs_press.power_gpio, "power-gpio");
            if(ret)
            {
                dev_err(&pdev->dev, "cs_press request power-gpio fail !,ret=%d.\n", ret);
            }
            ret = gpio_direction_output(g_cs_press.power_gpio,0);
            msleep(50);
        gpio_set_value(g_cs_press.power_gpio, 1);
    }

    power_3v3 = regulator_get(&pdev->dev, "vdd_3v3");
    if (power_3v3 != NULL) {
        dev_err(&pdev->dev,"%s:power_3v3 is not NULL\n", __func__);
        ret = regulator_enable(power_3v3);
        if (ret)
            dev_err(&pdev->dev,"%s: power_3v3 enable fail\n", __func__);
    } else {
            dev_err(&pdev->dev,"%s: power_3v3 is NULL\n", __func__);
    }

    ret = of_property_read_u32((pdev->dev).of_node, "cs,panel-max-x",
                 &g_cs_press.panel_max_x);
    if (ret) {
        LOG_ERR("failed get panel-max-x\n");
        return ret;
    }

    ret = of_property_read_u32((pdev->dev).of_node, "cs,panel-max-y",
                 &g_cs_press.panel_max_y);
    if (ret) {
        LOG_ERR("failed get panel-max-y\n");
        return ret;
    }

    ret = of_property_read_u32((pdev->dev).of_node, "cs,panel-max-w",
                 &g_cs_press.panel_max_w);
    if (ret) {
        LOG_ERR("failed get panel-max-w\n");
        return ret;
    }

    ret = of_property_read_string((pdev->dev).of_node, "cs,fw-name", &name_tmp);
    if (!ret) {
        strncpy(g_cs_press.cs_fw_name, name_tmp, sizeof(g_cs_press.cs_fw_name));
        LOG_INFO("firmware name from dt: %s", g_cs_press.cs_fw_name);
    } else {
        strncpy(g_cs_press.cs_fw_name, FW_FILE_NAME, sizeof(g_cs_press.cs_fw_name));
        LOG_INFO("can't find firmware name, use default: %s", g_cs_press.cs_fw_name);
    }

    LOG_ERR("end---\n");
    return 0;
}
/**
  * @brief    free resource from dts info
  * @param
  * @retval 0:success, -1: fail
  */
void cs_unregister_dts(void){
    /*GPIO unregister*/
    if(gpio_is_valid(g_cs_press.rst_gpio)){
        gpio_free(g_cs_press.rst_gpio);
        LOG_DEBUG("reset gpio free\n");
    }
    #ifdef INT_SET_EN
    if(gpio_is_valid(cs_press_irq_gpio)){
        free_irq(cs_press_irq_num,g_cs_press.device_irq);
        gpio_free(cs_press_irq_gpio);
        LOG_DEBUG("irq gpio free\n");
        g_cs_press.device_irq = NULL;
    }
    #endif
}

static const char proc_list[PROC_FOPS_NUM][PROC_NAME_LEN]={
    "fw_info",
    "read_boot_version",
    "fw_update_file",
    "rw_reg",
    "cs_iic_test",
    "read_rawdata",
    "read_cali_coef",
    "read_force_data",
    "nav_mode",
};

#ifndef KERNEL_4_1_5
static const struct proc_ops proc_fops[PROC_FOPS_NUM] = {
#else
static const struct file_operations proc_fops[PROC_FOPS_NUM] = {
#endif

    FOPS_ARRAY(cs_proc_fw_info_open, NULL),    /*fw_info*/
    FOPS_ARRAY(cs_proc_read_boot_version_open, cs_proc_write),/*read boot version*/
    FOPS_ARRAY(cs_proc_open, cs_proc_fw_file_update_write),  /*update*/
    FOPS_ARRAY(cs_proc_rw_reg_open, cs_proc_rw_reg_write),   /*rw_reg*/
    FOPS_ARRAY(cs_proc_open, cs_proc_iic_rw_test_write),    /*IIC read write test*/
    FOPS_ARRAY(cs_proc_get_rawdata_open, NULL),
    FOPS_ARRAY(cs_proc_calicoef_open, NULL),
    FOPS_ARRAY(cs_proc_get_forcedata_open, NULL),
    FOPS_ARRAY(cs_proc_open, cs_proc_nav_mode_write),
};
/**
  * @brief  cs_sys_create
  * @param
  * @retval
*/
static int cs_procfs_create(void)
{
    int ret = 0;
    int i = 0;
    struct proc_dir_entry *file;

    g_cs_press.p_proc_dir = proc_mkdir(CS_PRESS_NAME, NULL);
    if(g_cs_press.p_proc_dir == NULL){
        ret = -1;
        goto exit;
    }
    for(i = 0;i < PROC_FOPS_NUM;i++)
    {
        file = proc_create_data(proc_list[i], 0666, g_cs_press.p_proc_dir, &proc_fops[i], NULL);
        /*file = proc_create(proc_list[i],0644,NULL,&proc_fops[i]);*/
        if(file == NULL){
            ret = -1;
            LOG_ERR("proc %s create failed",proc_list[i]);
            goto err_flag;
        }
    }
    return 0;
err_flag:
    remove_proc_entry(CS_PRESS_NAME,NULL);
exit:
    return ret;
}

static void cs_procfs_delete(void)
{
    int i = 0;

    for(i = 0;i < PROC_FOPS_NUM;i++)
    {
        remove_proc_entry(proc_list[i], g_cs_press.p_proc_dir);
        LOG_DEBUG("proc %s is removed", proc_list[i]);
    }
    remove_proc_entry(CS_PRESS_NAME,NULL);
}

/********sysfs node start*********/
/**
  * @brief      show navigation mode
  * @param[out] current nav mode type
  * @retval
  */
static ssize_t cs_sysfs_nav_mode_show(struct device *dev, struct device_attribute *attr,
                   char *buf)
{
    return scnprintf(buf, PAGE_SIZE, "navigation mode: %u\n", g_cs_press.nav_mode);
}

/**
  * @brief      switch navigation mode
  * @param[in] nav mode type 2:zoom, 3:scroll
  * @retval
  */
static ssize_t cs_sysfs_nav_mode_store(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t size)
{
    unsigned long value = 0;
    int err = 0;

    err = kstrtoul(buf, 10, &value);
    if (err < 0) {
        LOG_INFO("Failed to convert value");
        return -EINVAL;
    }

    g_cs_press.nav_mode = value;
    LOG_INFO("navigation mode: %d\n", g_cs_press.nav_mode);

    return size;
}
static DEVICE_ATTR(nav_mode, (S_IWUSR | S_IWGRP | S_IRUGO), cs_sysfs_nav_mode_show,
        cs_sysfs_nav_mode_store);

/**
  * @brief      upgrade IC firmware manually
  * @param[in] upgrade type 2:force file update, others:high version update
  * @retval
  */
static ssize_t cs_sysfs_fw_update_file_store(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t size)
{
    unsigned long value = 0;
    int err = 0;

    err = kstrtoul(buf, 10, &value);
    if (err < 0) {
        LOG_INFO("Failed to convert value");
        return -EINVAL;
    }

    if(value == 2) {
        g_cs_press.update_type = FORCE_FILE_UPDATE;
    } else {
        g_cs_press.update_type = HIGH_VER_FILE_UPDATE;
    }
    err = fml_fw_update_by_file();
    if (err == 0)
        LOG_DEBUG("pass!\n");
    else
        LOG_ERR("%d,failed!\n", err);

    return size;
}
static DEVICE_ATTR(fw_update_file, (S_IWUSR | S_IWGRP), NULL, cs_sysfs_fw_update_file_store);

/**
  * @brief      read firmware bootloader info from IC
  * @param[out] check result data buf
  * @retval     firmware bootloader info
  */
static ssize_t cs_sysfs_read_boot_version_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
    char result = 0;
    unsigned char boot_ver_buf[4];

    result = cs_read_boot_version(boot_ver_buf);
    if (result == 0){
        return scnprintf(buf, PAGE_SIZE, "%02X %02X %02X %02X\n",
                boot_ver_buf[0],boot_ver_buf[1],boot_ver_buf[2],boot_ver_buf[3]);
    } else {
        return scnprintf(buf, PAGE_SIZE, "ERR\n");
    }
}
static DEVICE_ATTR(read_boot_version, S_IRUGO, cs_sysfs_read_boot_version_show, NULL);

/**
  * @brief      read firmware info from IC
  * @param[out] check result data buf
  * @retval     firmware info
  */
static ssize_t cs_sysfs_fw_info_show(struct device *dev, struct device_attribute *attr,
                   char *buf)
{
    char ret = 0;
    unsigned char read_temp[FW_ONE_BLOCK_LENGTH_R] = {0};

    cs_press_wakeup_iic();
    ret |= cs_press_iic_read(AP_VERSION_REG, read_temp, CS_FW_VERSION_LENGTH);  /*FW Version*/
    if(ret==0)
    {
        return scnprintf(buf, PAGE_SIZE, "fw_ver: %d %d %d %d\n",
                read_temp[0], read_temp[1], read_temp[2], read_temp[3]);
    } else {
        return scnprintf(buf, PAGE_SIZE, "read fw info err\n");
    }
}
static DEVICE_ATTR(fw_info, S_IRUGO, cs_sysfs_fw_info_show, NULL);

static struct attribute *cs_press_attributes[] = {
    &dev_attr_fw_info.attr,
    &dev_attr_read_boot_version.attr,
    &dev_attr_fw_update_file.attr,
    &dev_attr_nav_mode.attr,
    NULL
};

static struct attribute_group cs_press_attribute_group = {
    .attrs = cs_press_attributes
};
/********sysfs node end*********/

/********misc node for ndt*********/
static int cs_press_open(struct inode *inode, struct file *filp)
{
    LOG_DEBUG("cs_press_open\n");
    if(g_cs_press.client == NULL)
    {
        return -1;
    }
    return 0;
}

static ssize_t cs_press_write(struct file *file, const char __user *buf,
        size_t count, loff_t *offset)
{
    int err;
    char *kbuf = NULL;
    char reg;

    kbuf = kzalloc(count, GFP_KERNEL );
    if (!kbuf)
    {
        err = -ENOMEM;
        goto exit;
    }

    if (copy_from_user(&reg, buf, 1) || copy_from_user(kbuf, buf+1, count))
    {
        err = -EFAULT;
        goto exit_kfree;
    }
    err = cs_press_iic_write(reg, kbuf, count);
    if(err >= 0)
    {
        err = 1;
    }

    exit_kfree:
    kfree(kbuf);
    exit:
    return err;
}

static ssize_t cs_press_read(struct file *filp, char __user *buf, size_t count, loff_t *off)
{
    int err = 0;
    char *kbuf = NULL;
    char reg = 0;

    kbuf = kzalloc(count, GFP_KERNEL );
    if (!kbuf) {
        err = -ENOMEM;
        goto exit;
    }
    /*get reg addr buf[0]*/
    if (copy_from_user(&reg, buf, 1)) {
        err = -EFAULT;
        goto exit_kfree;
    }
    err = cs_press_iic_read(reg, kbuf, count);
    if (err < 0)
        goto exit_kfree;
    if (copy_to_user(buf+1, kbuf, count))
    {
        err = -EFAULT;
    }else{
        err = 1;
    }
    exit_kfree:
    kfree(kbuf);
    exit:
    return err;
}

static int cs_press_release(struct inode *inode, struct file *filp)
{
    return 0;
}
/*
 * @ misc device file operation
 */
static const struct file_operations cs_press_fops = {
    .owner = THIS_MODULE,
    .open = cs_press_open,
    .write = cs_press_write,
    .read = cs_press_read,
    .release = cs_press_release,
};

static struct miscdevice cs_press_misc = {
    .minor = MISC_DYNAMIC_MINOR,
    .name  = MISC_DEVICE_NAME,
    .fops  = &cs_press_fops,
};

static int cs_press_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int ret = -1;
    struct device *parent = &client->dev;

    LOG_DEBUG("probe init\n");
    ret = misc_register(&cs_press_misc); /*dev node*/
    g_cs_press.client = client;
    cs_parse_dts(g_cs_press.client);
    cs_press_struct_init();
    cs_procfs_create();                 /*proc node*/
#ifdef INT_SET_EN
    eint_init();
#endif

    if (kfifo_alloc(&g_cs_press.data_queue,
        CS_DATA_MAX_QUEUE * sizeof(struct cs_press_coords), GFP_KERNEL)) {
        LOG_ERR("failed to alloc queue for raw data\n");
        goto ALLOC_KFIFO_FAILED;
    }

    dev_set_drvdata(&client->dev, &g_cs_press);
    //create sysfs node
    ret = alloc_chrdev_region(&g_cs_press.class_dev_no, 0, 1, CS_SYSFS_NAME);
    if (ret < 0) {
        LOG_ERR("get device number failed\n");
        goto GET_NEW_MINOR_FAILED;
    }

    g_cs_press.class_dev = device_create(cs_press_class, parent, g_cs_press.class_dev_no,
                &g_cs_press, "%s", CS_SYSFS_NAME);

    ret = sysfs_create_group(&g_cs_press.class_dev->kobj, &cs_press_attribute_group);
    if (ret < 0) {
        LOG_ERR("error creating sysfs attr files");
        goto CLASS_DEVICE_ATTR_CREATE_FAILED;
    }

    // INIT_DELAYED_WORK(&g_cs_press.update_worker, update_work_func);
    // schedule_delayed_work(&g_cs_press.update_worker, msecs_to_jiffies(2000));
    // LOG_ERR("update_work_func start,delay 2s.\n");

/*
#ifdef I2C_CHECK_SCHEDULE
    INIT_DELAYED_WORK(&g_cs_press.i2c_check_worker, i2c_check_func);
    schedule_delayed_work(&g_cs_press.i2c_check_worker, msecs_to_jiffies(CHECK_DELAY_TIME));
    LOG_ERR("i2c_check_func start,delay 20s.\n");
#endif
*/
    LOG_ERR("end!\n");
    return 0;

CLASS_DEVICE_ATTR_CREATE_FAILED:
    unregister_chrdev_region(g_cs_press.class_dev_no, 1);

GET_NEW_MINOR_FAILED:
    kfifo_free(&g_cs_press.data_queue);

ALLOC_KFIFO_FAILED:
    cs_unregister_dts();
    cs_procfs_delete();
#ifdef INT_SET_EN
    eint_exit();
#endif
    misc_deregister(&cs_press_misc);
    g_cs_press.client = NULL;

    return ret;
}

static int cs_press_remove(struct i2c_client *client)
{
    /* delete device */
    if(g_cs_press.client == NULL){
        return 0;
    }
    LOG_DEBUG("cs_remove\n");
    kfifo_free(&g_cs_press.data_queue);
    sysfs_remove_group(&g_cs_press.class_dev->kobj, &cs_press_attribute_group);
    unregister_chrdev_region(g_cs_press.class_dev_no, 1);
    cs_unregister_dts();
    cs_procfs_delete();
#ifdef INT_SET_EN
    eint_exit();
#endif
    misc_deregister(&cs_press_misc);
    g_cs_press.client = NULL;
    return 0;
}

/*
 * @traditional match list,use thie when not using dts
 */
static const struct i2c_device_id cs_press_id[] = {
    {CS_PRESS_NAME, 0},
    {/*northing to be done*/},
};

/*
 * @dts match list
 */
static const struct of_device_id cs_press_match[] = {
    { .compatible = "chipsea,cs_press" },
    {/*northing to be done*/},
};

/**
 * @Support fast loading of hot swap devices
 */
MODULE_DEVICE_TABLE(i2c, cs_press_id);

/*i2c driver struct*/
static struct i2c_driver cs_press_driver = {
    .probe = cs_press_probe,
    .remove = cs_press_remove,
    .driver = {
            .owner = THIS_MODULE,
            .name = CS_PRESS_NAME,
            .of_match_table = of_match_ptr(cs_press_match),/*if use dts,use of_match_table to match*/
           },
    .id_table = cs_press_id,
};

static int __init cs_press_init(void)
{
    int ret = 0;

    cs_press_class = class_create(THIS_MODULE, "g_cs_press");
    if (IS_ERR(cs_press_class))
    {
        LOG_ERR("cs press class init failed");
        return PTR_ERR(cs_press_class);
    }

    ret = i2c_add_driver(&cs_press_driver);
    return ret;
}

static void __exit cs_press_exit(void)
{
    LOG_DEBUG("module exit\n");
    i2c_del_driver(&cs_press_driver);
    cs_procfs_delete();
    if(cs_press_class) {
        class_destroy(cs_press_class);
    }
}

module_init(cs_press_init);
module_exit(cs_press_exit);
MODULE_LICENSE("GPL");
