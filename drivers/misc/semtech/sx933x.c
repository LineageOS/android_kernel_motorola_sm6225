/*! \file sx933x.c
 * \brief  SX933x Driver
 *
 * Driver for the SX933x
 * Copyright (c) 2018 Semtech Corp
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
//#define DEBUG
#define DRIVER_NAME "sx933x"

#define MAX_WRITE_ARRAY_SIZE 32

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/syscalls.h>
//#include <linux/wakelock.h>
#include <linux/uaccess.h>
#include <linux/sort.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>

#include "sx933x.h" 	/* main struct, interrupt,init,pointers */

#define SX933x_I2C_M_WR                 0 /* for i2c Write */
#define SX933x_I2C_M_RD                 1 /* for i2c Read */

#define IDLE			    0
#define ACTIVE			  1

#define MAIN_SENSOR		1 //CS1

/* Failer Index */
#define SX933x_ID_ERROR 	1
#define SX933x_NIRQ_ERROR	2
#define SX933x_CONN_ERROR	3
#define SX933x_I2C_ERROR	4

/*! \struct sx933x
 * Specialized struct containing input event data, platform data, and
 * last cap state read if needed.
 */
typedef struct sx933x
{
    pbuttonInformation_t pbuttonInformation;
    psx933x_platform_data_t hw;		/* specific platform data settings */
} sx933x_t, *psx933x_t;

static int irq_gpio_num;

static int sx933x_get_nirq_state(void)
{
    return  !gpio_get_value(irq_gpio_num);
}


/*! \fn static int sx933x_i2c_write_16bit(psx93XX_t this, u8 address, u8 value)
 * \brief Sends a write register to the device
 * \param this Pointer to main parent struct
 * \param address 8-bit register address
 * \param value   8-bit register value to write to address
 * \return Value from i2c_master_send
 */

static int sx933x_i2c_write_16bit(psx93XX_t this, u16 reg_addr, u32 buf)
{
    int ret =  -ENOMEM;
    struct i2c_client *i2c = 0;
    struct i2c_msg msg;
    unsigned char w_buf[6];

    if (this && this->bus)
    {
        i2c = this->bus;
        w_buf[0] = (u8)(reg_addr>>8);
        w_buf[1] = (u8)(reg_addr);
        w_buf[2] = (u8)(buf>>24);
        w_buf[3] = (u8)(buf>>16);
        w_buf[4] = (u8)(buf>>8);
        w_buf[5] = (u8)(buf);

        msg.addr = i2c->addr;
        msg.flags = SX933x_I2C_M_WR;
        msg.len = 6; //2bytes regaddr + 4bytes data
        msg.buf = (u8 *)w_buf;

        ret = i2c_transfer(i2c->adapter, &msg, 1);
        if (ret < 0)
            pr_err("[SX933x]: %s - i2c write error %d\n", __func__, ret);

    }
    return ret;
}



/*! \fn static int sx933x_i2c_read_16bit(psx93XX_t this, u8 address, u8 *value)
* \brief Reads a register's value from the device
* \param this Pointer to main parent struct
* \param address 8-Bit address to read from
* \param value Pointer to 8-bit value to save register value to
* \return Value from i2c_smbus_read_byte_data if < 0. else 0
*/
static int sx933x_i2c_read_16bit(psx93XX_t this, u16 reg_addr, u32 *data32)
{
    int ret =  -ENOMEM;
    struct i2c_client *i2c = 0;
    struct i2c_msg msg[2];
    u8 w_buf[2];
    u8 buf[4];

    if (this && this->bus)
    {
        i2c = this->bus;

        w_buf[0] = (u8)(reg_addr>>8);
        w_buf[1] = (u8)(reg_addr);

        msg[0].addr = i2c->addr;
        msg[0].flags = SX933x_I2C_M_WR;
        msg[0].len = 2;
        msg[0].buf = (u8 *)w_buf;

        msg[1].addr = i2c->addr;;
        msg[1].flags = SX933x_I2C_M_RD;
        msg[1].len = 4;
        msg[1].buf = (u8 *)buf;

        ret = i2c_transfer(i2c->adapter, msg, 2);
        if (ret < 0)
            pr_err("[SX933x]: %s - i2c read error %d\n", __func__, ret);

        data32[0] = ((u32)buf[0]<<24) | ((u32)buf[1]<<16) | ((u32)buf[2]<<8) | ((u32)buf[3]);

    }
    return ret;
}




//static int sx933x_set_mode(psx93XX_t this, unsigned char mode);

/*! \fn static int read_regStat(psx93XX_t this)
 * \brief Shortcut to read what caused interrupt.
 * \details This is to keep the drivers a unified
 * function that will read whatever register(s)
 * provide information on why the interrupt was caused.
 * \param this Pointer to main parent struct
 * \return If successful, Value of bit(s) that cause interrupt, else 0
 */
static int read_regStat(psx93XX_t this)
{
    u32 data = 0;
    if (this)
    {
        if (sx933x_i2c_read_16bit(this,SX933X_HOSTIRQSRC_REG,&data) > 0) //bug
            return (data & 0x00FF);
    }
    return 0;
}

static int sx933x_Hardware_Check(psx93XX_t this)
{
    int ret;
    u32 failcode;
    u8 loop = 0;
    this->failStatusCode = 0;

    //Check th IRQ Status
    while(this->get_nirq_low && this->get_nirq_low())
    {
        read_regStat(this);
        msleep(100);
        if(++loop >10)
        {
            this->failStatusCode = SX933x_NIRQ_ERROR;
            break;
        }
    }

    //Check I2C Connection
    ret = sx933x_i2c_read_16bit(this, SX933X_INFO_REG, &failcode);
    if(ret < 0)
    {
        this->failStatusCode = SX933x_I2C_ERROR;
    }

    if(failcode!= SX933X_WHOAMI_VALUE)
    {
        this->failStatusCode = SX933x_ID_ERROR;
    }

    dev_info(this->pdev, "[SX933x]: sx933x failcode = 0x%x\n",this->failStatusCode);
    return (int)this->failStatusCode;
}

static int sx933x_global_variable_init(psx93XX_t this)
{
    this->irq_disabled = 0;
    this->failStatusCode = 0;
    this->reg_in_dts = true;
    return 0;
}

/*! \brief Perform a manual offset calibration
* \param this Pointer to main parent struct
* \return Value return value from the write register
 */
static int manual_offset_calibration(psx93XX_t this)
{
    int ret = 0;
    ret = sx933x_i2c_write_16bit(this, SX933X_CMD_REG, I2C_REGCMD_COMPEN);
    return ret;

}

/****************************************************************************/
/*! \brief sysfs show function for manual calibration which currently just
 * returns register value.
 */
static ssize_t manual_offset_calibration_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    u32 reg_value = 0;
    psx93XX_t this = dev_get_drvdata(dev);

    dev_info(this->pdev, "[SX933x]: Reading IRQSTAT_REG\n");
    sx933x_i2c_read_16bit(this,SX933X_HOSTIRQSRC_REG,&reg_value);
    return sprintf(buf, "%d\n", reg_value);
}


static ssize_t manual_offset_calibration_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    unsigned long val;
    psx93XX_t this = dev_get_drvdata(dev);

    if (kstrtoul(buf, 10, &val))                //(strict_strtoul(buf, 10, &val)) {
    {
        pr_err("[SX933x]: %s - Invalid Argument\n", __func__);
        return -EINVAL;
    }

    if (val)
        manual_offset_calibration(this);

    return count;
}

/****************************************************************************/
static ssize_t sx933x_register_write_store(struct device *dev,
        struct device_attribute *attr,  const char *buf, size_t count)
{
    u32 reg_address = 0, val = 0;
    psx93XX_t this = dev_get_drvdata(dev);

    if (sscanf(buf, "%x,%x", &reg_address, &val) != 2)
    {
        pr_err("[SX933x]: %s - The number of data are wrong\n",__func__);
        return -EINVAL;
    }

    sx933x_i2c_write_16bit(this, reg_address, val);

    pr_info("[SX933x]: %s - Register(0x%x) data(0x%x)\n",__func__, reg_address, val);
    return count;
}

//read registers not include the advanced one
static ssize_t sx933x_register_read_store(struct device *dev,
        struct device_attribute *attr,  const char *buf, size_t count)
{
    u32 val=0;
    int regist = 0;
    int nirq_state = 0;
    psx93XX_t this = dev_get_drvdata(dev);

    dev_info(this->pdev, "Reading register\n");

    if (sscanf(buf, "%x", &regist) != 1)
    {
        pr_err("[SX933x]: %s - The number of data are wrong\n",__func__);
        return -EINVAL;
    }

    sx933x_i2c_read_16bit(this, regist, &val);
    nirq_state = sx933x_get_nirq_state();

    pr_info("[SX933x]: %s - Register(0x%2x) data(0x%4x) nirq_state(%d)\n",__func__, regist, val, nirq_state);
    return count;
}

static void read_rawData(psx93XX_t this)
{
    u8 csx, index;
    s32 useful;
    s32 average;
    s32 diff;
    u32 uData;
    u16 offset;
    s32 state = 0;

    if(this)
    {
        for(csx =0; csx<5; csx++)
        {
            index = csx*4;
            sx933x_i2c_read_16bit(this, SX933X_USEPH0_REG + index, &uData);
            useful = (s32)uData>>10;
            sx933x_i2c_read_16bit(this, SX933X_AVGPH0_REG + index, &uData);
            average = (s32)uData>>10;
            sx933x_i2c_read_16bit(this, SX933X_DIFFPH0_REG + index, &uData);
            diff = (s32)uData>>10;
            sx933x_i2c_read_16bit(this, SX933X_OFFSETPH0_REG + index, &uData);
            offset = (u16)(uData & 0x7FFF);
            state = psmtcButtons[csx].state;
            dev_info(this->pdev, "[SX933x]: [CS: %d] Sta = %d, Useful = %d Average = %d, DIFF = %d Offset = %d \n",
                     csx,state,useful,average,diff,offset);
        }
    }
}

static ssize_t sx933x_raw_data_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    psx93XX_t this = dev_get_drvdata(dev);
    read_rawData(this);
    return 0;
}



static DEVICE_ATTR(manual_calibrate, 0664, manual_offset_calibration_show,manual_offset_calibration_store);
static DEVICE_ATTR(register_write,  0664, NULL,sx933x_register_write_store);
static DEVICE_ATTR(register_read,0664, NULL,sx933x_register_read_store);
static DEVICE_ATTR(raw_data,0664,sx933x_raw_data_show,NULL);



static struct attribute *sx933x_attributes[] =
{
    &dev_attr_manual_calibrate.attr,
    &dev_attr_register_write.attr,
    &dev_attr_register_read.attr,
    &dev_attr_raw_data.attr,
    NULL,
};
static struct attribute_group sx933x_attr_group =
{
    .attrs = sx933x_attributes,
};

/**************************************/

/*! \brief  Initialize I2C config from platform data
 * \param this Pointer to main parent struct
 */
static void sx933x_reg_init(psx93XX_t this)
{
    psx933x_t pDevice = 0;
    psx933x_platform_data_t pdata = 0;
    int i = 0;
    uint32_t tmpvalue;
    /* configure device */
    dev_info(this->pdev, "[SX933x]:Going to Setup I2C Registers\n");
    if (this && (pDevice = this->pDevice) && (pdata = pDevice->hw))
    {
        /*******************************************************************************/
        // try to initialize from device tree!
        /*******************************************************************************/
        while ( i < ARRAY_SIZE(sx933x_i2c_reg_setup))
        {
            /* Write all registers/values contained in i2c_reg */
            dev_info(this->pdev, "[SX933x]:Going to Write Reg: 0x%x Value: 0x%x\n",
                     sx933x_i2c_reg_setup[i].reg,sx933x_i2c_reg_setup[i].val);
            tmpvalue = sx933x_i2c_reg_setup[i].val;
            if (sx933x_i2c_reg_setup[i].reg == SX933X_GNRLCTRL2_REG)
            {
                if((sx933x_i2c_reg_setup[i].val & 0x3F) == 0)
                {
                    tmpvalue = (sx933x_i2c_reg_setup[i].val|0x3F);
                }
            }
            //sx933x_i2c_write_16bit(this, sx933x_i2c_reg_setup[i].reg,sx933x_i2c_reg_setup[i].val);
            sx933x_i2c_write_16bit(this, sx933x_i2c_reg_setup[i].reg, tmpvalue);
            i++;
        }
#if 0
        if (this->reg_in_dts == true)
        {
            while ( i < pdata->i2c_reg_num)
            {
                /* Write all registers/values contained in i2c_reg */
                dev_info(this->pdev, "[SX933x]: Going to Write Reg from dts: 0x%x Value: 0x%x\n",
                         pdata->pi2c_reg[i].reg,pdata->pi2c_reg[i].val);
                sx933x_i2c_write_16bit(this, pdata->pi2c_reg[i].reg,pdata->pi2c_reg[i].val);
                i++;
            }
        }
        else     // use static ones!!
        {
            while ( i < ARRAY_SIZE(sx933x_i2c_reg_setup))
            {
                /* Write all registers/values contained in i2c_reg */
                dev_info(this->pdev, "[SX933x]:Going to Write Reg: 0x%x Value: 0x%x\n",
                         sx933x_i2c_reg_setup[i].reg,sx933x_i2c_reg_setup[i].val);
                sx933x_i2c_write_16bit(this, sx933x_i2c_reg_setup[i].reg,sx933x_i2c_reg_setup[i].val);
                i++;
            }
        }
#endif
        /*******************************************************************************/
        sx933x_i2c_write_16bit(this, SX933X_CMD_REG,SX933X_PHASE_CONTROL);  //enable phase control
    }
    else
    {
        dev_err(this->pdev, "[SX933x]: ERROR! platform data 0x%p\n",pDevice->hw);
    }

}


/*! \fn static int initialize(psx93XX_t this)
 * \brief Performs all initialization needed to configure the device
 * \param this Pointer to main parent struct
 * \return Last used command's return value (negative if error)
 */
static int initialize(psx93XX_t this)
{
    int ret;
    if (this)
    {
        pr_info("[SX933x]: SX933x income initialize\n");
        /* prepare reset by disabling any irq handling */
        this->irq_disabled = 1;
        disable_irq(this->irq);
        /* perform a reset */
        sx933x_i2c_write_16bit(this, SX933X_RESET_REG, I2C_SOFTRESET_VALUE);
        /* wait until the reset has finished by monitoring NIRQ */
        dev_info(this->pdev, "Sent Software Reset. Waiting until device is back from reset to continue.\n");
        /* just sleep for awhile instead of using a loop with reading irq status */
        msleep(100);
        ret = sx933x_global_variable_init(this);
        sx933x_reg_init(this);
        msleep(100); /* make sure everything is running */
        manual_offset_calibration(this);

        /* re-enable interrupt handling */
        enable_irq(this->irq);

        /* make sure no interrupts are pending since enabling irq will only
        * work on next falling edge */
        read_regStat(this);
        return 0;
    }
    return -ENOMEM;
}

/*!
 * \brief Handle what to do when a touch occurs
 * \param this Pointer to main parent struct
 */
static void touchProcess(psx93XX_t this)
{
    int counter = 0;
    u32 i = 0;
    int numberOfButtons = 0;
    psx933x_t pDevice = NULL;
    struct _buttonInfo *buttons = NULL;
    struct input_dev *input = NULL;

    struct _buttonInfo *pCurrentButton  = NULL;

    if (this && (pDevice = this->pDevice))
    {
        sx933x_i2c_read_16bit(this, SX933X_STAT0_REG, &i);

        buttons = pDevice->pbuttonInformation->buttons;
        input = pDevice->pbuttonInformation->input;
        numberOfButtons = pDevice->pbuttonInformation->buttonSize;

        if (unlikely( (buttons==NULL) || (input==NULL) ))
        {
            dev_err(this->pdev, "[SX933x]:ERROR!! buttons or input NULL!!!\n");
            return;
        }

        for (counter = 0; counter < numberOfButtons; counter++)
        {
            pCurrentButton = &buttons[counter];
            if (pCurrentButton==NULL)
            {
                dev_err(this->pdev,"[SX933x]:ERROR!! current button at index: %d NULL!!!\n", counter);
                return; // ERRORR!!!!
            }
            switch (pCurrentButton->state)
            {
            case IDLE: /* Button is not being touched! */
                if (((i & pCurrentButton->mask) == pCurrentButton->mask))
                {
                    /* User pressed button */
                    dev_info(this->pdev, "[SX933x]:Button %d touched\n", counter);
                    input_report_key(input, pCurrentButton->keycode, 1);
                    pCurrentButton->state = ACTIVE;
                }
                else
                {
                    dev_info(this->pdev, "[SX933x]:Button %d already released.\n",counter);
                }
                break;
            case ACTIVE: /* Button is being touched! */
                if (((i & pCurrentButton->mask) != pCurrentButton->mask))
                {
                    /* User released button */
                    dev_info(this->pdev, "[SX933x]:Button %d released\n",counter);
                    input_report_key(input, pCurrentButton->keycode, 0);
                    pCurrentButton->state = IDLE;
                }
                else
                {
                    dev_info(this->pdev, "[SX933x]:Button %d still touched.\n",counter);
                }
                break;
            default: /* Shouldn't be here, device only allowed ACTIVE or IDLE */
                break;
            };
        }
        input_sync(input);

        dev_info(this->pdev, "[SX933x]:Leaving touchProcess()\n");
    }
}



static int sx933x_parse_dt(struct sx933x_platform_data *pdata, struct device *dev)
{
    struct device_node *dNode = dev->of_node;
    enum of_gpio_flags flags;
    //int ret;

    if (dNode == NULL)
        return -ENODEV;

    pdata->irq_gpio= of_get_named_gpio_flags(dNode,
                     "Semtech,nirq-gpio", 0, &flags);
    irq_gpio_num = pdata->irq_gpio;
    if (pdata->irq_gpio < 0)
    {
        pr_err("[SENSOR]: %s - get irq_gpio error\n", __func__);
        return -ENODEV;
    }
#if 0
    /***********************************************************************/
    // load in registers from device tree
    of_property_read_u32(dNode,"Semtech,reg-num",&pdata->i2c_reg_num);
    // layout is register, value, register, value....
    // if an extra item is after just ignore it. reading the array in will cause it to fail anyway
    pr_info("[SX933x]:%s -  size of elements %d \n", __func__,pdata->i2c_reg_num);
    if (pdata->i2c_reg_num > 0)
    {
        // initialize platform reg data array
        pdata->pi2c_reg = devm_kzalloc(dev,sizeof(struct smtc_reg_data)*pdata->i2c_reg_num, GFP_KERNEL);
        if (unlikely(pdata->pi2c_reg == NULL))
        {
            return -ENOMEM;
        }

        // initialize the array
        if (of_property_read_u32_array(dNode,"Semtech,reg-init",(u32*)&(pdata->pi2c_reg[0]),sizeof(struct smtc_reg_data)*pdata->i2c_reg_num))
            return -ENOMEM;
    }
    /***********************************************************************/
#endif
    pr_info("[SX933x]: %s -[%d] parse_dt complete\n", __func__,pdata->irq_gpio);
    return 0;
}

/* get the NIRQ state (1->NIRQ-low, 0->NIRQ-high) */
static int sx933x_init_platform_hw(struct i2c_client *client)
{
    psx93XX_t this = i2c_get_clientdata(client);
    struct sx933x *pDevice = NULL;
    struct sx933x_platform_data *pdata = NULL;
    int rc = 0;

    pr_info("[SX933x] : %s init_platform_hw start!",__func__);

    if (this && (pDevice = this->pDevice) && (pdata = pDevice->hw))
    {
        if (gpio_is_valid(pdata->irq_gpio))
        {
            rc = gpio_request(pdata->irq_gpio, "sx933x_irq_gpio");
            if (rc < 0)
            {
                dev_err(this->pdev, "SX933x Request gpio. Fail![%d]\n", rc);
                return rc;
            }
            rc = gpio_direction_input(pdata->irq_gpio);
            if (rc < 0)
            {
                dev_err(this->pdev, "SX933x Set gpio direction. Fail![%d]\n", rc);
                return rc;
            }
            this->irq = client->irq = gpio_to_irq(pdata->irq_gpio);
        }
        else
        {
            dev_err(this->pdev, "SX933x Invalid irq gpio num.(init)\n");
        }
    }
    else
    {
        pr_err("[SX933x] : %s - Do not init platform HW", __func__);
    }

    pr_err("[SX933x]: %s - sx933x_irq_debug\n",__func__);
    return rc;
}

static void sx933x_exit_platform_hw(struct i2c_client *client)
{
    psx93XX_t this = i2c_get_clientdata(client);
    struct sx933x *pDevice = NULL;
    struct sx933x_platform_data *pdata = NULL;

    if (this && (pDevice = this->pDevice) && (pdata = pDevice->hw))
    {
        if (gpio_is_valid(pdata->irq_gpio))
        {
            gpio_free(pdata->irq_gpio);
        }
        else
        {
            dev_err(this->pdev, "Invalid irq gpio num.(exit)\n");
        }
    }
    return;
}

/*! \fn static int sx933x_probe(struct i2c_client *client, const struct i2c_device_id *id)
 * \brief Probe function
 * \param client pointer to i2c_client
 * \param id pointer to i2c_device_id
 * \return Whether probe was successful
 */
static int sx933x_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int i = 0;
    int err = 0;

    psx93XX_t this = 0;
    psx933x_t pDevice = 0;
    psx933x_platform_data_t pplatData = 0;
    struct totalButtonInformation *pButtonInformationData = NULL;
    struct input_dev *input = NULL;
    struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);

    dev_info(&client->dev, "[SX933x]:sx933x_probe()\n");

    if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_READ_WORD_DATA))
    {
        dev_err(&client->dev, "[SX933x]:Check i2c functionality.Fail!\n");
        err = -EIO;
        return err;
    }

    this = devm_kzalloc(&client->dev,sizeof(sx93XX_t), GFP_KERNEL); /* create memory for main struct */
    dev_info(&client->dev, "[SX933x]:\t Initialized Main Memory: 0x%p\n",this);

    pButtonInformationData = devm_kzalloc(&client->dev , sizeof(struct totalButtonInformation), GFP_KERNEL);
    if (!pButtonInformationData)
    {
        dev_err(&client->dev, "[SX933x]:Failed to allocate memory(totalButtonInformation)\n");
        err = -ENOMEM;
        return err;
    }

    pButtonInformationData->buttonSize = ARRAY_SIZE(psmtcButtons);
    pButtonInformationData->buttons =  psmtcButtons;
    pplatData = devm_kzalloc(&client->dev,sizeof(struct sx933x_platform_data), GFP_KERNEL);
    if (!pplatData)
    {
        dev_err(&client->dev, "[SX933x]:platform data is required!\n");
        return -EINVAL;
    }
    pplatData->get_is_nirq_low = sx933x_get_nirq_state;
    pplatData->pbuttonInformation = pButtonInformationData;

    client->dev.platform_data = pplatData;
    err = sx933x_parse_dt(pplatData, &client->dev);
    if (err)
    {
        dev_err(&client->dev, "[SX933x]:could not setup pin\n");
        return ENODEV;
    }

    pplatData->init_platform_hw = sx933x_init_platform_hw;
    dev_err(&client->dev, "[SX933x]:SX933x init_platform_hw done!\n");

    if (this)
    {
        dev_info(&client->dev, "[SX933x]:SX933x initialize start!!");
        /* In case we need to reinitialize data
        * (e.q. if suspend reset device) */
        this->init = initialize;
        /* shortcut to read status of interrupt */
        this->refreshStatus = read_regStat;
        /* pointer to function from platform data to get pendown
        * (1->NIRQ=0, 0->NIRQ=1) */
        this->get_nirq_low = pplatData->get_is_nirq_low;
        /* save irq in case we need to reference it */
        this->irq = client->irq;
        /* do we need to create an irq timer after interrupt ? */
        this->useIrqTimer = 0;

        /* Setup function to call on corresponding reg irq source bit */
        if (MAX_NUM_STATUS_BITS>= 8)
        {
            this->statusFunc[0] = 0; /* TXEN_STAT */
            this->statusFunc[1] = 0; /* UNUSED */
            this->statusFunc[2] = 0; /* UNUSED */
            this->statusFunc[3] = read_rawData; /* CONV_STAT */
            this->statusFunc[4] = 0; /* COMP_STAT */
            this->statusFunc[5] = touchProcess; /* RELEASE_STAT */
            this->statusFunc[6] = touchProcess; /* TOUCH_STAT  */
            this->statusFunc[7] = 0; /* RESET_STAT */
        }

        /* setup i2c communication */
        this->bus = client;
        i2c_set_clientdata(client, this);

        /* record device struct */
        this->pdev = &client->dev;

        /* create memory for device specific struct */
        this->pDevice = pDevice = devm_kzalloc(&client->dev,sizeof(sx933x_t), GFP_KERNEL);
        dev_info(&client->dev, "[SX933x]:\t Initialized Device Specific Memory: 0x%p\n",pDevice);

        if (pDevice)
        {
            /* for accessing items in user data (e.g. calibrate) */
            err = sysfs_create_group(&client->dev.kobj, &sx933x_attr_group);
            //sysfs_create_group(client, &sx933x_attr_group);

            /* Add Pointer to main platform data struct */
            pDevice->hw = pplatData;

            /* Check if we hava a platform initialization function to call*/
            if (pplatData->init_platform_hw)
                pplatData->init_platform_hw(client);

            /* Initialize the button information initialized with keycodes */
            pDevice->pbuttonInformation = pplatData->pbuttonInformation;
            /* Create the input device */
            input = input_allocate_device();
            if (!input)
            {
                return -ENOMEM;
            }
            /* Set all the keycodes */
            __set_bit(EV_KEY, input->evbit);
#if 1
            for (i = 0; i < pButtonInformationData->buttonSize; i++)
            {
                __set_bit(pButtonInformationData->buttons[i].keycode,input->keybit);
                pButtonInformationData->buttons[i].state = IDLE;
            }
#endif
            /* save the input pointer and finish initialization */
            pButtonInformationData->input = input;
            input->name = "SX933x Cap Touch";
            input->id.bustype = BUS_I2C;
            if(input_register_device(input))
            {
                return -ENOMEM;
            }
        }

        sx93XX_IRQ_init(this);
        /* call init function pointer (this should initialize all registers */
        if (this->init)
        {
            this->init(this);
        }
        else
        {
            dev_err(this->pdev,"[SX933x]:No init function!!!!\n");
            return -ENOMEM;
        }
    }
    else
    {
        return -1;
    }

    sx933x_Hardware_Check(this);
    pplatData->exit_platform_hw = sx933x_exit_platform_hw;

    dev_info(&client->dev, "[SX933x]:sx933x_probe() Done\n");

    return 0;
}

/*! \fn static int sx933x_remove(struct i2c_client *client)
 * \brief Called when device is to be removed
 * \param client Pointer to i2c_client struct
 * \return Value from sx93XX_remove()
 */
//static int __devexit sx933x_remove(struct i2c_client *client)
static int sx933x_remove(struct i2c_client *client)
{
    psx933x_platform_data_t pplatData =0;
    psx933x_t pDevice = 0;
    psx93XX_t this = i2c_get_clientdata(client);
    if (this && (pDevice = this->pDevice))
    {
        input_unregister_device(pDevice->pbuttonInformation->input);

        sysfs_remove_group(&client->dev.kobj, &sx933x_attr_group);
        pplatData = client->dev.platform_data;
        if (pplatData && pplatData->exit_platform_hw)
            pplatData->exit_platform_hw(client);
        kfree(this->pDevice);
    }
    return sx93XX_remove(this);
}
#if 1//def CONFIG_PM
/*====================================================*/
/***** Kernel Suspend *****/
static int sx933x_suspend(struct device *dev)
{
    psx93XX_t this = dev_get_drvdata(dev);
    sx93XX_suspend(this);
    return 0;
}
/***** Kernel Resume *****/
static int sx933x_resume(struct device *dev)
{
    psx93XX_t this = dev_get_drvdata(dev);
    sx93XX_resume(this);
    return 0;
}
/*====================================================*/
#else
#define sx933x_suspend		NULL
#define sx933x_resume		NULL
#endif /* CONFIG_PM */

static struct i2c_device_id sx933x_idtable[] =
{
    { DRIVER_NAME, 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, sx933x_idtable);
#ifdef CONFIG_OF
static struct of_device_id sx933x_match_table[] =
{
    { .compatible = "Semtech,sx933x",},
    { },
};
#else
#define sx933x_match_table NULL
#endif
static const struct dev_pm_ops sx933x_pm_ops =
{
    .suspend = sx933x_suspend,
    .resume = sx933x_resume,
};
static struct i2c_driver sx933x_driver =
{
    .driver = {
        .owner			= THIS_MODULE,
        .name			= DRIVER_NAME,
        .of_match_table	= sx933x_match_table,
        .pm				= &sx933x_pm_ops,
    },
    .id_table		= sx933x_idtable,
          .probe			= sx933x_probe,
                  .remove			= sx933x_remove,
                     };
static int __init sx933x_I2C_init(void)
{
    return i2c_add_driver(&sx933x_driver);
}
static void __exit sx933x_I2C_exit(void)
{
    i2c_del_driver(&sx933x_driver);
}

module_init(sx933x_I2C_init);
module_exit(sx933x_I2C_exit);

MODULE_AUTHOR("Semtech Corp. (http://www.semtech.com/)");
MODULE_DESCRIPTION("SX933x Capacitive Proximity Controller Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1");

static void sx93XX_schedule_work(psx93XX_t this, unsigned long delay)
{
    unsigned long flags;
    if (this)
    {
        dev_info(this->pdev, "sx93XX_schedule_work()\n");
        spin_lock_irqsave(&this->lock,flags);
        /* Stop any pending penup queues */
        cancel_delayed_work(&this->dworker);
        //after waiting for a delay, this put the job in the kernel-global workqueue. so no need to create new thread in work queue.
        schedule_delayed_work(&this->dworker,delay);
        spin_unlock_irqrestore(&this->lock,flags);
    }
    else
        printk(KERN_ERR "sx93XX_schedule_work, NULL psx93XX_t\n");
}

static irqreturn_t sx93XX_irq(int irq, void *pvoid)
{
    psx93XX_t this = 0;
    if (pvoid)
    {
        this = (psx93XX_t)pvoid;
        if ((!this->get_nirq_low) || this->get_nirq_low())
        {
            sx93XX_schedule_work(this,0);
        }
        else
        {
            dev_err(this->pdev, "sx93XX_irq - nirq read high\n");
        }
    }
    else
    {
        printk(KERN_ERR "sx93XX_irq, NULL pvoid\n");
    }
    return IRQ_HANDLED;
}

static void sx93XX_worker_func(struct work_struct *work)
{
    psx93XX_t this = 0;
    int status = 0;
    int counter = 0;
    u8 nirqLow = 0;
    if (work)
    {
        this = container_of(work,sx93XX_t,dworker.work);

        if (!this)
        {
            printk(KERN_ERR "sx93XX_worker_func, NULL sx93XX_t\n");
            return;
        }
        if (unlikely(this->useIrqTimer))
        {
            if ((!this->get_nirq_low) || this->get_nirq_low())
            {
                nirqLow = 1;
            }
        }
        /* since we are not in an interrupt don't need to disable irq. */
        status = this->refreshStatus(this);
        counter = -1;
        dev_dbg(this->pdev, "Worker - Refresh Status %d\n",status);

        while((++counter) < MAX_NUM_STATUS_BITS)   /* counter start from MSB */
        {
            if (((status>>counter) & 0x01) && (this->statusFunc[counter]))
            {
                dev_info(this->pdev, "SX933x Function Pointer Found. Calling\n");
                this->statusFunc[counter](this);
            }
        }
        if (unlikely(this->useIrqTimer && nirqLow))
        {
            /* Early models and if RATE=0 for newer models require a penup timer */
            /* Queue up the function again for checking on penup */
            sx93XX_schedule_work(this,msecs_to_jiffies(this->irqTimeout));
        }
    }
    else
    {
        printk(KERN_ERR "sx93XX_worker_func, NULL work_struct\n");
    }
}

int sx93XX_remove(psx93XX_t this)
{
    if (this)
    {
        cancel_delayed_work_sync(&this->dworker); /* Cancel the Worker Func */
        /*destroy_workqueue(this->workq); */
        free_irq(this->irq, this);
        kfree(this);
        return 0;
    }
    return -ENOMEM;
}
void sx93XX_suspend(psx93XX_t this)
{
    if (this)
        disable_irq(this->irq);
    //sx933x_i2c_write_16bit(this,SX933X_CPS_CTRL0_REG,0x20);//make sx933x in Sleep mode
}
void sx93XX_resume(psx93XX_t this)
{
    if (this)
    {
        enable_irq(this->irq);
        //sx933x_i2c_write_16bit(this,SX933X_CPS_CTRL0_REG,0x27);//Exit from Sleep mode
    }
}

int sx93XX_IRQ_init(psx93XX_t this)
{
    int err = 0;
    if (this && this->pDevice)
    {
        /* initialize spin lock */
        spin_lock_init(&this->lock);
        /* initialize worker function */
        INIT_DELAYED_WORK(&this->dworker, sx93XX_worker_func);
        /* initailize interrupt reporting */
        this->irq_disabled = 0;
        err = request_irq(this->irq, sx93XX_irq, IRQF_TRIGGER_FALLING,
                          this->pdev->driver->name, this);
        if (err)
        {
            dev_err(this->pdev, "irq %d busy?\n", this->irq);
            return err;
        }
        dev_info(this->pdev, "registered with irq (%d)\n", this->irq);
    }
    return -ENOMEM;
}
