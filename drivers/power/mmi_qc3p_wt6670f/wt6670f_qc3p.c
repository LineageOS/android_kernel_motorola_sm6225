#include <linux/types.h>
#include <linux/init.h>		/* For init/exit macros */
#include <linux/module.h>	/* For MODULE_ marcros  */
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/unistd.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
//#include <mt-plat/mtk_boot.h>
//#include <mt-plat/upmu_common.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/power_supply.h>
#include <linux/regulator/driver.h>
#include "wt6670f.h"

#define GET_FIRMWARE_NUM_SHOW_MAX_SIZE 50

static DEFINE_MUTEX(wt6670f_i2c_access);
//static DEFINE_MUTEX(wt6670f_access_lock);
//static struct i2c_client *new_client;
//static int wt6670f_reset_pin = -1;
static int wt6670f_int_pin = -1;
struct wt6670f *_wt = NULL;
int g_qc3p_id = 0;
int m_chg_type = 0;
bool qc3p_z350_init_ok = false;

static int __wt6670f_write_word(struct wt6670f *wt, u8 reg, u16 data)
{
	s32 ret;

	ret = i2c_smbus_write_word_data(wt->client, reg, data);
	if (ret < 0) {
		pr_err("i2c write fail: can't write from reg 0x%02X\n", reg);
		return ret;
	}

	return 0;
}


static int __wt6670f_read_word(struct wt6670f *wt, u8 reg, u16 *data)
{
	s32 ret;

	ret = i2c_smbus_read_word_data(wt->client, reg);
	if (ret < 0) {
		pr_err("i2c read fail: can't read from reg 0x%02X\n", reg);
		return ret;
	}

	*data = (u16) ret;

	return 0;
}

static int wt6670f_read_word(struct wt6670f *wt, u8 reg, u16 *data)
{
	int ret;

	mutex_lock(&wt->i2c_rw_lock);
	ret = __wt6670f_read_word(wt, reg, data);
	mutex_unlock(&wt->i2c_rw_lock);

	return ret;
}

static int wt6670f_write_word(struct wt6670f *wt, u8 reg, u16 data)
{
	int ret;

	mutex_lock(&wt->i2c_rw_lock);
	ret = __wt6670f_write_word(wt, reg, data);
	mutex_unlock(&wt->i2c_rw_lock);

	return ret;
}

static u16 wt6670f_get_vbus_voltage(void)
{
	int ret = 0;
	u8 data[2];
	u16 tmp;
	if(QC3P_WT6670F == g_qc3p_id)
		ret = wt6670f_read_word(_wt, 0xBE, (u16 *)data);
	else if(QC3P_Z350 == g_qc3p_id)
		ret = wt6670f_read_word(_wt, 0x12, (u16 *)data);
	else
		ret = -1;
	if(ret < 0)
	{
		pr_err("wt6670f get vbus voltage fail\n");
		return ret;
	}

	tmp = ((data[0] << 8) | data[1]) & 0x3ff;

	pr_err(">>>>>>wt6670f get vbus voltage = %04x  %02x  %02x\n", tmp,
			data[0], data[1]);
	return (u16)(tmp * 18.98);
}

static u16 wt6670f_get_id(u8 reg)
{
	int ret;
	u16 data;

	ret = wt6670f_read_word(_wt, reg, &data);//wt6670f-0xBC,Z350-0x13

	if(ret < 0)
	{
		pr_err("wt6670f get id fail\n");
		return ret;
	}

	pr_err(">>>>>>wt6670f get id = %x\n", data);
	return data;
}

int wt6670f_start_detection(void)
{
	int ret;
	u16 data = 0x01;

	ret = wt6670f_write_word(_wt, 0xB6, data);
	if (ret < 0)
	{
		pr_info("wt6670f start detection fail\n");
		return ret;
	}

	return data & 0xff;
}
EXPORT_SYMBOL_GPL(wt6670f_start_detection);

int wt6670f_re_run_apsd(void)
{
	int ret;
	u16 data = 0x01;

	ret = wt6670f_write_word(_wt, 0xB6, data);

	if (ret < 0)
	{
		pr_info("wt6670f re run apsd fail\n");
		return ret;
	}

	return data & 0xff;
}
EXPORT_SYMBOL_GPL(wt6670f_re_run_apsd);

int wt6670f_en_hvdcp(void)
{
	int ret;
	u16 data = 0x01;

	ret = wt6670f_write_word(_wt, 0x05, data);

	if (ret < 0)
	{
		pr_info("z350 en hvdcp fail\n");
		return ret;
	}

	return data & 0xff;
}
EXPORT_SYMBOL_GPL(wt6670f_en_hvdcp);

int wt6670f_get_protocol(void)
{
	int ret = 0;
	u16 data;
	u8 data1, data2;
	if(QC3P_WT6670F == g_qc3p_id){
		ret = wt6670f_read_word(_wt, 0xBD, &data);
		pr_err("wt6670f get protocol %x\n",data);
	}
	else if(QC3P_Z350 == g_qc3p_id){
		ret = wt6670f_read_word(_wt, 0x11, &data);
		pr_err("z350 get protocol %x\n",data);
	}
	else
		ret = -1;

	if (ret < 0)
	{
		pr_err("wt6670f get protocol fail\n");
		return ret;
	}

        // Get data2 part
        data1 = data & 0xFF;
        data2 = data >> 8;
	if((QC3P_Z350 == g_qc3p_id)&&(data1 <= 7))
		data1--;
        pr_err("Get charger type, rowdata = 0X%04x, data1= 0X%02x, data2=0X%02x \n", data, data1, data2);
        if((data2 == 0x03) && ((data1 > 0x9) || (data1 == 0x7)))
        {
                pr_err("fail to get charger type, error happens!\n");
                return -EINVAL;
        }

        if(data2 == 0x04)
        {
                pr_err("detected QC3+ charger:0X%02x!\n", data1);
        }

	if((data1 > 0x00 && data1 < 0x07) ||
           (data1 > 0x07 && data1 < 0x0a) ||(QC3P_Z350 == g_qc3p_id && data1 == 0x10)){
		ret = data1;
	}
	else {
		ret = 0x00;
	}

	_wt->chg_type = ret;
//	return data & 0xff;
	return ret;
}
EXPORT_SYMBOL_GPL(wt6670f_get_protocol);

int wt6670f_get_charger_type(void)
{
        return _wt->chg_type;
}
EXPORT_SYMBOL_GPL(wt6670f_get_charger_type);

bool wt6670f_is_charger_ready(void)
{
	return _wt->chg_ready;
}
EXPORT_SYMBOL_GPL(wt6670f_is_charger_ready);

int wt6670f_get_firmware_version(void)
{
	int ret;
	u16 data = 0;
	int read_firmware_retry = 3;

	ret = wt6670f_read_word(_wt, 0xBF, &data);
	if (ret < 0)
	{
		pr_err("wt6670f get firmware fail,retry\n");
		gpio_direction_output(_wt->intb_pin,0);
		usleep_range(5000,6000);
		gpio_direction_output(_wt->intb_pin,1);
		usleep_range(5000,6000);
		while(read_firmware_retry--){
			ret = wt6670f_read_word(_wt, 0xBF, &data);
			if(ret == 0)
				break;
		}
		gpio_direction_input(wt6670f_int_pin);
		if (ret < 0) {
			pr_err("wt6670f get firmware fail\n");
			return ret;
		}
	}

	return data & 0xff;
}
EXPORT_SYMBOL_GPL(wt6670f_get_firmware_version);


int z350_get_firmware_version(void)
{
	int ret;
	u16 data;

	ret = wt6670f_read_word(_wt, 0x14, &data);
	if (ret < 0)
	{
		pr_err("z350 get firmware fail\n");
		return ret;
	}
	return data;
}

int wt6670f_set_voltage(u16 voltage)
{
	int ret = 0;
	u16 step;
	u16 voltage_now;

	voltage_now = wt6670f_get_vbus_voltage();
        pr_err("wt6670f current voltage = %d, set voltage = %d\n", voltage_now, voltage);

	if(voltage - voltage_now < 0)
	{
		step = (u16)((voltage_now - voltage) / 20);
    _wt->count -= step;
		step &= 0x7FFF;
		step = ((step & 0xff) << 8) | ((step >> 8) & 0xff);

    if(_wt->count < 0)
        _wt->count = 0;
	}
	else
	{
		step = (u16)((voltage - voltage_now) / 20);
    _wt->count += step;
    step |= 0x8000;
    step = ((step & 0xff) << 8) | ((step >> 8) & 0xff);
	}
	pr_err("---->southchip count = %d   %04x,QC3P_WT6670F=%d,g_qc3p_id=%d,voltage=%d,voltage_now=%d\n", _wt->count, step,QC3P_WT6670F,g_qc3p_id,voltage,voltage_now);

	if(QC3P_WT6670F == g_qc3p_id)
		ret = wt6670f_write_word(_wt, 0xBB, step);
	else if(QC3P_Z350 == g_qc3p_id)
		ret = wt6670f_write_word(_wt, 0x83, step);
	else
		ret = -1;

	return ret;
}
EXPORT_SYMBOL_GPL(wt6670f_set_voltage);

int wt6670f_set_volt_count(int count)
{
        int ret = 0;
        u16 step = abs(count);

        pr_err("Set vbus with %d pulse!\n!", count);

        if(count < 0)
        {
    _wt->count -= step;
                step &= 0x7FFF;
                step = ((step & 0xff) << 8) | ((step >> 8) & 0xff);

    if(_wt->count < 0)
        _wt->count = 0;
        }
        else
        {
    _wt->count += step;
    step |= 0x8000;
    step = ((step & 0xff) << 8) | ((step >> 8) & 0xff);
        }

	if(QC3P_WT6670F == g_qc3p_id)
		ret = wt6670f_write_word(_wt, 0xBB, step);
	else if(QC3P_Z350 == g_qc3p_id)
		ret = wt6670f_write_word(_wt, 0x83, step);

        return ret;
}
EXPORT_SYMBOL_GPL(wt6670f_set_volt_count);

static ssize_t wt6670f_show_test(struct device *dev,
						struct device_attribute *attr, char *buf)
{
//	struct wt6670f *wt = dev_get_drvdata(dev);

	int idx = 0;
	u8 data;

	data = wt6670f_get_protocol();
	idx = snprintf(buf, PAGE_SIZE, ">>> protocol = %02x\n", data);
	return idx;
}

static ssize_t wt6670f_store_test(struct device *dev,
								struct device_attribute *attr, const char *buf, size_t count)
{
//	struct wt6670f *wt = dev_get_drvdata(dev);

	int val;
	int ret;

	ret = sscanf(buf, "%d", &val);

	ret = wt6670f_set_voltage(val);

	return count;

}


static ssize_t wt6670f_show_registers(struct device *dev,
						struct device_attribute *attr, char *buf)
{
	struct wt6670f *wt = dev_get_drvdata(dev);

	int idx = 0;
	int ret = 0;
	u16 data;
	if(QC3P_WT6670F == g_qc3p_id){
		ret = wt6670f_read_word(wt, 0xBD, &data);
		idx = snprintf(buf, PAGE_SIZE, ">>> reg[0xBD] = %04x\n", data);
		pr_err(">>>>>>>>>>WT6670F test southchip  0xBD = %04x\n", data);
	}

	if(QC3P_Z350 == g_qc3p_id){
		ret = wt6670f_read_word(wt, 0x11, &data);
		idx = snprintf(buf, PAGE_SIZE, ">>> reg[0x11] = %04x\n", data);
		pr_err(">>>>>>>>>>Z350 test southchip  0x11 = %04x\n", data);
	}

	return idx;
}

static ssize_t wt6670f_store_registers(struct device *dev,
								struct device_attribute *attr, const char *buf, size_t count)
{
	struct wt6670f *wt = dev_get_drvdata(dev);

	int val;
	int ret = 0;

	ret = sscanf(buf, "%x", &val);

	if(QC3P_WT6670F == g_qc3p_id)
		ret = wt6670f_write_word(wt, 0xBB, val);

	if(QC3P_Z350 == g_qc3p_id)
		ret = wt6670f_write_word(wt, 0x83, val);

	return count;

}


static DEVICE_ATTR(test, 0660, wt6670f_show_test, wt6670f_store_test);
static DEVICE_ATTR(registers, 0660, wt6670f_show_registers, wt6670f_store_registers);

static void wt6670f_create_device_node(struct device *dev)
{
	device_create_file(dev, &dev_attr_registers);
  device_create_file(dev, &dev_attr_test);
}

int wt6670f_do_reset(void)
{
	gpio_direction_output(_wt->reset_pin,1);
	msleep(1);
	gpio_direction_output(_wt->reset_pin,0);
	msleep(10);

	return 0;
}
EXPORT_SYMBOL_GPL(wt6670f_do_reset);

int wt6670f_isp_do_reset(void)
{
	gpio_direction_output(_wt->reset_pin,1);
	usleep_range(100,200);
	gpio_direction_output(_wt->reset_pin,0);
	usleep_range(2000,3000);

	return 0;
}
EXPORT_SYMBOL_GPL(wt6670f_isp_do_reset);

void wt6670f_reset_chg_type(void)
{
        _wt->chg_type = 0;
	_wt->chg_ready = false;
}
EXPORT_SYMBOL_GPL(wt6670f_reset_chg_type);

int moto_tcmd_wt6670f_get_firmware_version(void)
{
	int wt6670f_fm_ver = 0;
	int z350_fm_ver = 0;
	wt6670f_do_reset();
	wt6670f_fm_ver = wt6670f_get_firmware_version();
	z350_fm_ver = z350_get_firmware_version();
	pr_info("%s: get firmware wt6670f:%x,z350:%x!\n", __func__,wt6670f_fm_ver,z350_fm_ver);
	if((3 == wt6670f_fm_ver)||(0x080a == z350_fm_ver)||(0x0f0a == z350_fm_ver))
	return 1;
	else
	return 0;
}
EXPORT_SYMBOL_GPL(moto_tcmd_wt6670f_get_firmware_version);

static irqreturn_t wt6670f_intr_handler(int irq, void *data)
{
	m_chg_type = 0xff;
	pr_info("%s,chg_type 0x:%x\n", __func__,m_chg_type);
	_wt->chg_ready = true;

	return IRQ_HANDLED;
}

static int wt6670f_parse_dt(struct device *dev)
{
	struct device_node *np = dev->of_node;
	int ret = 0;

	pr_info("%s\n", __func__);
	if (!np) {
		pr_info("%s: no of node\n", __func__);
		return -ENODEV;
	}

	_wt->reset_pin = of_get_named_gpio(np, "wt6670f-reset-gpio", 0);
	if (_wt->reset_pin < 0)
		pr_info("[%s] of get wt6670 gpio failed, reset_pin:%d\n",__func__, _wt->reset_pin);

	if (gpio_request(_wt->reset_pin, "wt6670_reset_pin")) {
		pr_info("[%s]reset gpio_request failed", __func__);
	}
	if (gpio_direction_output(_wt->reset_pin, 0))
		pr_info("[%s] gpio_direction_output reset_pin failed", __func__);

	wt6670f_int_pin = of_get_named_gpio(np,"wt6670f-int-gpio",0);
	if(wt6670f_int_pin < 0){
		pr_info("%s: get wt6670f-int-gpio failed\n", __func__);
	}

	if (gpio_request(wt6670f_int_pin,"wt6670f_int")) {
		pr_info("[%s]wt6670f_int gpio_request failed", __func__);
	}
	if (gpio_direction_input(wt6670f_int_pin))
		pr_info("[%s] gpio_direction_input wt6670f_int_pin failed", __func__);
	_wt->intb_pin = wt6670f_int_pin;

	ret = request_irq(gpio_to_irq(wt6670f_int_pin), wt6670f_intr_handler,
		IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "wt6670f int", dev);
	enable_irq_wake(gpio_to_irq(wt6670f_int_pin));
	return 0;
}

static int wt6670_iio_write_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan, int val1,
		int val2, long mask)
{
	int rc = 0;

	switch (chan->channel) {
	case PSY_IIO_START_DETECTION:
		wt6670f_reset_chg_type();
		wt6670f_start_detection();
		pr_info("wt6670 start detection\n");
		break;
	case PSY_IIO_BATTERY_DP_DM:
		wt6670f_set_volt_count(val1);
		pr_info("wt6670 set volt count:%d\n",val1);
		break;
	default:
		pr_err("Unsupported wt6670 IIO chan %d\n", chan->channel);
		rc = -EINVAL;
		break;
	}

	if (rc < 0)
		pr_err("Couldn't write IIO channel %d, rc = %d\n",
			chan->channel, rc);

	return rc;
}

static int wt6670_iio_read_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan, int *val1,
		int *val2, long mask)
{
	int rc = 0;
	int result;
	int wait_count = 0;
	*val1 = 0;


	switch (chan->channel) {
	case PSY_IIO_QC3P_POWER:
		result = wt6670f_get_charger_type();
		pr_info("wt6670 get charger type for qc3p power:%d\n",result);
		if(result == WT6670_CHG_TYPE_QC3P_18W || result == WT6670_CHG_TYPE_QC3P_27W)
			*val1 = result;
		break;
	case PSY_IIO_QC3P_REAL_TYPE:
		wt6670f_get_protocol();
		result = wt6670f_get_charger_type();

		if((result == 0x10) &&(QC3P_Z350 == g_qc3p_id)) {
			wt6670f_en_hvdcp();
			pr_info("wt6670 wt6670f_en_hvdcp\n");
			wait_count = 0;
			while((result != 0xff)&&(wait_count<30)){
				msleep(100);
				wait_count++;
			}
			wt6670f_get_protocol();
			result = wt6670f_get_charger_type();
		}

		pr_info("wt6670 get charger type:%d\n", result);
		*val1 = result;
		break;
	case PSY_IIO_DETECTION_READY:
		result = wt6670f_is_charger_ready();
		pr_info("wt6670 get wt6670f_is_charger_ready status:%d\n", result);
		*val1 = result;
		break;
	case PSY_IIO_QC3P_FIRMWARE_NUM:

		if(QC3P_WT6670F == g_qc3p_id){
			result = wt6670f_get_firmware_version();
			pr_info("wt6670 get wt6670f_firmware_num:%d\n", result);
		} else if(QC3P_Z350 == g_qc3p_id){
			result = z350_get_firmware_version();
			pr_info("z350 get z350_get_firmware_version:0x%x\n", result);
		} else {
			result = 0;
			pr_info("could not get device id for used\n");
		}

		*val1 = result;
		break;
	default:
		pr_err("Unsupported wt6670 IIO chan %d\n", chan->channel);
		rc = -EINVAL;
		break;
	}

	if (rc < 0) {
		pr_err("Couldn't read IIO channel %d, rc = %d\n",
			chan->channel, rc);
		return rc;
	}

	return IIO_VAL_INT;
}

static int wt6670_iio_of_xlate(struct iio_dev *indio_dev,
				const struct of_phandle_args *iiospec)
{
	struct wt6670f *bq = iio_priv(indio_dev);
	struct iio_chan_spec *iio_chan = bq->iio_chan;
	int i;

	for (i = 0; i < ARRAY_SIZE(wt6670_iio_psy_channels);
					i++, iio_chan++)
		if (iio_chan->channel == iiospec->args[0])
			return i;

	return -EINVAL;
}

static const struct iio_info wt6670_iio_info = {
	.read_raw	= wt6670_iio_read_raw,
	.write_raw	= wt6670_iio_write_raw,
	.of_xlate	= wt6670_iio_of_xlate,
};

static int wt6670_init_iio_psy(struct wt6670f *chip)
{
	struct iio_dev *indio_dev = chip->indio_dev;
	struct iio_chan_spec *chan;
	int wt6670_num_iio_channels = ARRAY_SIZE(wt6670_iio_psy_channels);
	int rc, i;

	chip->iio_chan = devm_kcalloc(chip->dev, wt6670_num_iio_channels,
				sizeof(*chip->iio_chan), GFP_KERNEL);
	if (!chip->iio_chan)
		return -ENOMEM;

	chip->int_iio_chans = devm_kcalloc(chip->dev,
				wt6670_num_iio_channels,
				sizeof(*chip->int_iio_chans),
				GFP_KERNEL);
	if (!chip->int_iio_chans)
		return -ENOMEM;

	indio_dev->info = &wt6670_iio_info;
	indio_dev->dev.parent = chip->dev;
	indio_dev->dev.of_node = chip->dev->of_node;
	indio_dev->name = "wt6670";
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = chip->iio_chan;
	indio_dev->num_channels = wt6670_num_iio_channels;

	for (i = 0; i < wt6670_num_iio_channels; i++) {
		chip->int_iio_chans[i].indio_dev = indio_dev;
		chan = &chip->iio_chan[i];
		chip->int_iio_chans[i].channel = chan;
		chan->address = i;
		chan->channel = wt6670_iio_psy_channels[i].channel_num;
		chan->type = wt6670_iio_psy_channels[i].type;
		chan->datasheet_name =
			wt6670_iio_psy_channels[i].datasheet_name;
		chan->extend_name =
			wt6670_iio_psy_channels[i].datasheet_name;
		chan->info_mask_separate =
			wt6670_iio_psy_channels[i].info_mask;
	}

	rc = devm_iio_device_register(chip->dev, indio_dev);
	if (rc)
		pr_err("Failed to register wt6670 IIO device, rc=%d\n", rc);

	return rc;
}

extern int wt6670f_isp_flow(struct wt6670f *chip);

static enum power_supply_property wt6670f_charger_props[] = {
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
};

static int wt6670f_charger_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	return 0;
}

static int wt6670f_charger_set_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       const union power_supply_propval *val)
{
	return 0;
}

static const struct power_supply_desc wt6670f_psy_desc = {
	.name = "QC3P",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.properties = wt6670f_charger_props,
	.num_properties = ARRAY_SIZE(wt6670f_charger_props),
	.get_property = wt6670f_charger_get_property,
	.set_property = wt6670f_charger_set_property,
};

static int wt6670f_psy_register(struct wt6670f *wt)
{
	struct power_supply_config wt6670f_cfg = {};

	wt6670f_cfg.drv_data = wt;
	wt6670f_cfg.of_node = wt->dev->of_node;
	wt->qc3_psy = power_supply_register(wt->dev, &wt6670f_psy_desc, &wt6670f_cfg);
	if (IS_ERR(wt->qc3_psy)) {
		pr_err("Couldn't register wt6670f power supply\n");
		return PTR_ERR(wt->qc3_psy);
	}

	pr_info("power supply register wt6670f successfully\n");
	return 0;
}

static ssize_t firmware_num_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	return 0;
}

static ssize_t firmware_num_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int result;

		if(QC3P_WT6670F == g_qc3p_id){
			result = wt6670f_get_firmware_version();
			pr_info("wt6670 get wt6670f_firmware_num:%d\n", result);
		} else if(QC3P_Z350 == g_qc3p_id){
			result = z350_get_firmware_version();
			pr_info("z350 get z350_get_firmware_version:0x%x\n", result);
		} else {
			result = 0;
			pr_info("could not get device id for used\n");
		}


	return scnprintf(buf, GET_FIRMWARE_NUM_SHOW_MAX_SIZE, "%d\n", result);
}

static DEVICE_ATTR(firmware_num, 0664,
		firmware_num_show,
		firmware_num_store);

static struct attribute *wt6670f_get_firmware_num_attributes[] = {
	&dev_attr_firmware_num.attr,
	NULL,
};

static const struct attribute_group wt6670f_attribute_group = {
	.attrs = wt6670f_get_firmware_num_attributes,
};

static int wt6670f_create_sys(struct device *dev, const struct attribute_group * grp)
{
	int err = 0;
	struct power_supply *cp_ply;

	pr_err("enter wt6670f create sys \n");
	cp_ply = power_supply_get_by_name("QC3P");

	if(NULL == dev){
		pr_err("[BATT]: failed to register battery\n");
		return -EINVAL;
	}

	if(cp_ply)
	{
		err = sysfs_create_group(&cp_ply->dev.kobj, grp);

		if (!err)
		{
			pr_info("creat BMT sysfs group ok\n");
		}
		else
		{
			pr_err("creat BMT sysfs group fail\n");
			err =  -EIO;
		}
		power_supply_put(cp_ply);
	}
	else
	{
		pr_err("don't have /sys/class/power_supply/QC3P dir\n");
	}

	return err;
}

static int wt6670f_i2c_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	int ret = 0;
	u16 firmware_version = 0;
	u16 wt6670f_id = 0;
	int in_isp_flow_count = 3;
	struct wt6670f *wt;
	struct iio_dev *indio_dev;

	pr_info("[%s]\n", __func__);

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*wt));
	if (!indio_dev)
		return -ENOMEM;

	wt = iio_priv(indio_dev);
	if (!wt) {
		pr_err("Out of memory\n");
		return -ENOMEM;
	}
	wt->indio_dev = indio_dev;

	wt->dev = &client->dev;
	wt->client = client;
	i2c_set_clientdata(client, wt);
	mutex_init(&wt->i2c_rw_lock);

	_wt = wt;

	ret = wt6670f_parse_dt(&client->dev);
	if (ret < 0)
		pr_info("%s: parse dt error\n", __func__);

	wt6670f_create_device_node(&(client->dev));

	//wt6670f_do_reset();

	gpio_direction_output(_wt->reset_pin,1);
	usleep_range(5000,6000);
	gpio_direction_output(_wt->reset_pin,0);
	usleep_range(5000,6000);

	wt6670f_id = wt6670f_get_id(0x13);
	if(0x3349 == wt6670f_id){
		g_qc3p_id = QC3P_Z350;
		qc3p_z350_init_ok = true;
		//gpio_direction_output(_wt->reset_pin, 1);
		pr_info("[%s] wt is z350\n", __func__);

		ret = wt6670_init_iio_psy(wt);
		if (ret < 0)
			pr_info("%s: init iio sys error\n", __func__);

		if (gpio_direction_input(wt6670f_int_pin))
		pr_info("[%s] gpio_direction_input wt6670f_int_pin failed", __func__);
		goto probe_out;
	}

	wt6670f_reset_chg_type();
	gpio_direction_output(_wt->intb_pin,0);
	usleep_range(5000,6000);
	gpio_direction_output(_wt->intb_pin,1);
	usleep_range(5000,6000);
	firmware_version = wt6670f_get_firmware_version();

	pr_info("[%s] firmware_version = %d, chg_type = 0x%x\n", __func__,firmware_version, _wt->chg_type);

	if(firmware_version != WT6670_FIRMWARE_VERSION){
		pr_info("[%s]: firmware need upgrade, run wt6670_isp!", __func__);
		while(in_isp_flow_count--){
			ret = wt6670f_isp_flow(wt);
			if(ret == 0)
				break;
		}
	}
	wt6670f_id = wt6670f_get_id(0xBC);
	if(0x5457 == wt6670f_id){
		g_qc3p_id = QC3P_WT6670F;
		pr_info("[%s] is wt6670f,firmware_version = %x\n", __func__,firmware_version);
	}

	ret = wt6670_init_iio_psy(wt);
	if (ret < 0)
		pr_info("%s: init iio sys error\n", __func__);
	if (gpio_direction_input(wt6670f_int_pin))
		pr_info("[%s] gpio_direction_input wt6670f_int_pin failed", __func__);

	ret = wt6670f_psy_register(wt);
	if (ret) {
		pr_err("Failed to register psy\n");
	}
	ret = wt6670f_create_sys(&(client->dev), &wt6670f_attribute_group);
	if(ret){
		pr_err("[BATT]: Err failed to creat BMT attributes\n");
	}

probe_out:
	return 0;
}

static void wt6670f_shutdown(struct i2c_client *client)
{
	int ret = 0;
	u16 data = 0x01;
	struct wt6670f *wt = i2c_get_clientdata(client);

	sysfs_remove_group(&(wt->client->dev.kobj), &wt6670f_attribute_group);
	power_supply_unregister(wt->qc3_psy);
	if(QC3P_WT6670F == g_qc3p_id){
		ret = wt6670f_write_word(_wt, 0xB6, data);
		pr_info("%s set voltage ok wt6670\n", __func__);
	}else if(QC3P_Z350 == g_qc3p_id){
		ret = wt6670f_write_word(_wt, 0x02, data);
		pr_info("%s set voltage ok z350\n", __func__);
	}else{
		ret = wt6670f_write_word(_wt, 0xB6, data);
		pr_info("%s set voltage ok wt6670\n", __func__);
	}
	if (ret < 0)
	{
		pr_info("%s set voltage fail\n",__func__);
	}
}
//#define WT6670F_PM_OPS	(NULL)

static const struct i2c_device_id wt6670f_id_table[] = {
	{"wt6670f", 0},
	{},
};

static const struct of_device_id wt_match_table[] = {
	{.compatible = "mediatek,wt6670f_qc3p",},
	{},
};

static struct i2c_driver wt6670f_driver = {
	.driver = {
		.name = "wt6670f_qc3p",
		.owner = THIS_MODULE,
		.of_match_table = wt_match_table,
		//.pm = WT6670F_PM_OPS,
	},
	.probe = wt6670f_i2c_probe,
	//.remove = wt6670f_i2c_remove,
	.shutdown = wt6670f_shutdown,
	.id_table = wt6670f_id_table,
};

static int __init wt6670f_init(void)
{
	pr_info("[%s] init start with i2c DTS", __func__);
	if (i2c_add_driver(&wt6670f_driver) != 0) {
		pr_info(
			"[%s] failed to register wt6670f i2c driver.\n",__func__);
	} else {
		pr_info(
			"[%s] Success to register wt6670f i2c driver.\n",__func__);
	}
	return 0;
	//return i2c_add_driver(&wt6670f_driver);
}

static void __exit wt6670f_exit(void)
{
	i2c_del_driver(&wt6670f_driver);
}
module_init(wt6670f_init);
module_exit(wt6670f_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Lai Du <dulai@longcheer.com>");
MODULE_DESCRIPTION("WT6670F QC3P Driver");
