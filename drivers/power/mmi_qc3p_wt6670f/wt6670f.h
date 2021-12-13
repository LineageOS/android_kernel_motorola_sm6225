#ifndef  __QC3P_WT6670_
#define __QC3P_WT6670_
#include <linux/iio/iio.h>
#include <dt-bindings/iio/qti_power_supply_iio.h>
#include <linux/iio/consumer.h>

#define WT6670_FIRMWARE_VERSION  0x03

struct wt6670f {
	struct device *dev;
	struct i2c_client *client;

	int i2c_scl_pin;
	int i2c_sda_pin;
	int reset_pin;
	int intb_pin;

	int count;
	int chg_type;
	bool chg_ready;
	struct iio_dev		*indio_dev;
	struct iio_chan_spec	*iio_chan;
	struct iio_channel	*int_iio_chans;

	struct mutex i2c_rw_lock;
};

enum {
	QC3P_WT6670F,
	QC3P_Z350,
};

enum wt6670_charger_type{
	WT6670_CHG_TYPE_BEGIN = 0,
	WT6670_CHG_TYPE_FC,
	WT6670_CHG_TYPE_SDP,
	WT6670_CHG_TYPE_CDP,
	WT6670_CHG_TYPE_DCP,
	WT6670_CHG_TYPE_QC2,
	WT6670_CHG_TYPE_QC3,
	WT6670_CHG_TYPE_OCP,
	WT6670_CHG_TYPE_QC3P_18W,
	WT6670_CHG_TYPE_QC3P_27W,
	WT6670_CHG_TYPE_UNKNOWN,
};

struct wt6670_iio_channels {
	const char *datasheet_name;
	int channel_num;
	enum iio_chan_type type;
	long info_mask;
};

#define wt6670_IIO_CHAN(_name, _num, _type, _mask)		\
	{						\
		.datasheet_name = _name,		\
		.channel_num = _num,			\
		.type = _type,				\
		.info_mask = _mask,			\
	},

#define wt6670_CHAN_INDEX(_name, _num)			\
	wt6670_IIO_CHAN(_name, _num, IIO_INDEX,		\
		BIT(IIO_CHAN_INFO_PROCESSED))

static const struct wt6670_iio_channels wt6670_iio_psy_channels[] = {
	wt6670_CHAN_INDEX("wt6670_usb_real_type", PSY_IIO_QC3P_REAL_TYPE)
	wt6670_CHAN_INDEX("wt6670_usb_qc3p_power", PSY_IIO_QC3P_POWER)
	wt6670_CHAN_INDEX("wt6670_battery_dp_dm", PSY_IIO_BATTERY_DP_DM)
	wt6670_CHAN_INDEX("wt6670_start_detection", PSY_IIO_START_DETECTION)
	wt6670_CHAN_INDEX("wt6670_detection_ready", PSY_IIO_DETECTION_READY)
};

int wt6670f_isp_flow(struct wt6670f *chip);
int wt6670f_get_firmware_version(void);
#endif //__QC3P_WT6670_
