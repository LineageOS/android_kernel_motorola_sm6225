// SPDX-License-Identifier: GPL-2.0
#define LOG_TAG         "Test"

#include "cts_config.h"
#include "cts_platform.h"
#include "cts_core.h"

/* ICNL9911/S/C */
#define CTS_FIRMWARE_WORK_MODE_NORMAL   (0x00)
#define CTS_FIRMWARE_WORK_MODE_FACTORY  (0x01)
#define CTS_FIRMWARE_WORK_MODE_CONFIG   (0x02)
#define CTS_FIRMWARE_WORK_MODE_TEST     (0x03)

/* ICNL9916/9922 */
#define CTS_FIRMWARE_WORK_MODE_CFG           (0x01)
#define CTS_FIRMWARE_WORK_MODE_OPEN_SHORT    (0x02)

#define CTS_TEST_SHORT                  (0x01)
#define CTS_TEST_OPEN                   (0x02)

#define CTS_SHORT_TEST_UNDEFINED        (0x00)
#define CTS_SHORT_TEST_BETWEEN_COLS     (0x01)
#define CTS_SHORT_TEST_BETWEEN_ROWS     (0x02)
#define CTS_SHORT_TEST_BETWEEN_GND      (0x03)

#define TEST_RESULT_BUFFER_SIZE(cts_dev) \
	(cts_dev->hwdata->num_row * cts_dev->hwdata->num_col * 2)

#define RAWDATA_BUFFER_SIZE(cts_dev) \
	(cts_dev->hwdata->num_row * cts_dev->hwdata->num_col * 2)

int disable_fw_esd_protection(struct cts_device *cts_dev)
{
	return cts_fw_reg_writeb(cts_dev, 0x8000 + 342, 1);
}

int disable_fw_monitor_mode(struct cts_device *cts_dev)
{
	int ret;
	u8 value;

	ret = cts_fw_reg_readb(cts_dev, 0x8000 + 344, &value);
	if (ret)
		return ret;

	if (value & BIT(0))
		return cts_fw_reg_writeb(cts_dev, 0x8000 + 344,
					 value & (~BIT(0)));

	return 0;
}

int disable_fw_auto_compensate(struct cts_device *cts_dev)
{
	return cts_fw_reg_writeb(cts_dev, 0x8000 + 276, 1);
}

int set_fw_work_mode(const struct cts_device *cts_dev, u8 mode)
{
	int ret, retries;
	u8 pwr_mode;

	cts_info("Set firmware work mode to %u", mode);

	ret = cts_fw_reg_writeb(cts_dev, CTS_DEVICE_FW_REG_WORK_MODE, mode);
	if (ret) {
		cts_err("Write firmware work mode register failed %d", ret);
		return ret;
	}

	ret = cts_fw_reg_readb(cts_dev, 0x05, &pwr_mode);
	if (ret) {
		cts_err("Read firmware power mode register failed %d", ret);
		return ret;
	}

	if (pwr_mode == 1) {
		ret = cts_send_command(cts_dev, CTS_CMD_QUIT_GESTURE_MONITOR);
		if (ret) {
			cts_err("Send cmd QUIT_GESTURE_MONITOR failed %d", ret);
			return ret;
		}

		msleep(50);
	}

	retries = 0;
	do {
		u8 sys_busy, curr_mode;

		msleep(10);

		ret = cts_fw_reg_readb(cts_dev, 0x01, &sys_busy);
		if (ret) {
			cts_err("Read firmware system busy register failed %d",
				ret);
			return ret;
		}
		if (sys_busy)
			continue;

		ret = cts_fw_reg_readb(cts_dev, 0x3F, &curr_mode);
		if (ret) {
			cts_err("Read firmware current work mode failed %d",
				ret);
			return ret;
		}

		if (curr_mode == mode || curr_mode == 0xFF)
			break;
	} while (retries++ < 1000);

	return (retries >= 1000 ? -ETIMEDOUT : 0);
}

int set_display_state(const struct cts_device *cts_dev, bool active)
{
	int ret;
	u8 access_flag;

	cts_info("Set display state to %s", active ? "ACTIVE" : "SLEEP");

	ret = cts_hw_reg_readb_relaxed(cts_dev, 0x3002C, &access_flag);
	if (ret) {
		cts_err("Read display access flag failed %d", ret);
		return ret;
	}

	ret = cts_hw_reg_writeb_relaxed(cts_dev, 0x3002C, access_flag | 0x01);
	if (ret) {
		cts_err("Write display access flag %02x failed %d", access_flag,
			ret);
		return ret;
	}

	if (active) {
		ret = cts_hw_reg_writeb_relaxed(cts_dev, 0x3C044, 0x55);
		if (ret) {
			cts_err("Write DCS-CMD11 fail");
			return ret;
		}

		msleep(100);

		ret = cts_hw_reg_writeb_relaxed(cts_dev, 0x3C0A4, 0x55);
		if (ret) {
			cts_err("Write DCS-CMD29 fail");
			return ret;
		}

		msleep(100);
	} else {
		ret = cts_hw_reg_writeb_relaxed(cts_dev, 0x3C0A0, 0x55);
		if (ret) {
			cts_err("Write DCS-CMD28 fail");
			return ret;
		}

		msleep(100);

		ret = cts_hw_reg_writeb_relaxed(cts_dev, 0x3C040, 0x55);
		if (ret) {
			cts_err("Write DCS-CMD10 fail");
			return ret;
		}

		msleep(100);
	}

	ret = cts_hw_reg_writeb_relaxed(cts_dev, 0x3002C, access_flag);
	if (ret) {
		cts_err("Restore display access flag %02x failed %d",
			access_flag, ret);
		return ret;
	}

	return 0;
}

static int wait_test_complete(struct cts_device *cts_dev, int skip_frames)
{
	int ret, i, j;

	cts_info("Wait test complete skip %d frames", skip_frames);

	if ((cts_dev->hwdata->hwid == CTS_DEV_HWID_ICNL9916) ||
	    (cts_dev->hwdata->hwid == CTS_DEV_HWID_ICNL9922)) {
		return 0;
	}

	for (i = 0; i < (skip_frames + 1); i++) {
		u8 ready;

		for (j = 0; j < 100; j++) {
			mdelay(10);

			ready = 0;
			ret =
			    cts_dev->ops->get_data_ready_flag(cts_dev, &ready);
			if (ret) {
				cts_err("Get data ready flag failed %d", ret);
				return ret;
			}

			if (ready)
				break;
		}

		if (ready == 0)
			return -ETIMEDOUT;

		if (i < skip_frames) {
			ret = cts_dev->ops->clr_data_ready_flag(cts_dev);
			if (ret) {
				cts_err("Clr data ready flag failed %d", ret);
				return ret;
			}
		}
	}

	return 0;
}

struct cts_fw_short_test_param {
	u8 type;
	u32 col_pattern[2];
	u32 row_pattern[2];
};

bool set_short_test_type(struct cts_device *cts_dev, u8 type)
{
	static struct cts_fw_short_test_param param = {
		.type = CTS_SHORT_TEST_BETWEEN_COLS,
		.col_pattern = { 0, 0 },
		.row_pattern = { 0, 0 }
	};
	int i, ret;

	cts_info("Set short test type to %u", type);

	param.type = type;
	for (i = 0; i < 5; i++) {
		u8 type_readback;

		ret =
		    cts_fw_reg_writesb(cts_dev, 0x5000, &param, sizeof(param));
		if (ret) {
			cts_err("Set short test type to %u failed %d", type,
				ret);
			continue;
		}
		ret = cts_fw_reg_readb(cts_dev, 0x5000, &type_readback);
		if (ret) {
			cts_err("Get short test type failed %d", ret);
			continue;
		}
		if (type == type_readback)
			return 0;
		else {
			cts_err("Set test type %u != readback %u", type,
				type_readback);
			continue;
		}
	}

	return ret;
}

static void dump_test_data(struct cts_device *cts_dev,
			   const char *desc, const u16 *data)
{
#define SPLIT_LINE_STR \
	"--------------------------------------------------------"\
	"--------------------------------------------------------"
#define ROW_NUM_FORMAT_STR  "%2d | "
#define COL_NUM_FORMAT_STR  "%-5u "
#define DATA_FORMAT_STR     "%-5u "

	int r, c;
	u32 max, min, sum, average;
	int max_r, max_c, min_r, min_c;
	char line_buf[128];
	int count = 0;

	max = min = data[0];
	sum = 0;
	max_r = max_c = min_r = min_c = 0;
	for (r = 0; r < cts_dev->fwdata.rows; r++) {
		for (c = 0; c < cts_dev->fwdata.cols; c++) {
			u16 val = data[r * cts_dev->hwdata->num_col + c];

			sum += val;
			if (val > max) {
				max = val;
				max_r = r;
				max_c = c;
			} else if (val < min) {
				min = val;
				min_r = r;
				min_c = c;
			}
		}
	}
	average = sum / (cts_dev->fwdata.rows * cts_dev->fwdata.cols);

	cts_info(SPLIT_LINE_STR);
	count += snprintf(line_buf + count, sizeof(line_buf) - count,
			  " %s test data MIN: [%u][%u]=%u, MAX: [%u][%u]=%u, AVG=%u",
			  desc, min_r, min_c, min, max_r, max_c, max, average);
	cts_info("%s", line_buf);
	cts_info(SPLIT_LINE_STR);

	count = 0;
	count += snprintf(line_buf + count, sizeof(line_buf) - count, "   | ");
	for (c = 0; c < cts_dev->fwdata.cols; c++) {
		count += snprintf(line_buf + count, sizeof(line_buf) - count,
				  COL_NUM_FORMAT_STR, c);
	}
	cts_info("%s", line_buf);
	cts_info(SPLIT_LINE_STR);

	for (r = 0; r < cts_dev->fwdata.rows; r++) {
		count = 0;
		count += snprintf(line_buf + count, sizeof(line_buf) - count,
				  ROW_NUM_FORMAT_STR, r);
		for (c = 0; c < cts_dev->fwdata.cols; c++) {
			count +=
			    snprintf(line_buf + count, sizeof(line_buf) - count,
				     DATA_FORMAT_STR,
				     data[r * cts_dev->hwdata->num_col + c]);
		}
		cts_info("%s", line_buf);
	}
	cts_info(SPLIT_LINE_STR);

#undef SPLIT_LINE_STR
#undef ROW_NUM_FORMAT_STR
#undef COL_NUM_FORMAT_STR
#undef DATA_FORMAT_STR
}

/* Return number of failed nodes */
static int validate_test_result(struct cts_device *cts_dev,
				const char *desc, u16 *test_result, u16 min,
				u16 max)
{
#define SPLIT_LINE_STR \
	"------------------------------"

	int r, c;
	int failed_cnt = 0;

	for (r = 0; r < cts_dev->fwdata.rows; r++) {
		for (c = 0; c < cts_dev->fwdata.cols; c++) {
			if (test_result[r * cts_dev->hwdata->num_col + c] < min
			    || test_result[r * cts_dev->hwdata->num_col + c] >
			    max) {
				if (failed_cnt == 0) {
					cts_info(SPLIT_LINE_STR);
					cts_info("%s failed nodes:", desc);
				}
				failed_cnt++;

				cts_info("  %3d: [%-2d][%-2d] = %u", failed_cnt,
					 r, c,
					 test_result[r *
						     cts_dev->hwdata->num_col +
						     c]);
			}
		}
	}

	if (failed_cnt)
		cts_info(SPLIT_LINE_STR);

	cts_info("%s test %d node total failed", desc, failed_cnt);

	return failed_cnt;

#undef SPLIT_LINE_STR
}

static int test_short_to_gnd(struct cts_device *cts_dev,
			     u8 feature_ver, u16 *test_result, u16 threshold)
{
	int ret;

	cts_info("Test short to GND");

	ret =
	    cts_dev->ops->set_short_test_type(cts_dev,
					      CTS_SHORT_TEST_BETWEEN_GND);
	if (ret) {
		cts_err("Set short test type to SHORT_TO_GND failed %d", ret);
		return ret;
	}

	ret = cts_dev->ops->set_openshort_mode(cts_dev, CTS_TEST_SHORT);
	if (ret) {
		cts_err("Set test type to SHORT failed %d", ret);
		return ret;
	}

	ret =
	    cts_dev->ops->set_workmode(cts_dev,
				       CTS_FIRMWARE_WORK_MODE_OPEN_SHORT);
	if (ret) {
		cts_err("Set firmware work mode to WORK_MODE_TEST failed %d",
			ret);
		return ret;
	}

	ret = wait_test_complete(cts_dev, 0);
	if (ret) {
		cts_err("Wait test complete failed %d", ret);
		return ret;
	}

	ret =
	    cts_polling_test_data(cts_dev, (u8 *) test_result,
				  RAWDATA_BUFFER_SIZE(cts_dev));
	if (ret) {
		cts_err("Read test result failed %d", ret);
		return ret;
	}

	dump_test_data(cts_dev, "GND-short", test_result);

	return validate_test_result(cts_dev, "GND-short", test_result,
				    threshold, USHRT_MAX);
}

#define SHORT_COLS_TEST_LOOP                        3

static int test_short_between_cols(struct cts_device *cts_dev,
				   u16 *test_result, u16 threshold,
				   bool skip_first_frame)
{
	int ret;
	int loop = 0;

	cts_info("Test short between columns");

	ret =
	    cts_dev->ops->set_short_test_type(cts_dev,
					      CTS_SHORT_TEST_BETWEEN_COLS);
	if (ret) {
		cts_err("Set short test type to BETWEEN_COLS failed %d", ret);
		return ret;
	}

	ret = cts_dev->ops->set_openshort_mode(cts_dev, CTS_TEST_SHORT);
	if (ret) {
		cts_err("Set test type to SHORT failed %d", ret);
		return ret;
	}

	ret =
	    cts_dev->ops->set_workmode(cts_dev,
				       CTS_FIRMWARE_WORK_MODE_OPEN_SHORT);
	if (ret) {
		cts_err("Set firmware work mode to WORK_MODE_TEST failed %d",
			ret);
		return ret;
	}

	for (loop = 0; loop < SHORT_COLS_TEST_LOOP; loop++) {
		ret = wait_test_complete(cts_dev, 0);
		if (ret) {
			cts_err("Wait test complete failed %d", ret);
			return ret;
		}

		ret =
		    cts_polling_test_data(cts_dev, test_result,
					  RAWDATA_BUFFER_SIZE(cts_dev));
		if (ret) {
			cts_err("Read test result failed %d", ret);
			return ret;
		}

		dump_test_data(cts_dev, "Col-short", test_result);

		ret = validate_test_result(cts_dev, "Col-short", test_result,
					   threshold, USHRT_MAX);
		if (ret != 0)
			break;
	}

	return ret;
}

#define SHORT_ROWS_TEST_LOOP            6
static int test_short_between_rows(struct cts_device *cts_dev,
				   u16 *test_result, u16 threshold)
{
	int ret;
	int loopcnt;

	cts_info("Test short between rows");

	for (loopcnt = 0; loopcnt < SHORT_ROWS_TEST_LOOP; loopcnt++) {
		ret =
		    cts_dev->ops->set_short_test_type(cts_dev,
						      CTS_SHORT_TEST_BETWEEN_ROWS);
		if (ret) {
			cts_err("Set short test type to BETWEEN_ROWS failed %d",
				ret);
			return ret;
		}

		ret = cts_dev->ops->set_openshort_mode(cts_dev, CTS_TEST_SHORT);
		if (ret) {
			cts_err("Set test type to SHORT failed %d", ret);
			return ret;
		}

		ret =
		    cts_dev->ops->set_workmode(cts_dev,
					       CTS_FIRMWARE_WORK_MODE_OPEN_SHORT);
		if (ret) {
			cts_err
			    ("Set firmware work mode to WORK_MODE_TEST failed %d",
			     ret);
			return ret;
		}

		ret = wait_test_complete(cts_dev, 0);
		if (ret) {
			cts_err("Wait test complete failed %d", ret);
			return ret;
		}

		ret =
		    cts_polling_test_data(cts_dev, test_result,
					  RAWDATA_BUFFER_SIZE(cts_dev));
		if (ret) {
			cts_err("Read test result failed %d", ret);
			return ret;
		}

		dump_test_data(cts_dev, "Row-short", test_result);

		ret = validate_test_result(cts_dev, "Row-short", test_result,
					   threshold, USHRT_MAX);
		if (ret != 0)
			break;
	}

	return ret;
}

static int wait_fw_to_normal_work(struct cts_device *cts_dev)
{
	int i = 0;
	int ret;

	cts_info("Wait fw to normal work");

	do {
		u8 work_mode;

		ret = cts_dev->ops->get_workmode(cts_dev, &work_mode);
		if (ret) {
			cts_err("Get fw curr work mode failed %d", work_mode);
			continue;
		} else {
			if (work_mode == CTS_FIRMWARE_WORK_MODE_NORMAL)
				return 0;
		}

		mdelay(10);
	} while (++i < 100);

	return ret ? ret : -ETIMEDOUT;
}

static int prepare_test(struct cts_device *cts_dev)
{
	int ret;

	cts_info("Prepare test");

	cts_reset_device(cts_dev);

	ret = wait_fw_to_normal_work(cts_dev);
	if (ret) {
		cts_err("Wait fw to normal work failed %d", ret);
		return ret;
	}

	ret = cts_dev->ops->set_esd_enable(cts_dev, 0x00);
	if (ret) {
		cts_err("Disable firmware ESD protection failed %d", ret);
		return ret;
	}

	ret = cts_dev->ops->set_mnt_enable(cts_dev, 0x00);
	if (ret) {
		cts_err("Disable firmware monitor mode failed %d", ret);
		return ret;
	}

	ret = cts_dev->ops->set_cneg_enable(cts_dev, 0x00);
	if (ret) {
		cts_err("Disable firmware auto compensate failed %d", ret);
		return ret;
	}

	if ((cts_dev->hwdata->hwid == CTS_DEV_HWID_ICNL9916) ||
	    (cts_dev->hwdata->hwid == CTS_DEV_HWID_ICNL9922))
		ret =
		    cts_dev->ops->set_workmode(cts_dev,
					       CTS_FIRMWARE_WORK_MODE_CFG);
	else
		ret =
		    cts_dev->ops->set_workmode(cts_dev,
					       CTS_FIRMWARE_WORK_MODE_CONFIG);

	if (ret) {
		cts_err("Set firmware work mode to WORK_MODE_CONFIG failed %d",
			ret);
		return ret;
	}

	cts_dev->rtdata.testing = true;

	return 0;
}

static void post_test(struct cts_device *cts_dev)
{
	int ret;

	cts_info("Post test");

	cts_reset_device(cts_dev);

	ret =
	    cts_dev->ops->set_workmode(cts_dev, CTS_FIRMWARE_WORK_MODE_NORMAL);
	if (ret)
		cts_err("Set firmware work mode to WORK_MODE_NORMAL failed %d",
			ret);

	ret = wait_fw_to_normal_work(cts_dev);
	if (ret)
		cts_err("Wait fw to normal work failed %d", ret);

	cts_dev->rtdata.testing = false;
}

/* Return 0 success
 *  negative value while error occurs
 *  positive value means how many nodes fail
 */
int cts_short_test(struct cts_device *cts_dev, u16 threshold)
{
	int ret;
	u16 *test_result = NULL;
	bool recovery_display_state = false;
	u8 need_display_on;
	u8 feature_ver = 0;
	u8 old_int_data_method = cts_dev->fwdata.int_data_method;
	u16 old_int_data_types = cts_dev->fwdata.int_data_types;

	cts_info("Start short test, threshold = %u", threshold);

	test_result =
	    (u16 *) kmalloc(TEST_RESULT_BUFFER_SIZE(cts_dev), GFP_KERNEL);
	if (test_result == NULL) {
		cts_err("Allocate test result buffer failed");
		return -ENOMEM;
	}

	ret = cts_stop_device(cts_dev);
	if (ret) {
		cts_err("Stop device failed %d", ret);
		return ret;
	}

	cts_lock_device(cts_dev);
	ret = prepare_test(cts_dev);
	if (ret) {
		cts_err("Prepare test failed %d", ret);
		goto err_free_test_result;
	}

	ret = cts_dev->ops->is_display_on(cts_dev, &need_display_on);
	if (ret) {
		cts_err("Read need display on register failed %d", ret);
		goto err_free_test_result;
	}

	if (need_display_on == 0) {
		ret = cts_dev->ops->set_display_on(cts_dev, 0x00);
		if (ret) {
			cts_err("Set display state to SLEEP failed %d", ret);
			goto err_free_test_result;
		}
		recovery_display_state = true;
	}

	cts_set_int_data_types(cts_dev, INT_DATA_TYPE_RAWDATA);
	cts_set_int_data_method(cts_dev, INT_DATA_METHOD_POLLING);

	ret = test_short_to_gnd(cts_dev, feature_ver, test_result, threshold);
	if (ret) {
		cts_err("Short to GND test failed %d", ret);
		goto err_recovery_display_state;
	}

	ret = test_short_between_cols(cts_dev, test_result, threshold,
				      need_display_on == 0);
	if (ret) {
		cts_err("Short between columns test failed %d", ret);
		goto err_recovery_display_state;
	}

	ret = test_short_between_rows(cts_dev, test_result, threshold);
	if (ret) {
		cts_err("Short between rows test failed %d", ret);
		goto err_recovery_display_state;
	}

err_recovery_display_state:
	cts_set_int_data_method(cts_dev, old_int_data_method);
	cts_set_int_data_types(cts_dev, old_int_data_types);

	if (recovery_display_state) {
		int r = cts_dev->ops->set_display_on(cts_dev, 0x01);

		if (r)
			cts_err("Set display state to ACTIVE failed %d", r);
	}
err_free_test_result:
	kfree(test_result);
	post_test(cts_dev);

	cts_unlock_device(cts_dev);

	cts_info("Short test %s", ret ? "FAILED" : "PASSED");

	cts_start_device(cts_dev);

	return ret;
}

/* Return 0 success
 *  negative value while error occurs
 *  positive value means how many nodes fail
 */
int cts_open_test(struct cts_device *cts_dev, u16 threshold)
{
	int ret;
	u16 *test_result = NULL;
	bool recovery_display_state = false;
	u8 need_display_on;
	u8 old_int_data_method = cts_dev->fwdata.int_data_method;
	u16 old_int_data_types = cts_dev->fwdata.int_data_types;

	cts_info("Start open test, threshold = %u", threshold);

	test_result =
	    (u16 *) kmalloc(TEST_RESULT_BUFFER_SIZE(cts_dev), GFP_KERNEL);
	if (test_result == NULL) {
		cts_err("Allocate memory for test result faild");
		return -ENOMEM;
	}

	ret = cts_stop_device(cts_dev);
	if (ret) {
		cts_err("Stop device failed %d", ret);
		return ret;
	}

	cts_lock_device(cts_dev);
	ret = prepare_test(cts_dev);
	if (ret) {
		cts_err("Prepare test failed %d", ret);
		goto err_free_test_result;
	}

	ret = cts_dev->ops->is_display_on(cts_dev, &need_display_on);
	if (ret) {
		cts_err("Read need display on register failed %d", ret);
		goto err_free_test_result;
	}

	if (need_display_on == 0) {
		ret = cts_dev->ops->set_display_on(cts_dev, 0x00);
		if (ret) {
			cts_err("Set display state to SLEEP failed %d", ret);
			goto err_free_test_result;
		}
		recovery_display_state = true;
	}

	cts_set_int_data_types(cts_dev, INT_DATA_TYPE_RAWDATA);
	cts_set_int_data_method(cts_dev, INT_DATA_METHOD_POLLING);

	if ((cts_dev->hwdata->hwid == CTS_DEV_HWID_ICNL9911C) ||
	    (cts_dev->hwdata->hwid == CTS_DEV_HWID_ICNL9911S) ||
	    (cts_dev->hwdata->hwid == CTS_DEV_HWID_ICNL9911)) {
		ret = cts_send_command(cts_dev, CTS_CMD_RECOVERY_TX_VOL);
		if (ret) {
			cts_err("Recovery tx voltage failed %d", ret);
			goto err_recovery_display_state;
		}
	}

	ret = cts_dev->ops->set_openshort_mode(cts_dev, CTS_TEST_OPEN);
	if (ret) {
		cts_err("Set test type to OPEN_TEST failed %d", ret);
		goto err_recovery_display_state;
	}

	ret =
	    cts_dev->ops->set_workmode(cts_dev,
				       CTS_FIRMWARE_WORK_MODE_OPEN_SHORT);
	if (ret) {
		cts_err("Set firmware work mode to WORK_MODE_TEST failed %d",
			ret);
		goto err_recovery_display_state;
	}

	ret = wait_test_complete(cts_dev, 20);
	if (ret) {
		cts_err("Wait test complete failed %d", ret);
		goto err_recovery_display_state;
	}

	ret =
	    cts_polling_test_data(cts_dev, test_result,
				  RAWDATA_BUFFER_SIZE(cts_dev));
	if (ret) {
		cts_err("Read test result failed %d", ret);
		goto err_recovery_display_state;
	}

	dump_test_data(cts_dev, "Open-circuit", test_result);

	ret = validate_test_result(cts_dev, "Open-circuit", test_result,
				   threshold, USHRT_MAX);

err_recovery_display_state:
	cts_set_int_data_method(cts_dev, old_int_data_method);
	cts_set_int_data_types(cts_dev, old_int_data_types);

	if (recovery_display_state) {
		int r = cts_dev->ops->set_display_on(cts_dev, 0x01);

		if (r)
			cts_err("Set display state to ACTIVE failed %d", r);
	}
err_free_test_result:
	kfree(test_result);
	post_test(cts_dev);

	cts_unlock_device(cts_dev);
	cts_info("Open test %s", ret ? "FAILED" : "PASSED");

	cts_start_device(cts_dev);

	return ret;
}

#ifdef CFG_CTS_HAS_RESET_PIN
int cts_reset_test(struct cts_device *cts_dev)
{
	int ret;
	int val = 0;

	cts_info("Reset Pin");

	ret = cts_stop_device(cts_dev);
	if (ret) {
		cts_err("Stop device failed %d", ret);
		return ret;
	}

	cts_lock_device(cts_dev);

	cts_plat_set_reset(cts_dev->pdata, 0);
	mdelay(50);
#ifdef CONFIG_CTS_I2C_HOST
	/* Check whether device is in normal mode */
	if (!cts_plat_is_i2c_online
	    (cts_dev->pdata, CTS_DEV_NORMAL_MODE_I2CADDR)) {
#else
	if (!cts_plat_is_normal_mode(cts_dev->pdata))
#endif /* CONFIG_CTS_I2C_HOST */
		val++;
	else
		cts_err("Device is alive while reset is low");
	cts_plat_set_reset(cts_dev->pdata, 1);
	mdelay(50);

	ret = wait_fw_to_normal_work(cts_dev);
	if (ret)
		cts_err("Wait fw to normal work failed %d", ret);
#ifdef CONFIG_CTS_I2C_HOST
	/* Check whether device is in normal mode */
	if (cts_plat_is_i2c_online(cts_dev->pdata, CTS_DEV_NORMAL_MODE_I2CADDR)) {
#else
	if (cts_plat_is_normal_mode(cts_dev->pdata))
#endif /* CONFIG_CTS_I2C_HOST */
		val++;
	else
		cts_err("Device is offline while reset is high");
#ifdef CONFIG_CTS_CHARGER_DETECT
	if (cts_is_charger_exist(cts_dev)) {
		int r = cts_set_dev_charger_attached(cts_dev, true);

		if (r)
			cts_err("Set dev charger attached failed %d", r);
	}
#endif /* CONFIG_CTS_CHARGER_DETECT */

#ifdef CONFIG_CTS_GLOVE
	if (cts_is_glove_enabled(cts_dev))
		cts_enter_glove_mode(cts_dev);
#endif

#ifdef CFG_CTS_FW_LOG_REDIRECT
	if (cts_is_fw_log_redirect(cts_dev))
		cts_enable_fw_log_redirect(cts_dev);
#endif

	cts_unlock_device(cts_dev);

	ret = cts_start_device(cts_dev);
	if (ret)
		cts_err("Start device failed %d", ret);

	cts_info("Reset-Pin test %s", val == 2 ? "PASS" : "FAIL");
	if (val == 2) {
		if (!cts_dev->rtdata.program_mode)
			cts_set_normal_addr(cts_dev);
		return 0;
	}

	return -EFAULT;
}
#endif

int cts_test_int_pin(struct cts_device *cts_dev)
{
	int ret;

	cts_info("INT-pin");

	ret = cts_stop_device(cts_dev);
	if (ret) {
		cts_err("Stop device failed %d", ret);
		return ret;
	}

	cts_lock_device(cts_dev);

	ret = cts_send_command(cts_dev, CTS_CMD_WRTITE_INT_HIGH);
	if (ret) {
		cts_err("Send command WRTITE_INT_HIGH failed %d", ret);
		goto unlock_device;
	}
	mdelay(10);
	if (cts_plat_get_int_pin(cts_dev->pdata) == 0) {
		cts_err("INT pin state != HIGH");
		ret = -EFAULT;
		goto unlock_device;
	}

	ret = cts_send_command(cts_dev, CTS_CMD_WRTITE_INT_LOW);
	if (ret) {
		cts_err("Send command WRTITE_INT_LOW failed %d", ret);
		goto unlock_device;
	}
	mdelay(10);
	if (cts_plat_get_int_pin(cts_dev->pdata) != 0) {
		cts_err("INT pin state != LOW");
		ret = -EFAULT;
		goto unlock_device;
	}

	ret = cts_send_command(cts_dev, CTS_CMD_RELASE_INT_TEST);
	if (ret) {
		cts_err("Send command RELASE_INT_TEST failed %d", ret);
		ret = 0;
	}
	mdelay(10);

unlock_device:
	cts_unlock_device(cts_dev);

	ret = cts_start_device(cts_dev);
	if (ret) {
		cts_err("Start device failed %d", ret);
		ret = 0;
	}

	cts_info("Int-Pin test %s", ret == 0 ? "PASS" : "FAIL");
	return ret;
}

void dump_compensate_cap(struct cts_device *cts_dev, u8 *cap)
{
#define SPLIT_LINE_STR \
	"--------------------------------------"\
	"---------------------------------------"
#define ROW_NUM_FORMAT_STR  "%2d | "
#define COL_NUM_FORMAT_STR  "%3u "
#define DATA_FORMAT_STR     "%4d"

	int r, c;
	char line_buf[128];
	int count = 0;

	cts_info(SPLIT_LINE_STR);
	count += snprintf(line_buf + count, sizeof(line_buf) - count, "      ");
	for (c = 0; c < cts_dev->fwdata.cols; c++) {
		count += snprintf(line_buf + count, sizeof(line_buf) - count,
				  COL_NUM_FORMAT_STR, c);
	}
	cts_info("%s", line_buf);
	cts_info(SPLIT_LINE_STR);

	for (r = 0; r < cts_dev->fwdata.rows; r++) {
		count = 0;
		count += snprintf(line_buf + count, sizeof(line_buf) - count,
				  ROW_NUM_FORMAT_STR, r);
		for (c = 0; c < cts_dev->fwdata.cols; c++) {
			count +=
			    snprintf(line_buf + count, sizeof(line_buf) - count,
				     DATA_FORMAT_STR,
				     cap[r * cts_dev->hwdata->num_col + c]);
		}
		cts_info("%s", line_buf);
	}
	cts_info(SPLIT_LINE_STR);
#undef SPLIT_LINE_STR
#undef ROW_NUM_FORMAT_STR
#undef COL_NUM_FORMAT_STR
#undef DATA_FORMAT_STR
}

static int validate_compensate_cap(struct cts_device *cts_dev,
				   const char *desc, u8 *test_result, u16 min,
				   u16 max)
{
#define SPLIT_LINE_STR \
	"------------------------------"

	int r, c;
	int failed_cnt = 0;

	for (r = 0; r < cts_dev->fwdata.rows; r++) {
		for (c = 0; c < cts_dev->fwdata.cols; c++) {
			if (test_result[r * cts_dev->hwdata->num_col + c] < min
			    || test_result[r * cts_dev->hwdata->num_col + c] >
			    max) {
				if (failed_cnt == 0) {
					cts_info(SPLIT_LINE_STR);
					cts_info("%s failed nodes:", desc);
				}
				failed_cnt++;

				cts_info("  %3d: [%-2d][%-2d] = %u", failed_cnt,
					 r, c,
					 test_result[r *
						     cts_dev->hwdata->num_col +
						     c]);
			}
		}
	}

	if (failed_cnt)
		cts_info("%s", SPLIT_LINE_STR);

	cts_info("%s test %d node total failed", desc, failed_cnt);

	return failed_cnt;

#undef SPLIT_LINE_STR
}

int cts_compensate_cap_test(struct cts_device *cts_dev, u8 min_thres,
			    u8 max_thres)
{
	u8 *cap = NULL;
	int ret = 0;

	cts_info("Compensate cap, threshold [%u, %u]", min_thres, max_thres);

	cap = kzalloc(cts_dev->hwdata->num_row * cts_dev->hwdata->num_col,
		      GFP_KERNEL);
	if (cap == NULL) {
		cts_err("Allocate mem for cap failed %d", ret);
		return -ENOMEM;
	}

	cts_lock_device(cts_dev);
	ret = cts_dev->ops->top_get_cnegdata(cts_dev, cap,
					     cts_dev->hwdata->num_row *
					     cts_dev->hwdata->num_col);
	cts_unlock_device(cts_dev);
	if (ret) {
		cts_err("Get compensate cap failed %d", ret);
		goto free_cap;
	}

	dump_compensate_cap(cts_dev, cap);
	ret = validate_compensate_cap(cts_dev, "Compensate-Cap", cap,
				      min_thres, max_thres);

free_cap:
	kfree(cap);

	cts_info("Compensate-Cap test %s", ret == 0 ? "PASS" : "FAIL");
	return ret;
}

int cts_rawdata_test(struct cts_device *cts_dev, u16 min, u16 max)
{
	int ret;
	bool data_valid = false;
	u16 *rawdata;
	int i;

	cts_info("Rawdata, threshold[%u, %u]", min, max);

	rawdata = (u16 *) kmalloc(RAWDATA_BUFFER_SIZE(cts_dev), GFP_KERNEL);
	if (rawdata == NULL) {
		cts_err("Allocate memory for rawdata failed");
		return -ENOMEM;
	}

	cts_lock_device(cts_dev);

	if ((cts_dev->hwdata->hwid == CTS_DEV_HWID_ICNL9916) ||
	    (cts_dev->hwdata->hwid == CTS_DEV_HWID_ICNL9922)) {
		ret =
		    cts_dev->ops->top_get_rawdata(cts_dev, (u8 *) rawdata,
						  RAWDATA_BUFFER_SIZE(cts_dev));
		if (ret < 0) {
			cts_err("Get raw data failed: %d", ret);
			data_valid = false;
			goto unlock_device;
		}
		data_valid = true;
		goto unlock_device;
	}

	for (i = 0; i < 5; i++) {
		int r;
		u8 val;

		r = cts_dev->ops->enable_get_rawdata(cts_dev);
		if (r) {
			cts_err("Enable get ts data failed %d", r);
			continue;
		}
		mdelay(1);
		r = cts_dev->ops->is_enabled_get_rawdata(cts_dev, &val);
		if (r) {
			cts_err("Read enable get ts data failed %d", r);
			continue;
		}
		if (val != 0)
			break;
	}

	if (i >= 5) {
		cts_err("Enable read tsdata failed");
		ret = -EIO;
		goto unlock_device;
	}

	msleep(50);
	for (i = 0; i < 3; i++) {
		ret = cts_get_rawdata(cts_dev, rawdata);
		if (ret) {
			cts_err("Get rawdata failed %d", ret);
			continue;
		} else {
			data_valid = true;
			break;
		}
		mdelay(30);
	}

	if (i >= 3) {
		cts_err("Read rawdata failed");
		ret = -EIO;
	}

	for (i = 0; i < 5; i++) {
		int r = cts_dev->ops->disable_get_rawdata(cts_dev);

		if (r) {
			cts_err("Disable get rawdata failed %d", r);
			continue;
		} else
			break;
	}

unlock_device:
	cts_unlock_device(cts_dev);

	if (data_valid) {
		dump_test_data(cts_dev, "Rawdata", rawdata);
		ret =
		    validate_test_result(cts_dev, "Rawdata", rawdata, min, max);
	}
	kfree(rawdata);

	cts_info("Rawdata test %s",
		 (data_valid && (ret == 0)) ? "PASS" : "FAIL");

	return ret;
}

int cts_noise_test(struct cts_device *cts_dev, u32 frames, u16 max)
{
	u16 *curr_rawdata = NULL;
	u16 *max_rawdata = NULL;
	u16 *min_rawdata = NULL;
	u16 *noise = NULL;
	bool first_frame = true;
	bool data_valid = false;
	u16 i;
	int ret;

	cts_info("Noise frames: %u, threshold: %u", frames, max);

	curr_rawdata =
	    (u16 *) kmalloc(TEST_RESULT_BUFFER_SIZE(cts_dev) * 4, GFP_KERNEL);
	if (curr_rawdata == NULL) {
		cts_err("Alloc mem for rawdata failed");
		return -ENOMEM;
	}
	max_rawdata =
	    curr_rawdata + cts_dev->fwdata.rows * cts_dev->fwdata.cols;
	min_rawdata = max_rawdata + cts_dev->fwdata.rows * cts_dev->fwdata.cols;
	noise = min_rawdata + cts_dev->fwdata.rows * cts_dev->fwdata.cols;

	cts_lock_device(cts_dev);

	for (i = 0; i < 5; i++) {
		int r;
		u8 val;

		r = cts_dev->ops->enable_get_rawdata(cts_dev);
		if (r) {
			cts_err("Enable get ts data failed %d", r);
			continue;
		}
		mdelay(1);
		r = cts_dev->ops->is_enabled_get_rawdata(cts_dev, &val);
		if (r) {
			cts_err("Read enable get ts data failed %d", r);
			continue;
		}
		if (val != 0)
			break;
	}

	if (i >= 5) {
		cts_err("Enable read tsdata failed");
		ret = -EIO;
		goto unlock_device;
	}

	msleep(50);

	while (frames--) {
		for (i = 0; i < 3; i++) {
			int r;

			r = cts_get_rawdata(cts_dev, curr_rawdata);
			if (r) {
				cts_err("Get rawdata failed %d", r);
				continue;
			} else
				break;

			mdelay(30);
		}

		if (i >= 3) {
			cts_err("Read rawdata failed");
			ret = -EIO;
			goto disable_get_tsdata;
		}

		/* Dump raw data */
		dump_test_data(cts_dev, "Noise-rawdata", curr_rawdata);

		if (unlikely(first_frame)) {
			memcpy(max_rawdata, curr_rawdata,
			       TEST_RESULT_BUFFER_SIZE(cts_dev));
			memcpy(min_rawdata, curr_rawdata,
			       TEST_RESULT_BUFFER_SIZE(cts_dev));
			first_frame = false;

			continue;
		} else {
			for (i = 0;
			     i < cts_dev->fwdata.rows * cts_dev->fwdata.cols;
			     i++) {
				if (curr_rawdata[i] > max_rawdata[i])
					max_rawdata[i] = curr_rawdata[i];
				else if (curr_rawdata[i] < min_rawdata[i])
					min_rawdata[i] = curr_rawdata[i];
			}
		}
	}

	data_valid = true;

disable_get_tsdata:
	for (i = 0; i < 5; i++) {
		int r = cts_dev->ops->disable_get_rawdata(cts_dev);

		if (r) {
			cts_err("Disable get rawdata failed %d", r);
			continue;
		} else
			break;
	}

unlock_device:
	cts_unlock_device(cts_dev);

	if (data_valid) {
		for (i = 0; i < cts_dev->fwdata.rows * cts_dev->fwdata.cols;
		     i++) {
			noise[i] = max_rawdata[i] - min_rawdata[i];
		}

		dump_test_data(cts_dev, "Noise", noise);

		ret =
		    validate_test_result(cts_dev, "Noise test", noise, 0, max);
	}

	kfree(curr_rawdata);

	cts_info("Noise test %s", (data_valid && (ret == 0)) ? "PASS" : "FAIL");

	return ret;
}
