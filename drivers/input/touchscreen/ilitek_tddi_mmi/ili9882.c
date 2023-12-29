/*
 * ILITEK Touch IC driver
 *
 * Copyright (C) 2011 ILI Technology Corporation.
 *
 * Author: Dicky Chiang <dicky_chiang@ilitek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */
#include "firmware/ili9882_fw.h"
#include "ili9882.h"

/* Debug level */
bool debug_en = DEBUG_OUTPUT;
EXPORT_SYMBOL(debug_en);

static struct workqueue_struct *esd_wq;
static struct workqueue_struct *bat_wq;
static struct delayed_work esd_work;
static struct delayed_work bat_work;

#if RESUME_BY_DDI
static int ili_pinctrl_select_normal(struct ilitek_ts_data *ts);
static struct workqueue_struct	*resume_by_ddi_wq;
static struct work_struct	resume_by_ddi_work;

static void ilitek_resume_by_ddi_work(struct work_struct *work)
{
	mutex_lock(&ilits->touch_mutex);

	if (ilits->gesture)
		disable_irq_wake(ilits->irq_num);

	/* Set tp as demo mode and reload code if it's iram. */
	ilits->actual_tp_mode = P5_X_FW_AP_MODE;
	if (ilits->fw_upgrade_mode == UPGRADE_IRAM)
		ili_fw_upgrade_handler(NULL);
	else
		ili_reset_ctrl(ilits->reset);

	ili_pinctrl_select_normal(ilits);

	ili_irq_enable();
	ILI_INFO("TP resume end by wq\n");
	ili_wq_ctrl(WQ_ESD, ENABLE);
	ili_wq_ctrl(WQ_BAT, ENABLE);

	mutex_unlock(&ilits->touch_mutex);
}

void ili_resume_by_ddi(void)
{
	if (!resume_by_ddi_wq) {
		ILI_INFO("resume_by_ddi_wq is null\n");
		return;
	}

//	mutex_lock(&ilits->touch_mutex);

	ILI_INFO("TP resume start called by ddi\n");
	ilits->tp_suspend = false;
	/*
	 * To match the timing of sleep out, the first of mipi cmd must be sent within 10ms
	 * after TP reset. We then create a wq doing host download before resume.
	 */
	atomic_set(&ilits->fw_stat, ENABLE);
	ili_reset_ctrl(ilits->reset);
	ili_ice_mode_ctrl(ENABLE, OFF);
	ilits->ddi_rest_done = true;
	mdelay(5);
	queue_work(resume_by_ddi_wq, &(resume_by_ddi_work));
	ILI_INFO("TP resume work queued\n");

//	mutex_unlock(&ilits->touch_mutex);
}
#endif

int ili_mp_test_handler(char *apk, bool lcm_on)
{
	int ret = 0;

	if (atomic_read(&ilits->fw_stat)) {
		ILI_ERR("fw upgrade processing, ignore\n");
		return -EMP_FW_PROC;
	}

	atomic_set(&ilits->mp_stat, ENABLE);

	if (ilits->actual_tp_mode != P5_X_FW_TEST_MODE) {
		ret = ili_switch_tp_mode(P5_X_FW_TEST_MODE);
		if (ret < 0) {
			ILI_ERR("Switch MP mode failed\n");
			ret = -EMP_MODE;
			goto out;
		}
	}

	ret = ili_mp_test_main(apk, lcm_on);

out:
	/*
	 * If there's running mp test with lcm off, we suspose that
	 * users will soon call resume from suspend. TP mode will be changed
	 * from MP to AP mode until resume finished.
	 */
	if (!lcm_on) {
		atomic_set(&ilits->mp_stat, DISABLE);
		return ret;
	}

	ilits->actual_tp_mode = P5_X_FW_AP_MODE;
	if (ilits->fw_upgrade_mode == UPGRADE_IRAM) {
		if (ili_fw_upgrade_handler(NULL) < 0)
			ILI_ERR("FW upgrade failed during mp test\n");
	} else {
		if (ili_reset_ctrl(ilits->reset) < 0)
			ILI_ERR("TP Reset failed during mp test\n");
	}

	atomic_set(&ilits->mp_stat, DISABLE);
	return ret;
}

int ili_switch_tp_mode(u8 mode)
{
	int ret = 0;
	bool ges_dbg = false;

	atomic_set(&ilits->tp_sw_mode, START);

	ilits->actual_tp_mode = mode;

	/* able to see cdc data in gesture mode */
	if (ilits->tp_data_format == DATA_FORMAT_DEBUG &&
		ilits->actual_tp_mode == P5_X_FW_GESTURE_MODE)
		ges_dbg = true;

	switch (ilits->actual_tp_mode) {
	case P5_X_FW_AP_MODE:
		ILI_INFO("Switch to AP mode\n");
		ilits->wait_int_timeout = AP_INT_TIMEOUT;
		if (ilits->fw_upgrade_mode == UPGRADE_IRAM) {
			if (ili_fw_upgrade_handler(NULL) < 0)
				ILI_ERR("FW upgrade failed\n");
		} else {
			ret = ili_reset_ctrl(ilits->reset);
		}
		if (ret < 0)
			ILI_ERR("TP Reset failed\n");

		break;
	case P5_X_FW_GESTURE_MODE:
		ILI_INFO("Switch to Gesture mode\n");
		ilits->wait_int_timeout = AP_INT_TIMEOUT;
		ret = ilits->gesture_move_code(ilits->gesture_mode);
		if (ret < 0)
			ILI_ERR("Move gesture code failed\n");
		if (ges_dbg) {
			ILI_INFO("Enable gesture debug func\n");
			ili_set_tp_data_len(DATA_FORMAT_GESTURE_DEBUG, false, NULL);
		}
		break;
	case P5_X_FW_TEST_MODE:
		ILI_INFO("Switch to Test mode\n");
		ilits->wait_int_timeout = MP_INT_TIMEOUT;
		ret = ilits->mp_move_code();
		break;
	default:
		ILI_ERR("Unknown TP mode: %x\n", mode);
		ret = -1;
		break;
	}

	if (ret < 0)
		ILI_ERR("Switch TP mode (%d) failed \n", mode);

	ILI_DBG("Actual TP mode = %d\n", ilits->actual_tp_mode);
	atomic_set(&ilits->tp_sw_mode, END);
	return ret;
}

int ili_gesture_recovery(void)
{
	int ret = 0;

	atomic_set(&ilits->esd_stat, START);

	ILI_INFO("Doing gesture recovery\n");
	ret = ilits->ges_recover();

	atomic_set(&ilits->esd_stat, END);
	return ret;
}

void ili_spi_recovery(void)
{
	atomic_set(&ilits->esd_stat, START);

	ILI_INFO("Doing spi recovery\n");
	if (ili_fw_upgrade_handler(NULL) < 0)
		ILI_ERR("FW upgrade failed\n");

	atomic_set(&ilits->esd_stat, END);
}

int ili_wq_esd_spi_check(void)
{
	int ret = 0;
	u8 tx = SPI_WRITE, rx = 0;

	ret = ilits->spi_write_then_read(ilits->spi, &tx, 1, &rx, 1);
	ILI_DBG("spi esd check = 0x%x\n", ret);
	if (ret == DO_SPI_RECOVER) {
		ILI_ERR("ret = 0x%x\n", ret);
		return -1;
	}
	return 0;
}

int ili_wq_esd_i2c_check(void)
{
	ILI_DBG("");
	return 0;
}

static void ilitek_tddi_wq_esd_check(struct work_struct *work)
{
	if (mutex_is_locked(&ilits->touch_mutex)) {
		ILI_INFO("touch is locked, ignore\n");
		return;
	}
	mutex_lock(&ilits->touch_mutex);
	if (!ilits->tp_suspend && ilits->esd_recover() < 0) {
		ILI_ERR("SPI ACK failed, doing spi recovery\n");
		ili_spi_recovery();
	}
	mutex_unlock(&ilits->touch_mutex);
	complete_all(&ilits->esd_done);
	ili_wq_ctrl(WQ_ESD, ENABLE);
}

static int read_power_status(u8 *buf)
{
	struct file *f = NULL;
	mm_segment_t old_fs;
	ssize_t byte = 0;

	old_fs = get_fs();
	set_fs(get_ds());

	f = filp_open(POWER_STATUS_PATH, O_RDONLY, 0);
	if (ERR_ALLOC_MEM(f)) {
		ILI_ERR("Failed to open %s\n", POWER_STATUS_PATH);
		return -1;
	}

	f->f_op->llseek(f, 0, SEEK_SET);
	byte = f->f_op->read(f, buf, 20, &f->f_pos);

	ILI_DBG("Read %d bytes\n", (int)byte);

	set_fs(old_fs);
	filp_close(f, NULL);
	return 0;
}

static void ilitek_tddi_wq_bat_check(struct work_struct *work)
{
	u8 str[20] = {0};
	static int charge_mode;

	if (read_power_status(str) < 0)
		ILI_ERR("Read power status failed\n");

	ILI_DBG("Batter Status: %s\n", str);

	if (strstr(str, "Charging") != NULL || strstr(str, "Full") != NULL
		|| strstr(str, "Fully charged") != NULL) {
		if (charge_mode != 1) {
			ILI_DBG("Charging mode\n");
			if (ili_ic_func_ctrl("plug", DISABLE) < 0) // plug in
				ILI_ERR("Write plug in failed\n");
			charge_mode = 1;
		}
	} else {
		if (charge_mode != 2) {
			ILI_DBG("Not charging mode\n");
			if (ili_ic_func_ctrl("plug", ENABLE) < 0) // plug out
				ILI_ERR("Write plug out failed\n");
			charge_mode = 2;
		}
	}
	ili_wq_ctrl(WQ_BAT, ENABLE);
}

void ili_wq_ctrl(int type, int ctrl)
{
	switch (type) {
	case WQ_ESD:
		if (ENABLE_WQ_ESD || ilits->wq_ctrl) {
			if (!esd_wq) {
				ILI_ERR("wq esd is null\n");
				break;
			}
			ilits->wq_esd_ctrl = ctrl;
			if (ctrl == ENABLE) {
				ILI_DBG("execute esd check\n");
				if (!queue_delayed_work(esd_wq, &esd_work, msecs_to_jiffies(WQ_ESD_DELAY)))
					ILI_DBG("esd check was already on queue\n");
			} else {
				cancel_delayed_work_sync(&esd_work);
				flush_workqueue(esd_wq);
				ILI_DBG("cancel esd wq\n");
			}
		}
		break;
	case WQ_BAT:
		if (ENABLE_WQ_BAT || ilits->wq_ctrl) {
			if (!bat_wq) {
				ILI_ERR("WQ BAT is null\n");
				break;
			}
			ilits->wq_bat_ctrl = ctrl;
			if (ctrl == ENABLE) {
				ILI_DBG("execute bat check\n");
				if (!queue_delayed_work(bat_wq, &bat_work, msecs_to_jiffies(WQ_BAT_DELAY)))
					ILI_DBG("bat check was already on queue\n");
			} else {
				cancel_delayed_work_sync(&bat_work);
				flush_workqueue(bat_wq);
				ILI_DBG("cancel bat wq\n");
			}
		}
		break;
	default:
		ILI_ERR("Unknown WQ type, %d\n", type);
		break;
	}
}

static void ilitek_tddi_wq_init(void)
{
	esd_wq = alloc_workqueue("esd_check", WQ_MEM_RECLAIM, 0);
	bat_wq = alloc_workqueue("bat_check", WQ_MEM_RECLAIM, 0);

	WARN_ON(!esd_wq);
	WARN_ON(!bat_wq);

	INIT_DELAYED_WORK(&esd_work, ilitek_tddi_wq_esd_check);
	INIT_DELAYED_WORK(&bat_work, ilitek_tddi_wq_bat_check);

#if RESUME_BY_DDI
	resume_by_ddi_wq = create_singlethread_workqueue("resume_by_ddi_wq");
	WARN_ON(!resume_by_ddi_wq);
	INIT_WORK(&resume_by_ddi_work, ilitek_resume_by_ddi_work);
#endif
}
static int ili_pinctrl_init(struct ilitek_ts_data *ts)
{
	int ret = 0;

	ts->pinctrl = devm_pinctrl_get(ts->dev);
	if (IS_ERR_OR_NULL(ts->pinctrl)) {
		ILI_ERR("Failed to get pinctrl, please check dts");
		ret = PTR_ERR(ts->pinctrl);
		goto err_pinctrl_get;
	}

	ts->pins_active = pinctrl_lookup_state(ts->pinctrl, "pmx_ts_int_active");
	if (IS_ERR_OR_NULL(ts->pins_active)) {
		ILI_ERR("Pin state[active] not found");
		ret = PTR_ERR(ts->pins_active);
		goto err_pinctrl_lookup;
	}

	ts->pins_suspend = pinctrl_lookup_state(ts->pinctrl, "pmx_ts_int_suspend_high");
	if (IS_ERR_OR_NULL(ts->pins_suspend)) {
		ILI_ERR("Pin state[suspend] not found");
		ret = PTR_ERR(ts->pins_suspend);
	}

	return 0;
err_pinctrl_lookup:
	if (ts->pinctrl) {
		devm_pinctrl_put(ts->pinctrl);
	}
err_pinctrl_get:
	ts->pinctrl = NULL;
	ts->pins_suspend = NULL;
	ts->pins_active = NULL;
	return ret;
}

static int ili_pinctrl_select_normal(struct ilitek_ts_data *ts)
{
	int ret = 0;

	if (ts->pinctrl && ts->pins_active) {
		ret = pinctrl_select_state(ts->pinctrl, ts->pins_active);
		if (ret < 0) {
			ILI_ERR("Set normal pin state error:%d", ret);
		}
	}

	return ret;
}

static int ili_pinctrl_select_suspend(struct ilitek_ts_data *ts)
{
	int ret = 0;

	if (ts->pinctrl && ts->pins_suspend) {
		ret = pinctrl_select_state(ts->pinctrl, ts->pins_suspend);
		if (ret < 0) {
			ILI_ERR("Set suspend pin state error:%d", ret);
		}
	}

	return ret;
}

int ili_sleep_handler(int mode)
{
	int ret = 0;
	bool sense_stop = true;

	mutex_lock(&ilits->touch_mutex);
	atomic_set(&ilits->tp_sleep, START);

	if (atomic_read(&ilits->fw_stat) ||
		atomic_read(&ilits->mp_stat)) {
		ILI_INFO("fw upgrade or mp still running, ignore sleep requst\n");
		atomic_set(&ilits->tp_sleep, END);
		mutex_unlock(&ilits->touch_mutex);
		return 0;
	}

	ili_wq_ctrl(WQ_ESD, DISABLE);
	ili_wq_ctrl(WQ_BAT, DISABLE);
	ili_irq_disable();

	ILI_INFO("Sleep Mode = %d\n", mode);

	if (ilits->ss_ctrl)
		sense_stop = true;
	else if ((ilits->chip->core_ver >= CORE_VER_1430))
		sense_stop = false;
	else
		sense_stop = true;

	switch (mode) {
	case TP_SUSPEND:
		ILI_INFO("TP suspend start\n");
		if (sense_stop) {
			if (ili_ic_func_ctrl("sense", DISABLE) < 0)
				ILI_ERR("Write sense stop cmd failed\n");

			if (ili_ic_check_busy(50, 20) < 0)
				ILI_ERR("Check busy timeout during suspend\n");
		}

		if (ilits->gesture) {
			ili_switch_tp_mode(P5_X_FW_GESTURE_MODE);
			enable_irq_wake(ilits->irq_num);
			ili_irq_enable();
		} else {
			ili_pinctrl_select_suspend(ilits);
			if (ili_ic_func_ctrl("sleep", SLEEP_IN) < 0)
				ILI_ERR("Write sleep in cmd failed\n");
		}
		ILI_INFO("TP suspend end\n");
		ilits->tp_suspend = true;
		break;
	case TP_DEEP_SLEEP:
		ILI_INFO("TP deep suspend start\n");
		if (sense_stop) {
			if (ili_ic_func_ctrl("sense", DISABLE) < 0)
				ILI_ERR("Write sense stop cmd failed\n");

			if (ili_ic_check_busy(50, 20) < 0)
				ILI_ERR("Check busy timeout during deep suspend\n");
		}

		if (ilits->gesture) {
			ili_switch_tp_mode(P5_X_FW_GESTURE_MODE);
			enable_irq_wake(ilits->irq_num);
			ili_irq_enable();
		} else {
			ili_pinctrl_select_suspend(ilits);
			if (ili_ic_func_ctrl("sleep", DEEP_SLEEP_IN) < 0)
				ILI_ERR("Write deep sleep in cmd failed\n");
		}
		ILI_INFO("TP deep suspend end\n");
		ilits->tp_suspend = true;
		break;
	case TP_RESUME:
#if !RESUME_BY_DDI
		ILI_INFO("TP resume start\n");
		ilits->tp_suspend = false;

		if (ilits->gesture)
			disable_irq_wake(ilits->irq_num);
		else
			ili_pinctrl_select_normal(ilits);
		/* Set tp as demo mode and reload code if it's iram. */
		ilits->actual_tp_mode = P5_X_FW_AP_MODE;
		if (ilits->fw_upgrade_mode == UPGRADE_IRAM) {
			if (ili_fw_upgrade_handler(NULL) < 0)
				ILI_ERR("FW upgrade failed during resume\n");
		} else {
			if (ili_reset_ctrl(ilits->reset) < 0)
				ILI_ERR("TP Reset failed during resume\n");
			ili_ic_func_ctrl_reset();
		}
		ili_wq_ctrl(WQ_ESD, ENABLE);
		ili_wq_ctrl(WQ_BAT, ENABLE);

		ILI_INFO("TP resume end\n");
#else
		ili_resume_by_ddi();
#endif
		ili_irq_enable();
		break;
	default:
		ILI_ERR("Unknown sleep mode, %d\n", mode);
		ret = -EINVAL;
		break;
	}

	ili_touch_release_all_point();
	atomic_set(&ilits->tp_sleep, END);
	mutex_unlock(&ilits->touch_mutex);
	return ret;
}

int ili_fw_upgrade_handler(void *data)
{
	int ret = 0;

	atomic_set(&ilits->fw_stat, START);

	ilits->fw_update_stat = FW_STAT_INIT;
	ret = ili_fw_upgrade(ilits->fw_open);
	if (ret != 0) {
		ILI_INFO("FW upgrade fail\n");
		ilits->fw_update_stat = FW_UPDATE_FAIL;
	} else {
		ILI_INFO("FW upgrade pass\n");
#if CHARGER_NOTIFIER_CALLBACK
#if KERNEL_VERSION(4, 1, 0) <= LINUX_VERSION_CODE
		/* add_for_charger_start */
		if((ilits->usb_plug_status)&&(ilits->actual_tp_mode !=P5_X_FW_TEST_MODE)) {
			ret = ili_ic_func_ctrl("plug", !ilits->usb_plug_status);// plug in
			if(ret<0) {
				ILI_ERR("Write plug in failed\n");
			}
		}
		/*  add_for_charger_end  */
#endif
#endif
		ilits->fw_update_stat = FW_UPDATE_PASS;
	}

	if (!ilits->boot) {
		ilits->boot = true;
		ILI_INFO("Registre touch to input subsystem\n");
		ili_input_register();
		ili_wq_ctrl(WQ_ESD, ENABLE);
		ili_wq_ctrl(WQ_BAT, ENABLE);
	}

	atomic_set(&ilits->fw_stat, END);
	return ret;
}

int ili_set_tp_data_len(int format, bool send, u8* data)
{
	u8 cmd[10] = {0}, ctrl = 0, debug_ctrl = 0;
	u16 self_key = 2;
	int ret = 0, tp_mode = ilits->actual_tp_mode, len = 0;

	switch (format) {
	case DATA_FORMAT_DEMO:
	case DATA_FORMAT_GESTURE_DEMO:
		len = P5_X_DEMO_MODE_PACKET_LEN;
		ctrl = DATA_FORMAT_DEMO_CMD;
		break;
	case DATA_FORMAT_DEBUG:
	case DATA_FORMAT_GESTURE_DEBUG:
		len = (2 * ilits->xch_num * ilits->ych_num) + (ilits->stx * 2) + (ilits->srx * 2);
		len += 2 * self_key + (8 * 2) + 1 + 35;
		ctrl = DATA_FORMAT_DEBUG_CMD;
		break;
	case DATA_FORMAT_DEMO_DEBUG_INFO:
		/*only suport SPI interface now, so defult use size 1024 buffer*/
		len = P5_X_DEMO_MODE_PACKET_LEN +
			P5_X_DEMO_DEBUG_INFO_ID0_LENGTH + P5_X_INFO_HEADER_LENGTH;
		ctrl = DATA_FORMAT_DEMO_DEBUG_INFO_CMD;
		break;
	case DATA_FORMAT_GESTURE_INFO:
		len = P5_X_GESTURE_INFO_LENGTH;
		ctrl = DATA_FORMAT_GESTURE_INFO_CMD;
		break;
	case DATA_FORMAT_GESTURE_NORMAL:
		len = P5_X_GESTURE_NORMAL_LENGTH;
		ctrl = DATA_FORMAT_GESTURE_NORMAL_CMD;
		break;
	case DATA_FORMAT_GESTURE_SPECIAL_DEMO:
		if (ilits->gesture_demo_ctrl == ENABLE) {
			if (ilits->gesture_mode == DATA_FORMAT_GESTURE_INFO)
				len = P5_X_GESTURE_INFO_LENGTH + P5_X_INFO_HEADER_LENGTH + P5_X_INFO_CHECKSUM_LENGTH;
			else
				len = P5_X_DEMO_MODE_PACKET_LEN + P5_X_INFO_HEADER_LENGTH + P5_X_INFO_CHECKSUM_LENGTH;
		} else {
			if (ilits->gesture_mode == DATA_FORMAT_GESTURE_INFO)
				len = P5_X_GESTURE_INFO_LENGTH;
			else
				len = P5_X_GESTURE_NORMAL_LENGTH;
		}
		ILI_INFO("Gesture demo mode control = %d\n",  ilits->gesture_demo_ctrl);
		ili_ic_func_ctrl("gesture_demo_en", ilits->gesture_demo_ctrl);
		ILI_INFO("knock_en setting\n");
		ili_ic_func_ctrl("knock_en", 0x8);
		break;
	case DATA_FORMAT_DEBUG_LITE_ROI:
		debug_ctrl = DATA_FORMAT_DEBUG_LITE_ROI_CMD;
		ctrl = DATA_FORMAT_DEBUG_LITE_CMD;
		break;
	case DATA_FORMAT_DEBUG_LITE_WINDOW:
		debug_ctrl = DATA_FORMAT_DEBUG_LITE_WINDOW_CMD;
		ctrl = DATA_FORMAT_DEBUG_LITE_CMD;
		break;
	case DATA_FORMAT_DEBUG_LITE_AREA:
		/*if(cmd == NULL) {
			ILI_ERR("DATA_FORMAT_DEBUG_LITE_AREA error cmd\n");
			return -1;
		}*/
		debug_ctrl = DATA_FORMAT_DEBUG_LITE_AREA_CMD;
		ctrl = DATA_FORMAT_DEBUG_LITE_CMD;
		cmd[3] = data[0];
		cmd[4] = data[1];
		cmd[5] = data[2];
		cmd[6] = data[3];
		break;
	default:
		ILI_ERR("Unknow TP data format\n");
		return -1;
	}

	if (ctrl == DATA_FORMAT_DEBUG_LITE_CMD) {
		len = P5_X_DEBUG_LITE_LENGTH;
		cmd[0] = P5_X_MODE_CONTROL;
		cmd[1] = ctrl;
		cmd[2] = debug_ctrl;
		ret = ilits->wrapper(cmd, 10, NULL, 0, ON, OFF);

		if (ret < 0) {
			ILI_ERR("switch to format %d failed\n", format);
			ili_switch_tp_mode(P5_X_FW_AP_MODE);
		}

	} else if (tp_mode == P5_X_FW_AP_MODE ||
		format == DATA_FORMAT_GESTURE_DEMO ||
		format == DATA_FORMAT_GESTURE_DEBUG) {
		cmd[0] = P5_X_MODE_CONTROL;
		cmd[1] = ctrl;
		ret = ilits->wrapper(cmd, 2, NULL, 0, ON, OFF);

		if (ret < 0) {
			ILI_ERR("switch to format %d failed\n", format);
			ili_switch_tp_mode(P5_X_FW_AP_MODE);
		}
	} else if (tp_mode == P5_X_FW_GESTURE_MODE) {

		/*set gesture symbol*/
		ili_set_gesture_symbol();

		if (send) {
			ret = ili_ic_func_ctrl("lpwg", ctrl);
			if (ret < 0)
				ILI_ERR("write gesture mode failed\n");
		}
	}

	ilits->tp_data_format = format;
	ilits->tp_data_len = len;
	ILI_INFO("TP mode = %d, format = %d, len = %d\n",
		tp_mode, ilits->tp_data_format, ilits->tp_data_len);

	return ret;
}

int ili_report_handler(void)
{
	int ret = 0, pid = 0;
	u8  checksum = 0, pack_checksum = 0;
	u8 *trdata = NULL;
	int rlen = 0;
	int tmp = debug_en;

	/* Just in case these stats couldn't be blocked in top half context */
	if (!ilits->report || atomic_read(&ilits->tp_reset) ||
		atomic_read(&ilits->fw_stat) || atomic_read(&ilits->tp_sw_mode) ||
		atomic_read(&ilits->mp_stat) || atomic_read(&ilits->tp_sleep)) {
		ILI_INFO("ignore report request\n");
		return -EINVAL;
	}

	if (ilits->irq_after_recovery) {
		ILI_INFO("ignore int triggered by recovery\n");
		ilits->irq_after_recovery = false;
		return -EINVAL;
	}

	ili_wq_ctrl(WQ_ESD, DISABLE);
	ili_wq_ctrl(WQ_BAT, DISABLE);

	if (ilits->actual_tp_mode == P5_X_FW_GESTURE_MODE) {
		PM_STAY_AWAKE(ilits->ws);

		if (ilits->pm_suspend) {
			/* Waiting for pm resume completed */
			ret = wait_for_completion_timeout(&ilits->pm_completion, msecs_to_jiffies(700));
			if (!ret) {
				ILI_ERR("system(spi) can't finished resuming procedure.");
			}
		}
	}

	rlen = ilits->tp_data_len;
	ILI_DBG("Packget length = %d\n", rlen);

	if (!rlen || rlen > TR_BUF_SIZE) {
		ILI_ERR("Length of packet is invaild\n");
		goto out;
	}

	memset(ilits->tr_buf, 0x0, TR_BUF_SIZE);

	ret = ilits->wrapper(NULL, 0, ilits->tr_buf, rlen, OFF, OFF);
	if (ret < 0) {
		ILI_ERR("Read report packet failed, ret = %d\n", ret);
		if (ret == DO_SPI_RECOVER) {
			ili_ic_get_pc_counter(DO_SPI_RECOVER);
			if (ilits->actual_tp_mode == P5_X_FW_GESTURE_MODE && ilits->gesture && !ilits->prox_near) {
				ILI_ERR("Gesture failed, doing gesture recovery\n");
				if (ili_gesture_recovery() < 0)
					ILI_ERR("Failed to recover gesture\n");
				ilits->irq_after_recovery = true;
			} else {
				ILI_ERR("SPI ACK failed, doing spi recovery\n");
				ili_spi_recovery();
				ilits->irq_after_recovery = true;
			}
		}
		goto out;
	}

	ili_dump_data(ilits->tr_buf, 8, rlen, 0, "finger report");

	checksum = ili_calc_packet_checksum(ilits->tr_buf, rlen - 1);
	pack_checksum = ilits->tr_buf[rlen-1];
	trdata = ilits->tr_buf;
	pid = trdata[0];
	ILI_DBG("Packet ID = %x\n", pid);

	if (checksum != pack_checksum && pid != P5_X_I2CUART_PACKET_ID) {
		ILI_ERR("Checksum Error (0x%X)! Pack = 0x%X, len = %d\n", checksum, pack_checksum, rlen);
		debug_en = DEBUG_ALL;
		ili_dump_data(trdata, 8, rlen, 0, "finger report with wrong");
		debug_en = tmp;
		ret = -EINVAL;
		goto out;
	}

	if (pid == P5_X_INFO_HEADER_PACKET_ID) {
		trdata = ilits->tr_buf + P5_X_INFO_HEADER_LENGTH;
		pid = trdata[0];
	}

	switch (pid) {
	case P5_X_DEMO_PACKET_ID:
		ili_report_ap_mode(trdata, rlen);
		break;
	case P5_X_DEBUG_PACKET_ID:
		ili_report_debug_mode(trdata, rlen);
		break;
	case P5_X_DEBUG_LITE_PACKET_ID:
		ili_report_debug_lite_mode(trdata, rlen);
		break;
	case P5_X_I2CUART_PACKET_ID:
		ili_report_i2cuart_mode(trdata, rlen);
		break;
	case P5_X_GESTURE_PACKET_ID:
		ili_report_gesture_mode(trdata, rlen);
		break;
	case P5_X_GESTURE_FAIL_ID:
		ILI_INFO("gesture fail reason code = 0x%02x", trdata[1]);
		break;
	case P5_X_DEMO_DEBUG_INFO_PACKET_ID:
		ili_demo_debug_info_mode(trdata, rlen);
		break;
	default:
		ILI_ERR("Unknown packet id, %x\n", pid);
#if ( TDDI_INTERFACE == BUS_I2C )
		ili_ic_get_pc_counter(DO_I2C_RECOVER);
		if (ilits->fw_latch !=0){
			msleep(100);
			ili_ic_func_ctrl_reset();
			ILI_ERR("I2C func_ctrl_reset\n");
		}
		if ((ilits->actual_tp_mode == P5_X_FW_GESTURE_MODE) && ilits->fw_latch !=0){
			msleep(100);
			ili_set_gesture_symbol();
			ILI_ERR("I2C gesture_symbol\n");
		}
#endif
		break;
	}

out:
	if (ilits->actual_tp_mode != P5_X_FW_GESTURE_MODE) {
		ili_wq_ctrl(WQ_ESD, ENABLE);
		ili_wq_ctrl(WQ_BAT, ENABLE);
	}

	if (ilits->actual_tp_mode == P5_X_FW_GESTURE_MODE)
		PM_RELAX(ilits->ws);
	return ret;
}

int ili_reset_ctrl(int mode)
{
	int ret = 0;

	atomic_set(&ilits->tp_reset, START);

	switch (mode) {
	case TP_IC_CODE_RST:
		ILI_INFO("TP IC Code RST \n");
		ret = ili_ic_code_reset();
		if (ret < 0)
			ILI_ERR("IC Code reset failed\n");
		break;
	case TP_IC_WHOLE_RST:
		ILI_INFO("TP IC whole RST\n");
		ret = ili_ic_whole_reset();
		if (ret < 0)
			ILI_ERR("IC whole reset failed\n");
		break;
	case TP_HW_RST_ONLY:
		ILI_INFO("TP HW RST\n");
		ili_tp_reset();
		break;
	default:
		ILI_ERR("Unknown reset mode, %d\n", mode);
		ret = -EINVAL;
		break;
	}

	/*
	 * Since OTP must be folloing with reset, except for code rest,
	 * the stat of ice mode should be set as 0.
	 */
	if (mode != TP_IC_CODE_RST)
		atomic_set(&ilits->ice_stat, DISABLE);

	ilits->tp_data_format = DATA_FORMAT_DEMO;
	ilits->tp_data_len = P5_X_DEMO_MODE_PACKET_LEN;
	ilits->pll_clk_wakeup = true;
	atomic_set(&ilits->tp_reset, END);
	return ret;
}

static int ilitek_get_tp_module(void)
{
	/*
	 * TODO: users should implement this function
	 * if there are various tp modules been used in projects.
	 */

	return ilits->tp_module;
}

static void ili_update_tp_module_info(void)
{
	int module;

	module = ilitek_get_tp_module();

	switch (module) {
	case MODEL_CSOT:
		ilits->md_name = "CSOT";
		ilits->md_fw_filp_path = CSOT_FW_FILP_PATH;
		ilits->md_fw_rq_path = CSOT_FW_REQUEST_PATH;
		ilits->md_ini_path = CSOT_INI_NAME_PATH;
		ilits->md_ini_rq_path = CSOT_INI_REQUEST_PATH;
		ilits->md_fw_ili = CTPM_FW_CSOT;
		ilits->md_fw_ili_size = sizeof(CTPM_FW_CSOT);
		break;
	case MODEL_AUO:
		ilits->md_name = "AUO";
		ilits->md_fw_filp_path = AUO_FW_FILP_PATH;
		ilits->md_fw_rq_path = AUO_FW_REQUEST_PATH;
		ilits->md_ini_path = AUO_INI_NAME_PATH;
		ilits->md_ini_rq_path = AUO_INI_REQUEST_PATH;
		ilits->md_fw_ili = CTPM_FW_AUO;
		ilits->md_fw_ili_size = sizeof(CTPM_FW_AUO);
		break;
	case MODEL_BOE:
		ilits->md_name = "BOE";
		ilits->md_fw_filp_path = BOE_FW_FILP_PATH;
		ilits->md_fw_rq_path = BOE_FW_REQUEST_PATH;
		ilits->md_ini_path = BOE_INI_NAME_PATH;
		ilits->md_ini_rq_path = BOE_INI_REQUEST_PATH;
		ilits->md_fw_ili = CTPM_FW_BOE;
		ilits->md_fw_ili_size = sizeof(CTPM_FW_BOE);
		break;
	case MODEL_INX:
		ilits->md_name = "INX";
		ilits->md_fw_filp_path = INX_FW_FILP_PATH;
		ilits->md_fw_rq_path = INX_FW_REQUEST_PATH;
		ilits->md_ini_path = INX_INI_NAME_PATH;
		ilits->md_ini_rq_path = INX_INI_REQUEST_PATH;
		ilits->md_fw_ili = CTPM_FW_INX;
		ilits->md_fw_ili_size = sizeof(CTPM_FW_INX);
		break;
	case MODEL_DJ:
		ilits->md_name = "DJ";
		ilits->md_fw_filp_path = DJ_FW_FILP_PATH;
		ilits->md_fw_rq_path = DJ_FW_REQUEST_PATH;
		ilits->md_ini_path = DJ_INI_NAME_PATH;
		ilits->md_ini_rq_path = DJ_INI_REQUEST_PATH;
		ilits->md_fw_ili = CTPM_FW_DJ;
		ilits->md_fw_ili_size = sizeof(CTPM_FW_DJ);
		break;
	case MODEL_TXD:
		ilits->md_name = "TXD";
		ilits->md_fw_filp_path = TXD_FW_FILP_PATH;
		ilits->md_fw_rq_path = TXD_FW_REQUEST_PATH;
		ilits->md_ini_path = TXD_INI_NAME_PATH;
		ilits->md_ini_rq_path = TXD_FW_REQUEST_PATH;
		ilits->md_fw_ili = CTPM_FW_TXD;
		ilits->md_fw_ili_size = sizeof(CTPM_FW_TXD);
		break;
	case MODEL_TXD_9882H:
		ilits->md_name = "TXD_9882H";
		ilits->md_fw_filp_path = TXD_9882H_FW_FILP_PATH;
		ilits->md_fw_rq_path = TXD_9882H_FW_REQUEST_PATH;
		ilits->md_ini_path = TXD_9882H_INI_NAME_PATH;
		ilits->md_ini_rq_path = TXD_9882H_FW_REQUEST_PATH;
		ilits->md_fw_ili = CTPM_FW_TXD_9882H;
		ilits->md_fw_ili_size = sizeof(CTPM_FW_TXD_9882H);
		break;
	case MODEL_TXD_9882N:
		ilits->md_name = "TXD_9882N";
		ilits->md_fw_filp_path = TXD_9882N_FW_FILP_PATH;
		ilits->md_fw_rq_path = TXD_9882N_FW_REQUEST_PATH;
		ilits->md_ini_path = TXD_9882N_INI_NAME_PATH;
		ilits->md_ini_rq_path = TXD_9882N_FW_REQUEST_PATH;
		ilits->md_fw_ili = CTPM_FW_TXD_9882N;
		ilits->md_fw_ili_size = sizeof(CTPM_FW_TXD_9882N);
		break;
	case MODEL_TXD_7806S:
		ilits->md_name = "TXD_7806S";
		ilits->md_fw_filp_path = TXD_7806S_FW_FILP_PATH;
		ilits->md_fw_rq_path = TXD_7806S_FW_REQUEST_PATH;
		ilits->md_ini_path = TXD_7806S_INI_NAME_PATH;
		ilits->md_ini_rq_path = TXD_7806S_INI_REQUEST_PATH;
		ilits->md_fw_ili = CTPM_FW_TXD_7806S;
		ilits->md_fw_ili_size = sizeof(CTPM_FW_TXD_7806S);
		break;
	case MODEL_TM:
		ilits->md_name = "ILITEK_TDDI_TM";
		ilits->md_fw_filp_path = TM_FW_REQUEST_PATH;
		ilits->md_fw_rq_path = TM_FW_REQUEST_PATH;
		ilits->md_ini_path = TM_INI_NAME_PATH;
		ilits->md_ini_rq_path = TM_INI_REQUEST_PATH;
		ilits->md_fw_ili = CTPM_FW_TM;
		ilits->md_fw_ili_size = sizeof(CTPM_FW_TM);
		break;
	case MODEL_TM_9882N:
		ilits->md_name = "TM_9882N";
		ilits->md_fw_filp_path = TM_9882N_FW_FILP_PATH;
		ilits->md_fw_rq_path = TM_9882N_FW_REQUEST_PATH;
		ilits->md_ini_path = TM_9882N_INI_NAME_PATH;
		ilits->md_ini_rq_path = TM_9882N_FW_REQUEST_PATH;
		ilits->md_fw_ili = CTPM_FW_TM_9882N;
		ilits->md_fw_ili_size = sizeof(CTPM_FW_TM_9882N);
		break;
	case MODEL_TM_9882H:
		ilits->md_name = "TM_9882H";
		ilits->md_fw_filp_path = TM_9882H_FW_FILP_PATH;
		ilits->md_fw_rq_path = TM_9882H_FW_REQUEST_PATH;
		ilits->md_ini_path = TM_9882H_INI_NAME_PATH;
		ilits->md_ini_rq_path = TM_9882H_FW_REQUEST_PATH;
		ilits->md_fw_ili = CTPM_FW_TM_9882H;
		ilits->md_fw_ili_size = sizeof(CTPM_FW_TM_9882H);
		break;
	case MODEL_TIANMA_9882N:
		ilits->md_name = "TIANMA_9882N";
		ilits->md_fw_filp_path = TIANMA_9882N_FW_FILP_PATH;
		ilits->md_fw_rq_path = TIANMA_9882N_FW_REQUEST_PATH;
		ilits->md_ini_path = TIANMA_9882N_INI_NAME_PATH;
		ilits->md_ini_rq_path = TIANMA_9882N_FW_REQUEST_PATH;
		ilits->md_fw_ili = CTPM_FW_TIANMA_9882N;
		ilits->md_fw_ili_size = sizeof(CTPM_FW_TIANMA_9882N);
		break;
	case MODEL_HLT:
		ilits->md_name = "ILITEK_TDDI_HLT";
		ilits->md_fw_filp_path = HLT_FW_REQUEST_PATH;
		ilits->md_fw_rq_path = HLT_FW_REQUEST_PATH;
		ilits->md_ini_path = HLT_INI_NAME_PATH;
		ilits->md_ini_rq_path = HLT_INI_REQUEST_PATH;
		ilits->md_fw_ili = CTPM_FW_HLT;
		ilits->md_fw_ili_size = sizeof(CTPM_FW_HLT);
		break;
	default:
		break;
	}

	if (module == 0) {
		ILI_ERR("Couldn't find any tp modules, applying default settings\n");
		ilits->md_name = "DEF";
		ilits->md_fw_filp_path = DEF_FW_FILP_PATH;
		ilits->md_fw_rq_path = DEF_FW_REQUEST_PATH;
		ilits->md_ini_path = DEF_INI_NAME_PATH;
		ilits->md_ini_rq_path = DEF_INI_REQUEST_PATH;
		ilits->md_fw_ili = CTPM_FW_DEF;
		ilits->md_fw_ili_size = sizeof(CTPM_FW_DEF);
	}

	ILI_INFO("Found %s module: ini path = %s, fw path = (%s, %s, %d)\n",
			ilits->md_name,
			ilits->md_ini_path,
			ilits->md_fw_filp_path,
			ilits->md_fw_rq_path,
			ilits->md_fw_ili_size);
}

int ili_tddi_init(void)
{
#if (BOOT_FW_UPDATE | HOST_DOWN_LOAD)
	struct task_struct *fw_boot_th;
#endif

	ILI_INFO("driver version = %s\n", DRIVER_VERSION);

	mutex_init(&ilits->touch_mutex);
	mutex_init(&ilits->debug_mutex);
	mutex_init(&ilits->debug_read_mutex);
	init_waitqueue_head(&(ilits->inq));
	spin_lock_init(&ilits->irq_spin);
	init_completion(&ilits->esd_done);

	//atomic_set(&ilits->irq_stat, DISABLE);
	atomic_set(&ilits->ice_stat, DISABLE);
	atomic_set(&ilits->tp_reset, END);
	atomic_set(&ilits->fw_stat, END);
	atomic_set(&ilits->mp_stat, DISABLE);
	atomic_set(&ilits->tp_sleep, END);
	atomic_set(&ilits->cmd_int_check, DISABLE);
	atomic_set(&ilits->esd_stat, END);

	ili_ic_init();
	ilitek_tddi_wq_init();
	ili_pinctrl_init(ilits);
	ili_pinctrl_select_normal(ilits);
	/* Must do hw reset once in first time for work normally if tp reset is avaliable */
#if !TDDI_RST_BIND
	if (ili_reset_ctrl(ilits->reset) < 0)
		ILI_ERR("TP Reset failed during init\n");
#endif

	ilits->demo_debug_info[0] = ili_demo_debug_info_id0;
	ilits->tp_data_format = DATA_FORMAT_DEMO;
	ilits->boot = false;

	/*
	 * This status of ice enable will be reset until process of fw upgrade runs.
	 * it might cause unknown problems if we disable ice mode without any
	 * codes inside touch ic.
	 */
	if (ili_ice_mode_ctrl(ENABLE, OFF) < 0)
		ILI_ERR("Failed to enable ice mode during ili_tddi_init\n");

	if ( ili_ic_dummy_check() < 0 ){
		ILI_ERR("Not found ilitek chip\n");
		return -ENODEV;
	}

	if (ili_ic_get_info() < 0)
		ILI_ERR("Chip info is incorrect\n");

	ili_update_tp_module_info();

	ili_node_init();

	ili_fw_read_flash_info();

#if (BOOT_FW_UPDATE | HOST_DOWN_LOAD)
	fw_boot_th = kthread_run(ili_fw_upgrade_handler, NULL, "ili_fw_boot");
	if (fw_boot_th == (struct task_struct *)ERR_PTR) {
		fw_boot_th = NULL;
		WARN_ON(!fw_boot_th);
		ILI_ERR("Failed to create fw upgrade thread\n");
	}
#else
	if (ili_ice_mode_ctrl(DISABLE, OFF) < 0)
		ILI_ERR("Failed to disable ice mode failed during init\n");

#if (TDDI_INTERFACE == BUS_I2C)
	ilits->info_from_hex = DISABLE;
#endif

	ili_ic_get_core_ver();
	ili_ic_get_protocl_ver();
	ili_ic_get_fw_ver();
	ili_ic_get_tp_info();
	ili_ic_get_panel_info();

#if (TDDI_INTERFACE == BUS_I2C)
	ilits->info_from_hex = ENABLE;
#endif

	ILI_INFO("Registre touch to input subsystem\n");
	ili_input_register();
	ili_wq_ctrl(WQ_ESD, ENABLE);
	ili_wq_ctrl(WQ_BAT, ENABLE);
	ilits->boot = true;
#endif

	PM_WAKEUP_REGISTER(NULL, ilits->ws, "ili_wakelock");
	if (!ilits->ws) {
		ILI_ERR("wakeup source request failed\n");
		return -ENOMEM;
	}

	return 0;
}

void ili_dev_remove(void)
{
	ILI_INFO("remove ilitek dev\n");

	if (!ilits)
		return;

	gpio_free(ilits->tp_int);
	gpio_free(ilits->tp_rst);

	if (esd_wq != NULL) {
		cancel_delayed_work_sync(&esd_work);
		flush_workqueue(esd_wq);
		destroy_workqueue(esd_wq);
	}
	if (bat_wq != NULL) {
		cancel_delayed_work_sync(&bat_work);
		flush_workqueue(bat_wq);
		destroy_workqueue(bat_wq);
	}

	if (ilits->ws)
		PM_WAKEUP_UNREGISTER(ilits->ws);

	kfree(ilits->tr_buf);
	kfree(ilits->gcoord);
	ili_interface_dev_exit(ilits);
}

int ili_dev_init(struct ilitek_hwif_info *hwif)
{
	ILI_INFO("TP Interface: %s\n", (hwif->bus_type == BUS_I2C) ? "I2C" : "SPI");
	return ili_interface_dev_init(hwif);
}
