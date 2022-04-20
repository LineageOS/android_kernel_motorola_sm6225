/*
 * Copyright (C) 2020-2021 Motorola Mobility LLC
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/version.h>
#include <linux/alarmtimer.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/workqueue.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/soc/qcom/pmic_glink.h>
#include <linux/power/bm_adsp_ulog.h>

#include "mmi_charger.h"
#include "qti_glink_charger.h"

/* PPM specific definitions */
#define MSG_OWNER_OEM			32782
#define MSG_TYPE_REQ_RESP		1
#define MSG_TYPE_NOTIFY			2
#define OEM_PROPERTY_DATA_SIZE		16

#define OEM_READ_BUF_REQ		0x10000
#define OEM_WRITE_BUF_REQ		0x10001
#define OEM_NOTIFY_IND			0x10002

#define OEM_WAIT_TIME_MS		5000

#define BATT_DEFAULT_ID 107000
#define BATT_SN_UNKNOWN "unknown-sn"

#define OEM_BM_ULOG_SIZE		4096

#define VBUS_MIN_MV			4000

#define FOD_GAIN_MAX_LEN 16
#define FOD_CURR_MAX_LEN 7

static bool debug_enabled;
module_param(debug_enabled, bool, 0600);
MODULE_PARM_DESC(debug_enabled, "Enable debug for qti glink charger driver");

struct battery_info {
	int batt_uv;
	int batt_ua;
	int batt_soc; /* 0 ~ 10000 indicating 0% to 100% */
	int batt_temp; /* hundredth degree */
	int batt_status;
	int batt_full_uah;
	int batt_design_uah;
};

struct charger_info {
	int chrg_uv;
	int chrg_ua;
	int chrg_type;
	int chrg_pmax_mw;
	int chrg_present;
};

struct charger_profile_info {
	int fg_iterm;
        int chrg_iterm;
        int max_fv_uv;
        int max_fcc_ua;
        int vfloat_comp_uv;
        int demo_fv_uv;
        int data_bk_size; /* number of byte for each data block */
	int data_size; /* number of byte for while data */
	int profile_id; /* profile id for charging profile selection in ADSP */
};

struct lpd_info {
	int lpd_present;
	int lpd_rsbu1;
	int lpd_rsbu2;
};

struct oem_notify_ind_msg {
	struct pmic_glink_hdr	hdr;
	u32			notification;
	u32			receiver;
	u32			data[MAX_OEM_NOTIFY_DATA_LEN];
};

struct oem_read_buf_req_msg {
	struct pmic_glink_hdr	hdr;
	u32			oem_property_id;
	u32			data_size;
};

struct oem_read_buf_resp_msg {
	struct pmic_glink_hdr	hdr;
	u32			oem_property_id;
	u32			buf[OEM_PROPERTY_DATA_SIZE];
	u32			data_size;
};

struct oem_write_buf_req_msg {
	struct pmic_glink_hdr	hdr;
	u32			oem_property_id;
	u32			buf[OEM_PROPERTY_DATA_SIZE];
	u32			data_size;
};

struct oem_write_buf_resp_msg {
	struct pmic_glink_hdr	hdr;
	u32			ret_code;
};


struct fod_curr {
	u32	fod_array_curr[FOD_CURR_MAX_LEN];
};

struct fod_gain {
	u32	fod_array_gain[FOD_GAIN_MAX_LEN];
};

#ifdef WIRELESS_CPS4035B
struct wls_dump
{
    u32  chip_id;
    u32  mtp_fw_ver;
    u32  irq_status;
    u16  sys_mode;
    u16  op_mode;
    u16  rx_fop;
    u16  rx_vout_mv;
    s16  rx_vrect_mv;
    u16  rx_irect_ma;
    u16  rx_ept;
    u16  rx_ce;
    u32  rx_rp;
    s16  rx_dietemp;
    u16  rx_neg_power;
    s16  tx_iin_ma;
    u16  tx_vin_mv;
    u16  tx_vrect_mv;
    u16  tx_det_rx_power;
    u16  tx_power;
    s16  power_loss;
    u16  usb_otg;
    u16  wls_boost;
    u16  wls_icl_ma;
    u16  wls_icl_therm_ma;
};
#else
struct wls_dump
{
    u32  chip_id;
    u32  mtp_fw_ver;
    u32  irq_status;
    u16  sys_mode;
    u16  op_mode;
    u16  rx_fop;
    u16  rx_vout_mv;
    u16  rx_vrect_mv;
    u16  rx_irect_ma;
    u16  rx_neg_power;
    u16  tx_iin_ma;
    u16  tx_vin_mv;
    u16  tx_vrect_mv;
    u16  tx_fod_I;
    u16  tx_fod_II;
    u16  tx_fod_rp;
    u16  tx_det_rx_power;
    u16  tx_power;
    s16  power_loss;
    u16  folio_mode;
    u16  pen_status;
    u16  pen_soc;
    u16  pen_error;
    u16  usb_otg;
    u16  wls_boost;
    u16  wls_icl_ma;
    u16  wls_icl_therm_ma;
};
#endif

struct qti_charger {
	char				*name;
	struct device			*dev;
	struct pmic_glink_client	*client;
	struct completion		read_ack;
	struct completion		write_ack;
	struct mutex			read_lock;
	struct mutex			write_lock;
	struct oem_read_buf_resp_msg	rx_buf;
	atomic_t			rx_valid;
	struct work_struct		setup_work;
	struct work_struct		notify_work;
	struct oem_notify_ind_msg	notify_msg;
	atomic_t			state;
	u32				chrg_taper_cnt;
	struct mmi_battery_info		batt_info;
	struct mmi_charger_info		chg_info;
	struct mmi_charger_cfg		chg_cfg;
	struct mmi_charger_constraint	constraint;
	struct mmi_charger_driver	*driver;
	struct power_supply		*wls_psy;
	u32				*profile_data;
	struct charger_profile_info	profile_info;
	struct lpd_info			lpd_info;
	void				*ipc_log;
	struct fod_curr			rx_fod_curr;
	struct fod_gain			rx_fod_gain;
	u32				tx_mode;
	u32				folio_mode;
	bool				*debug_enabled;
	u32				wls_curr_max;
	u32				rx_connected;
	struct notifier_block		wls_nb;
	struct dentry		*debug_root;
	struct power_supply		*batt_psy;
};

static struct qti_charger *this_chip = NULL;
static BLOCKING_NOTIFIER_HEAD(qti_chg_notifier_list);

static int find_profile_id(struct qti_charger *chg)
{
	int i;
	int rc;
	int count;
	int profile_id = -EINVAL;
	struct profile_sn_map {
		const char *id;
		const char *sn;
	} *map_table;

	count = of_property_count_strings(chg->dev->of_node, "profile-ids-map");
	if (count <= 0 || (count % 2)) {
		mmi_err(chg, "Invalid profile-ids-map in DT, rc=%d\n", count);
		return -EINVAL;
	}

	map_table = devm_kmalloc_array(chg->dev, count / 2,
					sizeof(struct profile_sn_map),
					GFP_KERNEL);
	if (!map_table)
		return -ENOMEM;

	rc = of_property_read_string_array(chg->dev->of_node, "profile-ids-map",
					(const char **)map_table,
					count);
	if (rc < 0) {
		mmi_err(chg, "Failed to get profile-ids-map, rc=%d\n", rc);
		profile_id = rc;
		goto free_map;
	}

	for (i = 0; i < count / 2 && map_table[i].sn; i++) {
		mmi_info(chg, "profile_ids_map[%d]: id=%s, sn=%s\n", i,
					map_table[i].id, map_table[i].sn);
		if (!strcmp(map_table[i].sn, chg->batt_info.batt_sn))
			profile_id = i;
	}

	if (profile_id >= 0 && profile_id < count / 2) {
		i = profile_id;
		profile_id = 0;
		rc = kstrtou32(map_table[i].id, 0, &profile_id);
		if (rc) {
			mmi_err(chg, "Invalid id: %s, sn: %s\n",
						map_table[i].id,
						map_table[i].sn);
			profile_id = rc;
		} else {
			mmi_info(chg, "profile id: %s(%d), sn: %s\n",
						map_table[i].id,
						profile_id,
						map_table[i].sn);
		}
	} else {
		mmi_warn(chg, "No matched profile id in profile-ids-map\n");
	}

free_map:
	devm_kfree(chg->dev, map_table);

	return profile_id;
}

static int handle_oem_read_ack(struct qti_charger *chg, void *data, size_t len)
{
	if (len != sizeof(chg->rx_buf)) {
		mmi_err(chg, "Incorrect received length %zu expected %lu\n", len,
			sizeof(chg->rx_buf));
		atomic_set(&chg->rx_valid, 0);
		return -EINVAL;
	}

	memcpy(&chg->rx_buf, data, sizeof(chg->rx_buf));
	atomic_set(&chg->rx_valid, 1);
	complete(&chg->read_ack);
	mmi_dbg(chg, "read ack for property: %u\n", chg->rx_buf.oem_property_id);

	return 0;
}

static int handle_oem_write_ack(struct qti_charger *chg, void *data, size_t len)
{
	struct oem_write_buf_resp_msg *msg_ptr;

	if (len != sizeof(*msg_ptr)) {
		mmi_err(chg, "Incorrect received length %zu expected %lu\n", len,
			sizeof(*msg_ptr));
		return -EINVAL;
	}

	msg_ptr = data;
	if (msg_ptr->ret_code) {
		mmi_err(chg, "write ack, ret_code: %u\n", msg_ptr->ret_code);
		return -EINVAL;
	}

	mmi_dbg(chg, "write ack\n");
	complete(&chg->write_ack);

	return 0;
}

static int handle_oem_notification(struct qti_charger *chg, void *data, size_t len)
{
	struct oem_notify_ind_msg *notify_msg = data;
	if (len != sizeof(*notify_msg)) {
		mmi_err(chg, "Incorrect received length %zu expected %lu\n", len,
			sizeof(*notify_msg));
		return -EINVAL;
	}

	mmi_info(chg, "notification: %#x on receiver: %#x\n",
				notify_msg->notification,
				notify_msg->receiver);

	pm_stay_awake(chg->dev);
	memcpy(&chg->notify_msg, notify_msg, sizeof(*notify_msg));
	schedule_work(&chg->notify_work);

	return 0;
}

static int oem_callback(void *priv, void *data, size_t len)
{
	struct pmic_glink_hdr *hdr = data;
	struct qti_charger *chg = priv;

	mmi_dbg(chg, "owner: %u type: %u opcode: 0x%x len:%zu\n", hdr->owner,
		hdr->type, hdr->opcode, len);

	if (hdr->opcode == OEM_READ_BUF_REQ)
		handle_oem_read_ack(chg, data, len);
	else if (hdr->opcode == OEM_WRITE_BUF_REQ)
		handle_oem_write_ack(chg, data, len);
	else if (hdr->opcode == OEM_NOTIFY_IND)
		handle_oem_notification(chg, data, len);
	else
		mmi_err(chg, "Unknown message opcode: %d\n", hdr->opcode);

	return 0;
}

static void oem_state_cb(void *priv, enum pmic_glink_state state)
{
	struct qti_charger *chg = priv;

	mmi_dbg(chg, "state: %d\n", state);

	atomic_set(&chg->state, state);

	switch (state) {
	case PMIC_GLINK_STATE_DOWN:
	case PMIC_GLINK_STATE_UP:
		schedule_work(&chg->setup_work);
		break;
	default:
		break;
	}
}

static int qti_charger_write(struct qti_charger *chg, u32 property,
			       const void *val, size_t val_len)
{
	struct oem_write_buf_req_msg oem_buf = { { 0 } };
	int rc;

	if (val_len > (OEM_PROPERTY_DATA_SIZE * sizeof(u32))) {
		mmi_err(chg, "Incorrect data length %zu for property: %u\n",
						val_len, property);
		return -EINVAL;
	}

	if (atomic_read(&chg->state) == PMIC_GLINK_STATE_DOWN) {
		mmi_err(chg, "ADSP glink state is down\n");
		return -ENOTCONN;
	}

	memset(&oem_buf, 0, sizeof(oem_buf));
	oem_buf.hdr.owner = MSG_OWNER_OEM;
	oem_buf.hdr.type = MSG_TYPE_REQ_RESP;
	oem_buf.hdr.opcode = OEM_WRITE_BUF_REQ;
	oem_buf.oem_property_id = property;
	oem_buf.data_size = val_len;
	memcpy(oem_buf.buf, val, val_len);

	mutex_lock(&chg->write_lock);
	reinit_completion(&chg->write_ack);

	mmi_dbg(chg, "Start data write for property: %u, len=%zu\n",
		property, val_len);

	rc = pmic_glink_write(chg->client, &oem_buf,
					sizeof(oem_buf));
	if (rc < 0) {
		mmi_err(chg, "Error in sending message rc=%d on property: %u\n",
						rc, property);
		goto out;
	}

	rc = wait_for_completion_timeout(&chg->write_ack,
				msecs_to_jiffies(OEM_WAIT_TIME_MS));
	if (!rc) {
		mmi_err(chg, "timed out on property: %u\n", property);
		rc = -ETIMEDOUT;
		goto out;
	} else {
		rc = 0;
		bm_ulog_print_log(OEM_BM_ULOG_SIZE);
	}
out:
	mmi_dbg(chg, "Complete data write for property: %u\n", property);
	mutex_unlock(&chg->write_lock);
	return rc;
}

static int qti_charger_read(struct qti_charger *chg, u32 property,
			       void *val, size_t val_len)
{
	struct oem_read_buf_req_msg oem_buf = { { 0 } };
	int rc;

	if (val_len > (OEM_PROPERTY_DATA_SIZE * sizeof(u32))) {
		mmi_err(chg, "Incorrect data length %zu for property: %u\n",
						val_len, property);
		return -EINVAL;
	}

	if (atomic_read(&chg->state) == PMIC_GLINK_STATE_DOWN) {
		mmi_err(chg, "ADSP glink state is down\n");
		return -ENOTCONN;
	}

	oem_buf.hdr.owner = MSG_OWNER_OEM;
	oem_buf.hdr.type = MSG_TYPE_REQ_RESP;
	oem_buf.hdr.opcode = OEM_READ_BUF_REQ;
	oem_buf.oem_property_id = property;
	oem_buf.data_size = val_len;

	mutex_lock(&chg->read_lock);
	reinit_completion(&chg->read_ack);

	mmi_dbg(chg, "Start data read for property: %u, len=%zu\n",
		property, val_len);

	rc = pmic_glink_write(chg->client, &oem_buf,
					sizeof(oem_buf));
	if (rc < 0) {
		mmi_err(chg, "Error in sending message rc=%d on property: %u\n",
						rc, property);
		goto out;
	}

	rc = wait_for_completion_timeout(&chg->read_ack,
				msecs_to_jiffies(OEM_WAIT_TIME_MS));
	if (!rc) {
		mmi_err(chg, "timed out on property: %u\n", property);
		rc = -ETIMEDOUT;
		goto out;
	} else {
		rc = 0;
	}

	if (!atomic_read(&chg->rx_valid)) {
		rc = -ENODATA;
		goto out;
	}

	if (chg->rx_buf.data_size != val_len) {
		mmi_err(chg, "Invalid data size %u, on property: %u\n",
				chg->rx_buf.data_size, property);
		rc = -ENODATA;
		goto out;
	}

	memcpy(val, chg->rx_buf.buf, val_len);
	atomic_set(&chg->rx_valid, 0);
out:
	mmi_dbg(chg, "Complete data read for property: %u\n", property);
	mutex_unlock(&chg->read_lock);

	return rc;
}

int qti_charger_set_property(u32 property, const void *val, size_t val_len)
{
	struct qti_charger *chg = this_chip;

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	return qti_charger_write(chg, property, val, val_len);
}
EXPORT_SYMBOL(qti_charger_set_property);

int qti_charger_get_property(u32 property, void *val, size_t val_len)
{
	struct qti_charger *chg = this_chip;

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	return qti_charger_read(chg, property, val, val_len);
}
EXPORT_SYMBOL(qti_charger_get_property);

int qti_charger_register_notifier(struct notifier_block *nb)
{
        return blocking_notifier_chain_register(&qti_chg_notifier_list, nb);
}
EXPORT_SYMBOL(qti_charger_register_notifier);

int qti_charger_unregister_notifier(struct notifier_block *nb)
{
        return blocking_notifier_chain_unregister(&qti_chg_notifier_list, nb);
}
EXPORT_SYMBOL(qti_charger_unregister_notifier);

static int qti_charger_get_batt_info(void *data, struct mmi_battery_info *batt_info)
{
	int rc;
	struct qti_charger *chg = data;
	int batt_status = chg->batt_info.batt_status;

	rc = qti_charger_read(chg, OEM_PROP_BATT_INFO,
				&chg->batt_info,
				sizeof(struct battery_info));
	if (rc)
		return rc;

	if (chg->chg_cfg.full_charged)
		chg->batt_info.batt_status = POWER_SUPPLY_STATUS_FULL;

	chg->batt_info.batt_ma /= 1000;
	chg->batt_info.batt_mv /= 1000;
        chg->batt_info.batt_soc /= 100;
	chg->batt_info.batt_temp /= 100;
	memcpy(batt_info, &chg->batt_info, sizeof(struct mmi_battery_info));

	if (batt_status != chg->batt_info.batt_status) {
		bm_ulog_print_log(OEM_BM_ULOG_SIZE);
	}

	return rc;
}
#ifdef WIRELESS_CPS4035B
void qti_wireless_charge_dump_info(struct qti_charger *chg, struct wls_dump wls_info)
{
	mmi_info(chg, "Wireless dump info -1: CHIP_ID: 0x%04x, MTP_FW_VER: 0x%04x, IRQ STATUS: 0x%04x, "
		"SYS_MODE:  RX/TX %d, OP_MODE:  BPP/EPP 0x%x, RX_FOP: %dkHz, RX_VOUT: %dmV, "
		"RX_VRECT: %dmV, RX_IRECT: %dmV, RX_NEG_POWER: %dw ",
		wls_info.chip_id,
		wls_info.mtp_fw_ver,
		wls_info.irq_status,
		wls_info.sys_mode,
		wls_info.op_mode,
		wls_info.rx_fop,
		wls_info.rx_vout_mv,
		wls_info.rx_vrect_mv,
		wls_info.rx_irect_ma,
		wls_info.rx_neg_power);

	mmi_info(chg, "Wireless dump info -2: TX_IIN: %dmA, TX_VIN: %dmV, TX_VRECT: %dmV, "
		"TX_DET_RX_POWER: %dmW, TX_POWER: %dmW, POWER_LOSS: %dmW, TX_FOD: %d, ",
		wls_info.tx_iin_ma,
		wls_info.tx_vin_mv,
		wls_info.tx_vrect_mv,
		wls_info.tx_det_rx_power,
		wls_info.tx_power,
		wls_info.power_loss,
		(wls_info.irq_status & (0x01<<12)) ? 1 : 0);

	mmi_info(chg, "Wireless dump info -3: rx_ept: %d, rx_ce: %d, "
		"rx_rp: %d, rx_dietemp: %d, USB_OTG: %d, WLS_BOOST: %d, WLS_ICL_MA: %dmA, WLS_ICL_THERM_MA: %dmA",
		wls_info.rx_ept,
		wls_info.rx_ce,
		wls_info.rx_rp,
		wls_info.rx_dietemp,
		wls_info.usb_otg,
		wls_info.wls_boost,
		wls_info.wls_icl_ma,
		wls_info.wls_icl_therm_ma);
}
#else
void qti_wireless_charge_dump_info(struct qti_charger *chg, struct wls_dump wls_info)
{
	mmi_info(chg, "Wireless dump info -1: CHIP_ID: 0x%04x, MTP_FW_VER: 0x%04x, IRQ STATUS: 0x%04x, "
		"SYS_MODE:  RX/TX %d, OP_MODE:  BPP/EPP 0x%x, RX_FOP: %dkHz, RX_VOUT: %dmV, "
		"RX_VRECT: %dmV, RX_IRECT: %dmV, RX_NEG_POWER: %dw ",
		wls_info.chip_id,
		wls_info.mtp_fw_ver,
		wls_info.irq_status,
		wls_info.sys_mode,
		wls_info.op_mode,
		wls_info.rx_fop,
		wls_info.rx_vout_mv,
		wls_info.rx_vrect_mv,
		wls_info.rx_irect_ma,
		wls_info.rx_neg_power);

	mmi_info(chg, "Wireless dump info -2: TX_IIN: %dmA, TX_VIN: %dmV, TX_VRECT: %dmV, "
		"TX_DET_RX_POWER: %dmW, TX_POWER: %dmW, POWER_LOSS: %dmW, TX_FOD: %d, ",
		wls_info.tx_iin_ma,
		wls_info.tx_vin_mv,
		wls_info.tx_vrect_mv,
		wls_info.tx_det_rx_power,
		wls_info.tx_power,
		wls_info.power_loss,
		(wls_info.irq_status & (0x01<<12)) ? 1 : 0);

	mmi_info(chg, "Wireless dump info -3: FOLIO_MODE: %d, PEN_STATUS: %d, "
		"PEN_SOC: %d, PEN_ERROR: %d, USB_OTG: %d, WLS_BOOST: %d, WLS_ICL_MA: %dmA, WLS_ICL_THERM_MA: %dmA",
		wls_info.folio_mode,
		wls_info.pen_status,
		wls_info.pen_soc,
		wls_info.pen_error,
		wls_info.usb_otg,
		wls_info.wls_boost,
		wls_info.wls_icl_ma,
		wls_info.wls_icl_therm_ma);
}
#endif

static int qti_charger_get_chg_info(void *data, struct mmi_charger_info *chg_info)
{
	int rc;
	struct qti_charger *chg = data;
	struct wls_dump wls_info;

	rc = qti_charger_read(chg, OEM_PROP_CHG_INFO,
				&chg->chg_info,
				sizeof(struct charger_info));
	if (rc)
		return rc;

	rc = qti_charger_read(chg, OEM_PROP_LPD_INFO,
				&chg->lpd_info,
				sizeof(struct lpd_info));
	if (rc) {
		rc = 0;
		memset(&chg->lpd_info, 0, sizeof(struct lpd_info));
	}
	mmi_info(chg, "LPD: present=%d, rsbu1=%d, rsbu2=%d\n",
			chg->lpd_info.lpd_present,
			chg->lpd_info.lpd_rsbu1,
			chg->lpd_info.lpd_rsbu2);

	chg->chg_info.chrg_mv /= 1000;
	chg->chg_info.chrg_ma /= 1000;
	if (!chg->chg_info.chrg_present &&
	    chg->chg_info.chrg_type != 0)
		chg->chg_info.chrg_present = 1;

	chg->chg_info.vbus_present = chg->chg_info.chrg_mv > VBUS_MIN_MV;
	chg->chg_info.lpd_present = chg->lpd_info.lpd_present;
	memcpy(chg_info, &chg->chg_info, sizeof(struct mmi_charger_info));

	rc =  qti_charger_read(chg, OEM_PROP_WLS_DUMP_INFO,
				&wls_info,
				sizeof(struct wls_dump));

	qti_wireless_charge_dump_info(chg, wls_info);

	bm_ulog_print_log(OEM_BM_ULOG_SIZE);

	return rc;
}

static int qti_charger_config_charge(void *data, struct mmi_charger_cfg *config)
{
	int rc;
	u32 value;
	struct qti_charger *chg = data;

	/* configure the charger if changed */
	if (config->target_fv != chg->chg_cfg.target_fv) {
		value = config->target_fv * 1000;
		rc = qti_charger_write(chg, OEM_PROP_CHG_FV,
					&value,
					sizeof(value));
		if (!rc)
			chg->chg_cfg.target_fv = config->target_fv;
	}
	if (config->target_fcc != chg->chg_cfg.target_fcc) {
		value = config->target_fcc * 1000;
		rc = qti_charger_write(chg, OEM_PROP_CHG_FCC,
					&value,
					sizeof(value));
		if (!rc)
			chg->chg_cfg.target_fcc = config->target_fcc;
	}
	if (config->charger_suspend != chg->chg_cfg.charger_suspend) {
		value = config->charger_suspend;
		rc = qti_charger_write(chg, OEM_PROP_CHG_SUSPEND,
					&value,
					sizeof(value));
		if (!rc)
			chg->chg_cfg.charger_suspend = config->charger_suspend;
	}
	if (config->charging_disable != chg->chg_cfg.charging_disable) {
		value = config->charging_disable;
		rc = qti_charger_write(chg, OEM_PROP_CHG_DISABLE,
					&value,
					sizeof(value));
		if (!rc)
			chg->chg_cfg.charging_disable = config->charging_disable;
	}

	if (config->taper_kickoff != chg->chg_cfg.taper_kickoff) {
		chg->chg_cfg.taper_kickoff = config->taper_kickoff;
		chg->chrg_taper_cnt = 0;
	}

	if (config->full_charged != chg->chg_cfg.full_charged) {
		chg->chg_cfg.full_charged = config->full_charged;
	}

	if (config->chrg_iterm != chg->chg_cfg.chrg_iterm) {
		value = config->chrg_iterm;
		rc = qti_charger_write(chg, OEM_PROP_CHG_ITERM,
					&value,
					sizeof(value));
		if (!rc)
			chg->chg_cfg.chrg_iterm = config->chrg_iterm;
	}
	if (config->fg_iterm != chg->chg_cfg.fg_iterm) {
		value = config->fg_iterm;
		rc = qti_charger_write(chg, OEM_PROP_CHG_FG_ITERM,
					&value,
					sizeof(value));
		if (!rc)
			chg->chg_cfg.fg_iterm = config->fg_iterm;
	}

	if (config->charging_reset != chg->chg_cfg.charging_reset) {
		if (config->charging_reset) {
			value = 1;
			rc = qti_charger_write(chg, OEM_PROP_CHG_DISABLE,
						&value,
						sizeof(value));
			msleep(200);
			value = 0;
			rc = qti_charger_write(chg, OEM_PROP_CHG_DISABLE,
						&value,
						sizeof(value));
		}
		chg->chg_cfg.charging_reset = config->charging_reset;
	}

	return 0;
}

#define TAPER_COUNT 2
static bool qti_charger_is_charge_tapered(void *data, int tapered_ma)
{
	bool is_tapered = false;
	struct qti_charger *chg = data;

	if (abs(chg->batt_info.batt_ma) <= tapered_ma) {
		if (chg->chrg_taper_cnt >= TAPER_COUNT) {
			is_tapered = true;
			chg->chrg_taper_cnt = 0;
		} else
			chg->chrg_taper_cnt++;
	} else
		chg->chrg_taper_cnt = 0;

	return is_tapered;
}

static bool qti_charger_is_charge_halt(void *data)
{
	struct qti_charger *chg = data;

	if (chg->batt_info.batt_status == POWER_SUPPLY_STATUS_NOT_CHARGING ||
	    chg->batt_info.batt_status == POWER_SUPPLY_STATUS_FULL)
		return true;

	return false;
}

static void qti_charger_set_constraint(void *data,
			struct mmi_charger_constraint *constraint)
{
	int rc;
	u32 value;
	struct qti_charger *chg = data;

	if (constraint->demo_mode != chg->constraint.demo_mode) {
		value = constraint->demo_mode;
		rc = qti_charger_write(chg, OEM_PROP_DEMO_MODE,
					&value,
					sizeof(value));
		if (!rc)
			chg->constraint.demo_mode = constraint->demo_mode;
	}

	if (constraint->factory_version != chg->constraint.factory_version) {
		value = constraint->factory_version;
		rc = qti_charger_write(chg, OEM_PROP_FACTORY_VERSION,
					&value,
					sizeof(value));
		if (!rc)
			chg->constraint.factory_version = constraint->factory_version;
	}

	if (constraint->factory_mode != chg->constraint.factory_mode) {
		value = constraint->factory_mode;
		rc = qti_charger_write(chg, OEM_PROP_FACTORY_MODE,
					&value,
					sizeof(value));
		if (!rc)
			chg->constraint.factory_mode = constraint->factory_mode;
	}

	if (constraint->dcp_pmax != chg->constraint.dcp_pmax) {
		value = constraint->dcp_pmax;
		rc = qti_charger_write(chg, OEM_PROP_CHG_BC_PMAX,
					&value,
					sizeof(value));
		if (!rc)
			chg->constraint.dcp_pmax = constraint->dcp_pmax;
	}

	if (constraint->hvdcp_pmax != chg->constraint.hvdcp_pmax) {
		value = constraint->hvdcp_pmax;
		rc = qti_charger_write(chg, OEM_PROP_CHG_QC_PMAX,
					&value,
					sizeof(value));
		if (!rc)
			chg->constraint.hvdcp_pmax = constraint->hvdcp_pmax;
	}

	if (constraint->pd_pmax != chg->constraint.pd_pmax) {
		value = constraint->pd_pmax;
		rc = qti_charger_write(chg, OEM_PROP_CHG_PD_PMAX,
					&value,
					sizeof(value));
		if (!rc)
			chg->constraint.pd_pmax = constraint->pd_pmax;
	}

	if (constraint->wls_pmax != chg->constraint.wls_pmax) {
		value = constraint->wls_pmax;
		rc = qti_charger_write(chg, OEM_PROP_CHG_WLS_PMAX,
					&value,
					sizeof(value));
		if (!rc)
			chg->constraint.wls_pmax = constraint->wls_pmax;
	}
}

static int qti_charger_write_profile(struct qti_charger *chg)
{
	int rc;
	char *data;
	int offset;
	int max_size = OEM_PROPERTY_DATA_SIZE * sizeof(u32);

	rc = qti_charger_write(chg, OEM_PROP_CHG_PROFILE_INFO,
				&chg->profile_info,
				sizeof(struct charger_profile_info));
	if (rc) {
		mmi_err(chg, "qti charger write profile info failed, rc=%d\n", rc);
		return rc;
	}

	if (!chg->profile_data)
		return 0;

	offset = 0;
	data = (char *)chg->profile_data;
	while (offset < chg->profile_info.data_size) {
		if ((chg->profile_info.data_size - offset) > max_size) {
			rc = qti_charger_write(chg,
					OEM_PROP_CHG_PROFILE_DATA,
					data + offset,
					max_size);
			offset += max_size;
		} else {
			rc = qti_charger_write(chg,
					OEM_PROP_CHG_PROFILE_DATA,
					data + offset,
					chg->profile_info.data_size - offset);
			offset += chg->profile_info.data_size - offset;
		}
		if (rc) {
			mmi_err(chg, "qti charger write profile data failed, rc=%d\n", rc);
			break;
		}
	}

	return rc;
}

static ssize_t tcmd_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	unsigned long r;
	unsigned long mode;
	struct qti_charger *chg = dev_get_drvdata(dev);

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	r = kstrtoul(buf, 0, &mode);
	if (r) {
		mmi_err(chg, "Invalid TCMD = %lu\n", mode);
		return -EINVAL;
	}

	r = qti_charger_write(chg, OEM_PROP_TCMD,
				&mode,
				sizeof(mode));

	return r ? r : count;
}

static ssize_t tcmd_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int data;
	struct qti_charger *chg = dev_get_drvdata(dev);

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	qti_charger_read(chg, OEM_PROP_TCMD,
				&data,
				sizeof(int));

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", data);
}


static DEVICE_ATTR(tcmd, 0664,
		tcmd_show,
		tcmd_store);

static ssize_t force_pmic_icl_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	unsigned long r;
	unsigned long pmic_icl;
	struct qti_charger *chg = dev_get_drvdata(dev);

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	r = kstrtoul(buf, 0, &pmic_icl);
	if (r) {
		mmi_err(chg, "Invalid TCMD = %lu\n", pmic_icl);
		return -EINVAL;
	}

	r = qti_charger_write(chg, OEM_PROP_PMIC_ICL,
				&pmic_icl,
				sizeof(pmic_icl));

	return r ? r : count;
}

static ssize_t force_pmic_icl_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int data;
	struct qti_charger *chg = dev_get_drvdata(dev);

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	qti_charger_read(chg, OEM_PROP_PMIC_ICL,
				&data,
				sizeof(int));

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", data);
}


static DEVICE_ATTR(force_pmic_icl, 0664,
		force_pmic_icl_show,
		force_pmic_icl_store);

static ssize_t force_wls_en_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	unsigned long r;
	unsigned long wls_en;
	struct qti_charger *chg = dev_get_drvdata(dev);

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	r = kstrtoul(buf, 0, &wls_en);
	if (r) {
		mmi_err(chg, "Invalid TCMD = %lu\n", wls_en);
		return -EINVAL;
	}

	r = qti_charger_write(chg, OEM_PROP_WLS_EN,
				&wls_en,
				sizeof(wls_en));

	return r ? r : count;
}

static ssize_t force_wls_en_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int data;
	struct qti_charger *chg = dev_get_drvdata(dev);

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	qti_charger_read(chg, OEM_PROP_WLS_EN,
				&data,
				sizeof(int));

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", data);
}

static DEVICE_ATTR(force_wls_en, 0664,
		force_wls_en_show,
		force_wls_en_store);

static ssize_t force_usb_suspend_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	unsigned long r;
	unsigned long usb_suspend;
	struct qti_charger *chg = dev_get_drvdata(dev);

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	r = kstrtoul(buf, 0, &usb_suspend);
	if (r) {
		mmi_err(chg, "Invalid TCMD = %lu\n", usb_suspend);
		return -EINVAL;
	}

	r = qti_charger_write(chg, OEM_PROP_USB_SUSPEND,
				&usb_suspend,
				sizeof(usb_suspend));

	return r ? r : count;
}

static ssize_t force_usb_suspend_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int data;
	struct qti_charger *chg = dev_get_drvdata(dev);

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	qti_charger_read(chg, OEM_PROP_USB_SUSPEND,
				&data,
				sizeof(int));

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", data);
}

static DEVICE_ATTR(force_usb_suspend, 0664,
		force_usb_suspend_show,
		force_usb_suspend_store);

static ssize_t force_wls_volt_max_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	unsigned long r;
	unsigned long wls_volt_max;
	struct qti_charger *chg = dev_get_drvdata(dev);

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	r = kstrtoul(buf, 0, &wls_volt_max);
	if (r) {
		mmi_err(chg, "Invalid TCMD = %lu\n", wls_volt_max);
		return -EINVAL;
	}

	r = qti_charger_write(chg, OEM_PROP_WLS_VOLT_MAX,
				&wls_volt_max,
				sizeof(wls_volt_max));

	return r ? r : count;
}

static ssize_t force_wls_volt_max_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int data;
	struct qti_charger *chg = dev_get_drvdata(dev);

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	qti_charger_read(chg, OEM_PROP_WLS_VOLT_MAX,
				&data,
				sizeof(int));

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", data);
}

static DEVICE_ATTR(force_wls_volt_max, 0664,
		force_wls_volt_max_show,
		force_wls_volt_max_store);

static ssize_t force_wls_curr_max_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	unsigned long r;
	unsigned long wls_curr_max;
	struct qti_charger *chg = dev_get_drvdata(dev);

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	r = kstrtoul(buf, 0, &wls_curr_max);
	if (r) {
		mmi_err(chg, "Invalid TCMD = %lu\n", wls_curr_max);
		return -EINVAL;
	}

	r = qti_charger_write(chg, OEM_PROP_WLS_CURR_MAX,
				&wls_curr_max,
				sizeof(wls_curr_max));

	return r ? r : count;
}

static ssize_t force_wls_curr_max_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int data;
	struct qti_charger *chg = dev_get_drvdata(dev);

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	qti_charger_read(chg, OEM_PROP_WLS_CURR_MAX,
				&data,
				sizeof(int));

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", data);
}

static DEVICE_ATTR(force_wls_curr_max, 0664,
		force_wls_curr_max_show,
		force_wls_curr_max_store);

static ssize_t wireless_chip_id_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int data;
	struct qti_charger *chg = dev_get_drvdata(dev);

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	qti_charger_read(chg, OEM_PROP_WLS_CHIP_ID,
				&data,
				sizeof(int));

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "0x%04x\n", data);
}

static DEVICE_ATTR(wireless_chip_id, S_IRUGO,
		wireless_chip_id_show,
		NULL);

static int fod_gain_store(struct qti_charger *chip, const char *buf,
	u32 *fod_array)
{
	int i = 0, ret = 0, sum = 0;
	char *buffer;
	unsigned int temp;

	buffer = (char *)buf;

	for (i = 0; i < FOD_GAIN_MAX_LEN; i++) {
		ret = sscanf((const char *)buffer, "%x,%s", &temp, buffer);
		fod_array[i] = temp;
		sum++;
		if (ret != 2)
			break;
	}

	if (sum != FOD_GAIN_MAX_LEN) {
		pr_err("QTI: fod_gain array len err %d\n", sum);
		return -ENODEV;
	}

	ret = qti_charger_write(chip, OEM_PROP_WLS_RX_FOD_GAIN,
				fod_array,
				sizeof(struct fod_gain));
	if (ret) {
		mmi_err(chip, "qti charger write wls rx fod gain failed, rc=%d\n", ret);
		return ret;
	}


	return sum;
}

static ssize_t wls_fod_gain_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	struct qti_charger *chg = dev_get_drvdata(dev);

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	fod_gain_store(chg, buf, chg->rx_fod_gain.fod_array_gain);
	return count;
}


static ssize_t wls_fod_gain_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int count = 0, i = 0;
	struct qti_charger *chg = dev_get_drvdata(dev);

	for (i = 0; i < FOD_GAIN_MAX_LEN; i++) {
		count += scnprintf(buf+count, CHG_SHOW_MAX_SIZE,
				"0x%02x ", chg->rx_fod_gain.fod_array_gain[i]);
		if (i == FOD_GAIN_MAX_LEN - 1)
			count += scnprintf(buf+count, CHG_SHOW_MAX_SIZE, "\n");
	}

	return count;
}

static DEVICE_ATTR(wls_fod_gain, 0664,
		wls_fod_gain_show,
		wls_fod_gain_store);

static int fod_curr_store(struct qti_charger *chip, const char *buf,
	u32 *fod_array)
{
	int i = 0, ret = 0, sum = 0;
	char *buffer;
	unsigned int temp;

	buffer = (char *)buf;

	for (i = 0; i < FOD_CURR_MAX_LEN; i++) {
		ret = sscanf((const char *)buffer, "%x,%s", &temp, buffer);
		fod_array[i] = temp;
		sum++;
		if (ret != 2)
			break;
	}

	if (sum != FOD_CURR_MAX_LEN) {
		pr_err("QTI: fod_curr array len err %d\n", sum);
		return -ENODEV;
	}

	ret = qti_charger_write(chip, OEM_PROP_WLS_RX_FOD_CURR,
				fod_array,
				sizeof(struct fod_curr));
	if (ret) {
		mmi_err(chip, "qti charger write wls rx fod curr failed, rc=%d\n", ret);
		return ret;
	}

	return sum;
}

static ssize_t wls_fod_curr_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	struct qti_charger *chg = dev_get_drvdata(dev);

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	fod_curr_store(chg, buf, chg->rx_fod_curr.fod_array_curr);
	return count;
}

static ssize_t wls_fod_curr_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int count = 0, i = 0;
	struct qti_charger *chg = dev_get_drvdata(dev);

	for (i = 0; i < FOD_CURR_MAX_LEN; i++) {
		count += scnprintf(buf+count, CHG_SHOW_MAX_SIZE,
				"0x%02x ", chg->rx_fod_curr.fod_array_curr[i]);
		if (i == FOD_CURR_MAX_LEN - 1)
			count += scnprintf(buf+count, CHG_SHOW_MAX_SIZE, "\n");
	}

	return count;
}

static DEVICE_ATTR(wls_fod_curr, 0664,
		wls_fod_curr_show,
		wls_fod_curr_store);

static ssize_t addr_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	unsigned long r;
	u32 addr;
	struct qti_charger *chg = dev_get_drvdata(dev);

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	r = kstrtou32(buf, 0, &addr);
	if (r) {
		mmi_err(chg, "Invalid reg_address = 0x%x\n", addr);
		return -EINVAL;
	}

	r = qti_charger_write(chg, OEM_PROP_REG_ADDRESS,
				&addr,
				sizeof(addr));

	return r ? r : count;
}

static DEVICE_ATTR_WO(addr);

static ssize_t data_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	unsigned long r;
	u32 data;
	struct qti_charger *chg = dev_get_drvdata(dev);

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	r = kstrtou32(buf, 0, &data);
	if (r) {
		mmi_err(chg, "Invalid reg_data = 0x%x\n", data);
		return -EINVAL;
	}

	r = qti_charger_write(chg, OEM_PROP_REG_DATA,
				&data,
				sizeof(data));

	return r ? r : count;
}

static ssize_t data_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	u32 data;
	struct qti_charger *chg = dev_get_drvdata(dev);

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	qti_charger_read(chg, OEM_PROP_REG_DATA,
				&data,
				sizeof(data));

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%x\n", data);
}

static DEVICE_ATTR(data, 0664,
		data_show,
		data_store);

static ssize_t tx_mode_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long r;
	unsigned long tx_mode;
	struct qti_charger *chg = this_chip;

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	r = kstrtoul(buf, 0, &tx_mode);
	if (r) {
		pr_err("Invalid tx_mode = %lu\n", tx_mode);
		return -EINVAL;
	}

	r = qti_charger_write(chg, OEM_PROP_WLS_TX_MODE,
				&tx_mode,
				sizeof(tx_mode));
	chg->tx_mode = tx_mode;
	if (chg->wls_psy)
		sysfs_notify(&chg->wls_psy->dev.parent->kobj, NULL, "tx_mode");

	return r ? r : count;
}

static ssize_t tx_mode_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct qti_charger *chg = this_chip;

	if (!chg) {
		pr_err("PEN: chip not valid\n");
		return -ENODEV;
	}

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", chg->tx_mode);
}
static DEVICE_ATTR(tx_mode, S_IRUGO|S_IWUSR, tx_mode_show, tx_mode_store);

static ssize_t rx_connected_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct qti_charger *chg = this_chip;

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", chg->rx_connected);
}

static DEVICE_ATTR(rx_connected, S_IRUGO,
		rx_connected_show,
		NULL);

static ssize_t wls_input_current_limit_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long r;
	unsigned long wls_curr_max;
	struct qti_charger *chg = this_chip;

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	r = kstrtoul(buf, 0, &wls_curr_max);
	if (r) {
		mmi_err(chg, "Invalid TCMD = %lu\n", wls_curr_max);
		return -EINVAL;
	}

	r = qti_charger_write(chg, OEM_PROP_WLS_CURR_MAX,
				&wls_curr_max,
				sizeof(wls_curr_max));

	chg->wls_curr_max = wls_curr_max;
	return r ? r : count;
}

static ssize_t wls_input_current_limit_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct qti_charger *chg = this_chip;

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", chg->wls_curr_max);
}
static DEVICE_ATTR(wls_input_current_limit, S_IRUGO|S_IWUSR, wls_input_current_limit_show, wls_input_current_limit_store);

static ssize_t folio_mode_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long r;
	unsigned long folio_mode;
	struct qti_charger *chg = this_chip;

	if (!chg) {
		pr_err("QTI: chip not valid\n");
		return -ENODEV;
	}

	r = kstrtoul(buf, 0, &folio_mode);
	if (r) {
		pr_err("Invalid folio_mode = %lu\n", folio_mode);
		return -EINVAL;
	}

	r = qti_charger_write(chg, OEM_PROP_WLS_FOLIO_MODE,
				&folio_mode,
				sizeof(folio_mode));
	chg->folio_mode = folio_mode;

	return r ? r : count;
}

static ssize_t folio_mode_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct qti_charger *chg = this_chip;

	if (!chg) {
		pr_err("PEN: chip not valid\n");
		return -ENODEV;
	}

	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "%d\n", chg->folio_mode);
}
static DEVICE_ATTR(folio_mode, S_IRUGO|S_IWUSR, folio_mode_show, folio_mode_store);

//ATTRIBUTE_GROUPS(qti_charger);
#define TX_INT_FOD      (0x01<<12)
#ifdef WIRELESS_CPS4035B
static int show_wls_dump_info(struct seq_file *m, void *data)
{
	struct qti_charger *chip = m->private;
	struct wls_dump wls_info;

	qti_charger_read(chip, OEM_PROP_WLS_DUMP_INFO,
				&wls_info,
				sizeof(struct wls_dump));

	seq_printf(m, "CHIP_ID: 0x%04x\n", wls_info.chip_id);

	seq_printf(m, "MTP_FW_VER: 0x%04x\n", wls_info.mtp_fw_ver);

	seq_printf(m, "IRQ STATUS: 0x%04x\n", wls_info.irq_status);

	seq_printf(m, "SYS_MODE:  RX/TX %d\n", wls_info.sys_mode);

	seq_printf(m, "OP_MODE:  BPP/EPP/Moto50W 0x%x\n", wls_info.op_mode);

	seq_printf(m, "RX_FOP:   %dkHz\n", wls_info.rx_fop);

	seq_printf(m, "RX_VOUT: %dmV\n",  wls_info.rx_vout_mv);

	seq_printf(m, "RX_VRECT: %dmV\n",  wls_info.rx_vrect_mv);

	seq_printf(m, "RX_IRECT: %dmV\n",  wls_info.rx_irect_ma);

	seq_printf(m, "RX_EPT: 0x%04x\n",  wls_info.rx_ept);

	seq_printf(m, "RX_CE: %d\n",  wls_info.rx_ce);

	seq_printf(m, "RX_RP: %d\n",  wls_info.rx_rp);

	seq_printf(m, "RX_DieTemp: %dC\n",  wls_info.rx_dietemp);

	seq_printf(m, "RX_NEG_POWER: %dw\n",  wls_info.rx_neg_power);

	seq_printf(m, "TX_IIN: %dmA\n",  wls_info.tx_iin_ma);

	seq_printf(m, "TX_VIN: %dmV\n",  wls_info.tx_vin_mv);

	seq_printf(m, "TX_VRECT: %dmV\n",  wls_info.tx_vrect_mv);

	seq_printf(m, "TX_DET_RX_POWER: %dmW\n",  wls_info.tx_det_rx_power);

	seq_printf(m, "TX_POWER: %dmW\n",  wls_info.tx_power);

	seq_printf(m, "POWER_LOSS: %dmW\n",  wls_info.power_loss);

	seq_printf(m, "TX_FOD: %d\n",  (wls_info.irq_status & TX_INT_FOD) ? 1 : 0);

	seq_printf(m, "USB_OTG: %d\n",  wls_info.usb_otg);

	seq_printf(m, "WLS_BOOST: %d\n",  wls_info.wls_boost);

	seq_printf(m, "WLS_ICL_MA: %d\n",  wls_info.wls_icl_ma);

	seq_printf(m, "WLS_ICL_THERM_MA: %d\n",  wls_info.wls_icl_therm_ma);

	return 0;
}
#else
static int show_wls_dump_info(struct seq_file *m, void *data)
{
	struct qti_charger *chip = m->private;
	struct wls_dump wls_info;

	qti_charger_read(chip, OEM_PROP_WLS_DUMP_INFO,
				&wls_info,
				sizeof(struct wls_dump));

	seq_printf(m, "CHIP_ID: 0x%04x\n", wls_info.chip_id);

	seq_printf(m, "MTP_FW_VER: 0x%04x\n", wls_info.mtp_fw_ver);

	seq_printf(m, "IRQ STATUS: 0x%04x\n", wls_info.irq_status);

	seq_printf(m, "SYS_MODE:  RX/TX %d\n", wls_info.sys_mode);

	seq_printf(m, "OP_MODE:  BPP/EPP 0x%x\n", wls_info.op_mode);

	seq_printf(m, "RX_FOP:   %dkHz\n", wls_info.rx_fop);

	seq_printf(m, "RX_VOUT: %dmV\n",  wls_info.rx_vout_mv);

	seq_printf(m, "RX_VRECT: %dmV\n",  wls_info.rx_vrect_mv);

	seq_printf(m, "RX_IRECT: %dmV\n",  wls_info.rx_irect_ma);

	seq_printf(m, "RX_NEG_POWER: %dw\n",  wls_info.rx_neg_power);

	seq_printf(m, "TX_IIN: %dmA\n",  wls_info.tx_iin_ma);

	seq_printf(m, "TX_VIN: %dmV\n",  wls_info.tx_vin_mv);

	seq_printf(m, "TX_VRECT: %dmV\n",  wls_info.tx_vrect_mv);

	seq_printf(m, "TX_FOD_I: %d\n",  wls_info.tx_fod_I);

	seq_printf(m, "TX_FOD_II: %d\n",  wls_info.tx_fod_II);

	seq_printf(m, "TX_FOD_RP: %d\n",  wls_info.tx_fod_rp);

	seq_printf(m, "TX_DET_RX_POWER: %dmW\n",  wls_info.tx_det_rx_power);

	seq_printf(m, "TX_POWER: %dmW\n",  wls_info.tx_power);

	seq_printf(m, "POWER_LOSS: %dmW\n",  wls_info.power_loss);

	seq_printf(m, "TX_FOD: %d\n",  (wls_info.irq_status & TX_INT_FOD) ? 1 : 0);

	seq_printf(m, "FOLIO_MODE: %d\n",  wls_info.folio_mode);

	seq_printf(m, "PEN_STATUS: %d\n",  wls_info.pen_status);

	seq_printf(m, "PEN_SOC: %d\n",  wls_info.pen_soc);

	seq_printf(m, "PEN_ERROR: %d\n",  wls_info.pen_error);

	seq_printf(m, "USB_OTG: %d\n",  wls_info.usb_otg);

	seq_printf(m, "WLS_BOOST: %d\n",  wls_info.wls_boost);

	seq_printf(m, "WLS_ICL_MA: %d\n",  wls_info.wls_icl_ma);

	seq_printf(m, "WLS_ICL_THERM_MA: %d\n",  wls_info.wls_icl_therm_ma);

	return 0;
}
#endif

static int wls_dump_info_debugfs_open(struct inode *inode, struct file *file)
{
	struct qti_charger *chip = inode->i_private;

	return single_open(file, show_wls_dump_info, chip);
}

static const struct file_operations wls_dump_info_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= wls_dump_info_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void create_debugfs_entries(struct qti_charger *chip)
{
	struct dentry *ent;

	chip->debug_root = debugfs_create_dir("qti_glink_charger", NULL);
	if (!chip->debug_root) {
		mmi_err(chip, "Couldn't create debug dir\n");
		return;
	}

	ent = debugfs_create_file("wls_dump_info", S_IFREG | S_IRUGO,
				  chip->debug_root, chip,
				  &wls_dump_info_debugfs_ops);
	if (!ent)
		mmi_err(chip, "Couldn't create wls_dump_info debug file\n");
}


static int wireless_charger_notify_callback(struct notifier_block *nb,
		unsigned long event, void *data)
{
	struct qti_charger_notify_data *notify_data = data;
	struct qti_charger *chg = container_of(nb, struct qti_charger, wls_nb);

	if (notify_data->receiver != OEM_NOTIFY_RECEIVER_WLS_CHG) {
		pr_err("Skip mis-matched receiver: %#x\n", notify_data->receiver);
		return 0;
	}

        switch (event) {
        case NOTIFY_EVENT_WLS_RX_CONNECTED:
	/* RX connected update */
		if (notify_data->data[0] != chg->rx_connected) {
			if (chg->wls_psy) {
				pr_info("report rx_connected\n");
				sysfs_notify(&chg->wls_psy->dev.parent->kobj, NULL, "rx_connected");
			}
		}
		chg->rx_connected = notify_data->data[0];
	            break;
        case NOTIFY_EVENT_WLS_RX_OVERTEMP:
		break;
        case NOTIFY_EVENT_WLS_CHANGE:
		break;
        default:
		pr_err("Unknown wireless event: %#lx\n", event);
                break;
        }

	if (chg->wls_psy) {
		pr_info("wireless charger notify\n");
		power_supply_changed(chg->wls_psy);
	}

        return 0;
}


static void wireless_psy_init(struct qti_charger *chg)
{
	int rc;

	if (chg->wls_psy)
		return;

	chg->wls_psy = power_supply_get_by_name("wireless");
	if (!chg->wls_psy) {
		pr_err("No pen power supply found\n");
		return;
	}
	pr_info("wireless power supply is found\n");

	rc = device_create_file(chg->wls_psy->dev.parent,
				&dev_attr_tx_mode);
        if (rc)
		pr_err("couldn't create wireless tx mode\n");

	rc = device_create_file(chg->wls_psy->dev.parent,
				&dev_attr_rx_connected);
        if (rc)
		pr_err("couldn't create wireless rx_connected\n");

	rc = device_create_file(chg->wls_psy->dev.parent,
				&dev_attr_wls_input_current_limit);
        if (rc)
		pr_err("couldn't create wireless input current limit error\n");

	rc = device_create_file(chg->wls_psy->dev.parent,
				&dev_attr_folio_mode);
        if (rc)
		pr_err("couldn't create wireless folio mode error\n");

	chg->wls_nb.notifier_call = wireless_charger_notify_callback;
	rc = qti_charger_register_notifier(&chg->wls_nb);
	if (rc)
		pr_err("Failed to register notifier, rc=%d\n", rc);
}

static void wireless_psy_deinit(struct qti_charger *chg)
{
	if (!chg->wls_psy)
		return;

	device_remove_file(chg->wls_psy->dev.parent,
				&dev_attr_tx_mode);

	device_remove_file(chg->wls_psy->dev.parent,
				&dev_attr_rx_connected);

	device_remove_file(chg->wls_psy->dev.parent,
				&dev_attr_wls_input_current_limit);

	device_remove_file(chg->wls_psy->dev.parent,
				&dev_attr_folio_mode);
	qti_charger_unregister_notifier(&chg->wls_nb);

	power_supply_put(chg->wls_psy);
	chg->wls_psy = NULL;
}

/* Battery presence detection threshold on battery temperature */
#define BPD_TEMP_THRE -3000
static int battery_psy_get_prop(struct power_supply *psy,
		enum power_supply_property prop,
		union power_supply_propval *pval)
{
	int rc;
	struct battery_info info = {0};
	struct qti_charger *chg = power_supply_get_drvdata(psy);

	pval->intval = -ENODATA;

	rc = qti_charger_read(chg, OEM_PROP_BATT_INFO, &info,
				sizeof(struct battery_info));
	if (rc)
		return rc;

	switch (prop) {
	case POWER_SUPPLY_PROP_STATUS:
		pval->intval = info.batt_status;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		pval->intval = info.batt_temp > BPD_TEMP_THRE? 1 : 0;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		pval->intval = info.batt_uv;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		pval->intval = info.batt_ua;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		pval->intval = info.batt_soc / 100;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		pval->intval = info.batt_temp / 10;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		pval->intval = info.batt_full_uah;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		pval->intval = info.batt_design_uah;
		break;
	default:
		break;
	}

	return rc;
}

static int battery_psy_set_prop(struct power_supply *psy,
		enum power_supply_property prop,
		const union power_supply_propval *pval)
{
	struct qti_charger *chg = power_supply_get_drvdata(psy);

	switch (prop) {
	default:
		mmi_err(chg, "Not supported property: %d\n", prop);
		return -EINVAL;
	}

	return 0;
}

static enum power_supply_property battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
};

static const struct power_supply_desc batt_psy_desc = {
	.name			= "main_battery",
	.type			= POWER_SUPPLY_TYPE_MAINS,
	.properties		= battery_props,
	.num_properties		= ARRAY_SIZE(battery_props),
	.get_property		= battery_psy_get_prop,
	.set_property		= battery_psy_set_prop,
};

static int qti_charger_init(struct qti_charger *chg)
{
	int rc;
	u32 value;
	struct mmi_charger_driver *driver;

	if (chg->driver) {
		mmi_warn(chg, "qti charger has already inited\n");
		return 0;
	}

	value = mmi_is_factory_mode();
	rc = qti_charger_write(chg, OEM_PROP_FACTORY_MODE,
					&value,
					sizeof(value));
	if (rc) {
		mmi_err(chg, "qti charger set factory mode failed, rc=%d\n", rc);
		return rc;
	}
	chg->constraint.factory_mode = value;

	value = mmi_is_factory_version();
	rc = qti_charger_write(chg, OEM_PROP_FACTORY_VERSION,
					&value,
					sizeof(value));
	if (rc) {
		mmi_err(chg, "qti charger set factory ver failed, rc=%d\n", rc);
		return rc;
	}
	chg->constraint.factory_version = value;

	rc = qti_charger_write_profile(chg);
	if (rc) {
		mmi_err(chg, "qti charger set profile data failed, rc=%d\n", rc);
		return rc;
	}

	if (of_property_read_bool(chg->dev->of_node,
				"mmi,main-battery-enabled")) {
		struct power_supply_config psy_cfg = {};
		psy_cfg.drv_data = chg;
		psy_cfg.of_node = chg->dev->of_node;
		chg->batt_psy = devm_power_supply_register(chg->dev,
						&batt_psy_desc,
						&psy_cfg);
		if (IS_ERR(chg->batt_psy)) {
			rc = PTR_ERR(chg->batt_psy);
			chg->batt_psy = NULL;
			mmi_err(chg, "Failed to register main psy, rc=%d\n", rc);
			return rc;
		}
	}

	driver = devm_kzalloc(chg->dev,
				sizeof(struct mmi_charger_driver),
				GFP_KERNEL);
	if (!driver)
		return -ENOMEM;

	/* init driver */
	driver->name = chg->name;
	driver->dev = chg->dev;
	driver->data = chg;
	driver->get_batt_info = qti_charger_get_batt_info;
	driver->get_chg_info = qti_charger_get_chg_info;
	driver->config_charge = qti_charger_config_charge;
	driver->is_charge_tapered = qti_charger_is_charge_tapered;
	driver->is_charge_halt = qti_charger_is_charge_halt;
	driver->set_constraint = qti_charger_set_constraint;
	chg->driver = driver;

	/* register driver to mmi charger */
	rc = mmi_register_charger_driver(driver);
	if (rc) {
		mmi_err(chg, "qti charger init failed, rc=%d\n", rc);
	} else {
		mmi_info(chg, "qti charger init successfully\n");
	}

	rc = device_create_file(chg->dev,
				&dev_attr_tcmd);
	if (rc) {
		mmi_err(chg,
			   "Couldn't create tcmd\n");
	}

	rc = device_create_file(chg->dev,
				&dev_attr_force_pmic_icl);
	if (rc) {
		mmi_err(chg,
			   "Couldn't create force_pmic_icl\n");
	}

	rc = device_create_file(chg->dev,
				&dev_attr_force_wls_en);
	if (rc) {
		mmi_err(chg,
			   "Couldn't create force_wls_en\n");
	}

	rc = device_create_file(chg->dev,
				&dev_attr_force_usb_suspend);
	if (rc) {
		mmi_err(chg,
			   "Couldn't create force_usb_suspend\n");
	}

	rc = device_create_file(chg->dev,
				&dev_attr_force_wls_volt_max);
	if (rc) {
		mmi_err(chg,
			   "Couldn't create force_wls_volt_max\n");
	}

	rc = device_create_file(chg->dev,
				&dev_attr_force_wls_curr_max);
	if (rc) {
		mmi_err(chg,
			   "Couldn't create force_wls_curr_max\n");
	}

	rc = device_create_file(chg->dev,
				&dev_attr_wireless_chip_id);
	if (rc) {
		mmi_err(chg,
			   "Couldn't create wireless_chip_id\n");
	}

	rc = device_create_file(chg->dev,
				&dev_attr_addr);
	if (rc) {
		mmi_err(chg,
			   "Couldn't create addr\n");
	}

	rc = device_create_file(chg->dev,
				&dev_attr_data);
	if (rc) {
		mmi_err(chg,
			   "Couldn't create data\n");
	}

	rc = device_create_file(chg->dev,
				&dev_attr_wls_fod_curr);
	if (rc) {
		mmi_err(chg,
			   "Couldn't create wls_fod_curr\n");
	}

	rc = device_create_file(chg->dev,
				&dev_attr_wls_fod_gain);
	if (rc) {
		mmi_err(chg,
			   "Couldn't create wls_fod_gain\n");
	}

	bm_ulog_print_mask_log(BM_ALL, BM_LOG_LEVEL_INFO, OEM_BM_ULOG_SIZE);

	wireless_psy_init(chg);
	create_debugfs_entries(chg);
	return 0;
}

static void qti_charger_deinit(struct qti_charger *chg)
{
	int rc;

	if (!chg->driver) {
		mmi_info(chg, "qti charger has not inited yet\n");
		return;
	}

	device_remove_file(chg->dev, &dev_attr_tcmd);
	device_remove_file(chg->dev, &dev_attr_force_pmic_icl);
	device_remove_file(chg->dev, &dev_attr_force_wls_en);
	device_remove_file(chg->dev, &dev_attr_force_usb_suspend);
	device_remove_file(chg->dev, &dev_attr_force_wls_volt_max);
	device_remove_file(chg->dev, &dev_attr_force_wls_curr_max);
	device_remove_file(chg->dev, &dev_attr_wireless_chip_id);
	device_remove_file(chg->dev, &dev_attr_wls_fod_curr);
	device_remove_file(chg->dev, &dev_attr_wls_fod_gain);
	device_remove_file(chg->dev, &dev_attr_addr);
	device_remove_file(chg->dev, &dev_attr_data);

	wireless_psy_deinit(chg);

	if (chg->debug_root)
		debugfs_remove_recursive(chg->debug_root);

	/* unregister driver from mmi charger */
	rc = mmi_unregister_charger_driver(chg->driver);
	if (rc) {
		mmi_err(chg, "qti charger deinit failed, rc=%d\n", rc);
	} else {
		devm_kfree(chg->dev, chg->driver);
		chg->driver = NULL;
	}
}

static void qti_charger_setup_work(struct work_struct *work)
{
	struct qti_charger *chg = container_of(work,
				struct qti_charger, setup_work);
	enum pmic_glink_state state;

	state = atomic_read(&chg->state);
	if (state == PMIC_GLINK_STATE_UP) {
		mmi_info(chg, "ADSP glink state is up\n");
		qti_charger_init(chg);
	} else if (state == PMIC_GLINK_STATE_DOWN) {
		mmi_err(chg, "ADSP glink state is down\n");
		memset(&chg->chg_cfg, 0, sizeof(struct mmi_charger_cfg));
		memset(&chg->constraint, 0, sizeof(struct mmi_charger_constraint));
	}
}

static void qti_charger_notify_work(struct work_struct *work)
{
	unsigned long notification;
	struct qti_charger_notify_data notify_data;
	struct qti_charger *chg = container_of(work,
				struct qti_charger, notify_work);

	notification = chg->notify_msg.notification;
	notify_data.receiver = chg->notify_msg.receiver;
	memcpy(notify_data.data, chg->notify_msg.data,
				sizeof(u32) * MAX_OEM_NOTIFY_DATA_LEN);
	blocking_notifier_call_chain(&qti_chg_notifier_list,
				notification,
				&notify_data);
	pm_relax(chg->dev);
}

static int qti_charger_parse_dt(struct qti_charger *chg)
{
	int rc;
	int i, j;
	int n = 0;
	int bk_num;
	int bk_size;
	char bk_buf[128];
	int byte_len;
	const char *df_sn = NULL, *dev_sn = NULL;
	struct device_node *node;

	node = chg->dev->of_node;
	dev_sn = mmi_get_battery_serialnumber();
	if (!dev_sn) {
		rc = of_property_read_string(node, "mmi,df-serialnum",
						&df_sn);
		if (!rc && df_sn) {
			mmi_info(chg, "Default Serial Number %s\n", df_sn);
		} else {
			mmi_err(chg, "No Default Serial Number defined\n");
			df_sn = BATT_SN_UNKNOWN;
		}
		strcpy(chg->batt_info.batt_sn, df_sn);
	} else {
		strcpy(chg->batt_info.batt_sn, dev_sn);
	}

	chg->profile_info.profile_id = find_profile_id(chg);
	if (chg->profile_info.profile_id < 0)
		chg->profile_info.profile_id = BATT_DEFAULT_ID;

	rc = of_property_read_u32(node, "mmi,chrg-iterm-ma",
				  &chg->profile_info.chrg_iterm);
	if (rc) {
		chg->profile_info.chrg_iterm = 300000;
	} else {
		chg->profile_info.chrg_iterm *= 1000;
	}

	rc = of_property_read_u32(node, "mmi,fg-iterm-ma",
				  &chg->profile_info.fg_iterm);
	if (rc) {
		chg->profile_info.fg_iterm =
			chg->profile_info.chrg_iterm + 50000;
	} else {
		chg->profile_info.fg_iterm *= 1000;
	}

	rc = of_property_read_u32(node, "mmi,vfloat-comp-uv",
				  &chg->profile_info.vfloat_comp_uv);
	if (rc)
		chg->profile_info.vfloat_comp_uv = 0;

	rc = of_property_read_u32(node, "mmi,max-fv-mv",
				  &chg->profile_info.max_fv_uv);
	if (rc)
		chg->profile_info.max_fv_uv = 4400;
	chg->profile_info.max_fv_uv *= 1000;

	rc = of_property_read_u32(node, "mmi,max-fcc-ma",
				  &chg->profile_info.max_fcc_ua);
	if (rc)
		chg->profile_info.max_fcc_ua = 4000;
	chg->profile_info.max_fcc_ua *= 1000;

	rc = of_property_read_u32(node, "mmi,demo-fv-mv",
				  &chg->profile_info.demo_fv_uv);
	if (rc)
		chg->profile_info.demo_fv_uv = 4000;
	chg->profile_info.demo_fv_uv *= 1000;

	rc = of_property_read_u32(node, "mmi,profile-data-block-size",
				  &chg->profile_info.data_bk_size);
	if (rc)
		chg->profile_info.data_bk_size = 4;
	chg->profile_info.data_bk_size *= 4;

	chg->profile_info.data_size = 0;
	if (of_find_property(node, "mmi,profile-data", &byte_len)) {
		if (byte_len % chg->profile_info.data_bk_size) {
			mmi_err(chg, "DT error wrong profile data\n");
			chg->profile_info.data_bk_size = 0;
			return -ENODEV;
		}
		bk_num = byte_len / chg->profile_info.data_bk_size;
		chg->profile_data = (u32 *)devm_kzalloc(chg->dev, byte_len,
							GFP_KERNEL);
		if (chg->profile_data == NULL) {
			chg->profile_info.data_bk_size = 0;
			return -ENOMEM;
		}

		rc = of_property_read_u32_array(node,
				"mmi,profile-data",
				chg->profile_data,
				byte_len / sizeof(u32));
		if (rc < 0) {
			mmi_err(chg, "Couldn't read profile data, rc = %d\n", rc);
			devm_kfree(chg->dev, chg->profile_data);
			chg->profile_data = NULL;
			chg->profile_info.data_bk_size = 0;
			return rc;
		}

		chg->profile_info.data_size = byte_len;
		mmi_info(chg, "profile data: block size: %d, num: %d\n",
				chg->profile_info.data_bk_size, bk_num);
		bk_size = chg->profile_info.data_bk_size / 4;
		for (i = 0; i < bk_num; i++) {
			memset(bk_buf, '\0', sizeof(bk_buf));
			n = sprintf(bk_buf, "block%d:", i);
			for (j = 0; j < bk_size; j++) {
				n += sprintf(bk_buf + n, " %d",
					chg->profile_data[i * bk_size + j]);
			}
			mmi_info(chg, "%s\n", bk_buf);
		}
	}

	return 0;
}

static int qti_charger_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pmic_glink_client_data client_data;
	struct qti_charger *chg;
	int rc;

	chg = devm_kzalloc(dev, sizeof(*chg), GFP_KERNEL);
	if (!chg)
		return -ENOMEM;

	INIT_WORK(&chg->setup_work, qti_charger_setup_work);
	INIT_WORK(&chg->notify_work, qti_charger_notify_work);
	mutex_init(&chg->read_lock);
	mutex_init(&chg->write_lock);
	init_completion(&chg->read_ack);
	init_completion(&chg->write_ack);
	atomic_set(&chg->rx_valid, 0);
	atomic_set(&chg->state, PMIC_GLINK_STATE_UP);
	platform_set_drvdata(pdev, chg);
	chg->dev = dev;
	chg->name = "qti_glink_charger";

	chg->debug_enabled = &debug_enabled;
	chg->ipc_log = ipc_log_context_create(MMI_LOG_PAGES, MMI_LOG_DIR, 0);
	if (!chg->ipc_log)
		mmi_warn(chg, "Error in creating ipc_log_context\n");

	rc = qti_charger_parse_dt(chg);
	if (rc) {
		mmi_err(chg, "dt paser failed, rc=%d\n", rc);
		return rc;
	}

	client_data.id = MSG_OWNER_OEM;
	client_data.name = "oem";
	client_data.msg_cb = oem_callback;
	client_data.priv = chg;
	client_data.state_cb = oem_state_cb;

	chg->client = pmic_glink_register_client(dev, &client_data);
	if (IS_ERR(chg->client)) {
		rc = PTR_ERR(chg->client);
		if (rc != -EPROBE_DEFER)
			mmi_err(chg, "Error in registering with pmic_glink rc=%d\n",
				rc);
		return rc;
	}

	this_chip = chg;
	device_init_wakeup(chg->dev, true);
	qti_charger_init(chg);
	return 0;
}

static int qti_charger_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct qti_charger *chg= dev_get_drvdata(dev);
	int rc;

	qti_charger_deinit(chg);
	rc = pmic_glink_unregister_client(chg->client);
	if (rc < 0)
		mmi_err(chg, "pmic_glink_unregister_client failed rc=%d\n",
			rc);

	return rc;
}

static const struct of_device_id qti_charger_match_table[] = {
	{.compatible = "mmi,qti-glink-charger"},
	{},
};

static struct platform_driver qti_charger_driver = {
	.driver	= {
		.name = "qti_glink_charger",
		.of_match_table = qti_charger_match_table,
	},
	.probe	= qti_charger_probe,
	.remove	= qti_charger_remove,
};

module_platform_driver(qti_charger_driver);

MODULE_DESCRIPTION("QTI Glink Charger Driver");
MODULE_LICENSE("GPL v2");
