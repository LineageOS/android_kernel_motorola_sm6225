/* SPDX-License-Identifier: GPL-2.0 */
/*  Himax Android Driver Sample Code for inspection functions
 *
 *  Copyright (C) 2019 Himax Corporation.
 *
 *  This software is licensed under the terms of the GNU General Public
 *  License version 2,  as published by the Free Software Foundation,  and
 *  may be copied,  distributed,  and modified under those terms.
 *
 *  This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 */

#include "himax_inspection.h"

static int g_gap_vertical_partial = 3;
static int *g_gap_vertical_part;
static int g_gap_horizontal_partial = 3;
static int *g_gap_horizontal_part;

static int g_dc_max;

static int g_1kind_raw_size;
uint32_t g_rslt_data_len;
int **g_inspection_criteria;
int *g_inspt_crtra_flag;
int HX_CRITERIA_ITEM;
int HX_CRITERIA_SIZE;
char *g_rslt_data;
static char g_file_path[256];
static char g_rslt_log[256];
static char g_start_log[256];
char *hx_self_test_file_name = NULL;

char *g_himax_inspection_mode[] = {
	"HIMAX_INSPECTION_OPEN",
	"HIMAX_INSPECTION_MICRO_OPEN",
	"HIMAX_INSPECTION_SHORT",
	"HIMAX_INSPECTION_RAWDATA",
	"HIMAX_INSPECTION_BPN_RAWDATA",
	"HIMAX_INSPECTION_WEIGHT_NOISE",
	"HIMAX_INSPECTION_ABS_NOISE",
	"HIMAX_INSPECTION_SORTING",
	"HIMAX_INSPECTION_BACK_NORMAL",

	"HIMAX_INSPECTION_GAPTEST_RAW",

	"HIMAX_INSPECTION_ACT_IDLE_RAWDATA",
	"HIMAX_INSPECTION_ACT_IDLE_NOISE",

	"HIMAX_INSPECTION_LPWUG_RAWDATA",
	"HIMAX_INSPECTION_LPWUG_NOISE",
	"HIMAX_INSPECTION_LPWUG_IDLE_RAWDATA",
	"HIMAX_INSPECTION_LPWUG_IDLE_NOISE",
	NULL
};

/* for criteria */
char *g_hx_inspt_crtra_name[] = {
	"CRITERIA_RAW_MIN",
	"CRITERIA_RAW_MAX",
	"CRITERIA_RAW_BPN_MIN",
	"CRITERIA_RAW_BPN_MAX",
	"CRITERIA_SHORT_MIN",
	"CRITERIA_SHORT_MAX",
	"CRITERIA_OPEN_MIN",
	"CRITERIA_OPEN_MAX",
	"CRITERIA_MICRO_OPEN_MIN",
	"CRITERIA_MICRO_OPEN_MAX",
	"CRITERIA_NOISE_WT_MIN",
	"CRITERIA_NOISE_WT_MAX",
	"CRITERIA_NOISE_ABS_MIN",
	"CRITERIA_NOISE_ABS_MAX",
	"CRITERIA_SORT_MIN",
	"CRITERIA_SORT_MAX",

	"CRITERIA_GAP_RAW_HOR_MIN",
	"CRITERIA_GAP_RAW_HOR_MAX",
	"CRITERIA_GAP_RAW_VER_MIN",
	"CRITERIA_GAP_RAW_VER_MAX",

	"ACT_IDLE_NOISE_MIN",
	"ACT_IDLE_NOISE_MAX",
	"ACT_IDLE_RAWDATA_MIN",
	"ACT_IDLE_RAWDATA_MAX",

	"LPWUG_NOISE_MIN",
	"LPWUG_NOISE_MAX",
	"LPWUG_RAWDATA_MIN",
	"LPWUG_RAWDATA_MAX",
	"LPWUG_IDLE_NOISE_MIN",
	"LPWUG_IDLE_NOISE_MAX",
	"LPWUG_IDLE_RAWDATA_MIN",
	"LPWUG_IDLE_RAWDATA_MAX",
	NULL
};


void (*fp_himax_self_test_init)(void) = himax_inspection_init;


static void himax_press_powerkey(void)
{
	I(" %s POWER KEY event %x press\n", __func__, KEY_POWER);
	input_report_key(private_ts->input_dev, KEY_POWER, 1);
	input_sync(private_ts->input_dev);

	msleep(100);

	I(" %s POWER KEY event %x release\n", __func__, KEY_POWER);
	input_report_key(private_ts->input_dev, KEY_POWER, 0);
	input_sync(private_ts->input_dev);
}


static uint8_t	NOISEMAX;
static uint8_t	g_recal_thx;

static int hx_test_data_pop_out(char *rslt_buf, char *filepath)
{
	struct file *raw_file = NULL;
	struct filename *vts_name = NULL;
	mm_segment_t fs;
	loff_t pos = 0;
	int ret_val = NO_ERR;

	I("%s: Entering!\n", __func__);
	I("data size=0x%04X\n", (uint32_t)strlen(rslt_buf));

	if (!kp_getname_kernel) {
		E("kp_getname_kernel is NULL, not open file!\n");
		return -EIO;
	}
	vts_name = kp_getname_kernel(filepath);

	if (raw_file == NULL)
		raw_file = kp_file_open_name(vts_name, O_TRUNC|O_CREAT|O_RDWR,
					 0660);

	if (IS_ERR(raw_file)) {
		E("%s open file failed = %ld\n", __func__, PTR_ERR(raw_file));
		ret_val = -EIO;
		goto SAVE_DATA_ERR;
	}

	fs = get_fs();
	set_fs(get_ds());
	vfs_write(raw_file, rslt_buf,
		 g_1kind_raw_size * HX_CRITERIA_ITEM * sizeof(char), &pos);
	if (raw_file != NULL)
		filp_close(raw_file, NULL);
	if (!kp_putname_kernel) {
		E("kp_putname_kernel is NULL, not open file!\n");
		} else
		kp_putname_kernel(vts_name);

	set_fs(fs);

SAVE_DATA_ERR:
	I("%s: End!\n", __func__);
	return ret_val;
}

static int hx_test_data_get(uint32_t RAW[], char *start_log, char *result,
			 int now_item)
{
	uint32_t i;

	ssize_t len = 0;
	char *testdata = NULL;
	uint32_t SZ_SIZE = g_1kind_raw_size;

	I("%s: Entering, Now type=%s!\n", __func__,
		 g_himax_inspection_mode[now_item]);

	testdata = kzalloc(sizeof(char) * SZ_SIZE, GFP_KERNEL);
	if (testdata == NULL) {
		E("%s: Memory allocation falied!\n", __func__);
		return MEM_ALLOC_FAIL;
	}

	len += snprintf((testdata + len), SZ_SIZE - len, "%s", start_log);
	for (i = 0; i < ic_data->HX_TX_NUM*ic_data->HX_RX_NUM; i++) {
		if (i > 1 && ((i + 1) % ic_data->HX_RX_NUM) == 0)
			len += snprintf((testdata + len), SZ_SIZE - len,
				 "%5d,\n", RAW[i]);
		else
			len += snprintf((testdata + len), SZ_SIZE - len,
				 "%5d,", RAW[i]);
	}
	len += snprintf((testdata + len), SZ_SIZE - len, "\n%s", result);

	memcpy(&g_rslt_data[g_rslt_data_len], testdata, len);
	g_rslt_data_len += len;
	I("%s: g_rslt_data_len=%d!\n", __func__, g_rslt_data_len);

	/*memcpy(&g_rslt_data[now_item * SZ_SIZE], testdata, SZ_SIZE);*/

	/* dbg */
	/* for(i = 0; i < SZ_SIZE; i++)
	 * {
	 *	I("0x%04X, ", g_rslt_data[i + (now_item * SZ_SIZE)]);
	 *	if(i > 0 && (i % 16 == 15))
	 *		PI("\n");
	 * }
	 */

	kfree(testdata);
	I("%s: End!\n", __func__);
	return NO_ERR;

}

static int himax_switch_mode_inspection(int mode)
{
	uint8_t tmp_addr[4];
	uint8_t tmp_data[4];

	I("%s: Entering\n", __func__);

	/*Stop Handshaking*/
	tmp_addr[3] = 0x10; tmp_addr[2] = 0x00; tmp_addr[1] = 0x00;
	tmp_addr[0] = 0x00; tmp_data[3] = 0x00; tmp_data[2] = 0x00;
	tmp_data[1] = 0x00; tmp_data[0] = 0x00;
	g_core_fp.fp_register_write(tmp_addr, 4, tmp_data, 0);

	/*Swtich Mode*/
	switch (mode) {
	case HIMAX_INSPECTION_SORTING:
		tmp_data[3] = 0x00; tmp_data[2] = 0x00;
		tmp_data[1] = PWD_SORTING_START;
		tmp_data[0] = PWD_SORTING_START;
		break;
	case HIMAX_INSPECTION_OPEN:
		tmp_data[3] = 0x00; tmp_data[2] = 0x00;
		tmp_data[1] = PWD_OPEN_START;
		tmp_data[0] = PWD_OPEN_START;
		break;
	case HIMAX_INSPECTION_MICRO_OPEN:
		tmp_data[3] = 0x00; tmp_data[2] = 0x00;
		tmp_data[1] = PWD_OPEN_START;
		tmp_data[0] = PWD_OPEN_START;
		break;
	case HIMAX_INSPECTION_SHORT:
		tmp_data[3] = 0x00; tmp_data[2] = 0x00;
		tmp_data[1] = PWD_SHORT_START;
		tmp_data[0] = PWD_SHORT_START;
		break;

	case HIMAX_INSPECTION_GAPTEST_RAW:
	case HIMAX_INSPECTION_RAWDATA:
		tmp_data[3] = 0x00; tmp_data[2] = 0x00;
		tmp_data[1] = PWD_RAWDATA_START;
		tmp_data[0] = PWD_RAWDATA_START;
		break;
	case HIMAX_INSPECTION_BPN_RAWDATA:
		tmp_data[3] = 0x00; tmp_data[2] = 0x00;
		tmp_data[1] = PWD_RAWDATA_START;
		tmp_data[0] = PWD_RAWDATA_START;
		break;
	case HIMAX_INSPECTION_WEIGHT_NOISE:
	case HIMAX_INSPECTION_ABS_NOISE:
		tmp_data[3] = 0x00; tmp_data[2] = 0x00;
		tmp_data[1] = PWD_NOISE_START;
		tmp_data[0] = PWD_NOISE_START;
		break;

	case HIMAX_INSPECTION_ACT_IDLE_RAWDATA:
	case HIMAX_INSPECTION_ACT_IDLE_NOISE:
		tmp_data[3] = 0x00; tmp_data[2] = 0x00;
		tmp_data[1] = PWD_ACT_IDLE_START;
		tmp_data[0] = PWD_ACT_IDLE_START;
		break;

	case HIMAX_INSPECTION_LPWUG_RAWDATA:
	case HIMAX_INSPECTION_LPWUG_NOISE:
		tmp_data[3] = 0x00; tmp_data[2] = 0x00;
		tmp_data[1] = PWD_LPWUG_START;
		tmp_data[0] = PWD_LPWUG_START;
		break;
	case HIMAX_INSPECTION_LPWUG_IDLE_RAWDATA:
	case HIMAX_INSPECTION_LPWUG_IDLE_NOISE:
		tmp_data[3] = 0x00; tmp_data[2] = 0x00;
		tmp_data[1] = PWD_LPWUG_IDLE_START;
		tmp_data[0] = PWD_LPWUG_IDLE_START;
		break;

	default:
		I("%s,Nothing to be done!\n", __func__);
		break;
	}

	if (g_core_fp.fp_assign_sorting_mode != NULL)
		g_core_fp.fp_assign_sorting_mode(tmp_data);
	I("%s: End of setting!\n", __func__);

	return 0;

}

static int himax_get_rawdata(uint32_t RAW[], uint32_t datalen)
{
	uint8_t *tmp_rawdata;
	bool get_raw_rlst;
	uint8_t retry = 0;
	uint32_t i = 0;
	uint32_t j = 0;
	uint32_t index = 0;
	uint32_t Min_DATA = 0xFFFFFFFF;
	uint32_t Max_DATA = 0x00000000;

	/* We use two bytes to combine a value of rawdata.*/
	tmp_rawdata = kzalloc(sizeof(uint8_t) * (datalen * 2), GFP_KERNEL);
	if (tmp_rawdata == NULL) {
		E("%s: Memory allocation falied!\n", __func__);
		return MEM_ALLOC_FAIL;
	}

	while (retry < 200) {
		get_raw_rlst = g_core_fp.fp_get_DSRAM_data(tmp_rawdata, false);
		if (get_raw_rlst)
			break;
		retry++;
	}

	if (retry >= 200)
		goto DIRECT_END;

	/* Copy Data*/
	for (i = 0; i < ic_data->HX_TX_NUM*ic_data->HX_RX_NUM; i++)
		RAW[i] = tmp_rawdata[(i * 2) + 1] * 256 + tmp_rawdata[(i * 2)];
        /* don't output these logs, reduce factory test time
	for (j = 0; j < ic_data->HX_RX_NUM; j++) {
		if (j == 0)
			PI("      RX%2d", j + 1);
		else
			PI("  RX%2d", j + 1);
	}
	PI("\n");
        ******* delete these logs *********************/

	for (i = 0; i < ic_data->HX_TX_NUM; i++) {
		/*PI("TX%2d", i + 1);*/
		for (j = 0; j < ic_data->HX_RX_NUM; j++) {
			PI("%5d ", RAW[index]);
			if (RAW[index] > Max_DATA)
				Max_DATA = RAW[index];

			if (RAW[index] < Min_DATA)
				Min_DATA = RAW[index];

			index++;
		}
		/*PI("\n");*/
	}
	I("Max = %5d, Min = %5d\n", Max_DATA, Min_DATA);
DIRECT_END:
	kfree(tmp_rawdata);

	if (get_raw_rlst)
		return HX_INSPECT_OK;
	else
		return HX_INSPECT_EGETRAW;

}

static void himax_switch_data_type(uint8_t checktype)
{
	uint8_t datatype = 0x00;

	switch (checktype) {
	case HIMAX_INSPECTION_SORTING:
		datatype = DATA_SORTING;
		break;
	case HIMAX_INSPECTION_OPEN:
		datatype = DATA_OPEN;
		break;
	case HIMAX_INSPECTION_MICRO_OPEN:
		datatype = DATA_MICRO_OPEN;
		break;
	case HIMAX_INSPECTION_SHORT:
		datatype = DATA_SHORT;
		break;
	case HIMAX_INSPECTION_RAWDATA:
		datatype = DATA_RAWDATA;
		break;
	case HIMAX_INSPECTION_BPN_RAWDATA:
		datatype = DATA_RAWDATA;
		break;
	case HIMAX_INSPECTION_WEIGHT_NOISE:
	case HIMAX_INSPECTION_ABS_NOISE:
		datatype = DATA_NOISE;
		break;
	case HIMAX_INSPECTION_BACK_NORMAL:
		datatype = DATA_BACK_NORMAL;
		break;

	case HIMAX_INSPECTION_GAPTEST_RAW:
		datatype = DATA_RAWDATA;
		break;

	case HIMAX_INSPECTION_ACT_IDLE_RAWDATA:
		datatype = DATA_ACT_IDLE_RAWDATA;
		break;
	case HIMAX_INSPECTION_ACT_IDLE_NOISE:
		datatype = DATA_ACT_IDLE_NOISE;
		break;

	case HIMAX_INSPECTION_LPWUG_RAWDATA:
		datatype = DATA_LPWUG_RAWDATA;
		break;
	case HIMAX_INSPECTION_LPWUG_NOISE:
		datatype = DATA_LPWUG_NOISE;
		break;
	case HIMAX_INSPECTION_LPWUG_IDLE_RAWDATA:
		datatype = DATA_LPWUG_IDLE_RAWDATA;
		break;
	case HIMAX_INSPECTION_LPWUG_IDLE_NOISE:
		datatype = DATA_LPWUG_IDLE_NOISE;
		break;

	default:
		E("Wrong type=%d\n", checktype);
		break;
	}
	g_core_fp.fp_diag_register_set(datatype, 0x00);
}

static void himax_bank_search_set(uint16_t Nframe, uint8_t checktype)
{
	uint8_t tmp_addr[4];
	uint8_t tmp_data[4];

	/*skip frame*/
	tmp_addr[3] = 0x10; tmp_addr[2] = 0x00;
	tmp_addr[1] = 0x70; tmp_addr[0] = 0xF4;
	g_core_fp.fp_register_read(tmp_addr, 4, tmp_data, false);

	switch (checktype) {
	case HIMAX_INSPECTION_ACT_IDLE_RAWDATA:
	case HIMAX_INSPECTION_ACT_IDLE_NOISE:
		tmp_data[0] = BS_ACT_IDLE;
		break;
	case HIMAX_INSPECTION_LPWUG_RAWDATA:
	case HIMAX_INSPECTION_LPWUG_NOISE:
		tmp_data[0] = BS_LPWUG;
		break;
	case HIMAX_INSPECTION_LPWUG_IDLE_RAWDATA:
	case HIMAX_INSPECTION_LPWUG_IDLE_NOISE:
		tmp_data[0] = BS_LPWUG_dile;
		break;
	case HIMAX_INSPECTION_RAWDATA:
	case HIMAX_INSPECTION_BPN_RAWDATA:
		tmp_data[0] = BS_RAWDATA;
		break;
	case HIMAX_INSPECTION_WEIGHT_NOISE:
	case HIMAX_INSPECTION_ABS_NOISE:
		tmp_data[0] = BS_NOISE;
		break;
	default:
		tmp_data[0] = BS_OPENSHORT;
		break;
	}
	g_core_fp.fp_register_write(tmp_addr, 4, tmp_data, 0);
}

static void himax_neg_noise_sup(uint8_t *data)
{
	uint8_t tmp_addr[4];
	uint8_t tmp_data[4];

	/*0x10007FD8 Check support negative value or not */
	tmp_addr[3] = 0x10; tmp_addr[2] = 0x00;
	tmp_addr[1] = 0x7F; tmp_addr[0] = 0xD8;
	g_core_fp.fp_register_read(tmp_addr, 4, tmp_data, false);

	if ((tmp_data[3] & 0x04) == 0x04) {
		data[2] = 0x0C; data[3] = 0x7F;
	} else
		I("%s Not support negative noise\n", __func__);
}

static void himax_set_N_frame(uint16_t Nframe, uint8_t checktype)
{
	uint8_t tmp_addr[4];
	uint8_t tmp_data[4];

	himax_bank_search_set(Nframe, checktype);

	/*IIR MAX*/
	tmp_addr[3] = 0x10; tmp_addr[2] = 0x00; tmp_addr[1] = 0x72;
	tmp_addr[0] = 0x94;
	tmp_data[3] = 0x00; tmp_data[2] = 0x00;
	tmp_data[1] = (uint8_t)((Nframe & 0xFF00) >> 8);
	tmp_data[0] = (uint8_t)(Nframe & 0x00FF);
	g_core_fp.fp_register_write(tmp_addr, 4, tmp_data, 0);

	if (checktype == HIMAX_INSPECTION_WEIGHT_NOISE ||
		checktype == HIMAX_INSPECTION_ABS_NOISE)
		himax_neg_noise_sup(tmp_data);

	g_core_fp.fp_register_write(tmp_addr, 4, tmp_data, 0);
}

static void himax_get_noise_base(void)/*Normal Threshold*/
{
	uint8_t tmp_addr[4];
	uint8_t tmp_data[4];

	tmp_addr[3] = 0x10; tmp_addr[2] = 0x00;
	tmp_addr[1] = 0x70; tmp_addr[0] = 0x8C;/*0x1000708F*/
	g_core_fp.fp_register_read(tmp_addr, 4, tmp_data, false);

	NOISEMAX = tmp_data[3];

	tmp_addr[3] = 0x10; tmp_addr[2] = 0x00;
	tmp_addr[1] = 0x70; tmp_addr[0] = 0x90;/*0x10007092*/
	g_core_fp.fp_register_read(tmp_addr, 4, tmp_data, false);

	g_recal_thx = tmp_data[2];
	I("%s: NOISEMAX=%d, g_recal_thx = %d\n", __func__,
		NOISEMAX, g_recal_thx);
}

static uint16_t himax_get_palm_num(void)/*Palm Number*/
{
	uint8_t tmp_addr[4];
	uint8_t tmp_data[4];
	uint16_t palm_num;

	tmp_addr[3] = 0x10; tmp_addr[2] = 0x00;
	tmp_addr[1] = 0x70; tmp_addr[0] = 0xA8;/*0x100070AB*/
	g_core_fp.fp_register_read(tmp_addr, 4, tmp_data, false);
	palm_num = tmp_data[3];
	I("%s: palm_num = %d ", __func__, palm_num);

	return palm_num;
}

static int himax_get_noise_weight_test(void)
{
	uint8_t tmp_addr[4];
	uint8_t tmp_data[4];
	uint16_t weight = 0;
	uint16_t value = 0;

	tmp_addr[3] = 0x10; tmp_addr[2] = 0x00;
	tmp_addr[1] = 0x72; tmp_addr[0] = 0xC8;/*0x100072C8 weighting value*/
	g_core_fp.fp_register_read(tmp_addr, 4, tmp_data, false);
	if (tmp_data[3] != 0x72 || tmp_data[2] != 0xC8)
		return FW_NOT_READY;

	value = (tmp_data[1] << 8) | tmp_data[0];
	I("%s: value = %d, %d, %d ", __func__, value, tmp_data[2], tmp_data[3]);

	tmp_addr[3] = 0x10; tmp_addr[2] = 0x00;
	tmp_addr[1] = 0x70; tmp_addr[0] = 0x9C;/*0x1000709C weighting threshold*/
	g_core_fp.fp_register_read(tmp_addr, 4, tmp_data, false);
	weight = tmp_data[0];
	tmp_addr[3] = 0x10; tmp_addr[2] = 0x00;
	tmp_addr[1] = 0x70; tmp_addr[0] = 0x94;/*0x10007095 weighting threshold*/
	g_core_fp.fp_register_read(tmp_addr, 4, tmp_data, false);
	weight = tmp_data[1] * weight;
	I("%s: weight = %d ", __func__, weight);

	if (value > weight)
		return ERR_TEST_FAIL;
	else
		return 0;
}

static uint32_t himax_check_mode(uint8_t checktype)
{
	uint8_t tmp_data[4] = {0};
	uint8_t wait_pwd[2] = {0};

	switch (checktype) {
	case HIMAX_INSPECTION_SORTING:
		wait_pwd[0] = PWD_SORTING_END;
		wait_pwd[1] = PWD_SORTING_END;
		break;
	case HIMAX_INSPECTION_OPEN:
		wait_pwd[0] = PWD_OPEN_END;
		wait_pwd[1] = PWD_OPEN_END;
		break;
	case HIMAX_INSPECTION_MICRO_OPEN:
		wait_pwd[0] = PWD_OPEN_END;
		wait_pwd[1] = PWD_OPEN_END;
		break;
	case HIMAX_INSPECTION_SHORT:
		wait_pwd[0] = PWD_SHORT_END;
		wait_pwd[1] = PWD_SHORT_END;
		break;
	case HIMAX_INSPECTION_RAWDATA:
		wait_pwd[0] = PWD_RAWDATA_END;
		wait_pwd[1] = PWD_RAWDATA_END;
		break;
	case HIMAX_INSPECTION_BPN_RAWDATA:
		wait_pwd[0] = PWD_RAWDATA_END;
		wait_pwd[1] = PWD_RAWDATA_END;
		break;
	case HIMAX_INSPECTION_WEIGHT_NOISE:
	case HIMAX_INSPECTION_ABS_NOISE:
		wait_pwd[0] = PWD_NOISE_END;
		wait_pwd[1] = PWD_NOISE_END;
		break;

	case HIMAX_INSPECTION_ACT_IDLE_RAWDATA:
	case HIMAX_INSPECTION_ACT_IDLE_NOISE:
		wait_pwd[0] = PWD_ACT_IDLE_END;
		wait_pwd[1] = PWD_ACT_IDLE_END;
		break;

	case HIMAX_INSPECTION_LPWUG_RAWDATA:
	case HIMAX_INSPECTION_LPWUG_NOISE:
		wait_pwd[0] = PWD_LPWUG_END;
		wait_pwd[1] = PWD_LPWUG_END;
		break;
	case HIMAX_INSPECTION_LPWUG_IDLE_RAWDATA:
	case HIMAX_INSPECTION_LPWUG_IDLE_NOISE:
		wait_pwd[0] = PWD_LPWUG_IDLE_END;
		wait_pwd[1] = PWD_LPWUG_IDLE_END;
		break;

	default:
		E("Wrong type=%d\n", checktype);
		break;
	}

	if (g_core_fp.fp_check_sorting_mode != NULL)
		g_core_fp.fp_check_sorting_mode(tmp_data);

	if ((wait_pwd[0] == tmp_data[0]) && (wait_pwd[1] == tmp_data[1])) {
		I("Change to mode=%s\n", g_himax_inspection_mode[checktype]);
		return 0;
	} else {
		return 1;
	}
}

#define TEMP_LOG "%s: %s, tmp_data[0]=%x,tmp_data[1]=%x,tmp_data[2]=%x,tmp_data[3]=%x\n"

static uint32_t himax_wait_sorting_mode(uint8_t checktype)
{
	uint8_t tmp_addr[4] = {0};
	uint8_t tmp_data[4] = {0};
	uint8_t wait_pwd[2] = {0};
	int count = 0;

	switch (checktype) {
	case HIMAX_INSPECTION_SORTING:
		wait_pwd[0] = PWD_SORTING_END;
		wait_pwd[1] = PWD_SORTING_END;
		break;
	case HIMAX_INSPECTION_OPEN:
		wait_pwd[0] = PWD_OPEN_END;
		wait_pwd[1] = PWD_OPEN_END;
		break;
	case HIMAX_INSPECTION_MICRO_OPEN:
		wait_pwd[0] = PWD_OPEN_END;
		wait_pwd[1] = PWD_OPEN_END;
		break;
	case HIMAX_INSPECTION_SHORT:
		wait_pwd[0] = PWD_SHORT_END;
		wait_pwd[1] = PWD_SHORT_END;
		break;
	case HIMAX_INSPECTION_RAWDATA:
		wait_pwd[0] = PWD_RAWDATA_END;
		wait_pwd[1] = PWD_RAWDATA_END;
		break;
	case HIMAX_INSPECTION_BPN_RAWDATA:
		wait_pwd[0] = PWD_RAWDATA_END;
		wait_pwd[1] = PWD_RAWDATA_END;
		break;
	case HIMAX_INSPECTION_WEIGHT_NOISE:
	case HIMAX_INSPECTION_ABS_NOISE:
		wait_pwd[0] = PWD_NOISE_END;
		wait_pwd[1] = PWD_NOISE_END;
		break;

	case HIMAX_INSPECTION_GAPTEST_RAW:
		wait_pwd[0] = PWD_RAWDATA_END;
		wait_pwd[1] = PWD_RAWDATA_END;
		break;

	case HIMAX_INSPECTION_ACT_IDLE_RAWDATA:
	case HIMAX_INSPECTION_ACT_IDLE_NOISE:
		wait_pwd[0] = PWD_ACT_IDLE_END;
		wait_pwd[1] = PWD_ACT_IDLE_END;
		break;

	case HIMAX_INSPECTION_LPWUG_RAWDATA:
	case HIMAX_INSPECTION_LPWUG_NOISE:
		wait_pwd[0] = PWD_LPWUG_END;
		wait_pwd[1] = PWD_LPWUG_END;
		break;
	case HIMAX_INSPECTION_LPWUG_IDLE_RAWDATA:
	case HIMAX_INSPECTION_LPWUG_IDLE_NOISE:
		wait_pwd[0] = PWD_LPWUG_IDLE_END;
		wait_pwd[1] = PWD_LPWUG_IDLE_END;
		break;

	default:
		I("No Change Mode and now type=%d\n", checktype);
		break;
	}

	do {
		if (g_core_fp.fp_check_sorting_mode != NULL)
			g_core_fp.fp_check_sorting_mode(tmp_data);
		if ((wait_pwd[0] == tmp_data[0]) &&
			(wait_pwd[1] == tmp_data[1]))
			return 0;


		tmp_addr[3] = 0x90; tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x00; tmp_addr[0] = 0xA8;
		g_core_fp.fp_register_read(tmp_addr, 4, tmp_data, false);
		I(TEMP_LOG, __func__, "0x900000A8",
			tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);

		tmp_addr[3] = 0x90; tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x00; tmp_addr[0] = 0xE4;
		g_core_fp.fp_register_read(tmp_addr, 4, tmp_data, false);
		I(TEMP_LOG, __func__, "0x900000E4",
			tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);

		tmp_addr[3] = 0x10; tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x7F; tmp_addr[0] = 0x40;
		g_core_fp.fp_register_read(tmp_addr, 4, tmp_data, false);
		I(TEMP_LOG, __func__, "0x10007F40",
			tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);
		I("Now retry %d times!\n", count++);
		msleep(50);
	} while (count < 50);

	return 1;
}

static int hx_turn_on_mp_func(int on)
{
	int rslt = 0;
	int retry = 3;
	uint8_t tmp_addr[4] = {0};
	uint8_t tmp_data[4] = {0};
	uint8_t tmp_read[4] = {0};
	/* char *tmp_chipname = private_ts->chip_name; */

	tmp_addr[3] = 0x10; tmp_addr[2] = 0x00; tmp_addr[1] = 0x73; tmp_addr[0] = 0xEC;
	if (on) {
		I("%s : Turn on!\n", __func__);
			if (strcmp(HX_83102D_SERIES_PWON, private_ts->chip_name) == 0) {
				I("%s: need to enter Mp mode!\n", __func__);
				tmp_data[3] = 0x00; tmp_data[2] = 0x10; tmp_data[1] = 0x73; tmp_data[0] = 0x80;
				do {
					g_core_fp.fp_register_write(tmp_addr, 4, tmp_data, 0);
					usleep_range(10000, 10001);
					g_core_fp.fp_register_read(tmp_addr, 4, tmp_read, false);
					I("%s: now read[2]=0x%02X, read[1]=0x%02X, read[0]=0x%02X!\n",
					__func__, tmp_read[2], tmp_read[1], tmp_read[0]);
					retry--;
				} while (((retry > 0) && (tmp_read[2] != tmp_data[2] && tmp_read[1] != tmp_data[1] && tmp_read[0] != tmp_data[0])));
		} else {
			I("%s:Nothing to be done!\n", __func__);
		}
	} else {
		I("%s : Turn off!\n", __func__);
		if (strcmp(HX_83102D_SERIES_PWON, private_ts->chip_name) == 0) {
			I("%s: need to enter Mp mode!\n", __func__);
			tmp_data[3] = 0x00; tmp_data[2] = 0x00; tmp_data[1] = 0x00; tmp_data[0] = 0x00;
			do {
				g_core_fp.fp_register_write(tmp_addr, 4, tmp_data, 0);
				usleep_range(10000, 10001);
				g_core_fp.fp_register_read(tmp_addr, 4, tmp_read, false);
				I("%s: now read[2]=0x%02X, read[1]=0x%02X, read[0]=0x%02X!\n", __func__, tmp_read[2], tmp_read[1], tmp_read[0]);
				retry--;
			} while ((retry > 0) && (tmp_read[2] != tmp_data[2] && tmp_read[1] != tmp_data[1] && tmp_read[0] != tmp_data[0]));
		} else {
			I("%s Nothing to be done!\n", __func__);
		}
	}
	return rslt;
}

/*	 HX_GAP START */
/* gap test function */
/* extern int himax_write_to_ic_flash_flow(uint32_t start_addr,uint32_t
 *	 *write_data,uint32_t write_len);
 */

static int himax_gap_test_vertical_setting(void)
{
	g_gap_vertical_part = kcalloc(g_gap_vertical_partial,
				sizeof(int), GFP_KERNEL);
	if (g_gap_vertical_part == NULL) {
		E("%s: Memory allocation falied!\n", __func__);
		return MEM_ALLOC_FAIL;
	}
	g_gap_vertical_part[0] = 0;
	g_gap_vertical_part[1] = 4;
	g_gap_vertical_part[2] = 8;
	kfree(g_gap_vertical_part);

	return NO_ERR;
}

static void himax_cal_gap_data_vertical(int start, int end_idx, int direct,
				uint32_t *org_raw, uint32_t *result_raw)
{
	int i = 0;
	int rx_num = ic_data->HX_RX_NUM;

	I("%s:start=%d\n", __func__, start);
	I("%s:end_idx=%d\n", __func__, end_idx);
	for (i = start; i < (start + rx_num*end_idx); i++) {
		if (direct == 0) { /* up - down */
			if (i < start+rx_num)
				result_raw[i] = 0;
			else
				result_raw[i] = org_raw[i-rx_num] - org_raw[i];

		} else { /* down - up */
			if (i > (start + rx_num*(end_idx-1)-1))
				result_raw[i] = 0;
			else
				result_raw[i] = org_raw[i+rx_num] - org_raw[i];

		}
	}
}

static int himax_gap_test_vertical_raw(int test_type, uint32_t *org_raw)
{
	int i_partial = 0;
	int tmp_start = 0;
	int tmp_end_idx = 0;
	uint32_t *result_raw;
	int i = 0;
	int ret_val = NO_ERR;

	int tx_num = ic_data->HX_TX_NUM;
	int rx_num = ic_data->HX_RX_NUM;

	if (himax_gap_test_vertical_setting())
		return MEM_ALLOC_FAIL;

	I("Print vertical ORG RAW\n");
	for (i = 0; i < tx_num*rx_num; i++) {
		I("%04d,", org_raw[i]);
		if (i > 0 && i%rx_num == (rx_num-1))
			I("\n");
	}

	result_raw = kcalloc(tx_num*rx_num, sizeof(uint32_t), GFP_KERNEL);
	if (result_raw == NULL) {
		E("%s: Memory allocation falied!\n", __func__);
		goto alloc_result_raw_failed;
	}

	for (i_partial = 0; i_partial < g_gap_vertical_partial; i_partial++) {

		tmp_start	= g_gap_vertical_part[i_partial]*rx_num;
		if (i_partial+1 == g_gap_vertical_partial)
			tmp_end_idx = tx_num - g_gap_vertical_part[i_partial];
		else
			tmp_end_idx = g_gap_vertical_part[i_partial+1] -
				 g_gap_vertical_part[i_partial];

		if (i_partial % 2 == 0)
			himax_cal_gap_data_vertical(tmp_start, tmp_end_idx, 0,
						org_raw, result_raw);
		else
			himax_cal_gap_data_vertical(tmp_start, tmp_end_idx, 1,
						org_raw, result_raw);

	}

	I("Print Vertical New RAW\n");
	for (i = 0; i < tx_num*rx_num; i++) {
		I("%04d,", result_raw[i]);
		if (i > 0 && i%rx_num == (rx_num-1))
			I("\n");
	}

	for (i = 0; i < tx_num*rx_num; i++) {
		if (result_raw[i] < g_inspection_criteria[IDX_GAP_VER_RAWMIN][i]
		 &&
		 result_raw[i] > g_inspection_criteria[IDX_GAP_VER_RAWMAX][i]) {
			ret_val = NO_ERR - i;
			break;
		}
	}

	/* himax_write_to_ic_flash_flow(0x1A000,result_raw,tx_num*rx_num); */
	kfree(result_raw);
alloc_result_raw_failed:
	kfree(g_gap_vertical_part);
	g_gap_vertical_part = NULL;

	return ret_val;
}

static int himax_gap_test_horizontal_setting(void)
{
	g_gap_horizontal_part = kcalloc(g_gap_horizontal_partial,
				sizeof(int), GFP_KERNEL);
	if (g_gap_horizontal_part == NULL) {
		E("%s: Memory allocation falied!\n", __func__);
		return MEM_ALLOC_FAIL;
	}
	g_gap_horizontal_part[0] = 0;
	g_gap_horizontal_part[1] = 8;
	g_gap_horizontal_part[2] = 24;

	return NO_ERR;
}

static void himax_cal_gap_data_horizontal(int start, int end_idx, int direct,
				uint32_t *org_raw, uint32_t *result_raw)
{
	int i = 0;
	int j = 0;
	int rx_num = ic_data->HX_RX_NUM;
	int tx_num = ic_data->HX_TX_NUM;

	I("start=%d\n", start);
	I("end_idx=%d\n", end_idx);
	for (j = 0; j < tx_num; j++) {
		for (i = (start + (j*rx_num));
			i < (start + (j*rx_num) + end_idx); i++) {
			/* left - right */
			if (direct == 0) {
				if (i == (start + (j*rx_num)))
					result_raw[i] = 0;
				else
					result_raw[i] =
						org_raw[i-1] - org_raw[i];

			} else { /* right - left */
				if (i == ((start + (j*rx_num) + end_idx) - 1))
					result_raw[i] = 0;
				else
					result_raw[i] =
						org_raw[i + 1] - org_raw[i];
			}
		}
	}
}

static int himax_gap_test_honrizontal_raw(int test_type, uint32_t *raw)
{
	int rx_num = ic_data->HX_RX_NUM;
	int tx_num = ic_data->HX_TX_NUM;
	int tmp_start = 0;
	int tmp_end_idx = 0;
	int i_partial = 0;
	uint32_t *result_raw;
	int i = 0;
	int ret_val = NO_ERR;

	if (himax_gap_test_horizontal_setting())
		return MEM_ALLOC_FAIL;

	result_raw = kcalloc(ic_data->HX_TX_NUM * ic_data->HX_RX_NUM,
			sizeof(uint32_t), GFP_KERNEL);
	if (result_raw == NULL) {
		E("%s: Memory allocation falied!\n", __func__);
		goto alloc_result_raw_failed;
	}

	I("Print Horizontal ORG RAW\n");
	for (i = 0; i < tx_num*rx_num; i++) {
		I("%04d,", raw[i]);
		if (i > 0 && i%rx_num == (rx_num-1))
			I("\n");
	}

	for (i_partial = 0; i_partial < g_gap_horizontal_partial; i_partial++) {
		tmp_start	= g_gap_horizontal_part[i_partial];
		if (i_partial+1 == g_gap_horizontal_partial)
			tmp_end_idx = rx_num - g_gap_horizontal_part[i_partial];
		else
			tmp_end_idx = g_gap_horizontal_part[i_partial+1] -
				g_gap_horizontal_part[i_partial];

		if (i_partial % 2 == 0)
			himax_cal_gap_data_horizontal(tmp_start, tmp_end_idx,
						0, raw, result_raw);
		else
			himax_cal_gap_data_horizontal(tmp_start, tmp_end_idx,
						1, raw, result_raw);

	}
	I("Print Horizontal New RAW\n");
	for (i = 0; i < tx_num*rx_num; i++) {
		I("%04d,", result_raw[i]);
		if (i > 0 && i%rx_num == (rx_num-1))
			I("\n");
	}

	for (i = 0; i < tx_num*rx_num; i++) {
		if (result_raw[i] < g_inspection_criteria[IDX_GAP_HOR_RAWMIN][i]
		&&
		result_raw[i] > g_inspection_criteria[IDX_GAP_HOR_RAWMAX][i]) {
			ret_val = NO_ERR - i;
			break;
		}
	}

	/* himax_write_to_ic_flash_flow(0x1A800,result_raw,tx_num*rx_num); */
	kfree(result_raw);
alloc_result_raw_failed:
	kfree(g_gap_horizontal_part);
	g_gap_horizontal_part = NULL;

	return ret_val;
}
/*	 HX_GAP END*/
#define FAIL_IN_INDEX "%s: %s FAIL in index %d\n"
static uint32_t mpTestFunc(uint8_t checktype, uint32_t datalen)
{
	int32_t ret = 0;
	uint32_t i/*, j*/;
	uint32_t *RAW = NULL;
	uint16_t palm_num = 0;
	uint16_t noise_count = 0;
	int ret_val = -1;

	/*uint16_t* pInspectGridData = &gInspectGridData[0];*/
	/*uint16_t* pInspectNoiseData = &gInspectNoiseData[0];*/
	I("Now Check type = %d\n", checktype);

	RAW = kcalloc(datalen, sizeof(uint32_t), GFP_KERNEL);
	if (RAW == NULL) {
		E("%s, Failed to allocate memory\n", __func__);
		return HX_INSPECT_MEMALLCTFAIL;
	}

	if (himax_check_mode(checktype)) {
		/*himax_check_mode(checktype);*/

		I("Need Change Mode ,target=%s\n", g_himax_inspection_mode[checktype]);

		g_core_fp.fp_sense_off(true);
		hx_turn_on_mp_func(1);

#ifndef HX_ZERO_FLASH
		if (g_core_fp.fp_reload_disable != NULL)
			g_core_fp.fp_reload_disable(1);
#endif

		himax_switch_mode_inspection(checktype);

		switch (checktype) {
		case HIMAX_INSPECTION_SORTING:
			himax_set_N_frame(OTHERSFRAME, checktype);
			ret_val = HX_INSPECT_EOPEN;
			break;

		case HIMAX_INSPECTION_OPEN:
			himax_set_N_frame(OTHERSFRAME, checktype);
			ret_val = HX_INSPECT_EOPEN;
			break;

		case HIMAX_INSPECTION_MICRO_OPEN:
			himax_set_N_frame(OTHERSFRAME, checktype);
			ret_val = HX_INSPECT_EMOPEN;
			break;

		case HIMAX_INSPECTION_SHORT:
			himax_set_N_frame(OTHERSFRAME, checktype);
			ret_val = HX_INSPECT_ESHORT;
			break;

		case HIMAX_INSPECTION_RAWDATA:
			himax_set_N_frame(OTHERSFRAME, checktype);
			ret_val = HX_INSPECT_ERAW;
			break;

		case HIMAX_INSPECTION_BPN_RAWDATA:
			himax_set_N_frame(OTHERSFRAME, checktype);
			ret_val = HX_INSPECT_ERAW;
			break;

		case HIMAX_INSPECTION_WEIGHT_NOISE:
			himax_set_N_frame(NOISEFRAME, checktype);
			ret_val = HX_INSPECT_WT_ENOISE;
			break;

		case HIMAX_INSPECTION_ABS_NOISE:
			himax_set_N_frame(NOISEFRAME, checktype);
			ret_val = HX_INSPECT_ABS_ENOISE;
			break;

		case HIMAX_INSPECTION_GAPTEST_RAW:
			himax_set_N_frame(OTHERSFRAME, checktype);
			ret_val = HX_INSPECT_EGAP_RAW;
			break;

		case HIMAX_INSPECTION_ACT_IDLE_RAWDATA:
			himax_set_N_frame(NORMAL_IDLE_RAWDATA_NOISEFRAME, checktype);
			ret_val = HX_INSPECT_EACT_IDLE_RAW;
			break;

		case HIMAX_INSPECTION_ACT_IDLE_NOISE:
			himax_set_N_frame(NORMAL_IDLE_RAWDATA_NOISEFRAME, checktype);
			ret_val = HX_INSPECT_EACT_IDLE_NOISE;
			break;

		case HIMAX_INSPECTION_LPWUG_RAWDATA:
			himax_set_N_frame(OTHERSFRAME, checktype);
			ret_val = HX_INSPECT_ELPWUG_RAW;
			break;

		case HIMAX_INSPECTION_LPWUG_NOISE:
			himax_set_N_frame(LPWUG_NOISEFRAME, checktype);
			ret_val = HX_INSPECT_ELPWUG_NOISE;
			break;

		case HIMAX_INSPECTION_LPWUG_IDLE_RAWDATA:
			himax_set_N_frame(LPWUG_IDLE_RAWDATAFRAME, checktype);
			ret_val = HX_INSPECT_ELPWUG_IDLE_RAW;
			break;

		case HIMAX_INSPECTION_LPWUG_IDLE_NOISE:
			himax_set_N_frame(LPWUG_IDLE_NOISEFRAME, checktype);
			ret_val = HX_INSPECT_ELPWUG_IDLE_NOISE;
			break;

		default:
			E("UNDEFINED test type %d, exit\n", checktype);
			ret_val = 0xFFFFFFFF;
			break;
		}

/*
 *	if (checktype == HIMAX_INSPECTION_WEIGHT_NOISE ||
 *		checktype == HIMAX_INSPECTION_ABS_NOISE) {
 *		himax_set_N_frame(NOISEFRAME, checktype);
 *	} else if (checktype == HIMAX_INSPECTION_ACT_IDLE_RAWDATA ||
 *		checktype == HIMAX_INSPECTION_ACT_IDLE_NOISE) {
 *		I("N frame = %d\n", NORMAL_IDLE_RAWDATA_NOISEFRAME);
 *		himax_set_N_frame(NORMAL_IDLE_RAWDATA_NOISEFRAME, checktype);
 *	} else if (checktype == HIMAX_INSPECTION_LPWUG_RAWDATA) {
 *		I("N frame = %d\n", LPWUG_RAWDATAFRAME);
 *		himax_set_N_frame(LPWUG_RAWDATAFRAME, checktype);
 *	} else if (checktype == HIMAX_INSPECTION_LPWUG_NOISE) {
 *		I("N frame = %d\n", LPWUG_NOISEFRAME);
 *		himax_set_N_frame(LPWUG_NOISEFRAME, checktype);
 *	} else if (checktype == HIMAX_INSPECTION_LPWUG_IDLE_RAWDATA) {
 *		I("N frame = %d\n", LPWUG_IDLE_RAWDATAFRAME);
 *		himax_set_N_frame(LPWUG_IDLE_RAWDATAFRAME, checktype);
 *	} else if (checktype == HIMAX_INSPECTION_LPWUG_IDLE_NOISE) {
 *		I("N frame = %d\n", LPWUG_IDLE_NOISEFRAME);
 *		himax_set_N_frame(LPWUG_IDLE_NOISEFRAME, checktype);
 *	} else {
 *		himax_set_N_frame(OTHERSFRAME, checktype);
 *	}
 */

		g_core_fp.fp_sense_on(1);

		if (ret_val == 0xFFFFFFFF)
			return ret_val;
	}

	ret = himax_wait_sorting_mode(checktype);
	if (ret) {
		E("%s: himax_wait_sorting_mode FAIL\n", __func__);
		return ret_val;
	}
	himax_switch_data_type(checktype);

	ret = himax_get_rawdata(RAW, datalen);
	if (ret) {
		E("%s: himax_get_rawdata FAIL\n", __func__);
		return ret_val | ret;
	}

	/*get Max DC from FW*/
	g_dc_max = g_core_fp.fp_get_max_dc();

	/* back to normal */
	himax_switch_data_type(HIMAX_INSPECTION_BACK_NORMAL);

	I("%s: Init OK, start to test!\n", __func__);

	snprintf(g_start_log, 256 * sizeof(char), "\n%s%s\n",
		g_himax_inspection_mode[checktype], ": data as follow!\n");

	/*Check Data*/
	switch (checktype) {
	case HIMAX_INSPECTION_SORTING:
		for (i = 0; i < (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM); i++) {
			if ((int)RAW[i] < g_inspection_criteria[IDX_SORTMIN][i]) {
				E(FAIL_IN_INDEX, __func__, "sorting mode open test", i);
				ret_val = HX_INSPECT_EOPEN;
				goto FAIL_END;
			}
		}
		I("%s: sorting mode open test PASS\n", __func__);
		break;

	case HIMAX_INSPECTION_OPEN:
		for (i = 0; i < (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM); i++) {
			if ((int)RAW[i] > g_inspection_criteria[IDX_OPENMAX][i]
				|| (int)RAW[i] < g_inspection_criteria[IDX_OPENMIN][i]) {
				E(FAIL_IN_INDEX, __func__, "open test", i);
				ret_val = HX_INSPECT_EOPEN;
				goto FAIL_END;
			}
		}
		I("%s: open test PASS\n", __func__);
		break;

	case HIMAX_INSPECTION_MICRO_OPEN:
		for (i = 0; i < (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM); i++) {
			if ((int)RAW[i] > g_inspection_criteria[IDX_M_OPENMAX][i]
				|| (int)RAW[i] < g_inspection_criteria[IDX_M_OPENMIN][i]) {
				E(FAIL_IN_INDEX, __func__, "micro open test", i);
				ret_val = HX_INSPECT_EMOPEN;
				goto FAIL_END;
			}
		}
		I("%s: micro open test PASS\n", __func__);
		break;

	case HIMAX_INSPECTION_SHORT:
		for (i = 0; i < (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM); i++) {
			if ((int)RAW[i] > g_inspection_criteria[IDX_SHORTMAX][i]
				|| (int)RAW[i] < g_inspection_criteria[IDX_SHORTMIN][i]) {
				E(FAIL_IN_INDEX, __func__, "short test", i);
				ret_val = HX_INSPECT_ESHORT;
				goto FAIL_END;
			}
		}
		I("%s: short test PASS\n", __func__);
		break;

	case HIMAX_INSPECTION_RAWDATA:
		for (i = 0; i < (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM); i++) {
			/* I("Now new compare, datalen=%d!\n",
			 * ic_data->HX_TX_NUM*ic_data->HX_RX_NUM);
			 */
			if ((int)RAW[i] > g_inspection_criteria[IDX_RAWMAX][i]
				|| (int)RAW[i] < g_inspection_criteria[IDX_RAWMIN][i]) {
				E("%s: rawdata test FAIL:RAW[%d]=%d\n", __func__, i, RAW[i]);
				I("%s: Now Criteria max=%d,min=%d\n", __func__,
					g_inspection_criteria[IDX_RAWMAX][i],
					g_inspection_criteria[IDX_RAWMIN][i]);
				ret_val = HX_INSPECT_ERAW;
				goto FAIL_END;
			}
		}
		I("%s: rawdata test PASS\n", __func__);
		break;

	case HIMAX_INSPECTION_BPN_RAWDATA:
		for (i = 0; i < (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM); i++) {
			/* I("Now new compare, datalen=%d!\n",
			 * ic_data->HX_TX_NUM*ic_data->HX_RX_NUM);
			 */
			RAW[i] = (int)RAW[i] * 100 / g_dc_max;
			if ((int)RAW[i] > g_inspection_criteria[IDX_BPN_RAWMAX][i]
				|| (int)RAW[i] < g_inspection_criteria[IDX_BPN_RAWMIN][i]) {
				E("%s: rawdata test FAIL:BPN RAW[%d]=%d\n", __func__, i, RAW[i]);
				I("%s: Now Criteria max=%d,min=%d\n", __func__,
					g_inspection_criteria[IDX_BPN_RAWMAX][i],
					g_inspection_criteria[IDX_BPN_RAWMIN][i]);
				ret_val = HX_INSPECT_ERAW;
				goto FAIL_END;
			}
		}
		I("%s: BPN rawdata test PASS\n", __func__);
		break;

	case HIMAX_INSPECTION_WEIGHT_NOISE:
		/*I("NOISEMAX=%d\n", NOISEMAX);*/
		noise_count = 0;
		himax_get_noise_base();
		palm_num = himax_get_palm_num();
		for (i = 0; i < (ic_data->HX_TX_NUM * ic_data->HX_RX_NUM); i++) {
			if ((int)RAW[i] > NOISEMAX)
				noise_count++;
		}
		I("noise_count=%d\n", noise_count);
		if (noise_count > palm_num) {
			E("%s: noise test FAIL\n", __func__);
			ret_val = HX_INSPECT_WT_ENOISE;
			goto FAIL_END;
		}
		snprintf(g_start_log, 256 * sizeof(char), "\n Threshold = %d\n", NOISEMAX);
		/*Check weightingt*/
		ret = himax_get_noise_weight_test();
		if (ret < 0) {
			I("%s: noise test FAIL %d\n", __func__, ret);
			ret_val = HX_INSPECT_WT_ENOISE;
			goto FAIL_END;
		}

		/*Check negative side noise*/
		for (i = 0; i < (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM); i++) {
			if ((int)RAW[i] > (g_inspection_criteria[IDX_WT_NOISEMAX][i] * g_recal_thx / 100)
				|| (int)RAW[i] < (g_inspection_criteria[IDX_WT_NOISEMIN][i] * g_recal_thx / 100)) {
				E(FAIL_IN_INDEX, __func__, "HIMAX_INSPECTION_WEIGHT_NOISE", i);
				ret_val = HX_INSPECT_WT_ENOISE;
				goto FAIL_END;
			}
		}
		I("%s: noise test(weight) PASS\n", __func__);
		break;

	case HIMAX_INSPECTION_ABS_NOISE:
		for (i = 0; i < (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM); i++) {
			if ((int)RAW[i] > g_inspection_criteria[IDX_ABS_NOISEMAX][i]
			|| (int)RAW[i] < g_inspection_criteria[IDX_ABS_NOISEMIN][i]) {
				E(FAIL_IN_INDEX, __func__, "HIMAX_INSPECTION_ABS_NOISE", i);
				I("%s: Now Criteria max=%d,min=%d\n", __func__,
				  g_inspection_criteria[IDX_ABS_NOISEMAX][i],
				  g_inspection_criteria[IDX_ABS_NOISEMIN][i]);
				ret_val = HX_INSPECT_ABS_ENOISE;
				goto FAIL_END;
			}
		}
		I("%s: noise test(absolute) PASS\n", __func__);
		break;

	case HIMAX_INSPECTION_GAPTEST_RAW:
		if (himax_gap_test_vertical_raw(HIMAX_INSPECTION_GAPTEST_RAW, RAW) != NO_ERR) {
			E("%s: HIMAX_INSPECTION_GAPTEST_RAW FAIL\n", __func__);
			ret_val = HX_INSPECT_EGAP_RAW;
			goto FAIL_END;
		}
		if (himax_gap_test_honrizontal_raw(HIMAX_INSPECTION_GAPTEST_RAW, RAW) != NO_ERR) {
			E("%s: HIMAX_INSPECTION_GAPTEST_RAW FAIL\n", __func__);
			ret_val = HX_INSPECT_EGAP_RAW;
			goto FAIL_END;
		}
		break;

	case HIMAX_INSPECTION_ACT_IDLE_RAWDATA:
		for (i = 0; i < (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM); i++) {
			if ((int)RAW[i] > g_inspection_criteria[IDX_ACT_IDLE_RAWDATA_MAX][i]
				|| (int)RAW[i] < g_inspection_criteria[IDX_ACT_IDLE_RAWDATA_MIN][i]) {
				E(FAIL_IN_INDEX, __func__, "HIMAX_INSPECTION_ACT_IDLE_RAWDATA", i);
				ret_val = HX_INSPECT_EACT_IDLE_RAW;
				goto FAIL_END;
			}
		}
		I("%s: HIMAX_INSPECTION_ACT_IDLE_RAWDATA PASS\n", __func__);
		break;

	case HIMAX_INSPECTION_ACT_IDLE_NOISE:
		for (i = 0; i < (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM); i++) {
			if ((int)RAW[i] > g_inspection_criteria[IDX_ACT_IDLE_NOISE_MAX][i]
				|| (int)RAW[i] < g_inspection_criteria[IDX_ACT_IDLE_NOISE_MIN][i]) {
				E(FAIL_IN_INDEX, __func__, "HIMAX_INSPECTION_ACT_IDLE_NOISE", i);
				ret_val = HX_INSPECT_EACT_IDLE_NOISE;
				goto FAIL_END;
			}
		}
		I("%s: HIMAX_INSPECTION_ACT_IDLE_NOISE PASS\n", __func__);
		break;

	case HIMAX_INSPECTION_LPWUG_RAWDATA:
		for (i = 0; i < (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM); i++) {
			if ((int)RAW[i] > g_inspection_criteria[IDX_LPWUG_RAWDATA_MAX][i]
				|| (int)RAW[i] < g_inspection_criteria[IDX_LPWUG_RAWDATA_MIN][i]) {
				E(FAIL_IN_INDEX, __func__, "HIMAX_INSPECTION_LPWUG_RAWDATA", i);
				ret_val = HX_INSPECT_ELPWUG_RAW;
				goto FAIL_END;
			}
		}
		I("%s: HIMAX_INSPECTION_LPWUG_RAWDATA PASS\n", __func__);
		break;

	case HIMAX_INSPECTION_LPWUG_NOISE:
		for (i = 0; i < (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM); i++) {
			if ((int)RAW[i] > g_inspection_criteria[IDX_LPWUG_NOISE_MAX][i]
				|| (int)RAW[i] < g_inspection_criteria[IDX_LPWUG_NOISE_MIN][i]) {
				E(FAIL_IN_INDEX, __func__, "HIMAX_INSPECTION_LPWUG_NOISE", i);
				ret_val = HX_INSPECT_ELPWUG_NOISE;
				goto FAIL_END;
			}
		}
		I("%s: HIMAX_INSPECTION_LPWUG_NOISE PASS\n", __func__);
		break;

	case HIMAX_INSPECTION_LPWUG_IDLE_RAWDATA:
		for (i = 0; i < (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM); i++) {
			if ((int)RAW[i] > g_inspection_criteria[IDX_LPWUG_IDLE_RAWDATA_MAX][i]
				|| (int)RAW[i] < g_inspection_criteria[IDX_LPWUG_IDLE_RAWDATA_MIN][i]) {
				E(FAIL_IN_INDEX, __func__, "HIMAX_INSPECTION_LPWUG_IDLE_RAWDATA", i);
				ret_val = HX_INSPECT_ELPWUG_IDLE_RAW;
				goto FAIL_END;
			}
		}
		I("%s: HIMAX_INSPECTION_LPWUG_IDLE_RAWDATA PASS\n", __func__);
		break;

	case HIMAX_INSPECTION_LPWUG_IDLE_NOISE:
		for (i = 0; i < (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM); i++) {
			if ((int)RAW[i] > g_inspection_criteria[IDX_LPWUG_IDLE_NOISE_MAX][i]
				|| (int)RAW[i] < g_inspection_criteria[IDX_LPWUG_IDLE_NOISE_MIN][i]) {
				E(FAIL_IN_INDEX, __func__, "HIMAX_INSPECTION_LPWUG_IDLE_NOISE", i);
				ret_val = HX_INSPECT_ELPWUG_IDLE_NOISE;
				goto FAIL_END;
			}
		}
		I("%s: HIMAX_INSPECTION_LPWUG_IDLE_NOISE PASS\n", __func__);
		break;

	default:
		E("Wrong type=%d\n", checktype);
		ret_val = 0xFFFFFFFF;
		break;
	}

	ret_val = HX_INSPECT_OK;
	snprintf(g_rslt_log, 256 * sizeof(char), "\n%s%s\n",
	  g_himax_inspection_mode[checktype], " Test Pass!\n");
	I("pass write log\n");
	goto END_FUNC;

FAIL_END:
	snprintf(g_rslt_log, 256 * sizeof(char), "\n%s%s\n",
	  g_himax_inspection_mode[checktype], " Test Fail!\n");
	I("fail write log\n");
END_FUNC:
	hx_test_data_get(RAW, g_start_log, g_rslt_log, checktype);
	kfree(RAW);

	return ret_val;
}

/* claculate 10's power function */
static int himax_power_cal(int pow, int number)
{
	int i = 0;
	int result = 1;

	for (i = 0; i < pow; i++)
		result *= 10;
	result = result * number;

	return result;

}

/* String to int */
static int hiamx_parse_str2int(char *str)
{
	int i = 0;
	int temp_cal = 0;
	int result = 0;
	int str_len = strlen(str);
	int negtive_flag = 0;

	for (i = 0; i < strlen(str); i++) {
		if (str[i] != '-' && str[i] > '9' && str[i] < '0') {
			E("%s: Parsing fail!\n", __func__);
			result = -9487;
			negtive_flag = 0;
			break;
		}
		if (str[i] == '-') {
			negtive_flag = 1;
			continue;
		}
		temp_cal = str[i] - '0';
		result += himax_power_cal(str_len-i-1, temp_cal);
		/* str's the lowest char is the number's the highest number
		 * So we should reverse this number before using the power
		 * function
		 * -1: starting number is from 0 ex:10^0 = 1,10^1=10
		 */
	}

	if (negtive_flag == 1)
		result = 0 - result;

	return result;
}


/* Get sub-string from original string by using some charaters
 * return size of result
 */
static int himax_saperate_comma(const struct firmware *file_entry,
				char **result, int str_size)
{
	int count = 0;
	int str_count = 0; /* now string*/
	int char_count = 0; /* now char count in string*/

	do {
		switch (file_entry->data[count]) {
		case ASCII_COMMA:
		case ACSII_SPACE:
		case ASCII_CR:
		case ASCII_LF:
			count++;
			/* If end of line as above condifiton,
			 * differencing the count of char.
			 * If char_count != 0
			 * it's meaning this string is parsing over .
			 * The Next char is belong to next string
			 */
			if (char_count != 0) {
				char_count = 0;
				str_count++;
			}
			break;
		default:
			result[str_count][char_count++] =
				file_entry->data[count];
			count++;
			break;
		}
	} while (count < file_entry->size && str_count < str_size);

	return str_count;
}

static int hx_diff_str(char *str1, char *str2)
{
	int i = 0;
	int result = 0; /* zero is all same, non-zero is not same index*/
	int str1_len = strlen(str1);
	int str2_len = strlen(str2);

	if (str1_len != str2_len) {
		if (private_ts->debug_log_level & BIT(4))
			I("%s:Size different!\n", __func__);
		return LENGTH_FAIL;
	}

	for (i = 0; i < str1_len; i++) {
		if (str1[i] != str2[i]) {
			result = i+1;
			I("%s: different in %d!\n", __func__, result);
			return result;
		}
	}

	return result;
}

/* get idx of criteria whe parsing file */
int hx_find_crtra_id(char *input)
{
	int i = 0;
	int result = 0;

	for (i = 0 ; i < HX_CRITERIA_SIZE ; i++) {
		if (hx_diff_str(g_hx_inspt_crtra_name[i], input) == 0) {
			result = i;
			I("find the str=%s,idx=%d\n",
			  g_hx_inspt_crtra_name[i], i);
			break;
		}
	}
	if (i > (HX_CRITERIA_SIZE - 1)) {
		E("%s: find Fail!\n", __func__);
		return LENGTH_FAIL;
	}

	return result;
}

int hx_print_crtra_after_parsing(void)
{
	int i = 0, j = 0;
	int all_mut_len = ic_data->HX_TX_NUM*ic_data->HX_RX_NUM;

	for (i = 0; i < HX_CRITERIA_SIZE; i++) {
		I("Now is %s\n", g_hx_inspt_crtra_name[i]);
		if (g_inspt_crtra_flag[i] == 1) {
			for (j = 0; j < all_mut_len; j++) {
				I("%d, ", g_inspection_criteria[i][j]);
				if (j % 16 == 15)
					PI("\n");
			}
		} else {
			I("No this Item in this criteria file!\n");
		}
		PI("\n");
	}

	return 0;
}

static int hx_get_crtra_by_name(char **result, int size_of_result_str)
{
	int i = 0;
	/* count of criteria type */
	int count_type = 0;
	/* count of criteria data */
	int count_data = 0;
	int err = HX_INSPECT_OK;
	int all_mut_len = ic_data->HX_TX_NUM*ic_data->HX_RX_NUM;
	int temp = 0;

	/* get criteria and assign to a global array(2-Dimensional/int) */
	/* basiclly the size of criteria will be
	 * (crtra_count * (all_mut_len) + crtra_count)
	 * but we use file size to be the end of counter
	 */
	for (i = 0; i < size_of_result_str && result[i] != NULL; i++) {
		/* It have get one page(all mutual) criteria data!
		 * And we should skip the string of criteria name!
		 */
		if (i == 0 || i ==
		 ((i / (all_mut_len))+(i / (all_mut_len) * (all_mut_len)))) {
			count_data = 0;

			if (private_ts->debug_log_level & BIT(4))
				I("Now find str=%s ,idx=%d\n", result[i], i);

			/* check the item of criteria is in criteria file
			 * or not
			 */
			count_type = hx_find_crtra_id(result[i]);
			if (count_type < 0) {
				E("1. %s:Name Not match!\n", __func__);
				/* E("can recognize[%d]=%s\n", count_type,
				 * g_hx_inspt_crtra_name[count_type]);
				 */
				E("get from file[%d]=%s\n", i, result[i]);
				E("Please check criteria file again!\n");
				err = HX_INSPECT_EFILE;
				goto END_FUNCTION;
			} else {
				I("Now str=%s, idx=%d\n",
				g_hx_inspt_crtra_name[count_type], count_type);
				g_inspt_crtra_flag[count_type] = 1;
			}
			continue;
		}
		/* change string to int*/
		temp = hiamx_parse_str2int(result[i]);
		if (temp != -9487)
			g_inspection_criteria[count_type][count_data] = temp;
		else {
			E("%s: Parsing Fail in %d\n", __func__, i);
			E("in range:[%d]=%s\n", count_type,
			  g_hx_inspt_crtra_name[count_type]);
			E("btw, get from file[%d]=%s\n", i, result[i]);
			break;
		}
		/* dbg
		 * I("[%d]g_inspection_criteria[%d][%d]=%d\n",
		 * i, count_type, count_data,
		 * g_inspection_criteria[count_type][count_data]);
		 */
		count_data++;

	}

	if (private_ts->debug_log_level & BIT(4)) {
		/* dbg:print all of criteria from parsing file */
		hx_print_crtra_after_parsing();
	}

	I("Total loop=%d\n", i);
END_FUNCTION:
	return err;
}

static int himax_parse_criteria_file(void)
{
	int err = HX_INSPECT_OK;
	const struct firmware *file_entry = NULL;
	char **result;
	int i = 0;
	int crtra_count = HX_CRITERIA_SIZE;
	int data_size = 0; /* The maximum of number Data*/
	int all_mut_len = ic_data->HX_TX_NUM*ic_data->HX_RX_NUM;
	int str_max_len = 128;
	int result_all_len = 0;
	int file_size = 0;
	int size_of_result_str = 0;

	I("%s,Entering\n", __func__);
	if (hx_self_test_file_name == NULL) {
		E("file name is NULL\n");
		hx_self_test_file_name = kzalloc(80, GFP_KERNEL);
		if (private_ts->pdata->panel_supplier[0])
			snprintf(hx_self_test_file_name, 60, "%s_hx_criteria.csv", private_ts->pdata->panel_supplier);
		else
			snprintf(hx_self_test_file_name, 16, "hx_criteria.csv");
		I("%s: Use name %s\n", __func__, hx_self_test_file_name);
	}

	I("file name = %s\n", hx_self_test_file_name);
	/* default path is /system/etc/firmware */
	err = request_firmware(&file_entry, hx_self_test_file_name, private_ts->dev);
	if (err < 0) {
		E("%s,fail in line%d error code=%d\n", __func__, __LINE__, err);
		err = HX_INSPECT_EFILE;
		goto END_FUNC_REQ_FAIL;
	}

	/* size of criteria include name string */
	data_size = ((all_mut_len) * crtra_count) + crtra_count;

	/* init the array which store original criteria and include
	 *  name string
	 */
	result = kcalloc(data_size, sizeof(char *), GFP_KERNEL);
	if (result != NULL) {
		for (i = 0 ; i < data_size; i++) {
			result[i] = kcalloc(str_max_len, sizeof(char), GFP_KERNEL);
			if (result[i] == NULL) {
				E("%s: rst_arr Memory allocation falied!\n", __func__);
				err = HX_INSPECT_MEMALLCTFAIL;
				goto rst_arr_mem_alloc_failed;
			}
		}
	} else {
		E("%s: Memory allocation falied!\n", __func__);
		err = HX_INSPECT_MEMALLCTFAIL;
		goto rst_mem_alloc_failed;
	}

	result_all_len = data_size;
	file_size =	file_entry->size;
	I("Now result_all_len=%d\n", result_all_len);
	I("Now file_size=%d\n", file_size);

	/* dbg */
	if (private_ts->debug_log_level & BIT(4)) {
		I("first 4 bytes 0x%2X,0x%2X,0x%2X,0x%2X !\n",
		  file_entry->data[0], file_entry->data[1],
		  file_entry->data[2], file_entry->data[3]);
	}

	/* parse value in to result array(1-Dimensional/String) */
	size_of_result_str =
		himax_saperate_comma(file_entry, result, data_size);

	I("%s: now size_of_result_str=%d\n", __func__, size_of_result_str);

	err = hx_get_crtra_by_name(result, size_of_result_str);
	if (err != HX_INSPECT_OK) {
		E("%s:Load criteria from file fail, go end!\n", __func__);
		goto END_FUNC;
	}


END_FUNC:
rst_arr_mem_alloc_failed:
	for (i = 0 ; i < data_size; i++)
		if (result[i] != NULL)
			kfree(result[i]);
rst_mem_alloc_failed:
	kfree(result);
	release_firmware(file_entry);
END_FUNC_REQ_FAIL:
	I("%s,END\n", __func__);
	return err;
	/* parsing Criteria end */
}


int hx_get_size_str_arr(char **input)
{
	int i = 0;
	int result = 0;

	while (input[i] != NULL)
		i++;

	result = i;
	if (private_ts->debug_log_level & BIT(4))
		I("There is %d in [0]=%s\n", result, input[0]);

	return result;
}

static int himax_self_test_data_init(void)
{
	int ret = HX_INSPECT_OK;
	int i = 0;

	/*
	 * 5: one value will not over than 99999, so get this size of string
	 * 2: get twice size
	 */
	g_1kind_raw_size = 5 * ic_data->HX_RX_NUM * ic_data->HX_RX_NUM * 2;

	/* get test item and its items of criteria*/
	HX_CRITERIA_ITEM = hx_get_size_str_arr(g_himax_inspection_mode);
	HX_CRITERIA_SIZE = hx_get_size_str_arr(g_hx_inspt_crtra_name);
	I("There is %d HX_CRITERIA_ITEM and %d HX_CRITERIA_SIZE\n",
	  HX_CRITERIA_ITEM, HX_CRITERIA_SIZE);

	/* init criteria data*/
	g_inspt_crtra_flag = kzalloc(HX_CRITERIA_SIZE*sizeof(int), GFP_KERNEL);
	g_inspection_criteria = kzalloc(sizeof(int *)*HX_CRITERIA_SIZE, GFP_KERNEL);
	if (g_inspt_crtra_flag == NULL || g_inspection_criteria == NULL) {
		E("%s: %d, Memory allocation falied!\n", __func__, __LINE__);
		return MEM_ALLOC_FAIL;
	}

	for (i = 0; i < HX_CRITERIA_SIZE; i++) {
		g_inspection_criteria[i] = kcalloc(
		  (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM), sizeof(int), GFP_KERNEL);
		if (g_inspection_criteria[i] == NULL) {
			E("%s: %d, Memory allocation falied!\n", __func__, __LINE__);
			return MEM_ALLOC_FAIL;
		}
	}

	/* parsing criteria from file*/
	ret = himax_parse_criteria_file();

	if (private_ts->debug_log_level & BIT(4)) {
		/* print get criteria string */
		for (i = 0 ; i < HX_CRITERIA_SIZE ; i++) {
			if (g_inspt_crtra_flag[i] != 0)
				I("%s: [%d]There is String=%s\n",
				  __func__, i, g_hx_inspt_crtra_name[i]);
		}
	}

	if (g_rslt_data == NULL) {
		g_rslt_data = kcalloc(g_1kind_raw_size * HX_CRITERIA_ITEM,
			  sizeof(char), GFP_KERNEL);
		I("g_rslt_data is NULL");
	} else {
		memset(g_rslt_data, 0x00, g_1kind_raw_size * HX_CRITERIA_ITEM *
			  sizeof(char));
	}

	snprintf(g_file_path,
		  (int)(strlen(HX_RSLT_OUT_PATH) + strlen(HX_RSLT_OUT_FILE)+1),
		  "%s%s", HX_RSLT_OUT_PATH, HX_RSLT_OUT_FILE);

	return ret;
}

static void himax_self_test_data_deinit(void)
{
	int i = 0;

	/*dbg*/
	/* for (i = 0; i < HX_CRITERIA_ITEM; i++)
	 *	I("%s:[%d]%d\n", __func__, i, g_inspection_criteria[i]);
	 */
	if (g_inspection_criteria != NULL) {
		for (i = 0; i < HX_CRITERIA_SIZE; i++) {
			if (g_inspection_criteria[i] != NULL)
				kfree(g_inspection_criteria[i]);
		}
		kfree(g_inspection_criteria);
		I("Now it have free the g_inspection_criteria!\n");
	} else {
		I("No Need to free g_inspection_criteria!\n");
	}

	g_rslt_data_len = 0;

}

static int himax_chip_self_test(void)
{
	uint32_t ret = HX_INSPECT_OK;
	uint8_t tmp_addr[DATA_LEN_4] = {0x94, 0x72, 0x00, 0x10};
	uint8_t tmp_data[DATA_LEN_4] = {0x01, 0x00, 0x00, 0x00};

#if defined(HIMAX_V2_MULTI_BIN)||defined(HX_CODE_OVERLAY)
	uint8_t normalfw[32] = "Himax_firmware.bin";
	uint8_t mpapfw[32] = "Himax_mpfw.bin";
	if (private_ts->pdata->panel_supplier[0]) {
		snprintf(normalfw, 32, "%s_Himax_firmware.bin", private_ts->pdata->panel_supplier);
		snprintf(mpapfw, 32, "%s_Himax_mpfw.bin", private_ts->pdata->panel_supplier);
	}
#endif

	I("%s:IN\n", __func__);

	private_ts->suspend_resume_done = 0;

	ret = himax_self_test_data_init();
	if (ret != HX_INSPECT_OK) {
		E("%s: himax_self_test_data_init fail!\n", __func__);
		goto END_FUNC;
	}

#if defined(HIMAX_V2_MULTI_BIN)||defined(HX_CODE_OVERLAY)
	g_core_fp.fp_0f_op_file_dirly(mpapfw);
	g_core_fp.fp_reload_disable(0);
	g_core_fp.fp_sense_on(0x00);
#endif

	if (g_inspt_crtra_flag[IDX_OPENMIN] == 1 &&
	  g_inspt_crtra_flag[IDX_OPENMAX] == 1) {
		/*1. Open Test*/
		I("[MP_OPEN_TEST_RAW]\n");
		ret |= mpTestFunc(HIMAX_INSPECTION_OPEN,
		  (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM) + ic_data->HX_TX_NUM
		  + ic_data->HX_RX_NUM);
		I("1. Open Test: End %d\n\n\n", ret);
	} else {
		I("Now %s :flag=%d\n", g_hx_inspt_crtra_name[IDX_OPENMIN],
		  g_inspt_crtra_flag[IDX_OPENMIN]);
		I("Now %s :flag=%d\n", g_hx_inspt_crtra_name[IDX_OPENMAX],
		  g_inspt_crtra_flag[IDX_OPENMAX]);
	}

	if (g_inspt_crtra_flag[IDX_M_OPENMIN] == 1 &&
		  g_inspt_crtra_flag[IDX_M_OPENMAX] == 1) {
		/*2. Micro-Open Test*/
		I("[MP_MICRO_OPEN_TEST_RAW]\n");
		ret |= mpTestFunc(HIMAX_INSPECTION_MICRO_OPEN,
		  (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM) + ic_data->HX_TX_NUM
		  + ic_data->HX_RX_NUM);
		I("2. Micro Open Test: End %d\n\n\n", ret);
	} else {
		I("Now %s :flag=%d\n", g_hx_inspt_crtra_name[IDX_M_OPENMIN],
		  g_inspt_crtra_flag[IDX_M_OPENMIN]);
		I("Now %s :flag=%d\n", g_hx_inspt_crtra_name[IDX_M_OPENMAX],
		  g_inspt_crtra_flag[IDX_M_OPENMAX]);
	}

	if (g_inspt_crtra_flag[IDX_SHORTMIN] == 1 &&
		  g_inspt_crtra_flag[IDX_SHORTMAX] == 1) {
		/*3. Short Test*/
		I("[MP_SHORT_TEST_RAW]\n");
		ret |= mpTestFunc(HIMAX_INSPECTION_SHORT,
		  (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM) + ic_data->HX_TX_NUM
		  + ic_data->HX_RX_NUM);
		I("3. Short Test: End %d\n\n\n", ret);
	} else {
		I("Now %s :flag=%d\n", g_hx_inspt_crtra_name[IDX_SHORTMIN],
		  g_inspt_crtra_flag[IDX_SHORTMIN]);
		I("Now %s :flag=%d\n", g_hx_inspt_crtra_name[IDX_SHORTMAX],
		  g_inspt_crtra_flag[IDX_SHORTMAX]);
	}

	if (g_inspt_crtra_flag[IDX_RAWMIN] == 1 &&
		  g_inspt_crtra_flag[IDX_RAWMAX] == 1) {
		/*4. RawData Test*/
		I("==========================================\n");
		I("[MP_RAW_TEST_RAW]\n");
		ret |= mpTestFunc(HIMAX_INSPECTION_RAWDATA,
		  (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM) + ic_data->HX_TX_NUM
		  + ic_data->HX_RX_NUM);
		I("%d. %s: End %d\n\n\n", HIMAX_INSPECTION_RAWDATA,
		  g_himax_inspection_mode[HIMAX_INSPECTION_RAWDATA], ret);
	} else {
		I("Now %s :flag=%d\n", g_hx_inspt_crtra_name[IDX_RAWMIN],
		  g_inspt_crtra_flag[IDX_RAWMIN]);
		I("Now %s :flag=%d\n", g_hx_inspt_crtra_name[IDX_RAWMAX],
		  g_inspt_crtra_flag[IDX_RAWMAX]);
	}

	if (g_inspt_crtra_flag[IDX_BPN_RAWMIN] == 1 &&
		  g_inspt_crtra_flag[IDX_BPN_RAWMAX] == 1) {
		/*4. RawData BPN Test*/
		I("==========================================\n");
		I("[MP_BPN_RAW_TEST_RAW]\n");
		ret |= mpTestFunc(HIMAX_INSPECTION_BPN_RAWDATA,
		  (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM) + ic_data->HX_TX_NUM
		  + ic_data->HX_RX_NUM);
		I("%d. %s: End %d\n\n\n", HIMAX_INSPECTION_BPN_RAWDATA,
		  g_himax_inspection_mode[HIMAX_INSPECTION_BPN_RAWDATA], ret);
	} else {
		I("Now %s :flag=%d\n", g_hx_inspt_crtra_name[IDX_BPN_RAWMIN],
		  g_inspt_crtra_flag[IDX_BPN_RAWMIN]);
		I("Now %s :flag=%d\n", g_hx_inspt_crtra_name[IDX_BPN_RAWMAX],
		  g_inspt_crtra_flag[IDX_BPN_RAWMAX]);
	}

	if (g_inspt_crtra_flag[IDX_WT_NOISEMIN] == 1 &&
		g_inspt_crtra_flag[IDX_WT_NOISEMAX] == 1) {
		/*5. Noise Test by Weighting*/
		I("[MP_NOISE_TEST_WEIGHT]\n");
		ret |= mpTestFunc(HIMAX_INSPECTION_WEIGHT_NOISE,
		  (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM) + ic_data->HX_TX_NUM
		  + ic_data->HX_RX_NUM);
		I("5. Noise Test: End %d\n\n\n", ret);
	} else {
		I("Now %s :flag=%d\n", g_hx_inspt_crtra_name[IDX_WT_NOISEMAX],
		  g_inspt_crtra_flag[IDX_WT_NOISEMAX]);
		I("Now %s :flag=%d\n", g_hx_inspt_crtra_name[IDX_WT_NOISEMIN],
		  g_inspt_crtra_flag[IDX_WT_NOISEMIN]);
	}

	if (g_inspt_crtra_flag[IDX_ABS_NOISEMIN] == 1 &&
		g_inspt_crtra_flag[IDX_ABS_NOISEMAX] == 1) {
		/*5. Noise Test by per block*/
		I("[MP_NOISE_TEST_ABS]\n");
		ret |= mpTestFunc(HIMAX_INSPECTION_ABS_NOISE,
		  (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM) + ic_data->HX_TX_NUM
		  + ic_data->HX_RX_NUM);
		I("5. Noise Test: End %d\n\n\n", ret);
	} else {
		I("Now %s :flag=%d\n", g_hx_inspt_crtra_name[IDX_ABS_NOISEMAX],
		  g_inspt_crtra_flag[IDX_ABS_NOISEMAX]);
		I("Now %s :flag=%d\n", g_hx_inspt_crtra_name[IDX_ABS_NOISEMIN],
		  g_inspt_crtra_flag[IDX_ABS_NOISEMIN]);
	}

	if (g_inspt_crtra_flag[IDX_SORTMIN] == 1 &&
		  g_inspt_crtra_flag[IDX_SORTMAX] == 1) {
		/*6. Sorting Test*/
		I("[SORTING TEST]\n");
		ret |= mpTestFunc(HIMAX_INSPECTION_SORTING,
		  (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM) + ic_data->HX_TX_NUM
		  + ic_data->HX_RX_NUM);
		I("6. SORTING TEST: End %d\n\n\n", ret);
	} else {
		I("Now %s :flag=%d\n", g_hx_inspt_crtra_name[IDX_SORTMIN],
		  g_inspt_crtra_flag[IDX_SORTMIN]);
		I("Now %s :flag=%d\n", g_hx_inspt_crtra_name[IDX_SORTMAX],
		  g_inspt_crtra_flag[IDX_SORTMAX]);
	}

	if ((g_inspt_crtra_flag[IDX_GAP_HOR_RAWMAX] == 1 &&
	  g_inspt_crtra_flag[IDX_GAP_HOR_RAWMIN] == 1) &&
	  (g_inspt_crtra_flag[IDX_GAP_VER_RAWMAX] == 1 &&
	  g_inspt_crtra_flag[IDX_GAP_VER_RAWMIN] == 1)) {
		/*6. GAP Test*/
		I("[MP_GAP_TEST_RAW]\n");
		ret |= mpTestFunc(HIMAX_INSPECTION_GAPTEST_RAW,
		  (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM) + ic_data->HX_TX_NUM
		  + ic_data->HX_RX_NUM);
		I("6. MP_GAP_TEST_RAW: End %d\n\n\n", ret);
	} else {
		I("Now %s : HOR flag=%d\n",
		  g_hx_inspt_crtra_name[IDX_GAP_HOR_RAWMAX],
		  g_inspt_crtra_flag[IDX_GAP_HOR_RAWMAX]);
		I("Now %s : HOR flag=%d\n",
		  g_hx_inspt_crtra_name[IDX_GAP_HOR_RAWMIN],
		  g_inspt_crtra_flag[IDX_GAP_HOR_RAWMIN]);
		I("Now %s : VERTICAL flag=%d\n",
		  g_hx_inspt_crtra_name[IDX_GAP_VER_RAWMAX],
		  g_inspt_crtra_flag[IDX_GAP_VER_RAWMAX]);
		I("Now %s : VERTICAL flag=%d\n",
		  g_hx_inspt_crtra_name[IDX_GAP_VER_RAWMIN],
		  g_inspt_crtra_flag[IDX_GAP_VER_RAWMIN]);
	}

	if (g_inspt_crtra_flag[IDX_ACT_IDLE_RAWDATA_MIN] == 1 &&
	  g_inspt_crtra_flag[IDX_ACT_IDLE_RAWDATA_MAX] == 1) {
		/*7. ACT_IDLE RAWDATA*/
		I("[MP_ACT_IDLE_TEST_RAW]\n");
		ret |= mpTestFunc(HIMAX_INSPECTION_ACT_IDLE_RAWDATA,
		  (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM) + ic_data->HX_TX_NUM
		  + ic_data->HX_RX_NUM);
		I("7. MP_ACT_IDLE_TEST_RAW: End %d\n\n\n", ret);
	} else {
		I("Now %s :flag=%d\n",
		  g_hx_inspt_crtra_name[IDX_ACT_IDLE_RAWDATA_MAX],
		  g_inspt_crtra_flag[IDX_ACT_IDLE_RAWDATA_MAX]);
		I("Now %s :flag=%d\n",
		  g_hx_inspt_crtra_name[IDX_ACT_IDLE_RAWDATA_MIN],
		  g_inspt_crtra_flag[IDX_ACT_IDLE_RAWDATA_MIN]);
	}

	if (g_inspt_crtra_flag[IDX_ACT_IDLE_NOISE_MIN] == 1 &&
	  g_inspt_crtra_flag[IDX_ACT_IDLE_NOISE_MAX] == 1) {
		/*8. ACT_IDLE NOISE*/
		I("[MP_ACT_IDLE_TEST_NOISE]\n");
		ret |= mpTestFunc(HIMAX_INSPECTION_ACT_IDLE_NOISE,
		  (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM) + ic_data->HX_TX_NUM
		  + ic_data->HX_RX_NUM);
		I("8. MP_ACT_IDLE_TEST_NOISE: End %d\n\n\n", ret);
	} else {
		I("Now %s :flag=%d\n",
		  g_hx_inspt_crtra_name[IDX_ACT_IDLE_NOISE_MAX],
		  g_inspt_crtra_flag[IDX_ACT_IDLE_NOISE_MAX]);
		I("Now %s :flag=%d\n",
		  g_hx_inspt_crtra_name[IDX_ACT_IDLE_NOISE_MIN],
		  g_inspt_crtra_flag[IDX_ACT_IDLE_NOISE_MIN]);
	}

	/* check press power key or not for LPWUG test item*/
	if ((g_inspt_crtra_flag[IDX_LPWUG_NOISE_MAX] == 1 &&
	  g_inspt_crtra_flag[IDX_LPWUG_NOISE_MIN] == 1)	||
	  (g_inspt_crtra_flag[IDX_LPWUG_RAWDATA_MAX] == 1 &&
	  g_inspt_crtra_flag[IDX_LPWUG_RAWDATA_MIN] == 1) ||
	  (g_inspt_crtra_flag[IDX_LPWUG_IDLE_NOISE_MAX] == 1 &&
	  g_inspt_crtra_flag[IDX_LPWUG_IDLE_NOISE_MIN] == 1) ||
	  (g_inspt_crtra_flag[IDX_LPWUG_IDLE_RAWDATA_MAX] == 1 &&
	  g_inspt_crtra_flag[IDX_LPWUG_IDLE_RAWDATA_MIN] == 1)
	)
		himax_press_powerkey();

/* Workaround delete */
#if 0
	/* Wait suspend done */
	while (private_ts->suspend_resume_done != 1)
		usleep_range(1000, 1001);

	private_ts->suspend_resume_done = 0;
#endif

	if (g_inspt_crtra_flag[IDX_LPWUG_RAWDATA_MIN] == 1 &&
	  g_inspt_crtra_flag[IDX_LPWUG_RAWDATA_MAX] == 1) {
		/*9. LPWUG RAWDATA*/
		I("[MP_LPWUG_TEST_RAW]\n");
		ret |= mpTestFunc(HIMAX_INSPECTION_LPWUG_RAWDATA,
		  (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM) + ic_data->HX_TX_NUM
		  + ic_data->HX_RX_NUM);
		I("9. MP_LPWUG_TEST_RAW: End %d\n\n\n", ret);
	} else {
		I("Now %s :flag=%d\n",
		  g_hx_inspt_crtra_name[IDX_LPWUG_RAWDATA_MIN],
		  g_inspt_crtra_flag[IDX_LPWUG_RAWDATA_MIN]);
		I("Now %s :flag=%d\n",
		  g_hx_inspt_crtra_name[IDX_LPWUG_RAWDATA_MAX],
		  g_inspt_crtra_flag[IDX_LPWUG_RAWDATA_MAX]);
	}

	if (g_inspt_crtra_flag[IDX_LPWUG_NOISE_MAX] == 1 &&
	  g_inspt_crtra_flag[IDX_LPWUG_NOISE_MIN] == 1) {
		/*10. LPWUG NOISE*/
		I("[MP_LPWUG_TEST_NOISE]\n");
		ret |= mpTestFunc(HIMAX_INSPECTION_LPWUG_NOISE,
		  (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM) + ic_data->HX_TX_NUM
		  + ic_data->HX_RX_NUM);
		I("10. MP_LPWUG_TEST_NOISE: End %d\n\n\n", ret);
	} else {
		I("Now %s :flag=%d\n",
		  g_hx_inspt_crtra_name[IDX_LPWUG_NOISE_MAX],
		  g_inspt_crtra_flag[IDX_LPWUG_NOISE_MAX]);
		I("Now %s :flag=%d\n",
		  g_hx_inspt_crtra_name[IDX_LPWUG_NOISE_MIN],
		  g_inspt_crtra_flag[IDX_LPWUG_NOISE_MIN]);
	}

	if (g_inspt_crtra_flag[IDX_LPWUG_IDLE_RAWDATA_MIN] == 1 &&
	  g_inspt_crtra_flag[IDX_LPWUG_IDLE_RAWDATA_MAX] == 1) {
		/*11. LPWUG IDLE RAWDATA*/
		I("[MP_LPWUG_IDLE_TEST_RAW]\n");
		ret |= mpTestFunc(HIMAX_INSPECTION_LPWUG_IDLE_RAWDATA,
		  (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM) + ic_data->HX_TX_NUM
		  + ic_data->HX_RX_NUM);
		I("11. MP_LPWUG_IDLE_TEST_RAW: End %d\n\n\n", ret);
	} else {
		I("Now %s :flag=%d\n",
		  g_hx_inspt_crtra_name[IDX_LPWUG_IDLE_RAWDATA_MIN],
		  g_inspt_crtra_flag[IDX_LPWUG_IDLE_RAWDATA_MIN]);
		I("Now %s :flag=%d\n",
		  g_hx_inspt_crtra_name[IDX_LPWUG_IDLE_RAWDATA_MAX],
		  g_inspt_crtra_flag[IDX_LPWUG_IDLE_RAWDATA_MAX]);
	}

	if (g_inspt_crtra_flag[IDX_LPWUG_IDLE_NOISE_MIN] == 1 &&
	  g_inspt_crtra_flag[IDX_LPWUG_IDLE_NOISE_MAX] == 1) {
		/*12. LPWUG IDLE RAWDATA*/
		I("[MP_LPWUG_IDLE_TEST_NOISE]\n");
		ret |= mpTestFunc(HIMAX_INSPECTION_LPWUG_IDLE_NOISE,
		  (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM) + ic_data->HX_TX_NUM
		  + ic_data->HX_RX_NUM);
		I("12. MP_LPWUG_IDLE_TEST_NOISE: End %d\n\n\n", ret);
	} else {
		I("Now %s :flag=%d\n",
		  g_hx_inspt_crtra_name[IDX_LPWUG_IDLE_NOISE_MIN],
		  g_inspt_crtra_flag[IDX_LPWUG_IDLE_NOISE_MIN]);
		I("Now %s :flag=%d\n",
		  g_hx_inspt_crtra_name[IDX_LPWUG_IDLE_NOISE_MAX],
		  g_inspt_crtra_flag[IDX_LPWUG_IDLE_NOISE_MAX]);
	}

	/* check press power key or not for LPWUG test item*/
	if ((g_inspt_crtra_flag[IDX_LPWUG_NOISE_MAX] == 1 &&
	  g_inspt_crtra_flag[IDX_LPWUG_NOISE_MIN] == 1)	||
	  (g_inspt_crtra_flag[IDX_LPWUG_RAWDATA_MAX] == 1 &&
	  g_inspt_crtra_flag[IDX_LPWUG_RAWDATA_MIN] == 1) ||
	  (g_inspt_crtra_flag[IDX_LPWUG_IDLE_NOISE_MAX] == 1 &&
	  g_inspt_crtra_flag[IDX_LPWUG_IDLE_NOISE_MIN] == 1) ||
	  (g_inspt_crtra_flag[IDX_LPWUG_IDLE_RAWDATA_MAX] == 1 &&
	  g_inspt_crtra_flag[IDX_LPWUG_IDLE_RAWDATA_MIN] == 1)
	)
		himax_press_powerkey();

	hx_test_data_pop_out(g_rslt_data, g_file_path);

/* Workaround delete */
#if 0
	/* Wait resume done */
	while (private_ts->suspend_resume_done != 1)
		usleep_range(1000, 1001);
#endif

#if defined(HIMAX_V2_MULTI_BIN)||defined(HX_CODE_OVERLAY)
	private_ts->in_self_test = 0;
	g_core_fp.fp_0f_op_file_dirly(normalfw);
	hx_turn_on_mp_func(0);
	/* set N frame back to default value 1*/
	g_core_fp.fp_register_write(tmp_addr, 4, tmp_data, 0);
	g_core_fp.fp_reload_disable(0);
	g_core_fp.fp_sense_on(0);
#else
	g_core_fp.fp_sense_off(true);
	hx_turn_on_mp_func(0);
	/*himax_set_N_frame(1, HIMAX_INSPECTION_WEIGHT_NOISE);*/
	/* set N frame back to default value 1*/
	g_core_fp.fp_register_write(tmp_addr, 4, tmp_data, 0);
#ifndef HX_ZERO_FLASH
	if (g_core_fp.fp_reload_disable != NULL)
		g_core_fp.fp_reload_disable(0);
#endif
	g_core_fp.fp_sense_on(0);
#endif

END_FUNC:
	himax_self_test_data_deinit();


	I("running status = %d\n", ret);

	/*if (ret != 0)*/
		/*ret = 1;*/

	I("%s:OUT\n", __func__);
	return ret;
}

void himax_inspect_data_clear(void)
{
	if (!g_rslt_data) {
		kfree(g_rslt_data);
		g_rslt_data = NULL;
	}
}

void himax_inspection_init(void)
{
	I("%s: enter, %d\n", __func__, __LINE__);

	g_core_fp.fp_chip_self_test = himax_chip_self_test;
}
