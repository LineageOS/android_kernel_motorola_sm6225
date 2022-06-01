// SPDX-License-Identifier: GPL-2.0
/*
 * Synaptics TouchCom touchscreen driver
 *
 * Copyright (C) 2017-2020 Synaptics Incorporated. All rights reserved.
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
 * INFORMATION CONTAINED IN THIS DOCUMENT IS PROVIDED "AS-IS," AND SYNAPTICS
 * EXPRESSLY DISCLAIMS ALL EXPRESS AND IMPLIED WARRANTIES, INCLUDING ANY
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE,
 * AND ANY WARRANTIES OF NON-INFRINGEMENT OF ANY INTELLECTUAL PROPERTY RIGHTS.
 * IN NO EVENT SHALL SYNAPTICS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, PUNITIVE, OR CONSEQUENTIAL DAMAGES ARISING OUT OF OR IN CONNECTION
 * WITH THE USE OF THE INFORMATION CONTAINED IN THIS DOCUMENT, HOWEVER CAUSED
 * AND BASED ON ANY THEORY OF LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, AND EVEN IF SYNAPTICS WAS ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE. IF A TRIBUNAL OF COMPETENT JURISDICTION DOES
 * NOT PERMIT THE DISCLAIMER OF DIRECT DAMAGES OR ANY OTHER DAMAGES, SYNAPTICS'
 * TOTAL CUMULATIVE LIABILITY TO ANY PARTY SHALL NOT EXCEED ONE HUNDRED U.S.
 * DOLLARS.
 */

/**
 * @file syna_tcm2_testing.c
 *
 * This file implements the sample code to perform chip testing.
 */
#include "syna_tcm2_testing.h"
#include "syna_tcm2_testing_limits_csot.h"
#include "syna_tcm2_testing_limits_visionox.h"
#include "synaptics_touchcom_core_dev.h"
#include "synaptics_touchcom_func_base.h"
#include "syna_tcm2.h"


#define SYNA_LOG_SIZE_MAX (PAGE_SIZE * 6)
#define SYNA_TESTING_RESULT_IN_CSV (1)

/* g_testing_dir represents the root folder of testing sysfs
 */
static struct kobject *g_testing_dir;
static struct syna_tcm *g_tcm_ptr;
static unsigned char testing_config_id_data[MAX_SIZE_CONFIG_ID];

static void syna_save2file(char *buf, int size)
{
#if 1 // output to dmesg, only for debug
	char *str = NULL;
	char *src = NULL;
	char c = 'T';
	int offset = 0;

	if (!buf || (size == 0)) {
		LOGE("Invalid test result data or size\n");
		goto exit;
	}

	LOGI("size = %d\n", size);

	offset = 0;
	while (offset < size) {
		src = buf + offset;
		LOGW("%s", src);
		str = strchr(src, c);
		if (str == NULL)
			break;

		offset = &str[0] - &buf[0] + 1;
		//LOGI("offset = %d\n", offset);
	}
#endif

#if SYNA_TESTING_RESULT_IN_CSV
	if (!buf || (size == 0)) {
		LOGE("Invalid test result data or size\n");
		goto exit;
	}

	// save to file
	// open file
	// write to file
	// close file
#endif

exit:
	return;
}

/**
 * syna_testing_log_test_data()
 *
 * log the test data
 *
 * @param
 *    [ in] data: target test data
 *    [ in] data_size: size of test data
 *    [ in] test_item: test item
 *    [ in] rows: the number of rows
 *    [ in] cols: the number of column
 *
 * @return
 *    no return
 */
static void syna_testing_log_test_data(unsigned char *data,
		unsigned int data_size, unsigned char test_item,
		int rows, int cols)
{
	int i, j, cnt, offset, log_size;
	int *abs_raw_ptr = NULL;
	short *data_ptr = NULL;
	unsigned char *open_short_ptr = NULL;
	char *log_ptr = NULL;
	struct syna_tcm *tcm = g_tcm_ptr;

	if (!data || (data_size == 0)) {
		LOGE("Invalid test data or size\n");
		goto exit;
	}

	if (tcm->testing_log_data == NULL) {
		LOGE("Do not support log data\n");
		goto exit;
	}

	log_ptr = tcm->testing_log_data + tcm->testing_log_size;
	log_size = SYNA_LOG_SIZE_MAX - tcm->testing_log_size;

	if ((test_item >= TEST_PID01_TRX_TRX_SHORTS) && (test_item <= TEST_PID03_TRX_GROUND_SHORTS)) {
		goto open_short;
	}
	else if (test_item == TEST_PID18_HYBRID_ABS_RAW) {
		goto abs_raw;
	}

//trans_data:
	if (data_size != (2 * rows * cols)) {
		LOGE("Size mismatched, data:%d (exppected:%d)\n",
			data_size, (2 * rows * cols));
			goto exit;
	}
	data_ptr = (short *)&data[0];
	offset = 0;
	cnt = snprintf(log_ptr + offset, log_size - offset, "TEST PT$%02d:\n", test_item);
	offset += cnt;
	for (i = 0; i < rows; i++) {
		for (j = 0; j < cols; j++) {
			cnt = snprintf(log_ptr + offset, log_size - offset, "%4d ", *data_ptr);
			offset += cnt;
			data_ptr++;
		}
		cnt = snprintf(log_ptr + offset, log_size - offset, "\n");
		offset += cnt;
	}
	tcm->testing_log_size += offset;
	goto exit;

abs_raw:
	if (data_size != (4 * (rows + cols))) {
		LOGE("Size mismatched, data:%d (exppected:%d)\n",
			data_size, (4 * (rows + cols)));
			goto exit;
	}
	abs_raw_ptr = (int *)&data[0];

	offset = 0;
	cnt = snprintf(log_ptr + offset, log_size - offset, "TEST PT$%02d:\n", test_item);
	offset += cnt;
	cnt = snprintf(log_ptr + offset, log_size - offset, "abs raw-cols:");
	offset += cnt;
	for (i = 0; i < cols; i++) {
		cnt = snprintf(log_ptr + offset, log_size - offset, "%5d ", *abs_raw_ptr);
		offset += cnt;
		abs_raw_ptr++;
	}
	cnt = snprintf(log_ptr + offset, log_size - offset, "\n");
	offset += cnt;

	cnt = snprintf(log_ptr + offset, log_size - offset, "abs raw-rows:");
	offset += cnt;
	for (i = 0; i < rows; i++) {
		cnt = snprintf(log_ptr + offset, log_size - offset, "%5d ", *abs_raw_ptr);
		offset += cnt;
		abs_raw_ptr++;
	}
	cnt = snprintf(log_ptr + offset, log_size - offset, "\n");
	offset += cnt;
	tcm->testing_log_size += offset;
	goto exit;

open_short:
	open_short_ptr = (unsigned char *)&data[0];
	offset = 0;
	cnt = snprintf(log_ptr + offset, log_size - offset, "TEST PT$%02d:", test_item);
	offset += cnt;
	for (i = 0; i < data_size; i++) {
		cnt = snprintf(log_ptr + offset, log_size - offset, "0x%02x ", *open_short_ptr);
		offset += cnt;
		open_short_ptr++;
	}
	cnt = snprintf(log_ptr + offset, log_size - offset, "\n");
	offset += cnt;
	tcm->testing_log_size += offset;
	goto exit;

exit:
	if (tcm->testing_log_size >= SYNA_LOG_SIZE_MAX) {
		LOGE("There is no enough log buf (%d, %ld), please extend the buf size\n",
				tcm->testing_log_size, SYNA_LOG_SIZE_MAX);
	}

	return;
}


/**
 * syna_testing_compare_byte_vector()
 *
 * Sample code to compare the test result with limits
 * by byte vector
 *
 * @param
 *    [ in] data: target test data
 *    [ in] data_size: size of test data
 *    [ in] limit: test limit value to be compared with
 *    [ in] limit_size: size of test limit
 *
 * @return
 *    on success, true; otherwise, return false
 */
static bool syna_testing_compare_byte_vector(unsigned char *data,
		unsigned int data_size, const unsigned char *limit,
		unsigned int limit_size)
{
	bool result = false;
	unsigned char tmp;
	unsigned char p, l;
	int i, j;

	if (!data || (data_size == 0)) {
		LOGE("Invalid test data\n");
		return false;
	}
	if (!limit || (limit_size == 0)) {
		LOGE("Invalid limits\n");
		return false;
	}

	if (limit_size < data_size) {
		LOGE("Limit size mismatched, data size: %d, limits: %d\n",
			data_size, limit_size);
		return false;
	}

	result = true;
	for (i = 0; i < data_size; i++) {
		tmp = data[i];

		for (j = 0; j < 8; j++) {
			p = GET_BIT(tmp, j);
			l = GET_BIT(limit[i], j);
			if (p != l) {
				LOGE("Fail on TRX-%03d (data:%X, limit:%X)\n",
					(i*8 + j), p, l);
				result = false;
			}
		}
	}

	return result;
}

/**
 * syna_testing_compare_frame()
 *
 * Sample code to compare the test result with limits
 * by a lower-bound frame
 *
 * @param
 *    [ in] data: target test data
 *    [ in] data_size: size of test data
 *    [ in] rows: the number of rows
 *    [ in] cols: the number of column
 *    [ in] limits_hi: upper-bound test limit
 *    [ in] limits_lo: lower-bound test limit
 *
 * @return
 *    on success, true; otherwise, return false
 */
static bool syna_testing_compare_frame(unsigned char *data,
		unsigned int data_size, int rows, int cols,
		const short *limits_hi, const short *limits_lo)
{
	bool result = false;
	short *data_ptr = NULL;
	short limit;
	int i, j;

	if (!data || (data_size == 0)) {
		LOGE("Invalid test data\n");
		return false;
	}

	if (data_size < (2 * rows * cols)) {
		LOGE("Size mismatched, data:%d (exppected:%d)\n",
			data_size, (2 * rows * cols));
		result = false;
		return false;
	}

	if(limit_panel) {
		if (rows > LIMIT_BOUNDARY_V) {
			LOGE("Rows mismatched, rows:%d (exppected:%d)\n",
				rows, LIMIT_BOUNDARY_V);
			result = false;
			return false;
		}
		if (cols > LIMIT_BOUNDARY_V) {
			LOGE("Columns mismatched, cols: %d (exppected:%d)\n",
				cols, LIMIT_BOUNDARY_V);
			result = false;
			return false;
		}

	} else {
		if (rows > LIMIT_BOUNDARY_C) {
			LOGE("Rows mismatched, rows:%d (exppected:%d)\n",
				rows, LIMIT_BOUNDARY_C);
			result = false;
			return false;
		}

		if (cols > LIMIT_BOUNDARY_C) {
			LOGE("Columns mismatched, cols: %d (exppected:%d)\n",
				cols, LIMIT_BOUNDARY_C);
			result = false;
			return false;
		}
	}
	result = true;

	if (!limits_hi)
		goto end_of_upper_bound_limit;

	data_ptr = (short *)&data[0];

	if(limit_panel) {
		for (i = 0; i < rows; i++) {
			for (j = 0; j < cols; j++) {
				limit = limits_hi[i * LIMIT_BOUNDARY_V + j];
				if (*data_ptr > limit) {
					LOGE("Fail on (%2d,%2d)=%5d, limits_hi:%4d\n",
						i, j, *data_ptr, limit);
					result = false;
				}
				data_ptr++;
			}
		}
	} else {
		for (i = 0; i < rows; i++) {
			for (j = 0; j < cols; j++) {
				limit = limits_hi[i * LIMIT_BOUNDARY_C + j];
				if (*data_ptr > limit) {
					LOGE("Fail on (%2d,%2d)=%5d, limits_hi:%4d\n",
						i, j, *data_ptr, limit);
					result = false;
				}
				data_ptr++;
			}
		}
	}

end_of_upper_bound_limit:

	if (!limits_lo)
		goto end_of_lower_bound_limit;

	data_ptr = (short *)&data[0];

	if(limit_panel){
		for (i = 0; i < rows; i++) {
			for (j = 0; j < cols; j++) {
				limit = limits_lo[i * LIMIT_BOUNDARY_V + j];
				if (*data_ptr < limit) {
					LOGE("Fail on (%2d,%2d)=%5d, limits_lo:%4d\n",
						i, j, *data_ptr, limit);
					result = false;
				}
				data_ptr++;
			}
		}
	}else {
		for (i = 0; i < rows; i++) {
			for (j = 0; j < cols; j++) {
				limit = limits_lo[i * LIMIT_BOUNDARY_C + j];
				if (*data_ptr < limit) {
					LOGE("Fail on (%2d,%2d)=%5d, limits_lo:%4d\n",
						i, j, *data_ptr, limit);
					result = false;
				}
				data_ptr++;
			}
		}
	}

end_of_lower_bound_limit:
	return result;
}

/**
 * syna_testing_device_id()
 *
 * Sample code to ensure the device id is expected
 *
 * @param
 *    [ in] tcm: the driver handle
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_testing_device_id(struct syna_tcm *tcm)
{
	int retval;
	bool result;
	struct tcm_identification_info info;
	char *strptr = NULL;

	LOGI("Start testing\n");

	retval = syna_tcm_identify(tcm->tcm_dev, &info);
	if (retval < 0) {
		LOGE("Fail to get identification\n");
		result = false;
		goto exit;
	}

	if(limit_panel){
		strptr = strnstr(info.part_number,
						device_id_limit_v,
						strlen(info.part_number));
		if (strptr != NULL)
			result = true;
		else {
			LOGE("Device ID mismatched, FW: %s (limit: %s)\n",
				info.part_number, device_id_limit_v);
			result = false;
		}
	}else{
		strptr = strnstr(info.part_number,
						device_id_limit_c,
						strlen(info.part_number));
		if (strptr != NULL)
			result = true;
		else {
			LOGE("Device ID mismatched, FW: %s (limit: %s)\n",
				info.part_number, device_id_limit_c);
			result = false;
		}
	}


exit:
	LOGI("Result = %s\n", (result)?"pass":"fail");

	return ((result) ? 0 : -1);
}

/**
 * syna_testing_config_id()
 *
 * Sample code to ensure the config id is expected
 *
 * @param
 *    [ in] tcm: the driver handle
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_testing_config_id(struct syna_tcm *tcm)
{
	int retval;
	bool result;
	struct tcm_application_info info;
	int idx;

	LOGI("Start testing\n");

	syna_pal_mem_set(testing_config_id_data, 0, sizeof(testing_config_id_data));

	retval = syna_tcm_get_app_info(tcm->tcm_dev, &info);
	if (retval < 0) {
		LOGE("Fail to get app info\n");
		result = false;
		goto exit;
	}

	result = true;

	if(limit_panel){
		for (idx = 0; idx < sizeof(config_id_limit_v); idx++) {
			if (config_id_limit_v[idx] != info.customer_config_id[idx]) {
				LOGE("Fail on byte.%d (data: %02X, limit: %02X)\n",
					idx, info.customer_config_id[idx],
					config_id_limit_v[idx]);
				result = false;
			}
		}
	}else{
		for (idx = 0; idx < sizeof(config_id_limit_c); idx++) {
			if (config_id_limit_c[idx] != info.customer_config_id[idx]) {
				LOGE("Fail on byte.%d (data: %02X, limit: %02X)\n",
					idx, info.customer_config_id[idx],
					config_id_limit_c[idx]);
				result = false;
			}
		}
	}

	
	/* backup config id */
	for (idx = 0; idx < MAX_SIZE_CONFIG_ID; idx++) {
		testing_config_id_data[idx] = info.customer_config_id[idx];
	}

exit:
	LOGI("Result = %s\n", (result)?"pass":"fail");

	return ((result) ? 0 : -1);
}

/**
 * syna_testing_check_id_show()
 *
 * Attribute to show the result of ID comparsion to the console.
 *
 * @param
 *    [ in] kobj:  an instance of kobj
 *    [ in] attr:  an instance of kobj attribute structure
 *    [out] buf:  string buffer shown on console
 *
 * @return
 *    on success, number of characters being output;
 *    otherwise, negative value on error.
 */
static ssize_t syna_testing_check_id_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int retval;
	unsigned int count = 0;
	struct syna_tcm *tcm = g_tcm_ptr;

	if (!tcm->is_connected) {
		retval = snprintf(buf, PAGE_SIZE,
				"Device is NOT connected\n");
		goto exit;
	}

	count = 0;

	retval = syna_testing_device_id(tcm);

	retval = snprintf(buf, PAGE_SIZE - count,
			"Device ID check: %s\n",
			(retval < 0) ? "fail" : "pass");

	buf += retval;
	count += retval;

	retval = syna_testing_config_id(tcm);

	retval = snprintf(buf, PAGE_SIZE - count,
			"Config ID check: %s\n",
			(retval < 0) ? "fail" : "pass");

	buf += retval;
	count += retval;

	retval = count;
exit:
	return retval;
}

static struct kobj_attribute kobj_attr_check_id =
	__ATTR(check_id, 0444, syna_testing_check_id_show, NULL);

/**
 * syna_testing_pt01()
 *
 * Sample code to perform PT01 testing
 *
 * @param
 *    [ in] tcm: the driver handle
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_testing_pt01(struct syna_tcm *tcm)
{
	int retval;
	bool result = false;
	struct tcm_buffer test_data;

	syna_tcm_buf_init(&test_data);

	LOGI("Start testing\n");

	retval = syna_tcm_run_production_test(tcm->tcm_dev,
			TEST_PID01_TRX_TRX_SHORTS,
			&test_data);
	if (retval < 0) {
		LOGE("Fail to run test %d\n", TEST_PID01_TRX_TRX_SHORTS);
		result = false;
		goto exit;
	}

	if(limit_panel) {
		result = syna_testing_compare_byte_vector(test_data.buf,
			test_data.data_length,
			pt01_limits_v,
			ARRAY_SIZE(pt01_limits_v));
	}else {
		result = syna_testing_compare_byte_vector(test_data.buf,
			test_data.data_length,
			pt01_limits_c,
			ARRAY_SIZE(pt01_limits_c));
	}

	syna_testing_log_test_data(test_data.buf,
			test_data.data_length,
			TEST_PID01_TRX_TRX_SHORTS,
			tcm->tcm_dev->rows,
			tcm->tcm_dev->cols);

exit:
	LOGI("Result = %s\n", (result)?"pass":"fail");

	syna_tcm_buf_release(&test_data);

	return ((result) ? 0 : -1);
}

/**
 * syna_testing_pt01_show()
 *
 * Attribute to show the result of PT01 test to the console.
 *
 * @param
 *    [ in] kobj:  an instance of kobj
 *    [ in] attr:  an instance of kobj attribute structure
 *    [out] buf:  string buffer shown on console
 *
 * @return
 *    on success, number of characters being output;
 *    otherwise, negative value on error.
 */
static ssize_t syna_testing_pt01_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int retval;
	unsigned int count = 0;
	struct syna_tcm *tcm = g_tcm_ptr;

	if (!tcm->is_connected) {
		count = snprintf(buf, PAGE_SIZE,
				"Device is NOT connected\n");
		goto exit;
	}

	retval = syna_testing_pt01(tcm);

	count = snprintf(buf, PAGE_SIZE,
			"TEST PT$01: %s\n",
			(retval < 0) ? "fail" : "pass");

exit:
	return count;
}

static struct kobj_attribute kobj_attr_pt01 =
	__ATTR(pt01, 0444, syna_testing_pt01_show, NULL);

/**
 * syna_testing_pt02()
 *
 * Sample code to perform PT02 testing
 *
 * @param
 *    [ in] tcm: the driver handle
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_testing_pt02(struct syna_tcm *tcm)
{
	int retval;
	bool result = false;
	struct tcm_buffer test_data;

	syna_tcm_buf_init(&test_data);

	LOGI("Start testing\n");

	retval = syna_tcm_run_production_test(tcm->tcm_dev,
			TEST_PID02_TRX_SENSOR_OPENS,
			&test_data);
	if (retval < 0) {
		LOGE("Fail to run test %d\n", TEST_PID02_TRX_SENSOR_OPENS);
		result = false;
		goto exit;
	}

	if(limit_panel){
		result = syna_testing_compare_byte_vector(test_data.buf,
			test_data.data_length,
			pt02_limits_v,
			ARRAY_SIZE(pt02_limits_v));

	}else{
		result = syna_testing_compare_byte_vector(test_data.buf,
			test_data.data_length,
			pt02_limits_c,
			ARRAY_SIZE(pt02_limits_c));
	}

	syna_testing_log_test_data(test_data.buf,
			test_data.data_length,
			TEST_PID02_TRX_SENSOR_OPENS,
			tcm->tcm_dev->rows,
			tcm->tcm_dev->cols);

exit:
	LOGI("Result = %s\n", (result)?"pass":"fail");

	syna_tcm_buf_release(&test_data);

	return ((result) ? 0 : -1);
}

/**
 * syna_testing_pt02_show()
 *
 * Attribute to show the result of PT02 test to the console.
 *
 * @param
 *    [ in] kobj:  an instance of kobj
 *    [ in] attr:  an instance of kobj attribute structure
 *    [out] buf:  string buffer shown on console
 *
 * @return
 *    on success, number of characters being output;
 *    otherwise, negative value on error.
 */
static ssize_t syna_testing_pt02_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int retval;
	unsigned int count = 0;
	struct syna_tcm *tcm = g_tcm_ptr;

	if (!tcm->is_connected) {
		count = snprintf(buf, PAGE_SIZE,
				"Device is NOT connected\n");
		goto exit;
	}

	retval = syna_testing_pt02(tcm);

	count = snprintf(buf, PAGE_SIZE,
			"TEST PT$02: %s\n",
			(retval < 0) ? "fail" : "pass");

exit:
	return count;
}

static struct kobj_attribute kobj_attr_pt02 =
	__ATTR(pt02, 0444, syna_testing_pt02_show, NULL);

/**
 * syna_testing_pt03()
 *
 * Sample code to perform PT03 testing
 *
 * @param
 *    [ in] tcm: the driver handle
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_testing_pt03(struct syna_tcm *tcm)
{
	int retval;
	bool result = false;
	struct tcm_buffer test_data;

	syna_tcm_buf_init(&test_data);

	LOGI("Start testing\n");

	retval = syna_tcm_run_production_test(tcm->tcm_dev,
			TEST_PID03_TRX_GROUND_SHORTS,
			&test_data);
	if (retval < 0) {
		LOGE("Fail to run test %d\n", TEST_PID03_TRX_GROUND_SHORTS);
		result = false;
		goto exit;
	}

	if(limit_panel) {
		result = syna_testing_compare_byte_vector(test_data.buf,
			test_data.data_length,
			pt03_limits_v,
			ARRAY_SIZE(pt03_limits_v));
	} else {
		result = syna_testing_compare_byte_vector(test_data.buf,
			test_data.data_length,
			pt03_limits_c,
			ARRAY_SIZE(pt03_limits_c));
	}

	syna_testing_log_test_data(test_data.buf,
			test_data.data_length,
			TEST_PID03_TRX_GROUND_SHORTS,
			tcm->tcm_dev->rows,
			tcm->tcm_dev->cols);

exit:
	LOGI("Result = %s\n", (result)?"pass":"fail");

	syna_tcm_buf_release(&test_data);

	return ((result) ? 0 : -1);
}

/**
 * syna_testing_pt03_show()
 *
 * Attribute to show the result of PT03 test to the console.
 *
 * @param
 *    [ in] kobj:  an instance of kobj
 *    [ in] attr:  an instance of kobj attribute structure
 *    [out] buf:  string buffer shown on console
 *
 * @return
 *    on success, number of characters being output;
 *    otherwise, negative value on error.
 */
static ssize_t syna_testing_pt03_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int retval;
	unsigned int count = 0;
	struct syna_tcm *tcm = g_tcm_ptr;

	if (!tcm->is_connected) {
		count = snprintf(buf, PAGE_SIZE,
				"Device is NOT connected\n");
		goto exit;
	}

	retval = syna_testing_pt03(tcm);

	count = snprintf(buf, PAGE_SIZE,
			"TEST PT$03: %s\n",
			(retval < 0) ? "fail" : "pass");

exit:
	return count;
}

static struct kobj_attribute kobj_attr_pt03 =
	__ATTR(pt03, 0444, syna_testing_pt03_show, NULL);

/**
 * syna_testing_pt05()
 *
 * Sample code to perform PT05 testing
 *
 * @param
 *    [ in] tcm: the driver handle
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_testing_pt05(struct syna_tcm *tcm)
{
	int retval;
	bool result = false;
	struct tcm_buffer test_data;

	syna_tcm_buf_init(&test_data);

	LOGI("Start testing\n");

	retval = syna_tcm_run_production_test(tcm->tcm_dev,
			TEST_PID05_FULL_RAW_CAP,
			&test_data);
	if (retval < 0) {
		LOGE("Fail to run test %d\n", TEST_PID05_FULL_RAW_CAP);
		result = false;
		goto exit;
	}

	if (limit_panel) {
			result = syna_testing_compare_frame(test_data.buf,
			test_data.data_length,
			tcm->tcm_dev->rows,
			tcm->tcm_dev->cols,
			(const short *)&pt05_hi_limits_v[0],
			(const short *)&pt05_lo_limits_v[0]);
	} else {
		result = syna_testing_compare_frame(test_data.buf,
			test_data.data_length,
			tcm->tcm_dev->rows,
			tcm->tcm_dev->cols,
			(const short *)&pt05_hi_limits_c[0],
			(const short *)&pt05_lo_limits_c[0]);
	}

	syna_testing_log_test_data(test_data.buf,
				test_data.data_length,
				TEST_PID05_FULL_RAW_CAP,
				tcm->tcm_dev->rows,
				tcm->tcm_dev->cols);

exit:
	LOGI("Result = %s\n", (result)?"pass":"fail");

	syna_tcm_buf_release(&test_data);

	return ((result) ? 0 : -1);
}

/**
 * syna_testing_pt05_show()
 *
 * Attribute to show the result of PT05 test to the console.
 *
 * @param
 *    [ in] kobj:  an instance of kobj
 *    [ in] attr:  an instance of kobj attribute structure
 *    [out] buf:  string buffer shown on console
 *
 * @return
 *    on success, number of characters being output;
 *    otherwise, negative value on error.
 */
static ssize_t syna_testing_pt05_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int retval;
	unsigned int count = 0;
	struct syna_tcm *tcm = g_tcm_ptr;

	if (!tcm->is_connected) {
		count = snprintf(buf, PAGE_SIZE,
				"Device is NOT connected\n");
		goto exit;
	}

	retval = syna_testing_pt05(tcm);

	count = snprintf(buf, PAGE_SIZE,
			"TEST PT$05: %s\n", (retval < 0) ? "fail" : "pass");

exit:
	return count;
}

static struct kobj_attribute kobj_attr_pt05 =
	__ATTR(pt05, 0444, syna_testing_pt05_show, NULL);

/**
 * syna_testing_pt0a()
 *
 * Sample code to perform PT0A testing
 *
 * @param
 *    [ in] tcm: the driver handle
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_testing_pt0a(struct syna_tcm *tcm)
{
	int retval;
	bool result = false;
	struct tcm_buffer test_data;

	syna_tcm_buf_init(&test_data);

	LOGI("Start testing\n");

	retval = syna_tcm_run_production_test(tcm->tcm_dev,
			TEST_PID10_DELTA_NOISE,
			&test_data);
	if (retval < 0) {
		LOGE("Fail to run test %d\n", TEST_PID10_DELTA_NOISE);
		result = false;
		goto exit;
	}

	if (limit_panel) {
		result = syna_testing_compare_frame(test_data.buf,
			test_data.data_length,
			tcm->tcm_dev->rows,
			tcm->tcm_dev->cols,
			(const short *)&pt0a_hi_limits_v[0],
			(const short *)&pt0a_lo_limits_v[0]);

	} else {
		result = syna_testing_compare_frame(test_data.buf,
			test_data.data_length,
			tcm->tcm_dev->rows,
			tcm->tcm_dev->cols,
			(const short *)&pt0a_hi_limits_c[0],
			(const short *)&pt0a_lo_limits_c[0]);
	}
	
syna_testing_log_test_data(test_data.buf,
					test_data.data_length,
					TEST_PID10_DELTA_NOISE,
					tcm->tcm_dev->rows,
					tcm->tcm_dev->cols);

exit:
	LOGI("Result = %s\n", (result)?"pass":"fail");

	syna_tcm_buf_release(&test_data);

	return ((result) ? 0 : -1);
}

/**
 * syna_testing_pt0a_show()
 *
 * Attribute to show the result of PT0A test to the console.
 *
 * @param
 *    [ in] kobj:  an instance of kobj
 *    [ in] attr:  an instance of kobj attribute structure
 *    [out] buf:  string buffer shown on console
 *
 * @return
 *    on success, number of characters being output;
 *    otherwise, negative value on error.
 */
static ssize_t syna_testing_pt0a_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int retval;
	unsigned int count = 0;
	struct syna_tcm *tcm = g_tcm_ptr;

	if (!tcm->is_connected) {
		count = snprintf(buf, PAGE_SIZE,
				"Device is NOT connected\n");
		goto exit;
	}

	retval = syna_testing_pt0a(tcm);

	count = snprintf(buf, PAGE_SIZE,
			"TEST PT$0A: %s\n", (retval < 0) ? "fail" : "pass");

exit:
	return count;
}

static struct kobj_attribute kobj_attr_pt0a =
	__ATTR(pt0a, 0444, syna_testing_pt0a_show, NULL);

/**
 * syna_testing_pt10()
 *
 * Sample code to perform PT10 testing
 *
 * @param
 *    [ in] tcm: the driver handle
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_testing_pt10(struct syna_tcm *tcm)
{
	int retval;
	bool result = false;
	struct tcm_buffer test_data;

	syna_tcm_buf_init(&test_data);

	LOGI("Start testing\n");

	retval = syna_tcm_run_production_test(tcm->tcm_dev,
			TEST_PID16_SENSOR_SPEED,
			&test_data);
	if (retval < 0) {
		LOGE("Fail to run test %d\n", TEST_PID16_SENSOR_SPEED);
		result = false;
		goto exit;
	}

	if(limit_panel){
		result = syna_testing_compare_frame(test_data.buf,
			test_data.data_length,
			tcm->tcm_dev->rows,
			tcm->tcm_dev->cols,
			(const short *)&pt10_hi_limits_v[0],
			(const short *)&pt10_lo_limits_v[0]);
	}else {
		result = syna_testing_compare_frame(test_data.buf,
			test_data.data_length,
			tcm->tcm_dev->rows,
			tcm->tcm_dev->cols,
			(const short *)&pt10_hi_limits_c[0],
			(const short *)&pt10_lo_limits_c[0]);
	}

exit:
	LOGI("Result = %s\n", (result)?"pass":"fail");

	syna_tcm_buf_release(&test_data);

	return ((result) ? 0 : -1);
}

/**
 * syna_testing_pt10_show()
 *
 * Attribute to show the result of PT10 test to the console.
 *
 * @param
 *    [ in] kobj:  an instance of kobj
 *    [ in] attr:  an instance of kobj attribute structure
 *    [out] buf:  string buffer shown on console
 *
 * @return
 *    on success, number of characters being output;
 *    otherwise, negative value on error.
 */
static ssize_t syna_testing_pt10_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int retval;
	unsigned int count = 0;
	struct syna_tcm *tcm = g_tcm_ptr;

	if (!tcm->is_connected) {
		count = snprintf(buf, PAGE_SIZE,
				"Device is NOT connected\n");
		goto exit;
	}

	retval = syna_testing_pt10(tcm);

	count = snprintf(buf, PAGE_SIZE,
			"TEST PT$10: %s\n", (retval < 0) ? "fail" : "pass");

exit:
	return count;
}

static struct kobj_attribute kobj_attr_pt10 =
	__ATTR(pt10, 0444, syna_testing_pt10_show, NULL);

/**
 * syna_testing_pt11()
 *
 * Sample code to perform PT11 testing
 *
 * @param
 *    [ in] tcm: the driver handle
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_testing_pt11(struct syna_tcm *tcm)
{
	int retval;
	bool result = false;
	struct tcm_buffer test_data;

	syna_tcm_buf_init(&test_data);

	LOGI("Start testing\n");

	retval = syna_tcm_run_production_test(tcm->tcm_dev,
			TEST_PID17_ADC_RANGE,
			&test_data);
	if (retval < 0) {
		LOGE("Fail to run test %d\n", TEST_PID17_ADC_RANGE);
		result = false;
		goto exit;
	}

	if(limit_panel){
		result = syna_testing_compare_frame(test_data.buf,
			test_data.data_length,
			tcm->tcm_dev->rows,
			tcm->tcm_dev->cols,
			(const short *)&pt11_hi_limits_v[0],
			(const short *)&pt11_lo_limits_v[0]);
	}else{
		result = syna_testing_compare_frame(test_data.buf,
			test_data.data_length,
			tcm->tcm_dev->rows,
			tcm->tcm_dev->cols,
			(const short *)&pt11_hi_limits_c[0],
			(const short *)&pt11_lo_limits_c[0]);
	}
	
exit:
	LOGI("Result = %s\n", (result)?"pass":"fail");

	syna_tcm_buf_release(&test_data);

	return ((result) ? 0 : -1);
}

/**
 * syna_testing_pt11_show()
 *
 * Attribute to show the result of PT11 test to the console.
 *
 * @param
 *    [ in] kobj:  an instance of kobj
 *    [ in] attr:  an instance of kobj attribute structure
 *    [out] buf:  string buffer shown on console
 *
 * @return
 *    on success, number of characters being output;
 *    otherwise, negative value on error.
 */
static ssize_t syna_testing_pt11_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int retval;
	unsigned int count = 0;
	struct syna_tcm *tcm = g_tcm_ptr;

	if (!tcm->is_connected) {
		count = snprintf(buf, PAGE_SIZE,
				"Device is NOT connected\n");
		goto exit;
	}

	retval = syna_testing_pt11(tcm);

	count = snprintf(buf, PAGE_SIZE,
			"TEST PT$11: %s\n", (retval < 0) ? "fail" : "pass");

exit:
	return count;
}

static struct kobj_attribute kobj_attr_pt11 =
	__ATTR(pt11, 0444, syna_testing_pt11_show, NULL);

/**
 * syna_testing_pt12()
 *
 * Sample code to perform PT12 testing
 *
 * @param
 *    [ in] tcm: the driver handle
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_testing_pt12(struct syna_tcm *tcm)
{
	int retval;
	bool result = false;
	struct tcm_buffer test_data;
	int *data_ptr = NULL;
	int rows = tcm->tcm_dev->rows;
	int cols = tcm->tcm_dev->cols;
	int i;

	syna_tcm_buf_init(&test_data);

	LOGI("Start testing\n");
	
	retval = syna_tcm_run_production_test(tcm->tcm_dev,
			TEST_PID18_HYBRID_ABS_RAW,
			&test_data);
	if (retval < 0) {
		LOGE("Fail to run test %d\n", TEST_PID18_HYBRID_ABS_RAW);
		result = false;
		goto exit;
	}

	if(limit_panel){
		if ((LIMIT_BOUNDARY_V < rows) || (LIMIT_BOUNDARY_V < cols)) {
			LOGE("Size mismatched, rows:%d cols:%d\n",
				rows, cols);
			result = false;
			goto exit;
		}
	}else{
		if ((LIMIT_BOUNDARY_C < rows) || (LIMIT_BOUNDARY_C < cols)) {
			LOGE("Size mismatched, rows:%d cols:%d\n",
				rows, cols);
			result = false;
			goto exit;
		}
	}

	if (test_data.data_length < 4 * (rows + cols)) {
		LOGE("Data mismatched, size:%d (expected:%d)\n",
			test_data.data_length, 4 * (rows + cols));
		result = false;
		goto exit;
	}

	result = true;
	data_ptr = (int *)&test_data.buf[0];
	
	if(limit_panel){
		for (i = 0; i < cols; i++) {
			if (*data_ptr > pt12_limits_v[i]) {
				LOGE("Fail on Col-%2d=%5d, limits:%4d\n",
					i, *data_ptr, pt12_limits_v[i]);

				result = false;
			}
			data_ptr++;
		}
	}else {
		for (i = 0; i < cols; i++) {
			if (*data_ptr > pt12_limits_c[i]) {
				LOGE("Fail on Col-%2d=%5d, limits:%4d\n",
					i, *data_ptr, pt12_limits_c[i]);

				result = false;
			}
			data_ptr++;
		}
	}
	
	if(limit_panel){
		for (i = 0; i < rows; i++) {
			if (*data_ptr > pt12_limits_v[LIMIT_BOUNDARY_V + i]) {
				LOGE("Fail on Row-%2d=%5d, limits:%4d\n",
					i, *data_ptr,
					pt12_limits_v[LIMIT_BOUNDARY_V + i]);

				result = false;
			}
			data_ptr++;
		}
	}else{
		for (i = 0; i < rows; i++) {
			if (*data_ptr > pt12_limits_c[LIMIT_BOUNDARY_C + i]) {
				LOGE("Fail on Row-%2d=%5d, limits:%4d\n",
					i, *data_ptr,
					pt12_limits_c[LIMIT_BOUNDARY_C + i]);

				result = false;
			}
			data_ptr++;
		}
	}

	syna_testing_log_test_data(test_data.buf,
				test_data.data_length,
				TEST_PID18_HYBRID_ABS_RAW,
				tcm->tcm_dev->rows,
				tcm->tcm_dev->cols);

exit:
	LOGI("Result = %s\n", (result)?"pass":"fail");

	syna_tcm_buf_release(&test_data);

	return ((result) ? 0 : -1);
}

/**
 * syna_testing_pt12_show()
 *
 * Attribute to show the result of PT12 test to the console.
 *
 * @param
 *    [ in] kobj:  an instance of kobj
 *    [ in] attr:  an instance of kobj attribute structure
 *    [out] buf:  string buffer shown on console
 *
 * @return
 *    on success, number of characters being output;
 *    otherwise, negative value on error.
 */
static ssize_t syna_testing_pt12_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int retval;
	unsigned int count = 0;
	struct syna_tcm *tcm = g_tcm_ptr;

	if (!tcm->is_connected) {
		count = snprintf(buf, PAGE_SIZE,
				"Device is NOT connected\n");
		goto exit;
	}

	retval = syna_testing_pt12(tcm);

	count = snprintf(buf, PAGE_SIZE,
			"TEST PT$12: %s\n", (retval < 0) ? "fail" : "pass");

exit:
	return count;
}

static struct kobj_attribute kobj_attr_pt12 =
	__ATTR(pt12, 0444, syna_testing_pt12_show, NULL);

/**
 * syna_testing_get_rawdata_show()
 *
 * Attribute to show the result of get_rawdata test to the console.
 *
 * @param
 *    [ in] kobj:  an instance of kobj
 *    [ in] attr:  an instance of kobj attribute structure
 *    [out] buf:  string buffer shown on console
 *
 * @return
 *    on success, number of characters being output;
 *    otherwise, negative value on error.
 */
static ssize_t syna_testing_get_rawdata_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int retval, offset, idx;
	unsigned int count = 0;
	unsigned int tmp = 0;
	struct syna_tcm *tcm = g_tcm_ptr;

	mutex_lock(&tcm->testing_mutex);

	if (!tcm->is_connected) {
		count = snprintf(buf, PAGE_SIZE,
				"Device is NOT connected\n");
		goto exit;
	}

#if SYNA_TESTING_RESULT_IN_CSV
	tcm->testing_log_data = vmalloc(SYNA_LOG_SIZE_MAX);//kzalloc(SAVE_BUF_SIZE, GFP_KERNEL);
	if (!tcm->testing_log_data) {
		LOGE("Failed to allocate memory for testing_log_data\n");
		retval = -ENOMEM;
		goto exit;
	}
	tcm->testing_log_size = 0;
#endif

	// config ID test
	retval = syna_testing_config_id(tcm);
	if (tcm->testing_log_data) {
		/* save config id to log */
		offset = 0;
		tmp = snprintf(tcm->testing_log_data + offset, SYNA_LOG_SIZE_MAX - offset,
						"TEST Config ID:\n");
		offset += tmp;
		for (idx = 0; idx < MAX_SIZE_CONFIG_ID; idx++) {
			tmp = snprintf(tcm->testing_log_data + offset, SYNA_LOG_SIZE_MAX - offset,
							"%02x ", testing_config_id_data[idx]);
			offset += tmp;
		}
		tmp = snprintf(tcm->testing_log_data + offset, SYNA_LOG_SIZE_MAX - offset, "\n");
		offset += tmp;

		tcm->testing_log_size = offset;
	}

	// output result to buf
	offset = 0;
	tmp = snprintf(buf + offset, PAGE_SIZE - offset,
			"Config ID Test: %s\n", (retval < 0) ? "fail" : "pass");
	offset += tmp;

	// TRX short Test (TEST_PID01_TRX_TRX_SHORTS)
	retval = syna_testing_pt01(tcm);
	tmp = snprintf(buf + offset, PAGE_SIZE - offset,
			"TEST PT$01: %s\n", (retval < 0) ? "fail" : "pass");
	offset += tmp;

	// Trx gnd test	 TEST_PID03_TRX_GROUND_SHORTS
	retval = syna_testing_pt03(tcm);
	tmp = snprintf(buf + offset, PAGE_SIZE - offset,
			"TEST PT$03: %s\n", (retval < 0) ? "fail" : "pass");
	offset += tmp;

	// Full raw cap	 TEST_PID05_FULL_RAW_CAP
	retval = syna_testing_pt05(tcm);
	tmp = snprintf(buf + offset, PAGE_SIZE - offset,
			"TEST PT$05: %s\n", (retval < 0) ? "fail" : "pass");
	offset += tmp;

	// noise test	 TEST_PID10_DELTA_NOISE
	retval = syna_testing_pt0a(tcm);
	tmp = snprintf(buf + offset, PAGE_SIZE - offset,
			"TEST PT$0a: %s\n", (retval < 0) ? "fail" : "pass");
	offset += tmp;
	
#if 0
	// abs raw	 TEST_PID18_HYBRID_ABS_RAW
	retval = syna_testing_pt12(tcm);
	tmp = snprintf(buf + offset, PAGE_SIZE - offset,
			"TEST PT$12: %s\n", (retval < 0) ? "fail" : "pass");
	offset += tmp;
#endif
	count = offset;

#if SYNA_TESTING_RESULT_IN_CSV
	tmp = snprintf(tcm->testing_log_data + tcm->testing_log_size,
					SYNA_LOG_SIZE_MAX - tcm->testing_log_size, "%s", buf);
	tcm->testing_log_size += tmp;

	if (tcm->testing_log_size >= SYNA_LOG_SIZE_MAX) {
		LOGE("There is no enough log buf (%u, %lu), please extend the buf size\n",
				tcm->testing_log_size, SYNA_LOG_SIZE_MAX);
	}
	syna_save2file(tcm->testing_log_data, tcm->testing_log_size);
#endif

exit:
#if SYNA_TESTING_RESULT_IN_CSV
	if(tcm->testing_log_data) {
		vfree(tcm->testing_log_data);
		tcm->testing_log_data = NULL;
	}
#endif

	mutex_unlock(&tcm->testing_mutex);

	return count;
}

static struct kobj_attribute kobj_attr_get_rawdata =
	__ATTR(get_rawdata, 0444, syna_testing_get_rawdata_show, NULL);

/*
 * declaration of sysfs attributes
 */
static struct attribute *attrs[] = {
	&kobj_attr_check_id.attr,
	&kobj_attr_pt01.attr,
	&kobj_attr_pt02.attr,
	&kobj_attr_pt03.attr,
	&kobj_attr_pt05.attr,
	&kobj_attr_pt0a.attr,
	&kobj_attr_pt10.attr,
	&kobj_attr_pt11.attr,
	&kobj_attr_pt12.attr,
	&kobj_attr_get_rawdata.attr,
	NULL,
};

static struct attribute_group attr_testing_group = {
	.attrs = attrs,
};

/**
 * syna_testing_create_dir()
 *
 * Create a directory and register it with sysfs.
 * Then, create all defined sysfs files.
 *
 * @param
 *    [ in] tcm:  the driver handle
 *    [ in] sysfs_dir: root directory of sysfs nodes
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
int syna_testing_create_dir(struct syna_tcm *tcm,
		struct kobject *sysfs_dir)
{
	int retval = 0;

	g_testing_dir = kobject_create_and_add("testing",
			sysfs_dir);
	if (!g_testing_dir) {
		LOGE("Fail to create testing directory\n");
		return -EINVAL;
	}

	retval = sysfs_create_group(g_testing_dir, &attr_testing_group);
	if (retval < 0) {
		LOGE("Fail to create sysfs group\n");

		kobject_put(g_testing_dir);
		return retval;
	}

syna_pal_mutex_alloc(&tcm->testing_mutex);

	g_tcm_ptr = tcm;

	return 0;
}
/**
 *syna_testing_remove_dir()
 *
 * Remove the allocate sysfs directory
 *
 * @param
 *    none
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
void syna_testing_remove_dir(void)
{
	struct syna_tcm *tcm = g_tcm_ptr;
	if (g_testing_dir) {
			syna_pal_mutex_free(&tcm->testing_mutex);
		if(tcm->testing_log_data) {
			vfree(tcm->testing_log_data);
			tcm->testing_log_data = NULL;
		}

		sysfs_remove_group(g_testing_dir, &attr_testing_group);

		kobject_put(g_testing_dir);
	}
}
