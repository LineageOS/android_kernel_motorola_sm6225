/*
 * Copyright (c) 2018-2020 The Linux Foundation. All rights reserved.
 *
 * Permission to use, copy, modify, and/or distribute this software for
 * any purpose with or without fee is hereby granted, provided that the
 * above copyright notice and this permission notice appear in all
 * copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
 * WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE
 * AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL
 * DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR
 * PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
 */

#include "qdf_file.h"
#include "qdf_module.h"
#include "qdf_parse.h"
#include "qdf_status.h"
#include "qdf_str.h"
#include "qdf_trace.h"
#include "qdf_types.h"

#define DEVICE_BOOTARG "androidboot.device="
#define RADIO_BOOTARG "androidboot.radio="

QDF_STATUS qdf_ini_parse(const char *ini_path, void *context,
			 qdf_ini_item_cb item_cb, qdf_ini_section_cb section_cb)
{
	QDF_STATUS status;
	char *fbuf;
	char *cursor;
	int ini_read_count = 0;
	char *device_ptr, *radio_ptr = NULL;
	const char *cmd_line = NULL;
	struct device_node *chosen_node = NULL;
	int len = 0;

	status = qdf_file_read(ini_path, &fbuf);
	if (QDF_IS_STATUS_ERROR(status)) {
		qdf_err("Failed to read *.ini file @ %s", ini_path);
		return status;
	}

	/* foreach line */
	cursor = fbuf;
	while (*cursor != '\0') {
		char *key = cursor;
		char *value = NULL;
		bool comment = false;
		bool eol = false;

		/*
		 * Look for the end of the line, while noting any
		 * value ('=') or comment ('#') indicators
		 */
		while (!eol) {
			switch (*cursor) {
			case '\r':
			case '\n':
				*cursor = '\0';
				cursor++;
				/* fall through */
			case '\0':
				eol = true;
				break;

			case '=':
				/*
				 * The first '=' is the value indicator.
				 * Subsequent '=' are valid value characters.
				 */
				if (!value && !comment) {
					value = cursor + 1;
					*cursor = '\0';
				}

				cursor++;
				break;

			case '#':
				/*
				 * We don't process comments, so we can null-
				 * terminate unconditionally here (unlike '=').
				 */
				comment = true;
				*cursor = '\0';
				/* fall through */
			default:
				cursor++;
				break;
			}
		}

		key = qdf_str_trim(key);

		qdf_debug("key:%.9s,value_priv:%.2s \n", key,value);
		if(strncmp(key, "BandCapability", 14) == 0 ){
			qdf_debug("wifi24g modify enter \n");
			chosen_node = of_find_node_by_name(NULL, "chosen");
			qdf_err("%s: get chosen node \n", __func__);

			if (!chosen_node){
				qdf_err("%s: get chosen node read failed \n", __func__);
				goto free_fbuf;
			} else {
				cmd_line = of_get_property(chosen_node, "bootargs", &len);
				if (!cmd_line || len <= 0) {
					qdf_err("%s: get the barcode bootargs failed \n", __func__);
					status = QDF_STATUS_E_FAILURE;
					goto free_fbuf;
				} else {
					device_ptr = strstr(cmd_line, DEVICE_BOOTARG);
					radio_ptr = strstr(cmd_line, RADIO_BOOTARG);
					if ((device_ptr == NULL) || (radio_ptr == NULL)) {
						qdf_err("%s: " DEVICE_BOOTARG" not present cmd line argc",__func__);
						status = QDF_STATUS_E_FAILURE;
						goto free_fbuf;
					} else {
						device_ptr += strlen(DEVICE_BOOTARG);
						radio_ptr += strlen(RADIO_BOOTARG);
						qdf_debug("device:%.9s, radio:%.9s \n", device_ptr, radio_ptr);
					}
				}
				if(((strncmp(device_ptr, "rav", 3) == 0) ||
							(strncmp(device_ptr, "sofiar", 6) == 0) ||
							(strncmp(device_ptr, "astro", 5) == 0))
							&& (strncmp(radio_ptr, "NA", 2) != 0)) {
					*value='1'; //wifi BandCapability = 2.4G only
					qdf_debug("value_new1:%.2s\n",value );
				}
				qdf_debug("device:%.5s, radio:%.5s, key:%.9s,value_new2:%.2s \n",
						device_ptr, radio_ptr, key,value);
			}
		}


		/*
		 * Ignoring comments, a valid ini line contains one of:
		 *	1) some 'key=value' config item
		 *	2) section header
		 *	3) a line containing whitespace
		 */
		if (value) {
			status = item_cb(context, key, value);
			if (QDF_IS_STATUS_ERROR(status))
				goto free_fbuf;
			else
				ini_read_count++;
		} else if (key[0] == '[') {
			qdf_size_t len = qdf_str_len(key);

			if (key[len - 1] != ']') {
				qdf_err("Invalid *.ini syntax '%s'", key);
			} else {
				key[len - 1] = '\0';
				status = section_cb(context, key + 1);
				if (QDF_IS_STATUS_ERROR(status))
					goto free_fbuf;
			}
		} else if (key[0] != '\0') {
			qdf_err("Invalid *.ini syntax '%s'", key);
		}

		/* skip remaining EoL characters */
		while (*cursor == '\n' || *cursor == '\r')
			cursor++;
	}

	qdf_debug("INI values read: %d", ini_read_count);
	if (ini_read_count != 0)
		status = QDF_STATUS_SUCCESS;
	else
		status = QDF_STATUS_E_FAILURE;

free_fbuf:
	qdf_file_buf_free(fbuf);

	return status;
}
qdf_export_symbol(qdf_ini_parse);

