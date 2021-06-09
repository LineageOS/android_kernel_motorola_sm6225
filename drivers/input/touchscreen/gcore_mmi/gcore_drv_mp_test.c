/*
 * GalaxyCore touchscreen driver
 *
 * Copyright (C) 2021 GalaxyCore Incorporated
 *
 * Copyright (C) 2021 Neo Chen <neo_chen@gcoreinc.com>
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
 */
#include "gcore_drv_common.h"
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#include <linux/gpio.h>
#include <uapi/linux/sched/types.h>


#define MP_TEST_PROC_FILENAME "ts03_selftest"


#define MP_TEST_OF_PREFIX                  "ts03,"
#define MP_TEST_OF_ITEM_TEST_INT_PIN       MP_TEST_OF_PREFIX"test-int-pin"
#define MP_TEST_OF_ITEM_TEST_CHIP_ID       MP_TEST_OF_PREFIX"test-chip-id"
#define MP_TEST_OF_TEST_CHIP_ID            MP_TEST_OF_PREFIX"chip-id"
#define MP_TEST_OF_ITEM_TEST_OPEN          MP_TEST_OF_PREFIX"test-open"
#define MP_TEST_OF_TEST_OPEN_CB            MP_TEST_OF_PREFIX"open-cb"
#define MP_TEST_OF_TEST_OPEN_MIN           MP_TEST_OF_PREFIX"open-min"
#define MP_TEST_OF_ITEM_TEST_SHORT         MP_TEST_OF_PREFIX"test-short"
#define MP_TEST_OF_TEST_SHORT_CB           MP_TEST_OF_PREFIX"short-cb"
#define MP_TEST_OF_TEST_SHORT_MIN          MP_TEST_OF_PREFIX"short-min"


#define MP_TEST_DATA_DIR              "data/vendor/touchpad"
#define MP_OPEN_TEST_DATA_FILEPATH    MP_TEST_DATA_DIR"/OpenTest.csv"
#define MP_SHORT_TEST_DATA_FILEPATH   MP_TEST_DATA_DIR"/ShortTest.csv"



DECLARE_COMPLETION(mp_test_complete);

struct gcore_mp_data {
	struct proc_dir_entry *mp_proc_entry;

	bool test_int_pin;
	int  int_pin_test_result;

	bool test_chip_id;
	int chip_id_test_result;
	u8 chip_id[2];

	bool test_open;
	int open_test_result;
	int open_cb;
	int open_min;
	u8 *open_data;
	u16 *open_node_val;

	bool test_short;
	int short_test_result;
	int short_cb;
	int short_min;
	u8 *short_data;
	u16 *short_node_val;

	struct gcore_dev *gdev;

};

struct gcore_mp_data *g_mp_data;
struct task_struct *mp_thread;

static int gcore_mp_test_fn_init(struct gcore_dev *gdev);
static void gcore_mp_test_fn_remove(struct gcore_dev *gdev);

static void dump_mp_data_to_seq_file(struct seq_file *m, const u16 *data,
	int rows, int cols);


struct gcore_exp_fn mp_test_fn = {
	.fn_type = GCORE_MP_TEST,
	.wait_int = false,
	.event_flag = false,
	.init = gcore_mp_test_fn_init,
	.remove = gcore_mp_test_fn_remove,
};

static void *gcore_seq_start(struct seq_file *m, loff_t *pos)
{
	return *pos < 1 ? (void *)1 : NULL;
}

static void *gcore_seq_next(struct seq_file *m, void *v, loff_t *pos)
{
	++*pos;
	return NULL;
}

static void gcore_seq_stop(struct seq_file *m, void *v)
{
	return;
}

static int gcore_seq_show(struct seq_file *m, void *v)
{
	struct gcore_mp_data *mp_data = (struct gcore_mp_data *)m->private;
	int rows = RAWDATA_ROW;
	int cols = RAWDATA_COLUMN;

	GTP_DEBUG("gcore seq show");

	if (mp_data == NULL) {
		GTP_ERROR("seq_file private data = NULL");
		return -1;
	}

	if (mp_data->test_int_pin) {
		seq_printf(m, "Int-Pin Test %s!\n\n",
			mp_data->int_pin_test_result == 0 ? "PASS" : "FAIL");
	}

	if (mp_data->test_chip_id) {
		seq_printf(m, "Chip-id Test %s!\n\n",
			mp_data->chip_id_test_result == 0 ? "PASS" : "FAIL");
	}

	if (mp_data->test_open) {
		if (mp_data->open_test_result == 0) {
			seq_printf(m, "Open Test PASS!\n\n");
		} else {
			seq_printf(m, "Open Test FAIL!\n");
			dump_mp_data_to_seq_file(m, mp_data->open_node_val, rows, cols);
			seq_putc(m, '\n');
		}

	}

	if (mp_data->test_short) {
		if (mp_data->short_test_result == 0) {
			seq_printf(m, "Short Test PASS!\n\n");
		} else {
			seq_printf(m, "Short Test FAIL!\n");
			dump_mp_data_to_seq_file(m, mp_data->short_node_val, rows, cols);
			seq_putc(m, '\n');
		}

	}

	return 0;
}


const struct seq_operations gcore_mptest_seq_ops = {
	.start = gcore_seq_start,
	.next  = gcore_seq_next,
	.stop  = gcore_seq_stop,
	.show  = gcore_seq_show,
};

int gcore_parse_mp_test_dt(struct gcore_mp_data *mp_data)
{
	struct device_node *np = mp_data->gdev->bus_device->dev.of_node;

	GTP_DEBUG("gcore parse mp test dt");

	mp_data->test_int_pin = of_property_read_bool(np, MP_TEST_OF_ITEM_TEST_INT_PIN);

	mp_data->test_chip_id = of_property_read_bool(np, MP_TEST_OF_ITEM_TEST_CHIP_ID);
	if (mp_data->test_chip_id) {
		of_property_read_u8_array(np, MP_TEST_OF_TEST_CHIP_ID, mp_data->chip_id,
			sizeof(mp_data->chip_id));
	}

	mp_data->test_open = of_property_read_bool(np, MP_TEST_OF_ITEM_TEST_OPEN);
	if (mp_data->test_open) {
		of_property_read_u32(np, MP_TEST_OF_TEST_OPEN_CB, &mp_data->open_cb);
		of_property_read_u32(np, MP_TEST_OF_TEST_OPEN_MIN, &mp_data->open_min);
	}

	mp_data->test_short = of_property_read_bool(np, MP_TEST_OF_ITEM_TEST_SHORT);
	if (mp_data->test_short) {
		of_property_read_u32(np, MP_TEST_OF_TEST_SHORT_CB, &mp_data->short_cb);
		of_property_read_u32(np, MP_TEST_OF_TEST_SHORT_MIN, &mp_data->short_min);
	}


	return 0;
}

int gcore_mp_test_int_pin(struct gcore_mp_data *mp_data)
{
	u8 read_buf[4] = { 0 };

	GTP_DEBUG("gcore mp test int pin");

	mutex_lock(&mp_data->gdev->transfer_lock);
	mp_data->gdev->irq_disable(mp_data->gdev);

	gcore_enter_idm_mode();

	msleep(1);

	gcore_idm_read_reg(0xC0000098, read_buf, sizeof(read_buf));
	GTP_DEBUG("reg addr: 0xC0000098 read_buf: %x %x %x %x",
		read_buf[0], read_buf[1], read_buf[2], read_buf[3]);

	msleep(1);

	read_buf[0] &= 0x7F;
	read_buf[2] &= 0x7F;


	gcore_idm_write_reg(0xC0000098, read_buf, sizeof(read_buf));

	msleep(1);

	if (gpio_get_value(mp_data->gdev->irq_gpio) == 1) {
		GTP_ERROR("INT pin state != LOW test fail!");
		return -1;
	}

	read_buf[2] |= BIT7;

	gcore_idm_write_reg(0xC0000098, read_buf, sizeof(read_buf));

	msleep(1);

	if (gpio_get_value(mp_data->gdev->irq_gpio) == 0) {
		GTP_ERROR("INT pin state != HIGH test fail!");
		return -1;
	}

	gcore_exit_idm_mode();

	mp_data->gdev->irq_enable(mp_data->gdev);
	mutex_unlock(&mp_data->gdev->transfer_lock);

	return 0;

}

int gcore_mp_test_chip_id(struct gcore_mp_data *mp_data)
{
	u8 buf[2] = { 0 };

	GTP_DEBUG("gcore mp test chip id");

	gcore_idm_read_id(buf, sizeof(buf));

	if ((buf[0] == mp_data->chip_id[0]) && (buf[1] == mp_data->chip_id[1])) {
		GTP_DEBUG("gcore mp test chip id match success!");
		return 0;
	} else {
		GTP_DEBUG("gcore mp test chip id match failed!");
		return -1;
	}
}


int gcore_mp_test_item_open(struct gcore_mp_data *mp_data)
{
	u8 cb_high = (u8)(mp_data->open_cb >> 8);
	u8 cb_low = (u8)mp_data->open_cb;
	u8 cmd[] = {0x40, 0xAA, 0x00, 0x32, 0x73, 0x71, cb_high, cb_low, 0x01};
	struct gcore_dev *gdev = mp_data->gdev;
	int ret = 0;
	int i = 0;
	u8 *opendata = NULL;
	u16 node_data = 0;

	GTP_DEBUG("gcore mp test item open");

	gdev->fw_event = FW_READ_OPEN;
	gdev->firmware = mp_data->open_data;
#ifdef CONFIG_TOUCH_DRIVER_INTERFACE_SPI
	gdev->fw_xfer = 2048;
#else
	gdev->fw_xfer = RAW_DATA_SIZE;
#endif


	mutex_lock(&gdev->transfer_lock);

	mp_test_fn.wait_int = true;


	ret = gcore_bus_write(cmd, sizeof(cmd));
	if (ret) {
		GTP_ERROR("write mp test open cmd fail!");
		return -1;
	}

	mutex_unlock(&gdev->transfer_lock);

	if (!wait_for_completion_interruptible_timeout(&mp_test_complete, 1 * HZ)) {
		GTP_ERROR("mp test item open timeout.");
		return -1;
	}

	opendata = mp_data->open_data;

	for (i = 0; i < RAW_DATA_SIZE/2; i++) {
		node_data = (opendata[1] << 8) | opendata[0];

		mp_data->open_node_val[i] = node_data;

		opendata += 2;
	}

	for (i = 0; i < RAW_DATA_SIZE/2; i++) {
		node_data = mp_data->open_node_val[i];
		GTP_DEBUG("i:%d open node_data:%d", i, node_data);


		if ((i == 0) || (i == 8) || (i == 9) || (i == 17)) {
			GTP_DEBUG("empty node need not judge.");
			continue;
		}

		if (node_data < mp_data->open_min) {
			GTP_ERROR("node_data %d < open_min %d test fail!", \
				i, mp_data->open_min);

			return -1;
		}

	}

	return 0;
}


int gcore_mp_test_item_open_reply(u8 *buf, int len)
{
	int ret = 0;

	if (buf == NULL) {
		GTP_ERROR("receive buffer is null!");
		return -1;
	}

	ret = gcore_bus_read(buf, len);
	if (ret) {
		GTP_ERROR("read mp test open data error.");
		return -1;
	}

	mp_test_fn.wait_int = false;
	complete(&mp_test_complete);


	return 0;
}


int gcore_mp_test_item_short(struct gcore_mp_data *mp_data)
{
	u8 cb_high = (u8)(mp_data->short_cb >> 8);
	u8 cb_low = (u8)mp_data->short_cb;
	u8 cmd[] = {0x40, 0xAA, 0x00, 0x32, 0x73, 0x71, cb_high, cb_low, 0x02};
	struct gcore_dev *gdev = mp_data->gdev;
	int ret = 0;
	int i = 0;
	u8 *shortdata = NULL;
	u16 node_data = 0;

	GTP_DEBUG("gcore mp test item short");

	gdev->fw_event = FW_READ_SHORT;
	gdev->firmware = mp_data->short_data;
#ifdef CONFIG_TOUCH_DRIVER_INTERFACE_SPI
	gdev->fw_xfer = 2048;
#else
	gdev->fw_xfer = RAW_DATA_SIZE;
#endif


	mutex_lock(&gdev->transfer_lock);

	mp_test_fn.wait_int = true;


	ret = gcore_bus_write(cmd, sizeof(cmd));
	if (ret) {
		GTP_ERROR("write mp test open cmd fail!");
		return -1;
	}

	mutex_unlock(&gdev->transfer_lock);

	if (!wait_for_completion_interruptible_timeout(&mp_test_complete, 1 * HZ)) {
		GTP_ERROR("mp test item open timeout.");
		return -1;
	}

	shortdata = mp_data->short_data;

	for (i = 0; i < RAW_DATA_SIZE/2; i++) {
		node_data = (shortdata[1] << 8) | shortdata[0];

		mp_data->short_node_val[i] = node_data;

		shortdata += 2;
	}

	for (i = 0; i < RAW_DATA_SIZE/2; i++) {
		node_data = mp_data->short_node_val[i];
		GTP_DEBUG("i:%d short node_data:%d", i, node_data);

		if ((i == 0) || (i == 8) || (i == 9) || (i == 17)) {
			GTP_DEBUG("empty node need not judge.");
			continue;
		}

		if (node_data < mp_data->short_min) {
			GTP_ERROR("node_data %d < short_min %d test fail!", \
				i, mp_data->short_min);

			return -1;
		}

	}

	return 0;
}

static int mp_test_event_handler(void *p)
{
	struct gcore_dev *gdev = (struct gcore_dev *)p;
	struct sched_param param = {.sched_priority = 4};

	sched_setscheduler(current, SCHED_RR, &param);
	do {
		set_current_state(TASK_INTERRUPTIBLE);

		wait_event_interruptible(gdev->wait, mp_test_fn.event_flag == true);
		mp_test_fn.event_flag = false;

		switch (gdev->fw_event) {
		case FW_READ_OPEN:
			gcore_mp_test_item_open_reply(gdev->firmware, gdev->fw_xfer);
			break;

		case FW_READ_SHORT:
			gcore_mp_test_item_open_reply(gdev->firmware, gdev->fw_xfer);
			break;

		default:
			break;
		}

		gdev->irq_enable(gdev);

	} while (!kthread_should_stop());

	return 0;
}

int gcore_alloc_mp_test_mem(struct gcore_mp_data *mp_data)
{
	if (mp_data->test_open) {
		if (mp_data->open_data == NULL) {
#ifdef CONFIG_TOUCH_DRIVER_INTERFACE_SPI
			mp_data->open_data = kzalloc(2048, GFP_KERNEL);
#else
			mp_data->open_data = kzalloc(RAW_DATA_SIZE, GFP_KERNEL);
#endif
			if (!mp_data->open_data) {
				GTP_ERROR("allocate mp test open_data mem fail!");
				return -ENOMEM;
			}
		}

		if (mp_data->open_node_val == NULL) {
			mp_data->open_node_val = (u16 *)kzalloc(RAW_DATA_SIZE, GFP_KERNEL);
			if (!mp_data->open_node_val) {
				GTP_ERROR("allocate mp test open_node_val mem fail!");
				return -ENOMEM;
			}
		}

	}

	if (mp_data->test_short) {
		if (mp_data->short_data == NULL) {
#ifdef CONFIG_TOUCH_DRIVER_INTERFACE_SPI
			mp_data->short_data = kzalloc(2048, GFP_KERNEL);
#else
			mp_data->short_data = kzalloc(RAW_DATA_SIZE, GFP_KERNEL);
#endif
			if (!mp_data->short_data) {
				GTP_ERROR("allocate mp test short_data mem fail!");
				return -ENOMEM;
			}
		}

		if (mp_data->short_node_val == NULL) {
			mp_data->short_node_val = (u16 *)kzalloc(RAW_DATA_SIZE, GFP_KERNEL);
			if (!mp_data->short_node_val) {
				GTP_ERROR("allocate mp test short_node_val mem fail!");
				return -ENOMEM;
			}
		}
	}

	return 0;
}

void gcore_free_mp_test_mem(struct gcore_mp_data *mp_data)
{
	if (mp_data->test_open) {
		if (mp_data->open_data) {
			kfree(mp_data->open_data);
			mp_data->open_data = NULL;
		}

		if (mp_data->open_node_val) {
			kfree(mp_data->open_node_val);
			mp_data->open_node_val = NULL;
		}
	}

	if (mp_data->test_short) {
		if (mp_data->short_data) {
			kfree(mp_data->short_data);
			mp_data->short_data = NULL;
		}

		if (mp_data->short_node_val) {
			kfree(mp_data->short_node_val);
			mp_data->short_node_val = NULL;
		}
	}
}

int dump_mp_data_row_to_buffer(char *buf, size_t size, const u16 *data,
	int cols, const char *prefix, const char *suffix, char seperator)
{
	int c, count = 0;

	if (prefix) {
		count += scnprintf(buf, size, "%s", prefix);
	}

	for (c = 0; c < cols; c++) {
		count += scnprintf(buf + count, size - count,
			"%4u%c ", data[c], seperator);
	}

	if (suffix) {
		count += scnprintf(buf + count, size - count, "%s", suffix);
	}

	return count;

}

int dump_mp_data_to_csv_file(const char *filepath, int flags,
	const u16 *data, int rows, int cols)
{
	struct file *file;
	int r = 0;
	int ret = 0;
	loff_t pos = 0;
	const u16 *n_val = data;

	GTP_DEBUG("dump mp data to csv file: %s row: %d	col: %d",
		filepath, rows, cols);

	file = filp_open(filepath, flags, 0666);
	if (IS_ERR(file)) {
		GTP_ERROR("Open file %s failed", filepath);
		return -1;
	}

	for (r = 0; r < rows; r++) {
		char linebuf[256];
		int len;

		len = dump_mp_data_row_to_buffer(linebuf, sizeof(linebuf), n_val,
			cols, NULL, "\n", ',');
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
		ret = kernel_write(file, linebuf, len, &pos);
#else
		ret = kernel_write(file, linebuf, len, pos);
		pos += len;
#endif
		if (ret != len) {
			GTP_ERROR("write to file %s failed %d", filepath, ret);
			goto fail1;
		}

		n_val += cols;
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
	ret = kernel_write(file, "\n", 1, &pos);
#else
	ret = kernel_write(file, "\n", 1, pos);
	pos++;
#endif
	if (ret != 1) {
		GTP_ERROR("write newline to file %s failed %d", filepath, ret);
		goto fail1;
	}


fail1:
	filp_close(file, NULL);

	return ret;

}

void dump_mp_data_to_seq_file(struct seq_file *m, const u16 *data,
	int rows, int cols)
{
	int r = 0;
	const u16 *n_vals = data;

	for (r = 0; r < rows; r++) {
		char linebuf[256];
		int len;

		len = dump_mp_data_row_to_buffer(linebuf, sizeof(linebuf),
			n_vals, cols, NULL, "\n", ',');
		seq_puts(m, linebuf);

		n_vals += cols;
	}
}


int gcore_save_mp_data_to_file(struct gcore_mp_data *mp_data)
{
	int rows = RAWDATA_ROW;
	int cols = RAWDATA_COLUMN;
	int ret = 0;

	GTP_DEBUG("save mp data to file");

	if (mp_data->test_open) {
		ret = dump_mp_data_to_csv_file(MP_OPEN_TEST_DATA_FILEPATH,
			O_RDWR | O_CREAT | O_TRUNC, mp_data->open_node_val, rows, cols);
		if (ret < 0) {
			GTP_ERROR("dump mp open test data to file failed");
			return ret;
		}
	}

	if (mp_data->test_short) {
		ret = dump_mp_data_to_csv_file(MP_SHORT_TEST_DATA_FILEPATH,
			O_RDWR | O_CREAT | O_TRUNC, mp_data->short_node_val, rows, cols);
		if (ret < 0) {
			GTP_ERROR("dump mp short test data to file failed");
			return ret;
		}
	}

	return 0;
}

static int32_t gcore_mp_test_open(struct inode *inode, struct file *file)
{
	struct gcore_mp_data *mp_data = PDE_DATA(inode);

	if (mp_data == NULL) {
		GTP_ERROR("Open selftest proc with mp data = NULL");
		return -1;
	}

	GTP_DEBUG("gcore mp test open");

	gcore_parse_mp_test_dt(mp_data);

	gcore_alloc_mp_test_mem(mp_data);

	if (mp_data->test_int_pin) {
		mp_data->int_pin_test_result = gcore_mp_test_int_pin(mp_data);
	}

	msleep(1);

	if (mp_data->test_chip_id) {
		mp_data->chip_id_test_result = gcore_mp_test_chip_id(mp_data);
	}

	msleep(1);

	if (mp_data->test_open || mp_data->test_short) {
		GTP_DEBUG("mp test begin to updata mp bin");

#ifdef CONFIG_GCORE_AUTO_UPDATE_FW_HOSTDOWNLOAD
		if (gcore_auto_update_hostdownload_proc(gcore_mp_FW)) {
			GTP_ERROR("mp bin update hostdownload proc fail");
		}
//		gcore_request_firmware_update_work(NULL);
#endif
		else {
			msleep(1);

			if (mp_data->test_open) {
				mp_data->open_test_result = gcore_mp_test_item_open(mp_data);
			}

			msleep(1);

			if (mp_data->test_short) {
				mp_data->short_test_result = gcore_mp_test_item_short(mp_data);
			}
		}

	}

	gcore_save_mp_data_to_file(mp_data);


	if (seq_open(file, &gcore_mptest_seq_ops)) {
		GTP_ERROR("seq open fail!");
		return -1;
	}

	((struct seq_file *)file->private_data)->private = mp_data;

	return 0;
}

struct file_operations gcore_mp_fops = {
	.owner = THIS_MODULE,
	.open = gcore_mp_test_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};


int gcore_mp_test_fn_init(struct gcore_dev *gdev)
{
	struct gcore_mp_data *mp_data = NULL;

	GTP_DEBUG("gcore mp test fn init");

	mp_thread = kthread_run(mp_test_event_handler, gdev, "gcore_mptest");

	mp_data = kzalloc(sizeof(struct gcore_mp_data), GFP_KERNEL);
	if (!mp_data) {
		GTP_ERROR("allocate mp_data mem fail! failed");
		return -1;
	}

	mp_data->gdev = gdev;

	g_mp_data = mp_data;

	mp_data->mp_proc_entry = proc_create_data(MP_TEST_PROC_FILENAME,
		S_IRUGO, NULL, &gcore_mp_fops, mp_data);
	if (mp_data->mp_proc_entry == NULL) {
		GTP_ERROR("create mp test proc entry selftest failed");
		goto fail1;
	}

	return 0;

fail1:
	kfree(mp_data);
	return -1;

}

void gcore_mp_test_fn_remove(struct gcore_dev *gdev)
{
	kthread_stop(mp_thread);

	if (g_mp_data->mp_proc_entry) {
		remove_proc_entry(MP_TEST_PROC_FILENAME, NULL);
	}

	gcore_free_mp_test_mem(g_mp_data);

	if (g_mp_data) {
		kfree(g_mp_data);
		g_mp_data = NULL;
	}

	return;
}

#if 0

static int __init gcore_mp_test_init(void)
{
	GTP_DEBUG("gcore_fw_update_init.");

	gcore_new_function_register(&mp_test_fn);

	return 0;
}

static void __exit gcore_mp_test_exit(void)
{
	GTP_DEBUG("gcore_fw_update_exit.");

	gcore_new_function_unregister(&mp_test_fn);

	return;
}

module_init(gcore_mp_test_init);
module_exit(gcore_mp_test_exit);

MODULE_AUTHOR("GalaxyCore, Inc.");
MODULE_DESCRIPTION("GalaxyCore Touch MP Test Module");
MODULE_LICENSE("GPL");

#endif
