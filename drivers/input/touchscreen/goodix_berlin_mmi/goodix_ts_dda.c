/*
 * Copyright (C) 2022 Motorola Mobility LLC
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
#include <linux/cdev.h>
#include <linux/string.h>
#include <linux/fs.h>
#include <linux/version.h>
#include "goodix_ts_core.h"

//cdev registration
#define DRIVER_NAME "goodix_penraw_driver"
#define DEVICE_NAME "goodix_penraw"
#define MINOR_NUMBER_START ((const unsigned int)0) /* the minor number starts at */
#define NUMBER_MINOR_NUMBER ((const unsigned int)1) /* the number of minor numbers */
#define DDA_MAX_BUFFER 32
#define MAX_IO_CONTROL_REPORT 16

/* for DDA goodix_penraw */
enum{
	DATA_TYPE_RAW = 0
};

struct dda_pen_coords {
	signed char status;
	signed char tool_type;
	signed char tilt_x;
	signed char tilt_y;
	unsigned long int x;
	unsigned long int y;
	unsigned long int p;
};

struct dda_pen_info {
	unsigned char frame_no;
	unsigned char data_type;
	unsigned char reserve[2];
	struct dda_pen_coords coords;
};

struct dda_io_pen_reports {
	unsigned char report_num;
	unsigned char reserve[3];
	struct dda_pen_info pen_info[MAX_IO_CONTROL_REPORT];
};

struct dda_pen_report_buffer{
	unsigned char pen_report_num;
	unsigned char frame_no;
	unsigned char buffer_wp;
	unsigned char reserve[1];
	struct dda_pen_info pen_report_buffer[DDA_MAX_BUFFER];
};

struct dda_cdev_control{
	unsigned int major_number; /* the major number of the device */
	struct cdev penraw_char_dev; /* character device */
	struct class* penraw_char_dev_class; /* class object */
};

//DDA cdev
static struct dda_cdev_control dda_cdev_ctrl;

//DDA raw pen report buffer
static struct dda_pen_report_buffer dda_pen_report_buf;

spinlock_t lock;
int open_count = 0;

// Linux 2.0/2.2
static int penraw_open(struct inode * inode, struct file * file)
{
	spin_lock(&lock);
	if (open_count) {
		spin_unlock(&lock);
		return -EBUSY;
	}
	open_count++;
	spin_unlock(&lock);

	return 0;
}

// Linux 2.1: int type
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 1, 0)
static int penraw_close(struct inode * inode, struct file * file)
{
	spin_lock(&lock);
	open_count--;
	spin_unlock(&lock);

	return 0;
}
#else
static void penraw_close(struct inode * inode, struct file * file)
{
	spinlock(&lock);
	open_count--;
	spin_unlock(&lock);
}
#endif

#define PENRAW_IOC_TYPE 'P'
#define PENRAW_GET_VALUES _IOR(PENRAW_IOC_TYPE, 0, struct dda_io_pen_reports)

static struct dda_io_pen_reports pen_reports; // return report
static long penraw_ioctl(struct file *file,
		unsigned int cmd, unsigned long arg)
{
	unsigned long flags;
	struct dda_pen_info *ppen_info;
	unsigned char cnt;
	unsigned char pen_buffer_rp;
	unsigned char wp;
	unsigned char num;

	switch (cmd) {
		case PENRAW_GET_VALUES:
			local_irq_save(flags);
			wp = dda_pen_report_buf.buffer_wp;
			num = dda_pen_report_buf.pen_report_num;
			local_irq_restore(flags);
			if(MAX_IO_CONTROL_REPORT <= num) {
				pen_buffer_rp = (unsigned char)((wp + (DDA_MAX_BUFFER -
					MAX_IO_CONTROL_REPORT)) % DDA_MAX_BUFFER);
			} else {
				pen_buffer_rp = 0;
			}
			memset(&pen_reports, 0, sizeof(pen_reports));
			pen_reports.report_num = num;
			ppen_info = (struct dda_pen_info *)&pen_reports.pen_info[0];
			for(cnt = 0; cnt < num; cnt++) {
				memcpy(ppen_info, &dda_pen_report_buf.pen_report_buffer[pen_buffer_rp],
					sizeof(struct dda_pen_info));
				ppen_info++;
				pen_buffer_rp++;
				if(DDA_MAX_BUFFER == pen_buffer_rp) {
					pen_buffer_rp = 0;
				}
			}
			if (copy_to_user((void __user *)arg, &pen_reports, sizeof(pen_reports))) {
				return -EFAULT;
			}
			break;
		default:
			ts_info("unsupported command %d", cmd);
			return -EFAULT;
	}
	return 0;
}

static struct file_operations penraw_fops = {
	.owner = THIS_MODULE,
	.open = penraw_open,
	.release = penraw_close,
	.unlocked_ioctl = penraw_ioctl,
};

/* functions for DDA Features */
void goodix_dda_process_pen_report(struct goodix_pen_data *pen_data) {
	struct dda_pen_info *ppen_info;

	if (pen_data->coords.status) {
		// ioctl-DAA Buffering pen raw data
		ppen_info  = &dda_pen_report_buf.pen_report_buffer[0];
		ppen_info += dda_pen_report_buf.buffer_wp;
		memset(ppen_info, 0, sizeof(struct dda_pen_info));
		ppen_info->coords.status = (signed char)pen_data->coords.status;
		ppen_info->coords.tool_type = (signed char)pen_data->coords.tool_type;
		ppen_info->coords.tilt_x = pen_data->coords.tilt_x;
		ppen_info->coords.tilt_y = pen_data->coords.tilt_y;
		ppen_info->coords.x = pen_data->coords.x;
		ppen_info->coords.y = pen_data->coords.y;
		ppen_info->coords.p = pen_data->coords.p;
		ppen_info->frame_no = dda_pen_report_buf.frame_no;
		ppen_info->data_type = DATA_TYPE_RAW;
		dda_pen_report_buf.frame_no++;
		if(MAX_IO_CONTROL_REPORT > dda_pen_report_buf.pen_report_num) {
			// Max count: MAX_IO_CONTROL_REPORT
			dda_pen_report_buf.pen_report_num++;
		}
		dda_pen_report_buf.buffer_wp++;
		if(DDA_MAX_BUFFER == dda_pen_report_buf.buffer_wp) {
			dda_pen_report_buf.buffer_wp = 0;
		}
	}
}

void goodix_stylus_dda_init(void) {
	dda_pen_report_buf.pen_report_num = 0;
	dda_pen_report_buf.frame_no = 0;
	dda_pen_report_buf.buffer_wp = 0;
	memset(&dda_cdev_ctrl.penraw_char_dev, 0, sizeof(dda_cdev_ctrl.penraw_char_dev));
	dda_cdev_ctrl.penraw_char_dev_class = NULL;
	spin_lock_init(&lock);
}

int goodix_stylus_dda_register_cdevice(void) {
	int ret;
	int alloc_ret;
	int cdev_err;
	dev_t dev;
	struct device *penraw_dev;

	ts_info("goodix stylus dda register cdevice start");
	/* get not assigned major numbers */
	alloc_ret = alloc_chrdev_region(&dev, MINOR_NUMBER_START, NUMBER_MINOR_NUMBER,
		DRIVER_NAME);
	if (alloc_ret != 0) {
		ts_err("failed to alloc_chrdev_region()");
		unregister_chrdev_region(dev, NUMBER_MINOR_NUMBER);
		return alloc_ret;
	}

	/* get one number from the not-assigend numbers */
	dda_cdev_ctrl.major_number = MAJOR(dev);

	/* initialize cdev and function table */
	cdev_init(&dda_cdev_ctrl.penraw_char_dev, &penraw_fops);
	dda_cdev_ctrl.penraw_char_dev.owner = THIS_MODULE;

	/* register the driver */
	cdev_err = cdev_add(&dda_cdev_ctrl.penraw_char_dev, dev, NUMBER_MINOR_NUMBER);
	if (cdev_err != 0) {
		ts_err(KERN_ERR "failed to cdev_add()");
		unregister_chrdev_region(dev, NUMBER_MINOR_NUMBER);
		return cdev_err;
	}

	/* register a class */
	dda_cdev_ctrl.penraw_char_dev_class = class_create(THIS_MODULE, DEVICE_NAME);
	if (IS_ERR(dda_cdev_ctrl.penraw_char_dev_class)) {
		ts_err("class_create()");
		cdev_del(&dda_cdev_ctrl.penraw_char_dev);
		unregister_chrdev_region(dev, NUMBER_MINOR_NUMBER);
		ret = PTR_ERR(dda_cdev_ctrl.penraw_char_dev_class);
		dda_cdev_ctrl.penraw_char_dev_class = NULL;
		//return error code
		return ret;
	}

	/* create devive /dev/goodix_penraw*/
	penraw_dev = device_create(dda_cdev_ctrl.penraw_char_dev_class, NULL,
		MKDEV(dda_cdev_ctrl.major_number, 0), NULL, DEVICE_NAME);
	if (IS_ERR(penraw_dev)) {
		ts_err("device_create()");
		class_destroy(dda_cdev_ctrl.penraw_char_dev_class);
		dda_cdev_ctrl.penraw_char_dev_class = NULL;
		cdev_del(&dda_cdev_ctrl.penraw_char_dev);
		unregister_chrdev_region(dev, NUMBER_MINOR_NUMBER);
		ret = PTR_ERR(penraw_dev);
		penraw_dev = NULL;
		return ret;
	}

	ts_info("goodix stylus dda register cdevice success");
	return 0;
}

void goodix_stylus_dda_exit(void) {
	dev_t dev = MKDEV(dda_cdev_ctrl.major_number, MINOR_NUMBER_START);

	/* remove "/dev/goodix_penraw" */
	if (NULL != dda_cdev_ctrl.penraw_char_dev_class) {
		device_destroy(dda_cdev_ctrl.penraw_char_dev_class,
			MKDEV(dda_cdev_ctrl.major_number, 0));
	}

	/* remove class */
	if (NULL != dda_cdev_ctrl.penraw_char_dev_class) {
		class_destroy(dda_cdev_ctrl.penraw_char_dev_class);
	}

	/* remove driver */
	cdev_del(&dda_cdev_ctrl.penraw_char_dev);

	/* release the major number */
	unregister_chrdev_region(dev, NUMBER_MINOR_NUMBER);

	ts_info("goodix_stylus_dda_exit success");
}
