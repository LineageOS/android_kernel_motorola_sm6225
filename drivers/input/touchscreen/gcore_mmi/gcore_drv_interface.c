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
#include "gcore_ioctl.h"
#include <linux/ioctl.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <asm/uaccess.h>



#define GCORE_PROC_FILE "gcore_app"

#define GCORE_CHRDEV_NUM  2

struct gcore_dev *gdev_intf;
struct reg_msg msg_reg;


static int gcore_app_open(struct inode *inode, struct file *file);
static ssize_t gcore_app_read(struct file *file, char __user *buffer, size_t count, loff_t *ppos);
static long gcore_app_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);


struct file_operations gcore_app_fops = {
	.open = gcore_app_open,
	.read = gcore_app_read,
	.write = NULL,
	.unlocked_ioctl = gcore_app_ioctl,
	.compat_ioctl = gcore_app_ioctl,
};


struct proc_dir_entry *gcore_proc_entry;


int gcore_app_open(struct inode *inode, struct file *file)
{
	GTP_DEBUG("gcore_cdev_open.");

	file->private_data = gdev_intf;

	return 0;
}

ssize_t gcore_app_read(struct file *file, char __user *buffer, size_t count, loff_t *ppos)
{
	int ret = 0;
	struct gcore_dev *gdev = file->private_data;
	u8 *touch_data_p = gdev->touch_data;
//	int i = 0;

	gdev->usr_read = true;

	GTP_DEBUG("app read before sleep.");
	while (gdev->data_ready == false) {
		if (wait_event_interruptible(gdev->usr_wait, gdev->data_ready == true))
			return -ERESTARTSYS;
	}
	GTP_DEBUG("app read after sleep.");
/*
	for (i = 0; i < 576; i++)
	{
		*(touch_data_p + 65 + 2*i) = (i & 0xFF);
		*(touch_data_p + 65 + 2*i + 1) = ((i & 0xFF00) >> 8);
	}

	GTP_DEBUG("read data36 37 %x %x", *(touch_data_p+65+36), *(touch_data_p+65+37));

	GTP_DEBUG("read data38 39 %x %x", *(touch_data_p+65+38), *(touch_data_p+65+39));

	GTP_DEBUG("read data72 73 %x %x", *(touch_data_p+65+72), *(touch_data_p+65+73));

	GTP_DEBUG("read data74 75 %x %x", *(touch_data_p+65+74), *(touch_data_p+65+75));
*/
	ret = copy_to_user(buffer, touch_data_p + DEMO_DATA_SIZE, RAW_DATA_SIZE);

	gdev->usr_read = false;
	gdev->data_ready = false;

	return RAW_DATA_SIZE;

}

ssize_t gcore_app_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	return count;
}

long gcore_app_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct gcore_dev *gdev = filp->private_data;
	u8 *fw_buf = NULL;
	u8 read_data[4] = { 0 };
	u8 version[20] = { 0 };
	u8 rawdata_res[10] = { 0 };
	int ret = 0;

	GTP_DEBUG("IOCTL CMD:%x", cmd);

	if (_IOC_TYPE(cmd) != GALAXYCORE_MAGIC_NUMBER) {
		GTP_ERROR("command type [%c] error!", _IOC_TYPE(cmd));
		return -ENOTTY;
	}

	if (_IOC_NR(cmd) > GALAXYCORE_MAX_NR) {
		GTP_ERROR("command numer [%d] exceeded!", _IOC_NR(cmd));
		return -ENOTTY;
	}

	switch (cmd) {
	case IOC_APP_READ_FW_VERSION:

		GTP_DEBUG("App ioctl get fw version");

		gcore_read_fw_version(read_data, sizeof(read_data));
		GTP_DEBUG("app read data %x %x %x %x", read_data[0], read_data[1], read_data[2], read_data[3]);

		ret = snprintf(version, sizeof(version), "%d.%d.%d.%d", read_data[1], read_data[0], read_data[3], read_data[2]);

		if (copy_to_user((char *)arg, version, strlen(version) + 1)) {
			return -EFAULT;
		}

		break;

	case IOC_APP_UPDATE_FW:
		fw_buf = (u8 *)kzalloc(FW_SIZE, GFP_KERNEL);
		if (fw_buf == NULL) {
			GTP_ERROR("fw data mem allocate fail");
			return -ENOMEM;
		}

		if (copy_from_user(fw_buf, (char *)arg, FW_SIZE)) {
			GTP_ERROR("copy fw_buf from user fail");
			return -EFAULT;
		}

		GTP_DEBUG("fw_buf: %x %x %x %x", fw_buf[0], fw_buf[1], \
			fw_buf[2], fw_buf[3]);


#ifdef CONFIG_GCORE_AUTO_UPDATE_FW_FLASHDOWNLOAD
		if (gcore_auto_update_flashdownload_proc(fw_buf)) {
			GTP_ERROR("app flashdownload proc fail");
			kfree(fw_buf);
			return -1;
		}
#else
		if (gcore_auto_update_hostdownload_proc(fw_buf)) {
			GTP_ERROR("app hostdownload proc fail");
			kfree(fw_buf);
			return -1;
		}
#endif

		if (fw_buf != NULL) {
			kfree(fw_buf);
			fw_buf = NULL;
		}

		break;

	case IOC_APP_DEMO:
		mutex_lock(&gdev->transfer_lock);
		gdev->irq_disable(gdev);

		if (gcore_fw_mode_set_proc2(0)) {
			GTP_ERROR("fw mode set demo proc fail");
			return -1;
		}

		gdev->irq_enable(gdev);
		mutex_unlock(&gdev->transfer_lock);

		break;

	case IOC_APP_DEMO_RAWDATA:
		mutex_lock(&gdev->transfer_lock);
		gdev->irq_disable(gdev);

		if (gcore_fw_mode_set_proc2(2)) {
			GTP_ERROR("fw mode set demo rawdata proc fail");
			return -1;
		}

		gdev->irq_enable(gdev);
		mutex_unlock(&gdev->transfer_lock);

		break;

	case IOC_APP_READ_REG:
		ret = copy_from_user(&msg_reg, (struct reg_msg __user *)arg, sizeof(msg_reg));
		if (ret) {
			GTP_ERROR("READ REG:copy from user fail.");
			return -1;
		}

		GTP_DEBUG("read reg: addr(%x) len(%d)", msg_reg.addr, msg_reg.length);

		gcore_fw_read_reg(msg_reg.addr, msg_reg.buffer, msg_reg.length);

		GTP_DEBUG("read msg buffer %x %x", msg_reg.buffer[0], msg_reg.buffer[1]);

		ret = copy_to_user((struct reg_msg __user *)arg, &msg_reg, sizeof(msg_reg));
		if (ret) {
			return -1;
		}

		break;

	case IOC_APP_WRITE_REG:
		ret = copy_from_user(&msg_reg, (struct reg_msg __user *)arg, sizeof(msg_reg));
		if (ret) {
			GTP_ERROR("WRITE REG:copy from user fail.");
			return -1;
		}

		GTP_DEBUG("write reg: addr(%x) len(%d)", msg_reg.addr, msg_reg.length);

		ret = gcore_fw_write_reg(msg_reg.addr, msg_reg.buffer, msg_reg.length);
		if (ret) {
			GTP_ERROR("app ioctl write reg fail");
			return -1;
		}

		break;

	case IOC_APP_GET_RAWDATA_RES:

		GTP_DEBUG("App ioctl get rawdata xy");

		snprintf(rawdata_res, sizeof(rawdata_res), "%d*%d", RAWDATA_ROW, RAWDATA_COLUMN);

		if (copy_to_user((char *)arg, rawdata_res, strlen(rawdata_res) + 1)) {
			return -EFAULT;
		}

		break;

	default:
		GTP_ERROR("ioctl unknow cmd!");
		return -ENOTTY;
		break;
	}

	return 0;
}


int gcore_app_node_init(void)
{
	gcore_proc_entry = proc_create(GCORE_PROC_FILE, 0644, NULL, &gcore_app_fops);
	if (gcore_proc_entry == NULL) {
		GTP_ERROR("create proc entry gcore_app failed");
		return -1;
	}

	return 0;
}

void gcore_app_node_deinit(void)
{
	if (gcore_proc_entry != NULL) {
		remove_proc_entry(GCORE_PROC_FILE, NULL);
	}
}

static int gcore_interface_fn_init(struct gcore_dev *gdev);
static void gcore_interface_fn_remove(struct gcore_dev *gdev);

struct gcore_exp_fn fs_interf_fn = {
	.fn_type = GCORE_FS_INTERFACE,
	.wait_int = false,
	.init = gcore_interface_fn_init,
	.remove = gcore_interface_fn_remove,
};


/****************** Tool **************************/

#define TOOL_HEADER_SIZE            8

static int gcore_tool_open(struct inode *inode, struct file *file);
static ssize_t gcore_tool_read(struct file *file, char __user *buffer, size_t count, loff_t *ppos);
static ssize_t gcore_tool_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos);
static long gcore_tool_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);

struct file_operations gcore_tool_fops = {
	.open = gcore_tool_open,
	.read = gcore_tool_read,
	.write = gcore_tool_write,
	.unlocked_ioctl = gcore_tool_ioctl,
	.compat_ioctl = gcore_tool_ioctl,
};

enum TOOL_MODE {
	LINE,
	ISP
};

struct gcore_tool_data {
	dev_t devno;
	struct cdev gcore_cdev;
	struct class *gcore_class;
	struct device *gcore_dev;

	u8 *isp_reply_data;
	u8 *tool_isp_buf;

	enum TOOL_MODE tmode;
};

struct gcore_tool_data tool_info = {
	.isp_reply_data = NULL,
	.tool_isp_buf = NULL,
	.tmode = LINE,
};

int gcore_tool_line_read(struct gcore_dev *gdev, char __user *buffer)
{
	int ret = 0;
	char tool_header[TOOL_HEADER_SIZE] = { 0 };
	int data_size = DEMO_DATA_SIZE;
	u8 data_length_h = 0;
	u8 data_length_l = 0;
	u8 *touch_data_p = gdev->touch_data;

//	struct sched_param param = {.sched_priority = 4};
//	sched_setscheduler(current, SCHED_RR, &param);

	gdev->usr_read = true;

	GTP_DEBUG("read before sleep.");
	while (gdev->data_ready == false) {
		if (wait_event_interruptible(gdev->usr_wait, gdev->data_ready == true))
			return -ERESTARTSYS;
	}
	GTP_DEBUG("read after sleep.");

	tool_header[0] = 0x40;
	tool_header[1] = 0xB1;
	tool_header[2] = gdev->fw_mode;

	if (gdev->fw_mode == DEMO) {
		data_size = DEMO_DATA_SIZE;
	} else if (gdev->fw_mode == RAWDATA) {
		data_size = RAW_DATA_SIZE;
	} else if (gdev->fw_mode == DEMO_RAWDATA) {
		data_size = DEMO_RAWDATA_SIZE;
	}

	data_length_h = (u8)(data_size >> 8);
	data_length_l = (u8)(data_size);

	tool_header[3] = 0x00;
	tool_header[4] = 0x73;
	tool_header[5] = 0x71;
	tool_header[6] = data_length_h;
	tool_header[7] = data_length_l;

	ret = copy_to_user(buffer, tool_header, 8);
	ret = copy_to_user(buffer + 8, touch_data_p, data_size);

	gdev->data_ready = false;
	gdev->usr_read = false;

	return data_size + TOOL_HEADER_SIZE;

}

int gcore_tool_isp_read(struct gcore_dev *gdev, char __user *buffer)
{
	int ret;

	GTP_DEBUG("isp read before sleep.");
	while (fs_interf_fn.wait_int == false) {
		if (wait_event_interruptible(gdev->wait, fs_interf_fn.wait_int == true))
			return -ERESTARTSYS;
	}
	GTP_DEBUG("isp read after sleep.");

	ret = gcore_bus_read(tool_info.isp_reply_data, 64);
	if (ret) {
		GTP_ERROR("write cmd read flash id error.");
		return -1;
	}

	ret = copy_to_user(buffer, tool_info.isp_reply_data, 64);

	return 64;
}


int tool_mode_is_isp(void)
{
	return (tool_info.tmode == ISP) ? 1 : 0;
}

int gcore_tool_open(struct inode *inode, struct file *file)
{
	GTP_DEBUG("gcore_cdev_open.");

	tool_info.isp_reply_data = (u8 *)kzalloc(1024, GFP_KERNEL);
	if (tool_info.isp_reply_data == NULL) {
		GTP_ERROR("flash_op_buf mem allocate fail!");
		return -ENOMEM;
	}

	tool_info.tool_isp_buf = (u8 *)kzalloc(1024, GFP_KERNEL);
	if (tool_info.tool_isp_buf == NULL) {
		GTP_ERROR("flash_op_buf mem allocate fail!");
		return -ENOMEM;
	}

	file->private_data = gdev_intf;

	return 0;
}

ssize_t gcore_tool_read(struct file *file, char __user *buffer, size_t count, loff_t *ppos)
{
	struct gcore_dev *gdev = file->private_data;
	int ret = 0;

	switch (tool_info.tmode) {
	case LINE:
		ret = gcore_tool_line_read(gdev, buffer);
		break;

	case ISP:
		ret = gcore_tool_isp_read(gdev, buffer);
		break;

	default:
		break;
	}

	return ret;

}

ssize_t gcore_tool_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	int ret = 0;
//	u8 buff_send[1024] = { 0 };

	ret = copy_from_user(tool_info.tool_isp_buf, buffer, count);
//	GTP_DEBUG("buff_send %x %x %x %x", buff_send[0], buff_send[1], buff_send[2], buff_send[3]);
	GTP_DEBUG("count=%d", (int)count);
	ret = gcore_bus_write(tool_info.tool_isp_buf, count);
	if (ret) {
		GTP_ERROR("write cmd read flash id error.");
		return -1;
	}

	return count;
}

long gcore_tool_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
//	int ret = 0;
	int idm_op = 0;

	GTP_DEBUG("IOCTL CMD:%x", cmd);

	if (_IOC_TYPE(cmd) != GALAXYCORE_MAGIC_NUMBER) {
		GTP_ERROR("command type [%c] error!", _IOC_TYPE(cmd));
		return -ENOTTY;
	}

	if (_IOC_NR(cmd) > GALAXYCORE_MAX_NR) {
		GTP_ERROR("command numer [%d] exceeded!", _IOC_NR(cmd));
		return -ENOTTY;
	}

	switch (cmd) {
	case IOC_DEBUG_TIME_RST0:
		GTP_DEBUG("set rst low");

		break;

	case IOC_DEBUG_TIME_RST1:
		GTP_DEBUG("set rst high");

		break;

	case IOC_TOOL_MODE:
		tool_info.tmode = (int)arg;
		/*
		ret = copy_from_user(&tool_info.tmode, &(int __user)arg, sizeof(int));
		if (ret)
		{
			return -EFAULT;
		}
		*/

		break;

	case IOC_TOOL_IDM_OPERATION:
		idm_op = (int)arg;
		if (idm_op == 0x01) {
//			gcore_enter_idm_mode();
		} else if (idm_op == 0x02) {
//			gcore_exit_idm_mode();
		}

		break;

	default:
		return -ENOTTY;

		break;
	}

	return 0;
}

int gcore_tool_node_init(void)
{
	alloc_chrdev_region(&tool_info.devno, 0, GCORE_CHRDEV_NUM, "gcore");

	cdev_init(&tool_info.gcore_cdev, &gcore_tool_fops);

	if (cdev_add(&tool_info.gcore_cdev, tool_info.devno, 1) < 0) {
		GTP_ERROR("cdev_add fail!");
		goto cdev_add_fail;
	}

	tool_info.gcore_class = class_create(THIS_MODULE, "gcore");
	if (IS_ERR(tool_info.gcore_class)) {
		GTP_ERROR("class create fail!");
		goto cls_cre_fail;
	}

	tool_info.gcore_dev = device_create(tool_info.gcore_class, NULL, tool_info.devno, NULL, "gcore");
	if (IS_ERR(tool_info.gcore_dev)) {
		GTP_ERROR("device create fail!");
		goto dev_cre_fail;
	}

	if (gcore_create_attribute(tool_info.gcore_dev)) {
		GTP_ERROR("tool init create attribute fail");
	}

	return 0;

dev_cre_fail:
	class_destroy(tool_info.gcore_class);
cls_cre_fail:
	cdev_del(&tool_info.gcore_cdev);
cdev_add_fail:
	unregister_chrdev_region(tool_info.devno, 1);
	return -1;
}

void gcore_tool_node_deinit(void)
{
	device_destroy(tool_info.gcore_class, tool_info.devno);
	class_destroy(tool_info.gcore_class);
	cdev_del(&tool_info.gcore_cdev);
	unregister_chrdev_region(tool_info.devno, 1);
}

static ssize_t path_show(struct device *pDevice, struct device_attribute *pAttr, \
	char *pBuf)
{
	ssize_t blen;
	const char *path;

	path = kobject_get_path(&gdev_intf->bus_device->dev.kobj, GFP_KERNEL);
	blen = scnprintf(pBuf, PAGE_SIZE, "%s\n", path ? path : "na");
	kfree(path);

	return blen;
}

/* Attribute: vendor (RO) */
static ssize_t vendor_show(struct device *dev, struct device_attribute *attr, \
	char *buf)
{
	GTP_DEBUG("*** %s() vendor = %s ***", __func__, "galaxycore");

	return scnprintf(buf, PAGE_SIZE, "galaxygcore\n");
}

/* Attribute: ic_ver (RO) */
static ssize_t ic_ver_show(struct device *dev, struct device_attribute *attr, \
	char *buf)
{
	u8 read_data[4] = { 0 };

	gcore_read_fw_version(read_data, sizeof(read_data));

	return scnprintf(buf, PAGE_SIZE, "FW Ver:%d.%d.%d.%d\n", \
		read_data[1], read_data[0], read_data[3], read_data[2]);
}

static struct device_attribute touchscreen_attributes[] = {
	__ATTR_RO(path),
	__ATTR_RO(vendor),
	__ATTR_RO(ic_ver),
	__ATTR_NULL
};


static struct class *touchscreen_class;
static struct device *touchscreen_class_dev;

int gcore_sys_node_init(void)
{
	int i = 0;
	int ret = 0;
	struct device_attribute *attrs = touchscreen_attributes;

	touchscreen_class = class_create(THIS_MODULE, "touchscreen");
	if (IS_ERR(touchscreen_class)) {
		GTP_ERROR("create touchscreen class failed");
		return -1;
	}

	touchscreen_class_dev = device_create(touchscreen_class, NULL, MKDEV(MAJOR(tool_info.devno), 1), \
		NULL, "ts_gcore");
	if (IS_ERR(touchscreen_class_dev)) {
		GTP_ERROR("create touchscreen device failed");
		return -1;
	}

	for (i = 0; attrs[i].attr.name != NULL; i++) {
		ret = device_create_file(touchscreen_class_dev, &attrs[i]);
		if (ret) {
			break;
		}
	}

	return ret;

}

void gcore_sys_node_deinit(void)
{
	device_destroy(touchscreen_class, MKDEV(MAJOR(tool_info.devno), 1));
	class_destroy(touchscreen_class);
}

int gcore_interface_fn_init(struct gcore_dev *gdev)
{
	GTP_DEBUG("gcore_interface_fn_init.");

	gdev_intf = gdev;

	if (gcore_app_node_init()) {
		GTP_ERROR("app node init failed");
		return -1;
	}

	if (gcore_tool_node_init()) {
		GTP_ERROR("tool node init failed");
		goto fail1;
	}

	if (gcore_sys_node_init()) {
		GTP_ERROR("sys node init failed");
		goto fail2;
	}

	return 0;

fail2:
	gcore_tool_node_deinit();
fail1:
	gcore_app_node_deinit();

	return -1;
}

void gcore_interface_fn_remove(struct gcore_dev *gdev)
{
	GTP_DEBUG("gcore_interface_fn_remove.");

	gcore_app_node_deinit();
	gcore_tool_node_deinit();
	gcore_sys_node_deinit();

	return;
}


#if 0

static int __init gcore_interface_init(void)
{

	GTP_DEBUG("gcore_interface_init.");

	gcore_new_function_register(&fs_interf_fn);

	return 0;
}

static void __exit gcore_interface_exit(void)
{
	GTP_DEBUG("gcore_interface_exit.");

	gcore_new_function_unregister(&fs_interf_fn);

	return;
}

module_init(gcore_interface_init);
module_exit(gcore_interface_exit);

MODULE_AUTHOR("GalaxyCore, Inc.");
MODULE_DESCRIPTION("GalaxyCore Drv FS Interface.");
MODULE_LICENSE("GPL");

#endif
