/*
 * Copyright (C) 2018 Motorola Mobility LLC.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

//#define DEBUG

#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/version.h>
#include <cam_cci_dev.h>
#include <media/v4l2-ioctl.h>
#include <media/cci_intf.h>

static int32_t cci_intf_xfer(
		struct msm_cci_intf_xfer *xfer,
		unsigned int cmd)
{
	int32_t rc, rc2;
	uint16_t addr;
	struct cam_sensor_cci_client cci_info = {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,19,0)
		.cci_subdev     = cam_cci_get_subdev(xfer->cci_device),
#else
		.cci_subdev     = cam_cci_get_subdev(),
#endif
		.cci_i2c_master = xfer->cci_bus,
		.sid            = xfer->slave_addr,
	};
	struct cam_cci_ctrl cci_ctrl = {
		.cci_info = &cci_info,
	};
	int i;
	struct cam_sensor_i2c_reg_array *reg_conf_tbl;

	pr_debug("%s cmd:%d bus:%d devaddr:%02x regw:%d rega:%04x count:%d\n",
			__func__, cmd, xfer->cci_bus, xfer->slave_addr,
			xfer->reg.width, xfer->reg.addr, xfer->data.count);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,19,0)
	if (xfer->cci_device > 1 || xfer->cci_bus > 1 || xfer->slave_addr > 0x7F ||
#else
	if (xfer->cci_bus > 1 || xfer->slave_addr > 0x7F ||
#endif
			xfer->reg.width < 1 || xfer->reg.width > 2 ||
			xfer->reg.addr > ((1<<(8*xfer->reg.width))-1) ||
			xfer->data.count < 1 ||
			xfer->data.count > MSM_CCI_INTF_MAX_XFER)
		return -EINVAL;

	/* init */
	cci_ctrl.cmd = MSM_CCI_INIT;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,19,0)
	rc = v4l2_subdev_call(cam_cci_get_subdev(xfer->cci_device),
#else
	rc = v4l2_subdev_call(cam_cci_get_subdev(),
#endif
			core, ioctl, VIDIOC_MSM_CCI_CFG, &cci_ctrl);
	if (rc < 0) {
		pr_err("%s: cci init fail (%d)\n", __func__, rc);
		return rc;
	}

	switch (cmd) {
	case MSM_CCI_INTF_READ:
		/* read */
		cci_ctrl.cmd = MSM_CCI_I2C_READ;
		cci_ctrl.cfg.cci_i2c_read_cfg.addr = xfer->reg.addr;
		cci_ctrl.cfg.cci_i2c_read_cfg.addr_type =
			(xfer->reg.width == 1 ?
			 CAMERA_SENSOR_I2C_TYPE_BYTE :
			 CAMERA_SENSOR_I2C_TYPE_WORD);
		cci_ctrl.cfg.cci_i2c_read_cfg.data = xfer->data.buf;
		cci_ctrl.cfg.cci_i2c_read_cfg.num_byte = xfer->data.count;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,19,0)
		rc = v4l2_subdev_call(cam_cci_get_subdev(xfer->cci_device),
#else
		rc = v4l2_subdev_call(cam_cci_get_subdev(),
#endif
			core, ioctl, VIDIOC_MSM_CCI_CFG, &cci_ctrl);
		if (rc < 0) {
			pr_err("%s: cci read fail (%d)\n", __func__, rc);
			goto release;
		}
		rc = cci_ctrl.status;
		break;
	case MSM_CCI_INTF_WRITE:
		/* write */
		reg_conf_tbl = kzalloc(xfer->data.count *
				sizeof(struct cam_sensor_i2c_reg_array),
				GFP_KERNEL);
		if (!reg_conf_tbl) {
			rc = -ENOMEM;
			goto release;
		}
		addr = xfer->reg.addr;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,19,0)
		for (i = 0; i < xfer->data.count; i += xfer->data.width) {
			reg_conf_tbl[i].reg_addr = addr++;
			if(xfer->data.width == 4) {
				reg_conf_tbl[i].reg_data =
					((uint32_t)(xfer->data.buf[i]) << 24) |
					((uint32_t)(xfer->data.buf[i+1]) << 16) |
					((uint32_t)(xfer->data.buf[i+2]) << 8) |
					((uint32_t)(xfer->data.buf[i+3]));
				pr_err("%s: cci writing %x", __func__, reg_conf_tbl[i].reg_data);
			} else if(xfer->data.width == 2) {
				reg_conf_tbl[i].reg_data =
					((uint32_t)(xfer->data.buf[i]) << 8) |
					((uint32_t)(xfer->data.buf[i+1]));
			} else {
				reg_conf_tbl[i].reg_data = xfer->data.buf[i];
			}
			reg_conf_tbl[i].delay = 0;
		}
#else
		for (i = 0; i < xfer->data.count; i += 1) {
			reg_conf_tbl[i].reg_addr = addr++;
			reg_conf_tbl[i].reg_data = xfer->data.buf[i];
			reg_conf_tbl[i].delay = 0;
		}
#endif
		cci_ctrl.cmd = MSM_CCI_I2C_WRITE;
		cci_ctrl.cfg.cci_i2c_write_cfg.reg_setting = reg_conf_tbl;
		cci_ctrl.cfg.cci_i2c_write_cfg.addr_type =
			(xfer->reg.width == 1 ?
				CAMERA_SENSOR_I2C_TYPE_BYTE :
				CAMERA_SENSOR_I2C_TYPE_WORD);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,19,0)
		cci_ctrl.cfg.cci_i2c_write_cfg.data_type = xfer->data.width;
#else
		cci_ctrl.cfg.cci_i2c_write_cfg.data_type =
			CAMERA_SENSOR_I2C_TYPE_BYTE;
#endif
		cci_ctrl.cfg.cci_i2c_write_cfg.size = xfer->data.count;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,19,0)
		rc = v4l2_subdev_call(cam_cci_get_subdev(xfer->cci_device),
#else
		rc = v4l2_subdev_call(cam_cci_get_subdev(),
#endif
				core, ioctl, VIDIOC_MSM_CCI_CFG, &cci_ctrl);
		kfree(reg_conf_tbl);
		if (rc < 0) {
			pr_err("%s: cci write fail (%d)\n", __func__, rc);
			goto release;
		}
		rc = cci_ctrl.status;
		break;
	default:
		pr_err("%s: Unknown command (%d)\n", __func__, cmd);
		rc = -EINVAL;
		break;
	}

release:
	/* release */
	cci_ctrl.cmd = MSM_CCI_RELEASE;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,19,0)
	rc2 = v4l2_subdev_call(cam_cci_get_subdev(xfer->cci_device),
#else
	rc2 = v4l2_subdev_call(cam_cci_get_subdev(),
#endif
			core, ioctl, VIDIOC_MSM_CCI_CFG, &cci_ctrl);
	if (rc2 < 0) {
		pr_err("%s: cci release fail (%d)\n", __func__, rc2);
		return rc2;
	}

	return rc;
}

static long cci_intf_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	struct msm_cci_intf_xfer xfer;
	int rc;

	pr_debug("%s cmd=%x arg=%lx\n", __func__, cmd, arg);

	switch (cmd) {
	case MSM_CCI_INTF_READ:
	case MSM_CCI_INTF_WRITE:
		if (copy_from_user(&xfer, (void __user *)arg, sizeof(xfer)))
			return -EFAULT;
		rc = cci_intf_xfer(&xfer, cmd);
		if (copy_to_user((void __user *)arg, &xfer, sizeof(xfer)))
			return -EFAULT;
		return rc;
	default:
		return -ENOIOCTLCMD;
	}
}

#ifdef CONFIG_COMPAT
static long cci_intf_ioctl_compat(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	pr_debug("%s cmd=%x\n", __func__, cmd);

	switch (cmd) {
	case MSM_CCI_INTF_READ32:
		cmd = MSM_CCI_INTF_READ;
		break;
	case MSM_CCI_INTF_WRITE32:
		cmd = MSM_CCI_INTF_WRITE;
		break;
	default:
		return -ENOIOCTLCMD;
	}
	return cci_intf_ioctl(file, cmd, arg);
}
#endif

static const struct file_operations cci_intf_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = cci_intf_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = cci_intf_ioctl_compat,
#endif
};

static struct miscdevice cci_intf_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "cci_intf",
	.fops = &cci_intf_fops,
};

static int __init cci_intf_init(void)
{
	int rc;

	pr_debug("%s\n", __func__);

	rc = misc_register(&cci_intf_misc);
	if (unlikely(rc)) {
		pr_err("failed to register misc device %s\n", cci_intf_misc.name);
		return rc;
	}

	return 0;
}

static void __exit cci_intf_exit(void)
{
	pr_debug("%s\n", __func__);

	misc_deregister(&cci_intf_misc);
}

module_init(cci_intf_init);
module_exit(cci_intf_exit);
MODULE_DESCRIPTION("CCI DEBUG INTF");
MODULE_LICENSE("GPL v2");
