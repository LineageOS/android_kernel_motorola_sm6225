/**************************************************************************
 * Copyright (c) 2016, STMicroelectronics - All Rights Reserved

 License terms: BSD 3-clause "New" or "Revised" License.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this
 list of conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice,
 this list of conditions and the following disclaimer in the documentation
 and/or other materials provided with the distribution.

 3. Neither the name of the copyright holder nor the names of its contributors
 may be used to endorse or promote products derived from this software
 without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ****************************************************************************/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/spi/spi.h>
#include <linux/miscdevice.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/time.h>
#include <linux/platform_device.h>
#include <linux/version.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>

#include "stmvl53l5_i2c.h"
#include "stmvl53l5_spi.h"
#include "stmvl53l5_load_fw.h"

#define STMVL53L5_DRV_NAME		"stmvl53l5"
#define STMVL53L5_SLAVE_ADDR		0x29

struct stmvl53l5_comms_struct {
	__u16   len;
	__u16   reg_index;
	__u64   buf;
	__u8    write_not_read;
};

#define ST_TOF_IOCTL_TRANSFER		_IOWR('a',0x1, struct stmvl53l5_comms_struct)

static struct miscdevice st_tof_miscdev;
static uint8_t * raw_data_buffer = NULL;

static uint8_t i2c_not_spi = 1;

static uint8_t i2c_driver_added = 0;
static uint8_t spi_driver_registered = 0;
static uint8_t misc_registered = 0;

static struct spi_data_t spi_data;

static struct regulator *vl53l5_avdd = NULL;
static struct regulator *vl53l5_iovdd = NULL;
struct stmvl53l5_pinctrl_info {
	struct pinctrl *pinctrl;
	struct pinctrl_state *gpio_state_active;
	struct pinctrl_state *gpio_state_suspend;
} vl53l5_pinctrl_info = {NULL, NULL, NULL};

// ------- i2c ---------------------------
static const struct i2c_device_id stmvl53l5_i2c_id[] = {
	{STMVL53L5_DRV_NAME, 0},
	{},
};

// ------- spi ---------------------------
static const struct spi_device_id stmvl53l5_spi_id[] = {
	{STMVL53L5_DRV_NAME, 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, stmvl53l5_i2c_id);

static const struct of_device_id st_tof_of_match[] = {
	{
		/* An older compatible */
		.compatible = "st,stmvl53l5",
		.data = STMVL53L5_DRV_NAME,
	},
	{},
};

MODULE_DEVICE_TABLE(of, st_tof_of_match);  // add to the kernel device tree table

static struct i2c_client *stmvl53l5_i2c_client = NULL;

static int stmvl53l5_open(struct inode *inode, struct file *file)
{
	pr_debug("stmvl53l5 : %s(%d)\n", __func__, __LINE__);
	return 0;
}

static int stmvl53l5_release(struct inode *inode, struct file *file)
{
	pr_debug("stmvl53l5 : %s(%d)\n", __func__, __LINE__);
	return 0;
}

static int stmvl53l5_ioctl_handler(struct file *file,
		unsigned int cmd, unsigned long arg, void __user *p)
{
	struct i2c_msg st_i2c_message;
	struct stmvl53l5_comms_struct comms_struct;
	int ret = 0;
	uint16_t index, transfer_size, chunk_size;
	u8 __user *data_ptr = NULL;
	pr_debug("stmvl53l5_ioctl : cmd = %u\n", cmd);

	switch (cmd) {
		case ST_TOF_IOCTL_TRANSFER:

			ret = copy_from_user(&comms_struct, (struct stmvl53l5_comms_struct *)p, sizeof(comms_struct));
			if (ret) {
				pr_err("Error at %s(%d)\n", __func__, __LINE__);
				return -EINVAL;
			}
			data_ptr = (u8 __user *)(comms_struct.buf);
			// printk("Transfer. write_not_read = %d, reg_index = 0x%x size = %d\n", comms_struct.write_not_read, comms_struct.reg_index, comms_struct.len);

			if (i2c_not_spi) {
				// address and buis the same whatever the transfers to be done !
				st_i2c_message.addr = 0x29;
				// st_i2c_message.buf is the same whatever the transfers to be done
				st_i2c_message.buf = raw_data_buffer;
			}

			// in case of i2c write, it is a single transfer with read index set in the 2 first bytes
			// the other case use fully the raw data buffer for raw data transfers
			if ((i2c_not_spi) && (comms_struct.write_not_read))
				chunk_size = VL53L5_COMMS_CHUNK_SIZE - 2;
			else
				chunk_size = VL53L5_COMMS_CHUNK_SIZE;

			// index is the number of bytes already transfered
			index = 0;

			do {
				// take the max number of bytes that can be transfered
				transfer_size = (comms_struct.len - index) > chunk_size ?  chunk_size : (comms_struct.len - index);

				// ----- WRITE
				if (comms_struct.write_not_read) {
					// ---- i2c
					if (i2c_not_spi) {
						// put red index at the beginning of the buffer
						raw_data_buffer[0] = (uint8_t)(((comms_struct.reg_index + index) & 0xFF00) >> 8);
						raw_data_buffer[1] = (uint8_t)((comms_struct.reg_index + index) & 0x00FF);

						ret = copy_from_user(&raw_data_buffer[2], data_ptr + index, transfer_size);
						if (ret) {
							pr_err("Error at %s(%d)\n", __func__, __LINE__);
							return -EINVAL;
						}

						st_i2c_message.len = transfer_size + 2;
						st_i2c_message.flags = 0;
						st_i2c_message.buf = raw_data_buffer;
						ret = i2c_transfer(stmvl53l5_i2c_client->adapter, &st_i2c_message, 1);
						if (ret != 1) {
							pr_err("Error %d at %s(%d)\n",ret,  __func__, __LINE__);
							return -EIO;
						}
					}
					// ---- spi
					else {
						ret = copy_from_user(raw_data_buffer, data_ptr + index, transfer_size);
						if (ret) {
							pr_err("stmvl53l5: Error at %s(%d)\n", __func__, __LINE__);
							return -EINVAL;
						}

						ret = stmvl53l5_spi_write(&spi_data, comms_struct.reg_index + index, raw_data_buffer, transfer_size);
						if (ret) {
							pr_err("Error %d at %s(%d)\n",ret,  __func__, __LINE__);
							return -EIO;
						}
					}
				}
				// ----- READ
				else {
					// ---- i2c
					if (i2c_not_spi) {
						// write reg_index
						st_i2c_message.len = 2;
						st_i2c_message.flags = 0;
						st_i2c_message.buf = raw_data_buffer;
						raw_data_buffer[0] = (uint8_t)(((comms_struct.reg_index + index) & 0xFF00) >> 8);
						raw_data_buffer[1] = (uint8_t)((comms_struct.reg_index + index) & 0x00FF);

						ret = i2c_transfer(stmvl53l5_i2c_client->adapter, &st_i2c_message, 1);
						if (ret != 1) {
							pr_err("Error at %s(%d)\n", __func__, __LINE__);
							return -EIO;
						}

						st_i2c_message.len = transfer_size;
						st_i2c_message.flags = 1;
						st_i2c_message.buf = raw_data_buffer;
						ret = i2c_transfer(stmvl53l5_i2c_client->adapter, &st_i2c_message, 1);
						if (ret != 1) {
							pr_err("Error at %s(%d)\n", __func__, __LINE__);
							return -EIO;
						}
					}
					// ---- spi
					else {
						ret = stmvl53l5_spi_read(&spi_data, comms_struct.reg_index + index, raw_data_buffer, transfer_size);
						if (ret) {
							pr_err("stmvl53l5: Error at %s(%d)\n", __func__, __LINE__);
							return -EIO;
						}
					}

					// copy to user buffer the read transfer
					ret = copy_to_user(data_ptr + index, raw_data_buffer, transfer_size);
					if (ret) {
						pr_err("Error at %s(%d)\n", __func__, __LINE__);
						return -EINVAL;
					}

				} // ----- READ

				index += transfer_size;

			} while (index < comms_struct.len);
			break;

		default:
			return -EINVAL;

	}
	return ret;
}

#ifdef CONFIG_COMPAT
static long stmvl53l5_compat_ioctl(struct file *file,
        unsigned int cmd, unsigned long arg)
{
	int ret;
	ret = stmvl53l5_ioctl_handler(file, cmd, arg, compat_ptr(arg));
	return ret;
}
#endif

static long stmvl53l5_ioctl(struct file *file,
		unsigned int cmd, unsigned long arg)
{
	int ret;
	ret = stmvl53l5_ioctl_handler(file, cmd, arg, (void __user *)arg);
	return ret;
}

static const struct file_operations stmvl53l5_ranging_fops = {
	.owner			= THIS_MODULE,
	.unlocked_ioctl		= stmvl53l5_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl		= stmvl53l5_compat_ioctl,
#endif
	.open			= stmvl53l5_open,
	.release		= stmvl53l5_release,
};

static int stmvl53l5_init(struct device *dev)
{
	int ret = 0;

	vl53l5_pinctrl_info.pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR_OR_NULL(vl53l5_pinctrl_info.pinctrl)) {
		vl53l5_pinctrl_info.gpio_state_active =
			pinctrl_lookup_state(vl53l5_pinctrl_info.pinctrl, "laser_default");
		vl53l5_pinctrl_info.gpio_state_suspend =
			pinctrl_lookup_state(vl53l5_pinctrl_info.pinctrl, "laser_suspend");
		pinctrl_select_state(vl53l5_pinctrl_info.pinctrl, vl53l5_pinctrl_info.gpio_state_active);
	} else {
		dev_err(dev, "stmvl53l5: Error. Getting pinctrl handle failed\n");
	}

	if (vl53l5_avdd || vl53l5_iovdd) {
		dev_warn(dev, "stmvl53l5: Power regulator already on\n");
		return 0;
	}

	vl53l5_avdd = devm_regulator_get_optional(dev, "avdd-vl53l5");
	if (IS_ERR(vl53l5_avdd) || vl53l5_avdd == NULL) {
		dev_err(dev, "stmvl53l5: Error. Could not get avdd-vl53l5 power regulator\n");
		ret = -EINVAL;
		goto avdd_fail;
	}

	vl53l5_iovdd = devm_regulator_get_optional(dev, "iovdd-vl53l5");
	if (IS_ERR(vl53l5_iovdd) || vl53l5_iovdd == NULL) {
		dev_err(dev, "stmvl53l5: Error. Could not get iovdd-vl53l5 power regulator\n");
		ret = -EINVAL;
		goto iovdd_fail;
	}

	ret = regulator_enable(vl53l5_avdd);
	if (ret) {
		dev_err(dev, "stmvl53l5: Error. Could not enable avdd-vl53l5 power regulator\n");
		goto avdd_en_fail;
	}
	ret = regulator_enable(vl53l5_iovdd);
	if (ret) {
		dev_err(dev, "stmvl53l5: Error. Could not enable iovdd-vl53l5 power regulator\n");
		goto iovdd_en_fail;
	}

	return 0;

iovdd_en_fail:
	regulator_disable(vl53l5_avdd);
avdd_en_fail:
	devm_regulator_put(vl53l5_iovdd);
iovdd_fail:
	devm_regulator_put(vl53l5_avdd);
avdd_fail:
	vl53l5_avdd = NULL;
	vl53l5_iovdd = NULL;

	return ret;
}

static int stmvl53l5_deinit(struct device *dev)
{
	int ret = 0;

	if (!vl53l5_avdd || !vl53l5_iovdd) {
		dev_err(dev, "stmvl53l5: Error. Power regulator already off\n");
		return 0;
	}

	ret = regulator_disable(vl53l5_avdd);
	if (ret) {
		dev_err(dev, "stmvl53l5: Error. Could not disable avdd-vl53l5 power regulator\n");
	}
	ret = regulator_disable(vl53l5_iovdd);
	if (ret) {
		dev_err(dev, "stmvl53l5: Error. Could not disable iovdd-vl53l5 power regulator\n");
	}

	devm_regulator_put(vl53l5_iovdd);
	devm_regulator_put(vl53l5_avdd);
	vl53l5_avdd = NULL;
	vl53l5_iovdd = NULL;

	if (!IS_ERR_OR_NULL(vl53l5_pinctrl_info.pinctrl))
		pinctrl_select_state(vl53l5_pinctrl_info.pinctrl, vl53l5_pinctrl_info.gpio_state_suspend);

	return ret;
}

static int stmvl53l5_i2c_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int ret;
	uint8_t page = 0, revision_id = 0, device_id = 0;
	stmvl53l5_i2c_client = client;

	printk("stmvl53l5: probing i2c\n");

	ret = stmvl53l5_init(&client->dev);
	if (ret) {
		pr_err("stmvl53l5: Error. Power up failed, ret = %d\n", ret);
		return ret;
	}

	i2c_not_spi = 1;

	raw_data_buffer = kzalloc(VL53L5_COMMS_CHUNK_SIZE, GFP_DMA | GFP_KERNEL);
	if (raw_data_buffer == NULL) {
		 ret = -ENOMEM;
		goto exit;
	}

	ret = stmvl53l5_write_multi(client, raw_data_buffer, 0x7FFF, &page, 1);
	ret |= stmvl53l5_read_multi(client, raw_data_buffer, 0x00, &device_id, 1);
	ret |= stmvl53l5_read_multi(client, raw_data_buffer, 0x01, &revision_id, 1);

	if ((device_id != 0xF0) || (revision_id != 0x02)) {
		pr_err("stmvl53l5: Error. Could not read device and revision id registers\n");
		ret = ret? ret : -ENXIO;
		goto exit;
	}
	printk("stmvl53l5: device_id : 0x%x. revision_id : 0x%x\n", device_id, revision_id);

	st_tof_miscdev.minor = MISC_DYNAMIC_MINOR;
	st_tof_miscdev.name = "stmvl53l5";
	st_tof_miscdev.fops = &stmvl53l5_ranging_fops;
	st_tof_miscdev.mode = 0444;

	ret = misc_register(&st_tof_miscdev);
	if (ret) {
		pr_err("stmvl53l5 : Failed to create misc device, err = %d\n", ret);
		goto exit;
	}

	misc_registered = 1;

	ret = stmvl53l5_load_fw_stm(client, raw_data_buffer);
	if (ret) {
		pr_err("stmvl53l5 : Failed in loading the FW into the device, err = %d\n", ret);
	}

	ret = stmvl53l5_move_device_to_low_power(client, NULL, 1, raw_data_buffer);
	if (ret) {
		pr_err("stmvl53l5 : could not move the device to low power = %d\n", ret);
	}

	if (ret)
		goto exit;

	return 0;

exit:
	if (raw_data_buffer) {
		kfree(raw_data_buffer);
		raw_data_buffer = NULL;
	}
	stmvl53l5_deinit(&client->dev);

	return ret;
}

static int stmvl53l5_i2c_remove(struct i2c_client *client)
{

	if (raw_data_buffer)
		kfree(raw_data_buffer);

	stmvl53l5_deinit(&client->dev);

	return 0;
}

static struct i2c_driver stmvl53l5_i2c_driver = {
	.driver = {
		.name = STMVL53L5_DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(st_tof_of_match), // for platform register to pick up the dts info
	},
	.probe = stmvl53l5_i2c_probe,
	.remove = stmvl53l5_i2c_remove,
	.id_table = stmvl53l5_i2c_id,
};


static int stmvl53l5_spi_probe(struct spi_device *spi)
{
	int ret;
	uint8_t page = 0, revision_id = 0, device_id = 0;

	printk("stmvl53l5: probing spi\n");

	ret = stmvl53l5_init(&spi->dev);
	if (ret) {
		pr_err("stmvl53l5: Error. Power up failed, ret = %d\n", ret);
		return ret;
	}

	i2c_not_spi = 0;

	spi_data.device = spi;
	spi_data.device->mode |= SPI_CPHA;
	spi_data.device->mode |= SPI_CPOL;

	ret = stmvl53l5_spi_write(&spi_data, 0x7FFF, &page, 1);
	ret |= stmvl53l5_spi_read(&spi_data, 0x00, &device_id, 1);
	ret |= stmvl53l5_spi_read(&spi_data, 0x01, &revision_id, 1);

	if ((device_id != 0xF0) || (revision_id != 0x02)) {
		pr_err("stmvl53l5: Error. Could not read device and revision id registers\n");
		ret = ret? ret : -ENXIO;
		goto exit;
	}
	printk("stmvl53l5: device_id : 0x%x. revision_id : 0x%x\n", device_id, revision_id);

	raw_data_buffer = kzalloc(VL53L5_COMMS_CHUNK_SIZE, GFP_DMA | GFP_KERNEL);
	if (raw_data_buffer == NULL) {
		ret = -ENOMEM;
		goto exit;
	}

	st_tof_miscdev.minor = MISC_DYNAMIC_MINOR;
	st_tof_miscdev.name = "stmvl53l5";
	st_tof_miscdev.fops = &stmvl53l5_ranging_fops;
	st_tof_miscdev.mode = 0444;

	ret = misc_register(&st_tof_miscdev);
	if (ret) {
		pr_err("stmvl53l5 : Failed to create misc device, err = %d\n", ret);
		ret = -1;
		goto exit;
	}

	misc_registered = 1;

	return 0;

exit:
	if (raw_data_buffer) {
		kfree(raw_data_buffer);
		raw_data_buffer = NULL;
	}
	stmvl53l5_deinit(&spi->dev);

	return ret;
}

static int stmvl53l5_spi_remove(struct spi_device *device)
{

	if (raw_data_buffer)
		kfree(raw_data_buffer);

	stmvl53l5_deinit(&device->dev);

	return 0;
}


static struct spi_driver stmvl53l5_spi_driver = {
	.driver = {
		.name   = STMVL53L5_DRV_NAME,
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(st_tof_of_match), // for platform register to pick up the dts info
	},
	.probe  = stmvl53l5_spi_probe,
	.remove = stmvl53l5_spi_remove,
	.id_table = stmvl53l5_spi_id,
};


static int __init st_tof_module_init(void)
{
	int ret = 0;

	printk("stmvl53l5: module init\n");

	/* register as a i2c client device */
	ret = i2c_add_driver(&stmvl53l5_i2c_driver);

	if (ret) {
		i2c_del_driver(&stmvl53l5_i2c_driver);
		printk("stmvl53l5: could not add i2c driver\n");
		return ret;
	}

	i2c_driver_added = 1;

	ret = spi_register_driver(&stmvl53l5_spi_driver);
	if (ret) {
		printk("stmvl53l5: could not register spi driver : %d", ret);
		return ret;
	}

	spi_driver_registered = 1;

	return ret;
}

static void __exit st_tof_module_exit(void)
{

	pr_debug("stmvl53l5 : module exit\n");

	if (misc_registered) {
		misc_deregister(&st_tof_miscdev);
		misc_registered = 0;
	}

	if (spi_driver_registered) {
		spi_unregister_driver(&stmvl53l5_spi_driver);
		spi_driver_registered = 0;
	}

	if (i2c_driver_added) {
		i2c_del_driver(&stmvl53l5_i2c_driver);
		i2c_driver_added = 0;
	}

}

module_init(st_tof_module_init);
module_exit(st_tof_module_exit);
MODULE_LICENSE("GPL");





