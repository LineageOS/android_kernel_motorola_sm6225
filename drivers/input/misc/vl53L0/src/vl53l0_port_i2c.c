/*
 * vl53l0_port_i2c.c
 *
 *  Created on: July, 2015
 *      Author:  Teresa Tao
 */

#include <linux/i2c.h>
#include <linux/module.h>
#include "stmvl53l0-i2c.h"
#include "vl53l0_platform.h"
#include "vl53l0_i2c_platform.h"
#include "stmvl53l0.h"

#define I2C_M_WR			0x00
#define STATUS_OK			0x00
#define STATUS_FAIL			(-1)
/** int  VL53L0_I2CWrite(VL53L0_Dev_t dev, void *buff, uint8_t len);
 * @brief       Write data buffer to VL53L0 device via i2c
 * @param dev   The device to write to
 * @param buff  The data buffer
 * @param len   The length of the transaction in byte
 * @return      0 on success
 */
int VL53L0_I2CWrite(VL53L0_DEV dev, uint8_t *buff, uint8_t len)
{
	int err = 0;

	if (dev->bus_type == CCI_BUS) {

	} else {
		struct i2c_msg msg[1];
		struct i2c_data *i2c_client_obj =
			(struct i2c_data *)dev->client_object;
		struct i2c_client *client =
			(struct i2c_client *)i2c_client_obj->client;

		msg[0].addr = client->addr;
		msg[0].flags = I2C_M_WR;
		msg[0].buf = buff;
		msg[0].len = len;

		err = i2c_transfer(client->adapter, msg, 1);
		/* return the actual messages transfer */
		if (err != 1) {
			pr_err("%s: i2c_transfer err:%d, addr:0x%x, reg:0x%x\n",
				__func__, err, client->addr,
				(buff[0] << 8 | buff[1]));
			return STATUS_FAIL;
		}
	}

	return 0;
}


/** int VL53L0_I2CRead(VL53L0_Dev_t dev, void *buff, uint8_t len);
 * @brief       Read data buffer from VL53L0 device via i2c
 * @param dev   The device to read from
 * @param buff  The data buffer to fill
 * @param len   The length of the transaction in byte
 * @return      transaction status
 */
int VL53L0_I2CRead(VL53L0_DEV dev, uint8_t *buff, uint8_t len)
{

	int err = 0;

	if (dev->bus_type == CCI_BUS) {
	} else {
		struct i2c_msg msg[1];
		struct i2c_data *i2c_client_obj =
			(struct i2c_data *)dev->client_object;
		struct i2c_client *client =
			(struct i2c_client *) i2c_client_obj->client;

		msg[0].addr = client->addr;
		msg[0].flags = I2C_M_RD|client->flags;
		msg[0].buf = buff;
		msg[0].len = len;

		err = i2c_transfer(client->adapter, &msg[0], 1);
		if (err != 1) {
			pr_err("%s: Read i2c_transfer err:%d, addr:0x%x\n",
				__func__, err, client->addr);
			return STATUS_FAIL;
		}
	}

	return 0;
}
