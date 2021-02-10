//
// Description: I2C Driver Interface
// Created on 1/23/2020
// Copyright (c) 2020 Boreas Technologies All rights reserved.
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#ifndef DKCORE_I2C_H
#define DKCORE_I2C_H

#include <linux/types.h>

#include "contribs/cmsis/CMSIS/Driver/Include/Driver_Common.h"

#define I2C_7BIT_ADDRESS_MASK (0x7F)

typedef enum
{
    I2C_ADR_PIN_GND,
    I2C_ADR_PIN_VCC,
    I2C_ADR_PIN_SDA,
    I2C_ADR_PIN_SCL,
    I2C_ADR_PIN_INVALID
} I2cAdrPins;

/**
 * @typedef I2c driver Instance
 */
typedef struct _i2c I2c;

/**
 * @brief I2C Master Read Primitive
 *
 * @param i2c       Driver instance
 * @param address   I2C slave address
 * @param data      Buffer for read data
 * @param num       Length to read
 *
 * @return ARM_DRIVER_OK on successful operation, ERRNO on failure
 */
typedef int32_t(*I2cRead)(I2c *i2c, uint8_t address, void *data, size_t num);

/**
 * @brief 2C Master Read Primitive with stop configuration
 *
 * @param i2c       Driver instance
 * @param address   I2C slave address
 * @param data      Buffer for read data
 * @param num       Length to read
 * @param writeStop Set to true for writing stop at the end of the transaction
 *
 * @return ARM_DRIVER_OK on successful operation, ERRNO on failure
 */
typedef int32_t(*I2cReadStop)(I2c *i2c, uint8_t address, void *data, size_t num, bool writeStop);

/**
 * @brief I2C Master Write Primitive
 *
 * @param i2c       Driver instance
 * @param address   I2C slave address
 * @param data      Data buffer to write
 * @param num       Length of the buffer to write
 *
 * @return ARM_DRIVER_OK on successful operation, ERRNO on failure
 */
typedef int32_t(*I2cWrite)(I2c *i2c, uint8_t address, const void *data, size_t num);

/**
 * @brief I2C Master Write Primitive  with stop configuration
 *
 * @param i2c       Driver instance
 * @param address   I2C slave address
 * @param data      Data buffer to write
 * @param num       Length of the buffer to write
 * @param writeStop Set to true for writing stop at the end of the transaction
 *
 * @return ARM_DRIVER_OK on successful operation, ERRNO on failure
 */
typedef int32_t(*I2cWriteStop)(I2c *i2c, uint8_t address, const void *data, size_t num, bool writeStop);

struct _i2c
{
    I2cWrite write;
    I2cWriteStop writeStop;
    
    I2cRead read;
    I2cReadStop readStop;
};

#endif //DKCORE_I2C_H
