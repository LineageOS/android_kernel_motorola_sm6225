//
// Description: BOS0614 Driver Instantiation
// Created on 5/5/2020
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

#ifndef DKCORE_BOS0614DRIVER_H
#define DKCORE_BOS0614DRIVER_H

#include "coreDef.h"
#include "bsp/drivers/haptic/bosDriver.h"
#include "bsp/drivers/i2c/i2c.h"

#define BOS0614_NBR_OF_CHANNEL  (4)
#define BOS0614_CHANNEL_MASK (0xF)

typedef struct
{
    I2c *i2c;
    Gpio *gpioA;
    Gpio *gpioB;
    Gpio *gpioC;
    Gpio *gpioD;
} Bos0614Resource;

HapticDriver *bos0614DriverI2cInit(Bos0614Resource resources);

HapticDriver *bos0614DriverSpiInit(Spi *spi, Gpio *gpioA, Gpio *gpioD);

Bos0614RegisterStruct *getAllRegsPtr(void);

#endif //DKCORE_BOS0614DRIVER_H
