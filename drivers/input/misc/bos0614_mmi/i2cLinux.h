//
// Description: Boréas I2C Wrapper for Linux Implementation
//
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
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#ifndef BOREAS_HAPTIC_DRIVER_LINUX_I2CLINUX_H
#define BOREAS_HAPTIC_DRIVER_LINUX_I2CLINUX_H

#include <linux/device.h>
#include <linux/i2c.h>
#include "libs/dk-core/src/bsp/drivers/i2c/i2c.h"
#include "libs/dk-core/src/bsp/drivers/haptic/bos0614Driver.h"

enum {
	PARAM_UCHAR8 = 1,
	PARAM_INT16,
	PARAM_UINT16,
	PARAM_INT32,
	PARAM_UINT32
};

typedef struct {
	int tid;
	void *data_ptr;
} ParamsLst;

#define PARAM_ADD(t, v) {\
	params[numP].tid = t;\
	params[numP].data_ptr = v;\
	numP++;\
}

int process_params(ParamsLst *params, int numP, const char *buffer);

I2c *i2cBoreasLinuxInit(struct i2c_client *client);

bool i2cBoreasLinuxFree(I2c * i2c);

#endif //BOREAS_HAPTIC_DRIVER_LINUX_I2CLINUX_H
