//
// Description: I2C Adapter for Boreas Haptic Driver on Linux Kernel
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
#define pr_fmt(fmt) "bos0614: %s: " fmt, __func__

#define DEBUG
//#undef DEBUG

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/types.h>

#include "i2cLinux.h"

#ifdef DEBUG
#undef pr_debug
#define pr_debug pr_err
#undef dev_dbg
#define dev_dbg dev_err
#endif

typedef struct
{
    I2c driver;
    struct i2c_client *client;
} Context;

static void initDriverFct(Context *ctx);

#define NBR_CHAR_PER_BYTE (5)
#define ESCAPE_CHAR (1)

int process_params(ParamsLst *params, int numP, const char *buffer)
{
	char *arg, *buf, *p;
	int n, err;
	unsigned int valueUI;
	int valueI;

	p = buf = kstrdup(buffer, GFP_KERNEL);
	for (n = 0; n < numP && p && *p; n++, params++) {
		valueI = INT16_MIN;
		valueUI = UINT16_MAX;
		arg = strsep(&p, " ");
		if (!arg || !*arg)
			break;

		pr_debug("arg=[%s] rest=[%s]\n", arg, p);

		switch (params->tid) {
		case PARAM_UCHAR8:
			err = kstrtou8(arg, 0, params->data_ptr);
			valueUI = (unsigned int)*(unsigned char *)params->data_ptr;
				break;
		case PARAM_INT16:
			err = kstrtos16(arg, 0, params->data_ptr);
			valueI = (int)*(short *)params->data_ptr;
				break;
		case PARAM_UINT16:
			err = kstrtou16(arg, 0, params->data_ptr);
			valueUI = (unsigned int)*(unsigned short *)params->data_ptr;
				break;
		case PARAM_INT32:
			err = kstrtoint(arg, 0, params->data_ptr);
			valueI = (int)*(int *)params->data_ptr;
				break;
		case PARAM_UINT32:
			err = kstrtouint(arg, 0, params->data_ptr);
			valueUI = *(unsigned int *)params->data_ptr;
				break;
		}

		if (err) {
			n = err;
			break;
		}

		if (valueUI != UINT16_MAX)
			pr_debug("[%d]=%u\n", n, valueUI);
		else if (valueI != INT16_MIN)
			pr_debug("[%d]=%d\n", n, valueI);
	}
	kfree(buf);
	pr_debug("processed %d input parameters\n", n);
	return n;
}

static void logBuffer(struct i2c_client* client, const char* message,
       uint8_t addr, const void* data, size_t length)
{
    size_t bufferLength = length * NBR_CHAR_PER_BYTE + ESCAPE_CHAR;
    char *buf = kzalloc(bufferLength, GFP_KERNEL);

    if (buf != NULL)
    {
        size_t index;
        char *ptr = buf;
        char *end = buf + bufferLength;

        uint8_t *_data = (uint8_t *) data;

        for (index = 0; index < length && ptr < end; index++)
        {
            ptr += sprintf(ptr, index < (length - 1) ? "0x%02x " : "0x%02x", _data[index]);
        }

        dev_dbg(&client->dev, "%s addr: 0x%x length: %zu [%s]\n", message, addr, length, buf);

        kfree(buf);
    }

}

/*
 * Private Section
 */

static int32_t i2cKernelSend(const I2c *i2c, uint8_t address, const void *data, size_t num)
{
    int32_t res = -EIO;

    if (i2c != NULL && data != NULL)
    {
        Context *ctx = container_of(i2c, Context, driver);
        char *buf = kzalloc(num, GFP_KERNEL);

        logBuffer(ctx->client, "Write: ", address, data, num);

        if (buf != NULL)
        {
	    int status;

            memcpy(buf, data, num);
            status = i2c_master_send(ctx->client, (const char *) buf, (int) num);
            res = status == num ? ARM_DRIVER_OK : status;

            kfree(buf);
        }
    }

    return res;
}

int32_t i2cKernelRead(const I2c *i2c, uint8_t address, void *data, size_t num)
{
    int32_t res = -EIO;

    if (i2c != NULL && data != NULL)
    {
        Context *ctx = container_of(i2c, Context, driver);
        char *buf = kzalloc(num, GFP_KERNEL);

        if (buf != NULL)
        {
            int status = i2c_master_recv(ctx->client, (char *) buf, num);

            res = status == num ? 0 : status;

            if (res == ARM_DRIVER_OK)
            {
                memcpy(data, buf, num);
                logBuffer(ctx->client, "Read: ", address, data, num);
            }

            kfree(buf);
        }


    }

    return res;
}

/*
 * Public Section
 */

I2c *i2cBoreasLinuxInit(struct i2c_client *client)
{
    I2c *driver = NULL;

    if (client != NULL)
    {
        Context *ctx = kzalloc(sizeof(Context), GFP_KERNEL);

        if (ctx != NULL)
        {
            ctx->client = client;
            initDriverFct(ctx);
            driver = &ctx->driver;
#if 0
            if (ctx->client->dev.of_node)
               readDevTree(ctx);
#endif
        }
    }

    pr_debug("I2C client: %p, driver: %p\n", client, driver);

    return driver;
}

bool i2cBoreasLinuxFree(I2c *i2c)
{
    bool res = false;

    if (i2c != NULL)
    {
        Context *ctx = container_of(i2c, Context, driver);

        dev_info(&ctx->client->dev, "I2C Linux Wrapper Free\n");

        kfree(ctx);

        res = true;
    }

    return res;
}

/*
 * Private Section
 */

void initDriverFct(Context *ctx)
{
    ctx->driver.write = i2cKernelSend;
    ctx->driver.read = i2cKernelRead;
}
