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

#include <i2cLinux.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/types.h>


typedef struct
{
    I2c driver;
    struct i2c_client *client;
} Context;

static void initDriverFct(Context *ctx);

#define NBR_CHAR_PER_BYTE (5)
#define ESCAPE_CHAR (1)

static void logBuffer(const char *message, const void *data, size_t length)
{
    size_t bufferLength = length * NBR_CHAR_PER_BYTE + ESCAPE_CHAR;
    char *buf = kzalloc(bufferLength, GFP_KERNEL);

    if (buf != NULL)
    {
        char *ptr = buf;
        char *end = buf + bufferLength;

        uint8_t *_data = (uint8_t *) data;

        for (size_t index = 0; index < length && ptr < end; index++)
        {
            ptr += sprintf(ptr, index < (length - 1) ? "0x%02x " : "0x%02x", _data[index]);
        }

        printk("%s length: %d [%s]\n", message, length, buf, bufferLength, ptr, end);

        kfree(buf);
    }

}


/*
 * Private Section
 */

static int32_t i2cKernelSend(I2c *i2c, uint8_t address, const void *data, size_t num)
{
    int32_t res = -EIO;

    if (i2c != NULL && data != NULL)
    {
        Context *ctx = container_of(i2c, Context, driver);

        dev_dbg(&ctx->client->dev, "I2C Write Address: 0x%x\n", address);

        logBuffer("Write: ", data, num);

        char *buf = kzalloc(num, GFP_KERNEL);

        if (buf != NULL)
        {
            memcpy(buf, data, num);
            int status = i2c_master_send(ctx->client, (const char *) buf, (int) num);
            res = status == num ? ARM_DRIVER_OK : status;

            kfree(buf);
        }
    }

    return res;
}

int32_t i2cKernelRead(I2c *i2c, uint8_t address, void *data, size_t num)
{
    int32_t res = -EIO;

    if (i2c != NULL && data != NULL)
    {
        Context *ctx = container_of(i2c, Context, driver);

        dev_dbg(&ctx->client->dev, "I2C Read Address: 0x%x \n", address);

        char *buf = kzalloc(num, GFP_KERNEL);

        if (buf != NULL)
        {
            int status = i2c_master_recv(ctx->client, (char *) buf, num);

            res = status == num ? 0 : status;

            if (res == ARM_DRIVER_OK)
            {
                memcpy(data, buf, num);
                logBuffer("Read: ", data, num);
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

        printk("[PF-Debug] I2C Client 0x%8x", client);

        if (ctx != NULL)
        {
            ctx->client = client;
            initDriverFct(ctx);
            driver = &ctx->driver;
        }
    }

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