//
// Description: Generic GPIO API
// Created on 1/10/2020
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
#ifndef DKCORE_GPIO_H
#define DKCORE_GPIO_H

#include <linux/types.h>

typedef struct _GPIO Gpio;
typedef struct _GPIO Gpio_t;

typedef enum
{
    GPIOState_High = 0,
    GPIOState_Low,
    GPIOState_Invalid
} GPIOState;

typedef enum
{
    GPIOISR_Falling = 0,
    GPIOISR_Rising,
    GPIOISR_Both,
    GPIOISR_None,
} GPIOIsr;

typedef enum
{
    GPIODir_Input = 0,
    GPIODir_Output,
    GPIODir_Bidirectional
} GPIODir;

typedef void (*GPIOIsrCb)(const Gpio *gpio, void *context, GPIOIsr event);
typedef bool (*GPIOSet)(const Gpio *gpio, GPIOState state);
typedef GPIOState (*GPIOGet)(const Gpio *gpio);
typedef bool (*GPIORegisterISR)(const Gpio *gpio, GPIOIsr isrMode, GPIOIsrCb cb, void *context);
typedef bool (*GPIOUnregisterISR)(const Gpio *gpio);

struct _GPIO
{
    GPIOSet set;
    GPIOGet get;
    GPIORegisterISR registerIsr;
    GPIOUnregisterISR unregisterIsr;
};

#endif //DKCORE_GPIO_H