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

typedef enum
{
    GPIOState_High,
    GPIOState_Low,
    GPIOState_Invalid
} GPIOState;

typedef enum
{
    GPIOISR_Falling,
    GPIOISR_Rising,
    GPIOISR_Both,
    GPIOISR_None,
} GPIOIsr;

typedef enum
{
    GPIODir_Input,
    GPIODir_Output,
    GPIODir_Bidirectional
} GPIODir;

typedef void(*GPIOIsrCb)(Gpio *gpio, void *context, GPIOIsr event);

typedef bool(*GPIOSet)(Gpio *gpio, GPIOState state);

typedef GPIOState(*GPIOGet)(Gpio *gpio);

typedef bool(*GPIORegisterISR)(Gpio *gpio, GPIOIsr isrMode, GPIOIsrCb cb, void *context, bool enable);

typedef bool (*GPIOClearISR)(Gpio *gpio);

typedef bool (*GPIOSetDir)(Gpio *gpio, GPIODir dir);


struct _GPIO
{
    GPIOSet set;
    GPIOGet get;
    GPIORegisterISR registerIsr;
    GPIOClearISR clearIsr;
    GPIOSetDir setDir;
};

#endif //DKCORE_GPIO_H
