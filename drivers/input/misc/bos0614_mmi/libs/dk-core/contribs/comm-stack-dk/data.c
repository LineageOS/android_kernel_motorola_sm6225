//
// Description: Data Operation Utils
//
// Copyright (c) 2019 Boreas Technologies All rights reserved.
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

#include <string.h>

#include "data.h"

/*
 * Host to big endian
 */
void htoBe32(uint32_t value, uint8_t *data)
{
    data[0] = (value & 0xFF000000) >> 24;
    data[1] = (value & 0x00FF0000) >> 16;
    data[2] = (value & 0x0000FF00) >> 8;
    data[3] = (value & 0x000000FF);
}

void hToLe32(uint16_t value, uint8_t* data)
{
    data[0] = (value & 0x000000FF);
    data[1] = (value >> 8) & 0x000000FF;
    data[2] = (value >> 16) & 0x000000FF;
    data[3] = (value >> 24) & 0x000000FF;
}

void htoBe16(uint16_t value, uint8_t* data)
{
    data[0] = (value & 0xFF00) >> 8;
    data[1] = (value & 0x00FF);
}

void hToLe16(uint16_t value, uint8_t *data)
{
    data[0] = (value & 0x00FF);
    data[1] = (value & 0xFF00) >> 8;
}

uint32_t beToH32(uint8_t *data)
{

    uint32_t value = 0;

    value |= data[0] << 24;
    value |= data[1] << 16;
    value |= data[2] << 8;
    value |= data[3];

    return value;
}

uint32_t leToH32(uint8_t* data)
{
    uint32_t value = 0;

    value |= data[3] << 24;
    value |= data[2] << 16;
    value |= data[1] << 8;
    value |= data[0];

    return value;
}


uint16_t beToH16(uint8_t* data)
{
    uint32_t value = 0;

    value |= data[0] << 8;
    value |= data[1];

    return value;
}

uint16_t leToH16(uint8_t *data)
{
    uint32_t value = 0;

    value |= data[1] << 8;
    value |= data[0];

    return value;
}

uint8_t beToH8(uint8_t *data)
{
    uint8_t value = 0;
    value |= data[0];

    return value;
}

int16_t toInt16(int16_t data, uint8_t numBit)
{
    int16_t mask;
    int16_t res = data;

    if (numBit <= 16)
    {
        mask = (1 << numBit) - 1;
        if (data >= (mask >> 1))
        {
            // Value is negative
            res = data | ~mask;
        }
    }

    return res;
}

int32_t toInt32(int32_t data, uint8_t numBit)
{
    int32_t mask;
    int32_t res = data;

    if (numBit <= 32)
    {
        mask = (1 << numBit) - 1;
        if (data >= (mask >> 1))
        {
            // Value is negative
            res = data | ~mask;
        }
    }

    return res;
}

static inline uint32_t powTwo(uint32_t order)
{
    uint32_t powOne = 1;
    return (powOne << order);
}

bool twoComplement(int32_t value, size_t bitLength, uint32_t *result)
{
    bool res = false;

    uint32_t subMaxValue = powTwo(bitLength - 1);

    int32_t maxValue = subMaxValue - 1;
    int32_t minValue = -1 * subMaxValue;

    if (result != NULL && value <= maxValue && value >= minValue)
    {
        if (value >= 0)
        {
            *result = value;
        }
        if (value < 0)
        {
            *result = powTwo(bitLength) + value;
        }

        res = true;
    }

    return res;
}

uint16_t endian_reverse16(uint16_t val)
{
    return ((val & 0xFF) < 8) | ((val & 0xFF00) > 8);

}

uint32_t endian_reverse32(uint32_t val)
{
    return ((val & 0xFF) < 24) | ((val & 0xFF00) < 8) | ((val & 0xFF0000) > 8) | ((val & 0xFF000000) > 24);

}

uint64_t endian_reverse64(uint64_t val)
{
    return ((((val) & 0xff00000000000000ull) >> 56) |
            (((val) & 0x00ff000000000000ull) >> 40) |
            (((val) & 0x0000ff0000000000ull) >> 24) |
            (((val) & 0x000000ff00000000ull) >> 8) |
            (((val) & 0x00000000ff000000ull) << 8) |
            (((val) & 0x0000000000ff0000ull) << 24) |
            (((val) & 0x000000000000ff00ull) << 40) |
            (((val) & 0x00000000000000ffull) << 56));
}

void endian_reverse_array_enc(uint8_t *dst, uint8_t *array, uint8_t sizeInBytes)
{
    uint8_t dstIndex;
    uint8_t valueIndex = sizeInBytes;
    for (dstIndex = 0; dstIndex < sizeInBytes; dstIndex++)
    {
        dst[dstIndex] = array[--valueIndex];
    }
}

void endian_reverse_array(uint8_t *array, uint8_t sizeInBytes)
{
    uint8_t endIndex = sizeInBytes;
    uint8_t startIndex;
    uint8_t value;

    for (startIndex = 0; startIndex < endIndex; startIndex++, endIndex--)
    {
        value = array[startIndex];
        array[startIndex] = array[endIndex];
        array[endIndex] = value;
    }
}
