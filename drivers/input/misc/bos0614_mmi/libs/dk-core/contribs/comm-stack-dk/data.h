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

#ifndef _DATA_H
#define _DATA_H

#include <linux/types.h>

#define DATA_ARRAY_LENGTH(array) (sizeof(array)/sizeof(array[0]))

#ifdef __cplusplus
extern "C" {
#endif

#define NANO_PER_UNITY    (1.0e9)
#define MICRO_PER_UNITY   (1.0e6)
#define MICRO_PER_MILLI   (1.0e3)
#define MILLI_PER_UNITY   (1.0e3)
#define UNITY_PER_KILO    (1.0e3)
#define UNITY_PER_MEGA    (1.0e6)
#define MEGA_PER_GIGA     (1.0e3)

#define MASK_1_BIT      (0x00000001)
#define MASK_2_BIT      (0x00000003)
#define MASK_3_BIT      (0x00000007)
#define MASK_4_BIT      (0x0000000F)
#define MASK_5_BIT      (0x0000001F)
#define MASK_6_BIT      (0x0000003F)
#define MASK_7_BIT      (0x0000007F)
#define MASK_8_BIT      (0x000000FF)
#define MASK_9_BIT      (0x000001FF)
#define MASK_10_BIT     (0x000003FF)
#define MASK_11_BIT     (0x000007FF)
#define MASK_12_BIT     (0x00000FFF)
#define MASK_13_BIT     (0x00001FFF)
#define MASK_14_BIT     (0x00003FFF)
#define MASK_15_BIT     (0x00007FFF)
#define MASK_16_BIT     (0x0000FFFF)
#define MASK_17_BIT     (0x0001FFFF)
#define MASK_18_BIT     (0x0003FFFF)
#define MASK_19_BIT     (0x0007FFFF)
#define MASK_20_BIT     (0x000FFFFF)
#define MASK_21_BIT     (0x001FFFFF)
#define MASK_22_BIT     (0x003FFFFF)
#define MASK_23_BIT     (0x007FFFFF)
#define MASK_24_BIT     (0x00FFFFFF)

#define SHIFT_1_BIT     (0x1)
#define SHIFT_2_BIT     (0x2)
#define SHIFT_4_BIT     (0x4)
#define SHIFT_8_BIT     (0x8)
#define SHIFT_16_BIT    (0x16)

#define BYTE_LENGTH_IN_BITS (8)

#define PI (3.14159265)

void htoBe32(uint32_t value, uint8_t *data);

void htoBe16(uint16_t value, uint8_t *data);

void hToLe16(uint16_t value, uint8_t *data);

uint32_t beToH32(uint8_t *data);

uint16_t beToH16(uint8_t *data);

uint16_t leToH16(uint8_t *data);

int16_t toInt16(int16_t data, uint8_t numBit);

int32_t toInt32(int32_t data, uint8_t numBit);

float be16ToFloat(const uint8_t *data);

void floatToBe16(float value, uint8_t *data);

bool twoComplement(int32_t value, size_t bitLength, uint32_t *result);

#define COMMUNICATION_STACK_1_BYTE (1)
#define COMMUNICATION_STACK_2_BYTES (2)
#define COMMUNICATION_STACK_4_BYTES (4)

uint16_t endian_reverse16(uint16_t val);

uint32_t endian_reverse32(uint32_t val);

uint64_t endian_reverse64(uint64_t val);

void endian_reverse_array(uint8_t* array, uint8_t sizeInBytes);

void endian_reverse_array_enc(uint8_t* dst, uint8_t* array, uint8_t sizeInBytes);

#if BYTE_ORDER == LITTLE_ENDIAN

#define htole16(x) (x)
#define le16toh(x) (x)
#if __GNUC__
#define htobe16(x) __builtin_bswap16(x)
#define be16toh(x) __builtin_bswap16(x)
#else
#define htobe16(x) do{x=endian_reverse16(x);} while (0)
#define be16toh(x) do{x=endian_reverse16(x);} while (0)
#endif //__GNUC__

#define htole32(x) (x)
#define le32toh(x) (x)
#if __GNUC__
#define htobe32(x) __builtin_bswap32(x)
#define be32toh(x) __builtin_bswap32(x)
#else
#define htobe32(x) do{x=endian_reverse32(x);} while (0)
#define be32toh(x) do{x=endian_reverse32(x);} while (0)
#endif //__GNUC__

#define htole64(x) (x)
#define le64toh(x) (x)
#if __GNUC__
#define htobe64(x) __builtin_bswap64(x)
#define be64toh(x) __builtin_bswap64(x)
#else
#define htobe64(x) do{x=endian_reverse64(x);} while (0)
#define be64toh(x) do{x=endian_reverse64(x);} while (0)
#endif //__GNUC__

#define htolevalue(array, size) (array, size)
#define levaluetoh(array, size) (array, size)

#define htolevalue_enc(dst, array, size) (dst, array, size)
#define levaluetoh_enc(dst, array, size) (dst, array, size)

#define htobevalue(array, size) do{endian_reverse_array(array, size;} while (0)
#define bevaluetoh(array, size) do{endian_reverse_array(array, size);} while (0)

#define htobevalue_enc(dst, array, size) do{endian_reverse_array_enc(dst, array, size;} while (0)
#define bevaluetoh_enc(dst, array, size) do{endian_reverse_array_enc(dst, array, size);} while (0)

#elif BYTE_ORDER == BIG_ENDIAN
#define htobe16(x) (x)
#define be16toh(x) (x)
#if __GNUC__
#define htole16(x) __builtin_bswap16(x)
#define le16toh(x) __builtin_bswap16(x)
#else
#define htole16(x) do{x=endian_reverse16(x);} while (0)
#define le16toh(x) do{x=endian_reverse16(x);} while (0)
#endif //__GNUC__

#define htobe32(x) (x)
#define be32toh(x) (x)
#if __GNUC__
#define htole32(x) __builtin_bswap32(x)
#define le32toh(x) __builtin_bswap32(x)
#else
#define htole32(x) do{x=endian_reverse32(x);} while (0)
#define le32toh(x) do{x=endian_reverse32(x);} while (0)
#endif //__GNUC__

#define htobe64(x) (x)
#define be64toh(x) (x)
#if __GNUC__
#define htole64(x) __builtin_bswap64(x)
#define le64toh(x) __builtin_bswap64(x)
#else
#define htole64(x) do{x=endian_reverse64(x);}while (0)
#define le64toh(x) do{x=endian_reverse64(x);}while (0)
#endif //__GNUC__


#define htobevalue(array, size) (array, size)
#define bevaluetoh(array, size) (array, size)

#define htobevalue_enc(dst, array, size) (dst, array, size)
#define bevaluetoh_end(dst, array, size) (dst, array, size)

#define htolevalue(array, size) do{endian_reverse_array(array, size);}while (0)
#define levaluetoh(array, size) do{endian_reverse_array(array, size);}while (0)

#define htolevalue_enc(dst, array, size) do{endian_reverse_array_enc(dst, array, size;} while (0)
#define levaluetoh_enc(dst, array, size) do{endian_reverse_array_enc(dst, array, size);} while (0)


#endif


#ifdef __cplusplus
}
#endif

#endif //_DATA_H
