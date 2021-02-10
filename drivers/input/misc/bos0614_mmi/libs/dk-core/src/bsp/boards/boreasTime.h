//
// Description: Generic Time API
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

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#ifndef DKCORE_LAUNCHPADTIME_H
#define DKCORE_LAUNCHPADTIME_H

#include <linux/types.h>

#define MICROSECOND_IN_MILLISECOND(_us) (_us/1000)

#define SECOND_IN_MILLISECOND(_s) (_s*1000)

#define SECOND_IN_MICROSECOND(_s) (_s*1000000)

typedef uint32_t TimeStamp;

/*
 * @brief Initiate the time service
 *
 * @return void
 */
bool timeInit(void);

/*
 * @brief Wait specific amount time in microsecond.
 *
 * @param[in] timeUs The time in microsecond to wait for
 *
 * @return void
 */
void timeWaitUs(uint32_t timeUs);

/**
 * @brief Get the time since platform has been initiated.
 *
 * @return uint32_t The time in millisecond
 */
TimeStamp timeGet(void);


#endif //DKCORE_LAUNCHPADTIME_H
