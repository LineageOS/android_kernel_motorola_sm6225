/* Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _MMI_KERNEL_COMMON_H
#define _MMI_KERNEL_COMMON_H

#include <linux/list.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/time.h>

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0))
#define TIME_TYPE timespec64
#define GET_TIME_OF_DAY(_timeval) \
	ktime_get_real_ts64(_timeval);
#else
#define TIME_TYPE timeval
#define GET_TIME_OF_DAY(_timeval) \
	do_gettimeofday(_timeval);

#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0))
#define SIGINFO kernel_siginfo
#else
#define SIGINFO siginfo
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
#define TIME_SPEC timespec64
#else
#define TIME_SPEC timespec
#endif

#endif    /*_MMI_KERNEL_COMMON_H*/
