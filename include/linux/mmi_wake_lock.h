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

#ifndef _MMI_WAKE_LOCK_H
#define _MMI_WAKE_LOCK_H

#include <linux/errno.h>
#include <linux/version.h>
#include <linux/pm_wakeup.h>
#include <linux/slab.h>

#define PM_STAY_AWAKE(_wakeup_source) __pm_stay_awake(_wakeup_source);
#define PM_RELAX(_wakeup_source) __pm_relax(_wakeup_source);
#define PM_WAKEUP_EVENT(_wakeup_source, _time) \
	__pm_wakeup_event(_wakeup_source, _time);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 110) || \
    (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 163) && LINUX_VERSION_CODE < KERNEL_VERSION(4, 19, 0)))
#define PM_WAKEUP_REGISTER(_dev, _wakeup_source, _name) \
        _wakeup_source = wakeup_source_register(_dev, _name);
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 81)
#define PM_WAKEUP_REGISTER(_dev, _wakeup_source, _name) \
        _wakeup_source = wakeup_source_register(_name);
#else
#define PM_WAKEUP_REGISTER(_dev, _wakeup_source, _name) { \
        _wakeup_source = (struct wakeup_source *)kmalloc(sizeof(struct wakeup_source), GFP_KERNEL); \
        if(!_wakeup_source) return -ENOMEM; \
        wakeup_source_init(_wakeup_source, _name); \
}
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 81)
#define PM_WAKEUP_UNREGISTER(_wakeup_source) \
        wakeup_source_unregister(_wakeup_source);
#else
#define PM_WAKEUP_UNREGISTER(_wakeup_source) { \
        wakeup_source_trash(_wakeup_source); \
        kfree(_wakeup_source); \
}
#endif

#endif // _MMI_WAKE_LOCK_H
