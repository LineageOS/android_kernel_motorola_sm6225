/*
 * Copyright (C) 2020 Motorola Mobility LLC
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __LINUX_FINGERPRINT_MMI_H_
#define __LINUX_FINGERPRINT_MMI_H_

#include <linux/notifier.h>

extern int FPS_register_notifier(struct notifier_block *nb, unsigned long stype, bool report);
extern int FPS_unregister_notifier(struct notifier_block *nb, unsigned long stype);

#endif
