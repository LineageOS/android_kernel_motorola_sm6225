/*
 * Copyright (c) 2019, The Linux Foundation. All rights reserved.
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
#ifndef _PEN_DETECTION_DRM_NOTIFY_H_
#define _PEN_DETECTION_DRM_NOTIFY_H_

#include <linux/notifier.h>

/* The event of pen insertion */
#define PEN_DETECTION_INSERT		0x01
/* The event of pen pull-out */
#define PEN_DETECTION_PULL		0x02

int pen_detection_register_client(struct notifier_block *nb);
int pen_detection_unregister_client(struct notifier_block *nb);
#endif
