/*
 * Synaptics DSX touchscreen driver MMI class extention
 *
 * Copyright (C) 2019 Motorola Mobility
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#define pr_fmt(fmt) "%s: " fmt, __func__

#include <linux/mutex.h>

#include "synaptics_dsx_i2c.h"

int drv_instance_counter;
DEFINE_MUTEX(instances_mutex);
LIST_HEAD(drv_instances_list);

struct synaptics_rmi4_data *synaptics_driver_getdata(
		struct synaptics_rmi4_data *prev)
{
	struct synaptics_rmi4_data *next = NULL;
	mutex_lock(&instances_mutex);
	if (!prev) {
		if (!list_empty(&drv_instances_list))
			next = list_first_entry(&drv_instances_list,
						struct synaptics_rmi4_data, node);
	} else {
		if (!list_is_last(&prev->node, &drv_instances_list))
			next = list_next_entry(prev, node);
	}
	mutex_unlock(&instances_mutex);
	pr_debug("instance ptr %p\n", next);
	return next;
}
EXPORT_SYMBOL(synaptics_driver_getdata);


