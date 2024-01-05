/**
 * User space driver API for JIIOV's fingerprint sensor.
 *
 * Copyright (C) 2020 JIIOV Corporation. <http://www.jiiov.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
 * Public License for more details.
 **/

#ifndef __JIIOV_PLATFORM_H__
#define __JIIOV_PLATFORM_H__

#define ANC_WAKELOCK_HOLD_TIME 500 /* ms */
typedef enum {
    CUSTOM_COMMAND_INIT,
    CUSTOM_COMMAND_CLEAR_TOUCH_STATE,
    CUSTOM_COMMAND_SET_PRODUCT_ID,
    CUSTOM_COMMAND_DEINIT,
} CUSTOM_COMMAND;

int custom_send_command(CUSTOM_COMMAND cmd, void *p_param);

#endif /* __JIIOV_PLATFORM_H__ */