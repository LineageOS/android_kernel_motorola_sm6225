/*
 * GalaxyCore touchscreen driver
 *
 * Copyright (C) 2021 GalaxyCore Incorporated
 *
 * Copyright (C) 2021 Neo Chen <neo_chen@gcoreinc.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef _GCORE_IOCTL_H_
#define _GCORE_IOCTL_H_

/* App */
struct reg_msg {
	u32 addr;
	int length;
	u8 buffer[50];
};

#define GALAXYCORE_MAGIC_NUMBER 'G'
#define GALAXYCORE_MAX_NR 12
#define IOC_APP_READ_FW_VERSION       _IOR(GALAXYCORE_MAGIC_NUMBER, 0, char)
#define IOC_APP_UPDATE_FW             _IOW(GALAXYCORE_MAGIC_NUMBER, 1, char)
#define IOC_APP_DEMO                  _IO(GALAXYCORE_MAGIC_NUMBER, 2)
#define IOC_APP_DEMO_RAWDATA          _IO(GALAXYCORE_MAGIC_NUMBER, 3)
#define IOC_APP_READ_REG              _IOR(GALAXYCORE_MAGIC_NUMBER, 4, struct reg_msg)
#define IOC_APP_WRITE_REG             _IOW(GALAXYCORE_MAGIC_NUMBER, 5, struct reg_msg)
#define IOC_APP_GET_RAWDATA_RES       _IOR(GALAXYCORE_MAGIC_NUMBER, 6, char)
#define IOC_APP_START_MP_TEST		  _IO(GALAXYCORE_MAGIC_NUMBER, 7)

/* Tool */
#define IOC_DEBUG_TIME_RST0           _IO(GALAXYCORE_MAGIC_NUMBER, 8)
#define IOC_DEBUG_TIME_RST1           _IO(GALAXYCORE_MAGIC_NUMBER, 9)
#define IOC_TOOL_MODE                 _IOW(GALAXYCORE_MAGIC_NUMBER, 10, int)
#define IOC_TOOL_IDM_OPERATION        _IOW(GALAXYCORE_MAGIC_NUMBER, 11, int)


#endif      /*  _GCORE_IOCTL_H_  */





