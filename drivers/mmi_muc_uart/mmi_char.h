/* Copyright (c) 2018, Motorola Mobility LLC. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef _DRIVERS_CHAR_MMI_H_
#define _DRIVERS_CHAR_MMI_H_

int mmi_char_init(int max_write_size, struct platform_device *pdev);
void mmi_char_exit(void);

#endif /* _DRIVERS_CHAR_MMI_H_ */