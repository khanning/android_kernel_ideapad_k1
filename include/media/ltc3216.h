/**
 * Copyright (c) 2008 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#ifndef __LTC3216_H__
#define __LTC3216_H__

/* COMPAL - START - [2011/2/22  19:28] - Jamin - [Camera]: enable flash TPS61052 */
#define FLASH_NAME_LTC3216  "ltc3216"
/* COMPAL - END */

#include <linux/ioctl.h>  /* For IOCTL macros */

#define LTC3216_IOCTL_FLASH_ON		_IOW('o', 1, __u16)
#define LTC3216_IOCTL_FLASH_OFF		_IOW('o', 2, __u16)
#define LTC3216_IOCTL_TORCH_ON		_IOW('o', 3, __u16)
#define LTC3216_IOCTL_TORCH_OFF		_IOW('o', 4, __u16)

struct flash_ltc3216_platform_data {
	int (*power_on)(void);
	int (*power_off)(void);

};

#endif  /* __OV5650_H__ */

