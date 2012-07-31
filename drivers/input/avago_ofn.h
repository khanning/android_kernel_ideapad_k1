/* include/linux/avago_ofn.h
 *
 * Copyright (C) 2010 Avago Technologies. 
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include "../arch/arm/mach-tegra/gpio-names.h"

#ifndef _LINUX_AVAGO_OFN_H
#define _LINUX_AVAGO_OFN_H
#define AVAGO_OFN_NAME "avago-ofn"
#define AVAGO_OFN_ID 	0x83

#define AVAGO_OFN_ADDR	0x33 	/* MOSI_A0 = 0 NCS_A1 = 0 */
//#define AVAGO_RESET_GPIO S3C64XX_GPK(4)		/* The GPIO that connect to NRST PIN to reset OFN */
//#define AVAGO_SHUTDOWN_GPIO S3C64XX_GPK(5)		/* The GPIO that connect to SHTDWN PIN */
//#define AVAGO_DOME_GPIO S3C64XX_GPK(7)
#define AVAGO_SHUTDOWN_GPIO		TEGRA_GPIO_PK3
#define AVAGO_DOME_GPIO			TEGRA_GPIO_PK2
//Sam add LED GPIO
#define AVAGO_LED_UP			TEGRA_GPIO_PJ0
#define AVAGO_LED_DOWN          TEGRA_GPIO_PJ2

#define AVAGO_UP_LED_GPIO		TEGRA_GPIO_PJ0
#define AVAGO_DN_LED_GPIO       TEGRA_GPIO_PJ2

#define AVAGO_MOTION_SCALE		6		
#define AVAGO_MOTION_COUNT		2 /* 7 X 8ms = 56ms */		

#define DOME_SINGLE_CLICK_COUNT		6	/* 500ms */
#define DOME_LONG_PRESS_COUNT		21	/* 2000ms */


#endif /*_LINUX_AVAGO_OFN_H */
