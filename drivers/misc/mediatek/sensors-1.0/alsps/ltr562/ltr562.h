/* 
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
/*
 * Definitions for LTR562 als/ps sensor chip.
 */
#ifndef _LTR562_H_
#define _LTR562_H_

#include <linux/ioctl.h>
/* LTR-562 Registers */
#define LTR562_ALS_CTRL			0x80
#define LTR562_PS_CTRL			0x81
#define LTR562_PS_LED			0x82
#define LTR562_PS_PULSES		0x83
#define LTR562_PS_MEAS_RATE		0x84
#define LTR562_ALS_INT_TIME		0x85
#define LTR562_INT_CFG			0x98
#define LTR562_INT_PST 			0x99
#define LTR562_PS_THRES_UP_0	0x9A
#define LTR562_PS_THRES_UP_1	0x9B
#define LTR562_PS_THRES_LOW_0	0x9C
#define LTR562_PS_THRES_LOW_1	0x9D
#define LTR562_PS_CAN_0			0x9E
#define LTR562_PS_CAN_1			0x9F
#define LTR562_LED_DRIVE		0xA4
#define LTR562_DARK_CTRL		0xB4
#define LTR562_IR_AMBIENT_THRES 0xB6
#define LTR562_DSS_CTRL			0xB7

/* 562's Read Only Registers */
#define LTR562_PART_ID			0x86
#define LTR562_MANUFAC_ID		0x87
#define LTR562_ALS_STATUS		0x88
#define LTR562_IR_DATA_0		0x89
#define LTR562_IR_DATA_1		0x8A
#define LTR562_ALS_DATA_0		0x8B
#define LTR562_ALS_DATA_1		0x8C
#define LTR562_PS_STATUS		0x91
#define LTR562_PS_DATA_0		0x92
#define LTR562_PS_DATA_1		0x93
#define LTR562_DARK_CONFIG		0xB9

/* Basic Operating Modes */
#define MODE_ALS_Range1			0x00 << 2  ///for als gain x1
#define MODE_ALS_Range4			0x01 << 2  ///for als gain x4
#define MODE_ALS_Range16		0x02 << 2  ///for als gain x16
#define MODE_ALS_Range64		0x03 << 2  ///for als gain x64
#define MODE_ALS_Range128		0x04 << 2  ///for als gain x128
#define MODE_ALS_Range512		0x05 << 2  ///for als gain x512

#define ALS_RANGE_1				1
#define ALS_RANGE_4 			4
#define ALS_RANGE_16 			16
#define ALS_RANGE_64 			64
#define ALS_RANGE_128			128
#define ALS_RANGE_512			512

#define ALS_DEF_GAIN			ALS_RANGE_16
#define ALS_DEF_GAIN_REG		MODE_ALS_Range128

#define ALS_DEF_INT_MEAS		0x05	// integration time: 100ms, measurement rate: 200ms
#define PS_DEF_MEAS_RATE		0x04	// 11bits & 100ms time
#define PS_DEF_AVG_PULSE		0x87	// 8 pulses & 4n averaging

#define ALS_WIN_FACTOR			80

#define ALS_USE_DRIVER_AVE
//#define PS_USE_DRIVER_AVE
#define PS_USE_XT_OFFSET
#define PS_XT_TARGET			100
#define PS_OFFSET_CHANGE_CONTER 3
#define PS_SAR_CTRL				1		// IR ambient light level is 8k

/* Power On response time in ms */
#define PON_DELAY				200
#define WAKEUP_DELAY			10

extern struct platform_device *get_alsps_platformdev(void);

#endif
