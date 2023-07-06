/*
 * Copyright (C) 2018 MediaTek Inc.
 * Copyright (C) 2020 XiaoMi, Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */
#ifndef __TMP_BTS_H__
#define __TMP_BTS_H__

#define APPLY_PRECISE_NTC_TABLE
#define APPLY_AUXADC_CALI_DATA

#define AUX_IN0_NTC (0)
#define AUX_IN1_NTC (1)
#define AUX_IN2_NTC (2)
/********MI ADD START*********/
#define AUX_IN4_NTC (4)
#define AUX_IN5_NTC (5)
#define AUX_IN6_NTC (6)
/***********END*************/

#define BTS_RAP_PULL_UP_R		100000 /* 100K, pull up resister */

#define BTS_TAP_OVER_CRITICAL_LOW	4397119 /* base on 100K NTC temp
						 * default value -40 deg
						 */

#define BTS_RAP_PULL_UP_VOLTAGE		1800 /* 1.8V ,pull up voltage */

#define BTS_RAP_NTC_TABLE		7 /* default is NCP15WF104F03RC(100K) */

#define BTS_RAP_ADC_CHANNEL		AUX_IN0_NTC /* default is 0 */

#define BTSMDPA_RAP_PULL_UP_R		100000 /* 100K, pull up resister */

#define BTSMDPA_TAP_OVER_CRITICAL_LOW	4397119 /* base on 100K NTC temp
						 * default value -40 deg
						 */

#define BTSMDPA_RAP_PULL_UP_VOLTAGE	1800 /* 1.8V ,pull up voltage */

#define BTSMDPA_RAP_NTC_TABLE		7 /* default is NCP15WF104F03RC(100K) */

#define BTSMDPA_RAP_ADC_CHANNEL		AUX_IN1_NTC /* default is 1 */


#define BTSNRPA_RAP_PULL_UP_R		100000	/* 100K,pull up resister */
#define BTSNRPA_TAP_OVER_CRITICAL_LOW	4397119	/* base on 100K NTC temp
						 *default value -40 deg
						 */

#define BTSNRPA_RAP_PULL_UP_VOLTAGE	1800	/* 1.8V ,pull up voltage */
#define BTSNRPA_RAP_NTC_TABLE		7

#define BTSNRPA_RAP_ADC_CHANNEL		AUX_IN2_NTC

/********MI ADD START*********/
#define XMCHARGER_RAP_PULL_UP_R		100000	/* 100K,pull up resister */
#define XMCHARGER_TAP_OVER_CRITICAL_LOW	4397119	/* base on 100K NTC temp
						 *default value -40 deg
						 */

#define XMCHARGER_RAP_PULL_UP_VOLTAGE	1800	/* 1.8V ,pull up voltage */
#define XMCHARGER_RAP_NTC_TABLE		7

#define XMCHARGER_RAP_ADC_CHANNEL		AUX_IN4_NTC

#define LCD_RAP_PULL_UP_R		100000	/* 100K,pull up resister */
#define LCD_TAP_OVER_CRITICAL_LOW	4397119	/* base on 100K NTC temp
						 *default value -40 deg
						 */

#define LCD_RAP_PULL_UP_VOLTAGE	1800	/* 1.8V ,pull up voltage */
#define LCD_RAP_NTC_TABLE		7

#define LCD_RAP_ADC_CHANNEL		AUX_IN5_NTC

#define QUITE_RAP_PULL_UP_R		100000	/* 100K,pull up resister */
#define QUITE_TAP_OVER_CRITICAL_LOW	4397119	/* base on 100K NTC temp
						 *default value -40 deg
						 */

#define QUITE_RAP_PULL_UP_VOLTAGE	1800	/* 1.8V ,pull up voltage */
#define QUITE_RAP_NTC_TABLE		7

#define QUITE_RAP_ADC_CHANNEL		AUX_IN6_NTC

/***********END*************/

extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int *rawdata);
extern int IMM_IsAdcInitReady(void);

#endif	/* __TMP_BTS_H__ */
