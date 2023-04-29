/*
 * Copyright (C) 2020 MediaTek Inc.
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

#ifndef __APUSYS_MDW_CMN_H__
#define __APUSYS_MDW_CMN_H__

#include <linux/printk.h>
#include <linux/seq_file.h>
#include <linux/time.h>

extern u32 g_mdw_klog;

enum {
	MDW_DBG_DRV = 0x01,
	MDW_DBG_FLW = 0x02,
	MDW_DBG_CMD = 0x04,
	MDW_DBG_MEM = 0x08,
	MDW_DBG_PEF = 0x10,
};

#define APUSYS_PREFIX "[apusys]"

static inline
int mdw_debug_on(int mask)
{
	return g_mdw_klog & mask;
}

#define mdw_debug(mask, x, ...) do { if (mdw_debug_on(mask)) \
		pr_debug(APUSYS_PREFIX " %s/%d " x, __func__, \
		__LINE__, ##__VA_ARGS__); \
	} while (0)

#define mdw_drv_err(x, args...) \
	pr_err(APUSYS_PREFIX "[error] %s " x, __func__, ##args)
#define mdw_drv_warn(x, args...) \
	pr_debug(APUSYS_PREFIX "[warn] %s " x, __func__, ##args)
#define mdw_drv_info(x, args...) \
	pr_debug(APUSYS_PREFIX "%s " x, __func__, ##args)

#define mdw_drv_debug(x, ...) mdw_debug(MDW_DBG_DRV, x, ##__VA_ARGS__)
#define mdw_flw_debug(x, ...) mdw_debug(MDW_DBG_FLW, x, ##__VA_ARGS__)
#define mdw_cmd_debug(x, ...) mdw_debug(MDW_DBG_CMD, x, ##__VA_ARGS__)
#define mdw_mem_debug(x, ...) mdw_debug(MDW_DBG_MEM, x, ##__VA_ARGS__)
#define mdw_pef_debug(x, ...) mdw_debug(MDW_DBG_PEF, x, ##__VA_ARGS__)

/* print to console via seq file */
#define mdw_con_info(s, x, args...) \
	{\
		if (s) \
			seq_printf(s, x, ##args); \
		else \
			mdw_drv_debug(x, ##args); \
	}

uint32_t mdw_cmn_get_time_diff(struct timespec *prev, struct timespec *next);

#endif
