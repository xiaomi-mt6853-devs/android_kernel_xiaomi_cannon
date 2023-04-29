/*
 * Copyright (C) 2016 MediaTek Inc.
 * Copyright (C) 2021 XiaoMi, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#ifndef __SITUATION_H__
#define __SITUATION_H__

#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <hwmsensor.h>
#include <linux/poll.h>
#include "sensor_attr.h"
#include "sensor_event.h"
#include <linux/pm_wakeup.h>

enum situation_index_table {
	inpocket = 0,
	stationary,
	wake_gesture,
	pickup_gesture,
	glance_gesture,
	answer_call,
	motion_detect,
	device_orientation,
	tilt_detector,
	flat,
	sar,
	ps_factory_strm,
	als_factory_strm,
	sar_algo,
	elevator_detect,
	max_situation_support,
};

struct situation_control_path {
	int (*open_report_data)(int open);
	int (*batch)(int flag, int64_t samplingPeriodNs,
		int64_t maxBatchReportLatencyNs);
	int (*flush)(void);
	int (*set_cali)(uint8_t *data, uint8_t count);
	//bool is_report_input_direct;
	bool is_support_wake_lock;
	bool is_support_batch;
};

struct situation_data_path {
	int (*get_data)(int *value, int *status);
};

struct situation_init_info {
	char *name;
	int (*init)(void);
	int (*uninit)(void);
	struct platform_driver *platform_diver_addr;
};

struct situation_data_control_context {
	struct situation_control_path situation_ctl;
	struct situation_data_path situation_data;
	bool is_active_data;
	bool is_active_nodata;
	bool is_batch_enable;
	int power;
	int enable;
	int64_t delay_ns;
	int64_t latency_ns;
};

struct situation_context {
	struct sensor_attr_t mdev;
	struct mutex situation_op_mutex;
	struct situation_data_control_context
		ctl_context[max_situation_support];
	struct wakeup_source ws[max_situation_support];
	char *wake_lock_name[max_situation_support];
};

extern int situation_data_report_t(int handle, uint32_t one_sample_data,
	int64_t time_stamp);
extern int situation_data_report(int handle, uint32_t one_sample_data);
extern int situation_notify_t(int handle, int64_t time_stamp);
extern int situation_notify(int handle);
extern int situation_flush_report(int handle);
extern int situation_driver_add(struct situation_init_info *obj, int handle);
extern int situation_register_control_path(
	struct situation_control_path *ctl, int handle);
extern int situation_register_data_path(struct situation_data_path *data,
	int handle);
extern int sar_data_report_t(int32_t value[3], int64_t time_stamp);
extern int sar_data_report(int32_t value[3]);
extern int sar_cali_report_t(int32_t value[3], int64_t time_stamp);
extern int sar_cali_report(int32_t value[3]);
extern int elevator_data_report_t(int32_t value[1], int64_t time_stamp);
extern int elevator_data_report(int32_t value[1]);
extern int als_factory_strm_data_report_t(int32_t value[3], int64_t time_stamp);
#endif
