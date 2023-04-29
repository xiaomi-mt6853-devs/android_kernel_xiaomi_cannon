/*
 * Copyright (C) 2016 MediaTek Inc.
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

/* system includes */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/cpu.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/kthread.h>
#include <linux/hrtimer.h>
#include <linux/sched/rt.h>
#include <linux/atomic.h>
#include <linux/clk.h>
#include <linux/ktime.h>
#include <linux/time.h>
#include <linux/jiffies.h>
#include <linux/bitops.h>
#include <linux/uaccess.h>
#include <linux/seq_file.h>
#include <linux/types.h>
#include <linux/suspend.h>
#include <linux/topology.h>
#include <linux/math64.h>
#include <mt-plat/sync_write.h>
#include <mt-plat/mtk_io.h>
#include <mt-plat/aee.h>
#include <trace/events/mtk_events.h>
#ifdef MET_SUPPORT
#include <mt-plat/met_drv.h>
#endif /* MET_SUPPORT */

#include <mtk_cm_mgr.h>
#include <mtk_cm_mgr_platform_data.h>
#ifdef CONFIG_MTK_CPU_FREQ
#include <mtk_cpufreq_api.h>
#endif /* CONFIG_MTK_CPU_FREQ */

#include <linux/pm_qos.h>
#include <helio-dvfsrc.h>
#if defined(CONFIG_MACH_MT6771)
#include <mtk_spm_vcore_dvfs.h>
#endif /* defined(CONFIG_MACH_MT6771) */

#if defined(CONFIG_MTK_TINYSYS_SSPM_SUPPORT) && defined(USE_CM_MGR_AT_SSPM)
#if defined(USE_SSMP_VER_V2)
#include <sspm_ipi_id.h>
#include <sspm_define.h>
static int cm_sspm_ready;
int cm_ipi_ackdata;
#else
#include <sspm_ipi.h>
#endif
#endif /* CONFIG_MTK_TINYSYS_SSPM_SUPPORT */

__attribute__((weak))
void cm_mgr_update_dram_by_cpu_opp(int cpu_opp) {};

spinlock_t cm_mgr_lock;

static unsigned long long test_diff;
static unsigned long long cnt;
static unsigned int test_max;
static unsigned int prev_freq_idx[CM_MGR_CPU_CLUSTER];
static unsigned int prev_freq[CM_MGR_CPU_CLUSTER];
/* 0: < 50us */
/* 1: 50~100us */
/* 2: 100~200us */
/* 3: 200~300us */
/* 4: over 300us */
static unsigned int time_cnt_data[5];

unsigned int cpu_power_up_array[CM_MGR_CPU_CLUSTER];
unsigned int cpu_power_down_array[CM_MGR_CPU_CLUSTER];
unsigned int cpu_power_up[CM_MGR_CPU_CLUSTER];
unsigned int cpu_power_down[CM_MGR_CPU_CLUSTER];
unsigned int v2f[CM_MGR_CPU_CLUSTER];
int vcore_power_up;
int vcore_power_down;
int cpu_opp_cur[CM_MGR_CPU_CLUSTER];
int ratio_max[CM_MGR_CPU_CLUSTER];
int ratio[CM_MGR_CPU_COUNT];
int count[CM_MGR_CPU_CLUSTER];
int count_ack[CM_MGR_CPU_CLUSTER];
int vcore_dram_opp;
int vcore_dram_opp_cur;
int cm_mgr_abs_load;
int cm_mgr_rel_load;
int total_bw;
int cps_valid;
int debounce_times_up;
int debounce_times_down;
int ratio_scale[CM_MGR_CPU_CLUSTER];
int max_load[CM_MGR_CPU_CLUSTER];
int cpu_load[NR_CPUS];
int loading_acc[NR_CPUS];
int loading_cnt;

static void update_v2f(int update, int debug)
{
	int i, j;
	int _f, _v, _v2f;

	for (j = 0; j < CM_MGR_CPU_CLUSTER; j++) {
		for (i = 0; i < 16; i++) {
#ifdef CONFIG_MTK_CPU_FREQ
			_f = mt_cpufreq_get_freq_by_idx(j, i) / 1000;
			_v = mt_cpufreq_get_volt_by_idx(j, i) / 100;
#else
			_f = 0;
			_v = 0;
#endif /* CONFIG_MTK_CPU_FREQ */
			_v2f = (_v / 10) * (_v / 10) * _f / 100000;
			if (update)
				_v2f_all[i][j] = _v2f;
			if (debug)
				pr_debug("%d-i %.2d v %.8d f %.8d v2f %.8d\n",
						j, i, _v, _f, _v2f);
		}
	}
}

#ifdef USE_TIMER_CHECK
struct timer_list cm_mgr_timer;

static void cm_mgr_timer_fn(unsigned long data)
{
	if (cm_mgr_timer_enable)
		check_cm_mgr_status_internal();
}
#endif /* USE_TIMER_CHECK */

static int cm_mgr_check_up_status(int level, int *cpu_ratio_idx)
{
	int idx;
	int cpu_power_total;
	int i;

	idx = CM_MGR_CPU_CLUSTER * level;
	cpu_power_total = 0;
#ifdef PER_CPU_STALL_RATIO
	for (i = 0; i < CM_MGR_CPU_COUNT; i++) {
		if (i < CM_MGR_CPU_LIMIT)
			cpu_power_up_array[0] +=
				cpu_power_gain_opp(total_bw, IS_UP,
						cpu_opp_cur[0],
						cpu_ratio_idx[i], idx);
#ifndef USE_SINGLE_CLUSTER
		else
			cpu_power_up_array[1] +=
				cpu_power_gain_opp(total_bw, IS_UP,
						cpu_opp_cur[1],
						cpu_ratio_idx[i], idx + 1);
#endif /* ! USE_SINGLE_CLUSTER */
	}

	for (i = 0; i < CM_MGR_CPU_CLUSTER; i++) {
		cpu_power_up[i] = cpu_power_up_array[i] * v2f[i] / 100;
		cpu_power_total += cpu_power_up[i];
	}
#else
	for (i = 0; i < CM_MGR_CPU_CLUSTER; i++) {
		cpu_power_up_array[i] =
			cpu_power_gain_opp(total_bw, IS_UP,
					cpu_opp_cur[i],
					max_ratio_idx[i], idx + i);
		cpu_power_up[i] = cpu_power_up_array[i] *
			count[i] * v2f[i] / 100;
		cpu_power_total += cpu_power_up[i];
	}
#endif /* PER_CPU_STALL_RATIO */

	if (cm_mgr_opp_enable == 0) {
		if (vcore_dram_opp != CM_MGR_EMI_OPP) {
			vcore_dram_opp = CM_MGR_EMI_OPP;
#ifdef DEBUG_CM_MGR
			pr_debug("#@# %s(%d) vcore_dram_opp %d->%d\n",
					__func__, __LINE__,
					vcore_dram_opp_cur, vcore_dram_opp);
#endif /* DEBUG_CM_MGR */
			cm_mgr_set_dram_level(
					CM_MGR_EMI_OPP - vcore_dram_opp);
		}

		return -1;
	}

	idx = level;
	vcore_power_up = vcore_power_gain(vcore_power_gain, total_bw, idx);
#ifdef DEBUG_CM_MGR
	pr_debug("#@# vcore_power_up %d < cpu_power_total %d\n",
			vcore_power_up, cpu_power_total);
#endif /* DEBUG_CM_MGR */
	if ((vcore_power_up * vcore_power_ratio_up[idx]) <
			(cpu_power_total * cpu_power_ratio_up[idx])) {
		debounce_times_down = 0;
		if (++debounce_times_up >= debounce_times_up_adb[idx]) {
			if (debounce_times_reset_adb)
				debounce_times_up = 0;
			vcore_dram_opp = vcore_dram_opp_cur - 1;
#ifdef DEBUG_CM_MGR
			pr_debug("#@# %s(%d) vcore_dram_opp up %d->%d\n",
					__func__, __LINE__,
					vcore_dram_opp_cur, vcore_dram_opp);
#endif /* DEBUG_CM_MGR */
			cm_mgr_set_dram_level(
					CM_MGR_EMI_OPP - vcore_dram_opp);
		} else {
			if (debounce_times_reset_adb)
				debounce_times_up = 0;
		}

		return -1;
	}

	return 0;
}

static int cm_mgr_check_down_status(int level, int *cpu_ratio_idx)
{
	int idx;
	int cpu_power_total;
	int i;

	idx = CM_MGR_CPU_CLUSTER * (level - 1);
	cpu_power_total = 0;
#ifdef PER_CPU_STALL_RATIO
	for (i = 0; i < CM_MGR_CPU_COUNT; i++) {
		if (i < CM_MGR_CPU_LIMIT)
			cpu_power_down_array[0] +=
				cpu_power_gain_opp(total_bw, IS_DOWN,
						cpu_opp_cur[0],
						cpu_ratio_idx[i], idx);
#ifndef USE_SINGLE_CLUSTER
		else
			cpu_power_down_array[1] +=
				cpu_power_gain_opp(total_bw, IS_DOWN,
						cpu_opp_cur[1],
						cpu_ratio_idx[i], idx + 1);
#endif /* ! USE_SINGLE_CLUSTER */
	}

	for (i = 0; i < CM_MGR_CPU_CLUSTER; i++) {
		cpu_power_down[i] = cpu_power_down_array[i] * v2f[i] / 100;
		cpu_power_total += cpu_power_down[i];
	}
#else
	for (i = 0; i < CM_MGR_CPU_CLUSTER; i++) {
		cpu_power_down_array[i] =
			cpu_power_gain_opp(total_bw, IS_DOWN,
					cpu_opp_cur[i],
					max_ratio_idx[i], idx + i);
		cpu_power_down[i] = cpu_power_down_array[i] *
			count[i] * v2f[i] / 100;
		cpu_power_total += cpu_power_down[i];
	}
#endif /* PER_CPU_STALL_RATIO */

	if (cm_mgr_opp_enable == 0) {
		if (vcore_dram_opp != CM_MGR_EMI_OPP) {
			vcore_dram_opp = CM_MGR_EMI_OPP;
#ifdef DEBUG_CM_MGR
			pr_debug("#@# %s(%d) vcore_dram_opp %d->%d\n",
					__func__, __LINE__,
					vcore_dram_opp_cur, vcore_dram_opp);
#endif /* DEBUG_CM_MGR */
			cm_mgr_set_dram_level(
					CM_MGR_EMI_OPP - vcore_dram_opp);
		}

		return -1;
	}

	idx = level - 1;
	vcore_power_down = vcore_power_gain(vcore_power_gain, total_bw, idx);
#ifdef DEBUG_CM_MGR
	pr_debug("#@# vcore_power_down %d > cpu_power_total %d\n",
			vcore_power_down, cpu_power_total);
#endif /* DEBUG_CM_MGR */
	if ((vcore_power_down * vcore_power_ratio_down[idx]) >
			(cpu_power_total * cpu_power_ratio_down[idx])) {
		debounce_times_up = 0;
		if (++debounce_times_down >= debounce_times_down_adb[idx]) {
			if (debounce_times_reset_adb)
				debounce_times_down = 0;
			vcore_dram_opp = vcore_dram_opp_cur + 1;
#ifdef DEBUG_CM_MGR
			pr_debug("#@# %s(%d) vcore_dram_opp down %d->%d\n",
					__func__, __LINE__,
					vcore_dram_opp_cur, vcore_dram_opp);
#endif /* DEBUG_CM_MGR */
			cm_mgr_set_dram_level(
					CM_MGR_EMI_OPP - vcore_dram_opp);
		} else {
			if (debounce_times_reset_adb)
				debounce_times_down = 0;
		}

		return -1;
	}

	return 0;
}

struct timer_list cm_mgr_perf_timer;
#define USE_TIMER_PERF_CHECK_TIME msecs_to_jiffies(50)

static void cm_mgr_perf_timer_fn(unsigned long data)
{
	if (cm_mgr_perf_timer_enable)
		check_cm_mgr_status_internal();
}

void cm_mgr_perf_set_status(int enable)
{
#ifdef CM_PERF_HINT_OFFSET
	qos_sram_write(CM_PERF_HINT_OFFSET,
			(qos_sram_read(CM_PERF_HINT_OFFSET) & ~0x1) |
			(enable & 0x1));
#endif /* CM_PERF_HINT_OFFSET */

	if (cm_mgr_disable_fb == 1 && cm_mgr_blank_status == 1)
		enable = 0;

	cm_mgr_perf_platform_set_force_status(enable);

	if (cm_mgr_perf_force_enable)
		return;

	cm_mgr_perf_platform_set_status(enable);

	if (enable != cm_mgr_perf_timer_enable) {
		cm_mgr_perf_timer_enable = enable;

#if defined(CONFIG_MTK_TINYSYS_SSPM_SUPPORT) && defined(USE_CM_MGR_AT_SSPM)
		if (cm_mgr_sspm_enable == 1)
			return;
#endif /* CONFIG_MTK_TINYSYS_SSPM_SUPPORT */

		if (enable == 1) {
			unsigned long expires;

			expires = jiffies + USE_TIMER_PERF_CHECK_TIME;
			mod_timer(&cm_mgr_perf_timer, expires);

			check_cm_mgr_status_internal();
		} else
			del_timer(&cm_mgr_perf_timer);
	}
}

void cm_mgr_perf_set_force_status(int enable)
{
	if (enable != cm_mgr_perf_force_enable) {
		cm_mgr_perf_force_enable = enable;
		if (enable == 0) {
			cm_mgr_perf_platform_set_force_status(enable);
			check_cm_mgr_status_internal();
		}
	}
}

void check_cm_mgr_status_internal(void)
{
	unsigned long long result = 0;
	ktime_t now = 0;
	ktime_t done = 0;
	int level;
	unsigned long flags;

#if !defined(CONFIG_MACH_MT6771)
	if (!is_dvfsrc_enabled())
		return;
#endif /* defined(CONFIG_MACH_MT6771) */

	if (cm_mgr_enable == 0)
		return;

#if defined(CONFIG_MTK_TINYSYS_SSPM_SUPPORT) && defined(USE_CM_MGR_AT_SSPM)
	if (cm_mgr_sspm_enable == 1)
		return;
#endif /* CONFIG_MTK_TINYSYS_SSPM_SUPPORT */

	if (cm_mgr_disable_fb == 1 && cm_mgr_blank_status == 1)
		return;

	if (cm_mgr_perf_force_enable)
		return;

	if (!cm_mgr_check_bw_status()) {
		cm_mgr_set_dram_level(0);
		return;
	}

	if (spin_trylock_irqsave(&cm_mgr_lock, flags)) {
		int ret;
		int max_ratio_idx[CM_MGR_CPU_CLUSTER];
#if defined(LIGHT_LOAD) && defined(CONFIG_MTK_SCHED_RQAVG_US)
		unsigned int cpu;
		unsigned int rel_load, abs_load;
#endif /* defined(LIGHT_LOAD) && defined(CONFIG_MTK_SCHED_RQAVG_US) */
#ifdef PER_CPU_STALL_RATIO
		int cpu_ratio_idx[CM_MGR_CPU_COUNT];
#endif /* PER_CPU_STALL_RATIO */
		int i;

		vcore_dram_opp_cur = cm_mgr_get_dram_opp();
		if (vcore_dram_opp_cur > CM_MGR_EMI_OPP) {
			spin_unlock_irqrestore(&cm_mgr_lock, flags);
			return;
		}

		if (--cm_mgr_loop > 0)
			goto cm_mgr_opp_end;
		cm_mgr_loop = cm_mgr_loop_count;

#if defined(LIGHT_LOAD) && defined(CONFIG_MTK_SCHED_RQAVG_US)
		cm_mgr_abs_load = 0;
		cm_mgr_rel_load = 0;

		for_each_online_cpu(cpu) {
			int tmp;

			if (cpu >= CM_MGR_CPU_COUNT)
				break;

#ifdef CONFIG_MTK_CPU_FREQ
			tmp = mt_cpufreq_get_cur_phy_freq_no_lock(
					cpu / CM_MGR_CPU_LIMIT) /
				100000;
#else
			tmp = 0;
#endif /* CONFIG_MTK_CPU_FREQ */
			sched_get_percpu_load2(cpu, 1, &rel_load, &abs_load);
			cm_mgr_abs_load += abs_load * tmp;
			cm_mgr_rel_load += rel_load * tmp;

			cpu_load[cpu] = rel_load;
			loading_acc[cpu] += rel_load;
		}
		loading_cnt++;

		if ((cm_mgr_abs_load < light_load_cps) &&
				(vcore_dram_opp_cur == CM_MGR_EMI_OPP)) {
			cps_valid = 0;
			goto cm_mgr_opp_end;
		}
#endif /* defined(LIGHT_LOAD) && defined(CONFIG_MTK_SCHED_RQAVG_US) */
		cps_valid = 1;

		now = ktime_get();

#ifdef USE_NEW_CPU_OPP
#ifdef USE_SINGLE_CLUSTER
		ret = cm_mgr_check_stall_ratio(
				prev_freq[0] / 1000,
				0);
#else
		ret = cm_mgr_check_stall_ratio(
				prev_freq[0] / 1000,
				prev_freq[1] / 1000);
#endif /* USE_SINGLE_CLUSTER */
#else
#ifdef CONFIG_MTK_CPU_FREQ
#ifdef USE_AVG_PMU
#ifdef USE_SINGLE_CLUSTER
		ret = cm_mgr_check_stall_ratio(
				mt_cpufreq_get_cur_phy_freq_no_lock(0) / 1000,
				0);
#else
		ret = cm_mgr_check_stall_ratio(
				mt_cpufreq_get_cur_phy_freq_no_lock(0) / 1000,
				mt_cpufreq_get_cur_phy_freq_no_lock(1) / 1000);
#endif /* USE_SINGLE_CLUSTER */
#else
#ifdef USE_SINGLE_CLUSTER
		ret = cm_mgr_check_stall_ratio(
				mt_cpufreq_get_cur_freq(0) / 1000,
				0);
#else
		ret = cm_mgr_check_stall_ratio(
				mt_cpufreq_get_cur_freq(0) / 1000,
				mt_cpufreq_get_cur_freq(1) / 1000);
#endif /* USE_SINGLE_CLUSTER */
#endif /* USE_AVG_PMU */
#else
		ret = 0;
#endif /* CONFIG_MTK_CPU_FREQ */
#endif /* USE_NEW_CPU_OPP */
		total_bw = cm_mgr_get_bw() / 512;
		memset(count_ack, 0, ARRAY_SIZE(count_ack));

		if (total_bw_value)
			total_bw = total_bw_value;
		if (total_bw >= vcore_power_array_size(cm_mgr_get_idx()))
			total_bw = vcore_power_array_size(cm_mgr_get_idx()) - 1;
		if (total_bw < 0)
			total_bw = 0;

		if (update_v2f_table == 1) {
			update_v2f(1, 0);
			update_v2f_table++;
		}

		/* get max loading */
		memset(max_load, 0, ARRAY_SIZE(count_ack));

		for_each_possible_cpu(i) {
			int avg_load;

			if (i >= CM_MGR_CPU_COUNT)
				break;

			if (unlikely(loading_cnt == 0))
				break;
			avg_load = loading_acc[i] / loading_cnt;
			if (avg_load > max_load[i / CM_MGR_CPU_LIMIT])
				max_load[i / CM_MGR_CPU_LIMIT] = avg_load;
			loading_acc[i] = 0;
		}

		for (i = 0; i < CM_MGR_CPU_CLUSTER; i++) {
			count[i] = cm_mgr_get_cpu_count(i);
			ratio_max[i] = cm_mgr_get_max_stall_ratio(i);
			max_ratio_idx[i] = ratio_max[i] / 5;
			if (max_ratio_idx[i] > RATIO_COUNT)
				max_ratio_idx[i] = RATIO_COUNT;
#ifdef USE_NEW_CPU_OPP
			cpu_opp_cur[i] = prev_freq_idx[i];
#else
#ifdef CONFIG_MTK_CPU_FREQ
			cpu_opp_cur[i] = mt_cpufreq_get_cur_freq_idx(i);
#else
			cpu_opp_cur[i] = 0;
#endif /* CONFIG_MTK_CPU_FREQ */
#endif /* USE_NEW_CPU_OPP */
			v2f[i] = _v2f_all[cpu_opp_cur[i]][i];
			cpu_power_up_array[i] = cpu_power_up[i] = 0;
			cpu_power_down_array[i] = cpu_power_down[i] = 0;

			/* calc scaled ratio */
			ratio_scale[i] = (max_load[i] > 0) ?
				(ratio_max[i] * 100 / max_load[i]) :
				ratio_max[i];
			if (ratio_scale[i] > 100)
				ratio_scale[i] = 100;
		}
#ifdef DEBUG_CM_MGR
		print_hex_dump(KERN_INFO, "cpu_opp_cur: ", DUMP_PREFIX_NONE, 16,
				1, &cpu_opp_cur[0], sizeof(cpu_opp_cur), 0);
#endif /* DEBUG_CM_MGR */

		vcore_power_up = 0;
		vcore_power_down = 0;

		for (i = 0; i < CM_MGR_CPU_COUNT; i++) {
			ratio[i] = cm_mgr_get_stall_ratio(i);
			cpu_ratio_idx[i] = ratio[i] / 5;
			if (cpu_ratio_idx[i] > RATIO_COUNT)
				cpu_ratio_idx[i] = RATIO_COUNT;
		}
#ifdef DEBUG_CM_MGR
		print_hex_dump(KERN_INFO, "ratio: ", DUMP_PREFIX_NONE, 16,
				1, &ratio[0], sizeof(ratio), 0);
		print_hex_dump(KERN_INFO, "cpu_ratio_idx: ",
				DUMP_PREFIX_NONE, 16,
				1, &cpu_ratio_idx[0],
				sizeof(cpu_ratio_idx), 0);
#endif /* DEBUG_CM_MGR */

		level = CM_MGR_EMI_OPP - vcore_dram_opp_cur;
		if (vcore_dram_opp_cur != 0) {
			if (cm_mgr_check_up_status(level, cpu_ratio_idx) < 0)
				goto cm_mgr_opp_end;
		}
		if (vcore_dram_opp_cur != CM_MGR_EMI_OPP) {
			if (cm_mgr_check_down_status(level, cpu_ratio_idx) < 0)
				goto cm_mgr_opp_end;
		}

		vcore_dram_opp = vcore_dram_opp_cur;
		if (vcore_dram_opp == CM_MGR_EMI_OPP)
			cm_mgr_set_dram_level(0);

cm_mgr_opp_end:
		cm_mgr_update_met();

		if (cm_mgr_perf_timer_enable) {
			unsigned long expires;

			expires = jiffies + USE_TIMER_PERF_CHECK_TIME;
			mod_timer(&cm_mgr_perf_timer, expires);
		}

#ifdef USE_TIMER_CHECK
		if (cm_mgr_timer_enable) {
			if (vcore_dram_opp != CM_MGR_EMI_OPP) {
				unsigned long expires;

				expires = jiffies +
					USE_TIMER_CHECK_TIME;
				mod_timer(&cm_mgr_timer, expires);
			} else {
				del_timer(&cm_mgr_timer);
			}
		}
#endif /* USE_TIMER_CHECK */

		done = ktime_get();
		if (!ktime_after(done, now)) {
			/* pr_debug("ktime overflow!!\n"); */
			spin_unlock_irqrestore(&cm_mgr_lock, flags);
			return;
		}
		result = ktime_to_us(ktime_sub(done, now));
		test_diff += result;
		cnt++;
		if (result > test_max)
			test_max = result;
		if (result >= 300)
			time_cnt_data[4]++;
		else if (result >= 200 && result < 300)
			time_cnt_data[3]++;
		else if (result >= 100 && result < 200)
			time_cnt_data[2]++;
		else if (result >= 50 && result < 100)
			time_cnt_data[1]++;
		else
			time_cnt_data[0]++;
		spin_unlock_irqrestore(&cm_mgr_lock, flags);
	}
}

void check_cm_mgr_status(unsigned int cluster, unsigned int freq)
{

#ifdef CM_TRIGEAR
	unsigned int clamping_idx = CM_MGR_CPU_OPP_SIZE - 1;
#endif
#ifdef CONFIG_MTK_CPU_FREQ
	int freq_idx = 0;
	struct mt_cpu_dvfs *p;

	p = id_to_cpu_dvfs(cluster);
	if (p)
		freq_idx = _search_available_freq_idx(p, freq, 0);

	if (freq_idx == prev_freq_idx[cluster])
		return;

	prev_freq_idx[cluster] = freq_idx;
	prev_freq[cluster] = freq;
#else
	prev_freq_idx[cluster] = 0;
	prev_freq[cluster] = 0;
#endif /* CONFIG_MTK_CPU_FREQ */


	/* if TRIGEAR use B cluster freq */
#ifdef CM_TRIGEAR
	clamping_idx = MIN(prev_freq_idx[CM_MGR_B], prev_freq_idx[CM_MGR_BB]);
	cm_mgr_update_dram_by_cpu_opp(clamping_idx);
#else
	cm_mgr_update_dram_by_cpu_opp(prev_freq_idx[CM_MGR_CPU_CLUSTER - 1]);
#endif

	check_cm_mgr_status_internal();
}

void cm_mgr_enable_fn(int enable)
{
	cm_mgr_enable = enable;
	if (!cm_mgr_enable)
		cm_mgr_set_dram_level(-1);
#if defined(CONFIG_MTK_TINYSYS_SSPM_SUPPORT) && defined(USE_CM_MGR_AT_SSPM)
	cm_mgr_to_sspm_command(IPI_CM_MGR_ENABLE,
			cm_mgr_enable);
#endif /* CONFIG_MTK_TINYSYS_SSPM_SUPPORT */
}

#if defined(CONFIG_MTK_TINYSYS_SSPM_SUPPORT) && defined(USE_CM_MGR_AT_SSPM)
#if defined(USE_SSMP_VER_V2)
int cm_mgr_to_sspm_command(u32 cmd, int val)
{
	unsigned int ret = 0;
	struct cm_mgr_data cm_mgr_d;

	if (cm_sspm_ready != 1) {
		pr_debug("#@# %s(%d) sspm not ready(%d) to receive cmd(%d)\n",
			__func__, __LINE__, cm_sspm_ready, cmd);
		ret = -1;
		return ret;
	}
	cm_ipi_ackdata = 0;

	switch (cmd) {
	case IPI_CM_MGR_INIT:
	case IPI_CM_MGR_ENABLE:
	case IPI_CM_MGR_OPP_ENABLE:
	case IPI_CM_MGR_SSPM_ENABLE:
	case IPI_CM_MGR_BLANK:
	case IPI_CM_MGR_DISABLE_FB:
	case IPI_CM_MGR_DRAM_TYPE:
	case IPI_CM_MGR_CPU_POWER_RATIO_UP:
	case IPI_CM_MGR_CPU_POWER_RATIO_DOWN:
	case IPI_CM_MGR_VCORE_POWER_RATIO_UP:
	case IPI_CM_MGR_VCORE_POWER_RATIO_DOWN:
	case IPI_CM_MGR_DEBOUNCE_UP:
	case IPI_CM_MGR_DEBOUNCE_DOWN:
	case IPI_CM_MGR_DEBOUNCE_TIMES_RESET_ADB:
	case IPI_CM_MGR_DRAM_LEVEL:
	case IPI_CM_MGR_LIGHT_LOAD_CPS:
	case IPI_CM_MGR_LOADING_ENABLE:
	case IPI_CM_MGR_LOADING_LEVEL:
	case IPI_CM_MGR_EMI_DEMAND_CHECK:
	case IPI_CM_MGR_OPP_FREQ_SET:
	case IPI_CM_MGR_OPP_VOLT_SET:
	case IPI_CM_MGR_BCPU_WEIGHT_MAX_SET:
	case IPI_CM_MGR_BCPU_WEIGHT_MIN_SET:
#ifdef CM_TRIGEAR
	/* FALLTHROUGH */
	case IPI_CM_MGR_BBCPU_WEIGHT_MAX_SET:
		/* FALLTHROUGH */
	case IPI_CM_MGR_BBCPU_WEIGHT_MIN_SET:
		/* FALLTHROUGH */
#endif
#ifdef CM_BCPU_MIN_OPP_WEIGHT
	case IPI_CM_MGR_BCPU_MIN_OPP_WEIGHT_SET:
		/* FALLTHROUGH */
	case IPI_CM_MGR_BCPU_LOW_OPP_WEIGHT_SET:
		/* FALLTHROUGH */
	case IPI_CM_MGR_BCPU_LOW_OPP_BOUND_SET:
		/* FALLTHROUGH */
#endif /* CM_BCPU_MIN_OPP_WEIGHT */
#ifdef DSU_DVFS_ENABLE
		/* FALLTHROUGH */
	case IPI_CM_MGR_DSU_DEBOUNCE_UP_SET:
		/* FALLTHROUGH */
	case IPI_CM_MGR_DSU_DEBOUNCE_DOWN_SET:
		/* FALLTHROUGH */
	case IPI_CM_MGR_DSU_DIFF_PWR_UP_SET:
		/* FALLTHROUGH */
	case IPI_CM_MGR_DSU_DIFF_PWR_DOWN_SET:
		/* FALLTHROUGH */
	case IPI_CM_MGR_DSU_L_PWR_RATIO_SET:
		/* FALLTHROUGH */
	case IPI_CM_MGR_DSU_B_PWR_RATIO_SET:
		/* FALLTHROUGH */
#ifdef CM_TRIGEAR
	case IPI_CM_MGR_DSU_BB_PWR_RATIO_SET:
		/* FALLTHROUGH */
#endif
#endif /* DSU_DVFS_ENABLE */
		cm_mgr_d.cmd = cmd;
		cm_mgr_d.arg = val;
		ret = mtk_ipi_send_compl(&sspm_ipidev, IPIS_C_CM,
		IPI_SEND_POLLING, &cm_mgr_d, CM_MGR_D_LEN, 2000);
		if (ret != 0) {
			pr_debug("#@# %s(%d) cmd(%d) error, return %d\n",
					__func__, __LINE__, cmd, ret);
		} else if (!cm_ipi_ackdata) {
			ret = cm_ipi_ackdata;
			pr_debug("#@# %s(%d) cmd(%d) ack fail %d\n",
					__func__, __LINE__, cmd, ret);
		}
	break;
	default:
		pr_debug("#@# %s(%d) wrong cmd(%d)!!!\n",
			__func__, __LINE__, cmd);
	break;
	}

	return ret;
}
#else
int cm_mgr_to_sspm_command(u32 cmd, int val)
{
	unsigned int ret = 0;
	struct cm_mgr_data cm_mgr_d;
	int ack_data;

	switch (cmd) {
	case IPI_CM_MGR_INIT:
	case IPI_CM_MGR_ENABLE:
	case IPI_CM_MGR_OPP_ENABLE:
	case IPI_CM_MGR_SSPM_ENABLE:
	case IPI_CM_MGR_BLANK:
	case IPI_CM_MGR_DISABLE_FB:
	case IPI_CM_MGR_DRAM_TYPE:
	case IPI_CM_MGR_CPU_POWER_RATIO_UP:
	case IPI_CM_MGR_CPU_POWER_RATIO_DOWN:
	case IPI_CM_MGR_VCORE_POWER_RATIO_UP:
	case IPI_CM_MGR_VCORE_POWER_RATIO_DOWN:
	case IPI_CM_MGR_DEBOUNCE_UP:
	case IPI_CM_MGR_DEBOUNCE_DOWN:
	case IPI_CM_MGR_DEBOUNCE_TIMES_RESET_ADB:
	case IPI_CM_MGR_DRAM_LEVEL:
	case IPI_CM_MGR_LIGHT_LOAD_CPS:
	case IPI_CM_MGR_LOADING_ENABLE:
	case IPI_CM_MGR_LOADING_LEVEL:
	case IPI_CM_MGR_EMI_DEMAND_CHECK:
	case IPI_CM_MGR_BCPU_WEIGHT_MAX_SET:
	case IPI_CM_MGR_BCPU_WEIGHT_MIN_SET:
#ifdef CM_BCPU_MIN_OPP_WEIGHT
		/* FALLTHROUGH */
	case IPI_CM_MGR_BCPU_MIN_OPP_WEIGHT_SET:
		/* FALLTHROUGH */
	case IPI_CM_MGR_BCPU_LOW_OPP_WEIGHT_SET:
		/* FALLTHROUGH */
	case IPI_CM_MGR_BCPU_LOW_OPP_BOUND_SET:
		/* FALLTHROUGH */
#endif /* CM_BCPU_MIN_OPP_WEIGHT */
#ifdef CM_TRIGEAR
	/* FALLTHROUGH */
	case IPI_CM_MGR_BBCPU_WEIGHT_MAX_SET:
		/* FALLTHROUGH */
	case IPI_CM_MGR_BBCPU_WEIGHT_MIN_SET:
		/* FALLTHROUGH */
#endif
#ifdef DSU_DVFS_ENABLE
		/* FALLTHROUGH */
	case IPI_CM_MGR_DSU_DEBOUNCE_UP_SET:
		/* FALLTHROUGH */
	case IPI_CM_MGR_DSU_DEBOUNCE_DOWN_SET:
		/* FALLTHROUGH */
	case IPI_CM_MGR_DSU_DIFF_PWR_UP_SET:
		/* FALLTHROUGH */
	case IPI_CM_MGR_DSU_DIFF_PWR_DOWN_SET:
		/* FALLTHROUGH */
	case IPI_CM_MGR_DSU_L_PWR_RATIO_SET:
		/* FALLTHROUGH */
	case IPI_CM_MGR_DSU_B_PWR_RATIO_SET:
		/* FALLTHROUGH */
#ifdef CM_TRIGEAR
	case IPI_CM_MGR_DSU_BB_PWR_RATIO_SET:
		/* FALLTHROUGH */
#endif
#endif /* DSU_DVFS_ENABLE */
		cm_mgr_d.cmd = cmd;
		cm_mgr_d.arg = val;
		ret = sspm_ipi_send_sync(IPI_ID_CM, IPI_OPT_POLLING,
				&cm_mgr_d, CM_MGR_D_LEN, &ack_data, 1);
		if (ret != 0) {
			pr_debug("#@# %s(%d) cmd(%d) error, return %d\n",
					__func__, __LINE__, cmd, ret);
		} else if (ack_data < 0) {
			ret = ack_data;
			pr_debug("#@# %s(%d) cmd(%d) return %d\n",
					__func__, __LINE__, cmd, ret);
		}
		break;
	default:
		pr_debug("#@# %s(%d) wrong cmd(%d)!!!\n",
				__func__, __LINE__, cmd);
		break;
	}

	return ret;
}
#endif
#endif /* CONFIG_MTK_TINYSYS_SSPM_SUPPORT */

static int dbg_cm_mgr_status_proc_show(struct seq_file *m, void *v)
{
#ifdef CONFIG_ARM64
	seq_printf(m, "diff/cnt/max/avg = %llu/%llu/%d/%llu\n",
			test_diff, cnt, test_max, test_diff / cnt);
#else
	seq_printf(m, "diff/cnt/max/avg = %llu/%llu/%d/%u\n",
			test_diff, cnt, test_max, do_div(test_diff, cnt));
#endif /* CONFIG_ARM64 */
	seq_printf(m, "< 50us    = %d\n", time_cnt_data[0]);
	seq_printf(m, "50~99us   = %d\n", time_cnt_data[1]);
	seq_printf(m, "100~199us = %d\n", time_cnt_data[2]);
	seq_printf(m, "200~299us = %d\n", time_cnt_data[3]);
	seq_printf(m, ">= 300us  = %d\n", time_cnt_data[4]);
	time_cnt_data[0] = 0;
	time_cnt_data[1] = 0;
	time_cnt_data[2] = 0;
	time_cnt_data[3] = 0;
	time_cnt_data[4] = 0;
	test_diff = 0;
	cnt = 0;
	test_max = 0;

	return 0;
}

PROC_FOPS_RO(dbg_cm_mgr_status);

void __weak dbg_cm_mgr_platform_show(struct seq_file *m) {}

void __weak dbg_cm_mgr_platform_write(int len, const char *cmd, u32 val_1,
	u32 val_2)
{}

#ifdef USE_CPU_TO_DRAM_MAP_NEW
void cm_mgr_cpu_map_update_table(void)
{
	int i;

	for (i = 0; i < CM_MGR_CPU_OPP_SIZE; i++) {
		if (i < cm_mgr_cpu_map_skip_cpu_opp)
			cm_mgr_cpu_opp_to_dram[i] = cm_mgr_cpu_map_emi_opp;
		else
			cm_mgr_cpu_opp_to_dram[i] =
				PM_QOS_DDR_OPP_DEFAULT_VALUE;
	}
}
#endif /* USE_CPU_TO_DRAM_MAP_NEW */

static int dbg_cm_mgr_proc_show(struct seq_file *m, void *v)
{
	int i, j;
	int count;

	seq_printf(m, "cm_mgr_opp_enable %d\n", cm_mgr_opp_enable);
	seq_printf(m, "cm_mgr_enable %d\n", cm_mgr_enable);
#if defined(CONFIG_MTK_TINYSYS_SSPM_SUPPORT) && defined(USE_CM_MGR_AT_SSPM)
	seq_printf(m, "cm_mgr_sspm_enable %d\n", cm_mgr_sspm_enable);
#endif /* CONFIG_MTK_TINYSYS_SSPM_SUPPORT */
#ifdef USE_TIMER_CHECK
	seq_printf(m, "cm_mgr_timer_enable %d\n", cm_mgr_timer_enable);
#endif /* USE_TIMER_CHECK */
	seq_printf(m, "cm_mgr_ratio_timer_enable %d\n",
			cm_mgr_ratio_timer_enable);
	seq_printf(m, "cm_mgr_perf_enable %d\n",
			cm_mgr_perf_enable);
	seq_printf(m, "cm_mgr_perf_timer_enable %d\n",
			cm_mgr_perf_timer_enable);
	seq_printf(m, "cm_mgr_perf_force_enable %d\n",
			cm_mgr_perf_force_enable);
#ifdef CM_BCPU_MIN_OPP_WEIGHT
	seq_printf(m, "cm_mgr_bcpu_min_opp_weight %d\n",
			cm_mgr_bcpu_min_opp_weight);
	seq_printf(m, "cm_mgr_bcpu_low_opp_weight %d\n",
			cm_mgr_bcpu_low_opp_weight);
	seq_printf(m, "cm_mgr_bcpu_low_opp_bound %d\n",
			cm_mgr_bcpu_low_opp_bound);
#endif
#ifdef USE_CM_USER_MODE
	seq_printf(m, "cm_user_mode %d\n",
			cm_user_mode);
	seq_printf(m, "cm_user_active %d\n",
			cm_user_active);
#endif
#ifdef USE_CPU_TO_DRAM_MAP
	seq_printf(m, "cm_mgr_cpu_map_dram_enable %d\n",
			cm_mgr_cpu_map_dram_enable);
#endif /* USE_CPU_TO_DRAM_MAP */
#ifdef USE_CPU_TO_DRAM_MAP_NEW
	seq_printf(m, "cm_mgr_cpu_map_emi_opp %d\n",
			cm_mgr_cpu_map_emi_opp);
	seq_printf(m, "cm_mgr_cpu_map_skip_cpu_opp %d\n",
			cm_mgr_cpu_map_skip_cpu_opp);
	seq_puts(m, "cm_mgr_cpu_opp_to_dram table");
	for (i = 0; i < CM_MGR_CPU_OPP_SIZE; i++)
		seq_printf(m, " %d", cm_mgr_cpu_opp_to_dram[i]);
	seq_puts(m, "\n");
#endif /* USE_CPU_TO_DRAM_MAP_NEW */
#ifdef USE_STEP_PERF_OPP
	seq_printf(m, "cm_mgr_dram_perf_opp %d\n",
			cm_mgr_dram_perf_opp);
	seq_printf(m, "cm_mgr_dram_step_opp %d\n",
			cm_mgr_dram_step_opp);
#endif /* USE_STEP_PERF_OPP */
	seq_printf(m, "cm_mgr_disable_fb %d\n", cm_mgr_disable_fb);
	seq_printf(m, "light_load_cps %d\n", light_load_cps);
	seq_printf(m, "total_bw_value %d\n", total_bw_value);
	seq_printf(m, "cm_mgr_loop_count %d\n", cm_mgr_loop_count);
	seq_printf(m, "cm_mgr_dram_level %d\n", cm_mgr_dram_level);
	seq_printf(m, "cm_mgr_loading_level %d\n", cm_mgr_loading_level);
	seq_printf(m, "cm_mgr_loading_enable %d\n", cm_mgr_loading_enable);
	seq_printf(m, "cm_mgr_emi_demand_check %d\n", cm_mgr_emi_demand_check);

#ifdef DEBUG_CM_PERF_OPP
	seq_printf(m, "perfx cm_mgr_dram_opp %d (base: %d)\n",
		cm_mgr_dram_opp_base, cm_mgr_dram_opp);
	seq_printf(m, "perfx debounce_time_down_local %d (force: %d)\n",
		debounce_times_perf_down_local,
		debounce_times_perf_down_force_local);
	seq_printf(m, "perfx pm_qos_update_request_status %d\n",
		pm_qos_update_request_status);
	seq_printf(m, "perfx cm_mgr_blank_status %d\n", cm_mgr_blank_status);
#endif
	seq_puts(m, "cpu_power_ratio_up");
	for (i = 0; i < CM_MGR_EMI_OPP; i++)
		seq_printf(m, " %d", cpu_power_ratio_up[i]);
	seq_puts(m, "\n");

	seq_puts(m, "cpu_power_ratio_down");
	for (i = 0; i < CM_MGR_EMI_OPP; i++)
		seq_printf(m, " %d", cpu_power_ratio_down[i]);
	seq_puts(m, "\n");

	seq_puts(m, "vcore_power_ratio_up");
	for (i = 0; i < CM_MGR_EMI_OPP; i++)
		seq_printf(m, " %d", vcore_power_ratio_up[i]);
	seq_puts(m, "\n");

	seq_puts(m, "vcore_power_ratio_down");
	for (i = 0; i < CM_MGR_EMI_OPP; i++)
		seq_printf(m, " %d", vcore_power_ratio_down[i]);
	seq_puts(m, "\n");

#ifdef USE_BCPU_WEIGHT
	seq_printf(m, "cpu_power_bcpu_weight_max %d\n",
			cpu_power_bcpu_weight_max);
	seq_printf(m, "cpu_power_bcpu_weight_min %d\n",
			cpu_power_bcpu_weight_min);
#endif /* USE_BCPU_WEIGHT */
#ifdef CM_TRIGEAR
	seq_printf(m, "cpu_power_bbcpu_weight_max %d\n",
			cpu_power_bbcpu_weight_max);
	seq_printf(m, "cpu_power_bbcpu_weight_min %d\n",
			cpu_power_bbcpu_weight_min);
#endif
#ifdef DSU_DVFS_ENABLE
	seq_printf(m, "dsu_dvfs_debounce_up %d\n",
			dsu_debounce_up);
	seq_printf(m, "dsu_dvfs_debounce_down %d\n",
			dsu_debounce_down);
	seq_printf(m, "dsu_dvfs_diff_pwr_up %d\n",
			dsu_diff_pwr_up);
	seq_printf(m, "dsu_dvfs_diff_pwr_down %d\n",
			dsu_diff_pwr_down);
	seq_printf(m, "dsu_dvfs_l_pwr_ratio %d\n",
			dsu_l_pwr_ratio);
	seq_printf(m, "dsu_dvfs_b_pwr_ratio %d\n",
			dsu_b_pwr_ratio);
#ifdef CM_TRIGEAR
	seq_printf(m, "dsu_dvfs_bb_pwr_ratio %d\n",
			dsu_bb_pwr_ratio);
#endif
#endif
	seq_puts(m, "debounce_times_up_adb");
	for (i = 0; i < CM_MGR_EMI_OPP; i++)
		seq_printf(m, " %d", debounce_times_up_adb[i]);
	seq_puts(m, "\n");

	seq_puts(m, "debounce_times_down_adb");
	for (i = 0; i < CM_MGR_EMI_OPP; i++)
		seq_printf(m, " %d", debounce_times_down_adb[i]);
	seq_puts(m, "\n");

	seq_printf(m, "debounce_times_reset_adb %d\n",
			debounce_times_reset_adb);
	seq_printf(m, "debounce_times_perf_down %d\n",
			debounce_times_perf_down);
	seq_printf(m, "debounce_times_perf_force_down %d\n",
			debounce_times_perf_force_down);
	seq_printf(m, "update_v2f_table %d\n", update_v2f_table);
	seq_printf(m, "update %d\n", update);

	for (count = 0; count < CM_MGR_MAX; count++) {
		seq_printf(m, "vcore_power_gain_%d\n", count);
		for (i = 0; i < vcore_power_array_size(count); i++) {
			seq_printf(m, "%.2d -", i);
			for (j = 0; j < VCORE_ARRAY_SIZE; j++)
				seq_printf(m, " %d",
						vcore_power_gain(
						vcore_power_gain_ptr(
							count), i, j));
			seq_puts(m, "\n");
		}
	}

#ifdef PER_CPU_STALL_RATIO
#ifndef USE_CM_POWER_ARGS
	for (count = 0; count < CM_MGR_MAX; count++) {
		cpu_power_gain_ptr(0, count, 0);

		seq_printf(m, "cpu_power_gain_UpLow%d\n", count);
		for (i = 0; i < 20; i++) {
			seq_printf(m, "%.2d -", i);
			for (j = 0; j < CM_MGR_CPU_ARRAY_SIZE; j++)
				seq_printf(m, " %d",
						cpu_power_gain(
							cpu_power_gain_up,
							i, j));
			seq_puts(m, "\n");
		}

		seq_printf(m, "cpu_power_gain_DownLow%d\n", count);
		for (i = 0; i < 20; i++) {
			seq_printf(m, "%.2d -", i);
			for (j = 0; j < CM_MGR_CPU_ARRAY_SIZE; j++)
				seq_printf(m, " %d",
						cpu_power_gain(
							cpu_power_gain_down,
							i, j));
			seq_puts(m, "\n");
		}

		cpu_power_gain_ptr(0, CM_MGR_LOWER_OPP, 0);

		seq_printf(m, "cpu_power_gain_UpHigh%d\n", count);
		for (i = 0; i < 20; i++) {
			seq_printf(m, "%.2d -", i);
			for (j = 0; j < CM_MGR_CPU_ARRAY_SIZE; j++)
				seq_printf(m, " %d",
						cpu_power_gain(
							cpu_power_gain_up,
							i, j));
			seq_puts(m, "\n");
		}

		seq_printf(m, "cpu_power_gain_DownHigh%d\n", count);
		for (i = 0; i < 20; i++) {
			seq_printf(m, "%.2d -", i);
			for (j = 0; j < CM_MGR_CPU_ARRAY_SIZE; j++)
				seq_printf(m, " %d",
						cpu_power_gain(
							cpu_power_gain_down,
							i, j));
			seq_puts(m, "\n");
		}
	}
#endif /* USE_CM_POWER_ARGS */
#endif /* PER_CPU_STALL_RATIO */

	seq_puts(m, "_v2f_all\n");
	for (i = 0; i < 16; i++) {
		seq_printf(m, "%.2d -", i);
		for (j = 0; j < CM_MGR_CPU_CLUSTER; j++)
			seq_printf(m, " %d", _v2f_all[i][j]);
		seq_puts(m, "\n");
	}

	seq_puts(m, "\n");

	dbg_cm_mgr_platform_show(m);

	return 0;
}

#define CPU_FW_FILE "cpu_data.bin"
#include <linux/firmware.h>
static struct device cm_mgr_device = {
	.init_name = "cm_mgr_device",
};

static void cm_mgr_update_fw(void)
{
	int j = 0;
	const struct firmware *fw = NULL;
	int err;
	int copy_size = 0;
	int offset = 0;
	int count;

	do {
		j++;
		pr_debug("try to request_firmware() %s - %d\n",
				CPU_FW_FILE, j);
		err = request_firmware(&fw, CPU_FW_FILE, &cm_mgr_device);
		if (err)
			pr_debug("Failed to load %s, err = %d.\n",
					CPU_FW_FILE, err);
	} while (err == -EAGAIN && j < 5);
	if (err)
		pr_debug("Failed to load %s, err = %d.\n",
				CPU_FW_FILE, err);

	if (!err) {
		pr_debug("request_firmware() %s, size 0x%x\n",
				CPU_FW_FILE, (int)fw->size);
		update++;

		for (count = 0; count < CM_MGR_MAX; count++) {
			copy_size = vcore_power_array_size(count) *
				VCORE_ARRAY_SIZE * sizeof(unsigned int);
			pr_debug("offset 0x%x, copy_size 0x%x\n",
					offset, copy_size);
			if (fw->size < (copy_size + offset)) {
				pr_debug("vcore_power_gain_%d 0x%x, 0x%x",
						count, (int)fw->size,
						copy_size + offset);
				goto out_fw;
			}
			memcpy(vcore_power_gain_ptr(count),
					fw->data, copy_size);
		}

#ifdef PER_CPU_STALL_RATIO
		for (count = 0; count < CM_MGR_MAX; count++) {
			unsigned int *ptr;

			offset += copy_size;
			copy_size = sizeof(cpu_power_gain_UpLow0);
			pr_debug("offset 0x%x, copy_size 0x%x\n",
					offset, copy_size);
			if (fw->size < (copy_size + offset)) {
				pr_debug("cpu_power_gain_UpLow%d 0x%x, 0x%x",
						count,
						(int)fw->size,
						copy_size + offset);
				goto out_fw;
			}
			ptr = _cpu_power_gain_ptr(1, 1, count);
			if (ptr)
				memcpy(ptr, fw->data + offset, copy_size);

			offset += copy_size;
			copy_size = sizeof(cpu_power_gain_DownLow0);
			pr_debug("offset 0x%x, copy_size 0x%x\n",
					offset, copy_size);
			if (fw->size < (copy_size + offset)) {
				pr_debug("cpu_power_gain_DownLow%d 0x%x, 0x%x",
						count,
						(int)fw->size,
						copy_size + offset);
				goto out_fw;
			}
			ptr = _cpu_power_gain_ptr(0, 1, count);
			if (ptr)
				memcpy(ptr, fw->data + offset, copy_size);

			offset += copy_size;
			copy_size = sizeof(cpu_power_gain_UpHigh0);
			pr_debug("offset 0x%x, copy_size 0x%x\n",
					offset, copy_size);
			if (fw->size < (copy_size + offset)) {
				pr_debug("cpu_power_gain_UpHigh%d 0x%x, 0x%x",
						count,
						(int)fw->size,
						copy_size + offset);
				goto out_fw;
			}
			ptr = _cpu_power_gain_ptr(1, 0, count);
			if (ptr)
				memcpy(ptr, fw->data + offset, copy_size);

			offset += copy_size;
			copy_size = sizeof(cpu_power_gain_DownHigh0);
			pr_debug("offset 0x%x, copy_size 0x%x\n",
					offset, copy_size);
			if (fw->size < (copy_size + offset)) {
				pr_debug("cpu_power_gain_DownHigh%d 0x%x, 0x%x",
						count,
						(int)fw->size,
						copy_size + offset);
				goto out_fw;
			}
			ptr = _cpu_power_gain_ptr(0, 0, count);
			if (ptr)
				memcpy(ptr, fw->data + offset, copy_size);
		}
#endif /* PER_CPU_STALL_RATIO */

		offset += copy_size;
		copy_size = sizeof(_v2f_all);
		pr_debug("offset 0x%x, copy_size 0x%x\n", offset, copy_size);
		if (fw->size < (copy_size + offset)) {
			pr_debug("_v2f_all 0x%x, 0x%x",
					(int)fw->size, copy_size + offset);
			goto out_fw;
		}
		memcpy(&_v2f_all, fw->data + offset, copy_size);

		release_firmware(fw);
		fw = NULL;
	}
out_fw:
	if (fw)
		release_firmware(fw);
}

static ssize_t dbg_cm_mgr_proc_write(struct file *file,
		const char __user *buffer, size_t count, loff_t *pos)
{

	int ret;
	char *buf = (char *) __get_free_page(GFP_USER);
	char cmd[64];
	u32 val_1;
	u32 val_2;

	if (!buf)
		return -ENOMEM;

	ret = -EINVAL;

	if (count >= PAGE_SIZE)
		goto out;

	ret = -EFAULT;

	if (copy_from_user(buf, buffer, count))
		goto out;

	buf[count] = '\0';

	ret = sscanf(buf, "%63s %d %d", cmd, &val_1, &val_2);
	if (ret < 1) {
		ret = -EPERM;
		goto out;
	}

#ifdef USE_CM_USER_MODE
	if (!strcmp(cmd, "cm_user_mode")) {
		cm_mgr_user_mode_set(val_1);
		goto out;
	}

	if (cm_user_mode) {
		if (cm_user_active)
			cm_mgr_user_mode_cmd(0, cmd, val_1, val_2);
		else
			pr_debug("skip cmd:%s for cm_user_mode: %d, active: %d\n",
				cmd, cm_user_mode, cm_user_active);
		goto out;
	}
#endif

	if (!strcmp(cmd, "cm_mgr_opp_enable")) {
		cm_mgr_opp_enable = val_1;
		if (!cm_mgr_opp_enable)
			cm_mgr_set_dram_level(-1);
#if defined(CONFIG_MTK_TINYSYS_SSPM_SUPPORT) && defined(USE_CM_MGR_AT_SSPM)
		cm_mgr_to_sspm_command(IPI_CM_MGR_OPP_ENABLE,
				cm_mgr_opp_enable);
#endif /* CONFIG_MTK_TINYSYS_SSPM_SUPPORT */
	} else if (!strcmp(cmd, "cm_mgr_enable")) {
		cm_mgr_enable = val_1;
		if (!cm_mgr_enable)
			cm_mgr_set_dram_level(-1);
#if defined(CONFIG_MTK_TINYSYS_SSPM_SUPPORT) && defined(USE_CM_MGR_AT_SSPM)
		cm_mgr_to_sspm_command(IPI_CM_MGR_ENABLE,
				cm_mgr_enable);
	} else if (!strcmp(cmd, "cm_mgr_sspm_enable")) {
		cm_mgr_sspm_enable = val_1;
		cm_mgr_to_sspm_command(IPI_CM_MGR_SSPM_ENABLE,
				cm_mgr_sspm_enable);
#endif /* CONFIG_MTK_TINYSYS_SSPM_SUPPORT */
#ifdef USE_TIMER_CHECK
	} else if (!strcmp(cmd, "cm_mgr_timer_enable")) {
		cm_mgr_timer_enable = val_1;
#endif /* USE_TIMER_CHECK */
	} else if (!strcmp(cmd, "cm_mgr_ratio_timer_enable")) {
		cm_mgr_ratio_timer_enable = val_1;
		cm_mgr_ratio_timer_en(val_1);
	} else if (!strcmp(cmd, "cm_mgr_perf_enable")) {
		cm_mgr_perf_enable = val_1;
	} else if (!strcmp(cmd, "cm_mgr_perf_timer_enable")) {
		cm_mgr_perf_timer_enable = val_1;
		cm_mgr_perf_set_status(val_1);
	} else if (!strcmp(cmd, "cm_mgr_perf_force_enable")) {
		cm_mgr_perf_force_enable = val_1;
		cm_mgr_perf_set_force_status(val_1);
	} else if (!strcmp(cmd, "cm_mgr_perf_set_status")) {
		cm_mgr_perf_set_status(!!val_1);
	} else if (!strcmp(cmd, "cm_mgr_disable_fb")) {
		cm_mgr_disable_fb = val_1;
		if (cm_mgr_disable_fb == 1 && cm_mgr_blank_status == 1)
			cm_mgr_set_dram_level(-1);
#if defined(CONFIG_MTK_TINYSYS_SSPM_SUPPORT) && defined(USE_CM_MGR_AT_SSPM)
		cm_mgr_to_sspm_command(IPI_CM_MGR_DISABLE_FB,
				cm_mgr_disable_fb);
#endif /* CONFIG_MTK_TINYSYS_SSPM_SUPPORT */
	} else if (!strcmp(cmd, "light_load_cps")) {
		light_load_cps = val_1;
#if defined(CONFIG_MTK_TINYSYS_SSPM_SUPPORT) && defined(USE_CM_MGR_AT_SSPM)
		cm_mgr_to_sspm_command(IPI_CM_MGR_LIGHT_LOAD_CPS, val_1);
#endif /* CONFIG_MTK_TINYSYS_SSPM_SUPPORT */
	} else if (!strcmp(cmd, "total_bw_value")) {
		total_bw_value = val_1;
	} else if (!strcmp(cmd, "cm_mgr_loop_count")) {
		cm_mgr_loop_count = val_1;
	} else if (!strcmp(cmd, "cm_mgr_dram_level")) {
		cm_mgr_dram_level = val_1;
#if defined(CONFIG_MTK_TINYSYS_SSPM_SUPPORT) && defined(USE_CM_MGR_AT_SSPM)
		cm_mgr_to_sspm_command(IPI_CM_MGR_DRAM_LEVEL, val_1);
#else
		cm_mgr_set_dram_level(val_1);
#endif /* CONFIG_MTK_TINYSYS_SSPM_SUPPORT */
	} else if (!strcmp(cmd, "cm_mgr_loading_level")) {
		cm_mgr_loading_level = val_1;
#if defined(CONFIG_MTK_TINYSYS_SSPM_SUPPORT) && defined(USE_CM_MGR_AT_SSPM)
		cm_mgr_to_sspm_command(IPI_CM_MGR_LOADING_LEVEL, val_1);
#endif /* CONFIG_MTK_TINYSYS_SSPM_SUPPORT */
	} else if (!strcmp(cmd, "cm_mgr_loading_enable")) {
		cm_mgr_loading_enable = val_1;
#if defined(CONFIG_MTK_TINYSYS_SSPM_SUPPORT) && defined(USE_CM_MGR_AT_SSPM)
		cm_mgr_to_sspm_command(IPI_CM_MGR_LOADING_ENABLE, val_1);
#endif /* CONFIG_MTK_TINYSYS_SSPM_SUPPORT */
	} else if (!strcmp(cmd, "cm_mgr_emi_demand_check")) {
		cm_mgr_emi_demand_check = val_1;
#if defined(CONFIG_MTK_TINYSYS_SSPM_SUPPORT) && defined(USE_CM_MGR_AT_SSPM)
		cm_mgr_to_sspm_command(IPI_CM_MGR_EMI_DEMAND_CHECK, val_1);
#endif /* CONFIG_MTK_TINYSYS_SSPM_SUPPORT */
	} else if (!strcmp(cmd, "cpu_power_ratio_up")) {
		if (ret == 3 && val_1 < CM_MGR_EMI_OPP)
			cpu_power_ratio_up[val_1] = val_2;
#if defined(CONFIG_MTK_TINYSYS_SSPM_SUPPORT) && defined(USE_CM_MGR_AT_SSPM)
		cm_mgr_to_sspm_command(IPI_CM_MGR_CPU_POWER_RATIO_UP,
				val_1 << 16 | val_2);
#endif /* CONFIG_MTK_TINYSYS_SSPM_SUPPORT */
	} else if (!strcmp(cmd, "cpu_power_ratio_down")) {
		if (ret == 3 && val_1 < CM_MGR_EMI_OPP)
			cpu_power_ratio_down[val_1] = val_2;
#if defined(CONFIG_MTK_TINYSYS_SSPM_SUPPORT) && defined(USE_CM_MGR_AT_SSPM)
		cm_mgr_to_sspm_command(IPI_CM_MGR_CPU_POWER_RATIO_DOWN,
				val_1 << 16 | val_2);
#endif /* CONFIG_MTK_TINYSYS_SSPM_SUPPORT */
	} else if (!strcmp(cmd, "vcore_power_ratio_up")) {
		if (ret == 3 && val_1 < CM_MGR_EMI_OPP)
			vcore_power_ratio_up[val_1] = val_2;
#if defined(CONFIG_MTK_TINYSYS_SSPM_SUPPORT) && defined(USE_CM_MGR_AT_SSPM)
		cm_mgr_to_sspm_command(IPI_CM_MGR_VCORE_POWER_RATIO_UP,
				val_1 << 16 | val_2);
#endif /* CONFIG_MTK_TINYSYS_SSPM_SUPPORT */
	} else if (!strcmp(cmd, "vcore_power_ratio_down")) {
		if (ret == 3 && val_1 < CM_MGR_EMI_OPP)
			vcore_power_ratio_down[val_1] = val_2;
#if defined(CONFIG_MTK_TINYSYS_SSPM_SUPPORT) && defined(USE_CM_MGR_AT_SSPM)
		cm_mgr_to_sspm_command(IPI_CM_MGR_VCORE_POWER_RATIO_DOWN,
				val_1 << 16 | val_2);
#endif /* CONFIG_MTK_TINYSYS_SSPM_SUPPORT */
	} else if (!strcmp(cmd, "debounce_times_up_adb")) {
		if (ret == 3 && val_1 < CM_MGR_EMI_OPP)
			debounce_times_up_adb[val_1] = val_2;
#if defined(CONFIG_MTK_TINYSYS_SSPM_SUPPORT) && defined(USE_CM_MGR_AT_SSPM)
		cm_mgr_to_sspm_command(IPI_CM_MGR_DEBOUNCE_UP,
				val_1 << 16 | val_2);
#endif /* CONFIG_MTK_TINYSYS_SSPM_SUPPORT */
	} else if (!strcmp(cmd, "debounce_times_down_adb")) {
		if (ret == 3 && val_1 < CM_MGR_EMI_OPP)
			debounce_times_down_adb[val_1] = val_2;
#if defined(CONFIG_MTK_TINYSYS_SSPM_SUPPORT) && defined(USE_CM_MGR_AT_SSPM)
		cm_mgr_to_sspm_command(IPI_CM_MGR_DEBOUNCE_DOWN,
				val_1 << 16 | val_2);
#endif /* CONFIG_MTK_TINYSYS_SSPM_SUPPORT */
	} else if (!strcmp(cmd, "debounce_times_reset_adb")) {
		debounce_times_reset_adb = val_1;
#if defined(CONFIG_MTK_TINYSYS_SSPM_SUPPORT) && defined(USE_CM_MGR_AT_SSPM)
		cm_mgr_to_sspm_command(IPI_CM_MGR_DEBOUNCE_TIMES_RESET_ADB,
				debounce_times_reset_adb);
#endif /* CONFIG_MTK_TINYSYS_SSPM_SUPPORT */
#ifdef USE_BCPU_WEIGHT
	} else if (!strcmp(cmd, "cpu_power_bcpu_weight_max")) {
		if (cpu_power_bcpu_weight_max < cpu_power_bcpu_weight_min) {
			ret = -1;
		} else {
			cpu_power_bcpu_weight_max = val_1;
#if defined(CONFIG_MTK_TINYSYS_SSPM_SUPPORT) && defined(USE_CM_MGR_AT_SSPM)
			cm_mgr_to_sspm_command(IPI_CM_MGR_BCPU_WEIGHT_MAX_SET,
					val_1);
#endif /* CONFIG_MTK_TINYSYS_SSPM_SUPPORT */
		}
	} else if (!strcmp(cmd, "cpu_power_bcpu_weight_min")) {
		if (cpu_power_bcpu_weight_max < cpu_power_bcpu_weight_min) {
			ret = -1;
		} else {
			cpu_power_bcpu_weight_min = val_1;
#if defined(CONFIG_MTK_TINYSYS_SSPM_SUPPORT) && defined(USE_CM_MGR_AT_SSPM)
			cm_mgr_to_sspm_command(IPI_CM_MGR_BCPU_WEIGHT_MIN_SET,
					val_1);
#endif /* CONFIG_MTK_TINYSYS_SSPM_SUPPORT */
		}
#endif /* USE_BCPU_WEIGHT */
#ifdef CM_BCPU_MIN_OPP_WEIGHT
	} else if (!strcmp(cmd, "cm_mgr_bcpu_min_opp_weight")) {
		cm_mgr_bcpu_min_opp_weight = val_1;
#if defined(CONFIG_MTK_TINYSYS_SSPM_SUPPORT) && defined(USE_CM_MGR_AT_SSPM)
		cm_mgr_to_sspm_command(IPI_CM_MGR_BCPU_MIN_OPP_WEIGHT_SET,
				val_1);
#endif /* CONFIG_MTK_TINYSYS_SSPM_SUPPORT */
	} else if (!strcmp(cmd, "cm_mgr_bcpu_low_opp_weight")) {
		cm_mgr_bcpu_low_opp_weight = val_1;
#if defined(CONFIG_MTK_TINYSYS_SSPM_SUPPORT) && defined(USE_CM_MGR_AT_SSPM)
		cm_mgr_to_sspm_command(IPI_CM_MGR_BCPU_LOW_OPP_WEIGHT_SET,
				val_1);
#endif /* CONFIG_MTK_TINYSYS_SSPM_SUPPORT */
	} else if (!strcmp(cmd, "cm_mgr_bcpu_low_opp_bound")) {
		cm_mgr_bcpu_low_opp_bound = val_1;
#if defined(CONFIG_MTK_TINYSYS_SSPM_SUPPORT) && defined(USE_CM_MGR_AT_SSPM)
		cm_mgr_to_sspm_command(IPI_CM_MGR_BCPU_LOW_OPP_BOUND_SET,
				val_1);
#endif /* CONFIG_MTK_TINYSYS_SSPM_SUPPORT */
#endif /* CM_BCPU_MIN_OPP_WEIGHT */
#ifdef CM_TRIGEAR
	} else if (!strcmp(cmd, "cpu_power_bbcpu_weight_max")) {
		if (cpu_power_bbcpu_weight_max < cpu_power_bbcpu_weight_min) {
			ret = -1;
		} else {
			cpu_power_bbcpu_weight_max = val_1;
#if defined(CONFIG_MTK_TINYSYS_SSPM_SUPPORT) && defined(USE_CM_MGR_AT_SSPM)
			cm_mgr_to_sspm_command(IPI_CM_MGR_BBCPU_WEIGHT_MAX_SET,
					val_1);
#endif /* CONFIG_MTK_TINYSYS_SSPM_SUPPORT */
		}
	} else if (!strcmp(cmd, "cpu_power_bbcpu_weight_min")) {
		if (cpu_power_bbcpu_weight_max < cpu_power_bbcpu_weight_min) {
			ret = -1;
		} else {
			cpu_power_bbcpu_weight_min = val_1;
#if defined(CONFIG_MTK_TINYSYS_SSPM_SUPPORT) && defined(USE_CM_MGR_AT_SSPM)
			cm_mgr_to_sspm_command(IPI_CM_MGR_BBCPU_WEIGHT_MIN_SET,
					val_1);
#endif /* CONFIG_MTK_TINYSYS_SSPM_SUPPORT */
		}
#endif /* CM_TRIGEAR */
#ifdef DSU_DVFS_ENABLE
	} else if (!strcmp(cmd, "dsu_dvfs_debounce_up")) {
		dsu_debounce_up = val_1;
#if defined(CONFIG_MTK_TINYSYS_SSPM_SUPPORT) && defined(USE_CM_MGR_AT_SSPM)
		cm_mgr_to_sspm_command(IPI_CM_MGR_DSU_DEBOUNCE_UP_SET,
				val_1);
#endif /* CONFIG_MTK_TINYSYS_SSPM_SUPPORT */
	} else if (!strcmp(cmd, "dsu_dvfs_debounce_down")) {
		dsu_debounce_down = val_1;
#if defined(CONFIG_MTK_TINYSYS_SSPM_SUPPORT) && defined(USE_CM_MGR_AT_SSPM)
		cm_mgr_to_sspm_command(IPI_CM_MGR_DSU_DEBOUNCE_DOWN_SET,
				val_1);
#endif /* CONFIG_MTK_TINYSYS_SSPM_SUPPORT */
	} else if (!strcmp(cmd, "dsu_dvfs_diff_pwr_up")) {
		dsu_diff_pwr_up = val_1;
#if defined(CONFIG_MTK_TINYSYS_SSPM_SUPPORT) && defined(USE_CM_MGR_AT_SSPM)
		cm_mgr_to_sspm_command(IPI_CM_MGR_DSU_DIFF_PWR_UP_SET,
				val_1);
#endif /* CONFIG_MTK_TINYSYS_SSPM_SUPPORT */
	} else if (!strcmp(cmd, "dsu_dvfs_diff_pwr_down")) {
		dsu_diff_pwr_down = val_1;
#if defined(CONFIG_MTK_TINYSYS_SSPM_SUPPORT) && defined(USE_CM_MGR_AT_SSPM)
		cm_mgr_to_sspm_command(IPI_CM_MGR_DSU_DIFF_PWR_DOWN_SET,
				val_1);
#endif /* CONFIG_MTK_TINYSYS_SSPM_SUPPORT */
	} else if (!strcmp(cmd, "dsu_dvfs_l_pwr_ratio")) {
		dsu_l_pwr_ratio = val_1;
#if defined(CONFIG_MTK_TINYSYS_SSPM_SUPPORT) && defined(USE_CM_MGR_AT_SSPM)
		cm_mgr_to_sspm_command(IPI_CM_MGR_DSU_L_PWR_RATIO_SET,
				val_1);
#endif /* CONFIG_MTK_TINYSYS_SSPM_SUPPORT */
	} else if (!strcmp(cmd, "dsu_dvfs_b_pwr_ratio")) {
		dsu_b_pwr_ratio = val_1;
#if defined(CONFIG_MTK_TINYSYS_SSPM_SUPPORT) && defined(USE_CM_MGR_AT_SSPM)
		cm_mgr_to_sspm_command(IPI_CM_MGR_DSU_B_PWR_RATIO_SET,
				val_1);
#endif /* CONFIG_MTK_TINYSYS_SSPM_SUPPORT */
#ifdef CM_TRIGEAR
	} else if (!strcmp(cmd, "dsu_dvfs_bb_pwr_ratio")) {
		dsu_bb_pwr_ratio = val_1;
#if defined(CONFIG_MTK_TINYSYS_SSPM_SUPPORT) && defined(USE_CM_MGR_AT_SSPM)
		cm_mgr_to_sspm_command(IPI_CM_MGR_DSU_BB_PWR_RATIO_SET,
				val_1);
#endif /* CONFIG_MTK_TINYSYS_SSPM_SUPPORT */
#endif /* CM_TRIGEAR */
#endif /* DSU_DVFS_ENABLE */
	} else if (!strcmp(cmd, "debounce_times_perf_down")) {
		debounce_times_perf_down = val_1;
	} else if (!strcmp(cmd, "debounce_times_perf_force_down")) {
		debounce_times_perf_force_down = val_1;
	} else if (!strcmp(cmd, "update_v2f_table")) {
		update_v2f_table = !!val_1;
	} else if (!strcmp(cmd, "update")) {
		cm_mgr_update_fw();
	} else if (!strcmp(cmd, "1")) {
		/* cm_mgr_perf_force_enable */
		cm_mgr_perf_force_enable = 1;
		cm_mgr_perf_set_force_status(cm_mgr_perf_force_enable);
	} else if (!strcmp(cmd, "0")) {
		/* cm_mgr_perf_force_enable */
		cm_mgr_perf_force_enable = 0;
		cm_mgr_perf_set_force_status(cm_mgr_perf_force_enable);
#ifdef USE_CPU_TO_DRAM_MAP
	} else if (!strcmp(cmd, "cm_mgr_cpu_map_dram_enable")) {
		cm_mgr_cpu_map_dram_enable = !!val_1;
#endif /* USE_CPU_TO_DRAM_MAP */
#ifdef USE_CPU_TO_DRAM_MAP_NEW
	} else if (!strcmp(cmd, "cm_mgr_cpu_map_skip_cpu_opp")) {
		cm_mgr_cpu_map_skip_cpu_opp = val_1;
		cm_mgr_cpu_map_update_table();
	} else if (!strcmp(cmd, "cm_mgr_cpu_map_emi_opp")) {
		cm_mgr_cpu_map_emi_opp = val_1;
		cm_mgr_cpu_map_update_table();
#endif /* USE_CPU_TO_DRAM_MAP_NEW */
#ifdef USE_STEP_PERF_OPP
	} else if (!strcmp(cmd, "cm_mgr_dram_perf_opp")) {
		cm_mgr_dram_perf_opp = val_1;
	} else if (!strcmp(cmd, "cm_mgr_dram_step_opp")) {
		if (val_1 < 0)
			val_1 = 0;
		cm_mgr_dram_step_opp = val_1;
#endif /* USE_STEP_PERF_OPP */
	} else {
		dbg_cm_mgr_platform_write(ret, cmd, val_1, val_2);
	}

out:
	free_page((unsigned long)buf);

	if (ret < 0)
		return ret;
	return count;
}
MODULE_FIRMWARE(CPU_FW_FILE);

PROC_FOPS_RW(dbg_cm_mgr);

static int create_cm_mgr_debug_fs(void)
{
	int i;
	struct proc_dir_entry *dir = NULL;

	struct pentry {
		const char *name;
		const struct file_operations *fops;
		void *data;
	};

	const struct pentry entries[] = {
		PROC_ENTRY(dbg_cm_mgr_status),
		PROC_ENTRY(dbg_cm_mgr),
	};

	/* create /proc/cm_mgr */
	dir = proc_mkdir("cm_mgr", NULL);
	if (!dir) {
		pr_debug("fail to create /proc/cm_mgr @ %s()\n", __func__);
		return -ENOMEM;
	}

	for (i = 0; i < ARRAY_SIZE(entries); i++) {
		if (!proc_create_data
		    (entries[i].name, 0660,
		     dir, entries[i].fops, entries[i].data))
			pr_debug("%s(), create /proc/cm_mgr/%s failed\n",
					__func__,
				    entries[i].name);
	}

	return 0;
}

int __weak cm_mgr_platform_init(void)
{
	return 0;
}

void __weak cm_mgr_setup_cpu_dvfs_info(void) {}

int __init cm_mgr_module_init(void)
{
	int r;

	r = create_cm_mgr_debug_fs();
	if (r) {
		pr_debug("FAILED TO CREATE DEBUG FILESYSTEM (%d)\n", r);
		return r;
	}

	r = device_register(&cm_mgr_device);
	if (r) {
		pr_debug("FAILED TO CREATE DEVICE(%d)\n", r);
		return r;
	}

	/* SW Governor Report */
	spin_lock_init(&cm_mgr_lock);

#if defined(CONFIG_MTK_TINYSYS_SSPM_SUPPORT) && defined(USE_CM_MGR_AT_SSPM)
#if defined(USE_SSMP_VER_V2)
	r = mtk_ipi_register(&sspm_ipidev, IPIS_C_CM, NULL, NULL,
				(void *) &cm_ipi_ackdata);
	if (r) {
		pr_debug("[SSPM] IPIS_C_CM ipi_register fail, ret %d\n", r);
		cm_sspm_ready = -1;
		return -1;
	}
	pr_debug("SSPM is ready to service CM IPI\n");
	cm_sspm_ready = 1;
#endif
#endif /* CONFIG_MTK_TINYSYS_SSPM_SUPPORT */

	r = cm_mgr_platform_init();
	if (r) {
		pr_debug("FAILED TO INIT PLATFORM(%d)\n", r);
		return r;
	}

	vcore_power_gain = vcore_power_gain_ptr(cm_mgr_get_idx());

	init_timer_deferrable(&cm_mgr_perf_timer);
	cm_mgr_perf_timer.function = cm_mgr_perf_timer_fn;
	cm_mgr_perf_timer.data = 0;

#ifdef USE_TIMER_CHECK
	init_timer_deferrable(&cm_mgr_timer);
	cm_mgr_timer.function = cm_mgr_timer_fn;
	cm_mgr_timer.data = 0;
#endif /* USE_TIMER_CHECK */

#if defined(CONFIG_MTK_TINYSYS_SSPM_SUPPORT) && defined(USE_CM_MGR_AT_SSPM)
	cm_mgr_to_sspm_command(IPI_CM_MGR_INIT, 0);

	cm_mgr_setup_cpu_dvfs_info();

	cm_mgr_to_sspm_command(IPI_CM_MGR_ENABLE,
			cm_mgr_enable);

	cm_mgr_to_sspm_command(IPI_CM_MGR_SSPM_ENABLE,
			cm_mgr_sspm_enable);

	cm_mgr_to_sspm_command(IPI_CM_MGR_EMI_DEMAND_CHECK,
			cm_mgr_emi_demand_check);

	cm_mgr_to_sspm_command(IPI_CM_MGR_LOADING_LEVEL,
			cm_mgr_loading_level);

	cm_mgr_to_sspm_command(IPI_CM_MGR_LOADING_ENABLE,
			cm_mgr_loading_enable);

	cm_mgr_to_sspm_command(IPI_CM_MGR_DEBOUNCE_TIMES_RESET_ADB,
			debounce_times_reset_adb);
#ifdef USE_BCPU_WEIGHT
	cm_mgr_to_sspm_command(IPI_CM_MGR_BCPU_WEIGHT_MAX_SET,
			cpu_power_bcpu_weight_max);

	cm_mgr_to_sspm_command(IPI_CM_MGR_BCPU_WEIGHT_MIN_SET,
			cpu_power_bcpu_weight_min);
#endif /* USE_BCPU_WEIGHT */
#ifdef CM_TRIGEAR
	cm_mgr_to_sspm_command(IPI_CM_MGR_BBCPU_WEIGHT_MAX_SET,
			cpu_power_bbcpu_weight_max);

	cm_mgr_to_sspm_command(IPI_CM_MGR_BBCPU_WEIGHT_MIN_SET,
			cpu_power_bbcpu_weight_min);
#endif
#ifdef DSU_DVFS_ENABLE
	cm_mgr_to_sspm_command(IPI_CM_MGR_DSU_DEBOUNCE_UP_SET,
			dsu_debounce_up);

	cm_mgr_to_sspm_command(IPI_CM_MGR_DSU_DEBOUNCE_DOWN_SET,
			dsu_debounce_down);

	cm_mgr_to_sspm_command(IPI_CM_MGR_DSU_DIFF_PWR_UP_SET,
			dsu_diff_pwr_up);

	cm_mgr_to_sspm_command(IPI_CM_MGR_DSU_DIFF_PWR_DOWN_SET,
			dsu_diff_pwr_down);

	cm_mgr_to_sspm_command(IPI_CM_MGR_DSU_L_PWR_RATIO_SET,
			dsu_l_pwr_ratio);

	cm_mgr_to_sspm_command(IPI_CM_MGR_DSU_B_PWR_RATIO_SET,
			dsu_b_pwr_ratio);
#ifdef CM_TRIGEAR
	cm_mgr_to_sspm_command(IPI_CM_MGR_DSU_BB_PWR_RATIO_SET,
			dsu_bb_pwr_ratio);
#endif /* CM_TRIGEAR */
#endif /* DSU_DVFS_ENABLE */
#endif /* CONFIG_MTK_TINYSYS_SSPM_SUPPORT */

	return 0;
}

late_initcall(cm_mgr_module_init);

MODULE_DESCRIPTION("CM Manager Driver v0.1");
