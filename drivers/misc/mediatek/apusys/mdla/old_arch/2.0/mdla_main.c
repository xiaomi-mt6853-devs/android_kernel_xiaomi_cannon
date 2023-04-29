/*
 * Copyright (C) 2018 MediaTek Inc.
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

#include <linux/semaphore.h>
#include <linux/completion.h>
#include "mdla.h"
#include "gsm.h"
#include "mdla_ioctl.h"
#include "mdla_ion.h"
#include "mdla_trace.h"
#include "mdla_debug.h"
#include "mdla_plat_api.h"
#include "mdla_util.h"
#include "mdla_power_ctrl.h"
#include "mdla_cmd_proc.h"
#ifndef __APUSYS_MDLA_SW_PORTING_WORKAROUND__
#include "apusys_device.h"
#endif
#include "apusys_power.h"
#include "mtk_devinfo.h"

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/ioport.h>
#include <linux/io.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/ioctl.h>
#include <linux/pci.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <asm/mman.h>
#include <linux/dmapool.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/device.h>
#include <linux/workqueue.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/jiffies.h>

#ifdef CONFIG_OF
#include <linux/cpu.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_fdt.h>
#endif

#define DEVICE_NAME "mdlactl"
#define CLASS_NAME  "mdla"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("SS5");
MODULE_DESCRIPTION("MDLA driver");
MODULE_VERSION("2.0");

/* internal function prototypes */
#ifndef __APUSYS_MDLA_SW_PORTING_WORKAROUND__
int apusys_mdla_handler(int type,
		void *hnd, struct apusys_device *dev);
#endif
static irqreturn_t mdla_irq0_handler(int irq, void *dev_id);
static irqreturn_t mdla_irq1_handler(int irq, void *dev_id);


/* TODO: move these global control vaiables into device specific data.
 * to support multiple instance (multiple MDLA).
 */
#define MDLA_TIMEOUT_DEFAULT 6000 /* ms */
#define MDLA_POWEROFF_TIME_DEFAULT 2000 /* ms */
#define MDLA_POLLING_LATENCY (5) /* ms */


u32 mdla_timeout = MDLA_TIMEOUT_DEFAULT;
u32 mdla_e1_detect_timeout = MDLA_POLLING_LATENCY;
u32 mdla_poweroff_time = MDLA_POWEROFF_TIME_DEFAULT;
u32 mdla_dvfs_rand;
u32 mdla_timeout_dbg;

u32 mdla_batch_number;
u32 mdla_preemption_times;
u32 mdla_preemption_debug;
struct mutex wake_lock_mutex = __MUTEX_INITIALIZER(wake_lock_mutex);

struct mdla_dev_worker mdla_dev_workers = {
	.worker = 0,
	.worker_lock = __MUTEX_INITIALIZER(mdla_dev_workers.worker_lock),
};

/*MDLA Multi-Core or per command info*/
struct mdla_dev mdla_devices[] = {
	{
		.mdlaid = 0,
		.mdla_zero_skip_count = 0,
		.mdla_e1_detect_count = 0,
		.async_cmd_id = 0,
		.cmd_lock = __MUTEX_INITIALIZER(mdla_devices[0].cmd_lock),
		.cmd_list_lock =
			__MUTEX_INITIALIZER(mdla_devices[0].cmd_list_lock),
		.power_pdn_work = mdla0_start_power_off,
		.power_lock =  __MUTEX_INITIALIZER(mdla_devices[0].power_lock),
		.cmd_buf_dmp_lock =
			__MUTEX_INITIALIZER(mdla_devices[0].cmd_buf_dmp_lock),
		.cmd_buf_len = 0,
		.cmd_list_cnt = 0,
		.error_bit = 0,
	},
	{
		.mdlaid = 1,
		.mdla_zero_skip_count = 0,
		.mdla_e1_detect_count = 0,
		.async_cmd_id = 0,
		.cmd_lock = __MUTEX_INITIALIZER(mdla_devices[1].cmd_lock),
		.cmd_list_lock =
			__MUTEX_INITIALIZER(mdla_devices[1].cmd_list_lock),
		.power_pdn_work = mdla1_start_power_off,
		.power_lock =  __MUTEX_INITIALIZER(mdla_devices[1].power_lock),
		.cmd_buf_dmp_lock =
			__MUTEX_INITIALIZER(mdla_devices[1].cmd_buf_dmp_lock),
		.cmd_buf_len = 0,
		.cmd_list_cnt = 0,
		.error_bit = 0,
	},
};
struct mdla_irq_desc mdla_irqdesc[] = {
	{.irq = 0, .handler = mdla_irq0_handler,},
	{.irq = 0, .handler = mdla_irq1_handler,}
};

#ifndef __APUSYS_MDLA_SW_PORTING_WORKAROUND__
struct apusys_device apusys_mdla_dev[] = {
	{
		.dev_type = APUSYS_DEVICE_MDLA,
		.preempt_type = APUSYS_PREEMPT_NONE,
		.preempt_level = 0,
		.private = 0,
		.send_cmd =  0,
	},
	{//for preemption virtual device
		.dev_type = APUSYS_DEVICE_MDLA_RT,
		.preempt_type = APUSYS_PREEMPT_NONE,
		.preempt_level = 0,
		.private = 0,
		.send_cmd =  0,
	},
	{
		.dev_type = APUSYS_DEVICE_MDLA,
		.preempt_type = APUSYS_PREEMPT_NONE,
		.preempt_level = 0,
		.private = 0,
		.send_cmd =  0,
	},
	{//for preemption virtual device
		.dev_type = APUSYS_DEVICE_MDLA_RT,
		.preempt_type = APUSYS_PREEMPT_NONE,
		.preempt_level = 0,
		.private = 0,
		.send_cmd =  0,
	},
};
#endif

u32 mdla_max_num_core = MTK_MDLA_MAX_NUM;//TODO: core num get from DTS

struct mdla_reg_ctl mdla_reg_control[] = {
	{
		.apu_mdla_cmde_mreg_top = 0,
		.apu_mdla_biu_top = 0,
		.apu_mdla_config_top = 0,
	},
	{
		.apu_mdla_cmde_mreg_top = 0,
		.apu_mdla_biu_top = 0,
		.apu_mdla_config_top = 0,
	}
};
//void *infracfg_ao_top;//bus protect, only for mt6779
void *apu_conn_top;
void *apu_mdla_gsm_top;
void *apu_mdla_gsm_base;

static u32 cmd_id;



void mdla_reset_lock(unsigned int core, int res)
{
	mdla_reset(core, res);

	if (res == REASON_TIMEOUT)
		mdla_devices[core].last_reset_id = cmd_id;

}

#if 0//disable for Vulnerability Scan
static int mdla_mmap(struct file *filp, struct vm_area_struct *vma)
{
	unsigned long offset = vma->vm_pgoff;
	unsigned long size = vma->vm_end - vma->vm_start;
	/*MDLA early verification*/
	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
	if (remap_pfn_range(vma, vma->vm_start, offset, size,
			vma->vm_page_prot)) {
		pr_debug("%s: remap_pfn_range error: %p\n",
			__func__, vma);
		return -EAGAIN;
	}
	return 0;
}
#endif

static irqreturn_t mdla_irq0_handler(int irq, void *dev_id)
{
	//return mdla_interrupt(0);
	return mdla_scheduler(0);
}

static irqreturn_t mdla_irq1_handler(int irq, void *dev_id)
{
	//return mdla_interrupt(1);
	return mdla_scheduler(1);
}

static int mdla_scheduler_init(struct device *dev)
{
	struct mdla_scheduler *sched;
	size_t i;

	for (i = 0; i < mdla_max_num_core; i++) {
		mdla_devices[i].sched = devm_kzalloc(dev,
							 sizeof(*sched),
							 GFP_KERNEL);
		sched = mdla_devices[i].sched;
		if (!sched)
			return -ENOMEM;

		spin_lock_init(&sched->lock);
		INIT_LIST_HEAD(&sched->active_ce_queue);

		sched->pro_ce = NULL;
		sched->enqueue_ce = mdla_enqueue_ce;
		sched->dequeue_ce = mdla_dequeue_ce;
		sched->process_ce = mdla_process_ce;
		sched->issue_ce = mdla_issue_ce;
		sched->complete_ce = mdla_complete_ce;
		sched->pro_ce_normal = NULL;
		sched->pro_ce_high = NULL;
	}
	return 0;
}

static int mdla_sw_multi_devices_init(void)
{
	int i;

	if (get_devinfo_with_index(30) == 1)
		mdla_max_num_core = 1;

	for (i = 0; i < mdla_max_num_core; i++) {
		mdla_devices[i].mdla_e1_detect_count = 0;
		mdla_devices[i].async_cmd_id = 0;
		mdla_devices[i].max_cmd_id = 0;
		mdla_devices[i].last_reset_id = 0;
		init_completion(&mdla_devices[i].command_done);
		spin_lock_init(&mdla_devices[i].hw_lock);
		mdla_devices[i].power_timer.data = i;
		mdla_devices[i].power_timer.function =
				mdla_power_timeup;
		init_timer(&mdla_devices[i].power_timer);
		INIT_WORK(&mdla_devices[i].power_off_work,
				mdla_devices[i].power_pdn_work);
		mdla_devices[i].mdla_power_status = 0;
		mdla_devices[i].mdla_sw_power_status = 0;
		mdla_devices[i].timer_started = 0;
		pmu_init(i);
	}
	return 0;
}
static int mdla_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
#ifndef __APUSYS_MDLA_SW_PORTING_WORKAROUND__
	u32 preempt_level_support = PRIORITY_LEVEL;
	int i;
#endif

	if (!apusys_power_check())
		return 0;

	if (mdla_dts_map(pdev)) {
		dev_info(dev, "%s: failed due to DTS failed\n", __func__);
		return -EINVAL;
	}

	if (mdla_register_power(pdev)) {
		dev_info(dev, "register mdla power fail\n");
		return -EINVAL;
	}

#ifdef CONFIG_MTK_MDLA_ION
	mdla_ion_init();
#endif

#if defined(CONFIG_FPGA_EARLY_PORTING)
	for (i = 0; i < mdla_max_num_core; i++)
		mdla_reset_lock(i, REASON_DRVINIT);//TODO consider multi core
#endif

#ifndef __APUSYS_MDLA_SW_PORTING_WORKAROUND__
	/*
	 *        |   normal   |    RT
	 *  ------|------------|------------
	 *  core0 | index = 0  | index = 1
	 *  ------|------------|------------
	 *  core1 | index = 2  | index = 3
	 *  ------|------------|------------
	 *  PRIORITY_LEVEL_MAX = 2
	 */

	for (i = 0; i < mdla_max_num_core; i++) {
		int j = i * PRIORITY_LEVEL_MAX;

		apusys_mdla_dev[j].private = &mdla_devices[i];
		apusys_mdla_dev[j].send_cmd = apusys_mdla_handler;
		if (apusys_register_device(&(apusys_mdla_dev[j]))) {
			dev_info(dev, "register apusys mdla %d info\n", i);
			return -EINVAL;
		}
		if (preempt_level_support > 1) {
			apusys_mdla_dev[j + 1].private =
				&mdla_devices[i];
			apusys_mdla_dev[j + 1].send_cmd =
				apusys_mdla_handler;
			if (apusys_register_device(
					&(apusys_mdla_dev[j + 1]))
			) {
				dev_info(
					dev,
					"register apusys mdla RT %d info\n",
					i);
				return -EINVAL;
			}
		}
	}
#endif

	if (mdla_scheduler_init(&pdev->dev) < 0)
		return -ENOMEM;
	dev_info(dev, "%s: done\n", __func__);

	return 0;

}

static int mdla_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	int i;

	if (!apusys_power_check())
		return 0;

	mdla_drv_debug("%s start -\n", __func__);

	for (i = 0; i < mdla_max_num_core; i++)
		mdla_start_power_off(i, 0, true);

	if (mdla_unregister_power(pdev)) {
		dev_info(dev, "unregister mdla power fail\n");
		return -EINVAL;
	}

	mdla_drv_debug("%s unregister power -\n", __func__);

	//iounmap(infracfg_ao_top);
	iounmap(apu_conn_top);
	iounmap(apu_mdla_gsm_top);

	for (i = 0; i < mdla_max_num_core; i++) {
		free_irq(mdla_irqdesc[i].irq, dev);
		iounmap(mdla_reg_control[i].apu_mdla_config_top);
		iounmap(mdla_reg_control[i].apu_mdla_cmde_mreg_top);
		iounmap(mdla_reg_control[i].apu_mdla_biu_top);
	}

#ifdef CONFIG_MTK_MDLA_ION
	mdla_ion_exit();
#endif
	platform_set_drvdata(pdev, NULL);

	mdla_drv_debug("%s done -\n", __func__);

	return 0;
}
static const struct of_device_id mdla_of_match[] = {
	{ .compatible = "mediatek,mdla", },
	{ .compatible = "mtk,mdla", },
	{ /* end of list */},
};

static int mdla_resume(struct platform_device *pdev)
{
	mdla_cmd_debug("%s: resume\n", __func__);
	return 0;
}
static int mdla_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	int i;

	for (i = 0; i < mdla_max_num_core; i++)
		mdla_start_power_off(i, 1, true);

	mdla_cmd_debug("%s: suspend\n", __func__);
	return 0;
}

MODULE_DEVICE_TABLE(of, mdla_of_match);
static struct platform_driver mdla_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = mdla_of_match,
	},
	.probe = mdla_probe,
#if 1
	.suspend = mdla_suspend,
	.resume = mdla_resume,
#endif
	.remove = mdla_remove,
};

static int mdlactl_init(void)
{
	int ret;

	mdla_sw_multi_devices_init();

	ret = platform_driver_register(&mdla_driver);
	if (ret != 0)
		return ret;

	cmd_id = 1;

	mdla_debugfs_init();
	mdla_profile_init();
	mdla_wakeup_source_init();
	mdla_drv_debug("%s done!\n", __func__);

	return 0;
}

#ifndef __APUSYS_MDLA_SW_PORTING_WORKAROUND__
int apusys_mdla_handler(int type,
	void *hnd, struct apusys_device *dev)//struct apusys_run_cmd *acd)
{
	long retval = 0;
	struct apusys_cmd_hnd *cmd_hnd = hnd;
	struct mdla_dev *mdla_info = (struct mdla_dev *)dev->private;

	if (dev->dev_type == APUSYS_DEVICE_MDLA) {
		mdla_info = (struct mdla_dev *)dev->private;
		if (mdla_info->mdlaid >= mdla_max_num_core)
			return -EINVAL;
	} else if (dev->dev_type == APUSYS_DEVICE_MDLA_RT) {
		mdla_info = (struct mdla_dev *)dev->private;
		if (mdla_info->mdlaid >= mdla_max_num_core)
			return -EINVAL;
		if (type != APUSYS_CMD_EXECUTE)
			return 0;
	} else
		return -EINVAL;

	switch (type) {
	case APUSYS_CMD_POWERON:
		retval = mdla_pwr_on(mdla_info->mdlaid, true);
		break;
	case APUSYS_CMD_POWERDOWN:
		mdla_start_power_off(mdla_info->mdlaid, 0, true);
		break;
	case APUSYS_CMD_RESUME:
		break;
	case APUSYS_CMD_SUSPEND:
		mdla_start_power_off(mdla_info->mdlaid, 1, true);
		break;
	case APUSYS_CMD_EXECUTE:
	{
		struct mdla_run_cmd_sync *cmd_data =
			(struct mdla_run_cmd_sync *)cmd_hnd->kva;
		bool enable_preempt =
			(dev->dev_type == APUSYS_DEVICE_MDLA_RT) ? false : true;

		retval = mdla_run_command_sync(
			&cmd_data->req,
			mdla_info,
			cmd_hnd,
			enable_preempt);
		break;
	}
	case APUSYS_CMD_PREEMPT:
		return -EINVAL;
	default:
		return -EINVAL;
	}
	return retval;

}
#endif


static void mdlactl_exit(void)
{
	mdla_drv_debug("MDLA: Goodbye from the LKM!\n");
	mdla_debugfs_exit();
	platform_driver_unregister(&mdla_driver);

}

//module_init(mdlactl_init);
late_initcall(mdlactl_init);
module_exit(mdlactl_exit);

