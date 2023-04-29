/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */
#include "gl_os.h"

#if KERNEL_VERSION(5, 4, 0) <= CFG80211_VERSION_CODE
/*TODO kernel 5.4 boost CPU */
#else
#include <cpu_ctrl.h>
#include <topo_ctrl.h>
#include <helio-dvfsrc-opp.h>
#endif
#include <linux/pm_qos.h>
#include "precomp.h"
#include "wmt_exp.h"

#ifdef CONFIG_WLAN_MTK_EMI
#if KERNEL_VERSION(5, 4, 0) <= CFG80211_VERSION_CODE
#include <soc/mediatek/emi.h>
#else
#include <memory/mediatek/emi.h>
#endif
#define WIFI_EMI_MEM_OFFSET    0x1D0000
#define WIFI_EMI_MEM_SIZE      0x140000
#define DOMAIN_AP	0
#define DOMAIN_CONN	2
#endif

#define MAX_CPU_FREQ (3 * 1024 * 1024) /* in kHZ */
#define MAX_CLUSTER_NUM  3
#define CPU_ALL_CORE (0xff)
#define CPU_BIG_CORE (0xc0)
#define CPU_LITTLE_CORE (CPU_ALL_CORE - CPU_BIG_CORE)

enum ENUM_CPU_BOOST_STATUS {
	ENUM_CPU_BOOST_STATUS_INIT = 0,
	ENUM_CPU_BOOST_STATUS_START,
	ENUM_CPU_BOOST_STATUS_STOP,
	ENUM_CPU_BOOST_STATUS_NUM
};

uint32_t kalGetCpuBoostThreshold(void)
{
	DBGLOG(SW4, TRACE, "enter kalGetCpuBoostThreshold\n");
	/* 5, stands for 250Mbps */
	return 5;
}

int32_t kalCheckTputLoad(IN struct ADAPTER *prAdapter,
			 IN uint32_t u4CurrPerfLevel,
			 IN uint32_t u4TarPerfLevel,
			 IN int32_t i4Pending,
			 IN uint32_t u4Used)
{
	uint32_t pendingTh =
		CFG_TX_STOP_NETIF_PER_QUEUE_THRESHOLD *
		prAdapter->rWifiVar.u4PerfMonPendingTh / 100;
	uint32_t usedTh = (HIF_TX_MSDU_TOKEN_NUM / 2) *
		prAdapter->rWifiVar.u4PerfMonUsedTh / 100;
	return u4TarPerfLevel >= 3 &&
	       u4TarPerfLevel < prAdapter->rWifiVar.u4BoostCpuTh &&
	       i4Pending >= pendingTh &&
	       u4Used >= usedTh ?
	       TRUE : FALSE;
}

#if KERNEL_VERSION(5, 4, 0) <= CFG80211_VERSION_CODE
#else
int32_t kalBoostCpu(IN struct ADAPTER *prAdapter,
		    IN uint32_t u4TarPerfLevel,
		    IN uint32_t u4BoostCpuTh)
{
	struct GLUE_INFO *prGlueInfo = NULL;
	struct ppm_limit_data freq_to_set[MAX_CLUSTER_NUM];
	int32_t i = 0, i4Freq = -1;

	static struct pm_qos_request wifi_qos_request;
	static u_int8_t fgRequested = ENUM_CPU_BOOST_STATUS_INIT;

	uint32_t u4ClusterNum = topo_ctrl_get_nr_clusters();

	WIPHY_PRIV(wlanGetWiphy(), prGlueInfo);
	ASSERT(u4ClusterNum <= MAX_CLUSTER_NUM);
	/* ACAO, we dont have to set core number */
	i4Freq = (u4TarPerfLevel >= u4BoostCpuTh) ? MAX_CPU_FREQ : -1;
	for (i = 0; i < u4ClusterNum; i++) {
		freq_to_set[i].min = i4Freq;
		freq_to_set[i].max = i4Freq;
	}

	if (fgRequested == ENUM_CPU_BOOST_STATUS_INIT) {
		/* initially enable rps working at small cores */
		kalSetRpsMap(prGlueInfo, CPU_LITTLE_CORE);
		fgRequested = ENUM_CPU_BOOST_STATUS_STOP;
	}

	if (u4TarPerfLevel >= u4BoostCpuTh) {
		if (fgRequested == ENUM_CPU_BOOST_STATUS_STOP) {
			pr_info("kalBoostCpu start (%d>=%d)\n",
				u4TarPerfLevel, u4BoostCpuTh);
			fgRequested = ENUM_CPU_BOOST_STATUS_START;

			set_task_util_min_pct(prGlueInfo->u4TxThreadPid, 100);
			set_task_util_min_pct(prGlueInfo->u4RxThreadPid, 100);
			set_task_util_min_pct(prGlueInfo->u4HifThreadPid, 100);
			kalSetRpsMap(prGlueInfo, CPU_BIG_CORE);
			update_userlimit_cpu_freq(CPU_KIR_WIFI,
				u4ClusterNum, freq_to_set);

			KAL_ACQUIRE_MUTEX(prAdapter, MUTEX_BOOST_CPU);
			pr_info("Max Dram Freq start\n");
			pm_qos_add_request(&wifi_qos_request,
					   PM_QOS_DDR_OPP,
					   DDR_OPP_0);
			pm_qos_update_request(&wifi_qos_request, DDR_OPP_0);
			KAL_RELEASE_MUTEX(prAdapter, MUTEX_BOOST_CPU);
		}
	} else {
		if (fgRequested == ENUM_CPU_BOOST_STATUS_START) {
			pr_info("kalBoostCpu stop (%d<%d)\n",
				u4TarPerfLevel, u4BoostCpuTh);
			fgRequested = ENUM_CPU_BOOST_STATUS_STOP;

			set_task_util_min_pct(prGlueInfo->u4TxThreadPid, 0);
			set_task_util_min_pct(prGlueInfo->u4RxThreadPid, 0);
			set_task_util_min_pct(prGlueInfo->u4HifThreadPid, 0);
			kalSetRpsMap(prGlueInfo, CPU_LITTLE_CORE);
			update_userlimit_cpu_freq(CPU_KIR_WIFI,
				u4ClusterNum, freq_to_set);

			KAL_ACQUIRE_MUTEX(prAdapter, MUTEX_BOOST_CPU);
			pr_info("Max Dram Freq end\n");
			pm_qos_update_request(&wifi_qos_request, DDR_OPP_UNREQ);
			pm_qos_remove_request(&wifi_qos_request);
			KAL_RELEASE_MUTEX(prAdapter, MUTEX_BOOST_CPU);
		}
	}
	kalTraceInt(fgRequested == ENUM_CPU_BOOST_STATUS_START, "kalBoostCpu");

	return 0;
}
#endif

#ifdef CONFIG_WLAN_MTK_EMI
void kalSetEmiMpuProtection(phys_addr_t emiPhyBase, bool enable)
{
}

void kalSetDrvEmiMpuProtection(phys_addr_t emiPhyBase, uint32_t offset,
			       uint32_t size)
{
	struct emimpu_region_t region;
	unsigned long long start = emiPhyBase + offset;
	unsigned long long end = emiPhyBase + offset + size - 1;
	int ret;

	DBGLOG(INIT, INFO, "emiPhyBase: 0x%p, offset: %d, size: %d\n",
				emiPhyBase, offset, size);

	ret = mtk_emimpu_init_region(&region, 18);
	if (ret) {
		DBGLOG(INIT, ERROR, "mtk_emimpu_init_region failed, ret: %d\n",
				ret);
		return;
	}
	mtk_emimpu_set_addr(&region, start, end);
	mtk_emimpu_set_apc(&region, DOMAIN_AP, MTK_EMIMPU_NO_PROTECTION);
	mtk_emimpu_set_apc(&region, DOMAIN_CONN, MTK_EMIMPU_NO_PROTECTION);
	mtk_emimpu_lock_region(&region, MTK_EMIMPU_LOCK);
	ret = mtk_emimpu_set_protection(&region);
	if (ret)
		DBGLOG(INIT, ERROR,
			"mtk_emimpu_set_protection failed, ret: %d\n",
			ret);
	mtk_emimpu_free_region(&region);
}
#endif

int32_t kalGetFwFlavorByPlat(uint8_t *flavor)
{
	int32_t ret = 1;
	const uint32_t adie_chip_id = mtk_wcn_wmt_ic_info_get(WMTCHIN_ADIE);

	DBGLOG(INIT, INFO, "adie_chip_id: 0x%x\n", adie_chip_id);

	switch (adie_chip_id) {
	case 0x6631:
		*flavor = 'c';
		break;
	case 0x6635:
		*flavor = 'd';
		break;
	default:
		ret = 0;
		break;
	}

	return ret;
}
