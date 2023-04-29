/*
 * Copyright (C) 2016 MediaTek Inc.
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

#include <linux/types.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>

#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/sched.h>

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/i2c.h>

#include "i2c-mtk.h"
#include <m4u.h>

#include "ccu_cmn.h"
#include "ccu_i2c.h"
#include "ccu_i2c_hw.h"
#include "ccu_mva.h"

/******************************************************************************
 *
 *****************************************************************************/
/*I2C Channel offset*/
#define I2C_BASE_OFS_CH1 (0x200)
#define MAX_I2C_CMD_LEN 255
#define CCU_I2C_APDMA_TXLEN 128
#define CCU_I2C_MAIN_HW_DRVNAME  "ccu_i2c_main_hwtrg"
#define CCU_I2C_MAIN2_HW_DRVNAME  "ccu_i2c_main2_hwtrg"
#define CCU_I2C_MAIN3_HW_DRVNAME  "ccu_i2c_main3_hwtrg"
#define CCU_I2C_SUB_HW_DRVNAME  "ccu_i2c_sub_hwtrg"
#define CCU_I2C_SUB2_HW_DRVNAME  "ccu_i2c_sub2_hwtrg"

static DEFINE_MUTEX(ccu_i2c_mutex);

/*i2c driver hook*/

static int ccu_i2c_probe_main(struct i2c_client *client,
	const struct i2c_device_id *id);
static int ccu_i2c_probe_main2(struct i2c_client *client,
	const struct i2c_device_id *id);
static int ccu_i2c_probe_main3(struct i2c_client *client,
	const struct i2c_device_id *id);
static int ccu_i2c_probe_sub(struct i2c_client *client,
	const struct i2c_device_id *id);
static int ccu_i2c_probe_sub2(struct i2c_client *client,
	const struct i2c_device_id *id);

static int ccu_i2c_remove(struct i2c_client *client);

/*ccu i2c operation*/
static struct i2c_client *get_ccu_i2c_client(
	enum CCU_I2C_CHANNEL i2c_controller_id);
static int ccu_i2c_controller_en(enum CCU_I2C_CHANNEL i2c_controller_id,
	int enable);
static int i2c_query_dma_buffer_addr(struct ccu_device_s *g_ccu_device,
	enum CCU_I2C_CHANNEL i2c_controller_id,
	uint32_t *mva, uint32_t *va_h, uint32_t *va_l,
	uint32_t *i2c_id);
static int ccu_i2c_controller_uninit(enum CCU_I2C_CHANNEL i2c_controller_id);

/*i2c reg operation*/
static inline u32 i2c_readl_dma(struct mt_i2c *i2c, u8 offset);
static inline void i2c_writel_dma(u32 value, struct mt_i2c *i2c,
				  u8 offset);
static inline u16 i2c_readw(struct mt_i2c *i2c, u8 offset);
static inline void i2c_writew(u16 value, struct mt_i2c *i2c,
			      u16 offset);


static enum CCU_I2C_CHANNEL g_ccuI2cChannel;
static struct i2c_client *g_ccuI2cClientMain;
static struct i2c_client *g_ccuI2cClientMain2;
static struct i2c_client *g_ccuI2cClientMain3;
static struct i2c_client *g_ccuI2cClientSub;
static struct i2c_client *g_ccuI2cClientSub2;

static const struct i2c_device_id ccu_i2c_main_ids[] = {
	{CCU_I2C_MAIN_HW_DRVNAME, 0}, {} };
static const struct i2c_device_id ccu_i2c_main2_ids[] = {
	{CCU_I2C_MAIN2_HW_DRVNAME, 0}, {} };
static const struct i2c_device_id ccu_i2c_main3_ids[]
	= { {CCU_I2C_MAIN3_HW_DRVNAME, 0}, {} };
static const struct i2c_device_id ccu_i2c_sub_ids[] = {
	{CCU_I2C_SUB_HW_DRVNAME, 0}, {} };
static const struct i2c_device_id ccu_i2c_sub2_ids[]
	= { {CCU_I2C_SUB2_HW_DRVNAME, 0}, {} };

static bool ccu_i2c_initialized[CCU_I2C_CHANNEL_MAX] = {0};

#ifdef CONFIG_OF
static const struct of_device_id ccu_i2c_main_driver_of_ids[] = {
	{.compatible = "mediatek,ccu_sensor_i2c_main_hw",},
	{}
};

static const struct of_device_id ccu_i2c_main2_driver_of_ids[] = {
	{.compatible = "mediatek,ccu_sensor_i2c_main2_hw",},
	{}
};

static const struct of_device_id ccu_i2c_main3_driver_of_ids[] = {
	{.compatible = "mediatek,ccu_sensor_i2c_main3_hw",},
	{}
};

static const struct of_device_id ccu_i2c_sub_driver_of_ids[] = {
	{.compatible = "mediatek,ccu_sensor_i2c_sub_hw",},
	{}
};

static const struct of_device_id ccu_i2c_sub2_driver_of_ids[] = {
	{.compatible = "mediatek,ccu_sensor_i2c_sub2_hw",},
	{}
};
#endif

struct i2c_driver ccu_i2c_main_driver = {
	.probe = ccu_i2c_probe_main,
	.remove = ccu_i2c_remove,
	.driver = {
		.name = CCU_I2C_MAIN_HW_DRVNAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = ccu_i2c_main_driver_of_ids,
#endif
	},
	.id_table = ccu_i2c_main_ids,
};

struct i2c_driver ccu_i2c_main2_driver = {
	.probe = ccu_i2c_probe_main2,
	.remove = ccu_i2c_remove,
	.driver = {
		.name = CCU_I2C_MAIN2_HW_DRVNAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = ccu_i2c_main2_driver_of_ids,
#endif
	},
	.id_table = ccu_i2c_main2_ids,
};

struct i2c_driver ccu_i2c_main3_driver = {
	.probe = ccu_i2c_probe_main3,
	.remove = ccu_i2c_remove,
	.driver = {
		   .name = CCU_I2C_MAIN3_HW_DRVNAME,
		   .owner = THIS_MODULE,
#ifdef CONFIG_OF
		   .of_match_table = ccu_i2c_main3_driver_of_ids,
#endif
		   },
	.id_table = ccu_i2c_main3_ids,
};

struct i2c_driver ccu_i2c_sub_driver = {
	.probe = ccu_i2c_probe_sub,
	.remove = ccu_i2c_remove,
	.driver = {
		.name = CCU_I2C_SUB_HW_DRVNAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = ccu_i2c_sub_driver_of_ids,
#endif
	},
	.id_table = ccu_i2c_sub_ids,
};

struct i2c_driver ccu_i2c_sub2_driver = {
	.probe = ccu_i2c_probe_sub2,
	.remove = ccu_i2c_remove,
	.driver = {
		   .name = CCU_I2C_SUB2_HW_DRVNAME,
		   .owner = THIS_MODULE,
#ifdef CONFIG_OF
		   .of_match_table = ccu_i2c_sub2_driver_of_ids,
#endif
		   },
	.id_table = ccu_i2c_sub2_ids,
};

/*---------------------------------------------------------------------------*/
/* CCU Driver: i2c driver funcs                                              */
/*---------------------------------------------------------------------------*/
static int ccu_i2c_probe_main(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	LOG_DBG_MUST(
	"[ccu_i2c_probe] Attach I2C for HW trriger g_ccuI2cClientMain %p\n",
	client);

	/* get sensor i2c client */
	g_ccuI2cClientMain = client;

	LOG_DBG_MUST("[ccu_i2c_probe] Attached!!\n");

	return 0;
}

static int ccu_i2c_probe_main2(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	LOG_DBG_MUST(
		"[ccu_i2c_probe] Attach I2C for HW trriger g_ccuI2cClientMain2 %p\n",
		client);

	/* get sensor i2c client */
	g_ccuI2cClientMain2 = client;

	LOG_DBG_MUST("[ccu_i2c_probe] Attached!!\n");

	return 0;
}

static int ccu_i2c_probe_main3(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	LOG_DBG_MUST(
		"[ccu_i2c_probe] Attach I2C for HW trriger g_ccuI2cClientMain3 %p\n",
		client);

	/* get sensor i2c client */
	g_ccuI2cClientMain3 = client;

	LOG_DBG_MUST("[ccu_i2c_probe] Attached!!\n");

	return 0;
}

static int ccu_i2c_probe_sub(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	/*int i4RetValue = 0;*/
	LOG_DBG_MUST(
		"[ccu_i2c_probe] Attach I2C for HW trriger g_ccuI2cClientSub %p\n",
		client);

	g_ccuI2cClientSub = client;

	LOG_DBG_MUST("[ccu_i2c_probe] Attached!!\n");

	return 0;
}

static int ccu_i2c_probe_sub2(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	/*int i4RetValue = 0;*/
	LOG_DBG_MUST(
		"[ccu_i2c_probe] Attach I2C for HW trriger g_ccuI2cClientSub2 %p\n",
		client);

	g_ccuI2cClientSub2 = client;

	LOG_DBG_MUST("[ccu_i2c_probe] Attached!!\n");

	return 0;
}

static int ccu_i2c_remove(struct i2c_client *client)
{
	return 0;
}

/*---------------------------------------------------------------------------*/
/* CCU i2c public funcs                                                      */
/*---------------------------------------------------------------------------*/
int ccu_i2c_register_driver(void)
{
	int i2c_ret = 0;

	mutex_lock(&ccu_i2c_mutex);
	LOG_DBG_MUST("i2c_add_driver(&ccu_i2c_main_driver)++\n");
	i2c_ret = i2c_add_driver(&ccu_i2c_main_driver);
	LOG_DBG_MUST("i2c_add_driver(&ccu_i2c_main_driver), ret: %d--\n",
		i2c_ret);
	LOG_DBG_MUST("i2c_add_driver(&ccu_i2c_main2_driver)++\n");
	i2c_ret = i2c_add_driver(&ccu_i2c_main2_driver);

	LOG_DBG_MUST("i2c_add_driver(&ccu_i2c_main2_driver), ret: %d--\n",
		i2c_ret);
	LOG_DBG_MUST("i2c_add_driver(&ccu_i2c_main3_driver)++\n");
	i2c_ret = i2c_add_driver(&ccu_i2c_main3_driver);
	LOG_DBG_MUST("i2c_add_driver(&main3_driver),ret:%d--\n", i2c_ret);
	LOG_DBG_MUST("i2c_add_driver(&ccu_i2c_sub_driver)++\n");
	i2c_ret = i2c_add_driver(&ccu_i2c_sub_driver);
	LOG_DBG_MUST("i2c_add_driver(&ccu_i2c_sub_driver), ret: %d--\n",
		i2c_ret);
	LOG_DBG_MUST("i2c_add_driver(&ccu_i2c_sub2_driver)++\n");
	i2c_ret = i2c_add_driver(&ccu_i2c_sub2_driver);
	LOG_DBG_MUST("i2c_add_driver(&sub2_driver),ret:%d--\n", i2c_ret);
	mutex_unlock(&ccu_i2c_mutex);

	return 0;
}

int ccu_i2c_delete_driver(void)
{
	mutex_lock(&ccu_i2c_mutex);
	i2c_del_driver(&ccu_i2c_main_driver);
	i2c_del_driver(&ccu_i2c_main2_driver);
	i2c_del_driver(&ccu_i2c_main3_driver);
	i2c_del_driver(&ccu_i2c_sub_driver);
	i2c_del_driver(&ccu_i2c_sub2_driver);
	mutex_unlock(&ccu_i2c_mutex);

	return 0;
}

int ccu_i2c_set_channel(enum CCU_I2C_CHANNEL channel)
{

	mutex_lock(&ccu_i2c_mutex);
	if ((channel == CCU_I2C_CHANNEL_MAINCAM) ||
		(channel == CCU_I2C_CHANNEL_SUBCAM) ||
		(channel == CCU_I2C_CHANNEL_MAINCAM2) ||
		(channel == CCU_I2C_CHANNEL_SUBCAM2) ||
		(channel == CCU_I2C_CHANNEL_MAINCAM3)) {
		g_ccuI2cChannel = channel;
		mutex_unlock(&ccu_i2c_mutex);
		return 0;
	} else {
		mutex_unlock(&ccu_i2c_mutex);
		return -EFAULT;
	}
}

int ccu_i2c_controller_init(enum CCU_I2C_CHANNEL
			    i2c_controller_id)
{
	mutex_lock(&ccu_i2c_mutex);
	if (i2c_controller_id >= CCU_I2C_CHANNEL_MAX) {
		LOG_ERR("i2c_id %d is invalid\n", i2c_controller_id);
		mutex_unlock(&ccu_i2c_mutex);
		return -EINVAL;
	}

	if (ccu_i2c_initialized[i2c_controller_id] == MTRUE) {
	/*if not first time init, release mutex first to avoid deadlock*/
		LOG_DBG_MUST("reinit, temporily release mutex.\n");
	}
	if (ccu_i2c_controller_en(i2c_controller_id, 1) == -1) {
		LOG_DBG("ccu_i2c_controller_en 1 fail\n");
		mutex_unlock(&ccu_i2c_mutex);
		return -1;
	}

	LOG_DBG_MUST("%s done.\n", __func__);

	mutex_unlock(&ccu_i2c_mutex);
	return 0;
}

int ccu_i2c_controller_uninit_all(void)
{
	int i;

	mutex_lock(&ccu_i2c_mutex);
	for (i = CCU_I2C_CHANNEL_MIN ; i < CCU_I2C_CHANNEL_MAX ; i++) {
		if (ccu_i2c_initialized[i])
			ccu_i2c_controller_uninit((enum CCU_I2C_CHANNEL)i);
	}

	LOG_INF_MUST("%s done.\n", __func__);

	mutex_unlock(&ccu_i2c_mutex);
	return 0;
}

int ccu_get_i2c_dma_buf_addr(struct ccu_device_s *g_ccu_device,
	struct ccu_i2c_buf_mva_ioarg *ioarg)
{
	int ret = 0;

	mutex_lock(&ccu_i2c_mutex);
	ret = i2c_query_dma_buffer_addr(
		g_ccu_device, ioarg->i2c_controller_id,
		&ioarg->mva, &ioarg->va_h, &ioarg->va_l, &ioarg->i2c_id);

	mutex_unlock(&ccu_i2c_mutex);
	return ret;
}


int ccu_i2c_free_dma_buf_mva_all(struct ccu_device_s *g_ccu_device)
{

	mutex_lock(&ccu_i2c_mutex);
	ccu_deallocate_mva(g_ccu_device->i2c_dma_mva);

	LOG_INF_MUST("%s done.\n", __func__);

	mutex_unlock(&ccu_i2c_mutex);
	return 0;
}

void ccu_i2c_dump_errr(void)
{
	struct i2c_client *pClient = NULL;
	struct mt_i2c *i2c;
	int i;

	for (i = CCU_I2C_CHANNEL_MIN ; i < CCU_I2C_CHANNEL_MAX ; i++) {
		LOG_INF_MUST(
		pClient = get_ccu_i2c_client((enum CCU_I2C_CHANNEL)i);
		if (pClient == NULL) {
			LOG_ERR("i2c_client is null\n");
			return;
		}
	}
}

/*---------------------------------------------------------------------------*/
/* CCU i2c static funcs                                              */
/*---------------------------------------------------------------------------*/
static int i2c_query_dma_buffer_addr(struct ccu_device_s *g_ccu_device,
	enum CCU_I2C_CHANNEL i2c_controller_id
	, uint32_t *mva, uint32_t *va_h, uint32_t *va_l, uint32_t *i2c_id)
{
	struct i2c_client *pClient = NULL;
	struct mt_i2c *i2c;
	int ret = 0;

	pClient = get_ccu_i2c_client(i2c_controller_id);

	if (pClient == MNULL) {
		LOG_ERR("ccu client is NULL");
		return -EFAULT;
	}
	if (g_ccu_device->dev == MNULL) {
		LOG_ERR("ccu device is NULL");
		return -EFAULT;
	}

	i2c = i2c_get_adapdata(pClient->adapter);

	if (g_ccu_device->i2c_dma_mva == 0)	{
		ret = ccu_allocate_mva(&g_ccu_device->i2c_dma_mva,
				g_ccu_device->i2c_dma_vaddr,
				CCU_I2C_DMA_BUF_SIZE);
		if (ret != 0) {
			LOG_ERR("ccu alloc mva fail");
			return -EFAULT;
		}
	}
	/*i2c_get_dma_buffer_addr_imp(pClient->adapter ,va);*/
	*mva = g_ccu_device->i2c_dma_mva;
	*va_l = g_ccu_device->i2c_dma_paddr;
	*va_h = ((g_ccu_device->i2c_dma_paddr) >> 32);
	*i2c_id = i2c->id;
	LOG_DBG_MUST("$$pa(%lld), mva(%x), i2c-id(%d)\n",
		g_ccu_device->i2c_dma_paddr,
		g_ccu_device->i2c_dma_mva, (uint32_t)i2c->id);

	return 0;
}

static int ccu_i2c_controller_en(
	enum CCU_I2C_CHANNEL i2c_controller_id, int enable)
{
	int ret = 0;
	struct i2c_client *pClient = NULL;
	struct mt_i2c *i2c;

	LOG_INF_MUST("%s, id(%d):(%d)->(%d)\n", __func__
		, i2c_controller_id, ccu_i2c_initialized[i2c_controller_id],
		enable);

	pClient = get_ccu_i2c_client(i2c_controller_id);
	LOG_DBG("%s, pClient: %p\n", __func__, pClient);

	if (pClient == NULL) {
		LOG_ERR("i2c_client is null\n");
		return -1;
	}

	if (enable) {
		if (ccu_i2c_initialized[i2c_controller_id] == MFALSE) {
			ret = i2c_ccu_enable(pClient->adapter,
				I2C_BASE_OFS_CH1);
			ccu_i2c_initialized[i2c_controller_id] = MTRUE;
			LOG_INF_MUST("i2c_ccu_enable done.\n");

			/*dump controller status*/
			i2c = i2c_get_adapdata(pClient->adapter);
			/*MCU_INTR re-direct to CCU only*/
			i2c_writew(2, i2c, 0x240);
		}
	} else {
		if (ccu_i2c_initialized[i2c_controller_id] == MTRUE) {
			ret = i2c_ccu_disable(pClient->adapter);
			ccu_i2c_initialized[i2c_controller_id] = MFALSE;
			LOG_INF_MUST("i2c_ccu_disable done.\n");
		}
	}
	return ret;
}

static int ccu_i2c_controller_uninit(enum CCU_I2C_CHANNEL i2c_controller_id)
{
	if (ccu_i2c_controller_en(i2c_controller_id, 0) == -1) {
		LOG_DBG("ccu_i2c_controller_en 0 fail\n");
		return -1;
	}

	LOG_DBG_MUST("%s done: id(%d)\n", __func__, i2c_controller_id);

	return 0;
}


static struct i2c_client *get_ccu_i2c_client(
	enum CCU_I2C_CHANNEL i2c_controller_id)
{
	switch (i2c_controller_id) {
	case CCU_I2C_CHANNEL_MAINCAM: {
		return g_ccuI2cClientMain;
	}
	case CCU_I2C_CHANNEL_MAINCAM2: {
		return g_ccuI2cClientMain2;
	}
	case CCU_I2C_CHANNEL_MAINCAM3:
	{
		return g_ccuI2cClientMain3;
	}
	case CCU_I2C_CHANNEL_SUBCAM:
	{
		return g_ccuI2cClientSub;
	}
	case CCU_I2C_CHANNEL_SUBCAM2:
	{
		return g_ccuI2cClientSub2;
	}
	default:
	{
		return MNULL;
	}
	}
}

static inline u32 i2c_readl_dma(struct mt_i2c *i2c, u8 offset)
{
	return readl(i2c->pdmabase + offset);
}

static inline void i2c_writel_dma(u32 value, struct mt_i2c *i2c,
				  u8 offset)
{
	writel(value, i2c->pdmabase + offset);
}

static inline u16 i2c_readw(struct mt_i2c *i2c, u8 offset)
{
	return readw(i2c->base + offset);
}

static inline void i2c_writew(u16 value, struct mt_i2c *i2c,
			      u16 offset)
{
	writew(value, i2c->base + offset);
}
