/*
 * Copyright (C) 2015 MediaTek Inc.
 * Copyright (C) 2021 XiaoMi, Inc.
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

/*
 * OV5645AF voice coil motor driver
 *
 *
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/fs.h>

#include "lens_info.h"

#define AF_DRVNAME "OV5645AF_DRV"
#define AF_I2C_SLAVE_ADDR        0x3c

#define AF_DEBUG
#ifdef AF_DEBUG
#define LOG_INF(format, args...) pr_debug(AF_DRVNAME " [%s] "\
		format, __func__, ##args)
#else
#define LOG_INF(format, args...)
#endif

#define kal_uint32 unsigned int
#define kal_uint16 unsigned short

static struct i2c_client *g_pstAF_I2Cclient;
static int *g_pAF_Opened;
static spinlock_t *g_pAF_SpinLock;

static unsigned long g_u4AF_INF;
static unsigned long g_u4AF_MACRO = 1023;
static unsigned long g_u4TargetPosition;
static unsigned long g_u4CurrPosition;

static int g_SR = 3;

static int i2c_read(u16 a_u2Addr, u8 *a_puBuff, u16 i2cId)
{
	int i4RetValue = 0;
	char puReadCmd[2] = { (char)(a_u2Addr >> 8), (char)(a_u2Addr & 0xFF) };

	g_pstAF_I2Cclient->addr = i2cId;

	i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puReadCmd, 2);
	if (i4RetValue != 2) {
		LOG_INF(" I2C write failed!!\n");
		return -1;
	}
	i4RetValue = i2c_master_recv(g_pstAF_I2Cclient, (char *)a_puBuff, 1);
	if (i4RetValue != 1) {
		LOG_INF(" I2C read failed!!\n");
		return -1;
	}

	return 0;
}

static u8 read_data(u16 addr)
{
	u8 get_byte = 0;

	i2c_read(addr, &get_byte, AF_I2C_SLAVE_ADDR);

	return get_byte;
}

static int s4AF_ReadReg(unsigned short *a_pu2Result)
{
	LOG_INF("OV5645 AF Status: state_3603 = %d, state_3602 = %d\n",
		read_data(0x3603), read_data(0x3602));
	*a_pu2Result = (read_data(0x3603) << 8) + (read_data(0x3602) & 0xff);

	return 0;
}

static int s4AF_WriteReg_OV5645AF(u8 *a_pSendData,
				u16 a_sizeSendData, u16 i2cId)
{
	int i4RetValue = 0;

	g_pstAF_I2Cclient->addr = i2cId;
	i4RetValue = i2c_master_send(g_pstAF_I2Cclient,
			a_pSendData, a_sizeSendData);

	if (i4RetValue < 0) {
		LOG_INF("I2C send failed!!, Addr = 0x%x, Data = 0x%x\n",
			a_pSendData[0], a_pSendData[1]);
		return -1;
	}

	return 0;
}

static int s4AF_WriteReg(u16 a_u2Data, u16 a_u2Para)
{
	char pusendcmd[3] = { (char)(a_u2Data >> 8),
			(char)(a_u2Data & 0xFF), (char)(a_u2Para & 0xFF) };

	LOG_INF("OV5645 AF WriteReg = 0x%x\n", a_u2Data);
	s4AF_WriteReg_OV5645AF(pusendcmd, sizeof(pusendcmd), AF_I2C_SLAVE_ADDR);

	return 0;
}

static inline int getAFInfo(__user struct stAF_MotorInfo *pstMotorInfo)
{
	struct stAF_MotorInfo stMotorInfo;

	stMotorInfo.u4MacroPosition = g_u4AF_MACRO;
	stMotorInfo.u4InfPosition = g_u4AF_INF;
	stMotorInfo.u4CurrentPosition = g_u4CurrPosition;
	stMotorInfo.bIsSupportSR = 1;

	stMotorInfo.bIsMotorMoving = 1;

	if (*g_pAF_Opened >= 1)
		stMotorInfo.bIsMotorOpen = 1;
	else
		stMotorInfo.bIsMotorOpen = 0;

	if (copy_to_user(pstMotorInfo, &stMotorInfo,
		sizeof(struct stAF_MotorInfo)))
		LOG_INF("copy to user failed when getting motor information\n");

	return 0;
}

static inline int moveAF(unsigned long a_u4Position)
{
	int ret = 0;
	unsigned long AFValueH = 0;
	unsigned long AFValueL = 0;

	if ((a_u4Position > g_u4AF_MACRO) || (a_u4Position < g_u4AF_INF)) {
		LOG_INF("out of range\n");
		return -EINVAL;
	}

	if (*g_pAF_Opened == 1) {
		unsigned short InitPos;

		ret = s4AF_ReadReg(&InitPos);

		if (ret == 0) {
			LOG_INF("Init Pos %6d\n", InitPos);

			spin_lock(g_pAF_SpinLock);
			g_u4CurrPosition = (unsigned long)InitPos;
			spin_unlock(g_pAF_SpinLock);

		} else {
			spin_lock(g_pAF_SpinLock);
			g_u4CurrPosition = 0;
			spin_unlock(g_pAF_SpinLock);
		}

		spin_lock(g_pAF_SpinLock);
		*g_pAF_Opened = 2;
		spin_unlock(g_pAF_SpinLock);
	}

	if (g_u4CurrPosition == a_u4Position)
		return 0;

	spin_lock(g_pAF_SpinLock);
	g_u4TargetPosition = a_u4Position;
	g_SR = 3;
	spin_unlock(g_pAF_SpinLock);

	AFValueH = (g_u4TargetPosition & 0xFFF0) >> 4;
	AFValueL = (g_u4TargetPosition & 0x000F) << 4;

	if ((s4AF_WriteReg(0x3602, AFValueL) == 0) &&
		(s4AF_WriteReg(0x3603, AFValueH) == 0) &&
		(s4AF_WriteReg(0x3604, 0x05) == 0) &&
		(s4AF_WriteReg(0x3605, 0x46) == 0) &&
		(s4AF_WriteReg(0x3606, 0x07) == 0)) {
		spin_lock(g_pAF_SpinLock);
		g_u4CurrPosition = (unsigned long)g_u4TargetPosition;
		spin_unlock(g_pAF_SpinLock);
	} else {
		LOG_INF("set I2C failed when moving the motor\n");
		ret = -1;
	}

	return ret;
}

static inline int setAFInf(unsigned long a_u4Position)
{
	spin_lock(g_pAF_SpinLock);
	g_u4AF_INF = a_u4Position;
	spin_unlock(g_pAF_SpinLock);
	return 0;
}

static inline int setAFMacro(unsigned long a_u4Position)
{
	spin_lock(g_pAF_SpinLock);
	g_u4AF_MACRO = a_u4Position;
	spin_unlock(g_pAF_SpinLock);
	return 0;
}

/* ////////////////////////////////////////////////////////////// */
long OV5645AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
		unsigned long a_u4Param)
{
	long i4RetValue = 0;

	switch (a_u4Command) {
	case AFIOC_G_MOTORINFO:
		i4RetValue =
		getAFInfo((__user struct stAF_MotorInfo *) (a_u4Param));
		break;

	case AFIOC_T_MOVETO:
		i4RetValue = moveAF(a_u4Param);
		break;

	case AFIOC_T_SETINFPOS:
		i4RetValue = setAFInf(a_u4Param);
		break;

	case AFIOC_T_SETMACROPOS:
		i4RetValue = setAFMacro(a_u4Param);
		break;

	default:
		LOG_INF("No CMD\n");
		i4RetValue = -EPERM;
		break;
	}

	return i4RetValue;
}

/* Main jobs: */
/* 1.Deallocate anything that "open" allocated in private_data. */
/* 2.Shut down the device on last close. */
/* 3.Only called once on last time. */
/* Q1 : Try release multiple times. */
int OV5645AF_Release(struct inode *a_pstInode, struct file *a_pstFile)
{
	LOG_INF("Start\n");

	if (*g_pAF_Opened == 2) {
		LOG_INF("Wait\n");
		g_SR = 5;
		/* s4AF_WriteReg(200); */
		msleep(20);
		/* s4AF_WriteReg(100); */
		/* msleep(20); */
	}

	if (*g_pAF_Opened) {
		LOG_INF("Free\n");

		spin_lock(g_pAF_SpinLock);
		*g_pAF_Opened = 0;
		spin_unlock(g_pAF_SpinLock);
	}

	LOG_INF("End\n");

	return 0;
}

int OV5645AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
		spinlock_t *pAF_SpinLock, int *pAF_Opened)
{
	g_pstAF_I2Cclient = pstAF_I2Cclient;
	g_pAF_SpinLock = pAF_SpinLock;
	g_pAF_Opened = pAF_Opened;

	return 1;
}
