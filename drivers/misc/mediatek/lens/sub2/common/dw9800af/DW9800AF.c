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
 * DW9800AF voice coil motor driver
 *
 *
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/fs.h>

#include "lens_info.h"

#define AF_DRVNAME "DW9800AF_DRV"
#define AF_I2C_SLAVE_ADDR        0x18

#define AF_NO_NOISE_POSITION_CODE 512

#define AF_DEBUG
#ifdef AF_DEBUG
#define LOG_INF(format, args...) pr_debug(AF_DRVNAME " [%s] " format, __func__, ##args)
#else
#define LOG_INF(format, args...)
#endif

static struct i2c_client *g_pstAF_I2Cclient;
static int *g_pAF_Opened;
static spinlock_t *g_pAF_SpinLock;

static unsigned long g_u4AF_INF;
static unsigned long g_u4AF_MACRO = 1023;
static unsigned long g_u4TargetPosition;
static unsigned long g_u4CurrPosition;

static const unsigned short PaceBoundary[4] = {20, 60, 100, 140}; //Boundary in DAC code

static int s4AF_WriteReg(u16 a_u2Data)
{
	int i4RetValue = 0;

	char puSendCmd[3] = { 0x03, (char)(a_u2Data >> 8), (char)(a_u2Data & 0xFF) };

	g_pstAF_I2Cclient->addr = AF_I2C_SLAVE_ADDR;

	g_pstAF_I2Cclient->addr = g_pstAF_I2Cclient->addr >> 1;

	i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puSendCmd, 3);

	if (i4RetValue < 0) {
		LOG_INF("I2C send failed!!\n");
		return -1;
	}

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

	if (copy_to_user(pstMotorInfo, &stMotorInfo, sizeof(struct stAF_MotorInfo)))
		LOG_INF("copy to user failed when getting motor information\n");

	return 0;
}

static int initdrv(void)
{
	int i4RetValue = 0;
	char puSendCmdArray[7][2] = {
	{0x02, 0x01}, {0x02, 0x00}, {0xFE, 0xFE},
	{0x02, 0x02}, {0x06, 0x80}, {0x07, 0x7F}, {0xFE, 0xFE},
	};
	unsigned char cmd_number;

	LOG_INF("InitDrv[1] %p, %p\n", &(puSendCmdArray[1][0]), puSendCmdArray[1]);
	LOG_INF("InitDrv[2] %p, %p\n", &(puSendCmdArray[2][0]), puSendCmdArray[2]);

	g_pstAF_I2Cclient->addr = AF_I2C_SLAVE_ADDR;
	g_pstAF_I2Cclient->addr = g_pstAF_I2Cclient->addr >> 1;

	for (cmd_number = 0; cmd_number < 7; cmd_number++) {
		if (puSendCmdArray[cmd_number][0] != 0xFE) {
			i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puSendCmdArray[cmd_number], 2);

			if (i4RetValue < 0)
				return -1;
		} else {
			udelay(100);
		}
	}

	return i4RetValue;
}


static inline int moveAF(unsigned long a_u4Position)
{
	int ret = 0;

	if ((a_u4Position > g_u4AF_MACRO) || (a_u4Position < g_u4AF_INF)) {
		LOG_INF("out of range\n");
		return -EINVAL;
	}

	if (*g_pAF_Opened == 1) {
		initdrv();

		spin_lock(g_pAF_SpinLock);
		g_u4CurrPosition = AF_NO_NOISE_POSITION_CODE;
		spin_unlock(g_pAF_SpinLock);

		if (g_u4CurrPosition == a_u4Position) {
			return 0;
		} else {
			if (s4AF_WriteReg(AF_NO_NOISE_POSITION_CODE) != 0) {
				LOG_INF("set I2C failed when moving the motor\n");
				ret = -1;
			}
		}

		spin_lock(g_pAF_SpinLock);
		*g_pAF_Opened = 2;
		spin_unlock(g_pAF_SpinLock);
	}

	if (g_u4CurrPosition == a_u4Position)
		return 0;

	spin_lock(g_pAF_SpinLock);
	g_u4TargetPosition = a_u4Position;
	spin_unlock(g_pAF_SpinLock);

	/* LOG_INF("move [curr] %d [target] %d\n", g_u4CurrPosition, g_u4TargetPosition); */

	if (s4AF_WriteReg((unsigned short)g_u4TargetPosition) == 0) {
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
long DW9800AF_Ioctl_Sub2(struct file *a_pstFile, unsigned int a_u4Command, unsigned long a_u4Param)
{
	long i4RetValue = 0;

	switch (a_u4Command) {
	case AFIOC_G_MOTORINFO:
		i4RetValue = getAFInfo((__user struct stAF_MotorInfo *) (a_u4Param));
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
		/*LOG_INF("No CMD\n");*/
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
int DW9800AF_Release_Sub2(struct inode *a_pstInode, struct file *a_pstFile)
{
	unsigned short RelaPos = 0;
	LOG_INF("Start\n");
	if (*g_pAF_Opened == 2) {
		LOG_INF("Wait\n");

		if (g_u4CurrPosition > AF_NO_NOISE_POSITION_CODE) {
			RelaPos = g_u4CurrPosition - AF_NO_NOISE_POSITION_CODE;
		} else {
			RelaPos = AF_NO_NOISE_POSITION_CODE - g_u4CurrPosition;
		}
		while (RelaPos > 0) {
			if (RelaPos > PaceBoundary[3]) {
				RelaPos -= PaceBoundary[3];
			} else if (RelaPos > PaceBoundary[2]) {
				RelaPos -= PaceBoundary[2];
			} else if (RelaPos > PaceBoundary[1]) {
				RelaPos -= PaceBoundary[1];
			} else if (RelaPos > PaceBoundary[0]) {
				RelaPos -= PaceBoundary[0];
			} else {
				RelaPos = 0;
			}

			if (g_u4CurrPosition > AF_NO_NOISE_POSITION_CODE) {
				s4AF_WriteReg(AF_NO_NOISE_POSITION_CODE + RelaPos);
				mdelay(5);
			} else {
				s4AF_WriteReg(AF_NO_NOISE_POSITION_CODE - RelaPos);
				mdelay(5);
			}
		}
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

int DW9800AF_SetI2Cclient_Sub2(struct i2c_client *pstAF_I2Cclient, spinlock_t *pAF_SpinLock, int *pAF_Opened)
{
	g_pstAF_I2Cclient = pstAF_I2Cclient;
	g_pAF_SpinLock = pAF_SpinLock;
	g_pAF_Opened = pAF_Opened;

	return 1;
}

int DW9800AF_GetFileName_Sub2(unsigned char *pFileName)
{
#if SUPPORT_GETTING_LENS_FOLDER_NAME
	char FilePath[256];
	char *FileString;

	sprintf(FilePath, "%s", __FILE__);
	FileString = strrchr(FilePath, '/');
	*FileString = '\0';
	FileString = (strrchr(FilePath, '/') + 1);
	strncpy(pFileName, FileString, AF_MOTOR_NAME);
	LOG_INF("FileName : %s\n", pFileName);
#else
	pFileName[0] = '\0';
#endif
	return 1;
}
