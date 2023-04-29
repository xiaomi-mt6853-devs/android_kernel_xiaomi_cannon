/*
 * Copyright (C) 2018 MediaTek Inc.
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

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/types.h>

#include "ov02b10ofilm_mipiraw_Sensor.h"

#define PFX "ov02b_camera_sensor"
#define LOG_INF(format, args...)    \
	pr_debug(PFX "[%s] " format, __func__, ##args)

/* Camera Hardwareinfo */
//extern struct global_otp_struct hw_info_main2_otp;
static kal_uint32 streaming_control(kal_bool enable);
static DEFINE_SPINLOCK(imgsensor_drv_lock);

static struct imgsensor_info_struct imgsensor_info = {
    .sensor_id = OV02B10OFILM_SENSOR_ID,
    .checksum_value = 0xb7c53a42,       //0x6d01485c // Auto Test Mode 蓄板..

    .pre = {
            .pclk = 16500000,            //record different mode's pclk
            .linelength = 448,            //record different mode's linelength
            .framelength = 1221,            //record different mode's framelength
            .startx = 0,                    //record different mode's startx of grabwindow
            .starty = 0,                    //record different mode's starty of grabwindow
            .grabwindow_width = 1600,        //record different mode's width of grabwindow
            .grabwindow_height = 1200,        //record different mode's height of grabwindow
             /*     following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario    */
            .mipi_data_lp2hs_settle_dc = 85,//unit , ns
			.mipi_pixel_rate = 33000000,
              /*     following for GetDefaultFramerateByScenario()    */
            .max_framerate = 300,
    },
    .cap = {
            .pclk = 16500000,            //record different mode's pclk
            .linelength = 448,            //record different mode's linelength
            .framelength = 1221,            //record different mode's framelength
            .startx = 0,                    //record different mode's startx of grabwindow
            .starty = 0,                    //record different mode's starty of grabwindow
            .grabwindow_width = 1600,        //record different mode's width of grabwindow
            .grabwindow_height = 1200,        //record different mode's height of grabwindow
             /*     following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario    */
            .mipi_data_lp2hs_settle_dc = 85,//unit , ns
	         .mipi_pixel_rate = 33000000,
              /*     following for GetDefaultFramerateByScenario()    */
            .max_framerate = 300,
    },
    .cap1 = {
            .pclk = 16500000,            //record different mode's pclk
            .linelength = 448,            //record different mode's linelength
            .framelength = 1221,            //record different mode's framelength
            .startx = 0,                    //record different mode's startx of grabwindow
            .starty = 0,                    //record different mode's starty of grabwindow
            .grabwindow_width = 1600,        //record different mode's width of grabwindow
            .grabwindow_height = 1200,        //record different mode's height of grabwindow
             /*     following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario    */
            .mipi_data_lp2hs_settle_dc = 85,//unit , ns
			  .mipi_pixel_rate = 33000000,
              /*     following for GetDefaultFramerateByScenario()    */
            .max_framerate = 300,
    },
    .normal_video = {
            .pclk = 16500000,            //record different mode's pclk
            .linelength = 448,            //record different mode's linelength
            .framelength = 1221,            //record different mode's framelength
            .startx = 0,                    //record different mode's startx of grabwindow
            .starty = 0,                    //record different mode's starty of grabwindow
            .grabwindow_width = 1600,        //record different mode's width of grabwindow
            .grabwindow_height = 1200,        //record different mode's height of grabwindow
             /*     following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario    */
  /*     following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario    */
            .mipi_data_lp2hs_settle_dc = 85,//unit , ns
			  .mipi_pixel_rate = 33000000,
              /*     following for GetDefaultFramerateByScenario()    */
            .max_framerate = 300,
	},
	.hs_video = {
		.pclk = 16500000,
		.linelength = 448,
		.framelength = 1221,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1600,
		.grabwindow_height = 1200,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 33000000,
		.max_framerate = 300,
	},
	.slim_video = {
		.pclk = 16500000,
		.linelength = 448,
		.framelength = 1221,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1600,
		.grabwindow_height = 1200,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 33000000,
		.max_framerate = 300,
	},
    .custom1 = {
            .pclk = 16500000,            //record different mode's pclk
            .linelength = 448,            //record different mode's linelength
            .framelength = 1221,            //record different mode's framelength
            .startx = 0,                    //record different mode's startx of grabwindow
            .starty = 0,                    //record different mode's starty of grabwindow
            .grabwindow_width = 1600,        //record different mode's width of grabwindow
            .grabwindow_height = 1200,        //record different mode's height of grabwindow
             /*     following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario    */
            .mipi_data_lp2hs_settle_dc = 85,//unit , ns
			 .mipi_pixel_rate = 33000000,
              /*     following for GetDefaultFramerateByScenario()    */
            .max_framerate = 300,
    },

    .margin = 7,            //sensor framelength & shutter margin
    .min_shutter = 4,        //min shutter
		.min_gain = 64,
		.max_gain = 1024,
		.min_gain_iso = 50,
		.gain_step = 2,
		.gain_type = 2,
    .max_frame_length = 0x7fff,//max framelength by sensor register's limitation
    .ae_shut_delay_frame = 0,    //shutter delay frame for AE cycle, 2 frame with ispGain_delay-shut_delay=2-0=2
    .ae_sensor_gain_delay_frame = 0,//sensor gain delay frame for AE cycle,2 frame with ispGain_delay-sensor_gain_delay=2-0=2
    .ae_ispGain_delay_frame = 2,//isp gain delay frame for AE cycle

    .ihdr_support = 0,      //1, support; 0,not support
    .ihdr_le_firstline = 0,  //1,le first ; 0, se first
	.sensor_mode_num = 6,

    .cap_delay_frame = 2,
    .pre_delay_frame = 2,
    .video_delay_frame = 2,
	.hs_video_delay_frame = 2,
	.slim_video_delay_frame = 2,
    .custom1_delay_frame = 2,
    .frame_time_delay_frame = 1,
    .isp_driving_current = ISP_DRIVING_6MA, //mclk driving current
    .sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,//sensor_interface_type
    .mipi_sensor_type = MIPI_OPHY_NCSI2, //0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2
    .mipi_settle_delay_mode = MIPI_SETTLEDELAY_AUTO,//0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
    .sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_MONO,//sensor output first pixel color
    .mclk = 24,//mclk value, suggest 24 or 26 for 24Mhz or 26Mhz
    .mipi_lane_num = SENSOR_MIPI_1_LANE,//mipi lane num
    .i2c_addr_table = {0x78,0xff},
    .i2c_speed = 400,
};


static struct imgsensor_struct imgsensor = {
    .mirror = IMAGE_NORMAL,                //mirrorflip information
    .sensor_mode = IMGSENSOR_MODE_INIT, //IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video
    .shutter = 0x3D0,                    //current shutter
    .gain = 0x100,                        //current gain
    .dummy_pixel = 0,                    //current dummypixel
    .dummy_line = 0,                    //current dummyline
    .current_fps = 300,  //full size current fps : 24fps for PIP, 30fps for Normal or ZSD
    .autoflicker_en = KAL_FALSE,  //auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
    .test_pattern = KAL_FALSE,        //test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output
    .current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,//current scenario id
    .ihdr_en = 0, //sensor need support LE, SE with HDR feature
    .i2c_write_id = 0x78,//record current sensor's i2c write id
};


/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[6] = {
	{1600, 1200, 0, 0, 1600, 1200, 1600, 1200, 0000, 0000, 1600, 1200, 0, 0, 1600, 1200}, /* Preview */
	{1600, 1200, 0, 0, 1600, 1200, 1600, 1200, 0000, 0000, 1600, 1200, 0, 0, 1600, 1200}, /* capture */
	{1600, 1200, 0, 0, 1600, 1200, 1600, 1200, 0000, 0000, 1600, 1200, 0, 0, 1600, 1200}, /* video */
	{1600, 1200, 0, 0, 1600, 1200, 1600, 1200, 0000, 0000, 1600, 1200, 0, 0, 1600, 1200}, /* HS video */
	{1600, 1200, 0, 0, 1600, 1200, 1600, 1200, 0000, 0000, 1600, 1200, 0, 0, 1600, 1200},  /* slim video */
	{1600, 1200, 0, 0, 1600, 1200, 1600, 1200, 0000, 0000, 1600, 1200, 0, 0, 1600, 1200}  /* custom 24fps */
};

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
    kal_uint16 get_byte=0;
    char pu_send_cmd[1] = {(char)(addr & 0xFF)};
    iReadRegI2C(pu_send_cmd, 1, (u8*)&get_byte, 1, imgsensor.i2c_write_id);

    return get_byte;
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
    char pu_send_cmd[2] = {(char)(addr & 0xFF), (char)(para & 0xFF)};
    iWriteRegI2C(pu_send_cmd, 2, imgsensor.i2c_write_id);
}


static void set_dummy(void)
{
    	if (imgsensor.frame_length%2 != 0) {
		imgsensor.frame_length = imgsensor.frame_length - imgsensor.frame_length % 2;
	}

	LOG_INF("imgsensor.frame_length = %d\n", imgsensor.frame_length);
	write_cmos_sensor(0xfd, 0x01);
	write_cmos_sensor(0x14, (imgsensor.frame_length - 0x4c4) >> 8);
    write_cmos_sensor(0x15, (imgsensor.frame_length - 0x4c4) & 0xFF);
    write_cmos_sensor(0xfe, 0x02);//fresh
}    /*    set_dummy  */

extern int s5kgm1st_ofilm_or_qtech;
static kal_uint32 return_sensor_id(void)
{
    kal_uint32 sensor_id = 0;
    write_cmos_sensor(0xfd, 0x00);
    if(1 ==s5kgm1st_ofilm_or_qtech)
    sensor_id = ((read_cmos_sensor(0x02) << 8) | read_cmos_sensor(0x03));
    return sensor_id;
}
static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{
    kal_uint32 frame_length = imgsensor.frame_length;


    frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;

    spin_lock(&imgsensor_drv_lock);
    imgsensor.frame_length = (frame_length > imgsensor.min_frame_length) ? frame_length : imgsensor.min_frame_length;
    imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
    if (imgsensor.frame_length > imgsensor_info.max_frame_length)
    {
        imgsensor.frame_length = imgsensor_info.max_frame_length;
        imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
    }
    if (min_framelength_en) {
        imgsensor.min_frame_length = imgsensor.frame_length;
    }
    spin_unlock(&imgsensor_drv_lock);

    set_dummy();
}    /*    set_max_framerate  */

static void write_shutter(kal_uint32 shutter)
{
    kal_uint32 realtime_fps = 0;

    spin_lock(&imgsensor_drv_lock);

    if (shutter > imgsensor.min_frame_length - imgsensor_info.margin) {
        imgsensor.frame_length = shutter + imgsensor_info.margin;
    } else {
	imgsensor.frame_length = imgsensor.min_frame_length;
    }
    if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
        imgsensor.frame_length = imgsensor_info.max_frame_length;
    }
    spin_unlock(&imgsensor_drv_lock);

    shutter = (shutter < imgsensor_info.min_shutter) ?
        imgsensor_info.min_shutter : shutter;
    shutter =
        (shutter > (imgsensor_info.max_frame_length -
        imgsensor_info.margin)) ? (imgsensor_info.max_frame_length -
        imgsensor_info.margin) : shutter;

//frame_length and shutter should be an even number.
    shutter = (shutter >> 1) << 1;
    imgsensor.frame_length = (imgsensor.frame_length >> 1) << 1;
//auroflicker:need to avoid 15fps and 30 fps
    if (imgsensor.autoflicker_en == KAL_TRUE) {
        realtime_fps = imgsensor.pclk /
            imgsensor.line_length * 10 / imgsensor.frame_length;
        if (realtime_fps >= 297 && realtime_fps <= 305) {
            realtime_fps = 296;
            set_max_framerate(realtime_fps, 0);
        } else if (realtime_fps >= 147 && realtime_fps <= 150) {
            realtime_fps = 146;
            set_max_framerate(realtime_fps, 0);
        } else {
            imgsensor.frame_length = (imgsensor.frame_length  >> 1) << 1;
            write_cmos_sensor(0xfd, 0x01);
            write_cmos_sensor(0x14, (imgsensor.frame_length - 0x4c4) >> 8);
            write_cmos_sensor(0x15, (imgsensor.frame_length - 0x4c4) & 0xFF);
            write_cmos_sensor(0xfe, 0x02);//fresh
            }
    } else {
        imgsensor.frame_length = (imgsensor.frame_length  >> 1) << 1;
        write_cmos_sensor(0xfd, 0x01);
        write_cmos_sensor(0x14, (imgsensor.frame_length - 0x4c4) >> 8);
        write_cmos_sensor(0x15, (imgsensor.frame_length - 0x4c4) & 0xFF);
        write_cmos_sensor(0xfe, 0x02);//fresh
   }

    write_cmos_sensor(0xfd, 0x01);
    write_cmos_sensor(0x0e, (shutter >> 8) & 0xFF);
    write_cmos_sensor(0x0f, shutter  & 0xFF);
    write_cmos_sensor(0xfe, 0x02);//fresh sss

    //LOG_INF("shutter =%d, framelength =%d\n", shutter,imgsensor.frame_length);
}

static void set_shutter(kal_uint32 shutter)
{
    unsigned long flags;
    spin_lock_irqsave(&imgsensor_drv_lock, flags);
    imgsensor.shutter = shutter;
    spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

    write_shutter(shutter);
	}
static void set_shutter_frame_length(kal_uint16 shutter,
			kal_uint16 frame_length)
{
	kal_uint16 realtime_fps = 0;
	kal_int32 dummy_line = 0;

	//spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	//spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	spin_lock(&imgsensor_drv_lock);
	if (frame_length > 1) {
		dummy_line = frame_length - imgsensor.frame_length;
	}
	imgsensor.frame_length = imgsensor.frame_length + dummy_line;

	if (shutter > imgsensor.frame_length - imgsensor_info.margin) {
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	}
	if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	}

	spin_unlock(&imgsensor_drv_lock);

	shutter = (shutter < imgsensor_info.min_shutter) ?
		imgsensor_info.min_shutter : shutter;
	shutter =
		(shutter > (imgsensor_info.max_frame_length -
		imgsensor_info.margin)) ? (imgsensor_info.max_frame_length -
		imgsensor_info.margin) : shutter;

	//frame_length and shutter should be an even number.
	shutter = (shutter >> 1) << 1;
	imgsensor.frame_length = (imgsensor.frame_length >> 1) << 1;
//auroflicker:need to avoid 15fps and 30 fps
	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk /
			imgsensor.line_length * 10 / imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305) {
			realtime_fps = 296;
	    set_max_framerate(realtime_fps, 0);
		} else if (realtime_fps >= 147 && realtime_fps <= 150) {
			realtime_fps = 146;
	    set_max_framerate(realtime_fps, 0);
		} else {
		imgsensor.frame_length = (imgsensor.frame_length  >> 1) << 1;
        write_cmos_sensor(0xfd, 0x01);
	    write_cmos_sensor(0x14, (imgsensor.frame_length - 0x4c4) >> 8);
	    write_cmos_sensor(0x15, (imgsensor.frame_length - 0x4c4) & 0xFF);
        write_cmos_sensor(0xfe, 0x02);
		}
	} else {
	    imgsensor.frame_length = (imgsensor.frame_length  >> 1) << 1;
        write_cmos_sensor(0xfd, 0x01);
	    write_cmos_sensor(0x14, (imgsensor.frame_length - 0x4c4) >> 8);
	    write_cmos_sensor(0x15, (imgsensor.frame_length - 0x4c4) & 0xFF);
        write_cmos_sensor(0xfe, 0x02);
	}

	/* Update Shutter */
	 write_cmos_sensor(0xfd, 0x01);
	 write_cmos_sensor(0x0e, (shutter >> 8) & 0xFF);
	 write_cmos_sensor(0x0f, shutter  & 0xFF);
	 write_cmos_sensor(0xfe, 0x02);

	//LOG_INF("shutter =%d, framelength =%d, realtime_fps =%d\n",
	//	shutter, imgsensor.frame_length, realtime_fps);
}				/* set_shutter_frame_length */




/*************************************************************************
 * FUNCTION
 *    set_gain
 *
 * DESCRIPTION
 *    This function is to set global gain to sensor.
 *
 * PARAMETERS
 *    iGain : sensor global gain(base: 0x40)
 *
 * RETURNS
 *    the actually gain set to sensor.
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
    kal_uint8  iReg;

    if((gain >= 0x40) && (gain <= (15*0x40))) //base gain = 0x40
    {
        iReg = 0x10 * gain/BASEGAIN;        //change mtk gain base to aptina gain base

        if(iReg<=0x10)
        {
            write_cmos_sensor(0xfd, 0x01);
            write_cmos_sensor(0x22, 0x10);//0x23
            write_cmos_sensor(0xfe, 0x02);//fresh
            LOG_INF("OV02BMIPI_SetGain = 16");
        }
        else if(iReg>= 0xf8)//gpw
        {
            write_cmos_sensor(0xfd, 0x01);
            write_cmos_sensor(0x22,0xf8);
            write_cmos_sensor(0xfe, 0x02);//fresh
            LOG_INF("OV02BMIPI_SetGain = 160");
        }
        else
        {
            write_cmos_sensor(0xfd, 0x01);
            write_cmos_sensor(0x22, (kal_uint8)iReg);
            write_cmos_sensor(0xfe, 0x02);//fresh
            LOG_INF("OV02BMIPI_SetGain = %d",iReg);
        }
    }
    else
        LOG_INF("error gain setting");

    return gain;
}    /*    set_gain  */

static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
	LOG_INF("le: 0x%x, se: 0x%x, gain: 0x%x\n", le, se, gain);
}
static void sensor_init(void)
{
    write_cmos_sensor(0xfc, 0x01);
    write_cmos_sensor(0xfd, 0x00);
    write_cmos_sensor(0xfd, 0x00);
    write_cmos_sensor(0x24, 0x02);
    write_cmos_sensor(0x25, 0x06);
    write_cmos_sensor(0x29, 0x03);
    write_cmos_sensor(0x2a, 0x34);
    write_cmos_sensor(0x1e, 0x17);
    write_cmos_sensor(0x33, 0x07);
    write_cmos_sensor(0x35, 0x07);
    write_cmos_sensor(0x4a, 0x0c);
    write_cmos_sensor(0x3a, 0x05);
    write_cmos_sensor(0x3b, 0x02);
    write_cmos_sensor(0x3e, 0x00);
    write_cmos_sensor(0x46, 0x01);
    write_cmos_sensor(0x6d, 0x03);
    write_cmos_sensor(0xfd, 0x01);
    write_cmos_sensor(0x0e, 0x02);
    write_cmos_sensor(0x0f, 0x1a);
    write_cmos_sensor(0x18, 0x00);
    write_cmos_sensor(0x22, 0xff);
    write_cmos_sensor(0x23, 0x02);
    write_cmos_sensor(0x17, 0x2c);
    write_cmos_sensor(0x19, 0x20);
    write_cmos_sensor(0x1b, 0x06);
    write_cmos_sensor(0x1c, 0x04);
    write_cmos_sensor(0x20, 0x03);
    write_cmos_sensor(0x30, 0x01);
    write_cmos_sensor(0x33, 0x01);
    write_cmos_sensor(0x31, 0x0a);
    write_cmos_sensor(0x32, 0x09);
    write_cmos_sensor(0x38, 0x01);
    write_cmos_sensor(0x39, 0x01);
    write_cmos_sensor(0x3a, 0x01);
    write_cmos_sensor(0x3b, 0x01);
    write_cmos_sensor(0x4f, 0x04);
    write_cmos_sensor(0x4e, 0x05);
    write_cmos_sensor(0x50, 0x01);
    write_cmos_sensor(0x35, 0x0c);
    write_cmos_sensor(0x45, 0x2a);
    write_cmos_sensor(0x46, 0x2a);
    write_cmos_sensor(0x47, 0x2a);
    write_cmos_sensor(0x48, 0x2a);
    write_cmos_sensor(0x4a, 0x2c);
    write_cmos_sensor(0x4b, 0x2c);
    write_cmos_sensor(0x4c, 0x2c);
    write_cmos_sensor(0x4d, 0x2c);
    write_cmos_sensor(0x56, 0x3a);
    write_cmos_sensor(0x57, 0x0a);
    write_cmos_sensor(0x58, 0x24);
    write_cmos_sensor(0x59, 0x20);
    write_cmos_sensor(0x5a, 0x0a);
    write_cmos_sensor(0x5b, 0xff);
    write_cmos_sensor(0x37, 0x0a);
    write_cmos_sensor(0x42, 0x0e);
    write_cmos_sensor(0x68, 0x90);
    write_cmos_sensor(0x69, 0xcd);
    write_cmos_sensor(0x6a, 0x8f);
    write_cmos_sensor(0x7c, 0x0a);
    write_cmos_sensor(0x7d, 0x0a);
    write_cmos_sensor(0x7e, 0x0a);
    write_cmos_sensor(0x7f, 0x08);
    write_cmos_sensor(0x83, 0x14);
    write_cmos_sensor(0x84, 0x14);
    write_cmos_sensor(0x86, 0x14);
    write_cmos_sensor(0x87, 0x07);
    write_cmos_sensor(0x88, 0x0f);
    write_cmos_sensor(0x94, 0x02);
    write_cmos_sensor(0x98, 0xd1);
    write_cmos_sensor(0xfe, 0x02);
    write_cmos_sensor(0xfd, 0x03);
    write_cmos_sensor(0x97,0x6c);
    write_cmos_sensor(0x98,0x60);
    write_cmos_sensor(0x99,0x60);
    write_cmos_sensor(0x9a,0x6c);
    write_cmos_sensor(0xa1,0x40);
    write_cmos_sensor(0xaf,0x04);
    write_cmos_sensor(0xb1,0x40);
    write_cmos_sensor(0xae, 0x0d);
    write_cmos_sensor(0x88,0x5b);
    write_cmos_sensor(0x89, 0x7c);
    write_cmos_sensor(0xb4, 0x05);
    write_cmos_sensor(0x8c, 0x40);
    write_cmos_sensor(0x8e, 0x40);
    write_cmos_sensor(0x90, 0x40);
    write_cmos_sensor(0x92, 0x40);
    write_cmos_sensor(0x9b,0x46);
    write_cmos_sensor(0xac, 0x40);
    write_cmos_sensor(0xfd, 0x00);
    write_cmos_sensor(0x5a, 0x15);
    write_cmos_sensor(0x74, 0x01);
    write_cmos_sensor(0xfd, 0x00);
    write_cmos_sensor(0x50, 0x40);
    write_cmos_sensor(0x52, 0xb0);
    write_cmos_sensor(0xfd, 0x01);
    write_cmos_sensor(0x03, 0x70);
    write_cmos_sensor(0x05, 0x10);
    write_cmos_sensor(0x07, 0x20);
    write_cmos_sensor(0x09, 0xb0);
    write_cmos_sensor(0xfd, 0x03);
    write_cmos_sensor(0xfb, 0x01);
}   
/*    MIPI_sensor_Init  */

static void preview_setting(void)
{
write_cmos_sensor(0xfc, 0x01);
write_cmos_sensor(0xfd, 0x00);
write_cmos_sensor(0xfd, 0x00);
write_cmos_sensor(0x24, 0x02);
write_cmos_sensor(0x25, 0x06);
write_cmos_sensor(0x29, 0x03);
write_cmos_sensor(0x2a, 0x34);
write_cmos_sensor(0x1e, 0x17);
write_cmos_sensor(0x33, 0x07);
write_cmos_sensor(0x35, 0x07);
write_cmos_sensor(0x4a, 0x0c);
write_cmos_sensor(0x3a, 0x05);
write_cmos_sensor(0x3b, 0x02);
write_cmos_sensor(0x3e, 0x00);
write_cmos_sensor(0x46, 0x01);
write_cmos_sensor(0x6d, 0x03);
write_cmos_sensor(0xfd, 0x01);
write_cmos_sensor(0x0e, 0x02);
write_cmos_sensor(0x0f, 0x1a);
write_cmos_sensor(0x18, 0x00);
write_cmos_sensor(0x22, 0xff);
write_cmos_sensor(0x23, 0x02);
write_cmos_sensor(0x17, 0x2c);
write_cmos_sensor(0x19, 0x20);
write_cmos_sensor(0x1b, 0x06);
write_cmos_sensor(0x1c, 0x04);
write_cmos_sensor(0x20, 0x03);
write_cmos_sensor(0x30, 0x01);
write_cmos_sensor(0x33, 0x01);
write_cmos_sensor(0x31, 0x0a);
write_cmos_sensor(0x32, 0x09);
write_cmos_sensor(0x38, 0x01);
write_cmos_sensor(0x39, 0x01);
write_cmos_sensor(0x3a, 0x01);
write_cmos_sensor(0x3b, 0x01);
write_cmos_sensor(0x4f, 0x04);
write_cmos_sensor(0x4e, 0x05);
write_cmos_sensor(0x50, 0x01);
write_cmos_sensor(0x35, 0x0c);
write_cmos_sensor(0x45, 0x2a);
write_cmos_sensor(0x46, 0x2a);
write_cmos_sensor(0x47, 0x2a);
write_cmos_sensor(0x48, 0x2a);
write_cmos_sensor(0x4a, 0x2c);
write_cmos_sensor(0x4b, 0x2c);
write_cmos_sensor(0x4c, 0x2c);
write_cmos_sensor(0x4d, 0x2c);
write_cmos_sensor(0x56, 0x3a);
write_cmos_sensor(0x57, 0x0a);
write_cmos_sensor(0x58, 0x24);
write_cmos_sensor(0x59, 0x20);
write_cmos_sensor(0x5a, 0x0a);
write_cmos_sensor(0x5b, 0xff);
write_cmos_sensor(0x37, 0x0a);
write_cmos_sensor(0x42, 0x0e);
write_cmos_sensor(0x68, 0x90);
write_cmos_sensor(0x69, 0xcd);
write_cmos_sensor(0x6a, 0x8f);
write_cmos_sensor(0x7c,0x0a);
write_cmos_sensor(0x7d,0x0a);
write_cmos_sensor(0x7e,0x0a);
write_cmos_sensor(0x7f, 0x08);
write_cmos_sensor(0x83, 0x14);
write_cmos_sensor(0x84, 0x14);
write_cmos_sensor(0x86, 0x14);
write_cmos_sensor(0x87, 0x07);
write_cmos_sensor(0x88, 0x0f);
write_cmos_sensor(0x94, 0x02);
write_cmos_sensor(0x98, 0xd1);
write_cmos_sensor(0x14, 0x00);
write_cmos_sensor(0x15, 0x01);
write_cmos_sensor(0xfe, 0x02);
write_cmos_sensor(0xfd, 0x03);
write_cmos_sensor(0x97,0x6c);
write_cmos_sensor(0x98,0x60);
write_cmos_sensor(0x99,0x60);
write_cmos_sensor(0x9a,0x6c);
write_cmos_sensor(0xa1,0x40);
write_cmos_sensor(0xaf,0x04);
write_cmos_sensor(0xb1,0x40);
write_cmos_sensor(0xae, 0x0d);
write_cmos_sensor(0x88,0x5b);
write_cmos_sensor(0x89, 0x7c);
write_cmos_sensor(0xb4, 0x05);
write_cmos_sensor(0x8c, 0x40);
write_cmos_sensor(0x8e, 0x40);
write_cmos_sensor(0x90, 0x40);
write_cmos_sensor(0x92, 0x40);
write_cmos_sensor(0x9b,0x46);
write_cmos_sensor(0xac, 0x40);
write_cmos_sensor(0xfd, 0x00);
write_cmos_sensor(0x5a, 0x15);
write_cmos_sensor(0x74, 0x01);
write_cmos_sensor(0xfd, 0x00);
write_cmos_sensor(0x50, 0x40);
write_cmos_sensor(0x52, 0xb0);
write_cmos_sensor(0xfd, 0x01);
write_cmos_sensor(0x03, 0x70);
write_cmos_sensor(0x05, 0x10);
write_cmos_sensor(0x07, 0x20);
write_cmos_sensor(0x09, 0xb0);
write_cmos_sensor(0xfd, 0x03);
write_cmos_sensor(0xc2, 0x01);
write_cmos_sensor(0xfb, 0x01);
}


static void capture_setting(void)
{
write_cmos_sensor(0xfc, 0x01);
write_cmos_sensor(0xfd, 0x00);
write_cmos_sensor(0xfd, 0x00);
write_cmos_sensor(0x24, 0x02);
write_cmos_sensor(0x25, 0x06);
write_cmos_sensor(0x29, 0x03);
write_cmos_sensor(0x2a, 0x34);
write_cmos_sensor(0x1e, 0x17);
write_cmos_sensor(0x33, 0x07);
write_cmos_sensor(0x35, 0x07);
write_cmos_sensor(0x4a, 0x0c);
write_cmos_sensor(0x3a, 0x05);
write_cmos_sensor(0x3b, 0x02);
write_cmos_sensor(0x3e, 0x00);
write_cmos_sensor(0x46, 0x01);
write_cmos_sensor(0x6d, 0x03);
write_cmos_sensor(0xfd, 0x01);
write_cmos_sensor(0x0e, 0x02);
write_cmos_sensor(0x0f, 0x1a);
write_cmos_sensor(0x18, 0x00);
write_cmos_sensor(0x22, 0xff);
write_cmos_sensor(0x23, 0x02);
write_cmos_sensor(0x17, 0x2c);
write_cmos_sensor(0x19, 0x20);
write_cmos_sensor(0x1b, 0x06);
write_cmos_sensor(0x1c, 0x04);
write_cmos_sensor(0x20, 0x03);
write_cmos_sensor(0x30, 0x01);
write_cmos_sensor(0x33, 0x01);
write_cmos_sensor(0x31, 0x0a);
write_cmos_sensor(0x32, 0x09);
write_cmos_sensor(0x38, 0x01);
write_cmos_sensor(0x39, 0x01);
write_cmos_sensor(0x3a, 0x01);
write_cmos_sensor(0x3b, 0x01);
write_cmos_sensor(0x4f, 0x04);
write_cmos_sensor(0x4e, 0x05);
write_cmos_sensor(0x50, 0x01);
write_cmos_sensor(0x35, 0x0c);
write_cmos_sensor(0x45, 0x2a);
write_cmos_sensor(0x46, 0x2a);
write_cmos_sensor(0x47, 0x2a);
write_cmos_sensor(0x48, 0x2a);
write_cmos_sensor(0x4a, 0x2c);
write_cmos_sensor(0x4b, 0x2c);
write_cmos_sensor(0x4c, 0x2c);
write_cmos_sensor(0x4d, 0x2c);
write_cmos_sensor(0x56, 0x3a);
write_cmos_sensor(0x57, 0x0a);
write_cmos_sensor(0x58, 0x24);
write_cmos_sensor(0x59, 0x20);
write_cmos_sensor(0x5a, 0x0a);
write_cmos_sensor(0x5b, 0xff);
write_cmos_sensor(0x37, 0x0a);
write_cmos_sensor(0x42, 0x0e);
write_cmos_sensor(0x68, 0x90);
write_cmos_sensor(0x69, 0xcd);
write_cmos_sensor(0x6a, 0x8f);
write_cmos_sensor(0x7c,0x0a);
write_cmos_sensor(0x7d,0x0a);
write_cmos_sensor(0x7e,0x0a);
write_cmos_sensor(0x7f, 0x08);
write_cmos_sensor(0x83, 0x14);
write_cmos_sensor(0x84, 0x14);
write_cmos_sensor(0x86, 0x14);
write_cmos_sensor(0x87, 0x07);
write_cmos_sensor(0x88, 0x0f);
write_cmos_sensor(0x94, 0x02);
write_cmos_sensor(0x98, 0xd1);
write_cmos_sensor(0x14, 0x00);
write_cmos_sensor(0x15, 0x01);
write_cmos_sensor(0xfe, 0x02);
write_cmos_sensor(0xfd, 0x03);
write_cmos_sensor(0x97,0x6c);
write_cmos_sensor(0x98,0x60);
write_cmos_sensor(0x99,0x60);
write_cmos_sensor(0x9a,0x6c);
write_cmos_sensor(0xa1,0x40);
write_cmos_sensor(0xaf,0x04);
write_cmos_sensor(0xb1,0x40);
write_cmos_sensor(0xae, 0x0d);
write_cmos_sensor(0x88,0x5b);
write_cmos_sensor(0x89, 0x7c);
write_cmos_sensor(0xb4, 0x05);
write_cmos_sensor(0x8c, 0x40);
write_cmos_sensor(0x8e, 0x40);
write_cmos_sensor(0x90, 0x40);
write_cmos_sensor(0x92, 0x40);
write_cmos_sensor(0x9b,0x46);
write_cmos_sensor(0xac, 0x40);
write_cmos_sensor(0xfd, 0x00);
write_cmos_sensor(0x5a, 0x15);
write_cmos_sensor(0x74, 0x01);
write_cmos_sensor(0xfd, 0x00);
write_cmos_sensor(0x50, 0x40);
write_cmos_sensor(0x52, 0xb0);
write_cmos_sensor(0xfd, 0x01);
write_cmos_sensor(0x03, 0x70);
write_cmos_sensor(0x05, 0x10);
write_cmos_sensor(0x07, 0x20);
write_cmos_sensor(0x09, 0xb0);
write_cmos_sensor(0xfd, 0x03);
write_cmos_sensor(0xfb, 0x01);
}    /*    capture_setting  */

static void normal_video_setting(void)
{
write_cmos_sensor(0xfc, 0x01);
write_cmos_sensor(0xfd, 0x00);
write_cmos_sensor(0xfd, 0x00);
write_cmos_sensor(0x24, 0x02);
write_cmos_sensor(0x25, 0x06);
write_cmos_sensor(0x29, 0x03);
write_cmos_sensor(0x2a, 0x34);
write_cmos_sensor(0x1e, 0x17);
write_cmos_sensor(0x33, 0x07);
write_cmos_sensor(0x35, 0x07);
write_cmos_sensor(0x4a, 0x0c);
write_cmos_sensor(0x3a, 0x05);
write_cmos_sensor(0x3b, 0x02);
write_cmos_sensor(0x3e, 0x00);
write_cmos_sensor(0x46, 0x01);
write_cmos_sensor(0x6d, 0x03);
write_cmos_sensor(0xfd, 0x01);
write_cmos_sensor(0x0e, 0x02);
write_cmos_sensor(0x0f, 0x1a);
write_cmos_sensor(0x18, 0x00);
write_cmos_sensor(0x22, 0xff);
write_cmos_sensor(0x23, 0x02);
write_cmos_sensor(0x17, 0x2c);
write_cmos_sensor(0x19, 0x20);
write_cmos_sensor(0x1b, 0x06);
write_cmos_sensor(0x1c, 0x04);
write_cmos_sensor(0x20, 0x03);
write_cmos_sensor(0x30, 0x01);
write_cmos_sensor(0x33, 0x01);
write_cmos_sensor(0x31, 0x0a);
write_cmos_sensor(0x32, 0x09);
write_cmos_sensor(0x38, 0x01);
write_cmos_sensor(0x39, 0x01);
write_cmos_sensor(0x3a, 0x01);
write_cmos_sensor(0x3b, 0x01);
write_cmos_sensor(0x4f, 0x04);
write_cmos_sensor(0x4e, 0x05);
write_cmos_sensor(0x50, 0x01);
write_cmos_sensor(0x35, 0x0c);
write_cmos_sensor(0x45, 0x2a);
write_cmos_sensor(0x46, 0x2a);
write_cmos_sensor(0x47, 0x2a);
write_cmos_sensor(0x48, 0x2a);
write_cmos_sensor(0x4a, 0x2c);
write_cmos_sensor(0x4b, 0x2c);
write_cmos_sensor(0x4c, 0x2c);
write_cmos_sensor(0x4d, 0x2c);
write_cmos_sensor(0x56, 0x3a);
write_cmos_sensor(0x57, 0x0a);
write_cmos_sensor(0x58, 0x24);
write_cmos_sensor(0x59, 0x20);
write_cmos_sensor(0x5a, 0x0a);
write_cmos_sensor(0x5b, 0xff);
write_cmos_sensor(0x37, 0x0a);
write_cmos_sensor(0x42, 0x0e);
write_cmos_sensor(0x68, 0x90);
write_cmos_sensor(0x69, 0xcd);
write_cmos_sensor(0x6a, 0x8f);
write_cmos_sensor(0x7c,0x0a);
write_cmos_sensor(0x7d,0x0a);
write_cmos_sensor(0x7e,0x0a);
write_cmos_sensor(0x7f, 0x08);
write_cmos_sensor(0x83, 0x14);
write_cmos_sensor(0x84, 0x14);
write_cmos_sensor(0x86, 0x14);
write_cmos_sensor(0x87, 0x07);
write_cmos_sensor(0x88, 0x0f);
write_cmos_sensor(0x94, 0x02);
write_cmos_sensor(0x98, 0xd1);
write_cmos_sensor(0x14, 0x00);
write_cmos_sensor(0x15, 0x01);
write_cmos_sensor(0xfe, 0x02);
write_cmos_sensor(0xfd, 0x03);
write_cmos_sensor(0x97,0x6c);
write_cmos_sensor(0x98,0x60);
write_cmos_sensor(0x99,0x60);
write_cmos_sensor(0x9a,0x6c);
write_cmos_sensor(0xa1,0x40);
write_cmos_sensor(0xaf,0x04);
write_cmos_sensor(0xb1,0x40);
write_cmos_sensor(0xae, 0x0d);
write_cmos_sensor(0x88,0x5b);
write_cmos_sensor(0x89, 0x7c);
write_cmos_sensor(0xb4, 0x05);
write_cmos_sensor(0x8c, 0x40);
write_cmos_sensor(0x8e, 0x40);
write_cmos_sensor(0x90, 0x40);
write_cmos_sensor(0x92, 0x40);
write_cmos_sensor(0x9b,0x46);
write_cmos_sensor(0xac, 0x40);
write_cmos_sensor(0xfd, 0x00);
write_cmos_sensor(0x5a, 0x15);
write_cmos_sensor(0x74, 0x01);
write_cmos_sensor(0xfd, 0x00);
write_cmos_sensor(0x50, 0x40);
write_cmos_sensor(0x52, 0xb0);
write_cmos_sensor(0xfd, 0x01);
write_cmos_sensor(0x03, 0x70);
write_cmos_sensor(0x05, 0x10);
write_cmos_sensor(0x07, 0x20);
write_cmos_sensor(0x09, 0xb0);
write_cmos_sensor(0xfd, 0x03);
write_cmos_sensor(0xc2, 0x01);

write_cmos_sensor(0xfb, 0x01);
}
static void  hs_video_setting(void)
{
write_cmos_sensor(0xfc, 0x01);
write_cmos_sensor(0xfd, 0x00);
write_cmos_sensor(0xfd, 0x00);
write_cmos_sensor(0x24, 0x02);
write_cmos_sensor(0x25, 0x06);
write_cmos_sensor(0x29, 0x03);
write_cmos_sensor(0x2a, 0x34);
write_cmos_sensor(0x1e, 0x17);
write_cmos_sensor(0x33, 0x07);
write_cmos_sensor(0x35, 0x07);
write_cmos_sensor(0x4a, 0x0c);
write_cmos_sensor(0x3a, 0x05);
write_cmos_sensor(0x3b, 0x02);
write_cmos_sensor(0x3e, 0x00);
write_cmos_sensor(0x46, 0x01);
write_cmos_sensor(0x6d, 0x03);
write_cmos_sensor(0xfd, 0x01);
write_cmos_sensor(0x0e, 0x02);
write_cmos_sensor(0x0f, 0x1a);
write_cmos_sensor(0x18, 0x00);
write_cmos_sensor(0x22, 0xff);
write_cmos_sensor(0x23, 0x02);
write_cmos_sensor(0x17, 0x2c);
write_cmos_sensor(0x19, 0x20);
write_cmos_sensor(0x1b, 0x06);
write_cmos_sensor(0x1c, 0x04);
write_cmos_sensor(0x20, 0x03);
write_cmos_sensor(0x30, 0x01);
write_cmos_sensor(0x33, 0x01);
write_cmos_sensor(0x31, 0x0a);
write_cmos_sensor(0x32, 0x09);
write_cmos_sensor(0x38, 0x01);
write_cmos_sensor(0x39, 0x01);
write_cmos_sensor(0x3a, 0x01);
write_cmos_sensor(0x3b, 0x01);
write_cmos_sensor(0x4f, 0x04);
write_cmos_sensor(0x4e, 0x05);
write_cmos_sensor(0x50, 0x01);
write_cmos_sensor(0x35, 0x0c);
write_cmos_sensor(0x45, 0x2a);
write_cmos_sensor(0x46, 0x2a);
write_cmos_sensor(0x47, 0x2a);
write_cmos_sensor(0x48, 0x2a);
write_cmos_sensor(0x4a, 0x2c);
write_cmos_sensor(0x4b, 0x2c);
write_cmos_sensor(0x4c, 0x2c);
write_cmos_sensor(0x4d, 0x2c);
write_cmos_sensor(0x56, 0x3a);
write_cmos_sensor(0x57, 0x0a);
write_cmos_sensor(0x58, 0x24);
write_cmos_sensor(0x59, 0x20);
write_cmos_sensor(0x5a, 0x0a);
write_cmos_sensor(0x5b, 0xff);
write_cmos_sensor(0x37, 0x0a);
write_cmos_sensor(0x42, 0x0e);
write_cmos_sensor(0x68, 0x90);
write_cmos_sensor(0x69, 0xcd);
write_cmos_sensor(0x6a, 0x8f);
write_cmos_sensor(0x7c,0x0a);
write_cmos_sensor(0x7d,0x0a);
write_cmos_sensor(0x7e,0x0a);
write_cmos_sensor(0x7f, 0x08);
write_cmos_sensor(0x83, 0x14);
write_cmos_sensor(0x84, 0x14);
write_cmos_sensor(0x86, 0x14);
write_cmos_sensor(0x87, 0x07);
write_cmos_sensor(0x88, 0x0f);
write_cmos_sensor(0x94, 0x02);
write_cmos_sensor(0x98, 0xd1);
write_cmos_sensor(0x14, 0x00);
write_cmos_sensor(0x15, 0x01);
write_cmos_sensor(0xfe, 0x02);
write_cmos_sensor(0xfd, 0x03);
write_cmos_sensor(0x97,0x6c);
write_cmos_sensor(0x98,0x60);
write_cmos_sensor(0x99,0x60);
write_cmos_sensor(0x9a,0x6c);
write_cmos_sensor(0xa1,0x40);
write_cmos_sensor(0xaf,0x04);
write_cmos_sensor(0xb1,0x40);
write_cmos_sensor(0xae, 0x0d);
write_cmos_sensor(0x88,0x5b);
write_cmos_sensor(0x89, 0x7c);
write_cmos_sensor(0xb4, 0x05);
write_cmos_sensor(0x8c, 0x40);
write_cmos_sensor(0x8e, 0x40);
write_cmos_sensor(0x90, 0x40);
write_cmos_sensor(0x92, 0x40);
write_cmos_sensor(0x9b,0x46);
write_cmos_sensor(0xac, 0x40);
write_cmos_sensor(0xfd, 0x00);
write_cmos_sensor(0x5a, 0x15);
write_cmos_sensor(0x74, 0x01);
write_cmos_sensor(0xfd, 0x00);
write_cmos_sensor(0x50, 0x40);
write_cmos_sensor(0x52, 0xb0);
write_cmos_sensor(0xfd, 0x01);
write_cmos_sensor(0x03, 0x70);
write_cmos_sensor(0x05, 0x10);
write_cmos_sensor(0x07, 0x20);
write_cmos_sensor(0x09, 0xb0);
write_cmos_sensor(0xfd, 0x03);
write_cmos_sensor(0xc2, 0x01);

write_cmos_sensor(0xfb, 0x01);
}
static void  slim_video_setting(void)
{
write_cmos_sensor(0xfc, 0x01);
write_cmos_sensor(0xfd, 0x00);
write_cmos_sensor(0xfd, 0x00);
write_cmos_sensor(0x24, 0x02);
write_cmos_sensor(0x25, 0x06);
write_cmos_sensor(0x29, 0x03);
write_cmos_sensor(0x2a, 0x34);
write_cmos_sensor(0x1e, 0x17);
write_cmos_sensor(0x33, 0x07);
write_cmos_sensor(0x35, 0x07);
write_cmos_sensor(0x4a, 0x0c);
write_cmos_sensor(0x3a, 0x05);
write_cmos_sensor(0x3b, 0x02);
write_cmos_sensor(0x3e, 0x00);
write_cmos_sensor(0x46, 0x01);
write_cmos_sensor(0x6d, 0x03);
write_cmos_sensor(0xfd, 0x01);
write_cmos_sensor(0x0e, 0x02);
write_cmos_sensor(0x0f, 0x1a);
write_cmos_sensor(0x18, 0x00);
write_cmos_sensor(0x22, 0xff);
write_cmos_sensor(0x23, 0x02);
write_cmos_sensor(0x17, 0x2c);
write_cmos_sensor(0x19, 0x20);
write_cmos_sensor(0x1b, 0x06);
write_cmos_sensor(0x1c, 0x04);
write_cmos_sensor(0x20, 0x03);
write_cmos_sensor(0x30, 0x01);
write_cmos_sensor(0x33, 0x01);
write_cmos_sensor(0x31, 0x0a);
write_cmos_sensor(0x32, 0x09);
write_cmos_sensor(0x38, 0x01);
write_cmos_sensor(0x39, 0x01);
write_cmos_sensor(0x3a, 0x01);
write_cmos_sensor(0x3b, 0x01);
write_cmos_sensor(0x4f, 0x04);
write_cmos_sensor(0x4e, 0x05);
write_cmos_sensor(0x50, 0x01);
write_cmos_sensor(0x35, 0x0c);
write_cmos_sensor(0x45, 0x2a);
write_cmos_sensor(0x46, 0x2a);
write_cmos_sensor(0x47, 0x2a);
write_cmos_sensor(0x48, 0x2a);
write_cmos_sensor(0x4a, 0x2c);
write_cmos_sensor(0x4b, 0x2c);
write_cmos_sensor(0x4c, 0x2c);
write_cmos_sensor(0x4d, 0x2c);
write_cmos_sensor(0x56, 0x3a);
write_cmos_sensor(0x57, 0x0a);
write_cmos_sensor(0x58, 0x24);
write_cmos_sensor(0x59, 0x20);
write_cmos_sensor(0x5a, 0x0a);
write_cmos_sensor(0x5b, 0xff);
write_cmos_sensor(0x37, 0x0a);
write_cmos_sensor(0x42, 0x0e);
write_cmos_sensor(0x68, 0x90);
write_cmos_sensor(0x69, 0xcd);
write_cmos_sensor(0x6a, 0x8f);
write_cmos_sensor(0x7c,0x0a);
write_cmos_sensor(0x7d,0x0a);
write_cmos_sensor(0x7e,0x0a);
write_cmos_sensor(0x7f, 0x08);
write_cmos_sensor(0x83, 0x14);
write_cmos_sensor(0x84, 0x14);
write_cmos_sensor(0x86, 0x14);
write_cmos_sensor(0x87, 0x07);
write_cmos_sensor(0x88, 0x0f);
write_cmos_sensor(0x94, 0x02);
write_cmos_sensor(0x98, 0xd1);
write_cmos_sensor(0x14, 0x00);
write_cmos_sensor(0x15, 0x01);
write_cmos_sensor(0xfe, 0x02);
write_cmos_sensor(0xfd, 0x03);
write_cmos_sensor(0x97,0x6c);
write_cmos_sensor(0x98,0x60);
write_cmos_sensor(0x99,0x60);
write_cmos_sensor(0x9a,0x6c);
write_cmos_sensor(0xa1,0x40);
write_cmos_sensor(0xaf,0x04);
write_cmos_sensor(0xb1,0x40);
write_cmos_sensor(0xae, 0x0d);
write_cmos_sensor(0x88,0x5b);
write_cmos_sensor(0x89, 0x7c);
write_cmos_sensor(0xb4, 0x05);
write_cmos_sensor(0x8c, 0x40);
write_cmos_sensor(0x8e, 0x40);
write_cmos_sensor(0x90, 0x40);
write_cmos_sensor(0x92, 0x40);
write_cmos_sensor(0x9b,0x46);
write_cmos_sensor(0xac, 0x40);
write_cmos_sensor(0xfd, 0x00);
write_cmos_sensor(0x5a, 0x15);
write_cmos_sensor(0x74, 0x01);
write_cmos_sensor(0xfd, 0x00);
write_cmos_sensor(0x50, 0x40);
write_cmos_sensor(0x52, 0xb0);
write_cmos_sensor(0xfd, 0x01);
write_cmos_sensor(0x03, 0x70);
write_cmos_sensor(0x05, 0x10);
write_cmos_sensor(0x07, 0x20);
write_cmos_sensor(0x09, 0xb0);
write_cmos_sensor(0xfd, 0x03);
write_cmos_sensor(0xc2, 0x01);

write_cmos_sensor(0xfb, 0x01);
}
static void custom1_setting(void)
{
    write_cmos_sensor(0xfc, 0x01);
    write_cmos_sensor(0xfd, 0x00);
    write_cmos_sensor(0xfd, 0x00);
    write_cmos_sensor(0x24, 0x02);
    write_cmos_sensor(0x25, 0x06);
    write_cmos_sensor(0x29, 0x03);
    write_cmos_sensor(0x2a, 0x34);
    write_cmos_sensor(0x1e, 0x17);
    write_cmos_sensor(0x33, 0x07);
    write_cmos_sensor(0x35, 0x07);
    write_cmos_sensor(0x4a, 0x0c);
    write_cmos_sensor(0x3a, 0x05);
    write_cmos_sensor(0x3b, 0x02);
    write_cmos_sensor(0x3e, 0x00);
    write_cmos_sensor(0x46, 0x01);
    write_cmos_sensor(0x6d, 0x03);
    write_cmos_sensor(0xfd, 0x01);
    write_cmos_sensor(0x0e, 0x02);
    write_cmos_sensor(0x0f, 0x1a);
    write_cmos_sensor(0x18, 0x00);
    write_cmos_sensor(0x22, 0xff);
    write_cmos_sensor(0x23, 0x02);
    write_cmos_sensor(0x17, 0x2c);
    write_cmos_sensor(0x19, 0x20);
    write_cmos_sensor(0x1b, 0x06);
    write_cmos_sensor(0x1c, 0x04);
    write_cmos_sensor(0x20, 0x03);
    write_cmos_sensor(0x30, 0x01);
    write_cmos_sensor(0x33, 0x01);
    write_cmos_sensor(0x31, 0x0a);
    write_cmos_sensor(0x32, 0x09);
    write_cmos_sensor(0x38, 0x01);
    write_cmos_sensor(0x39, 0x01);
    write_cmos_sensor(0x3a, 0x01);
    write_cmos_sensor(0x3b, 0x01);
    write_cmos_sensor(0x4f, 0x04);
    write_cmos_sensor(0x4e, 0x05);
    write_cmos_sensor(0x50, 0x01);
    write_cmos_sensor(0x35, 0x0c);
    write_cmos_sensor(0x45, 0x2a);
    write_cmos_sensor(0x46, 0x2a);
    write_cmos_sensor(0x47, 0x2a);
    write_cmos_sensor(0x48, 0x2a);
    write_cmos_sensor(0x4a, 0x2c);
    write_cmos_sensor(0x4b, 0x2c);
    write_cmos_sensor(0x4c, 0x2c);
    write_cmos_sensor(0x4d, 0x2c);
    write_cmos_sensor(0x56, 0x3a);
    write_cmos_sensor(0x57, 0x0a);
    write_cmos_sensor(0x58, 0x24);
    write_cmos_sensor(0x59, 0x20);
    write_cmos_sensor(0x5a, 0x0a);
    write_cmos_sensor(0x5b, 0xff);
    write_cmos_sensor(0x37, 0x0a);
    write_cmos_sensor(0x42, 0x0e);
    write_cmos_sensor(0x68, 0x90);
    write_cmos_sensor(0x69, 0xcd);
    write_cmos_sensor(0x6a, 0x8f);
    write_cmos_sensor(0x7c,0x0a);
    write_cmos_sensor(0x7d,0x0a);
    write_cmos_sensor(0x7e,0x0a);
    write_cmos_sensor(0x7f, 0x08);
    write_cmos_sensor(0x83, 0x14);
    write_cmos_sensor(0x84, 0x14);
    write_cmos_sensor(0x86, 0x14);
    write_cmos_sensor(0x87, 0x07);
    write_cmos_sensor(0x88, 0x0f);
    write_cmos_sensor(0x94, 0x02);
    write_cmos_sensor(0x98, 0xd1);
    write_cmos_sensor(0x14, 0x00);
    write_cmos_sensor(0x15, 0x01);
    write_cmos_sensor(0xfe, 0x02);
    write_cmos_sensor(0xfd, 0x03);
    write_cmos_sensor(0x97,0x6c);
    write_cmos_sensor(0x98,0x60);
    write_cmos_sensor(0x99,0x60);
    write_cmos_sensor(0x9a,0x6c);
    write_cmos_sensor(0xa1,0x40);
    write_cmos_sensor(0xaf,0x04);
    write_cmos_sensor(0xb1,0x40);
    write_cmos_sensor(0xae,0x0d);
    write_cmos_sensor(0x88,0x5b);
    write_cmos_sensor(0x89, 0x7c);
    write_cmos_sensor(0xb4, 0x05);
    write_cmos_sensor(0x8c, 0x40);
    write_cmos_sensor(0x8e, 0x40);
    write_cmos_sensor(0x90, 0x40);
    write_cmos_sensor(0x92,0x40);
    write_cmos_sensor(0x9b,0x46);
    write_cmos_sensor(0xac,0x40);
    write_cmos_sensor(0xfd, 0x00);
    write_cmos_sensor(0x5a, 0x15);
    write_cmos_sensor(0x74, 0x01);
    write_cmos_sensor(0xfd, 0x00);
    write_cmos_sensor(0x50, 0x40);
    write_cmos_sensor(0x52, 0xb0);
    write_cmos_sensor(0xfd, 0x01);
    write_cmos_sensor(0x03, 0x70);
    write_cmos_sensor(0x05, 0x10);
    write_cmos_sensor(0x07, 0x20);
    write_cmos_sensor(0x09, 0xb0);
    write_cmos_sensor(0xfd, 0x03);
    write_cmos_sensor(0xc2, 0x01);

    write_cmos_sensor(0xfb, 0x01);
}    /*    capture_setting  */

static char otp_data[19] = { 0 };

unsigned int
ov02b10ofilm_read_otp_info(struct i2c_client *client,
                 unsigned int addr, unsigned char *data, unsigned int size)
{
        int ii = 0;
        for (ii = 0; ii < 19; ii++) {
                data[ii] = otp_data[addr + ii];
        }
        return size;
}


static void ov02b10_read_otp(void)
{
    int otp_addr[] = { 17,18,19, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
    kal_uint8 otp_info[19];
    int i = 0;
    for(i=0; i<19;i++)
    {
        write_cmos_sensor(0xfd, 0x06);
        otp_info[i] = read_cmos_sensor(otp_addr[i]);
    }
    for (i = 0; i < 19; i++)
    {
        otp_data[i] = otp_info[i];
    }
}

static int get_ov02b10_vendor_id(void)
{
        int module_id;
        write_cmos_sensor(0xfd, 0x06);
        module_id = read_cmos_sensor(0x11);
        return module_id;
}

static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
    kal_uint8 i = 0;
    kal_uint8 retry = 2,vendor_id = 0;
    ov02b10_read_otp();
    vendor_id = get_ov02b10_vendor_id();
    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
        spin_lock(&imgsensor_drv_lock);
        imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
        spin_unlock(&imgsensor_drv_lock);
        do {
            if (vendor_id == 0x07) {
                *sensor_id = return_sensor_id();
                if (*sensor_id == imgsensor_info.sensor_id) {
                    LOG_INF("i2c write id : 0x%x, sensor id: 0x%x\n",
                    imgsensor.i2c_write_id, *sensor_id);
                    return ERROR_NONE;
                }
            } else {
                LOG_INF("hi259 ofilm vendor_id not match :%x", vendor_id);
            }
            retry--;
        } while (retry > 0);
        i++;
        retry = 2;
    }
    if (*sensor_id != imgsensor_info.sensor_id) {
        LOG_INF("Read id fail,sensor id: 0x%x\n", *sensor_id);
        *sensor_id = 0xFFFFFFFF;
        return ERROR_SENSOR_CONNECT_FAIL;
    }

    return ERROR_NONE;
}

static kal_uint32 open(void)
{
    kal_uint8 i = 0;
    kal_uint8 retry = 2;
    kal_uint16 sensor_id = 0;
    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
        spin_lock(&imgsensor_drv_lock);
        imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
        spin_unlock(&imgsensor_drv_lock);
        do {
            sensor_id = return_sensor_id();
            if (sensor_id == imgsensor_info.sensor_id) {
                LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id, sensor_id);
                break;
            }

            retry--;
        } while(retry > 0);
        i++;
        if (sensor_id == imgsensor_info.sensor_id)
            break;
        retry = 2;
    }
    if (imgsensor_info.sensor_id != sensor_id) 
        return ERROR_SENSOR_CONNECT_FAIL;
    /* initail sequence write in  */
    sensor_init();

    spin_lock(&imgsensor_drv_lock);

    imgsensor.autoflicker_en= KAL_FALSE;
    imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
    imgsensor.pclk = imgsensor_info.pre.pclk;
    imgsensor.frame_length = imgsensor_info.pre.framelength;
    imgsensor.line_length = imgsensor_info.pre.linelength;
    imgsensor.min_frame_length = imgsensor_info.pre.framelength;
    imgsensor.dummy_pixel = 0;
    imgsensor.dummy_line = 0;
    imgsensor.ihdr_en = 0;
    imgsensor.test_pattern = KAL_FALSE;
    imgsensor.current_fps = imgsensor_info.pre.max_framerate;
    spin_unlock(&imgsensor_drv_lock);

    return ERROR_NONE;
}    /*    open  */

static kal_uint32 close(void)
{
	LOG_INF("E\n");
	streaming_control(KAL_FALSE);
	/*No Need to implement this function*/
	return ERROR_NONE;
}	/*	close  */

static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
        pr_info("[ov02b] preview mode start\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();
	return ERROR_NONE;
}    /*    preview   */

static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
        pr_info("[ov02b] capture mode start\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;

    if (imgsensor.current_fps == imgsensor_info.cap.max_framerate)	{
        imgsensor.pclk = imgsensor_info.cap.pclk;
        imgsensor.line_length = imgsensor_info.cap.linelength;
        imgsensor.frame_length = imgsensor_info.cap.framelength;
        imgsensor.min_frame_length = imgsensor_info.cap.framelength;
        imgsensor.autoflicker_en = KAL_FALSE;
    } else {
     //PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M
        imgsensor.pclk = imgsensor_info.cap1.pclk;
        imgsensor.line_length = imgsensor_info.cap1.linelength;
        imgsensor.frame_length = imgsensor_info.cap1.framelength;
        imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
        imgsensor.autoflicker_en = KAL_FALSE;
    }
    spin_unlock(&imgsensor_drv_lock);
    capture_setting();

	return ERROR_NONE;
}    /* capture() */
static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	/* imgsensor.current_fps = 300; */
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	normal_video_setting();
	return ERROR_NONE;
}    /*    normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	/* imgsensor.video_mode = KAL_TRUE; */
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();
	return ERROR_NONE;
}
static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();
	return ERROR_NONE;
}
static kal_uint32 custom1(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
    MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    pr_info("[ov02b] custom1 mode start\n");
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM1;
    imgsensor.pclk = imgsensor_info.custom1.pclk;
    imgsensor.line_length = imgsensor_info.custom1.linelength;
    imgsensor.frame_length = imgsensor_info.custom1.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom1.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    custom1_setting();
    return ERROR_NONE;
}
static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;
	sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;
	sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;
	sensor_resolution->SensorHighSpeedVideoWidth = imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight = imgsensor_info.hs_video.grabwindow_height;
	sensor_resolution->SensorSlimVideoWidth = imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight = imgsensor_info.slim_video.grabwindow_height;
	sensor_resolution->SensorCustom1Width = imgsensor_info.custom1.grabwindow_width;
    sensor_resolution->SensorCustom1Height = imgsensor_info.custom1.grabwindow_height;
    return ERROR_NONE;
}    /*    get_resolution    */

static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
		MSDK_SENSOR_INFO_STRUCT *sensor_info,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW; /* not use */
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW; /* inverse with datasheet */
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4; /* not use */
	sensor_info->SensorResetActiveHigh = FALSE; /* not use */
	sensor_info->SensorResetDelayCount = 5; /* not use */

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;
	sensor_info->Custom1DelayFrame = imgsensor_info.custom1_delay_frame;/*add custom1*/

    sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;
	sensor_info->FrameTimeDelayFrame =
		imgsensor_info.frame_time_delay_frame;
	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;
	sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;
	sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;

	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
    sensor_info->SensorClockFreq = imgsensor_info.mclk;
    sensor_info->SensorClockDividCount = 3; /* not use */
    sensor_info->SensorClockRisingCount = 0;
    sensor_info->SensorClockFallingCount = 2; /* not use */
    sensor_info->SensorPixelClockCount = 3; /* not use */
    sensor_info->SensorDataLatchCount = 2; /* not use */

    sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
    sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
    sensor_info->SensorWidthSampling = 0;  // 0 is default 1x
    sensor_info->SensorHightSampling = 0;    // 0 is default 1x
    sensor_info->SensorPacketECCOrder = 1;


	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
	    sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
	    sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

	    sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
				imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
	break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
	    sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
	    sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;

	    sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.cap.mipi_data_lp2hs_settle_dc;
	break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
	    sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx;
	    sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;

	    sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		sensor_info->SensorGrabStartX = imgsensor_info.custom1.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.custom1.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
            imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;
    break;
    default:
        sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
        sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

        sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
            imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
    break;
    }

    return ERROR_NONE;
}    /*    get_info  */

static kal_uint32 control(enum MSDK_SCENARIO_ID_ENUM scenario_id,
            MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
            MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("scenario_id = %d\n", scenario_id);
    spin_lock(&imgsensor_drv_lock);
    imgsensor.current_scenario_id = scenario_id;
    spin_unlock(&imgsensor_drv_lock);
    switch (scenario_id) {
    case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        preview(image_window, sensor_config_data);
    break;
    case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        capture(image_window, sensor_config_data);
    break;
    case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
        normal_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		hs_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		slim_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
        custom1(image_window, sensor_config_data);
    break;
    default:
        //LOG_INF("[odin]default mode\n");
        preview(image_window, sensor_config_data);
        return ERROR_INVALID_SCENARIO_ID;
    }
    return ERROR_NONE;
}    /* control() */

static kal_uint32 set_video_mode(UINT16 framerate)
{
	/*This Function not used after ROME*/
	LOG_INF("framerate = %d\n ", framerate);
	/* SetVideoMode Function should fix framerate */
	/***********
	 *if (framerate == 0)	 //Dynamic frame rate
	 *	return ERROR_NONE;
	 *spin_lock(&imgsensor_drv_lock);
	 *if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
	 *	imgsensor.current_fps = 296;
	 *else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
	 *	imgsensor.current_fps = 146;
	 *else
	 *	imgsensor.current_fps = framerate;
	 *spin_unlock(&imgsensor_drv_lock);
	 *set_max_framerate(imgsensor.current_fps, 1);
	 ********/
	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d ", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable) //enable auto flicker
		imgsensor.autoflicker_en = KAL_TRUE;
	else //Cancel Auto flick
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static kal_uint32 set_max_framerate_by_scenario(enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
	kal_uint32 frame_length;

//	LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ?
			(frame_length - imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		if (framerate == 0)
			return ERROR_NONE;
		frame_length = imgsensor_info.normal_video.pclk / framerate * 10 /
			imgsensor_info.normal_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength) ?
			(frame_length - imgsensor_info.normal_video.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
			frame_length = imgsensor_info.cap1.pclk / framerate * 10 / imgsensor_info.cap1.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.cap1.framelength) ?
				(frame_length - imgsensor_info.cap1.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.cap1.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
		} else {
			if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
				LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
					framerate, imgsensor_info.cap.max_framerate / 10);
			frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ?
				(frame_length - imgsensor_info.cap.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
		}
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
	break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ?
			(frame_length - imgsensor_info.hs_video.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ?
			(frame_length - imgsensor_info.slim_video.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
    case MSDK_SCENARIO_ID_CUSTOM1:
        if (imgsensor.current_fps != imgsensor_info.custom1.max_framerate)
        LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",framerate,imgsensor_info.custom1.max_framerate/10);
        frame_length = imgsensor_info.custom1.pclk / framerate * 10 / imgsensor_info.custom1.linelength;
        spin_lock(&imgsensor_drv_lock);
        imgsensor.dummy_line = (frame_length > imgsensor_info.custom1.framelength) ?
            (frame_length - imgsensor_info.custom1.framelength) : 0;
        imgsensor.frame_length = imgsensor_info.custom1.framelength + imgsensor.dummy_line;
        imgsensor.min_frame_length = imgsensor.frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (imgsensor.frame_length > imgsensor.shutter)
        set_dummy();
    break;
    default:  //coding with  preview scenario by default
		frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ?
			(frame_length - imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		LOG_INF("error scenario_id = %d, we use preview scenario\n", scenario_id);
		break;
    }
    return ERROR_NONE;
}

static kal_uint32 get_default_framerate_by_scenario(enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		*framerate = imgsensor_info.pre.max_framerate;
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		*framerate = imgsensor_info.normal_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		*framerate = imgsensor_info.cap.max_framerate;
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		*framerate = imgsensor_info.hs_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		*framerate = imgsensor_info.slim_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
        *framerate = imgsensor_info.custom1.max_framerate;
    break;
    default:
        break;
    }
    return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d\n", enable);

	if(enable)
	{
		write_cmos_sensor(0xfd,0x03);
		write_cmos_sensor(0x81,0x01);
	}
	else
	{
		write_cmos_sensor(0xfd,0x03);
		write_cmos_sensor(0x81,0x00);
	}

	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static kal_uint32 streaming_control(kal_bool enable)
{
	LOG_INF("streaming_control enable =%d\n", enable);
	if (enable){
		write_cmos_sensor(0xfd, 0x03);
		write_cmos_sensor(0xc2, 0x01);

	}else{
		write_cmos_sensor(0xfd, 0x03);
		write_cmos_sensor(0xc2, 0x00);
	}
	mdelay(10);

	return ERROR_NONE;
}



static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
		UINT8 *feature_para,UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16 = (UINT16 *)feature_para;
	UINT16 *feature_data_16 = (UINT16 *)feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *)feature_para;
	UINT32 *feature_data_32 = (UINT32 *)feature_para;
	unsigned long long *feature_data = (unsigned long long *)feature_para;

	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data = (MSDK_SENSOR_REG_INFO_STRUCT *)feature_para;

	LOG_INF("feature_id = %d\n", feature_id);
	switch (feature_id) {
	case SENSOR_FEATURE_GET_GAIN_RANGE_BY_SCENARIO:
		*(feature_data + 1) = imgsensor_info.min_gain;
		*(feature_data + 2) = imgsensor_info.max_gain;
		break;
	case SENSOR_FEATURE_GET_BASE_GAIN_ISO_AND_STEP:
		*(feature_data + 0) = imgsensor_info.min_gain_iso;
		*(feature_data + 1) = imgsensor_info.gain_step;
		*(feature_data + 2) = imgsensor_info.gain_type;
		break;
	case SENSOR_FEATURE_GET_MIN_SHUTTER_BY_SCENARIO:
		*(feature_data + 1) = imgsensor_info.min_shutter;
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ_BY_SCENARIO:
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.cap.pclk;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.normal_video.pclk;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.hs_video.pclk;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.slim_video.pclk;
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom1.pclk;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.pre.pclk;
			break;
		}
		break;
	case SENSOR_FEATURE_GET_PERIOD_BY_SCENARIO:
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.cap.framelength << 16)
				+ imgsensor_info.cap.linelength;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.normal_video.framelength << 16)
				+ imgsensor_info.normal_video.linelength;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.hs_video.framelength << 16)
				+ imgsensor_info.hs_video.linelength;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.slim_video.framelength << 16)
				+ imgsensor_info.slim_video.linelength;
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom1.framelength << 16)
				+ imgsensor_info.custom1.linelength;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.pre.framelength << 16)
				+ imgsensor_info.pre.linelength;
			break;
		}
		break;
	case SENSOR_FEATURE_GET_PERIOD:
		*feature_return_para_16++ = imgsensor.line_length;
		*feature_return_para_16 = imgsensor.frame_length;
		*feature_para_len=4;
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
	    *feature_return_para_32 = imgsensor.pclk;
	    *feature_para_len = 4;
	break;
	case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:
		{
			kal_uint32 rate;

			switch (*feature_data) {
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				rate = imgsensor_info.cap.mipi_pixel_rate;
				break;
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				rate = imgsensor_info.normal_video.mipi_pixel_rate;
				break;
			case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
				rate = imgsensor_info.hs_video.mipi_pixel_rate;
				break;
			case MSDK_SCENARIO_ID_SLIM_VIDEO:
				rate = imgsensor_info.slim_video.mipi_pixel_rate;
				break;
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			default:
				rate = imgsensor_info.pre.mipi_pixel_rate;
				break;
			}
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = rate;
		}
		break;
	case SENSOR_FEATURE_SET_ESHUTTER:
		set_shutter(*feature_data);
		break;
	/*case SENSOR_FEATURE_SET_NIGHTMODE:
		night_mode((BOOL)*feature_data);
		break;*/
	case SENSOR_FEATURE_SET_GAIN:
		set_gain((UINT16) *feature_data);
		break;
	case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
	case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
		break;
	case SENSOR_FEATURE_SET_REGISTER:
		write_cmos_sensor(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
		break;
	case SENSOR_FEATURE_GET_REGISTER:
		sensor_reg_data->RegData = read_cmos_sensor(sensor_reg_data->RegAddr);
		LOG_INF("adb_i2c_read 0x%x = 0x%x\n", sensor_reg_data->RegAddr, sensor_reg_data->RegData);
		break;
	case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
	    *feature_return_para_32 = LENS_DRIVER_ID_DO_NOT_CARE;
	    *feature_para_len = 4;
	break;
	case SENSOR_FEATURE_SET_VIDEO_MODE:
		set_video_mode(*feature_data);
		break;
	case SENSOR_FEATURE_CHECK_SENSOR_ID:
		get_imgsensor_id(feature_return_para_32);
		break;
	case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
		set_auto_flicker_mode((BOOL)*feature_data_16, *(feature_data_16 + 1));
		break;
	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
		set_max_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data+1));
		break;
	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
		get_default_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)*(feature_data),
			(MUINT32 *)(uintptr_t)(*(feature_data + 1)));
		break;
	case SENSOR_FEATURE_SET_TEST_PATTERN:
		set_test_pattern_mode((BOOL)*feature_data);
		break;
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: //for factory mode auto testing
		*feature_return_para_32 = imgsensor_info.checksum_value;
		*feature_para_len=4;
		break;
	case SENSOR_FEATURE_SET_FRAMERATE:
	    LOG_INF("current fps :%d\n", (UINT32)*feature_data);
	    spin_lock(&imgsensor_drv_lock);
	    imgsensor.current_fps = *feature_data;
	    spin_unlock(&imgsensor_drv_lock);
	break;
	case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
		set_shutter_frame_length((UINT16) *feature_data,
			(UINT16) *(feature_data + 1));
		break;
	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
		streaming_control(KAL_FALSE);
		break;
	case SENSOR_FEATURE_SET_STREAMING_RESUME:
		if (*feature_data != 0)
			set_shutter(*feature_data);
		streaming_control(KAL_TRUE);
		break;
	case SENSOR_FEATURE_SET_HDR:
	    LOG_INF("ihdr enable :%d\n", (BOOL)*feature_data);
	    spin_lock(&imgsensor_drv_lock);
	    imgsensor.ihdr_en = (BOOL)*feature_data;
	    spin_unlock(&imgsensor_drv_lock);
	break;
	case SENSOR_FEATURE_GET_BINNING_TYPE:	
		switch (*(feature_data + 1)) {	/*2sum = 2; 4sum = 4; 4avg = 1 not 4cell sensor is 4avg*/

		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		case MSDK_SCENARIO_ID_CUSTOM1:
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*feature_return_para_32 = 1; /*4sum*/
			break;
		default:
			*feature_return_para_32 = 1; /*BINNING_NONE,*/ 
			break;
		}
		pr_debug("SENSOR_FEATURE_GET_BINNING_TYPE AE_binning_type:%d,\n",
			*feature_return_para_32);
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_GET_CROP_INFO:
		LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", (UINT32)*feature_data);
		wininfo = (struct SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data + 1));
		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[1], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[2], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[3], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[4], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
            memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[3],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
        break;
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        default:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[0], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		}
        break;
	case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
		LOG_INF("SENSOR_SET_SENSOR_IHDR LE = %d, SE = %d, Gain = %d\n",
			(UINT16)*feature_data, (UINT16)*(feature_data + 1), (UINT16)*(feature_data + 2));
		ihdr_write_shutter_gain((UINT16)*feature_data, (UINT16)*(feature_data + 1),
			(UINT16)*(feature_data + 2));
		break;
	default:
		break;
	}

    return ERROR_NONE;
}    /*    feature_control()  */

static struct SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 OV02B10OFILM_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&sensor_func;
	return ERROR_NONE;
}   
