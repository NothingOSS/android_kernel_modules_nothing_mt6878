// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022 MediaTek Inc.

/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 sc202csmipiraw_Sensor.c
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#define PFX "SC202CS_camera_sensor"

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/types.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define_v4l2.h"
#include "kd_imgsensor_errcode.h"

#include "sc202cs_mipi_raw_Sensor.h"
#include "sc202cs_ana_gain_table.h"

#include "adaptor-subdrv.h"
#include "adaptor-i2c.h"
#if IS_ENABLED(CONFIG_T_PRODUCT_INFO)
#include "../../../../../../../device_info/dev_info.h"
#endif
#define read_cmos_sensor_8(...) subdrv_i2c_rd_u8(__VA_ARGS__)
#define read_cmos_sensor(...) subdrv_i2c_rd_u16(__VA_ARGS__)
#define write_cmos_sensor_8(...) subdrv_i2c_wr_u8(__VA_ARGS__)
#define write_cmos_sensor(...) subdrv_i2c_wr_u16(__VA_ARGS__)
#define sc202cs_table_write_cmos_sensor(...) subdrv_i2c_wr_regs_u8_u8(__VA_ARGS__)

#define LOG_INF(format, args...) pr_err(PFX "[%s] " format, __func__, ##args)
#define SC202CS_LOG_DBG(format, args...) pr_err(PFX "[%s] " format, __func__, ##args)

#define FPT_PDAF_SUPPORT 0
#define SC202CS_SENSOR_GAIN_MAX_VALID_INDEX  5
#define SC202CS_SENSOR_GAIN_MAP_SIZE         5
#define SC202CS_SENSOR_BASE_GAIN           0x400
#define SC202CS_SENSOR_MAX_GAIN            (16 * SC202CS_SENSOR_BASE_GAIN)

#define EEPROM_SC202CS_MOLDULEID 0x0E
#define EEPROM_SC202CS_MOLDULEID_0_ADDR 0x8009
#define EEPROM_SC202CS_MOLDULEID_1_ADDR 0x8016

extern char depth_camera_sn[64];
extern char main_module_id[64];
static void sensor_init(struct subdrv_ctx *ctx);

static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = SC202CS_SENSOR_ID,
	.checksum_value = 0x2c786cbf,

	.pre = {
		.pclk = 72000000,				//record different mode's pclk
		.linelength  = 1920,				//record different mode's linelength
		.framelength = 1250,			/*record different mode's framelength*/
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width = 1600,
		.grabwindow_height = 1200,
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 14,
		.mipi_pixel_rate = 72000000,
		.max_framerate = 300,
	},
	.cap = {
		.pclk = 72000000,				//record different mode's pclk
		.linelength  = 1920,				//record different mode's linelength
		.framelength = 1250,			/*record different mode's framelength*/
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width = 1600,
		.grabwindow_height = 1200,
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 14,
		.mipi_pixel_rate = 72000000,
		.max_framerate = 300,
	},
	.normal_video = {
		.pclk = 72000000,				//record different mode's pclk
		.linelength  = 1920,				//record different mode's linelength
		.framelength = 1250,			/*record different mode's framelength*/
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width = 1600,
		.grabwindow_height = 1200,
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 14,
		.mipi_pixel_rate = 72000000,
		.max_framerate = 300,
	},
	.hs_video = {
		.pclk = 72000000,				//record different mode's pclk
		.linelength  = 1920,				//record different mode's linelength
		.framelength = 1250,			/*record different mode's framelength*/
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width = 1600,
		.grabwindow_height = 1200,
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 14,
		.mipi_pixel_rate = 72000000,
		.max_framerate = 300,
	},
	.slim_video = {
		.pclk = 72000000,				//record different mode's pclk
		.linelength  = 1920,				//record different mode's linelength
		.framelength = 1250,			/*record different mode's framelength*/
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width = 1600,
		.grabwindow_height = 1200,
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 14,
		.mipi_pixel_rate = 72000000,
		.max_framerate = 300,
	},
	
	.margin = 6,
	.min_shutter = 2,
	.min_gain     = BASEGAIN, //1x
	.max_gain     = 16 * BASEGAIN,//40x
	.min_gain_iso = 100,
	.exp_step     = 1,
	.gain_step = 2,
	.gain_type = 2,
	.max_frame_length = 0xfff9,
	.ae_shut_delay_frame = 0,

	/* sensor gain delay frame for AE cycle,
	 * 2 frame with ispGain_delay-sensor_gain_delay=2-0=2
	 */
	.ae_sensor_gain_delay_frame = 0,

	.ae_ispGain_delay_frame = 2,	/* isp gain delay frame for AE cycle */
	.frame_time_delay_frame     = 2,
	.ihdr_support = 0,	/* 1, support; 0,not support */
	.ihdr_le_firstline = 0,	/* 1,le first ; 0, se first */
	.temperature_support        = 0, /* 1, support; 0,not support */
	.sensor_mode_num        = 5, /* support sensor mode num */

	.cap_delay_frame = 3,	/* enter capture delay frame num */
	.pre_delay_frame = 3,	/* enter preview delay frame num */
	.video_delay_frame = 3,	/* enter video delay frame num */

	/* enter high speed video  delay frame num */
	.hs_video_delay_frame = 3,

	.slim_video_delay_frame = 3,	/* enter slim video delay frame num */

	.isp_driving_current = ISP_DRIVING_6MA,

	/* sensor_interface_type */
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,

	/* 0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2 */
	.mipi_sensor_type = MIPI_OPHY_NCSI2,

	/* 0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL */
	.mipi_settle_delay_mode = 0,//1
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_MONO,
	.mclk = 24,
	.mipi_lane_num = SENSOR_MIPI_1_LANE,
	.i2c_speed = 400, /*support 1MHz write*/
	/* record sensor support all write id addr,
	 * only supprt 4 must end with 0xff
	 */
	.i2c_addr_table = {0x6C,0x20,0xff},
};

/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[] = {
	{ 1600, 1200,   0,   0,   1600, 1200,   1600, 1200,   0, 0, 1600, 1200,   0, 0, 1600, 1200}, // Preview
	{ 1600, 1200,   0,   0,   1600, 1200,   1600, 1200,   0, 0, 1600, 1200,   0, 0, 1600, 1200}, // capture
	{ 1600, 1200,   0,   0,   1600, 1200,   1600, 1200,   0, 0, 1600, 1200,   0, 0, 1600, 1200}, // video
	{ 1600, 1200,   0,   0,   1600, 1200,   1600, 1200,   0, 0, 1600, 1200,   0, 0, 1600, 1200},  //high speed video
	{ 1600, 1200,   0,   0,   1600, 1200,   1600, 1200,   0, 0, 1600, 1200,   0, 0, 1600, 1200}, // slim video
	{ 1600, 1200,   0,   0,   1600, 1200,   1600, 1200,   0, 0, 1600, 1200,   0, 0, 1600, 1200}, // custom1
};

static struct mtk_sensor_saturation_info imgsensor_saturation_info = {
	.gain_ratio = 1000,
	.OB_pedestal = 64,
	.saturation_level = 1023,
};
static void set_dummy(struct subdrv_ctx *ctx)
{
	LOG_INF("dummyline = %d, dummypixels = %d\n",
		ctx->dummy_line, ctx->dummy_pixel);

	/* return; //for test */
	write_cmos_sensor_8(ctx, 0x320e, (ctx->frame_length >> 8) & 0xFF);
	write_cmos_sensor_8(ctx, 0x320f, ctx->frame_length & 0xFF);
}

static void set_max_framerate(struct subdrv_ctx *ctx, UINT16 framerate, kal_bool min_framelength_en)
{

	kal_uint32 frame_length = ctx->frame_length;

	LOG_INF("framerate = %d, min framelength should enable %d\n",
		framerate, min_framelength_en);

	frame_length = ctx->pclk / framerate * 10 / ctx->line_length;
	if (frame_length >= ctx->min_frame_length)
		ctx->frame_length = frame_length;
	else
		ctx->frame_length = ctx->min_frame_length;

	ctx->dummy_line =
		ctx->frame_length - ctx->min_frame_length;

	if (ctx->frame_length > imgsensor_info.max_frame_length) {
		ctx->frame_length = imgsensor_info.max_frame_length;

		ctx->dummy_line =
			ctx->frame_length - ctx->min_frame_length;
	}
	if (min_framelength_en)
		ctx->min_frame_length = ctx->frame_length;
	set_dummy(ctx);
}

static void write_shutter(struct subdrv_ctx *ctx, kal_uint16 shutter)
{
	kal_uint16 realtime_fps = 0;
	//kal_uint32 frame_length = 0;

	/* 0x3500, 0x3501, 0x3502 will increase VBLANK to get exposure larger than frame exposure */
	/* AE doesn't update sensor gain at capture mode, thus extra exposure lines must be updated here. */

	// if shutter bigger than frame_length, should extend frame length first
	//printk("pangfei shutter %d line %d\n",shutter,__LINE__);
  //LOG_INF("song add shutter is %d  max_frame_length is %d min_frame_length is %d imgsensor_info.marign is %d \n",shutter,imgsensor_info.max_frame_length,imgsensor_info.min_shutter,imgsensor_info.margin);
	if (shutter > ctx->min_frame_length - imgsensor_info.margin)
		ctx->frame_length = shutter + imgsensor_info.margin;
	else
		ctx->frame_length = ctx->min_frame_length;
	if (ctx->frame_length > imgsensor_info.max_frame_length)
		ctx->frame_length = imgsensor_info.max_frame_length;

	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;
	if (ctx->autoflicker_en) {
		realtime_fps = ctx->pclk * 10 / (ctx->line_length * ctx->frame_length);
		if (realtime_fps >= 297 && realtime_fps <= 305) {
			set_max_framerate(ctx, 296, 0);
		} else if (realtime_fps >= 147 && realtime_fps <= 150) {
			set_max_framerate(ctx, 146, 0);
		} else {
			write_cmos_sensor_8(ctx, 0x320e, (ctx->frame_length >> 8) & 0xFF);
			write_cmos_sensor_8(ctx, 0x320f, ctx->frame_length & 0xFF);
		}
	} else {
		// Extend frame length
		write_cmos_sensor_8(ctx, 0x320e, (ctx->frame_length >> 8) & 0xFF);
		write_cmos_sensor_8(ctx, 0x320f, ctx->frame_length & 0xFF);
	}

	// Update Shutter

	write_cmos_sensor_8(ctx, 0x3e00, (shutter >> 12) & 0x0F);
	write_cmos_sensor_8(ctx, 0x3e01, (shutter >> 4) & 0xFF);
	write_cmos_sensor_8(ctx, 0x3e02, (shutter << 4) & 0xF0);
	LOG_INF("shutter =%d, framelength =%d", shutter, ctx->frame_length);
}

/*************************************************************************
 * FUNCTION
 *	set_shutter
 *
 * DESCRIPTION
 *	This function set e-shutter of sensor to change exposure time.
 *
 * PARAMETERS
 *	iShutter : exposured lines
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static void set_shutter(struct subdrv_ctx *ctx, kal_uint16 shutter)
{
	ctx->shutter = shutter;

	write_shutter(ctx, shutter);
} /* set_shutter */


static void set_frame_length(struct subdrv_ctx *ctx, kal_uint16 frame_length)
{
	if (frame_length > 1)
		ctx->frame_length = frame_length;

	if (ctx->frame_length > imgsensor_info.max_frame_length)
		ctx->frame_length = imgsensor_info.max_frame_length;
	if (ctx->min_frame_length > ctx->frame_length)
		ctx->frame_length = ctx->min_frame_length;

	/* Extend frame length */
	write_cmos_sensor_8(ctx, 0x320e, (ctx->frame_length >> 8) & 0xFF);
	write_cmos_sensor_8(ctx, 0x320f, ctx->frame_length & 0xFF);

	SC202CS_LOG_DBG("Framelength: set=%d/input=%d/min=%d\n",
		ctx->frame_length, frame_length, ctx->min_frame_length);
}


/*************************************************************************
 * FUNCTION
 *	set_shutter_frame_length
 *
 * DESCRIPTION
 *	for frame & 3A sync
 *
 *************************************************************************/
static void set_shutter_frame_length(struct subdrv_ctx *ctx, kal_uint16 shutter,
				     kal_uint16 frame_length,
				     kal_bool auto_extend_en)
{
	kal_uint16 realtime_fps = 0;
	kal_int32 dummy_line = 0;

	ctx->shutter = shutter;

	/* Change frame time */
	if (frame_length > 1)
		dummy_line = frame_length - ctx->frame_length;

	ctx->frame_length = ctx->frame_length + dummy_line;

	if (shutter > ctx->frame_length - imgsensor_info.margin)
		ctx->frame_length = shutter + imgsensor_info.margin;

	if (ctx->frame_length > imgsensor_info.max_frame_length)
		ctx->frame_length = imgsensor_info.max_frame_length;

	shutter = (shutter < imgsensor_info.min_shutter)
			? imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin))
			? (imgsensor_info.max_frame_length - imgsensor_info.margin)
			: shutter;

	if (ctx->autoflicker_en) {
		realtime_fps = ctx->pclk / ctx->line_length * 10 /
				ctx->frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(ctx, 296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(ctx, 146, 0);
		else {
			/* Extend frame length */
		write_cmos_sensor_8(ctx, 0x320e, (ctx->frame_length >> 8) & 0xFF);
		write_cmos_sensor_8(ctx, 0x320f, ctx->frame_length & 0xFF);
		}
	} else {
		write_cmos_sensor_8(ctx, 0x320e, (ctx->frame_length >> 8) & 0xFF);
		write_cmos_sensor_8(ctx, 0x320f, ctx->frame_length & 0xFF);
		}

	// Update Shutter

	write_cmos_sensor_8(ctx, 0x3e00, (shutter >> 12) & 0x0F);
	write_cmos_sensor_8(ctx, 0x3e01, (shutter >> 4) & 0xFF);
	write_cmos_sensor_8(ctx, 0x3e02, (shutter << 4) & 0xF0);
	SC202CS_LOG_DBG("set shutter =%d, framelength =%d\n", shutter, ctx->frame_length);

}	/* set_shutter_frame_length */

static void set_multi_shutter_frame_length(struct subdrv_ctx *ctx,
				kal_uint32 *shutters, kal_uint16 shutter_cnt,
				kal_uint16 frame_length)
{
	kal_uint16 realtime_fps = 0;
	kal_int32 dummy_line = 0;

	ctx->shutter = shutters[0];

	/* Change frame time */
	if (frame_length > 1)
		dummy_line = frame_length - ctx->frame_length;

	ctx->frame_length = ctx->frame_length + dummy_line;

	if (shutters[0] > ctx->frame_length - imgsensor_info.margin)
		ctx->frame_length = shutters[0] + imgsensor_info.margin;

	if (ctx->frame_length > imgsensor_info.max_frame_length)
		ctx->frame_length = imgsensor_info.max_frame_length;

	shutters[0] = (shutters[0] < imgsensor_info.min_shutter)
			? imgsensor_info.min_shutter : shutters[0];
	shutters[0] = (shutters[0] > (imgsensor_info.max_frame_length - imgsensor_info.margin))
			? (imgsensor_info.max_frame_length - imgsensor_info.margin)
			: shutters[0];

	if (ctx->autoflicker_en) {
		realtime_fps = ctx->pclk / ctx->line_length * 10 /
				ctx->frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(ctx, 296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(ctx, 146, 0);
		else {
			/* Extend frame length */
		write_cmos_sensor_8(ctx, 0x320e, (ctx->frame_length >> 8) & 0xFF);
		write_cmos_sensor_8(ctx, 0x320f, ctx->frame_length & 0xFF);
		}
	} else {
			realtime_fps = ctx->pclk / ctx->line_length * 10 /
				ctx->frame_length;
		write_cmos_sensor_8(ctx, 0x320e, (ctx->frame_length >> 8) & 0xFF);
		write_cmos_sensor_8(ctx, 0x320f, ctx->frame_length & 0xFF);
		}
	realtime_fps = ctx->pclk / ctx->line_length * 10 /
			ctx->frame_length;
	// Update Shutter

	write_cmos_sensor_8(ctx, 0x3e00, (shutters[0] >> 12) & 0x0F);
	write_cmos_sensor_8(ctx, 0x3e01, (shutters[0] >> 4) & 0xFF);
	write_cmos_sensor_8(ctx, 0x3e02, (shutters[0] << 4) & 0xF0);
	ctx->frame_length_rg = ctx->frame_length;
	SC202CS_LOG_DBG("shutter_cnt = %d, set shutter =%d, framelength =%d, vrealtime_fps = %d, ctx->autoflicker_en = %d\n",
		shutter_cnt, shutters[0], ctx->frame_length, realtime_fps, ctx->autoflicker_en);
}

static kal_uint16 gain2reg(struct subdrv_ctx *ctx, const kal_uint32 gain)
{
	kal_uint32 reg_gain = gain;

	if (reg_gain < SC202CS_SENSOR_BASE_GAIN)
		reg_gain = SC202CS_SENSOR_BASE_GAIN;
	else if (reg_gain > SC202CS_SENSOR_MAX_GAIN)
		reg_gain = SC202CS_SENSOR_MAX_GAIN;
	return (kal_uint16) reg_gain;
}

/*************************************************************************
 * FUNCTION
 *	set_gain
 *
 * DESCRIPTION
 *	This function is to set global gain to sensor.
 *
 * PARAMETERS
 *	iGain : sensor global gain(base: 0x40)
 *
 * RETURNS
 *	the actually gain set to sensor.
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 set_gain(struct subdrv_ctx *ctx, kal_uint32 gain)
{
	kal_uint16 reg_gain;
	kal_uint32 temp_gain;
	kal_int16 gain_index;
	kal_uint16 SC202CS_AGC_Param[SC202CS_SENSOR_GAIN_MAP_SIZE][2] = {
		{  1024,  0x00 },
		{  2048,  0x01 },
		{  4096,  0x03 },
		{  8192,  0x07 },
		{ 16384,  0x0f },

	};

	reg_gain = gain2reg(ctx, gain);

	for (gain_index = SC202CS_SENSOR_GAIN_MAX_VALID_INDEX - 1; gain_index >= 0; gain_index--)
		if (reg_gain >= SC202CS_AGC_Param[gain_index][0])
			break;

	write_cmos_sensor_8(ctx, 0x3e09, SC202CS_AGC_Param[gain_index][1]);
	temp_gain = reg_gain * SC202CS_SENSOR_BASE_GAIN / SC202CS_AGC_Param[gain_index][0];
	write_cmos_sensor_8(ctx, 0x3e07, (temp_gain >> 3) & 0xff);
	LOG_INF("SC202CS_AGC_Param[gain_index][1] = 0x%x, temp_gain = 0x%x, reg_gain = %d, gain = %d\n",
		SC202CS_AGC_Param[gain_index][1], temp_gain, reg_gain, gain);
	return reg_gain;
}

/*************************************************************************
 * FUNCTION
 *	night_mode
 *
 * DESCRIPTION
 *	This function night mode of sensor.
 *
 * PARAMETERS
 *	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 streaming_control(struct subdrv_ctx *ctx, kal_bool enable)
{
	LOG_INF("streaming_enable(0=Sw Standby,1=streaming): %d\n", enable);

	if (enable) {
		write_cmos_sensor_8(ctx, 0x0100, 0x01);
	} else {
		write_cmos_sensor_8(ctx, 0x0100, 0x00);
	}
	mdelay(10);
	return ERROR_NONE;
}

static void sensor_init(struct subdrv_ctx *ctx)
{
	//Sensor Information////////////////////////////
	//Sensor	  : sc202cs
	//Customer        : MTK
	//Image size      : 1600x1200
	//MCLK/PCLK       : 24MHz /264Mhz
	//MIPI speed(Mbps): 1320Mbps x 4Lane
	//Frame Length    : 2500
	//Line Length     : 1760
	//Max Fps         : 30.00fps(vblank>1.0ms)
	//Pixel order     : Green 1st (=B)
	//X/Y-flip        : -flip off
	//Firmware Ver.   : v10.5.1
	////////////////////////////////////////////////
	LOG_INF("%s E\n", __func__);
    write_cmos_sensor_8(ctx, 0x0103,0x01);
    write_cmos_sensor_8(ctx, 0x0100,0x00);
    write_cmos_sensor_8(ctx, 0x36e9,0x80);
    write_cmos_sensor_8(ctx, 0x36e9,0x24);
    write_cmos_sensor_8(ctx, 0x301f,0x01);
    write_cmos_sensor_8(ctx, 0x3301,0xff);
    write_cmos_sensor_8(ctx, 0x3304,0x68);
    write_cmos_sensor_8(ctx, 0x3306,0x40);
    write_cmos_sensor_8(ctx, 0x3308,0x08);
    write_cmos_sensor_8(ctx, 0x3309,0xa8);
    write_cmos_sensor_8(ctx, 0x330b,0xb0);
    write_cmos_sensor_8(ctx, 0x330c,0x18);
    write_cmos_sensor_8(ctx, 0x330d,0xff);
    write_cmos_sensor_8(ctx, 0x330e,0x20);
    write_cmos_sensor_8(ctx, 0x331e,0x59);
    write_cmos_sensor_8(ctx, 0x331f,0x99);
    write_cmos_sensor_8(ctx, 0x3333,0x10);
    write_cmos_sensor_8(ctx, 0x335e,0x06);
    write_cmos_sensor_8(ctx, 0x335f,0x08);
    write_cmos_sensor_8(ctx, 0x3364,0x1f);
    write_cmos_sensor_8(ctx, 0x337c,0x02);
    write_cmos_sensor_8(ctx, 0x337d,0x0a);
    write_cmos_sensor_8(ctx, 0x338f,0xa0);
    write_cmos_sensor_8(ctx, 0x3390,0x01);
    write_cmos_sensor_8(ctx, 0x3391,0x03);
    write_cmos_sensor_8(ctx, 0x3392,0x1f);
    write_cmos_sensor_8(ctx, 0x3393,0xff);
    write_cmos_sensor_8(ctx, 0x3394,0xff);
    write_cmos_sensor_8(ctx, 0x3395,0xff);
    write_cmos_sensor_8(ctx, 0x33a2,0x04);
    write_cmos_sensor_8(ctx, 0x33ad,0x0c);
    write_cmos_sensor_8(ctx, 0x33b1,0x20);
    write_cmos_sensor_8(ctx, 0x33b3,0x38);
    write_cmos_sensor_8(ctx, 0x33f9,0x40);
    write_cmos_sensor_8(ctx, 0x33fb,0x48);
    write_cmos_sensor_8(ctx, 0x33fc,0x0f);
    write_cmos_sensor_8(ctx, 0x33fd,0x1f);
    write_cmos_sensor_8(ctx, 0x349f,0x03);
    write_cmos_sensor_8(ctx, 0x34a6,0x03);
    write_cmos_sensor_8(ctx, 0x34a7,0x1f);
    write_cmos_sensor_8(ctx, 0x34a8,0x38);
    write_cmos_sensor_8(ctx, 0x34a9,0x30);
    write_cmos_sensor_8(ctx, 0x34ab,0xb0);
    write_cmos_sensor_8(ctx, 0x34ad,0xb0);
    write_cmos_sensor_8(ctx, 0x34f8,0x1f);
    write_cmos_sensor_8(ctx, 0x34f9,0x20);
    write_cmos_sensor_8(ctx, 0x3630,0xa0);
    write_cmos_sensor_8(ctx, 0x3631,0x92);
    write_cmos_sensor_8(ctx, 0x3632,0x64);
    write_cmos_sensor_8(ctx, 0x3633,0x43);
    write_cmos_sensor_8(ctx, 0x3637,0x49);
    write_cmos_sensor_8(ctx, 0x363a,0x85);
    write_cmos_sensor_8(ctx, 0x363c,0x0f);
    write_cmos_sensor_8(ctx, 0x3650,0x11);
    write_cmos_sensor_8(ctx, 0x3651,0x7f);
    write_cmos_sensor_8(ctx, 0x3670,0x0d);
    write_cmos_sensor_8(ctx, 0x3674,0xc0);
    write_cmos_sensor_8(ctx, 0x3675,0xa0);
    write_cmos_sensor_8(ctx, 0x3676,0xa0);
    write_cmos_sensor_8(ctx, 0x3677,0x92);
    write_cmos_sensor_8(ctx, 0x3678,0x96);
    write_cmos_sensor_8(ctx, 0x3679,0x9a);
    write_cmos_sensor_8(ctx, 0x367c,0x03);
    write_cmos_sensor_8(ctx, 0x367d,0x0f);
    write_cmos_sensor_8(ctx, 0x367e,0x01);
    write_cmos_sensor_8(ctx, 0x367f,0x0f);
    write_cmos_sensor_8(ctx, 0x3698,0x83);
    write_cmos_sensor_8(ctx, 0x3699,0x86);
    write_cmos_sensor_8(ctx, 0x369a,0x8c);
    write_cmos_sensor_8(ctx, 0x369b,0x94);
    write_cmos_sensor_8(ctx, 0x36a2,0x01);
    write_cmos_sensor_8(ctx, 0x36a3,0x03);
    write_cmos_sensor_8(ctx, 0x36a4,0x07);
    write_cmos_sensor_8(ctx, 0x36ae,0x0f);
    write_cmos_sensor_8(ctx, 0x36af,0x1f);
    write_cmos_sensor_8(ctx, 0x36bd,0x22);
    write_cmos_sensor_8(ctx, 0x36be,0x22);
    write_cmos_sensor_8(ctx, 0x36bf,0x22);
    write_cmos_sensor_8(ctx, 0x36d0,0x01);
    write_cmos_sensor_8(ctx, 0x370f,0x02);
    write_cmos_sensor_8(ctx, 0x3721,0x6c);
    write_cmos_sensor_8(ctx, 0x3722,0x8d);
    write_cmos_sensor_8(ctx, 0x3725,0xc5);
    write_cmos_sensor_8(ctx, 0x3727,0x14);
    write_cmos_sensor_8(ctx, 0x3728,0x04);
    write_cmos_sensor_8(ctx, 0x37b7,0x04);
    write_cmos_sensor_8(ctx, 0x37b8,0x04);
    write_cmos_sensor_8(ctx, 0x37b9,0x06);
    write_cmos_sensor_8(ctx, 0x37bd,0x07);
    write_cmos_sensor_8(ctx, 0x37be,0x0f);
    write_cmos_sensor_8(ctx, 0x3901,0x02);
    write_cmos_sensor_8(ctx, 0x3903,0x40);
    write_cmos_sensor_8(ctx, 0x3905,0x8d);
    write_cmos_sensor_8(ctx, 0x3907,0x00);
    write_cmos_sensor_8(ctx, 0x3908,0x41);
    write_cmos_sensor_8(ctx, 0x391f,0x41);
    write_cmos_sensor_8(ctx, 0x3933,0x80);
    write_cmos_sensor_8(ctx, 0x3934,0x02);
    write_cmos_sensor_8(ctx, 0x3937,0x6f);
    write_cmos_sensor_8(ctx, 0x393a,0x01);
    write_cmos_sensor_8(ctx, 0x393d,0x01);
    write_cmos_sensor_8(ctx, 0x393e,0xc0);
    write_cmos_sensor_8(ctx, 0x39dd,0x41);
    write_cmos_sensor_8(ctx, 0x3e00,0x00);
    write_cmos_sensor_8(ctx, 0x3e01,0x4d);
    write_cmos_sensor_8(ctx, 0x3e02,0xc0);
    write_cmos_sensor_8(ctx, 0x3e09,0x00);
    write_cmos_sensor_8(ctx, 0x4509,0x28);
    write_cmos_sensor_8(ctx, 0x450d,0x61);
	LOG_INF("init end！\n");

}

static void preview_setting(struct subdrv_ctx *ctx)
{
	write_cmos_sensor_8(ctx, 0x0100, 0x01);
	LOG_INF("%s E!\n", __func__);

}

static void capture_setting(struct subdrv_ctx *ctx, kal_uint16 currefps)
{
	LOG_INF("%s E!\n", __func__);

}

static void normal_video_setting(struct subdrv_ctx *ctx, kal_uint16 currefps)
{
	LOG_INF("%s", __func__);
	
}

static void hs_video_setting(struct subdrv_ctx *ctx)
{
	LOG_INF("%s", __func__);	

}

static void slim_video_setting(struct subdrv_ctx *ctx)
{
	LOG_INF("%s", __func__);
	
}
/*static void custom1_setting(struct subdrv_ctx *ctx)
{
	LOG_INF("%s", __func__);
	write_cmos_sensor_8(ctx, 0x0100, 0x00);

}*/

/*static kal_uint32 return_sensor_id(struct subdrv_ctx *ctx)
{
	kal_uint32 sensor_id = 0;

	sensor_id = ((read_cmos_sensor_8(ctx, 0xf0) << 8) | read_cmos_sensor_8(ctx, 0xf1));

	SC202CS_LOG_DBG("[%s] sensor_id: 0x%x", __func__, sensor_id);

	return sensor_id;
}*/
/*************************************************************************
 * FUNCTION
 *	get_imgsensor_id
 *
 * DESCRIPTION
 *	This function get the sensor ID
 *
 * PARAMETERS
 *	*sensorID : return the sensor ID
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static int get_imgsensor_id(struct subdrv_ctx *ctx, UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint8 depth_camera_sn1[64]={0};
	unsigned char buf[50];
  	char module_id;
	UINT32 module_id_addr = EEPROM_SC202CS_MOLDULEID_0_ADDR;

	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		ctx->i2c_write_id = imgsensor_info.i2c_addr_table[i];
		do {
			*sensor_id = ((read_cmos_sensor_8(ctx, 0x3107) << 8)
				      | read_cmos_sensor_8(ctx, 0x3108));
			module_id = read_cmos_sensor_8(ctx, module_id_addr);
			if (module_id != EEPROM_SC202CS_MOLDULEID){
				module_id = read_cmos_sensor_8(ctx, EEPROM_SC202CS_MOLDULEID_1_ADDR);
				module_id_addr = EEPROM_SC202CS_MOLDULEID_1_ADDR;
			}
			LOG_INF("[sc202cs_camera_sensor]get_imgsensor_id: ! imgsensor_info.sensor_id: 0x%x, sensor id: 0x%x module_id = %c\n",imgsensor_info.sensor_id, *sensor_id, module_id);
			if (*sensor_id == imgsensor_info.sensor_id && (module_id == EEPROM_SC202CS_MOLDULEID)) {//判断是否是一供
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", ctx->i2c_write_id, *sensor_id);

				for(i = 0; i < 20; i++)
				{
					if(((i != 3) && (i != 4 ) && (i != 5) && (i != 6) && (i != 7)) && module_id_addr == EEPROM_SC202CS_MOLDULEID_0_ADDR )//没有烧录错误
					{
							depth_camera_sn1[i]=read_cmos_sensor_8(ctx, module_id_addr+i);
							sprintf(buf, "%02x", depth_camera_sn1[i]);
							strcat(depth_camera_sn, buf);
							LOG_INF("sc202cs i= %d reg = 0x%x SN1: 0x%x \n",i,EEPROM_SC202CS_MOLDULEID_0_ADDR+i,depth_camera_sn1[i]);
					} else if(((i != 6) && (i != 8 ) ) && module_id_addr == EEPROM_SC202CS_MOLDULEID_1_ADDR )//sn存在烧录错误
					{
							depth_camera_sn1[i]=read_cmos_sensor_8(ctx, module_id_addr+i);
							sprintf(buf, "%02x", depth_camera_sn1[i]);
							strcat(depth_camera_sn, buf);
							LOG_INF("sc202cs i= %d reg = 0x%x SN1: 0x%x \n",i,EEPROM_SC202CS_MOLDULEID_1_ADDR+i,depth_camera_sn1[i]);
					}

				}
				for( i = 0; i < 20; i++){
					if( i==16 || i==17 || i==18 || i==19)
					depth_camera_sn[i] = '0';
				LOG_INF("sc202cs  SN: 0x%x,str=%c \n",depth_camera_sn[i],depth_camera_sn[i]);
				}
				depth_camera_sn[20]='\0';
				LOG_INF("sc202cs SNstr = %s \n",depth_camera_sn);
				main_module_id[0] = depth_camera_sn[0];
				main_module_id[1] = ',';
				LOG_INF("sc202cs main_module_id = %s \n",main_module_id);	
				return ERROR_NONE;
			} else {
				LOG_INF("Read sensor id fail, id: 0x%x\n", ctx->i2c_write_id);
				retry--;
			}
		} while (retry > 0);
		i++;
		retry = 2;
	}
	if (*sensor_id != imgsensor_info.sensor_id ||
	    (module_id != EEPROM_SC202CS_MOLDULEID)) {
	/* if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF */
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	return ERROR_NONE;
}


/*************************************************************************
 * FUNCTION
 *	open
 *
 * DESCRIPTION
 *	This function initialize the registers of CMOS sensor
 *
 * PARAMETERS
 *	None
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static int open(struct subdrv_ctx *ctx)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint32 sensor_id = 0;
	LOG_INF("%s", __func__);

	SC202CS_LOG_DBG("preview 1608*1204@30.1fps; capture1608*1204@30.1fps\n");
	/* sensor have two i2c address 0x6c 0x6d & 0x21 0x20,
	 * we should detect the module used i2c address
	 */
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		ctx->i2c_write_id = imgsensor_info.i2c_addr_table[i];
		do {
			sensor_id = ((read_cmos_sensor_8(ctx, 0x3107) << 8)
				      | read_cmos_sensor_8(ctx, 0x3108));
			LOG_INF("i2c read sensor_id addr: 0x%x, sensor id: 0x%x imgsensor_info.sensor_id %x\n", ctx->i2c_write_id, sensor_id, imgsensor_info.sensor_id);
			if (sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n",
					ctx->i2c_write_id, sensor_id);
				break;
			}
			LOG_INF("Read sensor id fail, id: 0x%x\n",
				ctx->i2c_write_id);
			retry--;
		} while (retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id)
			break;
		retry = 2;
	}
	if (imgsensor_info.sensor_id != sensor_id)
		return ERROR_SENSOR_CONNECT_FAIL;

	/* initail sequence write in  */
	sensor_init(ctx);


	ctx->autoflicker_en = KAL_FALSE;
	ctx->sensor_mode = IMGSENSOR_MODE_INIT;
	ctx->shutter = 0x3d0;
	ctx->gain = 0x100;
	ctx->pclk = imgsensor_info.pre.pclk;
	ctx->frame_length = imgsensor_info.pre.framelength;
	ctx->line_length = imgsensor_info.pre.linelength;
	ctx->min_frame_length = imgsensor_info.pre.framelength;
	ctx->dummy_pixel = 0;
	ctx->dummy_line = 0;
	ctx->ihdr_mode = 0;
	ctx->test_pattern = KAL_FALSE;
	ctx->current_fps = imgsensor_info.pre.max_framerate;

	return ERROR_NONE;
}

/*************************************************************************
 * FUNCTION
 *	close
 *
 * DESCRIPTION
 *
 *
 * PARAMETERS
 *	None
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static int close(struct subdrv_ctx *ctx)
{
	LOG_INF("E\n");
	streaming_control(ctx, KAL_FALSE);

	return ERROR_NONE;
}

/*************************************************************************
 * FUNCTION
 * preview
 *
 * DESCRIPTION
 *	This function start the sensor preview.
 *
 * PARAMETERS
 *	*image_window : address pointer of pixel numbers in one period of HSYNC
 *  *sensor_config_data : address pointer of line numbers in one period of VSYNC
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 preview(struct subdrv_ctx *ctx, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("%s E\n", __func__);

	ctx->sensor_mode = IMGSENSOR_MODE_PREVIEW;
	ctx->pclk = imgsensor_info.pre.pclk;
	ctx->line_length = imgsensor_info.pre.linelength;
	ctx->frame_length = imgsensor_info.pre.framelength;
	ctx->min_frame_length = imgsensor_info.pre.framelength;
	ctx->autoflicker_en = KAL_FALSE;

	preview_setting(ctx);

	return ERROR_NONE;
}

/*************************************************************************
 * FUNCTION
 *	capture
 *
 * DESCRIPTION
 *	This function setup the CMOS sensor in capture MY_OUTPUT mode
 *
 * PARAMETERS
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 capture(struct subdrv_ctx *ctx,
		MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("%s E\n", __func__);
	ctx->sensor_mode = IMGSENSOR_MODE_CAPTURE;

	if (ctx->current_fps != imgsensor_info.cap.max_framerate)
		SC202CS_LOG_DBG(
			"Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
			ctx->current_fps,
			imgsensor_info.cap.max_framerate / 10);
	ctx->pclk = imgsensor_info.cap.pclk;
	ctx->line_length = imgsensor_info.cap.linelength;
	ctx->frame_length = imgsensor_info.cap.framelength;
	ctx->min_frame_length = imgsensor_info.cap.framelength;
	ctx->autoflicker_en = KAL_FALSE;

	capture_setting(ctx, ctx->current_fps);

	return ERROR_NONE;
}

static kal_uint32 normal_video(struct subdrv_ctx *ctx,
	MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("%s E\n", __func__);

	ctx->sensor_mode = IMGSENSOR_MODE_VIDEO;
	ctx->pclk = imgsensor_info.normal_video.pclk;
	ctx->line_length = imgsensor_info.normal_video.linelength;
	ctx->frame_length = imgsensor_info.normal_video.framelength;
	ctx->min_frame_length = imgsensor_info.normal_video.framelength;
	ctx->autoflicker_en = KAL_FALSE;

	normal_video_setting(ctx, ctx->current_fps);

	return ERROR_NONE;
}

static kal_uint32 hs_video(struct subdrv_ctx *ctx, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			   MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("%s E\n", __func__);

	ctx->sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	ctx->pclk = imgsensor_info.hs_video.pclk;
	/* ctx->video_mode = KAL_TRUE; */
	ctx->line_length = imgsensor_info.hs_video.linelength;
	ctx->frame_length = imgsensor_info.hs_video.framelength;
	ctx->min_frame_length = imgsensor_info.hs_video.framelength;
	ctx->dummy_line = 0;
	ctx->dummy_pixel = 0;
	/* ctx->current_fps = 300; */
	ctx->autoflicker_en = KAL_FALSE;
	hs_video_setting(ctx);

	return ERROR_NONE;
}

static kal_uint32 slim_video(struct subdrv_ctx *ctx,
	MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("%s E\n", __func__);

	ctx->sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	ctx->pclk = imgsensor_info.slim_video.pclk;
	/* ctx->video_mode = KAL_TRUE; */
	ctx->line_length = imgsensor_info.slim_video.linelength;
	ctx->frame_length = imgsensor_info.slim_video.framelength;
	ctx->min_frame_length = imgsensor_info.slim_video.framelength;
	ctx->dummy_line = 0;
	ctx->dummy_pixel = 0;
	/* ctx->current_fps = 300; */
	ctx->autoflicker_en = KAL_FALSE;
	slim_video_setting(ctx);

	return ERROR_NONE;
}

static int get_resolution(struct subdrv_ctx *ctx,
	MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	int i = 0;

	for (i = SENSOR_SCENARIO_ID_MIN; i < SENSOR_SCENARIO_ID_MAX; i++) {
		if (i < imgsensor_info.sensor_mode_num) {
			sensor_resolution->SensorWidth[i] = imgsensor_winsize_info[i].w2_tg_size;
			sensor_resolution->SensorHeight[i] = imgsensor_winsize_info[i].h2_tg_size;
		} else {
			sensor_resolution->SensorWidth[i] = 0;
			sensor_resolution->SensorHeight[i] = 0;
		}
	}
	return ERROR_NONE;
}

static int get_info(struct subdrv_ctx *ctx, enum MSDK_SCENARIO_ID_ENUM scenario_id,
			   MSDK_SENSOR_INFO_STRUCT *sensor_info,
			   MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	/*LOG_INF("get_info -> scenario_id = %d\n", scenario_id);*/
	int i = 0;
	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;

	/* not use */
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW;

	/* inverse with datasheet */
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;

	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4;	/* not use */
	sensor_info->SensorResetActiveHigh = FALSE;	/* not use */
	sensor_info->SensorResetDelayCount = 5;	/* not use */

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;

	sensor_info->SensorOutputDataFormat =
		imgsensor_info.sensor_output_dataformat;

	sensor_info->DelayFrame[SENSOR_SCENARIO_ID_NORMAL_PREVIEW] =
		imgsensor_info.pre_delay_frame;
	sensor_info->DelayFrame[SENSOR_SCENARIO_ID_NORMAL_CAPTURE] =
		imgsensor_info.cap_delay_frame;
	sensor_info->DelayFrame[SENSOR_SCENARIO_ID_NORMAL_VIDEO] =
		imgsensor_info.video_delay_frame;
	sensor_info->DelayFrame[SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO] =
		imgsensor_info.hs_video_delay_frame;
	sensor_info->DelayFrame[SENSOR_SCENARIO_ID_SLIM_VIDEO] =
		imgsensor_info.slim_video_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

	/* The frame of setting shutter default 0 for TG int */
	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;

	/* The frame of setting sensor gain*/
	sensor_info->AESensorGainDelayFrame =
				imgsensor_info.ae_sensor_gain_delay_frame;

	sensor_info->AEISPGainDelayFrame =
				imgsensor_info.ae_ispGain_delay_frame;

	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
	sensor_info->TEMPERATURE_SUPPORT = imgsensor_info.temperature_support;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3; /* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2; /* not use */
	sensor_info->SensorPixelClockCount = 3; /* not use */
	sensor_info->SensorDataLatchCount = 2; /* not use */


	sensor_info->SensorWidthSampling = 0; /* 0 is default 1x */
	sensor_info->SensorHightSampling = 0; /* 0 is default 1x */
	sensor_info->SensorPacketECCOrder = 1;

	sensor_info->FrameTimeDelayFrame =
		imgsensor_info.frame_time_delay_frame;

	for (i = 0; i < imgsensor_info.sensor_mode_num; i++){
		sensor_info->gain_ratio[i] =
			imgsensor_saturation_info.gain_ratio;
		sensor_info->OB_pedestals[i] =
			imgsensor_saturation_info.OB_pedestal;
		sensor_info->saturation_level[i] =
			imgsensor_saturation_info.saturation_level;
	}

	for (i = 0; i < imgsensor_info.sensor_mode_num; i++){
		sensor_info->max_framelength[i] = imgsensor_info.max_frame_length;
	}

	return ERROR_NONE;
}	/*	get_info  */


static int control(struct subdrv_ctx *ctx,
		enum MSDK_SCENARIO_ID_ENUM scenario_id,
		MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	ctx->current_scenario_id = scenario_id;
	switch (scenario_id) {
	case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		preview(ctx, image_window, sensor_config_data);
		break;
	case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
		capture(ctx, image_window, sensor_config_data);
		break;
	case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
		normal_video(ctx, image_window, sensor_config_data);
		break;
	case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
		hs_video(ctx, image_window, sensor_config_data);
		break;
	case SENSOR_SCENARIO_ID_SLIM_VIDEO:
		slim_video(ctx, image_window, sensor_config_data);
		break;
	default:
		LOG_INF("Error ScenarioId setting");
		preview(ctx, image_window, sensor_config_data);
		return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}

static kal_uint32 set_video_mode(struct subdrv_ctx *ctx, UINT16 framerate)
{
	/* //LOG_INF("framerate = %d\n ", framerate); */
	/* SetVideoMode Function should fix framerate */
	if (framerate == 0)
		/* Dynamic frame rate */
		return ERROR_NONE;
	if ((framerate == 300) && (ctx->autoflicker_en == KAL_TRUE))
		ctx->current_fps = 296;
	else if ((framerate == 150) && (ctx->autoflicker_en == KAL_TRUE))
		ctx->current_fps = 146;
	else
		ctx->current_fps = framerate;
	set_max_framerate(ctx, ctx->current_fps, 1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(struct subdrv_ctx *ctx,
	kal_bool enable, UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d\n", enable, framerate);
	if (enable)		/* enable auto flicker */
		ctx->autoflicker_en = KAL_TRUE;
	else			/* Cancel Auto flick */
		ctx->autoflicker_en = KAL_FALSE;
	return ERROR_NONE;
}

static kal_uint32 set_max_framerate_by_scenario(struct subdrv_ctx *ctx,
	enum MSDK_SCENARIO_ID_ENUM scenario_id,	MUINT32 framerate)
{
	kal_uint32 frame_length;

	LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	switch (scenario_id) {
	case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		frame_length = imgsensor_info.pre.pclk / framerate * 10
				/ imgsensor_info.pre.linelength;
		ctx->dummy_line =
			(frame_length > imgsensor_info.pre.framelength)
		? (frame_length - imgsensor_info.pre.framelength) : 0;
		ctx->frame_length =
			imgsensor_info.pre.framelength
			+ ctx->dummy_line;
		ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
		break;
	case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
		if (framerate == 0)
			return ERROR_NONE;
		frame_length = imgsensor_info.normal_video.pclk /
				framerate * 10 /
				imgsensor_info.normal_video.linelength;
		ctx->dummy_line =
			(frame_length > imgsensor_info.normal_video.framelength)
		? (frame_length - imgsensor_info.normal_video.framelength)
		: 0;
		ctx->frame_length =
			imgsensor_info.normal_video.framelength
			+ ctx->dummy_line;
		ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
		break;
	case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
	if (ctx->current_fps != imgsensor_info.cap.max_framerate)
		SC202CS_LOG_DBG(
			"Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n"
			, framerate, imgsensor_info.cap.max_framerate/10);
		frame_length = imgsensor_info.cap.pclk / framerate * 10
				/ imgsensor_info.cap.linelength;
			ctx->dummy_line =
			(frame_length > imgsensor_info.cap.framelength)
			  ? (frame_length - imgsensor_info.cap.framelength) : 0;
			ctx->frame_length =
				imgsensor_info.cap.framelength
				+ ctx->dummy_line;
			ctx->min_frame_length = ctx->frame_length;

		if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
		break;
	case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
		frame_length = imgsensor_info.hs_video.pclk / framerate * 10
				/ imgsensor_info.hs_video.linelength;
		ctx->dummy_line =
			(frame_length > imgsensor_info.hs_video.framelength)
			  ? (frame_length - imgsensor_info.hs_video.framelength)
			  : 0;
		ctx->frame_length =
			imgsensor_info.hs_video.framelength
				+ ctx->dummy_line;
		ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
		break;
	case SENSOR_SCENARIO_ID_SLIM_VIDEO:
		frame_length = imgsensor_info.slim_video.pclk / framerate * 10
			/ imgsensor_info.slim_video.linelength;
		ctx->dummy_line =
			(frame_length > imgsensor_info.slim_video.framelength)
			? (frame_length - imgsensor_info.slim_video.framelength)
			: 0;
		ctx->frame_length =
			imgsensor_info.slim_video.framelength
			+ ctx->dummy_line;
		ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
		break;
	default:  /*coding with  preview scenario by default*/
		frame_length = imgsensor_info.pre.pclk / framerate * 10
			/ imgsensor_info.pre.linelength;
		ctx->dummy_line =
			(frame_length > imgsensor_info.pre.framelength)
			? (frame_length - imgsensor_info.pre.framelength) : 0;
		ctx->frame_length =
			imgsensor_info.pre.framelength + ctx->dummy_line;
		ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
		LOG_INF("error scenario_id = %d, we use preview scenario\n",
		scenario_id);
		break;
	}
	return ERROR_NONE;
}

static kal_uint32 get_default_framerate_by_scenario(struct subdrv_ctx *ctx,
	enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
	/*LOG_INF("scenario_id = %d\n", scenario_id);*/

	switch (scenario_id) {
	case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		*framerate = imgsensor_info.pre.max_framerate;
		break;
	case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
		*framerate = imgsensor_info.normal_video.max_framerate;
		break;
	case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
		*framerate = imgsensor_info.cap.max_framerate;
		break;
	case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
		*framerate = imgsensor_info.hs_video.max_framerate;
		break;
	case SENSOR_SCENARIO_ID_SLIM_VIDEO:
		*framerate = imgsensor_info.slim_video.max_framerate;
		break;
	default:
		break;
	}

	return ERROR_NONE;
}

static int feature_control(	struct subdrv_ctx *ctx,MSDK_SENSOR_FEATURE_ENUM feature_id,
				  UINT8 *feature_para, UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16 = (UINT16 *) feature_para;
	UINT16 *feature_data_16 = (UINT16 *) feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *) feature_para;
	UINT32 *feature_data_32 = (UINT32 *) feature_para;
	unsigned long long *feature_data = (unsigned long long *) feature_para;
	/* unsigned long long *feature_return_para
	 *  = (unsigned long long *) feature_para;
	 */
	struct u64_min_max multi_exposure_shutter_range[5];
	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
	struct SENSOR_VC_INFO_STRUCT *pvcinfo;
	/* SET_SENSOR_AWB_GAIN *pSetSensorAWB
	 *  = (SET_SENSOR_AWB_GAIN *)feature_para;
	 */
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data
		= (MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

	/*LOG_INF("feature_id = %d\n", feature_id);*/
	switch (feature_id) {
	case SENSOR_FEATURE_GET_OUTPUT_FORMAT_BY_SCENARIO:
		switch (*feature_data) {
		default:
			*(feature_data + 1)
			= (enum ACDK_SENSOR_OUTPUT_DATA_FORMAT_ENUM)
				imgsensor_info.sensor_output_dataformat;
			break;
		}
	break;
	case SENSOR_FEATURE_GET_AWB_REQ_BY_SCENARIO:
		switch (*feature_data) {
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 0;
			break;
		}
		break;
	case SENSOR_FEATURE_GET_ANA_GAIN_TABLE:
		if ((void *)(uintptr_t) (*(feature_data + 1)) == NULL) {
			*(feature_data + 0) =
				sizeof(sc202cs_ana_gain_table);
		} else {
			memcpy((void *)(uintptr_t) (*(feature_data + 1)),
			(void *)sc202cs_ana_gain_table,
			sizeof(sc202cs_ana_gain_table));
		}
		break;

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
		*(feature_data + 2) = imgsensor_info.exp_step;
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ_BY_SCENARIO:
		switch (*feature_data) {
		case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.cap.pclk;
			break;
		case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.normal_video.pclk;
			break;
		case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.hs_video.pclk;
			break;
		case SENSOR_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.slim_video.pclk;
			break;
		case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.pre.pclk;
			break;
		}
		break;
	case SENSOR_FEATURE_GET_PERIOD_BY_SCENARIO:
		switch (*feature_data) {
		case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.cap.framelength << 16)
				+ imgsensor_info.cap.linelength;
			break;
		case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.normal_video.framelength << 16)
				+ imgsensor_info.normal_video.linelength;
			break;
		case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.hs_video.framelength << 16)
				+ imgsensor_info.hs_video.linelength;
			break;
		case SENSOR_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.slim_video.framelength << 16)
				+ imgsensor_info.slim_video.linelength;
			break;
		case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.pre.framelength << 16)
				+ imgsensor_info.pre.linelength;
			break;
		}
		break;

	case SENSOR_FEATURE_GET_PERIOD:
		*feature_return_para_16++ = ctx->line_length;
		*feature_return_para_16 = ctx->frame_length;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
		*feature_return_para_32 = ctx->pclk;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_ESHUTTER:
		set_shutter(ctx, *feature_data);
		break;
	case SENSOR_FEATURE_SET_NIGHTMODE:

		break;
	#ifdef VENDOR_EDIT
	case SENSOR_FEATURE_CHECK_MODULE_ID:
		*feature_return_para_32 = imgsensor_info.module_id;
		break;
	#endif
	case SENSOR_FEATURE_SET_GAIN:
		set_gain(ctx, (UINT16) *feature_data);
		break;
	case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
	case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
		break;

	case SENSOR_FEATURE_SET_REGISTER:
		write_cmos_sensor_8(ctx, sensor_reg_data->RegAddr,
			sensor_reg_data->RegData);
		break;

	case SENSOR_FEATURE_GET_REGISTER:
		sensor_reg_data->RegData =
			read_cmos_sensor_8(ctx, sensor_reg_data->RegAddr);
		break;

	case SENSOR_FEATURE_GET_LENS_DRIVER_ID:

		*feature_return_para_32 = LENS_DRIVER_ID_DO_NOT_CARE;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_VIDEO_MODE:
		set_video_mode(ctx, *feature_data);
		break;
	case SENSOR_FEATURE_CHECK_SENSOR_ID:
		get_imgsensor_id(ctx, feature_return_para_32);
		break;
	case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
		set_auto_flicker_mode(ctx, (BOOL)*feature_data_16,
					*(feature_data_16 + 1));
		break;
	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
		set_max_framerate_by_scenario(ctx,
				(enum MSDK_SCENARIO_ID_ENUM)*feature_data,
				*(feature_data+1));
		break;

	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
		get_default_framerate_by_scenario(ctx,
			(enum MSDK_SCENARIO_ID_ENUM) *(feature_data),
			  (MUINT32 *) (uintptr_t) (*(feature_data + 1)));
		break;
	case SENSOR_FEATURE_GET_PDAF_DATA:
		SC202CS_LOG_DBG("SENSOR_FEATURE_GET_PDAF_DATA\n");
		break;

	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
		*feature_return_para_32 = imgsensor_info.checksum_value;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_FRAMERATE:
		LOG_INF("current fps :%d\n", *feature_data_32);
		ctx->current_fps = (UINT16)*feature_data_32;
		break;
	case SENSOR_FEATURE_SET_HDR:
		LOG_INF("ihdr enable :%d\n", *feature_data_32);
		ctx->ihdr_mode = (UINT8)*feature_data_32;
		break;

	case SENSOR_FEATURE_GET_CROP_INFO:
		wininfo =
	(struct SENSOR_WINSIZE_INFO_STRUCT *) (uintptr_t) (*(feature_data + 1));

		switch (*feature_data_32) {
		case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[1],
			       sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[2],
			       sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[3],
			       sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case SENSOR_SCENARIO_ID_SLIM_VIDEO:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[4],
			       sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		default:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[0],
			       sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		}
		break;
	case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
		LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",
			(UINT16) *feature_data,
			(UINT16) *(feature_data + 1),
			(UINT16) *(feature_data + 2));
		break;
	case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
		set_shutter_frame_length(ctx, (UINT16) (*feature_data),
					(UINT16) (*(feature_data + 1)),
					(BOOL) (*(feature_data + 2)));
		break;
	case SENSOR_FEATURE_GET_FRAME_CTRL_INFO_BY_SCENARIO:
		/*
		 * 1, if driver support new sw frame sync
		 * set_shutter_frame_length(ctx) support third para auto_extend_en
		 */
		*(feature_data + 1) = 1;
		/* margin info by scenario */
		*(feature_data + 2) = imgsensor_info.margin;
		break;
	case SENSOR_FEATURE_SET_HDR_SHUTTER:
		LOG_INF("SENSOR_FEATURE_SET_HDR_SHUTTER LE=%d, SE=%d\n",
			(UINT16)*feature_data, (UINT16)*(feature_data+1));
		break;
	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
		LOG_INF("SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
		streaming_control(ctx, KAL_FALSE);
		break;
	case SENSOR_FEATURE_SET_STREAMING_RESUME:
		LOG_INF("SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%llu\n",
			*feature_data);
		if (*feature_data != 0)
			set_shutter(ctx, *feature_data);
		streaming_control(ctx, KAL_TRUE);
		break;
	case SENSOR_FEATURE_GET_BINNING_TYPE:
		switch (*(feature_data + 1)) {
		default:
			*feature_return_para_32 = 1; /*BINNING_AVERAGED*/
			break;
		}
		SC202CS_LOG_DBG("SENSOR_FEATURE_GET_BINNING_TYPE AE_binning_type:%d,\n",
			*feature_return_para_32);
		*feature_para_len = 4;

		break;

	case SENSOR_FEATURE_GET_MULTI_EXP_SHUTTER_RANGE_BY_SCENARIO:
		switch (*(feature_data)) {
			case SENSOR_SCENARIO_ID_CUSTOM1:
			case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
			case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
			case SENSOR_SCENARIO_ID_SLIM_VIDEO:
			case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
			case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
			case SENSOR_SCENARIO_ID_CUSTOM2:
			case SENSOR_SCENARIO_ID_CUSTOM3:
			case SENSOR_SCENARIO_ID_CUSTOM4:
			case SENSOR_SCENARIO_ID_CUSTOM5:
			default:
			LOG_INF("SENSOR_FEATURE_GET_MULTI_EXP_SHUTTER_RANGE_BY_SCENARIO");
			multi_exposure_shutter_range[0].min=2;
			multi_exposure_shutter_range[0].max=(0xffff * 128) - 32;
			multi_exposure_shutter_range[1].min=2;
			multi_exposure_shutter_range[1].max=(0xffff * 128) - 32;
			multi_exposure_shutter_range[2].min=2;
			multi_exposure_shutter_range[2].max=(0xffff * 128) - 32;
			multi_exposure_shutter_range[3].min=2;
			multi_exposure_shutter_range[3].max=(0xffff * 128) - 32;
			multi_exposure_shutter_range[4].min=2;
			multi_exposure_shutter_range[4].max=(0xffff * 128) - 32;

			*(feature_data + 1)= 1; /*BINNING_AVERAGED*/

			memcpy((void *)(uintptr_t)(*(feature_data + 2)), multi_exposure_shutter_range,
				sizeof(struct u64_min_max)*IMGSENSOR_EXPOSURE_CNT);

			break;
			}
		break;

	case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:
	{
		switch (*feature_data) {
		case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.cap.mipi_pixel_rate;
			break;
		case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.normal_video.mipi_pixel_rate;
			break;
		case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.hs_video.mipi_pixel_rate;
			break;
		case SENSOR_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.slim_video.mipi_pixel_rate;
			break;
		case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.pre.mipi_pixel_rate;
			break;
		}
	}
		break;
	case SENSOR_FEATURE_GET_VC_INFO:
		pvcinfo =
		 (struct SENSOR_VC_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));
		switch (*feature_data_32) {
		case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
		case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
		case SENSOR_SCENARIO_ID_SLIM_VIDEO:
		default:
			//memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[0],sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;
		}
		break;
	case SENSOR_FEATURE_SET_AWB_GAIN:
		break;
	case SENSOR_FEATURE_SET_LSC_TBL:
		break;
	case SENSOR_FEATURE_SET_FRAMELENGTH:
		set_frame_length(ctx, (UINT16) (*feature_data));
		break;
	case SENSOR_FEATURE_SET_MULTI_SHUTTER_FRAME_TIME:
	LOG_INF("SENSOR_FEATURE_SET_MULTI_SHUTTER_FRAME_TIME");
		set_multi_shutter_frame_length(ctx, (UINT32 *)(*feature_data),
					(UINT16) (*(feature_data + 1)),
					(UINT16) (*(feature_data + 2)));
		break;
	default:
		break;
	}

	return ERROR_NONE;
}

#ifdef IMGSENSOR_VC_ROUTING
static struct mtk_mbus_frame_desc_entry frame_desc_prev[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 1600,
			.vsize = 1200,
		},
	},
};


static int get_frame_desc(struct subdrv_ctx *ctx,
		int scenario_id, struct mtk_mbus_frame_desc *fd)
{
	switch (scenario_id) {
	case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
	case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
	case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
	case SENSOR_SCENARIO_ID_SLIM_VIDEO:
	default:
		fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
		fd->num_entries = ARRAY_SIZE(frame_desc_prev);
		memcpy(fd->entry, frame_desc_prev, sizeof(frame_desc_prev));
		break;
	}

	return 0;
}
#endif

static const struct subdrv_ctx defctx = {

	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_max = BASEGAIN * 16,
	.ana_gain_min = BASEGAIN,
	.ana_gain_step = 1,
	.exposure_def = 0x9D8,
	.exposure_max = 0xffff - 6,
	.exposure_min = 2,
	.exposure_step = 1,
	.frame_time_delay_frame = 2,
	.margin = 6,
	.max_frame_length = 0xfff9,
	.mirror = IMAGE_NORMAL,

	/* IMGSENSOR_MODE enum value,record current sensor mode,such as:
	 * INIT, Preview, Capture, Video,High Speed Video, Slim Video
	 */
	.sensor_mode = IMGSENSOR_MODE_INIT,

	.shutter = 0x3d0,	/* current shutter */
	.gain = BASEGAIN * 4,		/* current gain */
	.dummy_pixel = 0,	/* current dummypixel */
	.dummy_line = 0,	/* current dummyline */
	.current_fps = 300,	/* full size current fps : 24fps for PIP,
				 * 30fps for Normal or ZSD
				 */

	/* auto flicker enable: KAL_FALSE for disable auto flicker,
	 * KAL_TRUE for enable auto flicker
	 */
	.autoflicker_en = KAL_FALSE,

	/* test pattern mode or not.
	* KAL_FALSE for in test pattern mode,
	* KAL_TRUE for normal output
	*/
	.test_pattern = KAL_FALSE,

	/* current scenario id */
	.current_scenario_id = SENSOR_SCENARIO_ID_NORMAL_PREVIEW,

	/* sensor need support LE, SE with HDR feature */
	.ihdr_mode = KAL_FALSE,
	.i2c_write_id = 0x6c,	/* record current sensor's i2c write id */
};

static int init_ctx(struct subdrv_ctx *ctx,
		struct i2c_client *i2c_client, u8 i2c_write_id)
{
	memcpy(ctx, &defctx, sizeof(*ctx));
	ctx->i2c_client = i2c_client;
	ctx->i2c_write_id = i2c_write_id;
	return 0;
}

static struct subdrv_ops ops = {
	.get_id = get_imgsensor_id,
	.init_ctx = init_ctx,
	.open = open,
	.get_info = get_info,
	.get_resolution = get_resolution,
	.control = control,
	.feature_control = feature_control,
	.close = close,
#ifdef IMGSENSOR_VC_ROUTING
	.get_frame_desc = get_frame_desc,
#endif
};

static struct subdrv_pw_seq_entry pw_seq[] = {
	{HW_ID_RST, 0, 10},
	{HW_ID_MCLK, 24, 10},
	{HW_ID_MCLK_DRIVING_CURRENT, 6, 1},
	{HW_ID_DOVDD, 1800000, 10},
	{HW_ID_DVDD, 1200000, 10},
	{HW_ID_AVDD, 2800000, 10},
	{HW_ID_RST, 1, 10},
};

const struct subdrv_entry sc202cs_mipi_raw_entry = {
	.name = "sc202cs_mipi_raw",
	.id = SC202CS_SENSOR_ID,
	.pw_seq = pw_seq,
	.pw_seq_cnt = ARRAY_SIZE(pw_seq),
	.ops = &ops,
};

