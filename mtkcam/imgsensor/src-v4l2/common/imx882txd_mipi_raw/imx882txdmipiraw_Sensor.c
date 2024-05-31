// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2019 MediaTek Inc.
/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 imx882txdmipi_Sensor.c
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
#include "imx882txdmipiraw_Sensor.h"
#include "imx882txd_ana_gain_table.h"
#include "imx882txd_eeprom.h"
#include "adaptor-subdrv.h"
#include "adaptor-i2c.h"
#define read_cmos_sensor_8(...) subdrv_i2c_rd_u8(__VA_ARGS__)
#define read_cmos_sensor(...) subdrv_i2c_rd_u16(__VA_ARGS__)
#define write_cmos_sensor_8(...) subdrv_i2c_wr_u8(__VA_ARGS__)
#define imx882txd_table_write_cmos_sensor(...) subdrv_i2c_wr_regs_u8(__VA_ARGS__)
#undef VENDOR_EDIT
/***************Modify Following Strings for Debug**********************/
#define DEBUG_LOG_EN 1
#define PFX "imx882txd_camera_sensor"
#define LOG_INF(format, args...) pr_info(PFX "[%s] " format, __func__, ##args)
#define LOG_DEBUG(...) do { if ((DEBUG_LOG_EN)) LOG_INF(__VA_ARGS__); } while (0)
#define MULTI_WRITE 1
#if MULTI_WRITE
#define I2C_BUFFER_LEN 765 /* trans# max is 255, each 3 bytes */
#else
#define I2C_BUFFER_LEN 3
#endif
#ifdef VENDOR_EDIT
#define MODULE_ID_OFFSET 0x0000
#endif
static DEFINE_SPINLOCK(imgsensor_drv_lock);

static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = IMX882TXD_SENSOR_ID,
	.checksum_value = 0x8ac2d94a,
	.pre = {
        .pclk = 878400000,
        .linelength = 7500,
        .framelength = 3900,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4000,
        .grabwindow_height = 3000,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 700800000,  //1022.00 Msps/Trio *3*2.28/10 = 699.048
        .max_framerate = 300,
	},
	.cap = {
        .pclk = 878400000,
        .linelength = 7500,
        .framelength = 3900,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4000,
        .grabwindow_height = 3000,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 700800000,  //1022.00 Msps/Trio *3*2.28/10 = 699.048
        .max_framerate = 300,
	},
	.normal_video = {
		.pclk = 878400000,
		.linelength = 7500,
		.framelength = 3900,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4096,
		.grabwindow_height = 2304,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 700800000,
		.max_framerate = 300, /* 30fps */
	},
	.hs_video = {
		.pclk = 708000000,
		.linelength = 4616,
		.framelength = 2547,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4096,
		.grabwindow_height = 2304,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 825600000,
		.max_framerate = 600, /* 60fps */
		.min_shutter = 5,
		.exp_step = 1,
	},
	.slim_video = {
		.pclk = 878400000,
		.linelength = 4024,
		.framelength = 1816,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2048,
		.grabwindow_height = 1152,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 1200,
		.mipi_pixel_rate = 741940000,
		.min_shutter = 12,
		.exp_step = 8,
	},
	.custom1 = {
		.pclk = 878400000,
		.linelength = 8960,
		.framelength = 6504,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 8192,
		.grabwindow_height = 6144,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 150,
		.mipi_pixel_rate = 1113600000,
	},
	.custom2 = {
		.pclk = 441600000,
		.linelength = 3768,
		.framelength = 1946,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3280,
		.grabwindow_height = 1856,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 418000000,
		.max_framerate = 600, /* 60fps */
	},
	.custom3 = {
		.pclk = 878400000,
		.linelength = 8960,
		.framelength = 3252,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4000,
		.grabwindow_height = 3000,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 685368000,
		.max_framerate = 300, /* 30fps */
		.min_shutter = 10, /* min shutter */
		.min_gain = 1024,
		.max_gain = 16384,
		.exp_step = 2,
	},
	.min_gain = 1463, /*1x gain*/
	.max_gain = BASEGAIN * 64, /*16x gain*/
	.min_gain_iso = 100,
	.margin = 64,		/* sensor framelength & shutter margin */
	.min_shutter = 6,	/* min shutter */
	.exp_step=4,
	.gain_step = 1,
	.gain_type = 0,
	.max_frame_length = 0xffff,
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 0,
	.ae_ispGain_delay_frame = 2,	/* isp gain delay frame for AE cycle */
	.ihdr_support = 0,	/* 1, support; 0,not support */
	.ihdr_le_firstline = 0,	/* 1,le first ; 0, se first */
	.temperature_support = 1, /* 1, support; 0,not support */
	.sensor_mode_num = 8,	/* support sensor mode num */
	.cap_delay_frame = 2,	/* enter capture delay frame num */
	.pre_delay_frame = 2,	/* enter preview delay frame num */
	.video_delay_frame = 2,	/* enter video delay frame num */
	.hs_video_delay_frame = 2,
	.slim_video_delay_frame = 2,	/* enter slim video delay frame num */
	.custom1_delay_frame = 2,	/* enter custom1 delay frame num */
	.custom2_delay_frame = 2,	/* enter custom2 delay frame num */
	.custom3_delay_frame = 2,
	.frame_time_delay_frame = 3,
	.isp_driving_current = ISP_DRIVING_4MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	/* .mipi_sensor_type = MIPI_OPHY_NCSI2, */
	/* 0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2 */
	.mipi_sensor_type = MIPI_CPHY, /* 0,MIPI_OPHY_NCSI2; 1,MIPI_OPHY_CSI2 */
	.mipi_settle_delay_mode = 0,
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_R,
	.mclk = 24, /* mclk value, suggest 24 or 26 for 24Mhz or 26Mhz */
	/*.mipi_lane_num = SENSOR_MIPI_4_LANE,*/
	.mipi_lane_num = SENSOR_MIPI_3_LANE,
	.i2c_addr_table = {0x34, 0xff},
	/* record sensor support all write id addr,
	 * only supprt 4 must end with 0xff
	 */
	.i2c_speed = 1000, /* i2c read/write speed */
};
/* Sensor output window information */
#if 0
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[6] = {
	{6560, 4928, 000, 000, 6560, 4928, 3280, 2464,0000, 0000, 3280, 2464, 0, 0, 3280, 2464}, /* Preview */
	{6560, 4928, 000, 000, 6560, 4928, 3280, 2464,0000, 0000, 3280, 2464, 0, 0, 3280, 2464}, /* capture */
	{6560, 4928, 000, 000, 6560, 4928, 3280, 2464,0000, 304,  3280, 1856, 0, 0, 3280, 1856}, /* video */
	{6560, 4928, 000, 000, 6560, 4928, 3280, 2464,0000, 304,  3280, 1856, 0, 0, 3280, 1856}, /* hs_video */
	{6560, 4928, 000, 000, 6560, 4928, 3280, 2464,0000, 0000, 3280, 2464, 0, 0, 3280, 2464}, /* slim_video */
	{6560, 4928, 000, 000, 6560, 4928, 6560, 4928,0000, 0000, 6560, 4928, 0, 0, 6560, 4928}, /* remosaic */
};
#else
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[8] = {
    // Preview
    {8192, 6144,    0,    0, 8192, 6144, 4096, 3072,    48,   36, 4000, 3000,    0,    0, 4000, 3000},
    // Capture
    {8192, 6144,    0,    0, 8192, 6144, 4096, 3072,    48,   36, 4000, 3000,    0,    0, 4000, 3000},
    // normal video
    {8192, 6144,    0,    0, 8192, 6144, 4096, 3072,    0,    384, 4096, 2304,    0,    0, 4096, 2304},
    // hs_video
    {8192, 6144,    0,    0, 8192, 6144, 4096, 3072,    0,    384, 4096, 2304,    0,    0, 4096, 2304},
    //slim_video
    {8192, 6144,    0,  768, 8192, 4608, 2048, 1152,    0,    0, 2048, 1152,    0,    0, 2048, 1152},
	// custom1
	{8192, 6144,    0,    0, 8192, 6144, 8192, 6144,    0,    0, 8192, 6144,    0,    0, 8192, 6144},
    //custom2
    {8192, 6144,    0,    0, 8192, 6144, 2048, 1536,    0,    192, 2048, 1152,    0,    0, 2048, 1152},
    //custom3
	{8192, 6144,    2048, 1536 , 4096, 3072, 4096, 3072, 0,    0, 4096, 3072,    48,    36, 4000, 3000},
};
#endif
#define IMX882_EEPROM_READ_ID 0xA0
static kal_uint16 read_cmos_eeprom_8(struct subdrv_ctx *ctx, kal_uint16 addr)
{
	kal_uint16 get_byte = 0;

	adaptor_i2c_rd_u8(ctx->i2c_client, IMX882_EEPROM_READ_ID >> 1, addr, (u8 *)&get_byte);
	return get_byte;
}
#define QSC_SIZE 3072
#define QSC_EEPROM_ADDR 0x0792
static kal_uint16 imx882_QSC_setting[QSC_SIZE * 2];
static void read_sensor_Cali(struct subdrv_ctx *ctx)
{
	kal_uint16 idx = 0;
	kal_uint16 addr_qsc = QSC_EEPROM_ADDR;

	for (idx = 0; idx < QSC_SIZE; idx++) {
		addr_qsc = QSC_EEPROM_ADDR + idx;
		imx882_QSC_setting[2*idx] = 0xC000 + idx;
		imx882_QSC_setting[2*idx + 1] = read_cmos_eeprom_8(ctx, addr_qsc);
	}
	ctx->is_read_preload_eeprom = 1;
}

static void write_sensor_QSC(struct subdrv_ctx *ctx)
{
    if (imx882_QSC_setting[0] != 0x00)
	{
		imx882txd_table_write_cmos_sensor(ctx, imx882_QSC_setting,
			sizeof(imx882_QSC_setting)/sizeof(kal_uint16));
		write_cmos_sensor_8(ctx, 0x3206, 0x01);
	} else{
		write_cmos_sensor_8(ctx, 0x3206, 0x00);
	}
}


static struct mtk_sensor_saturation_info imgsensor_saturation_info = {
	.gain_ratio = 1000,
	.OB_pedestal = 64,
	.saturation_level = 1023,
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info = {
    .i4OffsetX = 0,
    .i4OffsetY = 0,
    .i4PitchX = 0,
    .i4PitchY = 0,
    .i4PairNum = 0,
    .i4SubBlkW = 0,
    .i4SubBlkH = 0,
    .i4PosL = {{0, 0} },
    .i4PosR = {{0, 0} },
    .i4BlockNumX = 0,
    .i4BlockNumY = 0,
    .i4LeFirst = 0,
    .i4VCPackNum = 1,
    .i4FullRawW = 4096,
    .i4FullRawH = 3072,
    .PDAF_Support = PDAF_SUPPORT_CAMSV_QPD,
    .i4ModeIndex = 3,
    .sPDMapInfo[0] = {
        .i4PDPattern = 1,
        .i4BinFacX = 2,
        .i4BinFacY = 4,
        .i4PDRepetition = 0,
        .i4PDOrder = {0},
    },
    .i4Crop = {
        {48, 36}, {48, 36}, {48, 384}, {48, 384}, {0, 192},//3072-2304/2=384+576=4608
        {0, 0}, {0, 384}, {48, 36}, {0, 384}, {1024, 768}, {0, 848},
        {0, 0}, {0, 384}, {1024, 768}, {0, 384}
    },
    .iMirrorFlip = 0,
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pdseamless_info = {
    .i4OffsetX = 0,
    .i4OffsetY = 0,
    .i4PitchX = 0,
    .i4PitchY = 0,
    .i4PairNum = 0,
    .i4SubBlkW = 0,
    .i4SubBlkH = 0,
    .i4PosL = {{0, 0} },
    .i4PosR = {{0, 0} },
    .i4BlockNumX = 0,
    .i4BlockNumY = 0,
    .i4LeFirst = 0,
    .i4VCPackNum = 1,
    .i4FullRawW = 8192,
    .i4FullRawH = 6144,
    .PDAF_Support = PDAF_SUPPORT_CAMSV_QPD,
    .i4ModeIndex = 3,
    .sPDMapInfo[0] = {
        .i4PDPattern = 1,
        .i4BinFacX = 4,
        .i4BinFacY = 2,
        .i4PDRepetition = 0,
        .i4PDOrder = {0},
    },//960+
    .i4Crop = {
        {48, 36}, {48, 36}, {48, 384}, {48, 384}, {0, 192},//3072-2304/2=384+576=4608
        {0, 0}, {0, 384}, {2096, 1572}, {0, 384}, {1024, 768}, {0, 848},
        {0, 0}, {0, 384}, {1024, 768}, {0, 384}
    },
    .iMirrorFlip = 0,
};
static struct SET_PD_BLOCK_INFO_T imgsensor_pdslim_info = {
    .i4OffsetX = 0,
    .i4OffsetY = 0,
    .i4PitchX = 0,
    .i4PitchY = 0,
    .i4PairNum = 0,
    .i4SubBlkW = 0,
    .i4SubBlkH = 0,
    .i4PosL = {{0, 0} },
    .i4PosR = {{0, 0} },
    .i4BlockNumX = 0,
    .i4BlockNumY = 0,
    .i4LeFirst = 0,
    .i4VCPackNum = 1,
    .i4FullRawW = 2048,
    .i4FullRawH = 1152,
    .PDAF_Support = PDAF_SUPPORT_CAMSV_QPD,
    .i4ModeIndex = 3,
    .sPDMapInfo[0] = {
        .i4PDPattern = 1,
        .i4BinFacX = 2,
        .i4BinFacY = 4,
        .i4PDRepetition = 0,
        .i4PDOrder = {0},
    },//960+
    .i4Crop = {
        {48, 36}, {48, 36}, {48, 384}, {48, 384}, {0, 0},//3072-2304/2=384+576=4608
        {0, 0}, {0, 384}, {2096, 1572}, {0, 384}, {1024, 768}, {0, 848},
        {0, 0}, {0, 384}, {1024, 768}, {0, 384}
    },
    .iMirrorFlip = 0,
};
#if MULTI_WRITE
//#define IMX882_EEPROM_READ_ID  0xA4
//static void imx882_apply_LRC(struct subdrv_ctx *ctx)
//{
/*	u8 imx882txd_LRC_data[352] = {0};
	LOG_DEBUG("E");
	read_imx882txd_LRC(ctx, imx882txd_LRC_data);
	subdrv_i2c_wr_p8(ctx, 0x7520, imx882txd_LRC_data, 70);
	subdrv_i2c_wr_p8(ctx, 0x7568, imx882txd_LRC_data + 70, 70);
	LOG_DEBUG("readback LRC, L1(%d) L70(%d) R1(%d) R70(%d)\n",
		read_cmos_sensor_8(ctx, 0x7520), read_cmos_sensor_8(ctx, 0x7565),
		read_cmos_sensor_8(ctx, 0x7568), read_cmos_sensor_8(ctx, 0x75AD));
*/
//}

static void set_dummy(struct subdrv_ctx *ctx)
{
	LOG_DEBUG("dummyline = %d, dummypixels = %d\n",
		ctx->dummy_line, ctx->dummy_pixel);
	/* return;*/ /* for test */
	write_cmos_sensor_8(ctx, 0x0104, 0x01);
	write_cmos_sensor_8(ctx, 0x0340, ctx->frame_length >> 8);
	write_cmos_sensor_8(ctx, 0x0341, ctx->frame_length & 0xFF);
	write_cmos_sensor_8(ctx, 0x0342, ctx->line_length >> 8);
	write_cmos_sensor_8(ctx, 0x0343, ctx->line_length & 0xFF);
	write_cmos_sensor_8(ctx, 0x0104, 0x00);
}	/*	set_dummy  */
static void set_mirror_flip(struct subdrv_ctx *ctx, kal_uint8 image_mirror)
{
	kal_uint8 itemp;
	LOG_DEBUG("image_mirror = %d\n", image_mirror);
	itemp = read_cmos_sensor_8(ctx, 0x0101);
	itemp &= ~0x03;
	switch (image_mirror) {
	case IMAGE_NORMAL:
	write_cmos_sensor_8(ctx, 0x0101, itemp);
	break;
	case IMAGE_V_MIRROR:
	write_cmos_sensor_8(ctx, 0x0101, itemp | 0x02);
	break;
	case IMAGE_H_MIRROR:
	write_cmos_sensor_8(ctx, 0x0101, itemp | 0x01);
	break;
	case IMAGE_HV_MIRROR:
	write_cmos_sensor_8(ctx, 0x0101, itemp | 0x03);
	break;
	}
}
static void set_max_framerate(struct subdrv_ctx *ctx, UINT16 framerate, kal_bool min_framelength_en)
{
	/*kal_int16 dummy_line;*/
	kal_uint32 frame_length = ctx->frame_length;
	LOG_DEBUG("framerate = %d, min framelength should enable %d\n", framerate,
		min_framelength_en);
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
}	/*	set_max_framerate  */
#define MAX_CIT_LSHIFT 7
static void write_shutter(struct subdrv_ctx *ctx, kal_uint32 shutter)
{
	kal_uint16 realtime_fps = 0;
	kal_uint16 l_shift = 1;
	//if (shutter > ctx->min_frame_length - imgsensor_info.margin)
	//	ctx->frame_length = shutter + imgsensor_info.margin;
	//else
	ctx->frame_length = ctx->min_frame_length;
	if (ctx->frame_length > imgsensor_info.max_frame_length)
		ctx->frame_length = imgsensor_info.max_frame_length;
	if (shutter < imgsensor_info.min_shutter)
		shutter = imgsensor_info.min_shutter;
	if (ctx->autoflicker_en) {
		realtime_fps = ctx->pclk / ctx->line_length * 10
				/ ctx->frame_length;
		LOG_DEBUG("autoflicker enable, realtime_fps = %d\n",
			realtime_fps);
		if (realtime_fps >= 593 && realtime_fps <= 607)
	          set_max_framerate(ctx, 592, 0);
        else if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(ctx, 296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(ctx, 146, 0);
	}
	/* long expsoure */
	if (shutter >
		(imgsensor_info.max_frame_length - imgsensor_info.margin)) {
		for (l_shift = 1; l_shift < MAX_CIT_LSHIFT; l_shift++) {
			if ((shutter >> l_shift)
		    < (imgsensor_info.max_frame_length - imgsensor_info.margin))
				break;
		}
		if (l_shift > MAX_CIT_LSHIFT) {
			LOG_DEBUG(
			    "Unable to set such a long exposure %d, set to max\n",
			    shutter);
			l_shift = MAX_CIT_LSHIFT;
		}
		shutter = shutter >> l_shift;
		//ctx->frame_length = shutter + imgsensor_info.margin;
		LOG_DEBUG("enter long exposure mode, time is %d", l_shift);
		write_cmos_sensor_8(ctx, 0x3160,
			read_cmos_sensor(ctx, 0x3160) | (l_shift & 0x7));
		/* Frame exposure mode customization for LE*/
		ctx->ae_frm_mode.frame_mode_1 = IMGSENSOR_AE_MODE_SE;
		ctx->ae_frm_mode.frame_mode_2 = IMGSENSOR_AE_MODE_SE;
		ctx->current_ae_effective_frame = 2;
	} else {
		write_cmos_sensor_8(ctx, 0x0104, 0x01);
		write_cmos_sensor_8(ctx, 0x3160,
			read_cmos_sensor(ctx, 0x3160) & 0xf8);
		write_cmos_sensor_8(ctx, 0x0340, ctx->frame_length >> 8);
		write_cmos_sensor_8(ctx, 0x0341, ctx->frame_length & 0xFF);
		write_cmos_sensor_8(ctx, 0x0104, 0x00);
		ctx->current_ae_effective_frame = 2;
		LOG_DEBUG("set frame_length\n");
	}
	/* Update Shutter */
	write_cmos_sensor_8(ctx, 0x0104, 0x01);
	write_cmos_sensor_8(ctx, 0x0350, 0x01); /* Enable auto extend */
	write_cmos_sensor_8(ctx, 0x0202, (shutter >> 8) & 0xFF);
	write_cmos_sensor_8(ctx, 0x0203, shutter  & 0xFF);
	if (!ctx->ae_ctrl_gph_en)
		write_cmos_sensor_8(ctx, 0x0104, 0x00);
	LOG_DEBUG("shutter =%d, framelength =%d\n",
		shutter, ctx->frame_length);
}	/*	write_shutter  */
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
static void set_shutter(struct subdrv_ctx *ctx, kal_uint32 shutter)
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
	write_cmos_sensor_8(ctx, 0x0104, 0x01);
	write_cmos_sensor_8(ctx, 0x0340, ctx->frame_length >> 8);
	write_cmos_sensor_8(ctx, 0x0341, ctx->frame_length & 0xFF);
	write_cmos_sensor_8(ctx, 0x0104, 0x00);
	LOG_DEBUG("Framelength: set=%d/input=%d/min=%d, auto_extend=%d\n",
		ctx->frame_length, frame_length, ctx->min_frame_length,
		read_cmos_sensor_8(ctx, 0x0350));
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
{	kal_uint16 realtime_fps = 0;
	kal_int32 dummy_line = 0;
	ctx->shutter = shutter;
	/*0x3500, 0x3501, 0x3502 will increase VBLANK to
	 *get exposure larger than frame exposure
	 *AE doesn't update sensor gain at capture mode,
	 *thus extra exposure lines must be updated here.
	 */
	/* OV Recommend Solution */
	/*if shutter bigger than frame_length,
	 *should extend frame length first
	 */
	/* Change frame time */
	dummy_line = frame_length - ctx->frame_length;
	ctx->frame_length = ctx->frame_length + dummy_line;
	ctx->min_frame_length = ctx->frame_length;
	if (shutter > ctx->min_frame_length - imgsensor_info.margin)
		ctx->frame_length = shutter + imgsensor_info.margin;
	else
		ctx->frame_length = ctx->min_frame_length;
	if (ctx->frame_length > imgsensor_info.max_frame_length)
		ctx->frame_length = imgsensor_info.max_frame_length;
	shutter = (shutter < imgsensor_info.min_shutter)
			? imgsensor_info.min_shutter : shutter;
	shutter =
	(shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin))
		? (imgsensor_info.max_frame_length - imgsensor_info.margin)
		: shutter;
	if (ctx->autoflicker_en) {
		realtime_fps = ctx->pclk / ctx->line_length * 10 /
				ctx->frame_length;
		if (realtime_fps >= 593 && realtime_fps <= 607)
	          set_max_framerate(ctx, 592, 0);
        else if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(ctx, 296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(ctx, 146, 0);
		else {
			/* Extend frame length */
			write_cmos_sensor_8(ctx, 0x0104, 0x01);
			write_cmos_sensor_8(ctx, 0x0340,
					ctx->frame_length >> 8);
			write_cmos_sensor_8(ctx, 0x0341,
					ctx->frame_length & 0xFF);
			write_cmos_sensor_8(ctx, 0x0104, 0x00);
		}
	} else {
		/* Extend frame length */
		write_cmos_sensor_8(ctx, 0x0104, 0x01);
		write_cmos_sensor_8(ctx, 0x0340, ctx->frame_length >> 8);
		write_cmos_sensor_8(ctx, 0x0341, ctx->frame_length & 0xFF);
		write_cmos_sensor_8(ctx, 0x0104, 0x00);
	}
	/* Update Shutter */
	write_cmos_sensor_8(ctx, 0x0104, 0x01);
	if (auto_extend_en)
		write_cmos_sensor_8(ctx, 0x0350, 0x01); /* Enable auto extend */
	else
		write_cmos_sensor_8(ctx, 0x0350, 0x00); /* Disable auto extend */
	write_cmos_sensor_8(ctx, 0x0202, (shutter >> 8) & 0xFF);
	write_cmos_sensor_8(ctx, 0x0203, shutter  & 0xFF);
	write_cmos_sensor_8(ctx, 0x0104, 0x00);
	LOG_DEBUG(
		"Exit! shutter =%d, framelength =%d/%d, dummy_line=%d, auto_extend=%d\n",
		shutter, ctx->frame_length, frame_length,
		dummy_line, read_cmos_sensor_8(ctx, 0x0350));
}	/* set_shutter_frame_length */

static void set_multi_shutter_frame_length(struct subdrv_ctx *ctx,
				kal_uint32 *shutters, kal_uint16 shutter_cnt,
				kal_uint16 frame_length)
{
	kal_uint16 realtime_fps = 0;
	kal_int32 dummy_line = 0;

	if (shutter_cnt == 1) {
		ctx->shutter = shutters[0];

		/* Change frame time */
		if (frame_length > 1)
			dummy_line = frame_length - ctx->frame_length;
		ctx->frame_length = ctx->frame_length + dummy_line;

		/*  */
		if (shutters[0] > ctx->frame_length - imgsensor_info.margin)
			ctx->frame_length = shutters[0] + imgsensor_info.margin;

		if (ctx->frame_length > imgsensor_info.max_frame_length)
			ctx->frame_length = imgsensor_info.max_frame_length;

		shutters[0] = (shutters[0] < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutters[0];
		shutters[0] = (shutters[0] > (imgsensor_info.max_frame_length - imgsensor_info.margin))
			? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutters[0];

		if (ctx->autoflicker_en) {
			realtime_fps = ctx->pclk / ctx->line_length * 10 / ctx->frame_length;
			if (realtime_fps >= 593 && realtime_fps <= 607)
				set_max_framerate(ctx, 592, 0);
			else if (realtime_fps >= 297 && realtime_fps <= 305)
				set_max_framerate(ctx, 296, 0);
			else if (realtime_fps >= 147 && realtime_fps <= 150)
				set_max_framerate(ctx, 146, 0);
		}

		/* Update Shutter */
		write_cmos_sensor_8(ctx, 0x0340, ctx->frame_length >> 8);
		write_cmos_sensor_8(ctx, 0x0341, ctx->frame_length & 0xFF);
		write_cmos_sensor_8(ctx, 0x0202, (shutters[0] >> 8) & 0xFF);
		write_cmos_sensor_8(ctx, 0x0203, shutters[0] & 0xFF);
		ctx->frame_length_rg = ctx->frame_length;
		LOG_DEBUG("shutters[0] =%d, framelength =%d\n",
				shutters[0], ctx->frame_length);
	}
}

static kal_uint32 gain2reg(struct subdrv_ctx *ctx, const kal_uint32 gain)
{
	kal_uint32 reg_gain = 0x0;
	reg_gain = 16384 - (16384*BASEGAIN)/gain;
	return (kal_uint32) reg_gain;
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
	kal_uint32 reg_gain;
	if (gain < imgsensor_info.min_gain || gain > imgsensor_info.max_gain) {
		LOG_DEBUG("Error gain setting");
		if (gain < imgsensor_info.min_gain)
			gain = imgsensor_info.min_gain;
		else
			gain = imgsensor_info.max_gain;
	}
	reg_gain = gain2reg(ctx, gain);
	ctx->gain = reg_gain;
	LOG_DEBUG("gain = %d, reg_gain = 0x%x\n ", gain, reg_gain);
	if (!ctx->ae_ctrl_gph_en)
		write_cmos_sensor_8(ctx, 0x0104, 0x01);
	write_cmos_sensor_8(ctx, 0x0204, (reg_gain>>8) & 0xFF);
	write_cmos_sensor_8(ctx, 0x0205, reg_gain & 0xFF);
	write_cmos_sensor_8(ctx, 0x0104, 0x00);
	return gain;
} /* set_gain */
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
	LOG_DEBUG("streaming_enable(0=Sw Standby,1=streaming): %d\n",
		enable);
	if (enable)
		write_cmos_sensor_8(ctx, 0x0100, 0x01);
	else
		write_cmos_sensor_8(ctx, 0x0100, 0x00);
	return ERROR_NONE;
}
static kal_uint16 imx882txd_init_setting[] = {
    //Stand-by OFF Sequence
    //Power ON
    //Input EXTCLK
    //XCLR OFF
    //External Clock Setting
    //Address   value
        0x0136, 0x18,
        0x0137, 0x00,
	//Global Setting 0
0x961C,0x04,
0xF800,0x18,
0xF801,0x18,
0xF802,0xC5,
0xF803,0x26,
0xF804,0x55,
0xF805,0xC4,
0xF806,0x3F,
0xF807,0x00,
0xF808,0x17,
0xF809,0xFA,
0xF80A,0x84,
0xF80B,0xBE,
0xF80C,0x55,
0xF80D,0xC8,
0xF80E,0x3F,
0xF80F,0x00,
0xF810,0x40,
0xF811,0x80,
0xF812,0x3A,
0xF813,0xC4,
0xFA00,0x23,
0xFA01,0x23,
0xFA02,0xC5,
0xFA03,0x2B,
0xFA04,0x55,
0xFA05,0xC4,
0xFA06,0x3F,
0xFA07,0x00,
0xFA08,0x17,
0xFA09,0xFA,
0xFA0A,0x84,
0xFA0B,0xBE,
0xFA0C,0x55,
0xFA0D,0xC8,
0xFA0E,0x3F,
0xFA0F,0x00,
0xFA10,0x40,
0xFA11,0x80,
0xFA12,0x3A,
0xFA13,0xC4,
0xFC00,0x4F,
0xFC01,0x06,
0xFC02,0xAC,
0xFC03,0x27,
0xFC04,0xAC,
0xFC05,0x1D,
0xFC06,0xAC,
0xFC07,0x14,
0xFC08,0xFA,
0xFC09,0xE8,
0xFC0A,0xEB,
0xFC0B,0x2E,
0xFC0C,0xD0,
0xFC0D,0x20,
0xFC0E,0x40,
0xFC0F,0x4F,
0xFC10,0x1E,
0xFC11,0x05,
0xFC12,0xD4,
0xFC13,0x18,
0xFC14,0x08,
0xFC15,0x00,
0xFC16,0xF6,
0xFC17,0x40,
0xFC18,0x06,
0xFC19,0xB7,
0xFC1A,0xBE,
0xFC1B,0x06,
0xFC1C,0xF0,
0xFC1D,0x40,
0xFC1E,0x06,
0xFC1F,0xB8,
0xFC20,0x29,
0xFC21,0x25,
0xFC22,0x90,
0xFC23,0x2C,
0xFC24,0x80,
0xFC25,0x22,
0xFC26,0xF6,
0xFC27,0x41,
0xFC28,0x05,
0xFC29,0x1C,
0xFC2A,0xB8,
0xFC2B,0x8B,
0xFC2C,0xD0,
0xFC2D,0x18,
0xFC2E,0x27,
0xFC2F,0x6C,
0xFC30,0xF0,
0xFC31,0x02,
0xFC32,0x02,
0xFC33,0x74,
0xFC34,0xF6,
0xFC35,0x40,
0xFC36,0x07,
0xFC37,0x10,
0xFC38,0x0F,
0xFC39,0x20,
0xFC3A,0xFA,
0xFC3B,0xE8,
0xFC3C,0x03,
0xFC3D,0x8A,
0xFC3E,0x80,
0xFC3F,0x04,
0xFC40,0x5F,
0xFC41,0xF0,
0xFC42,0x29,
0xFC43,0x25,
0xFC44,0x90,
0xFC45,0x0A,
0xFC46,0x50,
0xFC47,0x31,
0xFC48,0xFA,
0xFC49,0xE8,
0xFC4A,0x03,
0xFC4B,0x7C,
0xFC4C,0x80,
0xFC4D,0x04,
0xFC4E,0x09,
0xFC4F,0x50,
0xFC50,0xF4,
0xFC51,0x40,
0xFC52,0x06,
0xFC53,0xBC,
0xFC54,0x50,
0xFC55,0xA1,
0xFC56,0xFA,
0xFC57,0xE8,
0xFC58,0x03,
0xFC59,0x6E,
0xFC5A,0xFA,
0xFC5B,0xE8,
0xFC5C,0x02,
0xFC5D,0x9E,
0xFC5E,0xA8,
0xFC5F,0x14,
0xFC60,0xA8,
0xFC61,0x1D,
0xFC62,0xA8,
0xFC63,0x27,
0xFC64,0xA0,
0xFC65,0x09,
0xFC66,0x00,
0xFC67,0x00,
0x4331,0x01,
    //PHY_VIF Setting
    //Address   value
        0x3304, 0x00,
    //Register version
    //Address   value
        0x33F0, 0x04	,
        0x33F1, 0x06,
    //Signaling mode setting
    //Address   value
        0x0111, 0x03,
    //ROI Setting 2
    //Address   value
        0x3855, 0x01,
    //Global Setting
    //Address   value
0x0D06,0x82,
0x0D07,0x02,
0x0D12,0x78,
0x0D13,0x01,
0x0D0D,0x00,
0x0D08,0x01,
0x1012,0x00,
0x2EDF,0x07,
0x3953,0x01,
0x3954,0x01,
0x3955,0x01,
0x3B30,0x01,
0x3B32,0x12,
0x3B33,0x04,
0x3B34,0x0A,
0x3B36,0xB2,
0x3B40,0x00,
0x3B42,0x0A,
0x3B43,0x04,
0x3B45,0x0D,
0x3B46,0x40,
0x5A1D,0x52,
0x5A27,0x19,
0x5A37,0xC8,
0x5A39,0x25,
0x5A3D,0x0E,
0x5A41,0x19,
0x5A47,0x62,
0x5A51,0x52,
0x5A5B,0x19,
0x5A69,0xC8,
0x5A6B,0x25,
0x5A6F,0x11,
0x5A73,0x19,
0x5A79,0x8A,
0x5A83,0x52,
0x5A8D,0x19,
0x5A9B,0xC8,
0x5A9D,0x25,
0x5AA1,0x52,
0x5AA5,0x19,
0x5AAB,0x93,
0x5AB5,0x52,
0x5ABF,0x19,
0x5ACD,0xC8,
0x5ACF,0x25,
0x5AD3,0x11,
0x5AD7,0x19,
0x5ADD,0x4B,
0x5AE7,0x52,
0x5AF1,0x19,
0x5AF5,0x2D,
0x5AFF,0xC8,
0x5B01,0x25,
0x5B05,0x0E,
0x5B09,0x19,
0x5B0D,0x9E,
0x5B0F,0x34,
0x5B21,0x19,
0x5B27,0x2D,
0x5B33,0x19,
0x5B39,0x9E,
0x5B3F,0x52,
0x5B49,0x19,
0x5B4D,0x87,
0x5B57,0xC8,
0x5B59,0x25,
0x5B5D,0x0E,
0x5B61,0x19,
0x5B65,0xDA,
0x5B6D,0x52,
0x5B79,0x19,
0x5B7F,0x2D,
0x5B8B,0x19,
0x5B91,0x9E,
0x5B97,0x52,
0x5BA1,0x19,
0x5BA5,0x2D,
0x5BAF,0xC8,
0x5BB1,0x25,
0x5BB5,0x11,
0x5BB9,0x19,
0x5BBD,0x9E,
0x5BBF,0x5C,
0x5BD1,0x19,
0x5BD7,0x2D,
0x5BE3,0x19,
0x5BE9,0x9E,
0x5BEF,0x52,
0x5BF9,0x19,
0x5BFD,0x87,
0x5C07,0xC8,
0x5C09,0x25,
0x5C0D,0x11,
0x5C11,0x19,
0x5C15,0xDA,
0x5C29,0x19,
0x5C2F,0x2D,
0x5C3B,0x19,
0x5C41,0x9E,
0x5C47,0x52,
0x5C51,0x19,
0x5C57,0x41,
0x5C5F,0xC8,
0x5C61,0x25,
0x5C65,0x11,
0x5C69,0x19,
0x5C6E,0x01,
0x5C6F,0x13,
0x5C85,0xC8,
0x5C87,0x25,
0x5C8B,0x1B,
0x5C8F,0x19,
0x5C95,0x41,
0x5C9B,0x52,
0x5CA5,0x19,
0x5CAA,0x03,
0x5CAB,0xDE,
0x5CB3,0xC8,
0x5CB5,0x25,
0x5CB9,0x11,
0x5CBD,0x19,
0x5CC3,0xB8,
0x5CD9,0xC8,
0x5CDB,0x25,
0x5CDF,0x1B,
0x5CE3,0x19,
0x5CE8,0x03,
0x5CE9,0xDE,
0x5CEF,0x52,
0x5CF9,0x19,
0x5CFF,0x79,
0x5D07,0xC8,
0x5D09,0x25,
0x5D0C,0x00,
0x5D0D,0x52,
0x5D11,0x19,
0x5D16,0x01,
0x5D17,0x28,
0x5D2D,0xC8,
0x5D2F,0x25,
0x5D33,0x53,
0x5D37,0x19,
0x5D3D,0x79,
0x5D43,0x52,
0x5D4D,0x19,
0x5D51,0x2D,
0x5D53,0x41,
0x5D55,0x6B,
0x5D5B,0xC8,
0x5D5D,0x25,
0x5D61,0x11,
0x5D65,0x19,
0x5D69,0x9E,
0x5D6B,0xD0,
0x5D7D,0x19,
0x5D83,0x2D,
0x5D95,0xC8,
0x5D97,0x25,
0x5D9B,0x1B,
0x5D9F,0x19,
0x5DA5,0x41,
0x5DB1,0x19,
0x5DB7,0x6B,
0x5DC3,0x19,
0x5DC9,0x9E,
0x5DCF,0x52,
0x5DD9,0x19,
0x5DDD,0x87,
0x5DDF,0x41,
0x5DE1,0x41,
0x5DE7,0xC8,
0x5DE9,0x25,
0x5DED,0x11,
0x5DF1,0x19,
0x5DF5,0xDA,
0x5DF7,0x4A,
0x5E09,0x19,
0x5E0F,0x2D,
0x5E21,0xC8,
0x5E23,0x25,
0x5E27,0x1B,
0x5E2B,0x19,
0x5E31,0x41,
0x5E3D,0x19,
0x5E43,0x6B,
0x5E4F,0x19,
0x5E55,0x9E,
0x5E5B,0x52,
0x5E65,0x19,
0x5E69,0x2D,
0x5E6B,0x41,
0x5E73,0xC8,
0x5E75,0x25,
0x5E79,0x11,
0x5E7D,0x19,
0x5E81,0x9E,
0x5E83,0xFB,
0x5E95,0x19,
0x5E9B,0x2D,
0x5EAD,0xC8,
0x5EAF,0x25,
0x5EB3,0x1B,
0x5EB7,0x19,
0x5EBD,0x41,
0x5EC9,0x19,
0x5ECF,0x9E,
0x5ED5,0x52,
0x5EDF,0x19,
0x5EE3,0x2D,
0x5EE5,0x5A,
0x5EE7,0x41,
0x5EEF,0xC8,
0x5EF1,0x25,
0x5EF5,0x11,
0x5EF9,0x19,
0x5EFD,0xDA,
0x5EFF,0xCB,
0x5F11,0x19,
0x5F17,0x2D,
0x5F23,0x19,
0x5F29,0x2D,
0x5F3B,0xC8,
0x5F3D,0x25,
0x5F41,0x1B,
0x5F45,0x19,
0x5F4B,0x41,
0x5F57,0x19,
0x5F5D,0x9E,
0x5F63,0x52,
0x5F6D,0x19,
0x5F7F,0x55,
0x5F83,0x0E,
0x5F87,0x19,
0x5F93,0x12,
0x6C06,0xFF,
0x6C07,0xFF,
0x6C0E,0x00,
0x6C0F,0x02,
0x6C11,0xD9,
0x6C12,0xFF,
0x6C13,0xFF,
0x6C1A,0x00,
0x6C1B,0x02,
0x6C1D,0xD9,
0x6C1E,0xFF,
0x6C1F,0xFF,
0x6C26,0x00,
0x6C27,0x02,
0x6C29,0xD9,
0x6C2A,0xFF,
0x6C2B,0xFF,
0x6C32,0x00,
0x6C33,0x02,
0x6C35,0xD9,
0x6C36,0xFF,
0x6C37,0xFF,
0x6C3E,0x00,
0x6C3F,0x02,
0x6C41,0xD9,
0x6C42,0xFF,
0x6C43,0xFF,
0x6C4A,0x00,
0x6C4B,0x02,
0x6C4D,0xD9,
0x6C4E,0xFF,
0x6C4F,0xFF,
0x6C5A,0xFF,
0x6C5B,0xFF,
0x6C66,0xFF,
0x6C67,0xFF,
0x6C72,0xFF,
0x6C73,0xFF,
0x6C7E,0xFF,
0x6C7F,0xFF,
0x6CAA,0x00,
0x6CAB,0x02,
0x6CAD,0xD9,
0x6CAE,0x00,
0x6CAF,0x02,
0x6CB1,0xD9,
0x6CB2,0x00,
0x6CB3,0x02,
0x6CB5,0xD9,
0x6CB6,0x00,
0x6CB7,0x02,
0x6CB9,0xD9,
0x6CBA,0x00,
0x6CBB,0x02,
0x6CBD,0xD9,
0x6E3F,0xDE,
0x6E47,0xDE,
0x6E4F,0xDE,
0x6E57,0xDE,
0x6E5F,0xDE,
0x6E67,0xDE,
0x6EB3,0xDE,
0x6EB7,0xDE,
0x6EBB,0xDE,
0x6EBF,0xDE,
0x6EC3,0xDE,
0x7476,0x00,
0x7477,0x00,
0x7478,0x00,
0x7509,0x00,
0x750B,0x00,
0x7516,0x01,
0x7524,0x0C,
0x7528,0x02,
0x7530,0x03,
0x7616,0x0C,
0x7619,0x0C,
0x761C,0x0C,
0x761D,0x0C,
0x761E,0x0C,
0x761F,0x0C,
0x7620,0x00,
0x7623,0x00,
0x7624,0x00,
0x7625,0x00,
0x7626,0x00,
0x7627,0x00,
0x7628,0x00,
0x7629,0x00,
0x762A,0x00,
0x762B,0x00,
0x762C,0x00,
0x762D,0x00,
0x762E,0x00,
0x762F,0x00,
0x7631,0x07,
0x7632,0x07,
0x7634,0x07,
0x7635,0x07,
0x76FC,0x44,
0x76FD,0x2A,
0x76FE,0x2C,
0x76FF,0x26,
0x7700,0x43,
0x7701,0x2A,
0x7702,0x2A,
0x7703,0x26,
0x7704,0x2C,
0x7705,0x2A,
0x7706,0x2A,
0x7707,0x65,
0x7708,0x44,
0x7709,0x48,
0x770A,0x40,
0x770B,0x64,
0x770C,0x44,
0x770D,0x44,
0x770E,0x40,
0x770F,0x48,
0x7710,0x44,
0x7711,0x44,
0x7712,0x45,
0x7713,0x43,
0x7714,0x47,
0x7715,0x40,
0x7716,0x42,
0x7717,0x43,
0x7718,0x43,
0x7719,0x40,
0x771A,0x47,
0x771B,0x43,
0x771C,0x43,
0x771D,0x5D,
0x771E,0x43,
0x771F,0x46,
0x7720,0x40,
0x7721,0x62,
0x7722,0x43,
0x7723,0x43,
0x7724,0x40,
0x7725,0x46,
0x7726,0x43,
0x7727,0x43,
0x7728,0x41,
0x7729,0x43,
0x772A,0x3F,
0x772B,0x41,
0x772C,0x41,
0x772D,0x3F,
0x772E,0x43,
0x772F,0x41,
0x7730,0x41,
0x7731,0x40,
0x7732,0x42,
0x7733,0x3F,
0x7734,0x40,
0x7735,0x40,
0x7736,0x3F,
0x7737,0x42,
0x7738,0x40,
0x7739,0x40,
0x773A,0x40,
0x773B,0x15,
0x773C,0x16,
0x773D,0x09,
0x773E,0x3B,
0x773F,0x15,
0x7740,0x15,
0x7741,0x09,
0x7742,0x16,
0x7743,0x15,
0x7744,0x15,
0x7745,0x63,
0x7746,0x2F,
0x7747,0x2F,
0x7748,0x19,
0x7749,0x5D,
0x774A,0x2F,
0x774B,0x2F,
0x774C,0x19,
0x774D,0x2F,
0x774E,0x2F,
0x774F,0x2F,
0x7750,0x45,
0x7751,0x2F,
0x7752,0x2F,
0x7753,0x24,
0x7754,0x43,
0x7755,0x2F,
0x7756,0x2F,
0x7757,0x24,
0x7758,0x2F,
0x7759,0x2F,
0x775A,0x2F,
0x775B,0x57,
0x775C,0x39,
0x775D,0x36,
0x775E,0x32,
0x775F,0x64,
0x7760,0x39,
0x7761,0x39,
0x7762,0x32,
0x7763,0x36,
0x7764,0x39,
0x7765,0x39,
0x7766,0x3B,
0x7767,0x3B,
0x7768,0x3B,
0x7769,0x3B,
0x776A,0x3B,
0x776B,0x3B,
0x776C,0x3B,
0x776D,0x3B,
0x776E,0x3B,
0x776F,0x41,
0x7770,0x40,
0x7771,0x3F,
0x7772,0x41,
0x7773,0x41,
0x7774,0x3F,
0x7775,0x40,
0x7776,0x41,
0x7777,0x41,
0x7778,0x0F,
0x7779,0x0A,
0x777A,0x0A,
0x777B,0x0A,
0x777C,0x0F,
0x777D,0x0A,
0x777E,0x0A,
0x777F,0x0A,
0x7780,0x0A,
0x7781,0x0A,
0x7782,0x0A,
0x7783,0x0F,
0x7784,0x0A,
0x7785,0x0A,
0x7786,0x0A,
0x7787,0x0F,
0x7788,0x0A,
0x7789,0x0A,
0x778A,0x0A,
0x778B,0x0A,
0x778C,0x0A,
0x778D,0x0A,
0x778E,0x14,
0x778F,0x0A,
0x7790,0x0A,
0x7791,0x0A,
0x7792,0x0F,
0x7793,0x0A,
0x7794,0x0A,
0x7795,0x0A,
0x7796,0x0A,
0x7797,0x0A,
0x7798,0x0A,
0x7799,0x14,
0x779A,0x0A,
0x779B,0x0A,
0x779C,0x0A,
0x779D,0x0F,
0x779E,0x0A,
0x779F,0x0A,
0x77A0,0x0A,
0x77A1,0x0A,
0x77A2,0x0A,
0x77A3,0x0A,
0x77A4,0x0A,
0x77A5,0x0A,
0x77A6,0x0A,
0x77A7,0x0A,
0x77A8,0x0A,
0x77A9,0x0A,
0x77AA,0x0A,
0x77AB,0x0A,
0x77AC,0x0A,
0x77AD,0x0B,
0x77AE,0x0A,
0x77AF,0x0A,
0x77B0,0x0B,
0x77B1,0x0B,
0x77B2,0x0A,
0x77B3,0x0A,
0x77B4,0x0B,
0x77B5,0x0B,
0x77F4,0x01,
0x77F5,0x01,
0x77F6,0x01,
0x77F8,0x01,
0x77F9,0x01,
0x77FB,0x01,
0x77FC,0x01,
0x7814,0x2A,
0x7815,0x01,
0x7816,0x01,
0x7818,0x2A,
0x7819,0x01,
0x781A,0x01,
0x781C,0x01,
0x781D,0x01,
0x781E,0x01,
0x781F,0x2A,
0x7820,0x01,
0x7821,0x01,
0x7823,0x2A,
0x7824,0x01,
0x7825,0x01,
0x7827,0x01,
0x7828,0x01,
0x7829,0x01,
0x782A,0x2A,
0x782B,0x0A,
0x782C,0x0A,
0x782E,0x2A,
0x782F,0x0A,
0x7830,0x0A,
0x7832,0x0A,
0x7833,0x0A,
0x7834,0x0A,
0x7835,0x2A,
0x7836,0x19,
0x7837,0x19,
0x7838,0x2A,
0x7839,0x2A,
0x783A,0x19,
0x783B,0x19,
0x783C,0x2A,
0x783D,0x19,
0x783E,0x19,
0x783F,0x19,
0x7840,0x1D,
0x7841,0x1D,
0x7842,0x2A,
0x7843,0x1D,
0x7844,0x1D,
0x7845,0x2A,
0x7846,0x1D,
0x7847,0x1D,
0x7848,0x1D,
0x7849,0x2A,
0x784A,0x2A,
0x784B,0x2A,
0x784C,0x2A,
0x784D,0x2A,
0x784E,0x2A,
0x784F,0x2A,
0x7850,0x2A,
0x7851,0x2A,
0x7853,0x50,
0x7856,0x50,
0x7857,0x50,
0x785A,0x50,
0x785B,0x50,
0x785D,0x50,
0x7860,0x50,
0x7861,0x50,
0x7864,0x50,
0x7865,0x50,
0x7867,0x50,
0x786A,0x50,
0x786B,0x50,
0x7902,0x15,
0x7904,0x13,
0x7905,0x15,
0x7908,0x13,
0x7909,0x08,
0x790B,0x11,
0x790D,0x0C,
0x790E,0x08,
0x790F,0x08,
0x7912,0x0B,
0x7919,0x06,
0x791E,0x06,
0x791F,0x06,
0x7929,0x00,
0x792E,0x00,
0x792F,0x00,
0x7A2A,0x19,
0x7A2B,0xF1,
0x7A2C,0x0F,
0x7A48,0x0F,
0x7A49,0x01,
0x7A4A,0x03,
0x7A51,0x09,
0x7A58,0x0F,
0x7A5D,0x32,
0x7A63,0x32,
0x7A6B,0x3E,
0x7A71,0x3E,
0x7AAD,0x99,
0x7AB3,0x99,
0x7ABB,0xA5,
0x7AC1,0xA5,
0x7AFD,0x66,
0x7B05,0x72,
0x7B26,0x12,
0x7B27,0x12,
0x7B28,0x12,
0x7B29,0x12,
0x7B2A,0x12,
0x7B2B,0x12,
0x7B2C,0x12,
0x7B2D,0x12,
0x7B2E,0x12,
0x7B2F,0x12,
0x7B30,0x12,
0x7B31,0x12,
0x7B32,0x12,
0x7B33,0x12,
0x7B34,0x12,
0x7B35,0x12,
0x7B36,0x12,
0x7B37,0x12,
0x7B38,0x12,
0x7B39,0x12,
0x7B3A,0x12,
0x7B3B,0x12,
0x7B3C,0x12,
0x7B3D,0x12,
0x7B3E,0x12,
0x7B3F,0x12,
0x7B40,0x12,
0x7B41,0x12,
0x7B42,0x12,
0x7B43,0x12,
0x7B44,0x12,
0x7B45,0x12,
0x7B46,0x12,
0x7B47,0x12,
0x7B48,0x12,
0x7B49,0x12,
0x7B4A,0x12,
0x7B4B,0x12,
0x7B4C,0x12,
0x7B4D,0x12,
0x7C73,0x0B,
0x7C74,0x0B,
0x7C75,0x0B,
0x7C76,0x0B,
0x7C77,0x0B,
0x7C78,0x0B,
0x7C79,0x0B,
0x7C7C,0x0B,
0x7C7D,0x0B,
0x7C7E,0x0B,
0x7C7F,0x0B,
0x7C80,0x0B,
0x7C81,0x0B,
0x7C83,0x0B,
0x7C84,0x0B,
0x7C85,0x0B,
0x7C86,0x0B,
0x7C87,0x0B,
0x7C88,0x0B,
0x7CA5,0x01,
0x7CAA,0x01,
0x7CB0,0x01,
0x7CB5,0x01,
0x7CBB,0x01,
0x7CC0,0x01,
0x7CD1,0x01,
0x7CD6,0x01,
0x7CDC,0x01,
0x7CE1,0x01,
0x7CE7,0x01,
0x7CEC,0x01,
0x90B3,0x80,
0x974A,0x09,
0x974B,0x08,
0x9752,0x0E,
0x9753,0xB4,
0x975B,0x0C,
0x9762,0x09,
0x9763,0x08,
0x976A,0x09,
0x976B,0x08,
0x9772,0x0F,
0x9773,0x7C,
0x978A,0x09,
0x978B,0xE8,
0xDDA9,0x4E,
0xDE8D,0x01,
0xE24E,0x00,
0x7854,0x50,
0x7859,0x50,
0x785E,0x50,
0x7863,0x50,
0x7869,0x50,
0x7A5E,0x00,
0x7A5F,0x32,
0x7A66,0x00,
0x7A67,0x32,
0x7A6C,0x00,
0x7A6D,0x3E,
0x7A74,0x00,
0x7A75,0x3E,
0x7AAE,0x00,
0x7AAF,0x99,
0x7AB6,0x00,
0x7AB7,0x99,
0x7ABC,0x00,
0x7ABD,0xA5,
0x7AC4,0x00,
0x7AC5,0xA5,
0x7B00,0x00,
0x7B01,0x66,
0x7B08,0x00,
0x7B09,0x72,
0x7CA6,0x01,
0x7CA9,0x01,
0x7CAC,0x01,
0x7CAD,0x01,
0x7CAE,0x01,
0x7CB1,0x01,
0x7CB4,0x01,
0x7CB7,0x01,
0x7CB8,0x01,
0x7CB9,0x01,
0x7CBC,0x01,
0x7CBF,0x01,
0x7CC2,0x01,
0x7CC3,0x01,
0x7CC4,0x01,
0x7CD2,0x01,
0x7CD5,0x01,
0x7CD8,0x01,
0x7CD9,0x01,
0x7CDA,0x01,
0x7CDD,0x01,
0x7CE0,0x01,
0x7CE3,0x01,
0x7CE4,0x01,
0x7CE5,0x01,
0x7CE8,0x01,
0x7CEB,0x01,
0x7CEE,0x01,
0x7CEF,0x01,
0x7CF0,0x01,
0xAB30,0x00,
0xAB31,0x3C,
0xAB32,0x00,
0xAB33,0x3C,
0xAB34,0x00,
0xAB35,0x3C,
0xAE24,0x01,
0xAE25,0x61,
0xAE26,0x01,
0xAE27,0xDF,
0xAE28,0x02,
0xAE29,0xD0,
0xAB55,0x23,
0xAB57,0x23,
0xAB59,0x23,
0xAB5B,0x0A,
0xAB5D,0x0A,
0xAB5F,0x0A,
0xAB73,0x23,
0xAB75,0x23,
0xAB77,0x23,
0xAB79,0x0A,
0xAB7B,0x0A,
0xAB7D,0x0A,
0x0D08,0x00,
};
static kal_uint16 imx882txd_capture_30_setting[] = {
//3Lane
//reg_A
//QBIN(VBIN)_4096x3072_30FPS with PDAF VB_max
//H: 4096
//V: 3072
//MIPI output setting
//Address   value
        0x0112, 0x0A,
        0x0113, 0x0A,
        0x0114, 0x02,
//Line Length PCK Setting
//Address   value
        0x0342, 0x1D,
        0x0343, 0x4C,
        0x3850, 0x00,
        0x3851, 0xCD,
//Frame Length Lines Setting
//Address   value
        0x0340, 0x0F,
        0x0341, 0x3C,
//ROI Setting
//Address   value
        0x0344, 0x00,
        0x0345, 0x00,
        0x0346, 0x00,
        0x0347, 0x40,
        0x0348, 0x1F,
        0x0349, 0xFF,
        0x034A, 0x17,
        0x034B, 0xBF,
//Mode Setting
//Address   value
0x0900,	0x01,
0x0901,	0x22,
0x0902,	0x00,
0x3005,	0x02,
0x3006,	0x02,
0x3140,	0x0A,
0x3144,	0x00,
0x3148,	0x04,
0x31C0,	0x41,
0x31C1,	0x41,
0x3205,	0x00,
0x323C,	0x01,
0x39AC,	0x01,

//Digital Crop & Scaling
//Address   value
0x0408,	0x00,
0x0409,	0x30,
0x040A,	0x00,
0x040B,	0x04,
0x040C,	0x0F,
0x040D,	0xA0,
0x040E,	0x0B,
0x040F,	0xB8,

//Output Size Setting
//Address   value
0x034C,	0x0F,
0x034D,	0xA0,
0x034E,	0x0B,
0x034F,	0xB8,

//Clock Setting
//Address   value
0x0301,	0x05,
0x0303,	0x02,
0x0305,	0x04,
0x0306,	0x01,
0x0307,	0x6E,
0x030B,	0x02,
0x030D,	0x06,
0x030E,	0x01,
0x030F,	0xFF,

//Other Setting
//Address   value
0x3104,	0x01,
0x38B0,	0x03,
0x38B1,	0xFF,
0x38B4,	0x03,
0x38B5,	0xFF,
0x38B8,	0x03,
0x38B9,	0xFF,
0x38BC,	0x03,
0x38BD,	0xFF,
0x38D8,	0x14,
0x38E2,	0x00,
0x38E3,	0x00,
0x38E4,	0x00,
0x38E5,	0x00,
0x38E6,	0x00,
0x38E7,	0x00,


//Integration Setting
//Address   value
        0x0202, 0x03,
        0x0203, 0xE8,
//Gain Setting
//Address   value
        0x0204, 0x13,
        0x0205, 0x34,
        0x020E, 0x01,
        0x020F, 0x00,
//PDAF TYPE2 Setting
//Address   value
        0x3103, 0x00,
        0x3422, 0x01,
        0x3423, 0xFC,
//EAE-Bracketing Setting

//PHASE PIX VCID Setting
//Address   value
        0x30A4, 0x00,
        0x30A6, 0x00,
        0x30F2, 0x01,
        0x30F3, 0x01,
//PHASE PIX data type Setting
//Address   value
        0x30A5, 0x30,
        0x30A7, 0x30,
//PDAF TYPE2 VCID Setting
//Address   value
        0x30A2, 0x00,
        0x30F1, 0x01,
//PDAF TYPE2 data type Setting
//Address   value
        0x30A3, 0x30,
		//new setiing
        0x38A0, 0x00,
        0x38A1, 0x0A,
        0x38A2, 0x00,
        0x38A3, 0x76,
        0x38A8, 0x00,
        0x38A9, 0x0A,
        0x38AA, 0x00,
        0x38AB, 0x76,
        0x38D0, 0x01,
        0x38D1, 0x1A,
        0x38D2, 0x01,
        0x38D3, 0x4A,
        0x3B00, 0x07,
        0x3B01, 0x34,
        0x3B04, 0x07,
        0x3B05, 0xD4,
        0x38E0, 0x02,
        0x38E1, 0x42,
};
static kal_uint16 imx882txd_preview_setting[] = {
//3Lane
//reg_A
//QBIN(VBIN)_4096x3072_30FPS with PDAF VB_max
//H: 4096
//V: 3072
//MIPI output setting
//Address   value
        0x0112, 0x0A,
        0x0113, 0x0A,
        0x0114, 0x02,
//Line Length PCK Setting
//Address   value
        0x0342, 0x1D,
        0x0343, 0x4C,
        0x3850, 0x00,
        0x3851, 0xCD,
//Frame Length Lines Setting
//Address   value
        0x0340, 0x0F,
        0x0341, 0x3C,
//ROI Setting
//Address   value
        0x0344, 0x00,
        0x0345, 0x00,
        0x0346, 0x00,
        0x0347, 0x40,
        0x0348, 0x1F,
        0x0349, 0xFF,
        0x034A, 0x17,
        0x034B, 0xBF,
//Mode Setting
//Address   value
0x0900,	0x01,
0x0901,	0x22,
0x0902,	0x00,
0x3005,	0x02,
0x3006,	0x02,
0x3140,	0x0A,
0x3144,	0x00,
0x3148,	0x04,
0x31C0,	0x41,
0x31C1,	0x41,
0x3205,	0x00,
0x323C,	0x01,
0x39AC,	0x01,

//Digital Crop & Scaling
//Address   value
0x0408,	0x00,
0x0409,	0x30,
0x040A,	0x00,
0x040B,	0x04,
0x040C,	0x0F,
0x040D,	0xA0,
0x040E,	0x0B,
0x040F,	0xB8,

//Output Size Setting
//Address   value
0x034C,	0x0F,
0x034D,	0xA0,
0x034E,	0x0B,
0x034F,	0xB8,

//Clock Setting
//Address   value
0x0301,	0x05,
0x0303,	0x02,
0x0305,	0x04,
0x0306,	0x01,
0x0307,	0x6E,
0x030B,	0x02,
0x030D,	0x06,
0x030E,	0x01,
0x030F,	0xFF,

//Other Setting
//Address   value
0x3104,	0x01,
0x38B0,	0x03,
0x38B1,	0xFF,
0x38B4,	0x03,
0x38B5,	0xFF,
0x38B8,	0x03,
0x38B9,	0xFF,
0x38BC,	0x03,
0x38BD,	0xFF,
0x38D8,	0x14,
0x38E2,	0x00,
0x38E3,	0x00,
0x38E4,	0x00,
0x38E5,	0x00,
0x38E6,	0x00,
0x38E7,	0x00,


//Integration Setting
//Address   value
        0x0202, 0x03,
        0x0203, 0xE8,
//Gain Setting
//Address   value
        0x0204, 0x13,
        0x0205, 0x34,
        0x020E, 0x01,
        0x020F, 0x00,
//PDAF TYPE2 Setting
//Address   value
        0x3103, 0x00,
        0x3422, 0x01,
        0x3423, 0xFC,
//EAE-Bracketing Setting

//PHASE PIX VCID Setting
//Address   value
        0x30A4, 0x00,
        0x30A6, 0x00,
        0x30F2, 0x01,
        0x30F3, 0x01,
//PHASE PIX data type Setting
//Address   value
        0x30A5, 0x30,
        0x30A7, 0x30,
//PDAF TYPE2 VCID Setting
//Address   value
        0x30A2, 0x00,
        0x30F1, 0x01,
//PDAF TYPE2 data type Setting
//Address   value
        0x30A3, 0x30,
		//new setiing
        0x38A0, 0x00,
        0x38A1, 0x0A,
        0x38A2, 0x00,
        0x38A3, 0x76,
        0x38A8, 0x00,
        0x38A9, 0x0A,
        0x38AA, 0x00,
        0x38AB, 0x76,
        0x38D0, 0x01,
        0x38D1, 0x1A,
        0x38D2, 0x01,
        0x38D3, 0x4A,
        0x3B00, 0x07,
        0x3B01, 0x34,
        0x3B04, 0x07,
        0x3B05, 0xD4,
        0x38E0, 0x02,
        0x38E1, 0x42,
};
static kal_uint16 imx882txd_normal_video_setting[] = {
0x0112,	0x0A,
0x0113,	0x0A,
0x0114,	0x02,

0x0342,	0x1D,
0x0343,	0x4C,
0x3850,	0x00,
0x3851,	0xCD,

0x0340,	0x0F,
0x0341,	0x3C,

0x0344,	0x00,
0x0345,	0x00,
0x0346,	0x03,
0x0347,	0x00,
0x0348,	0x1F,
0x0349,	0xFF,
0x034A,	0x14,
0x034B,	0xFF,

0x0900,	0x01,
0x0901,	0x22,
0x0902,	0x00,
0x3005,	0x02,
0x3006,	0x02,
0x3140,	0x0A,
0x3144,	0x00,
0x3148,	0x04,
0x31C0,	0x41,
0x31C1,	0x41,
0x3205,	0x00,
0x323C,	0x01,
0x39AC,	0x01,

0x0408,	0x00,
0x0409,	0x00,
0x040A,	0x00,
0x040B,	0x00,
0x040C,	0x10,
0x040D,	0x00,
0x040E,	0x09,
0x040F,	0x00,

0x034C,	0x10,
0x034D,	0x00,
0x034E,	0x09,
0x034F,	0x00,

0x0301,	0x05,
0x0303,	0x02,
0x0305,	0x04,
0x0306,	0x01,
0x0307,	0x6E,
0x030B,	0x02,
0x030D,	0x06,
0x030E,	0x01,
0x030F,	0xFF,

0x3104,	0x01,

0x38B0,	0x03,
0x38B1,	0xFF,
0x38B4,	0x03,
0x38B5,	0xFF,
0x38B8,	0x03,
0x38B9,	0xFF,
0x38BC,	0x03,
0x38BD,	0xFF,

0x38D8,	0x14,

0x38E2,	0x00,
0x38E3,	0x00,
0x38E4,	0x00,
0x38E5,	0x00,
0x38E6,	0x00,
0x38E7,	0x00,


0x0202,	0x03,
0x0203,	0xE8,

0x0204,	0x13,
0x0205,	0x34,
0x020E,	0x01,
0x020F,	0x00,

0x3103,	0x00,
0x3422,	0x01,
0x3423,	0xFC,

0x30A4,	0x00,
0x30A6,	0x00,
0x30F2,	0x01,
0x30F3,	0x01,

0x30A5,	0x30,
0x30A7,	0x30,

0x30A2,	0x00,
0x30F1,	0x01,

0x30A3,	0x30,
//new setiing
        0x38A0, 0x00,
        0x38A1, 0x0A,
        0x38A2, 0x00,
        0x38A3, 0x76,
        0x38A8, 0x00,
        0x38A9, 0x0A,
        0x38AA, 0x00,
        0x38AB, 0x76,
        0x38D0, 0x01,
        0x38D1, 0x1A,
        0x38D2, 0x01,
        0x38D3, 0x4A,
        0x3B00, 0x07,
        0x3B01, 0x34,
        0x3B04, 0x07,
        0x3B05, 0xD4,
        0x38E0, 0x02,
        0x38E1, 0x42,
};
static kal_uint16 imx882txd_hs_video_setting[] = {
	0x0112,	0x0A,
	0x0113,	0x0A,
	0x0114,	0x02,

	0x0342,	0x12,
	0x0343,	0x08,
	0x3850,	0x00,
	0x3851,	0x9D,

	0x0340,	0x09,
	0x0341,	0xF3,

	0x0344,	0x00,
	0x0345,	0x00,
	0x0346,	0x03,
	0x0347,	0x00,
	0x0348,	0x1F,
	0x0349,	0xFF,
	0x034A,	0x14,
	0x034B,	0xFF,

	0x0900,	0x01,
	0x0901,	0x22,
	0x0902,	0x00,
	0x3005,	0x06,
	0x3006,	0x01,
	0x3140,	0x0A,
	0x3144,	0x00,
	0x3148,	0x00,
	0x31C0,	0x41,
	0x31C1,	0x41,
	0x3205,	0x00,
	0x323C,	0x02,
	0x39AC,	0x01,

	0x0408,	0x00,
	0x0409,	0x00,
	0x040A,	0x00,
	0x040B,	0x00,
	0x040C,	0x10,
	0x040D,	0x00,
	0x040E,	0x09,
	0x040F,	0x00,

	0x034C,	0x10,
	0x034D,	0x00,
	0x034E,	0x09,
	0x034F,	0x00,

	0x0301,	0x05,
	0x0303,	0x02,
	0x0305,	0x04,
	0x0306,	0x01,
	0x0307,	0x27,
	0x030B,	0x01,
	0x030D,	0x0C,
	0x030E,	0x02,
	0x030F,	0x5A,

	0x3104,	0x00,
	0x38A0,	0x00,
	0x38A1,	0x00,
	0x38A2,	0x00,
	0x38A3,	0x00,
	0x38A8,	0x00,
	0x38A9,	0x00,
	0x38AA,	0x00,
	0x38AB,	0x00,
	0x38B0,	0x03,
	0x38B1,	0xFF,
	0x38B4,	0x03,
	0x38B5,	0xFF,
	0x38B8,	0x03,
	0x38B9,	0xFF,
	0x38BC,	0x03,
	0x38BD,	0xFF,
	0x38D0,	0x06,
	0x38D1,	0x0E,
	0x38D2,	0x06,
	0x38D3,	0x0E,
	0x38D8,	0x14,
	0x38E0,	0x00,
	0x38E1,	0x00,
	0x38E2,	0x00,
	0x38E3,	0x00,
	0x38E4,	0x00,
	0x38E5,	0x00,
	0x38E6,	0x00,
	0x38E7,	0x00,
	0x3B00,	0x00,
	0x3B01,	0x00,
	0x3B04,	0x00,
	0x3B05,	0x00,

	0x0202,	0x03,
	0x0203,	0xE8,

	0x0204,	0x13,
	0x0205,	0x34,
	0x020E,	0x01,
	0x020F,	0x00,

	0x3103,	0x01,
	0x3422,	0x01,
	0x3423,	0xFC,

	0x30A4,	0x00,
	0x30A6,	0x00,
	0x30F2,	0x01,
	0x30F3,	0x01,

	0x30A5,	0x30,
	0x30A7,	0x30,

	0x30A2,	0x00,
	0x30F1,	0x01,
	0x30A3,	0x30,
    /*global timimg
    0x0808, 0x02,
    0x080A, 0x00,
    0x080B, 0xD7,
    0x080C, 0x00,
    0x080D, 0x8F,
    0x080E, 0x00,
    0x080F, 0xF7,
    0x0810, 0x00,
    0x0811, 0x87,
    0x0812, 0x00,
    0x0813, 0x87,
    0x0814, 0x00,
    0x0815, 0x87,
    0x0816, 0x02,
    0x0817, 0x47,
    0x0818, 0x00,
    0x0819, 0x6F,
    0x0824, 0x00,
    0x0825, 0xDF,
    0x0826, 0x00,
    0x0827, 0x0F,*/
    //new setting
    0x3B00, 0x06,
    0x3B01, 0x14,
    0x38D0, 0x01,
    0x38D1, 0xF0,
    0x38A0, 0x00,
    0x38A1, 0xC8,
    0x38A2, 0x00,
    0x38A3, 0xC8,
    0x3B04, 0x05,
    0x3B05, 0xC7,
    0x38D2, 0x07,
    0x38D3, 0x64,
    0x38A8, 0x02,
    0x38A9, 0x32,
    0x38AA, 0x02,
    0x38AB, 0x32,
    0x38E0, 0x00,
    0x38E1, 0x00,
    0x38E2, 0x00,
    0x38E3, 0x00,
    0x38E4, 0x00,
    0x38E5, 0x00,
    0x38E6, 0x00,
    0x38E7, 0x00,
};

static kal_uint16 imx882txd_slim_video_setting[] = {
//MIPI output setting
0x0112,	0x0A,
0x0113,	0x0A,
0x0114,	0x02,
//Line Length PCK Setting
0x0342,	0x0F,
0x0343,	0xB8,
0x3850,	0x00,
0x3851,	0x6E,
//Frame Length Lines Setting
0x0340,	0x07,
0x0341,	0x18,
//ROI Setting
0x0344,	0x00,
0x0345,	0x00,
0x0346,	0x03,
0x0347,	0x00,
0x0348,	0x1F,
0x0349,	0xFF,
0x034A,	0x14,
0x034B,	0xFF,
//Mode Setting
0x0900,	0x01,
0x0901,	0x44,
0x0902,	0x02,
0x3005,	0x02,
0x3006,	0x02,
0x3140,	0x0A,
0x3144,	0x00,
0x3148,	0x04,
0x31C0,	0x43,
0x31C1,	0x43,
0x3205,	0x00,
0x323C,	0x01,
0x39AC,	0x01,
//Digital Crop & Scaling
0x0408,	0x00,
0x0409,	0x00,
0x040A,	0x00,
0x040B,	0x00,
0x040C,	0x08,
0x040D,	0x00,
0x040E,	0x04,
0x040F,	0x80,
//Output Size Setting
0x034C,	0x08,
0x034D,	0x00,
0x034E,	0x04,
0x034F,	0x80,
//Clock Setting
0x0301,	0x05,
0x0303,	0x02,
0x0305,	0x04,
0x0306,	0x01,
0x0307,	0x6E,
0x030B,	0x02,
0x030D,	0x0C,
0x030E,	0x04,
0x030F,	0x3A,
//Other Setting
0x3104,	0x01,
0x38A0,	0x00,
0x38A1,	0x00,
0x38A2,	0x00,
0x38A3,	0x00,
0x38A8,	0x00,
0x38A9,	0x00,
0x38AA,	0x00,
0x38AB,	0x00,
0x38B0,	0x03,
0x38B1,	0xFF,
0x38B4,	0x03,
0x38B5,	0xFF,
0x38B8,	0x03,
0x38B9,	0xFF,
0x38BC,	0x03,
0x38BD,	0xFF,
0x38D0,	0x05,
0x38D1,	0x14,
0x38D2,	0x05,
0x38D3,	0x14,
0x38D8,	0x14,
0x38E0,	0x00,
0x38E1,	0x00,
0x38E2,	0x00,
0x38E3,	0x00,
0x38E4,	0x00,
0x38E5,	0x00,
0x38E6,	0x00,
0x38E7,	0x00,
0x3B00,	0x00,
0x3B01,	0x00,
0x3B04,	0x00,
0x3B05,	0x00,
//Integration Setting
0x0202,	0x03,
0x0203,	0xE8,
//Gain Setting
0x0204,	0x13,
0x0205,	0x34,
0x020E,	0x01,
0x020F,	0x00,
//PDAF TYPE2 Setting
0x3103,	0x00,
0x3422,	0x01,
0x3423,	0xFC,
//PHASE PIX VCID Setting
0x30A4,	0x00,
0x30A6,	0x00,
0x30F2,	0x01,
0x30F3,	0x01,
//PHASE PIX data type Setting
0x30A5,	0x30,
0x30A7,	0x30,
//PDAF TYPE2 VCID Setting
0x30A2,	0x00,
0x30F1,	0x01,
//PDAF TYPE2 data type Setting
0x30A3,	0x30,
};

static kal_uint16 imx882txd_custom1_setting[] = {
//MIPI output setting
0x0112,	0x0A,
0x0113,	0x0A,
0x0114,	0x02,
//Line Length PCK Setting
0x0342,	0x23,
0x0343,	0x00,
0x3850,	0x00,
0x3851,	0xF6,
//Frame Length Lines Setting
0x0340,	0x19,
0x0341,	0x68,
//ROI Setting
0x0344,	0x00,
0x0345,	0x00,
0x0346,	0x00,
0x0347,	0x00,
0x0348,	0x1F,
0x0349,	0xFF,
0x034A,	0x17,
0x034B,	0xFF,
//Mode Setting
0x0900,	0x00,
0x0901,	0x11,
0x0902,	0x00,
0x3005,	0x00,
0x3006,	0x00,
0x3140,	0x0A,
0x3144,	0x00,
0x3148,	0x00,
0x31C0,	0x01,
0x31C1,	0x01,
0x3205,	0x01,
0x323C,	0x01,
0x39AC,	0x01,
//Digital Crop & Scaling
0x0408,	0x00,
0x0409,	0x00,
0x040A,	0x00,
0x040B,	0x00,
0x040C,	0x20,
0x040D,	0x00,
0x040E,	0x18,
0x040F,	0x00,
//Output Size Setting
0x034C,	0x20,
0x034D,	0x00,
0x034E,	0x18,
0x034F,	0x00,
//Clock Setting
0x0301,	0x05,
0x0303,	0x02,
0x0305,	0x04,
0x0306,	0x01,
0x0307,	0x6E,
0x030B,	0x01,
0x030D,	0x0C,
0x030E,	0x03,
0x030F,	0x2c,
//Other Setting
0x3104,	0x01,
0x38A0,	0x00,
0x38A1,	0x00,
0x38A2,	0x00,
0x38A3,	0x00,
0x38A8,	0x00,
0x38A9,	0x00,
0x38AA,	0x00,
0x38AB,	0x00,
0x38B0,	0x03,
0x38B1,	0xFF,
0x38B4,	0x03,
0x38B5,	0xFF,
0x38B8,	0x03,
0x38B9,	0xFF,
0x38BC,	0x03,
0x38BD,	0xFF,
0x38D0,	0x03,
0x38D1,	0x20,
0x38D2,	0x03,
0x38D3,	0x20,
0x38D8,	0x14,
0x38E0,	0x00,
0x38E1,	0x00,
0x38E2,	0x00,
0x38E3,	0x00,
0x38E4,	0x00,
0x38E5,	0x00,
0x38E6,	0x00,
0x38E7,	0x00,
0x3B00,	0x00,
0x3B01,	0x00,
0x3B04,	0x00,
0x3B05,	0x00,
//Integration Setting
0x0202,	0x03,
0x0203,	0xE8,
//Gain Setting
0x0204,	0x00,
0x0205,	0x00,
0x020E,	0x01,
0x020F,	0x00,
//PDAF TYPE2 Setting
0x3103,	0x00,
0x3422,	0x01,
0x3423,	0xFC,
//PHASE PIX VCID Setting
0x30A4,	0x00,
0x30A6,	0x00,
0x30F2,	0x01,
0x30F3,	0x01,
//PHASE PIX data type Setting
0x30A5,	0x30,
0x30A7,	0x30,
//PDAF TYPE2 VCID Setting
0x30A2,	0x00,
0x30F1,	0x01,
//PDAF TYPE2 data type Setting
0x30A3,	0x30,
};
static kal_uint16 imx882txd_custom2_setting[] = {
    0x0112,	0x0A,
    0x0113,	0x0A,
    0x0114,	0x03,
    0x0342,	0x0E,
    0x0343,	0xB8,
    0x0340,	0x07,
    0x0341,	0x9A,
    0x0344,	0x00,
    0x0345,	0x00,
    0x0346,	0x02,
    0x0347,	0x60,
    0x0348,	0x19,
    0x0349,	0x9F,
    0x034A,	0x10,
    0x034B,	0xDF,
    0x0900,	0x01,
    0x0901,	0x22,
    0x0902,	0x09,
    0x3246,	0x81,
    0x3247,	0x81,
    0x0401,	0x00,
    0x0404,	0x00,
    0x0405,	0x10,
    0x0408,	0x00,
    0x0409,	0x00,
    0x040A,	0x00,
    0x040B,	0x00,
    0x040C,	0x0C,
    0x040D,	0xD0,
    0x040E,	0x07,
    0x040F,	0x40,
    0x034C,	0x0C,
    0x034D,	0xD0,
    0x034E,	0x07,
    0x034F,	0x40,
    0x0301,	0x05,
    0x0303,	0x02,
    0x0305,	0x04,
    0x0306,	0x00,
    0x0307,	0xB8,
    0x030B,	0x02,
    0x030D,	0x18,
    0x030E,	0x08,
    0x030F,	0x2A,
    0x0310,	0x01,
    0x3620,	0x00,
    0x3621,	0x00,
    0x3C12,	0x56,
    0x3C13,	0x52,
    0x3C14,	0x3E,
    0x3F0C,	0x00,
    0x3F14,	0x01,
    0x3F80,	0x00,
    0x3F81,	0xA0,
    0x3F8C,	0x00,
    0x3F8D,	0x00,
    0x3FFC,	0x00,
    0x3FFD,	0x1E,
    0x3FFE,	0x00,
    0x3FFF,	0xDC,
    0x0202,	0x07,
    0x0203,	0x6A,
    0x0204,	0x00,
    0x0205,	0x70,
    0x020E,	0x01,
    0x020F,	0x00,
    0x0210,	0x01,
    0x0211,	0x00,
    0x0212,	0x01,
    0x0213,	0x00,
    0x0214,	0x01,
    0x0215,	0x00,
};
static kal_uint16 imx882txd_seamless_preview[] = {
	 0x0112, 0x0A,
        0x0113, 0x0A,
        0x0114, 0x02,
//Line Length PCK Setting
//Address   value
        0x0342, 0x1D,
        0x0343, 0x4C,
        0x3850, 0x00,
        0x3851, 0xCD,
//Frame Length Lines Setting
//Address   value
        0x0340, 0x0E,
        0x0341, 0x30,
//ROI Setting
//Address   value
        0x0344, 0x00,
        0x0345, 0x00,
        0x0346, 0x00,
        0x0347, 0x40,
        0x0348, 0x1F,
        0x0349, 0xFF,
        0x034A, 0x17,
        0x034B, 0xBF,
//Mode Setting
//Address   value
0x0900,	0x01,
0x0901,	0x22,
0x0902,	0x00,
0x3005,	0x02,
0x3006,	0x02,
0x3140,	0x0A,
0x3144,	0x00,
0x3148,	0x04,
0x31C0,	0x41,
0x31C1,	0x41,
0x3205,	0x00,
0x323C,	0x01,
0x39AC,	0x01,

//Digital Crop & Scaling
//Address   value
0x0408,	0x00,
0x0409,	0x30,
0x040A,	0x00,
0x040B,	0x04,
0x040C,	0x0F,
0x040D,	0xA0,
0x040E,	0x0B,
0x040F,	0xB8,

//Output Size Setting
//Address   value
0x034C,	0x0F,
0x034D,	0xA0,
0x034E,	0x0B,
0x034F,	0xB8,

//Clock Setting
//Address   value
0x0301,	0x05,
0x0303,	0x02,
0x0305,	0x04,
0x0306,	0x01,
0x0307,	0x6E,
0x030B,	0x02,
0x030D,	0x0C,
0x030E,	0x03,
0x030F,	0xEA,

//Other Setting
//Address   value
0x3104,	0x01,
0x38A0,	0x00,
0x38A1,	0xB4,
0x38A2,	0x00,
0x38A3,	0xE8,
0x38A8,	0x00,
0x38A9,	0x38,
0x38AA,	0x00,
0x38AB,	0x6A,
0x38B0,	0x03,
0x38B1,	0xFF,
0x38B4,	0x03,
0x38B5,	0xFF,
0x38B8,	0x03,
0x38B9,	0xFF,
0x38BC,	0x03,
0x38BD,	0xFF,
0x38D0,	0x0B,
0x38D1,	0x04,
0x38D2,	0x08,
0x38D3,	0xB0,
0x38D8,	0x14,
0x38E0,	0x00,
0x38E1,	0x00,
0x38E2,	0x00,
0x38E3,	0x00,
0x38E4,	0x00,
0x38E5,	0x00,
0x38E6,	0x00,
0x38E7,	0x00,
0x3B00,	0x08,
0x3B01,	0x8E,
0x3B04,	0x00,
0x3B05,	0x88,

//Integration Setting
//Address   value
        0x0202, 0x03,
        0x0203, 0xE8,
//Gain Setting
//Address   value
        0x0204, 0x13,
        0x0205, 0x34,
        0x020E, 0x01,
        0x020F, 0x00,
//PDAF TYPE2 Setting
//Address   value
        0x3103, 0x00,
        0x3422, 0x01,
        0x3423, 0xFC,
//EAE-Bracketing Setting

//PHASE PIX VCID Setting
//Address   value
        0x30A4, 0x00,
        0x30A6, 0x00,
        0x30F2, 0x01,
        0x30F3, 0x01,
//PHASE PIX data type Setting
//Address   value
        0x30A5, 0x30,
        0x30A7, 0x30,
//PDAF TYPE2 VCID Setting
//Address   value
        0x30A2, 0x00,
        0x30F1, 0x01,
//PDAF TYPE2 data type Setting
//Address   value
        0x30A3, 0x30,
//2x new
		0x0340, 0x0F,
        0x0341, 0x3C,
        0x030D, 0x06,
        0x030E, 0x01,
        0x030F, 0xFF,
        0x38A0, 0x00,
        0x38A1, 0x0A,
        0x38A2, 0x00,
        0x38A3, 0x76,
        0x38A8, 0x00,
        0x38A9, 0x0A,
        0x38AA, 0x00,
        0x38AB, 0x76,
        0x38D0, 0x01,
        0x38D1, 0x1A,
        0x38D2, 0x01,
        0x38D3, 0x4A,
        0x3B00, 0x07,
        0x3B01, 0x34,
        0x3B04, 0x07,
        0x3B05, 0xD4,
        0x38E0, 0x02,
        0x38E1, 0x42,
};
static kal_uint16 imx882txd_seamless_custom3[] = {
    0x0112,0x0A,
    0x0113,0x0A,
    0x0114,0x02,
    0x0342,0x23,
    0x0343,0x00,
    0x3850,0x00,
    0x3851,0xF6,
    0x0340,0x0C,
    0x0341,0xB4,
    0x0344,0x08,
    0x0345,0x00,
    0x0346,0x06,
    0x0347,0x00,
    0x0348,0x17,
    0x0349,0xFF,
    0x034A,0x11,
    0x034B,0xFF,
    0x0900,0x00,
    0x0901,0x11,
    0x0902,0x00,
    0x3005,0x00,
    0x3006,0x00,
    0x3140,0x0A,
    0x3144,0x00,
    0x3148,0x00,
    0x31C0,0x01,
    0x31C1,0x01,
    0x3205,0x01,
    0x323C,0x01,
    0x39AC,0x01,
    0x0408,0x00,
    0x0409,0x30,
    0x040A,0x00,
    0x040B,0x24,
    0x040C,0x0F,
    0x040D,0xA0,
    0x040E,0x0B,
    0x040F,0xB8,
    0x034C,0x0F,
    0x034D,0xA0,
    0x034E,0x0B,
    0x034F,0xB8,
    0x0301,0x05,
    0x0303,0x02,
    0x0305,0x04,
    0x0306,0x01,
    0x0307,0x6E,
    0x030B,0x02,
    0x030D,0x0C,
    0x030E,0x03,
    0x030F,0xEA,
    0x3104,0x01,
    0x38A0,0x00,
    0x38A1,0x00,
    0x38A2,0x00,
    0x38A3,0x00,
    0x38A8,0x00,
    0x38A9,0x00,
    0x38AA,0x00,
    0x38AB,0x00,
    0x38B0,0x03,
    0x38B1,0xFF,
    0x38B4,0x03,
    0x38B5,0xFF,
    0x38B8,0x03,
    0x38B9,0xFF,
    0x38BC,0x03,
    0x38BD,0xFF,
    0x38D0,0x03,
    0x38D1,0x52,
    0x38D2,0x03,
    0x38D3,0x52,
    0x38D8,0x14,
    0x38E0,0x00,
    0x38E1,0x00,
    0x38E2,0x00,
    0x38E3,0x00,
    0x38E4,0x00,
    0x38E5,0x00,
    0x38E6,0x00,
    0x38E7,0x00,
    0x3B00,0x00,
    0x3B01,0x00,
    0x3B04,0x00,
    0x3B05,0x00,
    0x0202,0x03,
    0x0203,0xE8,
    0x0204,0x00,
    0x0205,0x00,
    0x020E,0x01,
    0x020F,0x00,
    0x3103,0x00,
    0x3422,0x01,
    0x3423,0xFC,
    0x30A4,0x00,
    0x30A6,0x00,
    0x30F2,0x01,
    0x30F3,0x01,
    0x30A5,0x30,
    0x30A7,0x30,
    0x30A2,0x00,
    0x30F1,0x01,
    0x30A3,0x30,
//2x new
		0x0340, 0x0F,
        0x0341, 0x3C,
        0x030D, 0x06,
        0x030E, 0x01,
        0x030F, 0xFF,
        0x38A0, 0x00,
        0x38A1, 0x0A,
        0x38A2, 0x00,
        0x38A3, 0x76,
        0x38A8, 0x00,
        0x38A9, 0x0A,
        0x38AA, 0x00,
        0x38AB, 0x76,
        0x38D0, 0x01,
        0x38D1, 0x1A,
        0x38D2, 0x01,
        0x38D3, 0x4A,
        0x3B00, 0x07,
        0x3B01, 0x34,
        0x3B04, 0x07,
        0x3B05, 0xD4,
        0x38E0, 0x02,
        0x38E1, 0x42,
};
static kal_uint16 imx882txd_custom3_setting[] = {
	0x0112,0x0A,
    0x0113,0x0A,
    0x0114,0x02,
    0x0342,0x23,
    0x0343,0x00,
    0x3850,0x00,
    0x3851,0xF6,
    0x0340,0x0C,
    0x0341,0xB4,
    0x0344,0x08,
    0x0345,0x00,
    0x0346,0x06,
    0x0347,0x00,
    0x0348,0x17,
    0x0349,0xFF,
    0x034A,0x11,
    0x034B,0xFF,
    0x0900,0x00,
    0x0901,0x11,
    0x0902,0x00,
    0x3005,0x00,
    0x3006,0x00,
    0x3140,0x0A,
    0x3144,0x00,
    0x3148,0x00,
    0x31C0,0x01,
    0x31C1,0x01,
    0x3205,0x01,
    0x323C,0x01,
    0x39AC,0x01,
    0x0408,0x00,
    0x0409,0x30,
    0x040A,0x00,
    0x040B,0x24,
    0x040C,0x0F,
    0x040D,0xA0,
    0x040E,0x0B,
    0x040F,0xB8,
    0x034C,0x0F,
    0x034D,0xA0,
    0x034E,0x0B,
    0x034F,0xB8,
    0x0301,0x05,
    0x0303,0x02,
    0x0305,0x04,
    0x0306,0x01,
    0x0307,0x6E,
    0x030B,0x02,
    0x030D,0x0C,
    0x030E,0x03,
    0x030F,0xEA,
    0x3104,0x01,
    0x38A0,0x00,
    0x38A1,0x00,
    0x38A2,0x00,
    0x38A3,0x00,
    0x38A8,0x00,
    0x38A9,0x00,
    0x38AA,0x00,
    0x38AB,0x00,
    0x38B0,0x03,
    0x38B1,0xFF,
    0x38B4,0x03,
    0x38B5,0xFF,
    0x38B8,0x03,
    0x38B9,0xFF,
    0x38BC,0x03,
    0x38BD,0xFF,
    0x38D0,0x03,
    0x38D1,0x52,
    0x38D2,0x03,
    0x38D3,0x52,
    0x38D8,0x14,
    0x38E0,0x00,
    0x38E1,0x00,
    0x38E2,0x00,
    0x38E3,0x00,
    0x38E4,0x00,
    0x38E5,0x00,
    0x38E6,0x00,
    0x38E7,0x00,
    0x3B00,0x00,
    0x3B01,0x00,
    0x3B04,0x00,
    0x3B05,0x00,
    0x0202,0x03,
    0x0203,0xE8,
    0x0204,0x00,
    0x0205,0x00,
    0x020E,0x01,
    0x020F,0x00,
    0x3103,0x00,
    0x3422,0x01,
    0x3423,0xFC,
    0x30A4,0x00,
    0x30A6,0x00,
    0x30F2,0x01,
    0x30F3,0x01,
    0x30A5,0x30,
    0x30A7,0x30,
    0x30A2,0x00,
    0x30F1,0x01,
    0x30A3,0x30,
//2x new
		0x0340, 0x0F,
        0x0341, 0x3C,
        0x030D, 0x06,
        0x030E, 0x01,
        0x030F, 0xFF,
        0x38A0, 0x00,
        0x38A1, 0x0A,
        0x38A2, 0x00,
        0x38A3, 0x76,
        0x38A8, 0x00,
        0x38A9, 0x0A,
        0x38AA, 0x00,
        0x38AB, 0x76,
        0x38D0, 0x01,
        0x38D1, 0x1A,
        0x38D2, 0x01,
        0x38D3, 0x4A,
        0x3B00, 0x07,
        0x3B01, 0x34,
        0x3B04, 0x07,
        0x3B05, 0xD4,
        0x38E0, 0x02,
        0x38E1, 0x42,
};
#endif
static void sensor_init(struct subdrv_ctx *ctx)
{
	LOG_DEBUG("E\n");
	imx882txd_table_write_cmos_sensor(ctx, imx882txd_init_setting,
		sizeof(imx882txd_init_setting)/sizeof(kal_uint16));
	set_mirror_flip(ctx, ctx->mirror);
	write_sensor_QSC(ctx);
	LOG_DEBUG("X");
}	/*	  sensor_init  */
static void preview_setting(struct subdrv_ctx *ctx)
{
	LOG_DEBUG("E\n");
	imx882txd_table_write_cmos_sensor(ctx, imx882txd_preview_setting,
		sizeof(imx882txd_preview_setting)/sizeof(kal_uint16));
	LOG_DEBUG("X");
} /* preview_setting */
/*full size 30fps*/
static void capture_setting(struct subdrv_ctx *ctx, kal_uint16 currefps)
{
	LOG_DEBUG("%s(PD 012515) 30 fps E! currefps:%d\n", __func__, currefps);
	/*************MIPI output setting************/
	imx882txd_table_write_cmos_sensor(ctx, imx882txd_capture_30_setting,
		sizeof(imx882txd_capture_30_setting)/sizeof(kal_uint16));
	LOG_DEBUG("%s(PD 012515) 30 fpsX\n", __func__);
}
static void normal_video_setting(struct subdrv_ctx *ctx, kal_uint16 currefps)
{
	LOG_DEBUG("%s E! currefps:%d\n", __func__, currefps);
	imx882txd_table_write_cmos_sensor(ctx, imx882txd_normal_video_setting,
	sizeof(imx882txd_normal_video_setting)/sizeof(kal_uint16));
	LOG_DEBUG("X\n");
}
static void hs_video_setting(struct subdrv_ctx *ctx)
{
	LOG_DEBUG("%s E! currefps 120\n", __func__);
	/*************MIPI output setting************/
	imx882txd_table_write_cmos_sensor(ctx, imx882txd_hs_video_setting,
	sizeof(imx882txd_hs_video_setting)/sizeof(kal_uint16));
	LOG_DEBUG("X\n");
}
static void slim_video_setting(struct subdrv_ctx *ctx)
{
	LOG_DEBUG("%s E\n", __func__);
	imx882txd_table_write_cmos_sensor(ctx, imx882txd_slim_video_setting,
	sizeof(imx882txd_slim_video_setting)/sizeof(kal_uint16));
	LOG_DEBUG("X\n");
}
/*full size 16M@24fps*/
static void custom1_setting(struct subdrv_ctx *ctx)
{
	LOG_DEBUG("%s 240 fps E! currefps\n", __func__);
	/*************MIPI output setting************/
	imx882txd_table_write_cmos_sensor(ctx, imx882txd_custom1_setting,
		sizeof(imx882txd_custom1_setting)/sizeof(kal_uint16));
	LOG_DEBUG("X");
}
/*full size 8M@24fps*/
static void custom2_setting(struct subdrv_ctx *ctx)
{
	LOG_DEBUG("%s 480 fps E! currefps\n", __func__);
	/*************MIPI output setting************/
	imx882txd_table_write_cmos_sensor(ctx, imx882txd_custom2_setting,
		sizeof(imx882txd_custom2_setting)/sizeof(kal_uint16));
	LOG_DEBUG("X");
}
/*full size 16M@24fps*/
static void custom3_setting(struct subdrv_ctx *ctx)
{
	LOG_DEBUG("%s 4M*60 fps E! currefps\n", __func__);
	/*************MIPI output setting************/
	imx882txd_table_write_cmos_sensor(ctx, imx882txd_custom3_setting,
		sizeof(imx882txd_custom3_setting)/sizeof(kal_uint16));
	LOG_DEBUG("X");
}
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
	int module_id_addr = 0x01;
	char module_id;

	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		ctx->i2c_write_id = imgsensor_info.i2c_addr_table[i];
		do {
			*sensor_id = ((read_cmos_sensor_8(ctx, 0x0016) << 8)
					| read_cmos_sensor_8(ctx, 0x0017)) + 1;
			adaptor_i2c_rd_p8(ctx->i2c_client, 0xa0 >> 1,
				      module_id_addr,&module_id, 1);

			LOG_INF("[imx882txd_camera_sensor]get_imgsensor_id: ! imgsensor_info.sensor_id: 0x%x, sensor id: 0x%x, module_id = %c\n",imgsensor_info.sensor_id, *sensor_id, module_id);
			if (*sensor_id == imgsensor_info.sensor_id && (module_id == 0x07)) {
				LOG_INF("[imx882txd_camera_sensor]get_imgsensor_id: i2c write id: 0x%x, sensor id: 0x%x\n",
					ctx->i2c_write_id, *sensor_id);
				return ERROR_NONE;
			}
			LOG_INF("[imx882txd_camera_sensor]get_imgsensor_id: Read sensor id fail! i2c write id: 0x%x, sensor id: 0x%x\n",
				ctx->i2c_write_id, *sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		retry = 2;
	}
	if (*sensor_id != imgsensor_info.sensor_id ||
	    (module_id != 0x07)) {
		/* if Sensor ID is not correct,
		 * Must set *sensor_id to 0xFFFFFFFF
		 */
		*sensor_id = 0x0;
		LOG_INF("[imx882txd_camera_sensor]get_imgsensor_id: sensor id: 0x%x\n", *sensor_id);
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	return ERROR_NONE;
}

static kal_uint32 seamless_switch(struct subdrv_ctx *ctx,enum MSDK_SCENARIO_ID_ENUM scenario_id, struct mtk_hdr_ae *ae_ctrl)
{
    ctx->extend_frame_length_en = KAL_FALSE;
    LOG_INF("scenario_id: %d SHUTTER_NE_FRM_1: %lld GAIN_NE_FRM_1: %d SHUTTER_ME_FRM_1: %lld GAIN_ME_FRM_1: %d SHUTTER_SE_FRM_1: %lld GAIN_SE_FRM_1: %d",
            scenario_id, ae_ctrl->exposure.le_exposure, ae_ctrl->gain.le_gain, ae_ctrl->exposure.me_exposure, ae_ctrl->gain.me_gain, ae_ctrl->exposure.se_exposure, ae_ctrl->gain.se_gain);

    switch (scenario_id) {
    case SENSOR_SCENARIO_ID_CUSTOM3:
    {
        spin_lock(&imgsensor_drv_lock);
        ctx->sensor_mode = IMGSENSOR_MODE_CUSTOM3;
        ctx->autoflicker_en = KAL_FALSE;
        ctx->pclk = imgsensor_info.custom3.pclk;
        ctx->line_length = imgsensor_info.custom3.linelength;
        ctx->frame_length = imgsensor_info.custom3.framelength;
        ctx->min_frame_length = imgsensor_info.custom3.framelength;
        imgsensor_info.min_shutter = imgsensor_info.custom3.min_shutter;
        imgsensor_info.min_gain = imgsensor_info.custom3.min_gain;
        imgsensor_info.max_gain = imgsensor_info.custom3.max_gain;
        imgsensor_info.exp_step = imgsensor_info.custom3.exp_step;
        spin_unlock(&imgsensor_drv_lock);
        write_cmos_sensor_8(ctx, 0x0104, 0x01);
        write_cmos_sensor_8(ctx, 0x3010, 0x02);
        imx882txd_table_write_cmos_sensor(ctx ,imx882txd_seamless_custom3, sizeof(imx882txd_seamless_custom3) / sizeof(kal_uint16));
        if (ae_ctrl) {
            LOG_INF("call MSDK_SCENARIO_ID_CAMERA_CUSTOM3 %lld %d",
                    ae_ctrl->exposure.le_exposure, ae_ctrl->gain.le_gain);
            set_shutter(ctx, ae_ctrl->exposure.le_exposure);
            set_gain(ctx, ae_ctrl->gain.le_gain);
        }
        write_cmos_sensor_8(ctx, 0x0104, 0x00);
    }
    break;
    case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
    {
        spin_lock(&imgsensor_drv_lock);
        ctx->sensor_mode = IMGSENSOR_MODE_PREVIEW;
        ctx->autoflicker_en = KAL_FALSE;
        ctx->pclk = imgsensor_info.pre.pclk;
        ctx->line_length = imgsensor_info.pre.linelength;
        ctx->frame_length = imgsensor_info.pre.framelength;
        ctx->min_frame_length = imgsensor_info.pre.framelength;
        imgsensor_info.min_shutter = 6;
        imgsensor_info.min_gain = 1463;
        imgsensor_info.max_gain = BASEGAIN * 64;
        imgsensor_info.exp_step = 4;
        spin_unlock(&imgsensor_drv_lock);
        write_cmos_sensor_8(ctx, 0x0104, 0x01);
        write_cmos_sensor_8(ctx, 0x3010, 0x02);
        imx882txd_table_write_cmos_sensor(ctx ,imx882txd_seamless_preview, sizeof(imx882txd_seamless_preview) / sizeof(kal_uint16));
        if (ae_ctrl) {
            LOG_INF("call MSDK_SCENARIO_ID_CAMERA_PREVIEW %lld %d",
                    ae_ctrl->exposure.le_exposure, ae_ctrl->gain.le_gain);
            set_shutter(ctx, ae_ctrl->exposure.le_exposure);
            set_gain(ctx, ae_ctrl->gain.le_gain);
        }
        write_cmos_sensor_8(ctx, 0x0104, 0x00);
    }
    break;
    default:
    {
        pr_info( "error! wrong setting in set_seamless_switch = %d",scenario_id);
        return 0xff;
    }
	break;
    }
    ctx->fast_mode_on = KAL_TRUE;
    LOG_INF("%s success, scenario is switched to %d", __func__, scenario_id);
    return 0;
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
	kal_uint16 sensor_id = 0;
	/*sensor have two i2c address 0x6c 0x6d & 0x21 0x20,
	 *we should detect the module used i2c address
	 */
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		ctx->i2c_write_id = imgsensor_info.i2c_addr_table[i];
		do {
			sensor_id = ((read_cmos_sensor_8(ctx, 0x0016) << 8)
					| read_cmos_sensor_8(ctx, 0x0017)) + 1;
			if (sensor_id == imgsensor_info.sensor_id) {
				LOG_DEBUG("i2c write id: 0x%x, sensor id: 0x%x\n",
					ctx->i2c_write_id, sensor_id);
				break;
			}
			LOG_DEBUG("Read sensor id fail, i2c addr: 0x%x,sensor id: 0x%x\n",
				ctx->i2c_write_id,sensor_id);
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
	ctx->shutter = 0x3D0;
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
} /* open */
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
	LOG_DEBUG("E\n");
	/*No Need to implement this function*/
	write_cmos_sensor_8(ctx, 0x0100, 0x00);
	return ERROR_NONE;
} /* close */
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
	LOG_DEBUG("%s E\n", __func__);
	ctx->sensor_mode = IMGSENSOR_MODE_PREVIEW;
	ctx->pclk = imgsensor_info.pre.pclk;
	ctx->line_length = imgsensor_info.pre.linelength;
	ctx->frame_length = imgsensor_info.pre.framelength;
	ctx->min_frame_length = imgsensor_info.pre.framelength;
	ctx->autoflicker_en = KAL_FALSE;
	preview_setting(ctx);
//	if (ctx->pdaf_mode)
//		imx882txd_apply_LRC(ctx);
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
static kal_uint32 capture(struct subdrv_ctx *ctx, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_DEBUG("E\n");
	ctx->sensor_mode = IMGSENSOR_MODE_CAPTURE;
	if (ctx->current_fps != imgsensor_info.cap.max_framerate)
		LOG_DEBUG(
			"Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
			ctx->current_fps,
			imgsensor_info.cap.max_framerate / 10);
	ctx->pclk = imgsensor_info.cap.pclk;
	ctx->line_length = imgsensor_info.cap.linelength;
	ctx->frame_length = imgsensor_info.cap.framelength;
	ctx->min_frame_length = imgsensor_info.cap.framelength;
	ctx->autoflicker_en = KAL_FALSE;
	capture_setting(ctx, ctx->current_fps);
//	if (ctx->pdaf_mode)
//		imx882txd_apply_LRC(ctx);
	/* set_mirror_flip(ctx, ctx->mirror); */
	return ERROR_NONE;
}
static kal_uint32 normal_video(struct subdrv_ctx *ctx,
		MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_DEBUG("E\n");
	ctx->sensor_mode = IMGSENSOR_MODE_VIDEO;
	ctx->pclk = imgsensor_info.normal_video.pclk;
	ctx->line_length = imgsensor_info.normal_video.linelength;
	ctx->frame_length = imgsensor_info.normal_video.framelength;
	ctx->min_frame_length = imgsensor_info.normal_video.framelength;
	ctx->autoflicker_en = KAL_FALSE;
	normal_video_setting(ctx, ctx->current_fps);
//	if (ctx->pdaf_mode)
//		imx882txd_apply_LRC(ctx);
	/*set_mirror_flip(ctx, ctx->mirror);*/
	return ERROR_NONE;
}
static kal_uint32 hs_video(struct subdrv_ctx *ctx,
		MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_DEBUG("E\n");
	ctx->sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	ctx->pclk = imgsensor_info.hs_video.pclk;
	/*ctx->video_mode = KAL_TRUE;*/
	ctx->line_length = imgsensor_info.hs_video.linelength;
	ctx->frame_length = imgsensor_info.hs_video.framelength;
	ctx->min_frame_length = imgsensor_info.hs_video.framelength;
	ctx->dummy_line = 0;
	ctx->dummy_pixel = 0;
	/*ctx->current_fps = 300;*/
	ctx->autoflicker_en = KAL_FALSE;
	hs_video_setting(ctx);
	/*set_mirror_flip(ctx, ctx->mirror);*/
	return ERROR_NONE;
}	/*	hs_video   */
static kal_uint32 slim_video(struct subdrv_ctx *ctx,
		MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_DEBUG("E\n");
	ctx->sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	ctx->pclk = imgsensor_info.slim_video.pclk;
	/*ctx->video_mode = KAL_TRUE;*/
	ctx->line_length = imgsensor_info.slim_video.linelength;
	ctx->frame_length = imgsensor_info.slim_video.framelength;
	ctx->min_frame_length = imgsensor_info.slim_video.framelength;
	ctx->dummy_line = 0;
	ctx->dummy_pixel = 0;
	/*ctx->current_fps = 300;*/
	ctx->autoflicker_en = KAL_FALSE;
	slim_video_setting(ctx);
	/*set_mirror_flip(ctx, ctx->mirror);*/
	return ERROR_NONE;
}	/* slim_video */
static kal_uint32 custom1(struct subdrv_ctx *ctx,
		MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_DEBUG("%s.\n", __func__);
	ctx->sensor_mode = IMGSENSOR_MODE_CUSTOM1;
	ctx->pclk = imgsensor_info.custom1.pclk;
	ctx->line_length = imgsensor_info.custom1.linelength;
	ctx->frame_length = imgsensor_info.custom1.framelength;
	ctx->min_frame_length = imgsensor_info.custom1.framelength;
	ctx->autoflicker_en = KAL_FALSE;
	custom1_setting(ctx);
//	if (ctx->pdaf_mode)
//		imx882txd_apply_LRC(ctx);
	//write_sensor_QSC(ctx);
	return ERROR_NONE;
}	/* custom1 */
static kal_uint32 custom2(struct subdrv_ctx *ctx,
		MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_DEBUG("%s.\n", __func__);
	ctx->sensor_mode = IMGSENSOR_MODE_CUSTOM2;
	ctx->pclk = imgsensor_info.custom2.pclk;
	ctx->line_length = imgsensor_info.custom2.linelength;
	ctx->frame_length = imgsensor_info.custom2.framelength;
	ctx->min_frame_length = imgsensor_info.custom2.framelength;
	ctx->autoflicker_en = KAL_FALSE;
	custom2_setting(ctx);
//	if (ctx->pdaf_mode)
//		imx882txd_apply_LRC(ctx);
	return ERROR_NONE;
}
static kal_uint32 custom3(struct subdrv_ctx *ctx,
		MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_DEBUG("%s.\n", __func__);
	ctx->sensor_mode = IMGSENSOR_MODE_CUSTOM3;
	ctx->pclk = imgsensor_info.custom3.pclk;
	ctx->line_length = imgsensor_info.custom3.linelength;
	ctx->frame_length = imgsensor_info.custom3.framelength;
	ctx->min_frame_length = imgsensor_info.custom3.framelength;
	ctx->autoflicker_en = KAL_FALSE;
	custom3_setting(ctx);
//	if (ctx->pdaf_mode)
		//imx882txd_apply_LRC(ctx);
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
} /* get_resolution */
static int get_info(struct subdrv_ctx *ctx,
		enum MSDK_SCENARIO_ID_ENUM scenario_id,
		MSDK_SENSOR_INFO_STRUCT *sensor_info,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	int i = 0;
	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4; /* not use */
	sensor_info->SensorResetActiveHigh = FALSE; /* not use */
	sensor_info->SensorResetDelayCount = 5; /* not use */
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
	sensor_info->DelayFrame[SENSOR_SCENARIO_ID_CUSTOM1] =
		imgsensor_info.custom1_delay_frame;
	sensor_info->DelayFrame[SENSOR_SCENARIO_ID_CUSTOM2] =
		imgsensor_info.custom2_delay_frame;
	sensor_info->DelayFrame[SENSOR_SCENARIO_ID_CUSTOM3] =
		imgsensor_info.custom3_delay_frame;
	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;
	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;
	sensor_info->AESensorGainDelayFrame =
		imgsensor_info.ae_sensor_gain_delay_frame;
	sensor_info->AEISPGainDelayFrame =
		imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->TEMPERATURE_SUPPORT = imgsensor_info.temperature_support;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
	sensor_info->PDAF_Support = PDAF_SUPPORT_CAMSV_QPD;
	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3; /* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2; /* not use */
	sensor_info->SensorPixelClockCount = 3; /* not use */
	sensor_info->SensorDataLatchCount = 2; /* not use */
	sensor_info->SensorWidthSampling = 0; /* 0 is default 1x */
	sensor_info->SensorHightSampling = 0; /* 0 is default 1x */
	sensor_info->SensorPacketECCOrder = 1;
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
static int control(struct subdrv_ctx *ctx, enum MSDK_SCENARIO_ID_ENUM scenario_id,
			MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_DEBUG("scenario_id = %d\n", scenario_id);
	ctx->current_scenario_id = scenario_id;
	imgsensor_info.min_shutter = 6;
	imgsensor_info.min_gain = 1463;
	imgsensor_info.max_gain = BASEGAIN * 64;
	imgsensor_info.exp_step = 4;
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
		imgsensor_info.min_shutter = imgsensor_info.hs_video.min_shutter;
		imgsensor_info.exp_step = imgsensor_info.hs_video.exp_step;
		break;
	case SENSOR_SCENARIO_ID_SLIM_VIDEO:
		slim_video(ctx, image_window, sensor_config_data);
		imgsensor_info.min_shutter = imgsensor_info.slim_video.min_shutter;
		imgsensor_info.exp_step = imgsensor_info.slim_video.exp_step;
		break;
	case SENSOR_SCENARIO_ID_CUSTOM1:
		custom1(ctx, image_window, sensor_config_data);
		break;
	case SENSOR_SCENARIO_ID_CUSTOM2:
		custom2(ctx, image_window, sensor_config_data);
		break;
	case SENSOR_SCENARIO_ID_CUSTOM3:
		custom3(ctx, image_window, sensor_config_data);
		break;
	default:
		LOG_DEBUG("Error ScenarioId setting");
		preview(ctx, image_window, sensor_config_data);
		return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}	/* control(ctx) */
static kal_uint32 set_video_mode(struct subdrv_ctx *ctx, UINT16 framerate)
{
	LOG_DEBUG("framerate = %d\n ", framerate);
	/* SetVideoMode Function should fix framerate */
	if (framerate == 0)
		/* Dynamic frame rate */
		return ERROR_NONE;
	if ((framerate == 600) && (ctx->autoflicker_en == KAL_TRUE))
		ctx->current_fps = 592;
	else if ((framerate == 300) && (ctx->autoflicker_en == KAL_TRUE))
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
	LOG_DEBUG("enable = %d, framerate = %d\n", enable, framerate);
	if (enable) /*enable auto flicker*/
		ctx->autoflicker_en = KAL_TRUE;
	else /*Cancel Auto flick*/
		ctx->autoflicker_en = KAL_FALSE;
	return ERROR_NONE;
}
static kal_uint32 set_max_framerate_by_scenario(struct subdrv_ctx *ctx,
		enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
	kal_uint32 frame_length;
	LOG_DEBUG("scenario_id = %d, framerate = %d\n", scenario_id, framerate);
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
			LOG_DEBUG(
				"Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n"
				, framerate
				, imgsensor_info.cap.max_framerate/10);
			frame_length = imgsensor_info.cap.pclk / framerate * 10
					/ imgsensor_info.cap.linelength;
		if (frame_length > imgsensor_info.max_frame_length) {
			LOG_DEBUG(
				"Warning: frame_length %d > max_frame_length %d!\n"
				, frame_length
				, imgsensor_info.max_frame_length);
			break;
		}
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
		if(frame_length % 8)
		{
			frame_length -= (frame_length % 8);
		}
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
	case SENSOR_SCENARIO_ID_CUSTOM1:
		frame_length = imgsensor_info.custom1.pclk / framerate * 10
				/ imgsensor_info.custom1.linelength;
		ctx->dummy_line =
			(frame_length > imgsensor_info.custom1.framelength)
			? (frame_length - imgsensor_info.custom1.framelength)
			: 0;
		ctx->frame_length =
			imgsensor_info.custom1.framelength
			+ ctx->dummy_line;
		ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
		break;
	case SENSOR_SCENARIO_ID_CUSTOM2:
		frame_length = imgsensor_info.custom2.pclk / framerate * 10
				/ imgsensor_info.custom2.linelength;
		ctx->dummy_line =
			(frame_length > imgsensor_info.custom2.framelength)
			? (frame_length - imgsensor_info.custom2.framelength)
			: 0;
		ctx->frame_length =
			imgsensor_info.custom2.framelength
			+ ctx->dummy_line;
		ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
		break;
	case SENSOR_SCENARIO_ID_CUSTOM3:
		frame_length = imgsensor_info.custom3.pclk / framerate * 10
				/ imgsensor_info.custom3.linelength;
		if(frame_length % 2)
		{
			frame_length++;
		}
		ctx->dummy_line =
			(frame_length > imgsensor_info.custom3.framelength)
			? (frame_length - imgsensor_info.custom3.framelength)
			: 0;
		ctx->frame_length =
			imgsensor_info.custom3.framelength
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
		LOG_DEBUG("error scenario_id = %d, we use preview scenario\n",
			scenario_id);
		break;
	}
	return ERROR_NONE;
}
static kal_uint32 get_default_framerate_by_scenario(struct subdrv_ctx *ctx,
		enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
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
	case SENSOR_SCENARIO_ID_CUSTOM1:
		*framerate = imgsensor_info.custom1.max_framerate;
		break;
	case SENSOR_SCENARIO_ID_CUSTOM2:
		*framerate = imgsensor_info.custom2.max_framerate;
		break;
	case SENSOR_SCENARIO_ID_CUSTOM3:
		*framerate = imgsensor_info.custom3.max_framerate;
		break;
	default:
		break;
	}
	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(struct subdrv_ctx *ctx, kal_uint32 modes)
{
    LOG_INF("set_test_pattern enum: %d, ctx->test_pattern = %d\n", modes, ctx->test_pattern);
    if (modes) {
           switch(modes) {
           case 5:
               write_cmos_sensor_8(ctx, 0x020E, 0x00);
               write_cmos_sensor_8(ctx, 0x0218, 0x00);
               write_cmos_sensor_8(ctx, 0x3015, 0x00);
               break;
           default:
               write_cmos_sensor_8(ctx, 0x0601, modes);
               break;
           }
       } else if (ctx->test_pattern) {
           write_cmos_sensor_8(ctx, 0x0601, 0x00); /*No pattern*/
           write_cmos_sensor_8(ctx, 0x020E, 0x01);
           write_cmos_sensor_8(ctx, 0x0218, 0x01);
           write_cmos_sensor_8(ctx, 0x3015, 0x40);
       }

    ctx->test_pattern = modes;
    return ERROR_NONE;
}

static kal_uint16 imx882txd_feedback_awbgain[] = {
	0x0104, 0x01,
	0x0B8E, 0x01,
	0x0B8F, 0x00,
	0x0B90, 0x02,
	0x0B91, 0x36,
	0x0B92, 0x01,
	0x0B93, 0x88,
	0x0B94, 0x01,
	0x0B95, 0x00,
	0x0104, 0x00,
};
static void feedback_awbgain(struct subdrv_ctx *ctx, kal_uint32 r_gain, kal_uint32 b_gain)
{
	UINT32 r_gain_int = 0;
	UINT32 b_gain_int = 0;
	LOG_DEBUG("r_gain = %d, b_gain = %d, ctx->sensor_mode = %d", r_gain, b_gain,ctx->sensor_mode);
	if (ctx->sensor_mode == IMGSENSOR_MODE_CUSTOM3) {
		r_gain_int = r_gain / 512;
		b_gain_int = b_gain / 512;
		imx882txd_feedback_awbgain[7] = r_gain_int;
		imx882txd_feedback_awbgain[9] = (r_gain - r_gain_int * 512) / 2;
		imx882txd_feedback_awbgain[11] = b_gain_int;
		imx882txd_feedback_awbgain[13] = (b_gain - b_gain_int * 512) / 2;

		imx882txd_table_write_cmos_sensor(ctx ,imx882txd_feedback_awbgain,
			sizeof(imx882txd_feedback_awbgain)/sizeof(kal_uint16));
	}
}
static int imx882txdmain_set_awb_gain(struct subdrv_ctx *ctx, u8 *para) {
	struct SET_SENSOR_AWB_GAIN *awb_gain = (struct SET_SENSOR_AWB_GAIN *)para;
	feedback_awbgain(ctx, awb_gain->ABS_GAIN_R, awb_gain->ABS_GAIN_B);
	return 0;
}
static int feature_control(struct subdrv_ctx *ctx, MSDK_SENSOR_FEATURE_ENUM feature_id,
				 UINT8 *feature_para, UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16 = (UINT16 *) feature_para;
	UINT16 *feature_data_16 = (UINT16 *) feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *) feature_para;
	UINT32 *feature_data_32 = (UINT32 *) feature_para;
	struct mtk_hdr_ae *ae_ctrl = NULL;
	uint32_t *pScenarios;
	unsigned long long *feature_data = (unsigned long long *) feature_para;
	/* unsigned long long *feature_return_para
	 *  = (unsigned long long *) feature_para;
	 */
	struct u64_min_max multi_exposure_shutter_range[5];
	struct SET_PD_BLOCK_INFO_T *PDAFinfo;
	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data
		= (MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;
	/*LOG_DEBUG("feature_id = %d\n", feature_id);*/
	switch (feature_id) {
	case SENSOR_FEATURE_GET_OUTPUT_FORMAT_BY_SCENARIO:
		switch (*feature_data) {
        case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
        case SENSOR_SCENARIO_ID_CUSTOM1:
			*(feature_data + 1)
			= (enum ACDK_SENSOR_OUTPUT_DATA_FORMAT_ENUM)
				imgsensor_info.sensor_output_dataformat;
			break;
		}
	break;
	case SENSOR_FEATURE_GET_ANA_GAIN_TABLE:
		switch (*feature_data) {
		case SENSOR_SCENARIO_ID_CUSTOM3:
			if ((void *)(uintptr_t) (*(feature_data + 1)) == NULL) {
				*(feature_data + 0) =
					sizeof(IMX882TXD_ana_0db_gain_table);
			} else {
				memcpy((void *)(uintptr_t) (*(feature_data + 1)),
				(void *)IMX882TXD_ana_0db_gain_table,
				sizeof(IMX882TXD_ana_0db_gain_table));
			}
		break;
		default :
			if ((void *)(uintptr_t) (*(feature_data + 1)) == NULL) {
				*(feature_data + 0) =
					sizeof(imx882txd_ana_64db_gain_table);
			} else {
				memcpy((void *)(uintptr_t) (*(feature_data + 1)),
				(void *)imx882txd_ana_64db_gain_table,
				sizeof(imx882txd_ana_64db_gain_table));
			}
			break;
		}
		break;
	case SENSOR_FEATURE_GET_GAIN_RANGE_BY_SCENARIO:
		switch (*feature_data) {
		case SENSOR_SCENARIO_ID_CUSTOM3:
			*(feature_data + 1) = 1024;
			*(feature_data + 2) = BASEGAIN * 16;
			break;
		default :
			*(feature_data + 1) = 1463;
			*(feature_data + 2) = BASEGAIN * 64;
			break;
		}
		break;
	case SENSOR_FEATURE_GET_BASE_GAIN_ISO_AND_STEP:
		*(feature_data + 0) = imgsensor_info.min_gain_iso;
		*(feature_data + 1) = imgsensor_info.gain_step;
		*(feature_data + 2) = imgsensor_info.gain_type;
		break;
	case SENSOR_FEATURE_GET_MIN_SHUTTER_BY_SCENARIO:
		switch (*feature_data) {
			case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO: // B3
				*(feature_data + 1) = 5;
				*(feature_data + 2) = 1;
				break;
			case SENSOR_SCENARIO_ID_SLIM_VIDEO: // V2
				*(feature_data + 1) = 12;
				*(feature_data + 2) = 8;
			break;
			case SENSOR_SCENARIO_ID_CUSTOM3: // V2
				*(feature_data + 1) = 10;
				*(feature_data + 2) = 2;
			break;
			default: // B1
				*(feature_data + 1) = 6;
				*(feature_data + 2) = 4;
			break;
		}
		break;
	case SENSOR_FEATURE_GET_OFFSET_TO_START_OF_EXPOSURE:
		*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 1200000;
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
		case SENSOR_SCENARIO_ID_CUSTOM1:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom1.pclk;
			break;
		case SENSOR_SCENARIO_ID_CUSTOM2:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom2.pclk;
			break;
		case SENSOR_SCENARIO_ID_CUSTOM3:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom3.pclk;
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
		case SENSOR_SCENARIO_ID_CUSTOM1:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom1.framelength << 16)
				+ imgsensor_info.custom1.linelength;
			break;
		case SENSOR_SCENARIO_ID_CUSTOM2:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom2.framelength << 16)
				+ imgsensor_info.custom2.linelength;
			break;
		case SENSOR_SCENARIO_ID_CUSTOM3:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom3.framelength << 16)
				+ imgsensor_info.custom3.linelength;
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
		 /* night_mode((BOOL) *feature_data); */
		break;
	#ifdef VENDOR_EDIT
	case SENSOR_FEATURE_CHECK_MODULE_ID:
		*feature_return_para_32 = imgsensor_info.module_id;
		break;
	#endif
	case SENSOR_FEATURE_SET_GAIN:
		set_gain(ctx, (UINT32) *feature_data);
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
		/*get the lens driver ID from EEPROM
		 * or just return LENS_DRIVER_ID_DO_NOT_CARE
		 * if EEPROM does not exist in camera module.
		 */
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
				      *(feature_data_16+1));
		break;
	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
		 set_max_framerate_by_scenario(ctx,
				(enum MSDK_SCENARIO_ID_ENUM)*feature_data,
				*(feature_data+1));
		break;
	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
		 get_default_framerate_by_scenario(ctx,
				(enum MSDK_SCENARIO_ID_ENUM)*(feature_data),
				(MUINT32 *)(uintptr_t)(*(feature_data+1)));
		break;
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
		/* for factory mode auto testing */
		*feature_return_para_32 = imgsensor_info.checksum_value;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_FRAMERATE:
		LOG_DEBUG("current fps :%d\n", (UINT32)*feature_data_32);
		ctx->current_fps = *feature_data_32;
		break;
	case SENSOR_FEATURE_SET_HDR:
		LOG_DEBUG("ihdr enable :%d\n", (BOOL)*feature_data_32);
		ctx->ihdr_mode = *feature_data_32;
		break;
	case SENSOR_FEATURE_GET_CROP_INFO:
		LOG_DEBUG("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n",
			(UINT32)*feature_data);
		wininfo =
	(struct SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));
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
		case SENSOR_SCENARIO_ID_CUSTOM1:
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[5],
			sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case SENSOR_SCENARIO_ID_CUSTOM2:
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[6],
			sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case SENSOR_SCENARIO_ID_CUSTOM3:
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[7],
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
		case SENSOR_FEATURE_GET_PDAF_INFO:
		    LOG_INF("SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%d\n",
		    (UINT16) *feature_data);
		    PDAFinfo =
		    (struct SET_PD_BLOCK_INFO_T *)(uintptr_t)(*(feature_data+1));
			switch (*feature_data) {
			case SENSOR_SCENARIO_ID_CUSTOM3:
			memcpy((void *)PDAFinfo, (void *)&imgsensor_pdseamless_info,
		    sizeof(struct SET_PD_BLOCK_INFO_T));
			break;
			case SENSOR_SCENARIO_ID_SLIM_VIDEO:
			memcpy((void *)PDAFinfo, (void *)&imgsensor_pdslim_info,
		    sizeof(struct SET_PD_BLOCK_INFO_T));
			break;
			default:
		    memcpy((void *)PDAFinfo, (void *)&imgsensor_pd_info,
		    sizeof(struct SET_PD_BLOCK_INFO_T));
			break;
			}
        break;
		case SENSOR_FEATURE_SET_PDAF:
        LOG_INF("PDAF mode :%d\n", *feature_data_16);
        ctx->pdaf_mode= *feature_data_16;
        break;
		case SENSOR_FEATURE_GET_PDAF_DATA:
        LOG_INF("SENSOR_FEATURE_GET_PDAF_DATA\n");
        break;
        case SENSOR_FEATURE_SET_TEST_PATTERN:
            set_test_pattern_mode(ctx, (UINT32) (*feature_data));
        break;
        case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
            LOG_INF(
            "SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%d\n",
                (UINT16) *feature_data);
        /*PDAF capacity enable or not, 2p8 only full size support PDAF*/
        switch (*feature_data) {
        case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
        case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
            *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
            break;
        case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
            /* video & capture use same setting */
            *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
            break;
        case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
            *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
            break;
        case SENSOR_SCENARIO_ID_SLIM_VIDEO:
            *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
            break;
        case SENSOR_SCENARIO_ID_CUSTOM1:
            *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
            break;
        case SENSOR_SCENARIO_ID_CUSTOM2:
            *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
            break;
        case SENSOR_SCENARIO_ID_CUSTOM3:
            *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
            break;
        case SENSOR_SCENARIO_ID_CUSTOM4:
            *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
            break;
        case SENSOR_SCENARIO_ID_CUSTOM5:
            *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
            break;
        case SENSOR_SCENARIO_ID_CUSTOM6:
            *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
            break;
        case SENSOR_SCENARIO_ID_CUSTOM7:
            *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
            break;
        case SENSOR_SCENARIO_ID_CUSTOM8:
            *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
            break;
        case SENSOR_SCENARIO_ID_CUSTOM9:
            *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
            break;
        case SENSOR_SCENARIO_ID_CUSTOM10:
            *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
            break;
        default:
            *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
            break;
        }
        break;
	case SENSOR_FEATURE_GET_BINNING_TYPE:
		switch (*(feature_data + 1)) {
        case SENSOR_SCENARIO_ID_CUSTOM3:
            *feature_return_para_32 = 1000; /*full size 1 : 1*/
            break;
		case SENSOR_SCENARIO_ID_CUSTOM1:
			*feature_return_para_32 = 1000; /*BINNING_NONE*/
			break;
		case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
		case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
		case SENSOR_SCENARIO_ID_SLIM_VIDEO:
		case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
		case SENSOR_SCENARIO_ID_CUSTOM2:
		case SENSOR_SCENARIO_ID_CUSTOM4:
		case SENSOR_SCENARIO_ID_CUSTOM5:
		default:
			*feature_return_para_32 = 1000; /*BINNING_AVERAGED*/
			break;
		}
		pr_debug("SENSOR_FEATURE_GET_BINNING_TYPE AE_binning_type:%d,\n",
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

	case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
		LOG_DEBUG("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",
			(UINT16)*feature_data,
			(UINT16)*(feature_data+1),
			(UINT16)*(feature_data+2));
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
		LOG_DEBUG("SENSOR_FEATURE_SET_HDR_SHUTTER LE=%d, SE=%d\n",
			(UINT16)*feature_data, (UINT16)*(feature_data+1));
		break;
	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
		LOG_DEBUG("SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
		streaming_control(ctx, KAL_FALSE);
		break;
	case SENSOR_FEATURE_SET_STREAMING_RESUME:
		LOG_DEBUG("SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%llu\n",
			*feature_data);
		if (*feature_data != 0)
			set_shutter(ctx, *feature_data);
		streaming_control(ctx, KAL_TRUE);
		break;
	case SENSOR_FEATURE_GET_AE_FRAME_MODE_FOR_LE:
		memcpy(feature_return_para_32,
		&ctx->ae_frm_mode, sizeof(struct IMGSENSOR_AE_FRM_MODE));
		break;
	case SENSOR_FEATURE_GET_AE_EFFECTIVE_FRAME_FOR_LE:
		*feature_return_para_32 = ctx->current_ae_effective_frame;
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
		case SENSOR_SCENARIO_ID_CUSTOM1:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom1.mipi_pixel_rate;
			break;
		case SENSOR_SCENARIO_ID_CUSTOM2:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom2.mipi_pixel_rate;
			break;
		case SENSOR_SCENARIO_ID_CUSTOM3:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom3.mipi_pixel_rate;
			break;
		case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.pre.mipi_pixel_rate;
			break;
		}
	}
	break;
    case SENSOR_FEATURE_SET_AWB_GAIN:
        /* modify to separate izoom and remosaic portrait 2x*/
        /*write AWB gain to sensor*/
        imx882txdmain_set_awb_gain(ctx, (u8 *)(feature_data));
	break;
	case SENSOR_FEATURE_SET_FRAMELENGTH:
		set_frame_length(ctx, (UINT16) (*feature_data));
		break;
	case SENSOR_FEATURE_SET_MULTI_SHUTTER_FRAME_TIME:
		LOG_DEBUG("SENSOR_FEATURE_SET_MULTI_SHUTTER_FRAME_TIME");
		set_multi_shutter_frame_length(ctx, (UINT32 *)(*feature_data),
					(UINT16) (*(feature_data + 1)),
					(UINT16) (*(feature_data + 2)));
		break;
	case SENSOR_FEATURE_SEAMLESS_SWITCH:
    {
        if ((feature_data + 1) != NULL) {
            ae_ctrl = (struct mtk_hdr_ae *)((uintptr_t)(*(feature_data + 1)));
        } else {
            pr_debug("warning! no ae_ctrl input");
        }
        if (feature_data == NULL) {
            pr_info("error! input scenario is null!");
            return ERROR_INVALID_SCENARIO_ID;
        }
        LOG_INF("call seamless_switch");
        seamless_switch(ctx,(*feature_data), ae_ctrl);
    }
    break;
    case SENSOR_FEATURE_GET_SEAMLESS_SCENARIOS:
        if ((feature_data + 1) != NULL) {
            pScenarios = (MUINT32 *)((uintptr_t)(*(feature_data + 1)));
        } else {
            pr_info("input pScenarios vector is NULL!\n");
            return ERROR_INVALID_SCENARIO_ID;
        }
        switch (*feature_data) {
        case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
            *pScenarios = SENSOR_SCENARIO_ID_CUSTOM3;
            break;
		case SENSOR_SCENARIO_ID_CUSTOM3:
            *pScenarios = SENSOR_SCENARIO_ID_NORMAL_PREVIEW;
            break;
        default:
            *pScenarios = 0xff;
            break;
        }
        // pr_debug("SENSOR_FEATURE_GET_SEAMLESS_SCENARIOS %d %d\n",
        // *feature_data, *pScenarios);
        break;
		case SENSOR_FEATURE_PRELOAD_EEPROM_DATA:
			/*get eeprom preloader data*/
			*feature_return_para_32 = ctx->is_read_preload_eeprom;
			*feature_para_len = 4;
			if (ctx->is_read_preload_eeprom != 1)
				read_sensor_Cali(ctx);
			break;
	default:
		break;
	}
	return ERROR_NONE;
} /* feature_control(ctx) */
static struct mtk_mbus_frame_desc_entry frame_desc_prev[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 4000,
			.vsize = 3000,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 4000,
			.vsize = 750,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.user_data_desc = VC_PDAF_STATS,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cap[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 4000,
			.vsize = 3000,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 4000,
			.vsize = 750,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.user_data_desc = VC_PDAF_STATS,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0900,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 0x1000,
			.vsize = 0x0240,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.user_data_desc = VC_PDAF_STATS,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_hs_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0900,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_slim_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 2048,
			.vsize = 1152,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 2048,
			.vsize = 288,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.user_data_desc = VC_PDAF_STATS,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cust1[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1980,
			.vsize = 0x1320,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cust2[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0cd0,
			.vsize = 0x0740,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cust3[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 4000,
			.vsize = 3000,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 2000,
			.vsize = 1500,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.user_data_desc = VC_PDAF_STATS,
		},
	},
};
static int get_frame_desc(struct subdrv_ctx *ctx,
		int scenario_id, struct mtk_mbus_frame_desc *fd)
{
	switch (scenario_id) {
	case SENSOR_SCENARIO_ID_CUSTOM1:
		fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
		fd->num_entries = ARRAY_SIZE(frame_desc_cust1);
		memcpy(fd->entry, frame_desc_cust1, sizeof(frame_desc_cust1));
		break;
	case SENSOR_SCENARIO_ID_CUSTOM2:
		fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
		fd->num_entries = ARRAY_SIZE(frame_desc_cust2);
		memcpy(fd->entry, frame_desc_cust2, sizeof(frame_desc_cust2));
		break;
	case SENSOR_SCENARIO_ID_CUSTOM3:
		fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
		fd->num_entries = ARRAY_SIZE(frame_desc_cust3);
		memcpy(fd->entry, frame_desc_cust3, sizeof(frame_desc_cust3));
		break;
	case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
		fd->num_entries = ARRAY_SIZE(frame_desc_prev);
		memcpy(fd->entry, frame_desc_prev, sizeof(frame_desc_prev));
		break;
	case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
		fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
		fd->num_entries = ARRAY_SIZE(frame_desc_cap);
		memcpy(fd->entry, frame_desc_cap, sizeof(frame_desc_cap));
		break;
	case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
		fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
		fd->num_entries = ARRAY_SIZE(frame_desc_vid);
		memcpy(fd->entry, frame_desc_vid, sizeof(frame_desc_vid));
		break;
	case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
		fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
		fd->num_entries = ARRAY_SIZE(frame_desc_hs_vid);
		memcpy(fd->entry, frame_desc_hs_vid, sizeof(frame_desc_hs_vid));
		break;
	case SENSOR_SCENARIO_ID_SLIM_VIDEO:
		fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
		fd->num_entries = ARRAY_SIZE(frame_desc_slim_vid);
		memcpy(fd->entry, frame_desc_slim_vid, sizeof(frame_desc_slim_vid));
		break;
	default:
		return -1;
	}
	return 0;
}

static const struct subdrv_ctx defctx = {
	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_max = BASEGAIN * 16,
	.ana_gain_min = BASEGAIN * 1,
	.ana_gain_step = 1,
	.exposure_def = 0x3D0,
	/* support long exposure at most 128 times) */
	.exposure_max = (0xffff * 128) - 32,
	.exposure_min = 2,
	.exposure_step = 1,
	.frame_time_delay_frame = 3,
	.margin = 64,
	.is_hflip = 1,
	.is_vflip = 1,
	.max_frame_length = 0xffff,
	.mirror = IMAGE_NORMAL,	/* mirrorflip information */
	.sensor_mode = IMGSENSOR_MODE_INIT,
	/* IMGSENSOR_MODE enum value,record current sensor mode,such as:
	 * INIT, Preview, Capture, Video,High Speed Video, Slim Video
	 */
	.shutter = 0x3D0,	/* current shutter */
	.gain = BASEGAIN * 4,		/* current gain */
	.dummy_pixel = 0,	/* current dummypixel */
	.dummy_line = 0,	/* current dummyline */
	.current_fps = 300,
	.autoflicker_en = KAL_FALSE,
	.test_pattern = KAL_FALSE,
	.current_scenario_id = SENSOR_SCENARIO_ID_NORMAL_PREVIEW,
	.ihdr_mode = 0, /* sensor need support LE, SE with HDR feature */
	.i2c_write_id = 0x34, /* record current sensor's i2c write id */
	.current_ae_effective_frame = 2,
	.ae_ctrl_gph_en = 0,
};
static int init_ctx(struct subdrv_ctx *ctx,
		struct i2c_client *i2c_client, u8 i2c_write_id)
{
	memcpy(ctx, &defctx, sizeof(*ctx));
	ctx->i2c_client = i2c_client;
	ctx->i2c_write_id = i2c_write_id;
	return 0;
}

static int get_csi_param(struct subdrv_ctx *ctx,
	enum SENSOR_SCENARIO_ID_ENUM scenario_id,
	struct mtk_csi_param *csi_param)
{
	switch (scenario_id) {
	case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
	case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
		csi_param->dphy_trail = 0x47;
		break;
	case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
	case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
	case SENSOR_SCENARIO_ID_CUSTOM1:
	case SENSOR_SCENARIO_ID_CUSTOM2:
		csi_param->legacy_phy = 0;
		csi_param->not_fixed_trail_settle = 0;
		csi_param->not_fixed_dphy_settle = 1;
		csi_param->dphy_data_settle = 0x13;
		csi_param->dphy_clk_settle = 0x13;
		csi_param->dphy_trail = 0x31;
		csi_param->dphy_csi2_resync_dmy_cycle = 0xF;
		break;
	case SENSOR_SCENARIO_ID_SLIM_VIDEO:
		csi_param->dphy_trail = 0x37;
		break;
	case SENSOR_SCENARIO_ID_CUSTOM3:
		csi_param->legacy_phy = 0;
		csi_param->not_fixed_trail_settle = 1;
		csi_param->not_fixed_dphy_settle = 1;
		switch (ctx->aov_csi_clk) {
		case 242:
			csi_param->dphy_data_settle = 0x13;
			csi_param->dphy_clk_settle = 0x13;
			csi_param->dphy_trail = 0x7F;
			csi_param->dphy_csi2_resync_dmy_cycle = 0x25;
			break;
		case 130:
			csi_param->dphy_data_settle = 0xA;
			csi_param->dphy_clk_settle = 0xA;
			csi_param->dphy_trail = 0x44;
			csi_param->dphy_csi2_resync_dmy_cycle = 0x14;
			break;
		}
		break;
	default:
		csi_param->legacy_phy = 0;
		csi_param->not_fixed_trail_settle = 0;
		csi_param->not_fixed_dphy_settle = 0;
		break;
	}
	return 0;
}

static int vsync_notify(struct subdrv_ctx *ctx,	unsigned int sof_cnt)
{
	if (ctx->fast_mode_on && (sof_cnt > ctx->ref_sof_cnt)) {
		ctx->fast_mode_on = FALSE;
		ctx->ref_sof_cnt = 0;
		LOG_DEBUG("seamless_switch disabled.");
		write_cmos_sensor_8(ctx, 0x3010, 0x00);
	}
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
	.get_frame_desc = get_frame_desc,
	.get_csi_param = get_csi_param,
	.vsync_notify = vsync_notify,
};
static struct subdrv_pw_seq_entry pw_seq[] = {
	{HW_ID_MCLK, 24, 0},
	{HW_ID_RST, 0, 0},
	{HW_ID_AFVDD, 2800000, 1},
	{HW_ID_AVDD, 2800000, 0},
	{HW_ID_DVDD, 1200000, 1},
	{HW_ID_DOVDD, 1800000, 1},
	{HW_ID_MCLK_DRIVING_CURRENT, 4, 5},
	{HW_ID_RST, 1, 1},
};
const struct subdrv_entry imx882txd_mipi_raw_entry = {
	.name = "imx882txd_mipi_raw",
	.id = IMX882TXD_SENSOR_ID,
	.pw_seq = pw_seq,
	.pw_seq_cnt = ARRAY_SIZE(pw_seq),
	.ops = &ops,
};
