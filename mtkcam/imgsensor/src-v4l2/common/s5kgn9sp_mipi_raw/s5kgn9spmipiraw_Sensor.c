// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022 MediaTek Inc.

/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 s5kgn9spmipiraw_Sensor.c
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
#include "s5kgn9spmipiraw_Sensor.h"



static void set_sensor_cali(void *arg);
static u16 get_gain2reg(u32 gain);
static int s5kgn9sp_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int s5kgn9sp_set_test_pattern_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int s5kgn9sp_read_4cell_from_eeprom(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id);
int s5kgn9_common_open(struct subdrv_ctx *ctx);
// static int vsync_notify(struct subdrv_ctx *ctx,	unsigned int sof_cnt);

static struct subdrv_feature_control feature_control_list[] = {
	{SENSOR_FEATURE_SET_TEST_PATTERN, s5kgn9sp_set_test_pattern},
	{SENSOR_FEATURE_SET_TEST_PATTERN_DATA, s5kgn9sp_set_test_pattern_data},
	{SENSOR_FEATURE_GET_4CELL_DATA, s5kgn9sp_read_4cell_from_eeprom},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info = {
	.i4OffsetX = 8,
	.i4OffsetY = 8,
	.i4PitchX  = 8,
	.i4PitchY  = 8,
	.i4PairNum  =4,
	.i4SubBlkW  =8,
	.i4SubBlkH  =2,

	.i4PosL = {{9,8},{11,11},{15,12},{13,15}},
	.i4PosR = {{8,8},{10,11},{14,12},{12,15}},
	.iMirrorFlip = 3,
	.i4BlockNumX = 508,
	.i4BlockNumY = 382,
	.i4FullRawW = 4080,
	.i4FullRawH = 3072,
	.i4ModeIndex = 0,
	.i4Crop = {{0,0},{0,0},{0,388},{0,0},{1016,960},
	          {0,0},{1016,960},{1016,960},{0,388},{0,388}},
	.sPDMapInfo[0] = {
		.i4PDPattern = 3,//LR non-interleaved
		.i4PDRepetition = 2,
		.i4PDOrder = {1},//L first
	},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_slim_video = {
	.i4OffsetX = 4,
	.i4OffsetY = 4,
	.i4PitchX  = 8,
	.i4PitchY  = 8,
	.i4PairNum  =4,
	.i4SubBlkW  =4,
	.i4SubBlkH  =4,

	 .i4PosL = {{5, 4}, {9, 4}, {7, 8}, {11, 8}},
	.i4PosR = {{4, 4}, {8, 4}, {6, 8}, {10, 8}},
	.iMirrorFlip = 3,
	.i4BlockNumX = 254,
	.i4BlockNumY = 191,//143,
  .i4FullRawW = 2040,
	.i4FullRawH = 1536,
	// .i4ModeIndex = 0,
	.i4Crop = {{0,0},{0,0},{0,0},{0,0},{0,0},
	          {0,0},{0,0},{0,0},{0,0},{0,0}},
	.sPDMapInfo[0] = {
		.i4PDPattern = 3,//LR non-interleaved
		.i4PDRepetition = 2,
		.i4PDOrder = {1},//L first
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_prev[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0FF0,
			.vsize = 0x0C00,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2B,
			.hsize = 0x0200,
			.vsize = 0x0BF0,
			.user_data_desc = VC_PDAF_STATS,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cap[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0FF0,
			.vsize = 0x0C00,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2B,
			.hsize = 0x0200,
			.vsize = 0x0BF0,
			.user_data_desc = VC_PDAF_STATS,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0FF0,
			.vsize = 0x08F8,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2B,
			.hsize = 0x0200,
			.vsize = 0x08F0,
			.user_data_desc = VC_PDAF_STATS,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_hs_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0FF0,
			.vsize = 0x0C00,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2B,
			.hsize = 0x0200,
			.vsize = 0x0BF0,
			.user_data_desc = VC_PDAF_STATS,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_slim_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x09EC,
			.vsize = 0x0600,
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2B,
			.hsize = 0x0200,
			.vsize = 0x02f4,
			.user_data_desc = VC_PDAF_STATS,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cust1[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1FE0,
			.vsize = 0x1800,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cust2[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0800,
			.vsize = 0x0480,
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2B,
			.hsize = 0x0200,
			.vsize = 0x023C,
			.user_data_desc = VC_PDAF_STATS,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cust3[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0800,
			.vsize = 0x0480,
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2B,
			.hsize = 0x0200,
			.vsize = 0x023C,
			.user_data_desc = VC_PDAF_STATS,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cust4[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0FF0,
			.vsize = 0x08F8,
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2B,
			.hsize = 0x0200,
			.vsize = 0x08F0,
			.user_data_desc = VC_PDAF_STATS,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cust5[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0FF0,
			.vsize = 0x08F8,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cust6[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0AC0,
			.vsize = 0x0810,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};

static struct subdrv_mode_struct mode_struct[] = {
	{
		.frame_desc = frame_desc_prev,
		.num_entries = ARRAY_SIZE(frame_desc_prev),
		.mode_setting_table = s5kgn9sp_pre_setting,
		.mode_setting_len = ARRAY_SIZE(s5kgn9sp_pre_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1040000000,
		.linelength = 5280,
		.framelength = 6538,
		.max_framerate = 300,
		.mipi_pixel_rate = 1313280000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 8160,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8160,
			.h0_size = 6144,
			.scale_w = 4080,
			.scale_h = 3072,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4080,
			.h1_size = 3072,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4080,
			.h2_tg_size = 3072,
		},
		.pdaf_cap = true,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
		{
		.frame_desc = frame_desc_cap,
		.num_entries = ARRAY_SIZE(frame_desc_cap),
		.mode_setting_table = s5kgn9sp_cap_setting,
		.mode_setting_len = ARRAY_SIZE(s5kgn9sp_cap_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1040000000,
		.linelength = 5280,
		.framelength = 6538,
		.max_framerate = 300,
		.mipi_pixel_rate = 1313280000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 8160,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8160,
			.h0_size = 6144,
			.scale_w = 4080,
			.scale_h = 3072,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4080,
			.h1_size = 3072,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4080,
			.h2_tg_size = 3072,
		},
		.pdaf_cap = true,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
		{
		.frame_desc = frame_desc_vid,
		.num_entries = ARRAY_SIZE(frame_desc_vid),
		.mode_setting_table = s5kgn9sp_video_setting,
		.mode_setting_len = ARRAY_SIZE(s5kgn9sp_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1040000000,
		.linelength = 6816,
		.framelength = 5056,
		.max_framerate = 300,
		.mipi_pixel_rate = 1313280000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 8160,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8160,
			.h0_size = 6144,
			.scale_w = 4080,
			.scale_h = 3072,
			.x1_offset = 0,
			.y1_offset = 388,
			.w1_size = 4080,
			.h1_size = 2296,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4080,
			.h2_tg_size = 2296,
		},
		.pdaf_cap = true,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{
		.frame_desc = frame_desc_hs_vid,
		.num_entries = ARRAY_SIZE(frame_desc_hs_vid),
		.mode_setting_table = s5kgn9sp_hs_video_setting,
		.mode_setting_len = ARRAY_SIZE(s5kgn9sp_hs_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1040000000,
		.linelength = 5280,
		.framelength = 3274,
		.max_framerate = 600,
		.mipi_pixel_rate = 1324224000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 8160,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8160,
			.h0_size = 6144,
			.scale_w = 4080,
			.scale_h = 3072,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4080,
			.h1_size = 3072,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4080,
			.h2_tg_size = 3072,
		},
		.pdaf_cap = true,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{
		.frame_desc = frame_desc_slim_vid,
		.num_entries = ARRAY_SIZE(frame_desc_slim_vid),
		.mode_setting_table = s5kgn9sp_slim_video_setting,
		.mode_setting_len = ARRAY_SIZE(s5kgn9sp_slim_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1040000000,
		.linelength = 2816,
		.framelength = 3072,
		.max_framerate = 1200,
		.mipi_pixel_rate = 1324224000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 8160,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8160,
			.h0_size = 6144,
			.scale_w = 2040,
			.scale_h = 1536,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 2040,
			.h1_size = 1536,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2040,
			.h2_tg_size = 1536,
		},
		.pdaf_cap = true,
		.imgsensor_pd_info = &imgsensor_pd_info_slim_video,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{
		.frame_desc = frame_desc_cust1,
		.num_entries = ARRAY_SIZE(frame_desc_cust1),
		.mode_setting_table = s5kgn9sp_custom1_setting,
		.mode_setting_len = ARRAY_SIZE(s5kgn9sp_custom1_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1040000000,
		.linelength = 10944,
		.framelength = 6332,
		.max_framerate = 150,
		.mipi_pixel_rate = 1017792000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 8160,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8160,
			.h0_size = 6144,
			.scale_w = 8160,
			.scale_h = 6144,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 8160,
			.h1_size = 6144,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 8160,
			.h2_tg_size = 6144,
		},
		.pdaf_cap = true,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{
		.frame_desc = frame_desc_cust2,
		.num_entries = ARRAY_SIZE(frame_desc_cust2),
		.mode_setting_table = s5kgn9sp_custom2_setting,
		.mode_setting_len = ARRAY_SIZE(s5kgn9sp_custom2_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1040000000,
		.linelength = 2816,
		.framelength = 1528,
		.max_framerate = 2400,
		.mipi_pixel_rate = 1324224000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 8160,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8160,
			.h0_size = 6144,
			.scale_w = 4080,
			.scale_h = 3072,
			.x1_offset = 1016,
			.y1_offset = 960,
			.w1_size = 2048,
			.h1_size = 1152,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2048,
			.h2_tg_size = 1152,
		},
		.pdaf_cap = true,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{
		.frame_desc = frame_desc_cust3,
		.num_entries = ARRAY_SIZE(frame_desc_cust3),
		.mode_setting_table = s5kgn9sp_custom3_setting,
		.mode_setting_len = ARRAY_SIZE(s5kgn9sp_custom3_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1040000000,
		.linelength = 2816,
		.framelength = 6136,
		.max_framerate = 600,
		.mipi_pixel_rate = 1324224000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 8160,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8160,
			.h0_size = 6144,
			.scale_w = 4080,
			.scale_h = 3072,
			.x1_offset = 1016,
			.y1_offset = 960,
			.w1_size = 2048,
			.h1_size = 1152,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2048,
			.h2_tg_size = 1152,
		},
		.pdaf_cap = true,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{
		.frame_desc = frame_desc_cust4,
		.num_entries = ARRAY_SIZE(frame_desc_cust4),
		.mode_setting_table = s5kgn9sp_custom4_setting,
		.mode_setting_len = ARRAY_SIZE(s5kgn9sp_custom4_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1040000000,
		.linelength = 5280,
		.framelength = 3274,
		.max_framerate = 600,
		.mipi_pixel_rate = 1324224000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 8160,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8160,
			.h0_size = 6144,
			.scale_w = 4080,
			.scale_h = 3072,
			.x1_offset = 0,
			.y1_offset = 388,
			.w1_size = 4080,
			.h1_size = 2296,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4080,
			.h2_tg_size = 2296,
		},
		.pdaf_cap = true,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{
		.frame_desc = frame_desc_cust5,
		.num_entries = ARRAY_SIZE(frame_desc_cust5),
		.mode_setting_table = s5kgn9sp_custom5_setting,
		.mode_setting_len = ARRAY_SIZE(s5kgn9sp_custom5_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1040000000,
		.linelength = 5024,
		.framelength = 6844,
		.max_framerate = 300,
		.mipi_pixel_rate = 1324224000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 8160,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8160,
			.h0_size = 6144,
			.scale_w = 4080,
			.scale_h = 3072,
			.x1_offset = 0,
			.y1_offset = 388,
			.w1_size = 4080,
			.h1_size = 2296,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4080,
			.h2_tg_size = 2296,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{
		.frame_desc = frame_desc_cust6,
		.num_entries = ARRAY_SIZE(frame_desc_cust6),
		.mode_setting_table = s5kgn9sp_custom6_setting,
		.mode_setting_len = ARRAY_SIZE(s5kgn9sp_custom6_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1040000000,
		.linelength = 7584,
		.framelength = 4570,
		.max_framerate = 300,
		.mipi_pixel_rate = 1324224000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 8160,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8160,
			.h0_size = 6144,
			.scale_w = 4080,
			.scale_h = 3072,
			.x1_offset = 664,
			.y1_offset = 504,
			.w1_size = 2752,
			.h1_size = 2064,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2752,
			.h2_tg_size = 2064,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
 };

static struct subdrv_static_ctx static_ctx = {
	.sensor_id = S5KGN9SP_SENSOR_ID,
	.reg_addr_sensor_id = {0x0000, 0x0001},
	.i2c_addr_table = {0x5A, 0xFF},
	.i2c_burst_write_support = TRUE,
	.i2c_transfer_data_type = I2C_DT_ADDR_16_DATA_16,
	.eeprom_info = PARAM_UNDEFINED,
	.eeprom_num = PARAM_UNDEFINED,
	.resolution = {8160, 6144},
	.mirror = IMAGE_HV_MIRROR,

	.mclk = 24,
	.isp_driving_current = ISP_DRIVING_4MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_CPHY,
	.mipi_lane_num = SENSOR_MIPI_3_LANE,
	.ob_pedestal = 0x40,

	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_Gb,
	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_min = BASEGAIN * 1,
	.ana_gain_max = BASEGAIN * 64,
	.ana_gain_type = 2,
	.ana_gain_step = 2,
	.ana_gain_table = s5kgn9sp_ana_gain_table,
	.ana_gain_table_size = sizeof(s5kgn9sp_ana_gain_table),
	.min_gain_iso = 100,
	.exposure_def = 0x3D0,
	.exposure_min = 4,
	.exposure_max = (0xffff * 128) - 4,
	.exposure_step = 1,
	.exposure_margin = 32,
	.dig_gain_min = BASE_DGAIN * 1,
	.dig_gain_max = BASE_DGAIN * 64,
	.dig_gain_step = 2,
	// .saturation_info = &imgsensor_saturation_info_10bit,

	.frame_length_max = 0xFFFF,
	.ae_effective_frame = 2,
	.frame_time_delay_frame = 2,

	.pdaf_type = PDAF_SUPPORT_CAMSV,
	.hdr_type = HDR_SUPPORT_NA,
	.seamless_switch_support = FALSE,
	.g_gain2reg = get_gain2reg,
	.s_cali = set_sensor_cali,

	.reg_addr_stream = 0x0100,
	.reg_addr_mirror_flip = 0x0101,
	.reg_addr_exposure = {{0x0202, 0x0203},},
	.long_exposure_support = TRUE,
	.reg_addr_exposure_lshift = 0x0702,
	.reg_addr_ana_gain = {{0x0204, 0x0205},},
	.reg_addr_dig_gain = {{0x020E, 0x020F},},
	.reg_addr_frame_length = {{0x0340, 0x0341},},
	.reg_addr_temp_en = PARAM_UNDEFINED,
	.reg_addr_temp_read = PARAM_UNDEFINED,
	.reg_addr_auto_extend = PARAM_UNDEFINED,
	.reg_addr_frame_count = 0x0005,

	.init_setting_table = s5kgn9sp_init_setting,
	.init_setting_len = ARRAY_SIZE(s5kgn9sp_init_setting),
	.mode = mode_struct,
	.sensor_mode_num = ARRAY_SIZE(mode_struct),
	.list = feature_control_list,
	.list_len = ARRAY_SIZE(feature_control_list),
	.chk_s_off_sta = 1,
	.chk_s_off_end = 1,

	.checksum_value = 0xef23676a,
	.start_exposure_offset = 2000000,
};

static struct subdrv_ops ops = {
	.get_id = common_get_imgsensor_id,
	.init_ctx = init_ctx,
	.open = s5kgn9_common_open,
	.get_info = common_get_info,
	.get_resolution = common_get_resolution,
	.control = common_control,
	.feature_control = common_feature_control,
	.close = common_close,
	.get_frame_desc = common_get_frame_desc,
	//.get_temp = common_get_temp,
	//.get_csi_param = common_get_csi_param,
	// .vsync_notify = vsync_notify,
	//.update_sof_cnt = common_update_sof_cnt,
};

static struct subdrv_pw_seq_entry pw_seq[] = {
	{HW_ID_MCLK, 24, 0},
	{HW_ID_RST, 0, 0},
	{HW_ID_AFVDD, 2800000, 1},
	{HW_ID_DOVDD, 1800000, 1},
	//{HW_ID_PDN, 1, 3},
	{HW_ID_DVDD, 1050000, 1},
	{HW_ID_AVDD, 2800000, 2},
	{HW_ID_MCLK_DRIVING_CURRENT, 4, 5},
	{HW_ID_RST, 1, 1},
};
const struct subdrv_entry s5kgn9sp_mipi_raw_entry = {
	.name = "s5kgn9sp_mipi_raw",
	.id = S5KGN9SP_SENSOR_ID,
	.pw_seq = pw_seq,
	.pw_seq_cnt = ARRAY_SIZE(pw_seq),
	.ops = &ops,
};


/* FUNCTION */
static void s5kgn9_sensor_init(struct subdrv_ctx *ctx)
{
	/* initial sequence */
	// Convert from : "InitGlobal.sset"

	subdrv_i2c_wr_u16(ctx,0xFCFC,0x4000);
	subdrv_i2c_wr_u16(ctx,0x0000,0x0009);
	subdrv_i2c_wr_u16(ctx,0x6018,0x0001);
	mdelay(1);
	subdrv_i2c_wr_u16(ctx,0x7002,0x0008);
	subdrv_i2c_wr_u16(ctx,0x7004,0x1770);
	subdrv_i2c_wr_u16(ctx,0x70B8,0x0C82);

	subdrv_i2c_wr_u16(ctx,0x6014, 0x0001);
	mdelay(16);

	i2c_table_write(ctx, ctx->s_ctx.init_setting_table,ctx->s_ctx.init_setting_len);

}				/*      sensor_init  */

int s5kgn9_common_open(struct subdrv_ctx *ctx)
{
	u32 sensor_id = 0;
	u32 scenario_id = 0;

	/* get sensor id */
	if (common_get_imgsensor_id(ctx, &sensor_id) != ERROR_NONE)
		return ERROR_SENSOR_CONNECT_FAIL;
	/* initail setting */
	s5kgn9_sensor_init(ctx);

	memset(ctx->exposure, 0, sizeof(ctx->exposure));
	memset(ctx->ana_gain, 0, sizeof(ctx->gain));
	ctx->exposure[0] = ctx->s_ctx.exposure_def;
	ctx->ana_gain[0] = ctx->s_ctx.ana_gain_def;
	ctx->current_scenario_id = scenario_id;
	ctx->pclk = ctx->s_ctx.mode[scenario_id].pclk;
	ctx->line_length = ctx->s_ctx.mode[scenario_id].linelength;
	ctx->frame_length = ctx->s_ctx.mode[scenario_id].framelength;
	ctx->frame_length_rg = ctx->frame_length;
	ctx->current_fps = ctx->pclk / ctx->line_length * 10 / ctx->frame_length;
	ctx->readout_length = ctx->s_ctx.mode[scenario_id].readout_length;
	ctx->read_margin = ctx->s_ctx.mode[scenario_id].read_margin;
	ctx->min_frame_length = ctx->frame_length;
	ctx->autoflicker_en = FALSE;
	ctx->test_pattern = 0;
	ctx->ihdr_mode = 0;
	ctx->pdaf_mode = 0;
	ctx->hdr_mode = 0;
	ctx->extend_frame_length_en = 0;
	ctx->is_seamless = 0;
	ctx->fast_mode_on = 0;
	ctx->sof_cnt = 0;
	ctx->ref_sof_cnt = 0;
	ctx->is_streaming = 0;

	return ERROR_NONE;
}

static void set_sensor_cali(void *arg)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;

	u16 idx = 0;
	u8 support = FALSE;
	u8 *pbuf = NULL;
	u16 size = 0;
	u16 addr = 0;
	struct eeprom_info_struct *info = ctx->s_ctx.eeprom_info;

	if (!probe_eeprom(ctx))
		return;

	idx = ctx->eeprom_index;

	/* QSC data */
	support = info[idx].qsc_support;
	pbuf = info[idx].preload_qsc_table;
	size = info[idx].qsc_size;
	addr = info[idx].sensor_reg_addr_qsc;
	if (support) {
		if (pbuf != NULL && addr > 0 && size > 0) {
			subdrv_i2c_wr_seq_p8(ctx, addr, pbuf, size);
			subdrv_i2c_wr_u8(ctx, 0x3206, 0x01);
			DRV_LOG(ctx, "set QSC calibration data done.");
		} else {
			subdrv_i2c_wr_u8(ctx, 0x3206, 0x00);
		}
	}
}

static u16 get_gain2reg(u32 gain)
{
	kal_uint32 reg_gain = 0x0;

	reg_gain = gain * 32 / BASEGAIN;
	return (kal_uint32) reg_gain;
}

static int s5kgn9sp_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 mode = *((u32 *)para);

	if (mode != ctx->test_pattern)
		DRV_LOG(ctx, "mode(%u->%u)\n", ctx->test_pattern, mode);
	/* 1:Solid Color 2:Color Bar 5:Black */
	if (mode)
		subdrv_i2c_wr_u16(ctx, 0x0600, mode); /*100% Color bar*/
	else if (ctx->test_pattern)
		subdrv_i2c_wr_u16(ctx, 0x0600, 0x0000); /*No pattern*/

	ctx->test_pattern = mode;
	return ERROR_NONE;
}

static int s5kgn9sp_set_test_pattern_data(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	struct mtk_test_pattern_data *data = (struct mtk_test_pattern_data *)para;
	u16 R = (data->Channel_R >> 22) & 0x3ff;
	u16 Gr = (data->Channel_Gr >> 22) & 0x3ff;
	u16 Gb = (data->Channel_Gb >> 22) & 0x3ff;
	u16 B = (data->Channel_B >> 22) & 0x3ff;

	subdrv_i2c_wr_u16(ctx, 0x0602, Gr);
	subdrv_i2c_wr_u16(ctx, 0x0604, R);
	subdrv_i2c_wr_u16(ctx, 0x0606, B);
	subdrv_i2c_wr_u16(ctx, 0x0608, Gb);

	DRV_LOG(ctx, "mode(%u) R/Gr/Gb/B = 0x%04x/0x%04x/0x%04x/0x%04x\n",
		ctx->test_pattern, R, Gr, Gb, B);
	return ERROR_NONE;
}

static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id)
{
	memcpy(&(ctx->s_ctx), &static_ctx, sizeof(struct subdrv_static_ctx));
	subdrv_ctx_init(ctx);
	ctx->i2c_client = i2c_client;
	ctx->i2c_write_id = i2c_write_id;
	return 0;
}

#define FOUR_CELL_SIZE 666
#define FOUR_CELL_SIZE_GCC 346
#define FOUR_CELL_SIZE_XTC 320
#define EEPROM_READ_ID  0xA0
static int Is_Read_4Cell;
static char Four_Cell_Array[FOUR_CELL_SIZE + 2];
static int s5kgn9sp_read_4cell_from_eeprom(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64 *feature_data = (u64 *)para;
	u16 type = (kal_uint16)(*feature_data);
	char *data = (char *)(uintptr_t)(*(feature_data+1));
	int ret;
	int addr_gcc = 0x792;
	int addr_xtc = 0x8EE;
	char temp;

	if (type == FOUR_CELL_CAL_TYPE_XTALK_CAL) {
	    if (Is_Read_4Cell != 1) {
	        DRV_LOG(ctx,"Need to read i2C\n");

	        /* Check I2C is normal */
	        ret = adaptor_i2c_rd_u8(ctx->i2c_client,
	            EEPROM_READ_ID >> 1, addr_gcc, &temp);
	        if (ret != 0) {
	            DRV_LOG(ctx,"iReadRegI2C error\n");
	            return 0;
	        }

	        Four_Cell_Array[0] = (FOUR_CELL_SIZE & 0xff);/*Low*/
	        Four_Cell_Array[1] = ((FOUR_CELL_SIZE >> 8) & 0xff);/*High*/

	        /*Multi-Read*/
	        adaptor_i2c_rd_p8(ctx->i2c_client,
	            EEPROM_READ_ID >> 1, addr_xtc,
	            &Four_Cell_Array[2], FOUR_CELL_SIZE_XTC);

	        adaptor_i2c_rd_p8(ctx->i2c_client,
	            EEPROM_READ_ID >> 1, addr_gcc,
	            &Four_Cell_Array[322], FOUR_CELL_SIZE_GCC);

	        Is_Read_4Cell = 1;
	    }

	    if (data != NULL) {
	        DRV_LOG(ctx,"return data\n");
	        memcpy(data, Four_Cell_Array, FOUR_CELL_SIZE + 2);
	    }
	}
	return 0;
}