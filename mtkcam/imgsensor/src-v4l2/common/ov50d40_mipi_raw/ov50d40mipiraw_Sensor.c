// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022 MediaTek Inc.

/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 ov50d40mipiraw_Sensor.c
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
#include "ov50d40mipiraw_Sensor.h"

#define LOG_TAG "[ov50d40]"
#define LOG_INF(format, args...) pr_info(LOG_TAG "[%s] " format, __func__, ##args)

//static void set_sensor_cali(void *arg);
//static int get_sensor_temperature(void *arg);
static void set_group_hold(void *arg, u8 en);
static void ov50d40_set_dummy(struct subdrv_ctx *ctx);
static int ov50d40_set_max_framerate_by_scenario(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int ov50d40_get_ana_gain_table(struct subdrv_ctx *ctx,u8 *para,u32 *len);
static u16 get_gain2reg(u32 gain);
//static int ov50d40_seamless_switch(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int ov50d40_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id);
static int ov50d40_get_imgsensor_id(struct subdrv_ctx *ctx, u32 *para);

/* STRUCT */

static struct subdrv_feature_control feature_control_list[] = {
	{SENSOR_FEATURE_SET_TEST_PATTERN, ov50d40_set_test_pattern},
	//{SENSOR_FEATURE_SEAMLESS_SWITCH, ov50d40_seamless_switch},
	{SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO, ov50d40_set_max_framerate_by_scenario},
	{SENSOR_FEATURE_GET_ANA_GAIN_TABLE,ov50d40_get_ana_gain_table},
};

// static struct eeprom_info_struct eeprom_info[] = {
// 	{
// 		.header_id = 0x010B00FF,
// 		.addr_header_id = 0x00000001,
// 		.i2c_write_id = 0xA0,

// 		.pdc_support = TRUE,
// 		.pdc_size = 728,
// 		.addr_pdc = 0x1638,
// 		.sensor_reg_addr_pdc = 0x5900,

// 		.xtalk_support = TRUE,
// 		.xtalk_table = data_xtalk_ov50d40,
// 		.xtalk_size = ARRAY_SIZE(data_xtalk_ov50d40),
// 		.addr_xtalk = PARAM_UNDEFINED,
// 		.sensor_reg_addr_xtalk = 0x53C0,
// 	},
// };

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info = {
	 .i4OffsetX = 64,
	 .i4OffsetY = 16,
	 .i4PitchX = 16,
	 .i4PitchY = 16,
	 .i4PairNum = 8,
	 .i4SubBlkW = 8,
	 .i4SubBlkH = 4,
	 .i4PosL = {{70,18}, {78,18}, {66,22}, {74,22},
		{70,26}, {78,26}, {66,30}, {74,30} },
	 .i4PosR = {{69,18}, {77,18}, {65,22}, {73,22},
		{69,26}, {77,26}, {65,30}, {73,30} },
	 .iMirrorFlip = 3,
	 .i4BlockNumX = 248,
	 .i4BlockNumY = 190,
	 .i4FullRawW = 4096,
	 .i4FullRawH = 3072,
	 .i4Crop = { {0, 0}, {0, 0}, {0, 384}, {0, 384}, {0, 0},
			 {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0} },
	 .i4VCPackNum = 2,
	 .sPDMapInfo[0] = {
		.i4PDPattern = 2,//LR non-interleaved
		.i4PDRepetition = 2,
		.i4PDOrder = {1},//L first
	},
};

// static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_vid = {
// 	 .i4OffsetX = 16,
// 	 .i4OffsetY = 4,
// 	 .i4PitchX = 16,
// 	 .i4PitchY = 16,
// 	 .i4PairNum = 8,
// 	 .i4SubBlkW = 8,
// 	 .i4SubBlkH = 4,
// 	 .i4PosL = {{23, 6}, {31, 6}, {19, 10}, {27, 10},
// 		{23, 14}, {31, 14}, {19, 18}, {27, 18} },
// 	 .i4PosR = {{22, 6}, {30, 6}, {18, 10}, {26, 10},
// 		{22, 14}, {30, 14}, {18, 18}, {26, 18} },
// 	 .iMirrorFlip = 0,
// 	 .i4BlockNumX = 248,
// 	 .i4BlockNumY = 162,
// 	 .i4Crop = { {0, 0}, {0, 0}, {0, 200}, {0, 0}, {0, 0},
// 			 {0, 0}, {80, 420}, {0, 0}, {0, 0}, {0, 0} },
// 	 .i4VCPackNum = 2,
// };

// static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_cus2 = {
// 	 .i4OffsetX = 16,
// 	 .i4OffsetY = 4,
// 	 .i4PitchX = 16,
// 	 .i4PitchY = 16,
// 	 .i4PairNum = 8,
// 	 .i4SubBlkW = 8,
// 	 .i4SubBlkH = 4,
// 	 .i4PosL = {{23, 6}, {31, 6}, {19, 10}, {27, 10},
// 		{23, 14}, {31, 14}, {19, 18}, {27, 18} },
// 	 .i4PosR = {{22, 6}, {30, 6}, {18, 10}, {26, 10},
// 		{22, 14}, {30, 14}, {18, 18}, {26, 18} },
// 	 .iMirrorFlip = 0,
// 	 .i4BlockNumX = 240,
// 	 .i4BlockNumY = 135,
// 	 .i4Crop = { {0, 0}, {0, 0}, {0, 200}, {0, 0}, {0, 0},
// 			 {0, 0}, {80, 420}, {0, 0}, {0, 0}, {0, 0} },
// 	 .i4VCPackNum = 2,
// };

static struct mtk_mbus_frame_desc_entry frame_desc_prev[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0C00,
			//.user_data_desc = VC_RAW_DATA,
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 0x01f0,
			.vsize = 0x05f0,
			.user_data_desc = VC_PDAF_STATS,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cap[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0C00,
			//.user_data_desc = VC_RAW_DATA,
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 0x01f0,
			.vsize = 0x05f0,
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
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 0x01f0,
			.vsize = 0x0480,
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
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 0x01f0,
			.vsize = 0x0480,
			.user_data_desc = VC_PDAF_STATS,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_slim_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0800,
			.vsize = 0x0600,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus1[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0780,
			.vsize = 0x0438,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus2[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0780,
			.vsize = 0x0438,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus3[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0f00,
			.vsize = 0x0870,
		},
	},
};

static struct subdrv_mode_struct mode_struct[] = {
	{
		.frame_desc = frame_desc_prev,
		.num_entries = ARRAY_SIZE(frame_desc_prev),
		.mode_setting_table = addr_data_pair_preview_ov50d40,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_preview_ov50d40),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 100000000,
		.linelength = 425,
		.framelength = 7840,
		.max_framerate = 300,
		.mipi_pixel_rate = 760800000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8192,
			.h0_size = 6144,
			.scale_w = 4096,
			.scale_h = 3072,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 3072,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 3072,
		},
		.aov_mode = 0,
		.s_dummy_support = 1,
		.ae_ctrl_support = 1,
		.pdaf_cap = true,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ana_gain_max = BASEGAIN * 62,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 3,
		.csi_param = {
			.dphy_trail = 98,
		},
	},
	{
		.frame_desc = frame_desc_cap,
		.num_entries = ARRAY_SIZE(frame_desc_cap),
		.mode_setting_table = addr_data_pair_capture_ov50d40,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_capture_ov50d40),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = addr_data_pair_capture_ov50d40,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(addr_data_pair_capture_ov50d40),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 100000000,
		.linelength = 425,
		.framelength = 7840,
		.max_framerate = 300,
		.mipi_pixel_rate = 760800000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8192,
			.h0_size = 6144,
			.scale_w = 4096,
			.scale_h = 3072,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 3072,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 3072,
		},
		.aov_mode = 0,
		.s_dummy_support = 1,
		.ae_ctrl_support = 1,
		.pdaf_cap = true,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ana_gain_max = BASEGAIN * 62,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 3,
		.csi_param = {
			.dphy_trail = 95,
		},
	},
	{
		.frame_desc = frame_desc_vid,
		.num_entries = ARRAY_SIZE(frame_desc_vid),
		.mode_setting_table = addr_data_pair_video_ov50d40,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_video_ov50d40),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 100000000,
		.linelength = 325,
		.framelength = 10256,
		.max_framerate = 300,
		.mipi_pixel_rate = 760800000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8192,
			.h0_size = 6144,
			.scale_w = 4096,
			.scale_h = 3072,
			.x1_offset = 0,
			.y1_offset = 384,
			.w1_size = 4096,
			.h1_size = 2304,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 2304,
		},
		.aov_mode = 0,
		.s_dummy_support = 1,
		.ae_ctrl_support = 1,
		.pdaf_cap = true,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ana_gain_max = BASEGAIN * 15.5,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.cphy_settle = 95,
		},
	},
	{
		.frame_desc = frame_desc_hs_vid,
		.num_entries = ARRAY_SIZE(frame_desc_hs_vid),
		.mode_setting_table = addr_data_pair_hs_video_ov50d40,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_hs_video_ov50d40),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 100000000,
		.linelength = 325,
		.framelength = 5128,
		.max_framerate = 600,
		.mipi_pixel_rate = 760800000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8192,
			.h0_size = 6144,
			.scale_w = 4096,
			.scale_h = 3072,
			.x1_offset = 0,
			.y1_offset = 384,
			.w1_size = 4096,
			.h1_size = 2304,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 2304,
		},
		.aov_mode = 0,
		.s_dummy_support = 1,
		.ae_ctrl_support = 1,
		.pdaf_cap = true,
		.imgsensor_pd_info = &imgsensor_pd_info,
		.ana_gain_max = BASEGAIN * 15.5,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.cphy_settle = 95,
		},
	},
	{
		.frame_desc = frame_desc_slim_vid,
		.num_entries = ARRAY_SIZE(frame_desc_slim_vid),
		.mode_setting_table = addr_data_pair_slim_video_ov50d40,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_slim_video_ov50d40),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 100000000,
		.linelength = 425,
		.framelength = 7842,
		.max_framerate = 300,
		.mipi_pixel_rate = 540000000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8192,
			.h0_size = 6144,
			.scale_w = 2048,
			.scale_h = 1536,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 2048,
			.h1_size = 1536,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2048,
			.h2_tg_size = 1536,
		},
		.aov_mode = 0,
		.s_dummy_support = 1,
		.ae_ctrl_support = 1,
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ana_gain_max = BASEGAIN * 62,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.cphy_settle = 95,
		},
	},
	{
		.frame_desc = frame_desc_cus1,
		.num_entries = ARRAY_SIZE(frame_desc_cus1),
		.mode_setting_table = addr_data_pair_custom1,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_custom1),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 100000000,
		.linelength = 425,
		.framelength = 3920,
		.max_framerate = 600,
		.mipi_pixel_rate = 540000000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8192,
			.h0_size = 6144,
			.scale_w = 4096,
			.scale_h = 3072,
			.x1_offset = 1088,
			.y1_offset = 996,
			.w1_size = 1920,
			.h1_size = 1080,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 1920,
			.h2_tg_size = 1080,
		},
		.aov_mode = 0,
		.s_dummy_support = 1,
		.ae_ctrl_support = 1,
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ana_gain_max = BASEGAIN * 62,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.cphy_settle = 98,
		},
	},
	{
		.frame_desc = frame_desc_cus2,
		.num_entries = ARRAY_SIZE(frame_desc_cus2),
		.mode_setting_table = addr_data_pair_custom2,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_custom2),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 100000000,
		.linelength = 425,
		.framelength = 7840,
		.max_framerate = 300,
		.mipi_pixel_rate = 540000000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8192,
			.h0_size = 6144,
			.scale_w = 4096,
			.scale_h = 3072,
			.x1_offset = 1088,
			.y1_offset = 996,
			.w1_size = 1920,
			.h1_size = 1080,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 1920,
			.h2_tg_size = 1080,
		},
		.aov_mode = 0,
		.s_dummy_support = 1,
		.ae_ctrl_support = 1,
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ana_gain_max = BASEGAIN * 62,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.cphy_settle = 98,
		},
	},
	{
		.frame_desc = frame_desc_cus3,
		.num_entries = ARRAY_SIZE(frame_desc_cus3),
		.mode_setting_table = addr_data_pair_custom3,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_custom3),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 100000000,
		.linelength = 425,
		.framelength = 7840,
		.max_framerate = 300,
		.mipi_pixel_rate = 760800000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8192,
			.h0_size = 6144,
			.scale_w = 4096,
			.scale_h = 3072,
			.x1_offset = 128,
			.y1_offset = 456,
			.w1_size = 3840,
			.h1_size = 2160,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3840,
			.h2_tg_size = 2160,
		},
		.aov_mode = 0,
		.s_dummy_support = 1,
		.ae_ctrl_support = 1,
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ana_gain_max = BASEGAIN * 62,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.cphy_settle = 98,
		},
	},
};

static struct subdrv_static_ctx static_ctx = {
	.sensor_id = OV50D40_SENSOR_ID,
	.reg_addr_sensor_id = {0x300A, 0x300B, 0x300C},
	.i2c_addr_table = {0x6C,0x44, 0xFF},
	.i2c_burst_write_support = TRUE,
	.i2c_transfer_data_type = I2C_DT_ADDR_16_DATA_8,
	// .eeprom_info = eeprom_info,
	// .eeprom_num = ARRAY_SIZE(eeprom_info),
	.resolution = {8192, 6144},
	.mirror = IMAGE_HV_MIRROR,

	.mclk = 24,
	.isp_driving_current = ISP_DRIVING_8MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2,
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.ob_pedestal = 0x40,

	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gr,
	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_min = BASEGAIN * 1,
	.ana_gain_max = BASEGAIN * 62,
	.ana_gain_type = 1,
	.ana_gain_step = 4,
	.ana_gain_table = ov50d40_62_ana_gain_table,
	.ana_gain_table_size = sizeof(ov50d40_62_ana_gain_table),
	.min_gain_iso = 100,
	.exposure_def = 0x3D0,
	.exposure_min = 20,
	.exposure_max = 0xFFFFFF - 32,
	.exposure_step = 2,
	.exposure_margin = 32,

	.frame_length_max = 0xFFFFFF,
	.ae_effective_frame = 2,
	.frame_time_delay_frame = 2,
	.start_exposure_offset = 1400000,

	.pdaf_type = PDAF_SUPPORT_CAMSV,
	.hdr_type = HDR_SUPPORT_NA,
	.seamless_switch_support = TRUE,
	.temperature_support = TRUE,
	.g_temp = PARAM_UNDEFINED,
	.g_gain2reg = get_gain2reg,
	.s_gph = set_group_hold,
	//.s_cali = set_sensor_cali,

	.reg_addr_stream = 0x0100,
	.reg_addr_mirror_flip = PARAM_UNDEFINED,
	.reg_addr_exposure = {{0x3500, 0x3501, 0x3502},},
	.long_exposure_support = FALSE,
	.reg_addr_exposure_lshift = PARAM_UNDEFINED,
	.reg_addr_ana_gain = {{0x3508, 0x3509},},
	.reg_addr_frame_length = {0x3840, 0x380E, 0x380F},
	.reg_addr_temp_en = 0x4D12,
	.reg_addr_temp_read = 0x4D13,
	.reg_addr_auto_extend = PARAM_UNDEFINED,
	.reg_addr_frame_count = 0x387F,

	.init_setting_table = addr_data_pair_init_ov50d40,
	.init_setting_len = ARRAY_SIZE(addr_data_pair_init_ov50d40),
	.mode = mode_struct,
	.sensor_mode_num = ARRAY_SIZE(mode_struct),
	.list = feature_control_list,
	.list_len = ARRAY_SIZE(feature_control_list),
	.chk_s_off_sta = 0,
	.chk_s_off_end = 0,

	.checksum_value = 0xafc54ca8,
	.aov_sensor_support = FALSE,
	.init_in_open = TRUE,
	.streaming_ctrl_imp = FALSE,
};

static struct subdrv_ops ops = {
	.get_id = ov50d40_get_imgsensor_id,
	//.get_id = common_get_imgsensor_id,
	.init_ctx = init_ctx,
	.open = common_open,
	.get_info = common_get_info,
	.get_resolution = common_get_resolution,
	.control = common_control,
	.feature_control = common_feature_control,
	.close = common_close,
	.get_frame_desc = common_get_frame_desc,
	.get_temp = common_get_temp,
	.get_csi_param = common_get_csi_param,
	.update_sof_cnt = common_update_sof_cnt,
};

static struct subdrv_pw_seq_entry pw_seq[] = {
	{HW_ID_MCLK, 24, 0},
	{HW_ID_RST, 0, 1},
	{HW_ID_MCLK_DRIVING_CURRENT, 8, 0},
	{HW_ID_AFVDD, 2800000, 1},
	{HW_ID_DOVDD, 1800000, 0}, // pmic_ldo/gpio(1.8V ldo) for dovdd
	{HW_ID_AVDD, 2800000, 0}, // pmic_ldo for avdd
	{HW_ID_DVDD, 1200000, 2}, // pmic_ldo for dvdd
	{HW_ID_RST, 1, 5},
};

const struct subdrv_entry ov50d40_mipi_raw_entry = {
	.name = "ov50d40_mipi_raw",
	.id = OV50D40_SENSOR_ID,
	.pw_seq = pw_seq,
	.pw_seq_cnt = ARRAY_SIZE(pw_seq),
	.ops = &ops,
};

/* STRUCT */

// static void set_sensor_cali(void *arg)
// {
// 	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;

// 	u16 idx = 0;
// 	u8 support = FALSE;
// 	u8 *pbuf = NULL;
// 	u16 size = 0;
// 	u16 addr = 0;
// 	struct eeprom_info_struct *info = ctx->s_ctx.eeprom_info;

// 	if (!probe_eeprom(ctx))
// 		return;

// 	idx = ctx->eeprom_index;

// 	/* PDC data */
// 	support = info[idx].pdc_support;
// 	if (support) {
// 		pbuf = info[idx].preload_pdc_table;
// 		if (pbuf != NULL) {
// 			size = 8;
// 			addr = 0x5C0E;
// 			subdrv_i2c_wr_seq_p8(ctx, addr, pbuf, size);
// 			pbuf += size;
// 			size = 720;
// 			addr = 0x5900;
// 			subdrv_i2c_wr_seq_p8(ctx, addr, pbuf, size);
// 			DRV_LOG(ctx, "set PDC calibration data done.");
// 		}
// 	}
// }

// static int get_sensor_temperature(void *arg)
// {
// 	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;
// 	int temperature = 0;

// 	/*TEMP_SEN_CTL */
// 	subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_temp_en, 0x01);
// 	temperature = subdrv_i2c_rd_u8(ctx, ctx->s_ctx.reg_addr_temp_read);
// 	temperature = (temperature > 0xC0) ? (temperature - 0x100) : temperature;

// 	DRV_LOG(ctx, "temperature: %d degrees\n", temperature);
// 	return temperature;
// }

static void set_group_hold(void *arg, u8 en)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;

	if (en) {
		set_i2c_buffer(ctx, 0x3208, 0x00);
	} else {
		set_i2c_buffer(ctx, 0x3208, 0x10);
		set_i2c_buffer(ctx, 0x3208, 0xA0);
	}
}

static void ov50d40_set_dummy(struct subdrv_ctx *ctx)
{
	// bool gph = !ctx->is_seamless && (ctx->s_ctx.s_gph != NULL);

	// if (gph)
	// ctx->s_ctx.s_gph((void *)ctx, 1);
	write_frame_length(ctx, ctx->frame_length);
	// if (gph)
	// ctx->s_ctx.s_gph((void *)ctx, 0);

	commit_i2c_buffer(ctx);
}

/* FUNCTION */
static int ov50d40_get_imgsensor_id(struct subdrv_ctx *ctx,u32 *para)
{
	u8 i = 0;
	u8 retry = 2;
	u32 addr_h = ctx->s_ctx.reg_addr_sensor_id.addr[0];
	u32 addr_l = ctx->s_ctx.reg_addr_sensor_id.addr[1];
	u32 addr_ll = ctx->s_ctx.reg_addr_sensor_id.addr[2];
	u8 module_id_addr = 0x01;
	char module_id;
	u32 sensor_id = *((u32 *)para);

	while (ctx->s_ctx.i2c_addr_table[i] != 0xFF) {
		ctx->i2c_write_id = ctx->s_ctx.i2c_addr_table[i];
		do {
			sensor_id = (subdrv_i2c_rd_u8(ctx, addr_h) << 8) |
				subdrv_i2c_rd_u8(ctx, addr_l);
			if (addr_ll)
				sensor_id = ((sensor_id) << 8) | subdrv_i2c_rd_u8(ctx, addr_ll);

			adaptor_i2c_rd_p8(ctx->i2c_client, 0xb0 >> 1, module_id_addr, &module_id, 1);

			LOG_INF("[ov50d40]: i2c write id: 0x%x, sensor id: 0x%x, module id: 0x%x\n",
					ctx->i2c_write_id, sensor_id, module_id);

			if (sensor_id == ctx->s_ctx.sensor_id && (module_id == 0x09)) {
				LOG_INF("[ov50d40]: i2c write id: 0x%x, sensor id: 0x%x, module id: 0x%x\n",
					ctx->i2c_write_id, sensor_id, module_id);
				return ERROR_NONE;
			}
			LOG_INF("[ov50d40]: Read sensor id fail! i2c write id: 0x%x, sensor id: 0x%x\n",
				ctx->i2c_write_id, sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		retry = 2;
	}

	if (sensor_id != ctx->s_ctx.sensor_id ||
		(module_id != 0x09)) {
		sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	return ERROR_NONE;
}

static int ov50d40_set_max_framerate_by_scenario(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64 *feature_data = (u64 *)para;
	enum SENSOR_SCENARIO_ID_ENUM scenario_id = (enum SENSOR_SCENARIO_ID_ENUM)*feature_data;
	u32 framerate = *(feature_data + 1);
	u32 frame_length;

	if (scenario_id >= ctx->s_ctx.sensor_mode_num) {
		DRV_LOG(ctx, "invalid sid:%u, mode_num:%u\n",
			scenario_id, ctx->s_ctx.sensor_mode_num);
		scenario_id = SENSOR_SCENARIO_ID_NORMAL_PREVIEW;
	}

	if (framerate == 0) {
		DRV_LOG(ctx, "framerate should not be 0\n");
		return ERROR_NONE;
	}

	if (ctx->s_ctx.mode[scenario_id].linelength == 0) {
		DRV_LOG(ctx, "linelength should not be 0\n");
		return ERROR_NONE;
	}

	if (ctx->line_length == 0) {
		DRV_LOG(ctx, "ctx->line_length should not be 0\n");
		return ERROR_NONE;
	}

	if (ctx->frame_length == 0) {
		DRV_LOG(ctx, "ctx->frame_length should not be 0\n");
		return ERROR_NONE;
	}

	frame_length = ctx->s_ctx.mode[scenario_id].pclk / framerate * 10
		/ ctx->s_ctx.mode[scenario_id].linelength;
	ctx->frame_length =
		max(frame_length, ctx->s_ctx.mode[scenario_id].framelength);
	ctx->frame_length = min(ctx->frame_length, ctx->s_ctx.frame_length_max);
	ctx->current_fps = ctx->pclk / ctx->frame_length * 10 / ctx->line_length;
	ctx->min_frame_length = ctx->frame_length;
	DRV_LOG(ctx, "max_fps(input/output):%u/%u(sid:%u), min_fl_en:1\n",
		framerate, ctx->current_fps, scenario_id);
	if (ctx->frame_length > (ctx->exposure[0] + ctx->s_ctx.exposure_margin))
		ov50d40_set_dummy(ctx);
	return ERROR_NONE;
}

static int ov50d40_get_ana_gain_table(struct subdrv_ctx *ctx,u8 *para,u32 *len)
{
	u64 *feature_data = (u64 *)para;
	enum SENSOR_SCENARIO_ID_ENUM scenario_id = (enum SENSOR_SCENARIO_ID_ENUM)*feature_data;
	pr_info("scenario_id =%d",scenario_id);
	switch (scenario_id) {
		case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
		case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
			if ((void *)(uintptr_t) (*(feature_data + 1)) == NULL) {
				*(feature_data + 0) =
					sizeof(ov50d40_ana_gain_table);
			} else {
				memcpy((void *)(uintptr_t) (*(feature_data + 1)),
				(void *)ov50d40_ana_gain_table,
				sizeof(ov50d40_ana_gain_table));
			}
		break;
		default :
		if ((void *)(uintptr_t) (*(feature_data + 1)) == NULL) {
				*(feature_data + 0) =
					sizeof(ov50d40_62_ana_gain_table);
			} else {
				memcpy((void *)(uintptr_t) (*(feature_data + 1)),
				(void *)ov50d40_62_ana_gain_table,
				sizeof(ov50d40_62_ana_gain_table));
			}
		break;
		}
	return ERROR_NONE;
}

static u16 get_gain2reg(u32 gain)
{
	return gain * 256 / BASEGAIN;
}

// static int ov50d40_seamless_switch(struct subdrv_ctx *ctx, u8 *para, u32 *len)
// {
// 	enum SENSOR_SCENARIO_ID_ENUM scenario_id;
// 	u32 *ae_ctrl = NULL;
// 	u64 *feature_data = (u64 *)para;

// 	if (feature_data == NULL) {
// 		DRV_LOGE(ctx, "input scenario is null!");
// 		return ERROR_NONE;
// 	}
// 	scenario_id = *feature_data;
// 	if ((feature_data + 1) != NULL)
// 		ae_ctrl = (u32 *)((uintptr_t)(*(feature_data + 1)));
// 	else
// 		DRV_LOGE(ctx, "no ae_ctrl input");

// 	check_current_scenario_id_bound(ctx);
// 	DRV_LOG(ctx, "E: set seamless switch %u %u\n", ctx->current_scenario_id, scenario_id);
// 	if (!ctx->extend_frame_length_en)
// 		DRV_LOGE(ctx, "please extend_frame_length before seamless_switch!\n");
// 	ctx->extend_frame_length_en = FALSE;

// 	if (scenario_id >= ctx->s_ctx.sensor_mode_num) {
// 		DRV_LOGE(ctx, "invalid sid:%u, mode_num:%u\n",
// 			scenario_id, ctx->s_ctx.sensor_mode_num);
// 		return ERROR_NONE;
// 	}
// 	if (ctx->s_ctx.mode[scenario_id].seamless_switch_group == 0 ||
// 		ctx->s_ctx.mode[scenario_id].seamless_switch_group !=
// 			ctx->s_ctx.mode[ctx->current_scenario_id].seamless_switch_group) {
// 		DRV_LOGE(ctx, "seamless_switch not supported\n");
// 		return ERROR_NONE;
// 	}
// 	if (ctx->s_ctx.mode[scenario_id].seamless_switch_mode_setting_table == NULL) {
// 		DRV_LOGE(ctx, "Please implement seamless_switch setting\n");
// 		return ERROR_NONE;
// 	}

// 	ctx->is_seamless = TRUE;
// 	update_mode_info(ctx, scenario_id);

// 	i2c_table_write(ctx, addr_data_pair_seamless_switch_step1_ov50d40,
// 		ARRAY_SIZE(addr_data_pair_seamless_switch_step1_ov50d40));
// 	i2c_table_write(ctx,
// 		ctx->s_ctx.mode[scenario_id].seamless_switch_mode_setting_table,
// 		ctx->s_ctx.mode[scenario_id].seamless_switch_mode_setting_len);
// 	if (ae_ctrl) {
// 		set_shutter(ctx, ae_ctrl[0]);
// 		set_gain(ctx, ae_ctrl[5]);
// 	}
// 	i2c_table_write(ctx, addr_data_pair_seamless_switch_step2_ov50d40,
// 		ARRAY_SIZE(addr_data_pair_seamless_switch_step2_ov50d40));
// 	if (ae_ctrl) {
// 		set_shutter(ctx, ae_ctrl[10]);
// 		set_gain(ctx, ae_ctrl[15]);
// 	}
// 	i2c_table_write(ctx, addr_data_pair_seamless_switch_step3_ov50d40,
// 		ARRAY_SIZE(addr_data_pair_seamless_switch_step3_ov50d40));

// 	ctx->is_seamless = FALSE;
// 	DRV_LOG(ctx, "X: set seamless switch done\n");
// 	return ERROR_NONE;
// }

static int ov50d40_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 mode = *((u32 *)para);

	if (mode != ctx->test_pattern)
		DRV_LOGE(ctx, "mode(%u->%u)\n", ctx->test_pattern, mode);
	/* 1:Solid Color 2:Color Bar 5:Black */
	switch (mode) {
	case PATTERN_MODE_COLOR_BAR:
		subdrv_i2c_wr_u8(ctx, 0x50c1, 0x01);
		break;
	case PATTERN_MODE_NO_ACCESS:
	case PATTERN_MODE_EXPERT:
		subdrv_i2c_wr_u8(ctx, 0x350a, 0x00);
		subdrv_i2c_wr_u8(ctx, 0x401a, 0x00);
		break;
	default:
		break;
	}

	if (mode != ctx->test_pattern)
		switch (ctx->test_pattern) {
		case PATTERN_MODE_COLOR_BAR:
			subdrv_i2c_wr_u8(ctx, 0x50c1, 0x00);
			break;
		case PATTERN_MODE_NO_ACCESS:
		case PATTERN_MODE_EXPERT:
			subdrv_i2c_wr_u8(ctx, 0x350a, 0x01);
			subdrv_i2c_wr_u8(ctx, 0x401a, 0x40);
			break;
		default:
			break;
		}

	ctx->test_pattern = mode;
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
