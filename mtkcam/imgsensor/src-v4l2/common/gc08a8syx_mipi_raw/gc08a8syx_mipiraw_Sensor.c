// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022 MediaTek Inc.

/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 gc08a8syx_mipiraw_Sensor.c
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
#include "gc08a8syx_mipiraw_Sensor.h"

#define LOG_TAG "[gc08a8syx]"
#define LOG_INF(format, args...) pr_info(LOG_TAG "[%s] " format, __func__, ##args)

static int gc08a8syx_set_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int gc08a8syx_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int gc08a8syx_set_shutter_frame_length(struct subdrv_ctx *ctx,u8 *para, u32 *len);
static int gc08a8syx_set_multi_shutter_frame_length(struct subdrv_ctx *ctx,u8 *para, u32 *len);
static int gc08a8syx_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int gc08a8syx_set_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int gc08a8syx_set_max_framerate_by_scenario(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id);
static int gc08a8syx_get_imgsensor_id(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int get_imgsensor_id(struct subdrv_ctx *ctx, u32 *para);
static int gc08a8syx_open(struct subdrv_ctx *ctx);

/* STRUCT */

static struct subdrv_feature_control feature_control_list[] = {
	{SENSOR_FEATURE_SET_TEST_PATTERN, gc08a8syx_set_test_pattern},
	{SENSOR_FEATURE_SET_GAIN, gc08a8syx_set_gain},
	{SENSOR_FEATURE_SET_ESHUTTER, gc08a8syx_set_shutter},
	{SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME, gc08a8syx_set_shutter_frame_length},
	{SENSOR_FEATURE_SET_MULTI_SHUTTER_FRAME_TIME,gc08a8syx_set_multi_shutter_frame_length},
	{SENSOR_FEATURE_SET_FRAMELENGTH, gc08a8syx_set_frame_length},
	{SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO, gc08a8syx_set_max_framerate_by_scenario},
	{SENSOR_FEATURE_CHECK_SENSOR_ID, gc08a8syx_get_imgsensor_id},
};

// 1000 base for dcg gain ratio
//static u32 gc08a8syx_dag_ratio_table_12bit[] = {4000};

static struct mtk_sensor_saturation_info imgsensor_saturation_info_10bit = {
	.gain_ratio = 1000,
	.OB_pedestal = 64,
	.saturation_level = 1023,
};

// static struct mtk_sensor_saturation_info imgsensor_saturation_info_12bit = {
// 	.gain_ratio = 4000,
// 	.OB_pedestal = 256,
// 	.saturation_level = 4095,
// };

static struct mtk_mbus_frame_desc_entry frame_desc_prev[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0CC0,  // 3264
			.vsize = 0x0990,  // 2448
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cap[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0CC0,  // 3264
			.vsize = 0x0990,  // 2448
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0CC0,  // 3264
			.vsize = 0x072C,  // 1836
		},
	},
};
// static struct mtk_mbus_frame_desc_entry frame_desc_hs_vid[] = {
// 	{
// 		.bus.csi2 = {
// 			.channel = 0,
// 			.data_type = 0x2c,
// 			.hsize = 0x1070,  // 4208
// 			.vsize = 0x0C30,  // 3120
// 		},
// 	},
//};

static struct subdrv_mode_struct mode_struct[] = {
	{
		.frame_desc = frame_desc_prev,
		.num_entries = ARRAY_SIZE(frame_desc_prev),
		.mode_setting_table = gc08a8syx_3264x2448_30fps_addr_data,
		.mode_setting_len = ARRAY_SIZE(gc08a8syx_3264x2448_30fps_addr_data),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 280000000,
		.linelength = 3640,
		.framelength = 2548,
		.max_framerate = 300,
		.mipi_pixel_rate = 268800000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 3264,
			.full_h = 2448,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 3264,
			.h0_size = 2448,
			.scale_w = 3264,
			.scale_h = 2448,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 3264,
			.h1_size = 2448,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 2448,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 85,
		},
	},
	{
		.frame_desc = frame_desc_cap,
		.num_entries = ARRAY_SIZE(frame_desc_cap),
		.mode_setting_table = gc08a8syx_3264x2448_30fps_addr_data,
		.mode_setting_len = ARRAY_SIZE(gc08a8syx_3264x2448_30fps_addr_data),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 280000000,
		.linelength = 3640,
		.framelength = 2548,
		.max_framerate = 300,
		.mipi_pixel_rate = 268800000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 3264,
			.full_h = 2448,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 3264,
			.h0_size = 2448,
			.scale_w = 3264,
			.scale_h = 2448,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 3264,
			.h1_size = 2448,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 2448,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 85,
		},
	},
	{
		.frame_desc = frame_desc_vid,
		.num_entries = ARRAY_SIZE(frame_desc_vid),
		.mode_setting_table = gc08a8syx_3264x1836_30fps_addr_data,
		.mode_setting_len = ARRAY_SIZE(gc08a8syx_3264x1836_30fps_addr_data),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 280000000,
		.linelength = 3640,
		.framelength = 2548,
		.max_framerate = 300,
		.mipi_pixel_rate = 268800000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 3264,
			.full_h = 2448,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 3264,
			.h0_size = 2448,
			.scale_w = 3264,
			.scale_h = 2448,
			.x1_offset = 0,
			.y1_offset = 306,
			.w1_size = 3264,
			.h1_size = 1836,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 1836,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 85,
		},
	},
	// {
	// 	.frame_desc = frame_desc_hs_vid,
	// 	.num_entries = ARRAY_SIZE(frame_desc_hs_vid),
	// 	.mode_setting_table = gc08a8syx_3264x1836_30fps_addr_data,
	// 	.mode_setting_len = ARRAY_SIZE(gc08a8syx_3264x1836_30fps_addr_data),
	// 	.seamless_switch_group = PARAM_UNDEFINED,
	// 	.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
	// 	.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
	// 	.hdr_mode = HDR_RAW_DCG_COMPOSE,
	// 	.raw_cnt = 1,
	// 	.exp_cnt = 2,
	// 	.pclk = 412000000,
	// 	.linelength = 4224,
	// 	.framelength = 3248,
	// 	.max_framerate = 300,
	// 	.mipi_pixel_rate = 474666667,
	// 	.readout_length = 0,
	// 	.read_margin = 0,
	// 	.imgsensor_winsize_info = {
	// 		.full_w = 4208,
	// 		.full_h = 3120,
	// 		.x0_offset = 0,
	// 		.y0_offset = 0,
	// 		.w0_size = 4208,
	// 		.h0_size = 3120,
	// 		.scale_w = 4208,
	// 		.scale_h = 3120,
	// 		.x1_offset = 0,
	// 		.y1_offset = 0,
	// 		.w1_size = 4208,
	// 		.h1_size = 3120,
	// 		.x2_tg_offset = 0,
	// 		.y2_tg_offset = 0,
	// 		.w2_tg_size = 4208,
	// 		.h2_tg_size = 3120,
	// 	},
	// 	.pdaf_cap = FALSE,
	// 	.imgsensor_pd_info = PARAM_UNDEFINED,
	// 	.ae_binning_ratio = 1000,
	// 	.fine_integ_line = 0,
	// 	.delay_frame = 2,
	// 	.csi_param = {
	// 		.dphy_trail  = 98,
	// 	},
	// 	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW12_Gr,
	// 	.saturation_info = &imgsensor_saturation_info_12bit,
	// 	.dcg_info = {
	// 		.dcg_mode = IMGSENSOR_DCG_COMPOSE,
	// 		.dcg_gain_mode = IMGSENSOR_DCG_RATIO_MODE,
	// 		.dcg_gain_ratio_min = 4000,
	// 		.dcg_gain_ratio_max = 4000,
	// 		.dcg_gain_ratio_step = 0,
	// 		.dcg_gain_table = gc08a8syx_dag_ratio_table_12bit,
	// 		.dcg_gain_table_size = sizeof(gc08a8syx_dag_ratio_table_12bit),
	// 	},
	// },
};

static struct subdrv_static_ctx static_ctx = {
	.sensor_id = GC08A8SYX_SENSOR_ID,
	.reg_addr_sensor_id = {0x03F0, 0x03F1},
	.i2c_addr_table = {0x62, 0xFF},
	.i2c_burst_write_support = TRUE,
	.i2c_transfer_data_type = I2C_DT_ADDR_16_DATA_8,
	.eeprom_info = 0,
	.eeprom_num = 0,
	.resolution = {3264, 2448},
	.mirror = IMAGE_NORMAL,

	.mclk = 24,
	.isp_driving_current = ISP_DRIVING_6MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2,
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.ob_pedestal = 0x40,

	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_R,
	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_min = BASEGAIN * 1,
	.ana_gain_max = BASEGAIN * 16,
	.ana_gain_type = 4,
	.ana_gain_step = 1,
	.ana_gain_table = gc08a8syx_ana_gain_table,
	.ana_gain_table_size = sizeof(gc08a8syx_ana_gain_table),
	.min_gain_iso = 100,
	.exposure_def = 0x3D0,
	.exposure_min = 4,
	.exposure_max = 0xFFFF - 16,
	.exposure_step = 4,
	.exposure_margin = 16,
	.saturation_info = &imgsensor_saturation_info_10bit,

	.frame_length_max = 0xFFFF,
	.ae_effective_frame = 2,
	.frame_time_delay_frame = 2,
	.start_exposure_offset = 0,

	.pdaf_type = PDAF_SUPPORT_NA,
	.hdr_type = HDR_SUPPORT_DCG,
	.seamless_switch_support = FALSE,
	.temperature_support = FALSE,
	.g_temp = PARAM_UNDEFINED,
	.g_gain2reg = PARAM_UNDEFINED,
	.s_gph = PARAM_UNDEFINED,
	.s_cali = PARAM_UNDEFINED,

	.reg_addr_stream = 0x0100,
	.reg_addr_mirror_flip = PARAM_UNDEFINED,
	.reg_addr_exposure = {{0x0202, 0x0203},},
	.long_exposure_support = TRUE,
	.reg_addr_exposure_lshift = PARAM_UNDEFINED,
	.reg_addr_ana_gain = {{0x0204, 0x0205},},
	.reg_addr_frame_length = {0x0340, 0x0341},
	.reg_addr_temp_en = PARAM_UNDEFINED,
	.reg_addr_temp_read = PARAM_UNDEFINED,
	.reg_addr_auto_extend = PARAM_UNDEFINED,
	.reg_addr_frame_count = PARAM_UNDEFINED,
	.init_setting_table = gc08a8syx_init_setting,
	.init_setting_len = ARRAY_SIZE(gc08a8syx_init_setting),
	.mode = mode_struct,
	.sensor_mode_num = ARRAY_SIZE(mode_struct),
	.list = feature_control_list,
	.list_len = ARRAY_SIZE(feature_control_list),

	.checksum_value = 0xffffffff,
	/* custom stream control delay timing for hw limitation (ms) */
	//.custom_stream_ctrl_delay = TRUE,
};

static struct subdrv_ops ops = {
	.get_id = get_imgsensor_id,
	// .get_id = common_get_imgsensor_id,
	.init_ctx = init_ctx,
	// .open = common_open,
	.open =gc08a8syx_open,
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
	{HW_ID_MCLK,  24,      0},
	{HW_ID_MCLK_DRIVING_CURRENT, 6, 1},
	{HW_ID_RST,   0,       1},
	{HW_ID_DOVDD, 1800000, 1},
	{HW_ID_DVDD,  1230000, 1},
	{HW_ID_AVDD,  2800000, 2},
	{HW_ID_RST,   1,       5},
};

const struct subdrv_entry gc08a8syx_mipi_raw_entry = {
	.name = "gc08a8syx_mipi_raw",
	.id = 	GC08A8SYX_SENSOR_ID,
	.pw_seq = pw_seq,
	.pw_seq_cnt = ARRAY_SIZE(pw_seq),
	.ops = &ops,
};

/* FUNCTION */

static int gc08a8syx_open(struct subdrv_ctx *ctx)
{
	u32 sensor_id = 0;
	u32 scenario_id = 0;

	/* get sensor id */
	if (get_imgsensor_id(ctx, &sensor_id) != ERROR_NONE)
		return ERROR_SENSOR_CONNECT_FAIL;
	/* initail setting */
	if (ctx->s_ctx.aov_sensor_support && !ctx->s_ctx.init_in_open)
		DRV_LOG_MUST(ctx, "sensor init not in open stage!\n");
	else
		sensor_init(ctx);

	if (ctx->s_ctx.s_cali != NULL)
		ctx->s_ctx.s_cali((void *) ctx);
	else
		write_sensor_Cali(ctx);

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
	if (ctx->s_ctx.mode[ctx->current_scenario_id].hdr_mode == HDR_RAW_LBMF) {
		memset(ctx->frame_length_in_lut, 0,
			sizeof(ctx->frame_length_in_lut));

		switch (ctx->s_ctx.mode[ctx->current_scenario_id].exp_cnt) {
		case 2:
			ctx->frame_length_in_lut[0] = ctx->readout_length + ctx->read_margin;
			ctx->frame_length_in_lut[1] = ctx->frame_length -
				ctx->frame_length_in_lut[0];
			break;
		case 3:
			ctx->frame_length_in_lut[0] = ctx->readout_length + ctx->read_margin;
			ctx->frame_length_in_lut[1] = ctx->readout_length + ctx->read_margin;
			ctx->frame_length_in_lut[2] = ctx->frame_length -
				ctx->frame_length_in_lut[1] - ctx->frame_length_in_lut[0];
			break;
		default:
			break;
		}

		memcpy(ctx->frame_length_in_lut_rg, ctx->frame_length_in_lut,
			sizeof(ctx->frame_length_in_lut_rg));
	}

	return ERROR_NONE;
}


static int gc08a8syx_get_imgsensor_id(struct subdrv_ctx *ctx, u8 *para, u32 *len)
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

			sensor_id = sensor_id + 1;

			adaptor_i2c_rd_p8(ctx->i2c_client, 0xa0 >> 1, module_id_addr, &module_id, 1);

			LOG_INF("[gc08a8syx]: i2c write id: 0x%x, sensor id: 0x%x, module id: 0x%x\n",
					ctx->i2c_write_id, sensor_id, module_id);
			if (sensor_id == ctx->s_ctx.sensor_id && (module_id == 0x01)) {
				LOG_INF("[gc08a8syx]: i2c write id: 0x%x, sensor id: 0x%x, module id: 0x%x\n",
					ctx->i2c_write_id, sensor_id, module_id);
				return ERROR_NONE;
			}
			LOG_INF("[gc08a8syx]: Read sensor id fail! i2c write id: 0x%x, sensor id: 0x%x\n",
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

static int get_imgsensor_id(struct subdrv_ctx *ctx,u32 *para)
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
			sensor_id = sensor_id + 2;

			adaptor_i2c_rd_p8(ctx->i2c_client, 0xa0 >> 1, module_id_addr, &module_id, 1);

			LOG_INF("[gc08a8syx]: i2c write id: 0x%x, sensor id: 0x%x, module id: 0x%x\n",
					ctx->i2c_write_id, sensor_id, module_id);
			if (sensor_id == ctx->s_ctx.sensor_id && (module_id == 0x01)) {
				LOG_INF("[gc08a8syx]: i2c write id: 0x%x, sensor id: 0x%x, module id: 0x%x\n",
					ctx->i2c_write_id, sensor_id, module_id);
				return ERROR_NONE;
			}
			LOG_INF("[gc08a8syx]: Read sensor id fail! i2c write id: 0x%x, sensor id: 0x%x\n",
				ctx->i2c_write_id, sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		retry = 2;
	}

	if (sensor_id != ctx->s_ctx.sensor_id ||
		(module_id != 0x01)) {
		sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	return ERROR_NONE;
}

void gc08a8syx_set_dummy(struct subdrv_ctx *ctx)
{
	LOG_INF("frame_length = %d\n", ctx->frame_length);
	subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_frame_length.addr[0], ctx->frame_length >> 8);
	subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_frame_length.addr[1], ctx->frame_length & 0xFF);
}	/*	set_dummy  */

static void gc08a8syx_set_max_framerate(struct subdrv_ctx *ctx, UINT16 framerate, kal_bool min_framelength_en)
{
	/*kal_int16 dummy_line;*/
	kal_uint32 frame_length = ctx->frame_length;
	LOG_INF("framerate = %d, min framelength should enable %d\n", framerate,
		min_framelength_en);
	frame_length = ctx->pclk / framerate * 10 / ctx->line_length;
	if (frame_length >= ctx->min_frame_length)
		ctx->frame_length = frame_length;
	else
		ctx->frame_length = ctx->min_frame_length;
	ctx->dummy_line =
			ctx->frame_length - ctx->min_frame_length;
	if (ctx->frame_length > ctx->s_ctx.frame_length_max) {
		ctx->frame_length = ctx->s_ctx.frame_length_max;
		ctx->dummy_line =
			ctx->frame_length - ctx->min_frame_length;
	}
	if (min_framelength_en)
		ctx->min_frame_length = ctx->frame_length;
	gc08a8syx_set_dummy(ctx);
}	/*	set_max_framerate  */

static int gc08a8syx_set_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 gain = *((u32 *)para);
	u32 rg_gain;

	LOG_INF("platform_gain = 0x%x \n", gain);
	/* check boundary of gain */
	gain = max(gain, ctx->s_ctx.ana_gain_min);
	gain = min(gain, ctx->s_ctx.ana_gain_max);
	rg_gain = gain;
	/* restore gain */
	memset(ctx->ana_gain, 0, sizeof(ctx->ana_gain));
	ctx->ana_gain[0] = gain;
	/* write gain */
	subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_ana_gain[0].addr[0],
		(rg_gain >> 8) & 0xFF);
	subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_ana_gain[0].addr[1],
		(rg_gain) & 0xFF);
	// subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_ana_gain[0].addr[2],
	// 	rg_gain & 0xFF);

	LOG_INF( "08a8xl_rg_gain = 0x%x \n", rg_gain);
	return ERROR_NONE;
}

static int gc08a8syx_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	return gc08a8syx_set_shutter_frame_length(ctx, para, len);
}

static int gc08a8syx_set_shutter_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64 *feature_data = (u64 *)para;
	u32 shutter = *feature_data;
	u32 frame_length = *(feature_data + 1);
	u32 fine_integ_line = 0;
	static int longexposue = 0;
	u32 fll = 0;
	u32 fll_step = 0;
	u32 dol_cnt = 1;
	u32 long_exp_h = 0;
	u32 long_exp_m = 0;
	u32 long_exp_l = 0;
	u32 cal_shutter = 0;


	DRV_LOGE(ctx, "gc08a8_shutter = 0x%x \n", shutter);
	DRV_LOGE(ctx, "gc08a8_frame_length = 0x%x \n", frame_length);
	ctx->frame_length = frame_length ? frame_length : ctx->frame_length;
	check_current_scenario_id_bound(ctx);
	/* check boundary of framelength */
	ctx->frame_length =	max(shutter + ctx->s_ctx.exposure_margin, ctx->min_frame_length);
	ctx->frame_length =	min(ctx->frame_length, ctx->s_ctx.frame_length_max);
	/* check boundary of shutter */
	fine_integ_line = ctx->s_ctx.mode[ctx->current_scenario_id].fine_integ_line;
	shutter = FINE_INTEG_CONVERT(shutter, fine_integ_line);
	shutter = max(shutter, ctx->s_ctx.exposure_min);
	/* restore shutter */
	memset(ctx->exposure, 0, sizeof(ctx->exposure));
	ctx->exposure[0] = shutter;
	/* set_long_exposure */
	if (ctx->s_ctx.long_exposure_support == TRUE) {
		if (shutter > 0xffee) {
			DRV_LOGE(ctx, "gc08a8 enter long exposure!");
			longexposue = 1;
			//DRV_LOGE(ctx, "0_shutter = 0x%x \n", shutter);
			cal_shutter = (shutter - 0xa00)/4 - 1;
			//DRV_LOGE(ctx, "1_shutter = 0x%x \n", cal_shutter);
			long_exp_h = (cal_shutter >> 16) & 0xF;
			long_exp_m = (cal_shutter >> 8 ) & 0xFF;
			long_exp_l = cal_shutter & 0xFF;
			//DRV_LOGE(ctx, "0_shutter = 0x%x \n", shutter);
			subdrv_i2c_wr_u8(ctx, 0x0202, 0x0a);
			//DRV_LOGE(ctx, "1_shutter = 0x%x \n",subdrv_i2c_rd_u8(ctx, 0x0202));
			subdrv_i2c_wr_u8(ctx, 0x0203, 0x00);
			subdrv_i2c_wr_u8(ctx, 0x0340, 0x0a);
			subdrv_i2c_wr_u8(ctx, 0x0341, 0x10);
			subdrv_i2c_wr_u8(ctx, 0x022f, long_exp_l);
			subdrv_i2c_wr_u8(ctx, 0x022e, long_exp_m);
			subdrv_i2c_wr_u8(ctx, 0x022d, (0x30 | long_exp_h));
			//DRV_LOGE(ctx, "long_shutter = 0x%x/0x%x/0x%x \n",subdrv_i2c_rd_u8(ctx, 0x022d),subdrv_i2c_rd_u8(ctx, 0x022e),subdrv_i2c_rd_u8(ctx, 0x022f) );
		} else if (longexposue == 1) {
				DRV_LOGE(ctx, "gc08a8 exit long exposure!");
				subdrv_i2c_wr_u8(ctx, 0x0202, 0x0a);
				//DRV_LOGE(ctx, "1_shutter = 0x%x \n",subdrv_i2c_rd_u8(ctx, 0x0202));
				subdrv_i2c_wr_u8(ctx, 0x0203, 0x00);
				subdrv_i2c_wr_u8(ctx, 0x0340, 0x0a);
				subdrv_i2c_wr_u8(ctx, 0x0341, 0x10);
				subdrv_i2c_wr_u8(ctx, 0x022d, 0x20);
				subdrv_i2c_wr_u8(ctx, 0x022e, 0x00);
				subdrv_i2c_wr_u8(ctx, 0x022f, 0x00);
				longexposue = 0;
		}
		else
		{
			if (set_auto_flicker(ctx, 0) || frame_length ||
			!ctx->s_ctx.reg_addr_auto_extend) {
				fll = ctx->frame_length;
				fll_step = ctx->s_ctx.mode[ctx->current_scenario_id].framelength_step;

				if (fll_step)
				fll = roundup(fll, fll_step);

				ctx->frame_length = fll;

				if (ctx->s_ctx.mode[ctx->current_scenario_id].hdr_mode == HDR_RAW_STAGGER)
					dol_cnt = ctx->s_ctx.mode[ctx->current_scenario_id].exp_cnt;

				fll = fll / dol_cnt;
				if (ctx->extend_frame_length_en == FALSE) {
						subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_frame_length.addr[0],
						(fll >> 8) & 0xFF);
						subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_frame_length.addr[1],
						fll & 0xFF);
				}
			}
				subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_exposure[0].addr[0],(ctx->exposure[0] >> 8) & 0xFF);
				subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_exposure[0].addr[1], ctx->exposure[0] & 0xFF);
				// subdrv_i2c_wr_u8(ctx, 0x0340, (ctx->frame_length >> 8) & 0xFF );
				// subdrv_i2c_wr_u8(ctx, 0x0341, ctx->frame_length & 0xFF );
		}
	}

	// subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_exposure[0].addr[0],
	// 	(ctx->exposure[0] >> 8) & 0xFF);
	// subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_exposure[0].addr[1],
	// 	ctx->exposure[0] & 0xFF);

	DRV_LOG(ctx, "exp[0x%x], fll(input/output):%u/%u, flick_en:%u\n",
		ctx->exposure[0], frame_length, ctx->frame_length, ctx->autoflicker_en);
	return ERROR_NONE;
}

static int gc08a8syx_set_multi_shutter_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64 *feature_data = (u64 *)para;
	u32 *shutters = (u32 *)(*feature_data);
	u16 shutter_cnt = *(feature_data + 1);
	u16 frame_length = *(feature_data + 2);
	kal_uint16 realtime_fps = 0;
	kal_int32 dummy_line = 0;

	if (shutter_cnt == 1) {
		ctx->shutter = shutters[0];

		/* Change frame time */
		if (frame_length > 1)
			dummy_line = frame_length - ctx->frame_length;
		ctx->frame_length = ctx->frame_length + dummy_line;

		/*  */
		if (shutters[0] > ctx->frame_length - ctx->s_ctx.exposure_margin)
			ctx->frame_length = shutters[0] + ctx->s_ctx.exposure_margin;

		if (ctx->frame_length > ctx->s_ctx.frame_length_max)
			ctx->frame_length = ctx->s_ctx.frame_length_max;

		shutters[0] = (shutters[0] < ctx->s_ctx.exposure_min) ? ctx->s_ctx.exposure_min : shutters[0];
		shutters[0] = (shutters[0] > (ctx->s_ctx.frame_length_max - ctx->s_ctx.exposure_margin))
			? (ctx->s_ctx.frame_length_max - ctx->s_ctx.exposure_margin) : shutters[0];

		if (ctx->autoflicker_en) {
			realtime_fps = ctx->pclk / ctx->line_length * 10 / ctx->frame_length;
			if (realtime_fps >= 593 && realtime_fps <= 607)
				gc08a8syx_set_max_framerate(ctx, 592, 0);
			else if (realtime_fps >= 297 && realtime_fps <= 305)
				gc08a8syx_set_max_framerate(ctx, 296, 0);
			else if (realtime_fps >= 147 && realtime_fps <= 150)
				gc08a8syx_set_max_framerate(ctx, 146, 0);
		}

		/* Update Shutter */
		subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_frame_length.addr[0], ctx->frame_length >> 8);
		subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_frame_length.addr[1], ctx->frame_length & 0xFF);
		subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_exposure[0].addr[0], (shutters[0] >> 8) & 0xFF);
		subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_exposure[0].addr[1], shutters[0] & 0xFF);
		ctx->frame_length_rg = ctx->frame_length;
		LOG_INF("shutters[0] =%d, framelength =%d\n",
				shutters[0], ctx->frame_length);
	}
	return ERROR_NONE;
}


static int gc08a8syx_set_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64 *feature_data = (u64 *)para;
	u32 frame_length = *(feature_data);

	if (frame_length)
		ctx->frame_length = frame_length;

	if (ctx->frame_length > ctx->s_ctx.frame_length_max)
		ctx->frame_length = ctx->s_ctx.frame_length_max;
	if (ctx->min_frame_length > ctx->frame_length)
		ctx->frame_length = ctx->min_frame_length;

	subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_frame_length.addr[0], ctx->frame_length >> 8);
	subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_frame_length.addr[1], ctx->frame_length & 0xFF);
	LOG_INF("Framelength: set=%d/min=%d\n",
		ctx->frame_length, ctx->min_frame_length);
	return ERROR_NONE;
}


static int gc08a8syx_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 mode = *((u32 *)para);
	bool enable = mode;

	if (mode != ctx->test_pattern)
		LOG_INF( "mode(%u->%u)\n", ctx->test_pattern, mode);
	if (enable) {
		subdrv_i2c_wr_u8(ctx, 0x008c, 0x01);
		subdrv_i2c_wr_u8(ctx, 0x008d, 0x00);
	} else {
		subdrv_i2c_wr_u8(ctx, 0x008c, 0x00);
		subdrv_i2c_wr_u8(ctx, 0x008d, 0x10);
	}
	ctx->test_pattern = enable;

	return ERROR_NONE;
}


static int gc08a8syx_set_max_framerate_by_scenario(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64 *feature_data = (u64 *)para;
	enum SENSOR_SCENARIO_ID_ENUM scenario_id = (enum SENSOR_SCENARIO_ID_ENUM)*feature_data;
	u32 framerate = *(feature_data + 1);
	u32 frame_length, calc_fl, exp_cnt, i;

	if (scenario_id >= ctx->s_ctx.sensor_mode_num) {
		LOG_INF( "invalid sid:%u, mode_num:%u\n",
			scenario_id, ctx->s_ctx.sensor_mode_num);
		scenario_id = SENSOR_SCENARIO_ID_NORMAL_PREVIEW;
	}

	if (framerate == 0) {
		LOG_INF( "framerate should not be 0\n");
		return ERROR_NONE;
	}

	if (ctx->s_ctx.mode[scenario_id].linelength == 0) {
		LOG_INF( "linelength should not be 0\n");
		return ERROR_NONE;
	}

	if (ctx->line_length == 0) {
		LOG_INF( "ctx->line_length should not be 0\n");
		return ERROR_NONE;
	}

	if (ctx->frame_length == 0) {
		LOG_INF( "ctx->frame_length should not be 0\n");
		return ERROR_NONE;
	}
	exp_cnt = ctx->s_ctx.mode[scenario_id].exp_cnt;
	calc_fl = ctx->exposure[0];
	for (i = 1; i < exp_cnt; i++)
		calc_fl += ctx->exposure[i];
	calc_fl += ctx->s_ctx.exposure_margin*exp_cnt*exp_cnt;

	frame_length = ctx->s_ctx.mode[scenario_id].pclk / framerate * 10
		/ ctx->s_ctx.mode[scenario_id].linelength;
	ctx->frame_length =
		max(frame_length, ctx->s_ctx.mode[scenario_id].framelength);
	ctx->frame_length = min(ctx->frame_length, ctx->s_ctx.frame_length_max);
	ctx->current_fps = ctx->pclk / ctx->frame_length * 10 / ctx->line_length;
	ctx->min_frame_length = ctx->frame_length;
	LOG_INF( "max_fps(input/output):%u/%u(sid:%u), frame_length:%u, calc_fl:%u, min_fl_en:1\n",
		framerate, ctx->current_fps, scenario_id, ctx->frame_length, calc_fl);
	if (ctx->frame_length > calc_fl)
		gc08a8syx_set_dummy(ctx);
	else
		ctx->frame_length = calc_fl;

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
