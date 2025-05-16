// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#define PFX "CAM_CAL"
#define pr_fmt(fmt) PFX "[%s] " fmt, __func__

#include <linux/kernel.h>
#include "cam_cal_list.h"
#include "eeprom_i2c_common_driver.h"
#include "eeprom_i2c_custom_driver.h"
#include "cam_cal_config.h"

unsigned int do_2a_gain_gc08a8(struct EEPROM_DRV_FD_DATA *pdata,
		unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData);

static struct STRUCT_CALIBRATION_LAYOUT_STRUCT cal_layout_table = {
	0x00000001, 0x010b00ff, CAM_CAL_SINGLE_EEPROM_DATA,
	{
		{0x00000001, 0x00000000, 0x00000008, do_module_version},
		{0x00000001, 0x00000008, 0x00000014, do_part_number},
		{0x00000001, 0x00000044, 0x0000074C, do_single_lsc},
		{0x00000001, 0x0000001E, 0x00000010, do_2a_gain_gc08a8},
		{0x00000001, 0x00000FAE, 0x00000550, do_stereo_data},
		{0x00000001, 0x00000000, 0x00001600, do_dump_all},
		{0x00000001, 0x00000005, 0x00000001, do_lens_id}
	}
};

struct STRUCT_CAM_CAL_CONFIG_STRUCT gc08a8_mtk_eeprom = {
	.name = "gc08a8_mtk_eeprom",
	.check_layout_function = layout_check,
	.read_function = Common_read_region,
	.layout = &cal_layout_table,
	.sensor_id = GC08A8_SENSOR_ID,
	.i2c_write_id = 0xA0,
	.max_size = 0x4000,
	.enable_preload = 1,
	.preload_size = 0x1500,
	.has_stored_data = 1,
};

unsigned int do_2a_gain_gc08a8(struct EEPROM_DRV_FD_DATA *pdata,
		unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData)
{
	struct STRUCT_CAM_CAL_DATA_STRUCT *pCamCalData =
				(struct STRUCT_CAM_CAL_DATA_STRUCT *)pGetSensorCalData;
	int read_data_size;
	unsigned int err = CamCalReturnErr[pCamCalData->Command];

	unsigned int CalGain = 0, FacGain = 0;
	unsigned char AWBAFConfig = 0;
	unsigned char AWBConfig = 0;
	int tempMax = 0;
	int CalR = 1, CalGr = 1, CalGb = 1, CalG = 1, CalB = 1;
	int FacR = 1, FacGr = 1, FacGb = 1, FacG = 1, FacB = 1;

	AWBConfig = 1;


	printk("block_size=%d sensor_id=%x\n", block_size, pCamCalData->sensorID);
	memset((void *)&pCamCalData->Single2A, 0, sizeof(struct STRUCT_CAM_CAL_SINGLE_2A_STRUCT));
	/* Check rule */
	if (pCamCalData->DataVer >= CAM_CAL_TYPE_NUM) {
		err = CAM_CAL_ERR_NO_DEVICE;
		error_log("Read Failed\n");
		show_cmd_error_log(pCamCalData->Command);
		return err;
	}

	/* Check AWB & AF enable bit */
	read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
			start_addr - 1, 1, (unsigned char *)&AWBAFConfig);
	if (read_data_size > 0)
		err = CAM_CAL_ERR_NO_ERR;
	else {
		pCamCalData->Single2A.S2aBitEn = CAM_CAL_NONE_BITEN;
		error_log("Read Failed\n");
		show_cmd_error_log(pCamCalData->Command);
	}
	pCamCalData->Single2A.S2aVer = 0x01;
	pCamCalData->Single2A.S2aBitEn = (0x03 & AWBAFConfig);
	must_log("S2aBitEn=0x%02x", pCamCalData->Single2A.S2aBitEn);
	if (get_mtk_format_version(pdata, pGetSensorCalData) >= 0x18)
		if (0x2 & AWBAFConfig)
			pCamCalData->Single2A.S2aAfBitflagEn = 0x0C;
		else
			pCamCalData->Single2A.S2aAfBitflagEn = 0x00;
	else
		pCamCalData->Single2A.S2aAfBitflagEn = (0x0C & AWBAFConfig);
	/* AWB Calibration Data*/
	if (0x1 & AWBConfig) {
		pCamCalData->Single2A.S2aAwb.rGainSetNum = 0;
		read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
				start_addr, 4, (unsigned char *)&CalGain);
		read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
				start_addr, 2, (unsigned char *)&CalR);
		read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
				start_addr + 2, 2, (unsigned char *)&CalGr);
		read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
				start_addr + 4, 2, (unsigned char *)&CalGb);
		read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
				start_addr + 6, 2, (unsigned char *)&CalB);
		if (read_data_size > 0)	{
			must_log("Read CalGain OK %x\n", read_data_size);
			CalR  = ((CalR & 0xFF) << 8) | ((CalR & 0xFF00)>> 8);
			CalGr = ((CalGr & 0xFF) << 8) | ((CalGr & 0xFF00)>> 8);
			CalGb = ((CalGb & 0xFF) << 8) | ((CalGb & 0xFF00)>> 8);
			CalG  = ((CalGr + CalGb) + 1) >> 1;
			CalB  = ((CalB & 0xFF) << 8) | ((CalB & 0xFF00)>> 8);
			if (CalR > CalG)
				/* R > G */
				if (CalR > CalB)
					tempMax = CalR;
				else
					tempMax = CalB;
			else
				/* G > R */
				if (CalG > CalB)
					tempMax = CalG;
				else
					tempMax = CalB;
			must_log(
				"UnitR:%d, UnitG:%d, UnitB:%d, New Unit Max=%d",
				CalR, CalG, CalB, tempMax);
			err = CAM_CAL_ERR_NO_ERR;
		} else {
			pCamCalData->Single2A.S2aBitEn = CAM_CAL_NONE_BITEN;
			error_log("Read CalGain Failed\n");
			show_cmd_error_log(pCamCalData->Command);
		}
		if (CalGain != 0x00000000 &&
			CalGain != 0xFFFFFFFF &&
			CalR    != 0x00000000 &&
			CalG    != 0x00000000 &&
			CalB    != 0x00000000) {
			pCamCalData->Single2A.S2aAwb.rGainSetNum++;
			pCamCalData->Single2A.S2aAwb.rUnitGainu4R =
					(unsigned int)((tempMax * 512 + (CalR >> 1)) / CalR);
			pCamCalData->Single2A.S2aAwb.rUnitGainu4G =
					(unsigned int)((tempMax * 512 + (CalG >> 1)) / CalG);
			pCamCalData->Single2A.S2aAwb.rUnitGainu4B =
					(unsigned int)((tempMax * 512 + (CalB >> 1)) / CalB);
		} else
			error_log(
			"There are something wrong on EEPROM, plz contact module vendor!!\n");
		/* AWB Golden Gain (5100K) */
		read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
				start_addr + 8, 4, (unsigned char *)&FacGain);
		read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
				start_addr + 8, 2, (unsigned char *)&FacR);
		read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
				start_addr + 10, 2, (unsigned char *)&FacGr);
		read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
				start_addr + 12, 2, (unsigned char *)&FacGb);
		read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
				start_addr + 14, 2, (unsigned char *)&FacB);
		if (read_data_size > 0)	{
			debug_log("Read FacGain OK\n");
			FacR  = ((FacR & 0xFF) << 8) | ((FacR & 0xFF00)>> 8);
			FacGr = ((FacGr & 0xFF) << 8) | ((FacGr & 0xFF00)>> 8);
			FacGb = ((FacGb & 0xFF) << 8) | ((FacGb & 0xFF00)>> 8);
			FacG  = ((FacGr + FacGb) + 1) >> 1;
			FacB  = ((FacB & 0xFF) << 8) | ((FacB & 0xFF00)>> 8);
			if (FacR > FacG)
				if (FacR > FacB)
					tempMax = FacR;
				else
					tempMax = FacB;
			else
				if (FacG > FacB)
					tempMax = FacG;
				else
					tempMax = FacB;
			must_log(
				"GoldenR:%d, GoldenG:%d, GoldenB:%d, New Golden Max=%d",
				FacR, FacG, FacB, tempMax);
			err = CAM_CAL_ERR_NO_ERR;
		} else {
			pCamCalData->Single2A.S2aBitEn = CAM_CAL_NONE_BITEN;
			error_log("Read FacGain Failed\n");
			show_cmd_error_log(pCamCalData->Command);
		}
		if (FacGain != 0x00000000 &&
			FacGain != 0xFFFFFFFF &&
			FacR    != 0x00000000 &&
			FacG    != 0x00000000 &&
			FacB    != 0x00000000)	{
			pCamCalData->Single2A.S2aAwb.rGoldGainu4R =
					(unsigned int)((tempMax * 512 + (FacR >> 1)) / FacR);
			pCamCalData->Single2A.S2aAwb.rGoldGainu4G =
					(unsigned int)((tempMax * 512 + (FacG >> 1)) / FacG);
			pCamCalData->Single2A.S2aAwb.rGoldGainu4B =
					(unsigned int)((tempMax * 512 + (FacB >> 1)) / FacB);
		} else
			error_log(
			"There are something wrong on EEPROM, plz contact module vendor!!\n");
		/* Set AWB to 3A Layer */
		pCamCalData->Single2A.S2aAwb.rValueR   = CalR;
		pCamCalData->Single2A.S2aAwb.rValueGr  = CalGr;
		pCamCalData->Single2A.S2aAwb.rValueGb  = CalGb;
		pCamCalData->Single2A.S2aAwb.rValueB   = CalB;
		pCamCalData->Single2A.S2aAwb.rGoldenR  = FacR;
		pCamCalData->Single2A.S2aAwb.rGoldenGr = FacGr;
		pCamCalData->Single2A.S2aAwb.rGoldenGb = FacGb;
		pCamCalData->Single2A.S2aAwb.rGoldenB  = FacB;
		must_log("======================AWB CAM_CAL==================\n");
		must_log("[CalGain] = 0x%x\n", CalGain);
		must_log("[FacGain] = 0x%x\n", FacGain);
		must_log("[rCalGain.u4R] = %d\n", pCamCalData->Single2A.S2aAwb.rUnitGainu4R);
		must_log("[rCalGain.u4G] = %d\n", pCamCalData->Single2A.S2aAwb.rUnitGainu4G);
		must_log("[rCalGain.u4B] = %d\n", pCamCalData->Single2A.S2aAwb.rUnitGainu4B);
		must_log("[rFacGain.u4R] = %d\n", pCamCalData->Single2A.S2aAwb.rGoldGainu4R);
		must_log("[rFacGain.u4G] = %d\n", pCamCalData->Single2A.S2aAwb.rGoldGainu4G);
		must_log("[rFacGain.u4B] = %d\n", pCamCalData->Single2A.S2aAwb.rGoldGainu4B);
		must_log("======================AWB CAM_CAL==================\n");
	}
	return err;
}