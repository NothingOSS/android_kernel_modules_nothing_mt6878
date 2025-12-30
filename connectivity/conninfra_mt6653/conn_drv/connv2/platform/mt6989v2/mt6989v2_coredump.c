// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2024 MediaTek Inc.
 */

#include <linux/printk.h>

#include "../../debug_utility/coredump/connsys_coredump_hw_config.h"
#include "include/mt6989v2_coredump.h"

/*******************************************************************************
*                             D A T A   T Y P E S
********************************************************************************
*/
static char *gps_task_str[] = {
	"Task_GPS",
	"Task_TST_GPSSYS",
	"Task_Idle_GPSSYS",
};

static struct coredump_hw_config g_coredump_config[CONN_DEBUG_TYPE_END] = {
	/* Wi-Fi config */
	{
	},
	/* BT config */
	{
	},
	/* GPS config */
	{
		.name = "GPSSYS",
		.start_offset = 0x4b8000,
		.size = 0x18000,
		.seg1_cr = 0x18c16024,
		.seg1_value_end = 0x18ffffff,
		.seg1_start_addr = 0x18c16014,
		.seg1_phy_addr = 0x18d00000, // start of dynamic remapping 0
		.task_table_size = sizeof(gps_task_str)/sizeof(char *),
		.task_map_table = gps_task_str,
		.exception_tag_name = "combo_gps",
	},
};

struct coredump_hw_config *consys_plt_coredump_get_platform_config_mt6989v2(int conn_type)
{
	if (conn_type != CONN_DEBUG_TYPE_GPS) {
		pr_notice("[%s] incorrect type: %d\n", __func__, conn_type);
		return NULL;
	}

	return &g_coredump_config[conn_type];
}
