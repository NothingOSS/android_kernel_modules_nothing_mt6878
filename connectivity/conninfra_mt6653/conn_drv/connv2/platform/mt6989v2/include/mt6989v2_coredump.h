/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2024 MediaTek Inc.
 */

#ifndef MT6989V2_COREDUMP_H
#define MT6989V2_COREDUMP_H

#include "connsys_debug_utility.h"

/*******************************************************************************
 *                  F U N C T I O N   D E C L A R A T I O N S
 ********************************************************************************
 */
struct coredump_hw_config *consys_plt_coredump_get_platform_config_mt6989v2(int conn_type);

#endif /* MT6989V2_COREDUMP_H */
