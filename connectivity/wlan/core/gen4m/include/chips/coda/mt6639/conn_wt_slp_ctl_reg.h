/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#ifndef __CONN_WT_SLP_CTL_REG_REGS_H__
#define __CONN_WT_SLP_CTL_REG_REGS_H__

#include "hal_common.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CONN_WT_SLP_CTL_REG_BASE \
	(0x18091000 + CONN_INFRA_REMAPPING_OFFSET)

#define CONN_WT_SLP_CTL_REG_WB_SLP_TRG_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x000)
#define CONN_WT_SLP_CTL_REG_WB_SLP_CTL_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x004)
#define CONN_WT_SLP_CTL_REG_WB_STA_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x008)
#define CONN_WT_SLP_CTL_REG_WB_SLP_TMOUT_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x00C)
#define CONN_WT_SLP_CTL_REG_WB_BG_ADDR1_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x010)
#define CONN_WT_SLP_CTL_REG_WB_BG_ADDR2_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x014)
#define CONN_WT_SLP_CTL_REG_WB_BG_ADDR3_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x018)
#define CONN_WT_SLP_CTL_REG_WB_BG_ADDR4_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x01C)
#define CONN_WT_SLP_CTL_REG_WB_BG_ADDR5_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x020)
#define CONN_WT_SLP_CTL_REG_WB_BG_ADDR6_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x024)
#define CONN_WT_SLP_CTL_REG_WB_BG_ADDR7_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x028)
#define CONN_WT_SLP_CTL_REG_WB_BG_ADDR8_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x02C)
#define CONN_WT_SLP_CTL_REG_WB_BG_ON1_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x030)
#define CONN_WT_SLP_CTL_REG_WB_BG_ON2_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x034)
#define CONN_WT_SLP_CTL_REG_WB_BG_ON3_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x038)
#define CONN_WT_SLP_CTL_REG_WB_BG_ON4_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x03C)
#define CONN_WT_SLP_CTL_REG_WB_BG_ON5_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x040)
#define CONN_WT_SLP_CTL_REG_WB_BG_ON6_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x044)
#define CONN_WT_SLP_CTL_REG_WB_BG_ON7_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x048)
#define CONN_WT_SLP_CTL_REG_WB_BG_ON8_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x04C)
#define CONN_WT_SLP_CTL_REG_WB_BG_OFF1_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x050)
#define CONN_WT_SLP_CTL_REG_WB_BG_OFF2_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x054)
#define CONN_WT_SLP_CTL_REG_WB_BG_OFF3_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x058)
#define CONN_WT_SLP_CTL_REG_WB_BG_OFF4_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x05C)
#define CONN_WT_SLP_CTL_REG_WB_BG_OFF5_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x060)
#define CONN_WT_SLP_CTL_REG_WB_BG_OFF6_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x064)
#define CONN_WT_SLP_CTL_REG_WB_BG_OFF7_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x068)
#define CONN_WT_SLP_CTL_REG_WB_BG_OFF8_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x06C)
#define CONN_WT_SLP_CTL_REG_WB_WF_CK_ADDR_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x070)
#define CONN_WT_SLP_CTL_REG_WB_WF_WAKE_ADDR_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x074)
#define CONN_WT_SLP_CTL_REG_WB_WF_ZPS_ADDR_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x078)
#define CONN_WT_SLP_CTL_REG_WB_BT_CK_ADDR_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x07C)
#define CONN_WT_SLP_CTL_REG_WB_BT_WAKE_ADDR_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x080)
#define CONN_WT_SLP_CTL_REG_WB_TOP_CK_ADDR_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x084)
#define CONN_WT_SLP_CTL_REG_WB_GPS_CK_ADDR_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x088)
#define CONN_WT_SLP_CTL_REG_WB_WF_B0_CMD_ADDR_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x08c)
#define CONN_WT_SLP_CTL_REG_WB_WF_B1_CMD_ADDR_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x090)
#define CONN_WT_SLP_CTL_REG_WB_GPS_RFBUF_ADR_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x094)
#define CONN_WT_SLP_CTL_REG_WB_GPS_L5_EN_ADDR_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x098)
#define CONN_WT_SLP_CTL_REG_WB_STA1_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x0A0)
#define CONN_WT_SLP_CTL_REG_WB_RSV_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x0A4)
#define CONN_WT_SLP_CTL_REG_WB_CK_STA_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x0A8)
#define CONN_WT_SLP_CTL_REG_WB_FAKE_CK_EN_TOP_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x100)
#define CONN_WT_SLP_CTL_REG_WB_FAKE_CK_EN_WF_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x104)
#define CONN_WT_SLP_CTL_REG_WB_FAKE_CK_EN_BT_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x108)
#define CONN_WT_SLP_CTL_REG_WB_BT1_CK_ADDR_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x10C)
#define CONN_WT_SLP_CTL_REG_WB_BT1_WAKE_ADDR_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x110)
#define CONN_WT_SLP_CTL_REG_WB_FAKE_CK_EN_BT1_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x114)
#define CONN_WT_SLP_CTL_REG_WB_STA2_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x118)
#define CONN_WT_SLP_CTL_REG_WB_SLP_DEBUG_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0X11c)
#define CONN_WT_SLP_CTL_REG_WB_SLP_TOP_CK_0_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x120)
#define CONN_WT_SLP_CTL_REG_WB_SLP_TOP_CK_1_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x124)
#define CONN_WT_SLP_CTL_REG_WB_SLP_TOP_CK_2_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x128)
#define CONN_WT_SLP_CTL_REG_WB_SLP_TOP_CK_3_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x12C)
#define CONN_WT_SLP_CTL_REG_WB_SLP_TOP_CK_4_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x130)
#define CONN_WT_SLP_CTL_REG_WB_SLP_TOP_CK_5_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x134)
#define CONN_WT_SLP_CTL_REG_WB_SLP_MULT_TOP_CK_0_ADDR_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x138)
#define CONN_WT_SLP_CTL_REG_WB_SLP_MULT_TOP_CK_1_ADDR_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x13C)
#define CONN_WT_SLP_CTL_REG_WB_SLP_MULT_TOP_CK_2_ADDR_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x140)
#define CONN_WT_SLP_CTL_REG_WB_SLP_MULT_TOP_CK_3_ADDR_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x144)
#define CONN_WT_SLP_CTL_REG_WB_SLP_MULT_TOP_CK_4_ADDR_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x148)
#define CONN_WT_SLP_CTL_REG_WB_SLP_MULT_TOP_CK_5_ADDR_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x14C)
#define CONN_WT_SLP_CTL_REG_WB_STA3_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x150)
#define CONN_WT_SLP_CTL_REG_WB_BG_ADDR9_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x154)
#define CONN_WT_SLP_CTL_REG_WB_BG_ADDR10_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x158)
#define CONN_WT_SLP_CTL_REG_WB_BG_ADDR11_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x15c)
#define CONN_WT_SLP_CTL_REG_WB_BG_ADDR12_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x160)
#define CONN_WT_SLP_CTL_REG_WB_BG_ON9_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x164)
#define CONN_WT_SLP_CTL_REG_WB_BG_ON10_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x168)
#define CONN_WT_SLP_CTL_REG_WB_BG_ON11_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x16C)
#define CONN_WT_SLP_CTL_REG_WB_BG_ON12_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x170)
#define CONN_WT_SLP_CTL_REG_WB_BG_OFF9_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x174)
#define CONN_WT_SLP_CTL_REG_WB_BG_OFF10_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x178)
#define CONN_WT_SLP_CTL_REG_WB_BG_OFF11_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x17C)
#define CONN_WT_SLP_CTL_REG_WB_BG_OFF12_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x180)
#define CONN_WT_SLP_CTL_REG_WB_STA4_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x184)
#define CONN_WT_SLP_CTL_REG_WB_LIT_WF_CK_ADDR_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x18C)
#define CONN_WT_SLP_CTL_REG_WB_LIT_WF_WAKE_ADDR_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x190)
#define CONN_WT_SLP_CTL_REG_WB_LIT_WF_ZPS_ADDR_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x194)
#define CONN_WT_SLP_CTL_REG_WB_WF_LIT_CMD_ADDR_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x198)
#define CONN_WT_SLP_CTL_REG_WB_LIT_BT_CK_ADDR_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x19C)
#define CONN_WT_SLP_CTL_REG_WB_GPS_CTL_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x200)
#define CONN_WT_SLP_CTL_REG_WB_LIT_BT_WAKE_ADDR_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x204)
#define CONN_WT_SLP_CTL_REG_WB_CK_STA2_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x208)
#define CONN_WT_SLP_CTL_REG_WB_CK_SLPWK_ADDR \
	(CONN_WT_SLP_CTL_REG_BASE + 0x20C)

#ifdef __cplusplus
}
#endif

#endif /* __CONN_WT_SLP_CTL_REG_REGS_H__*/