/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#ifndef _DBG_WTBL_CONNAC3X_H
#define _DBG_WTBL_CONNAC3X_H

/*******************************************************************************
*                         C O M P I L E R   F L A G S
********************************************************************************
*/

/*******************************************************************************
*                    E X T E R N A L   R E F E R E N C E S
********************************************************************************
*/

/*******************************************************************************
*                              C O N S T A N T S
********************************************************************************
*/
/* This address is not generated by CODA and might be different by project */
#define WIFI_WTBL_BASE                  0x820D8000
#define WIFI_LWTBL_BASE                 WF_WTBLON_TOP_BASE
#define WIFI_UWTBL_BASE                 WF_UWTBL_TOP_BASE

#define LWTBL_IDX2BASE(_wlanIdx, _DW) \
		(WIFI_WTBL_BASE | ((_wlanIdx & 0x7F) << 8) | (_DW & 0x3F) << 2)

#define UWTBL_IDX2BASE(_wlanIdx, _DW) \
		(WIFI_UWTBL_BASE | 0x2000 | ((_wlanIdx & 0x7F) << 6) | (_DW & 0xF) << 2)

#define KEYTBL_IDX2BASE(_key_loc, _DW) \
		(WIFI_UWTBL_BASE | 0x2000 | ((_key_loc & 0x7F) << 6) | (_DW & 0xF) << 2)

#define NO_SHIFT_DEFINE			0xFFFFFFFF
#define LWTBL_LEN_IN_DW			36
#define UWTBL_LEN_IN_DW			10

/***** WTBL(UMAC) *****/
/* WTBL Group - Packet Number */
/* DW 2 */
#define WTBL_PN0_MASK                   BITS(0, 7)
#define WTBL_PN0_OFFSET                 0
#define WTBL_PN1_MASK                   BITS(8, 15)
#define WTBL_PN1_OFFSET                 8
#define WTBL_PN2_MASK                   BITS(16, 23)
#define WTBL_PN2_OFFSET                 16
#define WTBL_PN3_MASK                   BITS(24, 31)
#define WTBL_PN3_OFFSET                 24

/* DW 3 */
#define WTBL_PN4_MASK                   BITS(0, 7)
#define WTBL_PN4_OFFSET                 0
#define WTBL_PN5_MASK                   BITS(8, 15)
#define WTBL_PN5_OFFSET                 8

/* DW 4 */
#define WTBL_BIPN0_MASK                 BITS(0, 7)
#define WTBL_BIPN0_OFFSET               0
#define WTBL_BIPN1_MASK                 BITS(8, 15)
#define WTBL_BIPN1_OFFSET               8
#define WTBL_BIPN2_MASK                 BITS(16, 23)
#define WTBL_BIPN2_OFFSET               16
#define WTBL_BIPN3_MASK                 BITS(24, 31)
#define WTBL_BIPN3_OFFSET               24

/* DW 5 */
#define WTBL_BIPN4_MASK                 BITS(0, 7)
#define WTBL_BIPN4_OFFSET               0
#define WTBL_BIPN5_MASK                 BITS(8, 15)
#define WTBL_BIPN5_OFFSET               8

/* UWTBL DW 6 */
#define WTBL_AMSDU_LEN_MASK             BITS(0, 5)
#define WTBL_AMSDU_LEN_OFFSET           0
#define WTBL_AMSDU_NUM_MASK             BITS(6, 10)
#define WTBL_AMSDU_NUM_OFFSET           6
#define WTBL_AMSDU_EN_MASK              BIT(11)
#define WTBL_AMSDU_EN_OFFSET            11

/* LWTBL Rate field */
#define WTBL_RATE_TX_RATE_MASK          BITS(0, 5)
#define WTBL_RATE_TX_RATE_OFFSET        0
#define WTBL_RATE_TX_MODE_MASK          BITS(6, 9)
#define WTBL_RATE_TX_MODE_OFFSET        6
#define WTBL_RATE_NSTS_MASK             BITS(10, 13)
#define WTBL_RATE_NSTS_OFFSET           10
#define WTBL_RATE_STBC_MASK             BIT(14)
#define WTBL_RATE_STBC_OFFSET           14

/* DW 34*/
#define WTBL_RESP_RCPI0_MASK           BITS(0, 7)
#define WTBL_RESP_RCPI0_OFFSET         0
#define WTBL_RESP_RCPI1_MASK           BITS(8, 15)
#define WTBL_RESP_RCPI1_OFFSET         8
#define WTBL_RESP_RCPI2_MASK           BITS(16, 23)
#define WTBL_RESP_RCPI2_OFFSET         16
#define WTBL_RESP_RCPI3_MASK           BITS(24, 31)
#define WTBL_RESP_RCPI3_OFFSET         24

/***** WTBL(LMAC) DW Offset *****/
/* LMAC WTBL Group - Peer Unique Information */
#define WTBL_GROUP_PEER_INFO_DW_0               0
#define WTBL_GROUP_PEER_INFO_DW_1               1

/* WTBL Group - TxRx Capability/Information */
#define WTBL_GROUP_TRX_CAP_DW_2                 2
#define WTBL_GROUP_TRX_CAP_DW_3                 3
#define WTBL_GROUP_TRX_CAP_DW_4                 4
#define WTBL_GROUP_TRX_CAP_DW_5                 5
#define WTBL_GROUP_TRX_CAP_DW_6                 6
#define WTBL_GROUP_TRX_CAP_DW_7                 7
#define WTBL_GROUP_TRX_CAP_DW_8                 8
#define WTBL_GROUP_TRX_CAP_DW_9                 9

/* WTBL Group - Auto Rate Table*/
#define WTBL_GROUP_AUTO_RATE_1_2                10
#define WTBL_GROUP_AUTO_RATE_3_4                11
#define WTBL_GROUP_AUTO_RATE_5_6                12
#define WTBL_GROUP_AUTO_RATE_7_8                13

/* WTBL Group - Tx Counter */
#define WTBL_GROUP_TX_CNT_LINE_1                14
#define WTBL_GROUP_TX_CNT_LINE_2                15
#define WTBL_GROUP_TX_CNT_LINE_3                16
#define WTBL_GROUP_TX_CNT_LINE_4                17
#define WTBL_GROUP_TX_CNT_LINE_5                18
#define WTBL_GROUP_TX_CNT_LINE_6                19

/* WTBL Group - Admission Control Counter */
#define WTBL_GROUP_ADM_CNT_LINE_1               20
#define WTBL_GROUP_ADM_CNT_LINE_2               21
#define WTBL_GROUP_ADM_CNT_LINE_3               22
#define WTBL_GROUP_ADM_CNT_LINE_4               23
#define WTBL_GROUP_ADM_CNT_LINE_5               24
#define WTBL_GROUP_ADM_CNT_LINE_6               25
#define WTBL_GROUP_ADM_CNT_LINE_7               26
#define WTBL_GROUP_ADM_CNT_LINE_8               27

/* WTBL Group -MLO Info */
#define WTBL_GROUP_MLO_INFO_LINE_1              28
#define WTBL_GROUP_MLO_INFO_LINE_2              29
#define WTBL_GROUP_MLO_INFO_LINE_3              30

/* WTBL Group -RESP Info */
#define WTBL_GROUP_RESP_INFO_DW_31              31

/* WTBL Group -RX DUP Info */
#define WTBL_GROUP_RX_DUP_INFO_DW_32            32

/* WTBL Group - Rx Statistics Counter */
#define WTBL_GROUP_RX_STAT_CNT_LINE_1           33
#define WTBL_GROUP_RX_STAT_CNT_LINE_2           34
#define WTBL_GROUP_RX_STAT_CNT_LINE_3           35


/* UWTBL Group - HW AMSDU */
#define UWTBL_HW_AMSDU_DW                       WF_UWTBL_AMSDU_CFG_DW

/* LWTBL DW 4 */
#define WTBL_DIS_RHTR                           WF_LWTBL_DIS_RHTR_MASK

/* UWTBL DW 5 */
#define WTBL_KEY_LINK_DW_KEY_LOC0_MASK          BITS(0, 10)
#define WTBL_PSM				WF_LWTBL_PSM_MASK

/* Need to sync with FW define */
#define INVALID_KEY_ENTRY                       WTBL_KEY_LINK_DW_KEY_LOC0_MASK

#define ONE_KEY_ENTRY_LEN_IN_DW                8

enum _ENUM_WTBL_TYPE_T {
	WTBL_TYPE_LMAC = 0,     /** WTBL in LMAC */
	WTBL_TYPE_UMAC = 1,     /** WTBL in UMAC */
	WTBL_TYPE_KEY = 2,      /** Key Table */
	MAX_NUM_WTBL_TYPE
};

enum _ENUM_MUAR_INDEX_T {
	MUAR_INDEX_OWN_MAC_ADDR_0 = 0,
	MUAR_INDEX_OWN_MAC_ADDR_1,
	MUAR_INDEX_OWN_MAC_ADDR_2,
	MUAR_INDEX_OWN_MAC_ADDR_3,
	MUAR_INDEX_OWN_MAC_ADDR_4,
	MUAR_INDEX_OWN_MAC_ADDR_BC_MC = 0xE,
	MUAR_INDEX_UNMATCHED = 0xF,
	MUAR_INDEX_OWN_MAC_ADDR_11 = 0x11,
	MUAR_INDEX_OWN_MAC_ADDR_12,
	MUAR_INDEX_OWN_MAC_ADDR_13,
	MUAR_INDEX_OWN_MAC_ADDR_14,
	MUAR_INDEX_OWN_MAC_ADDR_15,
	MUAR_INDEX_OWN_MAC_ADDR_16,
	MUAR_INDEX_OWN_MAC_ADDR_17,
	MUAR_INDEX_OWN_MAC_ADDR_18,
	MUAR_INDEX_OWN_MAC_ADDR_19,
	MUAR_INDEX_OWN_MAC_ADDR_1A,
	MUAR_INDEX_OWN_MAC_ADDR_1B,
	MUAR_INDEX_OWN_MAC_ADDR_1C,
	MUAR_INDEX_OWN_MAC_ADDR_1D,
	MUAR_INDEX_OWN_MAC_ADDR_1E,
	MUAR_INDEX_OWN_MAC_ADDR_1F,
	MUAR_INDEX_OWN_MAC_ADDR_20,
	MUAR_INDEX_OWN_MAC_ADDR_21,
	MUAR_INDEX_OWN_MAC_ADDR_22,
	MUAR_INDEX_OWN_MAC_ADDR_23,
	MUAR_INDEX_OWN_MAC_ADDR_24,
	MUAR_INDEX_OWN_MAC_ADDR_25,
	MUAR_INDEX_OWN_MAC_ADDR_26,
	MUAR_INDEX_OWN_MAC_ADDR_27,
	MUAR_INDEX_OWN_MAC_ADDR_28,
	MUAR_INDEX_OWN_MAC_ADDR_29,
	MUAR_INDEX_OWN_MAC_ADDR_2A,
	MUAR_INDEX_OWN_MAC_ADDR_2B,
	MUAR_INDEX_OWN_MAC_ADDR_2C,
	MUAR_INDEX_OWN_MAC_ADDR_2D,
	MUAR_INDEX_OWN_MAC_ADDR_2E,
	MUAR_INDEX_OWN_MAC_ADDR_2F
};

enum _ENUM_IGTK_CIPHER_SUIT_T {
	IGTK_CIPHER_SUIT_NONE = 0,
	IGTK_CIPHER_SUIT_BIP,
	IGTK_CIPHER_SUIT_BIP_256
};

u_int8_t connac3x_wtbl_get_ldpc_info(
	uint8_t ucTxMode,
	struct bwtbl_lmac_struct *pWtbl);

u_int8_t connac3x_wtbl_get_sgi_info(
	struct bwtbl_lmac_struct *pWtbl);

/*******************************************************************************
*                  F U N C T I O N   D E C L A R A T I O N S
********************************************************************************
*/

#endif  /* _DBG_WTBL_CONNAC3X_H */
