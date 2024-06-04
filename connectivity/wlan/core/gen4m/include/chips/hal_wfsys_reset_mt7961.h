/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

/*! \file   hal_wfsys_reset_mt7961.h
*    \brief  WFSYS reset HAL API for MT7961
*
*    This file contains all routines which are exported
     from MediaTek 802.11 Wireless LAN driver stack to GLUE Layer.
*/

#ifndef _HAL_WFSYS_RESET_MT7961_H
#define _HAL_WFSYS_RESET_MT7961_H

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

/*******************************************************************************
*                             D A T A   T Y P E S
********************************************************************************
*/

/*******************************************************************************
*                            P U B L I C   D A T A
********************************************************************************
*/

/*******************************************************************************
*                           P R I V A T E   D A T A
********************************************************************************
*/

/*******************************************************************************
*                                 M A C R O S
********************************************************************************
*/

/*******************************************************************************
*                   F U N C T I O N   D E C L A R A T I O N S
********************************************************************************
*/

/*******************************************************************************
*                              F U N C T I O N S
********************************************************************************
*/

u_int8_t mt7961HalCbtopRguWfRst(struct ADAPTER *prAdapter,
				u_int8_t fgAssertRst);

u_int8_t mt7961HalPollWfsysSwInitDone(struct ADAPTER *prAdapter);

#if defined(_HIF_PCIE)

#endif /* defined(_HIF_PCIE) */

#if defined(_HIF_USB)

u_int8_t mt7961HalUsbEpctlRstOpt(struct ADAPTER *prAdapter,
				 u_int8_t fgIsRstScopeIncludeToggleBit);

#endif    /* defined(_HIF_USB) */

#if defined(_HIF_SDIO)

#endif /* defined(_HIF_SDIO) */

#endif /* _HAL_WFSYS_RESET_MT7961_H */