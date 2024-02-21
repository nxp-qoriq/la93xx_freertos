/* SPDX-License-Identifier: BSD-3-Clause */
/* Copyright 2021-2024 NXP
 */
#ifndef __RFIC_CMD_H
#define __RFIC_CMD_H

# include "FreeRTOS.h"
# include "rfic_core.h"

/*
 * Function Prototype
 */
void xRficProcessSetBand( RficDevice_t *pRficDev, rf_sw_cmd_desc_t *pSwCmdDesc );
void xRficProcessAdjustPllFreq( RficDevice_t *pRficDev, rf_sw_cmd_desc_t *pSwCmdDesc );
void xRficProcessCtrlLna( RficDevice_t *pRficDev, rf_sw_cmd_desc_t *pSwCmdDesc );
void xRficProcessReadReg( RficDevice_t *pRficDev, rf_sw_cmd_desc_t *pSwCmdDesc );
void xRficProcessWriteReg( RficDevice_t *pRficDev, rf_sw_cmd_desc_t *pSwCmdDesc );
void vRficProcessIqDump( RficDevice_t *pRficDev, rf_sw_cmd_desc_t *rfic_sw_cmd );
void xRficProcessCtrlDemodGain( RficDevice_t *pRficDev, rf_sw_cmd_desc_t *pSwCmdDesc );
void xRficProcessCtrlVgaGain( RficDevice_t *pRficDev, rf_sw_cmd_desc_t *pSwCmdDesc );
void xRficProcessFastCalib( RficDevice_t *pRficDev);
void vRficProcessSingleToneTX( rf_sw_cmd_desc_t * rfic_sw_cmd );
void vRficProcessLoopback( RficDevice_t * pRficDev,
                           rf_sw_cmd_desc_t * rfic_sw_cmd );
void vRficProcessTXIqData(rf_sw_cmd_desc_t *rfic_sw_cmd);

void vRficGetRxDcOffset(rf_sw_cmd_desc_t *rfic_sw_cmd);
void vRficSetDcOffset(rf_sw_cmd_desc_t *rfic_sw_cmd);
void vRficSetChannel(rf_sw_cmd_desc_t *rfic_sw_cmd);

#endif //__RFIC_CMD_H
