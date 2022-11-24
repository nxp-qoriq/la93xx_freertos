/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright 2022 NXP
 * All rights reserved.
 */
#include "rfic_synth.h"
#include "rfic_lmx2582_synth.h"
#include "rfic_max2870_synth.h"

int8_t board_rev  = NLM_BOARD_REV_1;

int32_t RficSynthWriteReg( RficDevice_t *pRficDev, uint8_t addr, uint32_t data )
{
    int32_t iRet;

    if( board_rev == NLM_BOARD_REV_2)
    {
        iRet = Rficmax2870WriteReg( pRficDev, addr, data );
    }
    else
        iRet = Rficlmx2582WriteReg( pRficDev, addr, (uint16_t)data );

    return iRet;
}

int32_t RficSynthReadReg( RficDevice_t *pRficDev, uint8_t addr, uint32_t *data )
{
    int32_t iRet;

    if ( board_rev == NLM_BOARD_REV_2)
        iRet = Rficmax2870ReadReg( pRficDev, addr, data );
    else
        iRet = Rficlmx2582ReadReg( pRficDev, addr, (uint16_t *)data );

    return iRet;
}

void RficSynthAdjustPllFreq( RficDevice_t *pRficDev, int32_t freq_khz )
{
    if ( board_rev == NLM_BOARD_REV_2)
        Rficmax2870AdjustPllFreq( pRficDev, freq_khz );
    else
        Rficlmx2582AdjustPllFreq( pRficDev, freq_khz );
}

void RficSynthAdjustPllFastCal(RficDevice_t *pRficDev)
{
    if ( board_rev == NLM_BOARD_REV_1)
        Rficlmx2582AdjustPllFastCal( pRficDev );
    else
        log_info("NLM Synth Fast Cal not supported");
}

int32_t RficSynthInit( RficDevice_t *pRficDev )
{
    int32_t iRet = 0;


    board_rev = iLa9310_Get_Board_Rev();
    if( board_rev == NLM_BOARD_REV_2)
    {
        iRet = Rficmax2870Init( pRficDev );
        if( 0 != iRet )
        {
            log_err( "%s: NLM Synth init failed \r\n", __func__ );
        }
    }
    else
    {
        iRet = Rficlmx2582Init( pRficDev );
        if( 0 != iRet )
        {
            log_err( "%s: NLM Synth init failed \r\n",__func__ );
        }
    }

    return iRet;
}

