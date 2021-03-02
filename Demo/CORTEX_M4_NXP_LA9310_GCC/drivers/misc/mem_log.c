/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2017, 2021 NXP
 */

#include <common.h>
#include "config.h"
#include "immap.h"
#include "soc.h"
#include "board.h"

void vMemlogWrite( const uint8_t * pucData,
                   size_t xLength )
{
    struct la9310_hif * pxHif = ( struct la9310_hif * ) ( ( uint8_t * ) TCML_PHY_ADDR
                                                          + LA9310_EP_HIF_OFFSET );
    int i;
    struct debug_log_regs * pxDbglog = &pxHif->dbg_log_regs;
    uint8_t * ucdbgptr = ( uint8_t * ) ( pxDbglog->buf );

    ucdbgptr += ulMemLogIndex;

    for( i = 0; i < xLength; i++ )
    {
        OUT_8( ucdbgptr, *pucData );
        pucData++;
        ucdbgptr++;
    }

    ulMemLogIndex += xLength;

    if( ulMemLogIndex >= pxDbglog->len )
    {
        ulMemLogIndex = 0;
    }
}
