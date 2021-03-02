/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright 2017, 2021 NXP
 */

#include <common.h>
#include "soc.h"
#include "config.h"
#include "immap.h"
#include <la9310.h>

void vSocInit( void )
{
    vSoCEnableUART();
}

void vSoCEnableUART( void )
{
    struct ccsr_pmux * pxPmuxCR = ( void * ) ( PMUXCR_BASE_ADDR );
    uint32_t ulPmux;

    ulPmux = IN_32( &pxPmuxCR->ulPmuxCR1 );
    ulPmux |= 1 << PMUXCR0_UART_PIN;
    OUT_32( &pxPmuxCR->ulPmuxCR1, ulPmux );
}

void vSocResetHandshake( void )
{
    uint32_t ulRstReady = 0, ulPbitCompletion, ulRcwCompletion;

    ulPbitCompletion = IN_32( ( uint32_t * ) PBI_COMPLETIONR );
    ulRcwCompletion = IN_32( ( uint32_t * ) RCW_COMPLETIONR );

    /* Write RCW Completion bit */
    OUT_32( ( uint32_t * ) RCW_COMPLETIONR, ulRcwCompletion |
            RCW_COMPLETION_DONE );
    /* Write PBI completion bit */
    OUT_32( ( uint32_t * ) PBI_COMPLETIONR, ulPbitCompletion |
            PBI_COMPLETION_DONE );
    /* Poll ready bit in ready status register */
    asm ( "isb" );
    asm ( "dsb" );

    do
    {
        ulRstReady = IN_32( ( uint32_t * ) RSTSR );

        if( ( ulRstReady & 0x01 ) == RST_READY )
        {
            break;
        }
    } while( 1 );
}
