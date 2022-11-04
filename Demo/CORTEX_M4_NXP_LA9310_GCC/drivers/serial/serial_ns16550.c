/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2017, 2021-2022 NXP
 */

#include <common.h>
#include <la9310.h>
#include "serial_ns16550.h"

#define SERIAL_LCRVAL    SERIAL_LCR_8N1
#define SERIAL_FCRVAL      \
    ( SERIAL_FCR_FIFO_EN | \
      SERIAL_FCR_RXSR |    \
      SERIAL_FCR_TXSR )
#define NS16550_IER      0x0

extern uint32_t BootSource;

void vSerialInit( NS16550_t base,
                  uint32_t ulBaudRate,
                  uint32_t ulSrcClockHz )
{
    int lBaudDivisor;

    while( !( IN_8( &base->lsr ) & SERIAL_LSR_TEMT ) )
    {
    }

    lBaudDivisor = ulSrcClockHz / ( 16 * ulBaudRate );

    OUT_8( &base->ier, NS16550_IER );
    OUT_8( &base->fcr, SERIAL_FCRVAL );
    OUT_8( &base->lcr, SERIAL_LCR_BKSE | SERIAL_LCRVAL );

    dmb();
    OUT_8( &base->dll, lBaudDivisor & 0xff );
    OUT_8( &base->dlm, ( lBaudDivisor >> 8 ) & 0xff );

    dmb();
    OUT_8( &base->lcr, SERIAL_LCRVAL );
}

void vSerialWriteBlocking( NS16550_t xBase,
                           const uint8_t * pucData,
                           size_t xLength )
{
	#ifdef TURN_ON_HOST_MODE
        vMemlogWrite( pucData, xLength );
	#endif
    while( xLength-- )
    {
        while( !( IN_8( &xBase->lsr ) & SERIAL_LSR_THRE ) )
        {
        }

        OUT_8( &xBase->thr, *pucData++ );
    }
}

void vSerialReadBlocking( NS16550_t xBase,
                          uint8_t * pucData,
                          size_t xLength )
{
    while( xLength )
    {
        if( !( IN_8( &xBase->lsr ) & SERIAL_LSR_DR ) )
        {
        }
        else
        {
            *pucData++ = IN_8( &xBase->rbr );
            xLength--;
        }
    }
}
