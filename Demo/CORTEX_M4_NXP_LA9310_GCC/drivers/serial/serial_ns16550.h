/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright 2017, 2021 NXP
 */

#ifndef _SERIAL_NS16550_H_
#define _SERIAL_NS16550_H_

#include <stdint.h>
#include <string.h>
#include <io.h>

#define SERIAL_REG( x )    volatile unsigned char x;


struct NS16550
{
    SERIAL_REG( rbr );  /* 0 */
    SERIAL_REG( ier );  /* 1 */
    SERIAL_REG( fcr );  /* 2 */
    SERIAL_REG( lcr );  /* 3 */
    SERIAL_REG( mcr );  /* 4 */
    SERIAL_REG( lsr );  /* 5 */
    SERIAL_REG( msr );  /* 6 */
    SERIAL_REG( spr );  /* 7 */
    SERIAL_REG( mdr1 ); /* 8 */
    SERIAL_REG( reg9 ); /* 9 */
    SERIAL_REG( regA ); /* A */
    SERIAL_REG( regB ); /* B */
    SERIAL_REG( regC ); /* C */
    SERIAL_REG( regD ); /* D */
    SERIAL_REG( regE ); /* E */
    SERIAL_REG( uasr ); /* F */
    SERIAL_REG( scr );  /* 10*/
    SERIAL_REG( ssr );  /* 11*/
};

#define thr    rbr
#define iir    fcr
#define dll    rbr
#define dlm    ier

typedef struct NS16550 * NS16550_t;

#define SERIAL_FCR_FIFO_EN    0x01

#define SERIAL_FCR_RXSR       0x02
#define SERIAL_FCR_TXSR       0x04

#define SERIAL_LSR_DR         0x01
#define SERIAL_LCR_BKSE       0x80
#define SERIAL_LSR_THRE       0x20
#define SERIAL_LCR_8N1        0x03
#define SERIAL_LSR_TEMT       0x40  /* Xmitter empty */


void vSerialInit( NS16550_t base,
                  uint32_t ulBaudRate,
                  uint32_t ulSrcClockHz );
void vSerialWriteBlocking( NS16550_t xBase,
                           const uint8_t * pucData,
                           size_t xLength );
void vSerialReadBlocking( NS16550_t xBase,
                          uint8_t * pucData,
                          size_t xLength );

#endif /* _SERIAL_NS16550_H_ */
