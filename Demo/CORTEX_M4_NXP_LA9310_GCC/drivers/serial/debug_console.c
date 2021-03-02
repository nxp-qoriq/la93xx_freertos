/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2017-2018, 2021 NXP
 */

#include <common.h>
#include "serial_ns16550.h"
#include "print_scan.h"

static int debug_putc( int ch,
                       void * stream );
static void prvSendDataPolling( void * base,
                                const uint8_t * txBuff,
                                uint32_t txSize );
static void prvReceiveDataPolling( void * base,
                                   uint8_t * rxBuff,
                                   uint32_t rxSize );

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief Operation functions definitions for debug console. */
typedef struct DebugConsoleOperationFunctions
{
    void ( * Send )( void * base,
                     const uint8_t * buf,
                     uint32_t count );
    void ( * Receive )( void * base,
                        uint8_t * buf,
                        uint32_t count );
} debug_console_ops_t;

/*! @brief State structure storing debug console. */
typedef struct DebugConsoleState
{
    bool inited;             /*<! Identify debug console inited or not. */
    void * base;             /*<! Base of the IP register. */
    debug_console_ops_t ops; /*<! Operation function pointers for debug uart operations. */
} debug_console_state_t;

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Debug UART state information.*/
static debug_console_state_t s_debugConsole;

/*******************************************************************************
 * Code
 ******************************************************************************/


debug_console_status_t xDebugConsoleInit( void * base,
                                          uint32_t clockRate,
                                          uint32_t baudRate )
{
    s_debugConsole.base = base;
    s_debugConsole.ops.Send = prvSendDataPolling;
    s_debugConsole.ops.Receive = prvReceiveDataPolling;

    vSerialInit( base, baudRate, clockRate );

    s_debugConsole.inited = true;

    return status_DEBUGCONSOLE_Success;
}

int debug_printf( const char * fmt_s,
                  ... )
{
    va_list ap;
    int result;

    /* Do nothing if the debug uart is not initialized.*/
    if( !s_debugConsole.inited )
    {
        return -1;
    }

    va_start( ap, fmt_s );
    result = _doprint( NULL, debug_putc, -1, ( char * ) fmt_s, ap );
    va_end( ap );

    return result;
}

static int debug_putc( int ch,
                       void * stream )
{
    const unsigned char c = ( unsigned char ) ch;

    /* Do nothing if the debug uart is not initialized.*/
    if( !s_debugConsole.inited )
    {
        return -1;
    }

    s_debugConsole.ops.Send( s_debugConsole.base, &c, 1 );

    return 0;
}

int debug_putchar( int ch )
{
    /* Do nothing if the debug uart is not initialized.*/
    if( !s_debugConsole.inited )
    {
        return -1;
    }

    debug_putc( ch, NULL );

    return 1;
}

static int debug_getc( unsigned char * ch,
                       void * stream )
{
    /* Do nothing if the debug uart is not initialized.*/
    if( !s_debugConsole.inited )
    {
        return -1;
    }

    s_debugConsole.ops.Receive( s_debugConsole.base, ch, 1 );

    return 1;
}

int debug_getchar( unsigned char * ch )
{
    /* Do nothing if the debug uart is not initialized.*/
    if( !s_debugConsole.inited )
    {
        return -1;
    }

    debug_getc( ch, NULL );

    return 1;
}

void prvSendDataPolling( void * base,
                         const uint8_t * txBuff,
                         uint32_t txSize )
{
    vSerialWriteBlocking( base, txBuff, txSize );
}

void prvReceiveDataPolling( void * base,
                            uint8_t * rxBuff,
                            uint32_t rxSize )
{
    vSerialReadBlocking( base, rxBuff, rxSize );
}
