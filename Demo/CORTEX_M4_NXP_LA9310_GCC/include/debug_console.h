/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2017-2021 NXP
 */

#ifndef __DEBUG_CONSOLE_H__
#define __DEBUG_CONSOLE_H__

#include <stdint.h>
#include "la9310_main.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define IO_MAXLINE    20

/*! @brief Configuration for toolchain's printf or NXP version printf */
#define PRINTF        debug_printf
#define PUTCHAR       debug_putchar

#define log_err( ... )                                             \
    do {                                                           \
        struct la9310_hif * pHif = pLa9310Info->pHif;              \
        struct debug_log_regs * pDbgLogRegs = &pHif->dbg_log_regs; \
        if( pDbgLogRegs->log_level >= LA9310_LOG_LEVEL_ERR )       \
        PRINTF( __VA_ARGS__ );                                     \
    } while( 0 )

#define log_info( ... )                                            \
    do {                                                           \
        struct la9310_hif * pHif = pLa9310Info->pHif;              \
        struct debug_log_regs * pDbgLogRegs = &pHif->dbg_log_regs; \
        if( pDbgLogRegs->log_level >= LA9310_LOG_LEVEL_INFO )      \
        PRINTF( __VA_ARGS__ );                                     \
    }while( 0 )

#define log_dbg( ... )                                             \
    do {                                                           \
        struct la9310_hif * pHif = pLa9310Info->pHif;              \
        struct debug_log_regs * pDbgLogRegs = &pHif->dbg_log_regs; \
        if( pDbgLogRegs->log_level >= LA9310_LOG_LEVEL_DBG )       \
        PRINTF( __VA_ARGS__ );                                     \
    }while( 0 )

#define log_isr( ... )                                             \
    do {                                                           \
        struct la9310_hif * pHif = pLa9310Info->pHif;              \
        struct debug_log_regs * pDbgLogRegs = &pHif->dbg_log_regs; \
        if( pDbgLogRegs->log_level >= LA9310_LOG_LEVEL_ISR )       \
        PRINTF( __VA_ARGS__ );                                     \
    }while( 0 )

extern uint32_t ulMemLogIndex;

/*! @brief Error code for the debug console driver. */
typedef enum _debug_console_status
{
    status_DEBUGCONSOLE_Success = 0U,
    status_DEBUGCONSOLE_InvalidDevice,
    status_DEBUGCONSOLE_AllocateMemoryFailed,
    status_DEBUGCONSOLE_Failed
} debug_console_status_t;

/*******************************************************************************
 * API
 ******************************************************************************/

/*!
 * @brief Initializes the UART used for debug messages.
 *
 * Call this function to enable debug log messages to be output through the specified UART
 * base address and at the specified baud rate. Initialize the UART to the given baud
 * rate and 8N1. After this function returns, stdout gets connected to the
 * selected UART. The debug_printf() function also uses this UART.
 *
 * @param base Which UART instance is used to send debug messages.
 * @param clockRate The input clock of UART module.
 * @param baudRate The desired baud rate in bits per second.
 * @return Whether initialization is successful or not.
 */
debug_console_status_t xDebugConsoleInit( void * base,
                                          uint32_t clockRate,
                                          uint32_t baudRate );

/*!
 * @brief   Prints formatted output to the standard output stream.
 *
 * Call this function to print formatted output to the standard output stream.
 *
 * @param   fmt_s   Format control string.
 * @return  Returns the number of characters printed, or a negative value if an error occurs.
 */
int debug_printf( const char * fmt_s,
                  ... );

/*!
 * @brief   Writes a character to stdout.
 *
 * Call this function to write a character to stdout.
 *
 * @param   ch  Character to be written.
 * @return  Returns the character written.
 */
int debug_putchar( int ch );

/*!
 * @brief   Reads a character to stdin.
 *
 * Call this function to write a character to stdout.
 *
 * @param   ch  Character to be written.
 * @return  Returns the character written.
 */
int debug_getchar( unsigned char * ch );

void vMemlogWrite( const uint8_t * pucData,
                   size_t xLength );
#endif /* __DEBUG_CONSOLE_H__ */
