/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2021 NXP
 */

/* FreeRTOS includes. */
#include "FreeRTOS.h"

/* Standard includes. */
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

/* FreeRTOS+CLI includes. */
#include "FreeRTOS_CLI.h"

#include "debug_console.h"
#include "sync_timing_device.h"
#include "sync_timing_device_cli.h"
#include "sync_timing_common.h"
#include <phytimer.h>
#include "task.h"

static portBASE_TYPE prvTimesyncCLI( char * pcWriteBuffer,
                                     size_t xWriteBufferLen,
                                     const char * pcCommandString );

static const CLI_Command_Definition_t xTimesyncCLICommand =
{
    "sync", /* The command string to type. */
    "sync:\r\n"
    "\tsync 1 (boot)\r\n"
    "\tsync 2 (chip_mode)\r\n"
    "\tsync 3 (version)\r\n"
    "\tsync 4 <reg_addr> <num of bytes> (reg_read)\r\n"
    "\tsync 5 <reg_addr> <1 byte value> (reg_write)\r\n"
    "\tsync 6 (metadata)\r\n"
    "\tsync 7 <div> <steps> (var_dco)\r\n"
    "\tsync 8 (reset)\r\n"
    "\tsync 9 (reset_bl)\r\n",
    prvTimesyncCLI, /* The function to run. */
    -1              /* The user can enter any number of commands. */
};

void vRegisterTimesyncCLICommands( void )
{
    FreeRTOS_CLIRegisterCommand( &xTimesyncCLICommand );
}

static portBASE_TYPE prvTimesyncCLIShowHelp( void )
{
    log_info( "%s\r\n", xTimesyncCLICommand.pcHelpString );
    return pdFALSE;
}

static portBASE_TYPE prvTimesyncCLI( char * pcWriteBuffer,
                                     size_t xWriteBufferLen,
                                     const char * pcCommandString )
{
    BaseType_t lParameterStringLength;
    const char * pcParam1;
    const char * pcParam2;
    const char * pcParam3;
    uint32_t ulArg1 = 0;
    uint32_t ulArg2 = 0;
    uint32_t ulArg3 = 0;
    uint32_t ulTempVal1 = 0;
    uint32_t ulTempVal2 = 0;
    int32_t lArg;
    uint32_t i = 0;
    uint8_t ucReadBuff[ 16 ];
    SyncTimingDeviceMode_t xMode;

    /* Remove compile time warnings about unused parameters, and check the
     * write buffer is not NULL.  NOTE - for simplicity, this example assumes the
     * write buffer length is adequate, so does not check for buffer overflows. */
    ( void ) pcCommandString;
    ( void ) xWriteBufferLen;

    configASSERT( pcWriteBuffer );

    pcParam1 = FreeRTOS_CLIGetParameter( pcCommandString, 1, &lParameterStringLength );

    if( pcParam1 == NULL )
    {
        return prvTimesyncCLIShowHelp();
    }
    else
    {
        ulArg1 = strtoul( pcParam1, ( char ** ) NULL, 10 );
    }

    SyncTimingDeviceContext_t * pxContext = pxSyncTimingDeviceGetContext();

    if( pxContext == NULL )
    {
        return pdFALSE;
    }

    switch( ulArg1 )
    {
        case eSyncTimingDeviceCommandBoot:
            ulTempVal1 = ulPhyTimerCapture( PHY_TIMER_COMP_PPS_IN );
            xSyncTimingDeviceFWAPIBoot( pxContext );
            ulTempVal2 = ulPhyTimerDiffToUS( ulTempVal1, ulPhyTimerCapture( PHY_TIMER_COMP_PPS_IN ) );
            break;

        case eSyncTimingDeviceCommandGetMode:
            ulTempVal1 = ulPhyTimerCapture( PHY_TIMER_COMP_PPS_IN );
            xSyncTimingDeviceGetChipsetMode( pxContext, &xMode );
            ulTempVal2 = ulPhyTimerDiffToUS( ulTempVal1, ulPhyTimerCapture( PHY_TIMER_COMP_PPS_IN ) );
            log_info( "Sync timing device mode: %d\r\n", xMode );
            break;

        case eSyncTimingDeviceCommandGetVersion:
            ulTempVal1 = ulPhyTimerCapture( PHY_TIMER_COMP_PPS_IN );
            xSyncTimingDeviceGetVersionInfo( pxContext );
            ulTempVal2 = ulPhyTimerDiffToUS( ulTempVal1, ulPhyTimerCapture( PHY_TIMER_COMP_PPS_IN ) );
            break;

        case eSyncTimingDeviceCommandRegRead:
            pcParam2 = FreeRTOS_CLIGetParameter( pcCommandString, 2, &lParameterStringLength );

            if( pcParam2 == NULL )
            {
                return prvTimesyncCLIShowHelp();
            }

            ulArg2 = strtoul( pcParam2, ( char ** ) NULL, 10 );
            pcParam3 = FreeRTOS_CLIGetParameter( pcCommandString, 3, &lParameterStringLength );

            if( pcParam3 == NULL )
            {
                return prvTimesyncCLIShowHelp();
            }

            ulArg3 = strtoul( pcParam3, ( char ** ) NULL, 10 );

            if( ulArg3 <= 16 )
            {
                xSyncTimingDeviceMemReadDirect( pxContext, ulArg2, &ucReadBuff[ 0 ], ulArg3 );

                for( i = 0; i < 16; i++ )
                {
                    log_info( "0x%x ", ucReadBuff[ i ] );
                }

                log_info( "\r\n" );
            }

            break;

        case eSyncTimingDeviceCommandRegWrite:
            pcParam2 = FreeRTOS_CLIGetParameter( pcCommandString, 2, &lParameterStringLength );

            if( pcParam2 == NULL )
            {
                return prvTimesyncCLIShowHelp();
            }

            ulArg2 = strtoul( pcParam2, ( char ** ) NULL, 10 );
            pcParam3 = FreeRTOS_CLIGetParameter( pcCommandString, 3, &lParameterStringLength );

            if( pcParam3 == NULL )
            {
                return prvTimesyncCLIShowHelp();
            }

            ulArg3 = strtoul( pcParam3, ( char ** ) NULL, 10 );
            xSyncTimingDeviceMemWriteDirect( pxContext, ulArg2, ( uint8_t * ) &ulArg3, 1 );
            break;

        case eSyncTimingDeviceCommandPrintMetadata:
            ulTempVal1 = ulPhyTimerCapture( PHY_TIMER_COMP_PPS_IN );
            xSyncTimingDeviceFWAPIMetadata( pxContext );
            ulTempVal2 = ulPhyTimerDiffToUS( ulTempVal1, ulPhyTimerCapture( PHY_TIMER_COMP_PPS_IN ) );
            break;

        case eSyncTimingDeviceCommandSetDCO:
            pcParam2 = FreeRTOS_CLIGetParameter( pcCommandString, 2, &lParameterStringLength );

            if( pcParam2 == NULL )
            {
                return prvTimesyncCLIShowHelp();
            }

            ulArg2 = strtoul( pcParam2, ( char ** ) NULL, 10 );
            pcParam3 = FreeRTOS_CLIGetParameter( pcCommandString, 3, &lParameterStringLength );

            if( pcParam3 == NULL )
            {
                return prvTimesyncCLIShowHelp();
            }

            lArg = ( int ) strtol( pcParam3, ( char ** ) NULL, 10 );
            ulTempVal1 = ulPhyTimerCapture( PHY_TIMER_COMP_PPS_IN );
            xSyncTimingDeviceFWAPIVariableOffsetDco( pxContext, ulArg2, lArg );
            ulTempVal2 = ulPhyTimerDiffToUS( ulTempVal1, ulPhyTimerCapture( PHY_TIMER_COMP_PPS_IN ) );
            break;

        case eSyncTimingDeviceCommandSetReset:
            ulTempVal1 = ulPhyTimerCapture( PHY_TIMER_COMP_PPS_IN );
            xSyncTimingDeviceReset( pxContext, SYNC_TIMING_DEVICE_RESET_TOGGLE );
            ulTempVal2 = ulPhyTimerDiffToUS( ulTempVal1, ulPhyTimerCapture( PHY_TIMER_COMP_PPS_IN ) );
            break;

        case eSyncTimingDeviceCommandSetResetBL:
            ulTempVal1 = ulPhyTimerCapture( PHY_TIMER_COMP_PPS_IN );
            xSyncTimingDeviceReset( pxContext, SYNC_TIMING_DEVICE_RESET_BOOTLOADER_MODE );
            ulTempVal2 = ulPhyTimerDiffToUS( ulTempVal1, ulPhyTimerCapture( PHY_TIMER_COMP_PPS_IN ) );
            break;

        default:
            return prvTimesyncCLIShowHelp();
    }

    log_info( "Sync test id %d latency: %d\r\n", ulArg1, ulTempVal2 );

    return pdFALSE;
}
