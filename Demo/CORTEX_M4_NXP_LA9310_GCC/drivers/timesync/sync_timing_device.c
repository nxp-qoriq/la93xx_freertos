/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2021 NXP
 */

#include <stdlib.h>
#include <string.h>
#include <sync_timing_device.h>
#include <la9310_i2cAPI.h>
#include <sync_timing_aruba_cmd_map.h>
#include <sync_timing_core_aruba_interface.h>
#include <sync_timing_common.h>
#include <sync_timing_aruba_cmd.h>
#include "FreeRTOS.h"
#include "task.h"
#include <delay.h>

#define FW_PATCH_PATTERN_SIZE       7
#define FW_PATCH_PATTERN_SCAN_LEN   128

SyncTimingDeviceContext_t xSyncTimingDevice;

const char * SyncTimingDeviceModeNames[ 3 ] = { "Application", "Bootloader", "Invalid" };

SyncTimingDeviceContext_t * pxSyncTimingDeviceGetContext()
{
    if( xSyncTimingDevice.deviceStatus != eSyncTimingDeviceStatusInitialized )
    {
        log_err( "%s: uninitalized\r\n", __func__ );
        return NULL;
    }
    else
    {
        return &xSyncTimingDevice;
    }
}

SyncTimingDeviceContext_t * pxSyncTimingDeviceInit()
{
    if( xSyncTimingDevice.deviceStatus == eSyncTimingDeviceStatusInitialized )
    {
        return &xSyncTimingDevice;
    }

    xSyncTimingDevice.i2cInfo.i2cHandle = SYNC_TIMING_DEVICE_I2C_HANDLE;
    xSyncTimingDevice.i2cInfo.i2cAddr = SYNC_TIMING_DEVICE_I2C_ADDR;
    xSyncTimingDevice.i2cInfo.portalAddr = SYNC_TIMING_DEVICE_MCU_PORTAL_ADDR;
    xSyncTimingDevice.i2cInfo.portalAddrLen = SYNC_TIMING_DEVICE_MCU_PORTAL_ADDR_LEN;

    xSyncTimingDeviceGetVersionInfo( &xSyncTimingDevice );

    xSyncTimingDevice.deviceStatus = eSyncTimingDeviceStatusInitialized;

    return &xSyncTimingDevice;
}

static SyncStatus_t prvSyncTimingDeviceTrxCommand( SyncTimingDeviceContext_t * pxContext,
                                                   uint8_t * pucBuff,
                                                   uint32_t ulLen,
                                                   SyncTransactionType_t xTrxType )
{
    SyncStatus_t returnStatus = eSyncStatusFailure;
    int ret = -1;

    switch( xTrxType )
    {
        case eSyncWriteTransaction:
            ret = iLa9310_I2C_Write( pxContext->i2cInfo.i2cHandle,
                                     pxContext->i2cInfo.i2cAddr,
                                     pxContext->i2cInfo.portalAddr,
                                     pxContext->i2cInfo.portalAddrLen,
                                     pucBuff,
                                     ulLen );

            if( ret == ulLen )
            {
                returnStatus = eSyncStatusSuccess;
            }

            break;

        case eSyncReadTransaction:
            ret = iLa9310_I2C_Read( pxContext->i2cInfo.i2cHandle,
                                    pxContext->i2cInfo.i2cAddr,
                                    pxContext->i2cInfo.portalAddr,
                                    pxContext->i2cInfo.portalAddrLen,
                                    pucBuff,
                                    ulLen );

            if( ret == ulLen )
            {
                returnStatus = eSyncStatusSuccess;
            }

            break;
    }

    return returnStatus;
}

SyncStatus_t xSyncTimingDeviceMemWriteDirect( SyncTimingDeviceContext_t * pxContext,
                                              uint16_t usAddr,
                                              uint8_t * pucBuff,
                                              uint32_t ulLen )
{
    SyncStatus_t returnStatus = eSyncStatusFailure;
    int ret = -1;
    uint32_t usModifiedAddr = ( usAddr >> 8 ) | ( ( usAddr & 0xFF ) << 8 );

    ret = iLa9310_I2C_Write( pxContext->i2cInfo.i2cHandle,
                             pxContext->i2cInfo.i2cAddr,
                             usModifiedAddr,
                             sizeof( usAddr ),
                             pucBuff,
                             ulLen );

    if( ret == ulLen )
    {
        returnStatus = eSyncStatusSuccess;
    }

    return returnStatus;
}

SyncStatus_t xSyncTimingDeviceMemReadDirect( SyncTimingDeviceContext_t * pxContext,
                                             uint16_t usAddr,
                                             uint8_t * pucBuff,
                                             uint32_t ulLen )
{
    SyncStatus_t returnStatus = eSyncStatusFailure;
    int ret = -1;
    uint32_t usModifiedAddr = ( usAddr >> 8 ) | ( ( usAddr & 0xFF ) << 8 );

    ret = iLa9310_I2C_Write( pxContext->i2cInfo.i2cHandle,
                             pxContext->i2cInfo.i2cAddr,
                             usModifiedAddr,
                             sizeof( usAddr ),
                             NULL,
                             0 );

    if( ret != 0 )
    {
        return returnStatus;
    }

    ret = iLa9310_I2C_Read( pxContext->i2cInfo.i2cHandle,
                            pxContext->i2cInfo.i2cAddr,
                            0,
                            0,
                            pucBuff,
                            ulLen );

    if( ret == ulLen )
    {
        returnStatus = eSyncStatusSuccess;
    }

    return returnStatus;
}

static SyncStatus_t prvSyncTimingDeviceWaitForCTS( SyncTimingDeviceContext_t * pxContext )
{
    SyncStatus_t returnStatus = eSyncStatusFailure;
    uint32_t count = 0;
    uint8_t ucBuff[32];

    while( 1 )
    {
        returnStatus = prvSyncTimingDeviceTrxCommand( pxContext, &ucBuff[0], 32, eSyncReadTransaction );

        if( returnStatus == eSyncStatusSuccess )
        {
            if( ucBuff[0] & SYNC_TIMING_DEVICE_CLEAR_TO_SEND )
            {
                return returnStatus;
            }
        }
        else
        {
            log_err( "%s: Data read error\r\n", __func__ );
            return returnStatus;
        }

        if( ++count == SYNC_TIMING_DEVICE_RETRY_COUNT )
        {
            log_err( "%s: waited too long\r\n", __func__ );
            return eSyncStatusTimeout;
        }
    }

    return returnStatus;
}

SyncStatus_t xSyncTimingDeviceGetChipsetMode( SyncTimingDeviceContext_t * pxContext,
                                              SyncTimingDeviceMode_t * pxMode )
{
    SyncStatus_t returnStatus = eSyncStatusFailure;
    uint8_t ucMCUBootState = 0;

    if( pxMode == NULL )
    {
        return returnStatus;
    }

    returnStatus = xSyncTimingDeviceMemReadDirect( pxContext,
                                                   SYNC_TIMING_MCU_BOOTSTATE,
                                                   &ucMCUBootState,
                                                   1 );

    if( returnStatus != eSyncStatusSuccess )
    {
        log_err( "%s: SYNC_TIMING_MCU_BOOTSTATE read error\r\n", __func__ );
        return returnStatus;
    }

    if( ( ucMCUBootState > 0 ) && ( ucMCUBootState <= BOOTLOADER_READY_STATE ) )
    {
        *pxMode = eSyncTimingDeviceModeBootloader;
    }
    else if( ucMCUBootState >= APPLICATION_READY_STATE )
    {
        *pxMode = eSyncTimingDeviceModeAppln;
    }
    else
    {
        *pxMode = eSyncTimingDeviceModeInvalid;
    }

    pxContext->deviceMode = *pxMode;

    return returnStatus;
}

SyncStatus_t xSyncTimingDeviceFWAPIBoot( SyncTimingDeviceContext_t * pxContext )
{
    SyncStatus_t returnStatus = eSyncStatusFailure;
    struct cmd_BOOT_map xCmdBoot;

    xCmdBoot.CMD = cmd_ID_BOOT;

    returnStatus = prvSyncTimingDeviceWaitForCTS( pxContext );

    if( returnStatus != eSyncStatusSuccess )
    {
        return returnStatus;
    }

    returnStatus = prvSyncTimingDeviceTrxCommand( pxContext,
                                                  ( uint8_t * ) &xCmdBoot,
                                                  sizeof( xCmdBoot ),
                                                  eSyncWriteTransaction );

    if( returnStatus != eSyncStatusSuccess )
    {
        log_err( "%s: Data write error\r\n", __func__ );
        return returnStatus;
    }

    return returnStatus;
}

SyncStatus_t xSyncTimingDeviceFWAPIMetadata( SyncTimingDeviceContext_t * pxContext )
{
    SyncStatus_t returnStatus = eSyncStatusFailure;
    reply_METADATA_map_t * pxReplyMetadata = NULL;
    cmd_METADATA_map_t xCmdMetadata;
    uint8_t ucBuff[ sizeof( reply_METADATA_map_t ) + 1 ];
    uint32_t count = 0;

    returnStatus = prvSyncTimingDeviceWaitForCTS( pxContext );

    if( returnStatus != eSyncStatusSuccess )
    {
        return returnStatus;
    }

    xCmdMetadata.CMD = cmd_ID_METADATA;

    returnStatus = prvSyncTimingDeviceTrxCommand( pxContext,
                                                  ( uint8_t * ) &xCmdMetadata,
                                                  sizeof( xCmdMetadata ),
                                                  eSyncWriteTransaction );

    if( returnStatus != eSyncStatusSuccess )
    {
        log_err( "%s: data write error\r\n", __func__ );
        return returnStatus;
    }

    while( 1 )
    {
        returnStatus = prvSyncTimingDeviceTrxCommand( pxContext,
                                                      &ucBuff[ 0 ],
                                                      sizeof( reply_METADATA_map_t ) + 1,
                                                      eSyncReadTransaction );

        if( returnStatus != eSyncStatusSuccess )
        {
            log_err( "%s: data read error\r\n", __func__ );
            return returnStatus;
        }

        if( ucBuff[ 0 ] & SYNC_TIMING_DEVICE_CMD_REPLY_READY )
        {
            break;
        }

        if( ++count == SYNC_TIMING_DEVICE_RETRY_COUNT )
        {
            log_err( "%s: waited too long\r\n", __func__ );
            return eSyncStatusTimeout;
        }
    }

    pxReplyMetadata = ( reply_METADATA_map_t * ) &ucBuff[ 1 ];

    log_info( "DCO_NA_STEP_SIZE = 0x%x ppt\r\nDCO_MR_STEP_SIZE = 0x%x ppt\r\nDCO_MA_STEP_SIZE = 0x%x ppt\r\n",
              pxReplyMetadata->DCO_NA_STEP_SIZE,
              pxReplyMetadata->DCO_MR_STEP_SIZE,
              pxReplyMetadata->DCO_MA_STEP_SIZE );
    log_info( "DCO_NB_STEP_SIZE = 0x%x ppt\r\nDCO_MB_STEP_SIZE = 0x%x ppt\r\nPHASE_JAM_PPS_OUT_RANGE_HIGH = 0x%x\r\n",
              pxReplyMetadata->DCO_NB_STEP_SIZE,
              pxReplyMetadata->DCO_MB_STEP_SIZE,
              pxReplyMetadata->PHASE_JAM_PPS_OUT_RANGE_HIGH );
    log_info( "PHASE_JAM_PPS_OUT_RANGE_LOW = 0x%x\r\n", pxReplyMetadata->PHASE_JAM_PPS_OUT_RANGE_LOW );
    log_info( "PHASE_JAM_PPS_OUT_STEP_SIZE MSB = 0x%x\r\nPHASE_JAM_PPS_OUT_STEP_SIZE LSB = 0x%x\r\n",
              pxReplyMetadata->PHASE_JAM_PPS_OUT_STEP_SIZE >> 32,
              pxReplyMetadata->PHASE_JAM_PPS_OUT_STEP_SIZE & 0xFFFFFFFF );
    log_info( "PLAN_OPTIONS: %d\r\n", pxReplyMetadata->PLAN_OPTIONS );

    return returnStatus;
}

SyncStatus_t xSyncTimingDeviceFWAPIVariableOffsetDco( SyncTimingDeviceContext_t * pxContext,
                                                      uint8_t ucDivider,
                                                      int32_t lSteps )
{
    SyncStatus_t returnStatus = eSyncStatusFailure;
    uint32_t count = 0;
    cmd_VARIABLE_OFFSET_DCO_map_t xCmdDcoVo;
    reply_VARIABLE_OFFSET_DCO_map_t * pxReplyDcoVo;
    uint8_t ucBuff[ sizeof( reply_VARIABLE_OFFSET_DCO_map_t ) + 1 ];

    if( ( ucDivider != 0x1 ) &&
        ( ucDivider != 0x2 ) &&
        ( ucDivider != 0x4 ) &&
        ( ucDivider != 0x8 ) &&
        ( ucDivider != 0x10 ) )
    {
        return returnStatus;
    }

    returnStatus = prvSyncTimingDeviceWaitForCTS( pxContext );

    if( returnStatus != eSyncStatusSuccess )
    {
        return returnStatus;
    }

    xCmdDcoVo.CMD = cmd_ID_VARIABLE_OFFSET_DCO;
    xCmdDcoVo.DIVIDER_SELECT = ucDivider;
    xCmdDcoVo.OFFSET = lSteps;

    returnStatus = prvSyncTimingDeviceTrxCommand( pxContext,
                                                  ( uint8_t * ) &xCmdDcoVo,
                                                  sizeof( xCmdDcoVo ),
                                                  eSyncWriteTransaction );

    if( returnStatus != eSyncStatusSuccess )
    {
        log_err( "%s: data write error\r\n", __func__ );
        return returnStatus;
    }

    while( 1 )
    {
        returnStatus = prvSyncTimingDeviceTrxCommand( pxContext,
                                                      &ucBuff[ 0 ],
                                                      sizeof( reply_VARIABLE_OFFSET_DCO_map_t ) + 1,
                                                      eSyncReadTransaction );

        if( returnStatus != eSyncStatusSuccess )
        {
            log_err( "%s: data read error\r\n", __func__ );
            return returnStatus;
        }

        if( ucBuff[ 0 ] & SYNC_TIMING_DEVICE_CMD_REPLY_READY )
        {
            break;
        }

        if( ++count == SYNC_TIMING_DEVICE_RETRY_COUNT )
        {
            log_err( "%s: waited too long\r\n", __func__ );
            return eSyncStatusTimeout;
        }
    }

    pxReplyDcoVo = ( reply_VARIABLE_OFFSET_DCO_map_t * ) &ucBuff[ 1 ];
    ( void ) pxReplyDcoVo;

    return returnStatus;
}

static SyncStatus_t prvSyncTimingDeviceGetDeviceInfo( SyncTimingDeviceContext_t * pxContext )
{
    SyncStatus_t returnStatus = eSyncStatusFailure;
    uint32_t count = 0;
    cmd_DEVICE_INFO_map_t xCmdDevInfo;
    reply_DEVICE_INFO_map_t * pxDeviceInfo = NULL;
    uint8_t ucBuff[ sizeof( reply_DEVICE_INFO_map_t ) + 1 ];

    returnStatus = prvSyncTimingDeviceWaitForCTS( pxContext );

    if( returnStatus != eSyncStatusSuccess )
    {
        return returnStatus;
    }

    xCmdDevInfo.CMD = cmd_ID_DEVICE_INFO;

    returnStatus = prvSyncTimingDeviceTrxCommand( pxContext,
                                                  ( uint8_t * ) &xCmdDevInfo,
                                                  sizeof( cmd_DEVICE_INFO_map_t ),
                                                  eSyncWriteTransaction );

    if( returnStatus != eSyncStatusSuccess )
    {
        log_err( "%s: data write error\r\n", __func__ );
        return returnStatus;
    }

    while( 1 )
    {
        returnStatus = prvSyncTimingDeviceTrxCommand( pxContext,
                                                      &ucBuff[ 0 ],
                                                      sizeof( reply_DEVICE_INFO_map_t ) + 1,
                                                      eSyncReadTransaction );

        if( returnStatus != eSyncStatusSuccess )
        {
            log_err( "%s: data read error\r\n", __func__ );
            return returnStatus;
        }

        if( ucBuff[ 0 ] & SYNC_TIMING_DEVICE_CMD_REPLY_READY )
        {
            break;
        }

        if( ++count == SYNC_TIMING_DEVICE_RETRY_COUNT )
        {
            log_err( "%s: waited too long\r\n", __func__ );
            return eSyncStatusTimeout;
        }
    }

    pxDeviceInfo = ( reply_DEVICE_INFO_map_t * ) &ucBuff[ 1 ];

    log_info( "sync_timing_device blversion: %u.%u.%u.%u\r\n",
              ( uint32_t ) pxDeviceInfo->ROM,
              ( uint32_t ) pxDeviceInfo->MROM,
              ( uint32_t ) pxDeviceInfo->BROM,
              ( uint32_t ) pxDeviceInfo->SVN );
    pxContext->deviceRevision = ( pxDeviceInfo->M0 >> 0x4 ) & 0x0F;
    pxContext->deviceSubRevision = ( pxDeviceInfo->M0 ) & 0x0F;

    log_info( "sync_timing_device device revision: %u, subrevision: %u\r\n",
              pxContext->deviceRevision,
              pxContext->deviceSubRevision );

    return returnStatus;
}

static SyncStatus_t prvSyncTimingDeviceGetAppInfo( SyncTimingDeviceContext_t * pxContext )
{
    SyncStatus_t returnStatus = eSyncStatusFailure;
    uint32_t count = 0;
    cmd_APP_INFO_map_t xCmdAppInfo;
    reply_APP_INFO_map_t * pxAppInfo = NULL;
    uint8_t ucBuff[ sizeof( reply_APP_INFO_map_t ) + 1 ];

    returnStatus = prvSyncTimingDeviceWaitForCTS( pxContext );

    if( returnStatus != eSyncStatusSuccess )
    {
        return returnStatus;
    }

    xCmdAppInfo.CMD = cmd_ID_APP_INFO;

    returnStatus = prvSyncTimingDeviceTrxCommand( pxContext,
                                                  ( uint8_t * ) &xCmdAppInfo,
                                                  sizeof( cmd_APP_INFO_map_t ),
                                                  eSyncWriteTransaction );

    if( returnStatus != eSyncStatusSuccess )
    {
        log_err( "%s: data write error\r\n", __func__ );
        return returnStatus;
    }

    while( 1 )
    {
        returnStatus = prvSyncTimingDeviceTrxCommand( pxContext,
                                                      &ucBuff[ 0 ],
                                                      sizeof( reply_APP_INFO_map_t ) + 1,
                                                      eSyncReadTransaction );

        if( returnStatus != eSyncStatusSuccess )
        {
            log_err( "%s: data read error\r\n", __func__ );
            return returnStatus;
        }

        if( ucBuff[ 0 ] & SYNC_TIMING_DEVICE_CMD_REPLY_READY )
        {
            break;
        }

        if( ++count == SYNC_TIMING_DEVICE_RETRY_COUNT )
        {
            log_err( "%s: waited too long\r\n", __func__ );
            return eSyncStatusTimeout;
        }
    }

    pxAppInfo = ( reply_APP_INFO_map_t * ) &ucBuff[ 1 ];
    log_info( "sync_timing_device App Version = %u.%u.%u_svn_%u\r\n",
              ( uint32_t ) pxAppInfo->A_MAJOR,
              ( uint32_t ) pxAppInfo->A_MINOR,
              ( uint32_t ) pxAppInfo->A_BRANCH,
              ( uint32_t ) pxAppInfo->A_BUILD );
    log_info( "sync_timing_device Planner Version = %u.%u.%u_svn_%u\r\n",
              ( uint32_t ) pxAppInfo->P_MAJOR,
              ( uint32_t ) pxAppInfo->P_MINOR,
              ( uint32_t ) pxAppInfo->P_BRANCH,
              ( uint32_t ) pxAppInfo->P_BUILD );

    log_info( "sync_timing_device Fplan Design ID = %c%c%c%c%c%c%c%c\r\n",
              pxAppInfo->DESIGN_ID[ 0 ],
              pxAppInfo->DESIGN_ID[ 1 ],
              pxAppInfo->DESIGN_ID[ 2 ],
              pxAppInfo->DESIGN_ID[ 3 ],
              pxAppInfo->DESIGN_ID[ 4 ],
              pxAppInfo->DESIGN_ID[ 5 ],
              pxAppInfo->DESIGN_ID[ 6 ],
              pxAppInfo->DESIGN_ID[ 7 ] );

    pxContext->fwVersionMajor = ( uint32_t ) pxAppInfo->A_MAJOR;
    pxContext->fwVersionMinor = ( uint32_t ) pxAppInfo->A_MINOR;
    pxContext->fwVersionBuildNum = ( uint32_t ) pxAppInfo->A_BUILD;

    return returnStatus;
}

SyncStatus_t xSyncTimingDeviceGetVersionInfo( SyncTimingDeviceContext_t * pxContext )
{
    SyncStatus_t returnStatus = eSyncStatusFailure;
    SyncTimingDeviceMode_t xMode = eSyncTimingDeviceModeInvalid;

    returnStatus = prvSyncTimingDeviceGetDeviceInfo( pxContext );

    if( returnStatus != eSyncStatusSuccess )
    {
        log_err( "sync timing device GetDeviceInfo failed\r\n" );
        return returnStatus;
    }

    returnStatus = xSyncTimingDeviceGetChipsetMode( pxContext, &xMode );

    if( returnStatus != eSyncStatusSuccess )
    {
        log_err( "sync timing device GetChipsetMode failed\r\n" );
        return returnStatus;
    }

    if( xMode == eSyncTimingDeviceModeAppln )
    {
        returnStatus = prvSyncTimingDeviceGetAppInfo( pxContext );

        if( returnStatus != eSyncStatusSuccess )
        {
            log_err( "sync timing device GetAppInfo failed\r\n" );
            return returnStatus;
        }
    }
    else
    {
        log_info( "Device mode is %d so not trying to get fw info\r\n", xMode );
    }

    return returnStatus;
}

SyncStatus_t xSyncTimingDeviceReset( SyncTimingDeviceContext_t * pxContext,
                                     SYNC_TIMING_DEVICE_RESET_TYPE_E eResetType )
{
    SyncStatus_t returnStatus = eSyncStatusFailure;
    SyncTimingDeviceMode_t xMode = eSyncTimingDeviceModeInvalid;
    cmd_RESTART_map_t xCmdRestart = { 0 };
    uint8_t ucMCUBootState = 0;
    uint32_t ulCount = 0;

    returnStatus = prvSyncTimingDeviceWaitForCTS( pxContext );

    if( returnStatus != eSyncStatusSuccess )
    {
        return returnStatus;
    }

    xCmdRestart.CMD = cmd_ID_RESTART;

    if( ( pxContext->deviceRevision == 0x0 ) &&
        ( pxContext->deviceSubRevision == 0x1 ) &&
        ( pxContext->deviceMode == eSyncTimingDeviceModeAppln ) )
    {
        xCmdRestart.CMD = 0xDF;
    }

    switch( eResetType )
    {
        case SYNC_TIMING_DEVICE_RESET_BOOTLOADER_MODE:
            xCmdRestart.OPTIONS |= CMD_RESTART_ARG_OPTIONS_WAIT_TRUE_BIT;
            break;

        default:
            xCmdRestart.OPTIONS = 0;
            break;
    }

    returnStatus = prvSyncTimingDeviceTrxCommand( pxContext,
                                                  ( uint8_t * ) &xCmdRestart,
                                                  sizeof( cmd_RESTART_map_t ),
                                                  eSyncWriteTransaction );

    if( returnStatus != eSyncStatusSuccess )
    {
        log_err( "%s: data write error: %d\r\n", __func__, __LINE__ );
        return returnStatus;
    }

    vUDelay( MS_TO_US( 1000 ) ); /* 1s delay */

    if( ( eResetType == SYNC_TIMING_DEVICE_RESET_SOFT ) ||
        ( eResetType == SYNC_TIMING_DEVICE_RESET_TOGGLE ) )
    {
        while( ucMCUBootState < 0xB8 )
        {
            returnStatus = xSyncTimingDeviceMemReadDirect( pxContext,
                                                           SYNC_TIMING_MCU_BOOTSTATE,
                                                           &ucMCUBootState,
                                                           1 );

            if( returnStatus != eSyncStatusSuccess )
            {
                log_err( "%s: SYNC_TIMING_MCU_BOOTSTATE read error\r\n", __func__ );
                return returnStatus;
            }

            if( ucMCUBootState >= 0xB8 )
            {
                break;
            }

            if( ++ulCount > 100 )
            {
                log_err( "%s: SYNC_TIMING_MCU_BOOTSTATE invalid\r\n", __func__ );
                return returnStatus;
            }

            vUDelay( MS_TO_US( 50 ) );
        }
    }

    returnStatus = xSyncTimingDeviceGetChipsetMode( pxContext, &xMode );

    if( returnStatus != eSyncStatusSuccess )
    {
        log_err( "%s: sync timing device GetChipsetMode failed\r\n", __func__ );
        return returnStatus;
    }

    log_info( "Sync timing device is in %s mode\r\n", SyncTimingDeviceModeNames[ xMode ] );

    return returnStatus;
}

SyncStatus_t xSyncTimingDeviceFwUpgrade( SyncTimingDeviceContext_t* pxContext,
                                        struct la9310_std_fwupgrade_data *pxFwInfo )
{
    SyncStatus_t            returnStatus                                = eSyncStatusFailure;
    cmd_RESTART_map_t       xCmdRestart;
    cmd_HOST_LOAD_map_t     xCmdHostLoad;
    cmd_BOOT_map_t          xCmdBoot;
    cmd_NVM_STATUS_map_t    xCmdNvmStatus;
    reply_NVM_STATUS_map_t  *pxReplyNvmStatus;
    uint8_t                 ucBuff[10];
    SyncTimingDeviceMode_t  xMode;
    uint32_t                lDloadOrder[LA9310_MAX_STD_FW_COUNT]        = { -1, -1, -1, -1 };
    uint32_t                i                                           = 0;
    uint32_t                j                                           = 0;
    uint32_t                k                                           = 0;
    uint32_t                ulIndex                                     = 0;
    uint8_t                 ucA1RestartCmdMap                           = 0;
    uint8_t                 ucPatchFilePattern[FW_PATCH_PATTERN_SIZE]   = { 0x1B, 0x01, 0x1B, 0x1A, 0x1A, 0x1A, 0x1A };
    char                    *pucFw;
    uint8_t                 ucMCUBootState                              = 0;
    uint32_t                ulCount                                     = 0;

    xCmdRestart.CMD = cmd_ID_RESTART;
    xCmdRestart.OPTIONS = 0;
    xCmdRestart.OPTIONS |= CMD_RESTART_ARG_OPTIONS_WAIT_TRUE_BIT;
    xCmdHostLoad.CMD = cmd_ID_HOST_LOAD;
    xCmdBoot.CMD = cmd_ID_BOOT;
    xCmdNvmStatus.CMD = cmd_ID_NVM_STATUS;

    returnStatus = xSyncTimingDeviceGetChipsetMode( pxContext, &xMode );
    if( returnStatus != eSyncStatusSuccess )
    {
        log_err( "sync timing device GetChipsetMode failed\r\n" );
        return returnStatus;
    }

    if( ( pxContext->deviceRevision == 0x0 ) &&
            ( pxContext->deviceSubRevision == 0x1 ) )
    {
        ucA1RestartCmdMap = 1;
    }

    for( i = 0; i < pxFwInfo->fwcount; i++ )
    {
        lDloadOrder[i] = i;
        pucFw = ( char * ) ( pxFwInfo->fwinfo[i].addr );

        for( j = 0; j < FW_PATCH_PATTERN_SCAN_LEN; j++ )
        {
            if( memcmp( &ucPatchFilePattern[0], &pucFw[j], FW_PATCH_PATTERN_SIZE ) == 0 )
            {
                // Found patch file in the list of files provided
                // set lDloadOrder[0] to the current index
                if( i != k )
                {
                    lDloadOrder[i] = lDloadOrder[k];
                    lDloadOrder[k] = i;
                    k++;
                }
                else
                {
                    lDloadOrder[k] = i;
                    k++;
                }

                break;
            }
        }
    }

    log_info( "Original order\r\n" );
    for( i = 0; i < pxFwInfo->fwcount; i++ )
    {
        log_info( "addr: 0x%x, size: %d\r\n", pxFwInfo->fwinfo[i].addr,
                pxFwInfo->fwinfo[i].size );
    }
    log_info( "Download order\r\n" );
    for( i = 0; i < pxFwInfo->fwcount; i++ )
    {
        log_info( "addr: 0x%x, size: %d\r\n", pxFwInfo->fwinfo[lDloadOrder[i]].addr,
                pxFwInfo->fwinfo[lDloadOrder[i]].size );
    }
    /* If in application mode, put part in bootloader mode (modified RESTART_A1 command) */
    /* If in some bootloader mode (0x16), send RESTART command */
    if( ( xMode == eSyncTimingDeviceModeAppln ) && ( ucA1RestartCmdMap == 1 ) )
    {
        log_info( "Modified restart command\r\n" );
        xCmdRestart.CMD = 0xDF;
    }

    returnStatus = prvSyncTimingDeviceWaitForCTS( pxContext );
    if( returnStatus != eSyncStatusSuccess )
    {
        log_err( "%s: wait for CTS failed: %d\r\n", __func__, __LINE__ );
        return returnStatus;
    }

    returnStatus = prvSyncTimingDeviceTrxCommand( pxContext,
            ( uint8_t * ) &xCmdRestart,
            sizeof( cmd_RESTART_map_t ),
            eSyncWriteTransaction );
    if( returnStatus != eSyncStatusSuccess )
    {
        log_err( "%s: data write error: %d\r\n", __func__, __LINE__ );
        return returnStatus;
    }

    vUDelay( MS_TO_US( 100 ) );

    while( ucMCUBootState != BOOTLOADER_READY_STATE )
    {
        returnStatus = xSyncTimingDeviceMemReadDirect( pxContext,
                SYNC_TIMING_MCU_BOOTSTATE,
                &ucMCUBootState,
                1 );
        if( returnStatus != eSyncStatusSuccess )
        {
            log_err( "%s: SYNC_TIMING_MCU_BOOTSTATE read error %d\r\n", __func__, __LINE__ );
            return returnStatus;
        }

        log_info( "Bootloader state: 0x%x\r\n", ucMCUBootState );
        if( ucMCUBootState == BOOTLOADER_READY_STATE )
        {
            break;
        }

        vUDelay( MS_TO_US( 50 ) );

        if( ++ulCount >= 100 )
        {
            log_err( "%s: SYNC_TIMING_MCU_BOOTSTATE not ready 0x%x %d\r\n", __func__,
                    ucMCUBootState, __LINE__ );
            return eSyncStatusTimeout;
        }
    }

    for( i = 0; i < pxFwInfo->fwcount; i++ )
    {
        ulCount = 0;
        ulIndex = 0;

        while( ulIndex < pxFwInfo->fwinfo[lDloadOrder[i]].size )
        {
            ulCount = SYNC_TIMING_MAX_CMD_DATA_TRANSFER_SIZE;

            if( ( ulIndex + ulCount ) > pxFwInfo->fwinfo[lDloadOrder[i]].size )
            {
                ulCount = pxFwInfo->fwinfo[lDloadOrder[i]].size - ulIndex;
            }

            returnStatus = prvSyncTimingDeviceWaitForCTS( pxContext );
            if( returnStatus != eSyncStatusSuccess )
            {
                log_err( "wait for cts failed while sending host load command\r\n" );
                return returnStatus;
            }

            log_info( "addr: 0x%x chunk size: %d\r\n",
                    pxFwInfo->fwinfo[lDloadOrder[i]].addr + ulIndex, ulCount );

            memset( ( void * ) ( &(xCmdHostLoad.DATA[0]) ), 0, SYNC_TIMING_MAX_CMD_DATA_TRANSFER_SIZE );
            memcpy( ( void * ) ( &(xCmdHostLoad.DATA[0]) ),
                    ( void * ) ( pxFwInfo->fwinfo[lDloadOrder[i]].addr + ulIndex ), ulCount );

            returnStatus = prvSyncTimingDeviceTrxCommand( pxContext,
                    ( uint8_t * ) &xCmdHostLoad,
                    ulCount + 1,
                    eSyncWriteTransaction );
            if( returnStatus != eSyncStatusSuccess )
            {
                log_err( "%s: data write error: %d\r\n", __func__, __LINE__ );
                log_err( "%s: addr: 0x%x, %d, %d\r\n", pxFwInfo->fwinfo[lDloadOrder[i]].addr,
                        ulCount, ulIndex );
                return returnStatus;
            }

            ulIndex = ulIndex + ulCount;
        }
    }

    returnStatus = prvSyncTimingDeviceWaitForCTS( pxContext );
    if( returnStatus != eSyncStatusSuccess )
    {
        log_err( "wait for cts failed before sending nvm status command\r\n" );
        return returnStatus;
    }

    returnStatus = prvSyncTimingDeviceTrxCommand( pxContext,
            ( uint8_t * ) &xCmdNvmStatus,
            sizeof( cmd_NVM_STATUS_map_t ),
            eSyncWriteTransaction );
    if( returnStatus != eSyncStatusSuccess )
    {
        log_err( "%s: data write error: nvm status cmd %d\r\n", __func__, __LINE__ );
        return returnStatus;
    }

    ulCount = 0;
    while( 1 )
    {
        returnStatus = prvSyncTimingDeviceTrxCommand( pxContext,
                &ucBuff[0],
                sizeof( reply_NVM_STATUS_map_t ) + 1,
                eSyncReadTransaction );
        if( returnStatus != eSyncStatusSuccess )
        {
            log_err( "%s: data read error\r\n", __func__ );
            return returnStatus;
        }

        if( ucBuff[0] & SYNC_TIMING_DEVICE_CMD_REPLY_READY )
        {
            break;
        }

        if( ++ulCount == SYNC_TIMING_DEVICE_RETRY_COUNT )
        {
            log_err( "%s: waited too long\r\n", __func__ );
            return eSyncStatusTimeout;
        }
    }

    pxReplyNvmStatus = ( reply_NVM_STATUS_map_t * ) &ucBuff[1];
    log_info( "NVM STATUS: ERROR2CNT = %d, ERROR1CNT = %d, MISC = %d, DESCRIPTORS = %d, INVALIDATED = %d\n",
            pxReplyNvmStatus->ERROR2CNT,
            pxReplyNvmStatus->ERROR1CNT,
            pxReplyNvmStatus->MISC,
            pxReplyNvmStatus->DESCRIPTORS,
            pxReplyNvmStatus->INVALIDATED );

    returnStatus = xSyncTimingDeviceMemReadDirect( pxContext,
            SYNC_TIMING_MCU_BOOTSTATE,
            &ucMCUBootState,
            1 );
    if( returnStatus != eSyncStatusSuccess )
    {
        log_err( "%s: SYNC_TIMING_MCU_BOOTSTATE read error %d\r\n", __func__, __LINE__ );
        return returnStatus;
    }

    log_info( "Downloaded images to RAM; Now Sending BOOT Command - uMcuBootstate = 0x%x\r\n",
            ucMCUBootState );

    returnStatus = prvSyncTimingDeviceWaitForCTS( pxContext );
    if( returnStatus != eSyncStatusSuccess )
    {
        log_err( "wait for cts failed before sending boot command\r\n" );
        return returnStatus;
    }

    returnStatus = prvSyncTimingDeviceTrxCommand( pxContext,
            ( uint8_t * ) &xCmdBoot,
            sizeof( cmd_BOOT_map_t ),
            eSyncWriteTransaction );
    if( returnStatus != eSyncStatusSuccess )
    {
        log_err( "%s: data write error: boot cmd %d\r\n", __func__, __LINE__ );
        return returnStatus;
    }

    ucMCUBootState = 0;
    ulCount = 0;

    while( ucMCUBootState < CHIP_OUTPUT_READY_STATE )
    {
        returnStatus = xSyncTimingDeviceMemReadDirect( pxContext,
                SYNC_TIMING_MCU_BOOTSTATE,
                &ucMCUBootState,
                1 );
        if( returnStatus != eSyncStatusSuccess )
        {
            log_err( "%s: SYNC_TIMING_MCU_BOOTSTATE read error %d\r\n", __func__, __LINE__ );
            return returnStatus;
        }

        if( ucMCUBootState >= CHIP_OUTPUT_READY_STATE )
        {
            break;
        }

        vUDelay( MS_TO_US( 50 ) );

        if( ++ulCount >= 100 )
        {
            log_err( "%s: SYNC_TIMING_MCU_BOOTSTATE not chip ready %d\r\n", __func__, __LINE__ );
            return eSyncStatusTimeout;
        }
    }

    log_info( "Device Booted - uMcuBootstate = 0x%x\r\n", ucMCUBootState );

    xSyncTimingDeviceGetChipsetMode( pxContext, &xMode );
    log_info( "Sync timing device mode: %d\r\n", xMode );

    if( xMode == eSyncTimingDeviceModeAppln )
    {
        xSyncTimingDeviceFWAPIMetadata( pxContext );
    }

    xSyncTimingDeviceGetVersionInfo( pxContext );

    return returnStatus;
}
