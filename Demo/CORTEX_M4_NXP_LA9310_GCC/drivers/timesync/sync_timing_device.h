/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2021 NXP
 */

#ifndef __SYNC_TIMING_DEVICE_H
#define __SYNC_TIMING_DEVICE_H

#include <immap.h>
#include <sync_timing_common.h>
#include "la9310_host_if.h"

#define SYNC_TIMING_DEVICE_I2C_HANDLE             ( LA9310_FSL_I2C1 )
#define SYNC_TIMING_DEVICE_I2C_ADDR               ( 0x59 )
#define SYNC_TIMING_DEVICE_MCU_PORTAL_ADDR        ( 0xF00F )
#define SYNC_TIMING_DEVICE_MCU_PORTAL_ADDR_LEN    ( 0x2 )
#define SYNC_TIMING_DEVICE_RETRY_COUNT            ( 10 )
#define SYNC_TIMING_DEVICE_CMD_REPLY_READY        ( 0x80 )
#define SYNC_TIMING_DEVICE_CLEAR_TO_SEND          ( 0x80 )
#define SYNC_TIMING_MAX_CMD_DATA_TRANSFER_SIZE    ( 255 )

typedef enum
{
    eSyncStatusSuccess = 0,
    eSyncStatusFailure,
    eSyncStatusTimeout,
    eSyncStatusMax
} SyncStatus_t;

typedef enum
{
    eSyncWriteTransaction = 0,
    eSyncReadTransaction
} SyncTransactionType_t;

typedef enum
{
    eSyncTimingDeviceModeAppln = 0,
    eSyncTimingDeviceModeBootloader = 1,
    eSyncTimingDeviceModeInvalid = 2,
} SyncTimingDeviceMode_t;

typedef enum
{
    eSyncTimingDeviceStatusUnInitialized = 0,
    eSyncTimingDeviceStatusInitialized = 1
} SyncTimingDeviceStatus_t;

typedef struct
{
    uint32_t i2cHandle;
    uint32_t portalAddr;
    uint8_t i2cAddr;
    uint8_t portalAddrLen;
} SyncTimingDeviceI2CInfo_t;

typedef struct
{
    uint32_t fwVersionMajor;
    uint32_t fwVersionMinor;
    uint32_t fwVersionBuildNum;
    uint8_t deviceRevision;
    uint8_t deviceSubRevision;
    uint8_t deviceStatus;
    uint8_t reserved;
    SyncTimingDeviceMode_t deviceMode;
    SyncTimingDeviceI2CInfo_t i2cInfo;
} SyncTimingDeviceContext_t;

/**
 * @brief	Function is used to get the sync timing device context handle.
 * @return  context handle on success, NULL otherwise.
 */
SyncTimingDeviceContext_t * pxSyncTimingDeviceGetContext( void );

/**
 * @brief	Function is used to initialize the sync driver and get the context handle.
 * @return  context handle on success, NULL otherwise.
 */
SyncTimingDeviceContext_t * pxSyncTimingDeviceInit( void );

/**
 * @brief	Function is used to get the sync timing device version info.
 * @param[in]   pxContext Sync timing device context handle
 * @return      eSyncStatusSuccess on success, eSyncStatusFailure otherwise.
 */
SyncStatus_t xSyncTimingDeviceGetVersionInfo( SyncTimingDeviceContext_t * pxContext );

/**
 * @brief	Function is used to boot si5510.
 * @param[in]   pxContext Sync timing device context handle
 * @return      eSyncStatusSuccess on success, eSyncStatusFailure otherwise.
 */
SyncStatus_t xSyncTimingDeviceFWAPIBoot( SyncTimingDeviceContext_t * pxContext );

/**
 * @brief	Function is used to get the FW metadata.
 * @param[in]   pxContext Sync timing device context handle
 * @return      eSyncStatusSuccess on success, eSyncStatusFailure otherwise.
 */
SyncStatus_t xSyncTimingDeviceFWAPIMetadata( SyncTimingDeviceContext_t * pxContext );

/**
 * @brief	Function is used to set variable offset dco.
 * @param[in]   pxContext Sync timing device context handle
 * @param[in]   ucDivider Divider which needs to be 1,2,4,8 or 16
 * @param[in]   lSteps steps
 * @return      eSyncStatusSuccess on success, eSyncStatusFailure otherwise.
 */
SyncStatus_t xSyncTimingDeviceFWAPIVariableOffsetDco( SyncTimingDeviceContext_t * pxContext,
                                                      uint8_t ucDivider,
                                                      int32_t lSteps );

/**
 * @brief	Function is used to write register.
 * @param[in]   pxContext Sync timing device context handle
 * @param[in]   usAddr register address
 * @param[in]   pucBuff data to write
 * @param[in]   ulLen length of the data
 * @return      eSyncStatusSuccess on success, eSyncStatusFailure otherwise.
 */
SyncStatus_t xSyncTimingDeviceMemWriteDirect( SyncTimingDeviceContext_t * pxContext,
                                              uint16_t usAddr,
                                              uint8_t * pucBuff,
                                              uint32_t ulLen );

/**
 * @brief	Function is used to read register.
 * @param[in]   pxContext Sync timing device context handle
 * @param[in]   usAddr register address
 * @param[in]   pucBuff buffer to hold read data
 * @param[in]   ulLen length of the data
 * @return      eSyncStatusSuccess on success, eSyncStatusFailure otherwise.
 */
SyncStatus_t xSyncTimingDeviceMemReadDirect( SyncTimingDeviceContext_t * pxContext,
                                             uint16_t usAddr,
                                             uint8_t * pucBuff,
                                             uint32_t ulLen );

/**
 * @brief	Function is used to get chip mode.
 * @param[in]   pxContext Sync timing device context handle
 * @param[in]   pxMode buffer to hold chip mode
 * @return      eSyncStatusSuccess on success, eSyncStatusFailure otherwise.
 */
SyncStatus_t xSyncTimingDeviceGetChipsetMode( SyncTimingDeviceContext_t * pxContext,
                                              SyncTimingDeviceMode_t * pxMode );

/**
 * @brief	Function is used to get chip mode.
 * @param[in]   pxContext Sync timing device context handle
 * @param[in]   eResetType Reset type e.g. soft, toggle, bootloader etc
 * @return      eSyncStatusSuccess on success, eSyncStatusFailure otherwise.
 */
SyncStatus_t xSyncTimingDeviceReset( SyncTimingDeviceContext_t * pxContext,
                                     SYNC_TIMING_DEVICE_RESET_TYPE_E eResetType );

/**
 * @brief	Function is used to update fw.
 * @param[in]   pxContext Slab timing device context handle
 * @param[in]   pxFwInfo Fw info addresses and size
 * @return      eSlabStatusSuccess on success, eSlabStatusFailure otherwise.
 */
SyncStatus_t xSyncTimingDeviceFwUpgrade( SyncTimingDeviceContext_t* pxContext,
                                        struct la9310_std_fwupgrade_data *pxFwInfo );
#endif /* __SYNC_TIMING_DEVICE_H */
