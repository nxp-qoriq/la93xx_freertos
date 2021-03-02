/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2021 NXP
 */

#ifndef __SYNC_TIMING_DEVICE_CLI_H
#define __SYNC_TIMING_DEVICE_CLI_H

typedef enum
{
    eSyncTimingDeviceCommandBoot = 1,
    eSyncTimingDeviceCommandGetMode = 2,
    eSyncTimingDeviceCommandGetVersion = 3,
    eSyncTimingDeviceCommandRegRead = 4,
    eSyncTimingDeviceCommandRegWrite = 5,
    eSyncTimingDeviceCommandPrintMetadata = 6,
    eSyncTimingDeviceCommandSetDCO = 7,
    eSyncTimingDeviceCommandSetReset = 8,
    eSyncTimingDeviceCommandSetResetBL = 9,
    eSyncTimingDeviceCommandEndMarker
} SyncTimingDeviceCommand_t;

void vRegisterTimesyncCLICommands( void );

#endif /* __SYNC_TIMING_DEVICE_CLI_H */
