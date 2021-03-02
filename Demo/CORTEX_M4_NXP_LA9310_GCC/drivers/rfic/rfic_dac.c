/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright 2021 NXP
 */

#include "rfic_dac.h"
#include "immap.h"
#include "la9310_i2cAPI.h"

int32_t prvDacWriteUpdate( uint16_t usData, uint8_t usAddr )
{
    uint8_t dac_cmd;
    uint8_t ucData[ 2 ];

    /* Writing cmd data*/
    dac_cmd =  ( DAC_CMD_WRITEUPDATE << 4 );

    /* Ignoring the 4 bits of LSB*/
    usData = usData << 4;

    /* Writing data */
    ucData[ 0 ] = ( uint8_t )(( usData & 0xFF00 ) >> 8 );
    ucData[ 1 ] = ( uint8_t )( usData & 0x00FF);
    log_dbg( "Write Byte Num 0 val 0x%x , Byte Num 1 val 0x%x \n\r", ucData[ 0 ], ucData[ 1 ] );
    iLa9310_I2C_Write( LA9310_FSL_I2C1, usAddr, dac_cmd, 1, ucData, 2 );

    return 0;
}
