/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright 2021 NXP
 */
#ifndef __RFIC_VGA_H
#define __RFIC_VGA_H

# include "FreeRTOS.h"
# include "rfic_core.h"
# include <fsl_dspi.h>

extern uint16_t RficVgaGainTable[61][2];

/*Enum Definition
 */
typedef enum RficVgaBw
{
    RFIC_VGA_BW_36MHZ,
    RFIC_VGA_BW_72MHZ,
    RFIC_VGA_BW_144MHZ,
    RFIC_VGA_BW_288MHZ,
    RFIC_VGA_BW_432MHZ,
    RFIC_VGA_BW_576MHZ,
    RFIC_VGA_BW_720MHZ,
    RFIC_VGA_BW_BYPASS,
    RFIC_VGA_BW_MAX
} RficVgaBw_t;

/*
 * Macro Definition
 */
#define RFIC_VGA_REG_ADDR       ( 0x10 )
#define RFIC_VGA_READ_OPR       ( 1 << 7 )
#define RFIC_VGA_DC_OFFSET_EN   ( 1 << 4 )
#define RFIC_VGA_POWER_EN       ( 1 << 7 )

/* Function prototype */
int32_t RficVgaInit( RficDevice_t *pRficDev );
int32_t RficVgaWriteReg( RficDevice_t *pRficDev, uint8_t data );
int32_t RficVgaReadReg( RficDevice_t *pRficDev, uint8_t *data );
#endif //__RFIC_VGA_H

