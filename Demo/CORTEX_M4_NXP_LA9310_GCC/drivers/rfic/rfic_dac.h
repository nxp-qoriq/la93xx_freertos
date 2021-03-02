/* SPDX-License-Identifier: BSD-3-Clause */
/* Copyright 2021 NXP
 */

#ifndef __RFIC_DAC_H
#define __RFIC_DAC_H

#include "FreeRTOS.h"

/* Macro definitions */
#define DAC1_I2C_ADDR           (0x62)
#define DAC2_I2C_ADDR           (0x61)

/* supported commands */
#define DAC_CMD_WRITE           ( 0x0 )
#define DAC_CMD_UPDATE          ( 0x1 )
#define DAC_CMD_WRITEUPDATE     ( 0x3 )
#define DAC_CMD_POWERDOWN       ( 0x4 )

/* Function declaration */
int32_t prvDacWriteUpdate(uint16_t usData, uint8_t usAddr);
#endif //RFIC_DAC_H
