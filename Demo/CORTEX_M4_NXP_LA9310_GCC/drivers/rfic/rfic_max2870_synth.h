/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright 2022 NXP
 * All rights reserved.
 */

# include "rfic_synth.h"

#define MAX2870_SYNTH_REG0         0x0
#define MAX2870_SYNTH_REG1         0x1
#define MAX2870_SYNTH_REG2         0x2
#define MAX2870_SYNTH_REG3         0x3
#define MAX2870_SYNTH_REG4         0x4
#define MAX2870_SYNTH_REG5         0x5
#define MAX2870_SYNTH_REG6         0x6
/*
 * Function Declaration
 */
int32_t Rficmax2870Init( RficDevice_t *pRficDev );
int32_t Rficmax2870WriteReg( RficDevice_t *pRficDev, uint8_t addr, uint32_t data );
int32_t Rficmax2870ReadReg( RficDevice_t *pRficDev, uint8_t addr, uint32_t *data );
void Rficmax2870AdjustPllFreq( RficDevice_t *pRficDev, int32_t freq_khz );

