/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright 2022 NXP
 * All rights reserved.
 */
#ifndef __RFIC_SYNTH_H
#define __RFIC_SYNTH_H

# include "FreeRTOS.h"
# include "rfic_core.h"
# include "fsl_dspi.h"
/*
 * Macro Definition
 */
#define RFIC_SYNTH_DEFAULT_FREQ  ( 751000 )
#define RFIC_SYNTH_OSC_FREQ                 ( 122880 )

/*
 * Function Declaration
 */
int32_t RficSynthInit( RficDevice_t *pRficDev );
int32_t RficSynthReadReg( RficDevice_t *pRficDev, uint8_t addr,
			  uint32_t *data );
int32_t RficSynthWriteReg( RficDevice_t *pRficDev, uint8_t addr,
			  uint32_t data );
void RficSynthAdjustPllFreq( RficDevice_t *pRficDev, int32_t freq_khz );
void RficSynthAdjustPllFastCal(RficDevice_t *pRficDev);

#endif //__RFIC_SYNTH_H
