/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright 2021 NXP
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
#define RFIC_SYNTH_READ_OPR      ( 1 << 7 )
#define RFIC_SYNTH_ADDR_MASK     ( 0x00FF0000 )
#define RFIC_SYNTH_DATA_MASK     ( 0x0000FFFF )

/* Register R0 bit description */
#define RFIC_SYNTH_REG0          ( 0x0 )
#define RFIC_SYNTH_POWERDOWN     ( 1 << 0 )
#define RFIC_SYNTH_RESET         ( 1 << 1 )
#define RFIC_SYNTH_FCAL          ( 1 << 3 )

/* Registers that needs to be programmed for freq change */
#define RFIC_SYNTH_CH_DIV_SEG_REG           ( 0x23 )
#define RFIC_SYNTH_BUF_CH_DIV_REG           ( 0x24 )
#define RFIC_SYNTH_INT_N_DIV_REG            ( 0x26 )
#define RFIC_SYNTH_NUM_MSB_N_DIV_FRAC_REG   ( 0x2C )
#define RFIC_SYNTH_NUM_LSB_N_DIV_FRAC_REG   ( 0x2D )
#define RFIC_SYNTH_MUX_OUT_A_REG            ( 0x2F )
#define RFIC_SYNTH_BUF_VCO_OUT_REG          ( 0x1F )

#define RFIC_SYNTH_MASK_INT_N_DIV	    ( 0xE001 )
#define RFIC_SYNTH_CHDIV_SEG1               ( 2 )
#define RFIC_SYNTH_CHDIV_SEG2_EN            ( 7 )
#define RFIC_SYNTH_CHDIV_SEG2               ( 9 )
#define RFIC_SYNTH_OSC_FREQ                 ( 122880 )

/*
 * Function Declaration
 */
int32_t RficSynthInit( RficDevice_t *pRficDev );
int32_t RficSynthReadReg( RficDevice_t *pRficDev, uint8_t addr,
			  uint16_t *data );
int32_t RficSynthWriteReg( RficDevice_t *pRficDev, uint8_t addr,
			   uint16_t data );
void RficSynthAdjustPllFreq( RficDevice_t *pRficDev, int32_t freq_khz );
void RficSynthAdjustPllFastCal(RficDevice_t *pRficDev);

#endif //__RFIC_SYNTH_H
