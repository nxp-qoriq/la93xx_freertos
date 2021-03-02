/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright 2021 NXP
 */
#ifndef __RFIC_DEMOD_H
#define __RFIC_DEMOD_H

# include "FreeRTOS.h"
# include "rfic_core.h"
# include "fsl_dspi.h"
/*
 * Macro Definition
 */
#define RFIC_DEMOD_READ_OPR	    ( 1 << 7 )

#define RFIC_DEMOD_REG22            ( 0x16 )
#define RFIC_DEMOD_RESET            ( 0xF8 )

#define RFIC_DEMOD_REG16            ( 0x10 )
#define RFIC_DEMOD_ATTN_MASK        ( ~( 0xF8 ))
#define RFIC_DEMOD_ATTN_SHIFT       ( 3 )
#define RFIC_DEMOD_ATTN_MIN         ( 0 )
#define RFIC_DEMOD_ATTN_MAX         ( 31 )

#define RFIC_DEMOD_REG21            ( 0x15 )
#define RFIC_DEMOD_GAIN_MASK        ( ~( 0x70 ))
#define RFIC_DEMOD_GAIN_SHIFT       ( 4 )
#define RFIC_DEMOD_GAIN_MIN         ( 0 )
#define RFIC_DEMOD_GAIN_MAX         ( 7 )

/* Lo matching */
#define RFIC_DEMOD_REG18            ( 0x12 )
#define RFIC_DEMOD_REG19            ( 0x13 )

#define RFIC_DEMOD_B13_LO_MATCH1    ( 0x4F )
#define RFIC_DEMOD_B13_LO_MATCH2    ( 0x3F )

#define RFIC_DEMOD_GNSS_LO_MATCH1   ( 0x51 )
#define RFIC_DEMOD_GNSS_LO_MATCH2   ( 0xDA )

#define RFIC_DEMOD_B3_LO_MATCH1     ( 0x4F )
#define RFIC_DEMOD_B3_LO_MATCH2     ( 0xDF ) //Default - 0xBF

#define RFIC_DEMOD_N77L_LO_MATCH1   ( 0x53 ) //Default - 0x41
#define RFIC_DEMOD_N77L_LO_MATCH2   ( 0xE3 ) //Default - 0x93
#define RFIC_DEMOD_N77H_LO_MATCH1   ( 0x53 ) //Default - 0x40
#define RFIC_DEMOD_N77H_LO_MATCH2   ( 0xE0 ) //Default - 0x80
#define RFIC_DEMOD_N77_LO_SHIFT     ( 3500000 )

/*
 * Function Declaration
 */
int32_t RficDemodInit( RficDevice_t *pRficDev );
int32_t RficDemodReadReg( RficDevice_t *pRficDev, uint8_t addr, uint8_t *data );
int32_t RficDemodWriteReg( RficDevice_t *pRficDev, uint8_t addr, uint8_t data );
BaseType_t RficDemodGainCtrl( RficDevice_t *pRficDev, int32_t iAttn,
			      int32_t iGain );
int32_t RficDemodLoMatch( RficDevice_t *pRficDev, rf_band_t band, int32_t freq );
#endif //__RFIC_DEMOD_H
