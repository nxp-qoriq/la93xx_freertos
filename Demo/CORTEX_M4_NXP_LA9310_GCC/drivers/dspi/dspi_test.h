/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2021-2022 NXP
 */

#ifndef __NLM_DSPI_INIT_H
#define __NLM_DSPI_INIT_H

/*
 * Phy timer comparator 15 is used to drive PA_EN(GPIO_12) which is used
 * for selecting chip select between 2 DSPI devices LMX2582 and ADRF6520.
 */
#define PHY_TIMER_COMP_PA_EN    ( 15 )

#define SPI_CS0_SEL_GPIO           12

/*  Macro Definition */
#define LMX2582_SYNTH_READ_OPR     ( 1 << 7 )
#define LMX2582_SYNTH_ADDR_MASK    0x00FF0000
#define LMX2582_SYNTH_DATA_MASK    0x0000FFFF

#define LMX2582_SYNTH_REG0         0x0
#define LMX2582_SYNTH_REG1         0x1
#define LMX2582_SYNTH_POWERDOWN    ( 1 << 0 )
#define LMX2582_SYNTH_RESET        ( 1 << 1 )
#define LMX2582_SYNTH_FCAL         ( 1 << 3 )

#define NLM_V2_ENABLE              1
#define MAX2870_SYNTH_REG0         0x0
#define MAX2870_SYNTH_REG1         0x1
#define MAX2870_SYNTH_REG2         0x2
#define MAX2870_SYNTH_REG3         0x3
#define MAX2870_SYNTH_REG4         0x4
#define MAX2870_SYNTH_REG5         0x5
#define MAX2870_SYNTH_REG6         0x6

#define MAX2870_SET_MUX_OUT_HIGH_Z  0
#define MAX2870_SET_MUX_OUT_VDD     1
#define MAX2870_SET_MUX_OUT_GROUND  2
#define MAX2870_SET_MUX_OUT_READ    3


/* Function Declaration */

int32_t LMX2582SynthTest( struct LA931xDspiInstance * pDspiHandle );
int32_t prvLMX2582SynthReadReg( struct LA931xDspiInstance * pDspiHandle,
                                uint8_t addr,
                                uint16_t * data );
int32_t prvLMX2582SynthWriteReg( struct LA931xDspiInstance * pDspiHandle,
                                 uint8_t addr,
                                 uint16_t data );

/*  Macro Definition */
#define LTC5586_DEMOD_READ_OPR      ( 1 << 7 )

#define LTC5586_DEMOD_REG22         ( 0x16 )
#define LTC5586_DEMOD_REG16         ( 0x10 )
#define LTC5586_DEMOD_RESET         ( 0xF8 )

#define LTC5586_DEMOD_REG16         ( 0x10 )
#define LTC5586_DEMOD_ATTN_MASK     ( ~( 0xF8 ) )
#define LTC5586_DEMOD_ATTN_SHIFT    ( 3 )
#define LTC5586_DEMOD_ATTN_MIN      ( 0 )
#define LTC5586_DEMOD_ATTN_MAX      ( 31 )

#define LTC5586_DEMOD_REG21         ( 0x15 )
#define LTC5586_DEMOD_GAIN_MASK     ( ~( 0x70 ) )
#define LTC5586_DEMOD_GAIN_SHIFT    ( 4 )
#define LTC5586_DEMOD_GAIN_MIN      ( 0 )
#define LTC5586_DEMOD_GAIN_MAX      ( 7 )

/* LO matching register - For 3.5G */
#define LTC5586_DEMOD_REG18         ( 0x12 )
#define LTC5586_DEMOD_REG19         ( 0x13 )
#define LTC5586_DEMOD_LO_MATCH1     ( 0x40 )
#define LTC5586_DEMOD_LO_MATCH2     ( 0x80 )
int32_t LTC5586DemodTest( struct LA931xDspiInstance * pDspiHandle );
int32_t prvLTC5586DemodWriteReg( struct LA931xDspiInstance * pDspiHandle,
                                 uint8_t addr,
                                 uint8_t data );
int32_t prvLTC5586DemodReadReg( struct LA931xDspiInstance * pDspiHandle,
                                uint8_t addr,
                                uint8_t * data );

/*  Macro Definition */
#define ADRF6520_READ_OPR    ( 1 << 15 )
#define ADRF6520_REG         ( 0x0010 )

int32_t ADRF6520Test( struct LA931xDspiInstance * pDspiHandle );
int32_t prvADRF6520WriteReg( struct LA931xDspiInstance * pDspiHandle,
                             uint16_t addr,
                             uint8_t data );
int32_t prvADRF6520ReadReg( struct LA931xDspiInstance * pDspiHandle,
                            uint16_t addr,
                            uint16_t * data );

#endif //__NLM_DSPI_INIT_H
