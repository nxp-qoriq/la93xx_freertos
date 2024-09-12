/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright 2017, 2021, 2024 NXP
 */

#ifndef __DCS_H__
#define __DCS_H__

/*
 * NLM: Only initializing the required ADC,
 * others have been kept disabled.
 */

#define ADC_DAC_SYNC_RETRY            1000
#define ADC_RST_SYNC_RETRY            1000
#define ADC_POWERON_SYNC_RETRY        1000
#define ADC_RDY_RETRY                 10000
#define DAC_RDY_RETRY                 1000

/* ADC register */
#define HSADC_ENCTL                   ( DCS_BASE_ADDR + 0xA0 )
#define HSDAC_ENCTL                   ( DCS_BASE_ADDR + 0x250 )
#define HSADC_RSTCTL                  ( DCS_BASE_ADDR + 0x20 )
#define HS_RX_ADC0_CFGCTL             ( DCS_BASE_ADDR + 0x30 )
#define HS_RX_ADC1_CFGCTL             ( DCS_BASE_ADDR + 0x34 )
#define HS_RXO_ADC0_CFGCTL            ( DCS_BASE_ADDR + 0x38 )
#define HS_RXO_ADC1_CFGCTL            ( DCS_BASE_ADDR + 0x3C )
#define HSADC_STAT                    ( DCS_BASE_ADDR + 0x400 )

/* DAC register */
#define HSDAC_RSTCTL                  ( DCS_BASE_ADDR + 0x200 )
#define HSDAC_CFGCTL1                 ( DCS_BASE_ADDR + 0x210 )
#define HSDAC_STAT                    ( DCS_BASE_ADDR + 0x440 )
#define HSDAC_ERRSTAT1                ( DCS_BASE_ADDR + 0x448 )
#define HSDAC_ERRSTAT2                ( DCS_BASE_ADDR + 0x44C )

/* ADC/DAC common register */
#define ADC_DAC_CLKCFG                ( DCS_BASE_ADDR + 0x300 )
#define ADC_DAC_CLKCTRL               ( DCS_BASE_ADDR + 0x310 )
#define REG_SPACE_SYNC                ( DCS_BASE_ADDR + 0xF34 )
#define GATE_READY                    ( DCS_BASE_ADDR + 0x320 )

#define RX1_Full_Freq_MASK    ~( 0x1 << 0 )
#define RX1_Half_Freq_SET            ( 0x1 << 0 )
#define RX2_Full_Freq_MASK    ~( 0x1 << 1 )
#define RX2_Half_Freq_SET            ( 0x1 << 1 )
#define RO1_Full_Freq_MASK    ~( 0x1 << 8 )
#define RO1_Half_Freq_SET            ( 0x1 << 8 )
#define RO2_Full_Freq_MASK    ~( 0x1 << 9 )
#define RO2_Half_Freq_SET            ( 0x1 << 9 )
#define TX_Full_Freq_MASK     ~( 0x1 << 16 )
#define TX_Half_Freq_SET             ( 0x1 << 16 )

#define TRGR_CLKDIV_SYNC              ( 0x1 << 1 )
#define TRGR_GATE_RDY_SYNC            ( 0x1 << 2 )
#define TRGR_ADC_SYNC                 ( 0x1 << 3 )
#define TRGR_DAC_SYNC                 ( 0x1 << 4 )

#define TRGR_ADC_RSTCTL_MASK          0xFFFF
#define TRGR_DAC_RSTCTL_MASK          0x1
#define TX_DAC_RST                    ( 0x1 << 0 )
#define RX1_ADC_RST                   ( 0x1 << 0 )
#define RX2_ADC_RST                   ( 0x1 << 1 )
#define RO1_ADC_RST                   ( 0x1 << 2 )
#define RO2_ADC_RST                   ( 0x1 << 3 )

#define TX_DAC_POWERCTL               ( 0x1 << 0 )
#define RX1_ADC_POWERCTL              ( 0x1 << 0 )
#define RX2_ADC_POWERCTL              ( 0x1 << 1 )
#define RO1_ADC_POWERCTL              ( 0x1 << 2 )
#define RO2_ADC_POWERCTL              ( 0x1 << 3 )

/* ADC Configuration Control Register */
#define ADC_OPM_MASK                  0x3
#define ADC_OPM_POWERDN               0x0
#define ADC_OPM_POWERUP               0x3
#define ADC_OPM_POWERSLEEP            0x1
#define ADC_SELOF_CTRL                ( 0x1 << 4 )
#define ADC_VCMOUT_CTRL               ( 0x1 << 8 )
#define ADC_CPLING_CTRL               ( 0x1 << 12 )
#define ADC_I2C_ADDR_SHIFT            16
#define ADC_I2C_ADDR_MASK             0x7F0000

/* DAC Configuration Control Register1 */
#define DAC_OPM_MASK                  0x7
#define DAC_OPM_POWERDN               0x0
#define DAC_OPM_POWERUP               0x3
#define DAC_OPM_STNDBY                0x7
#define DAC_SELIF_CTRL                ( 0x1 << 24 )

#define HSDAC_STAT_RDY                ( 0x1 << 0 )
#define RX1_ADC_STAT_RDY              ( 0x1 << 0 )
#define RX2_ADC_STAT_RDY              ( 0x1 << 1 )
#define RO1_ADC_STAT_RDY              ( 0x1 << 2 )
#define RO2_ADC_STAT_RDY              ( 0x1 << 3 )

#define RX_ADC_GATE_RDY               0xf
#define RX1_ADC_GATE_RDY              ( 0x1 << 0 )
#define RX2_ADC_GATE_RDY              ( 0x1 << 1 )
#define RO1_ADC_GATE_RDY              ( 0x1 << 2 )
#define RO2_ADC_GATE_RDY              ( 0x1 << 3 )

#define TX_DAC_GATE_RDY               ( 0x1 << 16 )

#define RX1_ADC_CLK_CTRL              ( 0x1 << 0 )
#define RX2_ADC_CLK_CTRL              ( 0x1 << 1 )
#define RO1_ADC_CLK_CTRL              ( 0x1 << 2 )
#define RO2_ADC_CLK_CTRL              ( 0x1 << 3 )
#define TX_DAC_CLK_CTRL               ( 0x1 << 16 )

typedef enum AdcDacOPMConfig
{
    OPM_DISABLE,
    OPM_ENABLE,
    OPM_SLEEP,
} AdcDacOPMConfig_t;
#endif /*__DCS_H__*/
