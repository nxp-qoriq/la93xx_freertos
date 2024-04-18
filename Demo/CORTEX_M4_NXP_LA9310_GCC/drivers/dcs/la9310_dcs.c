/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright 2017-2018, 2021, 2024 NXP
 */

#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "common.h"
#include "immap.h"
#include "debug_console.h"
#include "la9310_irq.h"
#include "config.h"
#include "la9310_dcs.h"
#include <la9310_dcs_api.h>

#define CHECK_BIT(var,pos) (((var) & (1<<(pos)))>>(pos))

BaseType_t xLa9310AdcDacPowerUp( LA9310XcvrDCS_t dcs )
{
    uint32_t ulAdcDacRegVal;

        ulAdcDacRegVal = IN_32( ( uint32_t * ) ( HSADC_ENCTL ) );

    if( dcs == XCVR_TRX_TX_DAC )
    {
        ulAdcDacRegVal |= TX_DAC_POWERCTL;
        OUT_32( HSDAC_ENCTL, ulAdcDacRegVal );
        ulAdcDacRegVal = ADC_POWERON_SYNC_RETRY;
        OUT_32( REG_SPACE_SYNC, TRGR_DAC_SYNC |
                IN_32( ( uint32_t * ) REG_SPACE_SYNC ) );

        while( TRGR_DAC_SYNC & IN_32( ( uint32_t * ) REG_SPACE_SYNC ) )
        {
            if( !( ulAdcDacRegVal-- ) )
            {
                log_err( "%s: DAC reset sync failed\n",
                         __func__ );
                return pdFAIL;
            }
        }
    }
    else
    {

        if( dcs == XCVR_TRX_RX1_ADC )
        {
            ulAdcDacRegVal |= RX1_ADC_POWERCTL;
        }
        else if( dcs == XCVR_TRX_RX2_ADC )
        {
            ulAdcDacRegVal |= RX2_ADC_POWERCTL;
        }
        else if( dcs == XCVR_RO1_ADC )
        {
            ulAdcDacRegVal |= RO1_ADC_POWERCTL;
        }
        else if( dcs == XCVR_RO2_ADC )
        {
            ulAdcDacRegVal |= RO2_ADC_POWERCTL;
        }
        else
        {
            return pdFAIL;
        }

        OUT_32( HSADC_ENCTL, ulAdcDacRegVal );
        ulAdcDacRegVal = ADC_POWERON_SYNC_RETRY;
        OUT_32( REG_SPACE_SYNC, TRGR_ADC_SYNC |
                IN_32( ( uint32_t * ) REG_SPACE_SYNC ) );

        while( TRGR_ADC_SYNC & IN_32( ( uint32_t * ) REG_SPACE_SYNC ) )
        {
            if( !( ulAdcDacRegVal-- ) )
            {
                log_err( "%s: ADC reset sync failed\n",
                         __func__ );
                return pdFAIL;
            }
        }
    }

    return pdPASS;
}

BaseType_t xLa9310ConfigAdcDacClock( LA9310XcvrDCS_t dcs,
                                     int freq )
{
    uint32_t ulClkCfgRegVal, ulClkCtrlRegVal;

    ulClkCfgRegVal = IN_32( ( uint32_t * ) ( ADC_DAC_CLKCFG ) );
    ulClkCtrlRegVal = IN_32( ( uint32_t * ) ( ADC_DAC_CLKCTRL ) );

    if( dcs == XCVR_TRX_TX_DAC )
    {
        ulClkCfgRegVal = ( freq == Full_Freq ) ?
                         ( ulClkCfgRegVal & TX_Full_Freq_MASK ) :
                         ( ulClkCfgRegVal | TX_Half_Freq_SET );
        ulClkCtrlRegVal |= TX_DAC_CLK_CTRL;
    }
    else if( dcs == XCVR_TRX_RX1_ADC )
    {
        ulClkCfgRegVal = ( freq == Full_Freq ) ?
                         ( ulClkCfgRegVal & RX1_Full_Freq_MASK ) :
                         ( ulClkCfgRegVal | RX1_Half_Freq_SET );
        ulClkCtrlRegVal |= RX1_ADC_CLK_CTRL;
    }
    else if( dcs == XCVR_TRX_RX2_ADC )
    {
        ulClkCfgRegVal = ( freq == Full_Freq ) ?
                         ( ulClkCfgRegVal & RX2_Full_Freq_MASK ) :
                         ( ulClkCfgRegVal | RX2_Half_Freq_SET );
        ulClkCtrlRegVal |= RX2_ADC_CLK_CTRL;
    }
    else if( dcs == XCVR_RO1_ADC )
    {
        ulClkCfgRegVal = ( freq == Full_Freq ) ?
                         ( ulClkCfgRegVal & RO1_Full_Freq_MASK ) :
                         ( ulClkCfgRegVal | RO1_Half_Freq_SET );
        ulClkCtrlRegVal |= RO1_ADC_CLK_CTRL;
    }
    else if( dcs == XCVR_RO2_ADC )
    {
        ulClkCfgRegVal = ( freq == Full_Freq ) ?
                         ( ulClkCfgRegVal & RO2_Full_Freq_MASK ) :
                         ( ulClkCfgRegVal | RO2_Half_Freq_SET );
        ulClkCtrlRegVal |= RO2_ADC_CLK_CTRL;
    }
    else
    {
        return pdFAIL;
    }

    OUT_32( ADC_DAC_CLKCFG, ulClkCfgRegVal );
    OUT_32( ADC_DAC_CLKCTRL, ulClkCtrlRegVal );

    OUT_32( REG_SPACE_SYNC, TRGR_CLKDIV_SYNC |
            IN_32( ( uint32_t * ) REG_SPACE_SYNC ) );
    ulClkCfgRegVal = ADC_DAC_SYNC_RETRY;

    while( TRGR_CLKDIV_SYNC & IN_32( ( uint32_t * ) REG_SPACE_SYNC ) )
    {
        if( !( ulClkCfgRegVal-- ) )
        {
            log_err( "%s: adc/dac clk sync failed\n", __func__ );
            return pdFAIL;
        }
    }

    return pdPASS;
}

BaseType_t xLa9310ResetAdcDac( LA9310XcvrDCS_t dcs )
{
    uint32_t ulAdcDacRegVal;

    if( dcs == XCVR_TRX_TX_DAC )
    {
        ulAdcDacRegVal = IN_32( ( uint32_t * ) ( HSDAC_RSTCTL ) );
        ulAdcDacRegVal |= TX_DAC_RST;
        OUT_32( HSDAC_RSTCTL, ulAdcDacRegVal );
        ulAdcDacRegVal = ADC_RST_SYNC_RETRY;
        OUT_32( REG_SPACE_SYNC, TRGR_DAC_SYNC |
                IN_32( ( uint32_t * ) REG_SPACE_SYNC ) );

        while( TRGR_DAC_SYNC & IN_32( ( uint32_t * ) REG_SPACE_SYNC ) )
        {
            if( !( ulAdcDacRegVal-- ) )
            {
                log_err( "%s: DAC reset sync failed\n",
                         __func__ );
                return pdFAIL;
            }
        }
    }
    else
    {
        ulAdcDacRegVal = IN_32( ( uint32_t * ) ( HSADC_RSTCTL ) );

        if( dcs == XCVR_TRX_RX1_ADC )
        {
            ulAdcDacRegVal |= RX1_ADC_RST;
        }
        else if( dcs == XCVR_TRX_RX2_ADC )
        {
            ulAdcDacRegVal |= RX2_ADC_RST;
        }
        else if( dcs == XCVR_RO1_ADC )
        {
            ulAdcDacRegVal |= RO1_ADC_RST;
        }
        else if( dcs == XCVR_RO2_ADC )
        {
            ulAdcDacRegVal |= RO2_ADC_RST;
        }
        else
        {
            return pdFAIL;
        }

        OUT_32( HSADC_RSTCTL, ulAdcDacRegVal );
        ulAdcDacRegVal = ADC_RST_SYNC_RETRY;
        OUT_32( REG_SPACE_SYNC, TRGR_ADC_SYNC |
                IN_32( ( uint32_t * ) REG_SPACE_SYNC ) );

        while( TRGR_ADC_SYNC & IN_32( ( uint32_t * ) REG_SPACE_SYNC ) )
        {
            if( !( ulAdcDacRegVal-- ) )
            {
                log_err( "%s: ADC reset sync failed\n",
                         __func__ );
                return pdFAIL;
            }
        }
    }

    return pdPASS;
}

BaseType_t xLa9310AdcDacPowerDown( LA9310XcvrDCS_t dcs )
{
    uint32_t ulAdcDacRegVal;

    if( dcs == XCVR_TRX_TX_DAC )
    {
        ulAdcDacRegVal = IN_32( ( uint32_t * ) ( HSDAC_ENCTL ) );
        ulAdcDacRegVal &= ~TX_DAC_POWERCTL;
        OUT_32( HSDAC_ENCTL, ulAdcDacRegVal );
    }
    else
    {
        ulAdcDacRegVal = IN_32( ( uint32_t * ) ( HSADC_ENCTL ) );

        if( dcs == XCVR_TRX_RX1_ADC )
        {
            ulAdcDacRegVal &= ~RX1_ADC_POWERCTL;
        }
        else if( dcs == XCVR_TRX_RX2_ADC )
        {
            ulAdcDacRegVal &= ~RX2_ADC_POWERCTL;
        }
        else if( dcs == XCVR_RO1_ADC )
        {
            ulAdcDacRegVal &= ~RO1_ADC_POWERCTL;
        }
        else if( dcs == XCVR_RO2_ADC )
        {
            ulAdcDacRegVal &= ~RO2_ADC_POWERCTL;
        }
        else
        {
            return pdFAIL;
        }

        OUT_32( HSADC_ENCTL, ulAdcDacRegVal );
    }

    return pdPASS;
}

BaseType_t xLa9310AdcDacOpModeCtrl( LA9310XcvrDCS_t dcs,
                                    AdcDacOPMConfig_t OPM )
{
    uint32_t ulAdcDacRegVal;

    if( dcs == XCVR_TRX_TX_DAC )
    {
        ulAdcDacRegVal = IN_32( ( uint32_t * ) ( HSDAC_CFGCTL1 ) );

        if( OPM == OPM_DISABLE )
        {
            /* Configure the power-down mode */
            ulAdcDacRegVal = ( ( ulAdcDacRegVal & ~DAC_OPM_MASK ) |
                               DAC_OPM_POWERDN );
            OUT_32( HSDAC_CFGCTL1, ulAdcDacRegVal );
        }
        else if( OPM == OPM_ENABLE )
        {
            /* Configure the power-up mode */
            ulAdcDacRegVal = ( ( ulAdcDacRegVal & ~DAC_OPM_MASK ) |
                               DAC_OPM_POWERUP );
            /* Enable the 2's compliment */
            ulAdcDacRegVal |= DAC_SELIF_CTRL;

            OUT_32( HSDAC_CFGCTL1, ulAdcDacRegVal );
        }
        else
        {
            /* Configure the sleep mode */
            ulAdcDacRegVal = ( ( ulAdcDacRegVal & ~DAC_OPM_MASK ) |
                               DAC_OPM_STNDBY );
            OUT_32( HSDAC_CFGCTL1, ulAdcDacRegVal );
        }

        /* Set the sync bit and poll for its clearing */
        ulAdcDacRegVal = ADC_POWERON_SYNC_RETRY;
        OUT_32( REG_SPACE_SYNC, TRGR_DAC_SYNC |
                IN_32( ( uint32_t * ) REG_SPACE_SYNC ) );

        while( TRGR_DAC_SYNC &
               IN_32( ( uint32_t * ) REG_SPACE_SYNC ) )
        {
            if( !( ulAdcDacRegVal-- ) )
            {
                log_err( "%s: DAC reset sync failed\n",
                         __func__ );
                return pdFAIL;
            }
        }

        /* Poll the DAC ready bit */
        ulAdcDacRegVal = DAC_RDY_RETRY;

        if( OPM == OPM_ENABLE )
        {
            while( 0 == ( HSDAC_STAT_RDY &
                          IN_32( ( uint32_t * ) HSDAC_STAT ) ) )
            {
                if( !( ulAdcDacRegVal-- ) )
                {
                    log_err( "%s: DAC ready bit is not set.\n",
                             __func__ );
                    return pdFAIL;
                }
            }
        }
        else
        {
            while( HSDAC_STAT_RDY & IN_32( ( uint32_t * ) HSDAC_STAT ) )
            {
                if( !( ulAdcDacRegVal-- ) )
                {
                    log_err( "%s: DAC ready bit is not clear.\n",
                             __func__ );
                    return pdFAIL;
                }
            }
        }
    }
    else
    {
        if( OPM == OPM_DISABLE )
        {
            if( dcs == XCVR_TRX_RX1_ADC )
            {
                ulAdcDacRegVal =
                    IN_32( ( uint32_t * ) ( HS_RX_ADC0_CFGCTL ) );
                ulAdcDacRegVal = ( ( ulAdcDacRegVal &
                                     ~ADC_OPM_MASK ) |
                                   ADC_OPM_POWERDN );
                OUT_32( HS_RX_ADC0_CFGCTL, ulAdcDacRegVal );
            }

            else if( dcs == XCVR_TRX_RX2_ADC )
            {
                ulAdcDacRegVal =
                    IN_32( ( uint32_t * ) ( HS_RX_ADC1_CFGCTL ) );
                ulAdcDacRegVal = ( ( ulAdcDacRegVal &
                                     ~ADC_OPM_MASK ) |
                                   ADC_OPM_POWERDN );
                OUT_32( HS_RX_ADC1_CFGCTL, ulAdcDacRegVal );
            }

            else if( dcs == XCVR_RO1_ADC )
            {
                ulAdcDacRegVal =
                    IN_32( ( uint32_t * ) ( HS_RXO_ADC0_CFGCTL ) );
                ulAdcDacRegVal = ( ( ulAdcDacRegVal &
                                     ~ADC_OPM_MASK ) |
                                   ADC_OPM_POWERDN );
                OUT_32( HS_RXO_ADC0_CFGCTL, ulAdcDacRegVal );
            }

            else if( dcs == XCVR_RO2_ADC )
            {
                ulAdcDacRegVal =
                    IN_32( ( uint32_t * ) ( HS_RXO_ADC1_CFGCTL ) );
                ulAdcDacRegVal = ( ( ulAdcDacRegVal &
                                     ~ADC_OPM_MASK ) |
                                   ADC_OPM_POWERDN );
                OUT_32( HS_RXO_ADC1_CFGCTL, ulAdcDacRegVal );
            }
        }
        else if( OPM == OPM_ENABLE )
        {
            if( dcs == XCVR_TRX_RX1_ADC )
            {
                ulAdcDacRegVal =
                    IN_32( ( uint32_t * ) ( HS_RX_ADC0_CFGCTL ) );
                ulAdcDacRegVal = ( ( ulAdcDacRegVal &
                                     ~ADC_OPM_MASK ) |
                                   ADC_OPM_POWERUP );
                ulAdcDacRegVal |= ADC_SELOF_CTRL;
                ulAdcDacRegVal |= ADC_VCMOUT_CTRL;
                ulAdcDacRegVal |= ADC_CPLING_CTRL;
                ulAdcDacRegVal = ( ( ulAdcDacRegVal & ~ADC_I2C_ADDR_MASK )
                                   | ( 1 << ADC_I2C_ADDR_SHIFT ) );
                OUT_32( HS_RX_ADC0_CFGCTL, ulAdcDacRegVal );
            }

            else if( dcs == XCVR_TRX_RX2_ADC )
            {
                ulAdcDacRegVal =
                    IN_32( ( uint32_t * ) ( HS_RX_ADC1_CFGCTL ) );
                ulAdcDacRegVal = ( ( ulAdcDacRegVal &
                                     ~ADC_OPM_MASK ) |
                                   ADC_OPM_POWERUP );
                ulAdcDacRegVal |= ADC_SELOF_CTRL;
                ulAdcDacRegVal |= ADC_VCMOUT_CTRL;
                ulAdcDacRegVal |= ADC_CPLING_CTRL;
                ulAdcDacRegVal = ( ( ulAdcDacRegVal & ~ADC_I2C_ADDR_MASK )
                                   | ( 2 << ADC_I2C_ADDR_SHIFT ) );
                OUT_32( HS_RX_ADC1_CFGCTL, ulAdcDacRegVal );
            }

            else if( dcs == XCVR_RO1_ADC )
            {
                ulAdcDacRegVal =
                    IN_32( ( uint32_t * ) ( HS_RXO_ADC0_CFGCTL ) );
                ulAdcDacRegVal = ( ( ulAdcDacRegVal &
                                     ~ADC_OPM_MASK ) |
                                   ADC_OPM_POWERUP );
                ulAdcDacRegVal |= ADC_SELOF_CTRL;
                ulAdcDacRegVal |= ADC_VCMOUT_CTRL;
                ulAdcDacRegVal |= ADC_CPLING_CTRL;
                ulAdcDacRegVal = ( ( ulAdcDacRegVal & ~ADC_I2C_ADDR_MASK )
                                   | ( 4 << ADC_I2C_ADDR_SHIFT ) );
                OUT_32( HS_RXO_ADC0_CFGCTL, ulAdcDacRegVal );
            }

            else if( dcs == XCVR_RO2_ADC )
            {
                ulAdcDacRegVal =
                    IN_32( ( uint32_t * ) ( HS_RXO_ADC1_CFGCTL ) );
                ulAdcDacRegVal = ( ( ulAdcDacRegVal &
                                     ~ADC_OPM_MASK ) |
                                   ADC_OPM_POWERUP );
                ulAdcDacRegVal |= ADC_SELOF_CTRL;
                ulAdcDacRegVal |= ADC_VCMOUT_CTRL;
                ulAdcDacRegVal |= ADC_CPLING_CTRL;
                ulAdcDacRegVal = ( ( ulAdcDacRegVal & ~ADC_I2C_ADDR_MASK )
                                   | ( 8 << ADC_I2C_ADDR_SHIFT ) );
                OUT_32( HS_RXO_ADC1_CFGCTL, ulAdcDacRegVal );
            }
        }
        else
        {
            if( dcs == XCVR_TRX_RX1_ADC )
            {
                ulAdcDacRegVal =
                    IN_32( ( uint32_t * ) ( HS_RX_ADC0_CFGCTL ) );
                ulAdcDacRegVal = ( ( ulAdcDacRegVal &
                                     ~ADC_OPM_MASK ) |
                                   ADC_OPM_POWERSLEEP );
                OUT_32( HS_RX_ADC0_CFGCTL, ulAdcDacRegVal );
            }

            else if( dcs == XCVR_TRX_RX2_ADC )
            {
                ulAdcDacRegVal =
                    IN_32( ( uint32_t * ) ( HS_RX_ADC1_CFGCTL ) );
                ulAdcDacRegVal = ( ( ulAdcDacRegVal &
                                     ~ADC_OPM_MASK ) |
                                   ADC_OPM_POWERSLEEP );
                OUT_32( HS_RX_ADC1_CFGCTL, ulAdcDacRegVal );
            }

            else if( dcs == XCVR_RO1_ADC )
            {
                ulAdcDacRegVal =
                    IN_32( ( uint32_t * ) ( HS_RXO_ADC0_CFGCTL ) );
                ulAdcDacRegVal = ( ( ulAdcDacRegVal &
                                     ~ADC_OPM_MASK ) |
                                   ADC_OPM_POWERSLEEP );
                OUT_32( HS_RXO_ADC0_CFGCTL, ulAdcDacRegVal );
            }

            else if( dcs == XCVR_RO2_ADC )
            {
                ulAdcDacRegVal =
                    IN_32( ( uint32_t * ) ( HS_RXO_ADC1_CFGCTL ) );
                ulAdcDacRegVal = ( ( ulAdcDacRegVal &
                                     ~ADC_OPM_MASK ) |
                                   ADC_OPM_POWERSLEEP );
                OUT_32( HS_RXO_ADC1_CFGCTL, ulAdcDacRegVal );
            }
        }

        /* Set the sync bit and poll for its clearing */
        ulAdcDacRegVal = ADC_POWERON_SYNC_RETRY;
        OUT_32( REG_SPACE_SYNC, TRGR_ADC_SYNC |
                IN_32( ( uint32_t * ) REG_SPACE_SYNC ) );

        while( TRGR_ADC_SYNC &
               IN_32( ( uint32_t * ) REG_SPACE_SYNC ) )
        {
            if( !( ulAdcDacRegVal-- ) )
            {
                log_err( "%s: ADC reset sync failed\n",
                         __func__ );
                return pdFAIL;
            }
        }

        /* Poll the ADC ready bit */
        ulAdcDacRegVal = ADC_RDY_RETRY;

        if( OPM == OPM_ENABLE )
        {
            while( 0 == ( ( 1 << ( dcs - 1 ) ) &
                          IN_32( ( uint32_t * ) HSADC_STAT ) ) )
            {
                if( !( ulAdcDacRegVal-- ) )
                {
                    log_err( "%s: ADC ready bit is not set.\n",
                             __func__ );
                    return pdFAIL;
                }
            }
        }
        else
        {
            while( ( 1 << ( dcs - 1 ) ) & IN_32( ( uint32_t * ) HSADC_STAT ) )
            {
                if( !( ulAdcDacRegVal-- ) )
                {
                    log_err( "%s: ADC ready bit is not clear.\n",
                             __func__ );
                    return pdFAIL;
                }
            }
        }
    }

    return pdPASS;
}

BaseType_t xLa9310AdcDacSetGateReady( LA9310XcvrDCS_t dcs )
{
    uint32_t ulAdcDacRegVal;

    ulAdcDacRegVal = IN_32( ( uint32_t * ) ( GATE_READY ) );

    if( dcs == XCVR_TRX_TX_DAC )
    {
        ulAdcDacRegVal &= ~TX_DAC_GATE_RDY;
    }

    else if( dcs == XCVR_TRX_RX1_ADC )
    {
        ulAdcDacRegVal &= ~RX1_ADC_GATE_RDY;
    }

    else if( dcs == XCVR_TRX_RX2_ADC )
    {
        ulAdcDacRegVal &= ~RX2_ADC_GATE_RDY;
    }

    else if( dcs == XCVR_RO1_ADC )
    {
        ulAdcDacRegVal &= ~RO1_ADC_GATE_RDY;
    }

    else if( dcs == XCVR_RO2_ADC )
    {
        ulAdcDacRegVal &= ~RO2_ADC_GATE_RDY;
    }

    OUT_32( GATE_READY, ulAdcDacRegVal );

    OUT_32( REG_SPACE_SYNC, TRGR_GATE_RDY_SYNC |
            IN_32( ( uint32_t * ) REG_SPACE_SYNC ) );
    ulAdcDacRegVal = ADC_DAC_SYNC_RETRY;

    while( TRGR_GATE_RDY_SYNC & IN_32( ( uint32_t * ) REG_SPACE_SYNC ) )
    {
        if( !( ulAdcDacRegVal-- ) )
        {
            log_err( "%s: ADC/DAC Gate Ready failed\n",
                     __func__ );
            return pdFAIL;
        }
    }

    return pdPASS;
}

void vDcsInit( int adc_mask, int adc_freq_mask, int dac_mask, int dac_freq_mask)
{
    /* Init all ADC */
    uint8_t dcs = XCVR_TRX_RX1_ADC;
    int Freq;

    for(dcs = XCVR_TRX_RX1_ADC; dcs <= XCVR_RO2_ADC; dcs++ )
    {

        if (CHECK_BIT(adc_mask, (dcs -1)) == 0) {
                /* keep it power down */
                xLa9310AdcDacPowerDown(dcs);
                continue;
        }

	PRINTF("CHECK_BIT(adc_freq_mask, (dcs -1) = %d\r\n", CHECK_BIT(adc_freq_mask, (dcs -1)));
        if (CHECK_BIT(adc_freq_mask, (dcs -1)) == Half_Freq)
                Freq = Half_Freq;
        else
                Freq = Full_Freq;

        /* Configure the ADC clock */
        xLa9310ConfigAdcDacClock( dcs, Freq );

    	/* Enable and Reset the ADC */
        xLa9310AdcDacPowerUp( dcs );
        xLa9310ResetAdcDac( dcs );

    	/* Configure the ADC */
        xLa9310AdcDacOpModeCtrl( dcs, OPM_ENABLE );

    	/* Wait for ADC ready and trigger the Gate signal */
        xLa9310AdcDacSetGateReady( dcs );
    }

    /* Init DAC */
    if (dac_mask & 0x1) {
		if (dac_freq_mask == Half_Freq)
			Freq = Half_Freq;
		else
			Freq = Full_Freq;

        /* Configure the DAC clock */
        xLa9310ConfigAdcDacClock( XCVR_TRX_TX_DAC, Freq );

        /* Enable and Reset the DAC */
        xLa9310AdcDacPowerUp( XCVR_TRX_TX_DAC );
        xLa9310ResetAdcDac( XCVR_TRX_TX_DAC );

        /* Configure the DAC */
        xLa9310AdcDacOpModeCtrl( XCVR_TRX_TX_DAC, OPM_ENABLE );

        /* Wait for DAC ready and trigger the Gate signal */
        xLa9310AdcDacSetGateReady( XCVR_TRX_TX_DAC );

        log_info( "%s: DAC Init completed\r\n",  __func__ );
    }

    log_info("DCS Init completed\r\n");
}


BaseType_t vLa9310DCSClockSwitch( LA9310XcvrDCS_t dcs,  DCSFreq_t freq )
{
    BaseType_t xRet;

    do
    {
        /* Power down the DAC */
        xRet = xLa9310AdcDacOpModeCtrl( dcs, OPM_DISABLE );

        if( pdPASS != xRet )
        {
            log_err( "%s: DCS (%d) power down failed.\n\r", dcs, __func__ );
            break;
        }

        /* Configure the clock */
        xRet = xLa9310ConfigAdcDacClock( dcs, freq );

        if( pdPASS != xRet )
        {
            log_err( "%s: DCS (%d) clk config failed.\n\r", dcs, __func__ );
            break;
        }

        /* Power up the DAC */
        xRet = xLa9310AdcDacOpModeCtrl( dcs, OPM_ENABLE );

        if( pdPASS != xRet )
        {
            log_err( "%s: DCS (%d) power up failed.\n\r", dcs, __func__ );
            break;
        }
    } while( false );

    return xRet;
}
