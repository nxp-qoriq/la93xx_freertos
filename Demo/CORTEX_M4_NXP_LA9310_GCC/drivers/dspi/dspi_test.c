/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2021 NXP
 */

#include <fsl_dspi.h>
#include <config.h>
#include "dspi_test.h"
#include "FreeRTOS.h"
#include "task.h"
#include "la9310_gpio.h"
#include "la9310_pinmux.h"
#include <phytimer.h>

int32_t prvLMX2582SynthWriteReg( struct LA931xDspiInstance * pDspiHandle,
                                 uint8_t addr,
                                 uint16_t data )
{
    int32_t iRet;
    uint8_t ucData[ 4 ] = { 0 };

    log_info( "%s: WriteReg[%x : %x]\r\n", __func__, addr, data );

    /* Write synthesizer register */
    ucData[ 0 ] = addr;
    ucData[ 1 ] = ( uint8_t ) ( ( data & 0xFF00 ) >> 8 );
    ucData[ 2 ] = ( uint8_t ) ( data & 0x00FF );
    iRet = lDspiPush( pDspiHandle, DSPI_CS0, DSPI_DEV_WRITE,
                      ucData, 4 );

    if( 0 > iRet )
    {
        log_err( "%s: lDspiPush failed, error[%d]\r\n", __func__,
                 iRet );
    }

    return iRet;
}

int32_t prvLMX2582SynthReadReg( struct LA931xDspiInstance * pDspiHandle,
                                uint8_t addr,
                                uint16_t * data )
{
    int32_t iRet;
    uint8_t ucData[ 4 ] = { 0 };

    /* Push address to be read, Set MSB bit
     * which indicate read operation */
    ucData[ 0 ] = addr | LMX2582_SYNTH_READ_OPR;
    iRet = lDspiPush( pDspiHandle, DSPI_CS0, DSPI_DEV_READ,
                      ucData, 4 );

    if( 0 > iRet )
    {
        log_err( "%s: lDspiPush failed, error[%d]\r\n", __func__,
                 iRet );
        return iRet;
    }

    iRet = lDspiPop( pDspiHandle, ucData, 4 );

    if( 0 > iRet )
    {
        log_err( "%s: lDspiPop failed, error[%d]\r\n", __func__,
                 iRet );
        return iRet;
    }

    /* index 0 should be ignored, this is outcome
     * of above push operation */
    *data = ( ( int16_t ) ( ucData[ 1 ] ) << 8 ) | ucData[ 2 ];

    return iRet;
}

int32_t prvLTC5586DemodWriteReg( struct LA931xDspiInstance * pDspiHandle,
                                 uint8_t addr,
                                 uint8_t data )
{
    int32_t iRet;
    uint8_t ucData[ 4 ] = { 0 };
    DspiChipSel_t eChipSelect = DSPI_CS3;

    log_info( "%s: WriteReg[%x : %x]\r\n", __func__, addr, data );
    /* Read demod register before writing */
    ucData[ 0 ] = addr | LTC5586_DEMOD_READ_OPR;
    iRet = lDspiPush( pDspiHandle, eChipSelect, DSPI_DEV_READ,
                      ucData, 4 );

    if( 0 > iRet )
    {
        log_err( "%s: lDspiPush failed, error[%d]\r\n", __func__,
                 iRet );
        return iRet;
    }

    iRet = lDspiPop( pDspiHandle, ucData, 4 );

    if( 0 > iRet )
    {
        log_err( "%s: lDspiPop failed, error[%d]\r\n", __func__,
                 iRet );
        return iRet;
    }

    /* Write demodulator register */
    ucData[ 0 ] = addr;
    ucData[ 1 ] = data;
    iRet = lDspiPush( pDspiHandle, eChipSelect, DSPI_DEV_WRITE,
                      ucData, 4 );

    if( 0 > iRet )
    {
        log_err( "%s: lDspiPush failed, error[%d]\r\n", __func__,
                 iRet );
    }

    return iRet;
}

int32_t prvLTC5586DemodReadReg( struct LA931xDspiInstance * pDspiHandle,
                                uint8_t addr,
                                uint8_t * data )
{
    int32_t iRet;
    uint8_t ucData[ 4 ] = { 0 };
    DspiChipSel_t eChipSelect = DSPI_CS3;

    /* Push address to be read, Set MSB bit
     * which indicate read operation */
    ucData[ 0 ] = addr | LTC5586_DEMOD_READ_OPR;
    iRet = lDspiPush( pDspiHandle, eChipSelect, DSPI_DEV_READ,
                      ucData, 4 );

    if( 0 > iRet )
    {
        log_err( "%s: lDspiPush failed, error[%d]\r\n", __func__,
                 iRet );
        return iRet;
    }

    iRet = lDspiPop( pDspiHandle, ucData, 4 );

    if( 0 > iRet )
    {
        log_err( "%s: lDspiPop failed, error[%d]\r\n", __func__,
                 iRet );
        return iRet;
    }

    /* index 0 should be ignored, this is outcome
     * of above push operation */
    *data = ucData[ 1 ];

    return iRet;
}

int32_t LMX2582SynthTest( struct LA931xDspiInstance * pDspiHandle )
{
    int32_t iRet;
    uint16_t usRdData, usWrData;

    /* soft reset */
    usWrData = LMX2582_SYNTH_RESET;
    iRet = prvLMX2582SynthWriteReg( pDspiHandle, LMX2582_SYNTH_REG0, usWrData );

    if( iRet < 0 )
    {
        log_err( "%s: synth write reg[%x] failed\r\n", __func__, LMX2582_SYNTH_REG0 );
        return -1;
    }

    usWrData = 0x2218;
    iRet = prvLMX2582SynthWriteReg( pDspiHandle, LMX2582_SYNTH_REG0, usWrData );

    if( iRet < 0 )
    {
        log_err( "%s: synth write reg[%x] failed\r\n", __func__, LMX2582_SYNTH_REG0 );
        return -1;
    }

    usRdData = 0;
    iRet = prvLMX2582SynthReadReg( pDspiHandle, LMX2582_SYNTH_REG0, &usRdData );

    if( iRet < 0 )
    {
        log_err( "%s: synth read reg[%x] failed\r\n", __func__, LMX2582_SYNTH_REG0 );
        return -1;
    }

    if( usWrData == usRdData )
    {
        log_info( "LMX2582_SYNTH_REG0 value: 0x%x\r\n", usRdData );
        log_info( "LMX2582 DSPI test 1 PASS\r\n" );
    }
    else
    {
        log_err( "LMX2582_SYNTH_REG0 value: 0x%x (should be 0x%x)\r\n", usRdData, usWrData );
        log_err( "LMX2582 DSPI test 1 FAIL\r\n" );
    }

    usRdData = 0;
    iRet = prvLMX2582SynthReadReg( pDspiHandle, LMX2582_SYNTH_REG1, &usRdData );

    if( iRet < 0 )
    {
        log_err( "%s: synth read reg[%x] failed\r\n", __func__, LMX2582_SYNTH_REG0 );
        return -1;
    }

    if( usRdData == 0x80b )
    {
        log_info( "LMX2582_SYNTH_REG1 value: 0x%x (Matching with default value of datasheet)\r\n", usRdData );
        log_info( "LMX2582 DSPI test 2 PASS\r\n" );
    }
    else
    {
        log_err( "LMX2582 DSPI test 2 FAIL\r\n" );
    }

    return 0;
}

int32_t LTC5586DemodTest( struct LA931xDspiInstance * pDspiHandle )
{
    int32_t iRet;
    uint8_t usWrData, usRdData;

    /* Demodulator-1 soft reset */
    usWrData = LTC5586_DEMOD_RESET;
    iRet = prvLTC5586DemodWriteReg( pDspiHandle, LTC5586_DEMOD_REG22, usWrData );

    if( iRet < 0 )
    {
        log_err( "%s: demod write reg[%x] failed\r\n", __func__,
                 LTC5586_DEMOD_REG22 );
        return -1;
    }

    usRdData = 0x00;
    iRet = prvLTC5586DemodReadReg( pDspiHandle, LTC5586_DEMOD_REG16, &usRdData );

    if( iRet < 0 )
    {
        log_err( "%s: demod read reg[%x] failed\r\n", __func__,
                 LTC5586_DEMOD_REG16 );
        return -1;
    }
    else if( usRdData != 0x4 )
    {
        log_err( "LTC5586_DEMOD_REG 0x10 default value != 0x4\r\n" );
        log_err( "LTC5586_DEMOD test failed\r\n" );
        return -1;
    }

    usWrData = 0x55;
    iRet = prvLTC5586DemodWriteReg( pDspiHandle, LTC5586_DEMOD_REG16, usWrData );

    if( iRet < 0 )
    {
        log_err( "%s: demod write reg[%x] failed\r\n", __func__,
                 LTC5586_DEMOD_REG16 );
        return -1;
    }

    usRdData = 0x00;
    iRet = prvLTC5586DemodReadReg( pDspiHandle, LTC5586_DEMOD_REG16, &usRdData );

    if( iRet < 0 )
    {
        log_err( "%s: demod read reg[%x] failed\r\n", __func__,
                 LTC5586_DEMOD_REG16 );
        return -1;
    }

    if( usRdData == usWrData )
    {
        log_info( "%s: Test Pass [%x]  Val [Write: %x Read %x] \r\n", __func__, LTC5586_DEMOD_REG16, usWrData, usRdData );
    }
    else
    {
        log_info( "%s: Test Fail [%x]  Val [Write: %x Read %x] \r\n", __func__, LTC5586_DEMOD_REG16, usWrData, usRdData );
    }

    return 0;
}

int32_t prvADRF6520WriteReg( struct LA931xDspiInstance * pDspiHandle,
                             uint16_t addr,
                             uint8_t data )
{
    int32_t iRet;
    uint8_t ucData[ 4 ] = { 0 };

    log_info( "%s: WriteReg[%x : %x]\r\n", __func__, addr, data );

    /* Write synthesizer register */
    ucData[ 0 ] = ( uint8_t ) ( ( addr & 0xFF00 ) >> 8 );
    ucData[ 1 ] = ( uint8_t ) ( addr & 0x00FF );
    ucData[ 2 ] = data;
    iRet = lDspiPush( pDspiHandle, DSPI_CS0, DSPI_DEV_WRITE,
                      ucData, 4 );

    if( 0 > iRet )
    {
        log_err( "%s: lDspiPush failed, error[%d]\r\n", __func__,
                 iRet );
    }

    return iRet;
}

int32_t ADRF6520Test( struct LA931xDspiInstance * pDspiHandle )
{
    int32_t iRet;
    uint8_t usWrData = 0x97;

    iRet = prvADRF6520WriteReg( pDspiHandle, ADRF6520_REG, usWrData );

    if( iRet < 0 )
    {
        log_err( "%s: synth write reg[%x] failed\r\n", __func__, ADRF6520_REG );
        return -1;
    }

    log_info( "ADRF6520 written: 0x%x, Test PASS\r\n", usWrData );

    return 0;
}

/* ulTestID - DSPI peripheral identifier
 *            0 - LTC5586
 *            1 - LMX2582
 *            2 - ADRF6520
 *
 * ulMode - CS driving mode
 *            0 - Drive LMX2582 vs ADRF6520 cs using GPIO_12
 *            1 - Drive LMX2582 vs ADRF6520 cs using PA_EN (through phy timer)
 */
void vDspiTest( uint32_t ulTestID,
                uint32_t ulMode )
{
    int32_t iRet = 0;
    struct LA931xDspiInstance * pDspiHandle = NULL;
    uint32_t ulCapture = 0;

    /* Initialize DSPI Handler*/
    pDspiHandle = pxDspiInit( ( ( 1 << DSPI_CS3 ) | ( 1 << DSPI_CS0 ) ),
			      DSPI_DEFAULT_FREQUENCY );

    if( NULL != pDspiHandle )
    {
        if( ulTestID == 0 )
        {
            iRet = LTC5586DemodTest( pDspiHandle );

            if( 0 != iRet )
            {
                log_err( "%s: NLM Demod init failed \r\n",
                         __func__ );
            }
        }
        else if( ulTestID == 1 )
        {
            /* Initialize GPIO 12 to select CS for Synthizer LMX2582 */
            if( ulMode == 0 )
            {
                vGpioSetPinMuxSingle( SPI_CS0_SEL_GPIO, SET_MUX_GPIO_MODE );
                iGpioInit( SPI_CS0_SEL_GPIO, output, false );
                iGpioSetData( SPI_CS0_SEL_GPIO, 0 );
            }
            else
            {
                /*
                 * Configure PA_EN pin as gpio and drive it to 1 so that
                 * CS is selected for ADRF6520 and not for LMX2582, This
                 * way we can make sure that phy timer is really working
                 * and making the pin 0 again.
                 */
                vGpioSetPinMuxSingle( SPI_CS0_SEL_GPIO, SET_MUX_GPIO_MODE );
                iGpioInit( SPI_CS0_SEL_GPIO, output, false );
                iGpioSetData( SPI_CS0_SEL_GPIO, 1 );

                /* Drive PA_EN to 0 using phy timer */
                vGpioSetPinMuxSingle( SPI_CS0_SEL_GPIO, SET_MUX_NON_GPIO_MODE );
                ulCapture = ulPhyTimerCapture( PHY_TIMER_COMP_PA_EN );
                vPhyTimerComparatorConfig(
                    PHY_TIMER_COMP_PA_EN,
                    PHY_TIMER_COMPARATOR_CLEAR_INT,
                    ePhyTimerComparatorOut0,
                    ulCapture + 30720 ); /* 500 us */
                vTaskDelay( 1 );         /* Wait for more than 500 us */
            }

            iRet = LMX2582SynthTest( pDspiHandle );

            if( 0 != iRet )
            {
                log_err( "%s: NLM Synth init failed \r\n",
                         __func__ );
            }
        }
        else if( ulTestID == 2 )
        {
            /* Initialize GPIO 12 to select CS for ADRF6520 */
            if( ulMode == 0 )
            {
                vGpioSetPinMuxSingle( SPI_CS0_SEL_GPIO, SET_MUX_GPIO_MODE );
                iGpioInit( SPI_CS0_SEL_GPIO, output, false );
                iGpioSetData( SPI_CS0_SEL_GPIO, 1 );
            }
            else
            {
                vGpioSetPinMuxSingle( SPI_CS0_SEL_GPIO, SET_MUX_NON_GPIO_MODE );
                ulCapture = ulPhyTimerCapture( PHY_TIMER_COMP_PA_EN );
                vPhyTimerComparatorConfig(
                    PHY_TIMER_COMP_PA_EN,
                    PHY_TIMER_COMPARATOR_CLEAR_INT,
                    ePhyTimerComparatorOut1,
                    ulCapture + 30720 ); /* 500 us */
                vTaskDelay( 1 );         /* Wait for more than 500 us */
            }

            iRet = ADRF6520Test( pDspiHandle );

            if( 0 != iRet )
            {
                log_err( "%s: NLM Synth init failed \r\n",
                         __func__ );
            }
        }
        else
        {
            log_err( "Wrong DSPI test id\r\n" );
        }

        vDspiExit();
    }
    else
    {
        log_err( "%s: NLM DSPI init failed \r\n",
                 __func__ );
    }
}
