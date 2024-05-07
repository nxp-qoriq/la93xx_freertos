/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright 2021-2022, 2024 NXP
 */
#include "rfic_core.h"
#include "debug_console.h"
#include "config.h"
#include "fsl_dspi.h"
#include "rfic_vga.h"
#include "rfic_synth.h"
#include "rfic_demod.h"

/* [0 : 1] = [Pin Number : Default Value] */
static const uint32_t RF_GPIO[ RF_GPIO_MAX ][ 2 ] = {
				{ 6, 1 },	/* Demod RFA selected */
				{ 7, 0 },	/* RFA-B13 selected */
				{ 8, 0 },	/* RFB-B3 selected */
				{ 9, 0 },	/* B3 LNA bypass */
				{ 10, 0 },	/* B13 LNA bypass */
				{ 11, 0 },	/* N77 LNA bypass */
				{ 12, 0 }};	/* PLL selected */


int32_t vRficInitGpio( RficDevice_t *pRficDev )
{
    uint8_t index;

    /* Initialize RF GPIO */
    for( index = 0; index < RF_GPIO_MAX; index++ )
    {
	pRficDev->eGpio[ index ].etype = output;
	pRficDev->eGpio[ index ].ulPin = RF_GPIO[ index ][ 0 ];
	pRficDev->eGpio[ index ].ucDefaultVal = RF_GPIO[ index ][ 1 ];

	if( iGpioInit( pRficDev->eGpio[ index ].ulPin,
		   pRficDev->eGpio[ index ].etype,
		   false ))
	{
	    log_err( "%s: GPIO-%d init failed\r\n", __func__,
		     pRficDev->eGpio[ index ].ulPin );
	    return -1;
	}

	if( iGpioSetData( pRficDev->eGpio[ index ].ulPin,
		      pRficDev->eGpio[ index ].ucDefaultVal ))
	{
	    log_err( "%s: GPIO-%d set failed\r\n", __func__,
		     pRficDev->eGpio[ index ].ulPin );
	    return -1;
	}
    }

    return 0;
}

void vRficInitHif( rf_host_if_t *pRfHif )
{
    /* Init rf priv mdata */
    pRfHif->rf_priv_mdata.band = RF_SW_BAND_B13;
    pRfHif->rf_priv_mdata.freq_khz = RFIC_SYNTH_DEFAULT_FREQ;
    pRfHif->rf_priv_mdata.bw = RF_BW_36MHZ;
    pRfHif->rf_priv_mdata.lna_state = 0;
    pRfHif->rf_priv_mdata.gain_in_db = 0;
    pRfHif->rf_priv_mdata.demod_rf_attn = 0x0;
    pRfHif->rf_priv_mdata.demod_bb_gain = 0x6;
    pRfHif->rf_priv_mdata.vga_dac1_val = 2048;
    pRfHif->rf_priv_mdata.vga_dac2_val = 2048;

    /* Init rf statistic */
    pRfHif->rf_stats.local_cmd_count = 0;
    pRfHif->rf_stats.remote_cmd_count = 0;
    pRfHif->rf_stats.local_cmd_failed_count = 0;
    pRfHif->rf_stats.remote_cmd_failed_count = 0;
}

int32_t iRficSelectDspiSlave( RficDevice_t *pRficDev , RficDspiSlv_t eSlaveDev )
{
    int32_t iRet = 0;

    if( RF_DSPI_SLV_SYNTH == eSlaveDev )
    {
	iRet = iGpioSetData( pRficDev->eGpio[ RF_GPIO_SPI_CS0_SEL ].ulPin, 0 );
    }
    else if( RF_DSPI_SLV_VGA == eSlaveDev )
    {
	iRet = iGpioSetData( pRficDev->eGpio[ RF_GPIO_SPI_CS0_SEL ].ulPin, 1 );
    }
    else
    {
	/* Do nothing */
    }

    if( 0 > iRet )
    {
	log_err( "%s: Set GPIO[%d] fail, error[%d]\r\n", __func__,
		 pRficDev->eGpio[ RF_GPIO_SPI_CS0_SEL ].ulPin, iRet );
    }

    return iRet;
}

BaseType_t iRficInit( struct la9310_info *pLa9310Info )
{
    BaseType_t xRet = pdTRUE;
    RficDevice_t *pRficDev;
    struct la9310_hif *pHif = pLa9310Info->pHif;

    log_info( "%s: Started\n\r", __func__ );

    /* Allocate RFIC Device Structures */
    pRficDev = pvPortMalloc(sizeof( RficDevice_t ));
    if( !pRficDev ) {
        log_err( "%s: Failed to allocate rfic_dev\n\r", __func__ );
        return pdFALSE;
    }
    memset( pRficDev, 0, sizeof( RficDevice_t ));

    /* Initialize DSPI instance */
    pRficDev->pDspiHandle = pxDspiInit( (( 1 << DSPI_CS3 ) | ( 1 << DSPI_CS0 )),
					RF_DSPI_MAX_CLK );

    pRficDev->iq_phys_addr = IN_32(&pHif->iq_phys_addr);
    pRficDev->iq_mem_addr = IN_32(&pHif->iq_mem_addr);
    pRficDev->iq_mem_size = IN_32(&pHif->iq_mem_size);
    log_dbg("iq_phys_addr:0x%x, iq_mem_addr:0x%x, iq_mem_size:0x%x\n",
		    pRficDev->iq_phys_addr, pRficDev->iq_mem_addr,
		    pRficDev->iq_mem_size);
    /* Init RF GPIO */
    if( vRficInitGpio( pRficDev ))
    {
	log_err( "%s: GPIO Init failed\n\r", __func__ );
	return pdFALSE;
    }

    /* Select DPSI Slave - synthesizer */
    if( iRficSelectDspiSlave( pRficDev, RF_DSPI_SLV_SYNTH ))
    {
        log_err( "%s: Select DSPI Slave failed\n\r", __func__ );
        return pdFALSE;
    }
    /* Init RF SYNTH - LMX2582 */
    if( RficSynthInit( pRficDev ))
    {
        log_err( "%s: Synth Init failed\n\r", __func__ );
        return pdFALSE;
    }

    /* Select DPSI Slave - VGA */
    if( iRficSelectDspiSlave( pRficDev, RF_DSPI_SLV_VGA ))
    {
	log_err( "%s: Select DSPI Slave failed\n\r", __func__ );
	return pdFALSE;
    }
    /* Init RF VGA - ADRF6520 */
    if( RficVgaInit( pRficDev ))
    {
	log_err( "%s: Vga Init failed\n\r", __func__ );
	return pdFALSE;
    }

    if( RficDemodInit( pRficDev ))
    {
        log_err( "%s: Demod Init failed\n\r", __func__ );
        return pdFALSE;
    }

    /* Init RF HIF */
    vRficInitHif( &pHif->rf_hif );
    pRficDev->pRfHif = &pHif->rf_hif;

    #ifndef TURN_ON_STANDALONE_MODE
    /* Create local and remote queue */
    pRficDev->xLocalQueue = xQueueCreate( 1, sizeof( void * ));
    if( NULL == pRficDev->xLocalQueue ) {
        log_err( "%s: Local queue create failed\n\r", __func__ );
        return pdFALSE;
    }

    pRficDev->xLocalRspQueue = xQueueCreate( 1, sizeof( void * ));
    if( NULL == pRficDev->xLocalRspQueue ) {
        log_err( "%s: Local response queue create failed\n\r", __func__ );
        return pdFALSE;
    }

    pRficDev->xRemoteQueue = xQueueCreate( 1, sizeof( void * ));
    if( NULL == pRficDev->xRemoteQueue ) {
        log_err( "%s: Remote queue create failed\n\r", __func__ );
        return pdFALSE;
    }

    /* Create event group for SWCMD */
    pRficDev->xSwCmdEvent = xEventGroupCreate();
    if( NULL == pRficDev->xSwCmdEvent )
    {
	log_err( "%s: Event group create failed\n\r", __func__ );
	return pdFALSE;
    }

    /* Create core task */
    xRet = xTaskCreate( vRficCoreTask,
			"RFcoreTask",
			RF_CORE_TASK_STACK_SIZE,
                        ( void * )pRficDev,
			RF_CORE_TASK_PRIORITY,
                        ( TaskHandle_t * )&( pRficDev->xCoreTask ));
    if( xRet != pdPASS ) {
	log_err( "%s: Core task create failed\n\r", __func__ );
	return xRet;
    }
    #endif

    /* All good, link it up! */
    pRficDev->pLa9310Info = pLa9310Info;
    pLa9310Info->pRficDev = pRficDev;
    pRficDev->pRfHif->ready = RF_READY;
    log_info( "%s: Completed\n\r", __func__ );

    return xRet;
}
