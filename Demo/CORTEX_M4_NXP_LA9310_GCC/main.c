/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2017-2024 NXP
 */

#include "FreeRTOS.h"
#include "task.h"
#include "common.h"
#include "config.h"
#include "immap.h"
#include "la9310_main.h"
#include "la9310_irq.h"
#include "la9310_demo.h"
#include "la9310_gpio.h"
#include "la9310_edmaAPI.h"
#include "la9310_i2cAPI.h"
#include "exceptions.h"
#include <la9310.h>
#include <fsl_dspi.h>
#include "la9310_pinmux.h"
#include "bbdev_ipc.h"
#include "la9310_dcs_api.h"
#include <phytimer.h>
#include <sync_timing_device.h>
#include <sync_timing_device_cli.h>
#include <sw_cmd_engine.h>
#include "la9310_v2h_if.h"
#include "drivers/avi/la9310_vspa_dma.h"
#include <la9310_dcs_api.h>

#if NXP_ERRATUM_A_009410
    #include "la9310_pci.h"
#endif
#ifdef RUN_V2H_TEST_APP
    #include "la9310_v2h_test.h"
#endif
#include "la9310_avi.h"
#ifdef LA9310_DFE_APP
    #include "dfe_app.h"
#endif
#ifdef LA9310_ENABLE_COMMAND_LINE
    #include "UARTCommandConsole.h"
    #define mainUART_COMMAND_CONSOLE_STACK_SIZE       ( configMINIMAL_STACK_SIZE * 2 )
    #define mainUART_COMMAND_CONSOLE_TASK_PRIORITY    ( tskIDLE_PRIORITY + 1 )

/*
 * Register commands that can be used with FreeRTOS+CLI.  The commands are
 * defined in CLI-Commands.c.
 */
    extern void vRegisterNLMTestCommands( void );
#ifdef LA9310_DFE_APP
    extern void vRegisterDFETestCommands( void );
#endif

/* cOutputBuffer is used by FreeRTOS+CLI.  It is declared here so the
 * persistent qualifier can be used.  For the buffer to be declared here, rather
 * than in FreeRTOS_CLI.c, configAPPLICATION_PROVIDES_cOutputBuffer must be set to
 * 1 in FreeRTOSConfig.h. */
    char cOutputBuffer[ configCOMMAND_INT_MAX_OUTPUT_SIZE ] = { 0 };
#endif

uint32_t ulMemLogIndex;
struct la9310_info * pLa9310Info;
extern void vVSPAMboxInit();

void v_main_Hif_Init( struct la9310_info * pLa9310Info )
{
    struct la9310_hif * pxHif = pLa9310Info->pHif;

    pxHif->hif_ver = LA9310_VER_MAKE( LA9310_HIF_MAJOR_VERSION, LA9310_HIF_MINOR_VERSION );
    log_dbg( "%s: Initialized HIF - %d.%d", __func__, LA9310_HIF_MAJOR_VERSION,
             LA9310_HIF_MINOR_VERSION );

    pLa9310Info->stats = &pxHif->stats;
}

#ifdef TURN_ON_HOST_MODE
static int iCheckReadyState( struct la9310_info * pLa9310Info )
{
    int rc = HOST_NOT_READY;
    struct la9310_hif * pHif = pLa9310Info->pHif;

    log_info( "Waiting for HOST ready 0x%x\n\r", pHif->host_ready );

    while( 1 )
    {
        dmb();

        if( ( pHif->host_ready & LA9310_HOST_READY_MASK ) ==
            LA9310_HOST_READY_MASK )
        {
            rc = SUCCESS;
            break;
        }
    }

    return rc;
}
#endif

static void prvInitLa9310Info( struct la9310_info * pLa9310Info )
{
	#ifdef TURN_ON_HOST_MODE
    int i = 0;
    uint32_t uMSIAddrVal = 0;
    uint32_t __IO * pMsiAddrReg;
    uint32_t __IO * pMsiDataAddr;
    struct la9310_msi_info * pMsiInfo;
	#endif
    struct ccsr_dcr * pxDcr;

    pLa9310Info->itcm_addr = ( void * ) TCML_PHY_ADDR;
    pLa9310Info->dtcm_addr = ( void * ) TCMU_PHY_ADDR;
    pLa9310Info->pcie_addr = ( void * ) PCIE_BASE_ADDR;
    pLa9310Info->msg_unit = ( struct la9310_msg_unit * ) MSG_UNIT_BASE_ADDR;

    pLa9310Info->pcie_obound = ( void * ) PCIE_PHY_ADDR;
    pLa9310Info->pHif = ( struct la9310_hif * ) ( ( uint32_t ) pLa9310Info->itcm_addr +
                                                  LA9310_EP_HIF_OFFSET );
	pLa9310Info->pHif->dbg_log_regs.log_level = LA9310_LOG_LEVEL_INFO;
    pLa9310Info->pxDcr = ( void * ) DCR_BASE_ADDR;

	#ifdef TURN_ON_HOST_MODE
    /* Initialize MSI */
    pMsiAddrReg = ( uint32_t * ) ( ( uint32_t ) pLa9310Info->pcie_addr +
                                   PCIE_MSI_ADDR_REG );

    pMsiDataAddr = ( uint32_t * ) ( ( uint32_t ) pLa9310Info->pcie_addr +
                                    PCIE_MSI_DATA_REG_1 );

    pMsiInfo = &pLa9310Info->msi_info[ MSI_IRQ_MUX ];

    /* According to PCI bus standerd multiple MSIs are allocated consecutively*/
    uMSIAddrVal = ( IN_32( pMsiAddrReg ) & 0xFFF );
    #ifdef LS1046_HOST_MSI_RAISE
        /*This method to initialize MSI structure is non-standard and dedicated for
         * LS1046 host */

        for( i = 0; i < LA9310_MSI_MAX_CNT; i++ )
        {
            pMsiInfo[ i ].addr = ( LA9310_EP_TOHOST_MSI_PHY_ADDR | uMSIAddrVal );
            val = IN_32( pMsiDataAddr );
            pMsiInfo[ i ].data = ( ( val + i ) << 2 );
            log_info( "%s: MSI init[%d], addr 0x%x, data 0x%x\n\r", __func__,
                      i, pMsiInfo[ i ].addr, pMsiInfo[ i ].data );
        }
    #else /* ifdef LS1046_HOST_MSI_RAISE */
        for( i = 0; i < LA9310_MSI_MAX_CNT; i++ )
        {
            pMsiInfo[ i ].addr = ( LA9310_EP_TOHOST_MSI_PHY_ADDR | uMSIAddrVal );
            pMsiInfo[ i ].data = ( IN_32( pMsiDataAddr ) + i );
            log_info( "%s: MSI init[%d], addr 0x%x, data 0x%x\n\r", __func__,
                      i, pMsiInfo[ i ].addr, pMsiInfo[ i ].data );
        }
    #endif /* ifdef LS1046_HOST_MSI_RAISE */
	#endif //TURN_ON_STANDALONE_MODE

	pxDcr = ( struct ccsr_dcr * ) pLa9310Info->pxDcr;
	OUT_32(&pxDcr->ulScratchrw[LA9310_BOOT_HSHAKE_HIF_REG], LA9310_EP_HIF_OFFSET);
	dmb();
	OUT_32(&pxDcr->ulScratchrw[LA9310_BOOT_HSHAKE_HIF_SIZ_REG], sizeof(struct la9310_hif));
	dmb();
}

#ifdef TURN_ON_HOST_MODE
#ifdef RUN_V2H_TEST_APP
    void vV2H( void * pvParameters )
    {
        log_info( "V2H - In task\n\n\r" );
        log_info( "%s:Waiting of V2H_HOST ready\n\r", __func__ );
        struct la9310_hif * pHif = pLa9310Info->pHif;

        while( !( pHif->host_ready & LA9310_HIF_STATUS_V2H_READY ) )
        {
            vTaskDelay( 10 );
            dmb();
        }
        vV2HDemo( pLa9310Info );

        while( 1 )
        {
            vTaskDelay( 500 );
        }
    }
    #endif /* if RUN_V2H_TEST_APP */
#endif

/* iLa9310HostPreInit: Host and La9310 both do a handshake for clock configuration
 * and init synchronization vLa9310_do_handshake(). Host waits in while loop for
 * La9310 to indicate Host that it can proceed with Initialization
 * (LA9310_HOST_START_DRIVER_INIT).
 *
 * So Do all the initialization that you want in ratller which your host peer
 * code needs for initialization. If your host code wants some values
 * initialized by la9310 for it's initialization, then add call your init
 * function here.
 */
int iLa9310HostPreInit( struct la9310_info * pLa9310Info )
{
    int irc = 0;

    v_main_Hif_Init( pLa9310Info );

    irc = iLa9310IRQInit( pLa9310Info );

    if( irc )
    {
        goto out;
    }

    /*LA9310 IRQ MUX demo EVT (IRQ_EVT_TEST_BIT) registration */
    vLa9310DemoIrqEvtRegister( pLa9310Info );

    vVSPAMboxInit();

out:
    return irc;
}

/* iLa9310HostPostInit: Host and La9310 both do a handshake for clock configuration
 * and init synchronization vLa9310_do_handshake(). Host waits in while loop for
 * La9310 to indicate Host that it can proceed with Initialization
 * (LA9310_HOST_START_DRIVER_INIT).
 *
 * If your module code is dependent on any initialization that has to be done by
 * your peer code on host then add your post handskake init code here.
 */
int iLa9310HostPostInit( struct la9310_info * pLa9310Info )
{
    int irc = 0;

    irc = iEdmaInit();

    if( irc == 0 )
    {
        log_info( "%s: eDMA init DONE\n\r", __func__ );
    }

    return irc;
}

void iGpioInitRFIC( void )
{
    int iCnt = 0;

    for( iCnt = 6; iCnt < 12; iCnt++ )
    {
        vGpioSetPinMuxSingle( iCnt, SET_MUX_GPIO_MODE );
        iGpioInit( iCnt, output, false );
    }
}

void vInitMsgUnit( void )
{
    NVIC_SetPriority( IRQ_MSG3, 3 );
    NVIC_EnableIRQ( IRQ_MSG3 );
}

#if LA9310_UPGRADE_TIMESYNC_FW
int lSyncTimingDeviceUpgradeFirmware( SyncTimingDeviceContext_t *pxContext )
{
    struct ccsr_dcr * pxDcr = ( struct ccsr_dcr * ) pLa9310Info->pxDcr;
    struct la9310_hif * pxHif = pLa9310Info->pHif;
    struct la9310_sw_cmd_desc * pxCmdDesc = &( pxHif->sw_cmd_desc );
    struct la9310_std_fwupgrade_data *pxStdFwCmdData = ( struct la9310_std_fwupgrade_data * )( pxCmdDesc->data );
    SyncStatus_t returnStatus = eSyncStatusFailure;

    OUT_32( &pxDcr->ulScratchrw[ 1 ], LA9310_HOST_TIMESYNC_FW_LOAD );
    PRINTF( "Waiting to update timesync firmware\r\n" );
    while( IN_32( &pxDcr->ulScratchrw[ 1 ] ) != LA9310_HOST_TIMESYNC_FW_LOADED )
    {
        dmb();
    }

    returnStatus = xSyncTimingDeviceFwUpgrade( pxContext, pxStdFwCmdData );
    if( returnStatus != eSyncStatusSuccess )
    {
        return -1;
    }

    return 0;
}
#endif

void tmuInit( void ) {
	int idx;
	TmuRegs_t *pTmuHandle = ( TmuRegs_t * ) TMU_BASE_ADDR;
	out_le32( &pTmuHandle->tmr, TMU_TMR_DISABLE );

	out_le32( &pTmuHandle->ttrcr[ 0 ], TMU_TTRCR0_INIT );
	for (idx = 0;idx < TMU_TTRCR0_POINT;idx++) {
		out_le32( &pTmuHandle->ttcfgr,
			(TMU_TTCFGR_INIT0 + (idx*TMU_TTCFGR_DIFF)) );
		if (idx%2 == 0) {
			out_le32( &pTmuHandle->tscfgr, (TMU_TSCFGR_INIT0 +
					(((idx/2)*(TMU_TSCFGR_DIFF0)) +
					((idx/2)*(TMU_TSCFGR_DIFF0 + 1)))));
		} else {
			out_le32( &pTmuHandle->tscfgr, (TMU_TSCFGR_INIT0 +
					((((idx + 1)/2)*(TMU_TSCFGR_DIFF0)) +
					((idx/2)*(TMU_TSCFGR_DIFF0 + 1)))));
		}
	}

	out_le32( &pTmuHandle->ttrcr[ 1 ], TMU_TTRCR1_INIT );
	for (idx = 0;idx < TMU_TTRCR1_POINT;idx++) {
		out_le32( &pTmuHandle->ttcfgr,
				(TMU_TTCFGR_INIT1 + (idx * TMU_TTCFGR_DIFF)));
		out_le32( &pTmuHandle->tscfgr, (TMU_TSCFGR_INIT1 +
					(idx * TMU_TSCFGR_DIFF1)));
	}

	for (idx = 0;idx < TMU_TTRCR2_POINT;idx++) {
		out_le32( &pTmuHandle->ttcfgr,
				(TMU_TTCFGR_INIT2 + (idx * TMU_TTCFGR_DIFF)));
		out_le32( &pTmuHandle->tscfgr, (TMU_TSCFGR_INIT2 +
					(idx * TMU_TSCFGR_DIFF2)));
	}

	out_le32( &pTmuHandle->ttrcr[ 3 ], TMU_TTRCR3_INIT );
	for (idx=0;idx < TMU_TTRCR3_POINT;idx++) {
		out_le32( &pTmuHandle->ttcfgr,
				(TMU_TTCFGR_INIT3 + (idx * TMU_TTCFGR_DIFF)));
		if (idx == (TMU_TTRCR3_POINT - 1)) {
			out_le32( &pTmuHandle->tscfgr, (TMU_TSCFGR_INIT3 +
						(idx * TMU_TSCFGR_DIFF3) + 1));
		}
		else {
			out_le32( &pTmuHandle->tscfgr, (TMU_TSCFGR_INIT3 +
						(idx * TMU_TSCFGR_DIFF3)));
		}
	}
	out_le32( &pTmuHandle->teumr[ 0 ], TMU_TEUMR0_ENABLE );
	out_le32( &pTmuHandle->tdemar, TMU_TDEMAR_ENABLE );
	out_le32( &pTmuHandle->tmtmir, TMU_TMTMIR_ENABLE );
	out_le32( &pTmuHandle->monitoringSite[0].tmsar, TMU_TMSAR0_INIT );
	out_le32( &pTmuHandle->monitoringSite[1].tmsar, TMU_TMSAR1_INIT );
	out_le32( &pTmuHandle->monitoringSite[2].tmsar, TMU_TMSAR2_INIT );
	out_le32( &pTmuHandle->tmrtrcr, TMU_TMRTRCTR_INIT );
	out_le32( &pTmuHandle->tmftrcr, TMU_TMFTRCTR_INIT );
	out_le32( &pTmuHandle->tsr, TMU_TSR_INIT );

	for(int i=0;i<10000;i++);
	out_le32( &pTmuHandle->tmr, TMU_TMR_ENABLE );
}

int iInitHandler ( void )
{
    int irc = 0;
    SyncTimingDeviceContext_t *pxContext = NULL;
    void * avihndl = NULL;

    ulMemLogIndex = 0;

    iGpioInitRFIC();
    vEnableExceptions();

    pLa9310Info = pvPortMalloc( sizeof( struct la9310_info ) );

    if( !pLa9310Info )
    {
        PRINTF( "pLA9310info alloc failed. going for while(true)\n\r" );
        irc = -pdFREERTOS_ERRNO_ENOMEM;
        goto out;
    }

    memset( pLa9310Info, 0, sizeof( struct la9310_info ) );
#ifdef TURN_ON_HOST_MODE
    if( sizeof( struct la9310_hif ) > LA9310_EP_HIF_SIZE )
    {
        PRINTF( "Invalid HIF size\r\n" );
        irc = FAILURE;
        goto out;
    }
#endif //TURN_ON_STANDALONE_MODE
    /*XXX: DO NOT CALL log_*() before prvInitLa9310Info(), use PRINTF instead.*/
    prvInitLa9310Info( pLa9310Info );

    log_dbg( "%s: Allocated pLa9310Info %#x\n\r\n", __func__, pLa9310Info );

    /* XXX:NOTE - Do all initialization that is required by Host driver to
     * function like IRQ MUX, IPC, DMA in iLa9310HostPreInit().
     */
    irc = iLa9310HostPreInit( pLa9310Info );

    if( irc )
    {
        log_err( "%s: iLa9310HostPreInit Failed, rc %d\n\r", __func__, irc );
        goto out;
    }


    pxContext = pxSyncTimingDeviceInit();
#if LA9310_UPGRADE_TIMESYNC_FW
    if( lSyncTimingDeviceUpgradeFirmware( pxContext ) )
    {
        log_err( "\r\n" );
        goto out;
    }
#else
    ( void ) pxContext;
#endif
    /*Till Here system is running at 100 Mhz*/
    vLa9310_do_handshake( pLa9310Info );

    #ifdef TURN_ON_HOST_MODE
    #if NXP_ERRATUM_A_009410
        vPCIEInterruptInit();
    #endif
    #endif //TURN_ON_STANDALONE_MODE

    vInitMsgUnit();
#ifdef LA9310_SYNC_TIME_MODE
    if( lSwCmdEngineInit() != 0 )
    {
            log_err( "sw cmd engine init failed\r\n" );
            irc = FAILURE;
            goto out;
    }
#endif
#ifdef __RFIC
    if( pdTRUE != iRficInit( pLa9310Info ))
    {
        log_err( "%s: RFIC Init Failed, rc %d\n\r", __func__, irc );
        irc = FAILURE;
        goto out;
    }
#endif

    irc = iLa9310HostPostInit( pLa9310Info );

    if( irc )
    {
        log_err( "%s: iLa9310HostPostInit Failed, rc %d\n\r", __func__, irc );
        goto out;
    }
#ifdef TURN_ON_HOST_MODE
    if( bbdev_ipc_init( 0, 0 ) )
    {
        log_err( "IPC Init failed\r\n" );
        irc = FAILURE;
        goto out;
    }

    /* Till here Host should be ready */
    irc = iCheckReadyState( pLa9310Info );

    if( irc )
    {
        log_err( "%s: iCheckReadyState Failed, rc %d\n\r", __func__, irc );
        goto out;
    }

#endif //TURN_ON_STANDALONE_MODE
    vPhyTimerReset();
    /* Run phy timer at PLAT_FREQ / 8 = ( 122.88 * 4 ) / 8 = 61.44MHz */
    vPhyTimerEnable( PHY_TMR_DIVISOR );
#ifndef LA9310_DFE_APP
    vPhyTimerPPSOUTConfigGPSlike();
    /* Force RO1 always on */
    vPhyTimerComparatorForce(PHY_TIMER_COMP_R01, ePhyTimerComparatorOut1);
#endif
    /*VSPA AVI Init*/
    avihndl = iLa9310AviInit();

    if( NULL == avihndl )
    {
        log_err( "ERR: %s: AVI Initialization Failed\n\r", __func__ );
    }
#ifdef TURN_ON_STANDALONE_MODE
	iLoadTableToTCM();
    LoadVSPAImage();
#endif //TURN_ON_STANDALONE_MODE
    iLa9310AviConfig();

    vDcsInit(IN_32(&pLa9310Info->pHif->adc_mask),
		IN_32(&pLa9310Info->pHif->adc_rate_mask),
		IN_32(&pLa9310Info->pHif->dac_mask),
		IN_32(&pLa9310Info->pHif->dac_rate_mask));

    #ifdef TURN_ON_HOST_MODE
    #ifdef RUN_V2H_TEST_APP
        irc = xTaskCreate( vV2H, "LA9310 V2H task", configMINIMAL_STACK_SIZE,
                           pLa9310Info, tskIDLE_PRIORITY + 1, NULL );

        if( irc != pdPASS )
        {
            log_info( "Failed to create V2H task, not starting scheduler\n\r" );
            goto out;
        }
    #endif
    #endif //TURN_ON_STANDALONE_MODE

    #ifdef LA9310_ENABLE_COMMAND_LINE
        vRegisterTimesyncCLICommands();
    #endif

    irc = SUCCESS;

out:
    return irc;
}

/*
 *        La9310 Application Entry point
 */
int main( void )
{
    int irc = 0;
	uint32_t BootSource;

    /* Initialize hardware */
    vHardwareEarlyInit();
    BootSource = ((IN_32( ( uint32_t * ) DCR_BASE_ADDR )) >>
                   LX9310_BOOT_SRC_SHIFT) & LX9310_BOOT_SRC_MASK;

    if ( BootSource == LA9310_BOOT_SRC_PCIE ) {
        PRINTF( "STARTING NLM.. Boot Source (PCIe) \n\r" );
    } else if ( BootSource == LA9310_BOOT_SRC_I2C ) {
        PRINTF( "STARTING NLM.. Boot Source (I2C) \n\r" );
    } else {
      log_err( "Invalid Boot Source \n\r");
      goto out;
    }

	PRINTF("FreeRTOS Kernel vesrion %s\n\r",tskKERNEL_VERSION_NUMBER);
    irc = iInitHandler();
    if ( irc )
    {
      goto out;
    }

    #ifdef LA9310_ENABLE_COMMAND_LINE
    #ifdef LA9310_DFE_APP
        vRegisterDFETestCommands();
    #else
        vRegisterNLMTestCommands();
    #endif
        vUARTCommandConsoleStart( mainUART_COMMAND_CONSOLE_STACK_SIZE,
                                  mainUART_COMMAND_CONSOLE_TASK_PRIORITY );
    #endif

    #ifdef LA9310_DFE_APP
    irc = vDFEInit();
    if ( irc )
    {
        log_err( "%s: vDFEInit, rc %d\n\r", __func__, irc );
    }
    #endif


    tmuInit();

    /* Start FreeRTOS scheduler */
    vTaskStartScheduler();

out:
    log_err( "%s: Something terrible has happend, rc %d\n\r", __func__, irc );
    log_err( "%s: Going for infitite loop of death\n\r", __func__ );

    /* Should never reach this point */
    while( true )
    {
    }
}
