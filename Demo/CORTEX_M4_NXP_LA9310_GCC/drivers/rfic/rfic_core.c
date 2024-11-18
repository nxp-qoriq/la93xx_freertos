/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright 2021-2024 NXP
 */
#include "rfic_core.h"
#include "debug_console.h"
#include "rfic_sw_cmd.h"
#include "rfic_cmd.h"

#include <rfic_avi_ctrl.h>
#include <la9310_irq.h>

#ifndef TURN_ON_STANDALONE_MODE
BaseType_t xRficPostLocalSwCmd( RficDevice_t *pRficDev,
				rf_sw_cmd_desc_t *pSwCmdDesc )
{
    BaseType_t xRet;

    xRet = xQueueSend( pRficDev->xLocalQueue, &pSwCmdDesc,
		       RF_LOCAL_QUEUE_SEND_TIMEOUT );
    if( xRet == pdPASS )
    {
	pSwCmdDesc->status = RF_SW_CMD_STATUS_POSTED;
	xEventGroupSetBits( pRficDev->xSwCmdEvent, RF_LOCAL_CMD_EVENT );

	xRet = xQueueReceive( pRficDev->xLocalRspQueue, pSwCmdDesc,
			      RF_LOCAL_RESP_QUEUE_RECV_TIMEOUT );

	if( pdTRUE != xRet)
	{
	    log_err( "%s: Command response timeout\n\r", __func__);
	    RF_STATS_ADD( pRficDev->pRfHif->rf_stats.local_cmd_failed_count );
	}
    }
    else
    {
	log_err( "%s: Command request send failed\n\r", __func__);
	RF_STATS_ADD( pRficDev->pRfHif->rf_stats.local_cmd_failed_count );
    }

    return xRet;
}

void vRficSwCmdIrq( RficDevice_t *pRficDev )
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    rf_sw_cmd_desc_t *pSwCmdDesc;
    BaseType_t xRet = pdTRUE;

    pSwCmdDesc = &( pRficDev->pRfHif->rf_mdata.host_swcmd);

    xRet = xQueueSendFromISR( pRficDev->xRemoteQueue, &pSwCmdDesc, NULL );
    if( pdPASS == xRet )
    {
	pSwCmdDesc->status = RF_SW_CMD_STATUS_POSTED;
        if( xEventGroupSetBitsFromISR( pRficDev->xSwCmdEvent, RF_REMOTE_CMD_EVENT,
				       &xHigherPriorityTaskWoken ) == pdPASS )
        {
            portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
        }
        else
        {
            log_err( "%s: Host cmd event failed\n\r", __func__ );
            RF_STATS_ADD( pRficDev->pRfHif->rf_stats.remote_cmd_failed_count );
        }
    }
    else
    {
        log_err( "%s: Host cmd enqueue failed\n\r", __func__ );
        RF_STATS_ADD( pRficDev->pRfHif->rf_stats.remote_cmd_failed_count );
    }
}
#endif

void vRficSetIqImbalance(rf_sw_cmd_desc_t *rfic_sw_cmd);
BaseType_t xHandleSwCmd( RficDevice_t *pRficDev, rf_sw_cmd_desc_t *pSwCmdDesc)
{
    BaseType_t xRet = pdTRUE;

    log_dbg( "%s: CmdId[%d]\n\r", __func__, pSwCmdDesc->cmd );
    switch( pSwCmdDesc->cmd )
    {
        case RF_SW_CMD_SET_BAND:
            xRficProcessSetBand( pRficDev, pSwCmdDesc );
            break;

        case RF_SW_CMD_ADJUST_PLL_FREQ:
	    xRficProcessAdjustPllFreq( pRficDev, pSwCmdDesc );
            break;

        case RF_SW_CMD_CTRL_GAIN:
            break;

        case RF_SW_CMD_GET_ABS_GAIN:
            break;

        case RF_SW_CMD_REG_READ:
	    xRficProcessReadReg( pRficDev, pSwCmdDesc );
            break;

        case RF_SW_CMD_REG_WRITE:
	    xRficProcessWriteReg( pRficDev, pSwCmdDesc );
            break;

        case RF_SW_CMD_CTRL_VGA_GAIN:
	    xRficProcessCtrlVgaGain( pRficDev, pSwCmdDesc );
            break;

        case RF_SW_CMD_CTRL_DEMOD_GAIN:
	    xRficProcessCtrlDemodGain( pRficDev, pSwCmdDesc );
            break;

        case RF_SW_CMD_CTRL_LNA:
	    xRficProcessCtrlLna( pRficDev, pSwCmdDesc );
            break;

        case RF_SW_CMD_DUMP_IQ_DATA:
            vRficProcessIqDump(pRficDev, pSwCmdDesc);
            break;

        case RF_SW_GET_RX_DC_OFFSET:
            vRficGetRxDcOffset(pSwCmdDesc);
            break;

        case RF_SW_SET_RX_DC_OFFSET:
            vRficSetDcOffset(pSwCmdDesc);
            break;

        case RF_SW_SET_IQ_IMBALANCE:
            vRficSetIqImbalance(pSwCmdDesc);
            break;

        case RF_SW_SET_CHANNEL:
            vRficSetChannel(pSwCmdDesc);
            break;

        case RF_SW_CMD_TX_IQ_DATA:
            vRficProcessTXIqData(pSwCmdDesc);
            break;

        case RF_SW_CMD_FAST_CALIB:
            xRficProcessFastCalib( pRficDev );
            break;

        case RF_SW_CMD_SINGLE_TONE_TX:
            vRficProcessSingleToneTX( pSwCmdDesc );
            break;

        case RF_SW_CMD_SET_LOOPBACK:
            vRficProcessLoopback( pRficDev, pSwCmdDesc );
            pRficDev->xLoopback =
                ( ( struct sw_cmddata_set_loopback * ) pSwCmdDesc->data )->loopback_type;
            break;

        default:
            break;
    }
    return xRet;
}

#ifndef TURN_ON_STANDALONE_MODE
void vRficCoreTask( void * pvParameters )
{
    RficDevice_t *pRficDev = pvParameters;
    rf_sw_cmd_desc_t *pSwCmdDesc;
    EventBits_t uxBits;

    while( 1 )
    {
	uxBits = xEventGroupWaitBits( pRficDev->xSwCmdEvent,
				      RF_CORE_TASK_EVENT_MASK,
				      pdTRUE,  //Clear all the events before returning
				      pdFALSE, //Any event should make this API return
#if defined(RFNM) || defined(SEEVE)
				      1 ); //portMAX_DELAY
#else
				      portMAX_DELAY );
#endif
	if( uxBits & RF_LOCAL_CMD_EVENT )
	{
	    RF_STATS_ADD( pRficDev->pRfHif->rf_stats.local_cmd_count );
	    if( pdPASS == xQueueReceive( pRficDev->xLocalQueue, &pSwCmdDesc, 0 ))
	    {
		pSwCmdDesc->status = RF_SW_CMD_STATUS_IN_PROGRESS;
		xHandleSwCmd( pRficDev, pSwCmdDesc);
		pSwCmdDesc->status = RF_SW_CMD_STATUS_DONE;

		if( pdFAIL == xQueueSend( pRficDev->xLocalRspQueue, &pSwCmdDesc, 0 ))
                {
                    RF_STATS_ADD( pRficDev->pRfHif->rf_stats.local_cmd_failed_count );
                }
	    }
	}
	else if ( uxBits & RF_REMOTE_CMD_EVENT )
	{
	    RF_STATS_ADD( pRficDev->pRfHif->rf_stats.remote_cmd_count );
	    if( pdPASS == xQueueReceive( pRficDev->xRemoteQueue, &pSwCmdDesc, 0 ))
            {
		pSwCmdDesc->status = RF_SW_CMD_STATUS_IN_PROGRESS;
		xHandleSwCmd( pRficDev, pSwCmdDesc);
		pSwCmdDesc->status = RF_SW_CMD_STATUS_DONE;
	    }
	}
	else
	{
#ifndef RFNM
	    log_err( "%s: Invalid com event.\n\r", __func__ );
#else
	    struct avi_hndlr *avihndl = NULL;
	    struct avi_mbox vspa_mbox;

	    avihndl = iLa9310AviHandle();
	    if( NULL != avihndl )
	    {
	            /* Read VSPA inbox 0 */
	        if ( 0 == iLa9310AviHostRecvMboxFromVspa(avihndl, &vspa_mbox, 0 ))
	        {
	            //*((uint32_t *)(&mbox_v2h->status))  = vspa_mbox.lsb;
	            //log_info("\r\n **V2H: MSB_LSB(Hex):%x::%x retries %d\r\n", mbox_v2h->msb32, *((uint32_t *)(&mbox_v2h->status)), retries);

	            if(1 && (vspa_mbox.msb & 0xf0) == 0x80) {
                
	                vRaiseMsi( pLa9310Info, MSI_IRQ_FLOOD_0 );

	                uint32_t *bufferStatusPtr = (uint32_t*) (pRficDev->iq_phys_addr + (1024 * 1024 * 17));
	                *bufferStatusPtr = vspa_mbox.msb;

	                //log_info("set %p off %x to %08x \r\n", bufferStatusPtr, cmd_data->addr, *bufferStatusPtr);
	                //log_info("received vspa interrupt???\r\n" );
	                // vaddr = 0xC0000000;

	            }
	        }
	    } else {
	        log_err( "%s: iLa9310AviHandle error\n\r", __func__ );
	    }
#endif
	}
    }
}
#endif
