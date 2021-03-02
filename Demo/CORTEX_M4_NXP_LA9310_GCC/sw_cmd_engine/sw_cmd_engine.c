/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2021 NXP
 */

#include "FreeRTOS.h"
#include <task.h>
#include <semphr.h>

#include "debug_console.h"
#include "io.h"
#include "la9310_host_if.h"
#include "sync_timing_device.h"

SemaphoreHandle_t xSwCmdSemaphore;

void vSwCmdTask( void * pvParameters )
{
    struct la9310_hif * pxHif = pLa9310Info->pHif;
    struct la9310_sw_cmd_desc * pxCmdDesc = &( pxHif->sw_cmd_desc );

    while( 1 )
    {
        xSemaphoreTake( xSwCmdSemaphore, portMAX_DELAY );

        if( pxCmdDesc->status != LA9310_SW_CMD_STATUS_POSTED )
        {
            log_err( "sw cmd status is not posted\r\n" );
        }

        switch( pxCmdDesc->cmd )
        {
            default:
                log_err( "sw cmd not implemented: %d\r\n", pxCmdDesc->cmd );
                break;
        }

        pxCmdDesc->status = LA9310_SW_CMD_STATUS_DONE;
        dmb();
    }
}

int lSwCmdEngineInit()
{
    BaseType_t xRet;

    xSwCmdSemaphore = xSemaphoreCreateBinary();

    if( xSwCmdSemaphore == NULL )
    {
        log_err( "sw cmd sema create failed\r\n" );
        return -1;
    }

    xRet = xTaskCreate( vSwCmdTask, "sw cmd task", 512, NULL, tskIDLE_PRIORITY + 1, NULL );

    if( xRet != pdPASS )
    {
        log_err( "Failed to create sw cmd task\r\n" );
        return -1;
    }

    NVIC_SetPriority( IRQ_MSG2, 3 );
    NVIC_EnableIRQ( IRQ_MSG2 );

    return 0;
}

void La9310MSG_2_IRQHandler( void )
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    struct la9310_msg_unit * pMsgUnit = &pLa9310Info->msg_unit[ LA9310_MSG_UNIT_2 ];
    uint32_t msir = IN_32( &pMsgUnit->msir );

    ( void ) msir;

    xSemaphoreGiveFromISR( xSwCmdSemaphore, &xHigherPriorityTaskWoken );

    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

    NVIC_ClearPendingIRQ( IRQ_MSG2 );

    #if ARM_ERRATUM_838869
        dsb();
    #endif
}
