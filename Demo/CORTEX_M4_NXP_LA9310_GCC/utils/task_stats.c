/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2024 NXP
 */
 
#include "FreeRTOS.h"
#include "FreeRTOS_CLI.h"
#include "debug_console.h"
#include "task.h"
#include "timers.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

extern uint32_t uiTaskStackSize( TaskHandle_t *pxTaskHandle );

void vStatsUsageCommand( void )
{
	TaskStatus_t *pxTaskStatusArray;
	UBaseType_t uxArraySize = uxTaskGetNumberOfTasks();
	UBaseType_t x = 0;
	uint32_t ulTotalTime = 0;
	TaskHandle_t *xHandle = NULL;
	uint32_t uiStackSize = 0;
	eTaskState eTaskState = eInvalid;
	char * pcTaskState = NULL;

	pxTaskStatusArray = pvPortMalloc( uxArraySize * sizeof( TaskStatus_t ) );

	if( pxTaskStatusArray != NULL )
	{
		/* Generate the (binary) data. */
		uxArraySize = uxTaskGetSystemState( pxTaskStatusArray, uxArraySize, &ulTotalTime );

		PRINTF("\n\r-----------------------------------------------------------------------");
		PRINTF("\n\rTask Name\tBase Addr     Stack Size\tRemaining\tState");
		PRINTF("\n\r-----------------------------------------------------------------------\n\r");

		/* Create a human readable table from the binary data. */
		for( x = 0; x < uxArraySize; x++ )
		{
			xHandle = (TaskHandle_t *) pxTaskStatusArray[ x ].xHandle;
			uiStackSize = uiTaskStackSize( xHandle );

			eTaskState = pxTaskStatusArray[ x ].eCurrentState;
			switch( eTaskState )
			{
				case eRunning:
					pcTaskState = "RUNNING";
					break;
				case eReady:
					pcTaskState = "READY";
					break;
				case eBlocked:
					pcTaskState = "BLOCKED";
					break;
				case eSuspended:
					pcTaskState = "SUSPENDED";
					break;
				case eDeleted:
					pcTaskState = "DELETED";
					break;
				case eInvalid:
				default:
					pcTaskState = "INVALID";
					break;
			}

			PRINTF("%-15s 0x%p           %-4d\t%-4d\t\t%s\n\r",
				pxTaskStatusArray[ x ].pcTaskName,
				pxTaskStatusArray[ x ].pxStackBase,
				uiStackSize,
				pxTaskStatusArray[ x ].usStackHighWaterMark,
				pcTaskState );
		}
		PRINTF("-----------------------------------------------------------------------\n\r");
		vPortFree(pxTaskStatusArray);
	}
}
