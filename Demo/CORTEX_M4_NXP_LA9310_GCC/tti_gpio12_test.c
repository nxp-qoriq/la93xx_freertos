/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2024 NXP
 */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "common.h"
#include "immap.h"
#include "la9310_main.h"
#include "la9310_irq.h"
#include "core_cm4.h"
#include "la9310_edmaAPI.h"
#include "bbdev_ipc.h"
#include "rfic_api.h"
#include "sync_timing_device.h"
#include "math.h"
#include "rfic_api.h"
#include "phytimer.h"
#include <time.h>
#include "rfnm_rf_ctrl.h"

uint32_t ulLastPpsInTimestamp = 0;
static uint32_t ulNextTick;
uint32_t ulTotalTicks = 0;

static uint32_t tick_interval = 30720;      /* 61.44MHz * 500us */

bool stop = pdTRUE;

static void prvTick( void *pvParameters, long unsigned int param1);

void vPhyTimerTickConfig()
{
	uint32_t debug_ts = 0;
	NVIC_SetPriority( IRQ_PPS_OUT, 1 );
	NVIC_EnableIRQ( IRQ_PPS_OUT );

	ulNextTick = ulPhyTimerCapture( PHY_TIMER_COMP_PPS_OUT );
	ulNextTick += 0xF0000;
	ulNextTick -= tick_interval;

	vPhyTimerComparatorConfig( PHY_TIMER_COMP_PPS_OUT,
			PHY_TIMER_COMPARATOR_CLEAR_INT | PHY_TIMER_COMPARATOR_CROSS_TRIG,
			ePhyTimerComparatorOutToggle,
			ulNextTick );

	vPhyTimerComparatorConfig( PHY_TIMER_COMP_RFCTL_5,
			PHY_TIMER_COMPARATOR_CLEAR_INT | PHY_TIMER_COMPARATOR_CROSS_TRIG,
			ePhyTimerComparatorOutToggle,
			ulNextTick );

	debug_ts =  ulPhyTimerCapture( PHY_TIMER_COMPARATOR_COUNT - 1 );
	PRINTF("       now = %#x\r\n" \
	       "ulNextTick = %#x\r\n\r\n",
	       debug_ts,
	       ulNextTick);
}

void vPhyTimerPPSINHandler()
{
	NVIC_ClearPendingIRQ( IRQ_PPS_IN );
	ulLastPpsInTimestamp = ulPhyTimerComparatorRead( PHY_TIMER_COMP_PPS_IN );
	NVIC_DisableIRQ( IRQ_PPS_IN );
}

void vPhyTimerPPSOUTHandler()
{
	NVIC_ClearPendingIRQ( IRQ_PPS_OUT );

	/* Update next tick's timestamp */
	ulNextTick += tick_interval;

	vPhyTimerComparatorConfig( PHY_TIMER_COMP_PPS_OUT,
				PHY_TIMER_COMPARATOR_CLEAR_INT | PHY_TIMER_COMPARATOR_CROSS_TRIG,
				ePhyTimerComparatorOutToggle,
				ulNextTick );

	/* Call the tick callback */
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xTimerPendFunctionCallFromISR( prvTick, NULL, 0, &xHigherPriorityTaskWoken );
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

static void prvTick( void *pvParameters, long unsigned int param1 )
{

	if ( stop ) {
		/* disable tick and any other used comparator */
		vPhyTimerComparatorDisable( PHY_TIMER_COMP_PPS_OUT );
		vPhyTimerComparatorDisable( PHY_TIMER_COMP_RFCTL_5 );
		return;
	}

	vPhyTimerComparatorConfig( PHY_TIMER_COMP_RFCTL_5,
			PHY_TIMER_COMPARATOR_CLEAR_INT | PHY_TIMER_COMPARATOR_CROSS_TRIG,
			ePhyTimerComparatorOutToggle,
			ulNextTick );

	ulTotalTicks++;
}

void prvConfigTickMachine()
{
	/* reset all counters for easy debugging */
	ulTotalTicks = 0;

	/* configure tick interval and setup PhyTimer */
	vPhyTimerTickConfig();
}

void vTickMachineStart(uint32_t period)
{
	tick_interval = period;

	/* check if tick machine is running */
	if (!stop && ulTotalTicks) {
		PRINTF("Tick machine already running!\r\n");
		return;
	}

	stop = pdFALSE;

	prvConfigTickMachine();
	return;
}

void vTickMachineStop(void)
{
	stop = pdTRUE;
	vTaskDelay(1000);
	PRINTF("ulTotalTicks = %d\r\n", ulTotalTicks);
}

void vApplicationMallocFailedHook()
{
	PRINTF("\r\nMemory allocation failed!\r\n");
	for(;;);
}

void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName )
{
	PRINTF("\r\nMemory allocation failed!\r\n");
	for(;;);
}
