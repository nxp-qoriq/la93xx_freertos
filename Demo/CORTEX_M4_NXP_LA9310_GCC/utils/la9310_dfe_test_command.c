/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2024 NXP
 */

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <fsl_dspi.h>
#include "FreeRTOS_CLI.h"
#include "debug_console.h"
#include "la9310_pci.h"
#include "la9310_demo.h"
#include "la9310_main.h"
#include "la9310_gpio.h"
#include "la9310_irq.h"
#include "la9310_avi.h"
#include "exceptions.h"
#include <config.h>
#include "dspi_test.h"
#include "bbdev_ipc.h"
#include <phytimer.h>
#include <delay.h>
#include "../drivers/avi/la9310_vspa_dma.h"
#include "dfe_vspa_mbox.h"
#include "dfe_app.h"
#include "dfe_host_if.h"
#include "rfnm_rf_ctrl.h"
#include "task_stats.h"

uint32_t ulEdmaDemoInfo = 0xaa55aa55;
extern struct la9310_info * pLa9310Info;

#ifndef  configINCLUDE_TRACE_RELATED_CLI_COMMANDS
    #define configINCLUDE_TRACE_RELATED_CLI_COMMANDS    0
#endif
#define MAX_CMD_DESCRIPTION_SIZE                        125

enum eLa9310DfeTestCmdID
{
	TEST_HELP = 0,
	TEST_GPIO = 1,
	TEST_PHYTIMER_COUNTER = 2,
	TEST_BUSYDELAY_ACCURACY = 3,
	TEST_VSPA = 4,
	TEST_PHYTIMER_TDD = 5,
	TEST_PHYTIMER_TDD_STOP = 6,
	TEST_PHYTIMER_FDD = 7,
	TEST_CONFIG_PATTERN = 8,
	TEST_CONFIG_AXIQ_LB = 9,
	TEST_DEBUG = 10,
	TEST_TA = 11,
	TEST_BENCH_VSPA = 12,
	TEST_CONFIG_ANT = 13,
	TEST_CONFIG_BUFFER = 14,
	TEST_CONFIG_QEC = 15,
	TEST_RX_TO_TX = 16,
	TEST_TX_TO_RX = 17,
	TEST_TTI_TRIGGER = 18,
	TEST_TTI_STOP = 19,
	TEST_TASKS = 20,
	MAX_TEST_CMDS
};

static const char cCmdDescriptinArr[ MAX_TEST_CMDS ][ MAX_CMD_DESCRIPTION_SIZE ] =
{
    " invokes help ( dfe help )",
    " To force set PhyTimer rf_ctl( dfe 1 <0:5> <0/1>)",
    " To print phy timer counter every 1s for no of. iter (dfe 2 <iter>)",
    " To test busy delay accuracy using phy timer(dfe 3)",
    " To trigger VPSA debug breakpoint (dfe 4)",
    " To start PhyTimer TDD demo (dfe 5). Check debug var for total ticks",
    " To stop PhyTimer TDD demo (dfe 6).",
    " To set Tx(ch5)/Rx(ch2) Allowed for FDD (dfe 7 0/1)",
    " To config SCS and set TDD pattern to DDDSU, S=6:4:4 (dfe 8 <scs>) ",
    " To set AXIQ loopback (dfe 9 0/1)",
    " To display debug var (dfe 10)",
    " To set TA (for TDD) (dfe 11 <number of phytimer ticks>)",
    " To benchmark VPSA DMA (dfe 12 <size_in_bytes: max 8 KB> <mode: 1-rd, 2-wr> <parallel DMAs: 1,2,4> <number of iterations>)",
    " To config Rx Antenna (dfe 13 <1:4>)",
    " To config Tx/Rx buffers (dfe 14 <rx_addr> <tx_addr> <sym_size/128> <rx_sym_num>, tx_sym_num>)",
    " To config Tx/Rx QEC (dfe 15 <tx/rx> <pt>\r\n\tdfe 15 <tx/rx> <corr> <index> <value>)",
    " Switch Rx->Tx (dfe 16)",
    " Switch Tx->Rx (dfe 17)",
    " TTI trigger (dfe 18)",
    " TTI stop (dfe 19)",
    " Dump Tasks Info (dfe 20)"
};

static portBASE_TYPE prvDFETest( char * pcWriteBuffer,
                                 size_t xWriteBufferLen,
                                 const char * pcCommandString );

static const CLI_Command_Definition_t xDFETestCommand =
{
    "dfe",     /* The command string to type. */
    "\r\ndfe:\r\n dfe Usage : dfe help \r\n",
    prvDFETest, /* The function to run. */
    -1          /* The user can enter any number of commands. */
};

/*-----------------------------------------------------------*/

void vRegisterDFETestCommands( void )
{
    FreeRTOS_CLIRegisterCommand( &xDFETestCommand );
}
/*-----------------------------------------------------------*/

static portBASE_TYPE prvDFETest( char * pcWriteBuffer,
                                 size_t xWriteBufferLen,
                                 const char * pcCommandString )
{
    const char * pcParam1, * pcParam2, * pcParam3, * pcParam4, * pcParam5, * pcParam6;
    BaseType_t lParameterStringLength;
    uint32_t ulCmd = 0;
    uint32_t ulTempVal2 = 0;
    uint32_t ulTempVal3 = 0;
    uint32_t ulTempVal4 = 0;
    uint32_t ulTempVal5 = 0;
    uint32_t ulTempVal6 = 0;
    uint32_t i;
	float fTempVal = 0;

    ( void ) pcCommandString;
    ( void ) xWriteBufferLen;
    configASSERT( pcWriteBuffer );

    pcParam1 = FreeRTOS_CLIGetParameter( pcCommandString, 1, &lParameterStringLength );
    ulCmd = strtoul( pcParam1, ( char ** ) NULL, 10 );

	switch( ulCmd )
	{
		case TEST_PHYTIMER_COUNTER:
			pcParam2 = FreeRTOS_CLIGetParameter( pcCommandString, 2, &lParameterStringLength );
			ulTempVal2 = strtoul( pcParam2, ( char ** ) NULL, 10 );
			for( i = 0; i < ulTempVal2; i++ )
			{
				ulTempVal3 = ulPhyTimerCapture( PHY_TIMER_COMP_PPS_OUT );
				PRINTF( "Phy timer counter: 0x%x\r\n", ulTempVal3 );
				vTaskDelay( 1000 );
			}
			break;

		case TEST_BUSYDELAY_ACCURACY:
			ulTempVal3 = ulPhyTimerCapture( PHY_TIMER_COMP_PA_EN );
			vUDelay( 1000 );
			ulTempVal4 = ulPhyTimerDiffToUS( ulTempVal3, ulPhyTimerCapture( PHY_TIMER_COMP_PA_EN ) );
			PRINTF( "Phy timer Us passed: %d for vUDelay(%d)\r\n", ulTempVal4, 1000 );
			break;
		case TEST_GPIO:
			pcParam2 = FreeRTOS_CLIGetParameter( pcCommandString, 2, &lParameterStringLength );
			ulTempVal2 = strtoul( pcParam2, ( char ** ) NULL, 10 );
			pcParam3 = FreeRTOS_CLIGetParameter( pcCommandString, 3, &lParameterStringLength );
			ulTempVal3 = strtoul( pcParam3, ( char ** ) NULL, 10 );
			PRINTF( "RFCTL PhyTimer Force set rfctl[%d] to %d:\r\n\r\n", ulTempVal2, ulTempVal3 );
			if (ulTempVal3 != 0)
				ulTempVal3 = ePhyTimerComparatorOut1;

			if ((ulTempVal2<0) || (ulTempVal2>5)) {
				PRINTF("wrong param\r\n");
				break;
			}

			vPhyTimerComparatorForce(PHY_TIMER_COMP_RFCTL_0 + ulTempVal2 , ulTempVal3);
			break;

		case TEST_DEBUG:
			PRINTF("ulVspaMsgCnt = %d\r\n", ulVspaMsgCnt);
			PRINTF("ulTotalTicks = %d\r\n", ulTotalTicks);
 			int status = ulPhyTimerComparatorGetStatus(uRxAntennaComparator);
 			PRINTF("PHY_TIMER_COMP_RX_ALLOWED: %#x\r\n", status);
 			status = ulPhyTimerComparatorGetStatus(uTxAntennaComparator);
 			PRINTF("PHY_TIMER_COMP_TX_ALLOWED: %#x\r\n", status);

			dump_slots();
			vTraceEventShow();
			break;

		case TEST_VSPA:
			vVSPADebugBreakPoint();
			break;

		case TEST_PHYTIMER_TDD:
            iTddStart();
			break;

		case TEST_PHYTIMER_TDD_STOP:
            vTddStop();
			break;
		case TEST_PHYTIMER_FDD:
			pcParam2 = FreeRTOS_CLIGetParameter( pcCommandString, 2, &lParameterStringLength );
			ulTempVal2 = strtoul( pcParam2, ( char ** ) NULL, 10 );
			PRINTF("Set TxAllowed(CH5), RxAllowed(CH2) to ");
			ulTempVal3 = uGetPhyTimerTimestamp() + 0x96000;
			if (ulTempVal2 != 0) {
				ulTempVal2 = ePhyTimerComparatorOut1;
				PRINTF("1\r\n");

				vConfigFddStart(ulTempVal3);
			}
			else {
				ulTempVal2 = ePhyTimerComparatorOut0;
				PRINTF("0\r\n");
				vConfigFddStop();
			}

			/* delay of 2 symbols */
			vPhyTimerDelay( 2 * ofdm_short_sym_time[SCS_kHz30]);

			vPhyTimerComparatorConfig( uTxAntennaComparator,
				PHY_TIMER_COMPARATOR_CLEAR_INT | PHY_TIMER_COMPARATOR_CROSS_TRIG,
				ulTempVal2,
				ulTempVal3);

			vPhyTimerComparatorConfig( uRxAntennaComparator,
				PHY_TIMER_COMPARATOR_CLEAR_INT | PHY_TIMER_COMPARATOR_CROSS_TRIG,
				ulTempVal2,
				ulTempVal3);
			break;
		case TEST_CONFIG_PATTERN:
			pcParam2 = FreeRTOS_CLIGetParameter( pcCommandString, 2, &lParameterStringLength );
			ulTempVal2 = strtoul( pcParam2, ( char ** ) NULL, 10 );

			vSetupTddPattern((ulTempVal2 == 15) ? SCS_kHz15 : SCS_kHz30);
			break;

		case TEST_CONFIG_AXIQ_LB:
			pcParam2 = FreeRTOS_CLIGetParameter( pcCommandString, 2, &lParameterStringLength );
			ulTempVal2 = strtoul( pcParam2, ( char ** ) NULL, 10 );
			vAxiqLoopbackSet(ulTempVal2, SET_AXIQ_LOOPBACK_MASK_ALL);
			PRINTF("Setting AXIQ Loopback for CH2(RX1) to %s\r\n", ulTempVal2 ? "Enabled" : "Disabled");
			break;

		case TEST_TA:
			pcParam2 = FreeRTOS_CLIGetParameter( pcCommandString, 2, &lParameterStringLength );
			ulTempVal2 = strtoul( pcParam2, ( char ** ) NULL, 10 );
			vSetTimeAdvance(ulTempVal2);
			break;

		case TEST_BENCH_VSPA:
			pcParam2 = FreeRTOS_CLIGetParameter( pcCommandString, 2, &lParameterStringLength );
			ulTempVal2 = strtoul( pcParam2, ( char ** ) NULL, 10 );
			pcParam3 = FreeRTOS_CLIGetParameter( pcCommandString, 3, &lParameterStringLength );
			ulTempVal3 = strtoul( pcParam3, ( char ** ) NULL, 10 );
			pcParam4 = FreeRTOS_CLIGetParameter( pcCommandString, 4, &lParameterStringLength );
			ulTempVal4 = strtoul( pcParam4, ( char ** ) NULL, 10 );
			pcParam5 = FreeRTOS_CLIGetParameter( pcCommandString, 5, &lParameterStringLength );
			ulTempVal5 = strtoul( pcParam5, ( char ** ) NULL, 10 );
			uint32_t gbits, mbits, vspa_cycles;
			vspa_cycles =  iBenchmarkVspaTest(ulTempVal2, ulTempVal3, ulTempVal4, ulTempVal5, &gbits, &mbits);
			PRINTF("Benchmark result = %d.%d Gb/s (vspa_cycles = %d), %d DMAs in parallel, %d iterations\r\n",
				   gbits, mbits, vspa_cycles, ulTempVal4, ulTempVal5);
			break;

		case TEST_CONFIG_ANT:
			pcParam2 = FreeRTOS_CLIGetParameter( pcCommandString, 2, &lParameterStringLength );
			ulTempVal2 = strtoul( pcParam2, ( char ** ) NULL, 10 );
			iSetRxAntenna(ulTempVal2);
			break;

		case TEST_CONFIG_BUFFER:
			pcParam2 = FreeRTOS_CLIGetParameter( pcCommandString, 2, &lParameterStringLength );
			ulTempVal2 = strtoul( pcParam2, ( char ** ) NULL, 16 );
			pcParam3 = FreeRTOS_CLIGetParameter( pcCommandString, 3, &lParameterStringLength );
			ulTempVal3 = strtoul( pcParam3, ( char ** ) NULL, 16 );
			pcParam4 = FreeRTOS_CLIGetParameter( pcCommandString, 4, &lParameterStringLength );
			ulTempVal4 = strtoul( pcParam4, ( char ** ) NULL, 10 );
			pcParam5 = FreeRTOS_CLIGetParameter( pcCommandString, 5, &lParameterStringLength );
			ulTempVal5 = strtoul( pcParam5, ( char ** ) NULL, 10 );
			pcParam6 = FreeRTOS_CLIGetParameter( pcCommandString, 6, &lParameterStringLength );
			ulTempVal6 = strtoul( pcParam6, ( char ** ) NULL, 10 );

			vSetTxRxBuffers(ulTempVal2, ulTempVal3, ulTempVal4, ulTempVal5, ulTempVal6);
			break;

		case TEST_CONFIG_QEC:
		{
			/* tx/rx*/
			pcParam2 = FreeRTOS_CLIGetParameter( pcCommandString, 2, &lParameterStringLength );
			ulTempVal2 = (!strncmp(pcParam2, "tx", 2)) ? QEC_TX_CORR : QEC_RX_CORR;

			/* mode */
			pcParam3 = FreeRTOS_CLIGetParameter( pcCommandString, 3, &lParameterStringLength );
			ulTempVal3 = (!strncmp(pcParam3, "pt", 2)) ? QEC_IQ_DC_OFFSET_PASSTHROUGH : QEC_IQ_DC_OFFSET_CORR;

			if (ulTempVal3 == QEC_IQ_DC_OFFSET_PASSTHROUGH) {
				vSetQecParam(ulTempVal2, ulTempVal3, 0, 0);
				break;
			}

			/* idx */
			pcParam4 = FreeRTOS_CLIGetParameter( pcCommandString, 4, &lParameterStringLength );
			ulTempVal4 = strtoul( pcParam4, ( char ** ) NULL, 10 );

			/* value - avoid using strtof causing 5K overlow in text */
			pcParam5 = FreeRTOS_CLIGetParameter( pcCommandString, 5, &lParameterStringLength );
			if (ulTempVal4 != MBOX_IQ_CORR_FDELAY) {
				char *del = strtok((char *)pcParam5, ".");
				fTempVal = strtoul(del, ( char ** ) NULL, 10 );
				del = strtok (NULL, ".");
				if (del) {
					fTempVal += (float) (strtoul(del, ( char ** ) NULL, 10 )) / (10*strlen(del));
				}
				ulTempVal5 = *(uint32_t *) &fTempVal;
			} else {
				ulTempVal5 = strtoul( pcParam5, ( char ** ) NULL, 10 );
			}

			vSetQecParam(ulTempVal2, ulTempVal3, ulTempVal4, ulTempVal5);
			break;
		}

		case TEST_RX_TO_TX:
			switch_rf(0xAAAAAAAA);
			break;
		case TEST_TX_TO_RX:
			switch_rf(0xBBBBBBBB);
			break;
		case TEST_TTI_TRIGGER:
			tti_trigger(ulPhyTimerCapture( PHY_TIMER_COMPARATOR_COUNT - 1 ) + 10000, 0x7800);
			break;
		case TEST_TTI_STOP:
			PRINTF("tti stop\r\n");
			tti_stop();
			break;
		case TEST_TASKS:
			vStatsUsageCommand();
			break;
		default:

			for( ulCmd = 0; ulCmd < MAX_TEST_CMDS; ulCmd++ )
			{
				PRINTF( "[%2d] %s\r\n", ulCmd, cCmdDescriptinArr[ ulCmd ] );
			}

			break;
	}

    pcWriteBuffer[ 0 ] = 0;

    return pdFALSE;
}
/*-----------------------------------------------------------*/
