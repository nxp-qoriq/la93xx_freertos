/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2021-2024 NXP
 */

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Standard includes. */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <fsl_dspi.h>
/* FreeRTOS+CLI includes. */
#include "FreeRTOS_CLI.h"
#include "debug_console.h"
#include "la9310_demo.h"
#include "la9310_main.h"
#include "exceptions.h"
#include "la9310_pci.h"

#include <config.h>
#include "dspi_test.h"
#include "la9310_gpio.h"
#include "la9310_irq.h"
#include "bbdev_ipc.h"
#include <phytimer.h>
#include <delay.h>
#ifdef __RFIC
    #include "rfic_api.h"
#endif
#include "../drivers/avi/la9310_vspa_dma.h"

extern void switch_rf(uint32_t mode);


uint32_t ulEdmaDemoInfo = 0xaa55aa55;
extern struct la9310_info * pLa9310Info;

#ifndef  configINCLUDE_TRACE_RELATED_CLI_COMMANDS
    #define configINCLUDE_TRACE_RELATED_CLI_COMMANDS    0
#endif
#define MAX_CMD_DESCRIPTION_SIZE                        125
enum eLa9310TestCmdID
{
    TEST_HELP = 0,
    TEST_GPIO = 1,
    TEST_eDMA = 2,
    TEST_I2C = 3,
    TEST_AVI = 4,
    TEST_EXCEPTIONS = 5,
    TEST_DSPI = 6,
    TEST_WDOG = 7,
    TEST_PCI_DUMP = 8,
    TEST_BBDEV_IPC_RAW = 9,
    TEST_MSI = 10,
    TEST_PHYTIMER_COUNTER = 11,
    TEST_TICK_SECONDS = 12,
    TEST_RFIC = 13,
    TEST_PPS_INT_PHY_COMP = 14,
    TEST_BUSYDELAY_ACCURACY =15,
    TEST_UNSUPPORTED =16,
    TEST_OVERLAY =17,
    TEST_VSPA_TABLES =18,
#ifdef TURN_ON_STANDALONE_MODE
    TEST_CRC=19,
#endif
#if defined (RFNM) || defined (SEEVE)
    TEST_RX_TO_TX = 20,
    TEST_TX_TO_RX = 21,
    TEST_TTI_START = 22,
    TEST_TTI_STOP = 23,
#endif
    MAX_TEST_CMDS
};

#ifdef __RFIC
    enum eRficApis
    {
        RFIC_SETBAND,
        RFIC_ADJPLLFREQ,
        RFIC_LNA_CTRL,
        RFIC_DEMOD_GAIN,
        RFIC_VGA_GAIN,
        RFIC_FAST_CAL,
        RFIC_IQ_DUMP,
        RFIC_GET_VALUES
    };
#endif

#ifdef RFNM
extern void vTickMachineStart(uint32_t period);
extern void vTickMachineStop();
#endif

static const char cCmdDescriptinArr[ MAX_TEST_CMDS ][ MAX_CMD_DESCRIPTION_SIZE ] =
{
    " invokes help ( test help )",
#ifdef TURN_ON_HOST_MODE
    " To test GPIO( test 1 pin_num dir high_low)",
#endif
    " To test eDMA ( test 2)",
    " To test I2C (test 3  i2c_ctrl rw add reg_num tx_len num_bye bytesWr)",
    " To test AVI ( test 4 num_iterration )",
    " To test EXCEPTION ( test 5)",
    " To test DSPI ( test 6 <testid - 0/1/2 (LTC5586/(NLMv1-LMX2582, NLMv2-MAX2870/ADRF6520) > <mode - 0/1 (GPIO/PA_EN)>)",
    " To test Watch Dog (test 7)",
#ifdef TURN_ON_HOST_MODE
    " To PCI REG DUMP (test 8)",
    " To test BBDEV IPC RAW ops validation (test 9 <mode - latency/validation>)",
    " To raise MSI (test 10 <msi number>)",
#endif    
    " To print phy timer counter every 1s for no of. iter (test 11 <iter>)",
#ifdef __RFIC
    " To test RFIC (test 13 <cmd_id> <cmd_parameter>)",
#endif
    " To enable/disable phy timer pps_in interrupt (test 14 0/1)",
    " To test busy delay accuracy using phy timer(test 15)",
#ifdef TURN_ON_STANDALONE_MODE
    " To load vspa overlay (test 17 ovl_num (1/2 (.overlay_1/.overlay_2)",
    " To verify vspa table (test 18)",
    " To verify CRC of images (test 19)",
#endif
#ifdef RFNM
    " Switch Rx->Tx (test 20)",
    " Switch Tx->Rx (test 21)",
    " TTI start (test 22)",
    " TTI stop (test 23)"
#endif
};


static portBASE_TYPE prvNLMTest( char * pcWriteBuffer,
                                 size_t xWriteBufferLen,
                                 const char * pcCommandString );

static const CLI_Command_Definition_t xNLMTestCommand =
{
    "test",     /* The command string to type. */
    "\r\ntest:\r\n test Usage : test help \r\n",
    prvNLMTest, /* The function to run. */
    -1          /* The user can enter any number of commands. */
};

/*-----------------------------------------------------------*/

void vRegisterNLMTestCommands( void )
{
    FreeRTOS_CLIRegisterCommand( &xNLMTestCommand );
}
/*-----------------------------------------------------------*/

static portBASE_TYPE prvNLMTest( char * pcWriteBuffer,
                                 size_t xWriteBufferLen,
                                 const char * pcCommandString )
{
    const char * pcParam1, * pcParam2, * pcParam3, * pcParam4, * pcParam5, * pcParam6, * pcParam7, * pcParam8;
    BaseType_t lParameterStringLength;
    uint32_t ulCmd = 0;
    uint32_t ulTempVal2 = 0;
    uint32_t ulTempVal3 = 0;
    uint32_t ulTempVal4 = 0;
    uint32_t ulTempVal5 = 0;
    uint32_t ulTempVal6 = 0;
    uint32_t ulTempVal7 = 0;
    uint32_t ulTempVal8 = 0;
    uint32_t i;
#ifdef __RFIC
    uint32_t starttime, endtime;
#endif
    /* Remove compile time warnings about unused parameters, and check the
     * write buffer is not NULL.  NOTE - for simplicity, this example assumes the
     * write buffer length is adequate, so does not check for buffer overflows. */
    ( void ) pcCommandString;
    ( void ) xWriteBufferLen;
    configASSERT( pcWriteBuffer );

    pcParam1 = FreeRTOS_CLIGetParameter( pcCommandString, 1, &lParameterStringLength );
    ulCmd = strtoul( pcParam1, ( char ** ) NULL, 10 );

	switch( ulCmd )
	{
		case TEST_PPS_INT_PHY_COMP:
			pcParam2 = FreeRTOS_CLIGetParameter( pcCommandString, 2, &lParameterStringLength );
			ulTempVal2 = strtoul( pcParam2, ( char ** ) NULL, 10 );
			if( ulTempVal2 )
			{
				vPhyTimerPPSINEnable();
			}
			else
			{
				vPhyTimerPPSINDisable();
			}
			break;

		case TEST_PHYTIMER_COUNTER:
			pcParam2 = FreeRTOS_CLIGetParameter( pcCommandString, 2, &lParameterStringLength );
			ulTempVal2 = strtoul( pcParam2, ( char ** ) NULL, 10 );
			for( i = 0; i < ulTempVal2; i++ )
			{
				ulTempVal3 = ulPhyTimerCapture( PHY_TIMER_COMP_PA_EN );
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

		case TEST_TICK_SECONDS:
			pcParam2 = FreeRTOS_CLIGetParameter( pcCommandString, 2, &lParameterStringLength );
			ulTempVal2 = strtoul( pcParam2, ( char ** ) NULL, 10 );
			pcParam3 = FreeRTOS_CLIGetParameter( pcCommandString, 3, &lParameterStringLength );
			ulTempVal3 = strtoul( pcParam3, ( char ** ) NULL, 10 );

			for( i = 0; i < ulTempVal2; i++ )
			{
				PRINTF( "Timer running\r\n" );
				vTaskDelay( ulTempVal3 * configTICK_RATE_HZ );
			}

			break;

		case TEST_MSI:
			pcParam2 = FreeRTOS_CLIGetParameter( pcCommandString, 2, &lParameterStringLength );
			ulTempVal2 = strtoul( pcParam2, ( char ** ) NULL, 10 );
			vRaiseMsi( pLa9310Info, ( int ) ulTempVal2 );
			break;

		#ifdef __RFIC
			case TEST_RFIC:
				pcParam2 = FreeRTOS_CLIGetParameter( pcCommandString, 2, &lParameterStringLength );
				ulTempVal2 = strtoul( pcParam2, ( char ** ) NULL, 10 );
				pcParam3 = FreeRTOS_CLIGetParameter( pcCommandString, 3, &lParameterStringLength );
				ulTempVal3 = strtoul( pcParam3, ( char ** ) NULL, 10 );

				RficHandle_t pRficDev;
				pRficDev = xRficLibInit();
				if(pRficDev == NULL)
				{
					log_err( "%s: Handle not found \n\r", __func__ );
					break;
				}

				RficResp_t ret;
				switch( ulTempVal2 )
				{

					case RFIC_SETBAND:
						starttime = ulPhyTimerCapture( 11 );
						ret = xRficSetBand(pRficDev, ulTempVal3);
						endtime = ulPhyTimerCapture( 11 );
						PRINTF( "ST[%d], ET[%d], LT[%d]\r\n", starttime, endtime,
								endtime-starttime);
						if(ret != RFIC_SUCCESS)
							log_err( "%s: SetBand Failed ,ret  %d\n\r", __func__, ret );
						break;

					case RFIC_ADJPLLFREQ:
						starttime = ulPhyTimerCapture( 11 );
						ret = xRficAdjustPllFreq(pRficDev, ulTempVal3);
						endtime = ulPhyTimerCapture( 11 );
						PRINTF( "ST[%d], ET[%d], LT[%d]\r\n", starttime, endtime,
								endtime-starttime);
						if(ret != RFIC_SUCCESS)
							log_err( "%s: AdjPllFreq Failed ,ret  %d\n\r", __func__, ret );
						break;

					case RFIC_LNA_CTRL:
						starttime = ulPhyTimerCapture( 11 );
						ret = xRficLnaCtrl(pRficDev, ulTempVal3);
						endtime = ulPhyTimerCapture( 11 );
						PRINTF( "ST[%d], ET[%d], LT[%d]\r\n", starttime, endtime,
								endtime-starttime);
						if(ret != RFIC_SUCCESS)
							log_err( "%s: LNA Ctrl Failed ,ret  %d\n\r", __func__, ret );
						break;

					case RFIC_DEMOD_GAIN:
						pcParam4 = FreeRTOS_CLIGetParameter( pcCommandString, 4, &lParameterStringLength );
						ulTempVal4 = strtoul( pcParam4, ( char ** ) NULL, 10 );
						starttime = ulPhyTimerCapture( 11 );
						ret = xRficDemodGainCtrl(pRficDev, ulTempVal3, ulTempVal4);
						endtime = ulPhyTimerCapture( 11 );
						PRINTF( "ST[%d], ET[%d], LT[%d]\r\n", starttime, endtime,
								endtime-starttime);
						if(ret != RFIC_SUCCESS)
							log_err( "%s: Demod Gain Failed ,ret  %d\n\r", __func__, ret );

						break;

					case RFIC_VGA_GAIN:
						starttime = ulPhyTimerCapture( 11 );
						ret = xRficAdrfGainCtrl(pRficDev, ulTempVal3);
						endtime = ulPhyTimerCapture( 11 );
						PRINTF( "ST[%d], ET[%d], LT[%d]\r\n", starttime, endtime,
								endtime-starttime);
						if(ret != RFIC_SUCCESS)
							log_err( "%s: ADRF Gain Failed ,ret  %d\n\r", __func__, ret );
						break;

                    case RFIC_FAST_CAL:
                        starttime = ulPhyTimerCapture( 11 );
                        ret = xRficFastCal( pRficDev );
                        endtime = ulPhyTimerCapture( 11 );
                        PRINTF( "ST[%d], ET[%d], LT[%d]\r\n", starttime, endtime,
                                endtime-starttime);
                        if(ret != RFIC_SUCCESS)
                            log_err( "%s: Fast cal API Failed ,ret  %d\n\r", __func__, ret );
                        break;

                    case RFIC_IQ_DUMP:
                        if (ulTempVal3 < 1 || ulTempVal3 > 10)
                        {
                            log_err( "%s: Number of 4KB blocks should be between 1 to 10 \n\r", __func__);
                            ret = RFIC_PARAM_INVALID;
                        }
                        else
                        {
                            ret = xRficIQDump( pRficDev, ulTempVal3 );
                        }
                        if(ret != RFIC_SUCCESS)
                            log_err( "%s: IQ Dump Failed ,ret  %d\n\r", __func__, ret );
                        break;
                    case RFIC_GET_VALUES:
                        xRficGetRFConf( pRficDev );
                        break;

					default:
						log_err("%s: Invalid Command Id - %d\n\r", __func__, ulTempVal2 );
						break;
				}
				break;
		#endif

		case TEST_WDOG:
			vWdogDemo( pLa9310Info );
			break;

		case TEST_GPIO:
			pcParam2 = FreeRTOS_CLIGetParameter( pcCommandString, 2, &lParameterStringLength );
			ulTempVal2 = strtoul( pcParam2, ( char ** ) NULL, 10 );
			pcParam3 = FreeRTOS_CLIGetParameter( pcCommandString, 3, &lParameterStringLength );
			ulTempVal3 = strtoul( pcParam3, ( char ** ) NULL, 10 );
			pcParam4 = FreeRTOS_CLIGetParameter( pcCommandString, 4, &lParameterStringLength );
			ulTempVal4 = strtoul( pcParam4, ( char ** ) NULL, 10 );
			log_info( "GPIO Test GpioNo %d Dir %d Val %d:\r\n\r\n", ulTempVal2, ulTempVal3, ulTempVal4 );
			vLa9310GpioTest( ulTempVal2, ulTempVal3, ulTempVal4 );
			break;

		case TEST_eDMA:
			vLa9310EdmaDemo( &ulEdmaDemoInfo );
			break;

		case TEST_I2C:
			pcParam2 = FreeRTOS_CLIGetParameter( pcCommandString, 2, &lParameterStringLength );
			ulTempVal2 = strtoul( pcParam2, ( char ** ) NULL, 10 );

			pcParam3 = FreeRTOS_CLIGetParameter( pcCommandString, 3, &lParameterStringLength );
			ulTempVal3 = strtoul( pcParam3, ( char ** ) NULL, 10 );

			pcParam4 = FreeRTOS_CLIGetParameter( pcCommandString, 4, &lParameterStringLength );
			ulTempVal4 = strtoul( pcParam4, ( char ** ) NULL, 16 );

			pcParam5 = FreeRTOS_CLIGetParameter( pcCommandString, 5, &lParameterStringLength );
			ulTempVal5 = strtoul( pcParam5, ( char ** ) NULL, 16 );

			pcParam6 = FreeRTOS_CLIGetParameter( pcCommandString, 6, &lParameterStringLength );
			ulTempVal6 = strtoul( pcParam6, ( char ** ) NULL, 10 );

			pcParam7 = FreeRTOS_CLIGetParameter( pcCommandString, 7, &lParameterStringLength );
			ulTempVal7 = strtoul( pcParam7, ( char ** ) NULL, 10 );

			pcParam8 = FreeRTOS_CLIGetParameter( pcCommandString, 8, &lParameterStringLength );
			ulTempVal8 = strtoul( pcParam8, ( char ** ) NULL, 16 );

			vLa9310I2CTest( ulTempVal2, ulTempVal3, ulTempVal4, ulTempVal5, ulTempVal6, ulTempVal7, ulTempVal8 );
			break;

		case TEST_DSPI:
			pcParam2 = FreeRTOS_CLIGetParameter( pcCommandString, 2, &lParameterStringLength );
			ulTempVal2 = strtoul( pcParam2, ( char ** ) NULL, 10 );
			pcParam3 = FreeRTOS_CLIGetParameter( pcCommandString, 3, &lParameterStringLength );
			ulTempVal3 = strtoul( pcParam3, ( char ** ) NULL, 10 );
			vDspiTest( ulTempVal2, ulTempVal3 );
			break;

		case TEST_AVI:
			pcParam2 = FreeRTOS_CLIGetParameter( pcCommandString, 2, &lParameterStringLength );
			ulTempVal2 = strtoul( pcParam2, ( char ** ) NULL, 10 );
			vAVIDemo( ulTempVal2 );

			break;

		case TEST_EXCEPTIONS:
			vGenerateExceptions( UNALIGNED_ACCESS );
			break;
		#ifdef TURN_ON_HOST_MODE
		case TEST_PCI_DUMP:
			vLA9310DumpPCIeRegs();
			break;
		case TEST_BBDEV_IPC_RAW:
			pcParam2 = FreeRTOS_CLIGetParameter( pcCommandString, 2, &lParameterStringLength );

			if (!strcmp(pcParam2, "validation"))
				vLa9310DemoIpcRawTest(0);
			else if (!strcmp(pcParam2, "latency"))
				vLa9310DemoIpcRawTest(1);
			else
				log_info("Invalid mode %s\n\r", pcParam2);

			break;
		#endif

		#ifdef TURN_ON_STANDALONE_MODE
		case TEST_OVERLAY:
			pcParam2 = FreeRTOS_CLIGetParameter( pcCommandString, 2, &lParameterStringLength );
			ulTempVal2 = strtoul( pcParam2, ( char ** ) NULL, 10 );
			switch(ulTempVal2)
			{
				case 1:
					iLoadOverlay(".overlay_1", 1);
					log_info("Overlay .overlay_1 loaded successfully\r\n");
					break;
				case 2:
					iLoadOverlay(".overlay_2", 1);
					log_info(".overlay_2 loaded successfully\r\n");
					break;
				default:
					log_info("Invalid overlay number..\r\n");
					break;
			}
			break;
		case TEST_VSPA_TABLES:
			iVerifyVSPATable();
			break;
		case TEST_CRC:
			vLa9310VerifyCRC();
			break;
		#endif //TURN_ON_STANDALONE_MODE
		#ifdef RFNM
		case TEST_RX_TO_TX:
			switch_rf(0xAAAAAAAA);
			break;
		case TEST_TX_TO_RX:
			switch_rf(0xBBBBBBBB);
			break;
		case TEST_TTI_START:
			vTickMachineStart(0xF000);
			break;
		case TEST_TTI_STOP:
			vTickMachineStop();
			break;
		#endif
		default:

			for( ulCmd = 0; ulCmd < MAX_TEST_CMDS; ulCmd++ )
			{
				log_info( "%s \r\n\r\n", cCmdDescriptinArr[ ulCmd ] );
			}

			break;
	}

    pcWriteBuffer[ 0 ] = 0;

    return pdFALSE;
}
/*-----------------------------------------------------------*/
