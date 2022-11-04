/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2017, 2021-2022 NXP
 */

#include <common.h>
#include "config.h"
#include "immap.h"
#include "board.h"
#include <la9310.h>
#include <la9310_i2cAPI.h>
#include "debug_console.h"

static int iNLMBoardRev;

void vLa9310_Read_Board_Rev_pca9570(void)
{
	int ret;
	uint8_t ucval[2 ];
	ret = iLa9310_I2C_Read( LA9310_FSL_I2C1, 0x24, 0,2, ucval, 1 );
	if(ret < NLM_BOARD_REV_1)
	{
		iNLMBoardRev = NLM_BOARD_REV_1;
		PRINTF("NLM Board Rev#1\n\r");
	}
	else
	{
		iNLMBoardRev = NLM_BOARD_REV_2;
		PRINTF("NLM Board Rev#2\n\r");
	}
}

int iLa9310_Get_Board_Rev(void)
{
	return iNLMBoardRev;
}

void vBoardEarlyInit( void )
{
	xDebugConsoleInit( ( void * ) UART_BASEADDR, EARLY_UART_CLOCK_FREQUENCY,
			UART_BAUDRATE );
	iLa9310_I2C_Init( LA9310_FSL_I2C1, EARLY_I2C_CLOCK_FREQUENCY, LA9310_I2C_FREQ );
}

void vBoardFinalInit( void )
{
	xDebugConsoleInit( ( void * ) UART_BASEADDR, FINAL_UART_CLOCK_FREQUENCY,
			UART_BAUDRATE );

	iLa9310_I2C_Init( LA9310_FSL_I2C1, FINAL_I2C_CLOCK_FREQUENCY, LA9310_I2C_FREQ );
	vLa9310_Read_Board_Rev_pca9570();
}
