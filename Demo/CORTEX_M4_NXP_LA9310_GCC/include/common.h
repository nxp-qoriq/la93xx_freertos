/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2017, 2021-2022 NXP
 */

#ifndef _COMMON_H_
#define _COMMON_H_

#include <stdbool.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <debug_console.h>
#include <io.h>
#include <la9310_host_if.h>
#include <la9310_main.h>
#include <la9310_boot_mode.h>

typedef enum NLMBordRevision {
	NLM_BOARD_REV_1 = 1,
	NLM_BOARD_REV_2,
	NLM_MAX_BOARD_REV
}NLMBoardRev_t;


#define DEBUG_HEX_DUMP
void vHardwareEarlyInit( void );
void vSocInit( void );
void vBoardEarlyInit( void );
void vBoardFinalInit( void );
void vSocResetHandshake( void );
void vLa9310_do_handshake( struct la9310_info * vLa9310Info );
void vLa9310_Read_Board_Rev_pca9570(void);
int iLa9310_Get_Board_Rev(void);

#ifdef DEBUG_HEX_DUMP
extern void hexdump( const unsigned char * src,
		uint32_t ulCount );
#endif
#endif /* _COMMON_H_ */
