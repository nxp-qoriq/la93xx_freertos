/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2017, 2021 NXP
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

void vHardwareEarlyInit( void );
void vSocInit( void );
void vBoardEarlyInit( void );
void vBoardFinalInit( void );
void vSocResetHandshake( void );
void vLa9310_do_handshake( struct la9310_info * vLa9310Info );
#endif /* _COMMON_H_ */
