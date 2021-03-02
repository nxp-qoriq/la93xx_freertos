/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2017, 2021 NXP
 */

#ifndef __LA9310_HOST_V2H_H__
#define __LA9310_HOST_V2H_H__

#include "la9310_v2h_if.h"

int raiseMSI( struct la9310_info * pLa9310Info );
void printBDRing( struct v2h_buffer_desc * bdPointer );
int findNextValidBDindex( struct v2h_buffer_desc * bdPointer,
                          int headPointer );
int vV2HDemo( struct la9310_info * pLa9310Info );

#endif /* ifndef __LA9310_HOST_V2H_H__ */
