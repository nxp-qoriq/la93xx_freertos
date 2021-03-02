/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2022 NXP
 */

#include "immap.h"
#include "debug_console.h"
#include "llcp_api.h"

void vLlcpRegRW( uint16_t * val,
                 uint16_t addr,
                 uint8_t rw )
{
    uint32_t uiLlcpAddr = LA9310_RFIC_LLCP_BASE_ADDR;

    /* During the LLCP standalone verification (LA8310  LLCP + Venom
     * LLCP(reference rfic card)), it was noticed that the
     * LA9310 addressing scheme is AMBA APB
     * compliant while the Venom is not. For this reason, in case the
     * LA9310 LLCP is used to communicate with the Venom LLCP, a special
     * care should be taken when addressing the venom memory map. The
     * address should be shifted one bit to the left.
     */
    addr = addr << 1;

    if( rw == LLCP_REG_W )
    {
        llcp_WRITE( ( uint16_t * ) ( uiLlcpAddr + addr ), *val );
        log_dbg( "%s:write 0x%x: 0x%x\n", __func__,
                 ( addr >> 1 ), *val );
    }
    else
    {
        *val = llcp_READ( ( uint16_t * ) ( uiLlcpAddr + addr ) );
        log_dbg( "%s:read 0x%x: 0x%x\n", __func__,
                 ( addr >> 1 ), *val );
    }
}
