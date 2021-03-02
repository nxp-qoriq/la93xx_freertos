/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2017, 2021 NXP
 */

#ifndef __EXCEPTIONS_H__
#define __EXCEPTIONS_H__

#include <la9310.h>

typedef enum exp_id
{
    UNALIGNED_ACCESS = 1,
    DIVIDE_BY_ZERO = 2,
    NULL_ACCESS = 3,
    ILLEGAL_ADDR = 4
} EXCEPTION_ID;

/* Enable Divide by Zero */
#define ENABLE_DIV_UA_EXCEPTIONS    ( 0x10 )

/* Reserved address in RDEF */
#define ILLEGAL_ADDR_LOCATION       ( 0x2020004 )
#define DUMMY_TEST_VAL              ( 10 )

/*****************************************************************************
 * @vEnableExceptions
 *
 * Enables Divide by Zero and Unaligned memory access exceptions
 *
 ****************************************************************************/
void vEnableExceptions( void );

/*****************************************************************************
 * @vAnalyzeFault
 *
 * Analyzes the SCB registers and prints the reason for fault.
 *
 * pulMSP - [IN][M]  Stack frame address prior to exception has been generated.
 *
 * ulFaultAddr - [IN][M]  PC value. Point to the location where exception has
 * been generated.
 *
 ****************************************************************************/
void vAnalyzeFault( uint32_t * pulMSP,
                    uint32_t ulFaultAddr );

#endif /* ifndef __EXCEPTIONS_H__ */
