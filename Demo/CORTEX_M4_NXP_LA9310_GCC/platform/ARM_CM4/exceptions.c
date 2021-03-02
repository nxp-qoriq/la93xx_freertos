/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright 2017, 2021 NXP
 */

#include "FreeRTOS.h"
#include <common.h>
#include "config.h"
#include "immap.h"
#include "board.h"
#include "la9310_main.h"
#include <la9310.h>
#include "debug_console.h"
#include "exceptions.h"

static void prvPrintUsageFaultReason( uint32_t ulVal )
{
    PRINTF( "Usage fault: " );

    if( ( ulVal & ( 1 << 0 ) ) )
    {
        PRINTF( "Undefined instruction\n\r" );
    }
    else if( ( ulVal & ( 1 << 1 ) ) )
    {
        PRINTF( "Invalid state Usage Fault\n\r" );
    }
    else if( ( ulVal & ( 1 << 2 ) ) )
    {
        PRINTF( "Invalid PC load UsageFault caused by invalid \
		       EXC_RETURN value\n\r" );
    }
    else if( ( ulVal & ( 1 << 3 ) ) )
    {
        PRINTF( "Coprocessor access error\n\r" );
    }
    else if( ( ulVal & ( 1 << 8 ) ) )
    {
        PRINTF( "Unaligned access error\n\r" );
    }
    else if( ( ulVal & ( 1 << 9 ) ) )
    {
        PRINTF( "Divide by zero\n\r" );
    }
}

static void prvPrintBusFaultReason( uint32_t ulVal )
{
    PRINTF( "Bus fault: " );

    if( ( ulVal & ( 1 << 0 ) ) )
    {
        PRINTF( "This bit indicates a bus fault on an instruction \
		       prefetch\n\r" );
    }
    else if( ( ulVal & ( 1 << 2 ) ) )
    {
        PRINTF( "Imprecise data access error\n\r" );
    }
    else if( ( ulVal & ( 1 << 3 ) ) )
    {
        PRINTF( "bus fault has occurred on exception return\n\r" );
    }
    else if( ( ulVal & ( 1 << 4 ) ) )
    {
        PRINTF( "Bus fault has occurred on exception entry\n\r" );
    }
    else if( ( ulVal & ( 1 << 5 ) ) )
    {
        PRINTF( " fault occurred during floating-point lazy state \
		       preservation\n\r" );
    }
    else if( ( ulVal & ( 1 << 7 ) ) && ( ulVal & ( 1 << 1 ) ) )
    {
        PRINTF( "Precise data access error. Bus Fault Address Register is: \
		       %x\n\r", SCB->BFAR );
    }
}

static void prvPrintMemMngFaultReason( uint32_t ulVal )
{
    PRINTF( "Memory fault: " );

    if( ( ulVal & ( 1 << 0 ) ) )
    {
        PRINTF( "Instruction access violation\n\r" );
    }
    else if( ( ulVal & ( 1 << 1 ) ) )
    {
        PRINTF( "Data access violation\n\r" );
    }
    else if( ( ulVal & ( 1 << 3 ) ) )
    {
        PRINTF( "Memory Management Fault on unstacking for a return \
		       from exception\n\r" );
    }
    else if( ( ulVal & ( 1 << 4 ) ) )
    {
        PRINTF( "Memory Management Fault on stacking for \
		       exception entry\n\r" );
    }
    else if( ( ulVal & ( 1 << 5 ) ) )
    {
        PRINTF( "Memory Management Fault during floating point \
		       lazy state preservation\n\r" );
    }

    if( ( ulVal & ( 1 << 7 ) ) )
    {
        PRINTF( "Memory management fault address : %x\n\r", SCB->MMFAR );
    }
}

static void prvGetHardFaultReason( void )
{
    uint32_t ulCFSRValue;

    PRINTF( "SCB->CFSR = 0x%08x\n\r", SCB->CFSR );
    PRINTF( "SCB->HFSR = 0x%08x\n\r", SCB->HFSR );

    ulCFSRValue = SCB->CFSR;

    if( ( SCB->HFSR & SCB_HFSR_DEBUGEVT_MSK ) )
    {
        PRINTF( "Debug Event Hard Fault\n\r" );
        PRINTF( "SCB->DFSR = 0x%08x\n", SCB->DFSR );
    }
    else if( ( SCB->HFSR & SCB_HFSR_VECTTBL_MSK ) )
    {
        PRINTF( "Fault was due to vector table read on exception \
		       processing\n\r" );
    }
    else if( ( SCB->HFSR & SCB_HFSR_FORCED_MSK ) )
    {
        PRINTF( "Forced Hard Fault\n\r" );

        if( ( SCB->CFSR & SCB_CFSR_USGFAULTSR_MSK ) )
        {
            ulCFSRValue >>= SCB_CFSR_USGFAULTSR_POS;
            prvPrintUsageFaultReason( ulCFSRValue );
        }
        else if( ( SCB->CFSR & SCB_CFSR_BUSFAULTSR_MSK ) )
        {
            ulCFSRValue >>= SCB_CFSR_BUSFAULTSR_POS;
            prvPrintBusFaultReason( ulCFSRValue );
        }
        else if( ( SCB->CFSR & SCB_CFSR_MEMFAULTSR_MSK ) )
        {
            ulCFSRValue >>= SCB_CFSR_MEMFAULTSR_POS;
            prvPrintMemMngFaultReason( ulCFSRValue );
        }
    }

    isb();
    dmb();

    while( 1 )
    {
    }
}

static void prvDumpStackFrame( uint32_t * pulMSP )
{
    volatile uint32_t stacked_r0;
    volatile uint32_t stacked_r1;
    volatile uint32_t stacked_r2;
    volatile uint32_t stacked_r3;
    volatile uint32_t stacked_r12;
    volatile uint32_t stacked_lr;
    volatile uint32_t stacked_pc;
    volatile uint32_t stacked_psr;

    stacked_r0 = ( ( uint32_t ) pulMSP[ 0 ] );
    stacked_r1 = ( ( uint32_t ) pulMSP[ 1 ] );
    stacked_r2 = ( ( uint32_t ) pulMSP[ 2 ] );
    stacked_r3 = ( ( uint32_t ) pulMSP[ 3 ] );
    stacked_r12 = ( ( uint32_t ) pulMSP[ 4 ] );
    stacked_lr = ( ( uint32_t ) pulMSP[ 5 ] );
    stacked_pc = ( ( uint32_t ) pulMSP[ 6 ] );
    stacked_psr = ( ( uint32_t ) pulMSP[ 7 ] );

    PRINTF( "========STACK FRAME=========\r\n" );
    PRINTF( "\tr0 : %x\r\n", stacked_r0 );
    PRINTF( "\tr1 : %x\r\n", stacked_r1 );
    PRINTF( "\tr2 : %x\r\n", stacked_r2 );
    PRINTF( "\tr3 : %x\r\n", stacked_r3 );
    PRINTF( "\tr12 : %x\r\n", stacked_r12 );
    PRINTF( "\tlr : %x\r\n", stacked_lr );
    PRINTF( "\tpc : %x\r\n", stacked_pc );
    PRINTF( "\tpsr : %x\r\n", stacked_psr );
    PRINTF( "=====END OF STACK FRAME======\r\n" );
}

void vAnalyzeFault( uint32_t * pulMSP,
                    uint32_t ulFaultAddr )
{
    prvDumpStackFrame( pulMSP );
    PRINTF( "\nFault Address : %x\r\n", ulFaultAddr );
    prvGetHardFaultReason();
}

void vHardFaultHandler( void )
{
    __asm volatile
    (
        " tst lr, #4		\n"
        " ite eq		\n"
        " mrseq r0, msp        \n"
        " mrsne r0, psp	\n"
        " ldr r1, [r0,#24]	\n"
        " b vAnalyzeFault	\n"
    );
}

void vEnableExceptions( void )
{
    SCB->CCR |= ENABLE_DIV_UA_EXCEPTIONS;
}
