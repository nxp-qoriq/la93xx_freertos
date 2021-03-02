/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2017-2018, 2021 NXP
 */

#include "io.h"
#include "la9310_pci.h"
#include "debug_console.h"
#include "config.h"

#define lower_32_bits( n )    ( ( uint32_t ) ( n ) )
#define upper_32_bits( n )    ( ( uint32_t ) ( ( ( n ) >> 16 ) >> 16 ) )

#ifdef LA9310_ENABLE_COMMAND_LINE
    void vLA9310DumpPCIeRegs()
    {
        volatile uint32_t i = 0, j = 0, val[ 4 ];

        volatile uint32_t * pcie_regs = ( volatile uint32_t * ) PCIE_BASE_ADDR;
        volatile uint32_t * addr = pcie_regs;

        for( i = 0; i < 22; i++ )
        {
            for( j = 0; j < 4; j++ )
            {
                val[ j ] = *pcie_regs;
                pcie_regs += 1;
            }

            PRINTF( "%08x : %08x %08x %08x %08x\r\n", addr, val[ 0 ], val[ 1 ],
                    val[ 2 ], val[ 3 ] );

            addr = pcie_regs;
        }

        pcie_regs = ( volatile uint32_t * ) ( PCIE_CONTROL_BASE_ADDR + 0x14 );
        val[ 0 ] = *pcie_regs;
        PRINTF( "%08x : %08x\r\n", pcie_regs, val[ 0 ] );

        pcie_regs = ( volatile uint32_t * ) ( PCIE_CONTROL_BASE_ADDR + 0x18 );
        val[ 0 ] = *pcie_regs;
        PRINTF( "%08x : %08x\r\n", pcie_regs, val[ 0 ] );

        pcie_regs = ( volatile uint32_t * ) ( PCIE_CONTROL_BASE_ADDR + 0x20 );
        val[ 0 ] = *pcie_regs;
        PRINTF( "%08x : %08x\r\n", pcie_regs, val[ 0 ] );

        pcie_regs = ( volatile uint32_t * ) ( PCIE_CONTROL_BASE_ADDR + 0x24 );
        val[ 0 ] = *pcie_regs;
        PRINTF( "%08x : %08x\r\n", pcie_regs, val[ 0 ] );

        pcie_regs = ( volatile uint32_t * ) ( PCIE_CONTROL_BASE_ADDR + 0x28 );
        val[ 0 ] = *pcie_regs;
        PRINTF( "%08x : %08x\r\n", pcie_regs, val[ 0 ] );

        pcie_regs = ( volatile uint32_t * ) ( PCIE_CONTROL_BASE_ADDR + 0x2C );
        val[ 0 ] = *pcie_regs;
        PRINTF( "%08x : %08x\r\n", pcie_regs, val[ 0 ] );

        pcie_regs = ( volatile uint32_t * ) ( PCIE_CONTROL_BASE_ADDR + 0x140 );
        val[ 0 ] = *pcie_regs;
        PRINTF( "%08x : %08x\r\n", pcie_regs, val[ 0 ] );

        pcie_regs = ( volatile uint32_t * ) ( PCIE_CONTROL_BASE_ADDR + 0x200 );
        val[ 0 ] = *pcie_regs;
        PRINTF( "%08x : %08x\r\n", pcie_regs, val[ 0 ] );

        pcie_regs = ( volatile uint32_t * ) ( PCIE_CONTROL_BASE_ADDR + 0x208 );
        val[ 0 ] = *pcie_regs;
        PRINTF( "%08x : %08x\r\n", pcie_regs, val[ 0 ] );

        pcie_regs = ( volatile uint32_t * ) ( PCIE_CONTROL_BASE_ADDR + 0x210 );
        val[ 0 ] = *pcie_regs;
        PRINTF( "%08x : %08x\r\n", pcie_regs, val[ 0 ] );

        pcie_regs = ( volatile uint32_t * ) ( PCIE_CONTROL_BASE_ADDR + 0x7FC );
        val[ 0 ] = *pcie_regs;
        PRINTF( "%08x : %08x\r\n", pcie_regs, val[ 0 ] );
    }
#endif /* ifdef LA9310_ENABLE_COMMAND_LINE */

void vLa9310PcieiAtuOutboundSet( void * dbi,
                                 int idx,
                                 int type,
                                 uint64_t cpu_addr,
                                 uint64_t pciAddr,
                                 uint32_t size )
{
    log_dbg( "dbi %llx lcpu %#x ucpu %#x lpci %#x upci %#x size %lu\n",
             dbi, lower_32_bits( cpu_addr ), upper_32_bits( cpu_addr ),
             lower_32_bits( pciAddr ), upper_32_bits( pciAddr ),
             ( unsigned long ) size );

    write_32( PCIE_ATU_REGION_OUTBOUND | idx,
              dbi + PCIE_ATU_VIEWPORT );
    write_32( lower_32_bits( cpu_addr ), dbi + PCIE_ATU_LOWER_BASE );
    write_32( upper_32_bits( cpu_addr ), dbi + PCIE_ATU_UPPER_BASE );
    write_32( lower_32_bits( cpu_addr + size - 1 ), dbi + PCIE_ATU_LIMIT );
    write_32( lower_32_bits( pciAddr ), dbi + PCIE_ATU_LOWER_TARGET );
    write_32( upper_32_bits( pciAddr ), dbi + PCIE_ATU_UPPER_TARGET );
    write_32( type, dbi + PCIE_ATU_CR1 );
    write_32( PCIE_ATU_ENABLE, dbi + PCIE_ATU_CR2 );
}

void vBusyDelay( uint32_t ulLoopCount )
{
    volatile uint32_t i;
    volatile uint32_t j;

    for( i = 0; i < ulLoopCount; i++ )
    {
        for( j = 0; j < ulLoopCount; j++ )
        {
            j += 1;
            j -= 1;
        }
    }
}

void vWaitForPCIeLinkStability()
{
    volatile uint32_t ulDbgRegVal;
    volatile uint32_t ulCount;
    uint32_t * pulPEX_PF0_DBG;

    pulPEX_PF0_DBG = ( uint32_t * ) ( PCIE_PF0_DBG_REG );
    ulCount = 0;

    /* Check link Stability @ L0 state */
    while( 1 )
    {
        ulDbgRegVal = IN_32( pulPEX_PF0_DBG );

        if( ( ulDbgRegVal & PCIE_LTSSM_MASK ) ==
            PCIE_LTSSM_L0_STATE )
        {
            ulCount += 1;
        }
        else
        {
            ulCount = 0;
        }

        if( ulCount >= PCIE_LINK_STABLE_COUNT )
        {
            break;
        }

        vBusyDelay( PCIE_LTSSM_DELAY_COUNT );
    }
}

#if NXP_ERRATUM_A_009531
    void vPCIEIDOClear( void )
    {
        uint32_t * pulDeviceReg2;
        uint32_t ulDeviceReg2Value;

        pulDeviceReg2 = ( uint32_t * ) ( PCIE_DEVICE_CONTROL_2_REG );

        ulDeviceReg2Value = IN_32( pulDeviceReg2 );

        /*
         * Clear IDO Request Enable and IDO Completion Enable bits
         * and update the Device Control Register 2.
         */
        ulDeviceReg2Value &= ~PCIE_DC2_IDO_MASK;
        OUT_32( pulDeviceReg2, ulDeviceReg2Value );
    }
#endif /* if NXP_ERRATUM_A_009531 */

#if NXP_ERRATUM_A_009410
    void vPCIEInterruptInit( void )
    {
        uint32_t * pulPEX_PF0_PME_MES_IER;
        uint32_t * pulPEX_PF0_PME_MES_DR;

        uint32_t ulIntEnableMask;
        uint32_t ulIntEnableRegValue;

        pulPEX_PF0_PME_MES_IER = ( uint32_t * ) ( PCIE_PF0_PME_MES_IER_REG );
        pulPEX_PF0_PME_MES_DR = ( uint32_t * ) ( PCIE_PF0_PME_MES_DR_REG );

        vWaitForPCIeLinkStability();

        /* Clear all errors once link is stable @ L0 State*/
        OUT_32( pulPEX_PF0_PME_MES_DR, 0xFFFF );

        ulIntEnableMask = PCIE_LDDIE | PCIE_LUDIE |
                          PCIE_HRDIE;
        ulIntEnableRegValue = IN_32( pulPEX_PF0_PME_MES_IER );

        /*Enable Link Down, Link Up and Hot Reset interrupt*/
        ulIntEnableRegValue |= ulIntEnableMask;

        NVIC_EnableIRQ( IRQ_PCIE );
        OUT_32( pulPEX_PF0_PME_MES_IER, ulIntEnableRegValue );
    }

    static void prvSetupPcieAtu( void )
    {
        /* ATU 0 : INBOUND : map BAR0 */
        OUT_32( ( uint32_t * ) PCIE_ATU_VIEWPORT_E, ( PCIE_ATU_REGION_INBOUND |
                                                             PCIE_ATU_REGION_INDEX0 ) );
        OUT_32( ( uint32_t * ) PCIE_ATU_LOWER_TARGET_E, PCIE_ATU_BAR0_TARGET );
        OUT_32( ( uint32_t * ) PCIE_ATU_UPPER_TARGET_E, 0x0 );
        OUT_32( ( uint32_t * ) PCIE_ATU_CR1_E, PCIE_ATU_TYPE_MEM );
        OUT_32( ( uint32_t * ) PCIE_ATU_CR2_E, ( PCIE_ATU_ENABLE |
                                                        PCIE_ATU_BAR_MODE_ENABLE |
                                                        PCIE_ATU_BAR_NUM( 0 ) ) );

        /* ATU 1 : INBOUND : map BAR1 */
        OUT_32( ( uint32_t * ) PCIE_ATU_VIEWPORT_E, ( PCIE_ATU_REGION_INBOUND |
                                                             PCIE_ATU_REGION_INDEX1 ) );
        OUT_32( ( uint32_t * ) PCIE_ATU_LOWER_TARGET_E, PCIE_ATU_BAR1_TARGET );
        OUT_32( ( uint32_t * ) PCIE_ATU_UPPER_TARGET_E, 0x0 );
        OUT_32( ( uint32_t * ) PCIE_ATU_CR1_E, PCIE_ATU_TYPE_MEM );
        OUT_32( ( uint32_t * ) PCIE_ATU_CR2_E, ( PCIE_ATU_ENABLE |
                                                        PCIE_ATU_BAR_MODE_ENABLE |
                                                        PCIE_ATU_BAR_NUM( 1 ) ) );

        /* ATU 2 : INBOUND : map BAR2 */
        OUT_32( ( uint32_t * ) PCIE_ATU_VIEWPORT_E, ( PCIE_ATU_REGION_INBOUND |
                                                             PCIE_ATU_REGION_INDEX2 ) );
        OUT_32( ( uint32_t * ) PCIE_ATU_LOWER_TARGET_E, PCIE_ATU_BAR2_TARGET );
        OUT_32( ( uint32_t * ) PCIE_ATU_UPPER_TARGET_E, 0x0 );
        OUT_32( ( uint32_t * ) PCIE_ATU_CR1_E, PCIE_ATU_TYPE_MEM );
        OUT_32( ( uint32_t * ) PCIE_ATU_CR2_E, ( PCIE_ATU_ENABLE |
                                                        PCIE_ATU_BAR_MODE_ENABLE |
                                                        PCIE_ATU_BAR_NUM( 2 ) ) );
    }
#endif /* if NXP_ERRATUM_A_009410 */

void PCIE_IRQHandler( void )
{
    #if NXP_ERRATUM_A_009410
        uint32_t * pulPEX_PF0_PME_MES_DR;
        uint32_t ulIntDetectRegValue;
        uint32_t ulLDorHRMask;
        uint32_t ulLUMask;
        static uint8_t ucLDOrHRWasDetected = 0;

        pulPEX_PF0_PME_MES_DR = ( uint32_t * ) ( PCIE_PF0_PME_MES_DR_REG );
        ulIntDetectRegValue = IN_32( pulPEX_PF0_PME_MES_DR );
        ulLDorHRMask = PCIE_LDDD | PCIE_HRDD;
        ulLUMask = PCIE_LUDD;

        /*Detect Link up after Link down or Hot reset*/
        if( ( ulIntDetectRegValue & ulLUMask ) && ucLDOrHRWasDetected )
        {
            prvSetupPcieAtu();
            ucLDOrHRWasDetected = 0;
            return;
        }

        /*Detect Link down or Hot reset*/
        if( ulIntDetectRegValue & ulLDorHRMask )
        {
            ucLDOrHRWasDetected = 1;
        }
    #endif /* if NXP_ERRATUM_A_009410 */
    #if ARM_ERRATUM_838869
        dsb();
    #endif
}
