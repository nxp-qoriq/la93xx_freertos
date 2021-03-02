/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright 2017, 2021 NXP
 */

#ifndef __LA9310_CORE_CM4_H__
#define __LA9310_CORE_CM4_H__

#define __O         volatile
#define  __I        volatile
#define  __IO       volatile
#define __INLINE    inline
#define __ASM       asm

typedef struct
{
    __IO uint32_t ISER[ 8 ];             /*!< Offset: 0x000 (R/W)  Interrupt Set Enable Register           */
    uint32_t RESERVED0[ 24 ];
    __IO uint32_t ICER[ 8 ];             /*!< Offset: 0x080 (R/W)  Interrupt Clear Enable Register         */
    uint32_t RSERVED1[ 24 ];
    __IO uint32_t ISPR[ 8 ];             /*!< Offset: 0x100 (R/W)  Interrupt Set Pending Register          */
    uint32_t RESERVED2[ 24 ];
    __IO uint32_t ICPR[ 8 ];             /*!< Offset: 0x180 (R/W)  Interrupt Clear Pending Register        */
    uint32_t RESERVED3[ 24 ];
    __IO uint32_t IABR[ 8 ];             /*!< Offset: 0x200 (R/W)  Interrupt Active bit Register           */
    uint32_t RESERVED4[ 56 ];
    __IO uint8_t IP[ 240 ];              /*!< Offset: 0x300 (R/W)  Interrupt Priority Register (8Bit wide) */
    uint32_t RESERVED5[ 644 ];
    __O uint32_t STIR;                   /*!< Offset: 0xE00 ( /W)  Software Trigger Interrupt Register     */
} NVIC_Type;

/** \ingroup  CMSIS_core_register
 *  \defgroup CMSIS_SCB     System Control Block (SCB)
 *  \brief      Type definitions for the System Control Block Registers
 * @{
 */

/** \brief  Structure type to access the System Control Block (SCB).
 */
typedef struct
{
    __I uint32_t CPUID;                  /*!< Offset: 0x000 (R/ )  CPUID Base Register                                   */
    __IO uint32_t ICSR;                  /*!< Offset: 0x004 (R/W)  Interrupt Control and State Register                  */
    __IO uint32_t VTOR;                  /*!< Offset: 0x008 (R/W)  Vector Table Offset Register                          */
    __IO uint32_t AIRCR;                 /*!< Offset: 0x00C (R/W)  Application Interrupt and Reset Control Register      */
    __IO uint32_t SCR;                   /*!< Offset: 0x010 (R/W)  System Control Register                               */
    __IO uint32_t CCR;                   /*!< Offset: 0x014 (R/W)  Configuration Control Register                        */
    __IO uint8_t SHP[ 12 ];              /*!< Offset: 0x018 (R/W)  System Handlers Priority Registers (4-7, 8-11, 12-15) */
    __IO uint32_t SHCSR;                 /*!< Offset: 0x024 (R/W)  System Handler Control and State Register             */
    __IO uint32_t CFSR;                  /*!< Offset: 0x028 (R/W)  Configurable Fault Status Register                    */
    __IO uint32_t HFSR;                  /*!< Offset: 0x02C (R/W)  HardFault Status Register                             */
    __IO uint32_t DFSR;                  /*!< Offset: 0x030 (R/W)  Debug Fault Status Register                           */
    __IO uint32_t MMFAR;                 /*!< Offset: 0x034 (R/W)  MemManage Fault Address Register                      */
    __IO uint32_t BFAR;                  /*!< Offset: 0x038 (R/W)  BusFault Address Register                             */
    __IO uint32_t AFSR;                  /*!< Offset: 0x03C (R/W)  Auxiliary Fault Status Register                       */
    __I uint32_t PFR[ 2 ];               /*!< Offset: 0x040 (R/ )  Processor Feature Register                            */
    __I uint32_t DFR;                    /*!< Offset: 0x048 (R/ )  Debug Feature Register                                */
    __I uint32_t ADR;                    /*!< Offset: 0x04C (R/ )  Auxiliary Feature Register                            */
    __I uint32_t MMFR[ 4 ];              /*!< Offset: 0x050 (R/ )  Memory Model Feature Register                         */
    __I uint32_t ISAR[ 5 ];              /*!< Offset: 0x060 (R/ )  Instruction Set Attributes Register                   */
    uint32_t RESERVED0[ 5 ];
    __IO uint32_t CPACR;                 /*!< Offset: 0x088 (R/W)  Coprocessor Access Control Register                   */
} SCB_Type;

#define NVIC_BASE                       0xE000E100
#define NVIC                            ( ( NVIC_Type * ) NVIC_BASE )

#define SCS_BASE                        ( 0xE000E000UL )              /*!< System Control Space Base Address  */
#define SCB_BASE                        ( SCS_BASE + 0x0D00UL )       /*!< System Control Block Base Address  */
#define SCB                             ( ( SCB_Type * ) SCB_BASE )   /*!< SCB configuration struct           */

/* SCB CPUID Register Definitions */
#define SCB_CPUID_IMPLEMENTER_POS       24
#define SCB_CPUID_IMPLEMENTER_MSK       ( 0xFFUL << SCB_CPUID_IMPLEMENTER_POS )

#define SCB_CPUID_VARIANT_POS           20
#define SCB_CPUID_VARIANT_MSK           ( 0xFUL << SCB_CPUID_VARIANT_POS )

#define SCB_CPUID_ARCHITECTURE_POS      16
#define SCB_CPUID_ARCHITECTURE_MSK      ( 0xFUL << SCB_CPUID_ARCHITECTURE_POS )

#define SCB_CPUID_PARTNO_POS            4
#define SCB_CPUID_PARTNO_MSK            ( 0xFFFUL << SCB_CPUID_PARTNO_POS )

#define SCB_CPUID_REVISION_POS          0
#define SCB_CPUID_REVISION_MSK          ( 0xFUL << SCB_CPUID_REVISION_POS )

/* SCB Interrupt Control State Register Definitions */
#define SCB_ICSR_NMIPENDSET_POS         31
#define SCB_ICSR_NMIPENDSET_MSK         ( 1UL << SCB_ICSR_NMIPENDSET_POS )

#define SCB_ICSR_PENDSVSET_POS          28
#define SCB_ICSR_PENDSVSET_MSK          ( 1UL << SCB_ICSR_PENDSVSET_POS )

#define SCB_ICSR_PENDSVCLR_POS          27
#define SCB_ICSR_PENDSVCLR_MSK          ( 1UL << SCB_ICSR_PENDSVCLR_POS )

#define SCB_ICSR_PENDSTSET_POS          26
#define SCB_ICSR_PENDSTSET_MSK          ( 1UL << SCB_ICSR_PENDSTSET_POS )

#define SCB_ICSR_PENDSTCLR_POS          25
#define SCB_ICSR_PENDSTCLR_MSK          ( 1UL << SCB_ICSR_PENDSTCLR_POS )

#define SCB_ICSR_ISRPREEMPT_POS         23
#define SCB_ICSR_ISRPREEMPT_MSK         ( 1UL << SCB_ICSR_ISRPREEMPT_POS )

#define SCB_ICSR_ISRPENDING_POS         22
#define SCB_ICSR_ISRPENDING_MSK         ( 1UL << SCB_ICSR_ISRPENDING_POS )

#define SCB_ICSR_VECTPENDING_POS        12
#define SCB_ICSR_VECTPENDING_MSK        ( 0x1FFUL << SCB_ICSR_VECTPENDING_POS )

#define SCB_ICSR_RETTOBASE_POS          11
#define SCB_ICSR_RETTOBASE_MSK          ( 1UL << SCB_ICSR_RETTOBASE_POS )

#define SCB_ICSR_VECTACTIVE_POS         0
#define SCB_ICSR_VECTACTIVE_MSK         ( 0x1FFUL << SCB_ICSR_VECTACTIVE_POS )

/* SCB Vector Table Offset Register Definitions */
#define SCB_VTOR_TBLOFF_POS             7
#define SCB_VTOR_TBLOFF_MSK             ( 0x1FFFFFFUL << SCB_VTOR_TBLOFF_POS )

/* SCB Application Interrupt and Reset Control Register Definitions */
#define SCB_AIRCR_VECTKEY_POS           16
#define SCB_AIRCR_VECTKEY_MSK           ( 0xFFFFUL << SCB_AIRCR_VECTKEY_POS )

#define SCB_AIRCR_VECTKEYSTAT_POS       16
#define SCB_AIRCR_VECTKEYSTAT_MSK \
    ( 0xFFFFUL <<                 \
        SCB_AIRCR_VECTKEYSTAT_POS )

#define SCB_AIRCR_ENDIANESS_POS         15
#define SCB_AIRCR_ENDIANESS_MSK         ( 1UL << SCB_AIRCR_ENDIANESS_POS )

#define SCB_AIRCR_PRIGROUP_POS          8
#define SCB_AIRCR_PRIGROUP_MSK          ( 7UL << SCB_AIRCR_PRIGROUP_POS )

#define SCB_AIRCR_SYSRESETREQ_POS       2
#define SCB_AIRCR_SYSRESETREQ_MSK       ( 1UL << SCB_AIRCR_SYSRESETREQ_POS )

#define SCB_AIRCR_VECTCLRACTIVE_POS     1
#define SCB_AIRCR_VECTCLRACTIVE_MSK     ( 1UL << SCB_AIRCR_VECTCLRACTIVE_POS )

#define SCB_AIRCR_VECTRESET_POS         0
#define SCB_AIRCR_VECTRESET_MSK         ( 1UL << SCB_AIRCR_VECTRESET_POS )

/* SCB System Control Register Definitions */
#define SCB_SCR_SEVONPEND_POS           4
#define SCB_SCR_SEVONPEND_MSK           ( 1UL << SCB_SCR_SEVONPEND_POS )

#define SCB_SCR_SLEEPDEEP_POS           2
#define SCB_SCR_SLEEPDEEP_MSK           ( 1UL << SCB_SCR_SLEEPDEEP_POS )

#define SCB_SCR_SLEEPONEXIT_POS         1
#define SCB_SCR_SLEEPONEXIT_MSK         ( 1UL << SCB_SCR_SLEEPONEXIT_POS )

/* SCB Configuration Control Register Definitions */
#define SCB_CCR_STKALIGN_POS            9
#define SCB_CCR_STKALIGN_MSK            ( 1UL << SCB_CCR_STKALIGN_POS )

#define SCB_CCR_BFHFNMIGN_POS           8
#define SCB_CCR_BFHFNMIGN_MSK           ( 1UL << SCB_CCR_BFHFNMIGN_POS )

#define SCB_CCR_DIV_0_TRP_POS           4
#define SCB_CCR_DIV_0_TRP_MSK           ( 1UL << SCB_CCR_DIV_0_TRP_POS )

#define SCB_CCR_UNALIGN_TRP_POS         3
#define SCB_CCR_UNALIGN_TRP_MSK         ( 1UL << SCB_CCR_UNALIGN_TRP_POS )

#define SCB_CCR_USERSETMPEND_POS        1
#define SCB_CCR_USERSETMPEND_MSK        ( 1UL << SCB_CCR_USERSETMPEND_POS )

#define SCB_CCR_NONBASETHRDENA_POS      0
#define SCB_CCR_NONBASETHRDENA_MSK      ( 1UL << SCB_CCR_NONBASETHRDENA_POS )

/* SCB System Handler Control and State Register Definitions */
#define SCB_SHCSR_USGFAULTENA_POS       18
#define SCB_SHCSR_USGFAULTENA_MSK       ( 1UL << SCB_SHCSR_USGFAULTENA_POS )

#define SCB_SHCSR_BUSFAULTENA_POS       17
#define SCB_SHCSR_BUSFAULTENA_MSK       ( 1UL << SCB_SHCSR_BUSFAULTENA_POS )

#define SCB_SHCSR_MEMFAULTENA_POS       16
#define SCB_SHCSR_MEMFAULTENA_MSK       ( 1UL << SCB_SHCSR_MEMFAULTENA_POS )

#define SCB_SHCSR_SVCALLPENDED_POS      15
#define SCB_SHCSR_SVCALLPENDED_MSK      ( 1UL << SCB_SHCSR_SVCALLPENDED_POS )

#define SCB_SHCSR_BUSFAULTPENDED_POS    14
#define SCB_SHCSR_BUSFAULTPENDED_MSK    ( 1UL << SCB_SHCSR_BUSFAULTPENDED_POS )

#define SCB_SHCSR_MEMFAULTPENDED_POS    13
#define SCB_SHCSR_MEMFAULTPENDED_MSK    ( 1UL << SCB_SHCSR_MEMFAULTPENDED_POS )

#define SCB_SHCSR_USGFAULTPENDED_POS    12
#define SCB_SHCSR_USGFAULTPENDED_MSK    ( 1UL << SCB_SHCSR_USGFAULTPENDED_POS )

#define SCB_SHCSR_SYSTICKACT_POS        11
#define SCB_SHCSR_SYSTICKACT_MSK        ( 1UL << SCB_SHCSR_SYSTICKACT_POS )

#define SCB_SHCSR_PENDSVACT_POS         10
#define SCB_SHCSR_PENDSVACT_MSK         ( 1UL << SCB_SHCSR_PENDSVACT_POS )

#define SCB_SHCSR_MONITORACT_POS        8
#define SCB_SHCSR_MONITORACT_MSK        ( 1UL << SCB_SHCSR_MONITORACT_POS )

#define SCB_SHCSR_SVCALLACT_POS         7
#define SCB_SHCSR_SVCALLACT_MSK         ( 1UL << SCB_SHCSR_SVCALLACT_POS )

#define SCB_SHCSR_USGFAULTACT_POS       3
#define SCB_SHCSR_USGFAULTACT_MSK       ( 1UL << SCB_SHCSR_USGFAULTACT_POS )

#define SCB_SHCSR_BUSFAULTACT_POS       1
#define SCB_SHCSR_BUSFAULTACT_MSK       ( 1UL << SCB_SHCSR_BUSFAULTACT_POS )

#define SCB_SHCSR_MEMFAULTACT_POS       0
#define SCB_SHCSR_MEMFAULTACT_MSK       ( 1UL << SCB_SHCSR_MEMFAULTACT_POS )

/* SCB Configurable Fault Status Registers Definitions */
#define SCB_CFSR_USGFAULTSR_POS         16
#define SCB_CFSR_USGFAULTSR_MSK         ( 0xFFFFUL << SCB_CFSR_USGFAULTSR_POS )

#define SCB_CFSR_BUSFAULTSR_POS         8
#define SCB_CFSR_BUSFAULTSR_MSK         ( 0xFFUL << SCB_CFSR_BUSFAULTSR_POS )

#define SCB_CFSR_MEMFAULTSR_POS         0
#define SCB_CFSR_MEMFAULTSR_MSK         ( 0xFFUL << SCB_CFSR_MEMFAULTSR_POS )

/* SCB Hard Fault Status Registers Definitions */
#define SCB_HFSR_DEBUGEVT_POS           31
#define SCB_HFSR_DEBUGEVT_MSK           ( 1UL << SCB_HFSR_DEBUGEVT_POS )

#define SCB_HFSR_FORCED_POS             30
#define SCB_HFSR_FORCED_MSK             ( 1UL << SCB_HFSR_FORCED_POS )

#define SCB_HFSR_VECTTBL_POS            1
#define SCB_HFSR_VECTTBL_MSK            ( 1UL << SCB_HFSR_VECTTBL_POS )

/* SCB Debug Fault Status Register Definitions */
#define SCB_DFSR_EXTERNAL_POS           4
#define SCB_DFSR_EXTERNAL_MSK           ( 1UL << SCB_DFSR_EXTERNAL_POS )

#define SCB_DFSR_VCATCH_POS             3
#define SCB_DFSR_VCATCH_MSK             ( 1UL << SCB_DFSR_VCATCH_POS )

#define SCB_DFSR_DWTTRAP_POS            2
#define SCB_DFSR_DWTTRAP_MSK            ( 1UL << SCB_DFSR_DWTTRAP_POS )

#define SCB_DFSR_BKPT_POS               1
#define SCB_DFSR_BKPT_MSK               ( 1UL << SCB_DFSR_BKPT_POS )

#define SCB_DFSR_HALTED_POS             0
#define SCB_DFSR_HALTED_MSK             ( 1UL << SCB_DFSR_HALTED_POS )

/*@} end of group CMSIS_SCB */

/* Macros to Implment Critical sections */
#define disable_irq()                 \
    do {                              \
        __asm volatile ( "CPSID i" ); \
        __asm volatile ( "DSB" );     \
        __asm volatile ( "ISB" );     \
    }while( 0 )

#define enable_irq()                  \
    do {                              \
        __asm volatile ( "CPSIE i" ); \
        __asm volatile ( "DSB" );     \
        __asm volatile ( "ISB" );     \
    }while( 0 )

/*Define CM4 external IRQs*/
typedef enum
{
    IRQ_GPIO = 0,
    IRQ_I2C_1 = 1,
    IRQ_I2C_2 = 2,
    IRQ_PCIE = 3,
    IRQ_DSPI1 = 4,
    IRQ_RSVD = 5,
    IRQ_RSVD_2 = 6,
    IRQ_RSVD_3 = 7,
    IRQ_RSVD_4 = 8,
    IRQ_EDMA = 9,
    IRQ_MSG1 = 10,
    IRQ_MSG2 = 11,
    IRQ_MSG3 = 12,
    IRQ_WDOG = 13,
    IRQ_UART = 14,
    IRQ_AEM = 15,
    IRQ_MBEE = 16,
    IRQ_AXIQ = 17,
    IRQ_ADAC = 18,
    IRQ_VSPA = 19,
    IRQ_TMU_ALRM = 20,
    IRQ_TMU_CRIT_ALRM = 21,
    IRQ_EPU = 22,
    IRQ_PPS_IN = 23,
    IRQ_PPS_OUT = 24,
    IRQ_TCM_ALIGNMENT_ERR = 25,
    IRQ_RSVD1 = 26,
    IRQ_RSVD2 = 28,
    IRQ_RSVD3 = 29,
    IRQ_RSVD4 = 30,
    IRQ_RSVD5 = 31,
} IRQn_Type;

#define __NVIC_PRIO_BITS    3

/** \brief  Set Interrupt Priority
 *
 *  The function sets the priority of an interrupt.
 *
 *  \note The priority cannot be set for every core interrupt.
 *
 *  \param [in]      IRQn  Interrupt number.
 *  \param [in]  priority  Priority to set.
 */
static __INLINE void NVIC_SetPriority( IRQn_Type IRQn,
                                       uint32_t priority )
{
    if( IRQn < 0 )
    {
        SCB->SHP[ ( ( uint32_t ) ( IRQn ) & 0xF ) - 4 ] = ( ( priority << ( 8 - __NVIC_PRIO_BITS ) ) & 0xff );
    }                                                                                       /* set Priority for Cortex-M  System Interrupts */
    else
    {
        NVIC->IP[ ( uint32_t ) ( IRQn ) ] = ( ( priority << ( 8 - __NVIC_PRIO_BITS ) ) & 0xff );
    }                                                                                       /* set Priority for device specific Interrupts  */
}


/** \brief  Get Interrupt Priority
 *
 *  The function reads the priority of an interrupt. The interrupt
 *  number can be positive to specify an external (device specific)
 *  interrupt, or negative to specify an internal (core) interrupt.
 *
 *
 *  \param [in]   IRQn  Interrupt number.
 *  \return             Interrupt Priority. Value is aligned automatically to the implemented
 *                      priority bits of the microcontroller.
 */
static __INLINE uint32_t NVIC_GetPriority( IRQn_Type IRQn )
{
    if( IRQn < 0 )
    {
        return( ( uint32_t ) ( SCB->SHP[ ( ( uint32_t ) ( IRQn ) & 0xF ) - 4 ] >> ( 8 - __NVIC_PRIO_BITS ) ) );
    }                                                                                      /* get priority for Cortex-M  system interrupts */
    else
    {
        return( ( uint32_t ) ( NVIC->IP[ ( uint32_t ) ( IRQn ) ] >> ( 8 - __NVIC_PRIO_BITS ) ) );
    }                                                                            /* get priority for device specific interrupts  */
}


/** \brief  Enable External Interrupt
 *
 *  This function enables a device specific interupt in the NVIC interrupt controller.
 *  The interrupt number cannot be a negative value.
 *
 *  \param [in]      IRQn  Number of the external interrupt to enable
 */
static __INLINE void NVIC_EnableIRQ( IRQn_Type IRQn )
{
    NVIC->ISER[ ( ( uint32_t ) ( IRQn ) >> 5 ) ] = ( 1 << ( ( uint32_t ) ( IRQn ) & 0x1F ) ); /* enable interrupt */
}


/** \brief  Disable External Interrupt
 *
 *  This function disables a device specific interupt in the NVIC interrupt controller.
 *  The interrupt number cannot be a negative value.
 *
 *  \param [in]      IRQn  Number of the external interrupt to disable
 */
static __INLINE void NVIC_DisableIRQ( IRQn_Type IRQn )
{
    NVIC->ICER[ ( ( uint32_t ) ( IRQn ) >> 5 ) ] = ( 1 << ( ( uint32_t ) ( IRQn ) & 0x1F ) ); /* disable interrupt */
}

/* @brief  Clear the pending bit for an external interrupt
 *
 * @param  IRQn    The number of the interrupt for clear pending
 *
 * Clear the pending bit for the specified interrupt.
 * The interrupt number cannot be a negative value.
 */
static __INLINE void NVIC_ClearPendingIRQ( IRQn_Type IRQn )
{
    NVIC->ICPR[ ( ( uint32_t ) ( IRQn ) >> 5 ) ] = ( 1 << ( ( uint32_t ) ( IRQn ) & 0x1F ) ); /* Clear pending interrupt */
}

#endif /* ifndef __LA9310_CORE_CM4_H__ */
