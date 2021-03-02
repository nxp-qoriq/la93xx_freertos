/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright 2017, 2021 NXP
 */

/*****************************************************************************/
/* Version: GCC for ARM Embedded Processors                                  */
/*****************************************************************************/
    .syntax unified
    .arch armv7-m

    .section .isr_vector, "a"
    .align 2
    .globl __isr_vector
__isr_vector:
    .long   __StackTop                                      /* Top of Stack */
    .long   Reset_Handler                                   /* Reset Handler */
    .long   NMI_Handler                                     /* NMI Handler*/
    .long   HardFault_Handler                               /* Hard Fault Handler*/
    .long   MemManage_Handler                               /* MPU Fault Handler*/
    .long   BusFault_Handler                                /* Bus Fault Handler*/
    .long   UsageFault_Handler                              /* Usage Fault Handler*/
    .long   0                                               /* Reserved*/
    .long   0                                               /* Reserved*/
    .long   0                                               /* Reserved*/
    .long   0                                               /* Reserved*/
    .long   SVC_Handler                                     /* SVCall Handler*/
    .long   DebugMon_Handler                                /* Debug Monitor Handler*/
    .long   0                                               /* Reserved*/
    .long   PendSV_Handler                                  /* PendSV Handler*/
    .long   SysTick_Handler                                 /* SysTick Handler*/

                                                            /* External Interrupts*/
    .long   DefaultISR                                      /* GPIO */
    .long   DefaultISR                                      /* I2C1 */
    .long   DefaultISR                                      /* I2C2 */
    .long   PCIE_IRQHandler                           	    /* PCIE */
    .long   DefaultISR                                      /* SPI1 */
    .long   DefaultISR                                      /* Reserved */
    .long   DefaultISR                                      /* IP1 */
    .long   DefaultISR                                      /* IP2 */
    .long   DefaultISR                                      /* IP3 */
    .long   La9310eDMA_IRQHandler                              /* EDMA */
    .long   La9310MSG_1_IRQHandler                             /* MSG 1 */
    .long   La9310MSG_2_IRQHandler		                    /* MSG 2 */
    .long   La9310MSG_3_IRQHandler			            /* MSG 3 */
    .long   La9310WDOG_IRQHandler                              /* WDOG */
    .long   DefaultISR                                      /* UART */
    .long   DefaultISR                                      /* AEM */
    .long   DefaultISR                                      /* MBEE */
    .long   DefaultISR                                      /* AXIQ */
    .long   DefaultISR                                      /* ADC/DAC */
    .long   La9310VSPA_IRQHandler                              /* VSPA */
    .long   DefaultISR                                      /* TMU Thermal alarm */
    .long   DefaultISR                                      /* TMU Thermal critical alarm */
    .long   DefaultISR                                      /* EPU */
    .long   vPhyTimerPPSINHandler                           /* Pulse per second in */
    .long   vPhyTimerPPSOUTHandler                          /* Pulse per second out */
    .long   DefaultISR                                      /* Reserved */
    .long   DefaultISR                                      /* Reserved */
    .long   DefaultISR                                      /* Reserved */
    .long   DefaultISR                                      /* Reserved */
    .long   DefaultISR                                      /* Reserved */
    .long   DefaultISR                                      /* Reserved */
    .long   DefaultISR                                      /* Reserved */
    .long   DefaultISR                                      /* 48 */
    .long   DefaultISR                                      /* 49 */
    .long   DefaultISR                                      /* 50 */
    .long   DefaultISR                                      /* 51 */
    .long   DefaultISR                                      /* 52 */
    .long   DefaultISR                                      /* 53 */
    .long   DefaultISR                                      /* 54 */
    .long   DefaultISR                                      /* 55 */
    .long   DefaultISR                                      /* 56 */
    .long   DefaultISR                                      /* 57 */
    .long   DefaultISR                                      /* 58 */
    .long   DefaultISR                                      /* 59 */
    .long   DefaultISR                                      /* 60 */
    .long   DefaultISR                                      /* 61 */
    .long   DefaultISR                                      /* 62 */
    .long   DefaultISR                                      /* 63 */
    .long   DefaultISR                                      /* 64 */
    .long   DefaultISR                                      /* 65 */
    .long   DefaultISR                                      /* 66 */
    .long   DefaultISR                                      /* 67 */
    .long   DefaultISR                                      /* 68 */
    .long   DefaultISR                                      /* 69 */
    .long   DefaultISR                                      /* 70 */
    .long   DefaultISR                                      /* 71 */
    .long   DefaultISR                                      /* 72 */
    .long   DefaultISR                                      /* 73 */
    .long   DefaultISR                                      /* 74 */
    .long   DefaultISR                                      /* 75 */
    .long   DefaultISR                                      /* 76 */
    .long   DefaultISR                                      /* 77 */
    .long   DefaultISR                                      /* 78 */
    .long   DefaultISR                                      /* 79 */
    .long   DefaultISR                                      /* 80 */
    .long   DefaultISR                                      /* 81 */
    .long   DefaultISR                                      /* 82 */
    .long   DefaultISR                                      /* 83 */
    .long   DefaultISR                                      /* 84 */
    .long   DefaultISR                                      /* 85 */
    .long   DefaultISR                                      /* 86 */
    .long   DefaultISR                                      /* 87 */
    .long   DefaultISR                                      /* 88 */
    .long   DefaultISR                                      /* 89 */
    .long   DefaultISR                                      /* 90 */
    .long   DefaultISR                                      /* 91 */
    .long   DefaultISR                                      /* 92 */
    .long   DefaultISR                                      /* 93 */
    .long   DefaultISR                                      /* 94 */
    .long   DefaultISR                                      /* 95 */
    .long   DefaultISR                                      /* 96 */
    .long   DefaultISR                                      /* 97 */
    .long   DefaultISR                                      /* 98 */
    .long   DefaultISR                                      /* 99 */
    .long   DefaultISR                                      /* 100 */
    .long   DefaultISR                                      /* 101 */
    .long   DefaultISR                                      /* 102 */
    .long   DefaultISR                                      /* 103 */
    .long   DefaultISR                                      /* 104 */
    .long   DefaultISR                                      /* 105 */
    .long   DefaultISR                                      /* 106 */
    .long   DefaultISR                                      /* 107 */
    .long   DefaultISR                                      /* 108 */
    .long   DefaultISR                                      /* 109 */
    .long   DefaultISR                                      /* 110 */
    .long   DefaultISR                                      /* 111 */
    .long   DefaultISR                                      /* 112 */
    .long   DefaultISR                                      /* 113 */
    .long   DefaultISR                                      /* 114 */
    .long   DefaultISR                                      /* 115 */
    .long   DefaultISR                                      /* 116 */
    .long   DefaultISR                                      /* 117 */
    .long   DefaultISR                                      /* 118 */
    .long   DefaultISR                                      /* 119 */
    .long   DefaultISR                                      /* 120 */
    .long   DefaultISR                                      /* 121 */
    .long   DefaultISR                                      /* 122 */
    .long   DefaultISR                                      /* 123 */
    .long   DefaultISR                                      /* 124 */
    .long   DefaultISR                                      /* 125 */
    .long   DefaultISR                                      /* 126 */
    .long   DefaultISR                                      /* 127 */
    .long   DefaultISR                                      /* 128 */
    .long   DefaultISR                                      /* 129 */
    .long   DefaultISR                                      /* 130 */
    .long   DefaultISR                                      /* 131 */
    .long   DefaultISR                                      /* 132 */
    .long   DefaultISR                                      /* 133 */
    .long   DefaultISR                                      /* 134 */
    .long   DefaultISR                                      /* 135 */
    .long   DefaultISR                                      /* 136 */
    .long   DefaultISR                                      /* 137 */
    .long   DefaultISR                                      /* 138 */
    .long   DefaultISR                                      /* 139 */
    .long   DefaultISR                                      /* 140 */
    .long   DefaultISR                                      /* 141 */
    .long   DefaultISR                                      /* 142 */
    .long   DefaultISR                                      /* 143 */

    .size    __isr_vector, . - __isr_vector


    .equ CONTROL_NPRIV, 0x1

    .section .start_up, "a"
    .global m4_startup
m4_startup:
    /* Disable Interrupts */
    cpsid    i

    /* Initialize Core Registers */
    mov    r0, #0x0
    mov	   r1, r0
    mov	   r2, r0
    mov	   r3, r0
    mov	   r4, r0
    mov	   r5, r0
    mov	   r6, r0
    mov	   r7, r0

    /* Initialize stack pointer */
    ldr	r0, = __StackTop
    msr	msp, r0

    /* Initilize Vector Table */
    ldr	   r0, =__isr_vector
    ldr r1, = 0xE000ED08
    str	   r0, [r1]

    /* Set Priviledged Mode */
    mrs    r0, control
    bic    r0, #(CONTROL_NPRIV)
    msr    control, r0

    /* Jump to IBR Main Code */
    ldr    r0, =Reset_Handler

    /* Never returns from here */
    blx    r0
    .size m4_startup, . - m4_startup

    .text
    .thumb

/* Reset Handler */
    .thumb_func
    .align 2
    .globl   Reset_Handler
    .weak    Reset_Handler
    .type    Reset_Handler, %function
Reset_Handler:
    cpsid   i               /* Mask interrupts */
    cpsie   i               /* Unmask interrupts */
    bl    _start
    .pool
    .size Reset_Handler, . - Reset_Handler

    .align  1
    .thumb_func
    .weak DefaultISR
    .type DefaultISR, %function
DefaultISR:
    b DefaultISR
    .size DefaultISR, . - DefaultISR

/*    Macro to define default handlers. Default handler
 *    will be weak symbol and just dead loops. They can be
 *    overwritten by other handlers */
    .macro def_irq_handler  handler_name
    .weak \handler_name
    .set  \handler_name, DefaultISR
    .endm

/* Exception Handlers */
    def_irq_handler    NMI_Handler
    def_irq_handler    HardFault_Handler
    def_irq_handler    SVC_Handler
    def_irq_handler    PendSV_Handler
    def_irq_handler    SysTick_Handler
    def_irq_handler    MemManage_Handler
    def_irq_handler    BusFault_Handler
    def_irq_handler    UsageFault_Handler
    def_irq_handler    DebugMon_Handler
    def_irq_handler    La9310VSPA_IRQHandler
    def_irq_handler    PCIE_IRQHandler
    def_irq_handler    La9310eDMA_IRQHandler
    def_irq_handler    La9310MSG_1_IRQHandler
    def_irq_handler    La9310MSG_2_IRQHandler
    def_irq_handler    La9310MSG_3_IRQHandler
    def_irq_handler    La9310WDOG_IRQHandler 

    .end
