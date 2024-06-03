/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright 2017-2018, 2021-2024 NXP
 */

#ifndef __IMMAP_H__
#define __IMMAP_H__

#include <types.h>

#define CCSR_BASE_ADDR                0x40000000

#define UART_BASEADDR                 ( CCSR_BASE_ADDR + 0x21c0500 )
#define PMUXCR_BASE_ADDR              ( CCSR_BASE_ADDR + 0x1ff0e00 )
#define DCR_BASE_ADDR                 ( CCSR_BASE_ADDR + 0x1e00000 )
#define PCIE_BASE_ADDR                ( CCSR_BASE_ADDR + 0x3400000 )
#ifdef LA9310_ENABLE_COMMAND_LINE
    #define PCIE_CONTROL_BASE_ADDR    ( CCSR_BASE_ADDR + 0x34C0000 )
#endif
#define MSG_UNIT_BASE_ADDR            ( CCSR_BASE_ADDR + 0x1fc0000 )
#define EDMA_BASE_ADDR                ( CCSR_BASE_ADDR + 0x22c0000 )
#define VSPA_BASE_ADDR                ( CCSR_BASE_ADDR + 0x1000000 )
#define WDOG_BASE_ADDR                ( CCSR_BASE_ADDR + 0x23c0000 )
#define PCTBEN_BASE_ADDR              ( CCSR_BASE_ADDR + 0x1e308a0 )
#define LA9310_FSL_I2C1               ( CCSR_BASE_ADDR + 0x2000000 )
#define LA9310_FSL_I2C2               ( CCSR_BASE_ADDR + 0x2010000 )
#define DCS_BASE_ADDR                 ( CCSR_BASE_ADDR + 0x1040000 )
#define LA9310_RFIC_LLCP_BASE_ADDR    ( CCSR_BASE_ADDR + 0x4010000 )

#define PCIE_MSI_ADDR_REG             ( 0x54 )
#define PCIE_MSI_DATA_REG_1           ( 0x5c )

/* CCSR Reset base addr */
#define CCSR_RST_BASE_ADDRESS         ( CCSR_BASE_ADDR + 0x1E60000 )
#define RCW_COMPLETIONR_OFFSET        0x104 /* RCW comp reg off */
#define PBI_COMPLETIONR_OFFSET        0x114 /* PBI comp reg off */
#define RSTSR_OFFSET                  0x0C  /* Reset ready reg off */
#define RCW_COMPLETION_DONE           0x1   /* RCW comp bit */
#define PBI_COMPLETION_DONE           0x1   /* PBI comp bit */
#define RST_READY                     0x1   /* Reset ready bit */

#define RCW_COMPLETIONR               ( CCSR_RST_BASE_ADDRESS + RCW_COMPLETIONR_OFFSET )
#define PBI_COMPLETIONR               ( CCSR_RST_BASE_ADDRESS + PBI_COMPLETIONR_OFFSET )
#define RSTSR                         ( CCSR_RST_BASE_ADDRESS + RSTSR_OFFSET )

#define TMU_BASE_ADDR           (CCSR_BASE_ADDR + 0x1F80000)

/* Pin Mux configuration registers */
struct ccsr_pmux
{
    uint32_t ulPmuxCR0; /* PMUXCR0 */
    uint32_t ulPmuxCR1; /* PMUXCR1 */
#define PMUXCR0_PPSOUT_PIN  0x3
#define PMUXCR0_UART_PIN    0x0
};

/* Global Utilities Block */
struct ccsr_dcr
{
    uint32_t ulPorsr1;          /* POR status 1 */
    uint32_t ulPorsr2;          /* POR status 2 */
    uint8_t ucRes008[ 0x70 - 0x08 ];
    uint32_t ulDevdisr1;        /* Device disable control 1 */
    uint32_t ulDevdisr2;        /* Device disable control 2 */
    uint32_t ulDevdisr3;        /* Device disable control 3 */
    uint32_t ulDevdisr4;        /* Device disable control 4 */
    uint32_t ulDevdisr5;        /* Device disable control 5 */
    uint8_t ucRes084[ 0x94 - 0x84 ];
    uint32_t ulCoredisr;        /* core disable register */
    uint8_t ucRes098[ 0xa4 - 0x98 ];
    uint32_t ulSvr;             /* System version */
    uint8_t ucRes0a8[ 0x200 - 0xa8 ];
    uint32_t ulScratchrw[ 32 ]; /* Scratch Read/Write */
    uint8_t ucRes280[ 0x300 - 0x280 ];
    uint32_t ulScratchw1r[ 4 ]; /* Scratch Read (Write once) */
    uint8_t ucRes858[ 0xbfc - 0x310 ];
};
#endif /* __IMMAP_H__ */
