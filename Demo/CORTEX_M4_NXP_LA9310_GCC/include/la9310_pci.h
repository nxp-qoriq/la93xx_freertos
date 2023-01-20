// SPDX-License-Identifier: BSD-3-Clause
/*
 * Copyright 2017-2018, 2021-2023 NXP
 *
 * Copyright 2014-2016 Freescale Semiconductor, Inc.
 */
    
#ifndef __LA9310_PCI_H
#define __LA9310_PCI_H

#include "config.h"
#include <la9310_pci_def.h>

#if NXP_ERRATUM_A_009410
    #define PCIE_PEX_PF_CONTROL_BASE        ( PCIE_BASE_ADDR + 0xC0000 )

    #define PCIE_PF0_INT_STAT_REG_OFFSET    ( 0x18 )
    #define PCIE_PF0_INT_STAT_REG \
    ( PCIE_PEX_PF_CONTROL_BASE +  \
      PCIE_PF0_INT_STAT_REG_OFFSET )

    #define PCIE_PF0_PME_MES_DR_OFFSET      ( 0x20 )
    #define PCIE_PF0_PME_MES_DR_REG \
    ( PCIE_PEX_PF_CONTROL_BASE +    \
      PCIE_PF0_PME_MES_DR_OFFSET )

    #define PCIE_PF0_PME_MES_DISR_OFFSET    ( 0x24 )
    #define PCIE_PF0_PME_MES_DISR_REG \
    ( PCIE_PEX_PF_CONTROL_BASE +      \
      PCIE_PF0_PME_MES_DISR_OFFSET )

    #define PCIE_PF0_PME_MES_IER_OFFSET     ( 0x28 )
    #define PCIE_PF0_PME_MES_IER_REG \
    ( PCIE_PEX_PF_CONTROL_BASE +     \
      PCIE_PF0_PME_MES_IER_OFFSET )

    #define PCIE_PF0_MCR_OFFSET             ( 0x2C )
    #define PCIE_PF0_MCR_REG     \
    ( PCIE_PEX_PF_CONTROL_BASE + \
      PCIE_PF0_MCR_OFFSET )

    #define PCIE_PF0_ERR_DR_OFFSET          ( 0x200 )
    #define PCIE_PF0_ERR_DR_REG  \
    ( PCIE_PEX_PF_CONTROL_BASE + \
      PCIE_PF0_ERR_DR_OFFSET )

    #define PCIE_PF0_ERR_EN_OFFSET          ( 0x208 )
    #define PCIE_PF0_ERR_EN_REG  \
    ( PCIE_PEX_PF_CONTROL_BASE + \
      PCIE_PF0_ERR_EN_OFFSET )

    #define PCIE_PF0_ERR_DISR_OFFSET        ( 0x210 )
    #define PCIE_PF0_ERR_DISR_REG \
    ( PCIE_PEX_PF_CONTROL_BASE +  \
      PCIE_PF0_ERR_DISR_OFFSET )

    #define PCIE_PF0_DBG_OFFSET             ( 0x7FC )
    #define PCIE_PF0_DBG_REG     \
    ( PCIE_PEX_PF_CONTROL_BASE + \
      PCIE_PF0_DBG_OFFSET )

    #define LA9310_TCML_BASE_ADDR                  ( 0x1f800000 ) /* TCML base addr */
    #define LA9310_TCMU_BASE_ADDR                  ( 0x20000000 ) /* TCMU base addr */

/* Addr:CCSR_BASE_ADDR, Size 256M, maps to CCSR space */
    #define PCIE_ATU_BAR0_TARGET            CCSR_BASE_ADDR

/* Addr:TCML_BASE_ADDR, Size:128K, maps to TCM code mem */
    #define PCIE_ATU_BAR1_TARGET            LA9310_TCML_BASE_ADDR

/* Addr:TCMU_BASE_ADDR, Size:8M, maps to TCM data & VSPA DMEM mem */
    #define PCIE_ATU_BAR2_TARGET            LA9310_TCMU_BASE_ADDR

/* iATU registers */
    #define PCIE_ATU_BASE                   ( PCIE_BASE_ADDR + 0x900 )
    #define PCIE_ATU_VIEWPORT_E             ( PCIE_ATU_BASE + 0x0 )
    #define PCIE_ATU_CR1_E                  ( PCIE_ATU_BASE + 0x4 )
    #define PCIE_ATU_CR2_E                  ( PCIE_ATU_BASE + 0x8 )
    #define PCIE_ATU_BAR_NUM( bar )    ( ( bar ) << 8 )
    #define PCIE_ATU_LOWER_TARGET_E         ( PCIE_ATU_BASE + 0x18 )
    #define PCIE_ATU_UPPER_TARGET_E         ( PCIE_ATU_BASE + 0x1C )

    #define PCIE_INTM                       ( 1 << 15 ) /*Per PF Message int is pending*/
    #define PCIE_INTE                       ( 1 << 14 ) /*Per PF Error int is pending*/

    #define PCIE_LUDD                       ( 1 << 7 )  /*Link up was detected*/
    #define PCIE_LDDD                       ( 1 << 9 )  /*Link down was detected*/
    #define PCIE_HRDD                       ( 1 << 10 ) /*Host reset was detected*/

    #define PCIE_LUDIE                      ( 1 << 7 )  /*Enable Link up detection interrupt*/
    #define PCIE_LDDIE                      ( 1 << 9 )  /*Enable Link down detection interrupt*/
    #define PCIE_HRDIE                      ( 1 << 10 ) /*Enable Host reset detection interrupt*/

    #define PCIE_LINK_STABLE_COUNT          ( 10 )
    #define PCIE_LTSSM_MASK                 ( 0x3F )
    #define PCIE_LTSSM_L0_STATE             ( 0x11 )
    #define PCIE_LTSSM_DELAY_COUNT          ( 128 )
#endif /* if NXP_ERRATUM_A_009410 */

#if NXP_ERRATUM_A_009531
    #define PCIE_DEVICE_CONTROL_2_REG       ( PCIE_BASE_ADDR + 0x98 )
    #define PCIE_DC2_IDO_REQ_ENABLE_BIT     ( 8 )
    #define PCIE_DC2_IDO_COMP_ENABLE_BIT    ( 9 )
    #define PCIE_DC2_IDO_MASK                \
    ( ( 1 << PCIE_DC2_IDO_REQ_ENABLE_BIT ) | \
      ( 1 << PCIE_DC2_IDO_COMP_ENABLE_BIT ) )
#endif

/* Setting outbound window registers */
void vLa9310PcieiAtuOutboundSet( void * dbi,
                                 int idx,
                                 int type,
                                 uint64_t cpu_addr,
                                 uint64_t pciAddr,
                                 uint32_t size );

void vWaitForPCIeLinkStability();

#if NXP_ERRATUM_A_009410
    void vPCIEInterruptInit( void );
#endif

#if NXP_ERRATUM_A_009531
    void vPCIEIDOClear( void );
#endif

void vLA9310DumpPCIeRegs( void );
#endif /* __LA9310_PCI_H */
