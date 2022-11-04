/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2017, 2021-2022 NXP
 */

#ifndef __LA9310_MAIN_H__
#define __LA9310_MAIN_H__

#include "la9310_host_if.h"
#include "immap.h"
#include "core_cm4.h"
#include "la9310.h"
#ifdef __RFIC
#include "rfic_core.h"
#endif
#include <common.h>

/* CM4 and VSPA communication opcode */
typedef enum la9310_mbx_opcode
{
    SINGLE_TONE_TX = 1,
    SINGLE_TONE_RX_MEASUREMENT,
    DCOC_CAL,
    BW_CAL,
    IQ_MODULATED_TX,
    IQ_MODULATED_RX,
    RSSI_MEASUREMENT,
    RX_IQ_MISMATCH_COMP,
    TX_LO_LEAKAGE_IQ_ERROR_CAL,
    VSPA_FW_IMAGE_BASE_OFFSET,
    TX_DC_CORRECTION,
    RX_DC_CORRECTION
} la9310_mbx_opcode_t;

struct la9310_msi_info
{
    uint32_t __IO addr;
    uint32_t data;
};

#define LA9310_EVT_MAX    32

struct la9310_irq_evt_info
{
    int ievt_count;
    int ievt_en_mask;
    struct la9310_evt_hdlr * phdlr_tbl;
};

#define LA9310_HOST_READY_MASK    ( LA9310_HIF_STATUS_VSPA_READY )

#ifdef TURN_ON_HOST_MODE
struct la9310_info
{
    struct ccsr_dcr * pxDcr;
    void * itcm_addr;
    void * dtcm_addr;
    void * pcie_addr;
    void * pcie_obound;
    struct la9310_msg_unit * msg_unit;
    uint32_t llcp_rfic_addr;
    struct la9310_stats * stats;
    struct la9310_hif * pHif;
    struct la9310_msi_info msi_info[ LA9310_MSI_MAX_CNT ];
    struct la9310_irq_evt_info evt_info;
#ifdef __RFIC
    RficDevice_t *pRficDev;
#endif
};
#else //TURN_ON_STANDALONE_MODE
struct la9310_info //Fix-Me
{
    struct ccsr_dcr * pxDcr;
    void * itcm_addr;
    void * dtcm_addr;
    void * pcie_addr;
    void * pcie_obound;
    struct la9310_msg_unit * msg_unit;
    uint32_t llcp_rfic_addr;
    struct la9310_stats * stats;
    struct la9310_hif * pHif;  
    struct la9310_msi_info msi_info[ LA9310_MSI_MAX_CNT ];
    struct la9310_irq_evt_info evt_info;
#ifdef __RFIC
    RficDevice_t *pRficDev;
#endif
};
#endif //TURN_ON_STANDALONE_MODE
extern struct la9310_info * pLa9310Info;
#endif /* ifndef __LA9310_MAIN_H__ */
