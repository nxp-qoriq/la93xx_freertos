/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright 2017, 2021 NXP
 */

#ifndef __LA9310_EDMA_H__
#define __LA9310_EDMA_H__

#include "FreeRTOS.h"
#include "task.h"
#include "common.h"


/* This Release is supporting memcpy at the place of EDMA */
#define EDMA_CR_REG                      ( EDMA_BASE_ADDR + 0x00 )
#define EDMA_ES_REG                      ( EDMA_BASE_ADDR + 0x04 )
#define EDMA_ERQ_REG                     ( EDMA_BASE_ADDR + 0x0c )
#define EDMA_EEI_REG                     ( EDMA_BASE_ADDR + 0x14 )
#define EDMA_INT_REG                     ( EDMA_BASE_ADDR + 0x24 )
#define EDMA_ERR_REG                     ( EDMA_BASE_ADDR + 0x2c )
#define EDMA_DCHPRI_REG                  ( EDMA_BASE_ADDR + 0x100 )
#define EDMA_TCD0                        ( EDMA_BASE_ADDR + 0x1000 )
#define WDOG_eDMA_CHANNEL                15
#define EDMA_CHANNELS                    14
#define EDMA_TCD_SIZE                    0x20
#define EDMA_CR_ERCA                     0x00000004
#define EDMA_INT_VAL                     0x00000000
#define EDMA_EEI_VAL                     0x00000000
#define EDMA_DCHPRI_DCHPRI0              0x0
#define NO_CHANNEL_AVL                   0x00003fff
/* SOFF and DOFF register Values */
#define EDMA_TCD_SOFF_4B                 0x00000004
#define EDMA_TCD_DOFF_4B                 0x00000004
/* ATTR for 4 byte TCD register values */
#define EDMA_TCD_ATTR_SMOD_DIS           ( 0 << 27 )
#define EDMA_TCD_ATTR_DMOD_DIS           ( 0 << 19 )
#define EDMA_TCD_ATTR_SSIZE_4B           0x02000000
#define EDMA_TCD_ATTR_DSIZE_4B           0x00020000
/* Values for BITER and CITER registers */
#define EDMA_TCD_CITER_ELINKYES_CITER    0x00010000
#define EDMA_TCD_BITER_ELINKYES_BITER    0x00010000
#define TCD_ELINKYES_ELINK_DIS           ( 0 << 31 )
/* Values for bits in CSR register */
#define EDMA_TCD_CSR_START               ( 1 << 0 )
#define EDMA_TCD_CSR_INTMJR              ( 1 << 1 )
#define EDMA_TCD_CSR_DREQ                ( 1 << 3 )
#define EDMA_TCD_CSR_ESG                 ( 0 << 4 )
#define EDMA_TCD_CSR_ACTIVE              ( 1 << 6 )
#define EDMA_TCD_CSR_DONE                ( 1 << 7 )

typedef enum status
{
    eDMA_FAIL = 0,
    eDMA_PASS = 1
} edma_status;

typedef void (* Callback)( void * info,
                           edma_status st );

typedef void * pvEdmaHandle;

typedef struct edma_tcd
{
    uint32_t edma_tcd_saddr;
    uint32_t edma_tcd_attr_soff;
    uint32_t edma_tcd_nbytes;
    uint32_t edma_tcd_slast;
    uint32_t edma_tcd_daddr;
    uint32_t edma_tcd_citer_elink_doff;
    uint32_t edma_tcd_dlastsga;
    uint32_t edma_tcd_biter_elink_csr;
} edma_tcd_t;

typedef struct edma_channel_info
{
    uint16_t channel_id;
    uint32_t tcd_addr;
    void * info;
    Callback pCallbackFn;
} edma_channel_info_t;

edma_channel_info_t * pEdmaGlb[ EDMA_CHANNELS ];

#endif /* ifndef __LA9310_EDMA_H__ */
