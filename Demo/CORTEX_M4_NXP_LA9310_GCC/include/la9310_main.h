/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2017, 2021-2024 NXP
 */

#ifndef __LA9310_MAIN_H__
#define __LA9310_MAIN_H__

#include "FreeRTOS.h"
#include "la9310_host_if.h"
#include "immap.h"
#include "core_cm4.h"
#include "la9310.h"
#ifdef __RFIC
#include "rfic_core.h"
#endif
#include <common.h>

#define SITES_MAX       16
#define TMU_TTRCR0_INIT 0x000B0000
#define TMU_TTRCR1_INIT 0x000A0026
#define TMU_TTRCR2_INIT 0x00080048
#define TMU_TTRCR3_INIT 0x00070061
#define TMU_TTRCR0_POINT	12
#define TMU_TTRCR1_POINT	11
#define TMU_TTRCR2_POINT	11
#define TMU_TTRCR3_POINT	8

#define TMU_TTCFGR_INIT0	0x00000000
#define TMU_TSCFGR_INIT0	0x00000025
#define TMU_TSCFGR_DIFF0	0x00000006
#define TMU_TTCFGR_INIT1	0x00010000
#define TMU_TSCFGR_INIT1	0x0000001C
#define TMU_TSCFGR_DIFF1	0x00000008
#define TMU_TTCFGR_INIT2	0x00010000
#define TMU_TSCFGR_INIT2	0x0000001C
#define TMU_TSCFGR_DIFF2	0x00000008
#define TMU_TTCFGR_INIT3	0x00030000
#define TMU_TSCFGR_INIT3	0x0000000E
#define TMU_TSCFGR_DIFF3	0x0000000C
#define TMU_TTCFGR_DIFF		0x00000001

#define TMU_TEUMR0_ENABLE       0x51009C00
#define TMU_TDEMAR_ENABLE       0x0800FFFE
#define TMU_TMTMIR_ENABLE       0x0000000F
#define TMU_TMRTRCTR_INIT	0x0000000A
#define TMU_TMFTRCTR_INIT	0x0000000A
#define TMU_TMSAR0_INIT		0x0000000E
#define TMU_TMSAR1_INIT		0x0000000E
#define TMU_TMSAR2_INIT		0x0000000E

#define TMU_TMR_DISABLE 	0x0
#define TMU_TSR_INIT            0xF0000000
#define TMU_TMR_ENABLE          0x8000C000

typedef struct tmuSiteRegs {
	uint32_t tritsr; /*  Immediate Temperature Site Register */
	uint32_t tratsr; /*  Average Temperature Site Register */
	uint8_t res0[0x8];
} TmuSiteRegs_t;

typedef struct tmuCalibrationSiteRegs {
	uint32_t tscr;  /*  Immediate Temperature Site Register */
	uint32_t tmsar; /*  Average Temperature Site Register */
	uint8_t res0[0x8];
} TmuCalibrationSiteRegs_t;

typedef struct tmuRegsMap {
	uint32_t tmr;           /*  Mode Register */
	uint32_t tsr;           /*  Status Register */
	uint32_t tmtmir;                /*  Temperature measurement interval Register */
	uint8_t res0[0x14];
	uint32_t tier;          /*  Interrupt Enable Register */
	uint32_t tidr;          /*  Interrupt Detect Register */
	uint8_t res1[0x8];
	uint32_t tiiscr;                /*  Interrupt Immediate Site Capture Register */
	uint32_t tiascr;                /*  Interrupt Average Site Capture Register */
	uint32_t ticscr;                /*  Interrupt Critical Site Capture Register */
	uint8_t res2[0x4];
	uint32_t tmhtcrh;               /*  High Temperature Capture Register */
	uint32_t tmhtcrl;               /*  Low Temperature Capture Register */
	uint32_t tmrtrcr;               /*  Low Temperature Capture Register */
	uint32_t tmftrcr;               /*  Low Temperature Capture Register */
	uint32_t tmhtitr;               /*  High Temperature Immediate Threshold Reg */
	uint32_t tmhtatr;               /*  High Temperature Average Threshold Reg */
	uint32_t tmhtactr;              /*  High Temperature Average Crit Threshold Reg */
	uint8_t res3[0x4];
	uint32_t tmltitr;               /*  Low Temperature Immediate Threshold Reg */
	uint32_t tmltatr;               /*  Low Temperature Average Threshold Reg */
	uint32_t tmltactr;              /*  Low Temperature Average Critical Threshold Reg */
	uint8_t res4[0x4];
	uint32_t tmrtrctr;              /*  TMU monitor rising temperature rate critical threshold register */
	uint32_t tmftrctr;              /*  TMU monitor falling temperature rate critical threshold register */
	uint8_t res5[0x8];
	uint32_t ttcfgr;                /*  Temperature configuration register */
	uint32_t tscfgr;                /*  Sensor configuration register */
	uint8_t res6[0x8];
	uint32_t ttcr;          /*  Temperature calibration register */
	uint8_t res7[0x6C];
	TmuSiteRegs_t site[SITES_MAX];   /*  Temp site register */
	uint8_t res8[0x100];
	TmuCalibrationSiteRegs_t monitoringSite[SITES_MAX]; /*  Monitoring Site reg */
	uint8_t res9[0x7F8];
	uint32_t ipbrr0;                 /*  IP Block Revision Register 0 */
	uint32_t ipbrr1;                /*  IP Block Revision Register 1 */
	uint8_t res10[0x300];
	uint32_t teumr[0x3];    /*  Engineering use mode register */
	uint32_t tdemar;                /*  Dynamic element match averaging register */
	uint32_t ttrcr[SITES_MAX];      /*  Temperature Range Control Register */
} TmuRegs_t;

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
    RX_DC_CORRECTION,
    RFNM_SET_CHANNEL,
    RFNM_IQ_IMBALANCE
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
