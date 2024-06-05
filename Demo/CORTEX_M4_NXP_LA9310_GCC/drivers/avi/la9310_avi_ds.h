/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright 2017, 2021-2024 NXP
 */

#include "queue.h"
#include "semphr.h"

#define AVI_DEMO

#define HOST_INBOX_STATUS_SHIFT            2

#define STATUS_DMA_COMP_STAT               0x00000010
#define STATUS_DMA_COMP_ERR                0x00000020

#define WAIT_TIMEOUT_FOR_MBOX_MILLISECS    100
#define AVI_RECV_VSPA_MBOX_TIMEOUT         10
#define WAIT_TIMEOUT_FOR_MBOX \
    pdMS_TO_TICKS( WAIT_TIMEOUT_FOR_MBOX_MILLISECS )

#define VSPA_ENABLE_MAILBOX_IRQ            0x0000F000
#define VSPA_MBOX_MASK                     0x0000F000
#define VSPA_MBOX0_STATUS_SHIFT            12
#define VSPA_MBOX1_STATUS_SHIFT            13
#define CM4_MBOX0_STATUS_SHIFT             14
#define CM4_MBOX1_STATUS_SHIFT             15
#define VSPA_MBOX0_STATUS                  ( 1 << VSPA_MBOX0_STATUS_SHIFT )
#define VSPA_MBOX1_STATUS                  ( 1 << VSPA_MBOX1_STATUS_SHIFT )
#define CM4_MBOX0_STATUS                   ( 1 << CM4_MBOX0_STATUS_SHIFT )
#define CM4_MBOX1_STATUS                   ( 1 << CM4_MBOX1_STATUS_SHIFT )

#define VSPA_CM4_Q_LEN                     16

struct vspa_regs
{
    uint32_t hw_version;
    uint32_t sw_version;
    uint32_t vcpu_ctrl;
    uint32_t vspa_irqen;
    uint32_t vspa_status;
    uint32_t vcpu_host_flags0;
    uint32_t vcpu_host_flags1;
    uint32_t host_vcpu_flags0;
    uint32_t host_vcpu_flags1;
    uint8_t res1c[ 0x28 - 0x24 ];
    uint32_t ext_go_ena;
    uint32_t ext_go_stat;
    uint32_t ill_op_status;
    uint8_t res40[ 0x40 - 0x34 ];
    uint32_t param0;
    uint32_t param1;
    uint32_t param2;
    uint32_t vcpu_dmem_bytes;
    uint32_t thread_ctrl_stat;
    uint32_t prot_fault_stat;
    uint32_t exception_ctrl;
    uint32_t exception_stat;
    uint32_t axislv_flags0;
    uint32_t axislv_flags1;
    uint32_t axislv_goen0;
    uint32_t axislv_goen1;
    uint32_t plat_in_0;
    uint8_t res80[ 0x80 - 0x74 ];
    uint32_t plat_out_0;
    uint8_t res98[ 0x98 - 0x84 ];
    uint32_t cyc_counter_msb;
    uint32_t cyc_counter_lsb;
    uint8_t resB0[ 0xB0 - 0xA0 ];
    uint32_t dma_dmem_pram_addr;
    uint32_t dma_axi_address;
    uint32_t dma_axi_byte_cnt;
    uint32_t dma_xfr_ctrl;
    uint32_t dma_stat_abort;
    uint32_t dma_irq_stat;
    uint32_t dma_comp_stat;
    uint32_t dma_xfrerr_stat;
    uint32_t dma_cfgerr_stat;
    uint32_t dma_xrun_stat;
    uint32_t dma_go_stat;
    uint32_t dma_fifo_stat;
    uint8_t res100[ 0x100 - 0xE0 ];
    uint32_t ld_rf_control;
    uint32_t ld_rf_tb_real_0;
    uint32_t ld_rf_tb_imag_0;
    uint32_t ld_rf_tb_real_1;
    uint32_t ld_rf_tb_imag_1;
    uint32_t ld_rf_tb_real_2;
    uint32_t ld_rf_tb_imag_2;
    uint32_t ld_rf_tb_real_3;
    uint32_t ld_rf_tb_imag_3;
    uint32_t ld_rf_tb_real_4;
    uint32_t ld_rf_tb_imag_4;
    uint32_t ld_rf_tb_real_5;
    uint32_t ld_rf_tb_imag_5;
    uint32_t ld_rf_tb_real_6;
    uint32_t ld_rf_tb_imag_6;
    uint32_t ld_rf_tb_real_7;
    uint32_t ld_rf_tb_imag_7;
    uint8_t res400[ 0x400 - 0x144 ];
    uint32_t vcpu_mode0;
    uint32_t vcpu_mode1;
    uint32_t vcpu_creg0;
    uint32_t vcpu_creg1;
    uint32_t st_ul_vec_len;
    uint8_t res500[ 0x500 - 0x414 ];
    uint32_t gp_in[ 16 ];
    uint8_t res580[ 0x580 - 0x540 ];
    uint32_t gp_out[ 16 ];
    uint8_t res600[ 0x600 - 0x5c0 ];
    uint32_t dqm_small;
    uint32_t dqm_large_msb;
    uint32_t dqm_large_lsb;
    uint8_t res620[ 0x620 - 0x60c ];
    uint32_t vcpu_dbg_out_32;
    uint32_t vcpu_dbg_out_64_msb;
    uint32_t vcpu_dbg_out_64_lsb;
    uint32_t vcpu_dbg_in_32;
    uint32_t vcpu_dbg_in_64_msb;
    uint32_t vcpu_dbg_in_64_lsb;
    uint32_t vcpu_dbg_mbox_status;
    uint8_t res640[ 0x640 - 0x63c ];
    uint32_t vcpu_out_0_msb;
    uint32_t vcpu_out_0_lsb;
    uint32_t vcpu_out_1_msb;
    uint32_t vcpu_out_1_lsb;
    uint32_t vcpu_in_0_msb;
    uint32_t vcpu_in_0_lsb;
    uint32_t vcpu_in_1_msb;
    uint32_t vcpu_in_1_lsb;
    uint32_t vcpu_mbox_status;
    uint8_t res680[ 0x680 - 0x664 ];
    uint32_t host_out_0_msb;
    uint32_t host_out_0_lsb;
    uint32_t host_out_1_msb;
    uint32_t host_out_1_lsb;
    uint32_t host_in_0_msb;
    uint32_t host_in_0_lsb;
    uint32_t host_in_1_msb;
    uint32_t host_in_1_lsb;
    uint32_t host_mbox_status;
    uint8_t res700[ 0x700 - 0x6A4 ];
    uint32_t ippucontrol;
    uint32_t ippustatus;
    uint32_t ippurc;
    uint32_t ippuargbaseaddr;
    uint32_t ippuhwver;
    uint32_t ippuswver;
    uint8_t res2000[ 0x2000 - 0x718 ];
};

struct avi_hndlr
{
    struct vspa_regs * pVspaRegs;
    QueueHandle_t VspaToCm4QMbox0;
    QueueHandle_t VspaToCm4QMbox1;
    QueueHandle_t Cm4ToVspaQMbox0;
    QueueHandle_t Cm4ToVspaQMbox1;
    SemaphoreHandle_t Cm4ToVspaQMutex0;
    SemaphoreHandle_t Cm4ToVspaQMutex1;
};

#ifdef AVI_DEMO

/**
 * @brief : Function added to assist in Demo Testing
 *			Reads VSPA inbox sent from CM4
 *
 * @param[in] : AVI library handle
 * @param[in/out] : Structure to be populated
 * @param uint32_t[in] : MBOX index
 *
 * @return : 0 on success, -ve on failure
 */
    int iLa9310AviReadVspaInbox( void *, struct avi_mbox *, uint32_t );

/**
 * @brief : Function added to assist in Demo Testing
 *			Send VSPA outbos message to CM4
 *
 * @param[in]  : AVI library handle
 * @param uint32_t : Mailbox MSB
 * @param uint32_t : Mailbox LSB
 * @param uint32_t : Mailbox Index
 *
 * @return : 0 on success, -ve on failure
 */
    int iLa9310AviSendVspaOutbox( void *, uint32_t, uint32_t, uint32_t );
#endif
void AviHndleMboxInterrupt( struct avi_hndlr * );
void La9310VSPA_IRQDefaultHandler( void );
void La9310VSPA_IRQRelayHandler( void );
void La9310VSPA_IRQHandler( void );
unsigned int iLa9310AviVspaHwVer( void );
unsigned int iLa9310AviVspaSwVer( void );
void iLa9310VspaInit( void );
void iLa9310AviClose( void );
void vVSPAMboxMonitorMaskSet( uint32_t );
