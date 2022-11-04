/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2017-2022 NXP
 */

#include "FreeRTOS.h"
#include "task.h"
#include "common.h"
#include "immap.h"
#include "la9310_main.h"
#include "la9310_irq.h"
#include "la9310_edma.h"
#ifdef TURN_ON_HOST_MODE
#ifdef __RFIC
#include "rfic_core.h"
#endif
#endif

extern struct la9310_info * pLa9310Info;
extern void vIpcRxISR( int chid );

void vRaiseMsi( struct la9310_info * pla9310Info,
                enum la9310_msi_id msi )
{
    struct la9310_msi_info * pMsiInfo;
    struct la9310_stats * pStats = pla9310Info->stats;

    pMsiInfo = &pla9310Info->msi_info[ msi ];
    OUT_32( pMsiInfo->addr, pMsiInfo->data );

    log_dbg( "inside vRaiseMsi address =%p, data=%d\n",
             pMsiInfo->addr, pMsiInfo->data );

    pStats->irq_mux_tx_msi_cnt++;
    dmb();
}

void vLa9310IrqMuxIrq()
{
    struct la9310_irq_evt_info * pEvtInfo = &pLa9310Info->evt_info;
    struct la9310_hif * pHif = pLa9310Info->pHif;
    struct la9310_evt_hdlr * pLa9310EvtHdlr;
    struct irq_evt_regs * pIrqEvtRegs;
    uint32_t evts_clrd, pending_evt;
    struct la9310_stats * pStats;
    int i = 0;

    pIrqEvtRegs = &pHif->irq_evt_regs;
    pStats = pLa9310Info->stats;

    disable_irq();

    evts_clrd = pIrqEvtRegs->irq_evt_clr;

    dmb();

    /*XXX: Looping looks ugly but for first verion it is okay. Afterwards
     * we'll try to go for ffs()
     */
    for( i = 0; i < LA9310_EVT_MAX; i++ )
    {
        if( !( ( 1 << i ) & evts_clrd ) )
        {
            continue;
        }

        pLa9310EvtHdlr = &pEvtInfo->phdlr_tbl[ i ];

        if( pLa9310EvtHdlr->unmask )
        {
            pLa9310EvtHdlr->unmask( pLa9310Info, i, pLa9310EvtHdlr->cookie );
        }

        pIrqEvtRegs->irq_evt_status &= ~( 1 << i );
        pStats->irq_evt_cleared++;
    }

    pStats->irq_mux_rx_msi_cnt++;
    pending_evt = pIrqEvtRegs->irq_evt_status;
    enable_irq();

    if( pending_evt )
    {
        vRaiseMsi( pLa9310Info, LA9310_IRQ_MUX_MSI );
    }
}

void La9310MSG_1_IRQHandler( void )
{
    struct la9310_msg_unit * pMsgUnit;
    uint32_t msir;

    pMsgUnit = &pLa9310Info->msg_unit[ LA9310_MSG_UNIT_1 ];

    msir = IN_32( &pMsgUnit->msir );

    /*IRQ MUX*/
    if( ( msir & BITMASK( LA9310_IRQ_MUX_MSG_UNIT_BIT ) ) )
    {
        vLa9310IrqMuxIrq();
    }
#ifdef TURN_ON_HOST_MODE
#ifdef __RFIC
    /* RFIC SW CMD */
    if( ( msir & BITMASK( LA9310_RF_SW_CMD_MSG_UNIT_BIT ) ) )
    {
        vRficSwCmdIrq( pLa9310Info->pRficDev );
    }
#endif
#endif

    NVIC_ClearPendingIRQ( IRQ_MSG1 );
    #if ARM_ERRATUM_838869
        dsb();
    #endif
}

void La9310MSG_3_IRQHandler( void )
{
    log_isr( "%s: MSG3 IRQ\n\r", __func__ );
    NVIC_ClearPendingIRQ( IRQ_MSG3 );
    #if ARM_ERRATUM_838869
        dsb();
    #endif
}


int iLa9310RaiseIrqEvt( struct la9310_info * pla9310Info,
                        la9310_irq_evt_bits_t evt_bit )
{
    struct la9310_irq_evt_info * pEvtInfo = &pLa9310Info->evt_info;
    int irc = 0;
    uint32_t evt_mask;
    struct la9310_hif * pHif = pLa9310Info->pHif;
    struct la9310_evt_hdlr * pLa9310EvtHdlr;
    struct irq_evt_regs * pIrqEvtRegs;
    struct la9310_stats * pStats;

    if( !pEvtInfo->phdlr_tbl )
    {
        log_err( "%s: Event handler table NULL \n\r", __func__ );
        irc = -pdFREERTOS_ERRNO_EINVAL;
        goto out;
    }

    if( evt_bit >= LA9310_EVT_MAX )
    {
        log_err( "%s: Invalid event %d\n\r", __func__, evt_bit );
        irc = -pdFREERTOS_ERRNO_EINVAL;
        goto out;
    }

    evt_mask = ( 1 << evt_bit );
    pIrqEvtRegs = &pHif->irq_evt_regs;
    pStats = pla9310Info->stats;
    pLa9310EvtHdlr = &pEvtInfo->phdlr_tbl[ evt_bit ];


    dmb();

    /*Host has told us not to bother it with this event! It has more important
     * work to do
     */
    if( !( pIrqEvtRegs->irq_evt_en & evt_mask ) )
    {
        pStats->disabled_evt_try_cnt++;
        goto out;
    }

    dmb();

    /* Already raised this Evt, have some patience let Host breath and process */
    if( pIrqEvtRegs->irq_evt_status & evt_mask )
    {
        goto out;
    }

    disable_irq();

    if( pLa9310EvtHdlr->mask )
    {
        pLa9310EvtHdlr->mask( pLa9310Info, evt_bit,
                              pLa9310EvtHdlr->cookie );
    }

    pIrqEvtRegs->irq_evt_status |= evt_mask;
    log_dbg( "%s: irq_evt_status %#x\n\r", __func__,
             pIrqEvtRegs->irq_evt_status );
    dmb();

    vRaiseMsi( pLa9310Info, LA9310_IRQ_MUX_MSI );
    pStats->irq_evt_raised++;

    /*
     * This test code is used to raise one MSI interrupt
     * on each line (from 1-7, as 0 is already used by IRQ_MUX)
     *
     * for (i = 0; i < 8 ; i++)
     * {
     *  log_info("Raising MSI %d\n",i);
     *  vRaiseMsi(pLa9310Info, i);
     * }
     */

    enable_irq();

out:
    return irc;
}

static void vLa9310UpdateEvtHif( struct la9310_irq_evt_info * pEvtInfo,
                                 struct la9310_evt_hdlr * pLa9310EvtHdlr,
                                 la9310_irq_evt_bits_t evt_bit,
                                 struct la9310_hif * pHif )
{
    struct irq_evt_regs * pIrqEvtRegs;

    pIrqEvtRegs = &pHif->irq_evt_regs;

    switch( pLa9310EvtHdlr->type )
    {
        case LA9310_EVT_TYPE_VSPA:
            pIrqEvtRegs->vspa_evt_mask |= ( 1 << evt_bit );
            log_dbg( "%s: HIF update evt vspa_evt_mask %#x\n\r",
                     __func__, pIrqEvtRegs->vspa_evt_mask );
            break;

        case LA9310_EVT_TYPE_IPC:
            pIrqEvtRegs->ipc_evt_mask |= ( 1 << evt_bit );
            log_dbg( "%s: HIF update evt ipc_evt_mask %#x\n\r",
                     __func__, pIrqEvtRegs->ipc_evt_mask );
            break;

        case LA9310_EVT_TYPE_TEST:
            pIrqEvtRegs->test_evt_mask |= ( 1 << evt_bit );
            log_dbg( "%s: HIF update evt test_evt_mask %#x\r\n",
                     __func__, pIrqEvtRegs->test_evt_mask );
            break;

        default:
            log_dbg( "%s: unsupported type %d\n\r", __func__,
                     pLa9310EvtHdlr->type );
            return;
    }

    log_info( "%s: HIF update evtcnt %d, type %d, irq_evt_cfg %#x\n\r",
              __func__, pEvtInfo->ievt_count, pLa9310EvtHdlr->type,
              pIrqEvtRegs->irq_evt_cfg );
}

int iLa9310RegisterEvt( struct la9310_info * pla9310Info,
                        la9310_irq_evt_bits_t evt_bit,
                        struct la9310_evt_hdlr * pLa9310EvtHdlr )
{
    struct la9310_irq_evt_info * pEvtInfo = &pLa9310Info->evt_info;
    int irc = 0;
    struct la9310_hif * pHif = pLa9310Info->pHif;
    struct la9310_evt_hdlr * pLa9310EvtHdlrTbl;

    pLa9310EvtHdlrTbl = pEvtInfo->phdlr_tbl;

    if( !pLa9310EvtHdlrTbl )
    {
        log_err( "%s: Evt handler table is NULL\n\r", __func__ );
        irc = -pdFREERTOS_ERRNO_ENOENT;
        goto out;
    }

    if( evt_bit >= LA9310_EVT_MAX )
    {
        log_err( "%s: Invalid event %d\n\r", __func__, evt_bit );
        irc = -pdFREERTOS_ERRNO_EINVAL;
        goto out;
    }

    pLa9310EvtHdlrTbl[ evt_bit ].type = pLa9310EvtHdlr->type;
    pLa9310EvtHdlrTbl[ evt_bit ].mask = pLa9310EvtHdlr->mask;
    pLa9310EvtHdlrTbl[ evt_bit ].unmask = pLa9310EvtHdlr->unmask;

    pEvtInfo->ievt_count++;
    pEvtInfo->ievt_en_mask |= ( 1 << evt_bit );
    vLa9310UpdateEvtHif( pEvtInfo, pLa9310EvtHdlr, evt_bit, pHif );

    log_info( "%s: Registered evt %d, mask %#x, unmask %#x\n\r",
              __func__, evt_bit, pLa9310EvtHdlr->mask, pLa9310EvtHdlr->unmask );
out:
    return irc;
}

int iLa9310IRQInit( struct la9310_info * pLa9310Info )
{
    struct la9310_irq_evt_info * pEvtInfo = &pLa9310Info->evt_info;
    int irc = 0, isize;
    struct irq_evt_regs * pIrqEvtRegs;
    struct la9310_hif * pHif = pLa9310Info->pHif;

    pIrqEvtRegs = &pHif->irq_evt_regs;

    /* Initialize the IRQ Event Handler MUX */
    isize = LA9310_EVT_MAX * sizeof( struct la9310_evt_hdlr );
    pEvtInfo->phdlr_tbl = pvPortMalloc( isize );

    if( !pEvtInfo->phdlr_tbl )
    {
        log_err( "%s:Failed to allocate evt handler table\n\r",
                 __func__ );
        irc = -pdFREERTOS_ERRNO_ENOMEM;
        goto out;
    }

    log_dbg( "%s:Allocated evt handler table %#x\n\r", __func__,
             pEvtInfo->phdlr_tbl );
    memset( pEvtInfo->phdlr_tbl, 0, isize );

    /* Initialize HIF with number of Evts */
    LA9310_EVT_SET_EVT_CFG( pIrqEvtRegs, 1, IRQ_EVT_LAST_BIT );
    log_info( "%s:Initialized IRQ EVT mux, irq_evt_cfg %#x\n\r", __func__,
              pIrqEvtRegs->irq_evt_cfg );

    NVIC_SetPriority( IRQ_MSG1, MSG1_IRQ_PRIORITY );
    /*Initialize Message unit IRQ for Host clear notification */
    NVIC_EnableIRQ( IRQ_MSG1 );
out:
    return irc;
}
