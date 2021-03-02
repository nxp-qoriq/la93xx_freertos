/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2017, 2021 NXP
 */

#ifndef __LA9310_IRQ_H__
#define __LA9310_IRQ_H__

#include "la9310_host_if.h"

#define LA9310_EVT_MAX        32
#define LA9310_IRQ_MUX_MSI    ( MSI_IRQ_MUX )

#define IPC_IRQ_PRIORITY      3
#define EDMA_IRQ_PRIORITY     3
#define MSG1_IRQ_PRIORITY     3
#define VSPA_IRQ_PRIORITY     3

enum la9310_evt_type
{
    LA9310_EVT_TYPE_UNUSED = 0,
    LA9310_EVT_TYPE_VSPA,
    LA9310_EVT_TYPE_IPC,
    LA9310_EVT_TYPE_TEST,
    LA9310_EVT_TYPE_END,
};

struct la9310_evt_hdlr
{
    enum la9310_evt_type type;
    void * cookie;
    void (* mask) ( struct la9310_info * pLa9310Info,
                    la9310_irq_evt_bits_t evt_bit,
                    void * cookie );
    void (* unmask) ( struct la9310_info * pLa9310Info,
                      la9310_irq_evt_bits_t evt_bit,
                      void * cookie );
};

int iLa9310IRQInit( struct la9310_info * pLa9310Info );
int iLa9310RegisterEvt( struct la9310_info * pla9310Info,
                        la9310_irq_evt_bits_t evt_bit,
                        struct la9310_evt_hdlr * pLa9310EvtHdlr );

int iLa9310RaiseIrqEvt( struct la9310_info * pla9310Info,
                        la9310_irq_evt_bits_t evt_bit );
void vRaiseMsi( struct la9310_info * pla9310Info,
                enum la9310_msi_id msi );
#endif /* ifndef __LA9310_IRQ_H__ */
