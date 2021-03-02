/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright 2021 NXP
 */
#include <rfic_avi_ctrl.h>

void vLa9310MbxSend( struct la9310_mbox_h2v *mbox_h2v )
{
    struct avi_hndlr *avihndl = NULL;

    avihndl = iLa9310AviHandle();
    //log_info("\r\n ****H2V: MSB_LSB(Hex):%x::%x\r\n",
    //	(*(uint16_t *)(&mbox_h2v->ctrl)) << 16 |
    //	mbox_h2v->msbl16, mbox_h2v->lsb32);
    if( NULL != avihndl )
    {
        if( 0 != iLa9310AviHostSendMboxToVspa( avihndl, (*(uint16_t *)(&mbox_h2v->ctrl)) << 16 | mbox_h2v->msbl16,
                                mbox_h2v->lsb32, 0 ))
        {
            log_err( "\r\n***ERR: Send Host MBOX 0 Fail\n\r" );
        }
    }
    else
    {
        log_err( "MBox AVIhandler NULL\n\r" );
    }
    return;
}

BaseType_t vLa9310MbxReceive(struct la9310_mbox_v2h *mbox_v2h)
{
    struct avi_hndlr *avihndl = NULL;
    struct avi_mbox vspa_mbox;
    uint32_t retries = 0;

    avihndl = iLa9310AviHandle();
    if( NULL != avihndl )
    {
        while (retries < MAILBOX_VALID_STATUS_RETRIES)
        {
            /* Read VSPA inbox 0 */
            if ( 0 == iLa9310AviHostRecvMboxFromVspa(avihndl, &vspa_mbox, 0 ))
            {
                mbox_v2h->msb32 = vspa_mbox.msb;
                *((uint32_t *)(&mbox_v2h->status))  = vspa_mbox.lsb;
                log_info("\r\n **V2H: MSB_LSB(Hex):%x::%x retries %d\r\n",
                        mbox_v2h->msb32, *((uint32_t *)(&mbox_v2h->status)),retries);
                return pdPASS;
            }
            retries++;
        }
        if (retries == MAILBOX_VALID_STATUS_RETRIES)
            log_err("VSPA timeout\r\n");
    }
    else
    {
        log_err("MBox AVIhandler NULL\n\r");
    }
    return pdFAIL;
}
