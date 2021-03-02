/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright 2021 NXP
 */
#ifndef __RFIC_AVI_CTRL_H
#define __RFIC_AVI_CTRL_H

#include <la9310_avi.h>

struct la9310_mbox_ctrl {
    uint16_t rcvr:2;
    uint16_t bandwidth:2;
    uint16_t start_stop:1;
    uint16_t tx_rx:1;
    uint16_t unused:2;
    uint16_t op_code:8;
}__attribute__ ((packed));

struct la9310_mbox_status {
    uint32_t ack:1;
    uint32_t err_code:8;
    uint32_t unused:23;
}__attribute__ ((packed));

struct la9310_mbox_h2v {
    struct la9310_mbox_ctrl ctrl;
    uint16_t msbl16;
    uint32_t lsb32;
};

struct la9310_mbox_v2h {
    uint32_t msb32;
    struct la9310_mbox_status status;
};

void vLa9310MbxSend(struct la9310_mbox_h2v *mbox_h2v);
BaseType_t vLa9310MbxReceive(struct la9310_mbox_v2h *mbox_v2h);

#endif /* __RFIC_AVI_CTRL_H */
