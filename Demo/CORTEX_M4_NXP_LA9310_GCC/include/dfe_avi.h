/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2024 NXP
 */

#ifndef __DFE_AVI_H__
#define __DFE_AVI_H__

#include <types.h>
#include "FreeRTOS.h"
#include "dfe_vspa_mbox.h"

/* Send message to VSPA mbox_id */
int vDFEMbxSend(struct dfe_mbox *h2v, uint32_t mbox_id);
/*
 * Receive message from VSPA mbox_id and store it in v2h.
 * If timeout_ms is zero, block until a message arrives. Otherwise return after
 * timeout_ms miliseconds if no message is received.
 */
int vDFEMbxReceive(struct dfe_mbox *v2h, uint32_t mbox_id, uint32_t timeout_ms);

#endif /* __NMM_AVI_H__ */
