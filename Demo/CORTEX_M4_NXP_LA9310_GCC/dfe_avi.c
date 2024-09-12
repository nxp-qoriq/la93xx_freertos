/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2024 NXP
 */

#include "dfe_avi.h"
#include "la9310_avi.h"
#include "la9310_error_codes.h"

int vDFEMbxSend(struct dfe_mbox *h2v, uint32_t mbox_id)
{
	struct avi_hndlr *avih;

	avih = iLa9310AviHandle();
	if (!avih) {
		log_err("%s: AVI not initialized!\r\n", __func__);
		return -AVI_IPC_INIT_NOT_DONE;
	}

	return iLa9310AviHostSendMboxToVspa(avih, h2v->msb, h2v->lsb, mbox_id);
}

int vDFEMbxReceive(struct dfe_mbox *v2h, uint32_t mbox_id, uint32_t timeout_ms)
{
	struct avi_hndlr *avih;
	struct avi_mbox *vspa_mbox;
	uint32_t ret = 0;
	TickType_t start, crt;
	bool blocking = !timeout_ms;

	avih = iLa9310AviHandle();
	if (!avih) {
		log_err("AVI not initialized!\n");
		return -AVI_IPC_INIT_NOT_DONE;
	}

	vspa_mbox = (struct avi_mbox *)v2h;
	start = xTaskGetTickCount();
	crt = start;
	while (blocking || (crt - start < pdMS_TO_TICKS(timeout_ms))) {
		ret = iLa9310AviHostRecvMboxFromVspa(avih, vspa_mbox, mbox_id);
		if (ret == 0)
			break;
		crt = xTaskGetTickCount();
	}

	if (ret != 0)
		log_err("%s: VSPA timeout\r\n", __func__);

	return ret;
}
