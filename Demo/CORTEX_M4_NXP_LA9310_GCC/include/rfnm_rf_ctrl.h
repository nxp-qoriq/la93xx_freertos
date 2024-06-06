/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2024 NXP
 */

#ifndef __RFNM_RF_CTRL_H__
#define __RFNM_RF_CTRL_H__


#define PHYTIMER_500_US_61p44 	30720
#define GPT3_CORRECTION_FACTOR_500US 16

/* RF control structure */
typedef struct {
	uint32_t target_phytimer_ts;
	uint32_t issued_phytimer_ts;
	// GPT3 is used as TTI trigger inside the i.MX. To enable it, set 
	// tti_period_ts > 0. TTI will then trigger at the same time as
	// target_phytimer_ts. tti_period_ts > 0 should only be sent once,
	// unless you need to change the period, send tti_period_ts = 0
	// to disable the interrupt, set tti_period_ts = 1
	uint32_t tti_period_ts;
	uint32_t mode;
} rf_ctrl_s;

void switch_rf(uint32_t mode);
void tti_trigger(uint32_t target_ts, uint32_t period);
void tti_stop();

extern volatile rf_ctrl_s rf_ctrl;
#endif
