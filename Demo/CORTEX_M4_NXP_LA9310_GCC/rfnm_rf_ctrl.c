/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2024 NXP
 */

#include "FreeRTOS.h"
#include "common.h"
#include "core_cm4.h"
#include "phytimer.h"

#include "rfnm_rf_ctrl.h"
#include "la9310_dcs.h"

volatile rf_ctrl_s rf_ctrl __attribute__((section(".rfctrl")));

void switch_rf(uint32_t mode)
{
	uint32_t ts = uGetPhyTimerTimestamp();
	uint32_t ulClkCtrlRegVal = IN_32( ( uint32_t * ) ( ADC_DAC_CLKCFG ) );
	uint32_t half_rate = (ulClkCtrlRegVal & TX_Half_Freq_SET) >> 16 ;

	rf_ctrl.mode = mode;
	rf_ctrl.issued_phytimer_ts = ts + PHYTIMER_500_US_61p44;
	rf_ctrl.target_phytimer_ts = ts + PHYTIMER_500_US_61p44 * 2;

	if (half_rate)
		rf_ctrl.tti_period_ts = PHYTIMER_500_US_61p44 - GPT3_CORRECTION_FACTOR_500US;
	else
		rf_ctrl.tti_period_ts = (PHYTIMER_500_US_61p44 * 2) - GPT3_CORRECTION_FACTOR_500US;

#ifndef RFNM_CHECK_ALIGNMENT

vPhyTimerComparatorConfig( PHY_TIMER_COMP_RFCTL_5,
					PHY_TIMER_COMPARATOR_CLEAR_INT | PHY_TIMER_COMPARATOR_CROSS_TRIG,
					ePhyTimerComparatorOutToggle,
					rf_ctrl.issued_phytimer_ts);
#else

	// use to confirm alignment of my work gpt vs phy timer
	// recompile m4 code to trigger on rising edge of gpt capture only
	vPhyTimerComparatorConfig( PHY_TIMER_COMP_RFCTL_5,
					PHY_TIMER_COMPARATOR_CLEAR_INT | PHY_TIMER_COMPARATOR_CROSS_TRIG,
					ePhyTimerComparatorOut1,
					rf_ctrl.issued_phytimer_ts);


	while(uGetPhyTimerTimestamp() < rf_ctrl.issued_phytimer_ts) { }


	vPhyTimerComparatorConfig( PHY_TIMER_COMP_RFCTL_5,
					PHY_TIMER_COMPARATOR_CLEAR_INT | PHY_TIMER_COMPARATOR_CROSS_TRIG,
					ePhyTimerComparatorOut0,
					rf_ctrl.target_phytimer_ts);
#endif

	PRINTF("Switch RF (mode = %#x, issued = %#x, target = %#x ---> %p\n",
			rf_ctrl.mode,
			rf_ctrl.issued_phytimer_ts,
			rf_ctrl.target_phytimer_ts, &rf_ctrl);
}

void tti_trigger(uint32_t target_ts, uint32_t period)
{
	rf_ctrl.mode = 0;
	rf_ctrl.target_phytimer_ts = target_ts;
	rf_ctrl.tti_period_ts = period - GPT3_CORRECTION_FACTOR_500US;

	vPhyTimerComparatorConfig( PHY_TIMER_COMP_RFCTL_5,
					PHY_TIMER_COMPARATOR_CLEAR_INT | PHY_TIMER_COMPARATOR_CROSS_TRIG,
					ePhyTimerComparatorOutToggle,
					(rf_ctrl.issued_phytimer_ts = (uGetPhyTimerTimestamp() + 50)));
}

void tti_stop()
{
	rf_ctrl.mode = 0;
	rf_ctrl.target_phytimer_ts = 0;
	rf_ctrl.tti_period_ts = 1;

	vPhyTimerComparatorConfig( PHY_TIMER_COMP_RFCTL_5,
					PHY_TIMER_COMPARATOR_CLEAR_INT | PHY_TIMER_COMPARATOR_CROSS_TRIG,
					ePhyTimerComparatorOutToggle,
					(rf_ctrl.issued_phytimer_ts = (uGetPhyTimerTimestamp() + 50)));
}
