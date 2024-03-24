/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2024 NXP
 */

#include "FreeRTOS.h"
#include "common.h"
#include "core_cm4.h"
#include "phytimer.h"

#include "rfnm_rf_ctrl.h"

volatile rf_ctrl_s rf_ctrl __attribute__((section(".rfctrl")));

/* routine for returning the current timer value */
inline uint32_t uGetPhyTimerTimestamp(void)
{
	return ulPhyTimerCapture( PHY_TIMER_COMPARATOR_COUNT - 1 );
}

void switch_rf(uint32_t mode)
{
	uint32_t ts = uGetPhyTimerTimestamp();

	rf_ctrl.mode = mode;
	rf_ctrl.issued_phytimer_ts = ts + PHYTIMER_500_US_61p44;
	rf_ctrl.target_phytimer_ts = ts + PHYTIMER_500_US_61p44 * 2;
	rf_ctrl.tti_period_ts = PHYTIMER_500_US_61p44 * 2;

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
