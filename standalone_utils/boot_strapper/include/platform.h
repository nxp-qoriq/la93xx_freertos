/* SPDX-License-Identifier: (BSD-3-Clause) */
/*
 * Copyright 2022 NXP
 */

#ifndef PLATFORM_H
#define PLATFORM_H

#include "common.h"

#define CONFIG_HANDSHAKE_TIMEOUT	10

/* Run control register and EPU register values */
#define RCPM_CGACRE0_CGC	(0x0 << 0)
#define RCPM_CGACRE0_ICAC	(0x0 << 4)
#define RCPM_CGACRE0_PCAC	(0x1 << 8)
#define EPU_EPSMCR0_ISEL0	(0x0 << 24)
#define EPU_EPSMCR0_ISEL1	(0x0 << 16)
#define EPU_EPSMCR0_ISEL2	(0x0 << 8)
#define EPU_EPSMCR0_ISEL3	(0x26 << 0)
#define EPU_EPECR0_IC0		(0x0 << 30)
#define EPU_EPECR0_IC1		(0x0 << 28)
#define EPU_EPECR0_IC2		(0x0 << 26)
#define EPU_EPECR0_IC3		(0x2 << 24)
#define EPU_EPECR0_IIE0		(0x0 << 15)
#define EPU_EPECR0_IIE1		(0x0 << 14)
#define EPU_EPECR0_IIE2		(0x0 << 13)
#define EPU_EPECR0_IIE3		(0x0 << 12)
#define EPU_EPECR0_CIC		(0x0 << 6)
#define EPU_EPECR0_ICE		(0x0 << 5)
#define EPU_EPECR0_EDE		(0x0 << 4) /*(0x1 << 4)*/
#define EPU_EPECR0_SSE		(0x0 << 2)
#define EPU_EPECR0_STS		(0x0 << 0)

struct mem_range {
	uint32_t start;
	uint32_t end;
};

uint32_t reset_handshake(void);
void rom_reset_hs_comp(void);
uint8_t get_boot_src(void);
bool check_pcie_gen2(void);
void write_error_code_reg(uint32_t err_code);
uint8_t check_valid_addr_range(uint32_t addr, uint32_t size);

/***************************************************************************
 * Function	:	timer_init
 * Arguments	:	timer_wrap - timer wrap count
 * Return	:	Void
 * Description	:	This function initializes the timer with timeout in
 *			sec and enable the SysTick timer. Also initializes
 *			timer wrap count value.
 ***************************************************************************/
static inline void timer_init(uint32_t *timer_wrap)
{
#define MAX_24BIT_COUNTER_VALUE		0xffffff
	/* Initializes SysTick Reload value reg with timeout */
	OUT_32((void *)M4_SYST_RVR, MAX_24BIT_COUNTER_VALUE);

	/* Initializes SysTick Current value reg with timeout */
	OUT_32((void *)M4_SYST_CVR, 0x0);

	/* Enable SysTick timer */
	OUT_32((void *)M4_SYST_CSR, (IN_32((void *)M4_SYST_CSR) | 0x1));

	/* Initialize timer wrap value */
	*timer_wrap = 0x0;
}

/**************************************************************************
 * Function	:	get_timeout
 * Arguments	:	sec - Timeout value in seconds
 *			timer_wrap - timer wrap count
 * Return	:	True or False
 * Description	:	This funtion checks COUNTFLAG bit in M4_SYST_CSR reg
 *			to check if timer counted to 0 or not. As 24 bit
 *			counter can maximum count to 52.4 ms (320 Mhz clock)
 *			so we have to count seconds in terms of timer wrap
 *			arounds. So for 1 second, timer wraps = 19 (approx.).
 ***************************************************************************/
static inline uint8_t get_timeout(uint32_t sec, uint32_t *timer_wrap)
{
#define CSR_COUNTFLAG_MASK	0x00010000
#define CSR_COUNTFLAG_SHIFT	16
#define TIMER_WRAPS_IN_SEC	19		/* Corresponding to 320Mhz */
	if ((IN_32((void *)M4_SYST_CSR) & CSR_COUNTFLAG_MASK) >>
	    CSR_COUNTFLAG_SHIFT)
		(*timer_wrap)++;

	if (*timer_wrap >= (sec * TIMER_WRAPS_IN_SEC))
		return TRUE;
	else
		return FALSE;
}

/***************************************************************************
 * Function	:	timer_stop
 * Arguments	:	void
 * Return	:	void
 * Description	:	Write in the appropriate value in timer register to
			stop the timer.
 ***************************************************************************/
static inline void timer_stop(void)
{
	/* Disable SysTick timer */
	OUT_32((void *)M4_SYST_CSR,
	       (IN_32((void *)M4_SYST_CSR) & 0xfffffffe));

	/* Clear SysTick Reload value reg */
	OUT_32((void *)M4_SYST_RVR, 0x0);

	/* Clear SysTick Current value reg */
	OUT_32((void *)M4_SYST_CVR, 0x0);
}

/***************************************************************************
 * Function	:	get_sysclock
 * Arguments	:	Void
 * Return	:	System clock value
 * Description	:	Returns the system clock value
 ***************************************************************************/
static inline uint32_t get_sysclock(void)
{
	return 100000000; /* 100 MHz frequency */
}

/***************************************************************************
 * Function	:	get_personality
 * Arguments	:	Void
 * Return	:	Platform personality
 * Description	:	Returns platform personality
 ***************************************************************************/
static inline uint32_t get_personality(void)
{
#define SVR_PERSONALITY_MASK	0x00003f00
#define SVR_PERSONALITY_SHIFT	8
	uint32_t svr_val = 0;

	svr_val = IN_32((uint32_t *)(CCSR_DCFG_BASE_ADDR + DCFG_SVR_OFFSET));

	return (svr_val & SVR_PERSONALITY_MASK) >> SVR_PERSONALITY_SHIFT;
}

/***************************************************************************
 * Function	:	get_platform_rev
 * Arguments	:	Void
 * Return	:	Platform revision id
 * Description	:	Returns platform revision id
 ***************************************************************************/
static inline uint8_t get_platform_rev(void)
{
#define SVR_REVISION_MASK	0x000000ff
	uint32_t svr_val = 0;

	svr_val = IN_32((uint32_t *)(CCSR_DCFG_BASE_ADDR + DCFG_SVR_OFFSET));

	return (uint8_t)(svr_val & SVR_REVISION_MASK);
}

#endif /* ifndef PLATFORM_H */
