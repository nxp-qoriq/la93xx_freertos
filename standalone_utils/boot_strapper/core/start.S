/* SPDX-License-Identifier: (BSD-3-Clause) */
/*
 * Copyright 2022 NXP
 */

	.extern main_init
	.global m4_startup
	
m4_startup:
	cpsid    i
	ldr r0, = __StackTop
	msr msp, r0
	ldr     r0, =main_init
	blx r0

	/*control should nerver return to this point*/
	bl loop	
loop:
	bl loop
	.end
