/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2021 NXP
 */

void vUDelay( unsigned int ulUS )
{
    __asm volatile
    (
        /* 
         * 245.76(CPU Frequency)/5(Cycle count consumed by below loop) = ~49
         * sub = 1 cycle, cmp = 1 cycle , bne = 3 cycle(if branch taken)
         * */
        "mov r1, #49    \n\t"
        "mul r0, r0, r1 \n\t"
        "loop:          \n\t"
        "sub r0, #1     \n\t"
        "cmp r0, #0     \n\t"
        "bne loop       \n\t"
    );
}
