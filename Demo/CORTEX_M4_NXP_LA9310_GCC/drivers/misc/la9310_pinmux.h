/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2021 NXP
 */
#ifndef __LA9310_PINMUX_H__
#define __LA9310_PINMUX_H__

#include <types.h>

#define SET_MUX_GPIO_MODE        1
#define SET_MUX_NON_GPIO_MODE    0

/* Set multiple GPIO pin mux setting */
void vGpioSetPinMux( uint32_t pin_mask, uint8_t pin_mode );

/* Set single GPIO pin mux setting */
void vGpioSetPinMuxSingle( uint32_t pin_num, uint8_t pin_mode );

#endif /* __LA9310_PINMUX_H__ */
