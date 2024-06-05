/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2021, 2024 NXP
 */
#ifndef __LA9310_PINMUX_H__
#define __LA9310_PINMUX_H__

#include <types.h>

#define SET_MUX_GPIO_MODE        1
#define SET_MUX_NON_GPIO_MODE    0

/* return the PMUX register value */
uint32_t readPMUX( int index );

/* Set PMUX for the pmux_index and bit_index */
uint32_t setPMUX( int pmux_index, int bit_index );

/* Clear PMUX for the pmux_index and bit_index */
uint32_t clearPMUX( int pmux_index, int bit_index );

/* Set multiple GPIO pin mux setting */
void vGpioSetPinMux( uint32_t pin_mask, uint8_t pin_mode );

/* Set single GPIO pin mux setting */
void vGpioSetPinMuxSingle( uint32_t pin_num, uint8_t pin_mode );

#endif /* __LA9310_PINMUX_H__ */
