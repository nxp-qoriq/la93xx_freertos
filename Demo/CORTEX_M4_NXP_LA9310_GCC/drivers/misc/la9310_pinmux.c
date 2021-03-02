/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2017, 2021 NXP
 */
#include "immap.h"
#include "core_cm4.h"
#include "io.h"
#include "la9310_pinmux.h"

/* Set multiple GPIO pin mux setting */
void vGpioSetPinMux( uint32_t pin_mask,
                     uint8_t pin_mode )
{
    if( pin_mode == SET_MUX_GPIO_MODE )
    {
        setbits_le32( PMUXCR_BASE_ADDR, pin_mask );
    }
    else
    {
        clrbits_le32( PMUXCR_BASE_ADDR, pin_mask );
    }
}

/* Set single GPIO pin mux setting */
void vGpioSetPinMuxSingle( uint32_t pin_num,
                           uint8_t pin_mode )
{
    if( pin_mode == SET_MUX_GPIO_MODE )
    {
        setbits_le32( PMUXCR_BASE_ADDR, ( 1 << pin_num ) );
    }
    else
    {
        clrbits_le32( PMUXCR_BASE_ADDR, ( 1 << pin_num ) );
    }
}
