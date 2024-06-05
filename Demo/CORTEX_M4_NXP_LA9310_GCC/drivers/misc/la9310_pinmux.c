/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2017, 2021, 2024 NXP
 */
#include "immap.h"
#include "core_cm4.h"
#include "io.h"
#include "la9310_pinmux.h"


/* return the PMUX register value */
uint32_t readPMUX(int index)
{
  struct ccsr_pmux * pxPmuxCR = ( void * ) ( PMUXCR_BASE_ADDR );
  uint32_t ulPmux;

  if (0 == index)
    ulPmux = IN_32( &pxPmuxCR->ulPmuxCR0 );
  else
    ulPmux = IN_32( &pxPmuxCR->ulPmuxCR1 );

  return ulPmux;
}

/* Set PMUX for the pmux_index and bit_index */
uint32_t setPMUX(int pmux_index, int bit_index)
{
  struct ccsr_pmux * pxPmuxCR = ( void * ) ( PMUXCR_BASE_ADDR );
  uint32_t ulPmux;

  if (0 == pmux_index)
    ulPmux = IN_32( &pxPmuxCR->ulPmuxCR0 );
  else
    ulPmux = IN_32( &pxPmuxCR->ulPmuxCR1 );

  ulPmux |= (1 << bit_index);

  if (0 == pmux_index)
    OUT_32( &pxPmuxCR->ulPmuxCR0, ulPmux );
  else
    OUT_32( &pxPmuxCR->ulPmuxCR1, ulPmux );

  return ulPmux;
}

/* Clear PMUX for the pmux_index and bit_index */
uint32_t clearPMUX(int pmux_index, int bit_index)
{
  struct ccsr_pmux * pxPmuxCR = ( void * ) ( PMUXCR_BASE_ADDR );
  uint32_t ulPmux;

  if (0 == pmux_index)
    ulPmux = IN_32( &pxPmuxCR->ulPmuxCR0 );
  else
    ulPmux = IN_32( &pxPmuxCR->ulPmuxCR1 );

  ulPmux &= (0 << bit_index);

  if (0 == pmux_index)
    OUT_32( &pxPmuxCR->ulPmuxCR0, ulPmux );
  else
    OUT_32( &pxPmuxCR->ulPmuxCR1, ulPmux );

  return ulPmux;
}

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
