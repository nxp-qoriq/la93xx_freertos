/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2017, 2021 NXP
 */
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <string.h>
#include "debug_console.h"
#include "la9310_gpio.h"
#include "io.h"

gpio_port_t gpioports[ 1 ];

static uint32_t uGetportaddr( void )
{
    return GPIO_BASE_ADDR;
}

static int8_t iIsPinSupported( uint8_t pin )
{
    if( ( pin <= GPIO_PIN_LIST_12 ) || ( ( pin >= GPIO_PIN_LIST_16 ) && ( pin <= GPIO_PIN_LIST_19 ) ) )
    {
        return SUCCESS;
    }
    else
    {
        return -GPIO_PIN_NOT_SUPPORTED;
    }
}

void iGpioInputBufferEnable( uint8_t pin )
{
    gpio_port_t * gpioport = ( struct gpio_port * ) ( uGetportaddr() );
    uint32_t * p_reg = ( uint32_t * ) gpioport;

    pin = 31 - pin;
    setbits_le32( ( uint32_t ) ( &p_reg[ GPIO_IBE ] ), BITS( pin ) );
}

int32_t iGpioSetData( uint8_t pin,
                      uint32_t ulVal )
{
    uint32_t ulDir;

    if( iIsPinSupported( pin ) )
    {
        log_err( "iGpioSetData - GPIO PIN (%d)\
				is not supported \n\n\r", pin );
        return -GPIO_PIN_NOT_SUPPORTED;
    }

    iGpioInputBufferEnable( pin );
    gpio_port_t * gpioport = ( struct gpio_port * ) ( uGetportaddr() );
    uint32_t * p_reg = ( uint32_t * ) gpioport;

    ulDir = in_le32( ( uint32_t ) ( &p_reg[ GPIO_DIR ] ) );

    pin = 31 - pin;

    if( ulDir & BITS( pin ) )
    {
        if( ulVal )
        {
            setbits_le32( ( uint32_t ) ( &p_reg[ GPIO_DAT ] ), BITS( pin ) );
        }
        else
        {
            clrbits_le32( ( uint32_t ) ( &p_reg[ GPIO_DAT ] ), BITS( pin ) );
        }
    }
    else
    {
        log_err( "iGpioSetData - Pin is INPUT mode\n\n\r" );
        return -GPIO_TYPE_NOT_SET;
    }

    return SUCCESS;
}

int32_t iGpioGetData( uint8_t pin )
{
    uint32_t ulVal;

    if( iIsPinSupported( pin ) )
    {
        log_err( "iGpioGetData-GPIO PIN (%d) is\
					not supported\n\n\r", pin );
        return -GPIO_PIN_NOT_SUPPORTED;
    }

    gpio_port_t * gpioport = ( struct gpio_port * ) ( uGetportaddr() );
    uint32_t * p_reg = ( uint32_t * ) gpioport;

    ulVal = in_le32( ( uint32_t ) ( &p_reg[ GPIO_DAT ] ) );
    pin = 31 - pin;

    if( ulVal & BITS( pin ) )
    {
        return 1;
    }
    else
    {
        return 0;
    }
}


int32_t iGpioInit( uint8_t pin,
                   gpio_type_t type,
                   bool openDrain )
{
    if( iIsPinSupported( pin ) )
    {
        log_err( "iGpioInit:GPIO PIN (%d) is not supported\n\n\r", pin );
        return -GPIO_PIN_NOT_SUPPORTED;
    }

    gpio_port_t * gpioport = ( struct gpio_port * ) ( uGetportaddr() );
    uint32_t * p_reg = ( uint32_t * ) gpioport;

    pin = 31 - pin;

    if( type == input )
    {
        clrbits_le32( ( uint32_t ) ( &p_reg[ GPIO_DIR ] ), BITS( pin ) );
        setbits_le32( ( uint32_t ) ( &p_reg[ GPIO_IBE ] ), BITS( pin ) );
    }
    else if( !openDrain )
    {
        setbits_le32( ( uint32_t ) ( &p_reg[ GPIO_DIR ] ), BITS( pin ) );
        setbits_le32( ( uint32_t ) ( &p_reg[ GPIO_IBE ] ), BITS( pin ) );
    }
    else
    {
        setbits_le32( ( uint32_t ) ( &p_reg[ GPIO_ODR ] ), BITS( pin ) );
    }

    return SUCCESS;
}

BaseType_t iGpioSetBit( uint8_t pin,
                        uint8_t uVal )
{
    if( iIsPinSupported( pin ) )
    {
        log_err( "iGpioSetBit - GPIO PIN (%d)\
				is not supported \n\n\r", pin );
        return pdFAIL;
    }

    gpio_port_t * gpioport = ( struct gpio_port * ) ( uGetportaddr() );
    uint32_t * p_reg = ( uint32_t * ) gpioport;

    if( uVal )
    {
        setbits_le32( ( uint32_t ) ( &p_reg[ GPIO_DAT ] ), BITS( pin ) );
    }
    else
    {
        clrbits_le32( ( uint32_t ) ( &p_reg[ GPIO_DAT ] ), BITS( pin ) );
    }

    return pdPASS;
}
