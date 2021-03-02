/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2017, 2021 NXP
 */

#ifndef _LA9310_GPIO_H_
#define _LA9310_GPIO_H_

#include <stdint.h>
#include "la9310_error_codes.h"
#include <stdbool.h>

#define GPIO_BASE_ADDR      ( 0x2300000 + 0x40000000 ) /* base address (specified
                                                        * in CCSR address space) plus the module base address */

#define GPIO_PIN_LIST_12    12                         /* 0 - 12 total 13 pins */
#define GPIO_PIN_LIST_16    16                         /* 16 - 19 total 4 pins */
#define GPIO_PIN_LIST_19    19                         /* 16 - 19 total 4 pins */
#define GPIO_PIN_I2C_BOOT_RESET 18

#define BITS( x )           ( 1 << x )

/**************************************************************************
 *                        Abbreviations
 ***************************************************************************
 * IN    - Input Parameter
 * OUT   - Output Paramater
 * M     - Mandatory
 * O     - Optional
 *
 ***************************************************************************/

typedef enum gpioType
{
    input,
    output,
} gpio_type_t;

typedef enum gpio_reg
{
    GPIO_DIR = 0,
    GPIO_ODR = 1,
    GPIO_DAT = 2,
    GPIO_IER = 3,
    GPIO_IMR = 4,
    GPIO_ICR = 5,
    GPIO_IBE = 6,
} gpio_regs_t;

typedef struct gpio_port
{
    uint32_t gpdir;
    uint32_t gpodr;
    uint32_t gpdat;
    uint32_t gpier;
    uint32_t gpimr;
    uint32_t gpicr;
    uint32_t gpibe;
} gpio_port_t;

/***************************************************************************
* @iGpioInit
*
* Initializes GPIO pins with the given mode (input/output).
*
* pin		-  [IN] [M] Gpio number
*
* type		-  [IN] [M] type of gpio (input / output)
*
* openDrain     -  [IN] [M] it is flag to set pin as openDrain. If the Pin mode
*			is INPUT this case is not aplicable.
*
* Return Value  -
*	SUCCESS          No error
*	Negative value   error (see "la9310_error_codes.h" for reference)
*
* NOTE: This API will be called to set direction of the pin.
*
***************************************************************************/
int32_t iGpioInit( uint8_t pin,
                   gpio_type_t type,
                   bool openDrain );


/***************************************************************************
* @iGpioGetData
*
* Get the Data in the pin provided direction of this pin is output.
*
* pin		-  [IN] [M] Gpio number
*
*
* Return Value  -
*	Success		No error
*	Negative value	error (see "la9310_error_codes.h" for reference)
*
*
***************************************************************************/
int32_t iGpioGetData( uint8_t pin );


/***************************************************************************
* @iGpioSetData
*
* Set Data (0 or 1) in the pin provided direction of that pin is output.
*
* pin		- [IN] [M] Gpio number
*
* ulVal		- [IN] [M] value to be set in the pin (1 or 0)
*
*
* Return Value  -
*	SUCCESS     No Error
*	Negative value   error (see "la9310_error_codes.h" for reference)
*
*
***************************************************************************/
int32_t iGpioSetData( uint8_t pin,
                      uint32_t ulVal );

/***************************************************************************
* @iGpioSetBit
*
* Set Bit (0 or 1) in the pin provided for non gpio mode.
*
* pin		- [IN] [M] Gpio number
*
* ulVal		- [IN] [M] value to be set in the pin (1 or 0)
*
*
* Return Value  -
*	pdPASS     No Error
*	pdFAIL value   error
*
*
***************************************************************************/
BaseType_t iGpioSetBit( uint8_t pin,
                        uint8_t uVal );

/**************************************************************************
* @iGpioInputBufferEnable
*
* GPIO_IBE resister is set 1 for corespondence pin to get value in GPIO_DAT
* register.
*
*
*
*
* pin		- [IN] [M] Gpio number
*
**************************************************************************/

void iGpioInputBufferEnable( uint8_t pin );
#endif /* _LA9310_GPIO_H_ */
