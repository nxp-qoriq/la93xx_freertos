/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2017, 2021 NXP
 */

#ifndef LA9310_I2CAPI_H
#define LA9310_I2CAPI_H

#define LA9310_I2C_DEV_OFFSET_LEN_0_BYTE    0
#define LA9310_I2C_DEV_OFFSET_LEN_1_BYTE    1
#define LA9310_I2C_DEV_OFFSET_LEN_2_BYTE    2
#define LA9310_I2C_DEV_OFFSET_LEN_3_BYTE    3
#define LA9310_I2C_DEV_OFFSET_LEN_4_BYTE    4

#ifndef LA9310_I2C_FREQ
    #define LA9310_I2C_FREQ                 100000
#endif

#include <la9310_i2c.h>

/*****************************************************************************
* i@La9310_I2C_Init
*
* Initialize the I2C environment for application.
*
*
* ulI2C_Regs_P	- [IN][M]  address of I2C module.
*
* ulSys_Freq	- [IN][M]  frequency at which system works.
*			It is in HZ.
*
* ulI2C_Bus_Freq	- [IN][M]  frequency at which I2C Bus
*			works.It is in HZ.
*
* Return Value -
*	SUCCESS -	on success
*	ERROR -		on error
*
*****************************************************************************/

int iLa9310_I2C_Init( uint32_t ulI2C_Regs_P,
                      uint32_t ulSys_Freq,
                      uint32_t ulI2C_Bus_Freq );


/*****************************************************************************
* @iLa9310_I2C_Disable
*
* Disable the I2C environment for application.
*
* ulI2C_Regs_P	- [IN][M] address of I2C Module.
*
* Return Value -
*	SUCCESS -	on success
*	ERROR -		on error
*
*****************************************************************************/

int iLa9310_I2C_Disable( uint32_t ulI2C_Regs_P );


/*****************************************************************************
 *@iLa9310_I2C_Read
 *
 * For reading the data from specific device(e.g EEPROM) based on the address
 * get in parameter and write data to specific address mentioned in parameter.
 *
 * ulI2C_Regs_P	- [IN][M] address of I2C Module.
 *
 * ucDev_Addr      - [IN][M] address of the device from where data needs to
 *			read(slave address)
 *
 * ulDev_Offset    - [IN][M] offset in the device
 *
 * ucDev_Offset_Len    - [IN][M] no. of bytes of offset length .It can be any
 *                     value which is defined above in macros.
 *                     Pass above macros name as a parameter
 *
 *
 * *pDst          - [IN][M] pointer to the destination where data needs to copy
 *
 * ulD_Len        -  [IN] [M] length of the data needs to read
 *
 * Return Value -
 *      No. of bytes read - on success
 *      Error - on error
 ****************************************************************************/

int iLa9310_I2C_Read( uint32_t ulI2C_Regs_P,
                      uint8_t ucDev_Addr,
                      uint32_t ulDev_Offset,
                      uint8_t ucDev_Offset_Len,
                      uint8_t * pDst,
                      uint32_t ulD_Len );


/*****************************************************************************
 *@iLa9310_I2C_Write
 *
 * For writing the data from specific location to specific device(e.g EEPROM)
 *
 * ulI2C_Regs_P	- [IN][M] address of I2C Module.
 *
 * ucDev_Addr      - [IN][M] address of the device to where data needs to write
 *
 * ulDev_Offset    - [IN][M] offset in the device
 *
 * ucDev_Offset_Len    - [IN][M] no. of bytes of offset length .It can be any
 *                     value which is defined above in macros.
 *                     Pass above macros name as a parameter.
 *
 *
 * *psrc         - [IN][M] pointer to the src from where data needs to read
 *
 * ulD_Len        -  [IN] [M] length of the data needs to write
 *
 * Return Value -
 *      No. of bytes write - on success
 *      Error - on error
 ****************************************************************************/

int iLa9310_I2C_Write( uint32_t ulI2C_Regs_P,
                       uint8_t ucDev_Addr,
                       uint32_t ulDev_Offset,
                       uint8_t ucDev_Offset_Len,
                       uint8_t * psrc,
                       uint32_t ulD_Len );

#endif /* _LA9310_I2C_H_ */
