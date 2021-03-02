/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright 2017, 2021 NXP
 */

#ifndef LA9310_I2C_H
#define LA9310_I2C_H

#include <common.h>

#define LA9310_IS_CURRENT_MASTER                  1
#define LA9310_ISNOT_CURRENT_MASTER               0

/* Maximum FDR Value allowed is 0xBF (191)
 * 8 bit Value with b11xxxxxxxx as reserved
 */
#define LA9310_I2C_MAX_FDR                        191

#define LA9310_I2C_LOOP_TOTAL_DELAY_US            5000
#define LA9310_I2C_LOOP_COUNT                     100
#define LA9310_I2C_LOOP_DELAY_US                  ( LA9310_I2C_LOOP_TOTAL_DELAY_US / LA9310_I2C_LOOP_COUNT )

#define LA9310_I2C_CR_MDIS                        ( 1 << 7 )
#define LA9310_I2C_CR_MENA                        ( 0 << 7 )
#define LA9310_I2C_CR_MSSL                        ( 1 << 5 )
#define LA9310_I2C_CR_TXRX                        ( 1 << 4 )
#define LA9310_I2C_CR_NOACK                       ( 1 << 3 )
#define LA9310_I2C_CR_RSTA                        ( 1 << 2 )

#define LA9310_I2C_RESERVED_ID                    0x03

#define LA9310_I2C_MAX_SRC_OFFSET_0_BYTES         0x00
#define LA9310_I2C_MAX_READ_SIZE_0_BYTES          0x00
#define LA9310_I2C_MAX_SRC_OFFSET_1_BYTES         0xFF
#define LA9310_I2C_MAX_READ_SIZE_1_BYTES          0xFF
#define LA9310_I2C_MAX_SRC_OFFSET_2_BYTES         0xFFFF
#define LA9310_I2C_MAX_READ_SIZE_2_BYTES          0xFFFF
#define LA9310_I2C_MAX_SRC_OFFSET_3_BYTES         0xFFFFFF
#define LA9310_I2C_MAX_READ_SIZE_3_BYTES          0xFFFFFF
#define LA9310_I2C_MAX_SRC_OFFSET_4_BYTES         0xFFFFFFFF
#define LA9310_I2C_MAX_READ_SIZE_4_BYTES          0xFFFFFFFF

#define LA9310_I2C_SR_TCF                         ( 1 << 7 )
#define LA9310_I2C_SR_IBIF_CLEAR                  ( 1 << 1 )
#define LA9310_I2C_SR_IBIF                        ( 1 << 1 )
#define LA9310_I2C_SR_IBB_IDLE                    ( 0 << 5 )
#define LA9310_I2C_SR_IBB_BUSY                    ( 1 << 5 )
#define LA9310_I2C_SR_IBAL                        ( 1 << 4 )
#define LA9310_I2C_SR_NO_RXAK                     ( 1 << 0 )
#define LA9310_I2C_DBG_GLITCH_EN                  ( 1 << 3 )
#define LA9310_I2C_PIN_CONTROL_PMUXCR0_SCL_PIN    ( 1 << 2 )

typedef struct i2c_regs
{
    uint8_t ucI2C_Ibad;
    uint8_t ucI2C_Ibfd;
    uint8_t ucI2C_Ibcr;
    uint8_t ucI2C_Ibsr;
    uint8_t ucI2C_Ibdr;
    uint8_t ucI2C_Ibic;
    uint8_t ucI2C_Ibdbg;
} i2c_regs_t;

typedef struct i2c_pin_control
{
    uint32_t ulI2C_PMUXCR0;
} i2c_pin_control_t;

#endif /* I2C_H_ */
