/* SPDX-License-Identifier: (BSD-3-Clause) */
/*
 * Copyright 2022 NXP
 */

#ifndef I2C_H
#define I2C_H

#include "common.h"

#define CONFIG_I2C_TIMEOUT	10
#define I2C_MAX_FDR		191

#define SLAVE_ADDR		0x50
#define I2C_RESERVED_ID		0x03

/* For standard mode I2C Speed is 100KHz */
#define CONFIG_I2C_SPEED	100000

#define IS_CURRENT_MASTER	1
#define ISNOT_CURRENT_MASTER	0

#define I2C_MAX_SRC_OFFSET	0xFFFF
#define I2C_MAX_READ_SIZE	0xFFFF

#define I2C_CR_MDIS		(1 << 7)
#define I2C_CR_MENA		(0 << 7)
#define I2C_DBG_GLITCH_EN	(1 << 3)
#define I2C_SR_IBIF		(1 << 1)
#define I2C_SR_IBAL		(1 << 4)
#define I2C_CR_MSSL		(1 << 5)
#define I2C_CR_TXRX		(1 << 4)
#define I2C_CR_NOACK		(1 << 3)
#define I2C_CR_RSTA		(1 << 2)
#define I2C_SR_TCF		(1 << 7)
#define I2C_SR_IBIF_CLEAR	(1 << 1)
#define I2C_SR_IBB_IDLE		(0 << 5)
#define I2C_SR_IBB_BUSY		(1 << 5)
#define I2C_SR_NO_RXAK		(1 << 0)

uint32_t i2c_init(void *);
uint32_t i2c_read(uint32_t src_offset, uint8_t *dst, uint32_t size, void *);

typedef struct i2c_regs {
	uint8_t ibad;
	uint8_t ibfd;
	uint8_t ibcr;
	uint8_t ibsr;
	uint8_t ibdr;
	uint8_t ibic;
	uint8_t ibdbg;
} i2c_regs_t;

#endif /* ifndef I2C_H */
