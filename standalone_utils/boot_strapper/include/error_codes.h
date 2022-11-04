/* SPDX-License-Identifier: (BSD-3-Clause) */
/*
 * Copyright 2022 NXP
 */

#ifndef ERROR_CODES_H
#define ERROR_CODES_H

/*****************************************
	I2C device drivers errors
******************************************/
#define ERROR_I2C			0x10
#define ERROR_I2C_NO_WAKEUP_INIT	(ERROR_I2C + 1)
#define ERROR_I2C_INVALID_OFFSET	(ERROR_I2C + 2)
#define ERROR_I2C_TIMEOUT		(ERROR_I2C + 3)
#define ERROR_I2C_NODEV			(ERROR_I2C + 4)
#define ERROR_I2C_NO_WAKEUP_READ	(ERROR_I2C + 5)
#define ERROR_I2C_RESTART		(ERROR_I2C + 6)
#define ERROR_I2C_NOT_IDLE		(ERROR_I2C + 7)
#define ERROR_I2C_NOT_BUSY		(ERROR_I2C + 8)
#define ERROR_I2C_NOACK			(ERROR_I2C + 9)
#define ERROR_READ_TIMEOUT		(ERROR_I2C + 10)
#define ERROR_SLAVE_ADDR_TIMEOUT	(ERROR_I2C + 11)
#define ERROR_MEM_ADDR_TIMEOUT		(ERROR_I2C + 12)

#endif /* ifndef ERROR_CODES_H */
