/* SPDX-License-Identifier: (BSD-3-Clause) */
/*
 * Copyright 2022 NXP
 */

#include "common.h"

/***************************************************************************
 * Function	:	log_init
 * Arguments	:	void
 * Return	:	void
 * Description	:	This function initializes log buffer.
 ***************************************************************************/
void log_init(void)
{
	uint32_t *base = (uint32_t *)TCM_LOG_BUFF_START;
	*base = 0x0;
}

/***************************************************************************
 * Function	:	log_str
 * Arguments	:	str - Input string
 * Return	:	void
 * Description	:	This function logs string onto LOG_BUFF.
 ***************************************************************************/
void log_str(const char *str)
{
	uint32_t cnt = *(uint32_t *)TCM_LOG_BUFF_START;
	uint8_t *base = (uint8_t *)TCM_LOG_BUFF_START;
	uint8_t *log_ptr;
	uint32_t len;

	len = rom_strlen(str);

	/* Next log pointer */
	log_ptr = base + cnt + sizeof(uint32_t);

	/* Make the size of log buffer fixed */
	if (cnt + len > LOG_BUFF_SIZE) {
		cnt = 0;
		log_ptr = base + sizeof(uint32_t);
	}

	rom_memcpy((void *)log_ptr, (const void *)str, len);
	*(uint32_t *)base = cnt + len;
};

/***************************************************************************
 * Function	:	log_uint
 * Arguments	:	val - Input value
 * Return	:	void
 * Description	:	This function logs value onto LOG_BUFF.
 ***************************************************************************/
void log_uint(uint32_t val)
{
	uint32_t cnt = *(uint32_t *)TCM_LOG_BUFF_START;
	uint8_t *base = (uint8_t *)TCM_LOG_BUFF_START;
	uint8_t *log_ptr;
	uint8_t len = 10;
	char hex_ch[20] = "0123456789abcdef";

	/* Next log pointer */
	log_ptr = base + cnt + sizeof(uint32_t);

	/* Make the size of log buffer fixed */
	if (cnt + len > LOG_BUFF_SIZE) {
		cnt = 0;
		log_ptr = base + sizeof(uint32_t);
	}

	/* Write val on memory, fmt: 0x<val> */
	*log_ptr++ = '0';
	*log_ptr++ = 'x';

	/* Value writen in form of characters */
	log_ptr = log_ptr + 7;
	len = len - 2;

	while (len--) {
		*log_ptr-- = hex_ch[val & 0xf];
		val = val >> 4;
	}

	*(uint32_t *)base = cnt + 10;
};
