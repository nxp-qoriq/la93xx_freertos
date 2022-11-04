/* SPDX-License-Identifier: (BSD-3-Clause) */
/*
 * Copyright 2022 NXP
 */

#include "types.h"
#include "utils.h"
#include "common.h"

/***************************************************************************
 * Function	:	rom_memcpy
 * Arguments	:	dest - Destination address
 *			src - Source address
 *			n - Bytes to be copied
 * Return	:	Destination address
 * Description	:	This function copy data from src to dest memory.
 ***************************************************************************/
void *rom_memcpy(void *dest, const void *src, size_t n)
{
	const uint8_t *cs = src;
	uint8_t *cd = dest;

	while (n--)
		*cd++ = *cs++;

	return dest;
}

/***************************************************************************
 * Function	:	rom_word_memcpy
 * Arguments	:	dest - Destination address
 *			src - Source address
 *			n - Words to be copied
 * Return	:	Destination address
 * Description	:	This function copy data from src to dest memory.
 ***************************************************************************/
void *rom_word_memcpy(void *dest, const void *src, size_t n)
{
	const uint32_t *cs = src;
	uint32_t *cd = dest;

	while (n--)
		*cd++ = *cs++;

	return dest;
}

/***************************************************************************
 * Function	:	rom_memset
 * Arguments	:	dest - Destination address
 *			ch - Constant value to be filled in the memory
 *			n - Bytes to be filled
 * Return	:	dest - Destination address
 * Description	:	This function fills the first n bytes of the
 *			memory area pointed to by dest with the constant
 *			byte ch.
 ***************************************************************************/
void *rom_memset(void *dest, uint8_t ch, size_t n)
{
	uint8_t *cd = dest;

	while (n--)
		*cd++ = ch;

	return dest;
};

/***************************************************************************
 * Function	:	rom_strlen
 * Arguments	:	str - Input string
 * Return	:	len - Length of string
 * Description	:	This  function returns length of input string.
 ***************************************************************************/
size_t rom_strlen(const char *str)
{
	const char *s = str;

	while (*s)
		++s;
	return s - str;
};
/***************************************************************************
 * Function     :       error_handler
 * Arguments    :       err_code - error code returned
 * Return       :       void
 * Description  :       This function is main error handler.
 ***************************************************************************/
void error_handler(uint32_t err_code)
{
        /* Write error code in error code register */
        uint32_t *reg_addr = (uint32_t *)
                                (CCSR_DCFG_BASE_ADDR + DCFG_SCRATCH1_OFFSET);

        /* Write error code in SCRATCH 1 in DCFG space */
        OUT_32(reg_addr, err_code);

        spin_loop();
}

/***************************************************************************
 * Function     :       spin_loop
 * Arguments    :       void
 * Return       :       never
 * Description  :       Execution halts in this function, control is never
 *                      returned back.
 ***************************************************************************/
void spin_loop(void)
{
        while (1)
                ;
}
