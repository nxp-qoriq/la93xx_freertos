/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2022-2024 NXP
 */

#include <common.h>
#include "print_scan.h"
#include "nmm_logging.h"

static int nmm_putc(int ch, void *stream)
{
	const unsigned char c = ( unsigned char )ch;
	vMemlogWrite(&c, 1);

	return 0;
}

int nmm_log_to_mem(const char *fmt, ...)
{
	va_list ap;
	int ret;

	va_start(ap, fmt);
	ret = _doprint(NULL, nmm_putc, -1, (char *)fmt, ap);
	va_end(ap);

	return ret;
}
