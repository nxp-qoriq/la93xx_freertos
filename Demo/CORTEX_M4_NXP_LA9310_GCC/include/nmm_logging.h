/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2022-2024 NXP
 */

#ifndef __NMM_LOGGING_H__
#define __NMM_LOGGING_H__

#include "debug_console.h"

#define nmm_log_to_console      debug_printf

int nmm_log_to_mem(const char *fmt, ...);

#endif /* __NMM_LOGGING_H__ */
