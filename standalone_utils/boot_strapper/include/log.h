/* SPDX-License-Identifier: (BSD-3-Clause) */
/*
 * Copyright 2022 NXP
 */

#ifndef LOG_H
#define LOG_H

#include <types.h>

void log_init(void);
void log_str(const char *str);
void log_uint(uint32_t val);

#endif /* ifndef LOG_H */
