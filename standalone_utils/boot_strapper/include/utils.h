/* SPDX-License-Identifier: (BSD-3-Clause) */
/*
 * Copyright 2022 NXP
 */

#ifndef UTILS_H
#define UTILS_H

#include <types.h>

void *rom_memcpy(void *dest, const void *src, size_t n);
void *rom_word_memcpy(void *dest, const void *src, size_t n);
void *rom_memset(void *dest, uint8_t ch, size_t n);
size_t rom_strlen(const char *str);

void error_handler(uint32_t err_code);
void spin_loop(void);
#endif /* ifndef UTILS_H */
