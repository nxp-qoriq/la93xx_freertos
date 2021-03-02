/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2021 NXP
 *
 */

#ifndef _TYPES_H_
#define _TYPES_H_

#include <stdbool.h>
#include <stdint.h>

typedef unsigned char           u8;
typedef unsigned short          u16;
typedef unsigned int            u32;
typedef unsigned long long      u64;
typedef signed char             s8;
typedef short                   s16;
typedef int                     s32;
typedef long long               s64;

typedef unsigned int            u32le;
typedef unsigned int            u32be;
typedef unsigned short          u16le;
typedef unsigned short          u16be;

typedef int                     s32le;
typedef int                     s32be;
typedef short                   s16le;
typedef short                   s16be;

typedef s32                     error_t;

typedef u8                      octet_t;

typedef bool                    bool_t;
typedef unsigned int            uint32;

typedef volatile unsigned int   vuint32;     /* 32 bits */

#define BASE_DEC     10
#define BASE_HEXA    16

#ifndef NULL
    #define NULL     ( ( void * ) 0 )
#endif

#define TRUE         true
#define FALSE        false

#endif /* _TYPES_H_ */
