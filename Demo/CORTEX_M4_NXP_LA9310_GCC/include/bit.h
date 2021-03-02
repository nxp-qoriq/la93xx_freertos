/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2019-2021 NXP
 *
 */

#ifndef COMMON_BIT_H_
#define COMMON_BIT_H_

#define BITS_PER_LONG    32


#define BIT( nr )    ( 1UL << ( nr ) )

/*
 * Create a contiguous bitmask starting at bit position @l and ending at
 * position @h. For example
 * GENMASK_ULL(39, 21) gives us the 64bit vector 0x000000ffffe00000.
 */
#define GENMASK( h, l ) \
    ( ( ( ~0UL ) - ( 1UL << ( l ) ) + 1 ) & ( ~0UL >> ( BITS_PER_LONG - 1 - ( h ) ) ) )

/*
 * Create a contiguous bitmask starting at bit position @l and ending at
 * position @h. For example
 * GENMASK2(10, 12) gives us the 32bit vector 0x003FF000.
 */
#define GENMASK2( w, l ) \
    ( ( ( ~0UL ) - ( 1UL << ( l ) ) + 1 ) & ( ~0UL >> ( BITS_PER_LONG - ( ( l ) + ( w ) ) ) ) )


#endif /* COMMON_BIT_H_ */
