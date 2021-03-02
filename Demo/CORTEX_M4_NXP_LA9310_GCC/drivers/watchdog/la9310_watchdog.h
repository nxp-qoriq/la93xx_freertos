/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2017, 2021 NXP
 */

#ifndef __LA9310_WATCHDOG_H__
#define __LA9310_WATCHDOG_H__

#include "FreeRTOS.h"
#include "common.h"

#define WDOG_LOAD_REG         ( WDOG_BASE_ADDR + 0x000 )
#define WDOG_VALUE_REG        ( WDOG_BASE_ADDR + 0x004 )
#define WDOG_CONTROL_REG      ( WDOG_BASE_ADDR + 0x008 )
#define WDOG_INTCLR_REG       ( WDOG_BASE_ADDR + 0x00c )
#define WDOG_RIS_REG          ( WDOG_BASE_ADDR + 0x010 )
#define WDOG_MIS_REG          ( WDOG_BASE_ADDR + 0x014 )
#define WDOG_LOCK_REG         ( WDOG_BASE_ADDR + 0xc00 )
#define WDOG_ITCR_REG         ( WDOG_BASE_ADDR + 0xf00 )
#define WDOG_ITOP_REG         ( WDOG_BASE_ADDR + 0xf04 )

#define WDOG_CONTROL_INTEN    ( 1 << 0 )
#define WDOG_CONTROL_RESEN    ( 1 << 1 )
#define WDOG_UNLOCK           0x1ACCE551
#define WDOG_LOCK             0x00000001

uint32_t load_value;

#endif /* __LA9310_WATCHDOG_H__*/
