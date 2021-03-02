/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2017, 2021 NXP
 */

#ifndef __LA9310_WDOGAPI_H__
#define __LA9310_WDOGAPI_H__

#include "la9310_watchdog.h"

/**************************************************************************
 *                        Abbreviations
 ***************************************************************************
 * IN    - Input Parameter
 * OUT   - Output Parameter
 * M     - Mandatory
 * O     - Optional
 *
 ***************************************************************************/

/***************************************************************************
* @vWdogEnable
*
* Application can use this API to Enable Watchdog and to configure eDMA
* channel 15.
*
* wdog_load_val	- [IN][M] Watchdog loag value
*
* pLa9310Info	- [IN][M] pLa9310Info
*
* Return Value	-
*	SUCCESS - no error
*	Negative value - on error.
*
* NOTE: This API can be called only once.
*
***************************************************************************/
int iWdogEnable( uint32_t wdog_load_val,
                 struct la9310_info * pLa9310Info );

/***************************************************************************
 * @vWdogDisable
 *
 * Application can use this API to Disable Watchdog.
 *
 * Return Value -
 *   void
 *
 * NOTE: Use vWdogEnable to enable WDOG again if used vWdogDisable.
 *
 ****************************************************************************/
void vWdogDisable( void );

/****************************************************************************
* @vWdogReload
*
* Application can use this API to Reload Watchdog counter.
*
* Return Value -
*		void
*
****************************************************************************/
void vWdogReload( void );

#endif /* __LA9310_WDOGAPI_H__ */
