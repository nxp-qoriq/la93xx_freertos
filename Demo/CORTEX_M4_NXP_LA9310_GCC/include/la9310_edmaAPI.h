/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2017, 2021 NXP
 */

#ifndef __LA9310_EDMAAPI_H__
#define __LA9310_EDMAAPI_H__

#include "la9310_edma.h"

/**************************************************************************
 *                        Abbreviations
 ***************************************************************************
 * IN    - Input Parameter
 * OUT   - Output Paramater
 * M     - Mandatory
 * O     - Optional
 *
 ***************************************************************************/

/***************************************************************************
* @iEdmaInit
*
* Initializes eDMA.
*
* Return Value -
*	SUCCESS
*
* NOTE: This API can be called only once.
*
***************************************************************************/
int iEdmaInit( void );

/***************************************************************************
 * @pEdmaAllocChannel
 *
 * Application uses this API to give callback function and to get
 * an exclusive handle for a DMA channel.
 *
 * info		   - [IN][M] information for callback function
 *
 * pCallbackFn  - [IN][M] pointer of callback function
 *
 * Return Value -
 *            edma_handle_t - valid eDMA channel handle, on success
 *            NULL value - on error
 *
 ****************************************************************************/
pvEdmaHandle pEdmaAllocChannel( void * info,
                                Callback pCallbackFn );

/****************************************************************************
* @iEdmaXferReq
*
* Application can use this API to transfer data using eDMA channel.
*
* src_addr	- [IN][M] source address
*
* dst_addr	- [IN][M] Destination address
*
* size		- [IN][M] size(No of BYTES)
*
* handle	- [IN][M] eDMA channel handle
*
* Return Value -
*	SUCCESS - no error
*	Negative value - on error.
*
****************************************************************************/
int iEdmaXferReq( uint32_t src_addr,
                  uint32_t dst_addr,
                  uint32_t size,
                  pvEdmaHandle Handle );

/****************************************************************************
 * @vEdmaFreeChannel
 *
 * Application uses this API to free that particular handle, which has
 * benn allocated earlier for a DMA channel.
 *
 * handle	 - [IN][M] eDMA channel handle
 *
 * Return Value - void
 *
 *****************************************************************************/
void vEdmaFreeChannel( pvEdmaHandle Handle );

#endif /* ifndef __LA9310_EDMAAPI_H__ */
