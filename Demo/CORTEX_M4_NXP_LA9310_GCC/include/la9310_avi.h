/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2017, 2021-2024 NXP
 */

#ifndef __LA9310_AVI_H__
#define __LA9310_AVI_H__

#include "FreeRTOS.h"
#include "task.h"
#include "la9310_main.h"
#include "common.h"


#define MAILBOX_VALID_STATUS_RETRIES    200
/*#define AXIQ_LOOPBACK_ENABLE */

#define VCPU_MBOX_STATUS                ( VSPA_BASE_ADDR + 0x6A0 )
#define HOST_OUT_0_MSB                  ( VSPA_BASE_ADDR + 0x680 )
#define HOST_OUT_0_LSB                  ( VSPA_BASE_ADDR + 0x684 )
#define HOST_IN_0_MSB                   ( VSPA_BASE_ADDR + 0x690 )
#define HOST_IN_0_LSB                   ( VSPA_BASE_ADDR + 0x694 )
#define PHY_TIMER                       ( VSPA_BASE_ADDR + 0x20000 )
#define TRIGGER_1                       ( VSPA_BASE_ADDR + 0x2000C )
#define TRIGGER_2                       ( VSPA_BASE_ADDR + 0x20014 )
#define TRIGGER_3                       ( VSPA_BASE_ADDR + 0x2001C )
#define TRIGGER_4                       ( VSPA_BASE_ADDR + 0x20024 )
#define TRIGGER_11                      ( VSPA_BASE_ADDR + 0x2005C )
#define MSG_IN_0_VALID                  ( 1 << 2 )
#define DBGGNCR                         0xE00800EC

/* DCFG_DCSR_DBGGENCR1 */
#define EN_TX_RX0_LPBK                  ( 1 << 1 )
#define EN_TX_RX1_LPBK                  ( 1 << 2 )
#define EN_TX_OBS0_LPBK                 ( 1 << 3 )
#define EN_TX_OBS1_LPBK                 ( 1 << 4 )
#define EN_TX_TXDET_LPBK                ( 1 << 5 )
#define EN_TX_READY_LPBK                ( 1 << 6 )

#define SET_AXIQ_LOOPBACK_MASK          ( EN_TX_READY_LPBK )
#define SET_AXIQ_LOOPBACK_MASK_ALL      ( EN_TX_READY_LPBK | EN_TX_RX0_LPBK | EN_TX_RX1_LPBK | EN_TX_OBS0_LPBK | EN_TX_OBS1_LPBK )
#define REMOVE_AXIQ_LOOPBACK_MASK       0xffffffa1


#define FLOAT2FIXED( x, f )    ( ( int ) ( ( x ) * ( 1 << f ) ) )
#define FIXED2FLOAT( x, f )    ( ( ( float ) ( x ) ) / ( 1 << f ) )
#define MAX_BW_CAL    31


/* Structure for 64-bit mailbox */
struct avi_mbox
{
    uint32_t msb;
    uint32_t lsb;
};

/**
 * @brief : Sends MBOX to VSPA using 0/1 Host to VSPA
 *			 mailbox.
 *
 * @param[in] void * : AVI library handle
 * @param[in] uint32_t : MSB of the mailbox to be send
 * @param[in] uint32_t : LSB of the mailbox to be send
 * @param[in] uint32_t : Host to VSPA mailbox index to be used
 *
 * @return : 0 on sucess and appropriate -ve value on failure
 */
int iLa9310AviHostSendMboxToVspa( void *, uint32_t, uint32_t, uint32_t );

/**
 * @brief : Checks if VSPA out mailbox is valid , if valid reads the VSPA
 *			 mailbox and returns the MBOX else return immediately
 *
 * @param[in] void * : AVI library handle
 * @param[in/out] : Mailbox structure to be populated after reading
 * @param[in] uint32_t : VSPA mailbox index to be read
 *
 * @return : 0 on sucess and appropriate -ve value on failure
 */
int iLa9310AviHostRecvMboxFromVspa( void *, struct avi_mbox *, uint32_t );

/**
 * @brief : Initializes the AVI for use
 *
 * @return : Returns the pointer to AVI handler
 */
void * iLa9310AviInit( void );

/**
 * @brief : Initializes the AVI for use
 *
 * @return : Returns the pointer to AVI handler
 */
void * iLa9310AviHandle();

/**
 * @brief : Sets AXIQ loopback for RX1
 *
 * @return : NULL
 */
void vAxiqLoopbackSet( bool, uint32_t );

int iLa9310AviConfig( void );

#endif /* __LA9310_AVI_H__ */
