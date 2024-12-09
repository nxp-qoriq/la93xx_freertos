/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2017, 2021, 2024 NXP
 */


#include "la9310_error_codes.h"
#include "la9310_edmaAPI.h"
#include "la9310_irq.h"

int iEdmaInit( void )
{
    /* Enable Round Robin Channel Arbitration */
    OUT_32( EDMA_CR_REG, EDMA_CR_ERCA );
    /* No priority value for any channel */
    OUT_32( EDMA_ERQ_REG, 0 );
    OUT_32( EDMA_INT_REG, 0xffffffff );
    OUT_32( EDMA_EEI_REG, 0xffffffff );
    NVIC_SetPriority( IRQ_EDMA, EDMA_IRQ_PRIORITY );
    NVIC_EnableIRQ( IRQ_EDMA );
    return SUCCESS;
}

pvEdmaHandle pEdmaAllocChannel( void * info,
                                Callback pCallbackFn )
{
    uint32_t usChannelNumber = NO_CHANNEL_AVL, c = 0;
    struct la9310_stats * pStats;

    pStats = pLa9310Info->stats;
    edma_channel_info_t * pEdmaCh = pvPortMalloc( sizeof
                                                  ( struct edma_channel_info ) );

    log_info( "%s:EDMA_ERQ_REG:%x\n\r", __func__,
              IN_32( ( uint32_t * ) EDMA_ERQ_REG ) );

    for( c = 0; c < EDMA_CHANNELS; c++ )
    {
        if( ( IN_32( ( uint32_t * ) EDMA_ERQ_REG ) ^ NO_CHANNEL_AVL ) &
            1 << c )
        {
            usChannelNumber = c;
            break;
        }
    }

    if( usChannelNumber == NO_CHANNEL_AVL )
    {
        return NULL;
    }

    OUT_16( &pEdmaCh->channel_id, usChannelNumber );
    OUT_32( &pEdmaCh->tcd_addr, ( EDMA_TCD0 +
                                  ( usChannelNumber * EDMA_TCD_SIZE ) ) );
    OUT_32( &pEdmaCh->info, ( uint32_t ) info );
    OUT_32( &pEdmaCh->pCallbackFn, ( uint32_t ) pCallbackFn );
    pEdmaGlb[ usChannelNumber ] = pEdmaCh;
    OUT_32( EDMA_ERQ_REG, ( 1 << pEdmaCh->channel_id ) |
            IN_32( ( uint32_t * ) EDMA_ERQ_REG ) );
    pStats->eDMA_ch_allocated++;
    pStats->la9310_eDMA_ch[ usChannelNumber ].status = 1;
    return pEdmaCh;
}

int iEdmaXferReq( uint32_t src_addr,
                  uint32_t dst_addr,
                  uint32_t size,
                  pvEdmaHandle Handle )
{
    edma_channel_info_t * pEdmaCh = ( edma_channel_info_t * ) Handle;
    edma_tcd_t * pTcdRegs = ( edma_tcd_t * ) pEdmaCh->tcd_addr;
    struct la9310_stats * pStats;

    pStats = pLa9310Info->stats;

    if( pTcdRegs->edma_tcd_biter_elink_csr & EDMA_TCD_CSR_ACTIVE )
    {
        return -EDMA_BUSY;
    }
    else if( src_addr & 0x03 )
    {
        return -EDMA_SOURCE_UNALIGNED;
    }
    else if( dst_addr & 0x03 )
    {
        return -EDMA_DESTINATION_UNALIGNED;
    }
    else if( size == 0 )
    {
        return -EDMA_ZERO_BYTE_COUNT;
    }
    else if( size & 0x03 )
    {
        return -EDMA_SIZE_UNALIGNED;
    }
    else if( pEdmaCh->channel_id > WDOG_eDMA_CHANNEL )
    {
        return -EDMA_INVALID_CHANNEL;
    }

    OUT_32( &pTcdRegs->edma_tcd_saddr, src_addr );
    OUT_32( &pTcdRegs->edma_tcd_attr_soff, EDMA_TCD_ATTR_SMOD_DIS |
            EDMA_TCD_ATTR_SSIZE_4B |
            EDMA_TCD_ATTR_DMOD_DIS |
            EDMA_TCD_ATTR_DSIZE_4B |
            EDMA_TCD_SOFF_4B );
    OUT_32( &pTcdRegs->edma_tcd_nbytes, size );
    OUT_32( &pTcdRegs->edma_tcd_slast, 0x00000000 );
    OUT_32( &pTcdRegs->edma_tcd_daddr, dst_addr );
    OUT_32( &pTcdRegs->edma_tcd_citer_elink_doff, TCD_ELINKYES_ELINK_DIS |
            EDMA_TCD_CITER_ELINKYES_CITER |
            EDMA_TCD_DOFF_4B );
    OUT_32( &pTcdRegs->edma_tcd_dlastsga, 0x00000000 );

    if( pEdmaCh->channel_id == WDOG_eDMA_CHANNEL )
    {
        OUT_32( &pTcdRegs->edma_tcd_biter_elink_csr,
                TCD_ELINKYES_ELINK_DIS |
                EDMA_TCD_BITER_ELINKYES_BITER |
                EDMA_TCD_CSR_DREQ );
    }
    else if( ( pEdmaCh->pCallbackFn == NULL ) &&
             ( pEdmaCh->channel_id != WDOG_eDMA_CHANNEL ) )
    {
        OUT_32( &pTcdRegs->edma_tcd_biter_elink_csr,
                TCD_ELINKYES_ELINK_DIS |
                EDMA_TCD_BITER_ELINKYES_BITER |
                EDMA_TCD_CSR_START );
	    if (pEdmaCh->channel_id < LA9310_eDMA_CHANNELS)
		    pStats->la9310_eDMA_ch[ pEdmaCh->channel_id ].no_callback_reg++;
    }
    else
    {
        OUT_32( &pTcdRegs->edma_tcd_biter_elink_csr,
                TCD_ELINKYES_ELINK_DIS |
                EDMA_TCD_BITER_ELINKYES_BITER |
                EDMA_TCD_CSR_START |
                EDMA_TCD_CSR_INTMJR );
    }

    if( pEdmaCh->channel_id != WDOG_eDMA_CHANNEL &&
    	pEdmaCh->channel_id < LA9310_eDMA_CHANNELS)
    {
        pStats->la9310_eDMA_ch[ pEdmaCh->channel_id ].xfer_req++;
    }

    return SUCCESS;
}

void vEdmaFreeChannel( pvEdmaHandle Handle )
{
    edma_channel_info_t * pEdmaCh = ( edma_channel_info_t * ) Handle;
    struct la9310_stats * pStats;

    pStats = pLa9310Info->stats;
    OUT_32( EDMA_ERQ_REG, ( 1 << pEdmaCh->channel_id )
            ^ IN_32( ( uint32_t * ) EDMA_ERQ_REG ) );
    pStats->eDMA_ch_allocated--;
    pStats->la9310_eDMA_ch[ pEdmaCh->channel_id ].status = 0;
    pStats->la9310_eDMA_ch[ pEdmaCh->channel_id ].xfer_req = 0;
    pStats->la9310_eDMA_ch[ pEdmaCh->channel_id ].error_interrupt = 0;
    pStats->la9310_eDMA_ch[ pEdmaCh->channel_id ].success_interrupt = 0;
    pStats->la9310_eDMA_ch[ pEdmaCh->channel_id ].no_callback_reg = 0;
}

void La9310eDMA_IRQHandler( void )
{
    uint32_t usChannelNumber = NO_CHANNEL_AVL, c = 0, i = 0;
    struct la9310_stats * pStats;

    pStats = pLa9310Info->stats;

    if( IN_32( ( uint32_t * ) EDMA_ERR_REG ) )
    {
        for( i = 0; i < EDMA_CHANNELS; i++ )
        {
            if( IN_32( ( uint32_t * ) EDMA_ERR_REG ) & 1 << i )
            {
                usChannelNumber = i;
                break;
            }
        }

        if( NO_CHANNEL_AVL != usChannelNumber )
        {
            if( pEdmaGlb[ usChannelNumber ]->pCallbackFn != NULL )
            {
                ( *pEdmaGlb[ usChannelNumber ]->pCallbackFn )
                    ( pEdmaGlb[ usChannelNumber ]->info, eDMA_FAIL );
            }

            OUT_32( EDMA_ERR_REG,
                    ( 1 << pEdmaGlb[ usChannelNumber ]->channel_id )
                    & IN_32( ( uint32_t * ) EDMA_ERR_REG ) );
            pStats->la9310_eDMA_ch[ usChannelNumber ].error_interrupt++;
        }
    }
    else
    {
        for( c = 0; c < EDMA_CHANNELS; c++ )
        {
            if( IN_32( ( uint32_t * ) EDMA_INT_REG ) & 1 << c )
            {
                usChannelNumber = c;
                break;
            }
        }

        if( NO_CHANNEL_AVL != usChannelNumber )
        {
            if( pEdmaGlb[ usChannelNumber ]->pCallbackFn != NULL )
            {
                ( *pEdmaGlb[ usChannelNumber ]->pCallbackFn )
                    ( pEdmaGlb[ usChannelNumber ]->info, eDMA_PASS );
            }

            OUT_32( EDMA_INT_REG,
                    ( 1 << pEdmaGlb[ usChannelNumber ]->channel_id )
                    & IN_32( ( uint32_t * ) EDMA_INT_REG ) );
            pStats->la9310_eDMA_ch[ usChannelNumber ].success_interrupt++;
        }
    }

    NVIC_ClearPendingIRQ( IRQ_EDMA );
    #if ARM_ERRATUM_838869
        dsb();
    #endif
}
