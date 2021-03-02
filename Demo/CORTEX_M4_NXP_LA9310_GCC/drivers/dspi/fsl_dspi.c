/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2019-2021 NXP
 *
 */

#include <fsl_dspi.h>
#include <config.h>
#include "FreeRTOS.h"
#include <types.h>
#include <bit.h>

static struct LA931xDspiInstance * pxDspiHandle = { NULL };
int32_t lDspiHzToBaud( uint32_t * lpbr,
                       uint32_t * lbr,
                       uint32_t ulSpeedHz,
                       uint32_t ulBusClk )
{
    uint32_t lPbrTbl[ 4 ] = { 2, 3, 5, 7 };
    uint32_t lBrTbl[ 16 ] =
    {
        2,    4,    6,     8,
        16,   32,   64,    128,
        256,  512,  1024,  2048,
        4096, 8192, 16384, 32768
    };
    int lScaleNeeded, lScale, lMinScale = INT_MAX;
    uint32_t i = 0, j = 0;

    lScaleNeeded = ulBusClk / ulSpeedHz;

    if( ulBusClk % ulSpeedHz )
    {
        lScaleNeeded++;
    }

    for( i = 0; i < ARRAY_SIZE( lBrTbl ); i++ )
    {
        for( j = 0; j < ARRAY_SIZE( lPbrTbl ); j++ )
        {
            lScale = lBrTbl[ i ] * lPbrTbl[ j ];

            if( lScale >= lScaleNeeded )
            {
                if( lScale < lMinScale )
                {
                    lMinScale = lScale;
                    *lbr = i;
                    *lpbr = j;
                }

                break;
            }
        }
    }

    if( lMinScale == INT_MAX )
    {
        log_err( "Can not find valid baud rate,speed_hz is %d,clkrate is %ld, we use the max prescaler value.\n",
                 ulSpeedHz, ulBusClk );
        *lpbr = ARRAY_SIZE( lPbrTbl ) - 1;
        *lbr = ARRAY_SIZE( lBrTbl ) - 1;
        return DSPI_CLKSET_ERROR;
    }

    return 0;
}

void vDspiClkSet( struct LA931xDspiInstance * xDspiHandle,
                  uint32_t ulSpeed )
{
    int32_t lret;
    uint32_t lBesti, lBestj;
    uint32_t ulBusSetup, ulBusClk;

    ulBusClk = DSPI_INPUT_CLK_FREQUENCY;
    log_info( "DSPI Clock Set: Expected speed:%u  Bus clock:%u \r\n", ulSpeed, ulBusClk );
    ulBusSetup = in_dspile32( &xDspiHandle->DspiRegs->ulCtar[ 0 ] );
    ulBusSetup = ulBusSetup & ( ~( DSPI_CTAR_DBR | DSPI_CTAR_PBR( 0x3 ) | DSPI_CTAR_BR( 0xf ) ) );
    lret = lDspiHzToBaud( &lBesti, &lBestj, ulSpeed, ulBusClk );

    if( lret )
    {
        ulSpeed = DSPI_DEFAULT_FREQUENCY;
        log_err( "DSPI :setting failed,setting default \r\n" );
        lret = lDspiHzToBaud( &lBesti, &lBestj, ulSpeed, ulBusClk );
    }

    ulBusSetup |= ( DSPI_CTAR_PBR( lBesti ) | DSPI_CTAR_BR( lBestj ) );
    out_le32( &xDspiHandle->DspiRegs->ulCtar[ 0 ], ulBusSetup );
    xDspiHandle->ulBusClk = ulSpeed;
}

void vWaitFifo( struct LA931xDspiInstance * xDspiHandle )
{
    uint32_t ulSrVal;

    while( 1 )
    {
        ulSrVal = in_dspile32( &xDspiHandle->DspiRegs->ulSr );

        if( ulSrVal & DSPI_SR_TCF )
        {
            out_le32( &xDspiHandle->DspiRegs->ulSr, ulSrVal | DSPI_SR_TCF );
            break;
        }
    }
}

void vDspiHalt( struct LA931xDspiInstance * xDspiHandle,
                uint8_t ucHalt )
{
    uint32_t ulMcrVal;

    ulMcrVal = in_dspile32( &xDspiHandle->DspiRegs->ulMcr );

    if( ucHalt )
    {
        ulMcrVal |= DSPI_MCR_HALT;
    }
    else
    {
        ulMcrVal &= ( uint32_t ) ( ~DSPI_MCR_HALT );
    }

    out_le32( &xDspiHandle->DspiRegs->ulMcr, ulMcrVal );
}

void vDspiFslClearFifo( struct LA931xDspiInstance * xDspiHandle )
{
    uint32_t ulMcrVal;

    vDspiHalt( xDspiHandle, 1 );
    ulMcrVal = in_dspile32( &xDspiHandle->DspiRegs->ulMcr );
    /* flush RX and TX FIFO */
    ulMcrVal |= ( DSPI_MCR_CTXF | DSPI_MCR_CRXF );
    out_le32( &xDspiHandle->DspiRegs->ulMcr, ulMcrVal );
    vDspiHalt( xDspiHandle, 0 );
}

int8_t lDspiClaimBus( struct LA931xDspiInstance * xDspiHandle )
{
    uint32_t ulSrVal, retry = 10;
    int8_t ret = -1;

    vDspiFslClearFifo( xDspiHandle );

    log_info( "MCR    %x\r\n", in_le32( &xDspiHandle->DspiRegs->ulMcr ) );
    log_info( "TCR    %x\r\n", in_le32( &xDspiHandle->DspiRegs->ulTcr ) );
    log_info( "CTAR0  %x\r\n", in_le32( &xDspiHandle->DspiRegs->ulCtar[ 0 ] ) );
    log_info( "SR     %x\r\n", in_le32( &xDspiHandle->DspiRegs->ulSr ) );
    log_info( "RSER   %x\r\n", in_le32( &xDspiHandle->DspiRegs->ulIrsr ) );
    log_info( "CTARE0 %x\r\n", in_le32( &xDspiHandle->DspiRegs->ulCtarX[ 0 ] ) );

    /*Check module TX and RX status */
    while( retry )
    {
        ulSrVal = in_dspile32( &xDspiHandle->DspiRegs->ulSr );

        if( ( ulSrVal & DSPI_SR_TXRXS ) == DSPI_SR_TXRXS )
        {
            ret = 0;
            break;
        }

        retry--;
        log_err( " DSPI RX/TX not ready! SR[%x]\r\n", ulSrVal );
    }

    return ret;
}

int32_t lDspiPush( struct LA931xDspiInstance * pxDspiHandle,
                   DspiChipSel_t eChipSelect,
                   DspiOps_t eOps,
                   uint8_t * pucData,
                   uint32_t ulLen )
{
    struct DspiReg * pxReg = pxDspiHandle->DspiRegs;
    uint32_t ulCmdData, count;

    if( ulLen & 0x3 )
    {
        return DSPI_INCORRECT_LEN;
    }

    if( DSPI_SR_TXCTR( in_dspile32( &pxReg->ulSr ) ) >= DSPI_TX_FIFO_SIZE - 1 )
    {
        return DSPI_TXFIFO_FULL;
    }

    /* Send data in chunk of 4 bytes */
    for( count = 0; count < ulLen / 4; count += 1 )
    {
        /* Select chip select */
        ulCmdData = DSPI_TFR_CS( eChipSelect );
        ulCmdData = ulCmdData | ( ( ( uint32_t ) ( *( pucData + 2 ) ) ) << 8 ) |
                    ( ( uint32_t ) ( *( pucData + 3 ) ) );
        out_le32( &pxReg->ulTfr, ulCmdData );

        ulCmdData = DSPI_TFR_CS( eChipSelect );
        ulCmdData = ulCmdData | ( ( ( uint32_t ) ( *pucData ) ) << 8 ) |
                    ( ( uint32_t ) ( *( pucData + 1 ) ) );

        /* last chunk of this transfer */
        if( count == ( ( ulLen / 4 ) - 1 ) )
        {
            ulCmdData |= DSPI_TFR_EOQ;
        }

        out_le32( &pxReg->ulTfr, ulCmdData );
        pucData += 4;

        /* Wait for completion */
        vWaitFifo( pxDspiHandle );

        /* Clear Tx FIFO */
        ulCmdData = in_le32( &pxReg->ulMcr );
        out_le32( &pxReg->ulMcr, ulCmdData | DSPI_MCR_CTXF );

        /* Clearing the Rx fifo after Write */
        if( DSPI_DEV_WRITE == eOps )
        {
            ulCmdData = in_le32( &pxReg->ulMcr );
            out_le32( &pxReg->ulMcr, ulCmdData | DSPI_MCR_CRXF );
        }
    }

    return DSPI_WRITE_SUCCESS;
}

int32_t lDspiPop( struct LA931xDspiInstance * pxDspiHandle,
                  uint8_t * pucData,
                  uint32_t ulLen )
{
    uint32_t ulRegData;
    uint8_t ucAvailData, count;

    ulRegData = in_dspile32( &pxDspiHandle->DspiRegs->ulSr );

    if( ulRegData & DSPI_SR_SPEF )
    {
        out_le32( &pxDspiHandle->DspiRegs->ulSr, DSPI_SR_SPEF | ulRegData );
        return DSPI_ERROR_PARITY;
    }

    if( ulRegData & DSPI_SR_RFOF )
    {
        out_le32( &pxDspiHandle->DspiRegs->ulSr, DSPI_SR_RFOF | ulRegData );
        return DSPI_ERR_RX_OVERFLOW;
    }

    if( 0 != ( ulLen % 4 ) )
    {
        return DSPI_INCORRECT_LEN;
    }

    ucAvailData = ( ( ( ulRegData & DSPI_SR_COUNT_RX ) >> 4 ) * 4 );

    if( ucAvailData < ulLen )
    {
        return DSPI_NOT_ENOUGH_DATA;
    }

    for( count = 0; count < ulLen / 4; count += 1 )
    {
        ulRegData = in_dspile32( &pxDspiHandle->DspiRegs->ulRfr );

        *( pucData + count ) = ( ( ulRegData >> 24 ) & 0xFF );
        *( pucData + count + 1 ) = ( ( ulRegData >> 16 ) & 0xFF );
        *( pucData + count + 2 ) = ( ( ulRegData >> 8 ) & 0xFF );
        *( pucData + count + 3 ) = ( ( ulRegData ) & 0xFF );
    }

    return DSPI_READ_SUCCESS;
}

struct LA931xDspiInstance * pxDspiInit( uint8_t ucCsMask, uint32_t ulClk )
{
    uint32_t ulMcrCfgVal;
    uint32_t ulCtarCfgVal;
    uint32_t ulSrCfgVal;
    uint32_t ulIrsrCfgVal;
    uint32_t ulCtarXVal;

    if( pxDspiHandle != NULL )
    {
        return pxDspiHandle;
    }

    pxDspiHandle = ( struct LA931xDspiInstance * ) pvPortMalloc( sizeof( struct LA931xDspiInstance ) );

    if( pxDspiHandle == NULL )
    {
        log_err( "DSPI Handle allocation failed!! \r\n" );
        return pxDspiHandle;
    }

    pxDspiHandle->DspiRegs = ( struct DspiReg * ) ( DSPI_REG_BASE_ADDRESS
                                                    + ( ( 0 ) * DSPI_REG_BASE_OFFSET ) );

    pxDspiHandle->ucCsMask = ucCsMask;

    /* frame data length in bits, default 16 bits */
    /* default: all CS signals inactive state is high */
    ulMcrCfgVal = ( DSPI_MCR_MSTR | DSPI_MCR_PCSIS( ucCsMask ) |
                    DSPI_MCR_CTXF | DSPI_MCR_CRXF | DSPI_MCR_XSPI |
                    DSPI_MCR_DTXF | DSPI_MCR_DRXF | DSPI_MCR_HALT );
    ulCtarCfgVal = ( uint32_t ) DSPI_CTAR_TRSZ( 0xf );
    ulCtarXVal = DSPI_CTAR_X_TRSZ | DSPI_CTAR_X_DTCP( 0x1 );
    ulSrCfgVal = ( DSPI_SR_TCF | DSPI_SR_EOQF | DSPI_SR_TFFF |
                   DSPI_SR_RFOF | DSPI_SR_RFDF | DSPI_SR_TFIWF |
                   DSPI_SR_SPEF | DSPI_SR_CTCF );
    ulIrsrCfgVal = DSPI_IRSR_DISABLE;

    out_le32( &pxDspiHandle->DspiRegs->ulMcr, ulMcrCfgVal );
    out_le32( &pxDspiHandle->DspiRegs->ulCtar[ 0 ], ulCtarCfgVal );
    out_le32( &pxDspiHandle->DspiRegs->ulCtarX[ 0 ], ulCtarXVal );
    out_le32( &pxDspiHandle->DspiRegs->ulSr, ulSrCfgVal );
    out_le32( &pxDspiHandle->DspiRegs->ulIrsr, ulIrsrCfgVal );

    /*Enabling the Tx and RxFifo*/
    ulMcrCfgVal = ( ulMcrCfgVal & DSPI_MCR_TXRX_ENABLE );
    out_le32( &pxDspiHandle->DspiRegs->ulMcr, ulMcrCfgVal );

    /*Setting Default frequency 8MHz*/
    vDspiClkSet( pxDspiHandle, ulClk );

    /* remove DSPI halt */
    vDspiHalt( pxDspiHandle, 0 );

    if( lDspiClaimBus( pxDspiHandle ) < 0 )
    {
        /* Release memory for DSPI handler */
        vPortFree( pxDspiHandle );
        pxDspiHandle = NULL;
        return NULL;
    }

    return pxDspiHandle;
}

void vDspiExit()
{
    vPortFree( pxDspiHandle );
    pxDspiHandle = NULL;
}
