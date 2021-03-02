/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright 2021 NXP
 */
#include "rfic_demod.h"

int32_t RficDemodWriteReg( RficDevice_t *pRficDev, uint8_t addr, uint8_t data )
{
    int32_t iRet;
    uint8_t ucData[ 4 ] = { 0 };

    log_dbg("%s: WriteReg[%x : %x]\r\n", __func__, addr, data);
    /* Read demod register before writing */
    ucData[ 0 ] = addr | RFIC_DEMOD_READ_OPR;
    iRet = lDspiPush( pRficDev->pDspiHandle, DSPI_CS3, DSPI_DEV_READ,
                      ucData, 4 );
    if( 0 > iRet )
    {
        log_err( "%s: lDspiPush failed, error[%d]\r\n", __func__,
                 iRet );
        return iRet;
    }

    iRet = lDspiPop( pRficDev->pDspiHandle, ucData, 4 );
    if( 0 > iRet )
    {
        log_err( "%s: lDspiPop failed, error[%d]\r\n", __func__,
                 iRet );
        return iRet;
    }
    /* Write demodulator register */
    ucData[ 0 ] = addr;
    ucData[ 1 ] = data;
    iRet = lDspiPush( pRficDev->pDspiHandle, DSPI_CS3, DSPI_DEV_WRITE,
                      ucData, 4 );
    if( 0 > iRet )
    {
        log_err( "%s: lDspiPush failed, error[%d]\r\n", __func__,
                 iRet );
    }

    return iRet;
}

int32_t RficDemodReadReg( RficDevice_t *pRficDev, uint8_t addr, uint8_t *data )
{
    int32_t iRet;
    uint8_t ucData[ 4 ] = { 0 };

    /* Push address to be read, Set MSB bit
     * which indicate read operation */
    ucData[ 0 ] = addr | RFIC_DEMOD_READ_OPR;
    iRet = lDspiPush( pRficDev->pDspiHandle, DSPI_CS3, DSPI_DEV_READ,
                      ucData, 4 );
    if( 0 > iRet )
    {
        log_err( "%s: lDspiPush failed, error[%d]\r\n", __func__,
                 iRet );
        return iRet;
    }

    iRet = lDspiPop( pRficDev->pDspiHandle, ucData, 4 );
    if( 0 > iRet )
    {
        log_err( "%s: lDspiPop failed, error[%d]\r\n", __func__,
                 iRet );
        return iRet;
    }

    /* index 0 should be ignored, this is outcome
     * of above push operation */
    *data = ucData[ 1 ];

    return iRet;
}

BaseType_t RficDemodGainCtrl( RficDevice_t *pRficDev, int32_t iAttn,
			      int32_t iGain )
{
    uint8_t ucData;
    int32_t iRet;

    /* Read and update rf attenuation */
    iRet = RficDemodReadReg( pRficDev, RFIC_DEMOD_REG16, &ucData );
    if( 0 > iRet )
    {
        log_err( "%s: Attn read failed, error[%d]\r\n", __func__, iRet );
        return iRet;
    }

    ucData = (( ucData & RFIC_DEMOD_ATTN_MASK ) |
              (( uint8_t )( iAttn << RFIC_DEMOD_ATTN_SHIFT )));

    iRet = RficDemodWriteReg( pRficDev, RFIC_DEMOD_REG16, ucData );
    if( 0 > iRet )
    {
        log_err( "%s: Attn write failed, error[%d]\r\n", __func__, iRet );
        return iRet;
    }

    /* Read and update BB gain */
    iRet = RficDemodReadReg( pRficDev, RFIC_DEMOD_REG21, &ucData );
    if( 0 > iRet )
    {
        log_err( "%s: Gain read failed, error[%d]\r\n", __func__, iRet );
        return iRet;
    }

    ucData = (( ucData & RFIC_DEMOD_GAIN_MASK ) |
              (( uint8_t )( iGain << RFIC_DEMOD_GAIN_SHIFT )));

    iRet = RficDemodWriteReg( pRficDev, RFIC_DEMOD_REG21, ucData );
    if( 0 > iRet )
    {
        log_err( "%s: Gain write failed, error[%d]\r\n", __func__, iRet );
        return iRet;
    }

    return iRet;
}

int32_t RficDemodLoMatch( RficDevice_t *pRficDev, rf_band_t band, int32_t freq )
{
    uint8_t ucLoMatch1, ucLoMatch2;

    /* Select the correct LO matching settings */
    switch( band )
    {
        case RF_SW_BAND_N77:
            if( RFIC_DEMOD_N77_LO_SHIFT > freq )
            {
                ucLoMatch1 = RFIC_DEMOD_N77L_LO_MATCH1;
                ucLoMatch2 = RFIC_DEMOD_N77L_LO_MATCH2;
            }
            else
            {
                ucLoMatch1 = RFIC_DEMOD_N77H_LO_MATCH1;
                ucLoMatch2 = RFIC_DEMOD_N77H_LO_MATCH2;
            }
	    break;
        case RF_SW_BAND_B13:
            ucLoMatch1 = RFIC_DEMOD_B13_LO_MATCH1;
            ucLoMatch2 = RFIC_DEMOD_B13_LO_MATCH2;
            break;
        case RF_SW_BAND_B3:
            ucLoMatch1 = RFIC_DEMOD_B3_LO_MATCH1;
            ucLoMatch2 = RFIC_DEMOD_B3_LO_MATCH2;

            break;
        case RF_SW_BAND_GNSS:
            ucLoMatch1 = RFIC_DEMOD_GNSS_LO_MATCH1;
            ucLoMatch2 = RFIC_DEMOD_GNSS_LO_MATCH2;
            break;

	default:
            return -1;
    };

    /* Update the demod register */
    if( RficDemodWriteReg( pRficDev, RFIC_DEMOD_REG18, ucLoMatch1 ))
    {
        log_err( "%s: write reg[%x] failed\r\n", __func__, RFIC_DEMOD_REG18 );
        return -1;
    }

    if( RficDemodWriteReg( pRficDev, RFIC_DEMOD_REG19, ucLoMatch2 ))
    {
        log_err( "%s: write reg[%x] failed\r\n", __func__, RFIC_DEMOD_REG19 );
        return -1;
    }

    return 0;
}

int32_t RficDemodInit( RficDevice_t *pRficDev )
{
    int32_t iRet;
    uint16_t usData;

    /* Demodulator soft reset */
    usData = RFIC_DEMOD_RESET;
    iRet = RficDemodWriteReg( pRficDev, RFIC_DEMOD_REG22, usData );
    if( iRet < 0 )
    {
        log_err( "%s: Device reset failed\r\n", __func__ );
        return -1;
    }

    /* LO Matching - For default band - b13 */
    iRet = RficDemodLoMatch( pRficDev, RF_SW_BAND_B13, 0 );
    if( iRet < 0 )
    {
        log_err( "%s: LO matching failed\r\n", __func__ );
        return -1;
    }

    return 0;
}

