/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright 2022-2023 NXP
 * All rights reserved.
 */
#include "rfic_synth.h"
#include "rfic_max2870_synth.h"
#include "math.h"

/* Default freq is 751 MHz */
uint32_t Rficmax2870InitReg_u[ ] = {
    0x0030E4A8, 0x20037FF9, 0x0100BE42, 0x00000003,
    0x62A662FC, 0x01400005
};

int32_t Rficmax2870WriteReg( RficDevice_t *pRficDev, uint8_t addr, uint32_t data )
{
	int32_t iRet = 0,i,j=0;
	uint8_t ucData[ 4 ] = { 0 };

	data  =  (data & 0xFFFFFFF8)  |  (addr & 0x07);
	/* Write synthesizer register */
	for (i = 3; i >=0 ; i--) {
		ucData[j] = (data >> ( i * 8) ) & 0xFF;
		j++;
	}

	iRet = lDspiPush( pRficDev->pDspiHandle, DSPI_CS0, DSPI_DEV_WRITE,
			  ucData, 4 );

	if( 0 > iRet )
	{
		log_err( "%s: lDspiPush failed, error[%d]\r\n", __func__,
		 iRet );
	}

	/* log_info( "%s: WriteReg[%x : %x,%x,%x,%x] data is %x\r\n", __func__, addr,ucData[0],ucData[1],ucData[2],ucData[3],data ); */
	return iRet;
}

int32_t Rficmax2870ReadReg( RficDevice_t *pRficDev, uint8_t addr, uint32_t *data )
{
    int32_t iRet;
    uint8_t ucData[ 4 ] = { 0 };

    /* Push address to be read */
    ucData[ 0 ] = addr;
    iRet = lDspiPush( pRficDev->pDspiHandle, DSPI_CS0, DSPI_DEV_READ,
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
   
    *data = (( uint32_t ) ( ucData[ 3 ] ) << 24 ) | ( ( uint32_t ) ( ucData[ 2 ] ) << 16 ) | ( ( uint16_t) ( ucData[ 1 ] ) << 8 )| ucData[ 0 ];

    return iRet;
}

// Integer Division Value
void max2870Set_N(uint32_t j)
{
	Rficmax2870InitReg_u[0] &= ~(0xFFFF << 15);
	Rficmax2870InitReg_u[0] |= (j << 15);
}

// Fractional Division Value
void max2870Set_FRAC(uint32_t j)
{
	Rficmax2870InitReg_u[0] &= ~(0xFFF << 3);
	Rficmax2870InitReg_u[0] |= (j << 3);
}

// RFOUT_ Output Divider Mode
void max2870Set_DIVA(uint32_t j)
{
    Rficmax2870InitReg_u[4] &= ~(0x7 << 20);
    Rficmax2870InitReg_u[4] |= (j << 20);
}

// Calculates values for given frequency and INT_N / FRAC_N mode
void Rficmax2870AdjustPllFreq( RficDevice_t *pRficDev, int32_t freq_khz )
{
	// Determine DIVA
	int8_t  diva = -1,iRet, diva_val = 1;
    int32_t fPFD,N,F;
    /* const TickType_t xDelay = 20 / portTICK_PERIOD_MS; */

	if (freq_khz < 750000) {
		diva = 3;   // diva bits
		diva_val = 8;   //actual value is 8
    }
	else if (freq_khz < 1500000) {
		diva = 2;   // diva bits
		diva_val = 4;   //actual value is 4
    }
	else if (freq_khz < 3000000) {
		diva = 1;   // diva bits
		diva_val = 2;   //actual value is 2
    }
	else if (freq_khz <= 6000000) {
		diva = 0;   // diva bits
		diva_val = 1;   //actual value is 1
    }
	else
	{
		log_info(" Bad input frequency to max2870SetFrequency\r\n");
		return;
	}

    /* fPFD is calculated based on Input OSC_FREQ */
    /* Values of doubler and rdivider are choosen such that fPFD is 30.720MHz. */
    fPFD = 30720;
    F = 0;
	N = (freq_khz * diva_val)/ fPFD;
    /* using maximum modulus values supported */
    /* Modulus value is 4095, 0.13330078125 is derived using 4095/fPFD */
	F = round(((freq_khz * diva_val) * 0.13330078125) - (N * 4095));

	// Set registers for Frac-N configuration
	max2870Set_N(N);
	max2870Set_FRAC(F);
	max2870Set_DIVA(diva);
	/* log_info( "%s: N is %d and F is %d \r\n", __func__, N, F ); */
	/* log_info( "%s: diva value is %d \r\n", __func__, diva ); */

	// Update MAX registers with new frequency info
    iRet = Rficmax2870WriteReg( pRficDev, 4, Rficmax2870InitReg_u[4] );
    /* log_info( "%s: WriteReg[ : %x]\r\n", __func__, Rficmax2870InitReg_u[4] ); */
	if( iRet < 0 )
	{
		log_err( "%s: synth write reg[%x] failed\r\n", __func__, 4 );
	}
    iRet = Rficmax2870WriteReg( pRficDev, 0, Rficmax2870InitReg_u[0] );
    /* log_info( "%s: WriteReg[ : %x]\r\n", __func__, Rficmax2870InitReg_u[0] ); */
	if( iRet < 0 )
	{
		log_err( "%s: synth write reg[%x] failed\r\n", __func__, 0 );
	}
}

int32_t Rficmax2870Init( RficDevice_t *pRficDev )
{
    uint8_t iRet;
    /* const TickType_t xDelay = 20 / portTICK_PERIOD_MS; */
    /* Writing 2 times */
	for (int j = 1; j >=  0; j--)
	{
		for (int i = 5; i >= 0; i--)  /* 6 write registers */
		{
			iRet = Rficmax2870WriteReg( pRficDev, i , 0);
			if( iRet < 0 )
			{
				log_err( "%s: synth write reg[%x] failed\r\n", __func__, i );
				return -1;
			}
			/* if (i == 5) */
			/* 	vTaskDelay( xDelay ); */
		}
	}

    /* program register with default values for freq 751MHz */
    for(int i = 5; i >= 0; i-- ) /* write 6 registers for freq 751MHz */ 
    {
        iRet = Rficmax2870WriteReg( pRficDev, i, Rficmax2870InitReg_u[i] );
        if( iRet < 0 )
        {
            log_err( "%s: synth write reg[%x] failed\r\n", __func__, i );
            return -1;
        }
    }

    return 0;
}

