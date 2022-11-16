/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright 2021-2022 NXP
 */
#include "rfic_vga.h"

uint16_t RficVgaGainTable[61][2] = {
				{0, 0},       {807, 0},     {925, 0},     {1022, 0},    {1101, 0},
				{1188, 0},    {1270, 0},    {1351, 0},    {1431, 0},    {1510, 0},
				{1582, 0},    {1665, 0},    {1745, 0},    {1822, 0},    {1902, 0},
				{1976, 0},    {1976, 570},  {1976, 696},  {1976, 786},  {1976, 869},
				{1976, 962},  {1976, 1046}, {1976, 1121}, {1976, 1200}, {1976, 1276},
				{1976, 1356}, {1976, 1439}, {1976, 1521}, {1976, 1598}, {1976, 1676},
				{1976, 1756}, {2056, 1756}, {2139, 1756}, {2222, 1756}, {2295, 1756},
				{2365, 1756}, {2438, 1756}, {2517, 1756}, {2605, 1756}, {2689, 1756},
				{2767, 1756}, {2840, 1756}, {2915, 1756}, {3010, 1756}, {3164, 1756},
				{3386, 1756}, {3386, 1836}, {3386, 1917}, {3386, 2005}, {3386, 2077},
				{3386, 2149}, {3386, 2223}, {3386, 2303}, {3386, 2399}, {3386, 2484},
				{3386, 2563}, {3386, 2640}, {3386, 2724}, {3386, 2830}, {3386, 2980},
				{3386, 3540}
};


int32_t RficVgaWriteReg( RficDevice_t *pRficDev, uint8_t data )
{
    int32_t iRet;
    uint8_t ucData[ 4 ] = { 0 };
    uint16_t usAddr = RFIC_VGA_REG_ADDR;

    log_dbg("%s: WriteReg[%x : %x]\r\n", __func__, usAddr, data);

    /* Write synthesizer register */
    ucData[ 0 ] = ( uint8_t )(( usAddr & 0xFF00 ) >> 8 );
    ucData[ 1 ] = ( uint8_t )( usAddr & 0x00FF);
    ucData[ 2 ] = data;

    iRet = lDspiPush( pRficDev->pDspiHandle, DSPI_CS0, DSPI_DEV_WRITE,
		      ucData, 4 );
    if( 0 > iRet )
    {
	log_err( "%s: lDspiPush failed, error[%d]\r\n", __func__, iRet );
    }

    /* Update in device structure */
    pRficDev->xVgaRegVal = data;

    return iRet;
}

int32_t RficVgaInit( RficDevice_t *pRficDev )
{
    int32_t iRet;
    uint8_t ucData = 0;

    /* Set bandwidth to 36MHz */
    ucData = RFIC_VGA_BW_36MHZ;

    /* Enable DC offset compensation */
    ucData = ucData | RFIC_VGA_DC_OFFSET_EN;

    /* Power up the VGA */
    ucData = ucData | RFIC_VGA_POWER_EN;

    iRet = RficVgaWriteReg( pRficDev, ucData );
    if( iRet < 0 )
    {
	log_err( "%s: Vga write reg failed\r\n", __func__ );
	return -1;
    }

    return 0;
}
