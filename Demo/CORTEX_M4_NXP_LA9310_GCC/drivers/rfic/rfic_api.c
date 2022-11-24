/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright 2021-2022 NXP
 */
#include "rfic_api.h"
#include "rfic_core.h"
#include "la9310_main.h"
#include "debug_console.h"
#include "rfic_sw_cmd.h"
#include "rfic_demod.h"
#include "rfic_vga.h"

RficHandle_t xRficLibInit( void )
{
    struct RficDevice *pRficDev = pLa9310Info->pRficDev;
    rf_host_if_t *pRfHif = pRficDev->pRfHif;

    if( RF_READY != pRfHif->ready)
	return NULL;

    return (RficHandle_t)pRficDev;
}

RficResp_t xRficSetBand( RficHandle_t handle, RficBand_t band )
{
    RficDevice_t *pRficDev = (RficDevice_t *)handle;
    rf_sw_cmd_desc_t SwCmd;
    struct sw_cmddata_set_band *CmdData;

    if( RFIC_BAND_N77 > band || RFIC_BAND_MAX <= band )
	return RFIC_PARAM_INVALID;

    memset( &SwCmd, 0, sizeof( rf_sw_cmd_desc_t ));

    SwCmd.cmd = RF_SW_CMD_SET_BAND;
    SwCmd.flags = RF_SW_CMD_LOCAL;
    CmdData = (struct sw_cmddata_set_band *) &SwCmd.data[0];

    CmdData->band = band;

    #ifdef TURN_ON_STANDALONE_MODE
	    xHandleSwCmd( pRficDev, &SwCmd);
    #else
	    if( pdPASS != xRficPostLocalSwCmd( pRficDev, &SwCmd ))
	        return RFIC_ERROR;
    #endif
 
    if( RF_SW_CMD_RESULT_OK != SwCmd.result )
	return RFIC_ERROR;

    return RFIC_SUCCESS;
}

RficResp_t xRficAdjustPllFreq( RficHandle_t handle, uint32_t freq_in_khz )
{
    RficDevice_t *pRficDev = (RficDevice_t *)handle;
    rf_sw_cmd_desc_t SwCmd;
    struct sw_cmddata_adjust_pll_freq *CmdData;
    uint32_t band;

    band = pRficDev->pRfHif->rf_priv_mdata.band;

    switch( band )
    {
        case RFIC_BAND_N77:
            if( 3300000 > freq_in_khz || 4200000 < freq_in_khz )
                return RFIC_PARAM_INVALID;
            break;

        case RFIC_BAND_B13:
            if( 746000 > freq_in_khz || 756000 < freq_in_khz )
                return RFIC_PARAM_INVALID;
            break;

        case RFIC_BAND_B3:
            if( 1805000 > freq_in_khz || 1880000 < freq_in_khz )
                return RFIC_PARAM_INVALID;
            break;

        case RFIC_BAND_GNSS:
            if( 1176450 > freq_in_khz || 1609313 < freq_in_khz )
                return RFIC_PARAM_INVALID;
            break;

        default:
            return RFIC_ERROR;
    }

    memset( &SwCmd, 0, sizeof( rf_sw_cmd_desc_t ));

    SwCmd.cmd = RF_SW_CMD_ADJUST_PLL_FREQ;
    SwCmd.flags = RF_SW_CMD_LOCAL;
    CmdData = (struct sw_cmddata_adjust_pll_freq *) &SwCmd.data[0];
    CmdData->freq_khz = freq_in_khz;

    #ifdef TURN_ON_STANDALONE_MODE
	    xHandleSwCmd( pRficDev, &SwCmd);
    #else
	    if( pdPASS != xRficPostLocalSwCmd( pRficDev, &SwCmd ))
	        return RFIC_ERROR;
    #endif
 
    if( RF_SW_CMD_RESULT_OK != SwCmd.result )
        return RFIC_ERROR;

    return RFIC_SUCCESS;
}

RficResp_t xRficFastCal( RficHandle_t handle)
{
    RficDevice_t *pRficDev = (RficDevice_t *)handle;
    rf_sw_cmd_desc_t SwCmd;

    memset( &SwCmd, 0, sizeof( rf_sw_cmd_desc_t ));

    SwCmd.cmd = RF_SW_CMD_FAST_CALIB;
    SwCmd.flags = RF_SW_CMD_LOCAL;

    #ifdef TURN_ON_STANDALONE_MODE
	    xHandleSwCmd( pRficDev, &SwCmd);
    #else
	    if( pdPASS != xRficPostLocalSwCmd( pRficDev, &SwCmd ))
	        return RFIC_ERROR;
    #endif
 
    if( RF_SW_CMD_RESULT_OK != SwCmd.result )
	    return RFIC_ERROR;

    return RFIC_SUCCESS;
}

RficResp_t xRficLnaCtrl(RficHandle_t handle, bool_t state)
{
    RficDevice_t *pRficDev = (RficDevice_t *)handle;
    rf_sw_cmd_desc_t SwCmd;
    struct sw_cmddata_ctrl_lna *CmdData;

    memset( &SwCmd, 0, sizeof( rf_sw_cmd_desc_t ));

    SwCmd.cmd = RF_SW_CMD_CTRL_LNA;
    SwCmd.flags = RF_SW_CMD_LOCAL;
    CmdData = (struct sw_cmddata_ctrl_lna *) &SwCmd.data[0];

    CmdData->state = state;

    #ifdef TURN_ON_STANDALONE_MODE
	    xHandleSwCmd( pRficDev, &SwCmd);
    #else
	    if( pdPASS != xRficPostLocalSwCmd( pRficDev, &SwCmd ))
	        return RFIC_ERROR;
    #endif
 
    if( RF_SW_CMD_RESULT_OK != SwCmd.result )
        return RFIC_ERROR;

    return RFIC_SUCCESS;
}

RficResp_t xRficDemodGainCtrl(RficHandle_t handle, uint8_t rf_attn, uint8_t bb_gain)
{
    RficDevice_t *pRficDev = (RficDevice_t *)handle;
    rf_sw_cmd_desc_t SwCmd;
    struct sw_cmddata_demod_gain *CmdData;

    if( RFIC_DEMOD_GAIN_MAX < bb_gain || RFIC_DEMOD_ATTN_MAX < rf_attn)
        return RFIC_PARAM_INVALID;

    memset( &SwCmd, 0, sizeof( rf_sw_cmd_desc_t ));

    SwCmd.cmd = RF_SW_CMD_CTRL_DEMOD_GAIN;
    SwCmd.flags = RF_SW_CMD_LOCAL;
    CmdData = (struct sw_cmddata_demod_gain *) &SwCmd.data[0];

    CmdData->bb_gain = bb_gain;
    CmdData->rf_attn = rf_attn;

    #ifdef TURN_ON_STANDALONE_MODE
	    xHandleSwCmd( pRficDev, &SwCmd);
    #else
	    if( pdPASS != xRficPostLocalSwCmd( pRficDev, &SwCmd ))
	        return RFIC_ERROR;
    #endif
 
    if( RF_SW_CMD_RESULT_OK != SwCmd.result )
        return RFIC_ERROR;

    return RFIC_SUCCESS;
}

RficResp_t xRficAdrfGainCtrl(RficHandle_t handle, uint8_t gain)
{
    RficDevice_t *pRficDev = (RficDevice_t *)handle;
    rf_sw_cmd_desc_t SwCmd;
    struct sw_cmddata_vga_gain *CmdData;

    if( 60 < gain )
        return RFIC_PARAM_INVALID;

    memset( &SwCmd, 0, sizeof( rf_sw_cmd_desc_t ));

    SwCmd.cmd = RF_SW_CMD_CTRL_VGA_GAIN;
    SwCmd.flags = RF_SW_CMD_LOCAL;
    CmdData = (struct sw_cmddata_vga_gain *) &SwCmd.data[0];

    CmdData->dac1_val = RficVgaGainTable[gain][0];
    CmdData->dac2_val = RficVgaGainTable[gain][1];

    #ifdef TURN_ON_STANDALONE_MODE
	    xHandleSwCmd( pRficDev, &SwCmd);
    #else
	    if( pdPASS != xRficPostLocalSwCmd( pRficDev, &SwCmd ))
	        return RFIC_ERROR;
    #endif

    if( RF_SW_CMD_RESULT_OK != SwCmd.result )
        return RFIC_ERROR;

    return RFIC_SUCCESS;
}

RficResp_t xRficIQDump( RficHandle_t handle, uint32_t size )
{
    RficDevice_t *pRficDev = (RficDevice_t *)handle;
    rf_sw_cmd_desc_t SwCmd;
    struct sw_cmddata_dump_iq *CmdData;
    volatile uint8_t  *tcm_ptr;

    memset( &SwCmd, 0, sizeof( rf_sw_cmd_desc_t ));
    SwCmd.cmd = RF_SW_CMD_DUMP_IQ_DATA;
    SwCmd.flags = RF_SW_CMD_LOCAL;
    CmdData = (struct sw_cmddata_dump_iq *) &SwCmd.data[0];

    CmdData->addr = 0x20001000;
    CmdData->size = size;
	xHandleSwCmd( pRficDev, &SwCmd);

    tcm_ptr = (uint8_t * ) (( uint32_t ) CmdData->addr);
    {
        uint32_t ulIndex;

        for( ulIndex = 0; ulIndex < size * 4096; ++ulIndex )
        {
            PRINTF( "%02x ", tcm_ptr[ ulIndex ] );

            if( ( ulIndex + 1 ) % 16 == 0 )
            {
                PRINTF( "\r\n" );
            }
        }

        PRINTF( "\n" );
        PRINTF( "\r\n" );
    }
    if( RF_SW_CMD_RESULT_OK != SwCmd.result )
	return RFIC_ERROR;

    return RFIC_SUCCESS;
}

void xRficGetRFConf( RficHandle_t handle )
{
    RficDevice_t *pRficDev = (RficDevice_t *)handle;

    PRINTF( "Configured band is %d \r\n", pRficDev->pRfHif->rf_priv_mdata.band);
    PRINTF( "Configured freq(khz) is %d \r\n", pRficDev->pRfHif->rf_priv_mdata.freq_khz);
    PRINTF( "Configured bandwidth is %d \r\n", pRficDev->pRfHif->rf_priv_mdata.bw);
    PRINTF( "Configured lna state is %d \r\n", pRficDev->pRfHif->rf_priv_mdata.lna_state);
    PRINTF( "Configured gain(db) is %d \r\n", pRficDev->pRfHif->rf_priv_mdata.gain_in_db);
    PRINTF( "Configured demod rf attn is %d \r\n", pRficDev->pRfHif->rf_priv_mdata.demod_rf_attn);
    PRINTF( "Configured demod bb gain is %d \r\n", pRficDev->pRfHif->rf_priv_mdata.demod_bb_gain);
    PRINTF( "Configured vga dac1 is %d \r\n", pRficDev->pRfHif->rf_priv_mdata.vga_dac1_val);
    PRINTF( "Configured vga dac2 is %d \r\n", pRficDev->pRfHif->rf_priv_mdata.vga_dac2_val);

}
