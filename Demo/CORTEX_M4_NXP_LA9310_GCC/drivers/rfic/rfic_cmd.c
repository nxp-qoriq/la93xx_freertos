/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright 2021-2024 NXP
 */
#include "rfic_cmd.h"
#include "rfic_synth.h"
#include "rfic_demod.h"
#include "rfic_vga.h"
#include "rfic_sw_cmd.h"
#include "rfic_avi_ctrl.h"
#include "rfic_dac.h"
#include <la9310_irq.h>

int32_t iRficCtrlFemSwitch( RficDevice_t *pRficDev, rf_band_t band )
{
    uint8_t SwNum = ( band <= RF_SW_BAND_B13 ) ? RF_GPIO_RFA_PATH_SEL : RF_GPIO_RFB_PATH_SEL;
    uint8_t SwCtrlVal[4][2] = { /* Demod Sel, Switch Sel */
                               { 1, 1 },    /* N77 */
                               { 1, 0 },    /* B13 */
                               { 0, 1 },    /* B3 */
                               { 0, 0 }     /* GNSS */
                              };

    if( iGpioSetData( pRficDev->eGpio[ RF_GPIO_DEMOD_PATH_SEL ].ulPin,
                      SwCtrlVal[ band ][ 0 ] ))
    {
	log_err( "%s: GPIO-%d set failed\r\n", __func__,
	         pRficDev->eGpio[ RF_GPIO_DEMOD_PATH_SEL ].ulPin );
	return -1;
    }

    if( iGpioSetData( pRficDev->eGpio[ SwNum ].ulPin, SwCtrlVal[ band ][ 1 ] ))
    {
	log_err( "%s: GPIO-%d set failed\r\n", __func__,
	         pRficDev->eGpio[ RF_GPIO_RFA_PATH_SEL ].ulPin );
	return -1;
    }

    return 0;
}

void xRficProcessSetBand( RficDevice_t *pRficDev, rf_sw_cmd_desc_t *pSwCmdDesc )
{
    struct sw_cmddata_set_band *CmdData;
    rf_priv_mdata_t *mdata = &pRficDev->pRfHif->rf_priv_mdata;

    CmdData = ( struct sw_cmddata_set_band * )&pSwCmdDesc->data[0];
    log_dbg( "%s: Band[%d]\n\r", __func__, CmdData->band );

    /* Configure FEM */
    if( iRficCtrlFemSwitch( pRficDev, CmdData->band ))
    {
        log_err("%s: FEM Ctrl failed\r\n", __func__);
        goto err;
    }

    /* Configure Demod LO Matching */
    if( RficDemodLoMatch( pRficDev, CmdData->band, RFIC_DEMOD_N77_LO_SHIFT ))
    {
        log_err("%s: Demod LO matching failed\r\n", __func__);
        goto err;
    }

    /* Update mdata */
    mdata->band = CmdData->band;

    pSwCmdDesc->result = RF_SW_CMD_RESULT_OK;
    return;
err:
    pSwCmdDesc->result = RF_SW_CMD_RESULT_ERROR;
    return;
}

void xRficProcessAdjustPllFreq( RficDevice_t *pRficDev, rf_sw_cmd_desc_t *pSwCmdDesc )
{
    struct sw_cmddata_adjust_pll_freq *CmdData;
    rf_priv_mdata_t *mdata = &pRficDev->pRfHif->rf_priv_mdata;

    CmdData = ( struct sw_cmddata_adjust_pll_freq * )&pSwCmdDesc->data[0];
    log_dbg( "%s: Freq[%d]\n\r", __func__, CmdData->freq_khz );

    /* Select Synthesizer */
    if( iRficSelectDspiSlave( pRficDev, RF_DSPI_SLV_SYNTH ))
    {
        log_err( "%s: Select DSPI Slave failed\n\r", __func__ );
        goto err;
    }

    /* Program synthesizer for required freq */
    RficSynthAdjustPllFreq( pRficDev, CmdData->freq_khz );

    /* Configure Demod LO Matching */
    if( RF_SW_BAND_N77 == mdata->band )
    {
        if( RficDemodLoMatch( pRficDev, mdata->band, CmdData->freq_khz ))
        {
            log_err("%s: Demod LO matching failed\r\n", __func__);
            goto err;
        }
    }

    /* Update mdata */
    mdata->freq_khz = CmdData->freq_khz;

    pSwCmdDesc->result = RF_SW_CMD_RESULT_OK;
    return;
err:
    pSwCmdDesc->result = RF_SW_CMD_RESULT_ERROR;
    return;
}

void xRficProcessCtrlLna( RficDevice_t *pRficDev, rf_sw_cmd_desc_t *pSwCmdDesc )
{
    struct sw_cmddata_ctrl_lna *CmdData;
    rf_priv_mdata_t *mdata = &pRficDev->pRfHif->rf_priv_mdata;
    uint32_t uGpio;

    CmdData = ( struct sw_cmddata_ctrl_lna * )&pSwCmdDesc->data[0];
    log_dbg( "%s: State[%d]\n\r", __func__, CmdData->state );

    if( RF_SW_BAND_N77 == mdata->band )
        uGpio = RF_GPIO_BAND_N77_LNA_EN;
    else if( RF_SW_BAND_B13 == mdata->band )
	uGpio = RF_GPIO_BAND_B13_LNA_EN;
    else if( RF_SW_BAND_B3 ==  mdata->band )
	uGpio = RF_GPIO_BAND_B3_LNA_EN;
    else
	goto err;

    if( iGpioSetData( pRficDev->eGpio[ uGpio ].ulPin, CmdData->state ))
    {
        log_err( "%s: GPIO-%d set failed\r\n", __func__,
		 pRficDev->eGpio[ uGpio ].ulPin );
        goto err;
    }

    /* Update mdata */
    mdata->lna_state &= (~( 1 << mdata->band ));
    mdata->lna_state |= ( CmdData->state << mdata->band );

    pSwCmdDesc->result = RF_SW_CMD_RESULT_OK;
    return;
err:
    pSwCmdDesc->result = RF_SW_CMD_RESULT_ERROR;
    return;
}

void xRficProcessReadReg( RficDevice_t *pRficDev, rf_sw_cmd_desc_t *pSwCmdDesc )
{
    struct sw_cmddata_reg_opr *CmdData;
    int32_t iRet = 0;
    uint32_t val32;
    uint8_t val8;

    CmdData = ( struct sw_cmddata_reg_opr * )&pSwCmdDesc->data[0];
    log_dbg( "%s: DspiSlvId[%d], Addr[%x]\n\r", __func__,
	      CmdData->dspi_slv_id, CmdData->addr );

    /* Select DPSI Slave Device */
    if( iRficSelectDspiSlave( pRficDev, CmdData->dspi_slv_id ))
    {
        log_err( "%s: Select DSPI Slave failed\n\r", __func__ );
        goto err;
    }

    /* Call the respective read register */
    if( RF_DSPI_SLV_SYNTH == CmdData->dspi_slv_id )
    {
        iRet = RficSynthReadReg( pRficDev, ( uint8_t )CmdData->addr,
                                 &val32 );
        CmdData->val = val32;
    }
    else if( RF_DSPI_SLV_VGA == CmdData->dspi_slv_id )
        CmdData->val = pRficDev->xVgaRegVal;
    else if ( RF_DSPI_SLV_DEMOD == CmdData->dspi_slv_id )
    {
        iRet = RficDemodReadReg( pRficDev, ( uint8_t )CmdData->addr,
                                 &val8 );
        CmdData->val = val8;
    }
    else
	goto err;

    if( iRet )
    {
         log_err( "%s: Read register failed\n\r", __func__ );
	 goto err;
    }

    pSwCmdDesc->result = RF_SW_CMD_RESULT_OK;
    return;
err:
    pSwCmdDesc->result = RF_SW_CMD_RESULT_ERROR;
    return;
}

void xRficProcessWriteReg( RficDevice_t *pRficDev, rf_sw_cmd_desc_t *pSwCmdDesc )
{
    struct sw_cmddata_reg_opr *CmdData;
    int32_t iRet;

    CmdData = ( struct sw_cmddata_reg_opr * )&pSwCmdDesc->data[0];
    log_dbg( "%s: DspiSlvId[%d], Addr[%x], val[%x]\n\r", __func__,
              CmdData->dspi_slv_id, CmdData->addr, CmdData->val );

    /* Select DPSI Slave Device */
    if( iRficSelectDspiSlave( pRficDev, CmdData->dspi_slv_id ))
    {
        log_err( "%s: Select DSPI Slave failed\n\r", __func__ );
        goto err;
    }

    /* Call the respective write register */
    if( RF_DSPI_SLV_SYNTH == CmdData->dspi_slv_id )
        iRet = RficSynthWriteReg( pRficDev, ( uint8_t )CmdData->addr,
                                 ( uint32_t )CmdData->val );
    else if( RF_DSPI_SLV_VGA == CmdData->dspi_slv_id )
        iRet = RficVgaWriteReg( pRficDev, ( uint8_t )CmdData->val );
    else if ( RF_DSPI_SLV_DEMOD == CmdData->dspi_slv_id )
        iRet = RficDemodWriteReg( pRficDev, ( uint8_t )CmdData->addr,
                                 ( uint8_t )CmdData->val );
    else
        goto err;

    if( iRet )
    {
         log_err( "%s: Write register failed\n\r", __func__ );
         goto err;
    }

    pSwCmdDesc->result = RF_SW_CMD_RESULT_OK;
    return;
err:
    pSwCmdDesc->result = RF_SW_CMD_RESULT_ERROR;
    return;
}

void vRficProcessIqDump(RficDevice_t *pRficDev, rf_sw_cmd_desc_t *rfic_sw_cmd)
{
        BaseType_t xRet = pdFAIL;
        struct sw_cmddata_dump_iq *cmd_data;
        struct la9310_mbox_v2h mbox_v2h = {0};
        struct la9310_mbox_h2v mbox_h2v = {0};

        cmd_data = ((struct sw_cmddata_dump_iq *)rfic_sw_cmd->data);

        mbox_h2v.ctrl.op_code = IQ_MODULATED_RX;
        mbox_h2v.ctrl.bandwidth = 0;
        mbox_h2v.ctrl.rcvr = 0;
        mbox_h2v.msbl16 = cmd_data->size;
#ifdef FLOOD
        cmd_data->addr = pRficDev->iq_phys_addr;
#endif
        mbox_h2v.lsb32 = cmd_data->addr;
        vLa9310MbxSend(&mbox_h2v);

        log_info("starting iqflood with addr %p\r\n", cmd_data->addr);

#ifdef FLOOD
read_again:
#endif
                xRet = vLa9310MbxReceive(&mbox_v2h);

        if ((pdFAIL == xRet) || (0 != mbox_v2h.status.err_code))
        {
                log_err("%s: iqdump failed error[%d]", __func__, mbox_v2h.status.err_code);
                rfic_sw_cmd->result = RF_SW_CMD_RESULT_ERROR;
        }
        else
        {
                rfic_sw_cmd->result = RF_SW_CMD_RESULT_OK;
        }

#ifdef FLOOD
        //log_info("iqdump mbox is %08x \r\n", mbox_v2h.msb32);

        if(0 && (mbox_v2h.msb32 & 0xf0) == 0x80) {
            
            vRaiseMsi( pLa9310Info, MSI_IRQ_FLOOD_0 );

            uint32_t *bufferStatusPtr = (uint32_t*) (cmd_data->addr + (1024 * 1024 * 17));
            *bufferStatusPtr = mbox_v2h.msb32;

            //log_info("set %p off %x to %08x \r\n", bufferStatusPtr, cmd_data->addr, *bufferStatusPtr);
            
            goto read_again;
        }
#endif
        return;
}


void vRficGetRxDcOffset(rf_sw_cmd_desc_t *rfic_sw_cmd)
{
    BaseType_t xRet = pdFAIL;
    struct sw_cmddata_reg_opr *cmd_data;
    struct la9310_mbox_v2h mbox_v2h = {0};
    struct la9310_mbox_h2v mbox_h2v = {0};

    cmd_data = ((struct sw_cmddata_reg_opr *)rfic_sw_cmd->data);

    mbox_h2v.ctrl.op_code = DCOC_CAL;
    //mbox_h2v.ctrl.start_stop = cmd_data->start_stop;
    //mbox_h2v.ctrl.bandwidth = 0;
    mbox_h2v.ctrl.rcvr = 0;
    //mbox_h2v.msbl16 = cmd_data->size;
    //mbox_h2v.lsb32 = cmd_data->addr;

    vLa9310MbxSend(&mbox_h2v);
    xRet = vLa9310MbxReceive(&mbox_v2h);

    if ((pdFAIL == xRet) || (0 != mbox_v2h.status.err_code))
    {
        log_err("%s: dc offset failed error[%d] \r\n", __func__, mbox_v2h.status.err_code);
        rfic_sw_cmd->result = RF_SW_CMD_RESULT_ERROR;
    }
    else
    {
        rfic_sw_cmd->result = RF_SW_CMD_RESULT_OK;

        cmd_data->val = mbox_v2h.msb32;

        log_err("%s: dc offset might be %08x \r\n", __func__, mbox_v2h.msb32);
    }

    return;
}

void vRficSetDcOffset(rf_sw_cmd_desc_t *rfic_sw_cmd)
{
    BaseType_t xRet = pdFAIL;
    struct sw_cmddata_dump_iq *cmd_data;
    struct la9310_mbox_v2h mbox_v2h = {0};
    struct la9310_mbox_h2v mbox_h2v = {0};

    cmd_data = ((struct sw_cmddata_dump_iq *)rfic_sw_cmd->data);

    mbox_h2v.ctrl.op_code = TX_DC_CORRECTION;
    //mbox_h2v.ctrl.start_stop = cmd_data->start_stop;
    //mbox_h2v.ctrl.bandwidth = 0;
    mbox_h2v.ctrl.rcvr = 0;
    //mbox_h2v.msbl16 = cmd_data->size;
    mbox_h2v.lsb32 = cmd_data->addr;

    vLa9310MbxSend(&mbox_h2v);
    xRet = vLa9310MbxReceive(&mbox_v2h);

    if ((pdFAIL == xRet) || (0 != mbox_v2h.status.err_code))
    {
        log_err("%s: dc offset failed error[%d] \r\n", __func__, mbox_v2h.status.err_code);
        rfic_sw_cmd->result = RF_SW_CMD_RESULT_ERROR;
    }
    else
    {
        rfic_sw_cmd->result = RF_SW_CMD_RESULT_OK;

//        log_err("%s: dc offset set to %08x \r\n", __func__, mbox_h2v.lsb32);

    int16_t i, q;

    i = mbox_v2h.msb32 >> 16;
    q = mbox_v2h.msb32 & 0xffff;

        if(mbox_v2h.msb32 != mbox_h2v.lsb32) {
            log_err("%s: got back something else %08x vs %08x \r\n", __func__, mbox_v2h.msb32, mbox_h2v.lsb32);
        }

        log_err("%s: dc offset is %d and %d \r\n", __func__, i, q);
    }

    return;
}

void vRficSetIqImbalance(rf_sw_cmd_desc_t *rfic_sw_cmd)
{
    BaseType_t xRet = pdFAIL;
    struct sw_cmddata_dump_iq *cmd_data;
    struct la9310_mbox_v2h mbox_v2h = {0};
    struct la9310_mbox_h2v mbox_h2v = {0};

    cmd_data = ((struct sw_cmddata_dump_iq *)rfic_sw_cmd->data);

    mbox_h2v.ctrl.op_code = RFNM_IQ_IMBALANCE;
    //mbox_h2v.ctrl.start_stop = cmd_data->start_stop;
    //mbox_h2v.ctrl.bandwidth = 0;
    mbox_h2v.ctrl.rcvr = 0;
    //mbox_h2v.msbl16 = cmd_data->size;
    mbox_h2v.lsb32 = cmd_data->addr;

    vLa9310MbxSend(&mbox_h2v);
    xRet = vLa9310MbxReceive(&mbox_v2h);

    if ((pdFAIL == xRet) || (0 != mbox_v2h.status.err_code))
    {
        log_err("%s: dc imbalance failed error[%d] \r\n", __func__, mbox_v2h.status.err_code);
        rfic_sw_cmd->result = RF_SW_CMD_RESULT_ERROR;
    }
    else
    {
        rfic_sw_cmd->result = RF_SW_CMD_RESULT_OK;

//        log_err("%s: dc offset set to %08x \r\n", __func__, mbox_h2v.lsb32);

    int16_t i, q;

    i = mbox_v2h.msb32 >> 16;
    q = mbox_v2h.msb32 & 0xffff;

        if(mbox_v2h.msb32 != mbox_h2v.lsb32) {
            log_err("%s: got back something else %08x vs %08x \r\n", __func__, mbox_v2h.msb32, mbox_h2v.lsb32);
        }

        log_err("%s: dc imbalance is %d and %d \r\n", __func__, i, q);
    }

    return;
}

void vRficSetChannel(rf_sw_cmd_desc_t *rfic_sw_cmd)
{
    BaseType_t xRet = pdFAIL;
    struct sw_cmddata_dump_iq *cmd_data;
    struct la9310_mbox_v2h mbox_v2h = {0};
    struct la9310_mbox_h2v mbox_h2v = {0};

    cmd_data = ((struct sw_cmddata_dump_iq *)rfic_sw_cmd->data);

    mbox_h2v.ctrl.op_code = RFNM_SET_CHANNEL;
    //mbox_h2v.ctrl.start_stop = cmd_data->start_stop;
    //mbox_h2v.ctrl.bandwidth = 0;
    mbox_h2v.ctrl.rcvr = 0;
    //mbox_h2v.msbl16 = cmd_data->size;
    mbox_h2v.lsb32 = cmd_data->addr;

    vLa9310MbxSend(&mbox_h2v);
    xRet = vLa9310MbxReceive(&mbox_v2h);

    if ((pdFAIL == xRet) || (0 != mbox_v2h.status.err_code))
    {
        log_err("%s: failed to set channel [%d] \r\n", __func__, mbox_v2h.status.err_code);
        rfic_sw_cmd->result = RF_SW_CMD_RESULT_ERROR;
    }
    else
    {
        rfic_sw_cmd->result = RF_SW_CMD_RESULT_OK;

        log_err("%s: channel set to %08x \r\n", __func__, mbox_h2v.lsb32);
    }

    return;
}

void vRficProcessTXIqData(rf_sw_cmd_desc_t *rfic_sw_cmd)
{
    BaseType_t xRet = pdFAIL;
    struct sw_cmddata_dump_iq *cmd_data;
    struct la9310_mbox_v2h mbox_v2h = {0};
    struct la9310_mbox_h2v mbox_h2v = {0};

    cmd_data = ((struct sw_cmddata_dump_iq *)rfic_sw_cmd->data);

    mbox_h2v.ctrl.op_code = IQ_MODULATED_TX;
    mbox_h2v.ctrl.start_stop = cmd_data->start_stop;
    mbox_h2v.ctrl.bandwidth = 0;
    mbox_h2v.ctrl.rcvr = 0;
    mbox_h2v.msbl16 = cmd_data->size;
    mbox_h2v.lsb32 = cmd_data->addr;

    vLa9310MbxSend(&mbox_h2v);
    xRet = vLa9310MbxReceive(&mbox_v2h);

    if ((pdFAIL == xRet) || (0 != mbox_v2h.status.err_code))
    {
        log_err("%s: iqdump failed error[%d]", __func__, mbox_v2h.status.err_code);
        rfic_sw_cmd->result = RF_SW_CMD_RESULT_ERROR;
    }
    else
    {
        rfic_sw_cmd->result = RF_SW_CMD_RESULT_OK;
    }

    return;
}

void vRficProcessSingleToneTX( rf_sw_cmd_desc_t * rfic_sw_cmd )
{
    BaseType_t xRet = pdFAIL;
    struct sw_cmddata_single_tone_TX * cmd_data;
    struct la9310_mbox_v2h mbox_v2h = { 0 };
    struct la9310_mbox_h2v mbox_h2v = { 0 };

    cmd_data = ( ( struct sw_cmddata_single_tone_TX * ) rfic_sw_cmd->data );

    mbox_h2v.ctrl.op_code = SINGLE_TONE_TX;
    mbox_h2v.ctrl.start_stop = cmd_data->start_stop;
    mbox_h2v.msbl16 = 0;
    mbox_h2v.lsb32 = cmd_data->tone_index & 0x000000FF;


    vLa9310MbxSend( &mbox_h2v );
    xRet = vLa9310MbxReceive( &mbox_v2h );

    if( ( pdFAIL == xRet ) || ( 0 != mbox_v2h.status.err_code ) )
    {
        log_err( "%s: single tone TX failed with  error[%d]", __func__, mbox_v2h.status.err_code );
        rfic_sw_cmd->result = RF_SW_CMD_RESULT_ERROR;
        return;
    }

    rfic_sw_cmd->result = RF_SW_CMD_RESULT_OK;

    return;
}

void vRficProcessLoopback( RficDevice_t * pRficDev,
                           rf_sw_cmd_desc_t * rfic_sw_cmd )
{
    struct sw_cmddata_set_loopback * cmd_data;
    uint32_t loopback = pRficDev->xLoopback;

    cmd_data = ( ( struct sw_cmddata_set_loopback * ) rfic_sw_cmd->data );

    if( cmd_data->loopback_type == NO_LOOPBACK )      /* disable all loopback */
    {
        if( loopback == AXIQ_LOOPBACK )
        {
            OUT_32( DBGGNCR, ( REMOVE_AXIQ_LOOPBACK_MASK &
                               IN_32( DBGGNCR ) ) );
        }
        else
        {
            rfic_sw_cmd->result = RF_SW_CMD_RESULT_CMD_PARAMS_INVALID;
            log_err( "\r\n%s: No looopback set \r\n", __func__ );
        }
    }
    else
    {
        if( cmd_data->loopback_type == AXIQ_LOOPBACK )
        {
            OUT_32( DBGGNCR, ( SET_AXIQ_LOOPBACK_MASK |
                               IN_32( DBGGNCR ) ) );
        }
    }

    rfic_sw_cmd->result = RF_SW_CMD_RESULT_OK;

    return;
}

void xRficProcessCtrlDemodGain( RficDevice_t *pRficDev, rf_sw_cmd_desc_t *pSwCmdDesc )
{
    BaseType_t xRet;
    struct sw_cmddata_demod_gain *CmdData;
    rf_priv_mdata_t *mdata = &pRficDev->pRfHif->rf_priv_mdata;
    CmdData = ( struct sw_cmddata_demod_gain * )&pSwCmdDesc->data[0];
    log_dbg( "%s: Rf Attn : %d , BB Gain: %d \n\r", __func__, CmdData->rf_attn, CmdData->bb_gain );

    /* Update gain settings in demod registers */
    xRet = RficDemodGainCtrl(pRficDev, CmdData->rf_attn, CmdData->bb_gain);
    if( xRet < 0 )
    {
        log_err( "%s: Ctrl Demod Gain Failed, error[%d]\n\r", __func__, xRet );
        goto err;
    }

    /* Update mdata*/
    mdata->demod_rf_attn = CmdData->rf_attn;
    mdata->demod_bb_gain = CmdData->bb_gain;

    pSwCmdDesc->result = RF_SW_CMD_RESULT_OK;
    return;
err:
    pSwCmdDesc->result = RF_SW_CMD_RESULT_ERROR;
    return;
}

void xRficProcessCtrlVgaGain( RficDevice_t *pRficDev, rf_sw_cmd_desc_t *pSwCmdDesc )
{
    struct sw_cmddata_vga_gain *CmdData;
    rf_priv_mdata_t *mdata = &pRficDev->pRfHif->rf_priv_mdata;
    CmdData = ( struct sw_cmddata_vga_gain * )&pSwCmdDesc->data[0];

    /* Writing val for DAC1*/
    if( mdata->vga_dac1_val != CmdData->dac1_val )
    {
        prvDacWriteUpdate(CmdData->dac1_val, DAC1_I2C_ADDR);

	/* Update mdata*/
	mdata->vga_dac1_val = CmdData->dac1_val;
    }

    /* Writing val for DAC2*/
    if( mdata->vga_dac2_val != CmdData->dac2_val )
    {
        prvDacWriteUpdate(CmdData->dac2_val, DAC2_I2C_ADDR);

	/* Update mdata*/
	mdata->vga_dac2_val = CmdData->dac2_val;
    }

    /*TODO : Error handling part is missing as I2C API doesn't return any pass
     * or fail val*/

    pSwCmdDesc->result = RF_SW_CMD_RESULT_OK;
    return;
}

void xRficProcessFastCalib( RficDevice_t *pRficDev )
{
    RficSynthAdjustPllFastCal( pRficDev );
}

