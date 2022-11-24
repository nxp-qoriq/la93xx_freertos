/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright 2021-2022 NXP
 */
#ifndef __RFIC_API_H
#define __RFIC_API_H
#include <stdint.h>
#include "types.h"

typedef enum RficBand {
    RFIC_BAND_N77,
    RFIC_BAND_B13,
    RFIC_BAND_B3,
    RFIC_BAND_GNSS,
    RFIC_BAND_MAX
}RficBand_t;

typedef enum RficResp {
    RFIC_SUCCESS,
    RFIC_CMD_INVALID,
    RFIC_PARAM_INVALID,
    RFIC_ERROR,
    RFIC_NOT_IMPLEMENTED
}RficResp_t;

typedef void *RficHandle_t;

/*!
 * @brief xRficLibInit
 * Initialize the RF card and return the handle.
 *
 * @return : RFIC handler on success; NULL in case of failure
 */
RficHandle_t xRficLibInit(void);

/*!
 * @brief xRficSetBand
 * To select the required band.
 *
 * @param handle: NLM RFIC handler
 * @param band  : Band identifier; 0-N77, 1-B13, 2-B3, 3-GNSS (Default value - 1)
 *
 * @return      : 0 for success; error number in case of failure
 */
RficResp_t xRficSetBand(RficHandle_t handle, RficBand_t band);

/*!
 * @brief xRficAdjustPllFreq
 * To set the required frequency.
 *
 * @param handle      : NLM RFIC handler
 * @param freq_in_khz : Frequency in KHz (Default value - 751000KHz)
 *
 * @return            : 0 for success; error number in case of failure
 */
RficResp_t xRficAdjustPllFreq(RficHandle_t handle, uint32_t freq_in_khz);

/*!
 * @brief xRficLnaCtrl
 * To control the second LNA of current band.
 *
 * @param handle : NLM RFIC handler
 * @param state  : Set 0 to bypass LNA; set 1 to keep LNA (Default value - 0)
 *
 * @return       : 0 for success; error number in case of failure
 */
RficResp_t xRficLnaCtrl(RficHandle_t handle, bool_t state);

/*!
 * @brief xRficDemodGainCtrl
 * To control the Demodulator gain and attenuator
 *
 * @param handle  : NLM RFIC handler
 * @param rf_attn : RF attenuator (range: 0-31; 0 means 0dB & 31 means 31dB)
 *                                 Default value - 0
 * @param bb_gain : Baseband gain (range: 0-7; 0 means 8dB & 7 means 15dB)
 *                                 Default value - 6
 *
 * @return        : 0 for success; error number in case of failure
 */
RficResp_t xRficDemodGainCtrl(RficHandle_t handle, uint8_t rf_attn, uint8_t bb_gain);

/*!
 * @brief xRficAdrfGainCtrl
 * To control the ADRF6520 gain
 *
 * @param handle : NLM RFIC handler
 * @param gain   : Gain value (range: 0-60; 0 means -7dB & 60 means 53dB)
 *				   Default value - 30
 *
 * @return       : 0 for success; error number in case of failure
 */
RficResp_t xRficAdrfGainCtrl(RficHandle_t handle, uint8_t gain);

/*!
 * @brief xRficFastCal
 * PLL fast Calibration
 *
 * @param handle : NLM RFIC handler
 *
 * @return       : 0 for success; error number in case of failure
 */
RficResp_t xRficFastCal( RficHandle_t handle );

/*!
 * @brief xRficIQDump
 * Dump IQ samples on console after receiving response from VSPA
 * VSPA will start captruing IQ samples from AXIQ to LA931x memory and send response
 *
 * @param handle   : NLM RFIC handler
 * @param size     : size of IQ samples to dump to a file
 *                   size should be in number of 4KB blocks, valid is 1 to 10 
 *
 * @return       : 0 for success; error number in case of failure
 */
RficResp_t xRficIQDump( RficHandle_t handle, uint32_t size);

/*!
 * @brief xRficGetRFValues
 * Prints all configured values
 *
 * @param handle   : NLM RFIC handler
 *
 */
void xRficGetRFConf( RficHandle_t handle);
#endif //__RFIC_API_H
