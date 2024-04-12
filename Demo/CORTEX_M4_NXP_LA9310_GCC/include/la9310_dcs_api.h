/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright 2021, 2024 NXP
 */

#ifndef __LA9310DCS_API_H__
#define __LA9310DCS_API_H__

typedef enum LA9310cvrDCS{
    XCVR_TRX_TX_DAC = 0,
    XCVR_TRX_RX1_ADC,
    XCVR_TRX_RX2_ADC,
    XCVR_RO1_ADC,
    XCVR_RO2_ADC,
    AUX_ADC,
}LA9310XcvrDCS_t;

typedef enum DCSFreq {
	Full_Freq, /* 122.88 MHz */
	Half_Freq, /* 61.44MHz  */
}DCSFreq_t;

/* Use LA9310_DCS_BITMASK macro to generate
 * bitmask of any of LA9310XcvrDCS. This is required
 * for ADC/DAC API which operates on more than one ADC/DAC module
 */
#define LA9310_DCS_BITMASK (DCS) (1 << DCS)
#define LA9310_DCS_CHECK_MASK(DCS, mask) (LA9310_DCS_BITMASK(DCS) & mask)

/*!
 * @brief vDcsInit
 *
 * Init DCS
 * @param adc_mask: ADC channel enable  mask
 * @param adc_freq_mask: ADC chanel frequency mask
 * @param dac_mask: DAC channel enable mask
 * @param dac_freq_mask: DAC channel frequency mask
*/
void vDcsInit( int adc_mask, int adc_freq_mask, int dac_mask, int dac_freq_mask);

/*!
 * @brief vLA9310DCSClockSwitch
 *
 * Configure ADC/DAC clock
 * @param dcs: DCS channel type
 * @param freq: frequency
 * @return: pdPASS on success, error number in case of failure
 */
BaseType_t vLa9310DCSClockSwitch( LA9310XcvrDCS_t dcs,  DCSFreq_t freq );

#endif
