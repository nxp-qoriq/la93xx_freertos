/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright 2021 NXP
 */

#ifndef __LA9310DCS_API_H__
#define __LA9310DCS_API_H__

typedef enum LA9310cvrDCS{
    XCVR_TRX_TX_DAC,
    XCVR_TRX_RX1_ADC,
    XCVR_TRX_RX2_ADC,
    XCVR_RO1_ADC,
    XCVR_RO2_ADC,
    AUX_ADC,
}LA9310XcvrDCS_t;

typedef enum DCSFreq {
    Full_Freq,
    Half_Freq,
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
 * @param freq: DAC frequency
*/
void vDcsInit(DCSFreq_t Freq);

/*!
 * @brief vLA9310DacClockSwitch
 *
 * Configure DAC clock
 * @param freq: DAC frequency
 * @return: pdPASS on success, error number in case of failure
 */
BaseType_t vLA9310DacClockSwitch(DCSFreq_t freq);

#endif
