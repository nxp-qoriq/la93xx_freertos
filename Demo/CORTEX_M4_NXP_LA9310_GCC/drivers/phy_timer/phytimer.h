/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2021, 2024 NXP
 */

#ifndef __PHY_TIMER_H
#define __PHY_TIMER_H

#include <common.h>
#include <bit.h>

#define PHY_TMR_DIVISOR ( 1 )

#define PHY_TMR_FREQ    ( 61440000 )

/* PHY Timer Comparator flags */
/* Clear the interrupt notification bit for this comparator */
#define PHY_TIMER_COMPARATOR_CLEAR_INT     BIT( 7 )

/* Setting this flag disables this comparator. Enabling a comparator
 * is done by configuring it with a trigger value */
#define PHY_TIMER_COMPARATOR_DISABLE       BIT( 6 )

/* Setting this flag captures the current timer value as the comparator
 * value */
#define PHY_TIMER_COMPARATOR_CAPTURE       BIT( 5 )

/* Setting this flag enables compare equal to generate a cross-trigger
 * output to VSPA */
#define PHY_TIMER_COMPARATOR_CROSS_TRIG    BIT( 4 )

/* PHY Timer Status */
/* Current status of output signal */
#define PHY_TIMER_COMPARATOR_STATUS_OUT_HIGH    BIT( 31 )

/* If this is 0 compare equal did not occur, value of 1 means compare
 * equal occured */
#define PHY_TIMER_COMPARATOR_STATUS_INT         BIT( 7 )
/* if this is 1, comparator is enabled */
#define PHY_TIMER_COMPARATOR_STATUS_ENABLED     BIT( 6 )
#define PHY_TIMER_COMPARATOR_STATUS_MASK     \
    ( PHY_TIMER_COMPARATOR_STATUS_OUT_HIGH | \
      PHY_TIMER_COMPARATOR_STATUS_INT |      \
      PHY_TIMER_COMPARATOR_STATUS_ENABLED )

enum ePhyTimerComparatorTrigger
{
    ePhyTimerComparatorNoChange = 0x0,
    /** Comparator output signal set to '0' */
    ePhyTimerComparatorOut0,
    /** Comparator output signal set to '1' */
    ePhyTimerComparatorOut1,
    /** Comparator output signal is toggled */
    ePhyTimerComparatorOutToggle,
};

#define PHY_TIMER_COMPARATOR_COUNT              23
#define PHY_TIMER_BASE_ADDRESS                  ( 0x40000000 + 0x1020000 )

#define PHY_TIMER_COMP_VSPA_GO_0                0
#define PHY_TIMER_COMP_CH1_RX_ALLOWED           1
#define PHY_TIMER_COMP_CH2_RX_ALLOWED           2
#define PHY_TIMER_COMP_CH3_RX_ALLOWED           3
#define PHY_TIMER_COMP_CH4_RX_ALLOWED           4
#define PHY_TIMER_COMP_CH5_RX_ALLOWED           5
#define PHY_TIMER_COMP_CH6_RX_ALLOWED           6
#define PHY_TIMER_COMP_CH5_TX_ALLOWED           11
#define PHY_TIMER_COMP_VSPA_GO_1                12
#define PHY_TIMER_COMP_PPS_IN                   13
#define PHY_TIMER_COMP_PPS_OUT                  14
#define PHY_TIMER_COMP_RFCTL_0                  15
#define PHY_TIMER_COMP_RFCTL_1                  16
#define PHY_TIMER_COMP_RFCTL_2                  17
#define PHY_TIMER_COMP_RFCTL_3                  18
#define PHY_TIMER_COMP_RFCTL_4                  19
#define PHY_TIMER_COMP_RFCTL_5                  20

#define PHY_TIMER_COMP_R01                      PHY_TIMER_COMP_CH2_RX_ALLOWED

#define PHY_TIMER_CLOCK                         (PLAT_FREQ / 8) /* 61.44 MHz */
#define MSECONDS_TO_PHY_TIMER_COUNT( msec )   ( ( PHY_TIMER_CLOCK / 1000 ) * msec )

#define PHY_TIMER_PPS_OUT_PULSE_DELAY           500 /* ms */
#define PHY_TIMER_PPS_OUT_GPS_INTERVAL         1000 /* ms */
#define PHY_TIMER_PPS_OUT_GPS_HIGH              100 /* ms */
#define PHY_TIMER_PPS_OUT_GPS_LOW   (PHY_TIMER_PPS_OUT_GPS_INTERVAL - PHY_TIMER_PPS_OUT_GPS_HIGH)

struct xPhyTimerRegs
{
    uint32_t ulTmPhyTmrCtrl; /* PHY Timer Control Register */
    struct xTmPhyTmrN
    {
        uint32_t ulTmPhyTmrCncrs; /* PHY Timer Comparator n Control and Status Register */
        uint32_t ulTmPhyTmrCnv;   /* PHY Timer Comparator 0 Value Register */
    }
    xTmPhyTmrN[ PHY_TIMER_COMPARATOR_COUNT ];
};

/* PHY Timer Functions */

/**
 *  @Function	    vPhyTimerReset
 *
 *  @Description	Reset PHY Timer. You can get the timer out of reset with \ref
 *                  vPhyTimerEnable
 *
 *  @Return	        None
 */
void vPhyTimerReset();

/**
 *  @Function	    vPhyTimerEnable
 *
 *  @Description	Enable PHY timer. This triggers the counter and configures the
 *                  clock divisor
 *
 *  @Param[in]	    ucClockDiv - Select PHY timer clock division. Acceptable values
 *                  are between 1 and 63, and represent a division of the input
 *                  clock with this actual value.
 *
 *  @Return	        None
 */
void vPhyTimerEnable( uint8_t ucClockDiv );

/**
 *  @Function	    vPhyTimerEnable
 *
 *  @Description	    Disable PHY timer. This call will also reset the clock divisor.
 *
 *  @Return	        None
 */
void vPhyTimerDisable();

/**
 *  @Function	    ulPhyTimerCapture
 *
 *  @Description    Retrieve phy timer current value. This is done by configuring a
 *                  comparator to capture the current value and reading it's value.
 *                  Note that this routine configures the selected comparator but
 *                  does not clean it up afterwards, this is the applications
 *                  responsibility in order to provide an as-accurate-as-possible
 *                  value.
 *
 *  @Param[in]	    ucComparator - Select the comparator used to capture timer value.
 *                  Values range from 0 to 22
 *
 *  @Return		    Current timer value
 *
 */
uint32_t ulPhyTimerCapture( uint8_t ucComparator );

/* routine for returning the current timer value */
#define uGetPhyTimerTimestamp() \
	ulPhyTimerCapture( PHY_TIMER_COMPARATOR_COUNT - 1 )

/**
 *  @Function	    vPhyTimerComparatorDisable
 *
 *  @Description	Disable a comparator
 *
 *  @Param[in]	    ucComparator - Select the comparator to configure. Values range
 *                  from 0 to 22
 *
 *  @Return		    None
 *
 */
void vPhyTimerComparatorDisable( uint8_t ucComparator );

/**
 *  @Function	    vPhyTimerComparatorForce
 *
 *  @Description	Force an output variable on a disabled comparator
 *
 *  @Param[in]	    ucComparator - Select the comparator to configure. Values range
 *                  from 0 to 22
 *  @Param[in]	    eValue - See \ref enum ePhyTimerComparatorTrigger. This
 *                  parameter allows software to force a trigger on output when
 *                  a value different than \ref ePhyTimerComparatorNoChange is
 *                  used. See Cautions regarding this parameter
 *
 *  @Return		    None
 *
 *  Caution:	    In order to generate an output signal using eValue the
 *                  comparator needs to be already enabled. Enabling a comparator
 *                  at the same time with setting eValue will not produce the
 *                  expected behavior
 *
 */
void vPhyTimerComparatorForce( uint8_t ucComparator,
                               enum ePhyTimerComparatorTrigger eValue );

/**
 *  @Function	    vPhyTimerComparatorConfig
 *
 *  @Description	Configure and enable a comparator
 *
 *  @Param[in]	    ucComparator - Select the comparator to configure. Values range
 *                  from 0 to 22
 *  @Param[in]	    ucFlags - See \ref 'PHY Timer Comparator flags' for possible values
 *  @Param[in]	    eCmpTrig - See \ref enum ePhyTimerComparatorTrigger. This
 *                  parameter allows software to define comparator signal on
 *                  compare equal.
 *  @Param[in]	    ulTriggerValue - Comparator value for compare equal event
 *
 *  @Return		    None
 *
 */
void vPhyTimerComparatorConfig( uint8_t ucComparator,
                                uint8_t ucFlags,
                                enum ePhyTimerComparatorTrigger eCmpTrig,
                                uint32_t ulTriggerValue );

void vPhyTimerComparatorUpdate( uint8_t ucComparator,
                                uint8_t ucFlags,
                                enum ePhyTimerComparatorTrigger eCmpTrig,
                                uint32_t ulTriggerValue );

/**
 *  @Function	    ulPhyTimerComparatorRead
 *
 *  @Description	Returns value of comparator
 *
 *  @Param[in]	    ucComparator - Select the comparator to read. Values range
 *                  from 0 to 22
 *
 *  @Return		    32 bits value of comparator
 *
 */
uint32_t ulPhyTimerComparatorRead( uint8_t ucComparator );

/**
 *  @Function	    ulPhyTimerComparatorGetStatus
 *
 *  @Description	Returns status for a comparator.
 *
 *  @Param[in]	    ucComparator - Select the comparator to read. Values range
 *                  from 0 to 22
 *
 *  @Return		    Bitmask of \ref 'PHY Timer Status' describing current
 *                  status for comparator
 *
 */
uint32_t ulPhyTimerComparatorGetStatus( uint8_t ucComparator );

/**
 *  @Function	    vPhyTimerUpdateComparator
 *
 *  @Description	Updates the values of the comparator
 *
 *  @Param[in]	    ucComparator - comparator value between 0 to 22
 *
 *  @Param[in]	    ulTriggerValue - Comparator value for compare equal event
 *
 *  @Return		    None
 *
 */
void vPhyTimerUpdateComparator( uint8_t ucComparator, uint32_t ulTriggerValue );

/**
 *  @Function	    vPhyTimerPPSOUTConfig
 *
 *  @Description	Configure PPS OUT duty cycle and interrupt
 *
 *  @Return		    None
 *
 */
void vPhyTimerPPSOUTConfig( void );
void vPhyTimerPPSOUTConfigGPSlike( void );

/**
 *  @Function	    vPhyTimerPPSINEnable
 *
 *  @Description	Enable and Configure PPS IN interrupt
 *
 *  @Return		    None
 *
 */
void vPhyTimerPPSINEnable( void );

/**
 *  @Function	    vPhyTimerPPSINDisable
 *
 *  @Description    Disable PPS IN interrupt
 *
 *  @Return		    None
 *
 */
void vPhyTimerPPSINDisable( void );

/**
 * PPS OUT interrupt handler
 *
 */
void vPhyTimerPPSOUTHandler( void );
void vPhyTimerPPSOUTHandlerGPSlike( void );

/**
 * PPS OUT adjsutment for NLM handler, minor corrections
 *
 */
void vPhyTimerPPSOUTAdjustMinor(int32_t offset);

/**
 * PPS OUT adjsutment for NLM handler, major reset
 *
 */
void vPhyTimerPPSOUTAdjustMajor(uint32_t timestamp, uint32_t offset);

/**
 * PPS IN interrupt handler
 *
 */
void vPhyTimerPPSINHandler( void );

/**
 *  @Function       ulPhyTimerDiffToUS
 *
 *  @Description	phy timer counter diff to us
 *
 *  @Return         phy timer counter diff in us
 *
 */
uint32_t ulPhyTimerDiffToUS( uint32_t ulOlderTimestamp, uint32_t ulNewerTimestamp );

/**
 *  @Function	    vPhyTimerRx1Config
 *
 *  @Description	Configure Rx1 channel with persistent Rx_Allowed
 *
 *  @Return		    None
 *
 */
void vPhyTimerRx1Config( void );

typedef void (*PhyTimerPPSOUTCallback_t)( void *, long unsigned int );
/**
 *  @Function       vPhyTimerPPSOUTRegisterCallback
 *
 *  @Description    Configure user function to be called on PPS OUT
 *                  interrupt handler
 *
 *  @Return         None
 *
 */
void vPhyTimerPPSOUTRegisterCallback(PhyTimerPPSOUTCallback_t cb);

#endif /* __LA9310_PHY_TIMER_H */
