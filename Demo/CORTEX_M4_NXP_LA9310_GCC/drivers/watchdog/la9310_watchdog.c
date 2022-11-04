/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2017, 2021-2022 NXP
 */

#include "la9310_watchdog.h"
#include "la9310_edmaAPI.h"
#include "la9310_irq.h"
#include "la9310_gpio.h"

int iWdogEnable( uint32_t wdog_load_val,
                 struct la9310_info * pLa9310Info )
{
	#ifdef TURN_ON_HOST_MODE
	struct la9310_msi_info * pMsiInfo;
	#else
	uint8_t pin;
	#endif
    int ret = SUCCESS;

    edma_channel_info_t * pEdmaCh = pvPortMalloc( sizeof
                                                  ( struct edma_channel_info ) );
    NVIC_EnableIRQ( IRQ_WDOG );
    load_value = wdog_load_val;
    OUT_32( PCTBEN_BASE_ADDR, 0x1 |
            IN_32( ( uint32_t * ) PCTBEN_BASE_ADDR ) );

    /* eDMA channel 15 configuration */
    pEdmaCh->channel_id = WDOG_eDMA_CHANNEL;
    pEdmaCh->tcd_addr = ( EDMA_TCD0 +
                          ( WDOG_eDMA_CHANNEL * EDMA_TCD_SIZE ) );
    OUT_32( EDMA_ERQ_REG, ( 1 << WDOG_eDMA_CHANNEL ) |
            IN_32( ( uint32_t * ) EDMA_ERQ_REG ) );

    #ifdef TURN_ON_HOST_MODE
        pMsiInfo = &pLa9310Info->msi_info[ MSI_IRQ_WDOG ];
        ret = iEdmaXferReq( ( uint32_t ) &pMsiInfo->data,
                            pMsiInfo->addr, ( uint32_t ) 4, pEdmaCh );
	#else //TURN_ON_STANDALONE_MODE
        iGpioInit( GPIO_PIN_I2C_BOOT_RESET, output, false );
        gpio_port_t * gpioport = ( struct gpio_port * ) GPIO_BASE_ADDR;
        uint32_t * p_reg = ( uint32_t * ) gpioport;
        pin = 31 - GPIO_PIN_I2C_BOOT_RESET;
        uint32_t sa[ 1 ] = { 0 };
        sa[ 0 ] = (uint32_t) BITS( pin );
        ret = iEdmaXferReq( ( uint32_t ) sa,( uint32_t ) ( &p_reg[ GPIO_DAT ] ),
                     ( uint32_t ) 4, pEdmaCh );
	#endif
    /* WDOG Enable */
    OUT_32( WDOG_LOCK_REG, WDOG_UNLOCK );
    OUT_32( WDOG_LOAD_REG, load_value );
    OUT_32( WDOG_CONTROL_REG, WDOG_CONTROL_RESEN |
            WDOG_CONTROL_INTEN |
            IN_32( ( uint32_t * ) WDOG_CONTROL_REG ) );
    OUT_32( WDOG_LOCK_REG, WDOG_LOCK );
    return ret;
}

void vWdogDisable( void )
{
    log_info( "%s: WDOG DISABLE\n\r", __func__ );
    OUT_32( WDOG_LOCK_REG, WDOG_UNLOCK );
    OUT_32( WDOG_CONTROL_REG, 0xfffffffc &
            IN_32( ( uint32_t * ) WDOG_CONTROL_REG ) );
    OUT_32( WDOG_LOCK_REG, WDOG_LOCK );
}

void vWdogReload( void )
{
    log_info( "%s: WDOG Reload\n\r", __func__ );
    OUT_32( WDOG_LOCK_REG, WDOG_UNLOCK );
    OUT_32( WDOG_INTCLR_REG, 0x1 |
            IN_32( ( uint32_t * ) WDOG_INTCLR_REG ) );
    OUT_32( WDOG_LOAD_REG, load_value );
    OUT_32( WDOG_CONTROL_REG, WDOG_CONTROL_RESEN |
            WDOG_CONTROL_INTEN |
            IN_32( ( uint32_t * ) WDOG_CONTROL_REG ) );
    OUT_32( WDOG_LOCK_REG, WDOG_LOCK );
}

void La9310WDOG_IRQHandler( void )
{
    static unsigned int count;
    struct la9310_stats * pStats;

    pStats = pLa9310Info->stats;

    if( count == 0 )
    {
        log_isr( "%s: WDOG IRQ Handler\n\r", __func__ );
        count++;
        pStats->WDOG_interrupt++;
    }

    NVIC_ClearPendingIRQ( IRQ_WDOG );
    #if ARM_ERRATUM_838869
        dsb();
    #endif
}
