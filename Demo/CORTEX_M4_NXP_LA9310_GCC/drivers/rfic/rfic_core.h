/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright 2021-2022, 2024 NXP
 */
#ifndef __RFIC_CORE_H
#define __RFIC_CORE_H
#include <string.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"
#include "rfic_hif.h"
#include "la9310_gpio.h"

/* Forward declaration */
extern struct la9310_info * pLa9310Info;

/* Macro definition */
#define RF_ENABLE_STATS_COLLECTION	( 1 )

#define RF_CORE_TASK_PRIORITY           ( configMAX_PRIORITIES - 1 )
#define RF_CORE_TASK_STACK_SIZE         ( configMINIMAL_STACK_SIZE * 2 )

#define RF_LOCAL_RESP_QUEUE_RECV_TIMEOUT    ( 1000 )
#define RF_LOCAL_QUEUE_SEND_TIMEOUT         ( 0 )

#define RF_LOCAL_CMD_EVENT		( 1 << 0 )
#define RF_REMOTE_CMD_EVENT		( 1 << 1 )
#define RF_CORE_TASK_EVENT_MASK		( RF_LOCAL_CMD_EVENT | \
	                                  RF_REMOTE_CMD_EVENT )

/* Max speed that can be supported by NLM HW */
#define RF_DSPI_MAX_CLK			7000000	//6.14MHz

#if RF_ENABLE_STATS_COLLECTION
    #define RF_STATS_ADD( var )               ( var += 1 )
    #define RF_STATS_SET( var )               ( var = 1 )
    #define RF_STATS_SET_VALUE( var, val )    ( var = val )
#else
    #define RF_STATS_ADD( var )
    #define RF_STATS_SET( var )
    #define RF_STATS_SET_VALUE( var, val )
#endif

/* Enum Definition */
typedef enum RficGpioFunc
{
    RF_GPIO_DEMOD_PATH_SEL,
    RF_GPIO_RFA_PATH_SEL,
    RF_GPIO_RFB_PATH_SEL,
    RF_GPIO_BAND_B3_LNA_EN,
    RF_GPIO_BAND_B13_LNA_EN,
    RF_GPIO_BAND_N77_LNA_EN,
    RF_GPIO_SPI_CS0_SEL,
    RF_GPIO_MAX
} RficGpioFunc_t;

typedef enum RficDspiSlave
{
    RF_DSPI_SLV_SYNTH,
    RF_DSPI_SLV_DEMOD,
    RF_DSPI_SLV_VGA
} RficDspiSlv_t;

/* Structure Definition */
/* GPIO details */
typedef struct RficGpio
{
    uint32_t        ulPin;
    gpio_type_t     etype;
    bool            ucDefaultVal;
} RFICGpio_t;

/* RFIC device structure */
typedef struct RficDevice
{
    /* Parent Device Handler */
    struct la9310_info *pLa9310Info;
    /* RFIC Core Task */
    TaskHandle_t xCoreTask;
    /* Queues - Remote/Local */
    QueueHandle_t xLocalQueue;
    QueueHandle_t xLocalRspQueue;
    QueueHandle_t xRemoteQueue;
    EventGroupHandle_t xSwCmdEvent;
    /* DSPI Handler */
    struct LA931xDspiInstance *pDspiHandle;
    /* GPIO for FEM ctrl */
    RFICGpio_t eGpio[RF_GPIO_MAX];
    /* RF HIF */
    rf_host_if_t *pRfHif;
    uint32_t iq_phys_addr;
    uint32_t iq_mem_addr;
    uint32_t iq_mem_size;
    uint32_t xVgaRegVal;
    uint32_t xLoopback;
}RficDevice_t;

/* Function prototype */
BaseType_t iRficInit( struct la9310_info *pLa9310Info );
void vRficCoreTask( void * pvParameters );
void vRficSwCmdIrq( RficDevice_t *pRficDev );
BaseType_t xRficPostLocalSwCmd( RficDevice_t *pRficDev,
				rf_sw_cmd_desc_t *pSwCmdDesc );
int32_t iRficSelectDspiSlave( RficDevice_t *pRficDev , RficDspiSlv_t eSlaveDev );
BaseType_t xHandleSwCmd( RficDevice_t *pRficDev, rf_sw_cmd_desc_t *pSwCmdDesc );
#endif  //__RFIC_CORE_H
