/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2017-2018, 2021-2022 NXP
 */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "common.h"
#include "immap.h"
#include "la9310_main.h"
#include "la9310_irq.h"
#include "core_cm4.h"
#include "la9310_demo.h"
#include "la9310_edmaAPI.h"
#include "debug_console.h"
#include "bbdev_ipc.h"
#include "la9310_avi.h"
#include "la9310_gpio.h"
#include "drivers/avi/la9310_avi_ds.h"
#include "la9310.h"
#include "exceptions.h"
#include "la9310_wdogAPI.h"
#include "la9310_i2cAPI.h"
#include "la9310_pinmux.h"
#include "drivers/avi/la9310_vspa_dma.h"
#include "la9310_i2cAPI.h"
#include "common.h"
#include <config.h>

#define N_BYTES                   16
#define N_INT                     ( N_BYTES / 4 )

#define MAX_IPC_TRANS             32
#define MAX_PACKET_PER_CHANNEL    4
#define ITER_PER_CHANNEL          8
#define MAX_IPC_MSG_BUFF          IPC_MSG_SIZE
#define OVERLAY_SECTION_OFFSET    0x00  /*TBD*/
#define DEBUG_HEX_DUMP

#ifdef TURN_ON_STANDALONE_MODE
#define EXT_HEADER_OFFSET         0x20
#define CRC_HEADER_OFFSET         0x40
#define IS_VALID_MAGIC_NUM        0x796573

enum crc_images {
	FREERTOS_IMG	    = 0,
	VSPA_IMG    	    = 1,
	VSPA_TBL_IMG	    = 2,
	BOOTSTRAPPER_IMG    = 3,
	MAX_IMAGES          = 4
};

static const char * const crc_img_names[] = {
	[FREERTOS_IMG]	    = "FreeRTOS img",
	[VSPA_IMG]	    = "VSPA img",
	[VSPA_TBL_IMG]	    = "VSPA table img",
	[BOOTSTRAPPER_IMG]  = "BootStrapper img"
};
#endif //TURN_ON_STANDALONE_MODE
/*
 * Uncomment either of following two option to run the IPC
 * test cases either in blocking mode or non-blocking mode.
 */
#define TEST_BLOCK_OR_NONBLOCK    LA9310_IPC_BLOCK
/*#define TEST_BLOCK_OR_NONBLOCK	LA9310_IPC_NONBLOCK */

static QueueHandle_t xQueue;
static QueueSetHandle_t xQueueSet;
int status;

void print_data( uint32_t addr,
                 uint32_t size )
{
    uint32_t * ptr = ( uint32_t * ) addr, f;

    for( f = 0; f < ( size / ( 4 * 4 ) ); f++ )
    {
        log_info( "%x : %x  %x  %x  %x\n\r", ptr,
                  *ptr, *( ptr + 1 ), *( ptr + 2 ), *( ptr + 3 ) );
        ptr = ptr + 4;
    }
}

void vMyCallback( void * info,
                  edma_status st )
{
    log_info( "%s: EDMA CallBAck Function\n\r", __func__ );
    log_info( "%s: info: %x\n\r", __func__, ( uint32_t ) info );
    log_info( "%s: status: %x\n\r", __func__, st );
    status = st;
    xQueueSendToBackFromISR( xQueue, info, 0 );
}

static void prvDemoEvtMask( struct la9310_info * pLa9310Info,
                            la9310_irq_evt_bits_t evt_bit,
                            void * cookie )
{
    /*ADD code to mask your IRQ here. For NVIC interrupts use
     * NVIC_DisableIRQ. For Soft sources like IPC it is upto the source to
     * decide whether any masking operation required or not
     */

    /*For Demo Mask function nothing to be done*/
    log_dbg( "%s: mask, nothing to be done\n\r", __func__ );
}

static void prvDemoEvtUnmask( struct la9310_info * pLa9310Info,
                              la9310_irq_evt_bits_t evt_bit,
                              void * cookie )
{
    /* ADD code to mask your IRQ here. For NVIC interrupts use
     * NVIC_EnableIRQ. For Soft sources like IPC it is upto the source to
     * decide whether any masking operation required or not
     */

    /*For Demo unMask function nothing to be done*/

    log_dbg( "%s: unmask, nothing to be done\n\r", __func__ );
}
#ifdef TURN_ON_STANDALONE_MODE
unsigned int calculate_crc32(uint32_t start_addr, uint32_t size)
{
        char byte = 0;
        unsigned int i = 0, j = 0, k = 0, bit = 0;
        unsigned int crc = 0xFFFFFFFF, left_over = 0, nblocks = 0;
        uint8_t ucval[MAX_EEPROM_READ_SIZE] = {0};

        left_over = size % MAX_EEPROM_READ_SIZE;
        nblocks = (size - left_over) / MAX_EEPROM_READ_SIZE;
        for (k=0; k < nblocks; k++)
        {
                iLa9310_I2C_Read(LA9310_FSL_I2C1,IC2_EEPROM_DEV_ADDR,
                                start_addr + (k * MAX_EEPROM_READ_SIZE),
                                LA9310_I2C_DEV_OFFSET_LEN_2_BYTE, ucval, MAX_EEPROM_READ_SIZE );
                for(i=0; i<4; i++)
                {
                        byte = ucval[i];
                        for(j=0; j<8; j++)
                        {
                                bit = (byte ^ crc)&1;
                                crc >>= 1;
                                if (bit)
                                        crc = crc^0xEDB88320;
                                byte >>= 1;
                        }
                }
        }
        if (left_over) {
                iLa9310_I2C_Read(LA9310_FSL_I2C1,IC2_EEPROM_DEV_ADDR,
                                start_addr + (k * MAX_EEPROM_READ_SIZE),
                                LA9310_I2C_DEV_OFFSET_LEN_2_BYTE, ucval, left_over);

                for(i=0; i<left_over; i++)
                {
                        byte = ucval[i];
                        for(j=0; j<8; j++)
                        {
                                bit = (byte ^ crc)&1;
                                crc >>= 1;
                                if (bit)
                                        crc = crc^0xEDB88320;
                                byte >>= 1;
                        }
                }
        }
        return ~crc;
}

void get_header_info(struct ext_header *pExtBootHeader, struct crc_header *pCrcHeader)
{
        uint8_t i = 0, numVar = 0;
        uint32_t *dptr = NULL;
        uint8_t data[MAX_EEPROM_READ_SIZE] = {0};
        numVar = sizeof(struct ext_header) / sizeof(uint32_t);

        /* Fill Extended Header */
        dptr = (uint32_t *)pExtBootHeader;

        for(i=0; i<numVar; i++)
        {
                iLa9310_I2C_Read(LA9310_FSL_I2C1,IC2_EEPROM_DEV_ADDR,
                                EXT_HEADER_OFFSET + (i * MAX_EEPROM_READ_SIZE),
                                LA9310_I2C_DEV_OFFSET_LEN_2_BYTE, data, MAX_EEPROM_READ_SIZE);
                *dptr++ = (data[3]<<24)|(data[2]<<16)|(data[1]<<8)|(data[0]);
        }
        /* Fill CRC Header */
        numVar = sizeof(struct crc_header)/sizeof(uint32_t);
        dptr = (uint32_t *)pCrcHeader;

        for(i=0; i<numVar; i++)
        {
                iLa9310_I2C_Read(LA9310_FSL_I2C1,IC2_EEPROM_DEV_ADDR,
                                CRC_HEADER_OFFSET + (i * MAX_EEPROM_READ_SIZE),
                                LA9310_I2C_DEV_OFFSET_LEN_2_BYTE, data, MAX_EEPROM_READ_SIZE);
                *dptr++ = (data[3]<<24)|(data[2]<<16)|(data[1]<<8)|(data[0]);
        }
}

void vLa9310VerifyCRC()
{
    uint8_t img_index = 0;
    uint32_t calculated_crc = 0;
    struct ext_header extBootHeader;
    struct crc_header crcHeader;

    get_header_info(&extBootHeader, &crcHeader);

    for(img_index=0; img_index<MAX_IMAGES; img_index++)
    {
        log_info("Calculating CRC for %s ...\n\r",crc_img_names[img_index]);
        switch (img_index) {
            case FREERTOS_IMG :
                if (crcHeader.is_valid_freertos == IS_VALID_MAGIC_NUM) {
                    calculated_crc = calculate_crc32(
                            (extBootHeader.freertos_image_offset + 
                             extBootHeader.page_write_size),
                            crcHeader.size_freertos);
                    if (calculated_crc == crcHeader.crc_freertos_img ) {
                        log_info("Header CRC [ 0x%x ] = Calculated CRC [ 0x%x]\n\r",
                                crcHeader.crc_freertos_img,
                                calculated_crc);
                    } else {
                        log_info("Header CRC [ 0x%x ] = Calculated CRC [ 0x%x]\n\r",
                                crcHeader.crc_freertos_img,
                                calculated_crc);
                    }
                } else {
                    log_info("Pre-calculated CRC is not available in header.\n\r");
                }
                break;
            case VSPA_IMG :
                if (crcHeader.is_valid_vspa_bin == IS_VALID_MAGIC_NUM) {
                    calculated_crc = calculate_crc32(
                            (extBootHeader.vspa_bin_location),
                            crcHeader.size_vspa_bin );
                    if (calculated_crc == crcHeader.crc_vspa_bin ) {
                        log_info("Header CRC [ 0x%x ] = Calculated CRC [ 0x%x]\n\r",
                                crcHeader.crc_vspa_bin,
                                calculated_crc);
                    } else {
                        log_info("Header CRC [ 0x%x ] = Calculated CRC [ 0x%x]\n\r",
                                crcHeader.crc_vspa_bin,
                                calculated_crc);
                    }
                } else {
                    log_info("Pre-calculated CRC is not available in header.\n\r");
                }
                break;
            case VSPA_TBL_IMG:
                if (crcHeader.is_valid_vspa_table == IS_VALID_MAGIC_NUM) {
                    calculated_crc = calculate_crc32(
                            (extBootHeader.vspa_table_location),
                            crcHeader.size_vspa_table );
                    if (calculated_crc == crcHeader.crc_vspa_table ) {
                        log_info("Header CRC [ 0x%x ] = Calculated CRC [ 0x%x]\n\r",
                                crcHeader.crc_vspa_table,
                                calculated_crc);
                    } else {
                        log_info("Header CRC [ 0x%x ] = Calculated CRC [ 0x%x]\n\r",
                                crcHeader.crc_vspa_table,
                                calculated_crc);
                    }
                } else {
                    log_info("Pre-calculated CRC is not available in header.\n\r");
                }
                break;
            case BOOTSTRAPPER_IMG:
                if (crcHeader.is_valid_bootstrapper == IS_VALID_MAGIC_NUM) {
                    calculated_crc = calculate_crc32(
                            extBootHeader.bootstrapper_image_offset ,
                            crcHeader.size_bootstrapper);
                    if (calculated_crc == crcHeader.crc_bootstrapper ) {
                        log_info("Header CRC [ 0x%x ] = Calculated CRC [ 0x%x]\n\r",
                                crcHeader.crc_bootstrapper,
                                calculated_crc);
                    } else {
                        log_info("Header CRC [ 0x%x ] = Calculated CRC [ 0x%x]\n\r",
                                crcHeader.crc_bootstrapper,
                                calculated_crc);
                    }
                } else {
                    log_info("Pre-calculated CRC is not available in header.\n\r");
                }
                break;
            default:
                log_err("Invalid image index !\n\r");

        };

    }
}
#endif //TURN_ON_STANDALONE_MODE

void vLa9310EdmaDemo( uint32_t * pulEDMAInfo )
{
    int sa[ N_INT ] = { 0 };
    int da[ N_INT ] = { 0 };
    int i = 0, ret;
    Callback pMyCallbackFn = vMyCallback;
    edma_channel_info_t * handle = NULL;
    uint32_t ulNotificaion;
    QueueHandle_t xQueueThatContainsData;

    xQueue = xQueueCreate( 1, sizeof( edma_channel_info_t * ) );
    xQueueSet = xQueueCreateSet( 1 );
    xQueueAddToSet( xQueue, xQueueSet );

    for( i = 0; i < ( N_BYTES / 4 ); i++ )
    {
        sa[ i ] = ( int ) i + 1 + 0x0aaaa000;
    }

    log_info( "data at source address\n\r" );
    print_data( ( int ) sa, N_BYTES );
    log_info( "data at destination address\n\r" );
    print_data( ( int ) da, N_BYTES );

    handle = pEdmaAllocChannel( pulEDMAInfo, pMyCallbackFn );

    if( handle != NULL )
    {
        log_info( "Got an edma channel\n\r" );
        log_info( "channel_id:%d TCD_addr:%x info:%x pCallbackFn:%x\n\r",
                  handle->channel_id, handle->tcd_addr, handle->info,
                  handle->pCallbackFn );

        ret = iEdmaXferReq( ( int ) sa, ( int ) da, N_BYTES, handle );

        if( !ret )
        {
            log_info( "%s:eDMA Xfer requested\n\r", __func__ );
        }

        xQueueThatContainsData = ( QueueHandle_t ) xQueueSelectFromSet
                                     ( xQueueSet, portMAX_DELAY );
        xQueueReceive( xQueueThatContainsData,
                       &ulNotificaion, 0 );

        if( status )
        {
            ret = memcmp( sa, da, N_BYTES );

            if( ret == 0 )
            {
                log_info( "eDMA Test case PASS\n\r" );
            }
            else
            {
                log_info( "eDMA Test case FAIL\n\r" );
            }

            log_info( "data at destination address after EDMA xfer\n\r" );
            print_data( ( int ) da, N_BYTES );
        }
        else
        {
            log_info( "eDMA timeout\n\r" );
        }

        vEdmaFreeChannel( handle );
    }
    else
    {
        log_info( "All edma channels are busy\n\r" );
    }
}

void vLa9310I2CTest( uint8_t uI2C_Controller,
                     uint8_t rw,
                     uint8_t ucDev_Addr,
                     uint32_t ulDev_Offset,
                     uint8_t ulDev_Offset_len,
                     uint8_t uNum_Byte,
                     uint32_t uByteToWrite )
{
    int ret, ctr;
    uint8_t ucdata[ 5 ];
    uint8_t ucval[ 5 ];
    uint32_t ulI2C_Regs_P;
    uint8_t uValidFlag = 0;

    log_info( "vLa9310I2CTest App\n\r" );
    log_info( "i2cParameters Ctrl %d rw %d Addr(Dec %d Hex %x) offset (Dec %d Hex %x) ulDev_Offset_len %d Num_Byte %d \n\r",
              uI2C_Controller, rw, ucDev_Addr, ucDev_Addr, ulDev_Offset, ulDev_Offset, ulDev_Offset_len, uNum_Byte );

    if( uNum_Byte > 4 )
    {
        log_info( "i2c test length error %d\n\r", uNum_Byte );
        return;
    }

    if( uI2C_Controller == 1 )
    {
        ulI2C_Regs_P = LA9310_FSL_I2C1;
    }
    else if( uI2C_Controller == 2 )
    {
        ulI2C_Regs_P = LA9310_FSL_I2C2;
    }
    else
    {
        uValidFlag = 1;
        log_info( "Invalid Controoler ID, Valid Values are ( 1 and 2)  %d\n\r", uI2C_Controller );
    }

    if( !uValidFlag )
    {
        /* LA9310_FSL_I2C1--I2C controller address*/
        /* Example 0x50--Slave address(EEPROM connected at this addr)*/
        /* Example 0x22--slave offset*/
        /* LA9310_I2C_DEV_OFFSET_LEN_2_BYTE--address byte*/
        /* ucdata--pointer to buffer*/
        /* Example 5--no of  bytes to write*/

        if( rw == 1 )
        {
            for( ctr = 0; ctr < uNum_Byte; ctr++ )
            {
                ucdata[ ctr ] = ( uByteToWrite >> ( ctr * 8 ) & 0xff );
                log_info( "Write Byte Num %d val 0x%x \r\n", ctr, ucdata[ ctr ] );
            }

            /*write 5 bytes at slave device(0x50) at 0x22 offset*/
            ret = iLa9310_I2C_Write( ulI2C_Regs_P, ucDev_Addr, ulDev_Offset,
                                     ulDev_Offset_len, ucdata, uNum_Byte );
            log_info( "i2c_write:::: %d\n\r", ret );
        }
        else
        {
            /*read uNum_Byte bytes from slave device(example 0x50) from (example 0x22) offset*/
            ret = iLa9310_I2C_Read( ulI2C_Regs_P, ucDev_Addr, ulDev_Offset,
                                    ulDev_Offset_len, ucval, uNum_Byte );

            log_info( "i2c_read:::: %d\n\r", ret );
            log_info( "--------------------\n\r" );

            for( ctr = 0; ctr < uNum_Byte; ctr++ )
            {
                log_info( "ReadByteNum %d - read %x\n\r", ctr, ucval[ ctr ] );
            }
        }
    }
}


void vLa9310DemoIrqEvtRegister( struct la9310_info * pLa9310Info )
{
    struct la9310_evt_hdlr La9310EvtHdlr;

    /*EVT IRQ API test*/
    /*IRQ will be raised in demo mode controlled by MSG UNIT2 IRQ*/
    La9310EvtHdlr.type = LA9310_EVT_TYPE_TEST;
    La9310EvtHdlr.mask = prvDemoEvtMask;
    La9310EvtHdlr.unmask = prvDemoEvtUnmask;
    La9310EvtHdlr.cookie = pLa9310Info;
    iLa9310RegisterEvt( pLa9310Info, IRQ_EVT_TEST_BIT, &La9310EvtHdlr );
}

void hexdump( const unsigned char * src,
					 uint32_t ulCount )
{
	#ifdef DEBUG_HEX_DUMP
	uint32_t ulIndex;
	if( 0 == ulCount )
	{
		return;
	}

	for( ulIndex = 0; ulIndex < ulCount; ++ulIndex )
	{
		PRINTF( "%02x ", src[ ulIndex ] );

		if( ( ulIndex + 1 ) % 16 == 0 )
		{
			PRINTF( "\r\n" );
		}
	}

	PRINTF( "\r\n" );
	#endif /* if DEBUG */
}

void vGenerateExceptions( uint32_t ulID )
{
    switch( ulID )
    {
        case DIVIDE_BY_ZERO:
            PRINTF( "This is devide by zero exception\r\n" );
            /*fix-me compilation error PRINTF( "%x\r\n", 1 / 0 ); */
            break;

        case UNALIGNED_ACCESS:
            PRINTF( "This is unaligned memory access exception\r\n" );
            uint32_t * pulUA = ( uint32_t * ) -1;
            PRINTF( "Data : %x\r\n", *pulUA );
            break;

        case NULL_ACCESS:
            PRINTF( "This is NULL pointer access exception\r\n" );
            char * pn = NULL;
            PRINTF( "%c\r\n", *pn );
            break;

        case ILLEGAL_ADDR:
            PRINTF( "This is illegal address access exception\r\n" );
            uint32_t * pulIA = ( uint32_t * ) ILLEGAL_ADDR_LOCATION;
            *pulIA = DUMMY_TEST_VAL;
            PRINTF( "DUMMY TEST VAL : %x\r\n", *pulIA );
            break;

        default:
            break;
    }
}

void vAVIDemo( uint32_t ulNumIteration )
{
        struct avi_mbox host_mbox;
        struct avi_mbox test_mbox = { 0 };
        int mbox_index = 0;
        void * avihndl = NULL;
        uint32_t ulCnt;
        log_info( "%s: AVI Test Num Iteration %d..\n\r", __func__, ulNumIteration );

        avihndl = iLa9310AviInit();

        for( ulCnt = 0; ulCnt < ulNumIteration; ulCnt++ )
        {
            for( mbox_index = 0; mbox_index < 2; mbox_index++ )
            {
                if( NULL == avihndl )
                {
                    log_err( "ERR: %s: AVI Initialization Failed\n\r", __func__ );
                    goto avi_out;
                }

                host_mbox.msb = 0x0A << 24;           /*Opcode */
                host_mbox.msb = host_mbox.msb | 0x01; /*Offset */

                /*Set the lsb value */
                host_mbox.lsb = OVERLAY_SECTION_OFFSET; /* For offset value */

                log_info( "MBOX[%d].msb 0x%x MBOX[%d].lsb 0x%x\n\r",
                          mbox_index, host_mbox.msb,
                          mbox_index, host_mbox.lsb );

                /*Sending AVI API (CM4-->VSPA )*/
                if( 0 != iLa9310AviHostSendMboxToVspa( avihndl, host_mbox.msb,
                                                       host_mbox.lsb, mbox_index ) )
                {
                    log_err( "ERR: Overlay Section offset on MBOX%d Failed\n\r", mbox_index );
                }

                /* Reading ACK message from VSPA*/
                if( 0 != iLa9310AviHostRecvMboxFromVspa( avihndl, &test_mbox, mbox_index ) )
                {
                    log_err( "ERR: Overlay Read VSPA Mailbox%d  Fail\n\r", mbox_index );
                }

                /*Checking the ACK bit from the mbox received from VSPA */
                if( ( test_mbox.lsb & 0x01 ) )
                {
                    log_info( "ACK Rcvd successfully\n\r" );
                }
                else
                {
                    log_err( "ACK Rcd Failed..\n\r" );
                }
            }
        }

avi_out:
        log_info( "Exit %s\n\r", __func__ );
        return;
}

void vWdogDemo( struct la9310_info * pLa9310Info )
{
    uint32_t wdog_load_val;
    int ret, Num = 4;

    log_info( "%s:WDOG Demo\n\r", __func__ );
    wdog_load_val = Num * ( ( configCPU_CLOCK_HZ / 3000 ) /
                            configTICK_RATE_HZ );
    log_info( "%s: load_value: %x\n\r", __func__, wdog_load_val );
    ret = iWdogEnable( wdog_load_val, pLa9310Info );

    if( !ret )
    {
        log_info( "%s: WDOG Enabled\n\r", __func__ );
    }

    /* Call this API regularly before WDOG expire*/
    vWdogReload();
}

void delay1()
{
    int delay_cnt = 100000;

    while( delay_cnt-- )
    {
    }

    log_info( "delay 1 count pmux %d\n\r", delay_cnt );
}

void vLa9310GpioTest( uint8_t iGpioNum,
                      uint8_t iDir,
                      uint8_t iHighLow )
{
    log_info( "vLa9310GpioTest GPIO_Num %d Dir %d HighLow %d\n\n\r", iGpioNum, iDir, iHighLow );
    vGpioSetPinMuxSingle( iGpioNum, SET_MUX_GPIO_MODE );

    if( iDir )
    {
        iGpioInit( iGpioNum, output, false );
        iGpioSetData( iGpioNum, iHighLow );
    }
    else
    {
        iGpioInit( iGpioNum, input, false );
        log_info( "GPIO value is: %d\r\n", iGpioGetData( iGpioNum ) );
    }
}
#ifdef TURN_ON_STANDALONE_MODE
int iVerifyVSPATable()
{
    void * dtcm_addr;
    uint32_t num_vspa_tbls,ctr;
    struct vspa_table_hdr *vspa_hdr;
    dtcm_addr = pLa9310Info->dtcm_addr;

    vspa_hdr = (struct vspa_table_hdr * ) ((( uint32_t ) dtcm_addr + VSPA_HDR_TABLE_TCM_OFFSET));
    num_vspa_tbls = vspa_hdr->num_vspa_tbls;
    log_info("num_vspa_tbls %d \r\n",num_vspa_tbls);
    for(ctr = 0; ctr < num_vspa_tbls; ctr++)
    {
        log_info("tbl_name %s tbl_size 0x%x table_tcm 0x%08x \r\n",
                vspa_hdr->vspa_tbl_info[ctr].tbl_name, vspa_hdr->vspa_tbl_info[ctr].tbl_size,
				vspa_hdr->vspa_tbl_info[ctr].tbl_tcm_loc);
		hexdump((uint8_t * )vspa_hdr->vspa_tbl_info[ctr].tbl_tcm_loc,vspa_hdr->vspa_tbl_info[ctr].tbl_size);

    }

	return 0;
}
#endif //TURN_ON_STANDALONE_MODE
