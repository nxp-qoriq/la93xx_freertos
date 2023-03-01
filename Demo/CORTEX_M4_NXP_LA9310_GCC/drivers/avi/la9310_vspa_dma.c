/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2022-2023 NXP
 */

#include "la9310_avi.h"
#include "la9310_avi_ds.h"
#include "la9310_error_codes.h"
#include "debug_console.h"
#include "la9310_irq.h"
#include "semphr.h"
#include <delay.h>
#include "la9310_i2cAPI.h"
#include "la9310_vspa_dma.h"
#include <common.h>

#define DEBUG_VSPA_BOOT

uint32_t uTcmLocationFilled;
uint32_t uGetDTCMLocationFilled()
{
	return uTcmLocationFilled;
}

int iLoadOverlay(char *ovl_sec_name, bool bVerifyLoad)
{
	void * dtcm_addr;
	struct vspa_image_hdr *vspa_hdr;
	int  ctr,num_sections;
	char ovl_name[MAX_SECTION_NAME];
	struct vspa_dma_req z_dma_req;
	int found = 0;

	dtcm_addr = pLa9310Info->dtcm_addr;
	vspa_hdr = (struct vspa_image_hdr * ) ((( uint32_t ) dtcm_addr + VSPA_HDR_TCM_OFFSET));
	num_sections = vspa_hdr->num_sections;

	for(ctr = 0; ctr < num_sections; ctr++)
	{
		if(vspa_hdr->dma_sec_info[ctr].is_overlay)
		{
			strncpy(ovl_name,vspa_hdr->dma_sec_info[ctr].section_name,MAX_SECTION_NAME-1);
			if(!strncmp(ovl_sec_name ,ovl_name,MAX_SECTION_NAME))
			{
				z_dma_req.axi_addr = vspa_hdr->dma_sec_info[ctr].axi_tcm_addr;
				z_dma_req.dmem_addr = vspa_hdr->dma_sec_info[ctr].dmem_addr;
				z_dma_req.byte_cnt = vspa_hdr->dma_sec_info[ctr].byte_cnt;
				z_dma_req.xfr_ctrl = vspa_hdr->dma_sec_info[ctr].xfr_ctrl;
				dma_raw_transmit(&z_dma_req);
				found = 1;
				break;
			}
		}
	}
	if(found == 0)
	{
		log_err("Invalid VSPA overlay %s section\r\n",ovl_name);
		return 1;
	}
	else {
		if(bVerifyLoad)
		{
			log_info("Overlay %s Found\r\n",ovl_sec_name);
			log_info("Overlay %s axi_tcm_addr 0x%x dmem_addr 0x%x byte_cnt 0x%x xfr_ctrl 0x%x \r\n",
				ovl_sec_name,vspa_hdr->dma_sec_info[ctr].axi_tcm_addr, vspa_hdr->dma_sec_info[ctr].dmem_addr,
					vspa_hdr->dma_sec_info[ctr].byte_cnt,vspa_hdr->dma_sec_info[ctr].xfr_ctrl);
			//hexdump((uint8_t * )z_dma_req.axi_addr,z_dma_req.byte_cnt);
		}
		return 0;
	}
}


int iLoadTableToTCM()
{
	void * dtcm_addr;
	struct ext_header *ext_hdr;
	dtcm_addr = pLa9310Info->dtcm_addr;

	//Copy VSPA TABLE HEADER INFO to TCMU  VSPA_HDR_TABLE_TCM_OFFSET offset
	CopyToTCM("LOAD EXTENDED TABLE HEADER",( uint32_t ) dtcm_addr + EXTENDED_HDR_TABLE_TCM_OFFSET, EXTENDED_HDR_EEPROM_ADDR, MAX_EXTENDED_TABLE_HDR_SIZE);
	ext_hdr = (struct ext_header * ) ((( uint32_t ) dtcm_addr + EXTENDED_HDR_TABLE_TCM_OFFSET));

	log_info("page_write_size 0x%x \r\n",ext_hdr->page_write_size);
	log_info("addr_shift 0x%x \r\n",ext_hdr->addr_shift);
	log_info("freertos_image_offset 0x%x \r\n",ext_hdr->freertos_image_offset);
	log_info("vspa_bin_location 0x%x \r\n",ext_hdr->vspa_bin_location);
	log_info("vspa_table_location 0x%x \r\n",ext_hdr->vspa_table_location);

	return 0;

}

int iGetExtTableInfo(ExtTableInfo_t eExtTableInfo,uint32_t *uPtrInfo)
{
	void * dtcm_addr;
	struct ext_header *ext_hdr;
	dtcm_addr = pLa9310Info->dtcm_addr;
	ext_hdr = (struct ext_header * ) ((( uint32_t ) dtcm_addr + EXTENDED_HDR_TABLE_TCM_OFFSET));
	switch(eExtTableInfo)
	{
		case EXT_EEPROM_PAGE_SIZE:
			log_info("page_write_size 0x%x \r\n",ext_hdr->page_write_size);
			*uPtrInfo = ext_hdr->page_write_size;
			break;
		case EXT_EEPROM_ADDR_SHIFT:
			log_info("addr_shift 0x%x \r\n",ext_hdr->addr_shift);
			*uPtrInfo = ext_hdr->addr_shift;
			break;
		case EXT_FREE_RTOS_IMAGE_EEPROM_ADDR:
			log_info("freertos_image_offset 0x%x \r\n",ext_hdr->freertos_image_offset);
			*uPtrInfo = ext_hdr->freertos_image_offset;
			break;
		case EXT_VSPA_BIN_EEPROM_ADDR:
			log_info("vspa_bin_location 0x%x \r\n",ext_hdr->vspa_bin_location);
			*uPtrInfo = ext_hdr->vspa_bin_location;
			break;
		case EXT_VSPA_TABLE_LOCATION_ADDR:
			log_info("vspa_table_location 0x%x \r\n",ext_hdr->vspa_table_location);
			*uPtrInfo =  ext_hdr->vspa_table_location;
			break;
		default:
			log_info("Unsupported EXT Table Info\r\n");
			return 1;
    }
    return 0;
}

int iLoadVSPATablesEEPROM_To_TCM()
{
	void * dtcm_addr;
	uint32_t num_vspa_tbls,ctr;
	struct vspa_table_hdr *vspa_hdr;
	uint32_t table_tcm_offset;
	uint32_t table_tcm_last_offset;
	char table_name[MAX_VSPA_BIN_FILE_NAME_LEN];
	uint32_t uVspaHdrTableEEPROMAddr;
	int ret;

	dtcm_addr = pLa9310Info->dtcm_addr;
	ret = iGetExtTableInfo(EXT_VSPA_TABLE_LOCATION_ADDR,&uVspaHdrTableEEPROMAddr);
	if(ret != 0) {
		log_info("%s Invalid uVspaHdrTableEEPROMAddr 0x%x \r\n",__func__,uVspaHdrTableEEPROMAddr);
		return 1;
	}
	//Copy VSPA TABLE HEADER INFO to TCMU  VSPA_HDR_TABLE_TCM_OFFSET offset
	CopyToTCM("LOAD VSPA TABLE HEADER",( uint32_t ) dtcm_addr + VSPA_HDR_TABLE_TCM_OFFSET, uVspaHdrTableEEPROMAddr, MAX_VSPA_TABLE_HDR_SIZE);
	vspa_hdr = (struct vspa_table_hdr * ) ((( uint32_t ) dtcm_addr + VSPA_HDR_TABLE_TCM_OFFSET));
	num_vspa_tbls = vspa_hdr->num_vspa_tbls;
	log_info("num_vspa_tbls %d \r\n",num_vspa_tbls);
	table_tcm_offset = 0;
	table_tcm_last_offset = uGetDTCMLocationFilled();
	uint32_t eeprom_tbl_file_addr  = uVspaHdrTableEEPROMAddr + MAX_VSPA_TABLE_HDR_SIZE;
//log_info("loc of first VSPA table in eeprom 0x%x \r\n",eeprom_tbl_file_addr);
	for(ctr = 0; ctr < num_vspa_tbls; ctr++)
	{
		//Calculate start of table tcm location from bottom
		table_tcm_offset = table_tcm_last_offset - vspa_hdr->vspa_tbl_info[ctr].tbl_size;
		//save offset in haeder for later use
		vspa_hdr->vspa_tbl_info[ctr]. tbl_tcm_loc= table_tcm_offset;
		//copy table into TCML location
		//vspa_hdr->vspa_tbl_info[ctr].eeprom_rel_addr_tbl_offset = eeprom_tbl_file_addr;
		strncpy(table_name,vspa_hdr->vspa_tbl_info[ctr].tbl_name,MAX_VSPA_BIN_FILE_NAME_LEN-1);
		CopyToTCM(table_name,( uint32_t ) table_tcm_offset, eeprom_tbl_file_addr , vspa_hdr->vspa_tbl_info[ctr].tbl_size);
		#ifdef DEBUG_VSPA_BOOT
		//log_info("eeprom_tbl_file_base_address 0x%x \r\n",eeprom_tbl_file_base_address);
		log_info("tbl_name %s table_tcm 0x%08x tbl_size 0x%x eeprom_tbl_file_addr 0x%x \r\n",
				vspa_hdr->vspa_tbl_info[ctr].tbl_name,vspa_hdr->vspa_tbl_info[ctr].tbl_tcm_loc,
				vspa_hdr->vspa_tbl_info[ctr].tbl_size,eeprom_tbl_file_addr);
		#endif
		table_tcm_last_offset = table_tcm_offset;
		//Get next table location in EEPROM
		eeprom_tbl_file_addr += vspa_hdr->vspa_tbl_info[ctr].tbl_size;
	}
	return 0;
}

/* To start the vspa core after dma is programmed */
int startup()
{

    uint32_t val, msb, lsb;
    uint32_t dma_channels;
    uint32_t vspa_sw_version, ippu_sw_version;
    int ctr;
    struct avi_hndlr * AviHndlr  = iLa9310AviHandle();
    struct vspa_regs * pVspaRegs = ( struct vspa_regs * ) AviHndlr->pVspaRegs;
    /* DMA channel usage */
    uint8_t         spm_dma_chan;
    uint8_t         bulk_dma_chan;
    uint8_t         reply_dma_chan;
    uint8_t         cmd_dma_chan;

    /* Ask the VSPA to go */
    val = IN_32( &pVspaRegs->vcpu_ctrl ); 
    val = (val & CONTROL_REG_MASK) | CONTROL_HOST_GO;
    OUT_32( &pVspaRegs->vcpu_ctrl, val); 
    //vspa_loading_count++;
    vspa_sw_version = IN_32( &pVspaRegs->sw_version );
    ippu_sw_version = IN_32( &pVspaRegs->ippuswver ); 
    /* Wait for the 64 bit mailbox bit to be set */

    for (ctr = VSPA_STARTUP_TIMEOUT; ctr; ctr--) {
        if (IN_32( &pVspaRegs->host_mbox_status ) &
            MBOX_STATUS_IN_64_BIT)
                break;
        vUDelay(1000);
    }

    if (!ctr) {
        log_info("timeout waiting for Boot Complete msg\r\n");
        goto startup_fail;
    }
    msb = IN_32( &pVspaRegs->host_in_0_msb ); 
    lsb = IN_32( &pVspaRegs->host_in_0_lsb ); 

    log_info("Boot Ok Msg: msb = %08X, lsb = %08X\r\n", msb, lsb);

    /* Check Boot Complete message */
    if (msb != 0xF1000000) {
        log_info("Boot Complete msg did not match\n");
        //goto startup_fail;
    } else {
        log_info("Boot Ok Msg Verified: msb = %08X, lsb = %08X\r\n", msb, lsb);
    }
    dma_channels = lsb;

    vspa_sw_version = IN_32( &pVspaRegs->sw_version ); 
    ippu_sw_version = IN_32( &pVspaRegs->ippuswver ); 

    /* Set SPM buffer */
    msb = (0x70 << 24) ;
    lsb = 0x00;
    OUT_32( &pVspaRegs->host_out_0_msb, msb);
    OUT_32( &pVspaRegs->host_out_0_lsb, lsb);
    vspa_sw_version = IN_32( &pVspaRegs->sw_version );
    ippu_sw_version = IN_32( &pVspaRegs->ippuswver );
    log_info("SW Version: vspa = %08X, ippu = %08X\r\n",
         vspa_sw_version, ippu_sw_version);
    /* Wait for the 64 bit mailbox bit to be set */
    for (ctr = VSPA_STARTUP_TIMEOUT; ctr; ctr--) {
        if (IN_32( &pVspaRegs->host_mbox_status ) &
            MBOX_STATUS_IN_64_BIT)
            break;
        vUDelay(1000);

    }
    if (!ctr) {
        log_info("timeout waiting for SPM Ack msg \r\n");
        goto startup_fail;
    }
    msb = IN_32( &pVspaRegs->host_in_0_msb );
    lsb = IN_32( &pVspaRegs->host_in_0_lsb );
    log_info("SPM Ack Msg: msb = %08X, lsb = %08X\r\n",
         msb, lsb);
    log_info("SPM Ack Msg: msb = %08X, lsb = %08X\r\n", msb, lsb);
    if (msb != 0xF0700000) {
        log_info(" SPM Ack error %08X\n", msb);
        goto startup_fail;
    }

    /*This piece of code is for passing the PCI base address to VSPA via
     * mailbox 0. Error Print will come since VSPA FW image is not having
     * response support for base address implemented */

    val = IN_32( &pVspaRegs->vcpu_ctrl );

    if (dma_channels) {
        spm_dma_chan = (dma_channels >> 24) & 0xFF;
        bulk_dma_chan = (dma_channels >> 16) & 0xFF;
        reply_dma_chan = (dma_channels >> 8) & 0xFF;
        cmd_dma_chan = (dma_channels) & 0xFF;
    } else {			/* legacy images */
        spm_dma_chan = VSPA_DMA_CHANNELS;
        bulk_dma_chan = 0xFF;
        reply_dma_chan = 0xFF;
        cmd_dma_chan = 0;
    }

    log_info("SW Version: vspa = %08X, ippu = %08X\r\n",
         vspa_sw_version, ippu_sw_version);
    log_info("DMA chan: spm %02X, bulk %02X, reply %02X, cmd %02X\r\n",
         spm_dma_chan, bulk_dma_chan,
         reply_dma_chan, cmd_dma_chan);

    vspa_sw_version = vspa_sw_version;
    ippu_sw_version = ippu_sw_version;

	msb = 0x0A << 24;           /*Opcode */
    msb = msb | 0x01; /*Offset */
    /*Set the lsb value */
    lsb = OVERLAY_SECTION_OFFSET; /* For offset value */

    OUT_32( &pVspaRegs->host_out_0_msb, msb);
    OUT_32( &pVspaRegs->host_out_0_lsb, lsb);
    OUT_32( &pVspaRegs->host_out_1_msb, msb);
    OUT_32( &pVspaRegs->host_out_1_lsb, lsb);

    vUDelay(1000);

    msb = IN_32( &pVspaRegs->host_in_0_msb );
    lsb = IN_32( &pVspaRegs->host_in_0_lsb );
	log_info("MBOX0 msb 0%x lsb 0x%x \r\n",msb,lsb);
    msb = IN_32( &pVspaRegs->host_in_1_msb );
    lsb = IN_32( &pVspaRegs->host_in_1_lsb );
	log_info("MBOX1 msb 0%x lsb 0x%x \r\n",msb,lsb);
    return 0;

startup_fail:
    vspa_sw_version = ~0;
    ippu_sw_version = ~0;
    OUT_32( &pVspaRegs->vspa_irqen, 0);

    return -1;
}

/*This function will program the DMA in polling mode */
int dma_raw_transmit(struct vspa_dma_req *dr)
{
    int stat_abort;
    uint32_t counter = 0;
    volatile int dma_comp_stat, xfr_err, cfg_err;
    struct avi_hndlr * AviHndlr  = iLa9310AviHandle();
    struct vspa_regs * pVspaRegs = ( struct vspa_regs * ) AviHndlr->pVspaRegs;

    /* Program the DMA transfer */
    OUT_32( &pVspaRegs->dma_dmem_pram_addr, dr->dmem_addr);
    OUT_32( &pVspaRegs->dma_axi_address, (dr->axi_addr));
    OUT_32( &pVspaRegs->dma_axi_byte_cnt, dr->byte_cnt);

    dma_comp_stat =  IN_32( &pVspaRegs->dma_comp_stat );

    log_info("dma_comp_stat = %d \r\n", dma_comp_stat);
    log_info("dmem_addr=0x%x \t axi_addr=0x%x \t byte_cnt=0x%x \t\
        xfer_ctrl=0x%x \r\n", dr->dmem_addr, (uint32_t) dr->axi_addr,
                        dr->byte_cnt, dr->xfr_ctrl);

    OUT_32( &pVspaRegs->dma_xfr_ctrl, dr->xfr_ctrl);


    counter = VSPA_DMA_MAX_COUNTER;

    while (!(dma_comp_stat & DMA_COMP_STAT_SET)) {
        dma_comp_stat =
            IN_32( &pVspaRegs->dma_comp_stat );
        vUDelay(1000);
        counter--;
    }


    if (!counter) {
        log_info("Timeout Error in Raw transmit\r\n");
        goto error;
    }

    dma_comp_stat = IN_32( &pVspaRegs->dma_comp_stat );

    stat_abort = IN_32( &pVspaRegs->dma_stat_abort );
    log_info("dma_comp_stat = %x stat_abort %x\r\n",
        dma_comp_stat, stat_abort);

    OUT_32( &pVspaRegs->dma_comp_stat, DMA_COMP_STAT_SET);

    xfr_err = IN_32( &pVspaRegs->dma_xfrerr_stat );
    cfg_err = IN_32( &pVspaRegs->dma_cfgerr_stat );
    log_info("DMA_XFRERR value = %x CFG error = %x\r\n",
        xfr_err, cfg_err);
    return 0;
error:
    return -1;
}

int CopyToTCM(char *name, uint32_t tcm_addr, uint32_t eprom_addr, uint32_t eprom_read_size )
{
    volatile uint8_t  *tcm_ptr;
    int itr,num_iteration;
    uint8_t ucval[MAX_EEPROM_READ_SIZE];
    int ret = 0;

    tcm_ptr = (uint8_t * ) (( uint32_t ) tcm_addr);
    num_iteration = (eprom_read_size / MAX_EEPROM_READ_SIZE); 	
    for( itr = 0; itr < num_iteration; itr++) 
    {
        ret = iLa9310_I2C_Read(LA9310_FSL_I2C1,IC2_EEPROM_DEV_ADDR ,eprom_addr + (itr * MAX_EEPROM_READ_SIZE) ,
                LA9310_I2C_DEV_OFFSET_LEN_2_BYTE, ucval, MAX_EEPROM_READ_SIZE );
        memcpy(((uint8_t *)(tcm_ptr + (itr * MAX_EEPROM_READ_SIZE))), (uint8_t *)ucval,MAX_EEPROM_READ_SIZE); 
    }

	#ifdef DEBUG_VSPA_BOOT
    log_info("\r\n%s %s tcm_addr 0x%x eprom_addr 0x%x eprom_read_size 0x%x\r\n",
		__func__,name,tcm_addr,eprom_addr,eprom_read_size);
    //hexdump((uint8_t * )tcm_ptr,eprom_read_size);
	#endif

    return ret;
}

void LoadVSPAImage()
{
    struct avi_hndlr * AviHndlr  = iLa9310AviHandle();
    struct vspa_regs * pVspaRegs = ( struct vspa_regs * ) AviHndlr->pVspaRegs;
    uint32_t param1;
    uint32_t param2;
    uint32_t axi_data_width;
    uint32_t dma_channels;
    uint32_t gp_out_regs;
    uint32_t gp_in_regs;
    uint32_t dmem_bytes;
    uint32_t ippu_bytes;
    uint32_t arithmetic_units;
    uint32_t val;
    uint32_t vspa_hw_version;
    uint32_t ippu_hw_version;
    uint32_t vspa_sw_version;
    uint32_t ippu_sw_version;
    volatile uint8_t  *vspa_image_ptr;
    int ctr,ret;
    uint32_t last_offset = 0;
    struct vspa_dma_req z_dma_req;
    uint32_t dmem_addr;
    uint32_t axi_addr;
    uint32_t byte_cnt;
    uint32_t xfer_ctrl;
    void * dtcm_addr;
    struct vspa_image_hdr *vspa_hdr;
    uint32_t num_sections;
	char section_name[MAX_SECTION_NAME];
	uint32_t  uNumPages = 0;
	uint32_t uVspaHdrEEPROMAddr;
    /* Vspa hardware initialization */
    param1 = IN_32( &pVspaRegs->param1 );
    axi_data_width = 32 << ((param1 >> 28) & 7);
    dma_channels = (param1 >> 16) & 0xFF;
    gp_out_regs = (param1 >> 8) & 0xFF;
    gp_in_regs = param1 & 0xFF;
    param2 = IN_32( &pVspaRegs->param2 );
    dmem_bytes = ((param2 >> 8) & 0x3FF) * 400;
    ippu_bytes = (param2 >> 31) * 4096;
    arithmetic_units = param2 & 0xFF;
    log_info( "param0 0x%x \n\r", IN_32( &pVspaRegs->param0 ));
    log_info( "param1 0x%x \n\r", param1 );
    log_info( "param2 0x%x \n\r", param2 );
    log_info( "axi_data_width 0x%x \n\r", axi_data_width);
    log_info( "dma_channels 0x%x \n\r", dma_channels);
    log_info( "gp_out_regs 0x%x \n\r", gp_out_regs);
    log_info( "gp_in_regs 0x%x \n\r", gp_in_regs);
    log_info( "dmem_bytes 0x%x \n\r", dmem_bytes);
    log_info( "ippu_bytes 0x%x \n\r", ippu_bytes);
    log_info( "arithmetic_units 0x%x \n\r", arithmetic_units);



    vspa_hw_version = IN_32( &pVspaRegs->hw_version );
    ippu_hw_version =  IN_32( &pVspaRegs->ippuhwver );             
    vspa_sw_version = ~0;
    ippu_sw_version = ~0;
    log_info("vspa_hw_version 0x%x ippu_hw_version 0x%x vspa_sw_version 0x%x ippu_sw_version 0x%x \r\n",
        vspa_hw_version,ippu_hw_version,vspa_sw_version,ippu_sw_version);

    /* Enable core power gating */
    val = IN_32( &pVspaRegs->vcpu_ctrl );
    val = (val & CONTROL_REG_MASK) | CONTROL_PDN_EN;
    OUT_32( &pVspaRegs->vcpu_ctrl, val);

    /* Make sure all interrupts are disabled */
    OUT_32( &pVspaRegs->vspa_irqen, 0); 

    dtcm_addr = pLa9310Info->dtcm_addr;
    vspa_image_ptr = (uint8_t * ) ((( uint32_t ) dtcm_addr + VSPA_DMA_AXI_ADDR_OFFSET));
    log_info("DTCM ZEROIZED START 0x%x \r\n",(uint32_t)vspa_image_ptr);
    memset((uint8_t * )vspa_image_ptr, 0x00,MAX_STAGING_AREA_SIZE);

    dmem_addr=0;
    axi_addr= ( uint32_t ) vspa_image_ptr;

    byte_cnt=0x1900;
    xfer_ctrl=0x00;
    z_dma_req.dmem_addr = dmem_addr;
    z_dma_req.axi_addr = axi_addr;
    z_dma_req.byte_cnt = byte_cnt;
    z_dma_req.xfr_ctrl = xfer_ctrl;
    dma_raw_transmit(&z_dma_req);

    byte_cnt=0x8000;
    xfer_ctrl=0x200;
    z_dma_req.dmem_addr = dmem_addr;
    z_dma_req.axi_addr = axi_addr;
    z_dma_req.byte_cnt = byte_cnt;
    z_dma_req.xfr_ctrl = xfer_ctrl;
    dma_raw_transmit(&z_dma_req);

    byte_cnt=0x1000;
    xfer_ctrl=0x300;
    z_dma_req.dmem_addr = dmem_addr;
    z_dma_req.axi_addr = axi_addr;
    z_dma_req.byte_cnt = byte_cnt;
    z_dma_req.xfr_ctrl = xfer_ctrl;
    dma_raw_transmit(&z_dma_req);
	ret = iGetExtTableInfo(EXT_VSPA_BIN_EEPROM_ADDR,&uVspaHdrEEPROMAddr);
	if(ret != 0)
		log_info("%s Invalid uVspaHdrEEPROMAddr 0x%x \r\n",__func__,uVspaHdrEEPROMAddr);
	CopyToTCM("LOAD VSPA BIN HEADER",( uint32_t ) dtcm_addr + VSPA_HDR_TCM_OFFSET, uVspaHdrEEPROMAddr ,MAX_VSPA_HDR_SIZE);
	vspa_hdr = (struct vspa_image_hdr * ) ((( uint32_t ) dtcm_addr + VSPA_HDR_TCM_OFFSET));
	num_sections = vspa_hdr->num_sections;

	#ifdef NLM_VERBOSE_DEBUG 
	log_info("num_sections %d \r\n",vspa_hdr->num_sections);
	uTcmLocationFilled = VSPA_OVERLAY_TABLE_TCMU_START_LOC;
	for(ctr = 0 ; ctr < num_sections; ctr++ )
	{
		strncpy(section_name,vspa_hdr->dma_sec_info[ctr].section_name,MAX_SECTION_NAME-1);
        log_info("Section# %d SectionName %s is_overlay %d dmem_addr 0x%x byte_cnt 0x%x xfr_ctrl. 0x%x eeprom_rel_addr_offset %x\r\n",
                ctr, section_name,vspa_hdr->dma_sec_info[ctr].is_overlay,
				vspa_hdr->dma_sec_info[ctr].dmem_addr,vspa_hdr->dma_sec_info[ctr].byte_cnt,
					vspa_hdr->dma_sec_info[ctr].xfr_ctrl,vspa_hdr->dma_sec_info[ctr].eeprom_rel_addr_offset);
    }
	#endif
    z_dma_req.axi_addr = axi_addr;
    last_offset = 0;
    for(ctr = 0 ; ctr < num_sections; ctr++ )
	{
		strncpy(section_name,vspa_hdr->dma_sec_info[ctr].section_name,MAX_SECTION_NAME-1);
		if(!vspa_hdr->dma_sec_info[ctr].is_overlay)
		{
			z_dma_req.axi_addr = axi_addr;
		}
		else
		{   
			/* make overlay memory resident at  TCM overlay location at page aligned boundary*/ 
			uNumPages	= vspa_hdr->dma_sec_info[ctr].byte_cnt / TCM_PAGE_ALIGN;
			if(vspa_hdr->dma_sec_info[ctr].byte_cnt % TCM_PAGE_ALIGN)
				uNumPages++;
			z_dma_req.axi_addr = uTcmLocationFilled -  (uNumPages * TCM_PAGE_ALIGN);
			/* store last filled loaction*/
			uTcmLocationFilled = z_dma_req.axi_addr;
		}
		CopyToTCM(section_name,z_dma_req.axi_addr, uVspaHdrEEPROMAddr + MAX_VSPA_HDR_SIZE + last_offset, vspa_hdr->dma_sec_info[ctr].byte_cnt);
		vspa_hdr->dma_sec_info[ctr].axi_tcm_addr = z_dma_req.axi_addr;
		z_dma_req.dmem_addr =  vspa_hdr->dma_sec_info[ctr].dmem_addr;
		z_dma_req.byte_cnt = vspa_hdr->dma_sec_info[ctr].byte_cnt;
		z_dma_req.xfr_ctrl = vspa_hdr->dma_sec_info[ctr].xfr_ctrl;
		last_offset += vspa_hdr->dma_sec_info[ctr].byte_cnt;
		/* vspa dma to non overlay section */
		if(!vspa_hdr->dma_sec_info[ctr].is_overlay)
		{
			dma_raw_transmit(&z_dma_req);
		}

		#ifdef NLM_VERBOSE_DEBUG 
		log_info("Section# %d  SectionName %s NumPages %d axi_tcm_addr 0x%x dmem_addr 0x%x byte_cnt 0x%x xfr_ctrl. 0x%x eeprom_rel_addr_offset 0x%x last_offset 0x%x\r\n",
			ctr,section_name,uNumPages,vspa_hdr->dma_sec_info[ctr].axi_tcm_addr, vspa_hdr->dma_sec_info[ctr].dmem_addr,vspa_hdr->dma_sec_info[ctr].byte_cnt,vspa_hdr->dma_sec_info[ctr].xfr_ctrl,vspa_hdr->dma_sec_info[ctr].eeprom_rel_addr_offset,last_offset);
		#endif
	}
	//store last tcm filled location
	uTcmLocationFilled = z_dma_req.axi_addr;
	/* Load VSPA TABLES */
	iLoadVSPATablesEEPROM_To_TCM();
    startup();
}
