/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2022 NXP
 */


#include "FreeRTOS.h"
#include "task.h"
#include "la9310_main.h"
#include "common.h"
#include <delay.h>
#include "la9310_i2cAPI.h"

extern void LoadVSPAImage(void);
#define NLM_VERBOSE_DEBUG
#define OVERLAY_SECTION_OFFSET    0x00  /*TBD*/
#define VSPA_DMA_MAX_COUNTER       30
#define DMA_COMP_STAT_SET          0x01

#define VSPA_HALT_TIMEOUT       (100000)
#define VSPA_STARTUP_TIMEOUT    (100000)
#define VSPA_OVERLAY_TIMEOUT    (100)

#define CONTROL_REG_MASK        (~0x000100FF)
#define CONTROL_PDN_EN          (1<<31)
#define CONTROL_HOST_MSG_GO     (1<<20 | 1<<21 | 1<<22 | 1<<23)
#define CONTROL_VCPU_RESET      (1<<16)
#define CONTROL_DEBUG_MSG_GO    (1<<5)
#define CONTROL_IPPU_GO         (1<<1)
#define CONTROL_HOST_GO         (1<<0)

#define MBOX_STATUS_IN_1_64_BIT         (0x00000008)
#define MBOX_STATUS_IN_64_BIT           (0x00000004)
#define MBOX_STATUS_OUT_1_64_BIT        (0x00000002)
#define MBOX_STATUS_OUT_64_BIT          (0x00000001)


#define DMA_COMP_STAT_SET       0x01
#define VSPA_DMA_MAX_COUNTER    30
#define VSPA_DMA_TIMEOUT        100
#define VSPA_DMA_CHANNELS		(32)

#define MAX_VSPA_HDR_SIZE		512
#define MAX_VSPA_TABLE_HDR_SIZE    	512
#define MAX_EXTENDED_TABLE_HDR_SIZE	32

#define MAX_EEPROM_READ_SIZE 	4

#define EXTENDED_HDR_EEPROM_ADDR (0x20)
#define VSPA_HDR_TCM_OFFSET    0x0000
#define VSPA_HDR_TABLE_TCM_OFFSET   (VSPA_HDR_TCM_OFFSET + MAX_VSPA_HDR_SIZE)
#define EXTENDED_HDR_TABLE_TCM_OFFSET   (VSPA_HDR_TABLE_TCM_OFFSET + MAX_VSPA_TABLE_HDR_SIZE)
#define MAX_STAGING_AREA_SIZE (32 * 1024)
#define MAX_VSPA_DMA_OVERLAY_SECTIONS       6
#define MAX_OVERLAY_SECTIONS            3
#define VSPA_DMA_AXI_ADDR_OFFSET        0x1000 
#define IC2_EEPROM_DEV_ADDR    0x50
#define MAX_SECTION_NAME       20
#define TCML_END_LOC            0x1F81FFFF
#define TCMU_END_LOC            0x2000FFFF
#define LOG_BUFFER_SIZE         ( 4 * 1024 )
#define LOG_BUFFER_LOC          (TCMU_END_LOC - LOG_BUFFER_SIZE)
#define VSPA_OVERLAY_TABLE_TCMU_START_LOC  (TCMU_END_LOC - LOG_BUFFER_SIZE + 1)
#define MAX_VSPA_BIN_FILES            3
#define MAX_VSPA_BIN_FILE_NAME_LEN   32
#define TCM_PAGE_ALIGN               0x1000
struct vspa_dma_section_info {
	char            section_name[MAX_SECTION_NAME];
	unsigned int        is_overlay;
	unsigned int        dmem_addr;
	unsigned int        byte_cnt;
	unsigned int        xfr_ctrl;
	unsigned int        axi_tcm_addr;
	unsigned int        eeprom_rel_addr_offset;
};

struct vspa_bin_table_info {
	char            tbl_name[MAX_VSPA_BIN_FILE_NAME_LEN];
	unsigned int    tbl_size;
	unsigned int    tbl_tcm_loc;
};

struct vspa_table_hdr
{
	unsigned int num_vspa_tbls;
	struct vspa_bin_table_info vspa_tbl_info[MAX_VSPA_BIN_FILES];
};

struct vspa_image_hdr
{
	unsigned int num_sections;
	struct vspa_dma_section_info dma_sec_info[MAX_VSPA_DMA_OVERLAY_SECTIONS];
};
struct vspa_dma_req {
    union {
      uint32_t      control;
      struct {
        uint8_t     id;
        uint8_t     flags;
        uint8_t     rsvd;
        uint8_t     type;
      };
    };
    uint32_t        dmem_addr;
    uint32_t        axi_addr;
    uint32_t        byte_cnt;
    uint32_t        xfr_ctrl;
};

struct ext_header {
   uint32_t page_write_size;
   uint32_t addr_shift;
   uint32_t freertos_image_offset;
   uint32_t vspa_bin_location;
   uint32_t vspa_table_location;
   uint32_t bootstrapper_image_offset;
   uint32_t reserved2;
   uint32_t reserved3;
};

typedef enum ExtTableInfo
{
    EXT_EEPROM_PAGE_SIZE,
    EXT_EEPROM_ADDR_SHIFT,
    EXT_FREE_RTOS_IMAGE_EEPROM_ADDR,
    EXT_VSPA_BIN_EEPROM_ADDR,
	EXT_VSPA_TABLE_LOCATION_ADDR
} ExtTableInfo_t;

extern int CopyToTCM(char *name, uint32_t tcm_addr, uint32_t eprom_addr, uint32_t eprom_read_size );
/*This function will program the DMA in polling mode */
extern int dma_raw_transmit(struct vspa_dma_req *dr);
extern int iLoadOverlay(char *ovl_sec_name, bool bVerifyLoad);
extern int startup();
extern int iLoadTableToTCM();
extern int iGetExtTableInfo(ExtTableInfo_t eExtTableInfo,uint32_t *uPtrInfo);
