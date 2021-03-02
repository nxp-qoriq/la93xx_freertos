/*****************************************************************************************
 *
 * FILE NAME          : sync_timing_core_aruba_interface.h
 *
 * AUTHOR             : Srini Venkataraman
 *
 * DATE CREATED       : 12/18/2019
 *
 * DESCRIPTION        : Interface file for Aruba register access
 *
 ****************************************************************************************/

/****************************************************************************************/
/**                  Copyright (c) 2019, 2021 Skyworks Solution Inc.                   **/
/****************************************************************************************/
/** This software is provided 'as-is', without any express or implied warranty.        **/
/** In no event will the authors be held liable for any damages arising from the use   **/
/** of this software.                                                                  **/
/** Permission is granted to anyone to use this software for any purpose, including    **/
/** commercial applications, and to alter it and redistribute it freely, subject to    **/
/** the following restrictions:                                                        **/
/** 1. The origin of this software must not be misrepresented; you must not claim that **/
/**    you wrote the original software. If you use this software in a product,         **/
/**    an acknowledgment in the product documentation would be appreciated but is not  **/
/**    required.                                                                       **/
/** 2. Altered source versions must be plainly marked as such, and must not be         **/
/**    misrepresented as being the original software.                                  **/
/** 3. This notice may not be removed or altered from any source distribution.         **/
/****************************************************************************************/

#ifndef _SYNC_TIMING_CORE_ARUBA_INTERFACE_H_
#define _SYNC_TIMING_CORE_ARUBA_INTERFACE_H_

/*****************************************************************************************
    Include Header Files
    (No absolute paths - paths will be handled by Makefile)
*****************************************************************************************/


/*****************************************************************************************
    Macros
*****************************************************************************************/
//#define SYNC_ADDL_REG_TEST_DEBUG

#define SYNC_TIMING_OP_SET_OFFSET                   0x00
#define SYNC_TIMING_OP_SET_ADDR                     0x10
#define SYNC_TIMING_OP_SET_ADDR24                   0x20
#define SYNC_TIMING_OP_SET_BLEN                     0x30

#define SYNC_TIMING_OP_WR_WDATA                     0x40
#define SYNC_TIMING_OP_WR_WBURST                    0x50
#define SYNC_TIMING_OP_WR_WDATAU                    0x60
#define SYNC_TIMING_OP_WR_WBURSTU                   0x70

#define SYNC_TIMING_OP_RD_RDATA                     0x80
#define SYNC_TIMING_OP_RD_RBURST                    0x90
#define SYNC_TIMING_OP_RD_RDATAU                    0xA0
#define SYNC_TIMING_OP_RD_RBURSTU                   0xB0

#define SYNC_TIMING_OP_API_CMD                      0xC0
#define SYNC_TIMING_OP_API_REPLY                    0xD0
#define SYNC_TIMING_OP_API_CMD_REPLY                0xFF0

#define SYNC_TIMING_PLL_MAX_ADJ_OFFSET_PPB          65536


// Interrupt registers
#define SYNC_TIMING_INTC_IRQx_START_A1              0x0011
#define SYNC_TIMING_INTC_IENx_START_A1              0x001B

#define SYNC_TIMING_INTC_IRQx_START                 0x0011
#define SYNC_TIMING_INTC_IENx_START                 0x1000

// MCU BOOTSTATE Register
#define SYNC_TIMING_MCU_BOOTSTATE                   0x0F14

#define BOOTLOADER_READY_STATE                      0x16
#define APPLICATION_READY_STATE                     0x49
#define CHIP_OUTPUT_READY_STATE                     0xB0


/*****************************************************************************************
    User-Defined Types (Typedefs)
 ****************************************************************************************/

/*****************************************************************************************
    Global Variable Declarations
 ****************************************************************************************/

/*****************************************************************************************
    Prototypes
 ****************************************************************************************/

#endif //_SYNC_TIMING_CORE_ARUBA_INTERFACE_H_

