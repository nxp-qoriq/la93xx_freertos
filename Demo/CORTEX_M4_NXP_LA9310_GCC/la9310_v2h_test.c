/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2017-2018, 2021-2024 NXP
 */

#include "FreeRTOS.h"
#include <common.h>
#include <la9310_v2h_if.h>
#include <debug_console.h>
#include <string.h>
#include <la9310_irq.h>
#include <la9310_pci.h>
#include <config.h>
#include <task.h>

#define MAX_BACKOUT                  1
#define MAX_SENT_PKT                 1000000
#define ENABLE_V2H_INTR_OTHER_CPU    0
#define V2H_INTR_CPU_NUM             1

/* static initializes headPointer to 0 */
static int headPointer;

/* Prints ownership and value of every BD from Ring */
void printBDRing( struct v2h_buffer_desc * bdPointer )
{
    for( int headP = 0; headP < V2H_MAX_BD; headP++ )
    {
        if( bdPointer[ headP ].owner == OWNER_VSPA )
        {
            log_dbg( "BD [ %d ] = OWNER_VSPA\n\r", headP );
        }
        else if( bdPointer[ headP ].owner == OWNER_HOST )
        {
            log_dbg( "BD [ %d ] = OWNER_HOST\n\r", headP );
        }
        else
        {
            log_dbg( "BD [ %d ] = OWNER_UNKNOWN\n\r", headP );
        }

        log_dbg( "BD [ %d ] DMA Address Stored = %#x\n\r", headP,
                 bdPointer[ headP ].host_phys_addr );
    }
}

/* Creates outbound window between host an LA931x */
static int la9310_create_outbound_v2h( struct la9310_info * pLa9310Info,
                                       uint64_t physAddr,
                                       uint64_t pciAddr,
                                       uint32_t size )
{
    /*
     * Outbound iATU for LA931x bin memory.
     * Access memory from LA931x to host.
     * Copy .bin file from host memory to LA931x memory.
     */

    vLa9310PcieiAtuOutboundSet( ( void * ) PCIE_BASE_ADDR, LA9310_V2H_OUTBOUND_WIN,
                                PCIE_ATU_TYPE_MEM,
                                pciAddr,  /* CPU addr */
                                physAddr, /* PCIe addr one-to-one map */
                                size );
    return 0;
}

/* Copies pattern passed to it */
int copyFrame( struct v2h_buffer_desc * bdPointer,
               int head,
               uint8_t pattern,
               uint32_t frameSize )
{
    uint64_t physAddr;
    uint32_t pciAddr;
    uint32_t size;
    uint32_t headerSize, frameLength;
    struct v2h_frame_ctrl * frm_ctrl;

    if( head == -1 )
    {
        log_dbg( "All BDs are currently assigned to host\n" );
        return -1;
    }

    headerSize = sizeof( struct v2h_headroom );
    frameLength = sizeof( struct v2h_frame_ctrl );

    size = 4096;
    physAddr = bdPointer[ head ].host_phys_addr;
    pciAddr = bdPointer[ head ].la9310_pci_addr;
    la9310_create_outbound_v2h( pLa9310Info, physAddr, pciAddr, size );
    dmb();
    memset( ( uint32_t * ) ( pciAddr + headerSize ), pattern, frameSize );
    frm_ctrl = ( struct v2h_frame_ctrl * ) ( pciAddr + headerSize -
                                             frameLength );
    frm_ctrl->len = frameSize;

    /*
     * Since the frame has been copied to host, the ownership of
     * current BD is to be transferred to host.
     */
    dmb();
    bdPointer[ head ].owner = OWNER_HOST;

    return 1;
}

/*
 * Copies a pattern frame, changes ownership to Host and raises MSI
 * for number of iterations.
 */
void vV2HDemo( struct la9310_info * pLa9310Info )
{
    void * itcm_addr;
    struct v2h_ring * ring_ptr;
    struct v2h_buffer_desc * bdPointer;
    volatile int delay;
    volatile int delay_ctr = 0;
    uint32_t iterations = 1;
    int pattern = 0x00;
    struct la9310_hif * pHif = pLa9310Info->pHif;
    int flag_last_poll_busy = 0;
    int cnt = 0;

    #if ENABLE_V2H_INTR_OTHER_CPU
        struct la9310_msi_info * pMsiInfo;

        pMsiInfo = &pLa9310Info->msi_info[ MSI_IRQ_V2H ];
        pMsiInfo->data = pMsiInfo->data + V2H_INTR_CPU_NUM;
    #endif

    log_info( "%s:V2H Demo\n\r", __func__ );
    itcm_addr = pLa9310Info->itcm_addr;
    ring_ptr = ( struct v2h_ring * ) ( ( uint32_t ) itcm_addr +
                                       LA9310_V2H_RING_OFFSET );
    bdPointer = ring_ptr->bd_info;
    pHif->stats.v2h_intr_enabled = 1;
    headPointer = 0;

    while( iterations <= MAX_SENT_PKT )
    {
        /*Delay mimic the packet processing time
         *  in Air interace, VSPA etc*/
        for( delay = 0; delay < 250; delay++ )
        {
            delay_ctr++;
        }

        if( bdPointer[ headPointer ].owner == OWNER_VSPA )
        {
            /*
             * If BD Ring was filled previously and has some space
             * now then update resume stats
             */
            if( flag_last_poll_busy == 1 )
            {
                flag_last_poll_busy = 0;

                if( pHif->stats.v2h_resumed < MAX_SENT_RESUME )
                {
                    /*first 10 drop resume stats*/
                    pHif->stats
                       .v2h_last_sent_pkt_resumed
                    [ pHif->stats.v2h_resumed ] =
                        pHif->stats.v2h_last_sent_pkt;
                    pHif->stats
                       .v2h_last_dropped_pkt_resumed
                    [ pHif->stats.v2h_resumed ] =
                        pHif->stats
                           .v2h_last_dropped_pkt;
                }

                pHif->stats.v2h_resumed++;
            }

            /* Call function to copy frame from LA9310 to host */
            copyFrame( bdPointer, headPointer, pattern++, 128 );
            headPointer = ( headPointer + 1 ) % V2H_MAX_BD;

            /* Update stats */
            pHif->stats.v2h_sent_pkt++;
            pHif->stats.v2h_last_sent_pkt = iterations;
            dmb();

            if( pHif->stats.v2h_intr_enabled != 0 )
            {
                /* Raise MSI */
                vRaiseMsi( pLa9310Info, MSI_IRQ_V2H );
                dmb();
                enable_irq();
                dmb();
            }
        }
        else
        {
            pHif->stats.v2h_backout_count++;

            /*
             * If resending of packet fails for MAX_BACKOUT
             * times, then drop the packet.
             */
            pHif->stats.v2h_dropped_pkt++;
            pHif->stats.v2h_last_dropped_pkt = iterations;
            dmb();
            log_dbg( "pkt sent %d dropped %d\n\r",
                     pHif->stats.v2h_sent_pkt,
                     pHif->stats.v2h_dropped_pkt );
            flag_last_poll_busy = 1;
        }

        iterations++;
    }

    /*  There can be a situation when the data is written to ring but
     *  MSI could not ve raised for few last packets which can make those
     *  packets unprocessed and remain in ring.
     *  Send an interrupt explicity to make sure all packets are processed.
     */
     vRaiseMsi( pLa9310Info, MSI_IRQ_V2H );

    /* Print final BD Ring in stats */
    for( cnt = 0; cnt < V2H_MAX_BD; cnt++ )
    {
        pHif->stats.v2h_final_ring_owner[ cnt ] = bdPointer[ cnt ].owner;
    }
    log_info( "%s:V2H Demo done ..\n\r", __func__ );
}
