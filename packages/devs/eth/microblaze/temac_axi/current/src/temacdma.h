//==========================================================================
//
//      temacdma.h
//
//      Header file for temacdma.c file, contains functions for Temac DMA instance
//
//==========================================================================
//####ECOSGPLCOPYRIGHTBEGIN####
// -------------------------------------------
// This file is part of eCos, the Embedded Configurable Operating System.
//
// eCos is free software; you can redistribute it and/or modify it under
// the terms of the GNU General Public License as published by the Free
// Software Foundation; either version 2 or (at your option) any later version.
//
// eCos is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
// for more details.
//
// You should have received a copy of the GNU General Public License along
// with eCos; if not, write to the Free Software Foundation, Inc.,
// 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
//
// As a special exception, if other files instantiate templates or use macros
// or inline functions from this file, or you compile this file and link it
// with other works to produce a work based on this file, this file does not
// by itself cause the resulting work to be covered by the GNU General Public
// License. However the source code for this file must still be made available
// in accordance with section (3) of the GNU General Public License.
//
// This exception does not invalidate any other reasons why a work based on
// this file might be covered by the GNU General Public License.
//
// -------------------------------------------
//####ECOSGPLCOPYRIGHTEND####
//==========================================================================
//#####DESCRIPTIONBEGIN####
//
// Author(s):    Kolb Alexandr, Pavel Azizov
// Contributors: 
// Date:         2011-12-09
// Purpose:      
// Description:  
//              
//
//####DESCRIPTIONEND####
//
//==========================================================================


#ifndef TEMACDMA_H     /* prevent circular inclusions */
#define TEMACDMA_H     /* by using protection macros */


#include "src/xaxiethernet.h"
#include "src/xaxidma.h"
#include "src/xaxiethernet_hw.h"
#include "src/xil_assert.h"
#include "src/xil_io.h"
#include <cyg/io/eth/eth_drv.h>
#include <pkgconf/hal_microblaze_platform.h>



/// \brief This defines maximum transmission unit for dma MUST BE multiple of cache line size
#define TEMACDMA_MTU	1536

/// \brief temac irq status codes
#define TEMACDMA_IRQ_TX 0x01
#define TEMACDMA_IRQ_RX 0x02

#define MAX_MULTICAST_ADDR   (1<<23) //Maximum number of multicast ethernet mac addresses

cyg_uint32 McastAddressTable[(MAX_MULTICAST_ADDR>>3)/sizeof(cyg_uint32)]; //1 MB for storing multicast address table


/// \brief handle of Temac DMA, contains receive/transmit buffers
struct temacdma_handle
{
    XAxiDma      temac_dma __attribute__ ((aligned (HAL_DCACHE_LINE_SIZE)));

    char mem4rxring[XAxiDma_BdRingMemCalc(XAXIDMA_BD_MINIMUM_ALIGNMENT, CYGNUM_DEVS_ETH_MICROBLAZE_DMA_BDRING_SIZE)] __attribute__ ((aligned (XAXIDMA_BD_MINIMUM_ALIGNMENT)));
    char mem4txring[XAxiDma_BdRingMemCalc(XAXIDMA_BD_MINIMUM_ALIGNMENT, CYGNUM_DEVS_ETH_MICROBLAZE_DMA_BDRING_SIZE)] __attribute__ ((aligned (XAXIDMA_BD_MINIMUM_ALIGNMENT)));


	unsigned char temac_rx_bufs[CYGNUM_DEVS_ETH_MICROBLAZE_DMA_BDRING_SIZE][TEMACDMA_MTU] __attribute__ ((aligned (HAL_DCACHE_LINE_SIZE)));
	unsigned char temac_tx_bufs[CYGNUM_DEVS_ETH_MICROBLAZE_DMA_BDRING_SIZE][TEMACDMA_MTU] __attribute__ ((aligned (HAL_DCACHE_LINE_SIZE)));
};


void temacdma_tx_int_disable(struct temacdma_handle * handle);

void temacdma_rx_int_disable(struct temacdma_handle * handle);

void temacdma_int_enable(struct temacdma_handle * handle);

void temacdma_init(struct temacdma_handle * handle);

void temacdma_start(struct temacdma_handle * handle);

cyg_uint32 temacdma_check(struct temacdma_handle * handle);

int temacdma_can_send(struct temacdma_handle * handle);

void temacdma_rx_get(struct temacdma_handle * handle, cyg_uint8 * buf, cyg_uint16 * length);

cyg_uint32 temacdma_tx_finish(struct temacdma_handle * handle, cyg_uint32 * key);

void temacdma_send(struct temacdma_handle * handle, struct eth_drv_sg *sg_list,int sg_len, int total_len, unsigned long key);

#endif



