//==========================================================================
//
//
//      Xilinx SP605 ethernet support
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
// Author(s):    Alexandr Kolb, Pavel Azizov
// Contributors:
// Date:         2011-12-09
// Purpose:      
// Description:  eCos hardware driver for Xilinx SP605
//              
//
//####DESCRIPTIONEND####
//
//==========================================================================

/**
*
*@file temacdma.c
*
* Contains required functions of the eCos Ethernet driver on Xilinx SP605 board.
**/

#include <cyg/infra/cyg_type.h>
#include <cyg/infra/diag.h>

#include <cyg/hal/hal_arch.h>
#include <cyg/hal/hal_cache.h>
#include <cyg/hal/hal_intr.h>
#include <cyg/hal/drv_api.h>
#include <cyg/hal/hal_if.h>
#include <pkgconf/hal_microblaze_platform.h>
#include <pkgconf/devs_eth_microblaze_temac.h>


#include <src/xparameters.h>

//#include "adapter.h"
#include <src/xlltemac.h>
#include <src/xlldma.h>
#include <src/xlldma_bdring.h>
#include "temacdma.h"
#include "src/xenv.h"

#include <cyg/hal/platform.h>	/* platform setting */

/**
* Disable transmit interrupt of Temac DMA
*
* @param    handle is a pointer to Temac DMA structure
*
* @return   None
*
* @note     None
*
**/

void temacdma_tx_int_disable(struct temacdma_handle * handle)
{
    XLlDma* inst_ptr = &handle->temac_dma;

    cyg_uint32 mask = XLLDMA_CR_IRQ_EN_MASK | XLLDMA_CR_IRQ_COALESCE_EN_MASK;

    XLlDma_BdRing *tx_ring_ptr = &(XLlDma_GetTxRing(inst_ptr));

	XLlDma_BdRingIntDisable(tx_ring_ptr, mask);
}

/**
* Disable receive interrupt of Temac DMA
*
* @param    handle is a pointer to Temac DMA structure
*
* @return   None
*
* @note     None
*
**/


void temacdma_rx_int_disable(struct temacdma_handle * handle)
{
    XLlDma* inst_ptr = &handle->temac_dma;

    cyg_uint32 mask = XLLDMA_CR_IRQ_EN_MASK | XLLDMA_CR_IRQ_COALESCE_EN_MASK;

    XLlDma_BdRing *rx_ring_ptr = &(XLlDma_GetRxRing(inst_ptr));

	XLlDma_BdRingIntDisable(rx_ring_ptr, mask);
}

/**
* Enable interrupts of transmit/receive channels of Temac DMA
*
* @param    handle is a pointer to Temac DMA structure
*
* @return   None
*
* @note     None
*
**/

void temacdma_int_enable(struct temacdma_handle * handle)
{
    XLlDma* inst_ptr = &handle->temac_dma;

    cyg_uint32 mask = XLLDMA_CR_IRQ_EN_MASK | XLLDMA_CR_IRQ_COALESCE_EN_MASK;

	XLlDma_BdRing* tx_ring_ptr = &(XLlDma_GetTxRing(inst_ptr));
	XLlDma_BdRing* rx_ring_ptr = &(XLlDma_GetRxRing(inst_ptr));

	XLlDma_BdRingIntEnable(tx_ring_ptr, mask);
	XLlDma_BdRingIntEnable(rx_ring_ptr, mask);
}

/**
* Initializes a specific Temac DMA instance, initializes the transmit and receive ring buffers.
*
* @param    handle is a pointer to Temac DMA structure
*
* @return   None
*
* @note     None.
*
**/

void temacdma_init(struct temacdma_handle * handle)
{
    XLlDma_BdRing *rx_ring_ptr;
    XLlDma_BdRing *tx_ring_ptr;
    XLlDma_Bd BdTemplate;
    cyg_uint32 FreeBdCount;
    XLlDma_Bd* bd_ptr, *BdCurPtr;
    cyg_uint32 status;
    cyg_uint32 i;
    cyg_uint32 bd_count;

    /*
     * Initialize Temac DMA instance
     */
    XLlDma_Initialize(&handle->temac_dma, MON_DMA_EMAC_BASEADDR + 0x00000080);

    rx_ring_ptr = &(XLlDma_GetRxRing(&handle->temac_dma));
    tx_ring_ptr = &(XLlDma_GetTxRing(&handle->temac_dma));

    /*
     * Disable Temac DMA interrupts
     */
    XLlDma_BdRingIntDisable(rx_ring_ptr, XLLDMA_CR_IRQ_ALL_EN_MASK);
    XLlDma_BdRingIntDisable(tx_ring_ptr, XLLDMA_CR_IRQ_ALL_EN_MASK);

    /*
     * Set the value to control register of Temac DMA instance for receive channel
     */
    XLlDma_WriteReg(handle->temac_dma.RegBase, XLLDMA_RX_OFFSET + XLLDMA_CR_OFFSET,
            0x01010100);

    bd_count = XLlDma_BdRingCntCalc(XLLDMA_BD_MINIMUM_ALIGNMENT,sizeof(handle->mem4rxring));

    if (bd_count>CYGNUM_DEVS_ETH_MICROBLAZE_DMA_BDRING_SIZE)
    {
        bd_count = CYGNUM_DEVS_ETH_MICROBLAZE_DMA_BDRING_SIZE;
    }

    /*
     * Create ring buffer for receive channel of Temac DMA
     */
    status = XLlDma_BdRingCreate(rx_ring_ptr,
                                 handle->mem4rxring,
                                 handle->mem4rxring,
                                 XLLDMA_BD_MINIMUM_ALIGNMENT,
                                 bd_count);

    if (status != XST_SUCCESS) {
        diag_printf("_XLlDma_RxBuffer_ - can't initialize_, %X\n", status);
    }

    /*
     * Set the value to control register of Temac DMA instance for transmit channel
     */
    XLlDma_WriteReg(handle->temac_dma.RegBase, XLLDMA_TX_OFFSET + XLLDMA_CR_OFFSET,
                    0x01010100);

    /*
     * Create ring buffer for transmit channel of Temac DMA instance
     */
    status = XLlDma_BdRingCreate(tx_ring_ptr,
                                    handle->mem4txring,
                                    handle->mem4txring,
                                    XLLDMA_BD_MINIMUM_ALIGNMENT,
                                    CYGNUM_DEVS_ETH_MICROBLAZE_DMA_BDRING_SIZE);

    if (status != XST_SUCCESS) {
        diag_printf("_XLlDma_TxBuffer_ - can't initialize_, %X\n", status);
    }


    XLlDma_BdClear(&BdTemplate);

    /*
     * Fill transmit ring buffer by empty templates
     */
    status = XLlDma_BdRingClone(tx_ring_ptr, &BdTemplate);

    if (status) {
        diag_printf("temac_init: TX Clone operation failure\n");
    }

    /*
     * Fill receive ring buffer by empty templates
     */
    status = XLlDma_BdRingClone(rx_ring_ptr, &BdTemplate);

    if (status) {
        diag_printf("temac_init: RX Clone operation failure\n");
    }

    /*
     *  Get count of free descriptors in receive ring buffer
     */
    FreeBdCount = XLlDma_BdRingGetFreeCnt(rx_ring_ptr);

    /*
     * Allocate receive ring buffer
     */
    status = XLlDma_BdRingAlloc(rx_ring_ptr, FreeBdCount, &bd_ptr);

    if (status) {
        diag_printf("temac_init: RX BdRingAlloc failure\n");
    }

    BdCurPtr = bd_ptr;

    /*
     * Assign addresses of data to descriptors of receive ring buffer
     */
    for (i = 0; i < FreeBdCount; i++) {
        XLlDma_BdSetBufAddr(BdCurPtr, &handle->temac_rx_bufs[i][0]);
        XLlDma_BdSetLength(BdCurPtr, TEMACDMA_MTU);
        XLlDma_BdSetId(BdCurPtr, &handle->temac_rx_bufs[i][0]);
        BdCurPtr = XLlDma_BdRingNext(rx_ring_ptr, BdCurPtr);
    }

    status = XLlDma_BdRingToHw(rx_ring_ptr, FreeBdCount, bd_ptr);

    if (status) {
        diag_printf("temac_init: RX RingToHw failure\n");
    }

    /*
     *  Get count of free descriptors in transmit ring buffer
     */
    FreeBdCount = XLlDma_BdRingGetFreeCnt(tx_ring_ptr);

    /*
     * Allocate transmit ring buffer
     */
    status = XLlDma_BdRingAlloc(tx_ring_ptr, FreeBdCount, &bd_ptr);
    BdCurPtr = bd_ptr;

    if (status) {
        diag_printf("temac_init: Tx BdRingAlloc failure\n");
    }

    /*
     * Assign addresses of data to descriptors of transmit ring buffer
     */
    for (i = 0; i < FreeBdCount; i++) {
        XLlDma_BdSetBufAddr(BdCurPtr, &handle->temac_tx_bufs[i][0]);
        BdCurPtr = XLlDma_BdRingNext(tx_ring_ptr, BdCurPtr);
    }

    /*
     * Free descriptors for transmitting operations
     */
    XLlDma_BdRingUnAlloc(tx_ring_ptr, FreeBdCount, bd_ptr);
}

/**
* Check interrupt status register of Temac DMA instance
*
* @param    handle is a pointer to Temac DMA structure
*
* @return   Interrupt status register of Temac DMA instance
*
* @note     None.
*
**/
cyg_uint32 temacdma_check(struct temacdma_handle * handle) {

	cyg_uint32 irq_status = 0x00;
	cyg_uint32 bd_ptr;

    XLlDma_BdRing* tx_ring_ptr = &(XLlDma_GetTxRing(&handle->temac_dma));
    XLlDma_BdRing* rx_ring_ptr = &(XLlDma_GetRxRing(&handle->temac_dma));

    /*
     * Get interrupt status of transmit channel
     */
    cyg_uint32 irq_status_tx = (XLlDma_BdRingGetIrq(tx_ring_ptr))& XLLDMA_IRQ_COALESCE_MASK;
    /*
     * Acknowledge interrupt of transmit channel
     */
    XLlDma_BdRingAckIrq(tx_ring_ptr, XLLDMA_IRQ_COALESCE_MASK);

    /*
     * Get interrupt status of receive channel
     */
    cyg_uint32 irq_status_rx = (XLlDma_BdRingGetIrq(rx_ring_ptr))& XLLDMA_IRQ_COALESCE_MASK;
    /*
     * Acknowledge interrupt of receive channel
     */
    XLlDma_BdRingAckIrq(rx_ring_ptr, XLLDMA_IRQ_COALESCE_MASK);

    if(irq_status_tx)
    {
        irq_status = irq_status | TEMACDMA_IRQ_TX;
    }

    if (irq_status_rx)
    {
        irq_status = irq_status | TEMACDMA_IRQ_RX;
    }

    return irq_status;
}

/**
* Start of Temac DMA instance. Enable interrupts of receive/transmit channels
*
* @param    handle is a pointer to Temac DMA structure
*
* @return   None
*
* @note     None.
*
**/

void temacdma_start(struct temacdma_handle * handle) {

    XLlDma * inst_ptr = &handle->temac_dma;
    XLlDma_BdRing *tx_ring_ptr = &(XLlDma_GetTxRing(inst_ptr));
    XLlDma_BdRing *rx_ring_ptr = &(XLlDma_GetRxRing(inst_ptr));
    cyg_uint32 status;

    /*
     * Start ring buffer of transmit channel
     */
    status = XLlDma_BdRingStart(tx_ring_ptr);

    if (status != XST_SUCCESS)
        diag_printf("temacdma_start: TX BdRingStart failure...\n");

    /*
     * Start ring buffer of receive channel
     */
    status = XLlDma_BdRingStart(rx_ring_ptr);

    if (status != XST_SUCCESS)
        diag_printf("temacdma_start: RX BdRingStart failure...\n");

    /*
     * Enable DMA TX Interrupt
     */
    XLlDma_BdRingIntEnable(tx_ring_ptr, XLLDMA_CR_IRQ_EN_MASK | XLLDMA_CR_IRQ_COALESCE_EN_MASK);//XLLDMA_CR_IRQ_ALL_EN_MASK
    /*
     * Enable DMA RX Interrupt
     */
    XLlDma_BdRingIntEnable(rx_ring_ptr, XLLDMA_CR_IRQ_EN_MASK | XLLDMA_CR_IRQ_COALESCE_EN_MASK);//XLLDMA_CR_IRQ_ALL_EN_MASK
}

/**
* Get received data.
*
* @param    handle is a pointer to Temac DMA structure
*
* @param    buf is a pointer of received data
*
* @param    length of received data array
*
* @return   None
*
* @note     None.
*
**/

void temacdma_rx_get(struct temacdma_handle * handle, cyg_uint8 * buf, cyg_uint16 * length) {
	cyg_uint32  status;
	cyg_uint32  free_bd_count;
	XLlDma_Bd * bd_ptr;
	cyg_uint8 * data_ptr;
	cyg_uint32 num_bd;

	XLlDma_BdRing* rx_ring_ptr = &(XLlDma_GetRxRing(&handle->temac_dma));

	HAL_DCACHE_INVALIDATE(rx_ring_ptr, sizeof(XLlDma_BdRing));


    num_bd = XLlDma_BdRingFromHw(rx_ring_ptr, 1, &bd_ptr);

    if (num_bd)
    {
        *length = (cyg_uint16)(XLlDma_BdRead(bd_ptr, XLLDMA_BD_USR4_OFFSET) & 0x3FFF);

        data_ptr = (cyg_uint8 *)XLlDma_BdGetBufAddr(bd_ptr);

        HAL_DCACHE_INVALIDATE(((cyg_uint32)data_ptr), ((cyg_uint32)*length));

        memcpy(buf, data_ptr, *length);

        status = XLlDma_BdRingFree(rx_ring_ptr, num_bd, bd_ptr);

        if (status)
            diag_printf("ERROR: temacdma_rx_get: BdRingFree failure\n");
    }
    else
    {
        *length = 0;
        diag_printf("temacdma_rx_get: from_hw returned 0.\n");
    }

    /*
     * getting maximum available descriptors
     * and put them to the input queue
     */
    free_bd_count = XLlDma_BdRingGetFreeCnt(rx_ring_ptr);

    if (free_bd_count)
    {
        status = XLlDma_BdRingAlloc(rx_ring_ptr, free_bd_count, &bd_ptr);

        if (status)
            diag_printf("temac_recv: BdRingAlloc failure\n");

        status = XLlDma_BdRingToHw(rx_ring_ptr, free_bd_count, bd_ptr);

        if (status)
            diag_printf("temac_recv: BdRingToHw failure\n");
    }
}

/**
* Check number of free buffer in transmit ring buffers
*
* @param    handle is a pointer to Temac DMA structure
*
* @return   Interrupt status register of Temac DMA instance
*
* @note     None.
*
**/

int temacdma_can_send(struct temacdma_handle * handle) {
    XLlDma_BdRing *tx_ring_ptr = &(XLlDma_GetTxRing(&handle->temac_dma));

    return XLlDma_BdRingGetFreeCnt(tx_ring_ptr);
}

/**
* This routine is called to send data to the hardware.
* All data in/out of the driver is specified via a "scatter-gather" list. This is just an array of address/length
* pairs which describe sections of data to move (in the order given by the array).
*
* @param    handle is a pointer to Temac DMA structure
*
* @param    sg_list in/out data of a driver
*
* @param    sg_len the number of elements in sg_list array
*
* @param    total_len total length  of a data to send
*
* @param    key unused in current version of eCos
*
* @return   None
*
* @note     None.
*
**/

void temacdma_send(struct temacdma_handle * handle,struct eth_drv_sg *sg_list,int sg_len, int total_len, unsigned long key) {
    cyg_uint32 status;
    XLlDma_BdRing *tx_ring_ptr = &(XLlDma_GetTxRing(&handle->temac_dma));
    cyg_uint32 i;
    XLlDma_Bd *bd_ptr = NULL;
    cyg_uint8 * buf_ptr = NULL;

    status = XLlDma_BdRingAlloc(tx_ring_ptr, 1, &bd_ptr);

    buf_ptr = (cyg_uint8 *)XLlDma_BdGetBufAddr(bd_ptr);

    if (status) {
        diag_printf("temac_send: BdRingAlloc failure\n");
    }

    for (i = 0; i < sg_len; i++) {
        memcpy((void *) buf_ptr, (void *) sg_list[i].buf, sg_list[i].len);
        buf_ptr += sg_list[i].len;
    }

    /*
     * Set up the BD using the information of the packet to transmit
     */

    XLlDma_BdSetLength(bd_ptr, total_len);// + 14

    XLlDma_BdSetStsCtrl(bd_ptr, XLLDMA_BD_STSCTRL_SOP_MASK | XLLDMA_BD_STSCTRL_EOP_MASK);

    XLlDma_BdSetId(bd_ptr, key);

    status = XLlDma_BdRingToHw(tx_ring_ptr, 1, bd_ptr);

    if (status)
    {
        diag_printf("temac_send: BdRingToHw failure\n");
    }
}

/**
* Get received data.
*
* @param    handle is a pointer to Temac DMA structure
*
* @param    key a returned value for _eth_drv_tx_done( ) function
*
* @return   The key value for _eth_drv_tx_done( ) function
*
* @note     None.
*
**/

cyg_uint32 temacdma_tx_finish(struct temacdma_handle * handle, cyg_uint32 * key) {
    XLlDma_BdRing *tx_ring_ptr;
    cyg_uint32 status;
    cyg_uint32 num_bd;
    cyg_uint32 sr = 0;
    XLlDma_Bd * bd_ptr = 0;

    HAL_DCACHE_INVALIDATE_ALL();

    tx_ring_ptr = &(XLlDma_GetTxRing(&handle->temac_dma));

    HAL_DCACHE_INVALIDATE(tx_ring_ptr, sizeof(XLlDma_BdRing));

    num_bd = XLlDma_BdRingFromHw(tx_ring_ptr, 1, &bd_ptr);

    //HAL_DCACHE_INVALIDATE(bd_ptr, sizeof(XLlDma_Bd));

    if (num_bd)
    {
        if (key)
            *key = XLlDma_BdGetId(bd_ptr);

        sr = XLlDma_BdGetStsCtrl(bd_ptr);

        status = XLlDma_BdRingFree(tx_ring_ptr, num_bd, bd_ptr);

        if (status)
                diag_printf("temacdma_tx_finish: BdRingFree failure\n");
    }
    else
    {
        *key = 0;
        diag_printf("temacdma_tx_finish: fromhw returned 0.\n");
    }

    return  !(sr & XLLDMA_BD_STSCTRL_EOP_MASK);
}
