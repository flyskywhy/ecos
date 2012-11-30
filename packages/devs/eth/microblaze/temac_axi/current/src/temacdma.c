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
#include <src/xaxiethernet.h>
#include <src/xaxidma.h>
#include <src/xaxidma_bdring.h>
#include "temacdma.h"
#include "src/xenv.h"

#include <cyg/hal/platform.h>	/* platform setting */

#define DMA_DEV_ID 13

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
    XAxiDma* inst_ptr = &handle->temac_dma;

    cyg_uint32 mask = XAXIDMA_IRQ_ALL_MASK;
    
    
    cyg_uint32 irq_status = 0x00;
    cyg_uint32 bd_ptr;

    XAxiDma_BdRing* tx_ring_ptr = XAxiDma_GetTxRing(&handle->temac_dma);

	XAxiDma_BdRingIntDisable(tx_ring_ptr, mask);
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
    XAxiDma* inst_ptr = &handle->temac_dma;

    cyg_uint32 mask = XAXIDMA_IRQ_ALL_MASK;

    XAxiDma_BdRing *rx_ring_ptr = XAxiDma_GetRxRing(inst_ptr);

	XAxiDma_BdRingIntDisable(rx_ring_ptr, mask);
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
    XAxiDma* inst_ptr = &handle->temac_dma;

    cyg_uint32 mask = XAXIDMA_IRQ_ALL_MASK;

	XAxiDma_BdRing* tx_ring_ptr = XAxiDma_GetTxRing(inst_ptr);
	XAxiDma_BdRing* rx_ring_ptr = XAxiDma_GetRxRing(inst_ptr);

	XAxiDma_BdRingIntEnable(tx_ring_ptr, mask);
	XAxiDma_BdRingIntEnable(rx_ring_ptr, mask);
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
    XAxiDma_Config *Config;
    XAxiDma_BdRing *rx_ring_ptr;
    XAxiDma_BdRing *tx_ring_ptr;
    XAxiDma_Bd BdTemplate;
    cyg_uint32 FreeBdCount;
    XAxiDma_Bd* bd_ptr, *BdCurPtr;
    cyg_uint32 status;
    cyg_uint32 i;
    cyg_uint32 bd_count;

    /*
     * Initialize Temac DMA instance
     */
     Config = XAxiDma_LookupConfig(DMA_DEV_ID);
     if (!Config) {
	diag_printf("No config found for %d\r\n", DMA_DEV_ID);
	return;
	}

/* Initialize DMA engine */
    XAxiDma_CfgInitialize(&handle->temac_dma, Config);

    rx_ring_ptr = XAxiDma_GetRxRing(&handle->temac_dma);
    tx_ring_ptr = XAxiDma_GetTxRing(&handle->temac_dma);

    /*
     * Disable Temac DMA interrupts
     */
    XAxiDma_BdRingIntDisable(rx_ring_ptr, XAXIDMA_IRQ_ALL_MASK);
    XAxiDma_BdRingIntDisable(tx_ring_ptr, XAXIDMA_IRQ_ALL_MASK);

    /*
     * Set the value to control register of Temac DMA instance for receive channel
     */
//    XAxiDma_WriteReg(handle->temac_dma.RegBase, XAXIDMA_RX_OFFSET + XAXIDMA_CR_OFFSET,
//            0x01011000);

    bd_count = XAxiDma_BdRingCntCalc(XAXIDMA_BD_MINIMUM_ALIGNMENT,sizeof(handle->mem4rxring));

    if (bd_count>CYGNUM_DEVS_ETH_MICROBLAZE_DMA_BDRING_SIZE)
    {
        bd_count = CYGNUM_DEVS_ETH_MICROBLAZE_DMA_BDRING_SIZE;
    }

    /*
     * Create ring buffer for receive channel of Temac DMA
     */
    status = XAxiDma_BdRingCreate(rx_ring_ptr,
                                 handle->mem4rxring,
                                 handle->mem4rxring,
                                 XAXIDMA_BD_MINIMUM_ALIGNMENT,
                                 bd_count);

    if (status != XST_SUCCESS) {
        diag_printf("_XAxiDma_RxBuffer_ - can't initialize_, %X\n", status);
    }

    /*
     * Set the value to control register of Temac DMA instance for transmit channel
     */
//    XAxiDma_WriteReg(handle->temac_dma.RegBase, XAXIDMA_TX_OFFSET + XAXIDMA_CR_OFFSET,
//                    0x01011000);

    /*
     * Create ring buffer for transmit channel of Temac DMA instance
     */
    status = XAxiDma_BdRingCreate(tx_ring_ptr,
                                    handle->mem4txring,
                                    handle->mem4txring,
                                    XAXIDMA_BD_MINIMUM_ALIGNMENT,
                                    CYGNUM_DEVS_ETH_MICROBLAZE_DMA_BDRING_SIZE);

    if (status != XST_SUCCESS) {
        diag_printf("_XAxiDma_TxBuffer_ - can't initialize_, %X\n", status);
    }


    XAxiDma_BdClear(&BdTemplate);

    /*
     * Fill transmit ring buffer by empty templates
     */
    status = XAxiDma_BdRingClone(tx_ring_ptr, &BdTemplate);

    if (status) {
        diag_printf("temac_init: TX Clone operation failure\n");
    }

    /*
     * Fill receive ring buffer by empty templates
     */
    status = XAxiDma_BdRingClone(rx_ring_ptr, &BdTemplate);

    if (status) {
        diag_printf("temac_init: RX Clone operation failure\n");
    }

    /*
     *  Get count of free descriptors in receive ring buffer
     */
    FreeBdCount = XAxiDma_BdRingGetFreeCnt(rx_ring_ptr);

    /*
     * Allocate receive ring buffer
     */
    status = XAxiDma_BdRingAlloc(rx_ring_ptr, FreeBdCount, &bd_ptr);

    if (status) {
        diag_printf("temac_init: RX BdRingAlloc failure\n");
    }

    BdCurPtr = bd_ptr;

    /*
     * Assign addresses of data to descriptors of receive ring buffer
     */
    for (i = 0; i < FreeBdCount; i++) {
        XAxiDma_BdSetBufAddr(BdCurPtr, &handle->temac_rx_bufs[i][0]);
        XAxiDma_BdSetLength(BdCurPtr, TEMACDMA_MTU);
        XAxiDma_BdSetId(BdCurPtr, &handle->temac_rx_bufs[i][0]);
        BdCurPtr = XAxiDma_BdRingNext(rx_ring_ptr, BdCurPtr);
    }

    status = XAxiDma_BdRingToHw(rx_ring_ptr, FreeBdCount, bd_ptr);

    if (status) {
        diag_printf("temac_init: RX RingToHw failure\n");
    }

    /*
     *  Get count of free descriptors in transmit ring buffer
     */
    FreeBdCount = XAxiDma_BdRingGetFreeCnt(tx_ring_ptr);

    /*
     * Allocate transmit ring buffer
     */
    status = XAxiDma_BdRingAlloc(tx_ring_ptr, FreeBdCount, &bd_ptr);
    BdCurPtr = bd_ptr;

    if (status) {
        diag_printf("temac_init: Tx BdRingAlloc failure\n");
    }

    /*
     * Assign addresses of data to descriptors of transmit ring buffer
     */
    for (i = 0; i < FreeBdCount; i++) {
        XAxiDma_BdSetBufAddr(BdCurPtr, &handle->temac_tx_bufs[i][0]);
        BdCurPtr = XAxiDma_BdRingNext(tx_ring_ptr, BdCurPtr);
    }

    /*
     * Free descriptors for transmitting operations
     */
    XAxiDma_BdRingUnAlloc(tx_ring_ptr, FreeBdCount, bd_ptr);
}



int GetBdRingUsedCnt(XAxiDma_BdRing * RingPtr)
{
XAxiDma_Bd *CurBdPtr;
int BdCount;
int BdPartialCount;
u32 BdSts;
u32 BdCr;

CurBdPtr = RingPtr->HwHead;
BdCount = 0;
BdPartialCount = 0;
BdSts = 0;
BdCr = 0;

/* If no BDs in work group, then there's nothing to search */
if (RingPtr->HwCnt == 0) 
{
    return 0;
}


/* Starting at HwHead, keep moving forward in the list until:
 *  - A BD is encountered with its completed bit clear in the status
 *    word which means hardware has not completed processing of that
 *    BD.
 *  - RingPtr->HwTail is reached
 *  - The number of requested BDs has been processed
 */

while (BdCount < RingPtr->AllCnt)
{
    /* Read the status */
    HAL_DCACHE_INVALIDATE(CurBdPtr,sizeof(XAxiDma_Bd));
    BdSts = XAxiDma_BdRead(CurBdPtr, XAXIDMA_BD_STS_OFFSET);
    BdCr = XAxiDma_BdRead(CurBdPtr, XAXIDMA_BD_CTRL_LEN_OFFSET);

    /* If the hardware still hasn't processed this BD then we are
     * done
     */
    if (!(BdSts & XAXIDMA_BD_STS_COMPLETE_MASK)) {
	break;
    }

    BdCount++;

/* Hardware has processed this BD so check the "last" bit. If
 * it is clear, then there are more BDs for the current packet.
 * Keep a count of these partial packet BDs.
 *
 * For tx BDs, EOF bit is in the control word
 * For rx BDs, EOF bit is in the status word
 */
    if (((!(RingPtr->IsRxChannel) &&
	(BdCr & XAXIDMA_BD_CTRL_TXEOF_MASK)) ||
	((RingPtr->IsRxChannel) && (BdSts &
	XAXIDMA_BD_STS_RXEOF_MASK)))) {

	    BdPartialCount = 0;
    }
    else {
        BdPartialCount++;
    }

    /* Reached the end of the work group */
    if (CurBdPtr == RingPtr->HwTail)
    {
      break;
    }

/* Move on to the next BD in work group */
     CurBdPtr = XAxiDma_BdRingNext(RingPtr, CurBdPtr);
   }

   /* Subtract off any partial packet BDs found */
   BdCount -= BdPartialCount;



return BdCount;
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
    cyg_uint32 num_bd_rx = 0;
    cyg_uint32 num_bd_tx = 0;

    XAxiDma_BdRing* tx_ring_ptr = XAxiDma_GetTxRing(&handle->temac_dma);
    XAxiDma_BdRing* rx_ring_ptr = XAxiDma_GetRxRing(&handle->temac_dma);

    u32 BdSts;

    /*
     * Get interrupt status of transmit channel
     */
    cyg_uint32 irq_status_tx = (XAxiDma_BdRingGetIrq(tx_ring_ptr));//& XAXIDMA_IRQ_COALESCE_MASK;
    /*
     * Acknowledge interrupt of transmit channel
     */
    XAxiDma_BdRingAckIrq(tx_ring_ptr, XAXIDMA_IRQ_ALL_MASK);//XAXIDMA_IRQ_COALESCE_MASK);

    
    num_bd_tx = GetBdRingUsedCnt(tx_ring_ptr);

    
    /*
     * Get interrupt status of receive channel
     */
    cyg_uint32 irq_status_rx = (XAxiDma_BdRingGetIrq(rx_ring_ptr));//& XAXIDMA_IRQ_COALESCE_MASK;
    /*
     * Acknowledge interrupt of receive channel
     */
    XAxiDma_BdRingAckIrq(rx_ring_ptr, XAXIDMA_IRQ_ALL_MASK);//XAXIDMA_IRQ_COALESCE_MASK);

    num_bd_rx = GetBdRingUsedCnt(rx_ring_ptr);
    
    if(irq_status_tx || num_bd_tx)
    {
        irq_status = irq_status | TEMACDMA_IRQ_TX;
    }

    if (irq_status_rx || num_bd_rx)
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

    XAxiDma * inst_ptr = &handle->temac_dma;
    XAxiDma_BdRing *tx_ring_ptr = XAxiDma_GetTxRing(inst_ptr);
    XAxiDma_BdRing *rx_ring_ptr = XAxiDma_GetRxRing(inst_ptr);
    cyg_uint32 status;

    /*
     * Start ring buffer of transmit channel
     */
    status = XAxiDma_BdRingStart(tx_ring_ptr);

    if (status != XST_SUCCESS)
        diag_printf("temacdma_start: TX BdRingStart failure...\n");

    /*
     * Start ring buffer of receive channel
     */
    status = XAxiDma_BdRingStart(rx_ring_ptr);

    if (status != XST_SUCCESS)
        diag_printf("temacdma_start: RX BdRingStart failure...\n");

    /*
     * Enable DMA TX Interrupt
     */
    XAxiDma_BdRingIntEnable(tx_ring_ptr, XAXIDMA_IRQ_ALL_MASK);//XAXIDMA_CR_IRQ_EN_MASK | XAXIDMA_CR_IRQ_COALESCE_EN_MASK);//XLLDMA_CR_IRQ_ALL_EN_MASK
    /*
     * Enable DMA RX Interrupt
     */
    XAxiDma_BdRingIntEnable(rx_ring_ptr, XAXIDMA_IRQ_ALL_MASK);//XAXIDMA_CR_IRQ_EN_MASK | XAXIDMA_CR_IRQ_COALESCE_EN_MASK);//XLLDMA_CR_IRQ_ALL_EN_MASK
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
	XAxiDma_Bd * bd_ptr;
	XAxiDma_Bd * bd_cur_ptr;
	cyg_uint8 * data_ptr;
	cyg_uint32 num_bd;
	cyg_uint32 i;
	char MulticastAdd[6];
	cyg_uint32 RxStatusControlWord;
	cyg_uint32 Reg;
	cyg_uint16 deb_len;

	XAxiDma_BdRing* rx_ring_ptr = XAxiDma_GetRxRing(&handle->temac_dma);

	HAL_DCACHE_INVALIDATE(rx_ring_ptr, sizeof(XAxiDma_BdRing));


    num_bd = XAxiDma_BdRingFromHw(rx_ring_ptr, 1, &bd_ptr);//1

//    diag_printf("%d\n",num_bd);

    if (num_bd)
    //bd_cur_ptr = bd_ptr;
//    for(i = 0; i < num_bd; i++)
    {
        deb_len = XAxiDma_BdRead(bd_ptr, XAXIDMA_BD_USR4_OFFSET);
        *length = (cyg_uint16)(XAxiDma_BdRead(bd_ptr, XAXIDMA_BD_USR4_OFFSET) & 0xFFFF);

	diag_printf("l = %d\n",deb_len);

        data_ptr = (cyg_uint8 *)XAxiDma_BdGetBufAddr(bd_ptr);

        HAL_DCACHE_INVALIDATE(((cyg_uint32)data_ptr), ((cyg_uint32)*length));


//	RxStatusControlWord =XAxiDma_BdRead(bd_ptr,XAXIDMA_BD_USR2_OFFSET);
	if(RxStatusControlWord & XAE_BD_RX_USR2_IP_MCAST_MASK) 
	{
       /*
* The multicast MAC address is stored in status words
* 0 and 1 in the AXI4-Stream.
*/

	    Reg = XAxiDma_BdRead(bd_ptr, XAXIDMA_BD_USR0_OFFSET);
	    MulticastAdd[5] = ((Reg >> 8)  & 0xFF);
	    MulticastAdd[4] = (Reg & 0xFF);

	    Reg = XAxiDma_BdRead(bd_ptr, XAXIDMA_BD_USR1_OFFSET);
	    MulticastAdd[3] = ((Reg >> 24) & 0xFF);
	    MulticastAdd[2] = ((Reg >> 16) & 0xFF);
	    MulticastAdd[1] = ((Reg >> 8)  & 0xFF);
	    MulticastAdd[0] = (Reg & 0xFF);

	    if (!AxiDma_GetExtMulticast(MulticastAdd)) 
	    {
		diag_printf("Multicast address mismatch\n");
		return 1;
	    }
	}
	else 
	{
	    diag_printf("Not a multicast frame\n");
	    return 1;
	}


        memcpy(buf, data_ptr, *length);
        
//        bd_cur_ptr = XAxiDma_BdRingNext(rx_ring_ptr,bd_cur_ptr);

	status = XAxiDma_BdRingFree(rx_ring_ptr, num_bd, bd_ptr);

        if (status)
            diag_printf("ERROR: temacdma_rx_get: BdRingFree failure\n");
    }
/*    else
    {
        *length = 0;
        diag_printf("temacdma_rx_get: from_hw returned 0.\n");
    }*/


/*    status = XAxiDma_BdRingFree(rx_ring_ptr, num_bd, bd_ptr);

    if (status)
        diag_printf("ERROR: temacdma_rx_get: BdRingFree failure\n");
*/                    

    /*
     * getting maximum available descriptors
     * and put them to the input queue
     */
    free_bd_count = XAxiDma_BdRingGetFreeCnt(rx_ring_ptr);

    if (free_bd_count)
    {
        status = XAxiDma_BdRingAlloc(rx_ring_ptr, free_bd_count, &bd_ptr);

        if (status)
            diag_printf("temac_recv: BdRingAlloc failure\n");

        status = XAxiDma_BdRingToHw(rx_ring_ptr, free_bd_count, bd_ptr);

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
    XAxiDma_BdRing *tx_ring_ptr = XAxiDma_GetTxRing(&handle->temac_dma);

    return XAxiDma_BdRingGetFreeCnt(tx_ring_ptr);
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
    XAxiDma_BdRing *tx_ring_ptr = XAxiDma_GetTxRing(&handle->temac_dma);
    cyg_uint32 i;
    XAxiDma_Bd *bd_ptr = NULL;
    cyg_uint8 * buf_ptr = NULL;

    status = XAxiDma_BdRingAlloc(tx_ring_ptr, 1, &bd_ptr);

    buf_ptr = (cyg_uint8 *)XAxiDma_BdGetBufAddr(bd_ptr);

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

    XAxiDma_BdSetLength(bd_ptr, total_len);// + 14

    XAxiDma_BdSetCtrl(bd_ptr, XAXIDMA_BD_CTRL_TXSOF_MASK | XAXIDMA_BD_CTRL_TXEOF_MASK);

    XAxiDma_BdSetId(bd_ptr, key);

    status = XAxiDma_BdRingToHw(tx_ring_ptr, 1, bd_ptr);

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
    XAxiDma_BdRing *tx_ring_ptr;
    cyg_uint32 status;
    cyg_uint32 num_bd;
    cyg_uint32 sr = 0;
    XAxiDma_Bd * bd_ptr = 0;

    HAL_DCACHE_INVALIDATE_ALL();

    tx_ring_ptr = XAxiDma_GetTxRing(&handle->temac_dma);

    HAL_DCACHE_INVALIDATE(tx_ring_ptr, sizeof(XAxiDma_BdRing));

    num_bd = XAxiDma_BdRingFromHw(tx_ring_ptr, 1, &bd_ptr);

    //HAL_DCACHE_INVALIDATE(bd_ptr, sizeof(XLlDma_Bd));

    if (num_bd)
    {
        if (key)
            *key = XAxiDma_BdGetId(bd_ptr);

        sr = XAxiDma_BdGetCtrl(bd_ptr);

        status = XAxiDma_BdRingFree(tx_ring_ptr, num_bd, bd_ptr);

        if (status)
                diag_printf("temacdma_tx_finish: BdRingFree failure\n");
    }
    else
    {
        *key = 0;
        diag_printf("temacdma_tx_finish: fromhw returned 0.\n");
    }

    return  !(sr & XAXIDMA_BD_CTRL_TXEOF_MASK);
}
