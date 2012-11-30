//==========================================================================
//
//
//      Xilinx SP605 ethernet support
//
//==========================================================================
// ####ECOSGPLCOPYRIGHTBEGIN####                                            
// -------------------------------------------                              
// This file is part of eCos, the Embedded Configurable Operating System.   
// Copyright (C) 1998, 1999, 2000, 2001, 2002, 2003, 2005 Free Software Foundation, Inc.
//
// eCos is free software; you can redistribute it and/or modify it under    
// the terms of the GNU General Public License as published by the Free     
// Software Foundation; either version 2 or (at your option) any later      
// version.                                                                 
//
// eCos is distributed in the hope that it will be useful, but WITHOUT      
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or    
// FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License    
// for more details.                                                        
//
// You should have received a copy of the GNU General Public License        
// along with eCos; if not, write to the Free Software Foundation, Inc.,    
// 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.            
//
// As a special exception, if other files instantiate templates or use      
// macros or inline functions from this file, or you compile this file      
// and link it with other works to produce a work based on this file,       
// this file does not by itself cause the resulting work to be covered by   
// the GNU General Public License. However the source code for this file    
// must still be made available in accordance with section (3) of the GNU   
// General Public License v2.                                               
//
// This exception does not invalidate any other reasons why a work based    
// on this file might be covered by the GNU General Public License.         
// -------------------------------------------                              
// ####ECOSGPLCOPYRIGHTEND####                                              
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
 *@file adapter.c
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

#include <cyg/io/eth/netdev.h>
#include <cyg/io/eth/eth_drv.h>

#include "temacdma.h"

#ifdef CYGPKG_NET
#include <pkgconf/net.h>
#endif

#include <src/xparameters.h>

#include "adapter.h"
#include <src/xlltemac.h>
#include <src/xlldma.h>
#include <src/xlldma_bdring.h>

#ifdef CYGPKG_REDBOOT
#include <pkgconf/redboot.h>
#ifdef CYGSEM_REDBOOT_FLASH_CONFIG
#include <redboot.h>
#include <flash_config.h>
RedBoot_config_option("Network hardware address [MAC]",
        eth0_esa,
        ALWAYS_ENABLED, true,
        CONFIG_ESA, &temac0_info.enaddr
);
#endif
#endif

#include <cyg/hal/platform.h>	/* platform setting */

// CONFIG_ESA and CONFIG_BOOL are defined in redboot/include/flash_config.h
#ifndef CONFIG_ESA
#define CONFIG_ESA 6      // ethernet address length ...
#endif

#define TEMAC_SPEED    100 /* 100Mb/s for Mii */
#define TEMAC_SPEED_1G 1000    /* 1000Mb/s for GMii */

#ifndef CONFIG_BOOL
#define CONFIG_BOOL 1
#endif

#ifndef MIN
#define MIN(x,y) ((x)<(y) ? (x) : (y))
#endif

#define AUTO_NEG_100 0x4000
#define AUTO_NEG_1000 0x8000

extern microblaze_intc_t *intc;

//Driver interface callbacks
#define _eth_drv_init(sc,mac) \
 (sc->funs->eth_drv->init)(sc,(unsigned char *)mac)
#define _eth_drv_tx_done(sc,key,status) \
 (sc->funs->eth_drv->tx_done)(sc,key,status)
#define _eth_drv_recv(sc,len) \
 (sc->funs->eth_drv->recv)(sc,len)

static struct temac_info temac0_info = { MON_TXDMA_INTR, // Interrupt vector for transmit channel
        MON_RXDMA_INTR, // Interrupt vector for recieve channel
        "eth0_esa", { 0x08, 0x00, 0x3E, 0x28, 0x7A, 0xBA }, // Default ESA
        };
static struct temacdma_handle g_dma_handle;

ETH_DRV_SC(temac0_sc,
        &temac0_info, // Driver specific data
        "eth0", // Name for this interface
        temac_start,
        temac_stop,
        temac_control,
        temac_can_send,
        temac_send,
        temac_recv,
        temac_deliver,
        temac_int,
        temac_int_vector);

NETDEVTAB_ENTRY(sp605_netdev,
        "temac",
        temac_init,
        &temac0_sc);

static void temac_int(struct eth_drv_sc *data);
static void temac_RxEvent(void *sc);
static void temac_TxEvent(void *sc);

#ifdef CYGPKG_NET

/**
 * Interrupt routine for transmission of a packet
 *
 * @param    vector is interrupt vector for Temac DMA transmit channel
 *
 * @param    data is a pointer of driver structure
 *
 * @param    HAL_SavedRegister is a pointer of a structure that contains the state of processor
 *
 * @return   Call of DSR function
 *
 * @note     None
 *
 **/

static int
temac_dma_tx_isr(cyg_vector_t vector, cyg_addrword_t data, HAL_SavedRegisters *regs)
{
    struct eth_drv_sc *sc = (struct eth_drv_sc *)data;
    struct temac_info *qi = (struct temac_info *)sc->driver_private;

    temacdma_tx_int_disable(&g_dma_handle);
    cyg_drv_interrupt_mask(qi->int_tx_vector);

    cyg_drv_interrupt_acknowledge(qi->int_tx_vector);
    // Run the DSR
    return (CYG_ISR_HANDLED|CYG_ISR_CALL_DSR);
}

/**
 * Interrupt routine for receiving a packet
 *
 * @param    vector is interrupt vector for Temac DMA receive channel
 *
 * @param    data is a pointer of driver structure
 *
 * @param    HAL_SavedRegister is a pointer of a structure that contains the state of processor
 *
 * @return   Call of DSR function
 *
 * @note     None
 *
 **/
static int
temac_dma_rx_isr(cyg_vector_t vector, cyg_addrword_t data, HAL_SavedRegisters *regs)
{
    struct eth_drv_sc *sc = (struct eth_drv_sc *)data;
    struct temac_info *qi = (struct temac_info *)sc->driver_private;

    temacdma_rx_int_disable(&g_dma_handle);

    cyg_drv_interrupt_mask(qi->int_rx_vector);

    cyg_drv_interrupt_acknowledge(qi->int_rx_vector);

    // Run the DSR
    return (CYG_ISR_HANDLED|CYG_ISR_CALL_DSR);
}
#endif

/**
 * Deliver function (ex-DSR) handles the ethernet [logical] processing. It checks a DMA interrupt status
 * register (receive or transmit event occured) and calls the same function
 *
 * @param    sc is a pointer of driver structure
 *
 * @return   None
 *
 * @note     None
 *
 **/

static void temac_deliver(struct eth_drv_sc * sc) {
    cyg_uint32 irq_status;
    struct temac_info *qi = (struct temac_info *) sc->driver_private;

    do {
        irq_status = temacdma_check(&g_dma_handle);

        if (irq_status & TEMACDMA_IRQ_TX)
            temac_TxEvent(sc);

        if (irq_status & TEMACDMA_IRQ_RX)
            temac_RxEvent(sc);

    } while (irq_status);

#ifdef CYGPKG_NET
    cyg_drv_interrupt_unmask(qi->int_tx_vector);
    cyg_drv_interrupt_unmask(qi->int_rx_vector);
#endif

    temacdma_int_enable(&g_dma_handle);

}

/**
 * Initializes a specific Temac instance, initializes the transmit and receive ring buffers.
 * Set MAC address.
 *
 * @param    dtp is a pointer of standard eCos structure that contains pointer to the driver instance
 *
 * @return   None
 *
 * @note     None.
 *
 **/

static bool temac_init(struct cyg_netdevtab_entry *dtp) {
    struct eth_drv_sc *sc = (struct eth_drv_sc *) dtp->device_instance;
    struct temac_info *qi = (struct temac_info *) sc->driver_private;
    extern XLlTemac_Config XLlTemac_ConfigTable[0];

    cyg_uint16 Phy_Id;
    cyg_uint16 phy_status_autoneg;
    cyg_uint32 status;
    unsigned char _enaddr[6];
    bool esa_ok;

#if defined(CYGPKG_REDBOOT) && defined(CYGSEM_REDBOOT_FLASH_CONFIG)
    esa_ok = flash_get_config(qi->esa_key, _enaddr, CONFIG_ESA);
#else
    esa_ok = CYGACC_CALL_IF_FLASH_CFG_OP(CYGNUM_CALL_IF_FLASH_CFG_GET,
            qi->esa_key, _enaddr, CONFIG_ESA);
#endif
    if (esa_ok) {
        memcpy(qi->enaddr, _enaddr, sizeof(qi->enaddr));
    } else {
        /* No 'flash config' data available - use default */
        diag_printf("_TEMAC_ETH - Warning! Using default ESA for '%s'_\n",
                dtp->name);
    }

    /*
     * Initialize Xilinx driver  - device id 0
     */
    if (XLlTemac_CfgInitialize(&qi->dev, &XLlTemac_ConfigTable[0],
            XLlTemac_ConfigTable[0].BaseAddress) != XST_SUCCESS) {
        diag_printf("_TEMAC_ETH - can't initialize_\n");
        return false;
    }

    status = XLlTemac_ReadReg((u32) & qi->dev, XTE_RCW1_OFFSET);
    XLlTemac_WriteReg((u32) & qi->dev, XTE_RCW1_OFFSET, status
            | XTE_RCW1_LT_DIS_MASK);

    /*
     * Set MAC address to Temac
     */
    status = XLlTemac_SetMacAddress(&qi->dev, qi->enaddr);
    if (status) {
        diag_printf("temac_init: SetMacAddress failure\n");
    }

    /*
     * Initialization of Temac DMA instance
     */
    temacdma_init(&g_dma_handle);

    /*
     * Set operating speed
     */
    XLlTemac_PhyRead(&qi->dev, 0x00000007, 0x00000011, &phy_status_autoneg);

    switch (phy_status_autoneg & 0xC000) {
    case (AUTO_NEG_100): {
        diag_printf("Speed = 100Mbps\n");
        XLlTemac_SetOperatingSpeed(&qi->dev, TEMAC_SPEED);
        break;
    }
    case (AUTO_NEG_1000): {
        XLlTemac_SetOperatingSpeed(&qi->dev, TEMAC_SPEED_1G);
        diag_printf("Speed = 1Gbps\n");
        break;
    }
    default:
        diag_printf("10Mbps speed isn't supported");
    }

    /*
     * Set PHY<-->MAC data clock
     */

    // sleeping 2 secs for initializing PHY unit
    cyg_thread_delay(200); // assuming tick is 10 ms

#ifdef CYGPKG_NET
    /*
     * Set up to handle interrupts
     */
    cyg_drv_interrupt_create(qi->int_tx_vector,
            0, // Highest //CYGARC_SIU_PRIORITY_HIGH,
            (cyg_addrword_t)sc, //  Data passed to ISR
            (cyg_ISR_t *)temac_dma_tx_isr,
            (cyg_DSR_t *)eth_drv_dsr,
            &qi->temac_tx_interrupt_handle,
            &qi->temac_tx_interrupt);
    cyg_drv_interrupt_attach(qi->temac_tx_interrupt_handle);
    cyg_drv_interrupt_acknowledge(qi->int_tx_vector);
    cyg_drv_interrupt_unmask(qi->int_tx_vector);

    cyg_drv_interrupt_create(qi->int_rx_vector,
            0, // Highest //CYGARC_SIU_PRIORITY_HIGH,
            (cyg_addrword_t)sc, //  Data passed to ISR
            (cyg_ISR_t *)temac_dma_rx_isr,
            (cyg_DSR_t *)eth_drv_dsr,
            &qi->temac_rx_interrupt_handle,
            &qi->temac_rx_interrupt);
    cyg_drv_interrupt_attach(qi->temac_rx_interrupt_handle);
    cyg_drv_interrupt_acknowledge(qi->int_rx_vector);
    cyg_drv_interrupt_unmask(qi->int_rx_vector);
#endif

    /*
     * Initialize upper level driver for ecos
     */
    (sc->funs->eth_drv->init)(sc, (unsigned char *) &qi->enaddr);

    XLlTemac_PhyRead(&qi->dev, 0x00000007, 0x00000002, &Phy_Id);
    diag_printf("_Marvell PHY Address_  %X\n", Phy_Id);

    diag_printf("_Initialization of TEMAC_ETH finished succesful!!!_\n");
    return true;
}

/**
 * This function is called to "start up" the interface.  It may be called
 * multiple times, even when the hardware is already running.  It will be
 * called whenever something "hardware oriented" changes and should leave
 * the hardware ready to send/receive packets.
 *
 * @param    sc is a pointer of driver structure
 *
 * @param    enaddr in current version of eCos is unused parameter
 *
 * @param    flags in current version of eCos is unused parameter
 *
 * @return   None
 *
 * @note     None.
 *
 **/

static void temac_start(struct eth_drv_sc *sc, unsigned char *enaddr, int flags) {
    struct temac_info *qi = (struct temac_info *) sc->driver_private;

    XLlTemac_Start(&qi->dev);

    temacdma_start(&g_dma_handle);
}

/**
 * This function is the inverse of "start". It shuts down the hardware, disables the receiver, and keeps it from
 * interacting with the physical network.
 *
 * @param    sc is a pointer of driver structure
 *
 * @return   None
 *
 * @note     None.
 *
 **/

static void temac_stop(struct eth_drv_sc *sc) {
    struct temac_info *qi = (struct temac_info *) sc->driver_private;

    temacdma_rx_int_disable(&g_dma_handle);
    temacdma_tx_int_disable(&g_dma_handle);

    XLlTemac_Stop(&qi->dev);

}

/**
 * This function is called for low level "control" operations on the interface.
 *
 * @param    sc is a pointer of driver structure
 *
 * @param    key parameter selects the operation
 *
 * @param    data describes, as required, some data for the operation in question
 *
 * @param    length describes, as required, some data for the operation in question
 *
 * @return   Status of executing an operation
 *
 * @note     None.
 *
 **/

static int temac_control(struct eth_drv_sc *sc, unsigned long key, void *data,
        int length) {
    struct temac_info *qi = (struct temac_info *) sc->driver_private;
#ifdef CYGDBG_IO_ETH_DRIVERS_DEBUG
    diag_printf("Temac control started\n");
#endif
    switch (key) {
    case ETH_DRV_SET_MAC_ADDRESS:
        XLlTemac_SetMacAddress(&qi->dev, data);
        diag_printf("Temac control SETMACADDR started\n");
        return 0;
        break;
    default:
#ifdef CYGDBG_IO_ETH_DRIVERS_DEBUG
        diag_printf("Temac control undefined key\n");
#endif
        return 1;
        break;
    }
}

/**
 * This function is called to see if another packet can be sent.
 *
 * @param    sc is a pointer of driver structure
 *
 * @return   return the number of packets which can be handled.
 *           Zero returns if the interface is busy and can not send any more.
 *
 * @note     None.
 *
 **/

static int temac_can_send(struct eth_drv_sc *sc) {

    return temacdma_can_send(&g_dma_handle);
}

/**
 * This routine is called to send data to the hardware.
 * All data in/out of the driver is specified via a "scatter-gather" list. This is just an array of address/length
 * pairs which describe sections of data to move (in the order given by the array).
 *
 * @param    sc is a pointer of driver structure
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

static void temac_send(struct eth_drv_sc *sc, struct eth_drv_sg *sg_list,
        int sg_len, int total_len, unsigned long key) {

    if (total_len > TEMACDMA_MTU) {
        diag_printf(
                "ERROR: temac_send: total_len > maximum transfer unit; reducing length\n");
        total_len = TEMACDMA_MTU;
    }

    temacdma_send(&g_dma_handle, sg_list, sg_len, total_len, key);
}

/**
 * This function is called when a frame has been sent
 *
 * @param    _cb is a pointer of driver structure
 *
 * @return   None
 *
 * @note     None.
 *
 **/
static void temac_TxEvent(void *_cb) {
    cyg_uint32 sr;
    cyg_uint32 key;

    struct eth_drv_sc *sc = (struct eth_drv_sc *) _cb;

    sr = temacdma_tx_finish(&g_dma_handle, &key);

    if (!sr) {
        _eth_drv_tx_done(sc, key, 0);
    }
}

/**
 * This function is called when a packet has been received.  It's job is
 * to prepare to unload the packet from the hardware.  Once the length of
 * the packet is known, the upper layer of the driver can be told.
 *
 * @param    _cb is a pointer of driver structure
 *
 * @return   None
 *
 * @note     None.
 *
 **/

static void temac_RxEvent(void *_cb) {
    struct eth_drv_sc *sc = (struct eth_drv_sc *) _cb;
    struct temac_info *qi = (struct temac_info *) sc->driver_private;

    temacdma_rx_get(&g_dma_handle, qi->rx_buf, &qi->rx_buf_len);

    _eth_drv_recv(sc, qi->rx_buf_len);
}

/**
 * This function is called as a result of the "eth_drv_recv()" call above.
 * It's job is to actually fetch data for a packet from the hardware once
 * memory buffers have been allocated for the packet.
 *
 * @param    sc is a pointer of driver structure
 *
 * @param    sg_list in/out data of a driver
 *
 * @param    sg_len the number of elements in sg_list array
 *
 * @return   None
 *
 * @note     The buffers may come in pieces, using a scatter-gather list.
 *
 **/

static void temac_recv(struct eth_drv_sc *sc, struct eth_drv_sg *sg_list,
        int sg_len) {
    struct temac_info *qi = (struct temac_info *) sc->driver_private;
    cyg_uint32 rb_len, rb_offset;
    cyg_uint32 sg_idx, sg_offset;
    cyg_uint32 copy_cnt;
    cyg_uint8 * buf_ptr;

    sg_idx = sg_offset = rb_offset = 0;

    rb_len = qi->rx_buf_len;

    buf_ptr = qi->rx_buf;

    /*
     * For all scatter-gather buffers that we need to fill.
     */
    while (sg_idx < sg_len) {
        copy_cnt = MIN(sg_list[sg_idx].len - sg_offset, rb_len - rb_offset);

        if (copy_cnt && sg_list[sg_idx].buf)
            memcpy(((cyg_uint8 *) sg_list[sg_idx].buf) + sg_offset, buf_ptr,
                    copy_cnt);

        /*
         * incrementing pointer to  buffer-receiver
         */
        buf_ptr += copy_cnt;

        /*
         * If rx buffer is completely drained, move to next
         */
        rb_offset += copy_cnt;
        if (rb_len <= rb_offset)
            break;

        /*
         * If sg buffer is completely filled, move to next
         */

        sg_offset += copy_cnt;

        if (sg_offset >= sg_list[sg_idx].len) {
            sg_idx++;
            sg_offset = 0;
        }
    }
}

/**
 * Interrupt processing
 *
 * @param    sc is a pointer of driver structure
 *
 * @return   None
 *
 * @note     None
 *
 **/
static void temac_int(struct eth_drv_sc *sc) {
    temac_deliver(sc);

}

/**
 * Interrupt vector
 *
 * @param    sc is a pointer of driver structure
 *
 * @return   The interrupt vector of receive event
 *
 * @note     None
 *
 **/
static int temac_int_vector(struct eth_drv_sc *sc) {
    struct temac_info *qi = (struct temac_info *) sc->driver_private;
    return (qi->int_rx_vector);
}

