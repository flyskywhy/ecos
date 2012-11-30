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
#include <src/xaxiethernet.h>
#include <src/xaxidma.h>
#include <src/xaxidma_bdring.h>

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

#define CYGPKG_DEVS_ETH_MICROBLAZE_TEMAC_DEBUG 0

#ifndef CONFIG_BOOL
#define CONFIG_BOOL 1
#endif

#ifndef MIN
#define MIN(x,y) ((x)<(y) ? (x) : (y))
#endif

#define AUTO_NEG_100 0x4000
#define AUTO_NEG_1000 0x8000

#define MAX_MULTICAST_ADDR   (1<<23) //Maximum number of multicast ethernet mac addresses

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

//cyg_uint32 McastAddressTable[(MAX_MULTICAST_ADDR>>3)/sizeof(cyg_uint32)] //1 MB for storing multicast address table

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

/*static void temac_add_mc_mac(cyg_uint8 * mc_addr);
static void temac_rem_mc_mac(cyg_uint8 * mc_addr);

static int AxiEthernet_ClearExtMulticast(void *AddressPtr);
static int AxiEthernet_AddExtMulticast(void *AddressPtr);
*/

//#ifdef CYGPKG_NET


/*****************************************************************************/
/**
* AxiEthernet_AddExtMulticast adds the multicast Ethernet address table in
* DDR. This table is mainly for software driver to perform the final address
* filtering.
*
* Once an Ethernet address is programmed, the Axi Ethernet channel will begin
* receiving data sent from that address. The way to prevent the
* Axi Ethernet channel from receiving messages from an Ethernet address in the
* DDR table is to clear it with AxiEthernet_ClearExtMulticast().
*
* @paramAddressPtr is a pointer to the 6-byte Ethernet address to set.
*
* @return - XST_SUCCESS,on successful completion
*- XST_INVALID_PARAM, if input MAC address is not between
*01:00:5E:00:00:00 and 01:00:5E:7F:FF:FF, as per RFC1112.
* @note
*
* This table is independent to BRAM table that is used for HW address
* filtering. It is user's responsiblility to maintain these tables.
*
* In the multicast table, hardware requires it to be 'indexed' with bit 22-8,
* 15 bits in total of mac address. This address filtering is done in hardware
*  and provisioned with XAxiEthernet_[Add|Clear]ExtMulticastSet() APIs.
*
* This routine consider all 2**23 possible multicast ethernet addresses to be
* 8Mx1 bit or 1M bytes memory area. All defined multicast addresses are from
* 01.00.5E.00.00.00 to 01.00.5E.7F.FF.FF
* The first 25 bit out of 48 bit are static, so they will not be part of
* calculation.
*
* In memory, software used every bit represents every defined MAC address,
* then we have this formula for table lookup.
* offset + (address >> 3) => memory location.
* address % 32            => bit location for multicast address
* Let's take 01:00:5E:00:01:FF(hex),
* memory location : 0x3F
* bit location    : 0x1F
 *****************************************************************************/
static void AxiEthernet_AddExtMulticast(void *AddressPtr)
{
    cyg_uint32 MaOffset;
    cyg_uint32 Loc;
    cyg_uint32 Bval;
    cyg_uint32 BvalNew;
    cyg_uint8  *Aptr = (cyg_uint8 *) AddressPtr;

/*
 * Verify if address is a good/valid multicast address, between
 * 01:00:5E:00:00:00 to 01:00:5E:7F:FF:FF per RFC1112.
 * This address is referenced to be index to BRAM table.
*/
//    if ((0x01 != Aptr[0]) || (0x00 != Aptr[1]) || (0x5e != Aptr[2]) || (0x0 != (Aptr[3] & 0x80))) 
//    {
//	return 1;
//    }

/* Program software Multicast table in RAM */
/* Calculate memory locations in software table */
    MaOffset  = Aptr[5];
    MaOffset |= Aptr[4] << 8;
    MaOffset |= Aptr[3] << 16;

    Loc  = ((MaOffset >> 5) << 2); /* it is not same as ">> 3" */

/* Bit value */
    BvalNew = 1 << (MaOffset % 32);

/* Program software Multicast table in RAM */
    Bval = XAxiEthernet_ReadReg((u32)&McastAddressTable, Loc);
    XAxiEthernet_WriteReg((u32)&McastAddressTable, Loc, Bval | BvalNew);

//    return 0;
}


/*****************************************************************************/
/**
* AxiEthernet_ClearExtMulticast clears the Ethernet address in multicast
* address table in DDR.
*
* @param AddressPtr is a pointer to the 6-byte Ethernet address to set.
*
* @return - XST_SUCCESS,on successful completion
*- XST_INVALID_PARAM, if input MAC address is not between
*01:00:5E:00:00:00 and 01:00:5E:7F:FF:FF, as per RFC1112.
*
* @note
*
* Please reference AxiEthernet_AddExtMulticast for multicast ethernet address
* index and bit value calculation.
*
*****************************************************************************/
static int AxiEthernet_ClearExtMulticast(void *AddressPtr)
{
    cyg_uint32 MaOffset;
    cyg_uint32 Loc;
    cyg_uint32 Bval;
    cyg_uint32 BvalNew;
    cyg_uint8 *Aptr = (cyg_uint8 *) AddressPtr;

/*
 * Verify if address is a good/valid multicast address, between
 * 01:00:5E:00:00:00 to 01:00:5E:7F:FF:FF per RFC1112.
 * This address is referenced to be index to BRAM table.
 */

//    if ((0x01 != Aptr[0]) || (0x00 != Aptr[1]) || (0x5e != Aptr[2]) || (0x0 != (Aptr[3] & 0x80))) 
//    {
//	return 1;
//    }

/* Program software Multicast table in RAM */
/* Calculate memory locations in software table */
    MaOffset  = Aptr[5];
    MaOffset |= Aptr[4] << 8;
    MaOffset |= Aptr[3] << 16;

    Loc  = ((MaOffset >> 5) << 2);

/* Bit value */
    BvalNew = 1 << (MaOffset % 32);

/* Program software Multicast table in RAM */
    Bval = XAxiEthernet_ReadReg((u32)&McastAddressTable, Loc);
    XAxiEthernet_WriteReg((u32)&McastAddressTable, Loc, Bval & ~BvalNew);

    return 0;
}



/*static void
temac_add_mc_mac(cyg_uint8 * mc_addr)
{
    AxiEthernet_AddExtMulticast(mc_addr);
}



static void
temac_rem_mc_mac(cyg_uint8 * mc_addr)
{
    AxiEthernet_ClearExtMulticast(mc_addr);
}
*/




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

//    temacdma_tx_int_disable(&g_dma_handle);
    
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

//    temacdma_rx_int_disable(&g_dma_handle);

    cyg_drv_interrupt_mask(qi->int_rx_vector);

    cyg_drv_interrupt_acknowledge(qi->int_rx_vector);

    // Run the DSR
    return (CYG_ISR_HANDLED|CYG_ISR_CALL_DSR);
}
//#endif

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

//#ifdef CYGPKG_NET
    cyg_drv_interrupt_unmask(qi->int_tx_vector);
    cyg_drv_interrupt_unmask(qi->int_rx_vector);
//#endif

    //temacdma_int_enable(&g_dma_handle);

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
    extern XAxiEthernet_Config XAxiEthernet_ConfigTable[0];
    XAxiEthernet_Config *MacCfgPtr;

    cyg_uint16 Phy_Id;
//    cyg_uint16 phy_status_autoneg;
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
#ifdef CYGPKG_DEVS_ETH_MICROBLAZE_TEMAC_DEBUG
        diag_printf("_TEMAC_ETH - Warning! Using default ESA for '%s'_\n",
                dtp->name);
#endif
    }

    /*
     * Initialize Xilinx driver  - device id 0
     */
     MacCfgPtr = XAxiEthernet_LookupConfig(12);
     
    // diag_printf("Addr = %X \n", XAxiEthernet_ConfigTable[0].BaseAddress);
    /*
     * Initialization of Temac DMA instance
     */
    temacdma_init(&g_dma_handle);

#ifdef CYGPKG_DEVS_ETH_MICROBLAZE_TEMAC_DEBUG
    diag_printf("DMA initialized ok\n") ;
#endif
    if (XAxiEthernet_CfgInitialize( &qi->dev, &XAxiEthernet_ConfigTable[0],
            XAxiEthernet_ConfigTable[0].BaseAddress) != XST_SUCCESS) {
        diag_printf("_TEMAC_ETH - can't initialize_\n");
        return false;
    }

    //////////!!!!!!promiscuous mode
    
    XAxiEthernet_DisableControlFrameLenCheck(&qi->dev);
    
    XAxiEthernet_SetOptions(&qi->dev, XAE_RCW1_VLAN_MASK | XAE_PROMISC_OPTION | XAE_MULTICAST_OPTION | XAE_RCW1_LT_DIS_MASK);//

    status = XAxiEthernet_ReadReg( &qi->dev, XAE_RCW1_OFFSET);

    XAxiEthernet_WriteReg((u32) &qi->dev, XAE_RCW1_OFFSET, status
            | XAE_RCW1_LT_DIS_MASK);

    /*
     * Set MAC address to Temac
     */
     
    status = XAxiEthernet_SetMacAddress( &qi->dev, qi->enaddr);
    if (status) {
        diag_printf("temac_init: SetMacAddress failure\n");
    }

    /*
     * Set operating speed
     */
    //XAxiEthernet_PhyRead((u32) &qi->dev, 0x00000007, 0x00000011, &phy_status_autoneg);

    switch (XAxiEthernet_GetPhysicalInterface(&qi->dev)) {
    case (XAE_PHY_TYPE_MII): {
#ifdef CYGPKG_DEVS_ETH_MICROBLAZE_TEMAC_DEBUG
        diag_printf("Speed = 100Mbps\n");
#endif
        XAxiEthernet_SetOperatingSpeed( &qi->dev, TEMAC_SPEED);
        break;
    }
    case (XAE_PHY_TYPE_GMII): {
        XAxiEthernet_SetOperatingSpeed( &qi->dev, TEMAC_SPEED_1G);
#ifdef CYGPKG_DEVS_ETH_MICROBLAZE_TEMAC_DEBUG
        diag_printf("Speed = 1Gbps\n");
#endif
        break;
    }
    default:
        diag_printf("10Mbps speed isn't supported");
    }

    status = XAxiEthernet_SetOptions(&qi->dev,
     XAE_RECEIVER_ENABLE_OPTION |
     XAE_TRANSMITTER_ENABLE_OPTION);
    if (status ) {
        diag_printf("Error setting options");
    }

    /*
     * Set PHY<-->MAC data clock
     */

    // sleeping 2 secs for initializing PHY unit
    cyg_thread_delay(200); // assuming tick is 10 ms

//#ifdef CYGPKG_NET
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
//#endif

    /*
     * Initialize upper level driver for ecos
     */
    (sc->funs->eth_drv->init)(sc, (unsigned char *) &qi->enaddr);

    XAxiEthernet_PhyRead( &qi->dev, 0x00000007, 0x00000002, &Phy_Id);
#ifdef CYGPKG_DEVS_ETH_MICROBLAZE_TEMAC_DEBUG
    diag_printf("_Marvell PHY Address_  %X\n", Phy_Id);

    diag_printf("_Initialization of TEMAC_ETH finished succesful!!!_\n");
#endif
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
    cyg_uint32 Status;
    struct temac_info *qi = (struct temac_info *) sc->driver_private;

    XAxiEthernet_Start(&qi->dev);

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

    XAxiEthernet_Stop(&qi->dev);

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

#ifdef CYGPKG_DEVS_ETH_MICROBLAZE_TEMAC_DEBUG
    diag_printf("Temac control started\n");
#endif
    switch (key) {
    case ETH_DRV_SET_MAC_ADDRESS:
    {
        XAxiEthernet_SetMacAddress(&qi->dev, data);
#ifdef CYGPKG_DEVS_ETH_MICROBLAZE_TEMAC_DEBUG
        diag_printf("Temac control SETMACADDR started\n");
#endif
        return 0;
    }
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
    cyg_uint32 i;

    sg_idx = sg_offset = rb_offset = 0;

    rb_len = qi->rx_buf_len;

    buf_ptr = qi->rx_buf;

#ifdef CYGPKG_DEVS_ETH_MICROBLAZE_TEMAC_DEBUG
    diag_printf("sg_len %d\n",sg_len);
#endif

    /*
     * For all scatter-gather buffers that we need to fill.
     */
    while (sg_idx < sg_len) {
        copy_cnt = MIN(sg_list[sg_idx].len - sg_offset, rb_len - rb_offset);

        if (copy_cnt && sg_list[sg_idx].buf)
            memcpy(((cyg_uint8 *) sg_list[sg_idx].buf) + sg_offset, buf_ptr,
                    copy_cnt);

#ifdef CYGPKG_DEVS_ETH_MICROBLAZE_TEMAC_DEBUG
        for (i = 0; i < copy_cnt; i++)
            diag_printf("%X ",*(buf_ptr + i));
            
        diag_printf("\n");
#endif

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

