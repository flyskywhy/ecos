//==========================================================================
//
//      dev/if_fec.c
//
//      Fast ethernet device driver for PowerPC MPC8xxT boards
//
//==========================================================================
//####ECOSGPLCOPYRIGHTBEGIN####
// -------------------------------------------
// This file is part of eCos, the Embedded Configurable Operating System.
// Copyright (C) 1998, 1999, 2000, 2001, 2002 Red Hat, Inc.
// Copyright (C) 2002, 2003 Gary Thomas
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
// Alternative licenses for eCos may be arranged by contacting Red Hat, Inc.
// at http://sources.redhat.com/ecos/ecos-license/
// -------------------------------------------
//####ECOSGPLCOPYRIGHTEND####
//==========================================================================
//#####DESCRIPTIONBEGIN####
//
// Author(s):    gthomas
// Contributors: gthomas
// Date:         2001-01-21
// Purpose:
// Description:  hardware driver for MPC8xxT FEC
//
//
//####DESCRIPTIONEND####
//
//==========================================================================

// Ethernet device driver for MPC8xx FEC

#include <pkgconf/system.h>
#include <pkgconf/devs_eth_powerpc_mpc8xx.h>
#include <pkgconf/io_eth_drivers.h>

#ifdef CYGPKG_NET
#include <pkgconf/net.h>
#endif

#include <cyg/infra/cyg_type.h>
#include <cyg/infra/diag.h>

#include <cyg/hal/hal_arch.h>
#include <cyg/hal/hal_cache.h>
#include <cyg/hal/hal_intr.h>
#include <cyg/hal/drv_api.h>
#include <cyg/hal/hal_if.h>
#include <cyg/hal/ppc_regs.h>

#include <cyg/io/eth/netdev.h>
#include <cyg/io/eth/eth_drv.h>

#include "if_fec.h"

#ifndef CYGSEM_DEVS_ETH_POWERPC_MPC8xx_RESET_PHY
# error "RESET_PHY is required for this driver.  It doesn't make sense not to do this anyway."
#endif

int if_fec_num_interrupts;
int if_fec_num_rx_interrupts;
int if_fec_num_rx_drops;

int if_fec_num_rx_error_lg;
int if_fec_num_rx_error_no;
int if_fec_num_rx_error_sh;
int if_fec_num_rx_error_cr;
int if_fec_num_rx_error_ov;
int if_fec_num_rx_error_tr;

int if_fec_num_tx_error_lc;
int if_fec_num_tx_error_rl;
int if_fec_num_tx_error_ur;

void    (*fec_eth_rx_callback_func)(void) = NULL;
void    (*fec_eth_tx_callback_func)(void) = NULL;

externC bool    fec_eth_set_state(unsigned short mode);

/*FUNCTION:------------------------------------------------------
 *  NAME
 *      fec_eth_register_rx_callback
 *  DESCRIPTION
 *
 *  PARAMETERS
 *
 *  RETURNS
 *
 *---------------------------------------------------------------
 */
externC void
fec_eth_register_rx_callback(void (*rx_callback_func)(void))
{
    fec_eth_rx_callback_func = rx_callback_func;
} /* fec_eth_register_rx_callback */

/*FUNCTION:------------------------------------------------------
 *  NAME
 *      fec_eth_register_tx_callback
 *  DESCRIPTION
 *
 *  PARAMETERS
 *
 *  RETURNS
 *
 *---------------------------------------------------------------
 */
externC void
fec_eth_register_tx_callback(void (*tx_callback_func)(void))
{
    fec_eth_tx_callback_func = tx_callback_func;
} /* fec_eth_register_tx_callback */

// Align buffers on a cache boundary
#define RxBUFSIZE       (CYGNUM_DEVS_ETH_POWERPC_MPC8xx_RxNUM * CYGNUM_DEVS_ETH_POWERPC_MPC8xx_BUFSIZE)
#define TxBUFSIZE       (CYGNUM_DEVS_ETH_POWERPC_MPC8xx_TxNUM * CYGNUM_DEVS_ETH_POWERPC_MPC8xx_BUFSIZE)

unsigned char fec_eth_rxbufs[RxBUFSIZE] __attribute__((aligned(HAL_DCACHE_LINE_SIZE)));
unsigned int fec_eth_rxbuf_size = RxBUFSIZE;
unsigned char fec_eth_txbufs[TxBUFSIZE] __attribute__((aligned(HAL_DCACHE_LINE_SIZE)));
unsigned int fec_eth_txbuf_size = TxBUFSIZE;

static struct fec_eth_info fec_eth0_info;

static unsigned char _default_enaddr[] = { CYGDAT_DEVS_ETH_POWERPC_MPC8xx_DEFAULT_MAC };
static unsigned char enaddr[6];

#ifdef CYGPKG_REDBOOT
# include <pkgconf/redboot.h>
# ifdef CYGSEM_REDBOOT_FLASH_CONFIG
#  include <redboot.h>
#  include <flash_config.h>
   RedBoot_config_option("Network hardware address [MAC]", fec_esa,
                ALWAYS_ENABLED, true, CONFIG_ESA, _default_enaddr /*0*/);
# endif
#endif

#define os_printf               diag_printf

// For fetching the ESA from RedBoot
#include <cyg/hal/hal_if.h>
#ifndef CONFIG_ESA
# define CONFIG_ESA             6
#endif

ETH_DRV_SC(fec_eth0_sc,
           &fec_eth0_info, // Driver specific data
           "eth0", /* was CYGDAT_DEVS_ETH_POWERPC_MPC8xx_ETH_NAME */
           fec_eth_start,
           fec_eth_stop,
           fec_eth_control,
           fec_eth_can_send,
           fec_eth_send,
           fec_eth_recv,
           fec_eth_deliver,
           fec_eth_int,
           fec_eth_int_vector);

NETDEVTAB_ENTRY(fec_netdev, "fec_eth", fec_eth_init, &fec_eth0_sc);

#ifdef CYGINT_IO_ETH_INT_SUPPORT_REQUIRED
# define _FEC_USE_INTS
# ifdef _FEC_USE_INTS
   static cyg_interrupt fec_eth_interrupt;
   static cyg_handle_t  fec_eth_interrupt_handle;
# else
#  define STACK_SIZE CYGNUM_HAL_STACK_SIZE_MINIMUM
   static char fec_fake_int_stack[STACK_SIZE];
   static cyg_thread fec_fake_int_thread_data;
   static cyg_handle_t fec_fake_int_thread_handle;
   static void         fec_fake_int(cyg_addrword_t);
# endif
#endif

static void         fec_eth_int(struct eth_drv_sc *data);

#ifndef CYGNUM_DEVS_ETH_POWERPC_MPC8xx_INT_LEVEL
# error  CYGNUM_DEVS_ETH_POWERPC_MPC8xx_INT_LEVEL must be defined
#endif

#ifndef CYGNUM_DEVS_ETH_POWERPC_MPC8xx_PHY_ADDR
# error  CYGNUM_DEVS_ETH_POWERPC_MPC8xx_PHY_ADDR must be defined
#endif

#ifndef FEC_ETH_RESET_PHY
# define FEC_ETH_RESET_PHY()
#endif

#ifdef CYGSEM_DEVS_ETH_POWERPC_MPC8xx_STATUS_LEDS
// LED activity [exclusive of hardware bits]
# ifndef _get_led
#  define _get_led()
#  define _set_led(v)
# endif

# ifndef LED_TxACTIVE
#  define LED_TxACTIVE  7
#  define LED_RxACTIVE  6
#  define LED_IntACTIVE 5
# endif

/*FUNCTION:------------------------------------------------------
 *  NAME
 *      set_led
 *  DESCRIPTION
 *
 *  PARAMETERS
 *      int bit
 *  RETURNS
 *      void
 *---------------------------------------------------------------
 */
static void
set_led(int bit)
{
    _set_led(_get_led() | (1 << bit));
} /* set_led */

/*FUNCTION:------------------------------------------------------
 *  NAME
 *      clear_led
 *  DESCRIPTION
 *
 *  PARAMETERS
 *      int bit
 *  RETURNS
 *      void
 *---------------------------------------------------------------
 */
static void
clear_led(int bit)
{
    _set_led(_get_led() & ~(1 << bit));
} /* clear_led */
#else
# define set_led(b)
# define clear_led(b)
#endif

#ifdef _FEC_USE_INTS
/*FUNCTION:------------------------------------------------------
 *  NAME
 *      fec_eth_isr
 *  DESCRIPTION
 *      This ISR is called when the ethernet interrupt occurs
 *  PARAMETERS
 *      cyg_vector_t vector
 *      cyg_addrword_t data
 *      HAL_SavedRegisters * regs
 *  RETURNS
 *      int
 *---------------------------------------------------------------
 */
static int
fec_eth_isr(cyg_vector_t vector, cyg_addrword_t data, HAL_SavedRegisters * regs)
{
    cyg_drv_interrupt_mask(CYGNUM_DEVS_ETH_POWERPC_MPC8xx_INT_LEVEL);
    return (CYG_ISR_HANDLED | CYG_ISR_CALL_DSR);    /* Run the DSR */
} /* fec_eth_isr */
#endif

/*FUNCTION:------------------------------------------------------
 *  NAME
 *      fec_eth_deliver
 *  DESCRIPTION
 *      Deliver function (ex-DSR) handles the ethernet [logical] processing
 *  PARAMETERS
 *      struct eth_drv_sc * sc
 *  RETURNS
 *      void
 *---------------------------------------------------------------
 */
static void
fec_eth_deliver(struct eth_drv_sc * sc)
{
    fec_eth_int(sc);
#ifdef _FEC_USE_INTS
    /* Allow interrupts to happen again */
    cyg_drv_interrupt_acknowledge(CYGNUM_DEVS_ETH_POWERPC_MPC8xx_INT_LEVEL);
    cyg_drv_interrupt_unmask(CYGNUM_DEVS_ETH_POWERPC_MPC8xx_INT_LEVEL);
#endif
} /* fec_eth_deliver */

#ifdef CYGSEM_DEVS_ETH_POWERPC_MPC8xx_RESET_PHY
# ifndef CYGPKG_REDBOOT
cyg_mutex_t     mut_phy_write;
# endif
/*FUNCTION:------------------------------------------------------
 *  NAME
 *      phy_write
 *  DESCRIPTION
 *      PHY unit access (via MII channel)
 *  PARAMETERS
 *      int reg
 *      int addr
 *      unsigned short data
 *  RETURNS
 *      void
 *---------------------------------------------------------------
 */
externC void
phy_write(int reg, int addr, unsigned short data)
{
    volatile EPPC *                 eppc;
    volatile struct fec *           fec;
    int                             timeout;

    eppc = (volatile EPPC *)eppc_base();
    fec = (volatile struct fec *)((unsigned char *)eppc + FEC_OFFSET);

# ifndef CYGPKG_REDBOOT
    cyg_mutex_lock(&mut_phy_write);
# endif
    fec->iEvent = iEvent_MII;
    fec->MiiData = MII_Start | MII_Write | MII_Phy(addr) | MII_Reg(reg) | MII_TA | data;

    timeout = 0x100000;
    while ( !(fec->iEvent & iEvent_MII) && (--timeout > 0) ) {
        ;
    }
# ifndef CYGPKG_REDBOOT
    cyg_mutex_unlock(&mut_phy_write);
# endif
} /* phy_write */

/*FUNCTION:------------------------------------------------------
 *  NAME
 *      phy_read
 *  DESCRIPTION
 *
 *  PARAMETERS
 *
 *  RETURNS
 *
 *---------------------------------------------------------------
 */
externC bool
phy_read(int reg, int addr, unsigned short *val)
{
    volatile EPPC *             eppc;
    volatile struct fec *       fec;
    volatile int                timeout;

    eppc = (volatile EPPC *)eppc_base();
    fec = (volatile struct fec *)((unsigned char *)eppc + FEC_OFFSET);

    fec->iEvent = iEvent_MII;
    fec->MiiData = MII_Start | MII_Read | MII_Phy(addr) | MII_Reg(reg) | MII_TA;
    timeout = 100;
    while ( ! (fec->iEvent & iEvent_MII) && timeout-- ) {
        HAL_DELAY_US(1000);
    }
    if ( ! timeout ) {
        return false;
    }
    *val = fec->MiiData & 0x0000FFFF;
    return true;
} /* phy_read */

/*FUNCTION:------------------------------------------------------
 *  NAME
 *      fec_eth_li_status
 *  DESCRIPTION
 *      Reads the Link status from the PHY.
 *  PARAMETERS
 *      none
 *  RETURNS
 *      int
 *---------------------------------------------------------------
 */
int
fec_eth_li_status(void)
{
    unsigned short                  phy_state;

    /* Throw away the first read of this register since
     * the LI bit latches the last event that occurred. */
    //HAL_DELAY_US(10000);
    phy_state = 0;
    phy_read(PHY_BMSR, CYGNUM_DEVS_ETH_POWERPC_MPC8xx_PHY_ADDR, &phy_state);
    //HAL_DELAY_US(10000);

    /* Get the state */
    phy_state = 0;
    phy_read(PHY_BMSR, CYGNUM_DEVS_ETH_POWERPC_MPC8xx_PHY_ADDR, &phy_state);
    //HAL_DELAY_US(10000);

    if ( phy_state & PHY_BMSR_LINK ) {
        return 1;
    }
    else {
        return 0;
    }

} /** fec_eth_li_status **/

#endif // CYGSEM_DEVS_ETH_POWERPC_MPC8xx_RESET_PHY

/*FUNCTION:------------------------------------------------------
 *  NAME
 *      fec_eth_reset
 *  DESCRIPTION
 *      [re]Initialize the ethernet controller
 *      Done separately since shutting down the device requires a
 *      full reconfiguration when re-enabling.
 *  PARAMETERS
 *      struct eth_drv_sc * sc
 *      unsigned char * enaddr
 *      int flags
 *  RETURNS
 *      bool
 *---------------------------------------------------------------
 */
static bool
fec_eth_reset(struct eth_drv_sc *sc, unsigned char *enaddr, int flags)
{
    struct fec_eth_info *               qi;
    volatile EPPC *                     eppc;
    volatile struct fec *               fec;
    volatile struct fec_bd *            rxbd;
    volatile struct fec_bd *            txbd;
    unsigned char *                     RxBUF;
    unsigned char *                     TxBUF;
    int                                 cache_state;
    int                                 int_state;
    int                                 i;
    int                                 TxBD;
    int                                 RxBD;

    qi = (struct fec_eth_info *)sc->driver_private;
    eppc = (volatile EPPC *)eppc_base();
    fec = (volatile struct fec *)((unsigned char *)eppc + FEC_OFFSET);

    /* Device physical address */
    fec->addr[0] = *(unsigned long *)&enaddr[0];
    fec->addr[1] = *(unsigned long *)&enaddr[4];

    /* Ignore unless device is idle/stopped */
    if ( (qi->fec->eControl & eControl_EN) != 0 ) {
        return true;
    }

    /* Make sure interrupts are off while we mess with the device */
    HAL_DISABLE_INTERRUPTS(int_state);

    /* Ensure consistent state between cache and what the FEC sees */
    HAL_DCACHE_IS_ENABLED(cache_state);
    if ( cache_state ) {
        HAL_DCACHE_SYNC();
        HAL_DCACHE_DISABLE();
    }

    /* Shut down ethernet controller, in case it is already running */
    fec->eControl = eControl_RESET;
    i = 0;
    while ( (fec->eControl & eControl_RESET) != 0 ) {
        if ( ++i >= 500000 ) {
            os_printf("FEC Ethernet does not reset\n");
            if ( cache_state ) {
                HAL_DCACHE_ENABLE();
            }
            HAL_RESTORE_INTERRUPTS(int_state);
            return false;
        }
    }

    fec->iMask = 0x0000000; /* Disables all interrupts */
    fec->iEvent = 0xFFFFFFFF;   /* Clear all interrupts */
    fec->iVector = FEC_IVEC_VAL;

#if defined(CYGPKG_REDBOOT) && 0
    txbd = (struct fec_bd *)(FEC_EPPC_BD_OFFSET + (cyg_uint32)eppc);
    rxbd = &txbd[CYGNUM_DEVS_ETH_POWERPC_MPC8xx_TxNUM];
#else
    TxBD = _mpc8xx_allocBd(CYGNUM_DEVS_ETH_POWERPC_MPC8xx_TxNUM * sizeof(struct cp_bufdesc));
    RxBD = _mpc8xx_allocBd(CYGNUM_DEVS_ETH_POWERPC_MPC8xx_RxNUM * sizeof(struct cp_bufdesc));
    txbd = (struct fec_bd *)(TxBD + (cyg_uint32)eppc);
    rxbd = (struct fec_bd *)(RxBD + (cyg_uint32)eppc);
#endif

    qi->tbase = qi->txbd = qi->tnext = txbd;
    qi->rbase = qi->rxbd = qi->rnext = rxbd;
    qi->txactive = 0;

    RxBUF = &fec_eth_rxbufs[0];
    TxBUF = &fec_eth_txbufs[0];

    /* setup buffer descriptors */
    for ( i = 0; i < CYGNUM_DEVS_ETH_POWERPC_MPC8xx_RxNUM; i++ ) {
        rxbd->length = 0;
        rxbd->buffer = RxBUF;
        rxbd->ctrl = FEC_BD_Rx_Empty;
        RxBUF += CYGNUM_DEVS_ETH_POWERPC_MPC8xx_BUFSIZE;
        rxbd++;
    }
    rxbd--;
    rxbd->ctrl |= FEC_BD_Rx_Wrap;   /* Last buffer */
    for ( i = 0; i < CYGNUM_DEVS_ETH_POWERPC_MPC8xx_TxNUM; i++ ) {
        txbd->length = 0;
        txbd->buffer = TxBUF;
        txbd->ctrl = 0;
        TxBUF += CYGNUM_DEVS_ETH_POWERPC_MPC8xx_BUFSIZE;
        txbd++;
    }
    txbd--;
    txbd->ctrl |= FEC_BD_Tx_Wrap;   /* Last buffer */

    /* Reset interrupts */
    fec->iMask = 0x00000000;    /* No interrupts enabled */
    fec->iEvent = 0xFFFFFFFF;   /* Clear all interrupts */

    /* Initialize shared PRAM */
    fec->RxRing = qi->rbase;
    fec->TxRing = qi->tbase;

    /* Size of receive buffers */
    fec->RxBufSize = CYGNUM_DEVS_ETH_POWERPC_MPC8xx_BUFSIZE;

    /* Receiver control */
    fec->RxControl = RxControl_MII | RxControl_DRT;

#if 0
    fec->RxControl = RxControl_MII | RxControl_LOOP | RxControl_PROM;
#endif

    fec->RxHash = IEEE_8023_MAX_FRAME;  /* Largest possible ethernet frame */

    /* Transmit control */
    fec->TxControl = 4 + 0;

    /* Use largest possible Tx FIFO */
    fec->TxWater = 3;

    /* DMA control */
    fec->FunCode = ((2 << 29) | (2 << 27) | (0 << 24));

    /* MII speed control (50MHz) */
    fec->MiiSpeed = 0x14;

    /* Group address hash */
    fec->hash[0] = 0;
    fec->hash[1] = 0;

#if 0 /* BUGBUG - moved up */
    /* Device physical address */
    fec->addr[0] = *(unsigned long *)&enaddr[0];
    fec->addr[1] = *(unsigned long *)&enaddr[4];
#endif

    /* Enable device */
    fec->eControl = eControl_EN | eControl_MUX;
    fec->RxUpdate = 0x0F0F0F0F; /* Any write tells machine to look for work */

#ifdef _FEC_USE_INTS
    /* Set up for interrupts */
    fec->iMask = iEvent_TFINT | iEvent_TXB | iEvent_RFINT | iEvent_RXB;
    fec->iEvent = 0xFFFFFFFF;   /* Clear all interrupts */
#endif

    if (cache_state) {
        HAL_DCACHE_ENABLE();
    }

    /* Set LED state */
    clear_led(LED_TxACTIVE);
    clear_led(LED_RxACTIVE);

    HAL_RESTORE_INTERRUPTS(int_state);
    return true;
} /* fec_eth_reset */

#define PROC_REVB 0x0020
/*FUNCTION:------------------------------------------------------
 *  NAME
 *      fec_eth_init
 *  DESCRIPTION
 *      Initialize the interface - performed at system startup.
 *      This function must set up the interface, including arranging to
 *      handle interrupts, etc, so that it may be "started" cheaply later.
 *  PARAMETERS
 *      struct cyg_netdevtab_entry * tab
 *  RETURNS
 *      bool
 *---------------------------------------------------------------
 */
static bool
fec_eth_init(struct cyg_netdevtab_entry * tab)
{
    struct eth_drv_sc *             sc;
    struct fec_eth_info *           qi;
    volatile EPPC *                 eppc;
    volatile struct fec *           fec;
    int                             cache_state;
    int                             i;
    unsigned long                   proc_rev;
    bool                            esa_ok;
#ifdef CYGSEM_DEVS_ETH_POWERPC_MPC8xx_RESET_PHY
    int                             phy_timeout;
    bool                            phy_ok;
    unsigned short                  phy_state = 0;

# ifndef CYGPKG_REDBOOT
    cyg_mutex_init(&mut_phy_write);
# endif
#endif

    sc = (struct eth_drv_sc *)tab->device_instance;
    qi = (struct fec_eth_info *)sc->driver_private;
    eppc = (volatile EPPC *)eppc_base();
    fec = (volatile struct fec *)((unsigned char *)eppc + FEC_OFFSET);

    if_fec_num_interrupts = 0;
    if_fec_num_rx_interrupts = 0;
    if_fec_num_rx_drops = 0;
    if_fec_num_rx_error_lg = 0;
    if_fec_num_rx_error_no = 0;
    if_fec_num_rx_error_sh = 0;
    if_fec_num_rx_error_cr = 0;
    if_fec_num_rx_error_ov = 0;
    if_fec_num_rx_error_tr = 0;

    if_fec_num_tx_error_lc = 0;
    if_fec_num_tx_error_rl = 0;
    if_fec_num_tx_error_ur = 0;

    /* Ensure consistent state between cache and what the FEC sees */
    HAL_DCACHE_IS_ENABLED(cache_state);
    if ( cache_state ) {
        HAL_DCACHE_SYNC();
        HAL_DCACHE_DISABLE();
    }

    qi->fec = fec;

    /* Make sure it's not running yet */
    fec_eth_stop(sc);

#ifdef CYGINT_IO_ETH_INT_SUPPORT_REQUIRED
# ifdef _FEC_USE_INTS
    /* Set up to handle interrupts */
    cyg_drv_interrupt_create(CYGNUM_DEVS_ETH_POWERPC_MPC8xx_INT_LEVEL,
                             CYGARC_SIU_PRIORITY_HIGH,
                             (cyg_addrword_t)sc, //  Data item passed to interrupt handler
                             (cyg_ISR_t *)fec_eth_isr,
                             (cyg_DSR_t *)eth_drv_dsr,
                             &fec_eth_interrupt_handle,
                             &fec_eth_interrupt);
    cyg_drv_interrupt_attach(fec_eth_interrupt_handle);
    cyg_drv_interrupt_acknowledge(CYGNUM_DEVS_ETH_POWERPC_MPC8xx_INT_LEVEL);
    cyg_drv_interrupt_unmask(CYGNUM_DEVS_ETH_POWERPC_MPC8xx_INT_LEVEL);
# else
    // Hack - use a thread to simulate interrupts
    cyg_thread_create(1,                 // Priority
                      fec_fake_int,   // entry
                      (cyg_addrword_t)sc, // entry parameter
                      "CS8900 int",      // Name
                      &fec_fake_int_stack[0],         // Stack
                      STACK_SIZE,        // Size
                      &fec_fake_int_thread_handle,    // Handle
                      &fec_fake_int_thread_data       // Thread data structure
            );
    cyg_thread_resume(fec_fake_int_thread_handle);  // Start it
# endif
#endif

    // Set up parallel port for connection to ethernet tranceiver
    eppc->pio_pdpar = 0x1FFF;
    CYGARC_MFSPR( CYGARC_REG_PVR, proc_rev );

    if ((proc_rev & 0x0000FFFF) == PROC_REVB) {
        eppc->pio_pddir = 0x1C58;
    }
    else {
        eppc->pio_pddir = 0x1FFF;
    }

    // Get physical device address
#ifdef CYGPKG_REDBOOT
# ifdef CYGSEM_REDBOOT_FLASH_CONFIG
    esa_ok = flash_get_config("fec_esa", enaddr, CONFIG_ESA);
# else
    esa_ok = false;
# endif
#else
    esa_ok = false;
#endif

    if ( (!esa_ok) || (*(unsigned long *)enaddr == 0) ) {
        /* Can't figure out ESA */
        os_printf("FEC_ETH - Warning! ESA unknown\n");
        memcpy(&enaddr, &_default_enaddr, sizeof(enaddr));
    }

    /* Configure the device */
    if ( !fec_eth_reset(sc, enaddr, 0) ) {
        return false;
    }

    // Reset PHY (transceiver)
    FEC_ETH_RESET_PHY();

    // Enable transceiver (PHY)
    fec->MiiSpeed = 0x14;
    phy_ok = 0;
    phy_write(PHY_BMCR, CYGNUM_DEVS_ETH_POWERPC_MPC8xx_PHY_ADDR, PHY_BMCR_RESET);
    for (i = 0;  i < 10;  i++) {
        phy_ok = phy_read(PHY_BMCR, CYGNUM_DEVS_ETH_POWERPC_MPC8xx_PHY_ADDR, &phy_state);
        if ( !phy_ok ) {
            break;
        }
        if ( !(phy_state & PHY_BMCR_RESET) ) {
            break;
        }
    }
    if (!phy_ok || (phy_state & PHY_BMCR_RESET)) {
        os_printf("FEC: Can't get PHY unit to reset: %x\n", phy_state);
        return false;
    }
    /* Wait for some reason? */
    phy_timeout = 100;  /* was 5000 */
    while (phy_timeout--) {
        HAL_DELAY_US(1000);
    }

    (void)fec_eth_set_state(PHY_BMCR_AUTO_NEG);

#ifdef CYGPKG_REDBOOT
# if defined(CYGPKG_HAL_POWERPC_H307) || \
     defined(CYGPKG_HAL_POWERPC_H304) || \
     defined(CYGPKG_HAL_POWERPC_H304_REDBOOT)
    /*
     * Set LED Operation (PLED3 = LINK+STAT, PLED0 = BLINKING).
     * This is the best LSI has to offer, other than S/W control.
     */
    HAL_DELAY_US(10000);
    phy_write(PHY_LSI80223_REG17, CYGNUM_DEVS_ETH_POWERPC_MPC8xx_PHY_ADDR, (PHY_LSI80223_PLED3_1 | PHY_LSI80223_PLED3_0) |
                                               (PHY_LSI80223_PLED0_1) |
                                               (PHY_LSI80223_PLED2_1) |
                                               (PHY_LSI80223_LED_DEF0));
# elif defined(CYGPKG_HAL_POWERPC_H051)
    /*
     * The Q051 uses the GPIO pins to drive the LEDs. In RedBoot, we want
     * only the green LEDs to be on.
     */
#  define IMMR_BASE                      0xfff00000

#  define REG_PADAT                      (volatile unsigned short int *)(IMMR_BASE + 0x00000956)
#  define REG_PADIR                      (volatile unsigned short int *)(IMMR_BASE + 0x00000950)
#  define REG_PAPAR                      (volatile unsigned short int *)(IMMR_BASE + 0x00000952)

/* The lower 16 bits of the Port B register. */
#  define REG_PBDAT                      (volatile unsigned short int *)(IMMR_BASE + 0x00000AC6)
#  define REG_PBDIR                      (volatile unsigned short int *)(IMMR_BASE + 0x00000ABA)
#  define REG_PBPAR                      (volatile unsigned short int *)(IMMR_BASE + 0x00000ABE)

#  define LED_STAT_GRN_PA_BIT            0x0040      /* PA9 */
#  define LED_STAT_RED_PA_BIT            0x0080      /* PA8 */
#  define LED_NET_GRN_PA_BIT             0x2000      /* PA2 */
#  define LED_NET_RED_PA_BIT             0x8000      /* PA0 */
#  define LED_RF_GRN_PB_BIT              0x0040      /* PB25 */

    /* Program the Ports */
    (*REG_PADIR) = (LED_STAT_GRN_PA_BIT | LED_STAT_RED_PA_BIT | LED_NET_GRN_PA_BIT | LED_NET_RED_PA_BIT);
    (*REG_PAPAR) = (0x0000);
    (*REG_PADAT) = (LED_STAT_GRN_PA_BIT | LED_NET_GRN_PA_BIT);

    (*REG_PBDIR) = (LED_RF_GRN_PB_BIT);
    (*REG_PBPAR) = (0x0000);
    (*REG_PBDAT) = (LED_RF_GRN_PB_BIT);
# endif
#endif /* CYGPKG_REDBOOT */

    /* Initialize upper level driver */
    (sc->funs->eth_drv->init) (sc, (unsigned char *)&enaddr);

    if ( cache_state ) {
        HAL_DCACHE_ENABLE();
    }

    return true;
} /* fec_eth_init */

/*FUNCTION:------------------------------------------------------
 *  NAME
 *      fec_eth_stop
 *  DESCRIPTION
 *      This function is called to shut down the interface.
 *  PARAMETERS
 *      struct eth_drv_sc * sc
 *  RETURNS
 *      void
 *---------------------------------------------------------------
 */
static void
fec_eth_stop(struct eth_drv_sc * sc)
{
    struct fec_eth_info *           qi;

    qi = (struct fec_eth_info *)sc->driver_private;

    // Disable the device!
    qi->fec->eControl &= ~eControl_EN;
} /* fec_eth_stop */

/*FUNCTION:------------------------------------------------------
 *  NAME
 *      fec_eth_start
 *  DESCRIPTION
 *      This function is called to "start up" the interface.  It may be called
 *      multiple times, even when the hardware is already running.  It will be
 *      called whenever something "hardware oriented" changes and should leave
 *      the hardware ready to send/receive packets.
 *  PARAMETERS
 *      struct eth_drv_sc * sc
 *      unsigned char * enaddr
 *      int flags
 *  RETURNS
 *      void
 *---------------------------------------------------------------
 */
static void
fec_eth_start(struct eth_drv_sc * sc, unsigned char * enaddr, int flags)
{
    /* Enable the device! */
    fec_eth_reset(sc, enaddr, flags);
} /* fec_eth_start */

/*FUNCTION:------------------------------------------------------
 *  NAME
 *      fec_eth_set_state
 *  DESCRIPTION
 *      modes
 *          PHY_BMCR_AUTO_NEG
 *          PHY_BMCR_100MB
 *          PHY_BMCR_FULL_DUPLEX
 *  PARAMETERS
 *      unsigned short mode
 *  RETURNS
 *      bool
 *---------------------------------------------------------------
 */
externC bool
fec_eth_set_state(unsigned short mode)
{
    int                         phy_timeout;
    bool                        phy_ok;
    unsigned short              phy_state;
    volatile EPPC *             eppc;
    volatile struct fec *       fec;
    bool                        isAuto;

    eppc = (volatile EPPC *)eppc_base();
    fec = (volatile struct fec *)((unsigned char *)eppc + FEC_OFFSET);

    isAuto = (mode & PHY_BMCR_AUTO_NEG) ? true : false;

    /* Sanity check */
    if ( isAuto ) {
        mode = PHY_BMCR_AUTO_NEG;
    }

    /* Clear all interrupts */
    fec->iEvent = 0xFFFFFFFF;

    /* Set the PHY mode */
    phy_timeout = 100;
    phy_write(PHY_BMCR, 0, mode|PHY_BMCR_RESTART);
    while ( phy_timeout-- >= 0 ) {
        HAL_DELAY_US(1000);
    }

    /* Wait until the mode has been set */
    phy_timeout = 100;
    while ( phy_timeout-- ) {
        int                 ev;

        /* huh? */
        ev = fec->iEvent;
        fec->iEvent = ev;

        if ( ev & iEvent_MII ) {
            phy_ok = phy_read(PHY_BMSR, 0, &phy_state);
            if ( !phy_ok || (isAuto && !(phy_state & PHY_BMSR_AUTO_NEG)) ) {
                HAL_DELAY_US(1000);
                continue;
            }
            // os_printf("State: %x\n", phy_state);
            break;
        }
    } /* while */
    if ( (phy_timeout <= 0) && isAuto ) {
        os_printf("** FEC Warning: PHY auto-negotiation failed\n");
        return (false);
    }
    return (true);
} /* fec_eth_set_state */

typedef int(*SprintfFunc)(unsigned int, const char *, ...);
/*FUNCTION:------------------------------------------------------
 *  NAME
 *      fec_eth_dump_phy
 *  DESCRIPTION
 *      Debuging function to dump contents of PHY registers
 *  PARAMETERS
 *
 *  RETURNS
 *
 *---------------------------------------------------------------
 */
externC void
fec_eth_dump_phy(unsigned int arg, SprintfFunc inFunction)
{
    bool                        phy_ok;
    unsigned short              phy_state;
    int                         i;
    const char * const          miiregs[] = { "Control", "Status", "PHY Id(H)",
                                             "PHY Id(L)", "Advertisement",
                                             "Link Partner Ability", 0 };

    for ( i=0; miiregs[i]; i++ ) {
        phy_ok = phy_read(i, 0, &phy_state);
        inFunction(arg, "%30s: 0x%04x\n", miiregs[i], phy_ok ? phy_state : 0x0000);
    } /* for */
} /* fec_eth_dump_phy */

/*FUNCTION:------------------------------------------------------
 *  NAME
 *      fec_eth_control
 *  DESCRIPTION
 *      This function is called for low level "control" operations
 *  PARAMETERS
 *
 *  RETURNS
 *
 *---------------------------------------------------------------
 */
static int
fec_eth_control(struct eth_drv_sc *sc, unsigned long key, void *data, int length)
{
#ifdef ETH_DRV_SET_MC_ALL
    struct fec_eth_info *           qi;
    volatile struct fec *           fec;

    qi = (struct fec_eth_info *)sc->driver_private;
    fec = qi->fec;
#endif

    switch (key) {
        case ETH_DRV_SET_MAC_ADDRESS:
            return 0;

#ifdef ETH_DRV_SET_MC_ALL
        case ETH_DRV_SET_MC_ALL:
        case ETH_DRV_SET_MC_LIST:
            fec->RxControl &= ~RxControl_PROM;
            fec->hash[0] = 0xFFFFFFFF;
            fec->hash[1] = 0xFFFFFFFF;
            return 0;
#endif

        default:
            return 1;
    }
} /* fec_eth_control */

/*FUNCTION:------------------------------------------------------
 *  NAME
 *      fec_eth_can_send
 *  DESCRIPTION
 *      This function is called to see if another packet can be sent.
 *      It should return the number of packets which can be handled.
 *      Zero should be returned if the interface is busy and can not
 *      send any more.
 *  PARAMETERS
 *      struct eth_drv_sc * sc
 *  RETURNS
 *      int
 *---------------------------------------------------------------
 */
static int
fec_eth_can_send(struct eth_drv_sc * sc)
{
    struct fec_eth_info *       qi;

    qi = (struct fec_eth_info *)sc->driver_private;
    return (qi->txactive < CYGNUM_DEVS_ETH_POWERPC_MPC8xx_TxNUM);
} /* fec_eth_can_send */

/*FUNCTION:------------------------------------------------------
 *  NAME
 *      fec_eth_send
 *  DESCRIPTION
 *      This routine is called to send data to the hardware.
 *  PARAMETERS
 *      struct eth_drv_sc * sc
 *      struct eth_drv_sg * sg_list
 *      int sg_len
 *      int total_len
 *      unsigned long key
 *  RETURNS
 *      void
 *---------------------------------------------------------------
 */
static void
fec_eth_send(struct eth_drv_sc * sc, struct eth_drv_sg * sg_list, int sg_len, int total_len, unsigned long key)
{
    struct fec_eth_info *       qi = (struct fec_eth_info *)sc->driver_private;
    volatile struct fec_bd *    txbd, *txfirst;
    volatile char *             bp;
    int                         i, txindex, cache_state;

    /* Find a free buffer */
    txbd = txfirst = qi->txbd;
    while ( txbd->ctrl & FEC_BD_Tx_Ready ) {
        /* This buffer is busy, move to next one */
        if ( txbd->ctrl & FEC_BD_Tx_Wrap ) {
            txbd = qi->tbase;
        }
        else {
            txbd++;
        }
        if ( txbd == txfirst ) {
#ifdef CYGPKG_NET
            panic("No free xmit buffers");
#else
            os_printf("FEC Ethernet: No free xmit buffers\n");
#endif
        }
    }
    /* Set up buffer */
    bp = txbd->buffer;
    for ( i = 0; i < sg_len; i++ ) {
        memcpy((void *)bp, (void *)sg_list[i].buf, sg_list[i].len);
        bp += sg_list[i].len;
    }
    txbd->length = total_len;
    txindex = ((unsigned long)txbd - (unsigned long)qi->tbase) / sizeof(* txbd);
    qi->txkey[txindex] = key;
    /*
     * Make sure the data, which may reside in cache, is stored back to
     * RAM so the hardware can read it.
     */
    HAL_DCACHE_IS_ENABLED(cache_state);
    if ( cache_state ) {
        HAL_DCACHE_STORE(txbd->buffer, txbd->length);
    }
    /* Send it on it's way */
    txbd->ctrl |= FEC_BD_Tx_Ready | FEC_BD_Tx_Last | FEC_BD_Tx_TC;

    qi->txactive++;
    qi->fec->TxUpdate = 0x01000000;  // Any write tells machine to look for work
    set_led(LED_TxACTIVE);

    if ( fec_eth_tx_callback_func ) {
        fec_eth_tx_callback_func();
    }

    /* Remember the next buffer to try */
    if ( txbd->ctrl & FEC_BD_Tx_Wrap ) {
        qi->txbd = qi->tbase;
    }
    else {
        qi->txbd = txbd + 1;
    }
} /* fec_eth_send */

/*FUNCTION:------------------------------------------------------
 *  NAME
 *      fec_eth_RxEvent
 *  DESCRIPTION
 *      This function is called when a packet has been received.  It's job is
 *      to prepare to unload the packet from the hardware.  Once the length of
 *      the packet is known, the upper layer of the driver can be told.  When
 *      the upper layer is ready to unload the packet, the internal function
 *      'fec_eth_recv' will be called to actually fetch it from the hardware.
 *  PARAMETERS
 *      struct eth_drv_sc * sc
 *  RETURNS
 *      void
 *---------------------------------------------------------------
 */
static void
fec_eth_RxEvent(struct eth_drv_sc * sc)
{
    struct fec_eth_info         *qi = (struct fec_eth_info *)sc->driver_private;
    volatile struct fec_bd      *rxbd, *rxfirst;

    rxbd = rxfirst = qi->rnext;
    while (1) {
        /*
         * sal-20081024: This check was added to replace the wrap-around
         * check.  Before, this loop would check all the BD's for data,
         * but the next BD to check should be the next received one.
         * If it's empty, then we can assume that all the others are
         * empty as well.
         */
        if (rxbd->ctrl & FEC_BD_Rx_Empty) {
            break;
        }

        /* Determine if it has any errors, and the error type(s) */
        if ( rxbd->ctrl & FEC_BD_Rx_LG ) {
            /* Rx frame length violation */
            if_fec_num_rx_error_lg++;
        }
        if ( rxbd->ctrl & FEC_BD_Rx_NO ) {
            /* Rx nonoctet-aligned frame */
            if_fec_num_rx_error_no++;
        }
        if ( rxbd->ctrl & FEC_BD_Rx_SH ) {
            /* Short frame */
            if_fec_num_rx_error_sh++;
        }
        if ( rxbd->ctrl & FEC_BD_Rx_CR ) {
            /* CRC error */
            if_fec_num_rx_error_cr++;
        }
        if ( rxbd->ctrl & FEC_BD_Rx_OV ) {
            /* Overrun */
            if_fec_num_rx_error_ov++;
        }
        if ( rxbd->ctrl & FEC_BD_Rx_TR ) {
            /* Frame truncated */
            if_fec_num_rx_error_tr++;
        }

        /*
         * Even if the packet has errors, we still pass it on up the layers,
         * since we don't want to break the current implementation of the
         * FEC driver.
         */
        qi->rxbd = rxbd;

        /* We've received a packet.  Blink LEDs. */
        set_led(LED_RxACTIVE);
        if ( fec_eth_rx_callback_func ) {
            fec_eth_rx_callback_func();
        }

        /* Call eth_drv_recv() */
        sc->funs->eth_drv->recv(sc, rxbd->length);

#if 0
        /*
         * sal-2011-02-08: DISABLED - Causes lockup if we're being streamed
         * data at a very high rate.  We get out of the loop and never
         * receive any more interrupts because all of the BDs are full.
         */

        /* Check the return conditions */
        if ( ! (rxbd->ctrl & FEC_BD_Rx_Empty) ) {
            break;
        }
#endif /* 0 */

        /* Increment the BD index */
        if (rxbd->ctrl & FEC_BD_Rx_Wrap) {
            rxbd = qi->rbase;
        }
        else {
            rxbd++;
        }

#if 0
        /*
         * sal-20081024: This was here in the previous version of the
         * driver, but it creates a delay during a streaming connection
         * every CYGNUM_DEVS_ETH_POWERPC_MPC8xx_RxNUM packets.  I've added
         * a check at the start of this loop for an empty BD.
         */
        if (rxbd == rxfirst) {
            break;
        }
#endif /* 0 */
    } /* while */

    /* Remember where we left off */
    qi->rnext = (struct fec_bd *)rxbd;
    qi->fec->RxUpdate = 0x0F0F0F0F; /* Any write tells machine to look for work */
} /* fec_eth_RxEvent */

/*FUNCTION:------------------------------------------------------
 *  NAME
 *      fec_eth_recv
 *  DESCRIPTION
 *      This function is called as a result of the "eth_drv_recv()" call above.
 *      It's job is to actually fetch data for a packet from the hardware once
 *      memory buffers have been allocated for the packet.  Note that the buffers
 *      may come in pieces, using a scatter-gather list.  This allows for more
 *      efficient processing in the upper layers of the stack.
 *  PARAMETERS
 *      struct eth_drv_sc * sc
 *      struct eth_drv_sg * sg_list
 *      int sg_len
 *  RETURNS
 *      void
 *---------------------------------------------------------------
 */
static void
fec_eth_recv(struct eth_drv_sc * sc, struct eth_drv_sg * sg_list, int sg_len)
{
    struct fec_eth_info *           qi;
    unsigned char *                 bp;
    int                             i;
    int                             cache_state;

    qi = (struct fec_eth_info *)sc->driver_private;
    bp = (unsigned char *)qi->rxbd->buffer;
    /*
     * Make sure the consumer reads from RAM, where the data is,
     * instead of from cache
     */
    HAL_DCACHE_IS_ENABLED(cache_state);
    if ( cache_state ) {
        HAL_DCACHE_INVALIDATE(qi->rxbd->buffer, qi->rxbd->length);
    }
    for ( i = 0; i < sg_len; i++ ) {
        if ( sg_list[i].buf != 0 ) {
            memcpy((void *)sg_list[i].buf, bp, sg_list[i].len);
            bp += sg_list[i].len;
        }
    }
    qi->rxbd->ctrl |= FEC_BD_Rx_Empty;
    clear_led(LED_RxACTIVE);
} /* fec_eth_recv */

/*FUNCTION:------------------------------------------------------
 *  NAME
 *      fec_eth_TxEvent
 *  DESCRIPTION
 *
 *  PARAMETERS
 *      struct eth_drv_sc * sc
 *  RETURNS
 *      void
 *---------------------------------------------------------------
 */
static void
fec_eth_TxEvent(struct eth_drv_sc * sc)
{
    struct fec_eth_info *           qi;
    volatile struct fec_bd *        txbd;
    int                             key, txindex;

    qi = (struct fec_eth_info *)sc->driver_private;
    txbd = qi->tnext;
    // Note: TC field is used to indicate the buffer has/had data in it
    while ((txbd->ctrl & (FEC_BD_Tx_Ready|FEC_BD_Tx_TC)) == FEC_BD_Tx_TC) {
        txindex = ((unsigned long)txbd - (unsigned long)qi->tbase) / sizeof(*txbd);
        key = qi->txkey[txindex];
        if (key != 0) {
            qi->txkey[txindex] = 0;
            /* eth_drv_tx_done */
            sc->funs->eth_drv->tx_done(sc, key, 0);
        }
        if (--qi->txactive == 0) {
          clear_led(LED_TxACTIVE);
        }

        /* Check for tx errors */
        if ( txbd->ctrl & FEC_BD_Tx_LC ) {
            if_fec_num_tx_error_lc++;
        }
        if ( txbd->ctrl & FEC_BD_Tx_RL ) {
            if_fec_num_tx_error_rl++;
        }
        if ( txbd->ctrl & FEC_BD_Tx_UN ) {
            if_fec_num_tx_error_ur++;
        }

        txbd->ctrl &= ~FEC_BD_Tx_TC;
        if (txbd->ctrl & FEC_BD_Tx_Wrap) {
            txbd = qi->tbase;
        }
        else {
            txbd++;
        }
    }
    // Remember where we left off
    qi->tnext = (struct fec_bd *)txbd;
} /* fec_eth_TxEvent */

/*FUNCTION:------------------------------------------------------
 *  NAME
 *      fec_eth_int
 *  DESCRIPTION
 *      Interrupt processing
 *  PARAMETERS
 *      struct eth_drv_sc * sc
 *  RETURNS
 *      void
 *---------------------------------------------------------------
 */
static void
fec_eth_int(struct eth_drv_sc * sc)
{
    struct fec_eth_info *           qi;
    unsigned long                   event;

    qi = (struct fec_eth_info *)sc->driver_private;

    if_fec_num_interrupts++;

    while ((event = qi->fec->iEvent) != 0) {
        if ((event & iEvent_TFINT) != 0) {
            fec_eth_TxEvent(sc);
        }
        if ((event & iEvent_RFINT) != 0) {
            if_fec_num_rx_interrupts++;
            fec_eth_RxEvent(sc);
        }
        qi->fec->iEvent = event;  // Reset the bits we handled
    }
} /* fec_eth_int */

/*FUNCTION:------------------------------------------------------
 *  NAME
 *      fec_eth_int_vector
 *  DESCRIPTION
 *      Interrupt vector
 *  PARAMETERS
 *      struct eth_drv_sc * sc
 *  RETURNS
 *      int
 *---------------------------------------------------------------
 */
static int
fec_eth_int_vector(struct eth_drv_sc * sc)
{
    return (CYGNUM_DEVS_ETH_POWERPC_MPC8xx_INT_LEVEL);
} /* fec_eth_int_vector */

#if defined(CYGINT_IO_ETH_INT_SUPPORT_REQUIRED) && ~defined(_FEC_USE_INTS)
/*FUNCTION:------------------------------------------------------
 *  NAME
 *      fec_fake_int
 *  DESCRIPTION
 *
 *  PARAMETERS
 *      cyg_addrword_t param
 *  RETURNS
 *      void
 *---------------------------------------------------------------
 */
void
fec_fake_int(cyg_addrword_t param)
{
    struct eth_drv_sc *     sc;
    int                     int_state;

    sc = (struct eth_drv_sc *) param;

    while ( true ) {
        cyg_thread_delay(1);    /* 10ms */
        HAL_DISABLE_INTERRUPTS(int_state);
        fec_eth_int(sc);
        HAL_RESTORE_INTERRUPTS(int_state);
    }
} /* fec_fake_int */
#endif
