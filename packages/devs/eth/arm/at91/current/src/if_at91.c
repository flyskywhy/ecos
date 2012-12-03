//==========================================================================
//
//      if_at91.c
//
//
//
//==========================================================================
// ####ECOSGPLCOPYRIGHTBEGIN####
// -------------------------------------------
// This file is part of eCos, the Embedded Configurable Operating System.
// Copyright (C) 1998, 1999, 2000, 2001, 2002, 2006 Free Software Foundation, Inc.
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
// Author(s):    Daniel Helgason
// Contributors: Andrew Lunn, John Eigelaar
// Date:         2010-01-01
// Purpose:
// Description:
//
//####DESCRIPTIONEND####
//
//========================================================================*/

#include <pkgconf/system.h>
#include <pkgconf/hal.h>
#include <pkgconf/devs_eth_arm_at91.h>
#include <pkgconf/io_eth_drivers.h>
#if defined(CYGPKG_REDBOOT)
   #include <pkgconf/redboot.h>
#endif

#include <cyg/hal/hal_io.h>
#include <cyg/hal/hal_intr.h>
#include <cyg/hal/hal_arch.h>
#include <cyg/hal/hal_cache.h>
#include <cyg/hal/drv_api.h>
#include <cyg/hal/hal_diag.h>
#include <cyg/infra/cyg_type.h>
#include <cyg/infra/cyg_ass.h>
#include <cyg/infra/diag.h>
#include <cyg/io/eth/netdev.h>
#include <cyg/io/eth/eth_drv.h>
#include <cyg/io/eth/eth_drv_stats.h>
#include <cyg/io/eth_phy.h>
#include <errno.h>
#include <string.h>

#ifndef MIN
#define MIN(x,y) ((x)<(y) ? (x) : (y))
#endif

// Cache translation
#ifndef CYGARC_UNCACHED_ADDRESS
#define CYGARC_UNCACHED_ADDRESS(x) (x)
#endif

#define AT91_ETH_RX_INT_MASK \
          (AT91_EMAC_ISR_RCOM | AT91_EMAC_ISR_RBNA | AT91_EMAC_ISR_ROVR)

/*
#define AT91_ETH_TX_INT_MASK \
          (AT91_EMAC_ISR_TOVR | AT91_EMAC_ISR_TUND | AT91_EMAC_ISR_RTRY | \
           AT91_EMAC_ISR_TBRE | AT91_EMAC_ISR_TCOM)
*/
#define AT91_ETH_TX_INT_MASK \
          (AT91_EMAC_ISR_TUND | AT91_EMAC_ISR_RTRY | \
           AT91_EMAC_ISR_TBRE | AT91_EMAC_ISR_TCOM)

#define AT91_ETH_INT_MASK (AT91_ETH_RX_INT_MASK | AT91_ETH_TX_INT_MASK)

#define AT91_ETH_NEXT_RBD(idx);                         \
  CYG_MACRO_START                                       \
    idx++;                                              \
    if (idx >= CYGNUM_DEVS_ETH_ARM_AT91_RX_BUFS)        \
      idx = 0;                                          \
  CYG_MACRO_END

#define AT91_ETH_NEXT_TBD(idx);                         \
  CYG_MACRO_START                                       \
    idx++;                                              \
    if (idx >= CYGNUM_DEVS_ETH_ARM_AT91_TX_BUFS)        \
      idx = 0;                                          \
  CYG_MACRO_END

// Set up the level of debug output
#if CYGPKG_DEVS_ETH_ARM_AT91_DEBUG_LEVEL > 0
#define AT91_ETH_DEBUG1(cond, args...)                  \
  CYG_MACRO_START                                       \
    if ((cond))                                         \
      diag_printf(args);                                \
  CYG_MACRO_END
#else
#define AT91_ETH_DEBUG1(args...) CYG_EMPTY_STATEMENT
#endif

#if CYGPKG_DEVS_ETH_ARM_AT91_DEBUG_LEVEL > 1
#define AT91_ETH_DEBUG2(cond, args...)                  \
  CYG_MACRO_START                                       \
    if ((cond))                                         \
      diag_printf(args);                                \
  CYG_MACRO_END
#else
#define AT91_ETH_DEBUG2(args...) CYG_EMPTY_STATEMENT
#endif

//Driver interface callbacks
#define _eth_drv_init(sc,mac) \
  (sc->funs->eth_drv->init)(sc,(unsigned char *)mac)
#define _eth_drv_tx_done(sc,key,status) \
  (sc->funs->eth_drv->tx_done)(sc,key,status)
#define _eth_drv_recv(sc,len) \
  (sc->funs->eth_drv->recv)(sc,len)

//#ifdef CYGINT_IO_ETH_INT_SUPPORT_REQUIRED		// Commented
static cyg_uint32 at91_eth_isr(cyg_vector_t vector, cyg_addrword_t data);
//#endif

// --------------------------------------------------------------
// RedBoot configuration options for managing ESAs for us

// BUGBUG dkh - Need better MAC address management.

// Decide whether to have redboot config vars for it...
#if defined(CYGSEM_REDBOOT_FLASH_CONFIG) && defined(CYGPKG_REDBOOT_NETWORKING)
  #include <redboot.h>
  #include <flash_config.h>

  #ifdef CYGSEM_DEVS_ETH_ARM_AT91_REDBOOT_HOLDS_ESA_ETH0
  static unsigned char at91_eth0_mac_addr[6] = { CYGPKG_DEVS_ETH_ARM_AT91_MACADDR };

  RedBoot_config_option("Network hardware address [MAC] for eth0",
                        eth0_esa_data,
                        ALWAYS_ENABLED, true,
                        CONFIG_ESA, at91_eth0_mac_addr);
  #endif
#endif  // CYGPKG_REDBOOT_NETWORKING && CYGSEM_REDBOOT_FLASH_CONFIG

// and initialization code to read them
// - independent of whether we are building RedBoot right now:
#ifdef CYGPKG_DEVS_ETH_ARM_AT91_REDBOOT_HOLDS_ESA
  #include <cyg/hal/hal_if.h>

  #ifndef CONFIG_ESA
    #define CONFIG_ESA (6)
  #endif

  #define CYGHWR_DEVS_ETH_ARM_AT91_GET_ESA( mac_address, ok )           \
  CYG_MACRO_START                                                       \
  ok = CYGACC_CALL_IF_FLASH_CFG_OP( CYGNUM_CALL_IF_FLASH_CFG_GET,       \
                                    "eth0_esa_data",                    \
                                    mac_address,                        \
                                    CONFIG_ESA);                        \
  CYG_MACRO_END
#endif // CYGPKG_DEVS_ETH_AT91_ETH_REDBOOT_HOLDS_ESA

//============================================================================

// Private Data structures

#ifndef AT91_EMAC_RX_BUFF_SIZE
#define AT91_EMAC_RX_BUFF_SIZE  128
#endif

// Receive Buffer Descriptor
typedef struct rbd_s
{
  volatile cyg_uint32 addr;
  volatile cyg_uint32 sr;
} rbd_t;

// Receive Buffer
typedef struct rb_s
{
  cyg_uint8 rb[AT91_EMAC_RX_BUFF_SIZE];
} rb_t;

// Transmit Buffer Descriptor
typedef struct tbd_s
{
  cyg_uint32 addr;
  volatile cyg_uint32 sr;
} tbd_t;

// AT91 Ethernet private data
typedef struct at91_eth_priv_s
{
  cyg_uint32 int_vector;
  char *esa_key;        // RedBoot 'key' for device ESA
  cyg_uint8 *enaddr;
  cyg_uint32 base;      // Base address of device
  eth_phy_access_t *phy;
  tbd_t tbd[CYGNUM_DEVS_ETH_ARM_AT91_TX_BUFS] __attribute__ ((aligned (HAL_DCACHE_LINE_SIZE)));
  rbd_t rbd[CYGNUM_DEVS_ETH_ARM_AT91_RX_BUFS] __attribute__ ((aligned (HAL_DCACHE_LINE_SIZE)));
  rb_t  rb[CYGNUM_DEVS_ETH_ARM_AT91_RX_BUFS] __attribute__ ((aligned (HAL_DCACHE_LINE_SIZE)));
  unsigned long tx_keys[CYGNUM_DEVS_ETH_ARM_AT91_TX_BUFS];
  cyg_uint32 isr;
  cyg_uint32 tx_head_idx;
  cyg_uint32 tx_tail_idx;
  cyg_uint32 tx_free;
  cyg_uint32 tx_max_frame_sglen;
  cyg_bool tx_busy;
  cyg_uint32 rx_head_idx;
  cyg_uint32 rx_tail_idx;
  cyg_uint32 rx_frame_cnt;
//#ifdef CYGINT_IO_ETH_INT_SUPPORT_REQUIRED		// Commented
  cyg_interrupt intr;
  cyg_handle_t  intr_handle;
//#endif
} at91_eth_priv_t;

// AT91 multicast hash mask (64-bit hash in two 32-bit words)
typedef struct mc_hash_mask_s
{
  cyg_uint32 hi;
  cyg_uint32 lo;
} at91_mc_hash_mask_t;

//============================================================================

// Private Data

// Hash masks for multicast MAC addresses on AT91 EMAC.
static at91_mc_hash_mask_t at91_mc_hash_masks[] =
{
//  high word   low word
  { 0x00000410, 0x41041041 },
  { 0x00000820, 0x82082082 },
  { 0x00001041, 0x04104104 },
  { 0x00002082, 0x08208208 },
  { 0x00004104, 0x10410410 },
  { 0x00008208, 0x20820820 }
};

// Exegin specific. Registered callbacks.
static void    (*at91_eth_rx_callback_func)(void) = NULL;
static void    (*at91_eth_tx_callback_func)(void) = NULL;


//============================================================================
// Exegin specific. Register callbacks.
//

void
at91_eth_register_rx_callback(void (*rx_callback_func)(void))
{
  at91_eth_rx_callback_func = rx_callback_func;
} /* at91_eth_register_rx_callback */

void
at91_eth_register_tx_callback(void (*tx_callback_func)(void))
{
  at91_eth_tx_callback_func = tx_callback_func;
} /* at91_eth_register_tx_callback */

//============================================================================
// PHY access bits and pieces
//

static void
at91_mdio_enable(void)
{
  cyg_uint32 val;
  HAL_READ_UINT32(AT91_EMAC + AT91_EMAC_NCR, val);
  val |= AT91_EMAC_NCR_MPE;    /* enable management port */
  HAL_WRITE_UINT32(AT91_EMAC + AT91_EMAC_NCR, val);
}

static void
at91_mdio_disable(void)
{
  cyg_uint32 val;
  HAL_READ_UINT32(AT91_EMAC + AT91_EMAC_NCR, val);
  val &= ~AT91_EMAC_NCR_MPE;    /* disable management port */
  HAL_WRITE_UINT32(AT91_EMAC + AT91_EMAC_NCR, val);
}

// Write one of the PHY registers via the MII bus
static void
at91_write_phy(int reg_addr, int phy_addr, unsigned short data)
{
  cyg_uint32 val;

  CYG_ASSERTC(reg_addr >= 0 && reg_addr <= AT91_EMAC_MAN_REGA_MASK);
  CYG_ASSERTC(phy_addr >= 0 && phy_addr <= AT91_EMAC_MAN_PHY_MASK);

  val = (AT91_EMAC_MAN_SOF  |
         AT91_EMAC_MAN_WR   |
         AT91_EMAC_MAN_CODE |
         AT91_EMAC_MAN_PHYA(phy_addr) |
         AT91_EMAC_MAN_REGA(reg_addr) |
         AT91_EMAC_MAN_DATA(data));

  HAL_WRITE_UINT32(AT91_EMAC + AT91_EMAC_MAN, val);

  // Wait for operation to complete
  do {
    HAL_READ_UINT32((AT91_EMAC + AT91_EMAC_NSR), val);
  } while(!(val & AT91_EMAC_NSR_IDLE));
}

// Read one of the PHY registers via the MII bus
static bool
at91_read_phy(int reg_addr, int phy_addr, unsigned short *data)
{
  cyg_uint32 val;

  CYG_ASSERTC(reg_addr >= 0 && reg_addr <= AT91_EMAC_MAN_REGA_MASK);
  CYG_ASSERTC(phy_addr >= 0 && phy_addr <= AT91_EMAC_MAN_PHY_MASK);

  val = (AT91_EMAC_MAN_SOF  |
         AT91_EMAC_MAN_RD   |
         AT91_EMAC_MAN_CODE |
         AT91_EMAC_MAN_PHYA(phy_addr) |
         AT91_EMAC_MAN_REGA(reg_addr));


  HAL_WRITE_UINT32(AT91_EMAC + AT91_EMAC_MAN, val);

  // Wait for operation to complete
  do {
    HAL_READ_UINT32((AT91_EMAC + AT91_EMAC_NSR), val);
  } while(!(val & AT91_EMAC_NSR_IDLE));

  HAL_READ_UINT32(AT91_EMAC + AT91_EMAC_MAN, val);
  *data = val & AT91_EMAC_MAN_DATA_MASK;

  return (true);
}

/* Exegin specific. */
/* Give public names to the phy r/w functions for the Q5x firmware. */
void
phy_write(int reg_addr, int phy_addr, unsigned short data)
{
  at91_write_phy(reg_addr, phy_addr, data);
}

bool
phy_read(int reg_addr, int phy_addr, unsigned short *data)
{
  return at91_read_phy(reg_addr, phy_addr, data);
}

// Enable the MDIO bit in MAC control register so that we can talk to
// the PHY. Also set the clock divider so that MDC is less than 2.5MHz.
static void
at91_init_phy(void)
{
  cyg_uint32 cfg;
  cyg_uint32 div;

  HAL_READ_UINT32(AT91_EMAC + AT91_EMAC_NCFG, cfg);
  cfg &= ~AT91_EMAC_NCFG_CLK_MASK;

  div = (CYGNUM_HAL_ARM_AT91_CLOCK_SPEED / 2500000);
  if (div < 8)
    cfg |= AT91_EMAC_NCFG_CLK_HCLK_8;
  else if (div < 16)
    cfg |= AT91_EMAC_NCFG_CLK_HCLK_16;
  else if (div < 32)
    cfg |= AT91_EMAC_NCFG_CLK_HCLK_32;
  else if (div < 64)
    cfg |= AT91_EMAC_NCFG_CLK_HCLK_64;
  else
    CYG_FAIL("Unable to program MII clock");

  HAL_WRITE_UINT32(AT91_EMAC + AT91_EMAC_NCFG, cfg);
}

ETH_PHY_REG_LEVEL_ACCESS_FUNS(at91_phy,
                              at91_init_phy,
                              NULL,
                              at91_write_phy,
                              at91_read_phy);


//======================================================================
// Transmit and Receive buffer-descriptor initialization.

// Initialize the receiver buffer descriptors.
static void
at91_rb_init(at91_eth_priv_t *priv)
{
  int i;
  rbd_t *rbd = (rbd_t *)CYGARC_UNCACHED_ADDRESS(priv->rbd);

  for (i = 0 ; i < CYGNUM_DEVS_ETH_ARM_AT91_RX_BUFS; i++) {
    rbd[i].addr = ((cyg_uint32)&priv->rb[i]) & AT91_EMAC_RBD_ADDR_MASK;
    rbd[i].sr = 0;
  }
  // Set the wrap bit on the last entry.
  rbd[CYGNUM_DEVS_ETH_ARM_AT91_RX_BUFS - 1].addr |= AT91_EMAC_RBD_ADDR_WRAP;
}

// Initialize the transmit buffer descriptors.
static void
at91_tb_init(at91_eth_priv_t *priv)
{
  int i;
  tbd_t *tbd = (tbd_t *)CYGARC_UNCACHED_ADDRESS(priv->tbd);

  for (i = 0 ; i < CYGNUM_DEVS_ETH_ARM_AT91_TX_BUFS; i++) {
    tbd[i].addr = 0;
    tbd[i].sr = AT91_EMAC_TBD_SR_USED;
  }
  // Set the wrap bit on the last entry.
  tbd[CYGNUM_DEVS_ETH_ARM_AT91_TX_BUFS - 1].sr |= AT91_EMAC_TBD_SR_WRAP;
}


//======================================================================
// Enable and Disable of the receiver and transmitter.

static void
at91_disable_rx(at91_eth_priv_t *priv)
{
  cyg_uint32 ctl;

  HAL_READ_UINT32(priv->base + AT91_EMAC_NCR, ctl);
  ctl &= ~AT91_EMAC_NCR_RE;
  HAL_WRITE_UINT32(priv->base + AT91_EMAC_NCR, ctl);
}

static void
at91_disable_tx(at91_eth_priv_t *priv)
{
  cyg_uint32 reg_val;

  // Stop transmitter.
  HAL_READ_UINT32(priv->base + AT91_EMAC_NCR, reg_val);
  reg_val &= ~AT91_EMAC_NCR_TX;
  reg_val |= AT91_EMAC_NCR_THALT;
  HAL_WRITE_UINT32(priv->base + AT91_EMAC_NCR, reg_val);

  // Wait for transmitter to stop.
  do {
    HAL_READ_UINT32(priv->base + AT91_EMAC_TSR, reg_val);
  } while (reg_val & AT91_EMAC_TSR_TXIDLE);
}

static void
at91_enable_rx(at91_eth_priv_t *priv)
{
  cyg_uint32 ctl;

  HAL_READ_UINT32(priv->base + AT91_EMAC_NCR, ctl);
  ctl |= AT91_EMAC_NCR_RE;
  HAL_WRITE_UINT32(priv->base + AT91_EMAC_NCR, ctl);
}

static void
at91_enable_tx(at91_eth_priv_t *priv)
{
  cyg_uint32 ctl;

  HAL_READ_UINT32(priv->base + AT91_EMAC_NCR, ctl);
  if (!(ctl & AT91_EMAC_NCR_TX)) {
    priv->tx_free = CYGNUM_DEVS_ETH_ARM_AT91_TX_BUFS - 1;
    priv->tx_head_idx = priv->tx_tail_idx = 0;
    at91_tb_init(priv);
    ctl |= AT91_EMAC_NCR_TX;
    ctl &= ~AT91_EMAC_NCR_THALT;
    HAL_WRITE_UINT32(priv->base + AT91_EMAC_NCR, ctl);
  }
}

static void
at91_enable(at91_eth_priv_t *priv)
{
  at91_enable_tx(priv);
  at91_enable_rx(priv);
}

static void
at91_disable(at91_eth_priv_t *priv)
{
  at91_disable_tx(priv);
  at91_disable_rx(priv);
}

static void
at91_start_transmitter(at91_eth_priv_t *priv)
{
  cyg_uint32 ctl;

  HAL_READ_UINT32(priv->base + AT91_EMAC_NCR, ctl);
  ctl |= AT91_EMAC_NCR_TSTART;
  HAL_WRITE_UINT32(priv->base + AT91_EMAC_NCR, ctl);
}


//======================================================================
// Initialization code

// Configure the pins so that the EMAC has control of them.
static void
at91_cfg_pins(void)
{
#ifdef CYGHWR_DEV_ETH_ARM_AT91_USE_RMII
  // RMII doesn't need all the EMAC IO pins
  HAL_ARM_AT91_PIO_CFG(AT91_EMAC_EREFCK);

  HAL_ARM_AT91_PIO_CFG(AT91_EMAC_ERXDV);
  HAL_ARM_AT91_PIO_CFG(AT91_EMAC_ERX0);
  HAL_ARM_AT91_PIO_CFG(AT91_EMAC_ERX1);
  HAL_ARM_AT91_PIO_CFG(AT91_EMAC_ERXER);

  HAL_ARM_AT91_PIO_CFG(AT91_EMAC_ETXEN);
  HAL_ARM_AT91_PIO_CFG(AT91_EMAC_ETX0);
  HAL_ARM_AT91_PIO_CFG(AT91_EMAC_ETX1);

  HAL_ARM_AT91_PIO_CFG(AT91_EMAC_EMDC);
  HAL_ARM_AT91_PIO_CFG(AT91_EMAC_EMDIO);
#else
  HAL_ARM_AT91_PIO_CFG(AT91_EMAC_EREFCK);
  HAL_ARM_AT91_PIO_CFG(AT91_EMAC_ECRS);
  HAL_ARM_AT91_PIO_CFG(AT91_EMAC_ECOL);

  HAL_ARM_AT91_PIO_CFG(AT91_EMAC_ERXDV);
  HAL_ARM_AT91_PIO_CFG(AT91_EMAC_ERX0);
  HAL_ARM_AT91_PIO_CFG(AT91_EMAC_ERX1);
  HAL_ARM_AT91_PIO_CFG(AT91_EMAC_ERX2);
  HAL_ARM_AT91_PIO_CFG(AT91_EMAC_ERX3);
  HAL_ARM_AT91_PIO_CFG(AT91_EMAC_ERXER);
  HAL_ARM_AT91_PIO_CFG(AT91_EMAC_ERXCK);

  HAL_ARM_AT91_PIO_CFG(AT91_EMAC_ETXEN);
  HAL_ARM_AT91_PIO_CFG(AT91_EMAC_ETX0);
  HAL_ARM_AT91_PIO_CFG(AT91_EMAC_ETX1);
  HAL_ARM_AT91_PIO_CFG(AT91_EMAC_ETX2);
  HAL_ARM_AT91_PIO_CFG(AT91_EMAC_ETX3);
  HAL_ARM_AT91_PIO_CFG(AT91_EMAC_ETXER);

  HAL_ARM_AT91_PIO_CFG(AT91_EMAC_EMDC);
  HAL_ARM_AT91_PIO_CFG(AT91_EMAC_EMDIO);
#endif
}

// Set a specific-address match register to a given MAC address.
// Packets received which match this address will be passed on.
static void
at91_set_mac(at91_eth_priv_t *priv, cyg_uint8 *mac_addr, int sa)
{
  cyg_uint32 mac_hi, mac_lo;

  // Adjust specific-address register index.
  CYG_ASSERTC(sa > 0 && sa < 5);
  sa--;

  // Split MAC address into two words.
  mac_lo = (mac_addr[3] << 24) |
           (mac_addr[2] << 16) |
           (mac_addr[1] <<  8) |
            mac_addr[0];

  mac_hi = (mac_addr[5] <<  8) |
            mac_addr[4];

  // Write MAC address to the specific-address register.
  HAL_WRITE_UINT32(priv->base + AT91_EMAC_SA1L + (8 * sa), mac_lo);
  HAL_WRITE_UINT32(priv->base + AT91_EMAC_SA1H + (8 * sa), mac_hi);
}

// Compute number of bits in word (aka population count).
static int
at91_mc_bitcnt(cyg_uint32 bits)
{
  bits -= (bits >> 1) & 0x55555555;
  bits = ((bits >> 2) & 0x33333333) + (bits & 0x33333333);
  bits = ((bits >> 4) + bits) & 0x0f0f0f0f;
  bits += bits >> 8;
  bits += bits >> 16;

  return bits & 0x3f;
}

// Add a multicast MAC address to the hardware and enable hash matching.
// Packets received which match these addresses will be passed on.
static void
at91_add_mc_mac(at91_eth_priv_t * priv, cyg_uint8 * mc_addr)
{
  cyg_uint32 t, mc_lo, mc_hi, hash_lo, hash_hi;
  int i, bit_idx, bit_cnt;

  // Split multicast mac address into two words.
  mc_lo = (mc_addr[3] << 24) |
          (mc_addr[2] << 16) |
          (mc_addr[1] <<  8) |
           mc_addr[0];

  mc_hi = (mc_addr[5] << 8) |
           mc_addr[4];

  // Build the 6-bit hash_bit index.
  bit_idx = 0;
  for (i = 0; i < 6; i++) {
    bit_cnt =  at91_mc_bitcnt(mc_lo & at91_mc_hash_masks[i].lo);
    bit_cnt += at91_mc_bitcnt(mc_hi & at91_mc_hash_masks[i].hi);

    bit_idx |= (bit_cnt & 1) << i;
  }

  // Read the current 64-bit hardware hash value.
  HAL_READ_UINT32(priv->base + AT91_EMAC_HRB, hash_lo);
  HAL_READ_UINT32(priv->base + AT91_EMAC_HRT, hash_hi);

  // Set a single bit in the hash value according to the index.
  if (bit_idx > 31)
    hash_hi |= 1 << (bit_idx - 32);
  else
    hash_lo |= 1 << bit_idx;

  // Write the new 64-bit hardware hash value.
  HAL_WRITE_UINT32(priv->base + AT91_EMAC_HRB, hash_lo);
  HAL_WRITE_UINT32(priv->base + AT91_EMAC_HRT, hash_hi);

  // Enable hardware to do multicast hash matching.
  HAL_READ_UINT32(priv->base + AT91_EMAC_NCFG, t);
  t |= AT91_EMAC_NCFG_MTI;
  HAL_WRITE_UINT32(priv->base + AT91_EMAC_NCFG, t);
}

// Remove a multicast MAC address from the hardware and enable hash matching.
// Packets received which match these addresses will not be passed on.
static void
at91_rem_mc_mac(at91_eth_priv_t * priv, cyg_uint8 * mc_addr)		// Added function (Remove multicast MAC address)
{
  cyg_uint32 t, mc_lo, mc_hi, hash_lo, hash_hi;
  int i, bit_idx, bit_cnt;

  // Split multicast mac address into two words.
  mc_lo = (mc_addr[3] << 24) |
          (mc_addr[2] << 16) |
          (mc_addr[1] <<  8) |
           mc_addr[0];

  mc_hi = (mc_addr[5] << 8) |
           mc_addr[4];

  // Build the 6-bit hash_bit index.
  bit_idx = 0;
  for (i = 0; i < 6; i++) {
    bit_cnt =  at91_mc_bitcnt(mc_lo & at91_mc_hash_masks[i].lo);
    bit_cnt += at91_mc_bitcnt(mc_hi & at91_mc_hash_masks[i].hi);

    bit_idx |= (bit_cnt & 1) << i;
  }

  // Read the current 64-bit hardware hash value.
  HAL_READ_UINT32(priv->base + AT91_EMAC_HRB, hash_lo);
  HAL_READ_UINT32(priv->base + AT91_EMAC_HRT, hash_hi);

  // Set a single bit in the hash value according to the index.
  if (bit_idx > 31)
    hash_hi &= 0 << (bit_idx - 32);
  else
    hash_lo &= 0 << bit_idx;

  // Write the new 64-bit hardware hash value.
  HAL_WRITE_UINT32(priv->base + AT91_EMAC_HRB, hash_lo);
  HAL_WRITE_UINT32(priv->base + AT91_EMAC_HRT, hash_hi);

  // Enable hardware to do multicast hash matching.
  HAL_READ_UINT32(priv->base + AT91_EMAC_NCFG, t);
  t |= AT91_EMAC_NCFG_MTI;
  HAL_WRITE_UINT32(priv->base + AT91_EMAC_NCFG, t);
}

static void
at91_clear_stats(at91_eth_priv_t *priv)
{
  cyg_uint32 ctl;

  HAL_READ_UINT32(priv->base + AT91_EMAC_NCR, ctl);
  ctl |= AT91_EMAC_NCR_CSR;
  HAL_WRITE_UINT32(priv->base + AT91_EMAC_NCR, ctl);
}

// Initialize and configure the interface for use.
// Interrupts are grabbed, etc. This means the start function has
// little to do except enable the receiver
static bool
at91_eth_init(struct cyg_netdevtab_entry *tab)
{
  struct eth_drv_sc *sc = (struct eth_drv_sc *)tab->device_instance;
  at91_eth_priv_t *priv = (at91_eth_priv_t *)sc->driver_private;
  unsigned char mac_addr[6] = { CYGPKG_DEVS_ETH_ARM_AT91_MACADDR };
  unsigned char mac_addr_none[6] = { 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 };
  unsigned short phy_state = 0;
  cyg_uint32 ncfg = 0;
#ifdef CYGHWR_DEVS_ETH_ARM_AT91_GET_ESA
  bool esa_ok = false;
#endif

  AT91_ETH_DEBUG1(true, "AT91_ETH: Initialising at 0x%08x\n", priv->base);

  /* Disable all the interrupts for the moment            */
  /* The Start function actually enables all that we need */
  HAL_WRITE_UINT32(priv->base + AT91_EMAC_IDR, 0x3FFF);

  at91_disable(priv);

  priv->tx_free = CYGNUM_DEVS_ETH_ARM_AT91_TX_BUFS - 1;
  priv->tx_head_idx = priv->tx_tail_idx = 0;
  priv->tx_max_frame_sglen = 3;
  priv->rx_head_idx = priv->rx_tail_idx = 0;

  // Enable the clock to the EMAC
  HAL_WRITE_UINT32(AT91_PMC + AT91_PMC_PCER, AT91_PMC_PCER_EMAC);
  HAL_WRITE_UINT32(AT91_PMC + AT91_PMC_PCER, AT91_PMC_PCER_PIOB);
  HAL_WRITE_UINT32(AT91_PMC + AT91_PMC_PCER, AT91_PMC_PCER_PIOC);

  at91_cfg_pins();

  /* Enable IO clock and either MII or RMII interface */
#ifdef CYGHWR_DEV_ETH_ARM_AT91_USE_RMII
  HAL_WRITE_UINT32(priv->base+AT91_EMAC_USRIO,
                   AT91_EMAC_USRIO_CLKEN | AT91_EMAC_USRIO_RMII);
#else
  HAL_WRITE_UINT32(priv->base+AT91_EMAC_USRIO, AT91_EMAC_USRIO_CLKEN);
#endif

  // If we are building an interrupt enabled version,
  // then install the interrupt handler
//#ifdef CYGINT_IO_ETH_INT_SUPPORT_REQUIRED		// Commented
  AT91_ETH_DEBUG1(true, "AT91_ETH: Installing Interrupts on IRQ %d\n",
                  priv->int_vector);
  cyg_drv_interrupt_create(priv->int_vector,
                           4,
                           (cyg_addrword_t)sc,
                           at91_eth_isr,
                           eth_drv_dsr,
                           &priv->intr_handle,
                           &priv->intr);

  cyg_drv_interrupt_configure(priv->int_vector, false, false);
  cyg_drv_interrupt_attach(priv->intr_handle);
  cyg_drv_interrupt_unmask(priv->int_vector);				// !!! Warning	!!!	// FIXME unmask
//#endif

#ifdef CYGHWR_DEVS_ETH_ARM_AT91_GET_ESA
  // Get MAC address from RedBoot configuration variables.
  CYGHWR_DEVS_ETH_ARM_AT91_GET_ESA(&mac_addr[0], esa_ok);
  // If that fails, then MAC address is unchanged and the
  // MAC address from CDL is used.
    AT91_ETH_DEBUG1(esa_ok, "AT91_ETH: Warning! ESA unknown\n");
#endif

  AT91_ETH_DEBUG1(true, "AT91_ETH: MAC addr: %02x:%02x:%02x:%02x:%02x:%02x\n",
                  mac_addr[0], mac_addr[1], mac_addr[2],
                  mac_addr[3], mac_addr[4], mac_addr[5]);

  // Give the EMAC its MAC address
  at91_set_mac(priv, mac_addr, 1);
  at91_set_mac(priv, mac_addr_none, 2);
  at91_set_mac(priv, mac_addr_none, 3);
  at91_set_mac(priv, mac_addr_none, 4);

  // Setup the receiver buffer descriptors and tell EMAC where they are
  at91_rb_init(priv);
  HAL_WRITE_UINT32(priv->base + AT91_EMAC_RBQP, (cyg_uint32)priv->rbd);

  // Setup the transmit buffer descriptors and tell EMAC where they are
  at91_tb_init(priv);
  HAL_WRITE_UINT32(priv->base + AT91_EMAC_TBQP, (cyg_uint32)priv->tbd);

  // Setup the PHY
  CYG_ASSERTC(priv->phy);
  at91_mdio_enable();
  if (!_eth_phy_init(priv->phy)) {
    at91_mdio_disable();
    return (false);
  }

  // Set EMAC network configuration to match PHY state
  HAL_READ_UINT32(priv->base + AT91_EMAC_NCFG, ncfg);

  phy_state = _eth_phy_state(priv->phy);
//    at91_mdio_disable();
  AT91_ETH_DEBUG1(!(phy_state & ETH_PHY_STAT_LINK), "AT91_ETH: No Link\n");
  if ((phy_state & ETH_PHY_STAT_LINK) != 0) {
    if (((phy_state & ETH_PHY_STAT_100MB) != 0)) {
      AT91_ETH_DEBUG1(true, "AT91_ETH: 100Mbs");
      ncfg |= AT91_EMAC_NCFG_SPD_100Mbps;
    }
    else {
      AT91_ETH_DEBUG1(true, "AT91_ETH: 10Mbps");
      ncfg &= ~(AT91_EMAC_NCFG_SPD_100Mbps);
    }

    if ((phy_state & ETH_PHY_STAT_FDX)) {
      AT91_ETH_DEBUG1(true, " Full Duplex\n");
      ncfg |= AT91_EMAC_NCFG_FD;
    }
    else {
      AT91_ETH_DEBUG1(true, " Half Duplex\n");
      ncfg &= ~(AT91_EMAC_NCFG_FD);
    }
  }
  else
  {
    ncfg |= (AT91_EMAC_NCFG_SPD_100Mbps | AT91_EMAC_NCFG_FD);
  }

  // Write the network-configuration register
  ncfg |= (AT91_EMAC_NCFG_RLCE);
  HAL_WRITE_UINT32(priv->base + AT91_EMAC_NCFG, ncfg);

  // Clear the statistics counters
  at91_clear_stats(priv);

  /* Clear the EMAC status registers */
  HAL_READ_UINT32(priv->base + AT91_EMAC_ISR, ncfg);
  HAL_READ_UINT32(priv->base + AT91_EMAC_RSR, ncfg);
  HAL_WRITE_UINT32(priv->base + AT91_EMAC_RSR, ncfg);
  HAL_READ_UINT32(priv->base + AT91_EMAC_TSR, ncfg);
  HAL_WRITE_UINT32(priv->base + AT91_EMAC_TSR, ncfg);

  // Initialize the upper layer driver
  _eth_drv_init(sc, mac_addr);

  return (true);
}

// This function is called to stop the interface.
static void
at91_eth_stop(struct eth_drv_sc *sc)
{
  at91_eth_priv_t *priv = (at91_eth_priv_t *)sc->driver_private;

//#ifdef CYGINT_IO_ETH_INT_SUPPORT_REQUIRED		// Commented
  HAL_WRITE_UINT32(priv->base + AT91_EMAC_IDR, AT91_ETH_INT_MASK);
//#endif

  at91_disable(priv);
}

// This function is called to "start up" the interface. It may be called
// multiple times, even when the hardware is already running.
static void
at91_eth_start(struct eth_drv_sc *sc, unsigned char *mac_addr, int flags)
{
  at91_eth_priv_t *priv = (at91_eth_priv_t *)sc->driver_private;

  // Enable the receiver and transmitter
  at91_enable(priv);

//#ifdef CYGINT_IO_ETH_INT_SUPPORT_REQUIRED
  HAL_WRITE_UINT32(priv->base + AT91_EMAC_IER, AT91_ETH_INT_MASK);
//#endif
}

// This function is called for low level "control" operations
static int
at91_eth_control(struct eth_drv_sc *sc, unsigned long key,
                 void *data, int length)
{
  at91_eth_priv_t *priv = (at91_eth_priv_t *)sc->driver_private;

  switch (key) {
    // Set Ethernet MAC address
    case ETH_DRV_SET_MAC_ADDRESS:
    {
      cyg_uint8 *mac_addr = (cyg_uint8 *)data;

      if (mac_addr == NULL || length < ETHER_ADDR_LEN)
        return 1;

      AT91_ETH_DEBUG1(true, "AT91_ETH: set MAC %02x:%02x:%02x:%02x:%02x:%02x\n",
                      mac_addr[0], mac_addr[1], mac_addr[2],
                      mac_addr[3], mac_addr[4], mac_addr[5]);

      at91_eth_stop(sc);
      at91_set_mac(priv, mac_addr, 1);
      at91_eth_start(sc, mac_addr, 0);
      return 0;
    }

#ifdef ETH_DRV_GET_MAC_ADDRESS						// Added
    // Get Ethernet MAC address from hardware
    case ETH_DRV_GET_MAC_ADDRESS:
    {
      cyg_uint32 mac_hi, mac_lo;
      cyg_uint8 *mac_addr = (cyg_uint8 *)data;

      if (mac_addr == NULL || length < ETHER_ADDR_LEN)
        return 1;

      HAL_READ_UINT32(priv->base + AT91_EMAC_SA1L, mac_lo);
      HAL_READ_UINT32(priv->base + AT91_EMAC_SA1H, mac_hi);

      mac_addr[0] =         mac_lo & 0xFF;
      mac_addr[1] = (mac_lo >>  8) & 0xFF;
      mac_addr[2] = (mac_lo >> 16) & 0xFF;
      mac_addr[3] = (mac_lo >> 24) & 0xFF;
      mac_addr[4] =         mac_hi & 0xFF;
      mac_addr[5] = (mac_hi >>  8) & 0xFF;

      return 0;
    }
#endif

    // Set specified multicast MAC addresses or clear all
    case ETH_DRV_SET_MC_LIST:
    {
      int mc_cnt;
      cyg_uint32 t;
      struct eth_drv_mc_list *mc_list = (struct eth_drv_mc_list *)data;

      if (mc_list == NULL || length != sizeof(struct eth_drv_mc_list))
        return 1;

      mc_cnt = mc_list->len;
      if (mc_cnt > 0) {
        // Enable recognition of each multicast MAC address in list.
        while (mc_cnt) {
          at91_add_mc_mac(priv, &mc_list->addrs[--mc_cnt][0]);
          AT91_ETH_DEBUG1(true, "AT91_ETH: add multicast MAC "
                                "%02x:%02x:%02x:%02x:%02x:%02x\n",
                          mc_list->addrs[mc_cnt][0],
                          mc_list->addrs[mc_cnt][1],
                          mc_list->addrs[mc_cnt][2],
                          mc_list->addrs[mc_cnt][3],
                          mc_list->addrs[mc_cnt][4],
                          mc_list->addrs[mc_cnt][5]);
        }
      }
      else {
        // Disable recognition of all multicast addresses.
        AT91_ETH_DEBUG1(true, "AT91_ETH: disable all multicast MACs\n");
        HAL_WRITE_UINT32(priv->base + AT91_EMAC_HRB, 0);
        HAL_WRITE_UINT32(priv->base + AT91_EMAC_HRT, 0);
        HAL_READ_UINT32(priv->base + AT91_EMAC_NCFG, t);
        t &= ~AT91_EMAC_NCFG_MTI;
        HAL_WRITE_UINT32(priv->base + AT91_EMAC_NCFG, t);
      }

      return 0;
    }

    // Enable all multicast MAC addresses
    case ETH_DRV_SET_MC_ALL:
    {
      cyg_uint32 t;

      AT91_ETH_DEBUG1(true, "AT91_ETH: enable all multicast MACs\n");
      HAL_WRITE_UINT32(priv->base + AT91_EMAC_HRB, 0xFFFFFFFF);
      HAL_WRITE_UINT32(priv->base + AT91_EMAC_HRT, 0xFFFFFFFF);
      HAL_READ_UINT32(priv->base + AT91_EMAC_NCFG, t);
      t |= AT91_EMAC_NCFG_MTI;
      HAL_WRITE_UINT32(priv->base + AT91_EMAC_NCFG, t);

      return 0;
    }
    case ETH_DRV_ADD_MC:										// Added
    {
        cyg_uint8 *mac_addr = (cyg_uint8 *)data;

        if (mac_addr == NULL || length < ETHER_ADDR_LEN)
          return 1;

    	at91_add_mc_mac(sc->driver_private, mac_addr);
    	return 0;
    }
    case ETH_DRV_REM_MC:										// Added
    {
        cyg_uint8 *mac_addr = (cyg_uint8 *)data;

        if (mac_addr == NULL || length < ETHER_ADDR_LEN)
          return 1;

    	at91_rem_mc_mac(sc->driver_private, mac_addr);
    	return 0;
    }
    // Unknown or unhandled key
    default:
    {
      AT91_ETH_DEBUG1(true, "AT91_ETH: unknown IOCTL key %lx\n", key);
      return 1;
    }
  } // switch
}

// This function is called to see if another packet can be sent.
// It should return the number of packets which can be handled.
// Zero should be returned if the interface is busy and can not send
// any more.
//
// We allocate one buffer descriptor per scatter/gather entry. We start
// with the assumuption that a typical packet will not have more than
// three such entries, but we keep track of the largest number of entries
// ever used and compute the return value based on that.
static int
at91_eth_can_send(struct eth_drv_sc *sc)
{
  at91_eth_priv_t *priv = (at91_eth_priv_t *)sc->driver_private;

  return priv->tx_free >= priv->tx_max_frame_sglen;
}


// This routine is called to send data to the hardware
static int
at91_eth_send(struct eth_drv_sc *sc, struct eth_drv_sg *sg_list, int sg_len,
              int total_len, unsigned long key)
{
  at91_eth_priv_t *priv = (at91_eth_priv_t *)sc->driver_private;
  int i;
  cyg_uint32 sr, head_idx;
  tbd_t *tbd;

  // Keep track of the largest number of scatter/gather entries
  // ever used by a transmitted packet.
  if (sg_len > priv->tx_max_frame_sglen) {
    priv->tx_max_frame_sglen = sg_len;
    AT91_ETH_DEBUG1(true, "AT91_ETH: Tx sg_len increased to %d\n", sg_len);		// Commented
  }

  if (sg_len > priv->tx_free) {
    AT91_ETH_DEBUG1(true, "AT91_ETH: not enough free tx descriptors\n");		// Commented
//    (sc->funs->eth_drv->tx_done)(sc, key, 1);
    return 0;
//	  return;
  }

  // Disable transmit interrupts.
  cyg_drv_dsr_lock();

  head_idx = priv->tx_head_idx;
  for(i = 0; i < sg_len; i++) {
    HAL_DCACHE_STORE(sg_list[i].buf, sg_list[i].len);
    HAL_MEMORY_BARRIER();
    sr = sg_list[i].len & AT91_EMAC_TBD_SR_LEN_MASK;

    // If this is the first buffer in the frame, then keep USED bit on for now.
    if (i == 0)
      sr |= AT91_EMAC_TBD_SR_USED;

    // If this is the last buffer in the frame, then set EOF bit.
    // Also save the eCos key so we can return status to eCos at tx completion.
    if (i == (sg_len - 1)) {
      sr |= AT91_EMAC_TBD_SR_EOF;
      priv->tx_keys[head_idx] = key;
    }

    // If this is the last buffer in the ring, then set the WRAP bit.
    if (head_idx == (CYGNUM_DEVS_ETH_ARM_AT91_TX_BUFS - 1))
      sr |= AT91_EMAC_TBD_SR_WRAP;

    // Write buffer address and status to EMAC descriptor.
    tbd = (tbd_t *)CYGARC_UNCACHED_ADDRESS(&priv->tbd[head_idx]);
    tbd->addr = sg_list[i].buf;
    tbd->sr = sr;
    AT91_ETH_NEXT_TBD(head_idx);
  }

  // Release the frame to the EMAC
  priv->tx_free -= sg_len;
  tbd = (tbd_t *)CYGARC_UNCACHED_ADDRESS(&priv->tbd[priv->tx_head_idx]);
  tbd->sr &= ~AT91_EMAC_TBD_SR_USED;
  priv->tx_head_idx = head_idx;

  // Enable transmit interrupts and start transmit.
  at91_start_transmitter(priv);
  cyg_drv_dsr_unlock();
  return 1;
}


//======================================================================

//#ifdef CYGINT_IO_ETH_INT_SUPPORT_REQUIRED		// Commented
static cyg_uint32
at91_eth_isr (cyg_vector_t vector, cyg_addrword_t data)
{
  struct eth_drv_sc *sc = (struct eth_drv_sc *)data;
  at91_eth_priv_t *priv = (at91_eth_priv_t *)sc->driver_private;

  cyg_drv_interrupt_mask(priv->int_vector);
  cyg_drv_interrupt_acknowledge(priv->int_vector);
  return CYG_ISR_HANDLED | CYG_ISR_CALL_DSR;
}
//#endif


/* Handle transmit-complete events. */
static void
at91_eth_tx(struct eth_drv_sc *sc)
{
  at91_eth_priv_t *priv = (at91_eth_priv_t *)sc->driver_private;
  cyg_uint32 tsr, sr;
  cyg_uint32 processed_buffers;
  cyg_bool tx_err, is_sof;
  tbd_t *tbd;

  // Get the Transmit Status and clear it
  HAL_READ_UINT32(priv->base + AT91_EMAC_TSR, tsr);
  HAL_WRITE_UINT32(priv->base + AT91_EMAC_TSR, tsr);

  AT91_ETH_DEBUG1(true, "AT91_ETH: TxStat Reg = 0x%08x\n", tsr);
  AT91_ETH_DEBUG1(tsr & AT91_EMAC_TSR_OVR, "Tx_UBR ");
  AT91_ETH_DEBUG1(tsr & AT91_EMAC_TSR_COL, "Tx_COL ");
  AT91_ETH_DEBUG2((tsr & AT91_EMAC_TSR_TXIDLE) == 0, "Tx_IDLE ");
  AT91_ETH_DEBUG1(tsr & AT91_EMAC_TSR_RLE, "Tx_RLE ");
  AT91_ETH_DEBUG1(tsr & AT91_EMAC_TSR_BNQ, "Tx_BEX ");
  AT91_ETH_DEBUG1(tsr & AT91_EMAC_TSR_UND, "Tx_UND ");
  AT91_ETH_DEBUG1(tsr & AT91_EMAC_TSR_COMP, "Tx_COMP ");
  AT91_ETH_DEBUG1(true, "\n");

  // Only do work if the transmit-status register indicates work to do.
  if (tsr & (AT91_EMAC_TSR_COMP | AT91_EMAC_TSR_RLE |
             AT91_EMAC_TSR_BNQ | AT91_EMAC_TSR_UND)) {

    if (tsr & (AT91_EMAC_TSR_RLE | AT91_EMAC_TSR_BNQ | AT91_EMAC_TSR_UND))
      tx_err = true;
    else
      tx_err = false;

    priv->tx_busy = true;
    processed_buffers = 0;
    is_sof = true;

    /* For each possible outstanding descriptor. */
    while (priv->tx_tail_idx != priv->tx_head_idx) {
      tbd = (tbd_t *)CYGARC_UNCACHED_ADDRESS(&priv->tbd[priv->tx_tail_idx]);
      sr = tbd->sr;
      AT91_ETH_DEBUG2(true, "AT91_ETH: TX buffer status at %d = 0x%08x\n",
                      priv->tx_tail_idx, sr);

      // If this descriptor is the first in a frame, then check that
      // the EMAC has processed it. If so, record any errors.
      if (is_sof) {
        // If no previous errors were indicated and the EMAC hasn't
        // processed this descriptor yet, then we are caught up with it.
        // Don't look at any remaining descriptors until the EMAC
        // indicates completion.
        if (!tx_err && !(sr & AT91_EMAC_TBD_SR_USED))
          break;

        // Examine the error bits
        if (sr & (AT91_EMAC_TBD_SR_EXHAUST |
                  AT91_EMAC_TBD_SR_TXUNDER | AT91_EMAC_TBD_SR_RTRY)) {
          tx_err = true;
          AT91_ETH_DEBUG1(tx_err, "AT91_ETH: TX frame at %d has error\n",
                          priv->tx_tail_idx);
        }

        // Next buffer, if any, is not SOF
        is_sof = false;
      }

      // If this descriptor is the last in a frame, then notify
      // the TCP/IP stack about the status of the transmit.
      if (sr & AT91_EMAC_TBD_SR_EOF) {
//        _eth_drv_tx_done(sc, priv->tx_keys[priv->tx_tail_idx], tx_err);

        // Exegin specific. Do transmit callback.
        if (!tx_err && at91_eth_tx_callback_func)
          (at91_eth_tx_callback_func)();

        // Next buffer, if any, is SOF.
        is_sof = true;
      }

      // Mark descriptor as used, preserving wrap-bit.
      sr &= AT91_EMAC_TBD_SR_WRAP;
      sr |= AT91_EMAC_TBD_SR_USED;
      tbd->sr = sr;
      processed_buffers++;
      AT91_ETH_NEXT_TBD(priv->tx_tail_idx);
      cyg_drv_dsr_lock();
      priv->tx_free ++;
      cyg_drv_dsr_unlock();
    } /* while */

    // If error occured that caused EMAC to reset the TX hardware, then
    // we need to follow the TX hardware pointer.
    if (tx_err) {
      cyg_drv_dsr_lock();
      AT91_ETH_DEBUG1(true, "AT91_ETH: TX error - reset TX buffer indexes\n");
      at91_disable_tx(priv);
      at91_enable_tx(priv);
      at91_start_transmitter(priv);
      cyg_drv_dsr_unlock();
    }

    priv->tx_busy = false;
  } // endif transmit-status register indicates work to do
}


/* Return RX descriptors to EMAC, up to but not including, current head */
static void at91_return_rx_bds(at91_eth_priv_t *priv, int rbd_idx)
{
  rbd_t *rbd;

  while (rbd_idx != priv->rx_head_idx) {
    rbd = (rbd_t *)CYGARC_UNCACHED_ADDRESS(&priv->rbd[rbd_idx]);
    rbd->addr &= ~AT91_EMAC_RBD_ADDR_OWNER_SW;
    /* WRITE MEMORY BARRIER */
    HAL_MEMORY_BARRIER();
    AT91_ETH_NEXT_RBD(rbd_idx);
  }
}


/* Handle receive-ready events. */
static void
at91_eth_rx(struct eth_drv_sc *sc)
{
  at91_eth_priv_t *priv = (at91_eth_priv_t *)sc->driver_private;
  cyg_int32 sof_idx = -1;
  cyg_uint32 rsr, sr;
  rbd_t *rbd;

  // Get the Receive Status and clear it
  HAL_READ_UINT32(priv->base+AT91_EMAC_RSR, rsr);
  HAL_WRITE_UINT32(priv->base+AT91_EMAC_RSR, rsr);

  AT91_ETH_DEBUG1(true, "AT91_ETH: RxStat Reg = 0x%08x\n", rsr);
  AT91_ETH_DEBUG1(rsr & AT91_EMAC_RSR_BNA, "Rx_BNA ");
  AT91_ETH_DEBUG1(rsr & AT91_EMAC_RSR_OVR, "Rx_OVR ");
  AT91_ETH_DEBUG1(rsr & AT91_EMAC_RSR_REC, "Rx_REC ");
  AT91_ETH_DEBUG1(true, "\n");

  /* Service the RX buffers if required */
  if (rsr & AT91_EMAC_RSR_REC) {
    rbd = (rbd_t *)CYGARC_UNCACHED_ADDRESS(&priv->rbd[priv->rx_head_idx]);
    // For all descriptors that the EMAC has given ownership to us.
    while (rbd->addr & AT91_EMAC_RBD_ADDR_OWNER_SW) {
      /* READ MEMORY BARRIER */
      /* HAL_MEMORY_BARRIER(); */
      sr = rbd->sr;

      if (sr & AT91_EMAC_RBD_SR_SOF) {
        if (sof_idx != -1) {
          AT91_ETH_DEBUG2(true, "AT91_ETH: RX discarding fragments\n");
          at91_return_rx_bds(priv, sof_idx);
        }
        sof_idx = priv->rx_head_idx;
      }

      // Advance head ptr here, not at end of while_loop.
      AT91_ETH_NEXT_RBD(priv->rx_head_idx);
      rbd = (rbd_t *)CYGARC_UNCACHED_ADDRESS(&priv->rbd[priv->rx_head_idx]);

      if (sr & AT91_EMAC_RBD_SR_EOF) {
        // This is assumed to be unrecoverable hardware failure.
        CYG_ASSERT(sof_idx != -1, "receiver found EOF but no SOF\n");

        // Exegin specific. Do receive callback.
        if (at91_eth_rx_callback_func)
          (at91_eth_rx_callback_func)();

        priv->rx_tail_idx = sof_idx;
        priv->rx_frame_cnt = sr & AT91_EMAC_RBD_SR_LEN_MASK;
        _eth_drv_recv(sc, priv->rx_frame_cnt);
        at91_return_rx_bds(priv, sof_idx);
        sof_idx = -1;
      }
    }

    // If we found SOF but didn't find matching EOF.
    if (sof_idx != -1) {
      AT91_ETH_DEBUG2(true, "AT91_ETH: RX discarding fragments\n");
      priv->rx_head_idx = sof_idx;
    }
  }
}


/* Called by the TCP/IP stack when it wants us to handle hardware events. */
static void
at91_eth_deliver(struct eth_drv_sc *sc)
{
  at91_eth_priv_t *priv = (at91_eth_priv_t *)sc->driver_private;
  cyg_uint32 isr, imr;

  do {
    // Get interrupt status. This clears the register.
    HAL_READ_UINT32(priv->base + AT91_EMAC_ISR, isr);
    AT91_ETH_DEBUG2(true, "AT91_ETH: IrqStat Reg = 0x%08x\n", isr);

    // This is assumed to be unrecoverable hardware failure.
    // Unless caused by slow clock?
    CYG_ASSERT(!(isr & AT91_EMAC_ISR_HRESP), "EMAC DMA/bus failed\n");

    // Get interrupt mask.
    HAL_READ_UINT32(priv->base + AT91_EMAC_IMR, imr);

    isr = isr & ~imr & AT91_ETH_INT_MASK;

    if (isr & AT91_ETH_RX_INT_MASK)
      at91_eth_rx(sc);

    if (isr & AT91_ETH_TX_INT_MASK)
      at91_eth_tx(sc);
  } while (isr);

//#ifdef CYGINT_IO_ETH_INT_SUPPORT_REQUIRED		// Commented
  cyg_drv_interrupt_unmask(priv->int_vector);
//#endif
}


/* Called by the TCP/IP stack when it wants us to unload receive buffers. */
static void
at91_eth_recv(struct eth_drv_sc *sc, struct eth_drv_sg *sg_list, int sg_len)
{
  at91_eth_priv_t *priv = (at91_eth_priv_t *)sc->driver_private;
  cyg_uint32 rb_len, rb_offset;
  cyg_uint32 sg_idx, sg_offset;
  cyg_uint32 copy_cnt;

  // If the TCP/IP stack is broken or out of buffers.
  CYG_ASSERT(sg_list != NULL, "TCP/IP stack error or out of buffers\n");

  // Init working vars
  sg_idx = sg_offset = rb_offset = 0;

  // Get count of bytes in current buffer.
  rb_len = MIN(priv->rx_frame_cnt, AT91_EMAC_RX_BUFF_SIZE);
  HAL_DCACHE_INVALIDATE(priv->rb[priv->rx_tail_idx].rb, rb_len);

  // For all scatter-gather buffers that we need to fill.
  while (sg_idx < sg_len) {
    copy_cnt = MIN(sg_list[sg_idx].len - sg_offset, rb_len - rb_offset);

    if (copy_cnt && sg_list[sg_idx].buf)
      memcpy(((cyg_uint8 *)sg_list[sg_idx].buf) + sg_offset,
             &priv->rb[priv->rx_tail_idx].rb[rb_offset],
             copy_cnt);

    // Update number of bytes remaining in frame, exit if complete.
    priv->rx_frame_cnt -= copy_cnt;
    if (!priv->rx_frame_cnt)
      break;

    // If sg buffer is completely filled, move to next
    sg_offset += copy_cnt;
    if (sg_offset == sg_list[sg_idx].len) {
      sg_idx++;
      sg_offset = 0;
    }

    // If rx buffer is completely drained, move to next
    rb_offset += copy_cnt;
    if (rb_offset == rb_len) {
      AT91_ETH_NEXT_RBD(priv->rx_tail_idx);
      rb_offset = 0;
      // Get count of bytes in this buffer
      rb_len = MIN(priv->rx_frame_cnt, AT91_EMAC_RX_BUFF_SIZE);
      HAL_DCACHE_INVALIDATE(priv->rb[priv->rx_tail_idx].rb, rb_len);
    }
  }
}


// routine called to handle ethernet controller in polled mode
static void
at91_eth_poll(struct eth_drv_sc *sc)
{
  /* Service the buffers */
  at91_eth_deliver(sc);
}


static int
at91_eth_int_vector(struct eth_drv_sc *sc)
{
  return(CYGNUM_HAL_INTERRUPT_EMAC);
}

at91_eth_priv_t at91_priv_data = {
  .int_vector = CYGNUM_HAL_INTERRUPT_EMAC,
  .base = AT91_EMAC,
  .phy = &at91_phy
};

ETH_DRV_SC(at91_sc,
		   &at91_priv_data,       // Driver specific data
           "eth0",                // Name for this interface
           at91_eth_start,
           at91_eth_stop,
           at91_eth_control,
           at91_eth_can_send,
           at91_eth_send,
           at91_eth_recv,
           at91_eth_deliver,
           at91_eth_poll,
           at91_eth_int_vector);

NETDEVTAB_ENTRY(at91_netdev,
                "at91",
                at91_eth_init,
                &at91_sc);

// EOF if_at91.c

