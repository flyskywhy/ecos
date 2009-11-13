#ifndef CYGONCE_K9FXX08X0X_H
#define CYGONCE_K9FXX08X0X_H
//=============================================================================
//
//      k9fxx08x0x.h
//
//      Definitions header for the Samsung K9Fxx08x0x family
//
//=============================================================================
// ####ECOSGPLCOPYRIGHTBEGIN####                                            
// -------------------------------------------                              
// This file is part of eCos, the Embedded Configurable Operating System.   
// Copyright (C) 2009 eCosCentric Limited.
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
//=============================================================================
//#####DESCRIPTIONBEGIN####
//
// Author(s):   wry
// Date:        2009-06-05
// Description: Early definitions for the driver.
//              The intention is that the instantiating C file include
//              this file once, define the things it needs to, then
//              include k9fxx08x0x.inl as many times as necessary
//              (probably just once).
//
//####DESCRIPTIONEND####
//=============================================================================

#include <pkgconf/io_nand.h>
#include <cyg/nand/nand_device.h>
#include <cyg/hal/drv_api.h>
#include <cyg/infra/diag.h>

#define k9_WAIT_tWB() HAL_DELAY_US(1)

// This is a 1Gbit device with 1024 eraseblocks.
#define k9f1g_blockcount_bits 10
#define k9f1g_chipsize_log  27 // 1Gbit = 2^30 bits = 2^27 bytes

// The in-RAM BBT for a device is currently 2 bits per eraseblock.
// See nand_bbt.c for more.
#define k9f1g_bbt_datasize (1<<(k9f1g_blockcount_bits-2))

/* Our private structure ======================================= */
// Every instance of the chip needs its own copy of this struct.
// N.B. that this is too big to go on the stack in certain
// eCos configurations; it should normally be static.

struct _k9_priv {
    void *plat_priv; // For use by the platform HAL, if desired.
    cyg_nand_page_addr pagestash; // Guarded by dev lock.
#ifdef CYGSEM_IO_NAND_USE_BBT
    unsigned char bbt_data[k9f1g_bbt_datasize];
#endif
};

typedef struct _k9_priv k9_priv;

/* Prototypes for functions to be provided by the platform driver ==== */

// Low-level chip access functions ------------------

static inline void write_cmd(cyg_nand_device *ctx, unsigned char cmd);
static inline void write_addrbytes(cyg_nand_device *ctx, CYG_BYTE *bytes, size_t n);
static inline unsigned char read_data_1(cyg_nand_device *ctx);
static inline void read_data_bulk(cyg_nand_device *ctx, unsigned char *dp, size_t n);
static inline void write_data_1(cyg_nand_device *ctx, unsigned char b);
static inline void write_data_bulk(cyg_nand_device *ctx, const unsigned char *dp, size_t n);

// Chip BUSY line access / fallback -----------------

/* Waits either for the !BUSY line to signal that the device is finished,
   or for the prescribed amount of time, depending on what is available
   on the platform.
 * wait_init specifies the time in microseconds to wait to ensure that
   BUSY is asserted.
 * wait_fallback specifies the worst-case time to wait (from the spec
   sheet; again in microseconds) for the operation to complete. */
static void wait_ready_or_time(cyg_nand_device *ctx,
                               size_t wait_init, size_t wait_fallback);

/* Waits either for the !BUSY line to signal that the device is finished,
 * or (if not available) polls the chip by sending the Read Status command
 * and waits for (response & mask) to be non-zero. */
static void wait_ready_or_status(cyg_nand_device *ctx, CYG_BYTE mask);

// Chip concurrent-access protection ----------------
// (This need not do anything, if the library-provided
//  per-device locking is sufficient.)
 
static inline void k9_devlock(cyg_nand_device *ctx);
static inline void k9_devunlock(cyg_nand_device *ctx);

// Initialisation hooks -----------------------------

/* Platform-specific chip initialisation.
 * Set up chip access lines, GPIO config, &c; should also set up
 * locking to guard against concurrent access.
 * Return 0 on success, or a negative error code. */
static int k9_plf_init(cyg_nand_device *ctx);

/* (N.B. There is no deinit hook, as the library does not currently 
 *  support a device being deconfigured.) */

/* Platform-specific in-memory partition table setup.
 * Return 0 on success, or a negative error code.
 * (This is called at the end of devinit, so you can read from the chip
 * if need be.) */
static int k9_plf_partition_setup(cyg_nand_device *dev);

#endif
