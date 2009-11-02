#ifndef CYGONCE_NAND_DEVICE_H
# define CYGONCE_NAND_DEVICE_H
//=============================================================================
//
//      nand_device.h
//
//      Device driver interface for the eCos NAND flash library
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
// Date:        2009-02-23
//
//####DESCRIPTIONEND####
//=============================================================================

#include <pkgconf/io_nand.h>
#include <pkgconf/isoinfra.h>
#include <cyg/infra/cyg_type.h>
#include <cyg/hal/drv_api.h> // mutexes
#include <cyg/hal/hal_tables.h>
#include CYGBLD_ISO_ERRNO_CODES_HEADER
#include <cyg/nand/nand_ecc.h>
#include <cyg/nand/nand_oob.h>

#if (defined CYGSEM_IO_NAND_DEBUG_LEVEL) && (CYGSEM_IO_NAND_DEBUG_LEVEL > 0)
// Usage: NAND_CHATTER(level, device, fmt, printf args..)
// Important messages should be level 1; inane drivel should be level 9.
#define NAND_CHATTER(_l_, _dev_, _fmt_, ...)                    \
    do {                                                        \
        if (_l_ <= CYGSEM_IO_NAND_DEBUG_LEVEL && *(_dev_)->pf)  \
            (*(_dev_)->pf)((_fmt_), ## __VA_ARGS__);            \
    } while(0)
#else
#define NAND_CHATTER(_l_, _dev_, _fmt_, ...) CYG_EMPTY_STATEMENT
#endif

#ifdef CYGSEM_IO_NAND_DEBUG
#define NAND_ERROR(_dev_, _fmt_, ...)                   \
    do {                                                \
        if (*(_dev_)->pf)                               \
            (*(_dev_)->pf)((_fmt_), ## __VA_ARGS__);    \
    } while(0)
#else
#define NAND_ERROR(_dev_, _fmt_, ...) CYG_EMPTY_STATEMENT
#endif


typedef int (*cyg_nand_printf)(const char *fmt, ...) CYGBLD_ATTRIB_PRINTF_FORMAT(1,2);

typedef cyg_uint32 cyg_nand_page_addr, cyg_nand_block_addr, cyg_nand_column_addr;

/* ================================================================= */

/* Fwd def. A physical NAND device. */
struct _cyg_nand_device_t;
typedef struct _cyg_nand_device_t cyg_nand_device;

/* A partition of a NAND chip */
struct _cyg_nand_partition_t {
    cyg_nand_device *dev; /* The parent device. If NULL, this partition is inactive. */
    cyg_nand_block_addr first;
    cyg_nand_block_addr last;
};
typedef struct _cyg_nand_partition_t cyg_nand_partition;


/* A set of device functions. These are intended to be abstractable
 * and transplantable amongst different boards using the same (or even
 * only similar) chip by only changing the 'priv' struct.
 *
 * Notes about device functions:
 * 1. The NAND library is responsible for checking that the input
 * page/block address is valid; device drivers need not. Hence, the
 * drivers are not given partition information.
 * 2. These functions all return 0 on success (positive if a value
 * is required), or a negative error code.
 */
struct cyg_nand_dev_fns_v1 {
    /* Initialises the device, confirms its presence,
     * interrogates it as necessary, 
     * sets up the in-memory partition array (if configured),
     * and populates other members of the _cyg_nand_device_t.
     *
     * In a multi-threaded eCos configuration, devices MUST set up a
     * mutex at some appropriate level before returning from this call.
     *
     * Note that:
     * 1. The driver MUST NOT malloc; the NAND layer may run without
     * dynamic memory allocation, so anything the device requires should
     * be set up as a static global. Implementers should use the
     * driver API (cyg/hal/drv_api.h) as far as possible.
     * 2. We do not define at which level a mutex should be set up, as it
     * may differ from device to device - they may need to lock out just
     * a single device, or an entire bus, or something more esoteric.
     */
    int (*devinit) (cyg_nand_device *dev);

    /* Reads a single page and its spare area. See cyg_nand_read_page().
     * (N.B. This is a 'raw' read. No attempt should be made to check
     * and repair ECC, or to map spare data around the ECC location;
     * that's done by the caller.) */
    int (*read_page) (cyg_nand_device *dev, cyg_nand_page_addr page,
            void * dest, size_t size, void * spare, size_t spare_size);
    /* Writes a single page and its spare area. See cyg_nand_write_page().
     * (Again, this is a 'raw' write.) */
    int (*write_page)(cyg_nand_device *dev, cyg_nand_page_addr page, 
            const void * src, size_t size,
            const void * spare, size_t spare_size);

    /* Erases an eraseblock. See cyg_nand_erase_block(). */
    int (*erase_block)(cyg_nand_device *dev, cyg_nand_block_addr blk);

    /* Looks to see if a block contains the chip-specific factory-bad
     * marker. Returns 1 if bad, 0 if OK, or a -ve error code if 
     * something went really wrong. */
    int (*is_factory_bad)(cyg_nand_device *dev, cyg_nand_block_addr blk);
};

/* Context for a physical NAND device.
 * Each device on the system should have its own.
 */
struct _cyg_nand_device_t {
    int version; /* Reserved for future expansion. Currently set to 1 
                    by the NAND library.*/

    cyg_drv_mutex_t devlock; // Device-level locking applied by this library.

    int is_inited; /* Managed by the NAND library. 
                      If not set, only calls to devinit() are legal. */
    cyg_nand_printf pf; /* Diagnostic printf to use. The default
                           is configured by CDL, but a driver may
                           override it here if desired. */

    const char *devname; /* Device name for use by applications,
                            e.g. "onboard" or "bus1". */

    struct cyg_nand_dev_fns_v1 *fns; /* Access functions */
    void * priv; /* If required, points to private device-specific data.  */


    /* Data about the device need not be defined statically. Indeed
     * it is usually preferable to autodetect parameters when the chip
     * is initialised. The details here need only be set at some point
     * before fns->devinit() returns. */

    cyg_nand_partition partition[CYGNUM_NAND_MAX_PARTITIONS];

    size_t page_bits; /* log2 of no of regular bytes per page */
    size_t spare_per_page; /* OOB area size in bytes */
    size_t block_page_bits; /* log2 of no of pages per eraseblock */
    size_t blockcount_bits; /* log2 of number of blocks */
    size_t chipsize_log; /* log2 of total chip size in BYTES. */

    struct {
        cyg_nand_block_addr primary, mirror;
        CYG_BYTE *data; /* in-RAM bad block table. See nand_bbt.c.
            devinit must set up data: 2 bits * number of blocks. */
        size_t datasize; /* size of data in bytes, used to cross-check */
        CYG_BYTE version; /* _current_ version tag */
    } bbt;

    cyg_nand_ecc_t *ecc;
    const cyg_nand_oob_layout *oob;
} CYG_HAL_TABLE_TYPE;

#define CYG_NAND_FUNS(_funs_, _devinit_, _rdpg_, _wrpg_, _erasebl_, _factorybad_) \
struct cyg_nand_dev_fns_v1 _funs_ = \
{                                   \
    .devinit = _devinit_,           \
    .read_page = _rdpg_,            \
    .write_page = _wrpg_,           \
    .erase_block = _erasebl_,       \
    .is_factory_bad = _factorybad_, \
}

#define CYG_NAND_DEVICE(_structname_, _devname_, _funs_, _priv_, _ecc_, _oob_)\
struct _cyg_nand_device_t _structname_ CYG_HAL_TABLE_ENTRY(cyg_nand_dev) =    \
{                               \
    .version = 1,               \
    .is_inited = 0,             \
    .devname = _devname_,       \
    .fns = _funs_,              \
    .priv = _priv_,             \
    .ecc = _ecc_,               \
    .oob = _oob_,               \
}

/* To declare a new NAND device,
 * first provide its functions and tie them together with CYG_NAND_FUNCTIONS,
 * then instantiate it with CYG_NAND_DEVICE.
 * - You can optionally provide private data in 'priv' which will be passed
 *   to the driver functions (e.g. for register addresses or GPIO config).
 * - You can use a static ECC param block, or pass it as NULL in the macro
 *   and set it up in devinit().
 *
 * You also must decide on the partition layout for the device.
 * This could be hard-coded, defined in CDL, read from a "partition table",
 * user-supplied at runtime or something more esoteric.
 * Whatever you do, the partition table in the nand_device struct has to
 * be set up by the time devinit returns.
 */

/* The HAL table itself lives in nand.c. */
__externC cyg_nand_device cyg_nanddevtab[];
__externC cyg_nand_device cyg_nanddevtab_end;


#endif
