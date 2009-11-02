//=============================================================================
//
//      nand.c
//
//      Main application interface for the eCos NAND flash library
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
// Date:        2009-03-02
//
//####DESCRIPTIONEND####
//=============================================================================

#include <cyg/hal/drv_api.h> // mutexes
#include <cyg/infra/diag.h>
#include <cyg/infra/cyg_ass.h>

#include <cyg/nand/nand.h>
#include <cyg/nand/nand_device.h>
#include <cyg/nand/nand_devtab.h>
#include <cyg/nand/util.h>
#include "nand_bbt.h"
#include CYGBLD_ISO_ERRNO_CODES_HEADER
#include <string.h>

extern cyg_drv_mutex_t nand_bbt_pagebuf_lock; // Lives in nand_bbt.c

/* ============================================================ */

/* We have a global ("devinit") lock, protects the nanddevtab and all 
 * writes to device->isInited.
 *
 * Each device also has its own lock, which protects the rest of
 * the structure, and the hardware itself.
 *
 * Rules:
 * 1. Nothing can use a device until it has been fully initialised 
 * (i.e. they must check dev->isInited, which is not set until
 * initialisation is complete);
 * 2. All calls to a device which has been initialised must acquire
 * the device's lock, to prevent multiple access;
 * 3. A repeated call to initialise a device is a safe no-op (under the
 * global lock, see that isInited is set, so do nothing);
 * 4. It is not possible for multiple threads to simultaneously 
 * initialise the same device [via the lookup function]
 * (they are protected by the global lock - the second blocks
 * until the first has completed, at which point the second thread 
 * sees that the device has been inited and falls out into a no-op).
 *
 * Therefore, an operation in devinit which uses the chip need not
 * assert the per-device lock, as nothing else can use the chip until
 * it has completed and set isInited to 1.
 */

static cyg_drv_mutex_t devinit_lock;

#define LOCK_devinit() cyg_drv_mutex_lock(&devinit_lock)
#define UNLOCK_devinit() cyg_drv_mutex_unlock(&devinit_lock)

/* Per-device lock.
 * NB that if multiple devices might interact (e.g. sharing a CPLD),
 * their drivers must act in concert to prevent this, usually at the 
 * platform level. */
#define LOCK_DEV(dev)   cyg_drv_mutex_lock(&dev->devlock)
#define UNLOCK_DEV(dev) cyg_drv_mutex_unlock(&dev->devlock)

/* ============================================================ */
/* Initialisation and lookup */

static cyg_nand_printf nand_default_pf;

// This is called only by the C++ static constructor. Applications
// never need to call this themselves.
__externC void cyg_nand_initx(cyg_nand_printf pf)
{
    if (pf) CYG_CHECK_FUNC_PTRC(pf);
    cyg_drv_mutex_init(&devinit_lock);
    cyg_drv_mutex_init(&nand_bbt_pagebuf_lock);
    nand_default_pf = pf;
}

__externC
cyg_nand_partition* cyg_nand_get_partition(cyg_nand_device *dev, unsigned partno)
{
    if ((partno < 0) || (partno >= CYGNUM_NAND_MAX_PARTITIONS))
        return NULL;
    LOCK_DEV(dev);
    cyg_nand_partition *rv = &(dev->partition[partno]);
    UNLOCK_DEV(dev);
    if (!rv->dev) return NULL; /* partition inactive */
    return rv;
}

__externC
int cyg_nand_lookup(const char *devname, cyg_nand_device **dev_o)
{
    int rv = -ENOENT;
    if (dev_o) {
        CYG_CHECK_DATA_PTRC(dev_o);
        *dev_o = 0;
    }
    if (!devname) return -EINVAL;

    LOCK_devinit(); // ++++++++++++++++++++++++++++++++++++++++++++
    cyg_nand_device *dev;
    for (dev = &cyg_nanddevtab[0]; dev != &cyg_nanddevtab_end; dev++) {
        if (0==strcmp(devname, dev->devname)) {
            rv = 0;
            break;
        }
    }
    if (!rv) {
        if (!dev->is_inited) {
            int i;
            CYG_CHECK_DATA_PTRC(dev->fns);
            CYG_CHECK_FUNC_PTRC(dev->fns->devinit);
            CYG_CHECK_FUNC_PTRC(dev->fns->read_page);
            CYG_CHECK_FUNC_PTRC(dev->fns->write_page);
            CYG_CHECK_FUNC_PTRC(dev->fns->erase_block);
            CYG_CHECK_FUNC_PTRC(dev->fns->is_factory_bad);

            dev->version = 1;
            dev->pf = nand_default_pf;
            for (i=0; i<CYGNUM_NAND_MAX_PARTITIONS; i++)
                dev->partition[i].dev = 0;
            dev->bbt.data = 0; // Paranoia, ensure devinit sets up
            rv = dev->fns->devinit(dev);
            if (rv) {
                NAND_ERROR(dev,"Could not initialise NAND device \"%s\": code %d\n", devname, rv);
                goto done;
            }
            CYG_CHECK_DATA_PTRC(dev->bbt.data);
            CYG_CHECK_DATA_PTRC(dev->ecc);
            CYG_CHECK_DATA_PTRC(dev->oob);
            if (!dev->chipsize_log ||
                    !dev->blockcount_bits ||
                    !dev->block_page_bits ||
                    !dev->spare_per_page ||
                    !dev->page_bits ||
                    !dev->bbt.data ||
                    !dev->ecc ||
                    !dev->oob) {
                NAND_ERROR(dev,"BUG: NAND driver devinit did not fill in all required fields - disabling device\n");
                rv = -ENOSYS;
                goto done;
            }

            if (dev->bbt.datasize < (1 << (dev->blockcount_bits-2)) ) {
                NAND_ERROR(dev,"BUG: NAND driver declared bbt.data_size isn't big enough (got %lu, want %u) - disabling device\n", (unsigned long) dev->bbt.datasize, (1 << (dev->blockcount_bits-2)));
                rv = -ENOSYS;
                goto done;
            }

            if ( dev->oob->ecc_size != CYG_NAND_ECCPERPAGE(dev) ) {
                NAND_ERROR(dev,"BUG: NAND driver has inconsistent ECC size declaration (oob says %d, ecc says %d) - disabling device\n", dev->oob->ecc_size, CYG_NAND_ECCPERPAGE(dev));
                rv = -ENOSYS;
                goto done;
            }

            rv = cyg_nand_bbti_find_tables(dev);
            if (rv == -ENOENT) {
                NAND_CHATTER(1,dev, "Creating initial bad block table on device %s\n", devname);
                rv = cyg_nand_bbti_build_tables(dev);
            }
            if (rv != 0) {
                NAND_ERROR(dev,"Cannot find or build BBT (%d)\n", -rv);
                goto done;
            }

            cyg_drv_mutex_init(&dev->devlock);
            dev->is_inited = 1;

            int live_partitions = 0;
            for (i=0; i<CYGNUM_NAND_MAX_PARTITIONS; i++)
                if (dev->partition[i].dev) ++live_partitions;
            if (live_partitions)
                NAND_CHATTER(1,dev, "%s devinit complete, %u partitions configured\n", devname, live_partitions);
            else
                NAND_CHATTER(1,dev, "%s devinit complete, NO partitions configured!\n", devname); // hope they know what they're doing.
        }
        if (dev_o) *dev_o = dev;
    }
done:
    UNLOCK_devinit(); // ------------------------------------------
    return rv;
}

/* ============================================================ */
/* Device access */

#define DEV_INIT_CHECK(dev) do { if (!dev->is_inited) return -ENXIO; } while(0)
#define PARTITION_CHECK(p) do { if (!p->dev) return -ENXIO; } while(0)

static inline int valid_block_addr(cyg_nand_partition *part, cyg_nand_block_addr block)
{
    return ( (block < part->first) || (block > part->last) ) ? -ENOENT : 0;
}

static int valid_page_addr(cyg_nand_partition *part, cyg_nand_page_addr page)
{
    cyg_nand_block_addr block = CYG_NAND_PAGE2BLOCKADDR(part->dev,page);
    int rv = valid_block_addr(part, block);
    if (rv != 0)
        NAND_CHATTER(1,part->dev, "Invalid attempted access to page %d\n", page);
    return rv;
}

#define EG(what) do { rv = (what); if (rv != 0) goto err_exit; } while(0)

__externC
int cyg_nand_read_page(cyg_nand_partition *prt, cyg_nand_page_addr page,
                void * dest, size_t size, void * spare, size_t spare_size)
{
    int tries=0;
    int rv;
    PARTITION_CHECK(prt);
    cyg_nand_device *dev = prt->dev;
    DEV_INIT_CHECK(dev);

    if (dest)  CYG_CHECK_DATA_PTRC(dest);
    if (spare) CYG_CHECK_DATA_PTRC(spare);

    if (size > (1<<dev->page_bits)) return -EFBIG;
    if (spare_size > dev->spare_per_page) return -EFBIG;

    LOCK_DEV(dev);

    CYG_BYTE ecc_read[CYG_NAND_ECCPERPAGE(dev)],
             ecc_calc[CYG_NAND_ECCPERPAGE(dev)];
    CYG_BYTE oob_buf[dev->spare_per_page];

    EG(valid_page_addr(prt, page));
    cyg_nand_block_addr blk = CYG_NAND_PAGE2BLOCKADDR(dev,page);
    if (cyg_nand_bbti_query(dev, blk) != CYG_NAND_BBT_OK) {
        NAND_CHATTER(1,dev,"Asked to read page %u in bad block %u\n", page, blk);
        EG(-EINVAL);
    }

    do {
        ++tries;

        EG(dev->fns->read_page(dev, page, dest, size, oob_buf, dev->spare_per_page));

        nand_oob_unpack(dev, spare, spare_size, ecc_read, oob_buf);

        if (!dest) goto err_exit; // No data? Can't ECC it!
        if (size != 1<<dev->page_bits) goto err_exit;
        // FIXME: Make part-page ECC work.

        nand_ecci_calc_page(dev, dest, ecc_calc);
        rv = nand_ecci_repair_page(dev, dest, ecc_read, ecc_calc);
        if (rv==-1 && (tries < CYGNUM_NAND_MAX_READ_RETRIES) ) {
            NAND_CHATTER(4, dev, "NAND: ECC uncorrectable error on read, retrying\n");
        }
    } while (rv==-1 && (tries < CYGNUM_NAND_MAX_READ_RETRIES) );

    switch (rv) {
        case 0:
            NAND_CHATTER(8,dev,"Read page %u OK\n", page);
            break;
        case -1:
            NAND_ERROR(dev,"NAND: Page %u read gave ECC uncorrectable error\n", page);
            rv=-EIO;
            break;
        case 1:
        case 2:
        case 3:
            NAND_CHATTER(2,dev, "Page %u ECC correction, type %d\n", page,rv);
            rv=0;
            break;
    }
err_exit:
    UNLOCK_DEV(dev);
    return rv;
}

__externC
int cyg_nand_write_page(cyg_nand_partition *prt, cyg_nand_page_addr page,
        const void * src, size_t size, const void * spare, size_t spare_size)
{
#ifdef CYGSEM_IO_NAND_READONLY
    return -EROFS;
#else
    int rv;
    PARTITION_CHECK(prt);
    cyg_nand_device *dev = prt->dev;
    DEV_INIT_CHECK(dev);

    if (src)   CYG_CHECK_DATA_PTRC(src);
    if (spare) CYG_CHECK_DATA_PTRC(spare);

    LOCK_DEV(dev);

    CYG_BYTE ecc[CYG_NAND_ECCPERPAGE(dev)];
    CYG_BYTE oob_packed[dev->spare_per_page];

    EG(valid_page_addr(prt, page));
    cyg_nand_block_addr blk = CYG_NAND_PAGE2BLOCKADDR(dev,page);
    if (cyg_nand_bbti_query(dev, blk) != CYG_NAND_BBT_OK) {
        NAND_CHATTER(1,dev,"Asked to write page %u in bad block %u\n", page, blk);
        EG(-EINVAL);
    }

    if (src && (size == 1<<dev->page_bits)) {
        nand_ecci_calc_page(dev, src, ecc);
        // FIXME: Make ECC work on part-pages.
    } else {
        // No data, can't compute an ECC, hope they're not overwriting...
        memset(ecc, 0xff, CYG_NAND_ECCPERPAGE(dev));
    }
    nand_oob_pack(dev, spare, spare_size, ecc, oob_packed);

    NAND_CHATTER(8,dev,"Write page %u\n", page);
    EG(dev->fns->write_page(dev, page, src, size, oob_packed, dev->spare_per_page));
    /* N.B. We don't read-back to verify; drivers may do so themselves if
     * they wish. Typically the spec sheet says that a read-back test
     * is unnecessary if the device reports a successful program, and 
     * if ECC is being used. */

err_exit:
    UNLOCK_DEV(dev);
    return rv;
#endif
}

__externC
int cyg_nand_erase_block(cyg_nand_partition *prt, cyg_nand_block_addr blk)
{
#ifdef CYGSEM_IO_NAND_READONLY
    return -EROFS;
#else
    PARTITION_CHECK(prt);
    cyg_nand_device *dev = prt->dev;
    int rv;
    DEV_INIT_CHECK(dev);

    LOCK_DEV(dev);

    EG(valid_block_addr(prt, blk));
    if (cyg_nand_bbti_query(dev, blk) != CYG_NAND_BBT_OK) {
        NAND_CHATTER(1,dev,"Asked to erase bad block %u\n", blk);
        EG(-EINVAL);
    }
    NAND_CHATTER(8,dev,"Erasing block %u\n", blk);
    rv = dev->fns->erase_block(dev, blk);
    if (rv==-EIO) {
        cyg_nand_bbti_markbad(dev, blk); // deliberate ignore
        EG(rv);
    }
err_exit:
    UNLOCK_DEV(dev);
    return rv;
#endif
}

__externC
int cyg_nand_bbt_query(cyg_nand_partition *prt, cyg_nand_block_addr blk)
{
    int rv;
    PARTITION_CHECK(prt);
    cyg_nand_device *dev = prt->dev;
    DEV_INIT_CHECK(dev);
    LOCK_DEV(dev);
    EG(valid_block_addr(prt, blk));
    rv = cyg_nand_bbti_query(dev, blk);
err_exit:
    UNLOCK_DEV(dev);
    return rv;
}

__externC
int cyg_nand_bbt_markbad(cyg_nand_partition *prt, cyg_nand_block_addr blk)
{
    int rv;
    PARTITION_CHECK(prt);
    cyg_nand_device *dev = prt->dev;
    DEV_INIT_CHECK(dev);
    LOCK_DEV(dev);
    EG(valid_block_addr(prt, blk));
    EG(cyg_nand_bbti_markbad(dev, blk));
err_exit:
    UNLOCK_DEV(dev);
    return rv;
}

__externC
int cyg_nand_bbt_markbad_pageaddr(cyg_nand_partition *prt, cyg_nand_page_addr pg)
{
    int rv;
    PARTITION_CHECK(prt);
    cyg_nand_device *dev = prt->dev;
    DEV_INIT_CHECK(dev);
    LOCK_DEV(dev);
    cyg_nand_block_addr blk = CYG_NAND_PAGE2BLOCKADDR(dev, pg);
    EG(valid_block_addr(prt, blk));
    EG(cyg_nand_bbti_markbad(dev, blk));
err_exit:
    UNLOCK_DEV(dev);
    return rv;
}

/* Computes the ECC for a whole device page.
 * 'page' points to the data; a whole page will necessarily be read.
 * The computed ECC will be stored in 'ecc_o'; CYG_NAND_ECCPERPAGE(dev)
 * bytes will be written. */
void nand_ecci_calc_page(cyg_nand_device *dev, const CYG_BYTE *page, CYG_BYTE *ecc_o)
{
    int i;
    const int nblocks = (1<<dev->page_bits) / dev->ecc->data_size;

    CYG_CHECK_DATA_PTRC(page);
    CYG_CHECK_DATA_PTRC(ecc_o);

    for (i=0; i<nblocks; i++) {
        dev->ecc->calc(page,ecc_o);
        page += dev->ecc->data_size;
        ecc_o += dev->ecc->ecc_size;
    }
}

/* Checks and (if necessary) repairs the ECC for a whole device page.
 * 'page' points to the data; a whole page will necessarily be read.
 * Broadly the same semantics as for cyg_nand_ecc_t.repair; 
 * both ECCs are of size CYG_NAND_ECCPERPAGE(dev), and ecc_read may
 * be corrected as well as the data.
 * Returns:
 *      0 for no errors
 *      1 if there was at least one corrected data error
 *      2 if there was at least one corrected ECC error
 *      3 if there was at least one corrected error in both data and ECC
 *     -1 if there was an uncorrectable error (>1 bit in a single ECC block)
 */
int nand_ecci_repair_page(cyg_nand_device *dev, CYG_BYTE *page, CYG_BYTE *ecc_read, const CYG_BYTE *ecc_calc)
{
    int i, page_rv=0;
    const int nblocks = (1<<dev->page_bits) / dev->ecc->data_size;
    CYG_CHECK_DATA_PTRC(page);
    CYG_CHECK_DATA_PTRC(ecc_read);
    CYG_CHECK_DATA_PTRC(ecc_calc);

    for (i=0; i<nblocks; i++) {
        int chunk_rv = dev->ecc->repair(page,ecc_read,ecc_calc);
        if (chunk_rv < 0) return chunk_rv;
        page_rv |= chunk_rv;
        page += dev->ecc->data_size;
        ecc_read += dev->ecc->ecc_size;
        ecc_calc += dev->ecc->ecc_size;
    }
    return page_rv;
}

