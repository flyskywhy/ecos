//=============================================================================
//
//      k9fxx08x0x.inl
//
//      Inline include file for the Samsung K9Fxx08x0x family
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
// Date:        2009-03-03
//
//####DESCRIPTIONEND####
//=============================================================================

#include <cyg/devs/nand/k9fxx08x0x.h>
#include <cyg/infra/cyg_ass.h>

#define LOCK(dev)   k9_devlock(dev)
#define UNLOCK(dev) k9_devunlock(dev)

/* Helpers ===================================================== */

static inline void change_read_column(cyg_nand_device *dev, cyg_nand_column_addr col)
{
    CYG_BYTE addr[2] = { col & 0xff, (col>>8) & 0xff };
    write_cmd(dev,0x05);
    write_addrbytes(dev, &addr[0], 2);
    write_cmd(dev,0xe0);
    // wait tREA before reading further. 
    // This is 18ns, less than an instruction.
}

static inline void change_write_column(cyg_nand_device *dev, cyg_nand_column_addr col)
{
    CYG_BYTE addr[2] = { col & 0xff, (col>>8) & 0xff };
    write_cmd(dev,0x85);
    write_addrbytes(dev, &addr[0], 2);
    // We must wait at least tADL (100ns) before writing further. 
    HAL_DELAY_US(1); // TODO: 1us is a bit wasteful; prefer a tighter sleep duration
}

static inline CYG_BYTE read_status(cyg_nand_device *dev)
{
    write_cmd(dev,0x70);
    return read_data_1(dev);
}

/* Sends a reset command and waits for the RDY signal (or timeout) before
 * returning.
 * Timeout is tRST:
 *   5us during a Read or idle; 10us during Program; 500us during Erase.
 */
static void k9_reset(cyg_nand_device *dev, size_t howlong)
{
    NAND_CHATTER(8,dev, "Resetting device\n");
    write_cmd(dev,0xFF);
    wait_ready_or_time(dev, 1, howlong);
}

/* Entrypoints ================================================= */

static int k9_devinit(cyg_nand_device *dev)
{
    k9_priv *priv = dev->priv;
    CYG_BYTE addr = 0;
    CYG_BYTE id[4];
    int rv = 0;

    // We are protected by the master devinit lock, so we are safe
    // in the knowledge that there's only one of us running right now.
    // (We still need to take out the chip-lock, though, in case there's
    //  another thread trying to access another device which interferes
    //  with us.)

    rv = k9_plf_init(dev); // sets up device locking as necessary
    if (rv != ENOERR) return rv; // _not_ ER(), we haven't locked the chip yet

    LOCK(dev); // +++++++++++++++++++++++++++++++++++++++++++++
#define ER(x) do { rv = (x); if (rv != ENOERR) goto err_exit; } while(0)

    k9_reset(dev,500); /* We must wait the worst-case reset time */

    write_cmd(dev,0x90);
    write_addrbytes(dev, &addr, 1);
    // wait tWHR.
    read_data_bulk(dev, &id[0], sizeof(id) / sizeof(id[0]));

    NAND_CHATTER(6,dev, "Reading device ID... %02x %02x %02x %02x\n", id[0], id[1], id[2], id[3]);
    if (id[0] != 0xEC) {
        NAND_CHATTER(1,dev, "Unrecognised manufacturer code %02x (expected EC)\n", id[0]);
        ER(-ENODEV);
    }
    switch (id[1]) {
        case 0xF1:
            // We don't get chip size and block count from the fourth ID
            // byte, so have to hard-code based on chip ID.
            NAND_CHATTER(3,dev, "SAMSUNG K9F1G08X0X: 1Gbit, 8-bit data bus\n");
            // This is a 1Gbit device with 1024 eraseblocks.
            dev->blockcount_bits = k9f1g_blockcount_bits;
            dev->chipsize_log = k9f1g_chipsize_log;
            dev->bbt.data = priv->bbt_data;
            dev->bbt.datasize = sizeof(priv->bbt_data);
            break;
        default:
            NAND_CHATTER(1,dev, "Unrecognised device code (expected F1)\n");
            ER(-ENODEV);
    }

    // id[2] ignored

    if (id[3] != 0x15) {
        // TODO: How generic is this decode code?
        // Could relax this restriction later depending on chips to support.
        NAND_CHATTER(1,dev, "Unrecognised info code (expected 15)\n");
        ER(-ENODEV);
    }

    dev->page_bits = (id[3] & 3) + 10; // 00 -> 1kbyte; 01 -> 2k.
    int bytes_per_page = 1 << (dev->page_bits);
    CYG_ASSERT(bytes_per_page <= CYGNUM_NAND_PAGEBUFFER, "max pagebuffer not big enough");
    NAND_CHATTER(6,dev, "Page size: %u bytes (2^%u)\n", bytes_per_page, dev->page_bits);

    dev->spare_per_page = ( (id[3] & (1<<2)) ? 16 : 8 ) * (bytes_per_page /512);
    NAND_CHATTER(6,dev, "Spare per page: %u bytes\n", dev->spare_per_page);

    size_t log2_blocksize = 16 /* 64KB */ + ((id[3] >> 4) & 3); // 00 -> 64KB per block, &c.

    dev->block_page_bits = log2_blocksize - dev->page_bits;
    NAND_CHATTER(6,dev, "Pages per block: %u (2^%u)\n", 1<<dev->block_page_bits, dev->block_page_bits);

    if (id[3] & (1<<6)) {
        NAND_ERROR(dev, "NAND Data bus 16 bits is NYI\n");
        ER(-ENODEV);
    }

    if (dev->chipsize_log !=
            dev->page_bits + dev->block_page_bits + dev->blockcount_bits) {
        NAND_ERROR(dev, "NAND device error: Coded chip size (2^%u bytes) does not match computed (2^%u by/pg, 2^%u pg/bl, 2^%u bl/chip)\n", dev->chipsize_log, dev->page_bits, dev->block_page_bits, dev->blockcount_bits);
        ER(-ENODEV);
    }

    // Could read the serial-access timing as well and configure a delay loop?

    ER(k9_plf_partition_setup(dev));

err_exit:
    //k9_reset(dev,5); // Unnecessary.
    UNLOCK(dev); // ------------------------------------------------
    return rv;
}
#undef ER

static int k9_read_begin(cyg_nand_device *dev, cyg_nand_page_addr page)
{
    //k9_priv *priv = dev->priv;
    CYG_BYTE addr[4] = { 0,0, page & 0xff, (page >> 8) & 0xff };

    NAND_CHATTER(7,dev,"Reading page %d\n",page);
    LOCK(dev);

    write_cmd(dev,0x00);
    write_addrbytes(dev, &addr[0], 4);
    write_cmd(dev,0x30);
    wait_ready_or_time(dev, 1, 25 /* tR */);
    return 0;
}

static int k9_read_stride(cyg_nand_device *dev, void * dest, size_t size)
{
    if (size) {
        if (dest)
            read_data_bulk(dev, dest, size);
        else {
            // Dummy-sink reads
            while (size--) read_data_1(dev);
        }
    }
    return 0;
}

static int k9_read_finish(cyg_nand_device *dev, void * spare, size_t spare_size)
{

    if (spare && spare_size) {
        change_read_column(dev, 1 << dev->page_bits);
        read_data_bulk(dev, spare, spare_size);
    }
    //k9_reset(dev,5); // Unnecessary.
    UNLOCK(dev);
    return 0;
}

static int k9_write_begin(cyg_nand_device *dev, cyg_nand_page_addr page)
{
    k9_priv *priv = dev->priv;
    CYG_BYTE addr[4] = { 0,0, page & 0xff, (page >> 8) & 0xff };
    // NB: Program with random data input takes a _five_ byte address,
    // according to spec p24. This doesn't make sense - everything else
    // about this chip takes a four-byte address - and is contradicted by p32.

    LOCK(dev);
    priv->pagestash = page;
    write_cmd(dev,0x80);
    write_addrbytes(dev, &addr[0], 4);

    return 0;
}

static int k9_write_stride(cyg_nand_device *dev, const void * src, size_t size)
{
    if (size) {
        if (src)
            write_data_bulk(dev, src, size);
        else {
            // Dummy-source writes
            while (size--) write_data_1(dev, 0xff);
        }
    }
    return 0;
}

static int k9_write_finish(cyg_nand_device *dev, const void * spare, size_t spare_size)
{
    k9_priv *priv = dev->priv;
    int rv = 0;
    if (spare && spare_size) {
        change_write_column(dev, 1 << dev->page_bits);
        write_data_bulk(dev, spare, spare_size);
    }
    write_cmd(dev,0x10);
    wait_ready_or_status(dev, 1<<6);

    if (read_status(dev) & 1) {
        NAND_ERROR(dev, "NAND: k9fxx08x0x: Programming failed! Page %u", priv->pagestash);
        rv =-EIO;
        goto done;
    } else {
        NAND_CHATTER(7,dev, "Programmed %u OK\n", priv->pagestash);
    }
done:
    //k9_reset(dev,5); // Unnecessary.
    UNLOCK(dev);
    return rv;
}

static int k9_erase_block(cyg_nand_device *dev, cyg_nand_block_addr blk)
{
    //k9_priv *priv = dev->priv;
    unsigned row = blk << (dev->block_page_bits);
    CYG_BYTE addr[2] = { row & 0xff, (row >> 8) & 0xff };
    int rv = 0;

    LOCK(dev);
    write_cmd(dev,0x60);
    write_addrbytes(dev, &addr[0], 2);
    write_cmd(dev,0xd0);
    wait_ready_or_status(dev, 1<<6);
    if (read_status(dev) & 1) {
        rv = -EIO;
        NAND_ERROR(dev, "NAND: k9fxx08x0x: Erasing block %u failed.\n",blk);
        goto done;
    }
    NAND_CHATTER(7,dev, "Erased block %u OK\n", blk);
done:
    UNLOCK(dev);
    return rv;
}

// The spec sheet says to check the 1st OOB byte (address 2048) in
// both the 1st and 2nd page of the block. If both are 0xFF, the block is OK;
// else it's bad.
#define K9_FACTORY_BAD_COLUMN_ADDR 2048
static int k9_factorybad(cyg_nand_device *dev, cyg_nand_block_addr blk)
{
    cyg_nand_page_addr page,
                       pagebase = blk * (1<<dev->block_page_bits);
    const int col = K9_FACTORY_BAD_COLUMN_ADDR;
    int rv = 0, i;

    LOCK(dev);

    for (i=0; i<2; i++) {
        page = pagebase+i;
        CYG_BYTE addr[4] = { col & 0xff, (col>>8) & 0xff, 
            page & 0xff, (page>>8) & 0xff };

        write_cmd(dev,0x00);
        write_addrbytes(dev, &addr[0], 4);
        write_cmd(dev,0x30);
        wait_ready_or_time(dev, 1, 25 /* tR */);
        unsigned char t = read_data_1(dev);
        if (t != 0xff) rv=1;
        //k9_reset(dev,5); // Unnecessary.
    }

    UNLOCK(dev);
    return rv;
}

CYG_NAND_FUNS_V2(k9f8_funs, k9_devinit,
        k9_read_begin, k9_read_stride, k9_read_finish,
        k9_write_begin, k9_write_stride, k9_write_finish,
        k9_erase_block, k9_factorybad);

