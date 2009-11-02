//==========================================================================
//
//      nand.c
//
//      RedBoot - NAND library support
//
//==========================================================================
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
//==========================================================================
//#####DESCRIPTIONBEGIN####
//
// Author(s):    wry
// Date:         2009-05-29
// Purpose:      
// Description:  
//              
// This code is part of RedBoot (tm).
//
//####DESCRIPTIONEND####
//
//==========================================================================

#include <redboot.h>
#include <pkgconf/system.h>
#ifdef CYGPKG_IO_FILEIO
# include <cyg/fileio/fileio.h>
#endif
#include <cyg/nand/nand.h>
#include <cyg/nand/nand_device.h>
#include <cyg/nand/util.h>

CYG_HAL_TABLE_BEGIN( __NAND_cmds_TAB__, NAND_cmds);
CYG_HAL_TABLE_END( __NAND_cmds_TAB_END__, NAND_cmds);

extern struct cmd __NAND_cmds_TAB__[], __NAND_cmds_TAB_END__;

//==========================================================================

static void
nand_usage(char *why)
{
    diag_printf("*** invalid 'nand' command: %s\n", why);
    cmd_usage(__NAND_cmds_TAB__, &__NAND_cmds_TAB_END__, "nand ");
}

static void
do_nand(int argc, char *argv[])
{
    struct cmd *cmd;

    if (argc < 2) {
        nand_usage("too few arguments");
        return;
    }
    if ((cmd = cmd_search(__NAND_cmds_TAB__, &__NAND_cmds_TAB_END__, 
                          argv[1])) != (struct cmd *)0) {
        (cmd->fun)(argc-1, argv+1);
        return;
    }
    nand_usage("unrecognized command");
}

RedBoot_nested_cmd("nand", "Manage NAND arrays", "{cmds}", do_nand, 
        __NAND_cmds_TAB__, &__NAND_cmds_TAB_END__);

//==========================================================================
// nand list - enumerates the available devices

static void nand_list_guts(const char *banner, const char *eachdev,
        const char *donefound, const char *donenone)
{
    cyg_bool found=false;
    cyg_nand_device *nand;
    for (nand = &cyg_nanddevtab[0]; nand != &cyg_nanddevtab_end; nand++) {
        if (!found) {
            diag_printf(banner);
            found=true;
        }
        diag_printf(eachdev, nand->devname);
    }
    if (found) {
        if (donefound)
            diag_printf(donefound);
    } else {
        if (donenone)
            diag_printf(donenone);
    }
}

static void nand_list(int argc, char*argv[])
{
    nand_list_guts("NAND devices available:\n", "\t%s\n", 
            0, "No NAND devices available.\n");
}

// Startup banner is suspiciously similar..
void _nand_info(void)
{
    nand_list_guts("NAND:", " %s", "\n", 0);
    // We could do more here, maybe interrogate each device like nand_info,
    // but we'd have to be certain that calling each device's devinit
    // wouldn't risk an unrecoverable crash in case of trouble.
}

local_cmd_entry("list", "Lists available NAND devices", "", nand_list, NAND_cmds);

//==========================================================================
// nand info DEVICE - list chip info, partitions & sizes

static void pretty_print(unsigned log) {
    char si = ' ';
    if (log>30) {
        si='G'; log -= 30;
    } else if (log>20) {
        si='M'; log -= 20;
    } else if (log>10) {
        si='k'; log -= 10;
    }
    diag_printf("%u %cB", 1<<log, si);
}

static void do_info(cyg_nand_device *nand)
{
    // Dev must be initialised.
    diag_printf("NAND device `%s':\n"
                "  %u bytes/page, %u pages/block, "
                "capacity %u blocks x ",
                nand->devname,
                1 << nand->page_bits,
                1 << nand->block_page_bits,
                1 << nand->blockcount_bits);
    pretty_print(nand->page_bits + nand->block_page_bits);
    diag_printf(" = ");
    pretty_print(nand->chipsize_log);
    diag_printf("\n");

    cyg_bool found = false;
    int p;
    for (p=0; p < CYGNUM_NAND_MAX_PARTITIONS; p++) {
        if (nand->partition[p].dev) {
            if (!found) {
                found=true;
                diag_printf("  Partition Start Blocks\n");
            }
            diag_printf("  %6u    %4u  %5u\n",
                    p,
                    nand->partition[p].first,
                    1 + nand->partition[p].last - nand->partition[p].first);
        }
    }
    if (!found)
        diag_printf("  (No partitions defined.)\n");
}

static void nand_info(int argc, char*argv[])
{
    cyg_nand_device *nand;

    int arg = 1;
    while (arg < argc) {
        const char *dev = argv[arg];
        if (0==cyg_nand_lookup(dev, &nand))
            do_info(nand);
        else
            diag_printf("No such NAND device `%s'\n", dev);
        ++arg;
    }

    // However, if they've not provided an arg ...
    if (argc == 1) {
        for (nand = &cyg_nanddevtab[0]; nand != &cyg_nanddevtab_end; nand++) {
            cyg_nand_lookup(nand->devname,0);
            do_info(nand);
        }
    }
}

local_cmd_entry("info", "Displays information about one or more NAND devices", "[<device>] [<device>...]", nand_info, NAND_cmds);

//==========================================================================
// nand erase - erases a nand partition. (Who'd have thunk?)

#ifdef CYGSEM_REDBOOT_NAND_ERASE_CMD

#ifdef CYGPKG_IO_FILEIO
__externC cyg_mtab_entry cyg_mtab[];
__externC cyg_mtab_entry cyg_mtab_end;

static cyg_mtab_entry * find_mounted_nand(const char *dev)
{
    cyg_mtab_entry *m;
    for( m = &cyg_mtab[0]; m != &cyg_mtab_end; m++ ) {
        if( m->name == NULL || !m->valid ) continue;
        if (0==strcmp(m->devname, dev)) {
            return m;
        }
    }
    return NULL;
}
#endif

static void nand_erase(int argc, char*argv[])
{
    char *part_str=0;
    int rv;

    if (!scan_opts(argc, argv, 1, NULL, 0, &part_str, OPTION_ARG_TYPE_STR, "NAND partition, e.g. mynand/0"))
        return;

    if (!part_str) {
        err_printf("nand erase: partition required\n");
        return;
    }

    cyg_nand_device *nand=0;
    cyg_nand_partition *part=0;
    cyg_nand_resolve_device(part_str, &nand, &part);
    if (!nand) {
        err_printf("nand erase: device not found\n");
        return;
    }
    if (!part) {
        err_printf("nand erase: partition not found\n");
        return;
    }

#ifdef CYGPKG_IO_FILEIO
    cyg_mtab_entry *mte = find_mounted_nand(part_str);
    if (mte) {
        err_printf("nand erase: device %s is mounted on %s, must unmount first\n", part_str, mte->name);
        return;
    }
#endif

    diag_printf("Erasing device %s blocks %u to %u...\n", nand->devname, part->first, part->last);
    cyg_nand_block_addr blk;
    // Compute a modulus to print out about 72 x `.' as we go by
    int progmod = (part->last - part->first + 1) / 73 + 1;

    for (blk = part->first; blk <= part->last; blk++) {
        int st = cyg_nand_bbt_query(part, blk);
        if (st<0) {
            diag_printf("Block %d BBTI error %d\n", blk, -st);
        }
        const char *msg = 0;
        switch(st) {
            case CYG_NAND_BBT_OK:
                rv = cyg_nand_erase_block(part, blk);
                if (rv != 0)
                    diag_printf("Block %d: error %d\n", blk, -rv);
                break;
            case CYG_NAND_BBT_WORNBAD:
                msg="worn bad"; break;
            case CYG_NAND_BBT_FACTORY_BAD:
                msg="factory bad"; break;

            case CYG_NAND_BBT_RESERVED:
                // Skip quietly
                break;
        }
        if (msg)
            diag_printf("Skipping block %d (%s)\n", blk, msg);

        if (0==(blk % progmod))
            diag_printf(".");
    }
    diag_printf("\nErase complete.\n");
}

local_cmd_entry("erase", "Erases a NAND partition", "<partition> (e.g. mynand/0)", nand_erase, NAND_cmds);

#endif // CYGSEM_REDBOOT_NAND_ERASE_CMD

//==========================================================================

// end nand.c
