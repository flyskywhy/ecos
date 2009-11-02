//=============================================================================
//
//      nand_oob.c
//
//      eCos NAND flash library - Out Of Band (spare) area pack/unpack/defs
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
// Date:        2009-03-13
//
//####DESCRIPTIONEND####
//=============================================================================

#include <string.h>
#include <cyg/nand/nand_device.h>
#include <cyg/nand/nand_oob.h>
#include CYGBLD_ISO_ERRNO_CODES_HEADER

static
void pack(const oob_vector *v, int n_v, const CYG_BYTE *data, int n_data, CYG_BYTE *oob)
{
    int i;
    for (i=0; i<n_v; i++) {
        int stride = v[i].len;
        if (stride > n_data) stride = n_data;
        if (!stride) break;
        memcpy(&oob[v[i].pos], data, stride);
        data += stride;
        n_data -= stride;
    }
}

static
void unpack(const oob_vector *v, int n_v, CYG_BYTE *data_o, int data_max, const CYG_BYTE *oob)
{
    int i;
    for (i=0; i<n_v; i++) {
        int stride = v[i].len;
        if (stride > data_max) stride = data_max;
        if (!stride) break;
        memcpy(data_o, &oob[v[i].pos], stride);
        data_o += stride;
        data_max -= stride;
    }
}

__externC
void nand_oob_pack( struct _cyg_nand_device_t *dev,
                    const CYG_BYTE *app, const unsigned app_size,
                    const CYG_BYTE *ecc, CYG_BYTE *oob_o)
{
    const cyg_nand_oob_layout *layout = dev->oob;
    unsigned real_size = app_size;
    if(real_size > layout->app_size) real_size = layout->app_size;
    pack(layout->ecc, CYG_NAND_OOB_MAX_ECC_SLOTS, ecc, ecc? layout->ecc_size : 0, oob_o);
    pack(layout->app, CYG_NAND_OOB_MAX_APP_SLOTS, app, app_size, oob_o);
}

__externC
void nand_oob_unpack(struct _cyg_nand_device_t *dev,
                     CYG_BYTE *app_o, const unsigned app_max,
                     CYG_BYTE *ecc, const CYG_BYTE *oob)
{
    const cyg_nand_oob_layout *layout = dev->oob;
    unpack(layout->ecc, CYG_NAND_OOB_MAX_ECC_SLOTS, ecc, ecc? layout->ecc_size : 0, oob);
    unpack(layout->app, CYG_NAND_OOB_MAX_APP_SLOTS, app_o, app_max, oob);
}

/* Layouts from the Linux MTD layer. */

#if 0
/*
 * WARNING: 8 spare bytes per page WILL NOT WORK without further work on 
 * the BBT layer. (This is because the standard location for the BBT 
 * identity pattern is at offset 8.) Before enabling this,
 * ensure that the BBT layer has an alternative way of determining the
 * location of the on-chip BBT, if there is one, or other alternative
 * bad-block logic.
 */
const cyg_nand_oob_layout nand_mtd_oob_8 = {
    .ecc_size = 3,
    .ecc = { { .pos=0, .len=3 } },
    .app_size = 4,
    .app = { { .pos=3, .len=2 }, { .pos=6, .len=2} },
};
#endif

const cyg_nand_oob_layout nand_mtd_oob_16 = {
    .ecc_size = 6,
    .ecc = { { .pos=0, .len=4 }, { .pos=6, .len=2 } },
    .app_size = 8,
    .app = { { .pos=8, .len=8} },
};


const cyg_nand_oob_layout nand_mtd_oob_64 = {
    .ecc_size = 24,
    .ecc = { { .pos=40, .len=24 } },
    .app_size = 38,
    .app = { { .pos=2, .len=38} },
};

