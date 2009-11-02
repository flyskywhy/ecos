#ifndef CYGONCE_NAND_ECC_H
# define CYGONCE_NAND_ECC_H
//=============================================================================
//
//      nand_ecc.h
//
//      ECC algorithm interface for the eCos NAND flash library
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

#include <cyg/infra/cyg_type.h>

struct _cyg_nand_device_t;

/* ECC algorithm parameterisation block. */
typedef struct {
    /* How many data bytes does this algorithm read? */
    const unsigned  data_size;
    /* How many ECC bytes does this algorithm use per data block? */
    const unsigned  ecc_size;
    /* Calculates the ECC for the given data block */
    void (*calc)(const CYG_BYTE *dat, CYG_BYTE *ecc);
    /* Repairs the ECC for the given data block, if needed.
     * Call this if your read-from-chip ECC doesn't match what you computed
     * over the data block. Both *dat and *ecc_read may be corrected.
     * Returns: 
     *       0 for no errors
     *       1 for a corrected single bit error in the data
     *       2 for a corrected single bit error in the ECC
     *      -1 for an uncorrectable error (more than one bit)
     */
    int (*repair)(CYG_BYTE *dat, CYG_BYTE *ecc_read, const CYG_BYTE *ecc_calc);
} cyg_nand_ecc_t;

#define CYG_NAND_ECC_ALG(_name, _dataz, _eccz, _calc, _repair)  \
    cyg_nand_ecc_t _name = {                            \
        .data_size = _dataz,                                    \
        .ecc_size = _eccz,                                      \
        .calc = _calc,                                          \
        .repair = _repair,                                      \
    }

/* Useful code ==================================================== */

/* Quick & dirty calc: no of ECC bytes per page.
 * Assumes that ecc->data_size will only ever be a power of two. */
#define CYG_NAND_ECCPERPAGE(dev) ( (dev)->ecc->ecc_size * (1<<(dev)->page_bits) / (dev)->ecc->data_size )

/* Computes the ECC for a whole device page.
 * 'page' points to the data; a whole page will necessarily be read.
 * The computed ECC will be stored in 'ecc_o'; CYG_NAND_ECCPERPAGE(dev)
 * bytes will be written. */
void nand_ecci_calc_page(struct _cyg_nand_device_t *dev,
                         const CYG_BYTE *page, CYG_BYTE *ecc_o);

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
int nand_ecci_repair_page(struct _cyg_nand_device_t *dev,
                          CYG_BYTE *page,
                          CYG_BYTE *ecc_read, const CYG_BYTE *ecc_calc);

/* Implementations ================================================ */
// TODO: Hide by CDL, when more than one is added.

/* The ECC algorithm used by the Linux MTD layer. See nand_ecc_mtd.c. */
__externC cyg_nand_ecc_t linux_mtd_ecc;

#endif
