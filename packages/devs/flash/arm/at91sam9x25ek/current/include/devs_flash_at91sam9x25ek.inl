#ifndef CYGONCE_DEVS_FLASH_AT91SAM9X25_INL
#define CYGONCE_DEVS_FLASH_AT91SAM9X25_INL
//==========================================================================
//
//      devs_flash_at91sam9x25ek.inl
//
//      AT91SAM9X25 NAND Flash support
//
//==========================================================================
// ####ECOSGPLCOPYRIGHTBEGIN####                                            
// -------------------------------------------                              
// This file is part of eCos, the Embedded Configurable Operating System.   
// Copyright (C) 1998, 1999, 2000, 2001, 2002, 2003, 2004 Free Software Foundation, Inc.
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

//# ====================================================================
//######DESCRIPTIONBEGIN####
//#
//# Author(s):      Denis Mikhayevich <denis.mikhayevich@axonim.by>
//# Contributors:   
//# Date:           2011-08-30
//# Description:    FLASH drivers for NAND devices.
//#
//#####DESCRIPTIONEND####
//#
//# ====================================================================


#include <cyg/hal/hal_io.h> 
//#include "AT91SAM9GX5_inc.h"

#define AT91C_SMARTMEDIA_BASE	0x40000000
#define AT91_SMART_MEDIA_ALE    (1 << 21)       /* our ALE is AD21 */
#define AT91_SMART_MEDIA_CLE    (1 << 22)       /* our CLE is AD22 */


#define AT91C_BASE_SMC            (0xFFFFEA00)  // (SMC) Base Address
#define AT91C_BASE_CCFG           (0xFFFFDF10)  // (CCFG) Base Address

#define 	AT91C_EBI_CS3A_SM                   (0x1UL <<  3) 
#define AT91C_EBI_NFD0_ON_D16     (0x1UL << 24)
#define AT91C_EBI_DDR_MP_EN       (0x1UL << 25)

#define SMC_SETUP3      (48)    //  Setup Register for CS 3
#define SMC_PULSE3      (52)    //  Pulse Register for CS 3
#define SMC_CYCLE3      (56)    //  Cycle Register for CS 3
#define SMC_CTRL3       (60)    //  Control Register for CS 3

#define AT91C_SM_NWE_SETUP	(1 << 0)
#define AT91C_SM_NCS_WR_SETUP	(0 << 8)
#define AT91C_SM_NRD_SETUP	(1 << 16)
#define AT91C_SM_NCS_RD_SETUP	(0 << 24)

#define AT91C_SM_NWE_PULSE 	(3 << 0)
#define AT91C_SM_NCS_WR_PULSE	(5 << 8)
#define AT91C_SM_NRD_PULSE	(4 << 16)
#define AT91C_SM_NCS_RD_PULSE	(6 << 24)

#define AT91C_SM_NWE_CYCLE 	(5 << 0)
#define AT91C_SM_NRD_CYCLE	(6 << 16)

#define AT91C_SM_TDF	        (1 << 16)
#define CCFG_EBICSA     (16)
#define AT91C_EBI_DRV             (0x1UL << 17) 
#define AT91C_SMC_READMODE        (0x1UL <<  0) // (SMC) Read Mode
#define AT91C_SMC_WRITEMODE       (0x1UL <<  1) // (SMC) Write Mode
#define AT91C_SMC_NWAITM          (0x3UL <<  4) // (SMC) NWAIT Mode
#define 	AT91C_SMC_NWAITM_NWAIT_DISABLE        (0x0UL <<  4) 
#define 	AT91C_SMC_DBW_WIDTH_EIGTH_BITS     (0x0UL << 12) 

#define CM_REV_MASK			0x1F
#define CM_REV_OFFSET			0

#define NAND_DISABLE_CE() HAL_ARM_AT91_GPIO_SET(AT91_GPIO_PD4)
#define NAND_ENABLE_CE() HAL_ARM_AT91_GPIO_RESET(AT91_GPIO_PD4)

#define CYGNUM_FLASH_BLANK	(1)
#define CYGNUM_FLASH_INTERLEAVE	(1)
#define CYGNUM_FLASH_WIDTH      (8)
#define CYGNUM_FLASH_SERIES	(1)


//#define REV_Ax_EMBEST

#define WRITE_NAND_COMMAND(d) do { \
	*(volatile unsigned char *) \
	((unsigned long)AT91C_SMARTMEDIA_BASE | AT91_SMART_MEDIA_CLE) = \
	(unsigned char)(d); \
	} while(0)

#define WRITE_NAND_ADDRESS(d) do { \
	*(volatile unsigned char *) \
	((unsigned long)AT91C_SMARTMEDIA_BASE | AT91_SMART_MEDIA_ALE) = \
	(unsigned char)(d); \
	} while(0)

#define WRITE_NAND(d) do { \
	*(volatile unsigned char *) \
	((unsigned long)AT91C_SMARTMEDIA_BASE) = (unsigned char)d; \
	} while(0)

#define READ_NAND() ((unsigned char)(*(volatile unsigned char *) \
	(unsigned long)AT91C_SMARTMEDIA_BASE))


//-------------------------------------------------------------------------- 
// Platform specific access to control-lines 
//-------------------------------------------------------------------------- 
 
#define CYGHWR_FLASH_NAND_PLF_INIT      nand_plf_init 
#define CYGHWR_FLASH_NAND_PLF_CE        nand_plf_ce 
#define CYGHWR_FLASH_NAND_PLF_WP        nand_plf_wp 
#define CYGHWR_FLASH_NAND_PLF_CMD       nand_plf_cmd 
#define CYGHWR_FLASH_NAND_PLF_ADDR      nand_plf_addr 
#define CYGHWR_FLASH_NAND_PLF_WAIT      nand_plf_wait 
 

//-------------------------------------------------------------------------- 
// Global variables 
//-------------------------------------------------------------------------- 
 
// The device-specific data 
static cyg_nand_dev nand_device = 
{ 
    .flash_base         = (void*) (AT91C_SMARTMEDIA_BASE),
    .addr_r             = (void*) (AT91C_SMARTMEDIA_BASE),
    .addr_w             = (void*) (AT91C_SMARTMEDIA_BASE),
     
    .delay_cmd          = 10, 
    .delay_rst          = 500, 
}; 

void NAND_WAIT_READY()
{
	unsigned int BIT;
	
	do {
	        HAL_ARM_AT91_GPIO_GET(AT91_GPIO_PD5, BIT);
	} while (!BIT);   
}

//-------------------------------------------------------------------------- 
// Init platform nand 
//-------------------------------------------------------------------------- 
 
static inline void nand_plf_init(void) 
{ 

    unsigned int reg;
    // Init NAND Flash platform stuff 

    HAL_READ_UINT32( AT91C_BASE_CCFG + CCFG_EBICSA, reg );
    reg |= (1 << 3) | (1 << 24);

//    reg |= (AT91C_EBI_DDR_MP_EN | AT91C_EBI_NFD0_ON_D16);
//    reg &= ~AT91C_EBI_DRV;
    HAL_WRITE_UINT32(AT91C_BASE_CCFG + CCFG_EBICSA, reg);
    /*
     * Configure SMC CS3 
     */
    
    HAL_WRITE_UINT32(AT91C_BASE_SMC + SMC_SETUP3, (AT91C_SM_NWE_SETUP | AT91C_SM_NCS_WR_SETUP | AT91C_SM_NRD_SETUP | AT91C_SM_NCS_RD_SETUP));
    HAL_WRITE_UINT32(AT91C_BASE_SMC + SMC_PULSE3, (AT91C_SM_NWE_PULSE | AT91C_SM_NCS_WR_PULSE | AT91C_SM_NRD_PULSE | AT91C_SM_NCS_RD_PULSE));
    HAL_WRITE_UINT32(AT91C_BASE_SMC + SMC_CYCLE3, (AT91C_SM_NWE_CYCLE | AT91C_SM_NRD_CYCLE));
    HAL_WRITE_UINT32(AT91C_BASE_SMC + SMC_CTRL3, (AT91C_SMC_READMODE | AT91C_SMC_WRITEMODE | AT91C_SMC_NWAITM_NWAIT_DISABLE | AT91C_SMC_DBW_WIDTH_EIGTH_BITS | AT91C_SM_TDF));

    // NAND PIO Init
    HAL_ARM_AT91_GPIO_CFG_PULLUP(AT91_GPIO_PD2, AT91_PIN_PULLUP_ENABLE);
    HAL_ARM_AT91_GPIO_CFG_DIRECTION(AT91_GPIO_PD2, AT91_PIN_OUT);

    HAL_ARM_AT91_GPIO_CFG_PULLUP(AT91_GPIO_PD0, AT91_PIN_PULLUP_ENABLE);       // NANDOE
    HAL_ARM_AT91_GPIO_CFG_PULLUP(AT91_GPIO_PD1, AT91_PIN_PULLUP_ENABLE);       // NANDWE
    HAL_ARM_AT91_GPIO_CFG_PULLUP(AT91_GPIO_PD2, AT91_PIN_PULLUP_ENABLE);       // NANDALE
    HAL_ARM_AT91_GPIO_CFG_PULLUP(AT91_GPIO_PD3, AT91_PIN_PULLUP_ENABLE);       // NANDCLE
    HAL_ARM_AT91_GPIO_CFG_PULLUP(AT91_GPIO_PD4, AT91_PIN_PULLUP_ENABLE);       // NANDCS
    HAL_ARM_AT91_GPIO_CFG_PULLUP(AT91_GPIO_PD5, AT91_PIN_PULLUP_ENABLE);       // RDY_BSY
    HAL_ARM_AT91_GPIO_CFG_PULLUP(AT91_GPIO_PD6, AT91_PIN_PULLUP_ENABLE);       // D0
    HAL_ARM_AT91_GPIO_CFG_PULLUP(AT91_GPIO_PD7, AT91_PIN_PULLUP_ENABLE);       // D1
    HAL_ARM_AT91_GPIO_CFG_PULLUP(AT91_GPIO_PD8, AT91_PIN_PULLUP_ENABLE);       // D2
    HAL_ARM_AT91_GPIO_CFG_PULLUP(AT91_GPIO_PD9, AT91_PIN_PULLUP_ENABLE);       // D3
    HAL_ARM_AT91_GPIO_CFG_PULLUP(AT91_GPIO_PD10, AT91_PIN_PULLUP_ENABLE);      // D4
    HAL_ARM_AT91_GPIO_CFG_PULLUP(AT91_GPIO_PD11, AT91_PIN_PULLUP_ENABLE);      // D5
    HAL_ARM_AT91_GPIO_CFG_PULLUP(AT91_GPIO_PD12, AT91_PIN_PULLUP_ENABLE);      // D6
    HAL_ARM_AT91_GPIO_CFG_PULLUP(AT91_GPIO_PD13, AT91_PIN_PULLUP_ENABLE);      // D7

    HAL_ARM_AT91_PIO_CFG(AT91_NANDOE);
    HAL_ARM_AT91_PIO_CFG(AT91_NANDWE);
    HAL_ARM_AT91_PIO_CFG(AT91_NANDALE);
    HAL_ARM_AT91_PIO_CFG(AT91_NANDCLE);
    HAL_ARM_AT91_GPIO_CFG_DIRECTION(AT91_GPIO_PD4, AT91_PIN_OUT);
    HAL_ARM_AT91_GPIO_CFG_DIRECTION(AT91_GPIO_PD5, AT91_PIN_IN);
    HAL_ARM_AT91_PIO_CFG(AT91_PIN(3, 0, 6));
    HAL_ARM_AT91_PIO_CFG(AT91_PIN(3, 0, 7));    
    HAL_ARM_AT91_PIO_CFG(AT91_PIN(3, 0, 9));
    HAL_ARM_AT91_PIO_CFG(AT91_PIN(3, 0, 10));    
    HAL_ARM_AT91_PIO_CFG(AT91_PIN(3, 0, 11));
    HAL_ARM_AT91_PIO_CFG(AT91_PIN(3, 0, 12));    
    HAL_ARM_AT91_PIO_CFG(AT91_PIN(3, 0, 13));    

    // Reset NAND FLASH
    NAND_ENABLE_CE();
    WRITE_NAND_COMMAND(0xFF);
    NAND_WAIT_READY();
    NAND_WAIT_READY();
    NAND_DISABLE_CE();
} 
 
//-------------------------------------------------------------------------- 
// Enable/disable nand chip 
//-------------------------------------------------------------------------- 
 
static inline void nand_plf_ce(int state) 
{ 
    if(state) 
    { 
        NAND_ENABLE_CE();
    } 
    else 
    { 
        NAND_DISABLE_CE();
    } 
} 
 
//-------------------------------------------------------------------------- 
// Enable/disable write protect 
//-------------------------------------------------------------------------- 
 
static inline void nand_plf_wp(int nowrite) 
{ 
    if(nowrite) 
    { 
        // Enable WP 
    } 
    else 
    { 
        // Disable WP 
    } 
} 
 
//-------------------------------------------------------------------------- 
// Write nand command 
//-------------------------------------------------------------------------- 
 
static inline void nand_plf_cmd(int cmd) 
{ 
    // Enable CLE line 
    // Write command 
    // Disable CLE line 
    WRITE_NAND_COMMAND(cmd); 


} 
 
//-------------------------------------------------------------------------- 
// Write nand address 
//-------------------------------------------------------------------------- 
 
static inline void nand_plf_addr(int addr) 
{ 
    // Enable ALE line 
    // Write address 
    // Disable ALE line
    WRITE_NAND_ADDRESS(addr);
} 
 
//-------------------------------------------------------------------------- 
// Wait device ready pin 
//-------------------------------------------------------------------------- 
 
static inline void nand_plf_wait(void) 
{ 
    NAND_WAIT_READY();
} 
 
//-------------------------------------------------------------------------- 


#endif // CYGONCE_DEVS_FLASH_AT91SAM9X25_INL

// EOF devs_flash_at91sam9x25ek.inl
