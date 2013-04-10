#ifndef CYGONCE_DEVS_SPI_MICROBLAZE_H
#define CYGONCE_DEVS_SPI_MMICROBLAZE_H

//==========================================================================
//
//      spi_microblaze.h
//
//      SPI driver for Microblaze
//
//==========================================================================
// ####ECOSGPLCOPYRIGHTBEGIN####                                            
// -------------------------------------------                              
// This file is part of eCos, the Embedded Configurable Operating System.   
// Copyright (C) 1998, 1999, 2000, 2001, 2002 Free Software Foundation, Inc.
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
// Author(s):    Alexander Kolb
// Contributors: Li Zheng
// Date:         2011-10-12
// Purpose:      
// Description:  Ref to packages/devs/spi/arm/lpc2xxx/current/include/spi_lpc2xxx.h
//              
//####DESCRIPTIONEND####
//
//==========================================================================

#include <pkgconf/hal.h>
#include <pkgconf/io_spi.h>
#include <cyg/io/spi.h>

#include <cyg/infra/cyg_type.h>
#include <cyg/hal/drv_api.h>
//#include "src/xspi.h"
#include "src/xspi.h"


#include <pkgconf/hal_microblaze_platform.h>

typedef struct cyg_spi_microblaze_bus_s {
	cyg_spi_bus	spi_bus;
	cyg_interrupt   spi_intr;
	cyg_handle_t    spi_hand;
	cyg_vector_t    spi_vect;
	cyg_priority_t  spi_prio;
	cyg_drv_mutex_t spi_lock;
	cyg_drv_cond_t  spi_wait;
	XSpi xspi;
	XSpi *spi_dev;

	volatile cyg_uint32       count;
	volatile const cyg_uint8 *tx;
	volatile cyg_uint8       *rx;
} cyg_spi_microblaze_bus_t;

typedef struct cyg_spi_microblaze_dev_s {
  cyg_spi_device  spi_device;
  cyg_uint8       spi_cpha;
  cyg_uint8       spi_cpol;
  cyg_uint8       spi_lsbf;
} cyg_spi_microblaze_dev_t;

struct spi_config {
  XSpi_Config *conf;
};

/* For packages/devs/disk/generic/mmc/current/src/mmc_spi.c */
//#define cyg_spi_mmc_dev0 spi_mmc_dev0.spi_device;

#ifdef CYGPKG_DEVS_SPI_MICROBLAZE_BUS0
externC cyg_spi_microblaze_bus_t cyg_spi_microblaze_bus0;
#endif
#ifdef CYGPKG_DEVS_SPI_MICROBLAZE_BUS1
externC cyg_spi_microblaze_bus_t cyg_spi_microblaze_bus1;
#endif

#endif // CYGONCE_DEVS_SPI_MMICROBLAZE_H

//-----------------------------------------------------------------------------
// End of spi_microblaze.h
