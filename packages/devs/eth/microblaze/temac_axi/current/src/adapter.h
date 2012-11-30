//==========================================================================
//
//      adapter.h
//
//      Emaclite support
//      Taken from the Xilinx VIRTEX4 ethernet driver
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
// Author(s):    Michal Pfeifer
// Contributors: 
// Date:         2003-09-23
//               2005-04-21
// Purpose:      
// Description:  
//              
//
//####DESCRIPTIONEND####
//
//==========================================================================

#ifndef TEMAC_ADAPTER_H     /* prevent circular inclusions */
#define TEMAC_ADAPTER_H     /* by using protection macros */

#include "src/xaxiethernet.h"
#include "src/xaxidma.h"
#include "src/xaxiethernet_hw.h"
#include "src/xil_assert.h"
#include "src/xil_io.h"
#include <pkgconf/hal_microblaze_platform.h>
#include "temacdma.h"

/*
 * Info kept about interface
 */

struct temac_info { 
	/* These fields should be defined by the implementation */
	int			int_tx_vector;
	int			int_rx_vector;
	char		*esa_key;
	unsigned char	enaddr[6];
	/* The rest of the structure is set up at runtime */
	XAxiEthernet		dev;
//#ifdef CYGPKG_NET
	cyg_interrupt	temac_tx_interrupt;
	cyg_handle_t	temac_tx_interrupt_handle;

	cyg_interrupt	temac_rx_interrupt;
	cyg_handle_t	temac_rx_interrupt_handle;
//#endif

	cyg_uint16      rx_buf_len;                 ///< this member stores length of the receive buffer
	cyg_uint8       rx_buf[TEMACDMA_MTU];      ///< this buffer is used to store receive data
};

#endif
