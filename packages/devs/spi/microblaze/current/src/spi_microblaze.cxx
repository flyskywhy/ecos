//==========================================================================
//
//      spi_microblaze.cxx
//
//      SPI driver for Microblaze
//
//==========================================================================
// ####ECOSGPLCOPYRIGHTBEGIN####                                            
// -------------------------------------------                              
// This file is part of eCos, the Embedded Configurable Operating System.   
// Copyright (C) 1998, 1999, 2000, 2001, 2002, 2009 Free Software Foundation, Inc.
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
// Author(s):    Kolb Alexandr
// Contributors: Li Zheng
// Date:         2011-11-12
// Purpose:      
// Description:  Ref to packages/devs/spi/arm/lpc2xxx/current/src/spi_lpc2xxx.cxx
//              
//####DESCRIPTIONEND####
//
//==========================================================================

/**
*
*@file spi_microblaze.cxx 
*
* Contains required functions of the eCos SPI driver. Based on Xilinx SPI functions
**/

#include <pkgconf/hal.h>
#include <pkgconf/io_spi.h>
#include <cyg/infra/diag.h>


#include <cyg/infra/cyg_type.h>
#include <cyg/infra/cyg_ass.h>
#include <cyg/hal/hal_io.h>
#include <cyg/hal/hal_if.h>
#include <cyg/hal/hal_intr.h>
#include <pkgconf/devs_spi_microblaze.h>
#include <cyg/io/spi_microblaze.h>
#include <cyg/error/codes.h>
#include "src/xspi.h"
#include <pkgconf/hal_microblaze_platform.h>
#include <stdlib.h>


#define SPI_SPCR_LSBF 0x200
#define SPI_SPCR_MSTR 0x100
#define SPI_SPCR_MSSA 0x80
#define SPI_SPCR_RXFR 0x40
#define SPI_SPCR_TXFR 0x20
#define SPI_SPCR_CPHA 0x10
#define SPI_SPCR_CPOL 0x08
#define SPI_SPCR_CSPE 0x02
#define SPI_SPCR_LOOP 0x01

#define SPI_SPSR_SLMS 0x20
#define SPI_SPSR_MODF 0x10
#define SPI_SPSR_TXFL 0x08
#define SPI_SPSR_TXEY 0x04
#define SPI_SPSR_RXFL 0x02
#define SPI_SPSR_RXEY 0x01

#define SPI_SPINT     0x01

#ifdef CYGPKG_DEVS_SPI_MICROBLAZE_BUS0
cyg_spi_microblaze_bus_t cyg_spi_microblaze_bus0;
CYG_SPI_DEFINE_BUS_TABLE(cyg_spi_microblaze_dev_t, 0);
#endif
#ifdef CYGPKG_DEVS_SPI_MICROBLAZE_BUS1
cyg_spi_microblaze_bus_t cyg_spi_microblaze_bus1;
CYG_SPI_DEFINE_BUS_TABLE(cyg_spi_microblaze_dev_t, 1);
#endif

/**
* Interrupt routine
*
* @param	vec is interrupt vector for SPI instance
*
* @param	data is a pointer of driver structure
*
* @return	None
*
* @note		None
*
**/
static cyg_uint32
spi_microblaze_isr(cyg_vector_t vec, cyg_addrword_t data)
{
  cyg_spi_microblaze_bus_t *bus = (cyg_spi_microblaze_bus_t  *) data;
  cyg_drv_interrupt_acknowledge(bus->spi_vect);
  XSpi_InterruptHandler(&bus->spi_dev);
  return CYG_ISR_HANDLED | CYG_ISR_CALL_DSR;
}



static void 
spi_microblaze_dsr(cyg_vector_t vec, cyg_ucount32 count, cyg_addrword_t data)
{
  cyg_drv_cond_signal(&(( cyg_spi_microblaze_bus_t  *) data)->spi_wait);
}

/**
* Get config function configures bus for device. This function has to contain baud rate settings
* but Xilinx SPI  core doesn't have a possibility of baud rate configuration at run time
*
* @param	device is a pointer to the SPI device to be worked on.
*
*
* @return	Status code
*
* @note None
*
**/

static int
spi_microblaze_get_config(cyg_spi_device *device, cyg_uint32 key, void *buf, 
                       cyg_uint32 *len)
{
  cyg_spi_microblaze_dev_t *dev = (cyg_spi_microblaze_dev_t *) device;
  cyg_spi_microblaze_bus_t  *bus = (cyg_spi_microblaze_bus_t  *) dev->spi_device.spi_bus;
  
  return ENOERR;
}

/**
* Set config function configures bus for device. This function has to contain baud rate settings
* but Xilinx SPI  core doesn't have a possibility of baud rate configuration at run time
*
* @param	device is a pointer to the SPI device to be worked on.
*
*
* @return	Status code
*
* @note None
*
**/

static int
spi_microblaze_set_config(cyg_spi_device *device, cyg_uint32 key, const void *buf, 
                       cyg_uint32 *len)
{
  cyg_spi_microblaze_dev_t *dev = (cyg_spi_microblaze_dev_t *) device;
  cyg_spi_microblaze_bus_t  *bus = (cyg_spi_microblaze_bus_t  *) dev->spi_device.spi_bus;
  
  return ENOERR;
}


/**
* Begin transaction function configures bus for device 
*
* @param	device is a pointer to the SPI device to be worked on.
*
* @return None
*
* @note None
*
*/
static void
spi_microblaze_begin(cyg_spi_device *device)
{
  cyg_spi_microblaze_dev_t *dev = (cyg_spi_microblaze_dev_t *) device;
  cyg_spi_microblaze_bus_t  *bus = (cyg_spi_microblaze_bus_t  *) dev->spi_device.spi_bus;
  XSpi_Start(bus->spi_dev);
}


/**
* This function transfers a buffer to a device, fill another buffer with data from the device
*
* @param	device is a pointer to the SPI device to be worked on.
*
* @param	polled is determine mode of functionality.
*
* @param	count identiﬁes the number of data items to be transferred.
*
* @param	tx_data the data to be transferred to the device. 
*
* @param	rx_data a buffer for the data to be received from the device. If the device does not generate any output then a null
* pointer can be used.
*
* @param	drop_cs determine a state of CS signal
*
* @return	None
*
* @note		None
**/
static void
spi_microblaze_transfer(cyg_spi_device *device, cyg_bool polled, cyg_uint32 count, const cyg_uint8 *tx_data, cyg_uint8 *rx_data, cyg_bool drop_cs)
{
  cyg_uint32 Status;
  cyg_spi_microblaze_dev_t *dev = (cyg_spi_microblaze_dev_t *) device;
  cyg_spi_microblaze_bus_t  *bus = 
  (cyg_spi_microblaze_bus_t  *) dev->spi_device.spi_bus;
  if(!count) return;

  XSpi_SetSlaveSelect(bus->spi_dev, 0x01);
  XSpi_IntrGlobalDisable(bus->spi_dev);
  Status =XSpi_Transfer(bus->spi_dev,(cyg_uint8 *)tx_data,rx_data,count);
  if(Status != XST_SUCCESS)
   diag_printf("Transfer failure, error code = %d\n",Status);

  if (drop_cs)
   XSpi_SetSlaveSelect(bus->spi_dev, 0x00);
}

/**
* Some devices require a number of clock ticks on the SPI bus between transfers 
* so that they can complete some internal processing. These ticks must happen 
* at the appropriate clock rate but no chip select should be asserted and no data 
* transfer will happen. cyg_spi_tick provides this functionality. 
*
* @param	device is a pointer to the SPI device to be worked on.
*
* @param	polled is determine mode of functionality.
*
* @param	count specifies the number of data items that would be transferred, 
* which in conjunction with the size of each data item determines the number of clock ticks
*
* @return	None
*
* @note		None
**/
static void
spi_microblaze_tick(cyg_spi_device *device, cyg_bool polled, cyg_uint32 count)
{
  cyg_uint8 tx_zero[8] = {0,0,0,0,0,0,0,0};
  spi_microblaze_transfer(device, polled, count, tx_zero, NULL,false);
}


/*****************************************************************************/
/**
*
* This function stops the SPI device by disabling the device itself.
*
* @param	device is a pointer to the SPI device to be worked on.
*
* @return	None
*
* @note		None
*
*
******************************************************************************/


static void
spi_microblaze_end(cyg_spi_device *device)
{
  cyg_spi_microblaze_dev_t *dev = (cyg_spi_microblaze_dev_t *) device;
  cyg_spi_microblaze_bus_t  *bus = (cyg_spi_microblaze_bus_t  *) dev->spi_device.spi_bus;
/*
* Use XSpi_Stop function from Xilinx drivers
*/
  XSpi_Stop(bus->spi_dev);
}


static void AssertPrint(char *FilenamePtr, int LineNumber){
    diag_printf("ASSERT: File Name: %s ", FilenamePtr);
    diag_printf("Line Number: %d\r\n",LineNumber);
}

/*****************************************************************************/
/**
*
* Initializes a specific Spi instance such that the driver is ready to use.
*
*
* @param	bus is a pointer to the cyg_spi_microblaze_bus_t instance to be worked on.
*
* @param	dev is a pointer of driver structure
*
* @param	vec is interrupt vector for SPI instance
*
* @param	prio is the interrupt priority of the SPI bus @which ISR
*
* @param	which is the bus number
*
* @return	None
*
* @note		None.
*
******************************************************************************/



static void 
spi_microblaze_init_bus(cyg_spi_microblaze_bus_t  *bus, 
                     cyg_addrword_t dev,
                     cyg_vector_t vec,
                     cyg_priority_t prio,
                     cyg_uint32 which)
{
  cyg_uint32 Status;
  u32 control;
  diag_printf("__Start SPI Initialization__\n");
  XSpi *spi_dev = (XSpi *) malloc(sizeof(XSpi));
  extern XSpi_Config XSpi_ConfigTable[0];

/*
* Initialization of SPI bus structure
*/
  bus->spi_bus.spi_transaction_begin    = spi_microblaze_begin;
  bus->spi_bus.spi_transaction_transfer = spi_microblaze_transfer;
  bus->spi_bus.spi_transaction_tick     = spi_microblaze_tick;
  bus->spi_bus.spi_transaction_end      = spi_microblaze_end;
  bus->spi_bus.spi_get_config           = spi_microblaze_get_config;
  bus->spi_bus.spi_set_config           = spi_microblaze_set_config;
  CYG_SPI_BUS_COMMON_INIT(&bus->spi_bus);
  
  cyg_drv_mutex_init(&bus->spi_lock);
  cyg_drv_cond_init(&bus->spi_wait, &bus->spi_lock);
  
  bus->spi_dev = spi_dev;
  bus->spi_vect = vec;
  bus->spi_prio = prio;
  cyg_drv_interrupt_create(  
                           vec, prio, (cyg_addrword_t) bus,
                           &spi_microblaze_isr, &spi_microblaze_dsr,
                           &bus->spi_hand, &bus->spi_intr);
  cyg_drv_interrupt_attach(bus->spi_hand);
/*
* Initialize SPI core using XSpi_Initialize function from Xilinx driver
*/
  Status = XSpi_Initialize(bus->spi_dev, XSpi_ConfigTable[which].DeviceId);
  diag_printf("SPI base address: %x\n",XSpi_ConfigTable[which].BaseAddress);
  if(Status == XST_SUCCESS)
    diag_printf("!_!_! SPI init OK!_!_!\n");
  else
    diag_printf("SPI init failure(((\n");
/*
* Set control register of SPI core
*/
control = (XSP_CR_MASTER_MODE_MASK | XSP_CR_ENABLE_MASK | XSP_CR_MANUAL_SS_MASK);// | XSP_CR_LOOPBACK_MASK
XSpi_SetControlReg(bus->spi_dev,control);

}

/**
* Initialization class for SPI instance
**/
class cyg_spi_microblaze_init_class {
public:
  cyg_spi_microblaze_init_class(void) {
    cyg_uint32 addr, tmp;

  Xil_AssertSetCallback(AssertPrint);

#ifdef CYGPKG_DEVS_SPI_MICROBLAZE_BUS0
    spi_microblaze_init_bus(&cyg_spi_microblaze_bus0,
                         MON_SPI_0_BASE,
                         MON_SPI_0_INTR,
                         CYGNUM_IO_SPI_MICROBLAZE_BUS0_INTPRIO,
                         0);
#endif
#ifdef CYGPKG_DEVS_SPI_MICROBLAZE_BUS1
    spi_microblaze_init_bus(&cyg_spi_microblaze_bus1,
                         MON_SPI_1_BASE,
                         MON_SPI_1_INTR,
                         CYGNUM_IO_SPI_MICROBLAZE_BUS1_INTPRIO,
                         1);
#endif
  }
};

static cyg_spi_microblaze_init_class spi_microblaze_init 
    CYGBLD_ATTRIB_INIT_PRI(CYG_INIT_BUS_SPI);
