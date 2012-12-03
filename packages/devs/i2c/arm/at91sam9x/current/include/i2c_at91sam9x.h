#ifndef CYGONCE_DEVS_I2C_ARM_AT91SAM9X_H
#define CYGONCE_DEVS_I2C_ARM_AT91SAM9X_H
//==========================================================================
//
//      i2c_at91sam9x.h
//
//      Atmel AT91SAM9X (ARM) I2C driver defines
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

#include <pkgconf/hal.h>
#include <cyg/infra/cyg_type.h>
#include <cyg/hal/drv_api.h>
#include <cyg/io/i2c.h>

#define I2C_TXR_START    		(1 << 7)
#define I2C_TXR_END      		(1 << 6)
#define I2C_RXR_START    		(1 << 5)
#define I2C_RXR_END      		(1 << 4)

#define I2C_FLAG_AT91_COMPLETED		(1 << 0)
#define I2C_FLAG_AT91_ARBLOST		(1 << 1)
#define I2C_FLAG_AT91_NACK		(1 << 2)
#define I2C_FLAG_AT91_TIMEOUT		(1 << 3)


#define AT91_TWI_MMR_IADRZ_MASK  	(3 << 8) 	// Internal Device Address Size Mask
#define AT91_TWI_MMR_IADRZ_SHIFT  	(8)   	// Internal Device Address Size Shift

#define I2C_BASE        extra->i2c_twi_base
#define I2C_TXCOMP      AT91_TWI_SR_TXCOMP
#define I2C_RXRDY       AT91_TWI_SR_RXRDY
#define I2C_TXRDY       AT91_TWI_SR_TXRDY
#define I2C_NACK        AT91_TWI_SR_NACK

#define I2C_W8(r,x)     HAL_WRITE_UINT8(I2C_BASE + (r), (x))
#define I2C_R8(r,x)     HAL_READ_UINT8(I2C_BASE + (r), (x))
#define I2C_W32(r, x)   HAL_WRITE_UINT32(I2C_BASE + (r), (x))
#define I2C_R32(r, x)   HAL_READ_UINT32(I2C_BASE + (r), (x))

#define AT91_TWI_CR_QUICK	(1 << 6)

# define    I2C_BUS_FREQ(_extra_)   ((_extra_)->i2c_bus_freq)

#define COUNT_DOWN 12000
#define MANUAL_CLOCK_LOOP_COUNT	(10000)

#ifndef CYGOPT_IO_I2C_TIMEOUT
#define CYGOPT_IO_I2C_TIMEOUT	(2000)
#endif


typedef enum at91sam9_i2c_xfer_mode {
    AT91SAM9_I2C_XFER_MODE_INVALID = 0x00,
    AT91SAM9_I2C_XFER_MODE_TX = 0x01,
    AT91SAM9_I2C_XFER_MODE_RX = 0x02
} at91sam9_i2c_xfer_mode;

typedef struct cyg_at91sam9x_i2c_extra {
    cyg_uint32	     i2c_twi_base;
    cyg_uint8        i2c_twi_num;
    cyg_uint8        i2c_addr;
    cyg_uint32       i2c_count;
    cyg_uint8*       i2c_txbuf;
    cyg_uint8*	     i2c_rxbuf;
    cyg_bool         i2c_rxnak;
    cyg_bool         i2c_send_start;
    cyg_bool         i2c_send_stop;
    cyg_bool         i2c_send_nack;  
    at91sam9_i2c_xfer_mode i2c_mode;       	// TX, RX, ...

    cyg_uint32       i2c_flag;
    cyg_uint32       i2c_delay;

    cyg_uint8        i2c_lost_arb;      	// Error condition leading to loss of

    cyg_drv_mutex_t  i2c_lock; 			// For synchronizing between DSR and foreground
    cyg_drv_cond_t   i2c_wait;
    cyg_handle_t     i2c_interrupt_handle;	// For initializing the interrupt
    cyg_uint8        i2c_completed;     	// Set by DSR, checked by thread
    cyg_interrupt    i2c_interrupt_data;

    cyg_uint32      i2c_isr_id;
    cyg_uint32      i2c_isr_pri;
    // lower layer data
    cyg_uint8 int_addr_sz;    // most chips do have internal address size
} cyg_at91sam9x_i2c_extra;

typedef struct cyg_i2c_at91sam9x_dev_s
{
    // upper layer data (upper layer i2c device structure)
    cyg_i2c_device i2c_device;
    cyg_uint8        i2c_addr;
    cyg_uint32       i2c_count;

    // lower layer data
    cyg_uint8 int_addr_sz;    // most chips do have internal address size
} cyg_i2c_at91sam9x_dev_t;

#define CYG_I2C_AT91SAM9X_DEVICE_(_name_, _bus_, _address_, _flags_, _delay_, _int_adr_sz_) \
    static cyg_at91sam9x_i2c_extra _name_ = { \
                .i2c_device = { \
                .i2c_bus = _bus_, \
                .i2c_address = _address_, \
                .i2c_flags = _flags_, \
                .i2c_delay = _delay_, \
            }, \
            .int_addr_sz = _int_adr_sz_, \
    }
    
#define HAL_I2C_EXPORTED_DEVICES \
   // externC cyg_i2c_bus cyg_i2c_at91sam9x_bus0;
#endif // CYGONCE_DEVS_I2C_ARM_AT91SAM9X_H 


//==========================================================================
// I2C driver interface
//==========================================================================
void        i2c_at91sam9x_init(struct cyg_i2c_bus*);
cyg_uint32  i2c_at91sam9x_tx(const cyg_i2c_device*, 
                                       cyg_bool, const cyg_uint8*, 
                                       cyg_uint32, cyg_bool);
cyg_uint32  i2c_at91sam9x_rx(const cyg_i2c_device*, 
                                       cyg_bool, cyg_uint8*, 
                                       cyg_uint32, cyg_bool, cyg_bool);
/*externC*/ void        i2c_at91sam9x_stop(const cyg_i2c_device*);



//==========================================================================
// I2C bus declaration macros
//=========================================================================

# define CYG_AT91SAM9X_I2C_BUS(_name_, _twi_num_, _isr_pri_) \
   static cyg_at91sam9x_i2c_extra _name_ ## _extra = {                   \
       .i2c_twi_num = _twi_num_,                                        \
       .i2c_isr_pri = _isr_pri_                      			\
  } ;                                                                   \
  CYG_I2C_BUS(_name_,                                                   \
              i2c_at91sam9x_init,                                   \
              i2c_at91sam9x_tx,                                     \
              i2c_at91sam9x_rx,                                     \
              i2c_at91sam9x_stop,                                   \
              (void*) & ( _name_ ## _extra)) ;


