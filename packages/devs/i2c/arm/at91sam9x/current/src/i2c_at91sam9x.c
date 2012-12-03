//==========================================================================
//
//      i2c_at91sam9x.c
//
//      I2C driver for AT91SAM9x 
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
// Author(s):     AXONIM Devices
//                AXONIM
//                Pavel Frolov
//                Artiom Staliarou
//			
// Date:          2011-08-10
//
//####DESCRIPTIONEND####
//
//==========================================================================

#include <pkgconf/system.h>
#include <pkgconf/devs_i2c_arm_at91sam9x.h>
#include <cyg/infra/cyg_type.h>
#include <cyg/infra/diag.h>
#include <cyg/io/devtab.h>
#include <cyg/io/i2c.h>
#include <cyg/io/i2c_at91sam9x.h>
#include <cyg/hal/hal_arch.h>
#include <cyg/hal/hal_io.h>
#include <cyg/hal/hal_intr.h>
#include <cyg/hal/drv_api.h>
#include <cyg/infra/diag.h>

static void
i2c_at91sam9x_send_manual_clocks(cyg_at91sam9x_i2c_extra *extra, cyg_uint32 numclocks);
void
i2c_at91sam9x_reinit(cyg_at91sam9x_i2c_extra * extra);

//////////////////////////////////////////////////////////////////////

static cyg_uint32
i2c_at91sam9x_isr(cyg_vector_t vec, cyg_addrword_t data)
{
    cyg_at91sam9x_i2c_extra *extra = (cyg_at91sam9x_i2c_extra *) data;
    cyg_uint32 stat_reg;
    cyg_uint32 mask_reg;
    cyg_uint32      result = CYG_ISR_HANDLED | CYG_ISR_CALL_DSR;
    volatile cyg_uint8       dr;

    // Read status & mask register
    I2C_R32 (AT91_TWI_SR, stat_reg);
    I2C_R32 (AT91_TWI_IMR, mask_reg);
    stat_reg &= mask_reg;

    // Switch states

    // What to do next depends on the current transfer mode.
    if (AT91SAM9_I2C_XFER_MODE_TX == extra->i2c_mode) {
	    
	    // Check if TXCOMP
	    if ((stat_reg & AT91_TWI_SR_TXCOMP) && (extra->i2c_count == 0))
	    {
		I2C_W32(AT91_TWI_IDR, AT91_TWI_SR_TXCOMP | AT91_TWI_SR_TXRDY);
		extra->i2c_flag |= I2C_FLAG_AT91_COMPLETED;

	    } else
	    // Check if TXRDY
	    if (stat_reg & AT91_TWI_SR_TXRDY)
	    {
   	        I2C_W8(AT91_TWI_THR, *extra->i2c_txbuf++);

                extra->i2c_count -= 1;

		if (extra->i2c_count == 0)
		{
			if (extra->i2c_send_stop)
			{
			    I2C_W32(AT91_TWI_CR, AT91_TWI_CR_STOP);
			    extra->i2c_send_stop = 0;
			}

		    I2C_W32(AT91_TWI_IER, AT91_TWI_SR_TXCOMP);
		    I2C_W32(AT91_TWI_IDR, AT91_TWI_SR_TXRDY);
		}

	    }

	    // Arbitration lost
	    if (stat_reg & AT91_TWI_SR_ARBLST) {
		I2C_W32(AT91_TWI_IDR, AT91_TWI_SR_RXRDY | AT91_TWI_SR_TXRDY | AT91_TWI_SR_TXCOMP | AT91_TWI_SR_ARBLST);
		extra->i2c_flag |= I2C_FLAG_AT91_ARBLOST;
	    }	

	    // NACK
	    if (stat_reg & AT91_TWI_SR_NACK) {
		I2C_W32(AT91_TWI_IDR, AT91_TWI_SR_RXRDY | AT91_TWI_SR_TXRDY | AT91_TWI_SR_TXCOMP | AT91_TWI_SR_ARBLST | AT91_TWI_SR_NACK);
		extra->i2c_flag |= I2C_FLAG_AT91_NACK;
	    }	
	
    } else if (AT91SAM9_I2C_XFER_MODE_RX == extra->i2c_mode) {

	    // Check if TXCOMP
	    if (stat_reg & AT91_TWI_SR_TXCOMP)
	    {
		I2C_W32(AT91_TWI_IDR, AT91_TWI_SR_TXCOMP | AT91_TWI_SR_RXRDY);
		extra->i2c_flag |= I2C_FLAG_AT91_COMPLETED;
		
	    } else 	 
	    // Check if RX
	    if (stat_reg & AT91_TWI_SR_RXRDY) 
	    {

		I2C_R8(AT91_TWI_RHR, dr); 
		*(extra->i2c_rxbuf) = (cyg_uint8)dr;
		extra->i2c_rxbuf += 1;
		extra->i2c_count -= 1;

		switch(extra->i2c_count)
		{
		case 1:
			if (extra->i2c_send_stop)
			{
				I2C_W32(AT91_TWI_CR, AT91_TWI_CR_STOP);
				extra->i2c_send_stop = 0;
			}
			break;
		case 0:
			I2C_W32(AT91_TWI_IER, AT91_TWI_SR_TXCOMP);
			I2C_W32(AT91_TWI_IDR, AT91_TWI_SR_RXRDY);
			break;
		default:
			break;
		}

		
	    }

	    // Arbitration lost
	    if (stat_reg & AT91_TWI_SR_ARBLST) {
		I2C_W32(AT91_TWI_IDR, AT91_TWI_SR_RXRDY | AT91_TWI_SR_TXRDY | AT91_TWI_SR_TXCOMP | AT91_TWI_SR_ARBLST | AT91_TWI_SR_NACK);
		extra->i2c_flag |= I2C_FLAG_AT91_ARBLOST;

	    }	

	    // NACK
	    if (stat_reg & AT91_TWI_SR_NACK) {
		I2C_W32(AT91_TWI_IDR, AT91_TWI_SR_RXRDY | AT91_TWI_SR_TXRDY | AT91_TWI_SR_TXCOMP | AT91_TWI_SR_ARBLST | AT91_TWI_SR_NACK);
		extra->i2c_flag |= I2C_FLAG_AT91_NACK;
	    }	

    } else {
        // Invalid state? Some kind of spurious interrupt?
        // Just ignore it.
    }

    HAL_INTERRUPT_ACKNOWLEDGE(extra->i2c_isr_id);

    return result;

}

static void
i2c_at91sam9x_dsr(cyg_vector_t vec, cyg_ucount32 count, cyg_addrword_t data)
{
    cyg_at91sam9x_i2c_extra *extra = (cyg_at91sam9x_i2c_extra *) data;
    extra->i2c_completed = 1;
    cyg_drv_cond_signal(&extra->i2c_wait);
}

// set up I2C bus timing
static void
i2c_at91sam9x_delay(cyg_at91sam9x_i2c_extra *extra, cyg_uint32 delay)
{
    cyg_uint32 cdiv,ckdiv = 0;

    // (delay / 10^9) for converting delay nsec to sec
    // formula from datasheet => Tlow = ((CLDIV * 2^CKDIV) + 3) * Tmck
    // CLDIV = CHDIV = CDIV
    cdiv =  (delay / 1000) * (CYGNUM_HAL_ARM_AT91_CLOCK_SPEED / (2 * 1000000));
    cdiv -= 3;
    while (cdiv > 255) {
        ckdiv++;
        cdiv >>= 1;
    }
    I2C_W32(AT91_TWI_CWGR,ckdiv << AT91_TWI_CWGR_CKDIV_SHIFT |
        cdiv << AT91_TWI_CWGR_CHDIV_SHIFT | cdiv << AT91_TWI_CWGR_CLDIV_SHIFT);
    extra->i2c_delay = delay;
}

static void
i2c_at91sam9x_send_manual_clocks(cyg_at91sam9x_i2c_extra *extra, cyg_uint32 numclocks)
{
    	cyg_uint32 i, delay;
	cyg_uint32 gpio;


        if (!extra->i2c_twi_num){
	   HAL_WRITE_UINT32((AT91_PMC + AT91_PMC_PCER), AT91_PMC_PCER_PIOA);
   	   gpio = AT91_GPIO_PA21;
	} else {
	   HAL_WRITE_UINT32((AT91_PMC + AT91_PMC_PCER), AT91_PMC_PCER_PIOB);
	   gpio = AT91_GPIO_PB11;
	}

  	HAL_ARM_AT91_GPIO_CFG_PULLUP(gpio, AT91_PIN_PULLUP_DISABLE);
  	HAL_ARM_AT91_GPIO_CFG_DIRECTION(gpio, AT91_PIN_OUT);
  	HAL_ARM_AT91_GPIO_PUT(gpio, 1);


	for (i = 0; i < numclocks; i++)
	{
		HAL_ARM_AT91_GPIO_PUT(gpio, 0);
			
		delay = MANUAL_CLOCK_LOOP_COUNT;
		while(delay--);

		HAL_ARM_AT91_GPIO_PUT(gpio, 1);

		delay = MANUAL_CLOCK_LOOP_COUNT;
		while(delay--);
	}
	

	    if (!extra->i2c_twi_num){
		      HAL_ARM_AT91_GPIO_CFG_MULTIDRAIN(AT91_TWI0_TWD, 1);
		      HAL_ARM_AT91_GPIO_CFG_MULTIDRAIN(AT91_TWI0_TWCK, 1);
		      HAL_ARM_AT91_GPIO_CFG_PULLUP(AT91_TWI0_TWD,AT91_PIN_PULLUP_DISABLE);
		      HAL_ARM_AT91_GPIO_CFG_PULLUP(AT91_TWI0_TWCK,AT91_PIN_PULLUP_DISABLE);
		      HAL_ARM_AT91_PIO_CFG(AT91_TWI0_TWD);
		      HAL_ARM_AT91_PIO_CFG(AT91_TWI0_TWCK);

	    } else {
		      HAL_ARM_AT91_GPIO_CFG_MULTIDRAIN(AT91_TWI1_TWD, 1);
		      HAL_ARM_AT91_GPIO_CFG_MULTIDRAIN(AT91_TWI1_TWCK, 1);
		      HAL_ARM_AT91_GPIO_CFG_PULLUP(AT91_TWI1_TWD,AT91_PIN_PULLUP_DISABLE);
		      HAL_ARM_AT91_GPIO_CFG_PULLUP(AT91_TWI1_TWCK,AT91_PIN_PULLUP_DISABLE);
		      HAL_ARM_AT91_PIO_CFG(AT91_TWI1_TWD);
		      HAL_ARM_AT91_PIO_CFG(AT91_TWI1_TWCK);

	    }

		delay = MANUAL_CLOCK_LOOP_COUNT;
		while(delay--);

}


// A transfer has been started. Wait for completion
static inline void
i2c_at91sam9x_doit(cyg_at91sam9x_i2c_extra * extra)
{
#ifndef CYGOPT_IO_I2C_SUPPORT_TIMEOUTS
    while ((extra->i2c_flag & (I2C_FLAG_AT91_COMPLETED | I2C_FLAG_AT91_ARBLOST | I2C_FLAG_AT91_NACK)) == 0){
	    cyg_drv_mutex_lock(&(extra->i2c_lock));
	    cyg_drv_dsr_lock();
	    while (!extra->i2c_completed) {
		cyg_drv_cond_wait(&(extra->i2c_wait));
	    }
	    cyg_drv_dsr_unlock();
	    cyg_drv_mutex_unlock(&(extra->i2c_lock));
   }
   
   if (extra->i2c_flag & I2C_FLAG_AT91_ARBLOST)
	i2c_at91sam9x_reinit(extra);

   extra->i2c_flag = 0;
#else
   while ((extra->i2c_flag & (I2C_FLAG_AT91_COMPLETED | I2C_FLAG_AT91_ARBLOST | I2C_FLAG_AT91_NACK)) == 0){
	    cyg_drv_mutex_lock(&(extra->i2c_lock));
	    cyg_drv_dsr_lock();
	    if (!cyg_cond_timed_wait(&(extra->i2c_wait), cyg_current_time() + CYGOPT_IO_I2C_TIMEOUT))
	    {
  	        cyg_drv_dsr_unlock();
		cyg_drv_mutex_unlock(&(extra->i2c_lock));
		extra->i2c_completed = 0;
		extra->i2c_flag |= I2C_FLAG_AT91_TIMEOUT;
		break;
	    }
	    cyg_drv_dsr_unlock();
	    cyg_drv_mutex_unlock(&(extra->i2c_lock));
   }
   
   if (extra->i2c_flag & (I2C_FLAG_AT91_ARBLOST | I2C_FLAG_AT91_TIMEOUT))
	i2c_at91sam9x_reinit(extra);

   extra->i2c_flag = 0;
#endif
}


static inline void
i2c_at91sam9x_stopit(cyg_at91sam9x_i2c_extra * extra)
{
    extra->i2c_lost_arb = 0;
    extra->i2c_mode = AT91SAM9_I2C_XFER_MODE_INVALID;
}

static cyg_bool
i2c_at91sam9x_handle_xfer(cyg_at91sam9x_i2c_extra * extra, int address, int addr_size)
{
    cyg_uint32 	    tmp 	= 0;
    cyg_uint8 	    i 		= addr_size;
    
    // Nothing to send or receive
    if (extra->i2c_count == 0)
        return 0;


    // TX transfer
    if (extra->i2c_mode == AT91SAM9_I2C_XFER_MODE_TX) {

	    // calculate internal address
	    while (i--)
		tmp |= *extra->i2c_txbuf++ << (i << 3);

	    // Load device address, direction & internal address size
	    I2C_W32(AT91_TWI_MMR,AT91_TWI_MMR_MWRITE |
		((address << AT91_TWI_MMR_DADR_SHIFT) |
		((addr_size << AT91_TWI_MMR_IADRZ_SHIFT) &
		               AT91_TWI_MMR_IADRZ_MASK)));

	    // Load internal address
	    I2C_W32(AT91_TWI_IADR, tmp);

	    // Load START condition if requested
	    if (extra->i2c_send_start)
	    	I2C_W32(AT91_TWI_CR, AT91_TWI_CR_START);

    	    // Enable TWI TX ready IRQ and arbitration lost
	    I2C_W32(AT91_TWI_IER, AT91_TWI_SR_TXRDY | AT91_TWI_SR_ARBLST | AT91_TWI_SR_NACK);	

    } else {

    	    // Enable TWI RX ready IRQ and arbitration lost
	    I2C_W32(AT91_TWI_IER, AT91_TWI_SR_RXRDY | AT91_TWI_SR_ARBLST | AT91_TWI_SR_NACK);


    	    // calculate internal address
    	    while (i--)
        	tmp |= *extra->i2c_rxbuf++ << (i << 3);

	    // Load device address , direction & internal address size
	    I2C_W32(AT91_TWI_MMR, 0);
	    I2C_W32(AT91_TWI_MMR,AT91_TWI_MMR_MREAD |
		((address << AT91_TWI_MMR_DADR_SHIFT) |
		((0 << AT91_TWI_MMR_IADRZ_SHIFT) &
		                  AT91_TWI_MMR_IADRZ_MASK)));

	    I2C_W32(AT91_TWI_IADR, 0);
	    I2C_W32(AT91_TWI_IADR, tmp);
	    
	    // Single byte transfer, set the STOP bit
	    if (extra->i2c_send_stop && (extra->i2c_count == 1)) {
		I2C_W32(AT91_TWI_CR,   AT91_TWI_CR_START | AT91_TWI_CR_STOP);
		extra->i2c_send_stop = 0;
	    } else
		I2C_W32(AT91_TWI_CR,   AT91_TWI_CR_START);


    }

    return 1;
}


// disable all ints
static void
i2c_at91sam9x_disable_ints(cyg_at91sam9x_i2c_extra * extra)
{
    I2C_W32(AT91_TWI_IDR, AT91_TWI_SR_RXRDY | AT91_TWI_SR_TXRDY | AT91_TWI_SR_TXCOMP | AT91_TWI_SR_ARBLST | AT91_TWI_SR_NACK);
}

// Initialize driver & hardware state
void
i2c_at91sam9x_init(struct cyg_i2c_bus *bus)
{
    cyg_at91sam9x_i2c_extra *extra = (cyg_at91sam9x_i2c_extra *) bus->i2c_extra;

    extra->i2c_completed = 0;
    extra->i2c_lost_arb	 = 0;
    extra->i2c_flag	 = 0;
    extra->i2c_mode	 = AT91SAM9_I2C_XFER_MODE_INVALID;

    i2c_at91sam9x_send_manual_clocks(extra, 9);

    
    if (!extra->i2c_twi_num){
	      extra->i2c_twi_base	 = AT91_TWI0;
	      extra->i2c_isr_id 	 = CYGNUM_HAL_INTERRUPT_TWI0;

	      HAL_ARM_AT91_GPIO_CFG_MULTIDRAIN(AT91_TWI0_TWD, 1);
	      HAL_ARM_AT91_GPIO_CFG_MULTIDRAIN(AT91_TWI0_TWCK, 1);
	      HAL_ARM_AT91_GPIO_CFG_PULLUP(AT91_TWI0_TWD,AT91_PIN_PULLUP_DISABLE);
	      HAL_ARM_AT91_GPIO_CFG_PULLUP(AT91_TWI0_TWCK,AT91_PIN_PULLUP_DISABLE);
	      HAL_ARM_AT91_PIO_CFG(AT91_TWI0_TWD);
	      HAL_ARM_AT91_PIO_CFG(AT91_TWI0_TWCK);

    } else {
	      extra->i2c_twi_base	 = AT91_TWI1;
	      extra->i2c_isr_id 	 = CYGNUM_HAL_INTERRUPT_TWI1;

	      HAL_ARM_AT91_GPIO_CFG_MULTIDRAIN(AT91_TWI1_TWD, 1);
	      HAL_ARM_AT91_GPIO_CFG_MULTIDRAIN(AT91_TWI1_TWCK, 1);
	      HAL_ARM_AT91_GPIO_CFG_PULLUP(AT91_TWI1_TWD,AT91_PIN_PULLUP_DISABLE);
	      HAL_ARM_AT91_GPIO_CFG_PULLUP(AT91_TWI1_TWCK,AT91_PIN_PULLUP_DISABLE);
	      HAL_ARM_AT91_PIO_CFG(AT91_TWI1_TWD);
	      HAL_ARM_AT91_PIO_CFG(AT91_TWI1_TWCK);

    }

    cyg_drv_mutex_init(&extra->i2c_lock);
    cyg_drv_cond_init(&extra->i2c_wait, &extra->i2c_lock);
    cyg_drv_interrupt_create(extra->i2c_isr_id,
                             extra->i2c_isr_pri,
                             (cyg_addrword_t)extra,
                             &i2c_at91sam9x_isr,
                             &i2c_at91sam9x_dsr,
                             &(extra->i2c_interrupt_handle),
                             &(extra->i2c_interrupt_data));
    cyg_drv_interrupt_attach(extra->i2c_interrupt_handle);
    cyg_drv_interrupt_unmask(extra->i2c_isr_id);

    // enable TWI clock 
    if (!extra->i2c_twi_num)
	    HAL_WRITE_UINT32((AT91_PMC + AT91_PMC_PCER), AT91_PMC_PCER_TWI);
    else
	    HAL_WRITE_UINT32((AT91_PMC + AT91_PMC_PCER), AT91_PMC_PCER_TWI1);
    // Disable all TWI IRQs
    I2C_W32(AT91_TWI_IDR, 0xffffffff);
    // S/W reset
    I2C_W32(AT91_TWI_CR, AT91_TWI_CR_SWRST);
    // Works only as master
    I2C_W32(AT91_TWI_CR, AT91_TWI_CR_MSEN | AT91_TWI_CR_SDIS);


}

// ReInitialize driver & hardware state
void
i2c_at91sam9x_reinit(cyg_at91sam9x_i2c_extra * extra)
{
    // Disable all TWI IRQs
    I2C_W32(AT91_TWI_IDR, 0xffffffff);

    i2c_at91sam9x_send_manual_clocks(extra, 9);

    // S/W reset
    I2C_W32(AT91_TWI_CR, AT91_TWI_CR_SWRST);
    // Works only as master
    I2C_W32(AT91_TWI_CR, AT91_TWI_CR_MSEN | AT91_TWI_CR_SDIS);
}

// transmit a buffer to a device
cyg_uint32
i2c_at91sam9x_tx(const cyg_i2c_device *dev,
               cyg_bool send_start,
               const cyg_uint8 *tx_data,
               cyg_uint32 count,
               cyg_bool send_stop)
{
    cyg_at91sam9x_i2c_extra *extra = (cyg_at91sam9x_i2c_extra *) dev->i2c_bus->i2c_extra;

    if (!count)
        return 0;

    extra->i2c_txbuf 		= (cyg_uint8 *)tx_data;
    extra->i2c_count 		= count;
    extra->i2c_send_start	= send_start;
    extra->i2c_send_stop	= send_stop;
    extra->i2c_completed 	= 0;
    extra->i2c_flag	 	= 0;
    extra->i2c_mode		= (at91sam9_i2c_xfer_mode)AT91SAM9_I2C_XFER_MODE_TX;

    // set device specific speed
    i2c_at91sam9x_delay(extra, dev->i2c_delay);

    if (!extra->i2c_lost_arb) {
        if (!i2c_at91sam9x_handle_xfer(extra, dev->i2c_address, 0)) {
            return 0;
        }
        i2c_at91sam9x_doit(extra);
    }

    i2c_at91sam9x_stopit(extra);
    
    i2c_at91sam9x_disable_ints(extra);

    return (count - extra->i2c_count);
}


// receive into a buffer from a device
// there is no h/w specific way to send NACK
cyg_uint32
i2c_at91sam9x_rx(const cyg_i2c_device *dev,
               cyg_bool send_start,
               cyg_uint8 *rx_data,
               cyg_uint32 count,
               cyg_bool send_nak,
               cyg_bool send_stop)
{
    cyg_at91sam9x_i2c_extra *extra = (cyg_at91sam9x_i2c_extra *) dev->i2c_bus->i2c_extra;

    if (!count)
        return 0;
    
    extra->i2c_rxbuf 		= rx_data;
    extra->i2c_count 		= count;
    extra->i2c_send_start	= send_start;
    extra->i2c_send_stop	= send_stop;
    extra->i2c_send_nack	= send_nak;
    extra->i2c_flag	 	= 0;
    extra->i2c_completed 	= 0;
    extra->i2c_mode		= (at91sam9_i2c_xfer_mode)AT91SAM9_I2C_XFER_MODE_RX;

    // set device specific speed
    i2c_at91sam9x_delay(extra, dev->i2c_delay);

    if (!extra->i2c_lost_arb) {
        if (!i2c_at91sam9x_handle_xfer(extra, dev->i2c_address, 0)) {
            return 0;
        }
        i2c_at91sam9x_doit(extra);
    }

    i2c_at91sam9x_stopit(extra);
    
    i2c_at91sam9x_disable_ints(extra);
    
    return (count - extra->i2c_count);
}


void
i2c_at91sam9x_stop(const cyg_i2c_device * dev)
{
    cyg_at91sam9x_i2c_extra *extra = (cyg_at91sam9x_i2c_extra *) dev->i2c_bus->i2c_extra;
    i2c_at91sam9x_stopit(extra);
}

