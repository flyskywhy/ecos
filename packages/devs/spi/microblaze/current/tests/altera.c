//=============================================================================
// This is a simple test test for the AT91SAM9 SPI driver and Altera Passive 
// Serial programming.
//=============================================================================

#include <cyg/infra/cyg_type.h>
#include <cyg/infra/testcase.h>
#include <cyg/infra/cyg_ass.h>
#include <cyg/infra/diag.h>
#include <cyg/hal/hal_arch.h>
#include <cyg/kernel/kapi.h>
#include <cyg/io/spi.h>
#include <cyg/io/spi_at91.h>

#include <cyg/hal/hal_io.h>
#include <cyg/hal/hal_diag.h>

#include <string.h>

#include "_B621_top.h"

cyg_uint8 stack [CYGNUM_HAL_STACK_SIZE_TYPICAL];
cyg_thread thread_data;
cyg_handle_t thread_handle;


externC cyg_spi_at91_bus_t cyg_spi_at91_bus0, cyg_spi_at91_bus1;

// SPI loopback device driver data structures.

cyg_spi_at91_device_t loopback_device = {
    .spi_device.spi_bus = &cyg_spi_at91_bus1.spi_bus,
    .dev_num = 1,
    .cl_pol = 0,
    .cl_pha = 1,
    .cl_brate = 16000000,                // Nominal 16Mhz.
    .cs_up_udly = 1,
    .cs_dw_udly = 1,
    .tr_bt_udly = 1,
};

#define BYTESATONES 200
char rx_data [BYTESATONES]; // dummy read buffer

cyg_uint8 byte_data_revert(cyg_uint8 data)
{
    cyg_uint8 res = 0;
    cyg_uint8 i;
    for (i = 0; i < 8; i++)
    {
        if (data & (1 << i))
        res |= 1 << ((7 - i));
    }
    return res;
}

void gpio_nconfig_set(cyg_uint8 data) // used PB11 GPIO ad nCONFIG
{
    HAL_WRITE_UINT32((AT91_PMC + AT91_PMC_PCER), AT91_PMC_PCER_PIOB);
    HAL_ARM_AT91_GPIO_CFG_PULLUP(AT91_GPIO_PB11, AT91_PIN_PULLUP_ENABLE);
    HAL_ARM_AT91_GPIO_CFG_DIRECTION(AT91_GPIO_PB11, AT91_PIN_OUT);
    if (data)
        HAL_ARM_AT91_GPIO_PUT(AT91_GPIO_PB11, 1);
    else
        HAL_ARM_AT91_GPIO_PUT(AT91_GPIO_PB11, 0);
}

void altera_prog()
{
    cyg_uint32 i = 0;
    // Converting bit order
    for(i; i<sizeof(fpga_buffer); i++)
    {
        fpga_buffer[i] = byte_data_revert(fpga_buffer[i]);
    }
    gpio_nconfig_set(0);
    cyg_thread_delay(1);
    gpio_nconfig_set(1);
    cyg_uint32 writes = sizeof(fpga_buffer)/BYTESATONES;
    cyg_uint32 writesadd = sizeof(fpga_buffer)%BYTESATONES;
    for(i = 0; i < writes; i++){
        cyg_spi_transfer (&loopback_device.spi_device, 0, BYTESATONES, (cyg_uint8*) &fpga_buffer[i*BYTESATONES], (cyg_uint8*) rx_data);
        diag_printf("\r%3d %% written", i*100/sizeof(fpga_buffer));
    }
    if(writesadd>0)
	cyg_spi_transfer (&loopback_device.spi_device, 0, writesadd, (cyg_uint8*) &fpga_buffer[writes*BYTESATONES], (cyg_uint8*) rx_data);
    diag_printf("\r100 %% written\nProgramming done. %d bytes written\n", sizeof(fpga_buffer));
}


void run_test_1()
{
    diag_printf ("Test 1 : Simple SRAM interface (nCS3) transfer test ).\n");
    cyg_uint32 i;
    cyg_uint32 * mem=0x40000000;
    for(i=0;i<0x100000; i++){
        *mem++ = i;
    }
    diag_printf ("Test 1: done).\n");
}

void run_tests (void)
{
    diag_printf("\nRunning SPI Altera programming tests.\n");
    altera_prog(); 
    CYG_TEST_PASS_FINISH("Altera programming test OK\n");
    
    run_test_1();
}

void cyg_user_start(void)
{
    CYG_TEST_INIT();
    cyg_thread_create(
        10,
        (cyg_thread_entry_t*) run_tests,
        0,
        "altera_prog_thread",
        &stack[0],
        CYGNUM_HAL_STACK_SIZE_TYPICAL,
        &thread_handle,
        &thread_data
    );
    cyg_thread_resume(thread_handle);
    cyg_scheduler_start();
}

