#include <pkgconf/system.h>

#include <cyg/infra/testcase.h>         // test macros
#include <cyg/infra/cyg_ass.h>          // assertion macros
#include <pkgconf/io_spi.h>
#include <cyg/io/spi.h>
#include <cyg/io/spi_microblaze.h>


#include <pkgconf/kernel.h>


#include <cyg/hal/hal_arch.h>           // CYGNUM_HAL_STACK_SIZE_TYPICAL
#include <cyg/kernel/kapi.h>

#ifdef CYGPKG_DEVS_SPI_MICROBLAZE_BUS0
externC cyg_spi_microblaze_bus_t cyg_spi_microblaze_bus0;
#endif

int main(void)
{
 cyg_spi_microblaze_dev_t spi_device = 
 {
  .spi_device.spi_bus = &cyg_spi_microblaze_bus0.spi_bus,
  .spi_cpol = 1,
  .spi_cpha = 1,
  .spi_lsbf = 1,

 };
// cyg_spi_device spi_device;
// Cyg_ErrNo err,err_read;
 char read_string[50];
// const char test_string[] = "SPI is working correctly!\n";
// char read_string[];
const cyg_uint8 tx_buffer[10] = {0xA5,22,33,44,55,66,77,88,99,100};
const cyg_uint8 rx_buffer[10] ;
cyg_uint32 mul1 = 13;
cyg_uint32 mul2 = 12;
cyg_uint32 res,i;


// diag_printf("Starting SPI example\n");
// diag_printf("\n");
//  while(1)
/* cyg_spi_transfer(&spi_device, false, 1, tx_buffer, rx_buffer);
 cyg_spi_tick(&spi_device,false,1);*/
for ( i = 0; i < 10000; i++)
{
	res = mul1 * mul2;
	if(!(i%100))
		diag_printf("res[%d] = %d\n",i,res);
}
 printf("Writing operation finished succesful.\n");
}
