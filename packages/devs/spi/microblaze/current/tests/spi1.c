#include <cyg/infra/cyg_type.h>
#include <cyg/infra/testcase.h>         // Test macros
#include <cyg/infra/cyg_ass.h>          // Assertion macros
#include <cyg/infra/diag.h>             // Diagnostic output

#include <cyg/hal/hal_arch.h>           // CYGNUM_HAL_STACK_SIZE_TYPICAL
#include <cyg/kernel/kapi.h>

#include <cyg/io/spi.h>                 // Common SPI API
#include <cyg/io/spi_microblaze.h>           // Microblaze data structures

#include <string.h>

 //---------------------------------------------------------------------------
 // Thread data structures.

 cyg_uint8 stack [CYGNUM_HAL_STACK_SIZE_TYPICAL];
 cyg_thread thread_data;
 cyg_handle_t thread_handle;

 externC cyg_spi_microblaze_bus_t cyg_spi_microblaze_bus0;

 //---------------------------------------------------------------------------
 // SPI loopback device driver data structures.

 cyg_spi_microblaze_dev_t loopback_device = {
     .spi_device.spi_bus = &cyg_spi_microblaze_bus0.spi_bus,
     .cl_pol = 1,
     .cl_pha = 1,
     .spi_baud = 8000000,                // Nominal 8Mhz.
     .spi_lsbf = 1,
 };

 //---------------------------------------------------------------------------

 //const char tx_data[] = {0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1}; // To set
 //maximum resistance at address 00
 const char tx_data[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // To set
 //minimim resistance at address 00
 const char tx_data1[] = "Testing extended API...";
 const char tx_data2[] = "Testing extended API for a second transaction.";

 char rx_data [sizeof(tx_data)];
 char rx_data1 [sizeof(tx_data1)];
 char rx_data2 [sizeof(tx_data2)];

 //---------------------------------------------------------------------------
 // Run single loopback transaction using simple transfer API call.

 void run_test_1 (cyg_bool polled)
 {
     diag_printf ("Test 1 : Simple transfer test (polled = %d).\n",
 polled ? 1 : 0);
     cyg_spi_transfer (&loopback_device.spi_device, polled, sizeof (tx_data),
         (const cyg_uint8*) &tx_data[0], (cyg_uint8*) &rx_data[0]);

     diag_printf ("    Tx data : %s\n", tx_data);
     diag_printf ("    Rx data : %s\n", rx_data);
     CYG_ASSERT (memcmp (tx_data, rx_data, sizeof (tx_data)) == 0,
         "Simple transfer loopback failed - mismatched data.\n");
 }


 //---------------------------------------------------------------------------
 // Run all PL022 SPI interface loopback tests.

 void run_tests (void)
 {
     diag_printf ("Running Microblaze SPI driver loopback tests.\n");
     run_test_1 (true);
     CYG_TEST_PASS_FINISH ("Loopback tests ran OK");
 }

 //---------------------------------------------------------------------------
 // User startup - tests are run in their own thread.

 void cyg_user_start(void)
 {
     CYG_TEST_INIT();
     cyg_thread_create(
         10,                                   // Arbitrary priority
         (cyg_thread_entry_t*) run_tests,      // Thread entry point
         0,                                    //
         "test_thread",                        // Thread name
         &stack[0],                            // Stack
         CYGNUM_HAL_STACK_SIZE_TYPICAL,        // Stack size
         &thread_handle,                       // Thread handle
         &thread_data                          // Thread data structure
     );
     cyg_thread_resume(thread_handle);
     cyg_scheduler_start();

 }