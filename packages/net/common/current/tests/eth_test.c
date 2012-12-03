//==========================================================================
//
//      tests/socket_test.c
//
//      Test network socket functions
//
//==========================================================================
// ####BSDALTCOPYRIGHTBEGIN####                                             
// -------------------------------------------                              
// Portions of this software may have been derived from FreeBSD, OpenBSD,   
// or other sources, and if so are covered by the appropriate copyright     
// and license included herein.                                             
// -------------------------------------------                              
// ####BSDALTCOPYRIGHTEND####                                               
//==========================================================================
//#####DESCRIPTIONBEGIN####
//
// Author(s):    gthomas
// Contributors: gthomas
// Date:         2000-01-10
// Purpose:      HLD
// Description:  
//              
//
//####DESCRIPTIONEND####
//
//==========================================================================

// socket test code

#include <cyg/io/eth/netdev.h>
#include <cyg/io/eth/eth_drv.h>
#ifndef CYGPKG_LIBC_STDIO
#define perror(s) diag_printf(#s ": %s\n", strerror(errno))
#endif

#define STACK_SIZE CYGNUM_HAL_STACK_SIZE_TYPICAL
static char stack[STACK_SIZE];
static cyg_thread thread_data;
static cyg_handle_t thread_handle;

cyg_netdevtab_entry_t *t;      // Added
struct eth_drv_sc* sc;

CYG_HAL_TABLE_BEGIN(__NETDEVTAB__, netdev);  // Added
CYG_HAL_TABLE_END(__NETDEVTAB_END__, netdev); // Added


extern void
cyg_test_exit(void);

void
net_test(cyg_addrword_t param)
{

    int s;
    int one = 1;

/*    diag_printf("Start socket test\n");

    s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    diag_printf("socket() = %d\n", s);
    s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    diag_printf("socket() = %d\n", s);

    if (setsockopt(s, SOL_SOCKET, SO_BROADCAST, &one, sizeof(one))) {        
        perror("setsockopt");
    }
*/
}

const char eth_hdr[14] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x12,0x34,0x3E,0x28,0x7A,0xBA,0x00,0x0E};
const char buf[14] = {0x00,0x00,0x00,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A};

static void
cyg_net_init_devs(void *ignored)
{

    
    cyg_netdevtab_entry_t *t;
    
	eth_drv_buffers_init();
    
        // Initialize all network devices
    for (t = &__NETDEVTAB__[0]; t != &__NETDEVTAB_END__; t++) {
//	log(LOG_INIT, "Init device '%s'\n", t->name);
        if (t->init(t)) {
            t->status = CYG_NETDEVTAB_STATUS_AVAIL;
            sc = (struct eth_drv_sc*) (t->device_instance);
        } else {
        // What to do if device init fails?
            t->status = 0;  // Device not [currently] available
        }
    }
#if 0  // Bridge code not available yet
    #if NBRIDGE > 0
        bridgeattach(0);
    #endif
#endif
    cyg_uint32 i,j;
    for (i = 0; i < 16; i++)
    {
    	eth_drv_write((char *)eth_hdr, (char *)buf, 14);
    }
    char *eth_hdr_rx;
    char *buf_rx;
    int len_rx = 3;
    eth_drv_read(eth_hdr_rx,buf_rx,len_rx);
    diag_printf("!!!ETH_HDR = %X ; RX_DATA = %X!!!\n",eth_hdr_rx,buf_rx);
    diag_printf("!!!Test finished!!!\n");
//    cyg_drv_interrupt_mask(4);
//    cyg_drv_interrupt_acknowledge(3);
    cyg_test_exit();
}

void
cyg_start(void)
{
    // Create a main thread, so we can run the scheduler and have time 'pass'
    cyg_thread_create(10,                // Priority - just a number
                      cyg_net_init_devs,          // entry
                      0,                 // entry parameter
                      "Network test",    // Name
                      &stack[0],         // Stack
                      STACK_SIZE,        // Size
                      &thread_handle,    // Handle
                      &thread_data       // Thread data structure
            );
    cyg_thread_resume(thread_handle);  // Start it
    cyg_scheduler_start();
}
