//=============================================================================
//
//      mounttime.c
//
//      Mount timings exerciser, use on a filesystem filled by `makefiles'
//
//=============================================================================
// ####ECOSGPLCOPYRIGHTBEGIN####                                            
// -------------------------------------------                              
// This file is part of eCos, the Embedded Configurable Operating System.   
// Copyright (C) 2009 Free Software Foundation, Inc.
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
// Author(s):   wry
// Date:        2009-07-02
// Description: Read, write and erase timing benchmark.
//              Some timing code borrowed from mmfs tests.
//
//####DESCRIPTIONEND####
//=============================================================================

#include <cyg/infra/testcase.h>
#include <cyg/infra/diag.h>
#include <cyg/infra/cyg_ass.h>
#include <pkgconf/system.h>
#include <pkgconf/hal.h>
#include <cyg/hal/hal_intr.h>

#ifndef HAL_CLOCK_READ
# error "HAL_CLOCK_READ not defined!"
#endif

#if !defined(CYGPKG_LIBC_TIME) || !defined(CYGPKG_ERROR) || !defined(CYGPKG_LIBC_STDIO) || !defined(CYGPKG_KERNEL)

void cyg_user_start(void)
{
    CYG_TEST_NA("Needs CYGPKG_KERNEL, CYGPKG_LIBC_TIME, CYGPKG_ERROR and CYGPKG_LIBC_STDIO");
}

#else

#include <cyg/nand/nand.h>
#include <cyg/nand/nand_device.h>
#include <cyg/nand/util.h>

#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <dirent.h>
#include <stdlib.h>

#include <cyg/kernel/kapi.h>

#ifdef CYGPKG_DEVS_NAND_SYNTH
#define DEVICE "synth"
#else
#define DEVICE "onboard"
#endif

#define PARTITION 0

#define MUST(what) do { \
        if (0 != what) { \
                perror(#what);          \
                CYG_TEST_FAIL(#what);   \
                cyg_test_exit();        \
        }                           \
} while(0)

#define min(a,b) ( (a) > (b) ? (b) : (a) )

// --------------------------------------------------------------
// Timing code cribbed from pvr5.c

typedef struct
{
    cyg_int64           ticks;
    cyg_uint32           halticks;
} timestamp;

typedef struct
{
    timestamp           start;
    timestamp           end;
    cyg_int64           interval;       // In HAL ticks
    cyg_int64           us_interval;    // Interval in microseconds
} timing;

static cyg_int64        ticks_overhead = 0;
static cyg_int32        us_per_haltick = 0;
static cyg_int32        halticks_per_us = 0;

static cyg_int64        rtc_resolution[] = CYGNUM_KERNEL_COUNTERS_RTC_RESOLUTION;
static cyg_int64        rtc_period = CYGNUM_KERNEL_COUNTERS_RTC_PERIOD;

static void wait_for_tick( void )
{
    cyg_tick_count_t now = cyg_current_time();

    while( cyg_current_time() == now )
        continue;
}

static void get_timestamp( timestamp *ts )
{
    ts->ticks = cyg_current_time();
    HAL_CLOCK_READ( &ts->halticks );
}

static cyg_int64 ticks_to_us( cyg_int64 ticks )
{
    cyg_int64 us;

    if( us_per_haltick != 0 )
        us = ticks * us_per_haltick;
    else
        us = ticks / halticks_per_us;

    return us;
}

static void calculate_interval( timing *t )
{
    t->interval = t->end.halticks - t->start.halticks;

    t->interval += (t->end.ticks - t->start.ticks) * rtc_period;

    t->interval -= ticks_overhead;

    t->us_interval = ticks_to_us( t->interval );
}

static void init_timing( void )
{
    timing t;

    cyg_thread_delay(2); // ensure clock running - asserts if not

    us_per_haltick = 1000000/(rtc_period * rtc_resolution[1]);
    halticks_per_us = (rtc_period * rtc_resolution[1])/1000000;

    wait_for_tick();

    get_timestamp( &t.start );
    get_timestamp( &t.end );

    calculate_interval( &t );

    ticks_overhead = t.interval;

    diag_printf("Timing overhead %lld ticks (%lluus), this will be factored out of all other measurements\n", ticks_overhead, t.us_interval );
}

#define disable_clock_latency_measurement()
#define enable_clock_latency_measurement()

// --------------------------------------------------------------

void
show_times_hdr(void)
{
    disable_clock_latency_measurement();
    diag_printf("\n");
#ifdef _TM_BASIC_HAL_CLOCK_READ_UNDEFINED
    diag_printf("HAL_CLOCK_READ() is not supported on this platform.\n");
    diag_printf("Timing results are meaningless.\n");
#endif
    diag_printf("All times are in microseconds\n");
    diag_printf("\n");
    diag_printf("                                     Confidence\n");
    diag_printf("      Ave      Min      Max      Var  Ave  Min  Function\n");
    diag_printf("   ======   ======   ======   ====== ========== ========\n");
    enable_clock_latency_measurement();
}

// Display a time result
void
show_ns(cyg_uint64 ns)
{
    ns += 5;  // for rounding to .01us
    diag_printf("%6d.%02d", (int)(ns/1000), (int)((ns%1000)/10));
}

void
show_times_detail(timing ft[], int nsamples, char *title, bool ignore_first)
{
    int i;
    int start_sample, total_samples;   
    cyg_uint64 delta, total, ave, min, max, ave_dev;
    /* we measure in ticks, convert to us, but store as ns for good precision */
    cyg_int32 con_ave, con_min;

    if (ignore_first) {
        start_sample = 1;
        total_samples = nsamples-1;
    } else {
        start_sample = 0;
        total_samples = nsamples;
    }

    total = 0;
    min = 0xFFFFffffFFFFffffULL;
    max = 0;
    for (i = start_sample;  i < nsamples;  i++) {
        calculate_interval(&ft[i]);
        delta = ft[i].us_interval * 1000;
        total += delta;
        if (delta < min) min = delta;
        if (delta > max) max = delta;
    }
    ave = total / total_samples;

    ave_dev = 0;
    for (i = start_sample;  i < nsamples;  i++) {
        delta = ft[i].us_interval * 1000;
        if (delta > ave)
            delta = delta - ave;
        else
            delta = ave - delta;
        ave_dev += delta;
    }
    ave_dev /= total_samples;

    con_ave = 0;
    con_min = 0;
    for (i = start_sample;  i < nsamples;  i++) {
        delta = ft[i].us_interval * 1000;
        if ((delta <= (ave+ave_dev)) && (delta >= (ave-ave_dev))) con_ave++;
        if ((delta <= (min+ave_dev)) && (delta >= (min-ave_dev))) con_min++;
    }
    con_ave = (con_ave * 100) / total_samples;
    con_min = (con_min * 100) / total_samples;

    disable_clock_latency_measurement();
    show_ns(ave);
    show_ns(min);
    show_ns(max);
    show_ns(ave_dev);
    diag_printf("  %3d%% %3d%%", con_ave, con_min);
    diag_printf(" %s\n", title);

    CYG_ASSERT( ave <= max, "ave < max" );
    
    enable_clock_latency_measurement();
}

void
show_times(timing ft[], int nsamples, char *title)
{
    show_times_detail(ft, nsamples, title, false);
#ifdef STATS_WITHOUT_FIRST_SAMPLE
    show_times_detail(ft, nsamples, "", true);
#endif
}

// --------------------------------------------------------------

#define WORKDIR MOUNTPOINT "/" "test"

#define NREADS 100
#define NWRITES 30  /* TODO */
#define NERASES 30

void show_test_parameters(void)
{
    disable_clock_latency_measurement();
    diag_printf("\nTesting parameters:\n");
    diag_printf("   NAND reads:            %5d\n", NREADS);
    if (NWRITES)
        diag_printf("   NAND writes:           %5d\n", NWRITES);
    if (NERASES)
        diag_printf("   NAND erases:           %5d\n", NERASES);
    enable_clock_latency_measurement();
}

cyg_nand_block_addr find_spare_block(cyg_nand_partition *part)
{
    const int oobz = NAND_APPSPARE_PER_PAGE(part->dev);
    unsigned char oob[oobz];
    int i,rv;
    cyg_nand_block_addr b;

    for (b=part->last; b>=part->first; b--) {
        cyg_nand_page_addr pg = CYG_NAND_BLOCK2PAGEADDR(part->dev, b);
        rv = cyg_nand_read_page(part, pg, 0, 0, oob, oobz);
        if (rv != 0) continue; // bad block?

        for (i=0; i<oobz; i++)
            if (oob[i] != 0xff)
                goto next;

        // oob all FF: take it!
        return b;
next:
        ;
    }

    CYG_TEST_FAIL_EXIT("can't find an untagged block");
}

unsigned char pagebuffer[CYGNUM_NAND_PAGEBUFFER];

void test_reads(cyg_nand_partition *part, cyg_nand_block_addr b)
{
    cyg_nand_page_addr pgstart = CYG_NAND_BLOCK2PAGEADDR(part->dev, b),
                       pgend = CYG_NAND_BLOCK2PAGEADDR(part->dev, b+1)-1,
                       pg = pgstart;
    int i;
    const int oobz = NAND_APPSPARE_PER_PAGE(part->dev);
    unsigned char oob[oobz];
    timing ft[NREADS];

    for (i=0; i < NREADS; i++) {
        wait_for_tick();
        get_timestamp(&ft[i].start);
        cyg_nand_read_page(part, pg, pagebuffer, sizeof pagebuffer, 0, 0);
        get_timestamp(&ft[i].end);
        ++pg;
        if (pg > pgend) pg = pgstart;
    }
    show_times(ft, NREADS, "NAND page reads (page data only)");

    for (i=0; i < NREADS; i++) {
        wait_for_tick();
        get_timestamp(&ft[i].start);
        cyg_nand_read_page(part, pg, 0, 0, oob, oobz);
        get_timestamp(&ft[i].end);
        ++pg;
        if (pg > pgend) pg = pgstart;
    }
    show_times(ft, NREADS, "NAND page reads (OOB only)");

    for (i=0; i < NREADS; i++) {
        wait_for_tick();
        get_timestamp(&ft[i].start);
        cyg_nand_read_page(part, pg, pagebuffer, sizeof pagebuffer, oob, oobz);
        get_timestamp(&ft[i].end);
        ++pg;
        if (pg > pgend) pg = pgstart;
    }
    show_times(ft, NREADS, "NAND page reads (page + OOB)");
}


void test_writes(cyg_nand_partition *part, cyg_nand_block_addr b)
{
    cyg_nand_page_addr pgstart = CYG_NAND_BLOCK2PAGEADDR(part->dev, b),
                       pgend = CYG_NAND_BLOCK2PAGEADDR(part->dev, b+1)-1,
                       pg = pgstart;
    int i;
    const int oobz = NAND_APPSPARE_PER_PAGE(part->dev);
    unsigned char oob[oobz];
    timing ft[NWRITES];

    memset(pagebuffer, 0xFF, sizeof pagebuffer);
    memset(oob, 0xFF, oobz);

    for (i=0; i < NWRITES; i++) {
        wait_for_tick();
        get_timestamp(&ft[i].start);
        cyg_nand_write_page(part, pg, pagebuffer, sizeof pagebuffer, oob, oobz);
        get_timestamp(&ft[i].end);
        ++pg;
        if (pg > pgend) {
            cyg_nand_erase_block(part, b);
            pg = pgstart;
        }
    }
    cyg_nand_erase_block(part, b);
    show_times(ft, NWRITES, "NAND full-page writes");
}


void test_erases(cyg_nand_partition *part, cyg_nand_block_addr b)
{
    int i;
    timing ft[NERASES];

    for (i=0; i < NERASES; i++) {
        wait_for_tick();
        get_timestamp(&ft[i].start);
        cyg_nand_erase_block(part, b);
        get_timestamp(&ft[i].end);
    }
    show_times(ft, NERASES, "NAND block erases");
}



void rwbenchmark_main(void)
{
    cyg_nand_device *dev;
    cyg_nand_partition *part;
    cyg_nand_block_addr block;

    cyg_nand_lookup(DEVICE, &dev);
    if (!dev)
        CYG_TEST_FAIL_EXIT("can't get device "DEVICE);
    part = cyg_nand_get_partition(dev, PARTITION); 
    if (!part)
        CYG_TEST_FAIL_EXIT("can't get partition");

    block = find_spare_block(part);
    diag_printf("Using block %d\n",block);

    show_test_parameters();
    show_times_hdr();

    test_reads(part, block);
    test_writes(part,block);
    test_erases(part,block);

}

int main(void)
{
    CYG_TEST_INIT();
    init_timing();
#if defined(CYGVAR_KERNEL_COUNTERS_CLOCK_LATENCY) || defined(CYGVAR_KERNEL_COUNTERS_CLOCK_DSR_LATENCY)
    CYG_TEST_INFO("WARNING: Clock or DSR latency instrumentation can mess with the results, recommend you turn it off.");
#endif


    rwbenchmark_main();
    CYG_TEST_EXIT("Run complete");
}

#endif

