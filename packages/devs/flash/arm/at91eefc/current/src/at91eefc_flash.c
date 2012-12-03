//==========================================================================
//
//      at91eefc_flash.c
//
//      Atmel EEFC flash driver
//
//==========================================================================
//==========================================================================
//#####DESCRIPTIONBEGIN####
//
// Author(s):
// Date:
//
//####DESCRIPTIONEND####
//
//========================================================================*/

#include <cyg/infra/cyg_type.h>
#include <cyg/infra/cyg_ass.h>
#include <cyg/infra/diag.h>

#include <cyg/io/flash.h>
#include <cyg/io/flash_dev.h>

#include <cyg/hal/hal_arch.h>
#include <cyg/hal/hal_intr.h>
#include <cyg/hal/hal_cache.h>
#include <cyg/hal/hal_io.h>
#include <cyg/hal/hal_if.h>
#include <cyg/io/at91eefc_flash.h>

#include <string.h>


/* Interrupt control during flash operations. */
// Must have interrupts disabled if executing code from target flash.
#if 1
#define FLASH_INTSCACHE_STATE     CYG_INTERRUPT_STATE _saved_ints_
#define FLASH_INTSCACHE_BEGIN()   HAL_DISABLE_INTERRUPTS(_saved_ints_)
#define FLASH_INTSCACHE_END()     HAL_RESTORE_INTERRUPTS(_saved_ints_)
#else
#define FLASH_INTSCACHE_STATE
#define FLASH_INTSCACHE_BEGIN()   CYG_EMPTY_STATEMENT
#define FLASH_INTSCACHE_END()     CYG_EMPTY_STATEMENT
#endif

/* A buffer to ensure 32-bit alignment during flash programming. */
static unsigned char block_buffer[AT91_IFLASH_BLOCK_SIZE];

/* Info returned by query function. */
static char query_str[] = "AT91 EEFC Flash";

/* Initialize driver. */
static int
at91eefc_flash_init(struct cyg_flash_dev* dev)
{
    CYG_CHECK_DATA_PTR(dev, "bad flash device");

    return CYG_FLASH_ERR_OK;
}


/* Query driver. */
static size_t
at91eefc_flash_query(struct cyg_flash_dev* dev, void* data, size_t len)
{
    CYG_CHECK_DATA_PTR(dev, "bad flash device");
    CYG_CHECK_DATA_PTR(data, "bad data ptr");
    CYG_ASSERT(len > sizeof(query_str), "bad len");

    memcpy(data, query_str, sizeof(query_str));

    return sizeof(query_str);
}


/* This is the hardware flash programming function that must be in RAM. */
static int
at91eefc_flash_hw_program(cyg_uint32 eefc_base, cyg_uint32 eefc_cmd)
__attribute__((section (".2ram.at91eefc_flash_hw_program")));

static int
at91eefc_flash_hw_program(cyg_uint32 eefc_base, cyg_uint32 eefc_cmd)
{
    cyg_uint32 timeout = AT91_IFLASH_MAX_TIMEOUT * 1000;
    cyg_uint32 eefc_fsr;

    HAL_WRITE_UINT32(eefc_base + AT91_EEFC_FCR, eefc_cmd);
    do
    {
        HAL_DELAY_US(1);
        HAL_READ_UINT32(eefc_base + AT91_EEFC_FSR, eefc_fsr);
    } while( !(eefc_fsr & AT91_EEFC_FRDY) && timeout-- > 0);

    return eefc_fsr;
}


/* Program or erase the block in which specified range resides. */
static int
at91eefc_flash_program(struct cyg_flash_dev* dev,
                       cyg_flashaddr_t addr, const void* src, size_t len)
{
    FLASH_INTSCACHE_STATE;
    int (*hw_program_fn)(cyg_uint32, cyg_uint32);
    cyg_at91eefc_flash_dev *at91eefc_dev;
    unsigned block_size;
    unsigned block_addr;
    unsigned block_index;
    unsigned offset, padding;
    cyg_uint32 eefc_cmd, eefc_fsr;
    int i;
    cyg_uint32 *p, *q;

    CYG_CHECK_DATA_PTR(dev, "bad flash device");
    CYG_ASSERT((addr >= dev->start) && (addr <= dev->end), "bad flash address");
    CYG_ASSERT(addr + len <= dev->end, "bad flash range");

    // Get block size.
    at91eefc_dev = (cyg_at91eefc_flash_dev *)dev->priv;
    block_size = at91eefc_dev->block_info.block_size;

    // Translate write address to block index and block address.
    block_index = (addr - dev->start) / block_size;
    block_addr = addr - addr % block_size;

    // If erasing the block.
    if (src == block_buffer) {
        // Fill the buffer with erased 32-bit words.
        i = block_size / 4;
        p = (cyg_uint32 *)block_buffer;
        while (i--)
            *p++ = 0xFFFFFFFF;
    }
    // Else programming the block.
    else {
        // Compute span in buffer where write-data will be located.
        offset = addr - block_addr;
        padding = block_size - (offset + len);

        // If span not at start of block, copy from flash to start of buffer.
        if (offset)
            memcpy(block_buffer, (void *)block_addr, offset);

        // Copy write-data to buffer.
        memcpy(block_buffer + offset, src, len);

        // If span not to end of block, copy from flash to remaining buffer.
        if (padding)
            memcpy(block_buffer + offset + len, (void *)(addr + len), padding);
    }

    // Write the buffer to flash using 32-bit transfers.
    i = block_size / 4;
    p = (cyg_uint32 *)block_addr;
    q = (cyg_uint32 *)block_buffer;
    while (i--)
        *p++ = *q++;

    // Build EEFC command and execute hardware programming function in RAM.
    eefc_cmd = AT91_EEFC_FKEY | AT91_EEFC_FARG(block_index) | AT91_EEFC_FCMD_EWP;
    hw_program_fn = (int (*)(cyg_uint32, cyg_uint32))
                    cyg_flash_anonymizer(&at91eefc_flash_hw_program);
    FLASH_INTSCACHE_BEGIN();
    eefc_fsr = (*hw_program_fn)(at91eefc_dev->eefc_base, eefc_cmd);
    FLASH_INTSCACHE_END();

    if (!(eefc_fsr & AT91_EEFC_FRDY))
        return CYG_FLASH_ERR_PROGRAM;

    if (eefc_fsr & AT91_EEFC_FLOCKE)
        return CYG_FLASH_ERR_PROTECT;

    if (eefc_fsr & AT91_EEFC_FCMDE)
        return CYG_FLASH_ERR_PROGRAM;

    return CYG_FLASH_ERR_OK;
}


/* Erase the block in which specified address resides. */
static int
at91eefc_flash_erase(struct cyg_flash_dev* dev, cyg_flashaddr_t addr)
{
    return at91eefc_flash_program(dev, addr, block_buffer, 256);
}


// Table of driver functions
const CYG_FLASH_FUNS(cyg_at91eefc_flash_funs,
                     &at91eefc_flash_init,
                     &at91eefc_flash_query,
                     &at91eefc_flash_erase,
                     &at91eefc_flash_program,
                     (int (*)(struct cyg_flash_dev*, const cyg_flashaddr_t, void*, size_t))0,
                     cyg_flash_devfn_lock_nop,
                     cyg_flash_devfn_unlock_nop);

// ----------------------------------------------------------------------------
// End of at91eefc_flash.c
