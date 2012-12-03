#ifndef CYGONCE_DEVS_FLASH_AT91EEFC_H
#define CYGONCE_DEVS_FLASH_AT91EEFC_H
//==========================================================================
//
//      at91eefc_flash.h
//
//      Atmel AT91 EEFC flash driver definitions
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

// Driver per-instance private data.
typedef struct cyg_at91eefc_dev
{
    CYG_ADDRWORD            eefc_base;
    cyg_flash_block_info_t  block_info;

} cyg_at91eefc_flash_dev;

// Table of driver functions.
__externC const struct cyg_flash_dev_funs cyg_at91eefc_flash_funs;

#endif // CYGONCE_DEVS_FLASH_AT91EEFC_H

