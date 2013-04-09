//==========================================================================
//
//      spi_devices.c
//
//      microblaze SPI devices
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
//#####DESCRIPTIONBEGIN####
//
// Author(s):     Li Zheng <flyskywhy@gmail.com>
// Date:          2013-03-26
//
//####DESCRIPTIONEND####
//
//==========================================================================

#include <pkgconf/devs_spi_microblaze.h>
#include <cyg/io/spi_microblaze.h>
#include <cyg/io/spi.h>
#include "src/xspi.h"

#ifdef CYGPKG_DEVS_SPI_MICROBLAZE_BUS1

cyg_spi_microblaze_dev_t spi_mmc_dev0 CYG_SPI_DEVICE_ON_BUS(1) =
{
    .spi_device.spi_bus = &cyg_spi_microblaze_bus1.spi_bus
};

#ifdef CYGPKG_DEVS_DISK_MMC
#include <pkgconf/devs_disk_mmc.h>
#ifdef CYGINT_DEVS_DISK_MMC_SPI_CONNECTORS
cyg_spi_device cyg_spi_mmc_dev0 =
{
    .spi_bus = &cyg_spi_microblaze_bus1.spi_bus
};
#endif
#endif

#endif
