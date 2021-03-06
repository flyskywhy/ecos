# Easily CLONE, COMPILE and RUN

On this microblaze branch eCos branched by Li Zheng <flyskywhy@gmail.com>, you can:

- Easily clone the apps, os and tools git repositories related to eCos
- Easily compile the os and then the apps
- Easily run elf on different platform like i386 virtual machine or hardware

## Quick start

You can just use the following command if you can:

    mkdir ecos
    cd ecos
    git clone https://github.com/flyskywhy/proj.git -b ecos
    source proj/env.sh
    proj clone
    source proj/env.sh
    cp -a tools/ecos-tools/bin/.eCosPlatforms/ ~/
    cd apps/ecos_app_base
    configtool ecos_avnet_s6lx9_mmu_tiny_13_1.ecc (then click on `Build->Generate Build Tree` and `Build->Library`)
    lunch
    make

Now you will get the .elf file.

## Detailed Description

Or you can follow the steps described in [CLONE_COMPILE_RUN (English Version)](https://github.com/flyskywhy/g/blob/master/i主观的体验方式/t快乐的体验/电信/Os/Ecos/CLONE_COMPILE_RUN.en.md) or [CLONE_COMPILE_RUN (中文版)](https://github.com/flyskywhy/g/blob/master/i主观的体验方式/t快乐的体验/电信/Os/Ecos/CLONE_COMPILE_RUN.cn.md).

# Feature

## common

- Can use lunch command in your application folder (e.g. [ecos_app_base](https://github.com/flyskywhy/ecos_app_base.git)/) to easily switch .ecc (different platform) file after source ecos/scripts/envsetup.sh (actually I recommand source [proj(ecos branch)](https://github.com/flyskywhy/proj.git)/env.sh)
- Can build in Cygwin but use native gcc

## microblaze

- Support microblaze cpu (From [monecos-pre-1.tar.gz](http://www.monstr.eu/monecos-pre-1.tar.gz) in [mONeCos](http://www.monstr.eu/wiki/doku.php?id=ecos:ecos&#download))
- Support GPIO module (From [monecos-pre-1.tar.gz](http://www.monstr.eu/monecos-pre-1.tar.gz) in [mONeCos](http://www.monstr.eu/wiki/doku.php?id=ecos:ecos&#download))
- Support Uart16550 module (From [monecos-pre-1.tar.gz](http://www.monstr.eu/monecos-pre-1.tar.gz) in [mONeCos](http://www.monstr.eu/wiki/doku.php?id=ecos:ecos&#download))
- Support EmacLite module in BigEndian (From [monecos-pre-1.tar.gz](http://www.monstr.eu/monecos-pre-1.tar.gz) in [mONeCos](http://www.monstr.eu/wiki/doku.php?id=ecos:ecos&#download))
- Support SPI module (From [axi_ecos.zip](http://www.ecos4arm.com/redmine/attachments/download/153/axi_ecos.zip) in [eCos on Xilinx Microblaze for AXI](http://www.ecos4arm.com/redmine/projects/ecos-on-microblaze-axi/files))
- Support AXI module and thus LittleEndian (From [axi_ecos.zip](http://www.ecos4arm.com/redmine/attachments/download/153/axi_ecos.zip) in [eCos on Xilinx Microblaze for AXI](http://www.ecos4arm.com/redmine/projects/ecos-on-microblaze-axi/files))
- Support Avnet S6LX9 MMU tiny 13.1 board (Generated from [mbref](https://gitorious.org/~flyskywhy/mbref/flyskywhys-mbref)/working-designs/Avnet-S6LX9-MMU-tiny-13.1/, with the BSP generator [mbref](https://gitorious.org/~flyskywhy/mbref/flyskywhys-mbref)/edk-repository/ThirdParty/bsp/ecos_v2_00_a/ which is also hosted as [ecos_v2_00_a.tar.gz](http://www.monstr.eu/ecos_v2_00_a.tar.gz) in [mONeCos](http://www.monstr.eu/wiki/doku.php?id=ecos:ecos&#download))
- Support 2nd channel and tri_write() of GPIO module
- Support UartLite module (`packages/devs/serial/microblaze/uartlite/`)
- Support EmacLite module in LittleEndian
- More accurate Timer
- Support 2 SPI module instances (`packages/devs/spi/microblaze/`)
- Support SDHC card in SPI mode (`packages/devs/disk/generic/mmc/`)
- Support LogicCORE interrupt controller module

# TODO

I'm busy recently, maybe someone can deal with the remaining:

- To get the windows version of microblazeel-unknown-linux-gnu toolchain (Even microblaze-xilinx-elf from Xilinx 14.1 can pass compile in windows, elf can't run well)
- To reduce the interrupt occur frequency on SPI (which slowdown the speed of SD card)
- To support launch ecos by redboot
