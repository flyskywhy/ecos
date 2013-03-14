#! /bin/sh

# Author: Li Zheng <flyskywhy@gmail.com>

ECOS_ROOT=$(cd "${BASH_ARGV[0]%/*}/.."; pwd)

ECOS_REPOSITORY=$ECOS_ROOT/packages
export ECOS_REPOSITORY
echo
echo "ECOS_REPOSITORY: $ECOS_REPOSITORY"
echo

ECOS_BSP="ecos_v2_00_a"
export ECOS_BSP
