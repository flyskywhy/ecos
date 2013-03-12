#! /bin/sh

# Author: Li Zheng <flyskywhy@gmail.com>
# Ref to Android/build/envsetup.sh

ECOS_ROOT=$(cd "${BASH_ARGV[0]%/*}/.."; pwd)

ECOS_REPOSITORY=$ECOS_ROOT/packages
export ECOS_REPOSITORY
echo
echo "ECOS_REPOSITORY: $ECOS_REPOSITORY"
echo

ECOS_BSP="ecos_v2_00_a"
export ECOS_BSP

# determine whether arrays are zero-based (bash) or one-based (zsh)
_xarray=(a b c)
if [ -z "${_xarray[${#_xarray[@]}]}" ]
then
    _arrayoffset=1
else
    _arrayoffset=0
fi
unset _xarray

function add_lunch_combo()
{
    LUNCH_MENU_CHOICES=(`grep "template    redboot" *.ecc -L`)
}

function print_lunch_menu()
{
    local uname=$(uname)
    echo
    echo "You're building on" $uname
    echo
    echo "Lunch menu... pick a combo:"

    local i=1
    local choice
    add_lunch_combo
    for choice in ${LUNCH_MENU_CHOICES[@]}
    do
        echo "     $i. $choice"
        i=$(($i+1))
    done

    echo
}

function lunch()
{
    local answer

    if [ "$1" ] ; then
        answer=$1
    else
        print_lunch_menu
        echo -n "Which would you like? [${LUNCH_MENU_CHOICES[$((1-$_arrayoffset))]}] "
        read answer
    fi

    local selection=

    if [ -z "$answer" ]
    then
        selection=${ECOS_ECC_FILE}
    elif (echo -n $answer | grep -q -e "^[0-9][0-9]*$")
    then
        if [ $answer -le ${#LUNCH_MENU_CHOICES[@]} ]
        then
            selection=${LUNCH_MENU_CHOICES[$(($answer-$_arrayoffset))]}
        fi
    elif (echo -n $answer | grep -q -e "\.ecc$")
    then
        selection=$answer
    fi

    if [ -z "$selection" ]
    then
        echo
        echo "Invalid lunch combo: $answer"
        return 1
    fi

    export ECOS_ECC_FILE=$selection

    echo
    echo "ECOS_ECC_FILE: $ECOS_ECC_FILE"
    echo
}
