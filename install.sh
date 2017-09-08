#!/bin/bash

PRGDIR=`dirname $0` 
cd $PRGDIR

TOOLBOX="mikata_arm_toolbox"
if [ "$HOME" = "" ] ; then
    echo "[ERROR] env HOME is not set."
    exit 1
fi
DATADIR="$HOME/.robotis/mikata_arm"

if [ ! -d $DATADIR ] ; then
    mkdir -p $DATADIR
fi
if [ ! -f $DATADIR/positions.txt ] ; then
    cp ./$TOOLBOX/data/positions.txt $DATADIR/
fi
if [ ! -f $DATADIR/motion.txt ] ; then
    cp ./$TOOLBOX/data/motion.txt $DATADIR/
fi

cd ../..
source ./devel/setup.bash
rosrun mikata_arm_toolbox dxl_setup
if [ $? = 0 ] ; then
    echo -e "\n\033[0;32mDYNAMIXEL Mikata Arm 4DOF Sample Program installation succeeded.\033[0m\n"
fi

