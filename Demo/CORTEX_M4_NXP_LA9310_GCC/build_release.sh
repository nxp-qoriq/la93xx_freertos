#!/bin/bash
#SPDX-License-Identifier: (BSD-3-Clause OR GPL-2.0)
#Copyright 2022-2025 NXP
set -x

# Initialize our own variables:
target_model="sdr"
boot_mode="pcie"
log_level="info"
build_variant="release"
features=""
build_flags=""
file_name="la9310.bin"

show_help()
{
       echo "Usage: ./$1 -t <target_model> -l <log_level> -b <build_variant> -m <boot_mode> -f <features> -n <file_name>"
       echo "       target_model = {nlm, sdr, sdr_nxp, sdr_dfe, seeve, nmm}"
       echo "       log_level = {err, info, dbg, isr, all}"
       echo "       build_variant = {debug, release}"
       echo "       boot_mode = {i2c, pcie}"
       echo "       features = { NA }"
       echo "       file_name = { la9310.bin, la9310_release.bin, etc}"
}

while getopts "ht:l:b:m:c:f:n:" opt; do
       case "$opt" in
               h)  show_help
                   exit 0
                       ;;
               t)  target_model=$OPTARG
                       ;;
               l)  log_level=$OPTARG
                       ;;
               b)  build_variant=$OPTARG
                       ;;
               m)  boot_mode=$OPTARG
                       ;;
               f)  features+=":$OPTARG"
                       ;;
               n)  file_name="$OPTARG"
                       ;;
       esac
done
shift $((OPTIND -1))

if [ "$boot_mode" != "pcie" -a "$boot_mode" != "i2c" ]
then
show_help
exit 0
fi

if [ "$target_model" != "sdr" -a "$target_model" != "nlm" -a "$target_model" != "sdr_nxp" -a "$target_model" != "sdr_dfe"  -a "$target_model" != "seeve" ]
then
show_help
exit 0
fi

echo "features = { $features }"
#check feature define presence
if [ "$features" ]
then
        ENABLE_FEATURES=`echo $features | sed 's/:/ -D/g'`
        echo "ENABLE_FEATURES = $ENABLE_FEATURES"

        build_flags+=$ENABLE_FEATURES;
fi

file="include/la9310_boot_mode.h"
unlink $file

if [ "$boot_mode" = "pcie" ]
then
	echo "Build for PCIe Host Mode.."
	build_flags=" -DTURN_ON_STANDALONE_MODE=OFF $build_flags"
	echo "#define TURN_ON_HOST_MODE  1" > $file
else
	echo "Build for I2C Standalone Mode.."
	build_flags=" -DTURN_ON_STANDALONE_MODE=ON $build_flags"
	echo "#define TURN_ON_STANDALONE_MODE  1" > $file
fi

if [ "$target_model" = "sdr" ]
then
	build_flags=" -DBOARD_SDR=ON $build_flags"
else
	build_flags=" -DBOARD_SDR=OFF $build_flags"
fi
if [ "$target_model" = "seeve" ]
then
	build_flags=" -DBOARD_SEEVE=ON $build_flags"
else
	build_flags=" -DBOARD_SEEVE=OFF $build_flags"
fi
if [ "$target_model" == "sdr_dfe" ];then
	build_flags=" -DSDR_DFE=ON $build_flags"
else
	build_flags=" -DSDR_DFE=OFF $build_flags"
fi
if [ -n "$file_name" ];then
	build_flags=" -DFILE_NAME=$file_name $build_flags"
fi

echo "#######################################################################################################################"
echo "Building FreeRTOS Image for target_model=$target_model, log_level=$log_level build_variant=$build_variant boot_mode=$boot_mode features=$features#"
echo "build_flags=$build_flags"
cmake -DCMAKE_TOOLCHAIN_FILE="./armgcc.cmake" -G "Unix Makefiles" $build_flags -DCMAKE_BUILD_TYPE=$build_variant  .
make -j4
