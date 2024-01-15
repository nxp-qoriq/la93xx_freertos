#!/bin/sh
#SPDX-License-Identifier: (BSD-3-Clause OR GPL-2.0)
#Copyright 2022-2024 NXP

boot_mode=pcie

if [ "$#" != "1" ]; then
	echo "Invoke command as show below (boot_mode i2c,pcie)"
	echo "./build_release.sh -boot_mode=i2c"
	echo  "          OR                   "
        echo "./build_release.sh -boot_mode=pci"
else
	file="include/la9310_boot_mode.h"
	if [ -f $file ]; then
		unlink $file
	fi
	i2c_boot=`echo "$*" | grep -i "boot_mode=I2c"`
	[ -n "$i2c_boot" ] && {
		boot_mode=i2c
	}
	if [ "$boot_mode" = "pcie" ];then
		echo "Build NLM for PCIe Host Mode.."
		cmake -DCMAKE_TOOLCHAIN_FILE="./armgcc.cmake" -G "Unix Makefiles" -DTURN_ON_STANDALONE_MODE=OFF  -DCMAKE_BUILD_TYPE=Release  .
		echo "#define TURN_ON_HOST_MODE  1" > $file
	else
		echo "Build NLM for I2C Standalone Mode.."
		cmake -DCMAKE_TOOLCHAIN_FILE="./armgcc.cmake" -G "Unix Makefiles" -DTURN_ON_STANDALONE_MODE=ON  -DCMAKE_BUILD_TYPE=Release  .
		echo "#define TURN_ON_STANDALONE_MODE  1" > $file
	fi
fi
make -j4
