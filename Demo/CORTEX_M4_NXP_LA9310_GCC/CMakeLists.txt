#SPDX-License-Identifier: BSD-3-Clause
#Copyright 2017-2025 NXP

INCLUDE(CMakeForceCompiler)
INCLUDE(CMakeDependentOption)

# CROSS COMPILER SETTING
SET(CMAKE_SYSTEM_NAME Generic)
CMAKE_MINIMUM_REQUIRED (VERSION 2.8.12)
PROJECT(LA931X)

# THE VERSION NUMBER
SET (Tutorial_VERSION_MAJOR 1)
SET (Tutorial_VERSION_MINOR 0)

# ENABLE ASM
ENABLE_LANGUAGE(ASM)

SET(CMAKE_STATIC_LIBRARY_PREFIX)
SET(CMAKE_STATIC_LIBRARY_SUFFIX)

SET(CMAKE_EXECUTABLE_LIBRARY_PREFIX)
SET(CMAKE_EXECUTABLE_LIBRARY_SUFFIX)

# Treat warnings as errors
set(CMAKE_COMPILE_WARNING_AS_ERROR ON)

#SET(CMAKE_VERBOSE_MAKEFILE ON)
# CURRENT DIRECTORY
SET(ProjDirPath ${CMAKE_CURRENT_SOURCE_DIR})
SET(LA9310_COMMON_HEADERS ${ProjDirPath}/../../common_headers)

# RELEASE LINK FILE
if(TURN_ON_STANDALONE_MODE)
       set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -T${ProjDirPath}/platform/ARM_CM4/la9310_standalone_i2c_mode.ld  -static")
else()
       set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -T${ProjDirPath}/platform/ARM_CM4/la9310_host_pcie_mode.ld  -static")
endif()

# RELEASE ASM FLAGS
SET(CMAKE_ASM_FLAGS_RELEASE "${CMAKE_ASM_FLAGS_RELEASE} -g  -mcpu=cortex-m4  -mfloat-abi=soft  -mthumb  -Wall -Wno-packed-bitfield-compat  -Wno-address-of-packed-member -fno-common  -ffunction-sections  -fdata-sections  -ffreestanding  -fno-builtin  -Os  -mapcs  -std=gnu99")

# RELEASE C FLAGS
SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -g  -fno-strict-aliasing  -mcpu=cortex-m4  -mfloat-abi=soft  -mthumb  -MMD  -MP  -Wall -Wno-packed-bitfield-compat  -Wno-address-of-packed-member -fno-common  -ffunction-sections  -fdata-sections  -ffreestanding  -fno-builtin  -Os  -mapcs  -std=gnu99  -Wno-unused-function -mno-sched-prolog")

# RELEASE LD FLAGS
SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -g  -mcpu=cortex-m4  -mfloat-abi=soft  --specs=nano.specs  -lm  -Wall -Wno-packed-bitfield-compat  -Wno-address-of-packed-member  -fno-common  -ffunction-sections  -fdata-sections  -ffreestanding  -fno-builtin  -Os  -mthumb  -mapcs  -Xlinker --gc-sections  -Xlinker -static  -Xlinker -z  -Xlinker muldefs")

# ASM MACRO
SET(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG}  -D__DEBUG")

#Enable/Disable command line
if(SDR_DFE)
	SET(LA9310_1SEC_PPS ON)
	SET(LA9310_CONSECUTIVE_S_SLOTS ON)
else()
	SET(LA9310_TURN_ON_COMMAND_LINE ON)
endif()

if(BOARD_SDR)
    SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}  -DSDR")
endif()
if(BOARD_SEEVE)
	SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}  -DSEEVE")
endif()

# Enable/Disable RFIC driver
if(SDR_DFE)
	SET(RFIC OFF)
else()
	SET(RFIC ON)
endif()

# C MACRO
if(LA9310_TURN_ON_COMMAND_LINE)
    SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}  -D__NDEBUG -DLA9310_ENABLE_COMMAND_LINE")
else()
    SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}  -D__NDEBUG")
endif()

# C MACRO
if(LA9310_DFE_APP)
    SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}  -D__NDEBUG -DLA9310_DFE_APP")
else()
    SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}  -D__NDEBUG")
endif()

# C MACRO
if(LA9310_1SEC_PPS)
    SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}  -D__NDEBUG -DLA9310_1SEC_PPS")
else()
    SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}  -D__NDEBUG")
endif()

# C MACRO
if(LA9310_CONSECUTIVE_S_SLOTS)
    SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}  -D__NDEBUG -DDFE_CONSECUTIVE_S_SLOTS_WA")
else()
    SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}  -D__NDEBUG")
endif()

if(RFIC)
    SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}  -D__RFIC")
endif()

# CXX MACRO

# INCLUDE_DIRECTORIES
    INCLUDE_DIRECTORIES(${ProjDirPath}/../../FreeRTOS-Kernel/portable/GCC/ARM_CM4)
    INCLUDE_DIRECTORIES(${ProjDirPath}/../../FreeRTOS-Kernel/include)
    INCLUDE_DIRECTORIES(${ProjDirPath}/include)
    INCLUDE_DIRECTORIES(${ProjDirPath}/platform/ARM_CM4/include/)
    INCLUDE_DIRECTORIES(${ProjDirPath}/platform/boards/la9310/)
    INCLUDE_DIRECTORIES(${ProjDirPath}/drivers/misc/)
    INCLUDE_DIRECTORIES(${ProjDirPath}/drivers/edma/)
    INCLUDE_DIRECTORIES(${ProjDirPath}/drivers/watchdog/)
    INCLUDE_DIRECTORIES(${ProjDirPath}/drivers/i2c/)
    INCLUDE_DIRECTORIES(${ProjDirPath}/drivers/dspi)
    INCLUDE_DIRECTORIES(${ProjDirPath}/drivers/phy_timer)
    INCLUDE_DIRECTORIES(${ProjDirPath}/drivers/timesync)
    INCLUDE_DIRECTORIES(${ProjDirPath}/sw_cmd_engine)
    INCLUDE_DIRECTORIES(${LA9310_COMMON_HEADERS}/)
    INCLUDE_DIRECTORIES(${ProjDirPath}/utils/)
if(LA9310_TURN_ON_COMMAND_LINE)
    INCLUDE_DIRECTORIES(${ProjDirPath}/utils/FreeRTOS-Plus-CLI/)
endif()
if(RFIC)
    INCLUDE_DIRECTORIES(${ProjDirPath}/drivers/rfic/)
endif()
    INCLUDE_DIRECTORIES(${ProjDirPath}/drivers/dcs)

if(TURN_ON_STANDALONE_MODE)
SET(LA9310_SOURCES
    "${ProjDirPath}/../../FreeRTOS-Kernel/portable/GCC/ARM_CM4/port.c"
    "${ProjDirPath}/../../FreeRTOS-Kernel/portable/MemMang/heap_2.c"
    "${ProjDirPath}/../../FreeRTOS-Kernel/croutine.c"
    "${ProjDirPath}/../../FreeRTOS-Kernel/event_groups.c"
    "${ProjDirPath}/../../FreeRTOS-Kernel/list.c"
    "${ProjDirPath}/../../FreeRTOS-Kernel/queue.c"
    "${ProjDirPath}/../../FreeRTOS-Kernel/tasks.c"
    "${ProjDirPath}/../../FreeRTOS-Kernel/timers.c"
    "${ProjDirPath}/drivers/serial/debug_console.c"
    "${ProjDirPath}/drivers/serial/print_scan.c"
    "${ProjDirPath}/drivers/serial/serial_ns16550.c"
    "${ProjDirPath}/drivers/edma/la9310_edma.c"
    "${ProjDirPath}/drivers/misc/mem_log.c"
    "${ProjDirPath}/drivers/watchdog/la9310_watchdog.c"
    "${ProjDirPath}/platform/ARM_CM4/soc.c"
    "${ProjDirPath}/platform/ARM_CM4/start.S"
    "${ProjDirPath}/platform/boards/la9310/board.c"
    "${ProjDirPath}/platform/ARM_CM4/exceptions.c"
    "${ProjDirPath}/platform/boards/la9310/hardware_init.c"
    "${ProjDirPath}/main.c"
    "${ProjDirPath}/la9310_demo.c"
    "${ProjDirPath}/la9310_irq.c"
    "${ProjDirPath}/drivers/avi/la9310_avi.c"
    "${ProjDirPath}/drivers/avi/la9310_vspa_dma.c"
    "${ProjDirPath}/drivers/misc/la9310_gpio.c"
    "${ProjDirPath}/drivers/misc/la9310_pinmux.c"
    "${ProjDirPath}/drivers/i2c/la9310_i2c.c"
    "${ProjDirPath}/drivers/dspi/fsl_dspi.c"
    "${ProjDirPath}/drivers/dspi/dspi_test.c"
    "${ProjDirPath}/drivers/phy_timer/phytimer.c"
    "${ProjDirPath}/drivers/timesync/sync_timing_device.c"
    "${ProjDirPath}/sw_cmd_engine/sw_cmd_engine.c"
    "${ProjDirPath}/utils/delay.c"
  )
else()
SET(LA9310_SOURCES
    "${ProjDirPath}/../../FreeRTOS-Kernel/portable/GCC/ARM_CM4/port.c"
    "${ProjDirPath}/../../FreeRTOS-Kernel/portable/MemMang/heap_2.c"
    "${ProjDirPath}/../../FreeRTOS-Kernel/croutine.c"
    "${ProjDirPath}/../../FreeRTOS-Kernel/event_groups.c"
    "${ProjDirPath}/../../FreeRTOS-Kernel/list.c"
    "${ProjDirPath}/../../FreeRTOS-Kernel/queue.c"
    "${ProjDirPath}/../../FreeRTOS-Kernel/tasks.c"
    "${ProjDirPath}/../../FreeRTOS-Kernel/timers.c"
    "${ProjDirPath}/drivers/serial/debug_console.c"
    "${ProjDirPath}/drivers/serial/print_scan.c"
    "${ProjDirPath}/drivers/serial/serial_ns16550.c"
    "${ProjDirPath}/drivers/edma/la9310_edma.c"
    "${ProjDirPath}/drivers/misc/mem_log.c"
    "${ProjDirPath}/drivers/watchdog/la9310_watchdog.c"
    "${ProjDirPath}/platform/ARM_CM4/soc.c"
    "${ProjDirPath}/platform/ARM_CM4/start.S"
    "${ProjDirPath}/platform/boards/la9310/board.c"
    "${ProjDirPath}/platform/ARM_CM4/exceptions.c"
    "${ProjDirPath}/platform/boards/la9310/hardware_init.c"
    "${ProjDirPath}/main.c"
    "${ProjDirPath}/la9310_demo.c"
    "${ProjDirPath}/drivers/serial/nmm_logging.c"
    "${ProjDirPath}/la9310_irq.c"
    "${ProjDirPath}/la9310_v2h_test.c"
    "${ProjDirPath}/bbdev_ipc_raw_demo.c"
    "${ProjDirPath}/drivers/misc/bbdev_ipc.c"
    "${ProjDirPath}/drivers/avi/la9310_avi.c"
    "${ProjDirPath}/drivers/misc/la9310_gpio.c"
    "${ProjDirPath}/drivers/misc/la9310_pinmux.c"
    "${ProjDirPath}/drivers/i2c/la9310_i2c.c"
    "${ProjDirPath}/drivers/pcie/la9310_pcie.c"
    "${ProjDirPath}/drivers/dspi/fsl_dspi.c"
    "${ProjDirPath}/drivers/dspi/dspi_test.c"
    "${ProjDirPath}/drivers/phy_timer/phytimer.c"
    "${ProjDirPath}/drivers/timesync/sync_timing_device.c"
    "${ProjDirPath}/sw_cmd_engine/sw_cmd_engine.c"
    "${ProjDirPath}/utils/delay.c"
    "${ProjDirPath}/utils/task_stats.c"
  )
endif()

SET(LA9310_SOURCES ${LA9310_SOURCES}
    "${ProjDirPath}/drivers/dcs/la9310_dcs.c"
   )

if(BOARD_SDR OR BOARD_SEEVE)
    SET(LA9310_SOURCES ${LA9310_SOURCES}
         "${ProjDirPath}/tti_gpio12_test.c"
       )
endif()

if(LA9310_TURN_ON_COMMAND_LINE)
    SET(LA9310_SOURCES ${LA9310_SOURCES}
        "${ProjDirPath}/utils/FreeRTOS-Plus-CLI/FreeRTOS_CLI.c"
        "${ProjDirPath}/utils/la9310_nlm_test_command.c"
        "${ProjDirPath}/utils/UARTCommandConsole.c"
        "${ProjDirPath}/drivers/timesync/sync_timing_device_cli.c"
      )
endif()

if (LA9310_DFE_APP)
     SET(LA9310_SOURCES ${LA9310_SOURCES}
        "${ProjDirPath}/dfe_avi.c"
        "${ProjDirPath}/dfe_app.c"
        "${ProjDirPath}/sdr_rf_ctrl.c"
      )
     if(LA9310_TURN_ON_COMMAND_LINE)
         SET(LA9310_SOURCES ${LA9310_SOURCES}
             "${ProjDirPath}/utils/la9310_dfe_test_command.c"
          )
     endif()
endif()

if(RFIC)
    SET(LA9310_SOURCES ${LA9310_SOURCES}
        "${ProjDirPath}/drivers/rfic/rfic_init.c"
        "${ProjDirPath}/drivers/rfic/rfic_core.c"
        "${ProjDirPath}/drivers/rfic/rfic_avi_ctrl.c"
        "${ProjDirPath}/drivers/rfic/rfic_api.c"
        "${ProjDirPath}/drivers/rfic/rfic_vga.c"
        "${ProjDirPath}/drivers/rfic/rfic_synth.c"
        "${ProjDirPath}/drivers/rfic/rfic_lmx2582_synth.c"
        "${ProjDirPath}/drivers/rfic/rfic_max2870_synth.c"
        "${ProjDirPath}/drivers/rfic/rfic_demod.c"
        "${ProjDirPath}/drivers/rfic/rfic_cmd.c"
        "${ProjDirPath}/drivers/rfic/rfic_dac.c"
	"${ProjDirPath}/sdr_rf_ctrl.c"
        )
endif()

ADD_EXECUTABLE(la9310 ${LA9310_SOURCES})

SET_TARGET_PROPERTIES(la9310 PROPERTIES OUTPUT_NAME "la9310.elf")

TARGET_LINK_LIBRARIES(la9310 -Wl,--start-group)
# LIBRARIES
#IF(CMAKE_BUILD_TYPE MATCHES Debug)
#ELSEIF(CMAKE_BUILD_TYPE MATCHES Release)
#ENDIF()

# SYSTEM LIBRARIES
TARGET_LINK_LIBRARIES(la9310 m)
TARGET_LINK_LIBRARIES(la9310 c)
TARGET_LINK_LIBRARIES(la9310 gcc)
TARGET_LINK_LIBRARIES(la9310 nosys)
TARGET_LINK_LIBRARIES(la9310 -Wl,--end-group)

# MAP FILE
SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG}  -Xlinker -Map=debug/la9310.map")
SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE}  -Xlinker -Map=release/la9310.map")

# BIN AND HEX
ADD_CUSTOM_COMMAND(TARGET la9310 POST_BUILD COMMAND ${CMAKE_OBJCOPY} -Oihex ${EXECUTABLE_OUTPUT_PATH}/la9310.elf ${EXECUTABLE_OUTPUT_PATH}/la9310.hex)
ADD_CUSTOM_COMMAND(TARGET la9310 POST_BUILD COMMAND ${CMAKE_OBJCOPY} -Obinary
	${EXECUTABLE_OUTPUT_PATH}/la9310.elf
	${EXECUTABLE_OUTPUT_PATH}/${FILE_NAME})
