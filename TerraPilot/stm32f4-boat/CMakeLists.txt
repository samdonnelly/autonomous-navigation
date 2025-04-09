# STM32 Minimal CMake project for C/C++ projects
cmake_minimum_required(VERSION 3.12)

###############################################################################

include(./arm-none-eabi-gcc.cmake)

# Set project name and source code folder location
project(autonomous_navigation)

option(DUMP_ASM "Create full assembly of final executable" OFF)

# Set microcontroller information
set(MCU_FAMILY STM32F4xx)
set(MCU_MODEL STM32F411xE)
set(CPU_PARAMETERS
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard)

set(STARTUP_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/build_tools/startup_stm32f411xe.s)
set(MCU_LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/build_tools/STM32F411RETx_FLASH.ld)

###############################################################################
set(EXECUTABLE ${CMAKE_PROJECT_NAME})

enable_language(C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

###############################################################################

###############################################################################
# Headers 

# STM generated code 
set(STM32CUBEMX_INCLUDE_DIRECTORIES
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../STM32F4-driver-library/stmcode/FATFS/Target
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../STM32F4-driver-library/stmcode/FATFS/App
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../STM32F4-driver-library/stmcode/Middlewares/Third_Party/FatFs/src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../STM32F4-driver-library/stmcode/Drivers/CMSIS/Device/ST/STM32F4xx/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../STM32F4-driver-library/stmcode/Drivers/CMSIS/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../STM32F4-driver-library/stmcode/Drivers/STM32F4xx_HAL_Driver/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../STM32F4-driver-library/stmcode/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy)

# STM32F4 project code 
set(STM32F4_PROJECT_INCLUDE_DIRECTORIES 
    ${CMAKE_CURRENT_SOURCE_DIR}/headers/config_files/devices
    ${CMAKE_CURRENT_SOURCE_DIR}/headers/config_files/system
    ${CMAKE_CURRENT_SOURCE_DIR}/headers/core
    ${CMAKE_CURRENT_SOURCE_DIR}/headers/includes
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../STM32F4-driver-library/headers/core
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../STM32F4-driver-library/headers/devices
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../STM32F4-driver-library/headers/peripherals
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../STM32F4-driver-library/headers/tools)

# Autopilot project code 
set(AUTOPILOT_PROJECT_INCLUDE_DIRECTORIES
    ${CMAKE_CURRENT_SOURCE_DIR}/../autopilot/boat
    ${CMAKE_CURRENT_SOURCE_DIR}/../autopilot/system
    ${CMAKE_CURRENT_SOURCE_DIR}/../autopilot/vehicle)

# External library code 
set(LIBRARY_INCLUDE_DIRECTORIES 
    # MAVLink 
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../mavlink/c_library_v2
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../mavlink/c_library_v2/common
    # CMSIS 
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../CMSIS_6/CMSIS/RTOS2/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../CMSIS-FreeRTOS/CMSIS/RTOS2/FreeRTOS/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../CMSIS-FreeRTOS/Source/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../CMSIS-FreeRTOS/Source/portable/GCC/ARM_CM4F)

###############################################################################

###############################################################################
# Sources 

# STM generated code 
file(GLOB_RECURSE STM32CUBEMX_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../STM32F4-driver-library/stmcode/FATFS/Target/user_diskio.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../STM32F4-driver-library/stmcode/FATFS/App/fatfs.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../STM32F4-driver-library/stmcode/Middlewares/Third_Party/FatFs/src/diskio.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../STM32F4-driver-library/stmcode/Middlewares/Third_Party/FatFs/src/ff.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../STM32F4-driver-library/stmcode/Middlewares/Third_Party/FatFs/src/ff_gen_drv.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../STM32F4-driver-library/stmcode/Middlewares/Third_Party/FatFs/src/option/syscall.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../STM32F4-driver-library/stmcode/Middlewares/Third_Party/FatFs/src/option/ccsbcs.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../STM32F4-driver-library/stmcode/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../STM32F4-driver-library/stmcode/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../STM32F4-driver-library/stmcode/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../STM32F4-driver-library/stmcode/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../STM32F4-driver-library/stmcode/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../STM32F4-driver-library/stmcode/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../STM32F4-driver-library/stmcode/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../STM32F4-driver-library/stmcode/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../STM32F4-driver-library/stmcode/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../STM32F4-driver-library/stmcode/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../STM32F4-driver-library/stmcode/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../STM32F4-driver-library/stmcode/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../STM32F4-driver-library/stmcode/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../STM32F4-driver-library/stmcode/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../STM32F4-driver-library/stmcode/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c)

# STM32F4 project code 
file(GLOB_RECURSE STM32F4_PROJECT_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/sources/hardware.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/sources/config_files/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/sources/config_files/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/sources/config_files/devices/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/sources/config_files/devices/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/sources/config_files/system/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/sources/config_files/system/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/sources/core/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/sources/core/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../STM32F4-driver-library/sources/devices/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../STM32F4-driver-library/sources/devices/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../STM32F4-driver-library/sources/peripherals/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../STM32F4-driver-library/sources/peripherals/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../STM32F4-driver-library/sources/tools/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../STM32F4-driver-library/sources/tools/*.cpp)

# Autopilot project code 
file(GLOB_RECURSE AUTOPILOT_PROJECT_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/../autopilot/boat/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../autopilot/boat/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../autopilot/system/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../autopilot/system/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../autopilot/vehicle/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../autopilot/vehicle/*.cpp)

# External library code 
file(GLOB_RECURSE LIBRARY_PROJECT_SOURCES CONFIGURE_DEPENDS
    # CMSIS 
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../CMSIS-FreeRTOS/CMSIS/RTOS2/FreeRTOS/Source/cmsis_os2.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../CMSIS-FreeRTOS/Source/list.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../CMSIS-FreeRTOS/Source/queue.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../CMSIS-FreeRTOS/Source/tasks.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../CMSIS-FreeRTOS/Source/timers.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../CMSIS-FreeRTOS/Source/portable/MemMang/heap_4.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../CMSIS-FreeRTOS/Source/portable/GCC/ARM_CM4F/*.c)

###############################################################################

# Executable files 
add_executable(${EXECUTABLE}
    ${STM32CUBEMX_SOURCES} 
    ${STM32F4_PROJECT_SOURCES}
    ${AUTOPILOT_PROJECT_SOURCES}
    ${LIBRARY_PROJECT_SOURCES}
    ${STARTUP_SCRIPT})

# Embedded macros (defines) 
target_compile_definitions(${EXECUTABLE} PRIVATE
    #$<$<CONFIG:Debug>:DEBUG>
    ${MCU_MODEL}
    USE_HAL_DRIVER)

# Add header directories (***AFTER add_executable) 
target_include_directories(${EXECUTABLE} SYSTEM PRIVATE
    ${STM32CUBEMX_INCLUDE_DIRECTORIES})
target_include_directories(${EXECUTABLE} PRIVATE
    ${STM32F4_PROJECT_INCLUDE_DIRECTORIES})
target_include_directories(${EXECUTABLE} PRIVATE
    ${AUTOPILOT_PROJECT_INCLUDE_DIRECTORIES})
target_include_directories(${EXECUTABLE} PRIVATE
    ${LIBRARY_INCLUDE_DIRECTORIES})

###############################################################################

# Compiler and linker options 
target_compile_options(${EXECUTABLE} PRIVATE
    ${CPU_PARAMETERS}
        -Wall
        -Wextra
        -Wpedantic
        -Wshadow
        -Wdouble-promotion
        -Wformat=2 -Wformat-truncation
        -Wundef
        -fno-common
        -Wno-unused-parameter
        -Wno-format-nonliteral
        #######################################
        # Added by me 
        # Due to mavlink library inclusion: 
        -Wno-pedantic 
        -Wno-address-of-packed-member
        #######################################
        $<$<COMPILE_LANGUAGE:CXX>:
            # -Wconversion # STM libraries!
            -Wno-volatile
            # -Wold-style-cast
            # -Wuseless-cast
            -Wsuggest-override>
        $<$<CONFIG:Debug>:-Og -g3 -ggdb>
        $<$<CONFIG:Release>:-Og -g0>)

    target_link_options(${EXECUTABLE} PRIVATE
        -T${MCU_LINKER_SCRIPT}
        ${CPU_PARAMETERS}
        -Wl,-Map=${CMAKE_PROJECT_NAME}.map
        --specs=nosys.specs
        -Wl,--start-group
        -lc
        -lm
        -lstdc++
        -Wl,--end-group
        -Wl,--print-memory-usage)

# The last command can take a couple of seconds on larger project, usefull for debugging
add_custom_command(TARGET ${EXECUTABLE} POST_BUILD
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${EXECUTABLE}>
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${EXECUTABLE}> ${EXECUTABLE}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${EXECUTABLE}> ${EXECUTABLE}.bin)

if (${DUMP_ASM})
    add_custom_command(TARGET ${EXECUTABLE} POST_BUILD
        COMMAND ${CMAKE_OBJDUMP} -D $<TARGET_FILE:${EXECUTABLE}> > ${EXECUTABLE}.s)
endif()
