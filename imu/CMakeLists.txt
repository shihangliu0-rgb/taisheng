cmake_minimum_required(VERSION 3.16)

project(robocon_imu C ASM)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Definitions
add_compile_definitions(
    USE_HAL_DRIVER
    STM32F405xx
)

# Include directories
include_directories(
    Core/Inc
    Core/app
    Drivers/STM32F4xx_HAL_Driver/Inc
    Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
    Drivers/CMSIS/Device/ST/STM32F4xx/Include
    Drivers/CMSIS/Include
)

# Sources
file(GLOB_RECURSE USER_SOURCES
    "Core/Src/*.c"
    "Core/app/*.c"
    "Drivers/STM32F4xx_HAL_Driver/Src/*.c"
)

add_library(robocon_imu_lib STATIC ${USER_SOURCES})
