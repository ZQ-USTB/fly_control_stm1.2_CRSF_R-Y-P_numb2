cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
# 关闭未使用函数的警告
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-unused-function")

# 如果你连未使用变量的警告也不想看到，可以再加一个：
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-unused-variable")

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME fly_control_stm)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    algorithm/AHRS_middleware.c
    algorithm/biquad_filter.c
    algorithm/ekf.c
    algorithm/MahonyAHRS.c
    algorithm/pid.c
    
    application/Aircraft_ctrl_task.c
    application/INS_task.c
    application/remote_control.c
    
    bsp/bmi088_anymotiona.c
    bsp/bmi088_mma.c
    bsp/bmi08a.c
    bsp/bmi08g.c
    bsp/bmi08xa.c
    bsp/common.c
    # bsp/elrs.c
    bsp/qmc6310.c
    bsp/spl06.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    algorithm
    application
    bsp
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    ARM_MATH_CM7
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
add_compile_definitions(__FPU_PRESENT=1U)