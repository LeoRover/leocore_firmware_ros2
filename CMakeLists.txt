cmake_minimum_required(VERSION 3.22)

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

include (cmake/gcc-arm-none-eabi.cmake)
include (cmake/firmware_version.cmake)

project(leocore_firmware)
enable_language(C CXX ASM)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

include(FetchContent)

FetchContent_Declare(
    micro_ros_cmake
    SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/external/micro_ros_cmake"
)

set(MICRO_ROS_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/gcc-arm-none-eabi.cmake" CACHE STRING "" FORCE)
set(MICRO_ROS_COLCON_META_FILE "${CMAKE_CURRENT_SOURCE_DIR}/Micro-ROS/colcon.meta" CACHE STRING "" FORCE)
set(MICRO_ROS_EXTRA_PACKAGES_FILE "${CMAKE_CURRENT_SOURCE_DIR}/Micro-ROS/extra_packages.repos" CACHE STRING "" FORCE)

FetchContent_MakeAvailable(micro_ros_cmake)

message("Build type: " ${CMAKE_BUILD_TYPE})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

find_package(micro_ros_cmake REQUIRED)
find_package(ament_cmake REQUIRED)

# Create an executable object type
add_executable(${PROJECT_NAME})

# Link directories setup
target_link_directories(${PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${PROJECT_NAME} PRIVATE
    App/Src/app.cpp
    App/Src/imu_receiver.cpp
    App/Src/microros_allocators.cpp
    App/Src/motor_controller.cpp
    Drivers/icm42605/ICM42605.cpp
)

# Add include paths
target_include_directories(${PROJECT_NAME} PRIVATE
    App/Inc
    Drivers/icm42605
    external/diff_drive_lib/include
)

# Add project symbols (macros)
target_compile_definitions(${PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
