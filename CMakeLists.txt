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
find_package(geometry_msgs REQUIRED)
find_package(leo_msgs REQUIRED)
find_package(microxrcedds_client REQUIRED)
find_package(rcl REQUIRED)
find_package(rclc REQUIRED)
find_package(rclc_parameter REQUIRED)
find_package(rosidl_runtime_c REQUIRED)
find_package(rmw_microxrcedds REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(${PROJECT_NAME})

target_sources(${PROJECT_NAME} PRIVATE
    App/Src/app.cpp
    App/Src/imu_receiver.cpp
    App/Src/motor_controller.cpp
    App/Src/parameters.cpp
    Drivers/icm42605/ICM42605.cpp
    external/micro_ros_cmake/extra_sources/allocators/linear.c
    external/micro_ros_cmake/extra_sources/clocks/clock_stm32cube.c
    external/micro_ros_cmake/extra_sources/transports/stm32cube/microros_serial.c
)

target_include_directories(${PROJECT_NAME} PRIVATE
    App/Inc
    Drivers/icm42605
    external/diff_drive_lib/include
    external/micro_ros_cmake/extra_sources/allocators/include
    external/micro_ros_cmake/extra_sources/transports/stm32cube/include
)

target_compile_definitions(${PROJECT_NAME} PRIVATE
    STM32F4xx
    MICROROS_HEAP_SIZE=30000
)

target_link_libraries(${PROJECT_NAME}
    stm32cubemx
    microxrcedds_client
    rcl::rcl
    rclc::rclc
    rclc_parameter::rclc_parameter
    rosidl_runtime_c::rosidl_runtime_c
)

ament_target_dependencies(${PROJECT_NAME}
    "geometry_msgs"
    "leo_msgs"
    "rmw_microxrcedds"
    "std_msgs"
    "std_srvs"
)

target_link_options(${PROJECT_NAME} PRIVATE
  -Wl,--start-group
)

set(CMAKE_CXX_STANDARD_LIBRARIES "-Wl,--end-group")
