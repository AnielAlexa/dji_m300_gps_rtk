cmake_minimum_required(VERSION 3.16)
project(dji_m300_gps_rtk)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 99)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Path to DJI Payload SDK - support multiple location methods
# Priority: Environment variable > Relative path > Default path
if(DEFINED ENV{DJI_SDK_PATH})
  set(PSDK_PATH "$ENV{DJI_SDK_PATH}")
  message(STATUS "Using DJI SDK from environment: ${PSDK_PATH}")
elseif(EXISTS "${CMAKE_SOURCE_DIR}/../../../../Payload-SDK")
  # Relative path from workspace
  get_filename_component(PSDK_PATH "${CMAKE_SOURCE_DIR}/../../../../Payload-SDK" ABSOLUTE)
  message(STATUS "Using DJI SDK from relative path: ${PSDK_PATH}")
else()
  # Fallback to absolute path (for current setup)
  set(PSDK_PATH "/home/aniel/skyline_drone/Payload-SDK")
  message(STATUS "Using default DJI SDK path: ${PSDK_PATH}")
endif()

# Check if Payload SDK exists
if(NOT EXISTS "${PSDK_PATH}")
  message(FATAL_ERROR "DJI Payload SDK not found at '${PSDK_PATH}'")
endif()
message(STATUS "DJI Payload SDK found at: ${PSDK_PATH}")

# Add system dependencies
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

# Include directories for Payload SDK
include_directories(
  include
  ${PSDK_PATH}/psdk_lib/include
  ${PSDK_PATH}/samples/sample_c/platform/linux/manifold2/hal
  ${PSDK_PATH}/samples/sample_c/platform/linux/manifold2/osal
  ${PSDK_PATH}/samples/sample_c/platform/linux/common
  /usr/include/libusb-1.0
)

# Link directories
link_directories(
  ${PSDK_PATH}/psdk_lib/lib/x86_64-linux-gnu-gcc
)

# Source files from Payload SDK for UART/USB communication
set(PSDK_HAL_SOURCES
  ${PSDK_PATH}/samples/sample_c/platform/linux/manifold2/hal/hal_uart.c
  ${PSDK_PATH}/samples/sample_c/platform/linux/manifold2/hal/hal_network.c
  ${PSDK_PATH}/samples/sample_c/platform/linux/manifold2/hal/hal_usb_bulk.c
)

set(PSDK_OSAL_SOURCES
  ${PSDK_PATH}/samples/sample_c/platform/linux/common/osal/osal_fs.c
  ${PSDK_PATH}/samples/sample_c/platform/linux/common/osal/osal_socket.c
  ${PSDK_PATH}/samples/sample_c/platform/linux/common/osal/osal.c
)

# Create the ROS2 node executable
add_executable(gps_rtk_node
  src/gps_rtk_node.cpp
  ${PSDK_HAL_SOURCES}
  ${PSDK_OSAL_SOURCES}
)

# Link all libraries using plain syntax (must be consistent)
target_link_libraries(gps_rtk_node
  Threads::Threads
  ${LIBUSB_LIBRARIES}
  payloadsdk
)

# Add include directories from libusb
target_include_directories(gps_rtk_node PRIVATE ${LIBUSB_INCLUDE_DIRS})

# Add compile options
target_compile_options(gps_rtk_node PRIVATE
  -Wall
  -Wextra
  -Wpedantic
)

# Declare ROS2 dependencies
ament_target_dependencies(gps_rtk_node
  rclcpp
  sensor_msgs
  std_msgs
)

# Install the executable
install(TARGETS gps_rtk_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
