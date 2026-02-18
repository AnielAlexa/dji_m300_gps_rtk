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

# Vendored DJI Payload SDK (self-contained, no external dependency)
set(PSDK_VENDOR_DIR "${CMAKE_CURRENT_SOURCE_DIR}/psdk_vendor")

# Add system dependencies
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

# Include directories - all local
include_directories(
  include
  ${PSDK_VENDOR_DIR}/include
  ${PSDK_VENDOR_DIR}/hal
  ${PSDK_VENDOR_DIR}/osal
  ${PSDK_VENDOR_DIR}
  /usr/include/libusb-1.0
)

# Link the vendored static library
link_directories(
  ${PSDK_VENDOR_DIR}/lib
)

# HAL and OSAL source files (vendored)
set(PSDK_HAL_SOURCES
  ${PSDK_VENDOR_DIR}/hal/hal_uart.c
  ${PSDK_VENDOR_DIR}/hal/hal_network.c
  ${PSDK_VENDOR_DIR}/hal/hal_usb_bulk.c
)

set(PSDK_OSAL_SOURCES
  ${PSDK_VENDOR_DIR}/osal/osal_fs.c
  ${PSDK_VENDOR_DIR}/osal/osal_socket.c
  ${PSDK_VENDOR_DIR}/osal/osal.c
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
