cmake_minimum_required(VERSION 3.16)
project(pineapple_hardware_interface)

enable_language(C)
enable_language(CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)
## CANFD setup 
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

add_library(motor STATIC
    src/canfd/damiao.cpp 
    src/canfd/usb_rx_fix.cpp
)
target_include_directories(motor PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/canfd/include
    ${LIBUSB_INCLUDE_DIRS}
)

# Select proper library based on architecture
if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(ADAPTER_ARCH arm)
else()
    set(ADAPTER_ARCH x86)
endif()
set(ADAPTER_VENDOR "${CMAKE_CURRENT_SOURCE_DIR}/src/canfd/lib/${ADAPTER_ARCH}/libu2canfd.a")
set(ADAPTER_FIXED "${CMAKE_CURRENT_BINARY_DIR}/libu2canfd_rx_override.a")
add_custom_command(OUTPUT "${ADAPTER_FIXED}"
    COMMAND ${CMAKE_OBJCOPY}
      --weaken-symbol=_ZN9usb_class15get_data_threadEv
      --weaken-symbol=_ZN9usb_class14can_rev_threadEv
      "${ADAPTER_VENDOR}" "${ADAPTER_FIXED}"
    DEPENDS "${ADAPTER_VENDOR}" VERBATIM)
add_custom_target(adapter_rx_archive DEPENDS "${ADAPTER_FIXED}")
add_dependencies(motor adapter_rx_archive)
target_link_libraries(motor PRIVATE Threads::Threads "${ADAPTER_FIXED}" ${LIBUSB_LIBRARIES})
target_link_options(motor INTERFACE "LINKER:--undefined=_ZN9usb_class13receive_fixedEh")

## IMU setup
include_directories(
    # ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/xspublic
    ${CMAKE_SOURCE_DIR}/xspublic/xscontroller/include
    ${CMAKE_SOURCE_DIR}/xspublic/xscommon/include
    ${CMAKE_SOURCE_DIR}/xspublic/xstypes/include
)

link_directories(
    ${CMAKE_SOURCE_DIR}/xspublic/xscontroller
    ${CMAKE_SOURCE_DIR}/xspublic/xscommon
    ${CMAKE_SOURCE_DIR}/xspublic/xstypes
)

# Xsens libs
set(XSENS_OBJ_LIBS
    xscontroller
    xscommon
    xstypes
    dl
)

FILE (GLOB HARDWARE_SRC
        src/pineapple_hardware/*.cc
        src/imu/*.hpp)

set(HARDWARE_DEPENDENCIES 
    pthread
    yaml-cpp
    unitree_sdk2
)


add_executable(pineapple_hardware_interface ${HARDWARE_SRC} src/main.cc)
target_link_libraries(pineapple_hardware_interface  ${HARDWARE_DEPENDENCIES} motor ${XSENS_OBJ_LIBS})

# Standalone USB2CANFD serial-number scanner
add_executable(scan_canfd_sn src/scan_canfd_sn.cc)
target_include_directories(scan_canfd_sn PRIVATE ${LIBUSB_INCLUDE_DIRS})
target_link_libraries(scan_canfd_sn ${LIBUSB_LIBRARIES})

SET(CMAKE_BUILD_TYPE Release)

# Add Xsens examples
add_executable(example_mti_receive_data
    src/test/example_mti_receive_data.cpp
)
target_link_libraries(example_mti_receive_data ${XSENS_OBJ_LIBS})

add_executable(imu_probe src/test/imu_probe.cpp)
target_link_libraries(imu_probe ${XSENS_OBJ_LIBS} pthread)
