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
)
target_include_directories(motor PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/canfd/include
    ${LIBUSB_INCLUDE_DIRS}
)

# Select proper library based on architecture
if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
    target_link_libraries(motor PRIVATE
    Threads::Threads
    ${CMAKE_CURRENT_SOURCE_DIR}/src/canfd/lib/arm/libu2canfd.a
    ${LIBUSB_LIBRARIES}
)
else()
    target_link_libraries(motor PRIVATE
    Threads::Threads
    ${CMAKE_CURRENT_SOURCE_DIR}/src/canfd/lib/x86/libu2canfd.a
    ${LIBUSB_LIBRARIES}
)
endif()

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
# IMU-only diagnostic: no motor initialization or DDS traffic.
add_executable(imu_probe src/test/imu_probe.cpp)
target_link_libraries(imu_probe ${XSENS_OBJ_LIBS} pthread)
include(CTest)
if(BUILD_TESTING)
    add_executable(test_imu_delivery src/test/test_imu_delivery.cpp)
    target_link_libraries(test_imu_delivery ${XSENS_OBJ_LIBS} pthread)
    target_compile_options(test_imu_delivery PRIVATE -UNDEBUG)
    add_test(NAME imu_delivery COMMAND test_imu_delivery)
endif()

if(BUILD_TESTING)
    add_executable(test_command_watchdog src/test/test_command_watchdog.cpp)
    target_compile_options(test_command_watchdog PRIVATE -UNDEBUG)
    add_test(NAME command_watchdog COMMAND test_command_watchdog)
endif()

# Separate sensor-only publisher: deliberately no motor/CAN library dependency.
add_executable(imu_dds_probe src/test/imu_dds_probe.cpp)
target_link_libraries(imu_dds_probe ${XSENS_OBJ_LIBS} unitree_sdk2 pthread)

if(BUILD_TESTING)
    add_executable(test_feedback_only src/test/test_feedback_only.cpp)
    target_compile_options(test_feedback_only PRIVATE -UNDEBUG)
    add_test(NAME feedback_only COMMAND test_feedback_only)
endif()

if(BUILD_TESTING)
    add_executable(test_motor_feedback src/test/test_motor_feedback.cpp)
    target_link_libraries(test_motor_feedback motor pthread)
    target_compile_options(test_motor_feedback PRIVATE -UNDEBUG)
    add_test(NAME motor_feedback COMMAND test_motor_feedback)
endif()
