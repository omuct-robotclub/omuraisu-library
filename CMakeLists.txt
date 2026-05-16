cmake_minimum_required(VERSION 3.14)
project(omuraisu-library
    VERSION 1.0.0
    DESCRIPTION "omuraisu Library - Robot development C/C++ library"
    LANGUAGES C CXX
)

# C17を使用
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

# C++17を使用
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# コンパイルオプション
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ライブラリのインクルードディレクトリ
set(OMURAISU_INCLUDE_DIR_C ${CMAKE_CURRENT_SOURCE_DIR}/include/omuraisu/c)
set(OMURAISU_INCLUDE_DIR_CPP ${CMAKE_CURRENT_SOURCE_DIR}/include/omuraisu/cpp)

# coordinateライブラリ（実装ファイルあり）
add_library(omuraisu_coordinate
    src/coordinate/coordinate.c
)
target_include_directories(omuraisu_coordinate PUBLIC
    $<BUILD_INTERFACE:${OMURAISU_INCLUDE_DIR_C}>
    $<INSTALL_INTERFACE:include>
)

add_library(omuraisu_pid
    src/pid/pid.c
)
target_include_directories(omuraisu_pid PUBLIC
    $<BUILD_INTERFACE:${OMURAISU_INCLUDE_DIR_C}>
    $<INSTALL_INTERFACE:include>
)

add_library(omuraisu_chassis
    src/chassis/mecanum.c
)
target_include_directories(omuraisu_chassis PUBLIC
    $<BUILD_INTERFACE:${OMURAISU_INCLUDE_DIR_C}>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(omuraisu_chassis PUBLIC omuraisu_coordinate)

add_library(omuraisu_cobs
    src/cobs/cobs.c
)
target_include_directories(omuraisu_cobs PUBLIC
    $<BUILD_INTERFACE:${OMURAISU_INCLUDE_DIR_C}>
    $<INSTALL_INTERFACE:include>
)

add_library(omuraisu_can
    src/can/can_interface.c
    src/can/can_cube.c
)
target_include_directories(omuraisu_can PUBLIC
    $<BUILD_INTERFACE:${OMURAISU_INCLUDE_DIR_C}>
    $<INSTALL_INTERFACE:include>
)

option(OMURAISU_CAN_STM32_ENABLE "Enable STM32 Cube CAN adapter" OFF)
option(OMURAISU_CAN_STM32_FDCAN_ENABLE "Enable STM32 FDCAN support in STM32 CAN adapter" ON)
option(OMURAISU_BUILD_STM32_CAN "[Deprecated] Use OMURAISU_CAN_STM32_ENABLE" OFF)

if(OMURAISU_BUILD_STM32_CAN AND NOT OMURAISU_CAN_STM32_ENABLE)
    set(OMURAISU_CAN_STM32_ENABLE ON CACHE BOOL "Enable STM32 Cube CAN adapter" FORCE)
endif()

if(OMURAISU_CAN_STM32_ENABLE)
    target_sources(omuraisu_can PRIVATE
        src/can/can_stm32.c
    )
    target_compile_definitions(omuraisu_can PRIVATE OMURAISU_CAN_STM32_ENABLE)
    if(OMURAISU_CAN_STM32_FDCAN_ENABLE)
        target_compile_definitions(omuraisu_can PRIVATE OMURAISU_CAN_STM32_FDCAN_ENABLE)
    endif()
endif()

add_library(omuraisu_vesc
    src/vesc/vesc_core.c
)
target_include_directories(omuraisu_vesc PUBLIC
    $<BUILD_INTERFACE:${OMURAISU_INCLUDE_DIR_C}>
    $<INSTALL_INTERFACE:include>
)

add_library(omuraisu_dji
    src/dji/robomas_core.c
    src/dji/robomas.c
)
target_include_directories(omuraisu_dji PUBLIC
    $<BUILD_INTERFACE:${OMURAISU_INCLUDE_DIR_C}>
    $<INSTALL_INTERFACE:include>
)

add_library(omuraisu_controller
    src/controller/controller_core.c
    src/controller/controller_transport.c
)
target_include_directories(omuraisu_controller PUBLIC
    $<BUILD_INTERFACE:${OMURAISU_INCLUDE_DIR_C}>
    $<INSTALL_INTERFACE:include>
)

add_library(omuraisu_servo
    src/servo/servo_core.c
)
target_include_directories(omuraisu_servo PUBLIC
    $<BUILD_INTERFACE:${OMURAISU_INCLUDE_DIR_C}>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(omuraisu_servo PUBLIC omuraisu_can)

# C インターフェースライブラリ（全てをまとめる）
add_library(omuraisu_c INTERFACE)
target_link_libraries(omuraisu_c INTERFACE
    omuraisu_coordinate
    omuraisu_pid
    omuraisu_chassis
    omuraisu_cobs
    omuraisu_can
    omuraisu_dji
    omuraisu_controller
    omuraisu_servo
    omuraisu_vesc
)

# C++ ラッパ ライブラリ

add_library(omuraisu_cpp_can STATIC
    src/cpp/can/can_interface.cpp
    src/cpp/can/can_mbed.cpp
)
target_include_directories(omuraisu_cpp_can PUBLIC
    $<BUILD_INTERFACE:${OMURAISU_INCLUDE_DIR_CPP}>
    $<BUILD_INTERFACE:${OMURAISU_INCLUDE_DIR_C}>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(omuraisu_cpp_can PUBLIC
    omuraisu_can
)

add_library(omuraisu_cpp_chassis STATIC
    src/cpp/chassis/mecanum.cpp
)
target_include_directories(omuraisu_cpp_chassis PUBLIC
    $<BUILD_INTERFACE:${OMURAISU_INCLUDE_DIR_CPP}>
    $<BUILD_INTERFACE:${OMURAISU_INCLUDE_DIR_C}>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(omuraisu_cpp_chassis PUBLIC
    omuraisu_chassis
    omuraisu_cpp_coordinate
)

add_library(omuraisu_cpp_cobs STATIC
    src/cpp/cobs/cobs.cpp
)
target_include_directories(omuraisu_cpp_cobs PUBLIC
    $<BUILD_INTERFACE:${OMURAISU_INCLUDE_DIR_CPP}>
    $<BUILD_INTERFACE:${OMURAISU_INCLUDE_DIR_C}>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(omuraisu_cpp_cobs PUBLIC
    omuraisu_cobs
)

add_library(omuraisu_cpp_controller STATIC
    src/cpp/controller/controller_core.cpp
    src/cpp/controller/controller_transport.cpp
)
target_include_directories(omuraisu_cpp_controller PUBLIC
    $<BUILD_INTERFACE:${OMURAISU_INCLUDE_DIR_CPP}>
    $<BUILD_INTERFACE:${OMURAISU_INCLUDE_DIR_C}>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(omuraisu_cpp_controller PUBLIC
    omuraisu_controller
)

add_library(omuraisu_cpp_coordinate STATIC
    src/cpp/coordinate/coordinate.cpp
)
target_include_directories(omuraisu_cpp_coordinate PUBLIC
    $<BUILD_INTERFACE:${OMURAISU_INCLUDE_DIR_CPP}>
    $<BUILD_INTERFACE:${OMURAISU_INCLUDE_DIR_C}>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(omuraisu_cpp_coordinate PUBLIC
    omuraisu_coordinate
)

add_library(omuraisu_cpp_dji STATIC
    src/cpp/dji/robomas_core.cpp
    src/cpp/dji/robomas.cpp
)
target_include_directories(omuraisu_cpp_dji PUBLIC
    $<BUILD_INTERFACE:${OMURAISU_INCLUDE_DIR_CPP}>
    $<BUILD_INTERFACE:${OMURAISU_INCLUDE_DIR_C}>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(omuraisu_cpp_dji PUBLIC
    omuraisu_dji
    omuraisu_cpp_can
)

add_library(omuraisu_cpp_pid STATIC
    src/cpp/pid/pid.cpp
)
target_include_directories(omuraisu_cpp_pid PUBLIC
    $<BUILD_INTERFACE:${OMURAISU_INCLUDE_DIR_CPP}>
    $<BUILD_INTERFACE:${OMURAISU_INCLUDE_DIR_C}>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(omuraisu_cpp_pid PUBLIC
    omuraisu_pid
)

add_library(omuraisu_cpp_servo STATIC
    src/cpp/servo/servo_core.cpp
)
target_include_directories(omuraisu_cpp_servo PUBLIC
    $<BUILD_INTERFACE:${OMURAISU_INCLUDE_DIR_CPP}>
    $<BUILD_INTERFACE:${OMURAISU_INCLUDE_DIR_C}>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(omuraisu_cpp_servo PUBLIC
    omuraisu_servo
    omuraisu_cpp_can
)

# C++ インターフェースライブラリ（全てをまとめる）
add_library(omuraisu_cpp INTERFACE)
target_link_libraries(omuraisu_cpp INTERFACE
    omuraisu_cpp_can
    omuraisu_cpp_chassis
    omuraisu_cpp_cobs
    omuraisu_cpp_controller
    omuraisu_cpp_coordinate
    omuraisu_cpp_dji
    omuraisu_cpp_pid
    omuraisu_cpp_servo
)

# メインライブラリ（全てをまとめる）
add_library(omuraisu INTERFACE)
target_link_libraries(omuraisu INTERFACE
    omuraisu_c
    omuraisu_cpp
)

# サンプルのビルドオプション
option(BUILD_EXAMPLES "Build example programs" OFF)
if(BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# テストのビルドオプション
option(BUILD_TESTS "Build tests" OFF)
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# インストール設定
include(GNUInstallDirs)

install(TARGETS omuraisu_coordinate omuraisu_pid omuraisu_chassis omuraisu_cobs omuraisu_can omuraisu_vesc omuraisu_dji omuraisu_controller omuraisu_servo omuraisu_c omuraisu_cpp_can omuraisu_cpp_chassis omuraisu_cpp_cobs omuraisu_cpp_controller omuraisu_cpp_coordinate omuraisu_cpp_dji omuraisu_cpp_pid omuraisu_cpp omuraisu_cpp_servo omuraisu
    EXPORT omuraisu-targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

install(EXPORT omuraisu-targets
    FILE omuraisu-targets.cmake
    NAMESPACE omuraisu::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/omuraisu
)

# パッケージ設定ファイルの生成
include(CMakePackageConfigHelpers)

configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/omuraisu-config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/omuraisu-config.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/omuraisu
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/omuraisu-config-version.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/omuraisu-config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/omuraisu-config-version.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/omuraisu
)
