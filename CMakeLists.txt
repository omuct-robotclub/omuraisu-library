cmake_minimum_required(VERSION 3.14)
project(omuraisu-library
    VERSION 1.0.0
    DESCRIPTION "omuraisu Library - Robot development C++ library"
    LANGUAGES CXX
)

# C++17を使用
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# コンパイルオプション
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ライブラリのインクルードディレクトリ
set(OMURAISU_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

# coordinateライブラリ（実装ファイルあり）
add_library(coordinate
    src/coordinate/coordinate.cpp
)
target_include_directories(coordinate PUBLIC
    $<BUILD_INTERFACE:${OMURAISU_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
)

# ヘッダーオンリーライブラリ用のインターフェースライブラリ
add_library(omuraisu_pid INTERFACE)
target_include_directories(omuraisu_pid INTERFACE
    $<BUILD_INTERFACE:${OMURAISU_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
)

add_library(omuraisu_chassis INTERFACE)
target_include_directories(omuraisu_chassis INTERFACE
    $<BUILD_INTERFACE:${OMURAISU_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(omuraisu_chassis INTERFACE coordinate)

# DJI/Robomasライブラリはmbed依存のため、条件付きで追加
# option(ENABLE_DJI "Enable DJI/Robomas library (requires mbed)" OFF)
# if(ENABLE_DJI)
#     add_library(omuraisu_dji INTERFACE)
#     target_include_directories(omuraisu_dji INTERFACE
#         $<BUILD_INTERFACE:${OMURAISU_INCLUDE_DIR}>
#         $<INSTALL_INTERFACE:include>
#     )
# endif()

# メインライブラリ（全てをまとめる）
add_library(omuraisu INTERFACE)
target_link_libraries(omuraisu INTERFACE
    coordinate
    omuraisu_pid
    omuraisu_chassis
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

install(TARGETS coordinate omuraisu_pid omuraisu_chassis omuraisu
    EXPORT omuraisu-targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY src/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/omuraisu
    FILES_MATCHING PATTERN "*.hpp"
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
