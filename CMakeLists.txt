cmake_minimum_required(VERSION 3.20)

project(SimpleClipper VERSION 0.0.1 LANGUAGES CXX)

# Fetch dependencies
include(ExternalProject)
ExternalProject_Add(
    wykobi
    GIT_REPOSITORY https://github.com/ArashPartow/wykobi.git
    GIT_TAG 37d0cd3cd6e3e3d18115c23c994e827a75052248
    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/wykobi
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND ""
)
set(wykobi_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/wykobi)

add_subdirectory(src)
