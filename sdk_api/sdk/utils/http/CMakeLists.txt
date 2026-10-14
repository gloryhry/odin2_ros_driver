cmake_minimum_required(VERSION 3.10)
project(http_transfer LANGUAGES C CXX)

include(GNUInstallDirs)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(Threads REQUIRED)
if(NOT OPENSSL_FOUND)
    find_package(OpenSSL REQUIRED)
endif()

# Mongoose is built as a separate OBJECT library (3rdparty/mongoose) and merged
# into odin_sdk; here we only need its headers on the include path.
set(3RDPARTY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../3rdparty)

# HTTP/HTTPS module sources (mongoose.c is compiled by the mongoose target).
set(HTTP_TRANSFER_SRCS
    ${CMAKE_CURRENT_SOURCE_DIR}/http_client.cpp
)

# Note: previous revisions also produced a duplicate `http_transfer` SHARED
# library built from the same sources without any downstream consumer; it has
# been removed to keep the build minimal. The OBJECT library below is merged
# into odin_sdk.so via $<TARGET_OBJECTS:http_transfer_obj> in sdk/CMakeLists.txt.
add_library(http_transfer_obj OBJECT ${HTTP_TRANSFER_SRCS})

target_compile_definitions(http_transfer_obj
    PRIVATE
        MG_TLS=MG_TLS_OPENSSL
        MG_ENABLE_LINES=1
        MG_ENABLE_DIRECTORY_LISTING=1
        MG_ENABLE_SSI=0
)
target_include_directories(http_transfer_obj
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/http_transfer>
    PRIVATE
        ${3RDPARTY_DIR}
)
set_target_properties(http_transfer_obj PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    POSITION_INDEPENDENT_CODE ON
)
