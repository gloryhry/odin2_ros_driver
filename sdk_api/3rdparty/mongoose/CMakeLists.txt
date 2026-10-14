# Mongoose Web Server Library
# SPDX-License-Identifier: GPL-2.0-only

cmake_minimum_required(VERSION 3.10)
project(mongoose LANGUAGES C)

include(GNUInstallDirs)

# OBJECT library: only compiles mongoose.c into .o files (no .a/.so archive).
# The objects are merged into libodin_sdk.so via $<TARGET_OBJECTS:mongoose> in
# sdk/CMakeLists.txt, so mongoose is not built/shipped as a standalone library.
# PIC is required because the objects end up inside a SHARED library.
add_library(mongoose OBJECT mongoose.c)
set_target_properties(mongoose PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Include directories (use generator expressions for build/install paths)
target_include_directories(mongoose
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/mongoose>
)

# Compile definitions
target_compile_definitions(mongoose
    PUBLIC
        MG_ENABLE_LINES=1
        MG_ENABLE_DIRECTORY_LISTING=1
        MG_ENABLE_SSI=0
)

# Thread support
find_package(Threads REQUIRED)
target_link_libraries(mongoose PUBLIC Threads::Threads)

# Optional: Enable TLS
option(MG_ENABLE_OPENSSL "Enable OpenSSL TLS support" ON)
if(MG_ENABLE_OPENSSL)
    # Only find OpenSSL if a parent project hasn't already done it.  Use the
    # legacy variables for linking because the imported OpenSSL::SSL target
    # is scoped by directory and can fail to resolve through OBJECT libraries
    # on CMake 3.16 (Ubuntu 20.04).
    if(NOT OPENSSL_FOUND)
        find_package(OpenSSL REQUIRED)
    endif()
    target_compile_definitions(mongoose PUBLIC MG_TLS=MG_TLS_OPENSSL)
    target_include_directories(mongoose PUBLIC ${OPENSSL_INCLUDE_DIR})
    target_link_libraries(mongoose PUBLIC ${OPENSSL_LIBRARIES})
endif()

# No install rules: mongoose is an internal OBJECT library embedded into
# libodin_sdk.so; it is neither a standalone library nor a public header.
