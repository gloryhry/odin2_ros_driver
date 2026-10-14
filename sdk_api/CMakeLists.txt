cmake_minimum_required(VERSION 3.10)

# The SDK version & release date are defined in sdk/CMakeLists.txt, since only
# libodin_sdk consumes them (baked into odin_sdk.cpp). This keeps sdk/ buildable
# standalone without depending on this parent file.
project(odin_lidar LANGUAGES CXX)

option(ODIN_BUILD_SAMPLES "Build sample executables" ON)

# Detect target architecture (matches `uname -m`: x86_64, aarch64, ...)
if(NOT DEFINED ODIN_TARGET_ARCH)
    if(CMAKE_SYSTEM_PROCESSOR)
        set(ODIN_TARGET_ARCH "${CMAKE_SYSTEM_PROCESSOR}")
    else()
        execute_process(COMMAND uname -m
            OUTPUT_VARIABLE ODIN_TARGET_ARCH
            OUTPUT_STRIP_TRAILING_WHITESPACE)
    endif()
endif()
message(STATUS "odin_lidar: target arch = ${ODIN_TARGET_ARCH}")

# Per-arch shared library output directory: <sdk_api>/lib/<arch>/
set(ODIN_SDK_LIB_OUTPUT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/${ODIN_TARGET_ARCH}"
    CACHE PATH "odin_sdk per-arch shared library output directory")
file(MAKE_DIRECTORY "${ODIN_SDK_LIB_OUTPUT_DIR}")

# mongoose is added from sdk/CMakeLists.txt (merged into odin_sdk as an OBJECT
# library), so it is intentionally not added here.
add_subdirectory(sdk)
if(ODIN_BUILD_SAMPLES)
    add_subdirectory(sample)
endif()
