cmake_minimum_required(VERSION 3.10)

# =============================================================================
# SDK version & release date. This is the single source of truth (only
# libodin_sdk uses it, baked into odin_sdk.cpp). Edit the two lines below to
# bump them. CI/CD can override without editing the file:
#   -DODIN_SDK_VERSION=2.0.3 -DODIN_SDK_BUILD_DATE=20260720
# Fixed values (no build-time timestamp) keep the version reproducible.
# =============================================================================
set(ODIN_SDK_VERSION    "3.3.0"    CACHE STRING "SDK semantic version (X.Y.Z)")
set(ODIN_SDK_BUILD_DATE "20260822" CACHE STRING "SDK release date (YYYYMMDD)")

project(odin_sdk VERSION ${ODIN_SDK_VERSION} LANGUAGES C CXX)

set(ODIN_SDK_VERSION_MAJOR "${PROJECT_VERSION_MAJOR}")
set(ODIN_SDK_VERSION_MINOR "${PROJECT_VERSION_MINOR}")
set(ODIN_SDK_VERSION_PATCH "${PROJECT_VERSION_PATCH}")

set(ODIN_SDK_VERSION_STRING "${ODIN_SDK_VERSION}_${ODIN_SDK_BUILD_DATE}")
message(STATUS "odin_sdk: version = ${ODIN_SDK_VERSION_STRING}")

include(GNUInstallDirs)

# Default to Release build with -O3 optimization
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
set(CMAKE_C_FLAGS_RELEASE "-O3 -DNDEBUG")

# Optional Link-Time Optimization for Release builds.
option(ODIN_SDK_ENABLE_LTO "Enable Link-Time Optimization for Release builds" OFF)
if(ODIN_SDK_ENABLE_LTO AND CMAKE_BUILD_TYPE STREQUAL "Release")
    include(CheckIPOSupported)
    check_ipo_supported(RESULT ODIN_SDK_LTO_OK OUTPUT ODIN_SDK_LTO_ERR)
    if(ODIN_SDK_LTO_OK)
        set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
        message(STATUS "odin_sdk: LTO enabled")
    else()
        message(STATUS "odin_sdk: LTO not supported (${ODIN_SDK_LTO_ERR})")
    endif()
endif()

# Enable comprehensive warnings
add_compile_options(
  -Wall
  -Wextra
  -Wno-unused-parameter
  -Wno-sign-compare
  -Wno-format-truncation
  -Wno-variadic-macros
  -Wno-unused-function
)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Required for OBJECT libraries to be merged into a SHARED library
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Detect target architecture if invoked directly (e.g. via samples) without the
# parent sdk_api/CMakeLists.txt having defined ODIN_TARGET_ARCH.
if(NOT DEFINED ODIN_TARGET_ARCH)
    if(CMAKE_SYSTEM_PROCESSOR)
        set(ODIN_TARGET_ARCH "${CMAKE_SYSTEM_PROCESSOR}")
    else()
        execute_process(COMMAND uname -m
            OUTPUT_VARIABLE ODIN_TARGET_ARCH
            OUTPUT_STRIP_TRAILING_WHITESPACE)
    endif()
endif()
if(NOT DEFINED ODIN_SDK_LIB_OUTPUT_DIR)
    set(ODIN_SDK_LIB_OUTPUT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../lib/${ODIN_TARGET_ARCH}")
    file(MAKE_DIRECTORY "${ODIN_SDK_LIB_OUTPUT_DIR}")
endif()
message(STATUS "odin_sdk: arch=${ODIN_TARGET_ARCH} output=${ODIN_SDK_LIB_OUTPUT_DIR}")

# Find dependencies before any add_library/add_subdirectory so that
# they are in scope for odin_sdk and its merged OBJECT libraries.
# Use OpenSSL variables instead of OpenSSL::SSL targets because CMake
# 3.16 (Ubuntu 20.04) has scoped IMPORTED target issues with OBJECT
# libraries.
find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)

# A CMakeCache.txt reused from another machine or another architecture keeps
# OPENSSL_*_LIBRARY / OPENSSL_INCLUDE_DIR pointing at paths that do not exist
# here. find_package(OpenSSL REQUIRED) then still succeeds - the standard
# argument check only verifies that the variables are non-empty - but
# FindOpenSSL skips creating OpenSSL::SSL / OpenSSL::Crypto because it guards
# them with EXISTS. Consumers then fail at generate time with
# "links to target OpenSSL::SSL but the target was not found", and linking
# against ${OPENSSL_LIBRARIES} would point at the missing files instead.
# Drop such stale entries so the search really runs in this environment.
foreach(_openssl_cache_var OPENSSL_SSL_LIBRARY OPENSSL_CRYPTO_LIBRARY)
    if(${_openssl_cache_var} AND NOT EXISTS "${${_openssl_cache_var}}")
        message(STATUS "odin_sdk: dropping stale ${_openssl_cache_var}=${${_openssl_cache_var}}")
        unset(${_openssl_cache_var} CACHE)
    endif()
endforeach()
if(OPENSSL_INCLUDE_DIR AND NOT EXISTS "${OPENSSL_INCLUDE_DIR}/openssl/ssl.h")
    message(STATUS "odin_sdk: dropping stale OPENSSL_INCLUDE_DIR=${OPENSSL_INCLUDE_DIR}")
    unset(OPENSSL_INCLUDE_DIR CACHE)
endif()

find_package(OpenSSL REQUIRED)
foreach(_openssl_lib ${OPENSSL_LIBRARIES})
    if(IS_ABSOLUTE "${_openssl_lib}" AND NOT EXISTS "${_openssl_lib}")
        message(FATAL_ERROR
            "odin_sdk: OpenSSL resolved to a missing file: ${_openssl_lib}\n"
            "Configure a clean build directory (the cache belongs to another "
            "host/architecture) and make sure libssl-dev is installed for "
            "${ODIN_TARGET_ARCH}.")
    endif()
endforeach()

# Build protocol OBJECT library (will be merged into odin_sdk)
add_subdirectory(protocol)

# Build transport OBJECT library (will be merged into odin_sdk)
add_subdirectory(transport)

# Build device OBJECT library (will be merged into odin_sdk)
add_subdirectory(device)

# Build https OBJECT library (will be merged into odin_sdk)
# TODO: Uncomment when mongoose is available in 3rdparty/mongoose/
# add_subdirectory(secure_transport)

# Build http_transfer library (HTTP/HTTPS client). Used internally by
# OdinCommand's HTTP backends (network mgmt + file transfer).
add_subdirectory(utils/http)

# Build mongoose OBJECT library (will be merged into odin_sdk). Added here
# rather than in the parent so sdk/ can also be configured standalone; a
# dedicated binary dir is required since the source lives outside this tree.
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/mongoose
                 ${CMAKE_CURRENT_BINARY_DIR}/mongoose)

# SDK sources
file(GLOB ODIN_SDK_SOURCES
    logger/logger.cpp
    src/odin_sdk.cpp
    src/OdinSdkImpl.cpp
    src/SlamOdomSynchronizer.cpp
    src/odin_command_channel.cpp
    hotplugListener/UdpHotplugListener/UdpHotplugListener.cpp
    hotplugListener/CompositeHotplugListener/CompositeHotplugListener.cpp
    hotplugListener/HotplugListenerFactory.cpp
    discovery/OdinUdpDiscovery/OdinUdpDiscovery.cpp
    utils/network_utils.cpp
    utils/payload_builder/payload_builder.cpp
    ../3rdparty/tinyxml2/tinyxml2.cpp
    ../3rdparty/cJson/cJSON.c)

# Merge all OBJECT libraries into odin_sdk SHARED library - external consumers
# only need to link libodin_sdk.so (+ OpenSSL).
add_library(odin_sdk SHARED ${ODIN_SDK_SOURCES}
    $<TARGET_OBJECTS:odin_protocol_obj>
    $<TARGET_OBJECTS:odin_transport_obj>
    $<TARGET_OBJECTS:odin_device_obj>
    $<TARGET_OBJECTS:http_transfer_obj>
    $<TARGET_OBJECTS:mongoose>)

set_target_properties(odin_sdk PROPERTIES
    VERSION ${ODIN_SDK_VERSION}
    SOVERSION ${ODIN_SDK_VERSION_MAJOR}
    POSITION_INDEPENDENT_CODE ON
    LIBRARY_OUTPUT_DIRECTORY "${ODIN_SDK_LIB_OUTPUT_DIR}"
    RUNTIME_OUTPUT_DIRECTORY "${ODIN_SDK_LIB_OUTPUT_DIR}")

# Inject the version string as a compile definition consumed by GetSdkVersion().
# The value (e.g. "2.0.2_20260611") contains no spaces or quotes, so the quoted
# form passes through the Makefile generator without escaping issues.
target_compile_definitions(odin_sdk PRIVATE
    ODIN_SDK_VERSION_STRING="${ODIN_SDK_VERSION_STRING}")

# Hide inline / template-instantiation symbols (the source of STL weak-symbol
# leakage such as std::__future_base::*) while keeping every non-inline
# function exported by default. Without this the SDK ships its own copies of
# inline STL symbols, and a consumer like rclcpp may bind its STL calls to
# the SDK-local copies (because libodin_sdk.so appears before libstdc++.so
# in the NEEDED order). The two copies are not necessarily byte-identical
# (different optimization level, LTO, ODR edge cases) and can crash with a
# NULL function-pointer jump.
target_compile_options(odin_sdk PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:-fvisibility-inlines-hidden>)
foreach(_obj odin_protocol_obj odin_transport_obj odin_device_obj http_transfer_obj)
    if(TARGET ${_obj})
        target_compile_options(${_obj} PRIVATE
            $<$<COMPILE_LANGUAGE:CXX>:-fvisibility-inlines-hidden>)
    endif()
endforeach()

# Restrict the .so dynamic export table to the SDK public API surface using a
# linker version script (positive whitelist). This is what actually prevents
# the STL weak symbols (std::__future_base::*, std::vector<>::_M_*, etc.) from
# leaking out of libodin_sdk.so. -fvisibility-inlines-hidden alone is not
# enough because vtables and template-instantiated member functions are
# vague-linkage data, not "inlines".
set(ODIN_SDK_VERSION_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/odin_sdk.version")
if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND EXISTS "${ODIN_SDK_VERSION_SCRIPT}")
    target_link_options(odin_sdk PRIVATE
        "LINKER:--version-script=${ODIN_SDK_VERSION_SCRIPT}")
    set_property(TARGET odin_sdk APPEND PROPERTY LINK_DEPENDS
        "${ODIN_SDK_VERSION_SCRIPT}")
endif()

# Strip the shared library in Release builds to reduce its size while keeping
# the dynamic symbol table required for runtime linking.
option(ODIN_SDK_STRIP_RELEASE "Strip release shared libraries to reduce size" ON)
if(ODIN_SDK_STRIP_RELEASE AND CMAKE_BUILD_TYPE STREQUAL "Release" AND CMAKE_STRIP)
    add_custom_command(TARGET odin_sdk POST_BUILD
        COMMAND ${CMAKE_STRIP} --strip-unneeded $<TARGET_FILE:odin_sdk>
        COMMENT "Stripping $<TARGET_FILE_NAME:odin_sdk>")
endif()

# Public headers live under include/
target_include_directories(odin_sdk
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../include/logger
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/device
        ${CMAKE_CURRENT_SOURCE_DIR}/device/IDevice
        ${CMAKE_CURRENT_SOURCE_DIR}/device/Odin2
        ${CMAKE_CURRENT_SOURCE_DIR}/command
        ${CMAKE_CURRENT_SOURCE_DIR}/command/odin2Command
        ${CMAKE_CURRENT_SOURCE_DIR}/capture
        ${CMAKE_CURRENT_SOURCE_DIR}/capture/odinCapture
        ${CMAKE_CURRENT_SOURCE_DIR}/protocol/IProtocol
        ${CMAKE_CURRENT_SOURCE_DIR}/protocol/OdinProtocol
        ${CMAKE_CURRENT_SOURCE_DIR}/protocol/odinProtocol
        ${CMAKE_CURRENT_SOURCE_DIR}/transport/ITransport
        ${CMAKE_CURRENT_SOURCE_DIR}/transport/Network
        ${CMAKE_CURRENT_SOURCE_DIR}/hotplugListener
        ${CMAKE_CURRENT_SOURCE_DIR}/hotplugListener/IHotplugListener
        ${CMAKE_CURRENT_SOURCE_DIR}/hotplugListener/UdpHotplugListener
        ${CMAKE_CURRENT_SOURCE_DIR}/utils
        ${CMAKE_CURRENT_SOURCE_DIR}/utils/http
        ${CMAKE_CURRENT_SOURCE_DIR}/utils/payload_builder
        ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty
        ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/tinyxml2
        ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/cJson)

target_include_directories(odin_sdk
    PRIVATE
        ${EIGEN3_INCLUDE_DIRS}
        ${OPENSSL_INCLUDE_DIR})
# Link OpenSSL publicly so external consumers only need to link odin_sdk.
# Use the ${OPENSSL_LIBRARIES} variable rather than the OpenSSL::SSL targets
# because the imported targets are scoped by directory and CMake 3.16 cannot
# always resolve them through OBJECT libraries.
target_link_libraries(odin_sdk
    PUBLIC ${OPENSSL_LIBRARIES}
    PRIVATE Threads::Threads Eigen3::Eigen)

# Install odin_sdk shared library to per-arch lib folder
install(TARGETS odin_sdk
    EXPORT odin_sdkTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/${ODIN_TARGET_ARCH}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

# Public headers (including logger/logger.h) all live under include/.
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(EXPORT odin_sdkTargets
    FILE odin_sdkTargets.cmake
    NAMESPACE odin::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/odin_sdk)
    