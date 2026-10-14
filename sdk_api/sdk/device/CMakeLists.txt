# Device abstraction layer library.

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(DEVICE_SOURCES
    Odin2/odin2.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../command/odin2Command/OdinCommand.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../command/odin2Command/internal/http_config_backend.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../command/odin2Command/internal/http_file_backend.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../capture/odinCapture/OdinCaptureBase.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../protocol/odinProtocol/OdinFrameAssembler.cpp
)

set(DEVICE_HEADERS
    IDevice/IDevice.h
    Odin2/odin2.h
)

# Build as OBJECT library for merging into odin_sdk
add_library(odin_device_obj OBJECT ${DEVICE_SOURCES})

find_package(Eigen3 REQUIRED)

target_include_directories(odin_device_obj
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/IDevice
        ${CMAKE_CURRENT_SOURCE_DIR}/Odin2
        ${CMAKE_CURRENT_SOURCE_DIR}/../src
        ${CMAKE_CURRENT_SOURCE_DIR}/../../include
        ${EIGEN3_INCLUDE_DIRS}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../command
        ${CMAKE_CURRENT_SOURCE_DIR}/../command/odin2Command
        ${CMAKE_CURRENT_SOURCE_DIR}/../capture
        ${CMAKE_CURRENT_SOURCE_DIR}/../capture/odinCapture
        ${CMAKE_CURRENT_SOURCE_DIR}/../protocol/IProtocol
        ${CMAKE_CURRENT_SOURCE_DIR}/../protocol/OdinProtocol
        ${CMAKE_CURRENT_SOURCE_DIR}/../protocol/odinProtocol
        ${CMAKE_CURRENT_SOURCE_DIR}/../transport/ITransport
        ${CMAKE_CURRENT_SOURCE_DIR}/../transport/Network
        ${CMAKE_CURRENT_SOURCE_DIR}/../../include/logger
        ${CMAKE_CURRENT_SOURCE_DIR}/../command/odin2Command/internal
        ${CMAKE_CURRENT_SOURCE_DIR}/../utils/http
        ${CMAKE_CURRENT_SOURCE_DIR}/../utils/payload_builder
        ${CMAKE_CURRENT_SOURCE_DIR}/../../3rdparty
        ${CMAKE_CURRENT_SOURCE_DIR}/../../3rdparty/cJson
)

target_link_libraries(odin_device_obj PUBLIC Eigen3::Eigen)
