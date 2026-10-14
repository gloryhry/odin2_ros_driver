# Transport OBJECT library - merged into odin_sdk.
#
# Note: previous revisions also produced a duplicate `odin_transport` STATIC
# library built from the same sources without any downstream consumer; it has
# been removed to keep the build minimal.

set(TRANSPORT_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/Network/UdpTransport.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Network/TcpTransport.cpp
)

add_library(odin_transport_obj OBJECT ${TRANSPORT_SOURCES})

target_include_directories(odin_transport_obj
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/ITransport
        ${CMAKE_CURRENT_SOURCE_DIR}/Network
        ${CMAKE_CURRENT_SOURCE_DIR}/Usb
        ${CMAKE_CURRENT_SOURCE_DIR}/../../include/logger
)

set_target_properties(odin_transport_obj PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)
