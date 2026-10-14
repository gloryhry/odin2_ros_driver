# Protocol OBJECT library - merged into odin_sdk.
#
# Note: previous revisions also produced a duplicate `odin_protocol` STATIC
# library built from the same sources, which doubled compile time without any
# downstream consumer. Removed to keep the build minimal.

set(PROTOCOL_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/OdinProtocol/OdinProtocol.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../3rdparty/FastCRC/FastCRChw.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../3rdparty/FastCRC/FastCRCsw.cpp
)

add_library(odin_protocol_obj OBJECT ${PROTOCOL_SOURCES})

target_include_directories(odin_protocol_obj
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/IProtocol
        ${CMAKE_CURRENT_SOURCE_DIR}/OdinProtocol
        ${CMAKE_CURRENT_SOURCE_DIR}/../../3rdparty
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../../include/logger
)

set_target_properties(odin_protocol_obj PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)
