# Get the latest git tag as the firmware version
execute_process(
    COMMAND git describe --tags
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE FIRMWARE_VERSION
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

message(STATUS "Firmware Revision: ${FIRMWARE_VERSION}")

# Add the firmware version as a compile definition
add_compile_definitions(FIRMWARE_VERSION="${FIRMWARE_VERSION}")
