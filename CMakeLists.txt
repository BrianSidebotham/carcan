cmake_minimum_required(VERSION 3.12)

# Pull in PICO SDK (must be before project)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(carcan C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKD_BUILD_TYPE Debug)

# Initialize the SDK
pico_sdk_init()

add_executable(${PROJECT_NAME}
    ${PROJECT_SOURCE_DIR}/src/circular_buffer.c
    ${PROJECT_SOURCE_DIR}/src/circular_buffer.h
    ${PROJECT_SOURCE_DIR}/src/main.c
    ${PROJECT_SOURCE_DIR}/src/mem_allocator_malloc.c
    ${PROJECT_SOURCE_DIR}/src/mem_allocator.h
    ${PROJECT_SOURCE_DIR}/src/spi.c
    ${PROJECT_SOURCE_DIR}/src/spi.h
    ${PROJECT_SOURCE_DIR}/src/mcp2515.c
    ${PROJECT_SOURCE_DIR}/src/mcp2515.h
)

target_compile_options(${PROJECT_NAME} PRIVATE -Wall)

include_directories()
pico_add_extra_outputs(${PROJECT_NAME})

# Pull in our pico_stdlib which pulls in commonly used features
target_link_libraries(${PROJECT_NAME}
    pico_stdlib
    hardware_spi
)
