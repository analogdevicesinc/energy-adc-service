cmake_minimum_required(VERSION 3.22)

# Project name and executable
set(CMAKE_PROJECT_NAME sample_read_example)

# Base directories (allow override from command line or parent project)
set(PROJECT_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../" CACHE PATH "Project root directory")
set(BOARD_SUPPORT_DIR "${PROJECT_ROOT_DIR}/board_support" CACHE PATH "Board support directory")
set(ADE_REGISTERS_DIR "${PROJECT_ROOT_DIR}/ade_registers" CACHE PATH "Directory for ADE registers"  )
set(ADCSIF_DIR "${PROJECT_ROOT_DIR}/interface" CACHE PATH "Directory for ADC service interface")
set(ADCS_LIB_DIR "${PROJECT_ROOT_DIR}/source" CACHE PATH "Directory for ADC service library")
set(CONFIG_DIR "${CMAKE_CURRENT_LIST_DIR}/config" CACHE PATH "Configuration directory")

# Build options
set(EVB "app_mcu_h5" CACHE STRING "Name of evaluation board")
set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type")


# ------------------------------------------------------------------------------
# Toolchain & Project Setup
# ------------------------------------------------------------------------------
include(${BOARD_SUPPORT_DIR}/cmake/toolchain_wrapper.cmake)
project(${CMAKE_PROJECT_NAME})

# ------------------------------------------------------------------------------
# Source Files
# ------------------------------------------------------------------------------
set(ADCSIF_SRC
    ${ADCSIF_DIR}/source/adc_service_interface.c
    ${ADCSIF_DIR}/source/adc_service_adapter.c
    ${ADCSIF_DIR}/source/adc_service_dsp_interface.c
)

set(APP_SRC
    ${PROJECT_ROOT_DIR}/examples/sample_read_example.c
    ${ADE_REGISTERS_DIR}/crc/source/ade_crc.c
    ${ADCSIF_SRC}
)

# ------------------------------------------------------------------------------
# Include Directories
# ------------------------------------------------------------------------------
set(APP_INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include
    ${ADE_REGISTERS_DIR}/adema12x/include
    ${ADE_REGISTERS_DIR}/ade91xx/include
    ${ADE_REGISTERS_DIR}/include
    ${ADE_REGISTERS_DIR}/crc/include
    ${PROJECT_ROOT_DIR}/include
    ${CONFIG_DIR}
    ${ADCSIF_DIR}/include
)

# ------------------------------------------------------------------------------
# Add Subdirectories
# ------------------------------------------------------------------------------
add_subdirectory(${BOARD_SUPPORT_DIR} ${CMAKE_BINARY_DIR}/board_support)
add_subdirectory(${ADCS_LIB_DIR} ${CMAKE_BINARY_DIR}/adc_service)
target_include_directories(adc_service PRIVATE ${CONFIG_DIR})



# ------------------------------------------------------------------------------
# Executable Target Setup
# ------------------------------------------------------------------------------
add_executable(${CMAKE_PROJECT_NAME})
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${APP_SRC})
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${APP_INCLUDES})
target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE adc_service board_support)

# ------------------------------------------------------------------------------
# Post-build: Generate HEX file
# ------------------------------------------------------------------------------
add_hex_generation(${CMAKE_PROJECT_NAME})

