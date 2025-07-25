cmake_minimum_required(VERSION 3.22)

# Define an option for the build type
option(ZEPHYR_BUILD "Build app with Zephyr" OFF)

project(adc_service C)

set(ADC_SERVICE_DIR ${CMAKE_CURRENT_LIST_DIR}/..)

set(FW_SERVICES_DIR ${ADC_SERVICE_DIR}/firmware_services/services)

if (NOT DEFINED ADE_REG_DIR)
  set(ADE_REG_DIR ${ADC_SERVICE_DIR}/ade_registers)
endif()

set(SRC # ADC Service sources
        ${ADC_SERVICE_DIR}/source/adc_config_ade91xx.c
        ${ADC_SERVICE_DIR}/source/adc_config_adema12x.c
        ${ADC_SERVICE_DIR}/source/adc_configure.c
        ${ADC_SERVICE_DIR}/source/adc_datapath_configure.c
        ${ADC_SERVICE_DIR}/source/adi_adc_frame_assemble.c
        ${ADC_SERVICE_DIR}/source/adc_sync.c
        ${ADC_SERVICE_DIR}/source/adi_adc.c
        ${ADC_SERVICE_DIR}/source/adc_read_write.c
        ${ADC_SERVICE_DIR}/source/adc_private.c
        ${ADC_SERVICE_DIR}/source/adi_adc_dsp.c
        ${ADC_SERVICE_DIR}/source/adi_adc_utility.c
)

set(INCLUDE # ADC application includes
    ${ADC_SERVICE_DIR}/include
    ${ADE_REG_DIR}/adema12x/include
    ${ADE_REG_DIR}/include
    ${ADE_REG_DIR}/ade91xx/include
  )

set_source_files_properties(
  ${PROJECT_ROOT_DIR}/source/adi_adc.c
  ${PROJECT_ROOT_DIR}/source/adc_read_write.c
  ${PROJECT_ROOT_DIR}/source/adc_private.c
  PROPERTIES
)

# Set the project name
project(adc_service C)
# Baremetal build
# Add the executable
add_library(adc_service ${SRC})
#Provide include path of app_cfg to mathop      
target_include_directories(adc_service PUBLIC ${INCLUDE})

if (ZEPHYR_BUILD)
#This is required to pass correct build flags for library
target_link_libraries(adc_service PUBLIC zephyr_interface)
endif()

