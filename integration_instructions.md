
# Integrating ADC Service into Your Application

## Overview

The ADC Service provides APIs for interfacing with ADI metrology ADCs, enabling data collection and configuration. All source files for the ADC Service are located in the [`source`](source) directory, and header files are in the [`include`](include) directory. Register addresses and definitions required for communication are available in the appropriate product folders within the [energy-ade-registers](https://github.com/analogdevicesinc/energy-ade-registers) repository.

To use the ADC Service in your application, add the relevant source and include directories to your project. If you use CMake, you can add the [`source`](source) directory as a subdirectory using the `add_subdirectory` command. See the [`example CMakeLists.txt`](examples/projects/CMakeLists.txt) for a reference implementation.



## Quick Start Guide

Follow these steps to set up data collection from an ADC using the ADC Service:

1. **Clone the repository and submodules:**

   ```sh
   git clone --recursive https://github.com/analogdevicesinc/energy-adc-service.git
   ```

2. **Create your application folder:**
   - Copy the [examples](examples) folder to a new location for your application.

3. **Update your CMake configuration:**
   - Edit [CMakeLists.txt](examples/projects/CMakeLists.txt) to include your application sources.
   - The example CMakeLists already includes ADC Service, ADE Registers, and interface sources.
   - Update the configuration headers in [examples/projects/config](examples/projects/config) with settings for your application board.
   - Update the SPI driver and CRC routines in [adc_service_adapter.c](interface/source/adc_service_adapter.c) to point to your device's drivers.

The following sections provide more details on how to optimize and customize the ADC Service for different applications.


## SPI Configuration

ADI metrology ADCs communicate with the host MCU using a command-response protocol over SPI (mode 3). Connect the appropriate ADC pins to the host SPI interface. ADCs also provide a DREADY interrupt signal whenever a new sample is ready. It is recommended to connect this interrupt to the MCU to enable timely sample collection. Refer to the ADC datasheet for more information on SPI protocols and interrupt handling.

Your application should initialize SPI and GPIOs using the drivers and HAL APIs provided by your MCU vendor.


## Interfacing with MCU Drivers

The ADC Service is designed to be processor-agnostic. SPI access APIs are abstracted as function pointers within a configuration structure, allowing you to connect the service to your MCU's drivers. Populate this structure with the appropriate driver APIs for your platform.

See [adc_service_adapter.c](interface/source/adc_service_adapter.c) for an example implementation for an evaluation board. Typically, you can copy this file and the corresponding [header file](interface/include/adc_service_adapter.h), then modify them for your specific application.


## CRC Configuration

ADEMA12x and ADE9113 ADCs use CRC to verify communication data integrity. The ADC Service expects your application to provide routines for CRC calculation. You may reuse the CRC service code provided in this release. See the [adapter file](interface/source/adc_service_adapter.c) for an example.


## ADC Service APIs

The ADC Service API assembles and sends SPI commands to the ADC, parses responses, and stores data in a circular buffer. It verifies the CRC of each response and flags errors on mismatches. The service also provides helper APIs for configuring data path features such as gain, offsets, and filters.

Refer to the ADC Service Doxygen-generated documentation for details on all APIs. To generate the documentation, run the following command from the [docs](./docs) folder:

```sh
doxygen adc_service_doxy_config
```

This will generate HTML documentation under the [docs/html](docs/html) folder. The file [adc_service_api_documentation.html](docs/adc_service_api_documentation.html) serves as the entry point for the documentation.



## Multiple Instantiation

You can create multiple instances of the ADC Service by declaring handles of type `ADI_ADC_HANDLE` and calling the `adi_adc_Create` API for each handle. This allows you to manage multiple ADC devices independently within your application.


## Integration with Various Applications

To integrate the ADC Service into your application:

- Set `ADI_ADC_CONFIG.numAdc` to the number of ADCs you wish to use (typically 1 for most applications).
- Set `ADI_ADC_CONFIG.adcType` to the appropriate value from the `ADI_ADC_TYPE` enumeration.
- Alternatively, you can call `AdcIfInitService` with `numAdc` and `pAdcType` as described above for each application instance.

Customize configuration headers, SPI drivers, and CRC routines as needed for your hardware and use case.


