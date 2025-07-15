# ADC Service

## Introduction

ADC Service provides a set of APIs to communicate with ADI metrology ADCs and collect waveform samples. This release supports ADE9113, ADE9112, ADE9103, ADEMA124, and ADEMA127. These ADCs use a command and response protocol over the SPI interface. Refer to the datasheets of the respective ADCs and the [energy-ade-registers](https://github.com/analogdevicesinc/energy-ade-registers) repository for details on the command and response protocols.

This repository contains source code for the service and example code demonstrating how to use the APIs. The examples use the following submodules for demonstrations:
- [energy-ade-registers](https://github.com/analogdevicesinc/energy-ade-registers)
- [energy-board-support](https://github.com/analogdevicesinc/energy-board-support)
- [energy-firmware-services](https://github.com/analogdevicesinc/energy-firmware-services)

To clone the repository and its submodules, use the following command:

```sh
git clone --recursive https://github.com/analogdevicesinc/energy-adc-service.git
```

The [sample_read_example](examples) demonstrates how to call various APIs and interface functions of the ADC Service. The repository also contains sources for [ACE evaluation firmware](eval_firmware).


## Supported Features

- Configure and read samples from supported ADI ADCs
- Daisy chain multiple ADCs
- Read and write ADC registers without interrupting sample collection
- Synchronize multiple ADCs in a daisy chain
- Configure DSP registers
- Add sample delays to individual channels
- Example code for tamper detection mode



## Directory Structure

```text
energy-adc-service/
├── include/             # ADC service API header files and other include files
├── source/              # ADC service source files
├── interface/           # Example files showing how to call various service APIs
├── examples/            # ADC service example firmware files
├── eval_firmware/       # ADC service evaluation firmware files
├── ade_registers/       # Submodule for IC register definitions
├── board_support/       # Submodule for board drivers and support files
├── docs/                # Release notes and API documentation
├── firmware_services/   # Submodule for CLI and other firmware services
```


Refer to [integration_instructions.md](integration_instructions.md) for instructions on integrating the ADC Service into a new project.

## API Documentation

The API documentation can be generated using Doxygen. Install the following tools and add them to your system path:

- [Doxygen 1.9.3](https://www.doxygen.nl/download.html)
- [Graphviz](https://www2.graphviz.org/Archive/stable/windows/)

Use the following command from the documentation folder to generate the documentation:

```sh
doxygen adc_service_doxy_config
```

The documentation will be generated in the output directory specified in the Doxygen configuration.

## License

This project is licensed under the [Apache 2.0 License](LICENSE).

## Contact

For questions or support, please open an issue on the [GitHub repository](https://github.com/analogdevicesinc/energy-adc-service/issues).
