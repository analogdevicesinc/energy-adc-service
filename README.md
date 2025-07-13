# ADC Service

ADC Service provides a set of APIs to communicate with ADI metrology ADCs and collect waveform samples.
This release supports ADE9113, ADE9112, ADE9103, ADEMA124, and ADEMA127. These ADCs use a command and response protocol over the SPI interface. Refer to the datasheets of the respective ADCs and the [energy-ade-registers](https://github.com/analogdevicesinc/energy-ade-registers) repository for details on the command and response protocols.

These ADCs also support daisy chaining of multiple devices. The ADC Service supports sending commands to each ADC and collecting samples from each device as needed.

This repository contains source code for the service and example code demonstrating how to use the APIs. The examples use the [energy-ade-registers](https://github.com/analogdevicesinc/energy-ade-registers), [energy-board-support](https://github.com/analogdevicesinc/energy-board-support), and [energy-firmware-services](https://github.com/analogdevicesinc/energy-firmware-services) repositories for demonstrations. To clone the repository and its submodules, use the following command:

```sh
git clone --recursive https://github.com/analogdevicesinc/energy-adc-service.git
```

The [sample_read_example](examples) demonstrates how to call various APIs and interface functions of the ADC Service. The repository also contains sources for [ACE evaluation firmware](eval_firmware).

## Repository Structure

- **include**: ADC service API header files and other include files
- **source**: ADC service source files
- **interface**: Example files showing how to call various service APIs
- **examples**: ADC service example firmware files
- **eval_firmware**: ADC service evaluation firmware files
- **ade_registers**: Submodule for IC register definitions
- **board_support**: Submodule for board drivers and support files
- **docs**: Release notes and API documentation
- **firmware_services**: Submodule for CLI support

Refer to [integration_instructions.md](integration_instructions.md) for instructions on integrating the ADC Service into a new project.
