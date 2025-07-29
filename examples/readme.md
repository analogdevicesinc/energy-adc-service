# Sample read example

The example provided demonstrates how ADC samples can be collected using ADC service. First few samples collected are printed to the terminal.

## Hardware Setup and Connections

Connect the STM32H573 APP MCU board to the EVAL-ADEMA127KTZ using the on-board connector.

## Static Configuration

Example includes headers [app_cfg.h](projects/config/app_cfg.h) and [adc_datapath_cfg.h](projects/config/adc_datapath_cfg.h) for static configurations.

## Building, Running, and Debugging Examples

- [`CMakeLists.txt`](projects/CMakeLists.txt) is provided to build f the example project
- A [VS Code workspace](projects/sample_read_example.code-workspace) is also given as part of the example.
- See the [board support readme](https://github.com/analogdevicesinc/energy-board-support/blob/main/stm/app_mcu_h5/readme.md) for instructions on building, running, and debugging the example.

### Board Support Functions

The following functions from the [board_support repo](https://github.com/analogdevicesinc/energy-board-support/tree/main/generic/include) are used in this example. Users must implement these functions to ensure the example builds and operates correctly:

- `EvbInit`
- `EvbInitMessageBuffer`
- `EvbResetAdcs`
- `EvbDelayMs`
- `EvbEnableDreadyIrq`
- `EvbAdeSpiTransceive`
- `EvbFlushMessages`

The sample read example project shows how to call various APIs and interface functions of the ADC Service. The firmware initializes the ADC service, configures the ADCs, collect the samples and send it over UART.

