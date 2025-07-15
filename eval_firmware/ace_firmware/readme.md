# ACE Firmware

The workspace contains the firmware to communicate to ACE for evaluating the metering ADCs. The firmware uses IIO
protocol for ocmmunciating with the host device.

### Hardware Setup and Connections

Connect the STM32H573 APP MCU board to the ADEMA127 IEC 3-Phase Meter board using the on-board connector.

### Static Configuration

Example includes headers [app_cfg.h](config/app_cfg.h) and [adc_datapath_cfg.h](config/adc_datapath_cfg.h) for static configurations.


### Building, Running, and Debugging Examples

- [`CMakeLists.txt`](./CMakeLists.txt) is provided for building the firmware.
- A [VS Code workspace](ace_firmware.code-workspace) is also given as part of the example.
- See the [board support readme](https://github.com/analogdevicesinc/energy-board-support/blob/main/stm/app_mcu_h5/readme.md) for instructions on building, running, and debugging the example.
- Refer to [ACE Plugin User's Guide ](./ace_plugin_users_guide.md) for using ACE and the plugin with the firmware








