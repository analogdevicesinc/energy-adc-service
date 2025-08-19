# Instructions to Port Examples to a Different Board Design

## Overview
There are example projects designed for ADI evaluation boards. This document explains the steps or code changes required to port the example available to a different board design.

## Static Configuration
Example expects [application config](../include/config) folder containing app_cfg.h and adc_datapath_cfg.h for static configurations.<br>
In [app_cfg.h](../include/config/baremetal/app_cfg.h) file, update the macros based upon the system. <br>

## Interfacing to MCU Drivers
Following changes to code are required to run the example project on any customized board design:

1. MCU board support:<br>
Example projects are designed to support STM32 boards, utilizing HAL Drivers for MCU communication. All the MCU specific code is abstracted out from the library and are called from interface and adapter files. Developers need to redefine the functions starting with 'Evb' with MCU specific implementations.

2. ADC SPI:<br>
ADCs communicate with the host MCU using a command-response protocol through the SPI interface. Therefore, the application should connect the appropriate ADC pins to the host MCU SPI.<br>
In the example project, SPI1 is configured with the following pins: PA5 - SCLK, PA6 - MISO, PA7 - MOSI, PA4 - CS<br>
If different SPI port or different pins are to be used, then make the following changes:
    - SPI initialization - update "HAL_SPI_MspInit" and "HAL_SPI_MspDeInit" functions in <project_folder>/stm_gen_code/stm32h5xx_hal_msp.c
    - Replace "SPI1_IRQ_handler" in <project_folder>/stm_gen_code/stm32h5xx_it.c and <project_folder>/stm_gen_code/stm32h5xx_it.h files with the IRQ handler of the SPI port being used

4. GPIO pins:<br>
Example uses multiple GPIO pins to monitor different signals from ADC (ADC_DREADY and ADC_RESET) required for proper functioning.<br>
In the example project, they are configured with the following pins: ADC_DREADY - PB2, ADC_RESET - PD14.
Developers need to redefine the following macros based upon the board design -
"BOARD_CFG_ADC_RESET_PORT", "BOARD_CFG_ADC_DREADY_PORT", "BOARD_CFG_ADC_RESET_PIN", "BOARD_CFG_ADC_DREADY_PIN".

## Subsequent Steps
Once the code changes explained in the previous section are complete, then developer can build and run the project using the instructions given in [bsp readme](https://github.com/analogdevicesinc/energy-board-support/blob/main/stm/app_mcu_h5/readme.md).
