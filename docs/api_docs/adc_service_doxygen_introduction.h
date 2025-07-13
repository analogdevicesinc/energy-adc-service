/******************************************************************************
 Copyright (c) 2023 - 2025  Analog Devices Inc. All Rights Reserved.
 This software is proprietary & confidential to Analog Devices, Inc.
 and its licensors.
******************************************************************************/

/* "Mainpage" is the main page for the DoxyGen output.
   Use it to present overview of the chip, drivers, release notes, etc.
   Use the "@section" directive to create section headers.
   Use the "@htmlinclude" directive to import html documentation.
 */

/** @mainpage notitle

    @section Intro ADC Service description

    ADC Service contains a set of APIs to communicate with ADI metrology ADCs and to collect waveform samples.
    This release supports ADE9113, ADE9112, ADE9103, ADEMA124 and ADEMA127.  These ADCs support a command and response protocol
    over SPI interface. Refer to datasheets of the respective ADCs for details on the command and response protocols.

    These ADCs also supports daisy chaining of multiple ADCs. ADC Service also supports sending commands to each ADC and collect samples from each of them appropriately.
    The service expects the application to initialise and configures the following peripherals for correct functionality.
    - SPI is connected to ADCs. \n
      - SPI mode is 3. [clock polarity = 1, clock phase = 1]

    Following functions should be called from appropriate ISRs\n
          - #adi_adc_ValidateSamples -  When ADC samples have been received.

    Before calling any ADC service APIs, please make sure that the required peripherals are initialized and callbacks are set using appropriate MCU driver codes.
    Example codes are given along with the package.

    @subsection Init Initialization and configuration of ADC service

    The ADC service should be instantiated and configured properly before using other APIs

    - #adi_adc_Create \n
    This API creates the instance of ADC Service. \n

    - #adi_adc_Init \n
          Use this API to initialize and configure the service
          - Fill the #ADI_ADC_CONFIG appropriately and pass to this API. \n
            - #ADI_ADC_CONFIG.pfTransceive -  A function to transmit and receive SPI data  in blocking mode\n
            - #ADI_ADC_CONFIG.pfTransceiveAsync -  A function to transmit and receive SPI data \n
            - #ADI_ADC_CONFIG.pfCallback -  This function is called to indicate the completion of various ADC service events. \n
    The API #adi_adc_SetConfig can be used if any reconfigurations needed to be done later. \n
    The API #adi_adc_GetConfig can be used to get the current configuration.

    @subsection config_adc Configuration of ADCs

    The ADCs should be configured before accessing data from it.

    - #adi_adc_EnableClockOut \n
    This API configures the first ADC to provide clock to other ADCs in a daisy chain configuration. \n

    - #adi_adc_ConfigureAdcs \n
    This API configures all the ADCs in the daisy chain configuration. \n

    Following sample code shows initialization and configuration sequence. Refer to #AdcIfCreateService and #AdcIfInitService for implementation.

     @code
     #include <adi_adc.h>

      // Initialise the peripherals SPI, GPIO, UART, Timers
      adi_adc_Create()

      // Fill configuration structure #ADI_ADC_CONFIG before initialisation
      adi_adc_Init()

      // Reset ADCs.

      // Wait for #ADI_ADC_STARTUP_TIME_MS for the first ADC to become functional.

      // Enable the first ADC to provide clock out to the other ADCs
      adi_adc_EnableClockOut()

      // Wait for #ADI_ADC_STARTUP_TIME_MS for the ADCs to become functional.

      // Configure the ADCs.
      adi_adc_ConfigureAdcs()

     @endcode

    @subsection adc_read_write ADC Registers read and write
    After configuring the ADCs, ADC service can perform register read and write.
    If ADCs are not performing capture of samples, then register write and read can be done using #adi_adc_WriteRegister and #adi_adc_ReadRegister
    If ADCs are performing capture of samples, then register write and read can be done using #adi_adc_AssembleWriteRegister, #adi_adc_AssembleReadRegister and #adi_adc_GetLastRegister.

    @subsection adc_sample ADC sample capture
    After configuring the ADCs, collection of samples can be done. After starting the sample capture #adi_adc_CollectSamples
    can be called to collect the ADC samples. If a block of samples are ready, #adi_adc_ReadBlock can be called to read the samples from ADC service.

    @subsection adc_sync ADC synchronization
    ADC synchronization can be done using #adi_adc_Align and #adi_adc_StartAlign APIs.

    @subsection adc_status status management
    Internal status captured by the ADC service can be read using the APIs #adi_adc_GetLastFrameStatus and #adi_adc_GetRunData.

    @subsection error_management Error management
    ADC Service APIs returns an enumeration of type #ADI_ADC_STATUS. Refer to @ref ADCSTATUS for more details.

    @subsection datapath_configuration Datapath configuration
    ADC Service provides APIs to write and read the datapath configuration registers and has API's to write to the DSP region.
    The API's in @ref ADC_DATAPATH can be called to configure the datapath registers. Then the @ref ADC_DSP can be called to write to the uDSP region.

    To see the full API reference you can either search for a specific API or
    browse by selecting the "Modules" node. The main APIs to be used are as
    follows.


   @section Specifications Specifications
   Following section describes the memory usage of the ADC Service.

    | Metrics | Size in Bytes |
    |:--------:| --------------:|
    | Code Size | 23520 |
    | Ram Size | 1840 |
    | Stack Size | 548 |

   Following section describes the cycle usage by the ADC Service API's.
   Its measured on STM32H573ZI based eval board with clock speed of 250Mhz.

    1. Single ADEMA127 ADC

    | API | Cycles |
    |:--------:| --------------:|
    | #adi_adc_CollectSamples | 476 |
    | #adi_adc_ValidateSamples + ADI Software CRC calculation routines | 620 |
    | ADI Software CRC calculation routines | 395 |
    | #adi_adc_ReadBlock | 545 |

   @section Docs Documentation
   @subsection api_doc API document
   The ADC Service (this document) is released in html
   generated from doxygen comments in the code

   @subsection ReleaseNotes Release notes
    ADSW-ADC-Service.0.9.0_ReleaseNotes.pdf
    can be found in the in the docs folder.

   @section Support Technical or customer support
   You can reach Analog Devices, Inc. Customer Support at:
        - E-mail ADC service questions to
                 - energy.support@analog.com

   @section CopyrightIntro Copyright
    Copyright (c) 2025 Analog Devices.

 */

/*
** EOF
*/

/*@}*/
