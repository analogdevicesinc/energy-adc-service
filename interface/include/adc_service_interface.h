/******************************************************************************
 Copyright (c) 2024 - 2025  Analog Devices Inc.
******************************************************************************/

/**
 * @file        adc_service_interface.h
 * @defgroup    ADC_INTERFACE Interface to ADC Service
 * @brief       Interface for initialising ADC Service, handling user callbacks and functions
 * required by the application to get data by calling appropriate Service APIs.
 * @{
 */

#ifndef __ADC_SERVICE_INTERFACE_H__
#define __ADC_SERVICE_INTERFACE_H__

/*============= I N C L U D E S =============*/
#include "adc_service_adapter.h"
#include "adi_adc.h"
#include "adi_adc_dsp.h"
#include "adi_adc_memory.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup    ADCINTERFACEINITCONFIG Configurations and User Data
 * @brief This section contains the configurations required to initialise ADC Service and user
 * data required by the application to process or monitor.
 * @{
 */

#ifndef APP_CFG_IGNORE_RX_BUFFER_OVERFLOW
/** Ignore Rx Buffer Overflow */
#define APP_CFG_IGNORE_RX_BUFFER_OVERFLOW 1
#endif

/**
 * ADC Params list
 */
typedef struct
{
    /** CMI register */
    uint8_t cmiReg;
    /** INV register */
    uint8_t invReg;
    /** GAIN register */
    uint8_t gain;
    /** Dsp Datapath Configuration register */
    ADI_ADC_DSP_DATAPATH_PARAMS adcDatapathParams;
    /** Dsp Channel Register*/
    ADI_ADC_DSP_CHANNEL_PARAMS adcChannelParams;
    /** Dsp Channel registers backup - used to save and reload DSP */
    ADI_ADC_DSP_CHANNEL_PARAMS adcDspBackup;
} ADEMA12X_ADC_PARAMS;

/**
 * Board related Configurations
 */
typedef struct
{
    /** Voltage channel positions on the board */
    uint8_t voltageSlots[APP_CFG_MAX_NUM_VOLTAGE_CHANNELS];
    /** Current channel positions on the board */
    uint8_t currentSlots[APP_CFG_MAX_NUM_CURRENT_CHANNELS];
} ADC_BOARD_CONFIG;

/**
 * ADC example Info structure
 */
typedef struct
{
    /** ADC Handle */
    ADI_ADC_HANDLE hAdc;
    /** Library memory */
    uint32_t adcStateMemory[ADI_ADC_STATE_MEM_NUM_BYTES / 4];
    /** Stores ADC configuration parameters */
    ADI_ADC_CONFIG adcCfg;
    /** Buffer to store ADC status while reading a block of data. */
    ADI_ADC_STATUS_OUTPUT adcStatusOutput[APP_CFG_MAX_NUM_ADC];
    /** Buffer to store ADC output samples */
    int32_t adcSamples[APP_CFG_MAX_NUM_CHANNELS * APP_CFG_MAX_SAMPLE_BLOCK_SIZE];
    /** Channel info pointer*/
    ADI_ADC_RUN_DATA runInfo;
    /** DREADY Error*/
    volatile uint8_t dreadyError;
    /** Overflow Error*/
    volatile uint8_t overflowError;
    /** Timestamp for previous sample*/
    volatile uint32_t prevDreadyTime;
    /** Timestamp for current sample*/
    volatile uint32_t currDreadyTime;
    /** ADC sampling rate configuration for ADC*/
    uint32_t adcSamplingRate;
    /** Flag indicating if adc is collecting samples or not */
    volatile bool enableRun;
    /** count of dready */
    uint32_t dreadyCnt;
    /** count of tamper detected */
    uint32_t tamperCnt;
    /** Config registers. */
    ADI_ADC_CONFIG_REGISTERS configRegisters[APP_CFG_MAX_NUM_ADC];
    /** flag to indicate that the block is ready */
    volatile bool blockReady;
    /** ADC stream mode */
    ADI_ADC_STREAM_MODE adcStreamMode;
    /** suspend state */
    volatile uint8_t suspendState;
    /** Channel index */
    uint8_t channelIdx[APP_CFG_MAX_NUM_CHANNELS];
    /** ADC register params */
    ADEMA12X_ADC_PARAMS adcRegParams[APP_CFG_MAX_NUM_ADC];
    /** Indicates whether SPI transaction is in progress */
    volatile uint8_t isSpiRunning;
    /** Indicates whether TDM cycle is running */
    volatile uint8_t isTdmCycleRunning;
    /** Buffer to store ADC register values */
    uint8_t adcRegBuff[2 * APP_CFG_MAX_NUM_ADC];
    /** bandwidth Option for the ADCs -- used to calculate HPF Corner Freq */
    uint32_t bwOption[APP_CFG_MAX_NUM_ADC];
    /** Input clk to adc board */
    uint32_t clkIn;
    /** DataRate Decimate by 2 bit*/
    uint8_t decimateBy2;
    /** Board-Related Configs */
    ADC_BOARD_CONFIG adcBoardConfig;
    /** ADC_CMI and ADC_INV register value */
    uint8_t regCmiInvVal[APP_CFG_MAX_NUM_ADC];
    /** Flag set when DREADY interrupt occurs */
    volatile uint8_t dreadyFlag;
} ADC_INTERFACE_INFO;

/** @} */

/** @defgroup    ADCINTERFACEINIT ADC Service Interface Init Functions
 * @brief Interface functions required to create ADC Service instance and initialise with
 * appropriate configurations by calling service APIs.
 * @{
 */

/**
 * @brief ADC callback function.
 * @param[in] hUser - user handle.
 * @param[in] adcEvent - ADC event.
 * @return Result of the command
 */

ADI_ADC_STATUS AdcIfAdcCallback(void *hUser, uint32_t adcEvent);

/**
 * @brief Interface to APIs that creates instance for ADC Service.
 * @param[in]  pInfo - pointer to interface info structure.
 * @return Result of the command
 */
int32_t AdcIfCreateService(ADC_INTERFACE_INFO *pInfo);

/**
 * @brief Gets pointer to interface info structure.
 * @return pointer to interface info structure.
 */
ADC_INTERFACE_INFO *AdcIfGetInstance(void);

/**
 * @brief Interface to APIs that perform initialization of service, perform resets and configure the
 * first ADC in daisy chain.
 * @param[in]  pInfo - pointer to interface info structure.
 * @param[in]  numAdc - num adc.
 * @param[in]  pAdcType - adc type.
 * @return Result of the command
 */
ADI_ADC_STATUS AdcIfInitService(ADC_INTERFACE_INFO *pInfo, uint8_t numAdc, ADI_ADC_TYPE *pAdcType);

/**
 * @brief Interface to APIs that perform ADC configuration.
 * @param[in]  pInfo - pointer to interface info structure.
 * @return Result of the command
 */
ADI_ADC_STATUS AdcIfConfigureAdcs(ADC_INTERFACE_INFO *pInfo);

/**
 * @brief  Interface to APIs that perform reconfiguration.
 * @param[in]  pInfo - pointer to interface structure.
 * @return Result of the command
 */
ADI_ADC_STATUS AdcIfSetConfig(ADC_INTERFACE_INFO *pInfo);

/** @} */

/**
 * @brief Interface to APIs that write to ADC register.
 * @param[in]  pInfo - pointer to interface info structure.
 * @param[in]  address - address.
 * @param[in]  value - value.
 * @param[in]  adcIdx - adc index.
 * @return Result of the command
 */
ADI_ADC_STATUS AdcIfWriteRegister(ADC_INTERFACE_INFO *pInfo, uint16_t address, uint8_t value,
                                  int8_t adcIdx);

/**
 * @brief Interface to APIs that read data from ADC register.
 * @param[in]  pInfo - pointer to interface info structure.
 * @param[in]  address - address.
 * @param[in]  adcIdx - adc index.
 * @param[out]  pBuffer - pointer to buffer.
 * @param[out]  pNumBytes - pointer to number of bytes.
 * @return Result of the command
 */
ADI_ADC_STATUS AdcIfReadRegister(ADC_INTERFACE_INFO *pInfo, uint16_t address, int8_t adcIdx,
                                 uint8_t *pBuffer, uint32_t *pNumBytes);

/**
 * @brief Interface to APIs that start ADC sample capture.
 * @param[in]  pInfo - pointer to interface info structure.
 * @return Result of the command
 */
ADI_ADC_STATUS AdcIfStartCapture(ADC_INTERFACE_INFO *pInfo);

/**
 * @brief Interface to APIs that stop ADC sample capture.
 * @param[in]  pInfo - pointer to interface info structure.
 * @return Result of the command
 */
ADI_ADC_STATUS AdcIfStopCapture(ADC_INTERFACE_INFO *pInfo);

/**
 * @brief  Interface to SPI Rx callback.
 */
void AdcIfSpiRxCallback(void);

/**
 * @brief  Interface to GPIO callback.
 * @param[in]  port - Port.
 * @param[in]  pinFlag - pin flag.
 */
void AdcIfDreadyCallback(uint32_t port, uint32_t pinFlag);

/**
 * Connect approrpiate ADCs to the board.
 * @param[in]  numAdc - number of adcs.
 * @param[in]  pAdcType - type of adcs.
 */
int32_t EvbConnectAdc(int32_t numAdc, ADI_ADC_TYPE *pAdcType);

/**
 * Interface fucntion to read response status.
 * @param[in]  pInfo - pointer to interface info structure.
 */
ADI_ADC_STATUS AdcIfGetLastFrameStatus(ADC_INTERFACE_INFO *pInfo);

/**
 * @brief Interface to APIs that perform ADC synchronisation.
 * @param[in]  pInfo - pointer to interface info structure.
 * @return Result of the command
 */
ADI_ADC_STATUS AdcIfPerformSync(ADC_INTERFACE_INFO *pInfo);

/**
 * @brief Interface to APIs that read the SILICON_REVISION register and PRODUCT_ID register.
 * @param[in]  pInfo - pointer to interface info structure.
 * @param[in]  adcIdx - adc index.
 * @param[out]  pSiliconRevision - Pointer to Silicon Revision buffer.
 * @param[out]  pProductId - Pointer to Product ID buffer.
 * @return Result of the command
 */
int32_t AdcIfReadVersion(ADC_INTERFACE_INFO *pInfo, int8_t adcIdx, uint8_t *pSiliconRevision,
                         uint8_t *pProductId);

/**
 * Return DREADY Error Flag and Overflow Error.
 * @param[in]  pDreadyErr - Pointer to DREADY Error Flag Variable
 * @param[in]  pOverflowErr - Pointer to Overflow Error Flag Variable
 */
void ReturnAdcErrorFlags(volatile uint8_t *pDreadyErr, volatile uint8_t *pOverflowErr);

/**
 * @brief Interface to APIs that wait for ADC response.
 * @param[in]  pInfo - pointer to interface info structure.
 * @return Result of the command
 */
ADI_ADC_STATUS AdcIfWaitAdcResponse(ADC_INTERFACE_INFO *pInfo);

/**
 * @brief  Function that collect samples.
 * @param[in]  pInfo - pointer to interface info structure.
 * @param[in]  channelMask - channel mask.
 * @param[in]  numSamplesRequired - number of samples required.
 * @param[out]  pSamples - pointer to buffer.
 * @return Result of the command
 */
ADI_ADC_STATUS AdcIfCollectSamples(ADC_INTERFACE_INFO *pInfo, uint32_t channelMask,
                                   uint32_t numSamplesRequired, int32_t *pSamples);

/**
 * @brief Setting the values of ADEMA12X DSP RAM registers present in the structure
 * #ADI_ADC_DSP_CHANNEL_PARAMS.
 * @param[in]  pInfo - pointer to interface info structure.
 * @param[in]  numAdc - number of adcs.
 * @param[in]  pAdcType - type of adcs.
 * @return Result of the command
 */
ADI_ADC_STATUS AdcIfPopulateDspRegisterStruct(ADC_INTERFACE_INFO *pInfo, uint8_t numAdc,
                                              ADI_ADC_TYPE *pAdcType);

/**
 * @brief Getting the values of ADEMA12X DSP RAM registers present in the structure
 * #ADI_ADC_DSP_CHANNEL_PARAMS.
 * @param[in]  pInfo - pointer to interface info structure.
 * @param[in]  numAdc - number of adcs.
 * @param[in]  pAdcType - type of adcs.
 * @return Result of the command
 */
ADI_ADC_STATUS AdcIfGetDspRegisterStruct(ADC_INTERFACE_INFO *pInfo, uint8_t numAdc,
                                         ADI_ADC_TYPE *pAdcType);

/**
 * @brief Writes to channel integer sample delay configuration of ADC.
 * @param[in]  pInfo - pointer to interface info structure.
 * @param[in]  pValue - pointer to gain values.
 * @param[in]  pChanIdx 	Pointer to ADC channel indices.
 * @param[in]  numChan	 	Number of channel registers to be written.
 * @param[in]  adcIdx - adc index.
 * @return Result of the command
 */
ADI_ADC_STATUS AdcIfSetIntegerSampleDelay(ADC_INTERFACE_INFO *pInfo, uint8_t *pValue,
                                          uint8_t *pChanIdx, int8_t numChan, int8_t adcIdx);

/**
 * @brief Reads the channel integer sample delay configuration of ADC.
 * @param[in]  pInfo - pointer to interface info structure.
 * @param[in]  pChanIdx 	Pointer to ADC channel indices.
 * @param[in]  numChan	 	Number of channel registers to be written.
 * @param[in]  adcIdx - adc index.
 * @param[out]  pValue - pointer to gain values.
 * @return Result of the command
 */
ADI_ADC_STATUS AdcIfGetIntegerSampleDelay(ADC_INTERFACE_INFO *pInfo, uint8_t *pChanIdx,
                                          int8_t numChan, int8_t adcIdx, uint8_t *pValue);

#ifdef __cplusplus
}
#endif

#endif /* ADC_SERVICE_INTERFACE_H_ */
/**
 * @}
 */
