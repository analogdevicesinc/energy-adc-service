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
#include "adc_datapath_cfg.h"
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

#if (APP_CFG_USE_TIMESTAMP == 1)
/** Size of timestamp buffer. */
#define TIMESTAMP_BUFFER_SIZE (APP_CFG_MAX_SAMPLE_BLOCK_SIZE * APP_CFG_MAX_NUM_ADC)
#endif

/**
 * @brief Structure to store ADC DSP registers.
 */
typedef struct
{
    /** XT aggressor */
    ADI_ADC_CHAN_XT_AGGRESSOR xtAggressor[APP_CFG_MAX_NUM_CHANNELS_PER_ADC];
    /** channel offset */
    int32_t offset[APP_CFG_MAX_NUM_CHANNELS_PER_ADC];
    /** XT gain */
    float xtGain[APP_CFG_MAX_NUM_CHANNELS_PER_ADC];
    /** gain */
    float gain[APP_CFG_MAX_NUM_CHANNELS_PER_ADC];
    /** shift */
    uint8_t shift[APP_CFG_MAX_NUM_CHANNELS_PER_ADC];

} ADC_IF_DSP_PARAMS;

/**
 * @brief Structure to store ADC Datapath registers.
 */
typedef struct
{
    /** Datapath Config */
    ADI_ADC_CHAN_DATAPATH_CONFIG dataPathConfig[APP_CFG_MAX_NUM_CHANNELS_PER_ADC];
    /** phase offset */
    float phaseOffset[APP_CFG_MAX_NUM_CHANNELS_PER_ADC];
} ADC_IF_DATAPATH_PARAMS;

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
    /** Stores ADC DSP registers */
    ADC_IF_DSP_PARAMS dspParams;
    /** ADC Datapath params */
    ADC_IF_DATAPATH_PARAMS datapathParams;
#if APP_CFG_ENABLE_DSP_BACKUP == 1
    /** Dsp Channel registers backup - used to save and reload DSP */
    ADI_ADC_DSP_CHANNEL_PARAMS adcDspBackup;
    /** Stores DSP registers backup */
    ADC_IF_DSP_PARAMS dspBackupParams;
#endif
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
    /** Input clk to adc board */
    uint32_t clkIn;
    /** Datarate Decimate by 2 bit*/
    uint8_t decimateBy2;
    /** ADC stream mode */
    ADI_ADC_STREAM_MODE adcStreamMode;
    /** ADC sampling rate configuration for ADC*/
    uint32_t adcSamplingRate;
    /** Number of ADCs */
    uint8_t numAdc;
    /** ADC types */
    ADI_ADC_TYPE adcType[APP_CFG_MAX_NUM_ADC];
    /** ADC inversion bit */
    uint8_t adcInv[APP_CFG_MAX_NUM_ADC];
    /** Integer sample delay requested by user. */
    uint8_t integerSampleDelay[APP_CFG_MAX_NUM_CHANNELS];
} ADC_BOARD_CONFIG;

/**
 * ADC interface Info structure
 */
typedef struct
{
    /** ADC Handle */
    ADI_ADC_HANDLE hAdc;
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
    /** flag to indicate that the response is ready */
    volatile bool responseReady;
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
    /** ADC_CMI register value */
    uint8_t regCmiVal[APP_CFG_MAX_NUM_ADC];
    /** Flag set when DREADY interrupt occurs */
    volatile uint8_t dreadyFlag;
    /** Function Pointer to ADC events. */
    ADI_ADC_CALLBACK_FUNC pfCallback;
#if (APP_CFG_USE_TIMESTAMP == 1)
    /** Timestamp */
    uint32_t timestamp[TIMESTAMP_BUFFER_SIZE];
#endif
    /** Library memory */
    uint32_t adcStateMemory[ADI_ADC_STATE_MEM_NUM_BYTES_4XADEMA127_4XBLOCKSIZE / 4];
    /** Flag set when DSP LOCK occurs */
    uint8_t dspLockFlag;
    /** Flag to reset example samples buffer writeIndex */
    bool resetSamplesBuffer;
    /** Flag to indicate that collect samples has started */
    volatile bool isCollectSamplesStarted;
    /** Frame format */
    ADI_ADC_FRAME_FORMAT frameFormat;

} ADC_INTERFACE_INFO;

/** @} */

/** @defgroup    ADCINTERFACEINIT ADC Service Interface Init Functions
 * @brief Interface functions required to create ADC Service instance and initialise with
 * appropriate configurations by calling service APIs.
 * @{
 */

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
 * @param[in]  pAdcBoardConfig - pointer to board config.
 * @return Result of the command
 */
ADI_ADC_STATUS AdcIfInitService(ADC_INTERFACE_INFO *pInfo, ADC_BOARD_CONFIG *pAdcBoardConfig);

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
 * @brief Waits for ADC response. This function is to be implemented in application.
 * @param[in]  pInfo - pointer to interface info structure.
 * @return Result of the command
 */
ADI_ADC_STATUS AdcIfWaitAdcResponse(ADC_INTERFACE_INFO *pInfo);

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
 * @brief Saving a backup of ADEMA12X DSP RAM registers present in the structure
 * #ADI_ADC_DSP_CHANNEL_PARAMS.
 * @param[in]  pInfo - pointer to interface info structure.
 * @param[in]  numAdc - number of adcs.
 * @param[in]  pAdcType - type of adcs.
 * @return Result of the command
 */
ADI_ADC_STATUS AdcIfPopulateDspBackupRegisterStruct(ADC_INTERFACE_INFO *pInfo, uint8_t numAdc,
                                                    ADI_ADC_TYPE *pAdcType);

/**
 * @brief Reloading the values of ADEMA12X DSP RAM registers present in the structure
 * #ADI_ADC_DSP_CHANNEL_PARAMS.
 * @param[in]  pInfo - pointer to interface info structure.
 * @param[in]  numAdc - number of adcs.
 * @param[in]  pAdcType - type of adcs.
 * @return Result of the command
 */
ADI_ADC_STATUS AdcIfGetDspBackupRegisterStruct(ADC_INTERFACE_INFO *pInfo, uint8_t numAdc,
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
