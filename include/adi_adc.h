/******************************************************************************
 Copyright (c) 2022 - 2025  Analog Devices Inc.
******************************************************************************/

/**
 * @file adi_adc.h
 * @addtogroup ADI_ADC ADC Service library.
 * @{
 * @brief Functions and definitions required to create instance, initialise service with the given
 * configurations, and to communicate with ADCs via SPI to get the data from the ADCs.
 */

#ifndef __ADI_ADC_H__
#define __ADI_ADC_H__

/*============= I N C L U D E S =============*/
#include "app_cfg.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup ADCCONFIG Handle, Macros and Configurations
 * @brief This section contains the Configurations of ADC Service.
 * The function pointers to be initialised from the application, this should be of same type
 * otherwise it throws warnings while building.
 * @{
 */

/** Typical delay in startup of ADCs after applying clock */
#define ADI_ADC_STARTUP_TIME_MS 65

/** CRC type for ADC commands.*/
#define ADI_ADC_CMD_CRC_TYPE ADI_CRC_TYPE_CRC8
/** CRC Polynomial for ADC commands.*/
#define ADI_ADC_CMD_CRC_POLY (uint32_t)0x7
/** CRC seed for ADC commands.*/
#define ADI_ADC_CMD_CRC_SEED (uint32_t)0x0
/** CRC xor out value for ADC commands*/
#define ADI_ADC_CMD_CRC_XOR_OUT (uint32_t)0x55

/** CRC type for ADC response.*/
#define ADI_ADC_RESP_CRC_TYPE ADI_CRC_TYPE_CRC16
/** CRC Polynomial for ADC response.*/
/** FIXME: Fix MAX CRC service to reverse th polynomial as part of config*/
#define ADI_ADC_RESP_CRC_POLY (uint32_t)0x1021
/** CRC seed for ADC response.*/
#define ADI_ADC_RESP_CRC_SEED (uint32_t)0xFFFF
/** CRC xor out value for ADC response */
#define ADI_ADC_RESP_CRC_XOR_OUT (uint32_t)0x0

/** Mask indicating if a ADC block is ready to be collected. */
#define ADI_ADC_EVENT_BITM_BLOCK_READY 0x1
/** Mask indicating if the command response is ready to be collected. */
#define ADI_ADC_EVENT_BITM_RESPONSE_READY 0x2

/**
 * @brief The full-scale code value for the ADC.
 *
 * This constant represents the full-scale code value for the ADC.
 * The full-scale code value is the maximum digital value that the
 * ADC can produce when the input voltage is at its maximum level.
 */
#define ADEMA12x_ADC_FULL_SCALE_CODE 0x599999

/** The full-scale code value for the ADE91XX ADC. */
#define ADE91XX_ADC_FULL_SCALE_CODE 0x7FFFFF

/**
 * @brief Function pointer type for transmit and receive operations.
 * @param hUser User handle.
 * @param pTxData Pointer to transmit data buffer.
 * @param pRxData Pointer to receive data buffer.
 * @param numBytes Number of bytes to transmit/receive.
 * @return Status code.
 */
typedef int32_t (*ADI_ADC_TRANSCEIVE_FUNC)(void *hUser, uint8_t *pTxData, uint8_t *pRxData,
                                           uint32_t numBytes);

/**
 * @brief Function pointer type for CRC calculation.
 * @param pData Pointer to data buffer.
 * @param numBytes Number of bytes in data buffer.
 * @param pCrc Pointer to CRC output.
 * @return Status code.
 */
typedef int32_t (*ADI_ADC_CALC_CRC_FUNC)(uint8_t *pData, uint32_t numBytes, uint32_t *pCrc);

/**
 * @brief Device handle used in all API functions to identify the ADC instance.
 *        Obtained from the #adi_adc_Create API.
 */
typedef void *ADI_ADC_HANDLE;

/**
 * @brief Enumeration for ADC types.
 */
typedef enum
{
    /** ADE9113 */
    ADI_ADC_TYPE_ADE91XX = 0u,
    /** ADEMA124 */
    ADI_ADC_TYPE_ADEMA124,
    /** ADEMA127 */
    ADI_ADC_TYPE_ADEMA127,
} ADI_ADC_TYPE;

/**
 * Enumeration for stream mode.
 */
typedef enum
{
    /** Normal mode */
    ADI_ADC_STREAM_MODE_NORM = 0u,
    /** Incremental mode */
    ADI_ADC_STREAM_MODE_INCR,
    /** Static mode */
    ADI_ADC_STREAM_MODE_STATIC,
} ADI_ADC_STREAM_MODE;

/**
 * Structure to hold the configuration registers.
 */
typedef struct
{
    /** stream debug mode
     *  0 - Normal mode
     *  2 - Data Increments at ADC Conversion Rate
     */
    uint8_t config0StreamDbg;
    /** config_filt register value
     *  Used to configure the sampling rate in ADE911x */
    uint8_t configFilt;
    /** Datarate register value
     *  Used to configure the sampling rate in ADEMA12x */
    uint8_t datarate;

} ADI_ADC_CONFIG_REGISTERS;

/** @} */

/** @defgroup   ADCSTATUS Return Codes

    @brief ADC Service returns #ADI_ADC_STATUS for every API
    call. The application can look at the return codes and can take appropriate
    action for recovery. If multiple errors are there, the API will return  the
    return code of first encountered error.

    If an API returns an error, its description in the doxygen documentation
    gives hints at the cause of the error.

    Errors detected at development time
    regardless of the type of error, all errors should be identified and only
    ignored if the cause is known and understood, including informational
    errors.
    - Identify the API that returns the error.
    - Look at the doxygen description of the API that returned the error.
    - Ensure that any recommendations in the API description are followed.

    @note It is recommended that at the development stage all errors are
    evaluated, even the ones that can be ignored once the product is deployed,
    since they may help debugging issues in the application. Look into each
    error to determine the possible causes in each API.
 * @{
 */

/**
 * Status Codes of ADC Service
 */
typedef enum
{
    /** No error, API succeeded. */
    ADI_ADC_STATUS_SUCCESS = 0u,
    /** One of the pointers passed to the API is NULL. */
    ADI_ADC_STATUS_NULL_PTR,
    /** State memory provided is less than required memory
     * for the configuration. */
    ADI_ADC_STATUS_INSUFFICIENT_STATE_MEMORY,
    /** Value of "numAdc" in the structure #ADI_ADC_CONFIG passed to the function is invalid.
     */
    ADI_ADC_STATUS_INVALID_NUM_ADC,
    /** This status is returned by an API when
     * 1. Value of "adcType" in the structure #ADI_ADC_CONFIG passed to the function is invalid.
     * 2. An API not supported for the configured ADC is called.
     * 3. When the number of channels provided to the API is more than the channels
     *    present in the configured ADC.
     */
    ADI_ADC_STATUS_INVALID_ADC_TYPE,
    /** Value of sampling rate provided as argument to the API is invalid. */
    ADI_ADC_STATUS_INVALID_SAMPLING_RATE,
    /** Value of stream mode provided as argument to the API is invalid. */
    ADI_ADC_STATUS_INVALID_STREAM_MODE,
    /** ADC index provided as argument to the API is invalid. */
    ADI_ADC_STATUS_INVALID_ADC_INDEX,
    /** Errors occurred while configuring the ADCs to its operational state. */
    ADI_ADC_STATUS_CONFIGURE_FAILED,
    /** Errors in CRC calculation.
     *  This occurs when #ADI_ADC_CONFIG.pfCalcCmdCrc
     *  or #ADI_ADC_CONFIG.pfCalcRespCrc returns any errors.
     */
    ADI_ADC_STATUS_CRC_CALC_FAILED,
    /** Call to ADI_ADC_TRANSCEIVE_FUNC or ADI_ADC_TRANSCEIVE_ASYNC_FUNC failed. */
    ADI_ADC_STATUS_TRANSCEIVE_FAILED,
    /** Response received from ADC has CRC errors. */
    ADI_ADC_STATUS_CRC_ERROR,
    /**  Last register read is not ready. This could be because response is not yet received */
    ADI_ADC_STATUS_LAST_REGISTER_NOT_READY,
    /** No more data in the samples buffer. This occurs when no new block of samples are available.
     */
    ADI_ADC_STATUS_NO_DATA,
    /** ADC Frame Buffer overflow. This will occur when samples are not collected before they are
       overwritten in the frame buffer */
    ADI_ADC_STATUS_BUFFER_OVERFLOW,
    /** Datapath configuration failed.
     * This status is returned by an API when -
     * 1. The number of channels provided to the API is more than the channels
     *    present in the configured ADC.
     */
    ADI_ADC_STATUS_DATAPATH_CONFIGURATION_FAILED,
    /** Previous command issued to the ADC by #adi_adc_CollectSamples is not complete. */
    ADI_ADC_STATUS_PREV_CMD_RUNNING,
    /** Status to indicate that adc align issued by #adi_adc_StartAlign is still in progress. */
    ADI_ADC_STATUS_ALIGN_IN_PROGRESS,
    /** Internal error detected. The APIs are not expected to return this.
     * This shows either there is bug inside the library.
     */
    ADI_ADC_STATUS_INTERNAL_ERROR,
    /** Status from simulation model. The APIs are not expected to return this.
     * Invalid Register address requested.
     */
    ADI_ADC_STATUS_INVALID_ADDR,
    /** Status from simulation model. The APIs are not expected to return this.
     * Size of Tx data is less.
     */
    ADI_ADC_STATUS_SPI_TX_SIZE_LESS,
    /** The slot provided does not refer to a valid channel on the ADCs. */
    ADI_ADC_STATUS_INCORRECT_SLOT_CONFIG,
    /** This indicates that the frame format set is incorrect. Either short
     * format is set with numAdc > 1 or short format is set for sample collection
     */
    ADI_ADC_STATUS_INCORRECT_FRAME_FORMAT_SET,
    /** The integer sample delay provided is not valid */
    ADI_ADC_STATUS_INVALID_SAMPLE_DELAY,
    /** Value of channel index passed to the function is invalid. */
    ADI_ADC_STATUS_INVALID_CHANNEL_INDEX,
} ADI_ADC_STATUS;

/** @} */

/** @addtogroup ADCCONFIG
 * @{
 */

/** Callback type of ADC service. Function indicates occurance of a block ready or response ready */
typedef ADI_ADC_STATUS (*ADI_ADC_CALLBACK_FUNC)(void *hUser, uint32_t eventMask);

/**
 * Structure for ADC configuration parameters.
 */
typedef struct
{
    /** Number of ADCs in the board.
     */
    uint8_t numAdc;
    /** Ignore Rx Buffer overflow error and continue collecting ADC frames */
    volatile uint8_t ignoreRxBufferOverflow;
    /** Block size. */
    uint8_t numSamplesInBlock;
    /** Frame format. */
    uint8_t frameFormat;
    /** User handle. */
    void *hUser;
    /** Function pointer for SPI transceive. The function should return only after completing the
     * transfer.
     */
    ADI_ADC_TRANSCEIVE_FUNC pfTransceive;
    /** Function pointer for SPI transceive in asynchronous manner.
     */
    ADI_ADC_TRANSCEIVE_FUNC pfTransceiveAsync;
    /** Function Pointer to ADC events. */
    ADI_ADC_CALLBACK_FUNC pfCallback;

    /** Function pointer to calculate CRC for ADC command. This should point to a function which
     * calculates CRC-8. */
    ADI_ADC_CALC_CRC_FUNC pfCalcCmdCrc;
    /** Function pointer to calculate CRC for ADC response. This should point to a function which
     * calculates CRC-16. */
    ADI_ADC_CALC_CRC_FUNC pfCalcRespCrc;
    /** Integer sample delay requested by user. */
    uint8_t *pIntegerSampleDelay;
    /** Type of ADCs connected.
     *  This should be of type #ADI_ADC_TYPE.
     */
    ADI_ADC_TYPE *pAdcType;
    /** Maximum value for sample delay. */
    uint8_t maxSampleDelay;
} ADI_ADC_CONFIG;

/** @} */

/** @defgroup   ADCENUMS Enumerations
 *  @brief This section covers various enumerations used in ADC service.
 * @{
 */

/**
 * Enumeration for run state.
 */
typedef enum
{
    /** Run state - Instance is not created */
    ADI_ADC_RUN_STATE_NOT_CREATED,
    /** Run state - ADCs have not been configured */
    ADI_ADC_RUN_STATE_NOT_CONFGURED,
    /** Run state - Configuration of ADCs is in progress*/
    ADI_ADC_RUN_STATE_CONFIG_IN_PROGRESS,
    /** Run state - Ready to communicate with ADCs */
    ADI_ADC_RUN_STATE_READY,
    /** Run state - Transeive in progress */
    ADI_ADC_RUN_STATE_TRANSEIVE_IN_PROGRESS,
    /** Run state - CRC Check in progress */
    ADI_ADC_RUN_STATE_CRC_CHCK_IN_PROGRESS,
    /** Run state - Callback in progress */
    ADI_ADC_RUN_STATE_CALLBACK_IN_PROGRESS,
    /** Run state - Idle state */
    ADI_ADC_RUN_STATE_IDLE,

} ADI_ADC_RUN_STATE;

/** @} */

/** @defgroup   ADCSTRUCT Output structures
 *  @brief This section covers various structures used in ADC service.
 * @{
 */

/**
 * Structure to hold status output.
 */
typedef struct
{
    /** CRC Error. */
    uint8_t crcError : 1;
    /** Buffer overflow*/
    uint8_t overflowError : 1;
    /** STATUS 0 from ADC. */
    uint8_t status0;
    /** STATUS 1 from ADC . */
    uint8_t status1;
    /** STATUS 2 from ADC. */
    uint8_t status2;
} ADI_ADC_STATUS_OUTPUT;

/**
 * Structure to hold the run status.
 */
typedef struct
{
    /** total number of channels*/
    uint32_t totalChannels;
    /** Number of samples collected */
    uint32_t numSamplesCollected;
    /** Count of overflow error */
    uint32_t overflowCount;
    /** Count of SPI TxRx error */
    uint32_t busyCount;
    /** Number of remaining samples to be collected in a block */
    uint8_t numRemainingSamples;
    /** Current state of ADC service */
    ADI_ADC_RUN_STATE currentState;
#if (APP_CFG_USE_TIMESTAMP == 1)
    /** timestamp */
    uint32_t *pTimestamp;
#endif
} ADI_ADC_RUN_DATA;

/** @} */

/** @defgroup   ADCAPI ADC service initialization functions.
 * @brief This section covers APIs required to initialize the ADC service.
 * All the APIs return enumeration codes in #ADI_ADC_STATUS. Refer to #ADI_ADC_STATUS for detailed
 * documentation on return codes
 * @{
 */

/**
 * @brief Function to create instance for ADC Service.
 * @details Assign memory and sets up the internal structures and configuration
 * of the ADC Service.
 *
 * @param[out] phAdc         Pointer to a location where the handle to
 * the ADC Service is written. This handle is required in all other ADC Service APIs.
 * @param[in]  pStateMemory          The pointer to the memory for the
 * library.
 * This pointer must be 32-bit aligned. This memory must be persistent
 * in the application. So it is recommended that it is not allocated in the
 * stack.
 * @param[in]  stateMemorySize  Size of the memory pointed by pStateMemory
 *                              This must be at least
 *                              #ADI_ADC_STATE_MEM_NUM_BYTES bytes.
 *                              #adi_adc_SetConfig can also be used to set the
 *                              configuration at a later point.
 *
 * @return  #ADI_ADC_STATUS_SUCCESS \n
 * #ADI_ADC_STATUS_NULL_PTR \n
 * #ADI_ADC_STATUS_INSUFFICIENT_STATE_MEMORY \n
 */
ADI_ADC_STATUS adi_adc_Create(ADI_ADC_HANDLE *phAdc, void *pStateMemory, uint32_t stateMemorySize);

/**
 * @brief Function to initialize the ADC service with the given configurations.
 *        Before calling this API, its recommended to call #adi_adc_Create to create the instance
 *        and also to populate configurations #ADI_ADC_CONFIG from application accordingly.
 * @details    This API can be called after adi_adc_create
 * @param[in] hAdc   - ADC Service handle
 * @param[in] pConfig - ADC configuration parameters.
 *
 * @return  #ADI_ADC_STATUS_SUCCESS \n
 * #ADI_ADC_STATUS_NULL_PTR \n
 * #ADI_ADC_STATUS_INVALID_NUM_ADC \n
 * #ADI_ADC_STATUS_INVALID_ADC_TYPE \n
 */
ADI_ADC_STATUS adi_adc_Init(ADI_ADC_HANDLE hAdc, ADI_ADC_CONFIG *pConfig);

/**
 * @brief Function to reconfigure ADC service.
 * @details    This API can be called whenever the user wants to reconfigure
 *             ADC service. This API will not reset the internal states of the
 *             ADC service.
 *
 * @param[in] hAdc   - ADC Service handle
 * @param[in] pConfig - Pointer to the configuration structure.
 * @return  #ADI_ADC_STATUS_SUCCESS \n
 * #ADI_ADC_STATUS_NULL_PTR \n
 * #ADI_ADC_STATUS_INVALID_NUM_ADC \n
 * #ADI_ADC_STATUS_INVALID_ADC_TYPE \n
 */
ADI_ADC_STATUS adi_adc_SetConfig(ADI_ADC_HANDLE hAdc, ADI_ADC_CONFIG *pConfig);

/**
 * @brief Function to  get configuration of the library
 * @details    This API can be called whenever the user wants to get run-time
 *             configurations of the ADC service.
 *
 * @param[in] hAdc   - ADC Service handle
 * @param[out]  pConfig - Pointer to the configuration structure.
 * @return  #ADI_ADC_STATUS_SUCCESS \n
 * #ADI_ADC_STATUS_NULL_PTR \n
 */
ADI_ADC_STATUS adi_adc_GetConfig(ADI_ADC_HANDLE hAdc, ADI_ADC_CONFIG *pConfig);

/**
 * @brief Function to reset the frame buffers of ADC Service.
 *
 * @param[in] hAdc   - ADC Service handle
 * @return  #ADI_ADC_STATUS_SUCCESS \n
 * #ADI_ADC_STATUS_NULL_PTR \n
 */
ADI_ADC_STATUS adi_adc_ResetFrameBuffer(ADI_ADC_HANDLE hAdc);

/** @} */

/** @defgroup   ADC_CONFIG ADC configuration functions.
 * @brief This section covers APIs required to start the communication with ADC via SPI.
 * All the APIs return enumeration codes in #ADI_ADC_STATUS. Refer to #ADI_ADC_STATUS for detailed
 * documentation on return codes
 * @{
 */

/**
 * @brief Function to configure the specified ADC to provide CLKOUT instead of DREADY in a daisy
 * chain configuration.
 *
 * @param[in] hAdc   - ADC Service handle.
 * @param[in] adcIdx	- Adc index which can take values from 0 to 'numAdc' in #ADI_ADC_CONFIG.
 *                      It cannot be -1.
 *
 * @return  #ADI_ADC_STATUS_SUCCESS \n
 * #ADI_ADC_STATUS_NULL_PTR \n
 * #ADI_ADC_STATUS_CRC_CALC_FAILED \n
 */
ADI_ADC_STATUS adi_adc_EnableClockOut(ADI_ADC_HANDLE hAdc, uint8_t adcIdx);

/**
 * @brief Function to configure the ADCs. This API writes different ADC registers and brings the
 *        ADCs to the desired operational state. This API wil reset the data stored in DSP region.
 *        Therefore call this API before calling the APIs in @ref ADI_ADC_DSP
 *
 * @param[in] hAdc   - ADC Service handle
 * @param[in] pConfigReg  -  Pointer to configuration registers. Default configuration is applied if
 * NULL
 *
 * @return  #ADI_ADC_STATUS_SUCCESS \n
 * #ADI_ADC_STATUS_NULL_PTR \n
 * #ADI_ADC_STATUS_CRC_CALC_FAILED \n
 * #ADI_ADC_STATUS_TRANSCEIVE_FAILED \n
 * #ADI_ADC_STATUS_CRC_ERROR \n
 */
ADI_ADC_STATUS adi_adc_ConfigureAdcs(ADI_ADC_HANDLE hAdc, ADI_ADC_CONFIG_REGISTERS *pConfigReg);

/** @} */

/** @defgroup   ADC_READ_WRITE ADC Service read/write functions.
 * @brief This section covers APIs required to read/write registers.
 * All the APIs return enumeration codes in #ADI_ADC_STATUS. Refer to #ADI_ADC_STATUS for detailed
 * documentation on return codes
 * @{
 */

/**
 * @brief Function to form the WRITE command and sends data to the ADC over SPI.
 *        The function should be used to write to ADC register when ADCs are not collecting samples
 * using #adi_adc_CollectSamples. The function sends two SPI packets to the ADCS. First one is a
 * WRITE command with the address provided and second one is a read to status2 register. The
 * function is blocking and will return only after command is written to ADC.
 *
 * @param[in] hAdc   - ADC Service handle
 * @param[in] address - ADC Register address.
 * @param[in] value - Value to be written.
 * @param[in] adcIdx - ADC Index to which the command has to be sent.
 *                     This value ranges from 0 to numAdc-1.
 *                     This value will be -1 if command has to be set to
 *                     all ADC's.
 *
 * @return  #ADI_ADC_STATUS_SUCCESS \n
 * #ADI_ADC_STATUS_NULL_PTR \n
 * #ADI_ADC_STATUS_INVALID_ADC_INDEX \n
 * #ADI_ADC_STATUS_CRC_CALC_FAILED \n
 * #ADI_ADC_STATUS_TRANSCEIVE_FAILED \n
 * #ADI_ADC_STATUS_CRC_ERROR \n
 */
ADI_ADC_STATUS adi_adc_WriteRegister(ADI_ADC_HANDLE hAdc, uint16_t address, uint8_t value,
                                     int8_t adcIdx);

/**
 * @brief Function to form the READ command and sends to the ADC over SPI. The function returns
 * the ADC data read from the given register. The function should be used to READ from ADC register
 * when ADCs are not collecting samples using #adi_adc_CollectSamples.
 *        The function sends two SPI packets to the ADCS. First one is a READ command with the
 * address provided and second one is a read to status2 register to get back the register value. The
 * function is blocking and will return only after response is received from ADC.
 *
 * @param[in] hAdc   - ADC Service handle
 * @param[in] address - ADC Register address.
 * @param[in] adcIdx  - ADC Index to which the command has to be sent.
 *                      This value ranges from 0 to numAdc-1.
 *                      This value will be -1 if command has to be sent to
 *                      all ADC's
 * @param[out] pBuffer - Destination buffer to collect the register value.
 *                       This buffer should be of 2 bytes if
 *                       0 <= adcIdx < number of ADC.
 *                       The buffer should have a size of (2*number of ADC)
 *                       bytes, if adcIdx is -1.
 *                       The response is of the form [value at addr+1, value at addr, ...]
 * @param[out] pNumBytes - Number of bytes transferred to the destination
 *                         register.
 * @return  #ADI_ADC_STATUS_SUCCESS \n
 * #ADI_ADC_STATUS_NULL_PTR \n
 * #ADI_ADC_STATUS_INVALID_ADC_INDEX \n
 * #ADI_ADC_STATUS_CRC_CALC_FAILED \n
 * #ADI_ADC_STATUS_TRANSCEIVE_FAILED \n
 * #ADI_ADC_STATUS_CRC_ERROR \n
 */
ADI_ADC_STATUS adi_adc_ReadRegister(ADI_ADC_HANDLE hAdc, uint16_t address, int8_t adcIdx,
                                    uint8_t *pBuffer, uint32_t *pNumBytes);

/**
 * @brief Function to issue a WRITE command when the ADCs are collecting samples.
 *        This function updates next command sent to the ADC by #adi_adc_CollectSamples to write
 * register to the given address.
 *
 * @param[in] hAdc   - ADC Service handle
 * @param[in] address - ADC Register address.
 * @param[in] value - Value to be written.
 * @param[in] adcIdx - ADC Index to which the command has to be sent.
 *                     This value ranges from 0 to numAdc-1.
 *                     This value will be -1 if command has to be set to
 *                     all ADC's.
 *
 * @return  #ADI_ADC_STATUS_SUCCESS \n
 * #ADI_ADC_STATUS_NULL_PTR \n
 * #ADI_ADC_STATUS_ALIGN_IN_PROGRESS \n
 * #ADI_ADC_STATUS_INVALID_ADC_INDEX \n
 * #ADI_ADC_STATUS_CRC_CALC_FAILED \n
 */
ADI_ADC_STATUS adi_adc_AssembleWriteRegister(ADI_ADC_HANDLE hAdc, uint16_t address, uint8_t value,
                                             int8_t adcIdx);

/**
 * @brief Function to issue a READ command when the ADCs are collecting samples.
 *        This function updates next command sent to the ADC by #adi_adc_CollectSamples to read
 * register from the given address. The callback function #ADI_ADC_CONFIG.pfCallback will be invoked
 * when the read data is available. Application can call #adi_adc_GetLastRegister function to
 * get the regsiter read.
 *
 * @param[in] hAdc   - ADC Service handle
 * @param[in] address - ADC Register address.
 * @param[in] adcIdx - ADC Index to which the command has to be sent.
 *                     This value ranges from 0 to numAdc-1.
 *                     This value will be -1 if command has to be set to
 *                     all ADC's.
 *
 * @return  #ADI_ADC_STATUS_SUCCESS \n
 * #ADI_ADC_STATUS_NULL_PTR \n
 * #ADI_ADC_STATUS_ALIGN_IN_PROGRESS \n
 * #ADI_ADC_STATUS_INVALID_ADC_INDEX \n
 * #ADI_ADC_STATUS_CRC_CALC_FAILED \n
 */
ADI_ADC_STATUS adi_adc_AssembleReadRegister(ADI_ADC_HANDLE hAdc, uint16_t address, int8_t adcIdx);

/**
 * @brief Function to get the data received by the previous READ command issued through
 * #adi_adc_AssembleReadRegister.
 *
 * @param[in] hAdc   - ADC Service handle
 * @param[in] adcIdx   - ADC Index.
 * @param[out] pBuffer - Destination buffer to collect the register value.
 *                       This buffer should be of 2 bytes if
 *                       0 <= adcIdx < number of ADC.
 *                       The buffer should have a size of (2*number of ADC)
 *                       bytes, if adcIdx is -1.
 * @param[out] pNumBytes - Number of bytes transferred to the destination
 *                         register.
 *
 * @return  #ADI_ADC_STATUS_SUCCESS \n
 * #ADI_ADC_STATUS_NULL_PTR \n
 * #ADI_ADC_STATUS_LAST_REGISTER_NOT_READY \n
 * #ADI_ADC_STATUS_INVALID_ADC_INDEX \n
 */
ADI_ADC_STATUS adi_adc_GetLastRegister(ADI_ADC_HANDLE hAdc, int8_t adcIdx, uint8_t *pBuffer,
                                       uint32_t *pNumBytes);

/** @} */

/** @defgroup   ADC_SAMPLES ADC Service sample collection functions.
 * @brief This section covers APIs required to collect samples.
 * All the APIs return enumeration codes in #ADI_ADC_STATUS. Refer to #ADI_ADC_STATUS for detailed
 * documentation on return codes
 * @{
 */

/**
 * @brief This function issues sample read command and also collects the response of previous
 * command. Once the response is collected, CRC of the received frame is verified against the
 * expected value. Note that the function is non-blocking. #ADI_ADC_CONFIG.pfCallback will be
 * invoked when a block of samples are ready. Use #adi_adc_ReadBlock function to read the collected
 * samples.
 * @param[in] hAdc   - ADC Service handle
 * @param[in] timestamp - Timestamp associated with each sample.
 * @return  #ADI_ADC_STATUS_SUCCESS \n
 * #ADI_ADC_STATUS_NULL_PTR \n
 * #ADI_ADC_STATUS_PREV_CMD_RUNNING \n
 * #ADI_ADC_STATUS_BUFFER_OVERFLOW \n
 */
ADI_ADC_STATUS adi_adc_CollectSamples(ADI_ADC_HANDLE hAdc, uint32_t timestamp);

/**
 * @brief Function to validate the samples.
 * @param[in] hAdc   - ADC Service handle
 *
 */
ADI_ADC_STATUS adi_adc_ValidateSamples(ADI_ADC_HANDLE hAdc);

/**
 * @brief Function to read a block of samples collected by #adi_adc_CollectSamples.
 *        The number of samples that can be collected can be controlled by
 * #ADI_ADC_CONFIG.numSamplesInBlock
 *
 * @param[in] hAdc   - ADC Service handle
 * @param[out] pBuffer          - Pointer to buffer to collect the data.
 * @param[out] pAdcStatusOutput - Pointer to status output of the responses
 *                                read from the ADCs. It will contain the STATUS0-2
 *                                of the last frame read from each ADC in the given
 *                                block of frames. It will also contain the logical
 *                                OR performed for each frame in the block corresponding
 *                                to each ADC.
 *
 * @return  #ADI_ADC_STATUS_SUCCESS \n
 * #ADI_ADC_STATUS_NULL_PTR \n
 * #ADI_ADC_STATUS_NO_DATA \n
 */
ADI_ADC_STATUS adi_adc_ReadBlock(ADI_ADC_HANDLE hAdc, int32_t *pBuffer,
                                 ADI_ADC_STATUS_OUTPUT *pAdcStatusOutput);

/** @} */

/** @defgroup   ADC_ALIGN ADC Service align functions.
 * @brief This section covers APIs required to perform align.
 * All the APIs return enumeration codes in #ADI_ADC_STATUS. Refer to #ADI_ADC_STATUS for detailed
 * documentation on return codes
 * @{
 */

/**
 * @brief Function to issue an ADC Align operation.
 *        This function can be called to issue and ADC ALIGN when sample capture is not in progress.
 * The function is blocking and will return only after all steps involved in an ADC Align is
 * completed.
 * @param[in] hAdc   - ADC Service handle
 *
 * @return  #ADI_ADC_STATUS_SUCCESS \n
 * #ADI_ADC_STATUS_NULL_PTR \n
 * #ADI_ADC_STATUS_INVALID_ADC_INDEX \n
 * #ADI_ADC_STATUS_CRC_CALC_FAILED \n
 * #ADI_ADC_STATUS_TRANSCEIVE_FAILED \n
 * #ADI_ADC_STATUS_CRC_ERROR \n
 */
ADI_ADC_STATUS adi_adc_Align(ADI_ADC_HANDLE hAdc);

/**
 * @brief Function to start ADC Align Process.
 * The the Align process takes 6 dreadys and #ADI_ADC_CONFIG.pfCallback will be invoked when Align
 * is completed. Note that this API will will disturb the Dready and sample collection. Please Refer
 * to appropriate ADC datasheet for more details.
 *
 * @param[in] hAdc   - ADC Service handle
 * @return  #ADI_ADC_STATUS_SUCCESS \n
 * #ADI_ADC_STATUS_NULL_PTR \n
 * #ADI_ADC_STATUS_ALIGN_IN_PROGRESS \n
 */
ADI_ADC_STATUS adi_adc_StartAlign(ADI_ADC_HANDLE hAdc);

/** @} */

/** @defgroup   ADC_STATUS Status Functions.
 * @brief This section covers APIs that provide the required status.
 * All the APIs return enumeration codes in #ADI_ADC_STATUS. Refer to #ADI_ADC_STATUS for detailed
 * documentation on return codes
 * @{
 */

/**
 * @brief Get the error bits of ADC status registers after adi_adc_Configure and
 * adi_adc_ReadRegister.
 * @param[in] hAdc   - ADC Service handle
 * @param[out] pAdcStatusOutput - Pointer to status output of the responses
 *                                read from the ADCs. It will contain the STATUS0-1
 *                                read from each ADC. It will also contain the CRC calculation
 *                                performed on each ADC.
 * @return  #ADI_ADC_STATUS_SUCCESS \n
 * #ADI_ADC_STATUS_NULL_PTR \n
 *
 */
ADI_ADC_STATUS adi_adc_GetLastFrameStatus(ADI_ADC_HANDLE hAdc,
                                          ADI_ADC_STATUS_OUTPUT *pAdcStatusOutput);

/**
 * @brief Function to get run status.
 * @param[in] hAdc   - ADC Service handle
 * @param[out] pData   - pointer to output
 * @return  #ADI_ADC_STATUS_SUCCESS \n
 * #ADI_ADC_STATUS_NULL_PTR \n
 */
ADI_ADC_STATUS adi_adc_GetRunData(ADI_ADC_HANDLE hAdc, ADI_ADC_RUN_DATA *pData);

/** @} */

/** @defgroup   ADC_UTILS Utility Functions.
 * @brief This section covers utility APIs.
 * All the APIs return enumeration codes in #ADI_ADC_STATUS. Refer to #ADI_ADC_STATUS for detailed
 * documentation on return codes
 * @{
 */

/**
 * @brief Function to set the DREADY sampling rate of ADC.
 *
 * @param[in] hAdc   - ADC Service handle
 * @param[in] pConfigReg   - Pounter to user config registers.
 *
 * @return  #ADI_ADC_STATUS_SUCCESS \n
 * #ADI_ADC_STATUS_NULL_PTR \n
 * #ADI_ADC_STATUS_INVALID_ADC_INDEX \n
 * #ADI_ADC_STATUS_CRC_CALC_FAILED \n
 * #ADI_ADC_STATUS_TRANSCEIVE_FAILED \n
 */
ADI_ADC_STATUS adi_adc_SetSamplingRate(ADI_ADC_HANDLE hAdc, ADI_ADC_CONFIG_REGISTERS *pConfigReg);

/**
 * @brief Function to get the ADC index and channel number for a given slot number.
 *
 * @param[in] hAdc      - ADC Service handle
 * @param[in] slotNum   - Slot number.
 * @param[out] pAdcIdx  - Pointer to store the ADC index.
 * @param[out] pChanNum - Pointer to store the channel number for the ADC.
 *
 * @return  #ADI_ADC_STATUS_SUCCESS \n
 * #ADI_ADC_STATUS_INCORRECT_SLOT_CONFIG \n
 * #ADI_ADC_STATUS_NULL_PTR \n
 */
ADI_ADC_STATUS adi_adc_GetAdcIdxAndChan(ADI_ADC_HANDLE hAdc, uint8_t slotNum, uint8_t *pAdcIdx,
                                        uint8_t *pChanNum);

/**
 * @brief Function to set the ADC frame response frame format.
 *
 * @param[in] hAdc      - ADC Service handle
 * @param[in] format    - SPI response frame format.
 *
 * @return  #ADI_ADC_STATUS_SUCCESS \n
 * #ADI_ADC_STATUS_INCORRECT_FRAME_FORMAT_SET \n
 * #ADI_ADC_STATUS_NULL_PTR \n
 */
ADI_ADC_STATUS adi_adc_SetAdcFrameFormat(ADI_ADC_HANDLE hAdc, uint8_t format);

/**
 * @brief Function to set the integer sample delay for each channel.
 *
 * @param[in] hAdc      - ADC Service handle
 * @param[in] pIntegerDelay - Pointer to integer delay for each channel.
 * @param[in] pChanIdx - Pointer to ADC channel indices.
 * @param[in] numChan - Number of channel registers to be written.
 * @param[in] adcIdx - ADC Index to which the command has to be sent.
 *                     This value ranges from 0 to numAdc-1.
 *
 *
 * @return  #ADI_ADC_STATUS_SUCCESS \n
 * #ADI_ADC_STATUS_INVALID_SAMPLE_DELAY \n
 * #ADI_ADC_STATUS_INVALID_CHANNEL_INDEX \n
 * #ADI_ADC_STATUS_NULL_PTR \n
 */
ADI_ADC_STATUS adi_adc_SetIntegerSampleDelay(ADI_ADC_HANDLE hAdc, uint8_t *pIntegerDelay,
                                             uint8_t *pChanIdx, int8_t numChan, int8_t adcIdx);

/**
 * @brief Function to assign the stream mode to output structure based on adc type.
 *
 * @param[in] streamMode   - Stream mode.
 * @param[in] numAdc   - Number of ADCs.
 * @param[in] pAdcType   - Pointer ADC type.
 * @param[out] pConfigReg   - Pointer to configuration registers that stores the stream mode.
 *
 * @return  #ADI_ADC_STATUS_SUCCESS \n
 * #ADI_ADC_STATUS_INVALID_NUM_ADC \n
 * #ADI_ADC_STATUS_INVALID_ADC_TYPE \n
 * #ADI_ADC_STATUS_INVALID_STREAM_MODE \n
 */
ADI_ADC_STATUS adi_adcutil_PopulateStreamMode(ADI_ADC_STREAM_MODE streamMode, uint8_t numAdc,
                                              ADI_ADC_TYPE *pAdcType,
                                              ADI_ADC_CONFIG_REGISTERS *pConfigReg);

/**
 * @brief Function to assign the sampling rate to output structure based on adc type.
 *
 * @param[in] clkIn   - Input clk to ADCs. Applicable for ADEMA12x
 * @param[in] samplingRate   - Sampling rate.
 * @param[in] decimateBy2   - Decimate by 2. Value can be 0 or 1. Applicable for ADEMA12x
 * @param[in] numAdc   - Number of ADCs.
 * @param[in] pAdcType   - Pointer ADC type.
 * @param[out] pConfigReg   - Pointer to configuration registers that stores the sampling rate.
 *
 * @return  #ADI_ADC_STATUS_SUCCESS \n
 * #ADI_ADC_STATUS_INVALID_NUM_ADC \n
 * #ADI_ADC_STATUS_INVALID_ADC_TYPE \n
 * #ADI_ADC_STATUS_INVALID_SAMPLING_RATE \n
 */
ADI_ADC_STATUS adi_adcutil_PopulateSamplingRate(uint32_t clkIn, uint32_t samplingRate,
                                                uint8_t decimateBy2, uint8_t numAdc,
                                                ADI_ADC_TYPE *pAdcType,
                                                ADI_ADC_CONFIG_REGISTERS *pConfigReg);

/**
 * @brief Function to extract specific channel samples.
 *
 * @param[in] pSrc   - Pointer to sample's buffer.
 * @param[in] numSamples   - Number of samples.
 * @param[in] numChannels  - Number of channels.
 * @param[in] channelMask  - Channel mask.
 * @param[out] pDst  - Destination buffer.
 *
 * @return  Number of samples extracted.
 */
uint32_t adi_adcutil_ExtractChannel(int32_t *pSrc, uint32_t numSamples, uint32_t numChannels,
                                    uint32_t channelMask, int32_t *pDst);

/**
 * @brief Converts raw ADC codes to floating-point sample values.
 *
 * This function takes raw ADC codes from the input buffer and scales them
 * down by corresponding ADC Full Scale Code to floating-point sample values,
 * which are stored in the output buffer. The number of frames to process is
 * specified by the caller.
 *
 * @param[in]  hAdc       Handle to ADC Service Instance.
 * @param[in]  pInBuff    Pointer to the input buffer containing raw ADC codes.
 * @param[out] pOutBuff   Pointer to the output buffer where converted
 *                        floating-point sample values will be stored.
 * @param[in]  numFrames  Number of frames to process.
 *
 * @return ADI_ADC_STATUS Status of the operation. Returns success or an
 *                        appropriate error code.
 */
ADI_ADC_STATUS adi_adc_ConvertCodesToSamples(ADI_ADC_HANDLE hAdc, int32_t *pInBuff, float *pOutBuff,
                                             uint32_t numFrames);

/**
 * @brief Function to get the number of channels per ADC.
 *
 * This function retrieves the number of channels per ADC and puts them in pNumChannels.
 *
 * @param[in]  hAdc         - ADC Service handle.
 * @param[out] pNumChannels - Pointer to a buffer to store the number of channels.
 *
 * @return  #ADI_ADC_STATUS_SUCCESS \n
 *          #ADI_ADC_STATUS_NULL_PTR \n
 */
ADI_ADC_STATUS adi_adc_GetChannelsPerAdc(ADI_ADC_HANDLE hAdc, uint8_t *pNumChannels);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __ADI_ADC_H__ */
/** @} */
