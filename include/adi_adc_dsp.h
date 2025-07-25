/******************************************************************************
 Copyright (c) 2025  Analog Devices Inc.
******************************************************************************/

/**
 * @file adi_adc_dsp.h
 * @addtogroup ADI_ADC_DSP ADC Service DSP.
 * @{
 * @brief Functions and definitions required to initialise and configure ADC DSP.
 */

#ifndef __ADI_ADC_DSP_H__
#define __ADI_ADC_DSP_H__

/*============= I N C L U D E S =============*/
#include "adi_adc.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup ADCDSPMACROS Macros used in datapath configuration.
 * @brief This section contains the macros used in datapath configuration.
 * @{
 */

/** Number of coefficients for SCF */
#define ADEMA12x_NUM_SCF_COEFFS 3
/** Number of coefficients for LPF */
#define ADEMA12x_NUM_LPF_COEFFS 18
/** Number of numerator coefficients for HPF */
#define ADEMA12x_NUM_HPF_COEFFS_NUMERATOR 3
/** Number of denominator coefficients for HPF */
#define ADEMA12x_NUM_HPF_COEFFS_DENOMINATOR 2
/** Number of coefficients for Compensation Filter */
#define ADEMA12x_NUM_COMP_COEFFS 7

/** @} */

/** @defgroup   ADCDSPSTRUCT ADC DSP Structures
 *  @brief This section covers various structures used in ADC service DSP.
 * @{
 */

/**
 * Structure to hold the datapath configuration.
 */
typedef struct
{
    /** Bit to enable channel Electrostatic Compensation */
    uint8_t gainOffsetEn : 1;
    /** Bit to enable channel Sensor Compensation Filter */
    uint8_t scfEn : 1;
    /** Bit to enable channel High Pass Filter */
    uint8_t hpfEn : 1;
    /** Bit to enable channel Compensation Filter */
    uint8_t compFiltEn : 1;
    /** Bit to select channel Compensation Filter configuration
    0: Sinc Droop;	1: External BLF and Sinc Droop */
    uint8_t compFiltCfg : 1;
    /** Bit to enable channel Low Pass Filter */
    uint8_t lpfEn : 1;
    /** Bit to enable channel All Pass Filter */
    uint8_t allPassEn : 1;
    /** Reserved Bit */
    uint8_t reserved : 1;
} ADI_ADC_CHAN_DATAPATH_CONFIG;

/**
 * Structure to hold the XT Aggressor configuration.
 */
typedef struct
{
    /** Bit to choose the channel number of the aggressor which will be added to the channel for XT
     * compensation */
    uint8_t xtAggressor : 3;
    /** Reserved Bit */
    uint8_t reserved1 : 2;
    /** Bit to enable the XT Compensation */
    uint8_t xtCompEn : 1;
    /** Reserved Bit */
    uint8_t reserved2 : 2;
} ADI_ADC_CHAN_XT_AGGRESSOR;

/**
 * @brief Structure to store ADC DSP registers.
 */
typedef struct
{
    /** gain */
    float gain[APP_CFG_MAX_NUM_CHANNELS];
    /** channel offset */
    int32_t offset[APP_CFG_MAX_NUM_CHANNELS];
    /** shift */
    uint8_t shift[APP_CFG_MAX_NUM_CHANNELS];
    /** XT gain */
    float xtGain[APP_CFG_MAX_NUM_CHANNELS];
    /** XT aggressor */
    ADI_ADC_CHAN_XT_AGGRESSOR xtAggressor[APP_CFG_MAX_NUM_CHANNELS];

} ADI_ADC_DSP_CHANNEL_PARAMS;

/**
 * Structure to hold the ADC DSP datarate parameters.
 */
typedef struct
{
    /** Decimation by 2 */
    uint8_t decimationX2;
    /** Clock prescalar */
    uint8_t clkPreScalar;
    /** Decimation rate */
    uint8_t decimationRate;

} ADI_ADC_DSP_DATARATE;

/**
 * Structure to hold the alpha values for the ADC DSP datapath.
 */
typedef struct
{
    /** Channel 0 */
    uint8_t chan0;
    /** Channel 1 */
    uint8_t chan1;
    /** Channel 2 */
    uint8_t chan2;
    /** Channel 3 */
    uint8_t chan3;
    /** Channel 4 */
    uint8_t chan4;
    /** Channel 5 */
    uint8_t chan5;
    /** Channel 6 */
    uint8_t chan6;
    /** Channel 7 */
    uint8_t chan7;

} ADI_ADC_DSP_DATAPATH_ALPHA;

/**
 * Structure to hold the ADC DSP datapath parameters.
 */
typedef struct
{
    /** datarate register*/
    ADI_ADC_DSP_DATARATE datarate;
    /** alpha register*/
    ADI_ADC_DSP_DATAPATH_ALPHA alpha;
    /** data path configuration */
    ADI_ADC_CHAN_DATAPATH_CONFIG dataPathConfig[APP_CFG_MAX_NUM_CHANNELS];
    /** phase offset */
    float phaseOffset[APP_CFG_MAX_NUM_CHANNELS];
} ADI_ADC_DSP_DATAPATH_PARAMS;

/** @} */

/** @defgroup ADC_DATAPATH ADC datapath configuration functions.
 * @brief This section covers APIs that writes to the datapath configuration registers.
 * Calling the "set" API's will reset the values stored in the DSP memory region. Therfore, call
 * those API's before calling the "set" API's in @ref ADC_DSP All the APIs return enumeration codes
 * in #ADI_ADC_STATUS. Refer to #ADI_ADC_STATUS for detailed documentation on return codes.
 * @{
 */
/**
 * @brief Function to write to DATAPATH_CONFIG_CHx ADC registers.
 *        The function should be used to enable ADC DSP datapath functionality.
 *
 * @param[in] hAdc   - ADC Service handle
 * @param[in] pDatapathEnConfig - Datapath enable configuration value.
 * @param[in] pChanIdx - Pointer to ADC channel indices.
 * @param[in] numChan - Number of channel registers to be written.
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
 * #ADI_ADC_STATUS_DATAPATH_CONFIGURATION_FAILED \n
 */
ADI_ADC_STATUS adi_adc_EnableDatapathConfig(ADI_ADC_HANDLE hAdc,
                                            ADI_ADC_CHAN_DATAPATH_CONFIG *pDatapathEnConfig,
                                            uint8_t *pChanIdx, int8_t numChan, int8_t adcIdx);

/**
 * @brief Function to set the channel based ADC DSP registers. Writes are dones to the registers
 * mentioned in Table24 in the datasheet(excluded the filter coefficients registers).
 *
 * @param[in] hAdc   - ADC Service handle
 * @param[in] pDspRegisters - Pointer to ADC DSP channel parameters.
 * @param[in] pChanIdx - Pointer to ADC channel indices.
 * @param[in] numChannel - Number of channel registers to be written.
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
 * #ADI_ADC_STATUS_DATAPATH_CONFIGURATION_FAILED \n
 *
 */
ADI_ADC_STATUS adi_adc_SetChannelParams(ADI_ADC_HANDLE hAdc,
                                        ADI_ADC_DSP_CHANNEL_PARAMS *pDspRegisters,
                                        uint8_t *pChanIdx, uint8_t numChannel, int8_t adcIdx);

/**
 * @brief Function to get the channel based ADC DSP registers. Reads are dones to the registers
 * mentioned in Table24 in the datasheet(excluded the filter coefficients registers).
 *
 * @param[in] hAdc   - ADC Service handle
 *  @param[in] pChanIdx - Pointer to ADC channel indices.
 * @param[in] numChannel - Number of channel registers to be written.
 * @param[in] adcIdx - ADC Index to which the command has to be sent.
 *                     This value ranges from 0 to numAdc-1.
 * @param[out] pDspRegisters - Pointer to ADC DSP channel parameters.
 *
 * @return  #ADI_ADC_STATUS_SUCCESS \n
 * #ADI_ADC_STATUS_NULL_PTR \n
 * #ADI_ADC_STATUS_INVALID_ADC_INDEX \n
 * #ADI_ADC_STATUS_CRC_CALC_FAILED \n
 * #ADI_ADC_STATUS_TRANSCEIVE_FAILED \n
 * #ADI_ADC_STATUS_CRC_ERROR \n
 * #ADI_ADC_STATUS_DATAPATH_CONFIGURATION_FAILED \n
 *
 */

ADI_ADC_STATUS adi_adc_GetChannelParams(ADI_ADC_HANDLE hAdc, uint8_t *pChanIdx, uint8_t numChannel,
                                        int8_t adcIdx, ADI_ADC_DSP_CHANNEL_PARAMS *pDspRegisters);

/**
 * @brief Function to set the channel based ADC DSP registers. Writes are dones to the registers
 * mentioned in Table23 in the datasheet.
 *
 * @param[in] hAdc   - ADC Service handle
 * @param[in] pDspRegisters - Pointer to ADC DSP channel parameters.
 * @param[in] pChanIdx - Pointer to ADC channel indices.
 * @param[in] numChannel - Number of channel registers to be written.
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
 * #ADI_ADC_STATUS_DATAPATH_CONFIGURATION_FAILED \n
 *
 */

ADI_ADC_STATUS adi_adc_SetDatapathParams(ADI_ADC_HANDLE hAdc,
                                         ADI_ADC_DSP_DATAPATH_PARAMS *pDspRegisters,
                                         uint8_t *pChanIdx, uint8_t numChannel, int8_t adcIdx);

/**
 * @brief Function to get the channel based ADC DSP registers. Reads are dones to the registers
 * mentioned in Table23 in the datasheet.
 *
 * @param[in] hAdc   - ADC Service handle
 * @param[in] pChanIdx - Pointer to ADC channel indices.
 * @param[in] numChannel - Number of channel registers to be written.
 * @param[in] adcIdx - ADC Index to which the command has to be sent.
 *                     This value ranges from 0 to numAdc-1.
 * @param[out] pDspRegisters - Pointer to ADC DSP channel parameters.
 *
 * @return  #ADI_ADC_STATUS_SUCCESS \n
 * #ADI_ADC_STATUS_NULL_PTR \n
 * #ADI_ADC_STATUS_INVALID_ADC_INDEX \n
 * #ADI_ADC_STATUS_CRC_CALC_FAILED \n
 * #ADI_ADC_STATUS_TRANSCEIVE_FAILED \n
 * #ADI_ADC_STATUS_CRC_ERROR \n
 * #ADI_ADC_STATUS_DATAPATH_CONFIGURATION_FAILED \n
 *
 */
ADI_ADC_STATUS adi_adc_GetDatapathParams(ADI_ADC_HANDLE hAdc, uint8_t *pChanIdx, uint8_t numChannel,
                                         int8_t adcIdx, ADI_ADC_DSP_DATAPATH_PARAMS *pDspRegisters);

/**
 * @brief Function to set the channel phase offset registers of ADC.
 *        The function should be used to write to phase offset registers of ADC DSP.
 *
 * @param[in] hAdc   - ADC Service handle
 * @param[in] pPhOffsetVal - Phase offset values of all channels.
 * @param[in] pChanIdx - Pointer to ADC channel indices.
 * @param[in] numChan - Number of channel registers to be written.
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
 * ADI_ADC_STATUS_DATAPATH_CONFIGURATION_FAILED \n
 */
ADI_ADC_STATUS adi_adc_SetChannelPhaseOffset(ADI_ADC_HANDLE hAdc, float *pPhOffsetVal,
                                             uint8_t *pChanIdx, int8_t numChan, int8_t adcIdx);

/**
 * @brief Function to set the ADC data rate registers.
 *        The function should be used to write to DATARATE register of ADC DSP.
 *
 * @param[in] hAdc   - ADC Service handle
 * @param[in] decimationRate - Decimation Rate.
 * @param[in] clkPreScaler - ADC Clock PreScaler.
 * @param[in] decimateX2 - Bit configuration set to add a decimate by 2 in the data path.
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
 * ADI_ADC_STATUS_DATAPATH_CONFIGURATION_FAILED \n
 */
ADI_ADC_STATUS adi_adc_SetDataRate(ADI_ADC_HANDLE hAdc, uint8_t decimationRate,
                                   uint8_t clkPreScaler, uint8_t decimateX2, int8_t adcIdx);

/**
 * @brief Function to set the channel datapath alpha registers of ADC.
 *        The function should be used to write to datapath alpha registers of ADC DSP.
 *
 * @param[in] hAdc   - ADC Service handle
 * @param[in] pAlphaVal - Pointer to channel datapaht alpha values.
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
 * ADI_ADC_STATUS_DATAPATH_CONFIGURATION_FAILED \n
 */
ADI_ADC_STATUS adi_adc_SetDatapathAlpha(ADI_ADC_HANDLE hAdc, uint8_t *pAlphaVal, int8_t adcIdx);

/**
 * @brief Function to get the channel datapath alpha registers of ADC.
 *        The function should be used to write to datapath alpha registers of ADC DSP.
 *
 * @param[in] hAdc   - ADC Service handle
 * @param[in] adcIdx - ADC Index to which the command has to be sent.
 *                     This value ranges from 0 to numAdc-1.
 * @param[out] pAlphaVal - Pointer to channel datapath alpha values.
 *
 * @return  #ADI_ADC_STATUS_SUCCESS \n
 * #ADI_ADC_STATUS_NULL_PTR \n
 * #ADI_ADC_STATUS_INVALID_ADC_INDEX \n
 * #ADI_ADC_STATUS_CRC_CALC_FAILED \n
 * #ADI_ADC_STATUS_TRANSCEIVE_FAILED \n
 * #ADI_ADC_STATUS_CRC_ERROR \n
 * ADI_ADC_STATUS_DATAPATH_CONFIGURATION_FAILED \n
 */
ADI_ADC_STATUS adi_adc_GetDatapathAlpha(ADI_ADC_HANDLE hAdc, int8_t adcIdx, uint8_t *pAlphaVal);

/**
 * @brief Function to get the channel phase offset registers from ADC.
 *        The function should be used to read to phase offset registers from ADC DSP.
 *
 * @param[in] hAdc   - ADC Service handle
 * @param[in] pChanIdx - Pointer to ADC channel indices.
 * @param[in] numChan - Number of channel registers to be written.
 * @param[in] adcIdx - ADC Index to which the command has to be sent.
 *                     This value ranges from 0 to numAdc-1.
 * @param[out] pPhOffsetVal - Phase offset values of all channels.
 *
 * @return  #ADI_ADC_STATUS_SUCCESS \n
 * #ADI_ADC_STATUS_NULL_PTR \n
 * #ADI_ADC_STATUS_INVALID_ADC_INDEX \n
 * #ADI_ADC_STATUS_CRC_CALC_FAILED \n
 * #ADI_ADC_STATUS_TRANSCEIVE_FAILED \n
 * #ADI_ADC_STATUS_CRC_ERROR \n
 * ADI_ADC_STATUS_DATAPATH_CONFIGURATION_FAILED \n
 */
ADI_ADC_STATUS adi_adc_GetChannelPhaseOffset(ADI_ADC_HANDLE hAdc, uint8_t *pChanIdx, int8_t numChan,
                                             int8_t adcIdx, float *pPhOffsetVal);
/** @} */

/** @defgroup   ADC_DSP ADC DSP APIs.
 * @brief This section covers APIs that writes the ADC DSP values.
 * Call the "set" API's mentioned in @ref ADC_DSP after calling the "set" API's in @ref
 * ADC_DATAPATH. All the APIs return enumeration codes in #ADI_ADC_STATUS. Refer to #ADI_ADC_STATUS
 * for detailed documentation on return codes.
 * @{
 */
/**
 * @brief Function to set the channel gain registers of ADC.
 *        The function should be used to write to gain registers of ADC DSP.
 *
 * @param[in] hAdc   - ADC Service handle
 * @param[in] pGainVal - Gain values of all channels.
 * @param[in] pChanIdx - Pointer to ADC channel indices.
 * @param[in] numChan - Number of channel registers to be written..
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
 * ADI_ADC_STATUS_DATAPATH_CONFIGURATION_FAILED \n
 */
ADI_ADC_STATUS adi_adc_SetChannelGain(ADI_ADC_HANDLE hAdc, float *pGainVal, uint8_t *pChanIdx,
                                      int8_t numChan, int8_t adcIdx);

/**
 * @brief Function to set the channel XT Gain registers of ADC.
 *        The function should be used to write to XT gain registers of ADC DSP.
 *
 * @param[in] hAdc   - ADC Service handle
 * @param[in] pXtGainVal - Xt Gain values of all channels.
 * @param[in] pChanIdx - Pointer to ADC channel indices.
 * @param[in] numChan - Number of channel registers to be written..
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
 * ADI_ADC_STATUS_DATAPATH_CONFIGURATION_FAILED \n
 */

ADI_ADC_STATUS adi_adc_SetChannelXtGain(ADI_ADC_HANDLE hAdc, float *pXtGainVal, uint8_t *pChanIdx,
                                        int8_t numChan, int8_t adcIdx);
/**
 * @brief Function to set the channel XT Aggressor registers of ADC.
 *        The function should be used to write to XT gain registers of ADC DSP.
 *
 * @param[in] hAdc   - ADC Service handle
 * @param[in] pXtAggrVal - Xt Aggressor values of all channels.
 * @param[in] pChanIdx - Pointer to ADC channel indices.
 * @param[in] numChan - Number of channel registers to be written..
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
 * ADI_ADC_STATUS_DATAPATH_CONFIGURATION_FAILED \n
 */
ADI_ADC_STATUS adi_adc_SetChannelXtAggressor(ADI_ADC_HANDLE hAdc,
                                             ADI_ADC_CHAN_XT_AGGRESSOR *pXtAggrVal,
                                             uint8_t *pChanIdx, int8_t numChan, int8_t adcIdx);
/**
 * @brief Function to set the channel offset registers of ADC.
 *        The function should be used to write to offset registers of ADC DSP.
 *
 * @param[in] hAdc   - ADC Service handle
 * @param[in] pOffsetVal - Offset values of all channels.
 * @param[in] pChanIdx - Pointer to ADC channel indices.
 * @param[in] numChan - Number of channel registers to be written.
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
 * ADI_ADC_STATUS_DATAPATH_CONFIGURATION_FAILED \n
 */
ADI_ADC_STATUS adi_adc_SetChannelOffset(ADI_ADC_HANDLE hAdc, int32_t *pOffsetVal, uint8_t *pChanIdx,
                                        int8_t numChan, int8_t adcIdx);

/**
 * @brief Function to set the 17th order FIR LPF coefficients of ADC DSP.
 *        The function should be used to modify LPF coefficients of ADC DSP.
 *
 *
 * @param[in] hAdc   - ADC Service handle
 * @param[in] pCoeffs - Pointer to coefficients in order [B0, B1, B2, ..., B17].
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
 * ADI_ADC_STATUS_DATAPATH_CONFIGURATION_FAILED \n
 */
ADI_ADC_STATUS adi_adc_SetLpfCoeff(ADI_ADC_HANDLE hAdc, float *pCoeffs, int8_t adcIdx);

/**
 * @brief Function to set the 1st order IIR SCF coefficients of ADC DSP.
 *        The function should be used to modify SCF coefficients of ADC DSP.
 *
 * @param[in] hAdc   - ADC Service handle
 * @param[in] pCoeffs - Array of pointers to coefficients. Each element of array points to
 * coefficients of one channel in order [B0, B1, A1].
 * @param[in] pChanIdx - Pointer to ADC channel indices.
 * @param[in] numChan - Number of channel registers to be written.
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
 * ADI_ADC_STATUS_DATAPATH_CONFIGURATION_FAILED \n
 */
ADI_ADC_STATUS adi_adc_SetScfCoeff(ADI_ADC_HANDLE hAdc, float *pCoeffs[], uint8_t *pChanIdx,
                                   int8_t numChan, int8_t adcIdx);

/**
 * @brief Function to set the 6th order FIR Compensation filter coefficients of ADC DSP.
 *        The function should be used to modify Compensation Filter coefficients of ADC DSP.
 *
 * @param[in] hAdc   - ADC Service handle
 * @param[in] pCoeffs - Array of pointers to coefficients. Each element of array points to
 * coefficients of one channel in order [B0, B1, B2, ..., B6].
 * @param[in] pChanIdx - Pointer to ADC channel indices.
 * @param[in] numChan - Number of channel registers to be written.
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
 * ADI_ADC_STATUS_DATAPATH_CONFIGURATION_FAILED \n
 */
ADI_ADC_STATUS adi_adc_SetCompCoeff(ADI_ADC_HANDLE hAdc, float *pCoeffs[], uint8_t *pChanIdx,
                                    int8_t numChan, int8_t adcIdx);
/**
 * @brief Function to set 2nd order biquad HPF coefficients of ADC DSP.
 *        The function should be used to modify HPF coefficients of ADC DSP.
 *
 * @param[in] hAdc  	 - ADC Service handle
 * @param[in] pNumCoeffs - Pointer to numerator coefficients in order [B0, B1, B2].
 * @param[in] pDenCoeffs - Pointer to denominator coefficients in order [A0, A1, A2].
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
 * ADI_ADC_STATUS_DATAPATH_CONFIGURATION_FAILED \n
 */
ADI_ADC_STATUS adi_adc_SetHpfCoeff(ADI_ADC_HANDLE hAdc, float *pNumCoeffs, double *pDenCoeffs,
                                   int8_t adcIdx);

/**
 * @brief Function to set the HPF coefficients of ADC DSP based on the selected filter bandwidth.
 *        The function should be used to modify DSP HPF cutoff frequency.
 *
 * @param[in] hAdc  	- ADC Service handle
 * @param[in] bwOption 	- Bandwidth configuration of HPF. Bandwidth = 10 / 2^bwOption.
 *                        Supported values are 0, 1, 2 and 3.
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
 * ADI_ADC_STATUS_DATAPATH_CONFIGURATION_FAILED \n
 */
ADI_ADC_STATUS adi_adc_SetHpfCutoff(ADI_ADC_HANDLE hAdc, uint32_t bwOption, int8_t adcIdx);
/**
 * @brief Function to set shift compensation values of ADC DSP.
 *        The function should be used to set shift compensation for all channels of ADC.
 *
 * @param[in] hAdc   - ADC Service handle
 * @param[in] pShiftVal - Pointer to shift values of all channels. Supported values are from 0 to 7.
 * @param[in] pChanIdx - Pointer to ADC channel indices.
 * @param[in] numChan - Number of channel registers to be written.
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
 * ADI_ADC_STATUS_DATAPATH_CONFIGURATION_FAILED \n
 */
ADI_ADC_STATUS adi_adc_SetShift(ADI_ADC_HANDLE hAdc, uint8_t *pShiftVal, uint8_t *pChanIdx,
                                int8_t numChan, int8_t adcIdx);

/**
 * @brief Function to get the channel gain registers of ADC.
 *        The function should be used to read to gain registers of ADC DSP.
 *
 * @param[in] hAdc   - ADC Service handle
 * @param[in] pChanIdx - Pointer to ADC channel indices.
 * @param[in] numChan - Number of channel registers to be written.
 * @param[in] adcIdx - ADC Index to which the command has to be sent.
 *                     This value ranges from 0 to numAdc-1.
 * @param[out] pGainVal - Pointer to gain values of all channels.
 *
 * @return  #ADI_ADC_STATUS_SUCCESS \n
 * #ADI_ADC_STATUS_NULL_PTR \n
 * #ADI_ADC_STATUS_INVALID_ADC_INDEX \n
 * #ADI_ADC_STATUS_CRC_CALC_FAILED \n
 * #ADI_ADC_STATUS_TRANSCEIVE_FAILED \n
 * #ADI_ADC_STATUS_CRC_ERROR \n
 * ADI_ADC_STATUS_DATAPATH_CONFIGURATION_FAILED \n
 */
ADI_ADC_STATUS adi_adc_GetChannelGain(ADI_ADC_HANDLE hAdc, uint8_t *pChanIdx, int8_t numChan,
                                      int8_t adcIdx, float *pGainVal);

/**
 * @brief Function to get the channel XT Aggressor registers of ADC.
 *        The function should be used to read to gain registers of ADC DSP.
 *
 * @param[in] hAdc   - ADC Service handle
 * @param[in] pChanIdx - Pointer to ADC channel indices.
 * @param[in] numChan - Number of channel registers to be written.
 * @param[in] adcIdx - ADC Index to which the command has to be sent.
 *                     This value ranges from 0 to numAdc-1.
 * @param[out] pXtAggrVal - Pointer to XT Aggressor values of all channels.
 *
 * @return  #ADI_ADC_STATUS_SUCCESS \n
 * #ADI_ADC_STATUS_NULL_PTR \n
 * #ADI_ADC_STATUS_INVALID_ADC_INDEX \n
 * #ADI_ADC_STATUS_CRC_CALC_FAILED \n
 * #ADI_ADC_STATUS_TRANSCEIVE_FAILED \n
 * #ADI_ADC_STATUS_CRC_ERROR \n
 * ADI_ADC_STATUS_DATAPATH_CONFIGURATION_FAILED \n
 */
ADI_ADC_STATUS adi_adc_GetChannelXtAggressor(ADI_ADC_HANDLE hAdc, uint8_t *pChanIdx, int8_t numChan,
                                             int8_t adcIdx, ADI_ADC_CHAN_XT_AGGRESSOR *pXtAggrVal);

/**
 * @brief Function to get the channel XT Gain registers of ADC.
 *        The function should be used to read to gain registers of ADC DSP.
 *
 * @param[in] hAdc   - ADC Service handle
 * @param[in] pChanIdx - Pointer to ADC channel indices.
 * @param[in] numChan - Number of channel registers to be written.
 * @param[in] adcIdx - ADC Index to which the command has to be sent.
 *                     This value ranges from 0 to numAdc-1.
 * @param[out] pXtGainVal - Pointer to XT Gain values of all channels.
 *
 * @return  #ADI_ADC_STATUS_SUCCESS \n
 * #ADI_ADC_STATUS_NULL_PTR \n
 * #ADI_ADC_STATUS_INVALID_ADC_INDEX \n
 * #ADI_ADC_STATUS_CRC_CALC_FAILED \n
 * #ADI_ADC_STATUS_TRANSCEIVE_FAILED \n
 * #ADI_ADC_STATUS_CRC_ERROR \n
 * ADI_ADC_STATUS_DATAPATH_CONFIGURATION_FAILED \n
 */
ADI_ADC_STATUS adi_adc_GetChannelXtGain(ADI_ADC_HANDLE hAdc, uint8_t *pChanIdx, int8_t numChan,
                                        int8_t adcIdx, float *pXtGainVal);

/**
 * @brief Function to get the channel offset registers of ADC.
 *        The function should be used to write to offset registers of ADC DSP.
 *
 * @param[in] hAdc   - ADC Service handle
 * @param[in] pChanIdx - Pointer to ADC channel indices.
 * @param[in] numChan - Number of channel registers to be written.
 * @param[in] adcIdx - ADC Index to which the command has to be sent.
 *                     This value ranges from 0 to numAdc-1.
 * @param[out] pOffsetVal - Offset values of all channels.
 *
 * @return  #ADI_ADC_STATUS_SUCCESS \n
 * #ADI_ADC_STATUS_NULL_PTR \n
 * #ADI_ADC_STATUS_INVALID_ADC_INDEX \n
 * #ADI_ADC_STATUS_CRC_CALC_FAILED \n
 * #ADI_ADC_STATUS_TRANSCEIVE_FAILED \n
 * #ADI_ADC_STATUS_CRC_ERROR \n
 * ADI_ADC_STATUS_DATAPATH_CONFIGURATION_FAILED \n
 */
ADI_ADC_STATUS adi_adc_GetChannelOffset(ADI_ADC_HANDLE hAdc, uint8_t *pChanIdx, int8_t numChan,
                                        int8_t adcIdx, int32_t *pOffsetVal);

/**
 * @brief Function to get XT gain compensation values of ADC DSP.
 *        The function should be used to get XT gain compensation from all channels of ADC.
 *
 * @param[in] hAdc   - ADC Service handle
 * @param[in] pChanIdx - Pointer to ADC channel indices.
 * @param[in] numChan - Number of channel registers to be written.
 * @param[in] adcIdx - ADC Index to which the command has to be sent.
 *                     This value ranges from 0 to numAdc-1.
 * @param[out] pShiftVal - Pointer to shift values of all channels.
 *
 * @return  #ADI_ADC_STATUS_SUCCESS \n
 * #ADI_ADC_STATUS_NULL_PTR \n
 * #ADI_ADC_STATUS_INVALID_ADC_INDEX \n
 * #ADI_ADC_STATUS_CRC_CALC_FAILED \n
 * #ADI_ADC_STATUS_TRANSCEIVE_FAILED \n
 * #ADI_ADC_STATUS_CRC_ERROR \n
 * ADI_ADC_STATUS_DATAPATH_CONFIGURATION_FAILED \n
 */
ADI_ADC_STATUS adi_adc_GetShift(ADI_ADC_HANDLE hAdc, uint8_t *pChanIdx, int8_t numChan,
                                int8_t adcIdx, uint8_t *pShiftVal);

/**
 * @brief  Function to get the Datapath configuration of ADC.
 *
 * @param[in] hAdc - ADC Service handle
 * @param[in] pChanIdx - Pointer to ADC channel indices.
 * @param[in] numChan - Number of channel registers to be written.
 * @param[in] adcIdx - ADC Index to which the command has to be sent.
 * @param[out] pDatapathConfig - Pointer to the structure to hold the datapath configuration.
 * @return #ADI_ADC_STATUS_SUCCESS \n
 * #ADI_ADC_STATUS_NULL_PTR \n
 * #ADI_ADC_STATUS_INVALID_ADC_INDEX \n
 * #ADI_ADC_STATUS_CRC_CALC_FAILED \n
 * #ADI_ADC_STATUS_TRANSCEIVE_FAILED \n
 * #ADI_ADC_STATUS_CRC_ERROR \n
 * ADI_ADC_STATUS_DATAPATH_CONFIGURATION_FAILED \n
 */

ADI_ADC_STATUS adi_adc_GetDatapathConfig(ADI_ADC_HANDLE hAdc, uint8_t *pChanIdx, int8_t numChan,
                                         int8_t adcIdx,
                                         ADI_ADC_CHAN_DATAPATH_CONFIG *pDatapathConfig);

/**
 * @brief Function to get the 1st order IIR SCF coefficients from ADC DSP.
 *        The function should be used to read SCF coefficients from ADC DSP.
 *
 * @param[in] hAdc   - ADC Service handle
 * @param[in] pChanIdx - Pointer to ADC channel indices.
 * @param[in] numChan - Number of channel registers to be written.
 * @param[in] adcIdx - ADC Index to which the command has to be sent.
 *                     This value ranges from 0 to numAdc-1.
 * @param[in] pCoeffs - Array of pointers to coefficients. Each element of array points to
 * coefficients of one channel in order [B0, B1, A1].
 *
 * @return  #ADI_ADC_STATUS_SUCCESS \n
 * #ADI_ADC_STATUS_NULL_PTR \n
 * #ADI_ADC_STATUS_INVALID_ADC_INDEX \n
 * #ADI_ADC_STATUS_CRC_CALC_FAILED \n
 * #ADI_ADC_STATUS_TRANSCEIVE_FAILED \n
 * #ADI_ADC_STATUS_CRC_ERROR \n
 * ADI_ADC_STATUS_DATAPATH_CONFIGURATION_FAILED \n
 */
ADI_ADC_STATUS adi_adc_GetScfCoeff(ADI_ADC_HANDLE hAdc, uint8_t *pChanIdx, uint8_t numChan,
                                   int8_t adcIdx, float *pCoeffs[]);

/**
 * @brief Function to get the 2nd order biquad HPF coefficients from ADC DSP.
 *        The function should be used to read HPF coefficients from ADC DSP.
 *
 * @param[in] hAdc  	 - ADC Service handle
 * @param[in] adcIdx - ADC Index to which the command has to be sent.
 *                     This value ranges from 0 to numAdc-1.
 * @param[out] pNumCoeffs - Pointer to numerator coefficients in order [B0, B1, B2].
 * @param[out] pDenCoeffs - Pointer to denominator coefficients in order [A0, A1, A2].
 *
 * @return  #ADI_ADC_STATUS_SUCCESS \n
 * #ADI_ADC_STATUS_NULL_PTR \n
 * #ADI_ADC_STATUS_INVALID_ADC_INDEX \n
 * #ADI_ADC_STATUS_CRC_CALC_FAILED \n
 * #ADI_ADC_STATUS_TRANSCEIVE_FAILED \n
 * #ADI_ADC_STATUS_CRC_ERROR \n
 * ADI_ADC_STATUS_DATAPATH_CONFIGURATION_FAILED \n
 */
ADI_ADC_STATUS adi_adc_GetHpfCoeff(ADI_ADC_HANDLE hAdc, int8_t adcIdx, float *pNumCoeffs,
                                   double *pDenCoeffs);

/**
 * @brief  Function to get the datarate configuration of ADC.
 *
 * @param[in] hAdc - ADC Service handle
 * @param[in] adcIdx - ADC Index to which the command has to be sent.
 *                     This value ranges from 0 to numAdc-1.
 * @param[out] pDecimationRate - Pointer to the decimation rate.
 * @param[out] pClkPreScaler - Pointer to the clock prescaler.
 * @param[out] pDecimateX2 - Pointer to the decimate by 2 bit configuration.
 * @return #ADI_ADC_STATUS_SUCCESS \n
 * #ADI_ADC_STATUS_NULL_PTR \n
 * #ADI_ADC_STATUS_INVALID_ADC_INDEX \n
 * #ADI_ADC_STATUS_CRC_CALC_FAILED \n
 * #ADI_ADC_STATUS_TRANSCEIVE_FAILED \n
 * #ADI_ADC_STATUS_CRC_ERROR \n
 * ADI_ADC_STATUS_DATAPATH_CONFIGURATION_FAILED \n
 *
 */
ADI_ADC_STATUS adi_adc_GetDataRate(ADI_ADC_HANDLE hAdc, int8_t adcIdx, uint8_t *pDecimationRate,
                                   uint8_t *pClkPreScaler, uint8_t *pDecimateX2);

/**
 * @brief Retrieves the low-pass filter (LPF) coefficients for a specific ADC instance.
 *
 * @param[in]  hAdc      Handle to the ADC instance.
 * @param[in]  adcIdx    Index of the ADC whose LPF coefficients are to be retrieved.
 *                     This value ranges from 0 to numAdc-1.
 * @param[out] pCoeffs   Pointer to a float array to store the retrieved LPF coefficients.
 *
 * @return ADI_ADC_STATUS_SUCCESS if successful, error code otherwise.
 */
ADI_ADC_STATUS adi_adc_GetLpfCoeff(ADI_ADC_HANDLE hAdc, int8_t adcIdx, float *pCoeffs);

/**
 * @brief Retrieves compensation filter coefficients for specified channels of an ADC.
 *
 * @param[in]  hAdc       Handle to the ADC instance.
 * @param[in]  pChanIdx   Pointer to an array of channel indices to retrieve coefficients for.
 * @param[in]  numChan    Number of channels to read.
 * @param[in]  adcIdx     Index of the ADC instance.
 * @param[out] pCoeffs    Array of float pointers to store the retrieved compensation coefficients.
 *                     This value ranges from 0 to numAdc-1.
 * per channel.
 *
 * @return ADI_ADC_STATUS_SUCCESS if successful, error code otherwise.
 */
ADI_ADC_STATUS adi_adc_GetCompCoeff(ADI_ADC_HANDLE hAdc, uint8_t *pChanIdx, uint8_t numChan,
                                    int8_t adcIdx, float *pCoeffs[]);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __ADI_ADC_DSP_H__ */
/** @} */
