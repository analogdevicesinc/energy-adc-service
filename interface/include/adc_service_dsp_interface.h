/******************************************************************************
 Copyright (c) 2025  Analog Devices Inc.
******************************************************************************/

/**
 * @file        adc_service_dsp_interface.h
 * @defgroup    ADC_DSP_INTERFACE Interface to ADC Service DSP
 * @brief       Interface for initialising and configuring ADC DSP
 * by the application by calling appropriate Service APIs.
 * @{
 */

#ifndef __ADC_SERVICE_DSP_INTERFACE_H__
#define __ADC_SERVICE_DSP_INTERFACE_H__

/*============= I N C L U D E S =============*/

#include "adc_service_interface.h"
#include "adi_adc.h"
#include "adi_adc_dsp.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Sets the ADC DSP datapath structure with the default values as mentioned configuration
 * file.
 * @param[in]  pInfo - pointer to interface info structure.
 * @param[in]  pConfig - ADC Configs
 * @return Result of the command
 */
ADI_ADC_STATUS AdcIfDatapathSetVal(ADC_INTERFACE_INFO *pInfo, ADI_ADC_CONFIG *pConfig);

/**
 * @brief Writes to ADDR_ADEMA127_MMR_ACCESS_EXTENDED_MMAP ADC to access uDSP memory.
 * @param[in]  pInfo - pointer to interface info structure.
 * @param[in]  value - value to write to register.
 * @param[in]  adcIdx - adc index.
 * @return Result of the command
 */
ADI_ADC_STATUS AdcIfAccessDspMem(ADC_INTERFACE_INFO *pInfo, uint8_t value, int8_t adcIdx);

/**
 * @brief Writes to DATAPATH_CONFIG_LOCK ADC register.
 * @param[in]  pInfo - pointer to interface info structure.
 * @param[in]  value - value to write to register.
 * @param[in]  adcIdx - adc index.
 * @return Result of the command
 */
ADI_ADC_STATUS AdcIfConfigDspLock(ADC_INTERFACE_INFO *pInfo, uint8_t value, int8_t adcIdx);

/**
 * @brief Writes to DATAPATH_CONFIG_CHx registers of ADC.
 * @param[in]  pInfo - pointer to interface info structure.
 * @param[in]  pDatapathEn - array storing DATAPATH_CONFIG_CHx values.
 * @param[in]  pChanIdx 	Pointer to ADC channel indices.
 * @param[in]  numChan	 	Number of channel registers to be written.
 * @param[in]  adcIdx - adc index.
 * @return Result of the command
 */
ADI_ADC_STATUS AdcIfEnableDatapath(ADC_INTERFACE_INFO *pInfo,
                                   ADI_ADC_CHAN_DATAPATH_CONFIG *pDatapathEn, uint8_t *pChanIdx,
                                   int8_t numChan, int8_t adcIdx);

/**
 * @brief Retrieves the datapath configuration for the specified ADC channel.
 *
 * @param[in]  pInfo        Pointer to the ADC interface info structure.
 * @param[out] pChanIdx     Pointer to the channel index.
 * @param[in]  numChan      Number of channels registers to be read.
 * @param[in]  adcIdx       Index of the ADC.
 * @param[out] pDatapathCfg  Datapath config register value that is read.
 *
 * @return ADI_ADC_STATUS   Status of the operation.
 */
ADI_ADC_STATUS AdcIfGetDatapathConfig(ADC_INTERFACE_INFO *pInfo, uint8_t *pChanIdx, int8_t numChan,
                                      int8_t adcIdx, ADI_ADC_CHAN_DATAPATH_CONFIG *pDatapathCfg);

/**
 * @brief Retrieves the datarate configuration value.
 * @param[in]  pInfo - pointer to interface info structure.
 * @param[in]  adcIdx - adc index.
 * @param[out]  pDecimationRate - pointer to decimation rate.
 * @param[out]  pClkPreScaler - pointer to clock prescaler.
 * @param[out]  pSetDecimation - pointer to set decimate by 2 bit configuration.
 * @return Result of the command
 */
ADI_ADC_STATUS AdcIfGetDatarate(ADC_INTERFACE_INFO *pInfo, int8_t adcIdx, uint8_t *pDecimationRate,
                                uint8_t *pClkPreScaler, uint8_t *pSetDecimation);

/**
 * @brief Sets the datarate value.
 * @param[in]  pInfo - pointer to interface info structure.
 * @param[in]  decimationRate - decimation rate.
 * @param[in]  clkPreScalar - clock prescaler.
 * @param[in]  decimationByX2 -  decimate by 2 bit configuration value.
 * @param[in]  adcIdx - adc index.
 * @return Result of the command
 */
ADI_ADC_STATUS AdcIfSetDatarate(ADC_INTERFACE_INFO *pInfo, uint8_t decimationRate,
                                uint8_t clkPreScalar, uint8_t decimationByX2, int8_t adcIdx);

/**
 * @brief Writes to channel gain registers of ADC.
 * @param[in]  pInfo - pointer to interface info structure.
 * @param[in]  pGainVal - pointer to gain values.
 * @param[in]  pChanIdx 	Pointer to ADC channel indices.
 * @param[in]  numChan	 	Number of channel registers to be written.
 * @param[in]  adcIdx - adc index.
 * @return Result of the command
 */
ADI_ADC_STATUS AdcIfSetGain(ADC_INTERFACE_INFO *pInfo, float *pGainVal, uint8_t *pChanIdx,
                            int8_t numChan, int8_t adcIdx);
/**
 * @brief Writes to channel XT Gain registers of ADC.
 * @param[in]  pInfo - pointer to interface info structure.
 * @param[in]  pXtGainVal - pointer to Xt Gain values.
 * @param[in]  pChanIdx 	Pointer to ADC channel indices.
 * @param[in]  numChan	 	Number of channel registers to be written.
 * @param[in]  adcIdx - adc index.
 * @return Result of the command
 */
ADI_ADC_STATUS AdcIfSetXtGain(ADC_INTERFACE_INFO *pInfo, float *pXtGainVal, uint8_t *pChanIdx,
                              int8_t numChan, int8_t adcIdx);

/**
 * @brief Writes to channel XT Aggressor registers of ADC.
 * @param[in]  pInfo - pointer to interface info structure.
 * @param[in]  pXtAggrVal - pointer to Xt Aggressor values.
 * @param[in]  pChanIdx 	Pointer to ADC channel indices.
 * @param[in]  numChan	 	Number of channel registers to be written.
 * @param[in]  adcIdx - adc index.
 * @return Result of the command
 */
ADI_ADC_STATUS AdcIfSetXtAggressor(ADC_INTERFACE_INFO *pInfo, ADI_ADC_CHAN_XT_AGGRESSOR *pXtAggrVal,
                                   uint8_t *pChanIdx, int8_t numChan, int8_t adcIdx);

/**
 * @brief Writes to channel offset registers of ADC.
 * @param[in]  pInfo - pointer to interface info structure.
 * @param[in]  pOffsetVal - pointer to offset values.
 * @param[in]  pChanIdx 	Pointer to ADC channel indices.
 * @param[in]  numChan	 	Number of channel registers to be written.
 * @param[in]  adcIdx - adc index.
 * @return Result of the command
 */
ADI_ADC_STATUS AdcIfSetOffset(ADC_INTERFACE_INFO *pInfo, int32_t *pOffsetVal, uint8_t *pChanIdx,
                              int8_t numChan, int8_t adcIdx);

/**
 * @brief Writes to channel datapath alpha registers of ADC.
 * @param[in]  pInfo - pointer to interface info structure.
 * @param[in]  pAlphaVal - pointer to alpha values.
 * @param[in]  adcIdx - adc index.
 * @return Result of the command
 */
ADI_ADC_STATUS AdcIfSetDatapathAlpha(ADC_INTERFACE_INFO *pInfo, uint8_t *pAlphaVal, int8_t adcIdx);

/**
 * @brief Writes to channel phase offset registers of ADC.
 * @param[in]  pInfo - pointer to interface info structure.
 * @param[in]  pPhOffsetVal - pointer to phase offset values for all channels of ADC (in samples).
 * @param[in]  pChanIdx 	Pointer to ADC channel indices.
 * @param[in]  numChan	 	Number of channel registers to be written.
 * @param[in]  adcIdx - adc index.
 * @return Result of the command
 */
ADI_ADC_STATUS AdcIfSetPhaseOffset(ADC_INTERFACE_INFO *pInfo, float *pPhOffsetVal,
                                   uint8_t *pChanIdx, int8_t numChan, int8_t adcIdx);

/**
 * @brief Writes to shift registers of ADC.
 * @param[in]  pInfo - pointer to interface info structure.
 * @param[in]  pVal - pointer to shift values.
 * @param[in]  pChanIdx 	Pointer to ADC channel indices.
 * @param[in]  numChan	 	Number of channel registers to be written.
 * @param[in]  adcIdx - adc index.
 * @return Result of the command
 */
ADI_ADC_STATUS AdcIfSetShift(ADC_INTERFACE_INFO *pInfo, uint8_t *pVal, uint8_t *pChanIdx,
                             int8_t numChan, int8_t adcIdx);

/**
 * @brief Set sensor compensation filter coefficients in ADC registers.
 * @param[in]  pInfo - pointer to interface info structure.
 * @param[in] pCoeffs - Array of pointers to coefficients. Each element of array points to
 * coefficients of one channel in order [B0, B1, A1].
 * @param[in] pChanIdx - Pointer to ADC channel indices.
 * @param[in] numChan - Number of channel registers to be written.
 * @param[in] adcIdx - adc index.
 * @return Result of the command
 */
ADI_ADC_STATUS AdcIfSetScfCoeff(ADC_INTERFACE_INFO *pInfo, float *pCoeffs[], uint8_t *pChanIdx,
                                uint8_t numChan, int8_t adcIdx);
/**
 * @brief Set high pass filter coefficients in ADC registers.
 * @param[in]  pInfo - pointer to interface info structure.
 * @param[in]  pNumCoeffs - pointer to numerator coefficients of HPF transfer function.
 * @param[in]  pDenCoeffs - pointer to denominator coefficients of HPF transfer function.
 * @param[in]  adcIdx - adc index.
 * @return Result of the command
 */
ADI_ADC_STATUS AdcIfSetHpfCoeff(ADC_INTERFACE_INFO *pInfo, float *pNumCoeffs, double *pDenCoeffs,
                                int8_t adcIdx);
/**
 * @brief Set high pass filter cutoff frequency.
 * @param[in]  pInfo - pointer to interface info structure.
 * @param[in]  bwOption - Select HPF bandwidth options.
 * @param[in]  adcIdx - adc index.
 * @return Result of the command
 */
ADI_ADC_STATUS AdcIfSetHpfCutoff(ADC_INTERFACE_INFO *pInfo, uint32_t bwOption, int8_t adcIdx);

/**
 * @brief Reads the channel gain registers of ADC.
 * @param[in]  pInfo - pointer to interface info structure.
 * @param[in]  pChanIdx 	Pointer to ADC channel indices.
 * @param[in]  numChan	 	Number of channel registers to be written.
 * @param[in]  adcIdx - adc index.
 * @param[out]  pGainVal - pointer to gain values.
 * @return Result of the command
 */
ADI_ADC_STATUS AdcIfGetGain(ADC_INTERFACE_INFO *pInfo, uint8_t *pChanIdx, int8_t numChan,
                            int8_t adcIdx, float *pGainVal);

/**
 * @brief Reads the channel XT Gain registers of ADC.
 * @param[in]  pInfo - pointer to interface info structure.
 * @param[in]  pChanIdx 	Pointer to ADC channel indices.
 * @param[in]  numChan	 	Number of channel registers to be written.
 * @param[in]  adcIdx - adc index.
 * @param[out]  pXtGainVal - pointer to XT Gain values.
 * @return Result of the command
 */
ADI_ADC_STATUS AdcIfGetXtGain(ADC_INTERFACE_INFO *pInfo, uint8_t *pChanIdx, int8_t numChan,
                              int8_t adcIdx, float *pXtGainVal);

/**
 * @brief Reads the channel XT Aggressor registers of ADC.
 * @param[in]  pInfo - pointer to interface info structure.
 * @param[in]  pChanIdx 	Pointer to ADC channel indices.
 * @param[in]  numChan	 	Number of channel registers to be written.
 * @param[in]  adcIdx - adc index.
 * @param[out]  pXtAggrVal - pointer to XT Aggressor values.
 * @return Result of the command
 */
ADI_ADC_STATUS AdcIfGetXtAggressor(ADC_INTERFACE_INFO *pInfo, uint8_t *pChanIdx, int8_t numChan,
                                   int8_t adcIdx, ADI_ADC_CHAN_XT_AGGRESSOR *pXtAggrVal);

/**
 * @brief Reads the channel offset registers of ADC.
 * @param[in]  pInfo - pointer to interface info structure.
 * @param[in]  pChanIdx 	Pointer to ADC channel indices.
 * @param[in]  numChan	 	Number of channel registers to be written.
 * @param[in]  adcIdx - adc index.
 * @param[out]  pOffsetVal - pointer to offset values.
 * @return Result of the command
 */
ADI_ADC_STATUS AdcIfGetOffset(ADC_INTERFACE_INFO *pInfo, uint8_t *pChanIdx, int8_t numChan,
                              int8_t adcIdx, int32_t *pOffsetVal);

/**
 * @brief Reads the channel datapath alpha registers of ADC.
 * @param[in]  pInfo - pointer to interface info structure.
 * @param[in]  adcIdx - adc index.
 * @param[out]  pAlphaVal - pointer to alpha values.
 * @return Result of the command
 */
ADI_ADC_STATUS AdcIfGetDatapathAlpha(ADC_INTERFACE_INFO *pInfo, int8_t adcIdx, uint8_t *pAlphaVal);

/**
 * @brief Reads the channel phase offset registers of ADC.
 * @param[in]  pInfo - pointer to interface info structure.
 * @param[in]  pChanIdx 	Pointer to ADC channel indices.
 * @param[in]  numChan	 	Number of channel registers to be written.
 * @param[in]  adcIdx - adc index.
 * @param[out]  pPhOffsetVal - pointer to phase offset values for all channels of ADC (in samples).
 * @return Result of the command
 */
ADI_ADC_STATUS AdcIfGetPhaseOffset(ADC_INTERFACE_INFO *pInfo, uint8_t *pChanIdx, int8_t numChan,
                                   int8_t adcIdx, float *pPhOffsetVal);

/**
 * @brief Reads the shift registers of ADC.
 * @param[in]  pInfo - pointer to interface info structure.
 * @param[in]  pChanIdx 	Pointer to ADC channel indices.
 * @param[in]  numChan	 	Number of channel registers to be written.
 * @param[in]  adcIdx - adc index.
 * @param[out]  pVal - pointer to shift values.
 * @return Result of the command
 */
ADI_ADC_STATUS AdcIfGetShift(ADC_INTERFACE_INFO *pInfo, uint8_t *pChanIdx, int8_t numChan,
                             int8_t adcIdx, uint8_t *pVal);

/**
 * @brief Reads the sensor compensation filter coefficients in ADC registers.
 * @param[in]  pInfo - pointer to interface info structure.
 * @param[in] pChanIdx - Pointer to ADC channel indices.
 * @param[in] numChan - Number of channel registers to be written.
 * @param[in] adcIdx - ADC Index to which the command has to be sent.
 *                     This value ranges from 0 to numAdc-1.
 *                     This value will be -1 if command has to be set to
 *                     all ADC's.
 * @param[in] pCoeffs - Array of pointers to coefficients. Each element of array points to
 * coefficients of one channel in order [B0, B1, A1].
 */
ADI_ADC_STATUS AdcIfGetScfCoeff(ADC_INTERFACE_INFO *pInfo, uint8_t *pChanIdx, uint8_t numChan,
                                int8_t adcIdx, float *pCoeffs[]);

/**
 * @brief Reads the high pass filter coefficients in ADC registers.
 * @param[in]  pInfo - pointer to interface info structure.
 * @param[in]  adcIdx - adc index.
 * @param[out]  pNumCoeffs - pointer to numerator coefficients of HPF transfer function.
 * @param[out]  pDenCoeffs - pointer to denominator coefficients of HPF transfer function.
 * @return Result of the command
 */
ADI_ADC_STATUS AdcIfGetHpfCoeff(ADC_INTERFACE_INFO *pInfo, int8_t adcIdx, float *pNumCoeffs,
                                double *pDenCoeffs);

/**
 * @brief Retrieves the Bandwidth option (to set HPF cutoff frequency) for a specified ADC.
 * @param[in] pInfo - Pointer to the ADC interface info structure.
 * @param[in] adcIdx -  adc index.
 * @param[out] pBwOption - Bandwidth option that was used to set HPF cutoff frequency.
 * @return ADI_ADC_STATUS Status of the operation.
 */
ADI_ADC_STATUS AdcIfGetHpfCutoff(ADC_INTERFACE_INFO *pInfo, int8_t adcIdx, uint32_t *pBwOption);

/**
 * @brief Retrieves the DSP configuration for the given ADC.
 * @param[in]  pInfo - pointer to interface info structure.
 * @param[in]  pChanIdx - Pointer to ADC channel indices.
 * @param[in]  numChan - Number of channel registers to be written.
 * @param[in]  adcIdx - ADC index.
 * @param[out] pDatapathParams - pointer to ADC DSP datapath parameters.
 * @param[out] pChannelParams - pointer to ADC DSP channel parameters.
 * @return Result of the command
 */
ADI_ADC_STATUS AdcIfGetDspConfig(ADC_INTERFACE_INFO *pInfo, uint8_t *pChanIdx, int8_t numChan,
                                 int8_t adcIdx, ADI_ADC_DSP_DATAPATH_PARAMS *pDatapathParams,
                                 ADI_ADC_DSP_CHANNEL_PARAMS *pChannelParams);

/**
 * @brief Sets the DSP configuration for the given adc.
 * @param[in]  pInfo - pointer to interface info structure.
 * @param[in]  pDatapathParams - pointer to ADC DSP datapath parameters.
 * @param[in]  pChannelParams - pointer to ADC DSP channel parameters.
 * @param[in]  pChanIdx - Pointer to ADC channel indices.
 * @param[in]  numChan - Number of channel registers to be written.
 * @param[in]  adcIdx - ADC index.
 * @return Result of the command
 */
ADI_ADC_STATUS AdcIfSetDspConfig(ADC_INTERFACE_INFO *pInfo,
                                 ADI_ADC_DSP_DATAPATH_PARAMS *pDatapathParams,
                                 ADI_ADC_DSP_CHANNEL_PARAMS *pChannelParams, uint8_t *pChanIdx,
                                 int8_t numChan, int8_t adcIdx);

/**
 * @brief Retrieves the DSP configuration for the given ADC.
 * @param[in]  pInfo - pointer to interface info structure.
 * @param[in]  pChanIdx - Pointer to ADC channel indices.
 * @param[in]  numChan - Number of channel registers to be written.
 * @param[in]  adcIdx - ADC index.
 * @param[out] pChannelParams - pointer to ADC DSP channel parameters.
 * @return Result of the command
 */
ADI_ADC_STATUS AdcIfGetDspRam(ADC_INTERFACE_INFO *pInfo, uint8_t *pChanIdx, int8_t numChan,
                              int8_t adcIdx, ADI_ADC_DSP_CHANNEL_PARAMS *pChannelParams);

/**
 * @brief Sets the DSP configuration for the given adc.
 * @param[in]  pInfo - pointer to interface info structure.
 * @param[in]  pChannelParams - pointer to ADC DSP channel parameters.
 * @param[in]  pChanIdx - Pointer to ADC channel indices.
 * @param[in]  numChan - Number of channel registers to be written.
 * @param[in]  adcIdx - ADC index.
 * @return Result of the command
 */
ADI_ADC_STATUS AdcIfSetDspRam(ADC_INTERFACE_INFO *pInfo, ADI_ADC_DSP_CHANNEL_PARAMS *pChannelParams,
                              uint8_t *pChanIdx, int8_t numChan, int8_t adcIdx);

/**
 * @brief Sets the low-pass filter (LPF) coefficients for a specific ADC.
 *
 * @param[in]  pInfo     Pointer to the ADC interface information structure.
 * @param[in]  pCoeffs   Pointer to an array of LPF coefficients to be set.
 * @param[in]  adcIdx    Index of the ADC for which the coefficients are to be set.
 *
 * @return ADI_ADC_STATUS_SUCCESS if successful, error code otherwise.
 */
ADI_ADC_STATUS AdcIfSetLpfCoeff(ADC_INTERFACE_INFO *pInfo, float *pCoeffs, int8_t adcIdx);

/**
 * @brief Retrieves the low-pass filter (LPF) coefficients for a specific ADC.
 *
 * @param[in]  pInfo     Pointer to the ADC interface information structure.
 * @param[in]  adcIdx    Index of the ADC from which the coefficients are to be retrieved.
 * @param[out] pCoeffs   Pointer to an array to store the retrieved LPF coefficients.
 *
 * @return ADI_ADC_STATUS_SUCCESS if successful, error code otherwise.
 */
ADI_ADC_STATUS AdcIfGetLpfCoeff(ADC_INTERFACE_INFO *pInfo, int8_t adcIdx, float *pCoeffs);

/**
 * @brief Sets the ADCCMI (ADC Common Mode Input) value for the specified ADC.
 * @param[in]  pInfo - pointer to interface info structure.
 * @param[in]  cmiValue - Common Mode Input value to be set.
 * @param[in]  adcIdx - ADC index.
 * @return Result of the command
 */
ADI_ADC_STATUS AdcIfSetAdcCmi(ADC_INTERFACE_INFO *pInfo, int8_t adcIdx, uint8_t cmiValue);

/**
 * @brief Retrieves the ADCCMI (ADC Common Mode Input) value for the specified ADC.
 * @param[in]  pInfo - pointer to interface info structure.
 * @param[in]  adcIdx - ADC index.
 * @param[out] pCmiValue - Pointer to store the retrieved Common Mode Input value.
 * @return Result of the command
 */
ADI_ADC_STATUS AdcIfGetAdcCmi(ADC_INTERFACE_INFO *pInfo, int8_t adcIdx, uint8_t *pCmiValue);

/**
 * @brief Sets the ADCINV (ADC Inversion) value for the specified ADC.
 * @param[in]  pInfo - pointer to interface info structure.
 * @param[in]  invValue - Inversion value to be set.
 * @param[in]  adcIdx - ADC index.
 * @return Result of the command
 */
ADI_ADC_STATUS AdcIfSetAdcInv(ADC_INTERFACE_INFO *pInfo, int8_t adcIdx, uint8_t invValue);

/**
 * @brief Retrieves the ADCINV (ADC Inversion) value for the specified ADC.
 * @param[in]  pInfo - pointer to interface info structure.
 * @param[in]  adcIdx - ADC index.
 * @param[out] pInvValue - Pointer to store the retrieved Inversion value.
 * @return Result of the command
 */
ADI_ADC_STATUS AdcIfGetAdcInv(ADC_INTERFACE_INFO *pInfo, int8_t adcIdx, uint8_t *pInvValue);

/**
 * @brief Sets compensation filter coefficients for specified ADC channels.
 *
 * @param[in] pInfo      Pointer to the ADC interface information structure.
 * @param[in] pCoeffs    Array of pointers to compensation coefficients for each channel.
 * @param[in] pChanIdx   Pointer to array of channel indices to apply the coefficients to.
 * @param[in] numChan    Number of channels to configure.
 * @param[in] adcIdx     Index of the ADC for which the coefficients are to be set.
 *
 * @return ADI_ADC_STATUS_SUCCESS if successful, error code otherwise.
 */
ADI_ADC_STATUS AdcIfSetCompCoeff(ADC_INTERFACE_INFO *pInfo, float *pCoeffs[], uint8_t *pChanIdx,
                                 uint8_t numChan, int8_t adcIdx);

/**
 * @brief Retrieves compensation filter coefficients for specified ADC channels.
 *
 * @param[in]  pInfo     Pointer to the ADC interface information structure.
 * @param[in]  pChanIdx  Pointer to array of channel indices to retrieve coefficients from.
 * @param[in]  numChan   Number of channels to read.
 * @param[in]  adcIdx    Index of the ADC from which the coefficients are to be retrieved.
 * @param[out] pCoeffs   Array of pointers to store the retrieved compensation coefficients.
 *
 * @return ADI_ADC_STATUS_SUCCESS if successful, error code otherwise.
 */
ADI_ADC_STATUS AdcIfGetCompCoeff(ADC_INTERFACE_INFO *pInfo, uint8_t *pChanIdx, uint8_t numChan,
                                 int8_t adcIdx, float *pCoeffs[]);

#ifdef __cplusplus
}
#endif

#endif /* __ADC_SERVICE_DSP_INTERFACE_H__ */
/**
 * @}
 */
