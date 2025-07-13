/******************************************************************************
 Copyright (c) 2025  Analog Devices Inc.
******************************************************************************/

/**
 * @file        adc_datapath_configure.h
 * @brief       Definitions needed to configure ADC Datapath Registers
 * @{
 */

#ifndef __ADC_DATAPATH_CONFIGURE_H__
#define __ADC_DATAPATH_CONFIGURE_H__

/*============= I N C L U D E S =============*/

#include "adc_private.h"
#include "adi_adc.h"
#include "adi_adc_dsp.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Write to channel datapath config register (DATAPATH_CONFIG_CHx).
 * @param[in]  pInfo  				Pointer to ADC info structure.
 * @param[in]  pDatapathConfig    	Pointer to structure containing value to be copied to register.
 * @param[in]  pChanIdx 	Pointer to ADC channel indices.
 * @param[in]  numChan	 	Number of channel registers to be written.
 * @param[in]  adcNum				ADC number to write.
 * @return  One of the codes documented in #ADI_ADC_STATUS. Refer to
 * #ADI_ADC_STATUS for details.
 */
ADI_ADC_STATUS AdcWriteDataPathEnable(ADI_ADC_INFO *pInfo,
                                      ADI_ADC_CHAN_DATAPATH_CONFIG *pDatapathConfig,
                                      uint8_t *pChanIdx, int8_t numChan, int8_t adcNum);

/**
 * @brief Set channel gain values to ADC Registers.
 * @param[in]  pInfo  		Pointer to ADC info structure.
 * @param[in]  pGainVal 	Pointer to ADC channel gain values.
 * @param[in]  pChanIdx 	Pointer to ADC channel indices.
 * @param[in]  numChan	 	Number of channel registers to be written.
 * @param[in]  adcNum		ADC number to write.
 * @return  One of the codes documented in #ADI_ADC_STATUS. Refer to
 * #ADI_ADC_STATUS for details.
 */
ADI_ADC_STATUS AdcSetChannelGain(ADI_ADC_INFO *pInfo, float *pGainVal, uint8_t *pChanIdx,
                                 int8_t numChan, int8_t adcNum);
/**
 * @brief Set channel XT Gain values to ADC Registers.
 * @param[in]  pInfo  		Pointer to ADC info structure.
 * @param[in]  pXtGainVal 	Pointer to ADC channel XT Gain values.
 * @param[in]  pChanIdx 	Pointer to ADC channel indices.
 * @param[in]  numChan	 	Number of channel registers to be written.
 * @param[in]  adcNum		ADC number to write.
 * @return  One of the codes documented in #ADI_ADC_STATUS. Refer to
 * #ADI_ADC_STATUS for details.
 */
ADI_ADC_STATUS AdcSetChannelXtGain(ADI_ADC_INFO *pInfo, float *pXtGainVal, uint8_t *pChanIdx,
                                   int8_t numChan, int8_t adcNum);

/**
 * @brief Set channel XT Aggressor values to ADC Registers.
 * @param[in]  pInfo  		Pointer to ADC info structure.
 * @param[in]  pXtAggrVal 	Pointer to ADC channel XT Aggressor values.
 * @param[in]  pChanIdx 	Pointer to ADC channel indices.
 * @param[in]  numChan	 	Number of channel registers to be written.
 * @param[in]  adcNum		ADC number to write.
 * @return  One of the codes documented in #ADI_ADC_STATUS. Refer to
 * #ADI_ADC_STATUS for details.
 */
ADI_ADC_STATUS AdcSetChannelXtAggressor(ADI_ADC_INFO *pInfo, ADI_ADC_CHAN_XT_AGGRESSOR *pXtAggrVal,
                                        uint8_t *pChanIdx, int8_t numChan, int8_t adcNum);

/**
 * @brief Set channel offset values to ADC Registers.
 * @param[in]  pInfo  		Pointer to ADC info structure.
 * @param[in]  pOffsetVal 	Pointer to ADC channel offset values.
 * @param[in]  pChanIdx 	Pointer to ADC channel indices.
 * @param[in]  numChan	 	Number of channel registers to be written.
 * @param[in]  adcNum		ADC number to write.
 * @return  One of the codes documented in #ADI_ADC_STATUS. Refer to
 * #ADI_ADC_STATUS for details.
 */
ADI_ADC_STATUS AdcSetChannelOffset(ADI_ADC_INFO *pInfo, int32_t *pOffsetVal, uint8_t *pChanIdx,
                                   int8_t numChan, int8_t adcNum);

/**
 * @brief Set channel offset values to ADC Registers.
 * @param[in]  pInfo  			Pointer to ADC info structure.
 * @param[in]  pPhOffsetVal 	Pointer to ADC channel phase offset values.
 * @param[in]  pChanIdx 		Pointer to ADC channel indices.
 * @param[in]  numChan	 		Number of channel registers to be written.
 * @param[in]  adcNum			ADC number to write.
 * @return  One of the codes documented in #ADI_ADC_STATUS. Refer to
 * #ADI_ADC_STATUS for details.
 */
ADI_ADC_STATUS AdcSetPhaseOffset(ADI_ADC_INFO *pInfo, float *pPhOffsetVal, uint8_t *pChanIdx,
                                 int8_t numChan, int8_t adcNum);

/**
 * @brief Set ADC data rate Register.
 * @param[in]  pInfo  		Pointer to ADC info structure.
 * @param[in]  decimationRate 	Decimation Rate.
 * @param[in]  clkPreScaler 	ADC Clock PreScaler.
 * @param[in]  setDecimation 	Set to add a decimate by 2 in the data path.
 * @param[in]  adcNum		ADC number to write.
 * @return  One of the codes documented in #ADI_ADC_STATUS. Refer to
 * #ADI_ADC_STATUS for details.
 */
ADI_ADC_STATUS AdcSetDataRate(ADI_ADC_INFO *pInfo, uint8_t decimationRate, uint8_t clkPreScaler,
                              uint8_t setDecimation, int8_t adcNum);
/**
 * @brief Set channel datapath alpha values to ADC Registers.
 * @param[in]  pInfo  		Pointer to ADC info structure.
 * @param[in]  pAlphaVal 	Pointer to ADC channel alpha values.
 * @param[in]  adcNum		ADC number to write.
 * @return  One of the codes documented in #ADI_ADC_STATUS. Refer to
 * #ADI_ADC_STATUS for details.
 */
ADI_ADC_STATUS AdcSetDatapathAlpha(ADI_ADC_INFO *pInfo, uint8_t *pAlphaVal, int8_t adcNum);

/**
 * @brief Set LPF coefficients to ADC Registers.
 * @param[in]  pInfo  		Pointer to ADC info structure.
 * @param[in]  pCoeffs	 	Pointer to LPF coefficients.
 * @param[in]  adcNum		ADC number to write.
 * @return  One of the codes documented in #ADI_ADC_STATUS. Refer to
 * #ADI_ADC_STATUS for details.
 */
ADI_ADC_STATUS AdcSetLpfCoeff(ADI_ADC_INFO *pInfo, float *pCoeffs, int8_t adcNum);

/**
 * @brief Set SCF coefficients to ADC Registers.
 * @param[in]  pInfo  		Pointer to ADC info structure.
 * @param[in]  pCoeffs	 	Array of pointers storing each SCF coefficients.
 * @param[in]  pChanIdx 	Pointer to ADC channel indices.
 * @param[in]  numChan	 	Number of channel registers to be written.
 * @param[in]  adcNum		ADC number to write.
 * @return  One of the codes documented in #ADI_ADC_STATUS. Refer to
 * #ADI_ADC_STATUS for details.
 */
ADI_ADC_STATUS AdcSetScfCoeff(ADI_ADC_INFO *pInfo, float *pCoeffs[], uint8_t *pChanIdx,
                              int8_t numChan, int8_t adcNum);

/**
 * @brief Set Compensation Filter coefficients to ADC Registers.
 * @param[in]  pInfo  		Pointer to ADC info structure.
 * @param[in]  pCoeffs	 	Array of pointers storing each channel compensation filter coefficients.
 * @param[in]  pChanIdx 	Pointer to ADC channel indices.
 * @param[in]  numChan	 	Number of channel registers to be written.
 * @param[in]  adcNum		ADC number to write.
 * @return  One of the codes documented in #ADI_ADC_STATUS. Refer to
 * #ADI_ADC_STATUS for details.
 */
ADI_ADC_STATUS AdcSetCompCoeff(ADI_ADC_INFO *pInfo, float *pCoeffs[], uint8_t *pChanIdx,
                               int8_t numChan, int8_t adcNum);

/**
 * @brief  Gets the datarate register values.
 *
 * @param pInfo - Pointer to ADC info structure.
 * @param adcNum - ADC number to write.
 * @param pDecimationRate - Pointer to decimation rate.
 * @param pClkPreScaler - Pointer to clock pre-scaler.
 * @param pSetDecimation - Pointer to set decimation.
 * @return One of the codes documented in #ADI_ADC_STATUS. Refer to
 * #ADI_ADC_STATUS for details.
 */

ADI_ADC_STATUS AdcGetDataRate(ADI_ADC_INFO *pInfo, int8_t adcNum, uint8_t *pDecimationRate,
                              uint8_t *pClkPreScaler, uint8_t *pSetDecimation);

/**
 * @brief Get channel datapath config register (DATAPATH_CONFIG_CHx).
 *
 * @param[in] pInfo - Pointer to ADC info structure.
 * @param[in] pChanIdx - Pointer to ADC channel indices.
 * @param[in] numChan - Number of channel registers to be written.
 * @param[in] adcNum - ADC number to write.
 * @param[out] pDatapathConfig - Pointer to structure containing value to be copied from register.
 * @return One of the codes documented in #ADI_ADC_STATUS. Refer to
 * #ADI_ADC_STATUS for details.
 */

ADI_ADC_STATUS AdcGetDataPathEnable(ADI_ADC_INFO *pInfo, uint8_t *pChanIdx, int8_t numChan,
                                    int8_t adcNum, ADI_ADC_CHAN_DATAPATH_CONFIG *pDatapathConfig);

/**
 * @brief Set Compensation Filter coefficients to ADC Registers.
 * @param[in]  pInfo  		Pointer to ADC info structure.
 * @param[in]  pNumCoeffs 	Pointer to HPF numerator coefficients.
 * @param[in]  pDenCoeffs 	Pointer to HPF denominator coefficients.
 * @param[in]  adcNum		ADC number to write.
 * @return  One of the codes documented in #ADI_ADC_STATUS. Refer to
 * #ADI_ADC_STATUS for details.
 */
ADI_ADC_STATUS AdcSetHpfCoeff(ADI_ADC_INFO *pInfo, float *pNumCoeffs, double *pDenCoeffs,
                              int8_t adcNum);
/**
 * @brief Set Fix point HPF coefficients to ADC Registers.
 * @param[in]  pInfo  		Pointer to ADC info structure.
 * @param[in]  pCoeffs	 	Pointer to fix point HPF coefficients.
 * @param[in]  adcNum		ADC number to write.
 * @return  One of the codes documented in #ADI_ADC_STATUS. Refer to
 * #ADI_ADC_STATUS for details.
 */
ADI_ADC_STATUS AdcSetHpfCoeffFix(ADI_ADC_INFO *pInfo, uint32_t *pCoeffs, int8_t adcNum);
/**
 * @brief Set arithmetic shift value to ADC Registers.
 * @param[in]  pInfo  		Pointer to ADC info structure.
 * @param[in]  pShiftVal	Arithmetic shift value.
 * @param[in]  pChanIdx 	Pointer to ADC channel indices.
 * @param[in]  numChan	 	Number of channel registers to be written.
 * @param[in]  adcNum		ADC number to write.
 * @return  One of the codes documented in #ADI_ADC_STATUS. Refer to
 * #ADI_ADC_STATUS for details.
 */
ADI_ADC_STATUS AdcSetShift(ADI_ADC_INFO *pInfo, uint8_t *pShiftVal, uint8_t *pChanIdx,
                           int8_t numChan, int8_t adcNum);

/**
 * @brief Get channel gain values from ADC Registers.
 * @param[in]  pInfo  		Pointer to ADC info structure.
 * @param[in]  pChanIdx 	Pointer to ADC channel indices.
 * @param[in]  numChan	 	Number of channel registers to be written.
 * @param[in]  adcNum		ADC number to write.
 * @param[out]  pGainVal 	Pointer to ADC channel gain values.
 * @return  One of the codes documented in #ADI_ADC_STATUS. Refer to
 * #ADI_ADC_STATUS for details.
 */
ADI_ADC_STATUS AdcGetChannelGain(ADI_ADC_INFO *pInfo, uint8_t *pChanIdx, int8_t numChan,
                                 int8_t adcNum, float *pGainVal);

/**
 * @brief Get channel XT Gain values from ADC Registers.
 * @param[in]  pInfo  		Pointer to ADC info structure.
 * @param[in]  pChanIdx 	Pointer to ADC channel indices.
 * @param[in]  numChan	 	Number of channel registers to be written.
 * @param[in]  adcNum		ADC number to write.
 * @param[out]  pGainVal 	Pointer to ADC channel XT Gain values.
 * @return  One of the codes documented in #ADI_ADC_STATUS. Refer to
 * #ADI_ADC_STATUS for details.
 */
ADI_ADC_STATUS AdcGetChannelXtGain(ADI_ADC_INFO *pInfo, uint8_t *pChanIdx, int8_t numChan,
                                   int8_t adcNum, float *pGainVal);
/**
 * @brief Get channel XT Aggressor values from ADC Registers.
 * @param[in]  pInfo  		Pointer to ADC info structure.
 * @param[in]  pChanIdx 	Pointer to ADC channel indices.
 * @param[in]  numChan	 	Number of channel registers to be written.
 * @param[in]  adcNum		ADC number to write.
 * @param[out]  pXtAggrVal 	Pointer to ADC channel XT Aggressor values.
 * @return  One of the codes documented in #ADI_ADC_STATUS. Refer to
 * #ADI_ADC_STATUS for details.
 */
ADI_ADC_STATUS AdcGetChannelXtAggressor(ADI_ADC_INFO *pInfo, uint8_t *pChanIdx, int8_t numChan,
                                        int8_t adcNum, ADI_ADC_CHAN_XT_AGGRESSOR *pXtAggrVal);

/**
 * @brief Get channel offset values from ADC Registers.
 * @param[in]  pInfo  		Pointer to ADC info structure.
 * @param[in]  pChanIdx 	Pointer to ADC channel indices.
 * @param[in]  numChan	 	Number of channel registers to be written.
 * @param[in]  adcNum		ADC number to write.
 * @param[out]  pOffsetVal 	Pointer to ADC channel offset values.
 * @return  One of the codes documented in #ADI_ADC_STATUS. Refer to
 * #ADI_ADC_STATUS for details.
 */
ADI_ADC_STATUS AdcGetChannelOffset(ADI_ADC_INFO *pInfo, uint8_t *pChanIdx, int8_t numChan,
                                   int8_t adcNum, int32_t *pOffsetVal);

/**
 * @brief Get channel offset values from ADC Registers.
 * @param[in]  pInfo  			Pointer to ADC info structure.
 * @param[in]  pChanIdx 		Pointer to ADC channel indices.
 * @param[in]  numChan	 		Number of channel registers to be written.
 * @param[in]  adcNum			ADC number to write.
 * @param[out]  pPhOffsetVal 	Pointer to ADC channel phase offset values.
 * @return  One of the codes documented in #ADI_ADC_STATUS. Refer to
 * #ADI_ADC_STATUS for details.
 */
ADI_ADC_STATUS AdcGetPhaseOffset(ADI_ADC_INFO *pInfo, uint8_t *pChanIdx, int8_t numChan,
                                 int8_t adcNum, float *pPhOffsetVal);

/**
 * @brief Get channel datapath alpha values from ADC Registers.
 * @param[in]  pInfo  		Pointer to ADC info structure.
 * @param[in]  adcNum		ADC number to write.
 * @param[out]  pAlphaVal 	Pointer to ADC channel alpha values.
 * @return  One of the codes documented in #ADI_ADC_STATUS. Refer to
 * #ADI_ADC_STATUS for details.
 */
ADI_ADC_STATUS AdcGetDatapathAlpha(ADI_ADC_INFO *pInfo, int8_t adcNum, uint8_t *pAlphaVal);

/**
 * @brief Get arithmetic shift value from ADC Registers.
 * @param[in]  pInfo  		Pointer to ADC info structure.
 * @param[in]  pChanIdx 	Pointer to ADC channel indices.
 * @param[in]  numChan	 	Number of channel registers to be written.
 * @param[in]  adcNum		ADC number to write.
 * @param[out]  pShiftVal	Arithmetic shift value.
 * @return  One of the codes documented in #ADI_ADC_STATUS. Refer to
 * #ADI_ADC_STATUS for details.
 */
ADI_ADC_STATUS AdcGetShift(ADI_ADC_INFO *pInfo, uint8_t *pChanIdx, int8_t numChan, int8_t adcNum,
                           uint8_t *pShiftVal);

/**
 * @brief Get SCF coefficients from ADC Registers.
 * @param[in]  pInfo  		Pointer to ADC info structure.
 * @param[in]  pChanIdx 	Pointer to ADC channel indices.
 * @param[in]  numChan	 	Number of channel registers to be written.
 * @param[in]  adcNum		  ADC number to write.
 * @param[out]  pCoeffs	 	Array of pointers storing each SCF coefficients.
 * @return  One of the codes documented in #ADI_ADC_STATUS. Refer to
 * #ADI_ADC_STATUS for details.
 */
ADI_ADC_STATUS AdcGetScfCoeff(ADI_ADC_INFO *pInfo, uint8_t *pChanIdx, uint8_t numChan,
                              int8_t adcNum, float *pCoeffs[]);

/**
 * @brief Get Compensation Filter coefficients from ADC Registers.
 * @param[in]  pInfo  		Pointer to ADC info structure.
 * @param[in]  adcNum		ADC number to write.
 * @param[out]  pNumCoeffs 	Pointer to HPF numerator coefficients.
 * @param[out]  pDenCoeffs 	Pointer to HPF denominator coefficients.
 * @return  One of the codes documented in #ADI_ADC_STATUS. Refer to
 * #ADI_ADC_STATUS for details.
 */
ADI_ADC_STATUS AdcGetHpfCoeff(ADI_ADC_INFO *pInfo, int8_t adcNum, float *pNumCoeffs,
                              double *pDenCoeffs);

/**
 * @brief Retrieves the low-pass filter (LPF) coefficients for a given ADC.
 *
 * @param[in]  pInfo     Pointer to the ADC information structure.
 * @param[in]  adcNum    Index of the ADC instance.
 * @param[out] pCoeffs   Pointer to an array of floats to store the retrieved LPF coefficients.
 *
 * @return ADI_ADC_STATUS_SUCCESS if the operation is successful, otherwise returns an appropriate
 * error code.
 */
ADI_ADC_STATUS AdcGetLpfCoeff(ADI_ADC_INFO *pInfo, int8_t adcNum, float *pCoeffs);

/**
 * @brief Retrieves compensation filter coefficients for specified ADC channels.
 *
 * @param[in]  pInfo      Pointer to the ADC information structure.
 * @param[in]  pChanIdx   Pointer to an array containing the indices of the channels.
 * @param[in]  numChan    Number of channels for which coefficients are to be retrieved.
 * @param[in]  adcNum     Index of the ADC instance.
 * @param[out] pCoeffs    Array of float pointers where each element points to an array of
 * coefficients per channel.
 *
 * @return ADI_ADC_STATUS_SUCCESS if the operation is successful, otherwise returns an appropriate
 * error code.
 */
ADI_ADC_STATUS AdcGetCompCoeff(ADI_ADC_INFO *pInfo, uint8_t *pChanIdx, uint8_t numChan,
                               int8_t adcNum, float *pCoeffs[]);

#ifdef __cplusplus
}
#endif

#endif /* __ADC_DATAPATH_CONFIGURE_H__ */
/**
 * @}
 */
