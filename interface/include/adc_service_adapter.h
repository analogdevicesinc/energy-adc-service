/******************************************************************************
 Copyright (c) 2025  Analog Devices Inc.
******************************************************************************/

/**
 * @file        adc_service_adapter.h
 * @defgroup    ADC_ADAPTER Adapter Layer to ADC Service
 * @brief       These functions shows how ADC Service can be adapted to various MCUs.
 * @{
 */

#ifndef __ADC_SERVICE_ADAPTER_H__
#define __ADC_SERVICE_ADAPTER_H__

/*============= I N C L U D E S =============*/

#include "ade_crc.h"
#include "adi_adc.h"
#include "adi_adc_memory.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup    ADCADAPTERINITCONFIG Configurations and User Data
 * @brief This section contains the configurations required to initialise user
 * data required by the application specific to the platform.
 * @{
 */

/** @} */

/** @defgroup ADCADAPTERINIT ADC Service Adapter Functions
 * @brief Adapter functions required for ADC Service.
 * @{
 */

/**
 * @brief Adapter function to populate the ADC config strcture.
 * @param[in] pConfig - Pointer to ADC config.
 * @return Result of the command
 */
int32_t AdcAdptPopulateConfig(ADI_ADC_CONFIG *pConfig);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* ADC_SERVICE_ADAPTER_H_ */
/**
 * @}
 */
