/******************************************************************************
 Copyright (c) 2024 - 2025  Analog Devices Inc.
******************************************************************************/

#ifndef __ADC_CONFIG_ADEMA12X_H__
#define __ADC_CONFIG_ADEMA12X_H__

#include "adc_private.h"
#include "adi_adc.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Sets init configuration.
 *
 * @param[in] pConfig - Pointer to configuration structure.
 */
ADI_ADC_STATUS AdcInitAdema124(ADC_TYPE_CONFIG *pTypeConfig);

/**
 * @brief Initialize SPI shoft frame configuration.
 *
 * @param[in] pConfig - Pointer to configuration structure.
 */
ADI_ADC_STATUS AdcInitShortAdema127(ADC_TYPE_CONFIG *pTypeConfig);

/**
 * @brief Sets init configuration.
 *
 * @param[in] pConfig - Pointer to configuration structure.
 */
ADI_ADC_STATUS AdcInitAdema127(ADC_TYPE_CONFIG *pTypeConfig);

#ifdef __cplusplus
}
#endif

#endif /* __ADC_CONFIG_ADEMA12X_H__ */
