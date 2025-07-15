/******************************************************************************
 Copyright (c) 2025  Analog Devices Inc.
******************************************************************************/

/**
 * @file        adc_example_attributes.h
 * @brief       The functions to set/get attributes.
 * @{
 */

#ifndef __ADC_EXAMPLE_ATTRIBUTES_H__
#define __ADC_EXAMPLE_ATTRIBUTES_H__

/*============= I N C L U D E S =============*/
#include "adc_service_interface.h"
#include "adi_adc.h"
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Command to set sampling rate */
#define ADC_ATTRIBUTE_SAMPLING_RATE 0x1

/*======= P U B L I C   P R O T O T Y P E S ========*/

/**
 * @brief Sets the required attribute.
 * @return status
 */
int32_t SetAttribute(ADC_INTERFACE_INFO *pAdcIf, uint32_t attribute);

/**
 * @brief Sets datapath config attribute.
 * @return status
 */
int32_t SetDatapathConfig(ADC_INTERFACE_INFO *pAdcIf, uint8_t *pChanIdx, uint8_t adcIndex,
                          uint32_t writeVal);

#ifdef __cplusplus
}
#endif

#endif /* __ADC_EXAMPLE_ATTRIBUTES_H__ */
/**
 * @}
 */
