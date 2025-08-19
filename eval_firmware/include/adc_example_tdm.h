/******************************************************************************
 Copyright (c) 2025  Analog Devices Inc.
******************************************************************************/

/**
 * @file        adc_example_tdm.h
 * @brief       ADC Example definitions for start and stop tamper detection.
 * @{
 */

#ifndef __ADC_EXAMPLE_TDM_H__
#define __ADC_EXAMPLE_TDM_H__

/*============= I N C L U D E S =============*/
#include "adi_adc.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Interface to start the tamper detection.
 * @return Result of the command
 */

int32_t StartTamperDetection(void);

/**
 * @brief Interface to stop the tamper detection.
 * @return Result of the command
 */
int32_t StopTamperDetection(void);

#ifdef __cplusplus
}
#endif

#endif /* __ADC_EXAMPLE_TDM_H__ */
/**
 * @}
 */
