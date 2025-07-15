/******************************************************************************
 Copyright (c) 2025  Analog Devices Inc.
******************************************************************************/

/**
 * @file        adc_example_tdm.h
 * @defgroup    ADC_TDM_INTERFACE to ADC Service.
 * @brief       Interface to start and stop tamper detection.
 * @{
 */

#ifndef __ADC_TDM_INTERFACE_H__
#define __ADC_TDM_INTERFACE_H__

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

#endif /* ADC_TDM_INTERFACE_H_ */
/**
 * @}
 */
