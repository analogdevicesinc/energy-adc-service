/******************************************************************************
 Copyright (c) 2025  Analog Devices Inc.
******************************************************************************/

/**
 * @file        adc_example_freertos_config.h
 * @brief       Header file for ADC service interface with FreeRTOS.
 *
 * This file provides the definitions for integrating the ADC services with FreeRTOS.
 * It initializes the threads required to run the ADC service and the ISRs required.
 *
 * @note Ensure FreeRTOS is properly configured and running before using
 * the ADC service functions.
 *
 * @{
 */

#ifndef ADC_SERVICE_FREERTOS_INTERFACE_H_
#define ADC_SERVICE_FREERTOS_INTERFACE_H_

/*============= I N C L U D E S =============*/

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*======= P U B L I C   P R O T O T Y P E S ========*/

/**
 * @brief Initializes the threads for the ADC service example using FreeRTOS.
 *
 * This function sets up the necessary threads required to run the ADC service.
 * It ensures that all threads are created and initialized properly.
 *
 * @returns 0 if the thread creation is successful, otherwise returns -1.
 */
int32_t AdcIfInitThreads(void);

#ifdef __cplusplus
}
#endif

#endif /* ADC_SERVICE_FREERTOS_INTERFACE_H_ */
/**
 * @}
 */
