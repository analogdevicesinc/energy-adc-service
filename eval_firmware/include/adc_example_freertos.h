/******************************************************************************
 Copyright (c) 2024 - 2025  Analog Devices Inc.
******************************************************************************/

/**
 * @file        adc_example_freertos.h
 * @brief       Example demonstrating the usage of ADC with FreeRTOS.
 * @{
 */

#ifndef ADC_EXAMPLE_FREERTOS_H_
#define ADC_EXAMPLE_FREERTOS_H_

/*============= I N C L U D E S =============*/

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*======= P U B L I C   P R O T O T Y P E S ========*/

/**
 * @brief Function to initialize the threads for the example.
 * @return -1 if the thread creation fails, 0 otherwise.
 */
int32_t InitThreads(void);

#ifdef __cplusplus
}
#endif

#endif /* ADC_EXAMPLE_FREERTOS_H_ */
/**
 * @}
 */
