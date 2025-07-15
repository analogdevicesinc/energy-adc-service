/******************************************************************************
 Copyright (c) 2022 - 2025  Analog Devices Inc.
******************************************************************************/

/**
 * @file        adc_example.h
 * @brief       ADC example definitions for UCOMM and IIO examples
 * @{
 */

#ifndef __ADC_EXAMPLE_H__
#define __ADC_EXAMPLE_H__

/*============= I N C L U D E S =============*/
#include "adc_service_interface.h"
#include "adi_adc_memory.h"
#include "adi_circ_buf.h"
#include "adi_evb.h"

#ifdef __cplusplus
extern "C" {
#endif

/*============= D A T A  T Y P E S =============*/
/**
 * Enum for example result codes
 */
typedef enum
{
    ADC_EXAMPLE_STATUS_SUCCESS = 0,
    ADC_EXAMPLE_STATUS_BOARD_INIT_FAILED,
    ADC_EXAMPLE_STATUS_CLI_INIT_FAILED,
    ADC_EXAMPLE_STATUS_COMM_INIT_FAILED,
    ADC_EXAMPLE_STATUS_CRC_INIT_FAILED,
    ADC_EXAMPLE_STATUS_ADC_INIT_FAILED,
    ADC_EXAMPLE_STATUS_INVALID_CMD,
    ADC_EXAMPLE_STATUS_SAMPLES_NOT_AVAILABLE,
    ADC_EXAMPLE_STATUS_IF_CREATE_SERVICE_FAILED,
    ADC_EXAMPLE_STATUS_EXIT = 0xF
} ADC_EXAMPLE_STATUS;

/*======= P U B L I C   P R O T O T Y P E S ========*/

/**
 * @brief Initialises Com, Crc and Scomm services
 * @return status
 */
ADC_EXAMPLE_STATUS InitServices(void);

/**
 * @brief Process commands
 * @return status
 */
ADC_EXAMPLE_STATUS ProcessCommand(void);

#ifdef __cplusplus
}
#endif

#endif /* ADC_EXAMPLE_H_ */
/**
 * @}
 */
