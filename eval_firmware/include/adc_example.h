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

/**
 * @brief  SPI Rx callback.
 */
void AdcSpiRxCallback(void);

/**
 * @brief  Interface to GPIO callback.
 * @param[in]  port - Port.
 * @param[in]  pinFlag - pin flag.
 */
void AdcDreadyCallback(uint32_t port, uint32_t pinFlag);

/**
 * @brief  Function that collect samples.
 * @param[in]  pInfo - pointer to interface info structure.
 * @param[in]  channelMask - channel mask.
 * @param[in]  numSamplesRequired - number of samples required.
 * @param[out]  pSamples - pointer to buffer.
 * @return Result of the command
 */
ADI_ADC_STATUS AdcExmCollectSamples(ADC_INTERFACE_INFO *pInfo, uint32_t channelMask,
                                    uint32_t numSamplesRequired, int32_t *pSamples);

/**
 * @brief ADC callback function.
 * @param[in] hUser - user handle.
 * @param[in] adcEvent - ADC event.
 * @return Result of the command
 */

ADI_ADC_STATUS AdcExmAdcCallback(void *hUser, uint32_t adcEvent);

/**
 * @brief Gets pointer to Board config structure.
 * @return pointer to Board config structure.
 */
ADC_BOARD_CONFIG *AdcExmGetBoardConfig(void);

#ifdef __cplusplus
}
#endif

#endif /* ADC_EXAMPLE_H_ */
/**
 * @}
 */
