/******************************************************************************
 Copyright (c) 2023 - 2025  Analog Devices Inc.
******************************************************************************/

/**
 * @file    adi_adc_frame_assemble.h
 * @brief    Function to assemble ADC command frame
 * @addtogroup ADI_ADC
 * @{
 */

#ifndef __ADI_ADC_FRAME_ASSEMBLE_H__
#define __ADI_ADC_FRAME_ASSEMBLE_H__

/*============= I N C L U D E S =============*/
#include "adi_adc.h"
#include "adi_adc_frame_format.h"
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*=============  D E F I N I T I O N S  =============*/

/** @defgroup ADC_FRAME Frame formats
 * @brief This section contains the ADC Frame formats.
 * @{
 */

/**
 * @brief Function to assemble the command after computing CRC. Data from input
 * arguments are filled into ADC frame command structure.
 * @param[in] pfCalcCmdCrc - Function pointer to CRC calculation function.
 * @param[in] readWriteBit - Data that needs to be filled in readWriteBit field
 *                           of the frame.
 * @param[in] addr - Data that needs to be filled in address field of the frame.
 * @param[in] data - Data that needs to be filled in data field of the frame.
 * @param[in] pAdcCmd - Pointer to ADC command structure.
 * @param[in] format - SPI response format.
 * @return  One of the return codes documented in ADI_CRC_RESULT. Refer to
 * ADI_CRC_RESULT for details.
 */
ADI_ADC_STATUS adi_adc_AssembleCommand(int32_t (*pfCalcCmdCrc)(uint8_t *, uint32_t, uint32_t *),
                                       uint8_t readWriteBit, uint16_t addr, uint8_t data,
                                       ADI_ADC_CMD *pAdcCmd, ADI_ADC_FRAME_FORMAT format);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /*__ADI_ADC_FRAME_ASSEMBLE_H__*/

/** @} */
