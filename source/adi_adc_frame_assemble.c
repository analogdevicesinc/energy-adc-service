/******************************************************************************
 Copyright (c) 2023 - 2025  Analog Devices Inc.
******************************************************************************/

/**
 * @file     adi_adc_frame_assemble.c
 * @brief    Function to assemble ADC command frame
 * @{
 */

#include "adi_adc_frame_assemble.h"
#include <stdint.h>
#include <string.h>

ADI_ADC_STATUS adi_adc_AssembleCommand(int32_t (*pfCalcCmdCrc)(uint8_t *, uint32_t, uint32_t *),
                                       uint8_t readWriteBit, uint16_t addr, uint8_t data,
                                       ADI_ADC_CMD *pAdcCmd, ADI_ADC_FRAME_FORMAT format)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    int32_t crcStatus = 0;
    uint32_t crc = 0;

    pAdcCmd->addr = addr & 0xFF; // Extract lower 8 bits;
    pAdcCmd->data = data;
    pAdcCmd->addrHi = (addr >> 8) & 0x3F; // Extract bits [8:13]
    pAdcCmd->rwb = readWriteBit;
    pAdcCmd->formatLong = format;
    /* number of bytes will be (size of ADI_ADC_CMD structure) - 1.
     * -1 to exclude CRC byte field*/
    if (pfCalcCmdCrc != NULL)
    {
        crcStatus = pfCalcCmdCrc((uint8_t *)pAdcCmd, sizeof(ADI_ADC_CMD) - 1, &crc);
        if (crcStatus == 0)
        {
            pAdcCmd->crc = crc;
        }
        else
        {
            status = ADI_ADC_STATUS_CRC_CALC_FAILED;
        }
    }

    return status;
}

/**
 * @}
 */
