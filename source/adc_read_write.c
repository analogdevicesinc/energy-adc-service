/******************************************************************************
 Copyright (c) 2024 - 2025  Analog Devices Inc.
******************************************************************************/

/**
 * @file     adc_read_write.c
 * @brief    API definitions to communicate with ADCs.
 * @{
 */

/*============= I N C L U D E S =============*/
#include "adc_private.h"
#include "adc_sync.h"
#include "adi_adc.h"
#include "adi_adc_frame_assemble.h"
#include <string.h>

static ADI_ADC_STATUS AssembleReadWrite(ADI_ADC_INFO *pInfo, uint8_t operation, uint16_t address,
                                        uint8_t value, int8_t adcIdx);

ADI_ADC_STATUS AdcWriteRegister(ADI_ADC_INFO *pInfo, uint16_t address, uint8_t value, int8_t adcIdx)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;

    /* Assemble a WRITE command with given address and value for the specified
      ADC index */

    if (adcIdx < pInfo->adcCfg.numAdc)
    {
        /* Assembles a READ/WRITE command with given register address and value
           to be issued over the next SPI transfer */
        status = AssembleReadWrite(pInfo, ADI_ADC_RWB_WRITE, address, value, adcIdx);

        if (status == ADI_ADC_STATUS_SUCCESS)
        {
            /*Transmit the WRITE command over SPI to the ADC */
            status = TransferAdcData(pInfo);
        }
        if (status == ADI_ADC_STATUS_SUCCESS)
        {
            AdcAssembleNopAllAdc(pInfo, &pInfo->pTxCmdFramePtr[0]);
        }
        if (status == ADI_ADC_STATUS_SUCCESS)
        {
            /*Transmit the NOP command over SPI to the ADC */
            status = TransferAdcData(pInfo);
        }
    }
    else
    {
        status = ADI_ADC_STATUS_INVALID_ADC_INDEX;
    }
    return status;
}

ADI_ADC_STATUS AdcAssembleWriteRegister(ADI_ADC_INFO *pInfo, uint16_t address, uint8_t value,
                                        int8_t adcIdx)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    bool syncInProgress;

    syncInProgress = AdcSyncGetProgressStatus(pInfo);
    if (syncInProgress == false)
    {
        adcStatus = AdcAssembleReadWrite(pInfo, ADI_ADC_RWB_WRITE, address, value, adcIdx);
    }
    else
    {
        adcStatus = ADI_ADC_STATUS_ALIGN_IN_PROGRESS;
    }

    return adcStatus;
}

ADI_ADC_STATUS AdcAssembleReadWrite(ADI_ADC_INFO *pInfo, uint8_t operation, uint16_t address,
                                    uint8_t value, int8_t adcIdx)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    uint8_t numAdc = pInfo->adcCfg.numAdc;

    if (adcIdx < numAdc)
    {
        status = AssembleReadWrite(pInfo, operation, address, value, adcIdx);
    }
    else
    {
        status = ADI_ADC_STATUS_INVALID_ADC_INDEX;
    }

    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        pInfo->cmdStatus = ADI_ADC_READ_WRITE_CMD_STATUS_QUEUED;
    }

    return status;
}

ADI_ADC_STATUS AdcReadRegister(ADI_ADC_INFO *pInfo, uint16_t address, int8_t adcIdx,
                               uint8_t *pBuffer, uint32_t *pNumBytes)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    uint8_t numAdc = pInfo->adcCfg.numAdc;
    uint8_t *pRxFrame;

    if (adcIdx < numAdc)
    {
        /* Assembles a READ/WRITE command with given register address and value
           to be issued over the next SPI transfer */
        status = AssembleReadWrite(pInfo, ADI_ADC_RWB_READ, address, 0, adcIdx);

        if (status == ADI_ADC_STATUS_SUCCESS)
        {
            /* Transmit the READ command over SPI to the ADC */
            status = TransferAdcData(pInfo);
        }
        if (status == ADI_ADC_STATUS_SUCCESS)
        {
            /* Issue a NOP command in order to receive the register value along
               with its response. */
            AdcAssembleNopAllAdc(pInfo, &pInfo->pTxCmdFramePtr[0]);
            /* Transmit the NOP command over SPI to the ADC and get data
               for the previous READ command */
            status = TransferAdcData(pInfo);
        }
        if (status == ADI_ADC_STATUS_SUCCESS)
        {
            pRxFrame = pInfo->pRxFramePtr[pInfo->rxBuffer.frameReadIdx];
            /* Copy data Field from the Rx frame to the destination buffer */
            CopyDataFromRxFrame(pInfo, adcIdx, pRxFrame, pBuffer, pNumBytes);
        }
    }
    else
    {
        status = ADI_ADC_STATUS_INVALID_ADC_INDEX;
    }

    return status;
}

static ADI_ADC_STATUS AssembleReadWrite(ADI_ADC_INFO *pInfo, uint8_t operation, uint16_t address,
                                        uint8_t value, int8_t adcIdx)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    int32_t i = 0;
    uint8_t numAdc = pInfo->adcCfg.numAdc;
    ADI_ADC_CMD *pCmd;
    ADC_TYPE_CONFIG *pTypeConfig = &pInfo->pTypeConfig[0];
    ADI_ADC_CONFIG *pAdcCfg = &pInfo->adcCfg;

    memset(pInfo->pTxBufferCmd, 0xFF, sizeof(uint8_t) * numAdc * ADI_ADC_LONG_FRAME_NBYTES_MAX);
    if (adcIdx < numAdc)
    {
        if (adcIdx == -1)
        {
            /* Assemble command for all ADCs */
            for (i = 0; i < numAdc; i++)
            {
                pCmd = (ADI_ADC_CMD *)(pInfo->pTxCmdFramePtr[i] + pTypeConfig[i].cmdOffset);
                status |= adi_adc_AssembleCommand(pAdcCfg->pfCalcCmdCrc, operation, address, value,
                                                  pCmd, pAdcCfg->frameFormat);
            }
        }
        else
        {
            /* Assemble command for specified adcIdx, other ADCs are NOP */
            for (i = 0; i < numAdc; i++)
            {
                if (i == adcIdx)
                {
                    pCmd = (ADI_ADC_CMD *)(pInfo->pTxCmdFramePtr[i] + pTypeConfig[i].cmdOffset);
                    status |= adi_adc_AssembleCommand(pAdcCfg->pfCalcCmdCrc, operation, address,
                                                      value, pCmd, pAdcCfg->frameFormat);
                }
                else
                {
                    status |= AssembleNop(pInfo, &pTypeConfig[i], pInfo->pTxCmdFramePtr[i]);
                }
            }
        }
    }
    else
    {
        status = ADI_ADC_STATUS_INVALID_ADC_INDEX;
    }

    return status;
}

ADI_ADC_STATUS CopyDataFromRxFrame(ADI_ADC_INFO *pInfo, int8_t adcIdx, uint8_t *pRxFrame,
                                   volatile uint8_t *pDst, uint32_t *pNumBytes)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    /* Number of bytes in the data field of a frame */
    uint8_t numBytesInData = 2;
    int32_t i;
    uint8_t numAdc = pInfo->adcCfg.numAdc;
    ADC_TYPE_CONFIG *pTypeConfig = &pInfo->pTypeConfig[0];
    uint8_t frameStartIdx = 0;

    if (adcIdx < numAdc)
    {
        if (adcIdx == -1)
        {
            /* If ADC index is -1, iterate over response frame of all
             * ADC's and copy the data field into the destination buffer*/
            for (i = 0; i < numAdc; i++)
            {
                memcpy((uint8_t *)pDst, &pRxFrame[frameStartIdx + pTypeConfig[i].dataOffset],
                       numBytesInData);
                frameStartIdx += pTypeConfig[i].frameLength;
                pDst += numBytesInData;
            }
            *pNumBytes = numBytesInData * numAdc;
        }
        else
        {
            for (i = 0; i < adcIdx; i++)
            {
                frameStartIdx += pTypeConfig[i].frameLength;
            }
            memcpy((uint8_t *)pDst, &pRxFrame[frameStartIdx + pTypeConfig[adcIdx].dataOffset],
                   numBytesInData);
            pDst += numBytesInData;
            *pNumBytes = numBytesInData;
        }
    }
    else
    {
        status = ADI_ADC_STATUS_INVALID_ADC_INDEX;
    }

    return status;
}

/**
 * @}
 */
