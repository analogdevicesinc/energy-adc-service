/******************************************************************************
 Copyright (c) 2022 - 2025  Analog Devices Inc.
******************************************************************************/

/**
 * @file     adi_adc.c
 * @brief    API definitions to communicate with ADCs.
 * @{
 */

/*============= I N C L U D E S =============*/
#include "adi_adc.h"
#include "ADE911X_addr_rdef.h"
#include "ADEMA127_addr_def.h"
#include "ADEMA127_addr_rdef.h"
#include "adc_private.h"
#include "adc_sync.h"
#include <string.h>

/*=============  D E F I N I T I O N S  =============*/
ADI_ADC_STATUS adi_adc_Create(ADI_ADC_HANDLE *phAdc, void *pStateMemory, uint32_t stateMemorySize)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;

    ADI_ADC_INFO *pInfo = NULL;
    uint32_t reqSize = sizeof(ADI_ADC_INFO);

    /* Check the given pointers before we set their contents */
    if ((phAdc == (void *)NULL) || (pStateMemory == (void *)NULL))
    {
        status = ADI_ADC_STATUS_NULL_PTR;
    }

    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        /* Set handle to NULL in case of error */
        *phAdc = (ADI_ADC_HANDLE)NULL;
        if (stateMemorySize < reqSize)
        {
            status = ADI_ADC_STATUS_INSUFFICIENT_STATE_MEMORY;
        }
        else
        {
            pInfo = (ADI_ADC_INFO *)pStateMemory;
            memset(pInfo, 0, sizeof(ADI_ADC_INFO));
        }
    }

    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        *phAdc = (ADI_ADC_HANDLE *)pInfo;
    }

    return status;
}

ADI_ADC_STATUS adi_adc_Init(ADI_ADC_HANDLE hAdc, ADI_ADC_CONFIG *pConfig)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    ADI_ADC_INFO *pInfo = (ADI_ADC_INFO *)hAdc;

    if (hAdc == NULL)
    {
        adcStatus = ADI_ADC_STATUS_NULL_PTR;
    }
    else
    {
        if (pConfig == NULL)
        {
            pConfig = &pInfo->adcCfg;
            adcStatus = AdcSetDefaultConfig(pConfig);
        }

        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = adi_adc_SetConfig(hAdc, pConfig);
        }
    }

    return adcStatus;
}

ADI_ADC_STATUS adi_adc_SetConfig(ADI_ADC_HANDLE hAdc, ADI_ADC_CONFIG *pConfig)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;

    ADI_ADC_INFO *pInfo = (ADI_ADC_INFO *)hAdc;
    if ((hAdc == NULL) || (pConfig == NULL))
    {
        status = ADI_ADC_STATUS_NULL_PTR;
    }
    else
    {
        pInfo->adcCfg = *pConfig;

        if (status == ADI_ADC_STATUS_SUCCESS)
        {
            status = SetConfigurations(pInfo);
        }

        pInfo->cmdStatus = ADI_ADC_READ_WRITE_CMD_STATUS_IDLE;
    }

    return status;
}

ADI_ADC_STATUS adi_adc_GetConfig(ADI_ADC_HANDLE hAdc, ADI_ADC_CONFIG *pConfig)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;

    ADI_ADC_INFO *pInfo = (ADI_ADC_INFO *)hAdc;
    if (pConfig == NULL || hAdc == NULL)
    {
        adcStatus = ADI_ADC_STATUS_NULL_PTR;
    }
    else
    {
        *pConfig = pInfo->adcCfg;
    }

    return adcStatus;
}

ADI_ADC_STATUS adi_adc_EnableClockOut(ADI_ADC_HANDLE hAdc, uint8_t adcIdx)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    ADI_ADC_INFO *pInfo = (ADI_ADC_INFO *)hAdc;

    if (hAdc == NULL)
    {
        adcStatus = ADI_ADC_STATUS_NULL_PTR;
    }
    else
    {

        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = AdcEnableClockOut(pInfo, adcIdx);
        }
    }

    return adcStatus;
}

ADI_ADC_STATUS adi_adc_ConfigureAdcs(ADI_ADC_HANDLE hAdc, ADI_ADC_CONFIG_REGISTERS *pConfigReg)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    ADI_ADC_INFO *pInfo = (ADI_ADC_INFO *)hAdc;

    if (hAdc == NULL)
    {
        adcStatus = ADI_ADC_STATUS_NULL_PTR;
    }
    else
    {
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = AdcConfigure(pInfo, pConfigReg);
        }
    }

    return adcStatus;
}

ADI_ADC_STATUS adi_adc_ReadBlock(ADI_ADC_HANDLE hAdc, int32_t *pBuffer,
                                 ADI_ADC_STATUS_OUTPUT *pAdcStatusOutput)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    ADI_ADC_INFO *pInfo = (ADI_ADC_INFO *)hAdc;

    if (hAdc == NULL)
    {
        status = ADI_ADC_STATUS_NULL_PTR;
    }
    else
    {
#ifdef APP_CFG_USE_SAMPLE_DELAY
        status = AdcReadBlockWithDelay(pInfo, pBuffer, pAdcStatusOutput);
#else
        status = AdcReadBlock(pInfo, pBuffer, pAdcStatusOutput);
#endif
        if (status == ADI_ADC_STATUS_SUCCESS)
        {
            status = AdcClearQuantizationNoise(pInfo, pBuffer);
        }
    }

    return status;
}

ADI_ADC_STATUS adi_adc_ResetFrameBuffer(ADI_ADC_HANDLE hAdc)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    ADI_ADC_INFO *pInfo = (ADI_ADC_INFO *)hAdc;

    if (hAdc == NULL)
    {
        adcStatus = ADI_ADC_STATUS_NULL_PTR;
    }
    else
    {
        AdcResetStates(pInfo);
    }

    return adcStatus;
}

ADI_ADC_STATUS AdcResetStates(ADI_ADC_INFO *pInfo)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    volatile ADI_ADC_RX_BUFFER *pRxBuffer = &pInfo->rxBuffer;

    AdcAssembleNopAllAdc(pInfo, &pInfo->txFramePtr[0]);

    pInfo->blockReady = 0;
    pRxBuffer->readIdx = 0;
    pRxBuffer->writeIdx = 0;
    pRxBuffer->frameReadIdx = 0;
    pRxBuffer->frameWriteIdx = 0;
    pInfo->runData.currentState = ADI_ADC_RUN_STATE_READY;
    pInfo->cmdStatus = ADI_ADC_READ_WRITE_CMD_STATUS_IDLE;

    // Clear the read and write indices of delay buffers
    for (uint8_t ch = 0; ch < APP_CFG_MAX_NUM_CHANNELS; ch++)
    {
        pInfo->channelDelayBuffers[ch].writeIdx = 0;
        memset(pInfo->channelDelayBuffers[ch].buffer, 0,
               sizeof(pInfo->channelDelayBuffers[ch].buffer));
    }
    memset((void *)pRxBuffer->adcRxFrames, 0, sizeof(pRxBuffer->adcRxFrames));
    memset((void *)pRxBuffer->error, 0, sizeof(pRxBuffer->error));
    memset((void *)pRxBuffer->cmdType, 0, sizeof(pRxBuffer->cmdType));
#if (APP_CFG_USE_TIMESTAMP == 1)
    memset((void *)pRxBuffer->timestamp, 0, sizeof(pRxBuffer->timestamp));
#endif
    return adcStatus;
}

ADI_ADC_STATUS adi_adc_WriteRegister(ADI_ADC_HANDLE hAdc, uint16_t address, uint8_t value,
                                     int8_t adcIdx)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    ADI_ADC_INFO *pInfo = (ADI_ADC_INFO *)hAdc;

    if (hAdc == NULL)
    {
        adcStatus = ADI_ADC_STATUS_NULL_PTR;
    }
    else
    {
        adcStatus = AdcWriteRegister(pInfo, address, value, adcIdx);
    }

    return adcStatus;
}

ADI_ADC_STATUS adi_adc_ReadRegister(ADI_ADC_HANDLE hAdc, uint16_t address, int8_t adcIdx,
                                    uint8_t *pBuffer, uint32_t *pNumBytes)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    ADI_ADC_INFO *pInfo = (ADI_ADC_INFO *)hAdc;

    if (hAdc == NULL)
    {
        adcStatus = ADI_ADC_STATUS_NULL_PTR;
    }
    else
    {
        adcStatus = AdcReadRegister(pInfo, address, adcIdx, pBuffer, pNumBytes);
    }

    return adcStatus;
}

ADI_ADC_STATUS adi_adc_AssembleWriteRegister(ADI_ADC_HANDLE hAdc, uint16_t address, uint8_t value,
                                             int8_t adcIdx)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    ADI_ADC_INFO *pInfo = (ADI_ADC_INFO *)hAdc;

    if (hAdc == NULL)
    {
        adcStatus = ADI_ADC_STATUS_NULL_PTR;
    }
    else
    {
        adcStatus = AdcAssembleWriteRegister(pInfo, address, value, adcIdx);
    }

    return adcStatus;
}

ADI_ADC_STATUS adi_adc_AssembleReadRegister(ADI_ADC_HANDLE hAdc, uint16_t address, int8_t adcIdx)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    ADI_ADC_INFO *pInfo = (ADI_ADC_INFO *)hAdc;
    bool syncInProgress;

    if (hAdc == NULL)
    {
        adcStatus = ADI_ADC_STATUS_NULL_PTR;
    }
    else
    {
        syncInProgress = AdcSyncGetProgressStatus(pInfo);
        if (syncInProgress == false)
        {
            adcStatus = AdcAssembleReadWrite(hAdc, ADI_ADC_RWB_READ, address, 0x00, adcIdx);
        }
        else
        {
            adcStatus = ADI_ADC_STATUS_ALIGN_IN_PROGRESS;
        }
    }

    return adcStatus;
}

ADI_ADC_STATUS adi_adc_GetLastRegister(ADI_ADC_HANDLE hAdc, int8_t adcIdx, uint8_t *pBuffer,
                                       uint32_t *pNumBytes)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_LAST_REGISTER_NOT_READY;
    ADI_ADC_INFO *pInfo = (ADI_ADC_INFO *)hAdc;
    uint8_t *pLastCmdRxFrames = &pInfo->lastCmdRxFrames[0];

    if (hAdc == NULL)
    {
        adcStatus = ADI_ADC_STATUS_NULL_PTR;
    }
    else
    {
        if (pInfo->cmdStatus == ADI_ADC_READ_WRITE_CMD_STATUS_IDLE)
        {
            /* The response to the READ command issued is expected to be present in
               lastCmdRxFrames. Copy the data field from this to the destination buffer */
            CopyDataFromRxFrame(pInfo, adcIdx, pLastCmdRxFrames, pBuffer, pNumBytes);
            adcStatus = ADI_ADC_STATUS_SUCCESS;
        }
    }

    return adcStatus;
}

ADI_ADC_STATUS adi_adc_StartAlign(ADI_ADC_HANDLE hAdc)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    ADI_ADC_INFO *pInfo = (ADI_ADC_INFO *)hAdc;
    bool syncInProgress;

    if (hAdc == NULL)
    {
        adcStatus = ADI_ADC_STATUS_NULL_PTR;
    }
    else
    {
        syncInProgress = AdcSyncGetProgressStatus(pInfo);
        if (syncInProgress == false)
        {
            adcStatus = AdcSyncStart(pInfo);
        }
        else
        {
            adcStatus = ADI_ADC_STATUS_ALIGN_IN_PROGRESS;
        }
    }

    return adcStatus;
}

ADI_ADC_STATUS adi_adc_Align(ADI_ADC_HANDLE hAdc)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    ADI_ADC_INFO *pInfo = (ADI_ADC_INFO *)hAdc;

    if (hAdc == NULL)
    {
        adcStatus = ADI_ADC_STATUS_NULL_PTR;
    }
    else
    {
        adcStatus = AlignAdc(pInfo);
    }

    return adcStatus;
}

ADI_ADC_STATUS adi_adc_CollectSamples(ADI_ADC_HANDLE hAdc, uint32_t timestamp)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;

    ADI_ADC_INFO *pInfo = (ADI_ADC_INFO *)hAdc;

    if (hAdc == NULL)
    {
        status = ADI_ADC_STATUS_NULL_PTR;
    }
    else
    {
        if (pInfo->adcCfg.frameFormat == ADI_ADC_FRAME_FORMAT_SHORT)
        {
            status = adi_adc_SetAdcFrameFormat(hAdc, ADI_ADC_FRAME_FORMAT_LONG);
        }
        if (status == ADI_ADC_STATUS_SUCCESS)
        {
            status = AdcCollectSamples(pInfo, timestamp);
        }
    }

    return status;
}

ADI_ADC_STATUS adi_adc_GetLastFrameStatus(ADI_ADC_HANDLE hAdc,
                                          ADI_ADC_STATUS_OUTPUT *pAdcStatusOutput)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    ADI_ADC_INFO *pInfo = (ADI_ADC_INFO *)hAdc;
    uint32_t adcIdx;
    volatile ADI_ADC_RX_BUFFER *pRxBuffer;
    uint8_t numAdc;
    volatile uint8_t *pCrcError;
    volatile ADI_ADC_CMD_TYPE *pCmdType;
    volatile uint8_t *pRxFrames;

    if (hAdc == NULL)
    {
        adcStatus = ADI_ADC_STATUS_NULL_PTR;
    }
    else
    {
        memset(pAdcStatusOutput, 0x0, sizeof(ADI_ADC_STATUS_OUTPUT));
        pRxBuffer = &pInfo->rxBuffer;
        numAdc = pInfo->adcCfg.numAdc;
        pCrcError = &pRxBuffer->error[pRxBuffer->frameReadIdx];
        pCmdType = &pRxBuffer->cmdType[pRxBuffer->frameReadIdx];
        pRxFrames = &pRxBuffer->adcRxFrames[0];

        for (adcIdx = 0; adcIdx < numAdc; adcIdx++)
        {
            pAdcStatusOutput[adcIdx].crcError = pCrcError[adcIdx];
            /* NOTE: We are assuming that  frame buffer will not wrap around at the middle
             * */
            CopyStatusOutput(&pInfo->typeConfig[adcIdx], &pRxFrames[pRxBuffer->readIdx],
                             (ADI_ADC_CMD_TYPE *)&pCmdType[adcIdx], &pAdcStatusOutput[adcIdx]);
        }
    }

    return adcStatus;
}

ADI_ADC_STATUS adi_adc_ValidateSamples(ADI_ADC_HANDLE hAdc)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    ADI_ADC_INFO *pInfo = (ADI_ADC_INFO *)hAdc;

    if (hAdc == NULL)
    {
        adcStatus = ADI_ADC_STATUS_NULL_PTR;
    }
    else
    {
        AdcProcessRxFrames(pInfo);
    }
    return adcStatus;
}

ADI_ADC_STATUS adi_adc_GetRunData(ADI_ADC_HANDLE hAdc, ADI_ADC_RUN_DATA *pData)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    int32_t i;
    uint32_t totalChannels = 0;

    ADI_ADC_INFO *pInfo = (ADI_ADC_INFO *)hAdc;

    if (hAdc == NULL)
    {
        adcStatus = ADI_ADC_STATUS_NULL_PTR;
    }
    else
    {
        *pData = pInfo->runData;
        for (i = 0; i < pInfo->adcCfg.numAdc; i++)
        {
            pData->channelsPerAdc[i] = pInfo->typeConfig[i].samplesPerFrame;
            totalChannels += pInfo->typeConfig[i].samplesPerFrame;
        }
        pData->totalChannels = totalChannels;
    }

    return adcStatus;
}

ADI_ADC_STATUS adi_adc_SetSamplingRate(ADI_ADC_HANDLE hAdc, ADI_ADC_CONFIG_REGISTERS *pConfigReg)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    ADI_ADC_INFO *pInfo = (ADI_ADC_INFO *)hAdc;
    uint8_t idx;
    uint8_t numAdc = 0;

    if (hAdc == NULL)
    {
        adcStatus = ADI_ADC_STATUS_NULL_PTR;
    }
    else
    {
        numAdc = pInfo->adcCfg.numAdc;
        for (idx = 0; idx < numAdc; idx++)
        {
            if (adcStatus == ADI_ADC_STATUS_SUCCESS)
            {
                adcStatus = pInfo->typeConfig[idx].pfPopulateUserCfgReg(
                    &pInfo->typeConfig[idx].configReg, &pConfigReg[idx]);
            }
        }
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = AdcSetSamplingRate(pInfo);
        }
    }

    return adcStatus;
}

ADI_ADC_STATUS adi_adc_GetAdcIdxAndChan(ADI_ADC_HANDLE hAdc, uint8_t slotNum, uint8_t *pAdcIdx,
                                        uint8_t *pChanNum)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_INCORRECT_SLOT_CONFIG;
    ADI_ADC_INFO *pInfo = (ADI_ADC_INFO *)hAdc;
    uint8_t chanLow = 0;
    uint8_t chanHigh;
    uint8_t numAdc;

    if (hAdc == NULL)
    {
        adcStatus = ADI_ADC_STATUS_NULL_PTR;
    }
    else
    {
        numAdc = pInfo->adcCfg.numAdc;

        // Loop over all the ADCs to find the slot number
        for (uint8_t i = 0; i < numAdc; i++)
        {
            chanHigh = chanLow + pInfo->typeConfig[i].samplesPerFrame;
            if (slotNum >= chanLow && slotNum < chanHigh)
            {
                *pAdcIdx = i;
                *pChanNum = slotNum - chanLow;
                adcStatus = ADI_ADC_STATUS_SUCCESS;
                break;
            }
            chanLow = chanHigh;
        }
    }

    return adcStatus;
}

ADI_ADC_STATUS adi_adc_SetAdcFrameFormat(ADI_ADC_HANDLE hAdc, uint8_t format)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    ADI_ADC_INFO *pInfo = (ADI_ADC_INFO *)hAdc;
    uint8_t numAdc;

    if (hAdc == NULL)
    {
        adcStatus = ADI_ADC_STATUS_NULL_PTR;
    }
    else
    {
        numAdc = pInfo->adcCfg.numAdc;

        if ((numAdc > 1 && format == ADI_ADC_FRAME_FORMAT_SHORT) || (format > 1))
        {
            adcStatus = ADI_ADC_STATUS_INCORRECT_FRAME_FORMAT_SET;
        }

        if ((numAdc == 1) && (adcStatus == ADI_ADC_STATUS_SUCCESS))
        {
            if (format != pInfo->adcCfg.frameFormat)
            {
                pInfo->adcCfg.frameFormat = format;
                adcStatus = pInfo->typeConfig->pfSetFrameFormat(hAdc, format);
            }
        }

        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = ResetConfigurations(pInfo);
        }

        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = adi_adc_ResetFrameBuffer(hAdc);
        }
    }

    return adcStatus;
}

ADI_ADC_STATUS adi_adc_SetIntegerSampleDelay(ADI_ADC_HANDLE hAdc, uint8_t *pIntegerDelay,
                                             uint8_t *pChanIdx, int8_t numChan, int8_t adcIdx)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    ADI_ADC_INFO *pInfo;

    if (hAdc == NULL)
    {
        adcStatus = ADI_ADC_STATUS_NULL_PTR;
    }
    else
    {
        pInfo = (ADI_ADC_INFO *)hAdc;
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = CheckChannelValid(pInfo, adcIdx, pChanIdx, numChan);
        }
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = AdcSetIntegerSampleDelay(pInfo, pIntegerDelay, pChanIdx, numChan, adcIdx);
        }
    }

    return adcStatus;
}

/**
 * @}
 */
