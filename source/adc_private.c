/******************************************************************************
 Copyright (c) 2022 - 2025  Analog Devices Inc.
******************************************************************************/

/**
 * @file     adc_private.c
 * @brief    API definitions to communicate with ADCs.
 * @{
 */

/*============= I N C L U D E S =============*/
#include "adc_private.h"
#include "ADE911X_addr_def.h"
#include "ADE911X_addr_rdef.h"
#include "ADEMA127_addr_def.h"
#include "ADEMA127_addr_rdef.h"
#include "adc_config_ade91xx.h"
#include "adc_config_adema12x.h"
#include "adc_sync.h"
#include "adi_adc.h"
#include "adi_adc_frame_assemble.h"
#include <string.h>

/*=============  D E F I N I T I O N S  =============*/
/**
 * @brief Function to get remaining number of frames
 *        in Rx buffer
 *
 */
static int32_t GetAdcNumAvailableFrames(ADI_ADC_INFO *pInfo);

/**
 * Check CRC status of responses to ADC configuration commands
 * @param[in] numFrames - Number of frames
 */
static uint32_t CheckCRCStatus(ADI_ADC_INFO *pInfo, int32_t numFrames);

static void AdcSetRxFrameBufferPtr(uint8_t *pBuffer, uint8_t numAdc, uint8_t *pBufferPtr[],
                                   ADC_TYPE_CONFIG *pTypeConfig);
static void AdcSetTxFrameBufferPtr(uint8_t *pBuffer, uint8_t numAdc, uint8_t *pBufferPtr[],
                                   ADC_TYPE_CONFIG *pTypeConfig);

static void CopySamples(int32_t *pDst, volatile uint8_t *pSrc, uint8_t numSamples);

static ADI_ADC_STATUS AdcSpiTransferBlocking(ADI_ADC_INFO *pInfo, uint8_t *pTxData,
                                             uint8_t *pRxData, uint32_t numBytes);

static ADI_ADC_STATUS AdcSetTypeConfig(ADC_TYPE_CONFIG *pTypeConfig, uint8_t numAdc,
                                       ADI_ADC_TYPE *pAdcType);

static ADI_ADC_STATUS UpdateCmdStateAfterTxRxComplete(ADI_ADC_INFO *pInfo,
                                                      uint32_t *pCallbackStatus);
static void UpdateFrameReadIdx(ADI_ADC_INFO *pInfo);

/**
 * @brief Get the maximum valid channel index for a given ADC type.
 *
 * @param adcType The ADC type.
 * @return Maximum valid channel index + 1 (i.e., total number of channels).
 */
static uint8_t GetAdcChannelLimit(ADI_ADC_TYPE adcType);

static uint8_t *UpdateWriteCmdStatus(ADI_ADC_INFO *pInfo);

/**
 * @brief Function to update the service after recieving frames.
 * @param[in]  pInfo	- Pointer to the ADC info structure.
 *
 */
static ADI_ADC_STATUS UpdatesAfterFramesRxvd(ADI_ADC_INFO *pInfo);

/**
 * @brief Calls the CRC compute function to calculate CRC of ADC response
 * @param[in]  pInfo	- Pointer to the ADC info structure.
 * @param[out]  pRxFrame	- Pointer to Rx frame.
 *
 */
static ADI_ADC_STATUS FrameCrcValidate(ADI_ADC_INFO *pInfo, volatile uint8_t *pRxFrame);

/**
 * @brief Get delayed sample from delay buffer
 * @param[in]  pInfo	- Pointer to the ADC info structure.
 * @param[in]  pChBuf	- Pointer to the delay structure.
 * @param[in]  newSample	- New sample to store.
 * @param[out]  pOutSample	- output sample.
 *
 */
static void GetDelayedSample(ADI_ADC_INFO *pInfo, ADI_ADC_DELAY_BUFFER *pChBuf, int32_t newSample,
                             int32_t *pOutSample);

/**
 * @brief Store Datapath shift
 *
 */
#define DATAPATH_SHIFT(pInfo, adcIdx, chIdx)                                                       \
    ((pInfo)->pDatapathShift[(adcIdx) * (pInfo)->maxNumChannel + (chIdx)]);

/**
 * @brief Store Datapath SCF Enable
 *
 */
#define DATAPATH_SCF_EN(pInfo, adcIdx, chIdx)                                                      \
    ((pInfo)->pDatapathScfEn[(adcIdx) * (pInfo)->maxNumChannel + (chIdx)]);

ADI_ADC_STATUS AdcAssembleNopAllAdc(ADI_ADC_INFO *pInfo, uint8_t **pTxFramePtr)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    uint32_t adcIdx = 0;
    uint8_t numAdc = pInfo->adcCfg.numAdc;
    ADC_TYPE_CONFIG *pTypeConfig = &pInfo->pTypeConfig[0];

    /* Assemble the transmit command for ADC data */
    for (adcIdx = 0; adcIdx < numAdc; adcIdx++)
    {
        AssembleNop(pInfo, &pTypeConfig[adcIdx], pTxFramePtr[adcIdx]);
    }

    return adcStatus;
}

ADI_ADC_STATUS AssembleNop(ADI_ADC_INFO *pInfo, ADC_TYPE_CONFIG *pTypeConfig, uint8_t *pAdcFrame)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    uint8_t addr;
    uint8_t value;
    ADI_ADC_CMD *pCmd;
    ADI_ADC_CONFIG *pAdcCfg = &pInfo->adcCfg;

    addr = pTypeConfig->configReg.status2.addr;
    value = 0;
    pCmd = (ADI_ADC_CMD *)(pAdcFrame + pTypeConfig->cmdOffset);

    status = adi_adc_AssembleCommand(pAdcCfg->pfCalcCmdCrc, ADI_ADC_RWB_READ, addr, value, pCmd,
                                     pAdcCfg->frameFormat);

    return status;
}

volatile uint8_t *GetAdcRxWriteFrame(ADI_ADC_INFO *pInfo)
{
    volatile ADI_ADC_RX_BUFFER *pRxBuffer = &pInfo->rxBuffer;
    int32_t writeIdx = pRxBuffer->writeIdx;
    volatile uint8_t *pRxFrame = &pRxBuffer->pAdcRxFrames[writeIdx];

    return pRxFrame;
}

static int32_t GetAdcNumAvailableFrames(ADI_ADC_INFO *pInfo)
{
    int32_t numAvailableFrames = 0;
    volatile ADI_ADC_RX_BUFFER *pRxBuffer = &pInfo->rxBuffer;
    numAvailableFrames = pRxBuffer->frameWriteIdx - pRxBuffer->frameReadIdx;
    if (numAvailableFrames < 0)
    {
        numAvailableFrames += pInfo->maxFramesInBuffer;
    }
    return numAvailableFrames;
}

int32_t AdcIsBlockReady(ADI_ADC_INFO *pInfo)
{
    uint8_t numAdc = pInfo->adcCfg.numAdc;
    uint8_t numSamplesInBlock = pInfo->adcCfg.numSamplesInBlock;
    int32_t numFramesInBlock = numAdc * numSamplesInBlock;
    int32_t status = 0;
    int32_t numFramesAvailable = GetAdcNumAvailableFrames(pInfo);
    if (numFramesInBlock <= numFramesAvailable)
    {
        status = 1;
        pInfo->blockReady = 1;
    }
    return status;
}

void CopyStatusOutput(ADC_TYPE_CONFIG *pTypeConfig, volatile uint8_t *pRxFrame,
                      ADI_ADC_CMD_TYPE *pCmdType, ADI_ADC_STATUS_OUTPUT *pAdcStatusOutput)
{
    /* Copy STATUS0 and STATUS1 from the frame. */
    pAdcStatusOutput->status0 = pRxFrame[pTypeConfig->status0Offset];
    pAdcStatusOutput->status1 = pRxFrame[pTypeConfig->status1Offset];
    /* Copy STATUS2 from the frame depending on command type. */
    if ((*pCmdType) == ADI_ADC_CMD_TYPE_NOP)
    {
        /* Get STATUS2 from data fields */
        pAdcStatusOutput->status2 = pRxFrame[pTypeConfig->dataOffset];
    }
}

ADI_ADC_STATUS FrameCrcValidate(ADI_ADC_INFO *pInfo, volatile uint8_t *pRxFrame)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    int32_t crcStatus = 0;
    ADC_TYPE_CONFIG *pTypConfig = &pInfo->pTypeConfig[0];
    uint32_t crc;
    ADI_ADC_RX_BUFFER *pRxBuffer = &pInfo->rxBuffer;
    uint8_t *pError = &pRxBuffer->pError[pRxBuffer->frameWriteIdx];
    uint8_t numAdc = pInfo->adcCfg.numAdc;
    uint32_t frameOffset = 0;
    ADC_TYPE_CONFIG *pTypeConfig = &pInfo->pTypeConfig[0];
    uint8_t frameLength;
    volatile uint8_t *pFrameStart;

    pInfo->adcCrcIndex = 0;
    while (pInfo->adcCrcIndex < numAdc)
    {
        pFrameStart = &pRxFrame[frameOffset];
        frameLength = pTypConfig[pInfo->adcCrcIndex].frameLength - 2;
        pInfo->expectedCrc =
            (uint32_t)pFrameStart[frameLength + 1] << 8 | (uint32_t)pFrameStart[frameLength];
        crcStatus = pInfo->adcCfg.pfCalcRespCrc((uint8_t *)pFrameStart, frameLength, &crc);
        if (crcStatus == 0)
        {
            if (crc == pInfo->expectedCrc)
            {
                pError[pRxBuffer->frameWriteIdx + pInfo->adcCrcIndex] = 0x0;
            }
            else
            {
                pError[pRxBuffer->frameWriteIdx + pInfo->adcCrcIndex] = 0x1;
                status = ADI_ADC_STATUS_CRC_ERROR;
            }
        }
        else
        {
            status = ADI_ADC_STATUS_CRC_CALC_FAILED;
        }
        frameOffset += pTypeConfig[pInfo->adcCrcIndex].frameLength;
        pInfo->adcCrcIndex++;
    }

    return status;
}

static uint32_t CheckCRCStatus(ADI_ADC_INFO *pInfo, int32_t numAdc)
{
    int32_t i;
    uint32_t crcStatus = 0;
    ADI_ADC_RX_BUFFER *pRxBuffer = &pInfo->rxBuffer;
    uint8_t *pError = &pRxBuffer->pError[pRxBuffer->frameReadIdx];

    for (i = 0; i < numAdc; i++)
    {
        crcStatus = crcStatus | ((pError[i] & (uint32_t)0x1) << i);
    }

    return crcStatus;
}

ADI_ADC_STATUS TransferAdcData(ADI_ADC_INFO *pInfo)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    ADI_ADC_RX_BUFFER *pRxBuffer = &pInfo->rxBuffer;
    uint8_t *pRxFramesBuff = (uint8_t *)&pRxBuffer->pAdcRxFrames[0];
    uint8_t *pTxBuffer = &pInfo->pTxBufferCmd[0];
    uint32_t numBytes = pInfo->allAdcFrameLength;
    uint8_t numAdc = pInfo->adcCfg.numAdc;
    uint32_t crcStatus = 0;
    pRxBuffer->readIdx = 0;
    pRxBuffer->writeIdx = 0;
    pRxBuffer->frameReadIdx = 0;
    pRxBuffer->frameWriteIdx = 0;

    adcStatus = AdcSpiTransferBlocking(pInfo, pTxBuffer, pRxFramesBuff, numBytes);
    if (adcStatus == ADI_ADC_STATUS_SUCCESS)
    {
        crcStatus = CheckCRCStatus(pInfo, numAdc);
        if (crcStatus != 0)
        {
            adcStatus = ADI_ADC_STATUS_CRC_ERROR;
        }
    }
    return adcStatus;
}

static ADI_ADC_STATUS UpdateCmdStateAfterTxRxComplete(ADI_ADC_INFO *pInfo,
                                                      uint32_t *pCallbackStatus)
{
    int32_t i;
    ADI_ADC_RX_BUFFER *pRxBuffer = &pInfo->rxBuffer;
    volatile uint8_t *pRxFrame = GetAdcRxWriteFrame(pInfo);
    uint32_t numBytes = pInfo->allAdcFrameLength;
    uint8_t *pLastCmdRxFrames = &pInfo->pLastCmdRxFrames[0];
    bool syncInProgress;
    ADI_ADC_CMD_TYPE *pCmdType = &pRxBuffer->pCmdType[pInfo->rxBuffer.frameWriteIdx];

    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;

    switch (pInfo->cmdStatus)
    {
    case ADI_ADC_READ_WRITE_CMD_STATUS_TX_PROGRESS:
        pInfo->cmdStatus = ADI_ADC_READ_WRITE_CMD_STATUS_TX_DONE;
        break;
    case ADI_ADC_READ_WRITE_CMD_STATUS_RX_PROGRESS:
        syncInProgress = AdcSyncGetProgressStatus(pInfo);
        if (syncInProgress == false)
        {
            *pCallbackStatus = *pCallbackStatus | ADI_ADC_EVENT_BITM_RESPONSE_READY;
            /* We are not expecting buffer to change while mempcy happens.*/
            memcpy(pLastCmdRxFrames, (void *)pRxFrame, numBytes);
            pInfo->cmdStatus = ADI_ADC_READ_WRITE_CMD_STATUS_IDLE;
        }
        for (i = 0; i < pInfo->adcCfg.numAdc; i++)
        {
            pCmdType[i] = ADI_ADC_CMD_TYPE_CUSTOM_CMD;
        }
        break;
    case ADI_ADC_READ_WRITE_CMD_STATUS_IDLE:
        break;
    case ADI_ADC_READ_WRITE_CMD_STATUS_TX_DONE:
    case ADI_ADC_READ_WRITE_CMD_STATUS_RX_DONE:
    case ADI_ADC_READ_WRITE_CMD_STATUS_QUEUED:
        /* These states are not expected to be hit for Tx*/
        adcStatus = ADI_ADC_STATUS_INTERNAL_ERROR;
        break;
    }

    return adcStatus;
}

ADI_ADC_STATUS SetConfigurations(ADI_ADC_INFO *pInfo)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    ADI_ADC_CONFIG *pAdcCfg = &pInfo->adcCfg;
    ADI_ADC_RX_BUFFER *pRxBuffer = &pInfo->rxBuffer;
    uint8_t numSamplesInBlock = pAdcCfg->numSamplesInBlock;
    uint8_t numAdc = pAdcCfg->numAdc;
    uint8_t i;
    uint32_t allAdcFrameLength = 0;
    // Set frame format to long during initialzation.
    pAdcCfg->frameFormat = ADI_ADC_FRAME_FORMAT_LONG;
    if (numAdc <= 0)
    {
        status = ADI_ADC_STATUS_INVALID_NUM_ADC;
    }
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = AdcSetTypeConfig(&pInfo->pTypeConfig[0], numAdc, pAdcCfg->pAdcType);
    }

    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        pInfo->maxFramesInBuffer = numAdc * (numSamplesInBlock + 2);
        memset(&pInfo->pTxBuffer[0], 0xFF,
               sizeof(uint8_t) * numAdc * ADI_ADC_LONG_FRAME_NBYTES_MAX);
        memset(&pInfo->pTxBufferCmd[0], 0xFF,
               sizeof(uint8_t) * numAdc * ADI_ADC_LONG_FRAME_NBYTES_MAX);
        memset(&pRxBuffer->pAdcRxFrames[0], 0x0,
               sizeof(uint8_t) * (numAdc * (numSamplesInBlock + 2)) *
                   ADI_ADC_LONG_FRAME_NBYTES_MAX);
        memset(&pRxBuffer->pError[0], 0x0, sizeof(uint8_t) * (numAdc * (numSamplesInBlock + 2)));
        memset(&pRxBuffer->pCmdType[0], 0x0,
               sizeof(ADI_ADC_CMD_TYPE) * (numAdc * (numSamplesInBlock + 2)));
#if (APP_CFG_USE_TIMESTAMP == 1)
        memset(&pRxBuffer->pTimestamp[0], 0x0,
               sizeof(uint32_t) * (numAdc * (numSamplesInBlock + 2)));
#endif

        AdcSyncAbort(pInfo);

        AdcSetTxFrameBufferPtr(&pInfo->pTxBuffer[0], numAdc, &pInfo->pTxFramePtr[0],
                               &pInfo->pTypeConfig[0]);
        AdcSetTxFrameBufferPtr(&pInfo->pTxBufferCmd[0], numAdc, &pInfo->pTxCmdFramePtr[0],
                               &pInfo->pTypeConfig[0]);

        allAdcFrameLength = 0;
        for (i = 0; i < numAdc; i++)
        {
            allAdcFrameLength += pInfo->pTypeConfig[i].frameLength;
        }
        pInfo->allAdcFrameLength = allAdcFrameLength;
        for (i = 0; i < (numSamplesInBlock + 1); i++)
        {
            AdcSetRxFrameBufferPtr(&pRxBuffer->pAdcRxFrames[i * allAdcFrameLength], numAdc,
                                   &pInfo->pRxFramePtr[i * numAdc], &pInfo->pTypeConfig[0]);
        }
    }
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = AdcResetStates(pInfo);
    }
    pInfo->isAdcSpiRxComplete = true;

    return status;
}

ADI_ADC_STATUS ResetConfigurations(ADI_ADC_INFO *pInfo)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    ADI_ADC_CONFIG *pAdcCfg = &pInfo->adcCfg;
    uint8_t numAdc = pAdcCfg->numAdc;
    uint8_t numSamplesInBlock = pAdcCfg->numSamplesInBlock;
    ADI_ADC_RX_BUFFER *pRxBuffer = &pInfo->rxBuffer;
    uint8_t i;
    uint32_t allAdcFrameLength = 0;

    if ((numAdc > 1) && (pInfo->adcCfg.frameFormat == ADI_ADC_FRAME_FORMAT_SHORT))
    {
        status = ADI_ADC_STATUS_INCORRECT_FRAME_FORMAT_SET;
    }

    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        memset(&pInfo->pTxBuffer[0], 0xFF,
               sizeof(uint8_t) * numAdc * ADI_ADC_LONG_FRAME_NBYTES_MAX);
        memset(&pInfo->pTxBufferCmd[0], 0xFF,
               sizeof(uint8_t) * numAdc * ADI_ADC_LONG_FRAME_NBYTES_MAX);
        memset(&pRxBuffer->pAdcRxFrames[0], 0x0,
               sizeof(uint8_t) * (numAdc * (numSamplesInBlock + 2)) *
                   ADI_ADC_LONG_FRAME_NBYTES_MAX);
        pRxBuffer->readIdx = 0;
        pRxBuffer->writeIdx = 0;
        pRxBuffer->frameReadIdx = 0;
        pRxBuffer->frameWriteIdx = 0;
        memset(&pRxBuffer->pError[0], 0x0, sizeof(uint8_t) * (numAdc * (numSamplesInBlock + 2)));
        memset(&pRxBuffer->pCmdType[0], 0x0,
               sizeof(ADI_ADC_CMD_TYPE) * (numAdc * (numSamplesInBlock + 2)));
#if (APP_CFG_USE_TIMESTAMP == 1)
        memset(&pRxBuffer->pTimestamp[0], 0x0,
               sizeof(uint32_t) * (numAdc * (numSamplesInBlock + 2)));
#endif

        AdcSetTxFrameBufferPtr(&pInfo->pTxBuffer[0], numAdc, &pInfo->pTxFramePtr[0],
                               &pInfo->pTypeConfig[0]);
        AdcSetTxFrameBufferPtr(&pInfo->pTxBufferCmd[0], numAdc, &pInfo->pTxCmdFramePtr[0],
                               &pInfo->pTypeConfig[0]);

        allAdcFrameLength = 0;
        for (i = 0; i < numAdc; i++)
        {
            allAdcFrameLength += pInfo->pTypeConfig[i].frameLength;
        }
        pInfo->allAdcFrameLength = allAdcFrameLength;
        for (i = 0; i < (numSamplesInBlock + 1); i++)
        {
            AdcSetRxFrameBufferPtr(&pRxBuffer->pAdcRxFrames[i * allAdcFrameLength], numAdc,
                                   &pInfo->pRxFramePtr[i * numAdc], &pInfo->pTypeConfig[0]);
        }
    }

    return status;
}

void AdcSetRxFrameBufferPtr(uint8_t *pBuffer, uint8_t numAdc, uint8_t *pBufferPtr[],
                            ADC_TYPE_CONFIG *pTypeConfig)
{
    uint8_t i = 0;
    uint8_t currentBuffIdx = 0;

    for (i = 0; i < numAdc; i++)
    {
        pBufferPtr[i] = &pBuffer[currentBuffIdx];
        currentBuffIdx += pTypeConfig[i].frameLength;
    }
}

void AdcSetTxFrameBufferPtr(uint8_t *pBuffer, uint8_t numAdc, uint8_t *pBufferPtr[],
                            ADC_TYPE_CONFIG *pTypeConfig)
{
    uint8_t i = 0;
    uint8_t currentBuffIdx = 0;

    for (i = 0; i < numAdc; i++)
    {
        pBufferPtr[i] = &pBuffer[currentBuffIdx];
        currentBuffIdx += (pTypeConfig[i].frameLength);
    }
}

ADI_ADC_STATUS AdcReadBlock(ADI_ADC_INFO *pInfo, int32_t *pBuffer,
                            ADI_ADC_STATUS_OUTPUT *pAdcStatusOutput)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_NO_DATA;
    uint8_t adcIdx, sampleIdx;
    uint8_t cnt = 0;
    uint8_t cntPrev = 0;
    uint8_t *pError;
    uint8_t *pRxFramesBuff;
    ADI_ADC_CMD_TYPE *pCmdType;
    uint8_t samplesPerFrame;
#if (APP_CFG_USE_TIMESTAMP == 1)
    uint32_t *pTimestamp;
    uint8_t timestampCnt = 0;
#endif

    ADI_ADC_RX_BUFFER *pRxBuffer = &pInfo->rxBuffer;
    ADI_ADC_CONFIG *pAdcCfg = &pInfo->adcCfg;
    uint8_t numAdc = pAdcCfg->numAdc;
    uint8_t numSamplesInBlock = pAdcCfg->numSamplesInBlock;

    memset(pAdcStatusOutput, 0x0, (sizeof(ADI_ADC_STATUS_OUTPUT) * numAdc));
#if (APP_CFG_USE_TIMESTAMP == 1)
    memset(pInfo->runData.pTimestamp, 0x0, sizeof(uint32_t) * (numSamplesInBlock * numAdc));
#endif
    if (pInfo->blockReady == 1)
    {
        pInfo->blockReady = 0;
        /* Copy required samples from the circular buffer.
         * Iterate over #APP_CFG_NUM_SAMPLES_IN_BLOCK to get
         * (APP_CFG_NUM_SAMPLES_IN_BLOCK*numAdc*samplesPerFrame) samples.
         */
        for (sampleIdx = 0; sampleIdx < numSamplesInBlock; sampleIdx++)
        {
            /* Iterate over all the numAdc ADC's to get each
             * sample for each of the ADI_ADC_MAX_NUM_CHANNELS_PER_ADC
             * channels.
             */
            cntPrev = 0;
            pRxFramesBuff = &pRxBuffer->pAdcRxFrames[pRxBuffer->readIdx];
            pError = &pRxBuffer->pError[pRxBuffer->frameReadIdx];
            pCmdType = &pRxBuffer->pCmdType[pRxBuffer->frameReadIdx];
#if (APP_CFG_USE_TIMESTAMP == 1)
            pTimestamp = &pRxBuffer->pTimestamp[pRxBuffer->frameReadIdx];
#endif

            for (adcIdx = 0; adcIdx < numAdc; adcIdx++)
            {
                samplesPerFrame = pInfo->pTypeConfig[adcIdx].samplesPerFrame;
                if (pError[adcIdx] == 0)
                {
                    /* Copy samplesPerFrame samples that is present in each frame,
                     * which constitutes one sample from all the channels of one ADC.
                     */
                    CopySamples(&pBuffer[cnt], pRxFramesBuff, samplesPerFrame);
                    /* Copy one sample from each adc to separate buffer */
                    memcpy(&pInfo->pPrevSamples[cntPrev], &pBuffer[cnt], samplesPerFrame * 4);
                }
                else
                {
                    memcpy(&pBuffer[cnt], &pInfo->pPrevSamples[cntPrev], samplesPerFrame * 4);
                }

                /* Copy the ADC STATUS0-2 from the last frame of each ADC. */
                if (sampleIdx == (numSamplesInBlock - 1))
                {
                    CopyStatusOutput(&pInfo->pTypeConfig[adcIdx], pRxFramesBuff, &pCmdType[adcIdx],
                                     &pAdcStatusOutput[adcIdx]);
                }
                /* Perform OR of CRC calculated of all the frames corresponding to each ADC. */
                pAdcStatusOutput[adcIdx].crcError |= pError[adcIdx];
#if (APP_CFG_USE_TIMESTAMP == 1)
                /* Copy timestamps for the current block */
                pInfo->runData.pTimestamp[timestampCnt++] = pTimestamp[adcIdx];
#endif
                cnt += samplesPerFrame;
                cntPrev += samplesPerFrame;
                pRxFramesBuff += pInfo->pTypeConfig[adcIdx].frameLength;
            }
            UpdateFrameReadIdx(pInfo);
        }
        adcStatus = ADI_ADC_STATUS_SUCCESS;
    }

    return adcStatus;
}

void UpdateFrameReadIdx(ADI_ADC_INFO *pInfo)
{
    ADI_ADC_RX_BUFFER *pRxBuffer = &pInfo->rxBuffer;

    pRxBuffer->frameReadIdx += pInfo->adcCfg.numAdc;
    pRxBuffer->readIdx += pInfo->allAdcFrameLength;
    if (pRxBuffer->frameReadIdx >= (int32_t)pInfo->maxFramesInBuffer)
    {
        pRxBuffer->frameReadIdx = 0;
        pRxBuffer->readIdx = 0;
    }
}

static void CopySamples(int32_t *pDst, volatile uint8_t *pSrc, uint8_t numSamples)
{
    int32_t val = 0;
    for (uint8_t i = 0; i < numSamples; i++)
    {
        // form the channel waveform sample by WAV_LO | WAV_MD | WAV_HI
        val = (int32_t)((pSrc[4 * i + 1]) | (pSrc[4 * i + 2] << 8) | (pSrc[4 * i + 3] << 16));
        // Sign-extend to 32 bits if the 24-bit number is negative
        if (val & (1 << 23))
        {                      // Check if the 24th bit is set (negative number)
            val |= 0xFF000000; // Set the upper 8 bits to 1s for sign extension
        }
        pDst[i] = val;
    }
}

ADI_ADC_STATUS AdcSpiTransferBlocking(ADI_ADC_INFO *pInfo, uint8_t *pTxData, uint8_t *pRxData,
                                      uint32_t numBytes)
{

    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    int32_t spiStatus = 0;
    ADI_ADC_CONFIG *pConfig = &pInfo->adcCfg;

    if ((pConfig->pfTransceive == NULL) || (pInfo->adcCfg.hUser == NULL))
    {
        status = ADI_ADC_STATUS_NULL_PTR;
    }
    else
    {
        spiStatus =
            pConfig->pfTransceive(pInfo->adcCfg.hUser, (uint8_t *)pTxData, pRxData, numBytes);
        if (spiStatus == 0)
        {
            /* Not looking at return code because CRC errors are checked using CheckCRCStatus
             * function
             */
            AdcProcessRxFrames(pInfo);
        }
        else
        {
            status = ADI_ADC_STATUS_TRANSCEIVE_FAILED;
        }
    }

    return status;
}

ADI_ADC_STATUS AdcGetRunStatus(ADI_ADC_INFO *pInfo, bool *pRunStatus)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;

    memcpy(&pInfo->runData, pRunStatus, sizeof(pInfo->runData));

    return adcStatus;
}

static ADI_ADC_STATUS AdcSetTypeConfig(ADC_TYPE_CONFIG *pTypeConfig, uint8_t numAdc,
                                       ADI_ADC_TYPE *pAdcType)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    int32_t idx;

    for (idx = 0; idx < numAdc; idx++)
    {
        if (pAdcType[idx] == ADI_ADC_TYPE_ADE91XX)
        {
            status = AdcInitAde91xx(&pTypeConfig[idx]);
        }
        else if (pAdcType[idx] == ADI_ADC_TYPE_ADEMA124)
        {
            status = AdcInitAdema124(&pTypeConfig[idx]);
        }
        else if (pAdcType[idx] == ADI_ADC_TYPE_ADEMA127)
        {
            status = AdcInitAdema127(&pTypeConfig[idx]);
        }
        else
        {
            status = ADI_ADC_STATUS_INVALID_ADC_TYPE;
        }
        if (status != ADI_ADC_STATUS_SUCCESS)
        {
            break;
        }
    }

    return status;
}

ADI_ADC_STATUS AdcCollectSamples(ADI_ADC_INFO *pInfo, uint32_t timestamp)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    uint8_t *pAdcTxBuffer;
    volatile uint8_t *pRxFrame;
    uint8_t *pError;
    uint32_t numBytes;
    ADI_ADC_CONFIG *pConfig;
    int32_t numFramesAvailable;
    int32_t i;
#if (APP_CFG_USE_TIMESTAMP == 1)
    uint32_t *pTimestamp;
#else
    (void)timestamp; // Suppress unused variable warning if timestamp is not used
#endif

    pRxFrame = GetAdcRxWriteFrame(pInfo);
    pError = &pInfo->rxBuffer.pError[pInfo->rxBuffer.frameWriteIdx];
#if (APP_CFG_USE_TIMESTAMP == 1)
    pTimestamp = &pInfo->rxBuffer.pTimestamp[pInfo->rxBuffer.frameWriteIdx];
#endif
    numBytes = pInfo->allAdcFrameLength;
    pConfig = &pInfo->adcCfg;

    pAdcTxBuffer = UpdateWriteCmdStatus(pInfo);
    if (pInfo->isAdcSpiRxComplete == false)
    {
        pInfo->runData.busyCount++;
        status = ADI_ADC_STATUS_PREV_CMD_RUNNING;
    }
    else
    {
        numFramesAvailable = GetAdcNumAvailableFrames(pInfo);
        if (numFramesAvailable >= (int32_t)(pInfo->maxFramesInBuffer - 1))
        {
            pInfo->rxBufferOverflow += 1;
            pInfo->runData.overflowCount++;
            status = ADI_ADC_STATUS_BUFFER_OVERFLOW;
        }
        else
        {
            pInfo->rxBufferOverflow = 0;
        }
        if (status != ADI_ADC_STATUS_BUFFER_OVERFLOW || pConfig->ignoreRxBufferOverflow == 1)
        {
            /**
             * Collect ADC data if ignoreRxBufferOverflow is set to 1.
             * Error is returned to notify the user and they can decide to ignore the data.
             */
            for (i = 0; i < pInfo->adcCfg.numAdc; i++)
            {
                pError[i] = 0;
#if (APP_CFG_USE_TIMESTAMP == 1)
                /* Store the timestamp for each sample */
                pTimestamp[i] = timestamp;
#endif
            }
            pInfo->runData.currentState = ADI_ADC_RUN_STATE_TRANSEIVE_IN_PROGRESS;
            pInfo->isAdcSpiRxComplete = false;
            pConfig->pfTransceiveAsync(pInfo->adcCfg.hUser, (uint8_t *)pAdcTxBuffer,
                                       (uint8_t *)pRxFrame, numBytes);
        }
    }

    return status;
}

uint8_t *UpdateWriteCmdStatus(ADI_ADC_INFO *pInfo)
{
    uint8_t *pAdcTxBuffer;
    pAdcTxBuffer = &pInfo->pTxBuffer[0];
    switch (pInfo->cmdStatus)
    {
    case ADI_ADC_READ_WRITE_CMD_STATUS_QUEUED:
        pAdcTxBuffer = &pInfo->pTxBufferCmd[0];
        pInfo->cmdStatus = ADI_ADC_READ_WRITE_CMD_STATUS_TX_PROGRESS;
        break;
    case ADI_ADC_READ_WRITE_CMD_STATUS_TX_DONE:
        pInfo->cmdStatus = ADI_ADC_READ_WRITE_CMD_STATUS_RX_PROGRESS;
        break;
        // Below States are updated from Rx Callback
    case ADI_ADC_READ_WRITE_CMD_STATUS_IDLE:
    case ADI_ADC_READ_WRITE_CMD_STATUS_TX_PROGRESS:
    case ADI_ADC_READ_WRITE_CMD_STATUS_RX_PROGRESS:
        break;
    case ADI_ADC_READ_WRITE_CMD_STATUS_RX_DONE:
        pInfo->cmdStatus = ADI_ADC_READ_WRITE_CMD_STATUS_IDLE;
        break;
    }

    return pAdcTxBuffer;
}

ADI_ADC_STATUS UpdatesAfterFramesRxvd(ADI_ADC_INFO *pInfo)
{
    ADI_ADC_RX_BUFFER *pRxBuffer = &pInfo->rxBuffer;
    uint8_t numAdc = pInfo->adcCfg.numAdc;
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    uint32_t callbackStatus = 0;

    AdcSyncAlign(pInfo);

    UpdateCmdStateAfterTxRxComplete(pInfo, &callbackStatus);
    pInfo->runData.currentState = ADI_ADC_RUN_STATE_READY;
    pInfo->runData.numSamplesCollected++;

    pRxBuffer->frameWriteIdx += numAdc;
    pRxBuffer->writeIdx += pInfo->allAdcFrameLength;
    if (pRxBuffer->frameWriteIdx >= (int32_t)pInfo->maxFramesInBuffer)
    {
        pRxBuffer->frameWriteIdx = 0;
        /* Both wrap around should be at same time. */
        pRxBuffer->writeIdx = 0;
    }

    if (AdcIsBlockReady(pInfo))
    {
        callbackStatus = callbackStatus | ADI_ADC_EVENT_BITM_BLOCK_READY;
    }

    if ((pInfo->adcCfg.pfCallback != NULL) && (callbackStatus != 0))
    {
        pInfo->runData.currentState = ADI_ADC_RUN_STATE_CALLBACK_IN_PROGRESS;
        pInfo->adcCfg.pfCallback(pInfo->adcCfg.hUser, callbackStatus);
        pInfo->runData.currentState = ADI_ADC_RUN_STATE_IDLE;
    }

    return status;
}

ADI_ADC_STATUS AdcProcessRxFrames(ADI_ADC_INFO *pInfo)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    volatile uint8_t *pRxFrame;

    pRxFrame = GetAdcRxWriteFrame(pInfo);
    pInfo->isAdcSpiRxComplete = true;
    pInfo->runData.currentState = ADI_ADC_RUN_STATE_CRC_CHCK_IN_PROGRESS;
    adcStatus = FrameCrcValidate(pInfo, pRxFrame);
    UpdatesAfterFramesRxvd(pInfo);

    return adcStatus;
}

ADI_ADC_STATUS AdcClearQuantizationNoise(ADI_ADC_INFO *pInfo, int32_t *pBuffer)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    ADI_ADC_CONFIG *pAdcCfg = &pInfo->adcCfg;
    uint8_t numSamplesInBlock = pAdcCfg->numSamplesInBlock;
    int32_t channelIdx;
    int32_t sampleIdx;
    int32_t numAdc = pAdcCfg->numAdc;
    uint8_t shift;
    uint8_t scfEn;
    int32_t idx = 0;
    int32_t adcIdx;

    for (sampleIdx = 0; sampleIdx < numSamplesInBlock; sampleIdx++)
    {
        for (adcIdx = 0; adcIdx < numAdc; adcIdx++)
        {
            for (channelIdx = 0; channelIdx < pInfo->pTypeConfig[adcIdx].samplesPerFrame;
                 channelIdx++)
            {
                scfEn = DATAPATH_SCF_EN(pInfo, adcIdx, channelIdx);
                shift = DATAPATH_SHIFT(pInfo, adcIdx, channelIdx);

                if (scfEn)
                {
                    /** Correct quantization noise */
                    pBuffer[idx] = pBuffer[idx] + (APP_CFG_QUANTIZATION_NOISE << shift);
                }
                idx++;
            }
        }
    }

    return status;
}

ADI_ADC_STATUS AdcReadBlockWithDelay(ADI_ADC_INFO *pInfo, int32_t *pBuffer,
                                     ADI_ADC_STATUS_OUTPUT *pAdcStatusOutput)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_NO_DATA;
    uint8_t adcIdx, sampleIdx;
    uint8_t cnt = 0;
    uint8_t cntPrev = 0;
    uint8_t *pError;
    uint8_t *pRxFramesBuff;
    ADI_ADC_CMD_TYPE *pCmdType;
    uint8_t samplesPerFrame;
    ADI_ADC_RX_BUFFER *pRxBuffer = &pInfo->rxBuffer;
    ADI_ADC_CONFIG *pAdcCfg = &pInfo->adcCfg;
    uint8_t numAdc = pAdcCfg->numAdc;
    uint8_t numSamplesInBlock = pAdcCfg->numSamplesInBlock;
    uint8_t maxNumChannelPerAdc = pInfo->maxNumChannelPerAdc;

    uint8_t globalCh = 0;
    uint8_t chIdx = 0;
    ADI_ADC_DELAY_BUFFER *pChBuf;

#if (APP_CFG_USE_TIMESTAMP == 1)
    uint32_t *pTimestamp;
    uint8_t timestampCnt = 0;
    memset(pInfo->runData.pTimestamp, 0x0, sizeof(uint32_t) * (numSamplesInBlock * numAdc));
#endif
    memset(pAdcStatusOutput, 0x0, sizeof(ADI_ADC_STATUS_OUTPUT) * pAdcCfg->numAdc);
    memset(pInfo->pSampleLinearBuf, 0x0, sizeof(int32_t) * (maxNumChannelPerAdc));

    if (pInfo->blockReady == 1)
    {
        pInfo->blockReady = 0;
        adcStatus = ADI_ADC_STATUS_SUCCESS;

        for (sampleIdx = 0; sampleIdx < numSamplesInBlock; sampleIdx++)
        {
            cntPrev = 0;
            pRxFramesBuff = &pRxBuffer->pAdcRxFrames[pRxBuffer->readIdx];
            pError = &pRxBuffer->pError[pRxBuffer->frameReadIdx];
            pCmdType = &pRxBuffer->pCmdType[pRxBuffer->frameReadIdx];
#if (APP_CFG_USE_TIMESTAMP == 1)
            pTimestamp = &pRxBuffer->pTimestamp[pRxBuffer->frameReadIdx];
#endif

            for (adcIdx = 0; adcIdx < numAdc; adcIdx++)
            {
                samplesPerFrame = pInfo->pTypeConfig[adcIdx].samplesPerFrame;
                if (pError[adcIdx] == 0)
                {
                    CopySamples(pInfo->pSampleLinearBuf, pRxFramesBuff, samplesPerFrame);

                    for (chIdx = 0; chIdx < samplesPerFrame; chIdx++)
                    {
                        globalCh = adcIdx * maxNumChannelPerAdc + chIdx;
                        pChBuf = &pInfo->pChannelDelayBuffers[globalCh];

                        GetDelayedSample(pInfo, pChBuf, pInfo->pSampleLinearBuf[chIdx],
                                         &pBuffer[cnt + chIdx]);
                    }

                    memcpy(&pInfo->pPrevSamples[cntPrev], pBuffer,
                           samplesPerFrame * sizeof(int32_t));
                }
                else
                {
                    memcpy(&pBuffer[cnt], &pInfo->pPrevSamples[cntPrev],
                           samplesPerFrame * sizeof(int32_t));
                }

                /* Copy the ADC STATUS0-2 from the last frame of each ADC. */
                if (sampleIdx == (numSamplesInBlock - 1))
                {
                    CopyStatusOutput(&pInfo->pTypeConfig[adcIdx], pRxFramesBuff, &pCmdType[adcIdx],
                                     &pAdcStatusOutput[adcIdx]);
                }
                /* Perform OR of CRC calculated of all the frames corresponding to each ADC. */
                pAdcStatusOutput[adcIdx].crcError |= pError[adcIdx];
#if (APP_CFG_USE_TIMESTAMP == 1)
                pInfo->runData.pTimestamp[timestampCnt++] = pTimestamp[adcIdx];
#endif
                cnt += samplesPerFrame;
                cntPrev += samplesPerFrame;
                pRxFramesBuff += pInfo->pTypeConfig[adcIdx].frameLength;
            }

            UpdateFrameReadIdx(pInfo);
        }
    }

    return adcStatus;
}

ADI_ADC_STATUS AdcSetIntegerSampleDelay(ADI_ADC_INFO *pInfo, uint8_t *pIntegerDelay,
                                        uint8_t *pChanIdx, int8_t numChan, int8_t adcNum)
{
    int32_t i;
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    uint8_t globalCh;
    int8_t adcIdx;
    ADI_ADC_DELAY_BUFFER *pChBuf;
    uint8_t maxNumChannelPerAdc = pInfo->maxNumChannelPerAdc;

    for (i = 0; i < numChan; i++)
    {
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            if (pIntegerDelay[i] > pInfo->adcCfg.maxSampleDelay)
            {
                adcStatus = ADI_ADC_STATUS_INVALID_SAMPLE_DELAY;
                break;
            }
            if (adcStatus == ADI_ADC_STATUS_SUCCESS)
            {
                if (adcNum == -1)
                {
                    for (adcIdx = 0; adcIdx < pInfo->adcCfg.numAdc; adcIdx++)
                    {
                        globalCh = adcIdx * maxNumChannelPerAdc + pChanIdx[i];
                        pInfo->adcCfg.pIntegerSampleDelay[globalCh] = pIntegerDelay[i];

                        // Update readIdx based on current writeIdx and delay
                        pChBuf = &pInfo->pChannelDelayBuffers[globalCh];
                        pChBuf->readIdx =
                            (pChBuf->writeIdx + pInfo->delayBuffSize - pIntegerDelay[i]) %
                            pInfo->delayBuffSize;
                    }
                }
                else
                {
                    globalCh = adcNum * maxNumChannelPerAdc + pChanIdx[i];
                    pInfo->adcCfg.pIntegerSampleDelay[globalCh] = pIntegerDelay[i];

                    // Update readIdx based on current writeIdx and delay
                    pChBuf = &pInfo->pChannelDelayBuffers[globalCh];
                    pChBuf->readIdx = (pChBuf->writeIdx + pInfo->delayBuffSize - pIntegerDelay[i]) %
                                      pInfo->delayBuffSize;
                }
            }
        }
    }

    return adcStatus;
}

ADI_ADC_STATUS CheckChannelValid(ADI_ADC_INFO *pInfo, int8_t adcIdx, uint8_t *pChanIdx,
                                 int8_t numChan)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    int8_t i, ch;
    uint8_t maxChannels;

    if (adcIdx == -1)
    {
        for (i = 0; i < pInfo->adcCfg.numAdc; i++)
        {
            maxChannels = GetAdcChannelLimit(pInfo->adcCfg.pAdcType[i]);
            if (maxChannels == 0)
            {
                status = ADI_ADC_STATUS_INVALID_CHANNEL_INDEX;
            }

            for (ch = 0; ch < numChan; ch++)
            {
                if (status == ADI_ADC_STATUS_SUCCESS)
                {
                    if (pChanIdx[ch] >= maxChannels)
                    {
                        status = ADI_ADC_STATUS_INVALID_CHANNEL_INDEX;
                    }
                }
            }

            if (status != ADI_ADC_STATUS_SUCCESS)
            {
                break;
            }
        }
    }
    else
    {
        maxChannels = GetAdcChannelLimit(pInfo->adcCfg.pAdcType[adcIdx]);
        if (maxChannels == 0)
        {
            status = ADI_ADC_STATUS_INVALID_CHANNEL_INDEX;
        }

        for (ch = 0; ch < numChan; ch++)
        {
            if (status == ADI_ADC_STATUS_SUCCESS)
            {
                if (pChanIdx[ch] >= maxChannels)
                {
                    status = ADI_ADC_STATUS_INVALID_CHANNEL_INDEX;
                }
            }
        }
    }

    return status;
}

/**
 * @brief Get the maximum valid channel index for a given ADC type.
 *
 * @param adcType The ADC type.
 * @return Maximum valid channel index + 1 (i.e., total number of channels).
 */
static uint8_t GetAdcChannelLimit(ADI_ADC_TYPE adcType)
{
    uint8_t numCh = 0;

    if (adcType == ADI_ADC_TYPE_ADEMA124)
    {
        numCh = 4;
    }
    else if (adcType == ADI_ADC_TYPE_ADEMA127)
    {
        numCh = 7;
    }
    else if (adcType == ADI_ADC_TYPE_ADE91XX)
    {
        numCh = 3;
    }
    else
    {
        numCh = 0; // Invalid ADC type
    }

    return numCh;
}

ADI_ADC_STATUS AllocateMemory(ADI_ADC_INFO *pInfo, uint32_t *pStateMemory, uint32_t stateMemorySize,
                              ADI_ADC_CONFIG *pConfig)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    uint32_t offset = 0;
    uint8_t idx;
    uint8_t numAdc = pConfig->numAdc;
    uint8_t blockSize = pConfig->numSamplesInBlock;
    uint8_t maxNumChannel = pInfo->maxNumChannel;
    uint8_t maxNumChannelPerAdc = pInfo->maxNumChannelPerAdc;
    uint8_t delayBuffSize = pInfo->delayBuffSize;

    // Allocate 32-bit aligned pointers
    pInfo->pRxFramePtr = (uint8_t **)&pStateMemory[offset];
    offset += numAdc * (blockSize + 2);

    pInfo->pTxCmdFramePtr = (uint8_t **)&pStateMemory[offset];
    offset += numAdc;

    pInfo->pTxFramePtr = (uint8_t **)&pStateMemory[offset];
    offset += numAdc;

    pInfo->pTypeConfig = (ADC_TYPE_CONFIG *)&pStateMemory[offset];
    offset += (sizeof(ADC_TYPE_CONFIG) * numAdc + 3) / sizeof(uint32_t);

    pInfo->pSampleLinearBuf = (int32_t *)&pStateMemory[offset];
    offset += maxNumChannelPerAdc;

    pInfo->pPrevSamples = (int32_t *)&pStateMemory[offset];
    offset += maxNumChannel;

    pInfo->adcCfg.pAdcType = (ADI_ADC_TYPE *)&pStateMemory[offset];
    offset += numAdc;

    pInfo->rxBuffer.pCmdType = (ADI_ADC_CMD_TYPE *)&pStateMemory[offset];
    offset += (numAdc * (blockSize + 2));

    pInfo->pChannelDelayBuffers = (ADI_ADC_DELAY_BUFFER *)&pStateMemory[offset];
    offset += (sizeof(ADI_ADC_DELAY_BUFFER) * maxNumChannel + 3) / sizeof(uint32_t);

    for (idx = 0; idx < maxNumChannel; idx++)
    {
        pInfo->pChannelDelayBuffers[idx].pBuffer = (int32_t *)&pStateMemory[offset];
        offset += (sizeof(int32_t) * delayBuffSize) / sizeof(uint32_t);
    }

#if (APP_CFG_USE_TIMESTAMP == 1)

    pInfo->rxBuffer.pTimestamp = (uint32_t *)&pStateMemory[offset];
    offset += numAdc * (blockSize + 2);

    pInfo->runData.pTimestamp = (uint32_t *)&pStateMemory[offset];
    offset += blockSize * numAdc;
#endif

    pInfo->adcCfg.pIntegerSampleDelay = (uint8_t *)&pStateMemory[offset];
    offset += maxNumChannel;

    pInfo->pDatapathShift = (uint8_t *)&pStateMemory[offset];
    offset += numAdc * maxNumChannel;

    pInfo->pDatapathScfEn = (uint8_t *)&pStateMemory[offset];
    offset += numAdc * maxNumChannel;

    pInfo->pLastCmdRxFrames = (uint8_t *)&pStateMemory[offset];
    offset += numAdc * ADI_ADC_LONG_FRAME_NBYTES_MAX;

    pInfo->pTxBufferCmd = (uint8_t *)&pStateMemory[offset];
    offset += numAdc * ADI_ADC_LONG_FRAME_NBYTES_MAX;

    pInfo->pTxBuffer = (uint8_t *)&pStateMemory[offset];
    offset += numAdc * ADI_ADC_LONG_FRAME_NBYTES_MAX;

    pInfo->rxBuffer.pAdcRxFrames = (uint8_t *)&pStateMemory[offset];
    offset += (numAdc * (blockSize + 2)) * ADI_ADC_LONG_FRAME_NBYTES_MAX;

    pInfo->rxBuffer.pError = (uint8_t *)&pStateMemory[offset];
    offset += (numAdc * (blockSize + 2));

    if (offset > (stateMemorySize / sizeof(uint32_t)))
    {
        return ADI_ADC_STATUS_INSUFFICIENT_STATE_MEMORY;
    }

    return status;
}

static void GetDelayedSample(ADI_ADC_INFO *pInfo, ADI_ADC_DELAY_BUFFER *pChBuf, int32_t newSample,
                             int32_t *pOutSample)
{
    // Store the new sample in the circular buffer
    pChBuf->pBuffer[pChBuf->writeIdx] = newSample;

    // Read the delayed sample
    *pOutSample = pChBuf->pBuffer[pChBuf->readIdx];

    // Increment and wrap readIdx
    pChBuf->readIdx++;
    if (pChBuf->readIdx >= pInfo->delayBuffSize)
        pChBuf->readIdx = 0;

    // Increment and wrap writeIdx
    pChBuf->writeIdx++;
    if (pChBuf->writeIdx >= pInfo->delayBuffSize)
        pChBuf->writeIdx = 0;
}

ADI_ADC_STATUS SetMaxChannels(uint8_t numAdc, ADI_ADC_TYPE *pAdcType, ADI_ADC_INFO *pInfo)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    uint8_t i;
    pInfo->maxNumChannel = 0;

    for (i = 0; i < numAdc; i++)
    {
        if (pAdcType[i] == ADI_ADC_TYPE_ADEMA127)
        {
            pInfo->maxNumChannel += 7;
            pInfo->maxNumChannelPerAdc = 7;
        }
        else if (pAdcType[i] == ADI_ADC_TYPE_ADEMA124)
        {
            pInfo->maxNumChannel += 4;
            pInfo->maxNumChannelPerAdc = 4;
        }
        else if (pAdcType[i] == ADI_ADC_TYPE_ADE91XX)
        {
            pInfo->maxNumChannel += 3;
            pInfo->maxNumChannelPerAdc = 3;
        }
        else
        {
            status = ADI_ADC_STATUS_INVALID_ADC_TYPE;
        }
    }

    return status;
}

/**
 * @}
 */
