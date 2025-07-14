/******************************************************************************
 Copyright (c) 2022 - 2025  Analog Devices Inc.
******************************************************************************/

/**
 * @file        adc_example_test_cmd.c
 * @brief       Interface which calls ADC APIs.
 * These functions can be used as an example for library API usage.
 * @defgroup    ADC_EXM ADC Example codes
 * @{
 */

/*============= I N C L U D E S =============*/
#include "adc_example.h"
#include "adc_example_ucomm.h"
#include "adi_adc.h"
#include "adi_ucomm.h"
#ifdef ENABLE_SIMULATION
#include "simul_adc.h"
#endif
#include "adc_service_dsp_interface.h"
#include <string.h>

/*=============  D E F I N I T I O N S  =============*/
/** Command for board init */
#define ADC_BOARD_INIT 0xF0
/** Command for block size config */
#define ADC_TEST_CMD_SET_BLOCK_SIZE 0xF1
/** Command for sttaus */
#define ADC_TEST_CMD_GET_RESPONSE_STATUS 0xF5
/** Command for setting the timestamp stored as part of collect samples */
#define ADC_CMD_SET_TIMESTAMP 0xF6
/** Command for testing channel phase offset */
#define ADC_CMD_WRITE_READ_CHANNEL_PHASE_OFFSET 0xF7
/** Command for testing datarate */
#define ADC_CMD_WRITE_READ_DATARATE 0xF8
/** Command for testing datapath config */
#define ADC_CMD_WRITE_READ_DATAPATH_CONFIG 0xF9
/** Command for setting the ADEMA127 Shift */
#define ADC_CMD_SET_ADEMA127_SHIFT 0xFE
/** Command for testing HPF Coeffs */
#define ADC_CMD_WRITE_READ_HPF_COEFFS 0xFA
/** Command for testing LPF Coeffs */
#define ADC_CMD_WRITE_READ_LPF_COEFFS 0xFB
/** Command for testing COMP FILT Coeffs */
#define ADC_CMD_WRITE_READ_COMP_FILT_COEFFS 0xFC
/** Command for testing HPF CUTOFF */
#define ADC_CMD_WRITE_READ_HPF_CUT_OFF 0xFD

static void ProcessWriteReadChannelPhaseOffset(ADC_EXAMPLE *pExample);
static void ProcessWriteReadDatarate(ADC_EXAMPLE *pExample);
static void ProcessWriteReadTimestamp(ADC_EXAMPLE *pExample);
static void ProcessSetAdema127Shift(ADC_EXAMPLE *pExample);
static void ProcessWriteReadHpfCoeffs(ADC_EXAMPLE *pExample);
static void ProcessWriteReadLpfCoeffs(ADC_EXAMPLE *pExample);
static void ProcessWriteReadCompFilterCoeff(ADC_EXAMPLE *pExample);
static void ProcessWriteReadHpfCutOff(ADC_EXAMPLE *pExample);

ADC_EXAMPLE_STATUS ProcessTestCommand(ADC_EXAMPLE *pExample, uint16_t command)
{
    ADC_EXAMPLE_STATUS status = ADC_EXAMPLE_STATUS_SUCCESS;
    uint32_t idx;
    EXAMPLE_UCOMM_INFO *pCommInfo = pExample->pCommInfo;
    ADI_UCOMM_PACKET *pRxPacket = &pCommInfo->rxPacket;
    uint8_t numAdc;
    uint32_t simulStatus = 0;
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    ADC_INTERFACE_INFO *pAdcIf = pExample->pAdcIf;
    uint32_t totalSampleBlockSize;
    uint32_t numMaxBlocks;

    switch (command)
    {
    case ADC_BOARD_INIT:
        numAdc = pRxPacket->payload[0];
        for (idx = 0; idx < numAdc; idx++)
        {
            pExample->adcTypes[idx] = (ADI_ADC_TYPE)pRxPacket->payload[idx + 1];
        }
#ifdef ENABLE_SIMULATION
        simulStatus = EvbConnectAdc(numAdc, &pExample->adcTypes[0]);
#endif
        SendStatus(pCommInfo, simulStatus, ADI_UCOMM_ERROR_MASK);
        break;
    case ADC_TEST_CMD_SET_BLOCK_SIZE:
        pExample->pAdcIf->adcCfg.numSamplesInBlock = pRxPacket->payload[0];
        totalSampleBlockSize =
            pExample->pAdcIf->adcCfg.numSamplesInBlock * pExample->pAdcIf->runInfo.totalChannels;
        numMaxBlocks = APP_CFG_EXM_MAX_SAMPLES_TO_STORE / totalSampleBlockSize;
        pExample->samplesBuffer.bufferSize = pExample->pAdcIf->adcCfg.numSamplesInBlock *
                                             pExample->pAdcIf->runInfo.totalChannels * numMaxBlocks;
        adcStatus = AdcIfSetConfig(pExample->pAdcIf);
        SendStatus(pCommInfo, (uint32_t)adcStatus << 16, ADI_UCOMM_ERROR_MASK);
        break;
    case ADC_TEST_CMD_GET_RESPONSE_STATUS:
        adcStatus = AdcIfGetLastFrameStatus(pAdcIf);
        SendResponse(pExample->pCommInfo, (uint8_t *)&pAdcIf->adcStatusOutput[0],
                     sizeof(pAdcIf->adcStatusOutput[0]) * pAdcIf->adcCfg.numAdc);
        break;
#if (APP_CFG_USE_TIMESTAMP == 1)
    case ADC_CMD_SET_TIMESTAMP:
        ProcessWriteReadTimestamp(pExample);
        break;
#endif
    case ADC_CMD_WRITE_READ_CHANNEL_PHASE_OFFSET:
        ProcessWriteReadChannelPhaseOffset(pExample);
        break;
    case ADC_CMD_WRITE_READ_DATARATE:
        ProcessWriteReadDatarate(pExample);
        break;
    case ADC_CMD_SET_ADEMA127_SHIFT:
        ProcessSetAdema127Shift(pExample);
        break;
    case ADC_CMD_WRITE_READ_HPF_COEFFS:
        ProcessWriteReadHpfCoeffs(pExample);
        break;
    case ADC_CMD_WRITE_READ_LPF_COEFFS:
        ProcessWriteReadLpfCoeffs(pExample);
        break;
    case ADC_CMD_WRITE_READ_COMP_FILT_COEFFS:
        ProcessWriteReadCompFilterCoeff(pExample);
        break;
    case ADC_CMD_WRITE_READ_HPF_CUT_OFF:
        ProcessWriteReadHpfCutOff(pExample);
        break;
    default:
        status = ADC_EXAMPLE_STATUS_INVALID_CMD;

        break;
    }

    return status;
}

static void ProcessWriteReadChannelPhaseOffset(ADC_EXAMPLE *pExample)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    EXAMPLE_UCOMM_INFO *pCommInfo = pExample->pCommInfo;
    ADC_INTERFACE_INFO *pAdcIf = pExample->pAdcIf;
    ADI_UCOMM_PACKET *pRxPacket = &pCommInfo->rxPacket;
    uint8_t payloadCnt = 0;
    int8_t adcIdx;
    uint8_t numCh;
    uint8_t idx;
    memset(&pExample->chIdx[0], 0, sizeof(pExample->chIdx));
    memcpy(&adcIdx, &pRxPacket->payload[payloadCnt++], 1);
    memcpy(&numCh, &pRxPacket->payload[payloadCnt++], 1);
    memcpy(&pExample->chIdx[0], &pRxPacket->payload[payloadCnt], numCh);
    payloadCnt += numCh;

    for (idx = 0; idx < numCh; idx++)
    {
        memcpy(&pExample->channelPhaseOffset[idx], &pRxPacket->payload[payloadCnt], sizeof(float));
        payloadCnt += sizeof(float);
    }
    status = AdcIfSetPhaseOffset(pAdcIf, &pExample->channelPhaseOffset[0], &pExample->chIdx[0],
                                 numCh, adcIdx);
    memset(&pExample->channelPhaseOffset[0], 0, sizeof(pExample->channelPhaseOffset));
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = AdcIfGetPhaseOffset(pAdcIf, &pExample->chIdx[0], numCh, adcIdx,
                                     &pExample->channelPhaseOffset[0]);
    }

    if (status != ADI_ADC_STATUS_SUCCESS)
    {
        SendStatus(pCommInfo, (uint32_t)status, ADI_UCOMM_ERROR_MASK);
    }
    else
    {
        SendResponse(pExample->pCommInfo, (uint8_t *)&pExample->channelPhaseOffset[0],
                     sizeof(float) * numCh);
    }
}

static void ProcessWriteReadDatarate(ADC_EXAMPLE *pExample)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    EXAMPLE_UCOMM_INFO *pCommInfo = pExample->pCommInfo;
    ADC_INTERFACE_INFO *pAdcIf = pExample->pAdcIf;
    ADI_UCOMM_PACKET *pRxPacket = &pCommInfo->rxPacket;

    status = AdcIfSetDatarate(pAdcIf, pRxPacket->payload[1], pRxPacket->payload[2],
                              pRxPacket->payload[3], pRxPacket->payload[0]);
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = AdcIfGetDatarate(pAdcIf, pRxPacket->payload[0], &pExample->datarateReg[0],
                                  &pExample->datarateReg[1], &pExample->datarateReg[2]);
    }

    if (status != ADI_ADC_STATUS_SUCCESS)
    {
        SendStatus(pCommInfo, (uint32_t)status, ADI_UCOMM_ERROR_MASK);
    }
    else
    {
        SendResponse(pExample->pCommInfo, &pExample->datarateReg[0], sizeof(pExample->datarateReg));
    }
}

static void ProcessWriteReadHpfCoeffs(ADC_EXAMPLE *pExample)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    EXAMPLE_UCOMM_INFO *pCommInfo = pExample->pCommInfo;
    ADC_INTERFACE_INFO *pAdcIf = pExample->pAdcIf;
    ADI_UCOMM_PACKET *pRxPacket = &pCommInfo->rxPacket;
    uint8_t idx;
    uint8_t numHpfCoeffsNum = 3;
    uint8_t denHpfCoeffsNum = 2;
    uint8_t payloadCnt = 0;
    int8_t adcIdx;
    ADI_UCOMM_HANDLE hComm = pCommInfo->hComm;
    ADI_UCOMM_PACKET *pTxPacket = &pCommInfo->txPacket;
    memcpy(&adcIdx, &pRxPacket->payload[payloadCnt++], 1);
    for (idx = 0; idx < numHpfCoeffsNum; idx++)
    {
        memcpy(&pExample->numHpfCoeffs[idx], &pRxPacket->payload[payloadCnt], sizeof(float));
        payloadCnt += sizeof(float);
    }
    for (idx = 0; idx < denHpfCoeffsNum; idx++)
    {
        memcpy(&pExample->denHpfCoeffs[idx], &pRxPacket->payload[payloadCnt], sizeof(double));
        payloadCnt += sizeof(double);
    }

    status =
        AdcIfSetHpfCoeff(pAdcIf, &pExample->numHpfCoeffs[0], &pExample->denHpfCoeffs[0], adcIdx);
    memset(&pExample->numHpfCoeffs[0], 0, sizeof(pExample->numHpfCoeffs));
    memset(&pExample->denHpfCoeffs[0], 0, sizeof(pExample->denHpfCoeffs));
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = AdcIfGetHpfCoeff(pAdcIf, adcIdx, &pExample->numHpfCoeffs[0],
                                  &pExample->denHpfCoeffs[0]);
    }
    if (status != ADI_ADC_STATUS_SUCCESS)
    {
        SendStatus(pCommInfo, (uint32_t)status, ADI_UCOMM_ERROR_MASK);
    }
    else
    {
        pTxPacket->command = pRxPacket->command;
        payloadCnt = sizeof(float) * numHpfCoeffsNum;
        memcpy(&pTxPacket->payload[0], &pExample->numHpfCoeffs[0], payloadCnt);
        memcpy(&pTxPacket->payload[payloadCnt], &pExample->denHpfCoeffs[0],
               sizeof(double) * denHpfCoeffsNum);
        payloadCnt += sizeof(double) * denHpfCoeffsNum;
        pTxPacket->length = (uint8_t)payloadCnt;
        adi_ucomm_Transmit(hComm, pTxPacket);
    }
}

static void ProcessWriteReadLpfCoeffs(ADC_EXAMPLE *pExample)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    EXAMPLE_UCOMM_INFO *pCommInfo = pExample->pCommInfo;
    ADC_INTERFACE_INFO *pAdcIf = pExample->pAdcIf;
    ADI_UCOMM_PACKET *pRxPacket = &pCommInfo->rxPacket;
    int8_t adcIdx;
    uint8_t payloadCnt = 0;
    uint8_t idx;

    memcpy(&adcIdx, &pRxPacket->payload[payloadCnt++], 1);
    for (idx = 0; idx < ADEMA12x_NUM_LPF_COEFFS; idx++)
    {
        memcpy(&pExample->lpfCoeffs[idx], &pRxPacket->payload[payloadCnt], sizeof(float));
        payloadCnt += sizeof(float);
    }

    status = AdcIfSetLpfCoeff(pAdcIf, &pExample->lpfCoeffs[0], adcIdx);
    memset(&pExample->lpfCoeffs[0], 0, sizeof(pExample->lpfCoeffs));
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = AdcIfGetLpfCoeff(pAdcIf, adcIdx, &pExample->lpfCoeffs[0]);
    }

    if (status != ADI_ADC_STATUS_SUCCESS)
    {
        SendStatus(pCommInfo, (uint32_t)status, ADI_UCOMM_ERROR_MASK);
    }
    else
    {
        SendResponse(pCommInfo, (uint8_t *)&pExample->lpfCoeffs[0],
                     sizeof(float) * ADEMA12x_NUM_LPF_COEFFS);
    }
}

void ProcessWriteReadCompFilterCoeff(ADC_EXAMPLE *pExample)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    EXAMPLE_UCOMM_INFO *pCommInfo = pExample->pCommInfo;
    ADC_INTERFACE_INFO *pAdcIf = pExample->pAdcIf;
    ADI_UCOMM_PACKET *pRxPacket = &pCommInfo->rxPacket;
    uint8_t payloadCnt = 0;
    int8_t adcIdx;
    uint8_t numCh;
    uint8_t idx, compfiltCnt = 0, compfiltPtrCnt = 0;
    memset(&pExample->chIdx[0], 0, sizeof(pExample->chIdx));
    memset(&pExample->setCompFiltCoeffs[0], 0, sizeof(pExample->setCompFiltCoeffs));
    memset(&pExample->getCompFiltCoeffs[0], 0, sizeof(pExample->getCompFiltCoeffs));

    memcpy(&adcIdx, &pRxPacket->payload[payloadCnt++], 1);
    memcpy(&numCh, &pRxPacket->payload[payloadCnt++], 1);
    memcpy(&pExample->chIdx[0], &pRxPacket->payload[payloadCnt], numCh);
    payloadCnt += numCh;

    for (idx = 0; idx < numCh * ADEMA12x_NUM_COMP_COEFFS; idx++)
    {
        memcpy(&pExample->setCompFiltCoeffs[compfiltCnt], &pRxPacket->payload[payloadCnt],
               sizeof(float));
        payloadCnt += sizeof(float);
        compfiltCnt += 1;
    }
    for (idx = 0; idx < numCh; idx++)
    {
        pExample->pSetCompFiltCoeffs[idx] = &pExample->setCompFiltCoeffs[compfiltPtrCnt];
        pExample->pGetCompFiltCoeffs[idx] = &pExample->getCompFiltCoeffs[compfiltPtrCnt];
        compfiltPtrCnt += ADEMA12x_NUM_COMP_COEFFS;
    }

    status = AdcIfSetCompCoeff(pAdcIf, &pExample->pSetCompFiltCoeffs[0], &pExample->chIdx[0], numCh,
                               adcIdx);
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = AdcIfGetCompCoeff(pAdcIf, &pExample->chIdx[0], numCh, adcIdx,
                                   &pExample->pGetCompFiltCoeffs[0]);
        // Copy retrieved coefficients to the byte array
        if (status == ADI_ADC_STATUS_SUCCESS)
        {
            memcpy(&pExample->getCompFiltCoeffsBytes[0], &pExample->getCompFiltCoeffs[0],
                   sizeof(pExample->getCompFiltCoeffsBytes));
        }
    }

    if (status != ADI_ADC_STATUS_SUCCESS)
    {
        SendStatus(pCommInfo, (uint32_t)status, ADI_UCOMM_ERROR_MASK);
    }
    else
    {
        SendResponse(pExample->pCommInfo, &pExample->getCompFiltCoeffsBytes[0],
                     sizeof(float) * numCh * ADEMA12x_NUM_COMP_COEFFS);
    }
}

void ProcessWriteReadHpfCutOff(ADC_EXAMPLE *pExample)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    EXAMPLE_UCOMM_INFO *pCommInfo = pExample->pCommInfo;
    ADC_INTERFACE_INFO *pAdcIf = pExample->pAdcIf;
    ADI_UCOMM_PACKET *pRxPacket = &pCommInfo->rxPacket;
    int8_t adcIdx;
    uint32_t bwOption = 0;
    memcpy(&adcIdx, &pRxPacket->payload[0], 1);
    memcpy(&bwOption, &pRxPacket->payload[1], 4);
    status = AdcIfSetHpfCutoff(pAdcIf, bwOption, adcIdx);
    bwOption = 0;
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = AdcIfGetHpfCutoff(pAdcIf, adcIdx, &bwOption);
    }
    if (status != ADI_ADC_STATUS_SUCCESS)
    {
        SendStatus(pCommInfo, (uint32_t)status, ADI_UCOMM_ERROR_MASK);
    }
    else
    {
        SendResponse(pExample->pCommInfo, (uint8_t *)&bwOption, sizeof(bwOption));
    }
}

#if (APP_CFG_USE_TIMESTAMP == 1)
static void ProcessWriteReadTimestamp(ADC_EXAMPLE *pExample)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    EXAMPLE_UCOMM_INFO *pCommInfo = pExample->pCommInfo;
    ADC_INTERFACE_INFO *pAdcIf = pExample->pAdcIf;
    ADI_UCOMM_PACKET *pRxPacket = &pCommInfo->rxPacket;
    uint32_t numtimestamps = 0;
    uint32_t timestamp = 0;
    uint32_t payloadIdx = 0;
    uint8_t numTimestampToCollect = 0;
    uint8_t idx, jdx;
    // Initialize payload index to start reading from the beginning of the payload
    payloadIdx = 0;
    // Extract the number of timestamps from the payload
    memcpy(&numtimestamps, &pRxPacket->payload[payloadIdx], sizeof(uint32_t));
    payloadIdx += sizeof(uint32_t);
    // Reset the ADC frame buffer before starting data collection
    adi_adc_ResetFrameBuffer(pAdcIf->hAdc);
    // Loop through each block of timestamps
    for (idx = 0; idx < (numtimestamps / pAdcIf->adcCfg.numSamplesInBlock); idx++)
    {
        // Collect samples for each timestamp in the block
        for (jdx = 0; jdx < pAdcIf->adcCfg.numSamplesInBlock; jdx++)
        {

            // Extract a timestamp from the payload
            memcpy(&timestamp, &pRxPacket->payload[payloadIdx], sizeof(uint32_t));
            payloadIdx += sizeof(uint32_t);
            // Collect ADC samples for the given timestamp
            status |= adi_adc_CollectSamples(pAdcIf->hAdc, timestamp);
        }
        // Wait until the ADC block is ready
        while (pAdcIf->blockReady == false)
        {
        }
        // Read the collected ADC block into the sample and status output buffers
        status |=
            adi_adc_ReadBlock(pAdcIf->hAdc, &pAdcIf->adcSamples[0], &pAdcIf->adcStatusOutput[0]);
        if (status == ADI_ADC_STATUS_SUCCESS)
        {
            // If the read was successful, retrieve run data including timestamps
            status = adi_adc_GetRunData(pAdcIf->hAdc, &pAdcIf->runInfo);
            numTimestampToCollect = pAdcIf->adcCfg.numAdc * pAdcIf->adcCfg.numSamplesInBlock;
            // Calculate the number of timestamps to collect for this block
            memcpy(&pExample->timestampsbuffer[idx * numTimestampToCollect],
                   &pAdcIf->runInfo.timestamp[0], numTimestampToCollect);
        }
    }
    // Send the collected timestamps back as a response
    SendResponse(pExample->pCommInfo, (uint8_t *)&pExample->timestampsbuffer[0],
                 sizeof(pExample->timestampsbuffer[0]) * numtimestamps);
}
#endif

static void ProcessSetAdema127Shift(ADC_EXAMPLE *pExample)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    EXAMPLE_UCOMM_INFO *pCommInfo = pExample->pCommInfo;
    ADC_INTERFACE_INFO *pAdcIf = pExample->pAdcIf;
    ADI_UCOMM_PACKET *pRxPacket = &pCommInfo->rxPacket;
    uint8_t payloadCnt = 0;
    int8_t adcIdx;
    uint8_t numCh;
    memset(&pExample->chIdx[0], 0, sizeof(pExample->chIdx));
    memset(&pExample->alphasShifts[0], 0, sizeof(pExample->alphasShifts));

    memcpy(&adcIdx, &pRxPacket->payload[payloadCnt++], 1);
    memcpy(&numCh, &pRxPacket->payload[payloadCnt++], 1);
    memcpy(&pExample->chIdx[0], &pRxPacket->payload[payloadCnt], numCh);
    payloadCnt += numCh;
    memcpy(&pExample->alphasShifts[0], &pRxPacket->payload[payloadCnt], numCh);

    status = AdcIfSetShift(pAdcIf, &pExample->alphasShifts[0], &pExample->chIdx[0], numCh, adcIdx);
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        memset(&pExample->alphasShifts[0], 0, sizeof(pExample->alphasShifts));
        status =
            AdcIfGetShift(pAdcIf, &pExample->chIdx[0], numCh, adcIdx, &pExample->alphasShifts[0]);
    }

    if (status != ADI_ADC_STATUS_SUCCESS)
    {
        SendStatus(pCommInfo, (uint32_t)status, ADI_UCOMM_ERROR_MASK);
    }
    else
    {
        SendResponse(pExample->pCommInfo, &pExample->alphasShifts[0], sizeof(uint8_t) * numCh);
    }
}

/**
 * @}
 */
