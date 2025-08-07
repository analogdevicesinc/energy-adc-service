/******************************************************************************
 Copyright (c) 2024 - 2025  Analog Devices Inc.
******************************************************************************/

/**
 * @file        adc_service_interface.c
 * @brief       Interface file demonstrating use of ADC service APIs.
 * @{
 */

/*============= I N C L U D E S =============*/
#include "adc_service_interface.h"
#include "ADE911X_addr_def.h"
#include "ADE911X_addr_rdef.h"
#include "ADEMA127_addr_def.h"
#include "ADEMA127_addr_rdef.h"
#include "adc_service_dsp_interface.h"
#include "adi_adc.h"
#include "adi_evb.h"
#include "math.h"
#include <stdint.h>
#include <string.h>

/*============= D E F I N E S =============*/
/**
 * @brief ADC interface info variable.
 */
static ADC_INTERFACE_INFO *pAdcIf;

/**
 * @brief Populates the ADC configuration.
 *
 * This function populates the ADC configuration based on the provided information.
 *
 * @param pInfo Pointer to the ADC_INTERFACE_INFO structure.
 * @param numAdc Number of ADCs.
 * @param pAdcType Pointer to the ADI_ADC_TYPE array.
 * @return status of the operation.
 */
static ADI_ADC_STATUS PopulateAdcConfig(ADC_INTERFACE_INFO *pInfo, uint8_t numAdc,
                                        ADI_ADC_TYPE *pAdcType);

/**
 * @brief Writes the ADC DSP configuration.
 *
 * This function writes the initial ADC DSP configuration.
 *
 * @param pInfo Pointer to the ADC_INTERFACE_INFO structure.
 * @param numAdc Number of ADCs.
 * @param pAdcType Pointer to the ADI_ADC_TYPE array.
 * @return ADI_ADC_STATUS indicating the status of the operation.
 */
static ADI_ADC_STATUS WriteDatapathRegisters(ADC_INTERFACE_INFO *pInfo, ADI_ADC_CONFIG *pConfig);

/**
 * @brief Allocate memory for temp memory used by interface
 *
 * @param pInfo Pointer to the ADC_INTERFACE_INFO structure.
 * @param pTempMemory Pointer to the temp memory.
 * @param tempMemorySize Temp memory size.
 * @return ADI_ADC_STATUS indicating the status of the operation.
 */
ADI_ADC_STATUS AllocateIfTempMemory(ADC_INTERFACE_INFO *pInfo, uint32_t *pTempMemory,
                                    uint32_t tempMemorySize);

/*============= F U N C T I O N S =============*/

int32_t AdcIfCreateService(ADC_INTERFACE_INFO *pInfo)
{
    int32_t status;
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    void *pStateMemory = &pInfo->adcStateMemory[0];
    uint32_t stateMemSize = sizeof(pInfo->adcStateMemory);
    adcStatus = adi_adc_Create(&pInfo->hAdc, pStateMemory, stateMemSize);

    if (adcStatus == ADI_ADC_STATUS_SUCCESS)
    {
        status = AdcAdptPopulateConfig(&pInfo->adcCfg);
    }
    else
    {
        status = 1;
    }

    if (status == 0)
    {
        pAdcIf = pInfo;
#if (APP_CFG_USE_TIMESTAMP == 1)
        pInfo->runInfo.pTimestamp = pInfo->timestamp;
#endif
    }

    return status;
}

ADI_ADC_STATUS AdcIfInitService(ADC_INTERFACE_INFO *pInfo, uint8_t numAdc, ADI_ADC_TYPE *pAdcType)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    uint8_t idx = 0;

    EvbEnableDreadyIrq(0);
    pInfo->suspendState = 1;
    pInfo->isSpiRunning = 0;
    pAdcIf->overflowError = 0;
    pAdcIf->dreadyError = 0;

    status = PopulateAdcConfig(pInfo, numAdc, pAdcType);
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = adi_adc_Init(pInfo->hAdc, &pInfo->adcCfg);
        if (status == ADI_ADC_STATUS_SUCCESS)
        {
            /* Reset the ADC since it is not done inside the service */
            EvbResetAdcs();
            EvbDelayMs(ADI_ADC_STARTUP_TIME_MS);
        }
    }
    /* Enable CLKOUT in specified ADCs. */
    if (numAdc != 1)
    {
        for (idx = (numAdc - 1); idx > 0; idx--)
        {
            if (status == ADI_ADC_STATUS_SUCCESS)
            {
                status = adi_adc_EnableClockOut(pInfo->hAdc, idx);
                if (status == ADI_ADC_STATUS_SUCCESS)
                {
                    /* Wait for the ADC to start up. */
                    EvbDelayMs(ADI_ADC_STARTUP_TIME_MS);
                }
            }
        }
    }
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = AdcIfConfigureAdcs(pInfo);
    }

    EvbEnableDreadyIrq(1);
#if APP_CFG_ENABLE_DATAPATH == 1

    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = AllocateIfTempMemory(pInfo, pInfo->tempMemory, sizeof(pInfo->tempMemory));
    }
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = WriteDatapathRegisters(pInfo, &pInfo->adcCfg);
    }
#endif
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        pInfo->blockReady = false;
        /* Toggle LED to indicate that ADC init is a success */
        EvbLedOn(1);
    }

    return status;
}

ADI_ADC_STATUS AdcIfSetConfig(ADC_INTERFACE_INFO *pInfo)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;

    AdcIfStopCapture(pInfo);
    status = adi_adc_SetConfig(pInfo->hAdc, &pInfo->adcCfg);
    AdcIfStartCapture(pInfo);

    return status;
}

ADI_ADC_STATUS AdcIfConfigureAdcs(ADC_INTERFACE_INFO *pInfo)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;

    ADI_ADC_STATUS_OUTPUT *pAdcStatusOutput = &pInfo->adcStatusOutput[0];

    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = adi_adc_ConfigureAdcs(pInfo->hAdc, &pInfo->configRegisters[0]);
    }
    adi_adc_GetLastFrameStatus(pInfo->hAdc, pAdcStatusOutput);

    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = adi_adc_GetRunData(pInfo->hAdc, &pInfo->runInfo);
    }

    return status;
}

ADI_ADC_STATUS AdcIfStartCapture(ADC_INTERFACE_INFO *pInfo)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;

    status = adi_adc_ResetFrameBuffer(pInfo->hAdc);
    pAdcIf->currDreadyTime = EvbGetTime();
    pInfo->isSpiRunning = 0;
    pInfo->enableRun = 1;
    pInfo->dreadyCnt = 0;

    return status;
}

ADI_ADC_STATUS AdcIfStopCapture(ADC_INTERFACE_INFO *pInfo)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;

    pInfo->enableRun = 0;
    /* Wait till previous SPI Tx/Rx is complete */
    while (pInfo->isSpiRunning == 1)
    {
        ;
    }
    status = adi_adc_ResetFrameBuffer(pInfo->hAdc);

    return status;
}

ADI_ADC_STATUS AdcIfWriteRegister(ADC_INTERFACE_INFO *pInfo, uint16_t address, uint8_t value,
                                  int8_t adcIdx)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;

    if (pInfo->enableRun == 0)
    {
        /* Write value in the given ADC register address */
        status = adi_adc_WriteRegister(pInfo->hAdc, address, value, adcIdx);
    }
    else
    {
        status = adi_adc_AssembleWriteRegister(pInfo->hAdc, address, value, adcIdx);
    }

    return status;
}

ADI_ADC_STATUS AdcIfReadRegister(ADC_INTERFACE_INFO *pInfo, uint16_t address, int8_t adcIdx,
                                 uint8_t *pBuffer, uint32_t *pNumBytes)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    ADI_ADC_STATUS_OUTPUT *pAdcStatusOutput = &pInfo->adcStatusOutput[0];

    /* Read register value present in the given ADC address */
    if (pInfo->enableRun == 0)
    {
        status = adi_adc_ReadRegister(pInfo->hAdc, address, adcIdx, pBuffer, pNumBytes);
        if (status == ADI_ADC_STATUS_SUCCESS)
        {
            status = adi_adc_GetLastFrameStatus(pInfo->hAdc, pAdcStatusOutput);
        }
    }
    else
    {
        /* Issue an ADC read command to collect the value in the specified ADC
         * address*/
        status = adi_adc_AssembleReadRegister(pInfo->hAdc, address, adcIdx);
    }

    return status;
}

ADI_ADC_STATUS AdcIfPerformSync(ADC_INTERFACE_INFO *pInfo)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    if (pInfo->enableRun == 0)
    {
        status = adi_adc_Align(pInfo->hAdc);
    }
    else
    {
        status = adi_adc_StartAlign(pInfo->hAdc);
    }

    return status;
}

ADI_ADC_STATUS PopulateAdcConfig(ADC_INTERFACE_INFO *pInfo, uint8_t numAdc, ADI_ADC_TYPE *pAdcType)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    uint8_t i;
    ADI_ADC_CONFIG *pConfig = &pInfo->adcCfg;

    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        if (pAdcType != NULL)
        {
            pConfig->numAdc = numAdc;
            pConfig->pAdcType = pAdcType;
            pConfig->pIntegerSampleDelay = pInfo->integerSampleDelay;
            pConfig->maxSampleDelay = APP_CFG_MAX_SAMPLE_DELAY;
        }
        else
        {
            pConfig->numAdc = numAdc;
            for (i = 0; i < numAdc; i++)
            {
                pInfo->adcType[i] = ADI_ADC_TYPE_ADEMA127;
            }
            pConfig->pAdcType = pInfo->adcType;
        }
    }
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = adi_adcutil_PopulateStreamMode(pInfo->adcStreamMode, numAdc, pConfig->pAdcType,
                                                &pInfo->configRegisters[0]);
    }
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = adi_adcutil_PopulateSamplingRate(pInfo->clkIn, pInfo->adcSamplingRate,
                                                  pInfo->decimateBy2, numAdc, &pConfig->pAdcType[0],
                                                  &pInfo->configRegisters[0]);
    }

#ifdef ENABLE_SIMULATION
    /* FIXME: This need to be moved out to InitTestCmd  fuction*/
    EvbConnectAdc(numAdc, &pConfig->pAdcType[0]);
#endif
#if APP_CFG_ENABLE_ADCS_CALLBACK == 1
    pConfig->pfCallback = AdcIfAdcCallback;
#endif

    pConfig->hUser = pInfo;
    pConfig->numSamplesInBlock = APP_CFG_DEFAULT_SAMPLE_BLOCK_SIZE;
    pConfig->ignoreRxBufferOverflow = APP_CFG_IGNORE_RX_BUFFER_OVERFLOW;

    return status;
}

void ReturnAdcErrorFlags(volatile uint8_t *pDreadyErr, volatile uint8_t *pOverflowErr)
{
    *pDreadyErr = pAdcIf->dreadyError;
    *pOverflowErr = pAdcIf->overflowError;
}

ADC_INTERFACE_INFO *AdcIfGetInstance(void)
{
    return pAdcIf;
}

ADI_ADC_STATUS AdcIfGetLastFrameStatus(ADC_INTERFACE_INFO *pInfo)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    ADI_ADC_STATUS_OUTPUT *pAdcStatusOutput = &pInfo->adcStatusOutput[0];

    status = adi_adc_GetLastFrameStatus(pInfo->hAdc, pAdcStatusOutput);

    return status;
}

int32_t AdcIfReadVersion(ADC_INTERFACE_INFO *pInfo, int8_t adcIdx, uint8_t *pSiliconRevision,
                         uint8_t *pProductId)
{
    int32_t status = 0;
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    uint8_t buff[2 * APP_CFG_MAX_NUM_ADC];
    uint32_t numBytes, idx, piCnt = 0, srCnt = 0;
    /* Address is same for both ADE91xx and ADEMA12x */
    uint16_t address = ADDR_ADEMA127_MMR_SILICON_REVISION;

    adcStatus = AdcIfReadRegister(pInfo, address, adcIdx, &buff[0], &numBytes);

    // Ensure the number of bytes is even (PI and SR pairs)
    if (numBytes % 2 != 0)
    {
        // Handle error for invalid buffer format
        status = 1;
    }
    if (status == 0)
    {
        if (adcIdx == -1)
        {
            for (idx = 0; idx < numBytes; idx += 2)
            {
                pProductId[piCnt++] = buff[idx];           // PI value
                pSiliconRevision[srCnt++] = buff[idx + 1]; // SR value
            }
        }
        else
        {
            pProductId[0] = buff[0];       // First PI value
            pSiliconRevision[0] = buff[1]; // First SR value
        }
    }

    return adcStatus;
}

ADI_ADC_STATUS WriteDatapathRegisters(ADC_INTERFACE_INFO *pInfo, ADI_ADC_CONFIG *pConfig)
{

    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    uint8_t idx, chIdx;
    int8_t numChannel = 0;
    uint32_t numBytesRead = 0;
    uint32_t regVal = 0;
    uint8_t numAdc = pConfig->numAdc;
    ADI_ADC_TYPE *pAdcType = &pConfig->pAdcType[0];

    status = AdcIfDatapathSetVal(pInfo, pConfig);
    for (idx = 0; idx < numAdc; idx++)
    {
        if (status == ADI_ADC_STATUS_SUCCESS)
        {
            if (pAdcType[idx] != ADI_ADC_TYPE_ADE91XX)
            {
                if (pAdcType[idx] == ADI_ADC_TYPE_ADEMA124)
                {
                    numChannel = 4;
                }
                else if (pAdcType[idx] == ADI_ADC_TYPE_ADEMA127)
                {
                    numChannel = 7;
                }

                for (chIdx = 0; chIdx < numChannel; chIdx++)
                {
                    pInfo->channelIdx[chIdx] = chIdx;
                }
                if (status == ADI_ADC_STATUS_SUCCESS)
                {
                    status = AdcIfSetDspConfig(pInfo, &pInfo->adcRegParams[idx].adcDatapathParams,
                                               &pInfo->adcRegParams[idx].adcChannelParams,
                                               pInfo->channelIdx, numChannel, idx);
                }
            }
            else
            {
                status = AdcIfReadRegister(pInfo, ADDR_ADE911X_MAP0_CONFIG_FILT, idx,
                                           &pInfo->adcRegBuff[0], &numBytesRead);
                if (status == ADI_ADC_STATUS_SUCCESS)
                {
                    // Clear inverted channels and save invReg there
                    regVal = (pInfo->adcRegBuff[1] & 0x0F) |
                             (pInfo->adcRegParams[idx].invReg
                              << BITP_ADE911X_MAP0_CONFIG_FILT_I_ADC_INVERT);
                    // Save ADE91XX inverted channels mask in invReg
                    status = AdcIfWriteRegister(pInfo, ADDR_ADE911X_MAP0_CONFIG_FILT, regVal, idx);
                }
            }
        }
    }
    return status;
}

ADI_ADC_STATUS AdcIfPopulateDspRegisterStruct(ADC_INTERFACE_INFO *pInfo, uint8_t numAdc,
                                              ADI_ADC_TYPE *pAdcType)
{
    int32_t adcIdx;
    int32_t numChannel = 0;
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    ADC_INTERFACE_INFO *pAdcIf = pInfo;
    int32_t chIdx;

    for (adcIdx = 0; adcIdx < numAdc; adcIdx++)
    {

        if (pAdcType[adcIdx] != ADI_ADC_TYPE_ADE91XX)
        {
            if (pAdcType[adcIdx] == ADI_ADC_TYPE_ADEMA124)
            {
                numChannel = 4;
            }
            else if (pAdcType[adcIdx] == ADI_ADC_TYPE_ADEMA127)
            {
                numChannel = 7;
            }
            for (chIdx = 0; chIdx < numChannel; chIdx++)
            {
                pInfo->channelIdx[chIdx] = chIdx;
            }
            status = AdcIfSetDspRam(pAdcIf, &pInfo->adcRegParams[adcIdx].adcDspBackup,
                                    pInfo->channelIdx, numChannel, adcIdx);
        }
    }
    return status;
}

ADI_ADC_STATUS AdcIfGetDspRegisterStruct(ADC_INTERFACE_INFO *pInfo, uint8_t numAdc,
                                         ADI_ADC_TYPE *pAdcType)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    uint8_t idx, chIdx;
    int8_t numChannel = 0;
    for (idx = 0; idx < numAdc; idx++)
    {
        if (status == ADI_ADC_STATUS_SUCCESS)
        {
            if (pAdcType[idx] != ADI_ADC_TYPE_ADE91XX)
            {
                if (pAdcType[idx] == ADI_ADC_TYPE_ADEMA124)
                {
                    numChannel = 4;
                }
                else if (pAdcType[idx] == ADI_ADC_TYPE_ADEMA127)
                {
                    numChannel = 7;
                }

                for (chIdx = 0; chIdx < numChannel; chIdx++)
                {
                    pInfo->channelIdx[chIdx] = chIdx;
                }

                status = AdcIfGetDspRam(pInfo, pInfo->channelIdx, numChannel, idx,
                                        &pInfo->adcRegParams[idx].adcDspBackup);
            }
        }
    }

    return status;
}

ADI_ADC_STATUS AdcIfSetIntegerSampleDelay(ADC_INTERFACE_INFO *pInfo, uint8_t *pValue,
                                          uint8_t *pChanIdx, int8_t numChan, int8_t adcIdx)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    int8_t i;
    uint8_t globalCh;
    int8_t adcNum;

    // Update the integer sample delay configuration in the ADC Config instance in ADC Interface
    // structure
    for (i = 0; i < numChan; i++)
    {
        if (adcIdx == -1)
        {
            for (adcNum = 0; adcNum < pInfo->adcCfg.numAdc; adcNum++)
            {
                globalCh = adcNum * APP_CFG_MAX_NUM_CHANNELS_PER_ADC + pChanIdx[i];
                pInfo->adcCfg.pIntegerSampleDelay[globalCh] = pValue[i];
            }
        }
        else
        {
            globalCh = adcIdx * APP_CFG_MAX_NUM_CHANNELS_PER_ADC + pChanIdx[i];
            pInfo->adcCfg.pIntegerSampleDelay[globalCh] = pValue[i];
        }
    }
    // Set the integer sample delay configuration inside the ADC service
    status = adi_adc_SetIntegerSampleDelay(pInfo->hAdc, pValue, pChanIdx, numChan, adcIdx);
    return status;
}

ADI_ADC_STATUS AdcIfGetIntegerSampleDelay(ADC_INTERFACE_INFO *pInfo, uint8_t *pChanIdx,
                                          int8_t numChan, int8_t adcIdx, uint8_t *pValue)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    int8_t i;
    uint8_t globalCh;
    int8_t adcNum;
    status = adi_adc_GetConfig(pInfo->hAdc, &pInfo->adcCfg);

    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        for (i = 0; i < numChan; i++)
        {
            if (adcIdx == -1)
            {
                for (adcNum = 0; adcNum < pInfo->adcCfg.numAdc; adcNum++)
                {
                    globalCh = adcNum * APP_CFG_MAX_NUM_CHANNELS_PER_ADC + pChanIdx[i];
                    pValue[i] = pInfo->adcCfg.pIntegerSampleDelay[globalCh];
                }
            }
            else
            {
                globalCh = adcIdx * APP_CFG_MAX_NUM_CHANNELS_PER_ADC + pChanIdx[i];
                pValue[i] = pInfo->adcCfg.pIntegerSampleDelay[globalCh];
            }
        }
    }
    return status;
}

ADI_ADC_STATUS AllocateIfTempMemory(ADC_INTERFACE_INFO *pInfo, uint32_t *pTempMemory,
                                    uint32_t tempMemorySize)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    uint32_t offset = 0;

    for (int i = 0; i < APP_CFG_MAX_NUM_ADC; i++)
    {
        pInfo->adcRegParams[i].adcDatapathParams.pDataPathConfig =
            (ADI_ADC_CHAN_DATAPATH_CONFIG *)&pTempMemory[offset];
        offset += (2 + APP_CFG_MAX_NUM_CHANNELS);

        pInfo->adcRegParams[i].adcDatapathParams.pPhaseOffset = (float *)&pTempMemory[offset];
        offset += APP_CFG_MAX_NUM_CHANNELS;
    }
    for (int i = 0; i < APP_CFG_MAX_NUM_ADC; i++)
    {
        pInfo->adcRegParams[i].adcChannelParams.pXtAggressor =
            (ADI_ADC_CHAN_XT_AGGRESSOR *)&pTempMemory[offset];
        offset += APP_CFG_MAX_NUM_CHANNELS;

        pInfo->adcRegParams[i].adcChannelParams.pOffset = (int32_t *)&pTempMemory[offset];
        offset += APP_CFG_MAX_NUM_CHANNELS;

        pInfo->adcRegParams[i].adcChannelParams.pXtGain = (float *)&pTempMemory[offset];
        offset += APP_CFG_MAX_NUM_CHANNELS;

        pInfo->adcRegParams[i].adcChannelParams.pGain = (float *)&pTempMemory[offset];
        offset += APP_CFG_MAX_NUM_CHANNELS;

        pInfo->adcRegParams[i].adcChannelParams.pShift = (uint8_t *)&pTempMemory[offset];
        offset += APP_CFG_MAX_NUM_CHANNELS;
    }
    for (int i = 0; i < APP_CFG_MAX_NUM_ADC; i++)
    {
        pInfo->adcRegParams[i].adcDspBackup.pXtAggressor =
            (ADI_ADC_CHAN_XT_AGGRESSOR *)&pTempMemory[offset];
        offset += APP_CFG_MAX_NUM_CHANNELS;

        pInfo->adcRegParams[i].adcDspBackup.pOffset = (int32_t *)&pTempMemory[offset];
        offset += APP_CFG_MAX_NUM_CHANNELS;

        pInfo->adcRegParams[i].adcDspBackup.pXtGain = (float *)&pTempMemory[offset];
        offset += APP_CFG_MAX_NUM_CHANNELS;

        pInfo->adcRegParams[i].adcDspBackup.pGain = (float *)&pTempMemory[offset];
        offset += APP_CFG_MAX_NUM_CHANNELS;

        pInfo->adcRegParams[i].adcDspBackup.pShift = (uint8_t *)&pTempMemory[offset];
        offset += APP_CFG_MAX_NUM_CHANNELS;
    }

    if (offset > (tempMemorySize / 4))
    {
        status = ADI_ADC_STATUS_INSUFFICIENT_STATE_MEMORY;
    }

    return status;
}

/**
 * @}
 */
