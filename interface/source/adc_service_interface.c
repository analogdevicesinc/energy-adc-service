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
 * @return status of the operation.
 */
static ADI_ADC_STATUS PopulateAdcConfig(ADC_INTERFACE_INFO *pInfo,
                                        ADC_BOARD_CONFIG *pAdcBoardConfig);

/**
 * @brief Writes the ADC DSP configuration.
 *
 * This function writes the initial ADC DSP configuration.
 *
 * @param pInfo Pointer to the ADC_INTERFACE_INFO structure.
 * @param pAdcBoardConfig pointer to board config.
 * @param pConfig Pointer to the adc config.
 * @return ADI_ADC_STATUS indicating the status of the operation.
 */
static ADI_ADC_STATUS WriteDatapathRegisters(ADC_INTERFACE_INFO *pInfo,
                                             ADC_BOARD_CONFIG *pAdcBoardConfig,
                                             ADI_ADC_CONFIG *pConfig);

/**
 * @brief Allocate memory for temp memory used by interface
 *
 * @param pInfo Pointer to the ADC_INTERFACE_INFO structure.
 */
static void AllocateDspMem(ADC_INTERFACE_INFO *pInfo);

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
        AllocateDspMem(pInfo);
    }
#if APP_CFG_ENABLE_DATAPATH == 1
    if (status == 0)
    {
        adcStatus = AdcIfResetDatapathParams(&pInfo->adcRegParams[0]);
        if (adcStatus != ADI_ADC_STATUS_SUCCESS)
        {
            status = 1;
        }
    }
#endif

#if (APP_CFG_USE_TIMESTAMP == 1)
    if (status == 0)
    {
        pInfo->runInfo.pTimestamp = pInfo->timestamp;
    }
#endif
    if (status == 0)
    {
        pAdcIf = pInfo;
    }

    return status;
}

ADI_ADC_STATUS AdcIfInitService(ADC_INTERFACE_INFO *pInfo, ADC_BOARD_CONFIG *pAdcBoardConfig)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    uint8_t idx = 0;
    uint8_t numAdc = pAdcBoardConfig->numAdc;

    EvbEnableDreadyIrq(0);
    pInfo->suspendState = 1;
    pInfo->isSpiRunning = 0;
    pAdcIf->overflowError = 0;
    pAdcIf->dreadyError = 0;

    status = PopulateAdcConfig(pInfo, pAdcBoardConfig);
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
        status = WriteDatapathRegisters(pInfo, pAdcBoardConfig, &pInfo->adcCfg);
    }
#endif
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        pInfo->blockReady = false;
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
        if (status == ADI_ADC_STATUS_SUCCESS)
        {
            status = AdcIfWaitAdcResponse(pInfo);
        }
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
    }
    else
    {
        /* Issue an ADC read command to collect the value in the specified ADC
         * address*/
        status = adi_adc_AssembleReadRegister(pInfo->hAdc, address, adcIdx);
        if (status == ADI_ADC_STATUS_SUCCESS)
        {
            status = AdcIfWaitAdcResponse(pInfo);
            if (status == ADI_ADC_STATUS_SUCCESS)
            {
                status = adi_adc_GetLastRegister(pInfo->hAdc, adcIdx, pBuffer, pNumBytes);
            }
        }
    }

    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = adi_adc_GetLastFrameStatus(pInfo->hAdc, pAdcStatusOutput);
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

ADI_ADC_STATUS PopulateAdcConfig(ADC_INTERFACE_INFO *pInfo, ADC_BOARD_CONFIG *pAdcBoardConfig)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    ADI_ADC_CONFIG *pConfig = &pInfo->adcCfg;
    uint8_t numAdc = pAdcBoardConfig->numAdc;
    ADI_ADC_TYPE *pAdcType = &pAdcBoardConfig->adcType[0];

    if (pAdcType == NULL)
    {
        status = ADI_ADC_STATUS_INVALID_ADC_TYPE;
    }

    if ((numAdc == 0) || (numAdc > APP_CFG_MAX_NUM_ADC))
    {
        status = ADI_ADC_STATUS_INVALID_NUM_ADC;
    }

    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        pConfig->numAdc = numAdc;
        pConfig->pAdcType = pAdcType;
        pConfig->pIntegerSampleDelay = pInfo->integerSampleDelay;
        pConfig->maxSampleDelay = APP_CFG_MAX_SAMPLE_DELAY;
    }

    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = adi_adcutil_PopulateStreamMode(pAdcBoardConfig->adcStreamMode, numAdc,
                                                pConfig->pAdcType, &pInfo->configRegisters[0]);
    }
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = adi_adcutil_PopulateSamplingRate(
            pAdcBoardConfig->clkIn, pAdcBoardConfig->adcSamplingRate, pAdcBoardConfig->decimateBy2,
            numAdc, &pConfig->pAdcType[0], &pInfo->configRegisters[0]);
    }

#ifdef ENABLE_SIMULATION
    /* FIXME: This need to be moved out to InitTestCmd  fuction*/
    EvbConnectAdc(numAdc, &pConfig->pAdcType[0]);
#endif
#if APP_CFG_ENABLE_ADCS_CALLBACK == 1
    pConfig->pfCallback = pInfo->pfCallback;
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

ADI_ADC_STATUS WriteDatapathRegisters(ADC_INTERFACE_INFO *pInfo, ADC_BOARD_CONFIG *pAdcBoardConfig,
                                      ADI_ADC_CONFIG *pConfig)
{

    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    uint8_t idx, chIdx;
    int8_t numChannel = 0;
    uint32_t numBytesRead = 0;
    uint32_t regVal = 0;
    uint8_t numAdc = pConfig->numAdc;
    ADI_ADC_TYPE *pAdcType = &pConfig->pAdcType[0];

    status = AdcIfDatapathSetVal(pInfo, pAdcBoardConfig, numAdc);
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
    int32_t chIdx;
    uint32_t numBytesRead = 0;
    uint32_t regVal = 0;

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
            status = AdcIfSetDspConfig(pInfo, &pInfo->adcRegParams[adcIdx].adcDatapathParams,
                                       &pInfo->adcRegParams[adcIdx].adcChannelParams,
                                       pInfo->channelIdx, numChannel, adcIdx);
        }
        else
        {
            status = AdcIfReadRegister(pInfo, ADDR_ADE911X_MAP0_CONFIG_FILT, adcIdx,
                                       &pInfo->adcRegBuff[0], &numBytesRead);
            if (status == ADI_ADC_STATUS_SUCCESS)
            {
                // Clear inverted channels and save invReg there
                regVal =
                    (pInfo->adcRegBuff[1] & 0x0F) | (pInfo->adcRegParams[adcIdx].invReg
                                                     << BITP_ADE911X_MAP0_CONFIG_FILT_I_ADC_INVERT);
                // Save ADE91XX inverted channels mask in invReg
                status = AdcIfWriteRegister(pInfo, ADDR_ADE911X_MAP0_CONFIG_FILT, regVal, adcIdx);
            }
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
    uint32_t numBytesRead = 0;
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

                status = AdcIfGetDspConfig(pInfo, pInfo->channelIdx, numChannel, idx,
                                           &pInfo->adcRegParams[idx].adcDatapathParams,
                                           &pInfo->adcRegParams[idx].adcChannelParams);
            }
            else
            {
                status = AdcIfReadRegister(pInfo, ADDR_ADE911X_MAP0_CONFIG_FILT, idx,
                                           &pInfo->adcRegBuff[0], &numBytesRead);
                // Save ADE91XX inverted channels mask in invReg
                pInfo->adcRegParams[idx].invReg =
                    (pInfo->adcRegBuff[1] >> BITP_ADE911X_MAP0_CONFIG_FILT_I_ADC_INVERT);
            }
        }
    }

    return status;
}

ADI_ADC_STATUS AdcIfPopulateDspBackupRegisterStruct(ADC_INTERFACE_INFO *pInfo, uint8_t numAdc,
                                                    ADI_ADC_TYPE *pAdcType)
{
    int32_t adcIdx;
    int32_t numChannel = 0;
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    int32_t chIdx;
    uint32_t numBytesRead = 0;
    uint32_t regVal = 0;

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
#if APP_CFG_ENABLE_DSP_BACKUP == 1
            status = AdcIfSetDspRam(pInfo, &pInfo->adcRegParams[adcIdx].adcDspBackup,
                                    pInfo->channelIdx, numChannel, adcIdx);
#endif
        }
        else
        {
            status = AdcIfReadRegister(pInfo, ADDR_ADE911X_MAP0_CONFIG_FILT, adcIdx,
                                       &pInfo->adcRegBuff[0], &numBytesRead);
            if (status == ADI_ADC_STATUS_SUCCESS)
            {
                // Clear inverted channels and save invReg there
                regVal =
                    (pInfo->adcRegBuff[1] & 0x0F) | (pInfo->adcRegParams[adcIdx].invReg
                                                     << BITP_ADE911X_MAP0_CONFIG_FILT_I_ADC_INVERT);
                // Save ADE91XX inverted channels mask in invReg
                status = AdcIfWriteRegister(pInfo, ADDR_ADE911X_MAP0_CONFIG_FILT, regVal, adcIdx);
            }
        }
    }
    return status;
}

ADI_ADC_STATUS AdcIfGetDspBackupRegisterStruct(ADC_INTERFACE_INFO *pInfo, uint8_t numAdc,
                                               ADI_ADC_TYPE *pAdcType)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    uint8_t idx, chIdx;
    int8_t numChannel = 0;
    uint32_t numBytesRead = 0;

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

#if APP_CFG_ENABLE_DSP_BACKUP == 1
                status = AdcIfGetDspRam(pInfo, pInfo->channelIdx, numChannel, idx,
                                        &pInfo->adcRegParams[idx].adcDspBackup);
#endif
            }
            else
            {
                status = AdcIfReadRegister(pInfo, ADDR_ADE911X_MAP0_CONFIG_FILT, idx,
                                           &pInfo->adcRegBuff[0], &numBytesRead);
                // Save ADE91XX inverted channels mask in invReg
                pInfo->adcRegParams[idx].invReg =
                    (pInfo->adcRegBuff[1] >> BITP_ADE911X_MAP0_CONFIG_FILT_I_ADC_INVERT);
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

void AllocateDspMem(ADC_INTERFACE_INFO *pInfo)
{
    for (int i = 0; i < APP_CFG_MAX_NUM_ADC; i++)
    {
        pInfo->adcRegParams[i].adcDatapathParams.pDataPathConfig =
            pInfo->adcRegParams[i].datapathParams.dataPathConfig;
        pInfo->adcRegParams[i].adcDatapathParams.pPhaseOffset =
            pInfo->adcRegParams[i].datapathParams.phaseOffset;

        pInfo->adcRegParams[i].adcChannelParams.pXtAggressor =
            pInfo->adcRegParams[i].dspParams.xtAggressor;

        pInfo->adcRegParams[i].adcChannelParams.pOffset = pInfo->adcRegParams[i].dspParams.offset;

        pInfo->adcRegParams[i].adcChannelParams.pXtGain = pInfo->adcRegParams[i].dspParams.xtGain;

        pInfo->adcRegParams[i].adcChannelParams.pGain = pInfo->adcRegParams[i].dspParams.gain;

        pInfo->adcRegParams[i].adcChannelParams.pShift = pInfo->adcRegParams[i].dspParams.shift;
    }
#if APP_CFG_ENABLE_DSP_BACKUP == 1
    for (int i = 0; i < APP_CFG_MAX_NUM_ADC; i++)
    {
        pInfo->adcRegParams[i].adcDspBackup.pXtAggressor =
            pInfo->adcRegParams[i].dspBackupParams.xtAggressor;

        pInfo->adcRegParams[i].adcDspBackup.pOffset = pInfo->adcRegParams[i].dspBackupParams.offset;

        pInfo->adcRegParams[i].adcDspBackup.pXtGain = pInfo->adcRegParams[i].dspBackupParams.xtGain;

        pInfo->adcRegParams[i].adcDspBackup.pGain = pInfo->adcRegParams[i].dspBackupParams.gain;

        pInfo->adcRegParams[i].adcDspBackup.pShift = pInfo->adcRegParams[i].dspBackupParams.shift;
    }
#endif
}

/**
 * @}
 */
