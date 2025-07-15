/******************************************************************************
 Copyright (c) 2024 - 2025  Analog Devices Inc.
******************************************************************************/

/**
 * @file        adc_example_no_os_config.c
 * @brief       Interface file demonstrating ADC service for baremetal.
 * @{
 */

/*============= I N C L U D E S =============*/
#include "adc_service_interface.h"
#include "adi_evb.h"
#include "math.h"
#include <stdint.h>
#include <string.h>

/*============= F U N C T I O N S =============*/

static void UpdateDreadyErrorFlag(ADC_INTERFACE_INFO *pInfo);
/**
 * @brief Updates the Overflow error flag.
 *
 * This function updates the Overflow error flag based on the provided ADC interface information.
 *
 * @param pInfo Pointer to the ADC_INTERFACE_INFO structure.
 * @param status Status of adi_adc_CollectFrames().
 */
static void UpdateOverflowErrorFlag(ADC_INTERFACE_INFO *pInfo, ADI_ADC_STATUS status);

void AdcIfSpiRxCallback(void)
{
    ADC_INTERFACE_INFO *pAdcIf = AdcIfGetInstance();
    pAdcIf->isSpiRunning = 0;
    adi_adc_ValidateSamples(pAdcIf->hAdc);
}

void AdcIfDreadyCallback(uint32_t port, uint32_t pinFlag)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    ADC_INTERFACE_INFO *pAdcIf = AdcIfGetInstance();
    if ((port == (uint32_t)BOARD_CFG_ADC_DREADY_PORT) && (pinFlag == BOARD_CFG_ADC_DREADY_PIN))
    {
        pAdcIf->dreadyFlag = 1;
        pAdcIf->prevDreadyTime = pAdcIf->currDreadyTime;
        pAdcIf->currDreadyTime = EvbGetTime();
        if (pAdcIf->enableRun == 1)
        {
            pAdcIf->dreadyCnt++;
            UpdateDreadyErrorFlag(pAdcIf);
            pAdcIf->isSpiRunning = 1;
            status = adi_adc_CollectSamples(pAdcIf->hAdc, pAdcIf->currDreadyTime);
            UpdateOverflowErrorFlag(pAdcIf, status);
        }
    }
    if (pAdcIf->isTdmCycleRunning == 1)
    {
        pAdcIf->tamperCnt++;
    }
}

ADI_ADC_STATUS AdcIfAdcCallback(void *hUser, uint32_t adcEvent)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    ADC_INTERFACE_INFO *pAdcIf = AdcIfGetInstance();
    if (hUser != NULL)
    {
        if ((adcEvent & ADI_ADC_EVENT_BITM_RESPONSE_READY) != 0)
        {
            pAdcIf->suspendState = 0;
        }
        if ((adcEvent & ADI_ADC_EVENT_BITM_BLOCK_READY) != 0)
        {
            pAdcIf->blockReady = true;
        }
    }
    else
    {
        adcStatus = ADI_ADC_STATUS_NULL_PTR;
    }
    return adcStatus;
}

ADI_ADC_STATUS AdcIfWaitAdcResponse(ADC_INTERFACE_INFO *pInfo)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    uint32_t waitCount = 0;
    if (pInfo != NULL)
    {
        while ((pInfo->suspendState == 1) && (waitCount < APP_CFG_TIMEOUT_COUNT))
        {
            waitCount++;
        }
        pInfo->suspendState = 1;
        if (waitCount == APP_CFG_TIMEOUT_COUNT)
        {
            adcStatus = ADI_ADC_STATUS_TRANSCEIVE_FAILED;
        }
    }
    return adcStatus;
}

void UpdateDreadyErrorFlag(ADC_INTERFACE_INFO *pInfo)
{
    /* dreadyTimes are timer counter values and need to be divided by timer frequency to
     * convert to seconds. The timer frequency is 25MHz.
     */
    float timeFor1Sample = (float)(pInfo->currDreadyTime - pInfo->prevDreadyTime) / 25000000;
    float timeFor1sSamples = (float)pInfo->adcSamplingRate * timeFor1Sample;
    float error = (float)fabsf(timeFor1sSamples - 1.0f);

    if ((error > (float)0.25) && pInfo->dreadyCnt > 1)
    {
        pInfo->dreadyError = 1;
        EvbLedOn(2);
    }
}

ADI_ADC_STATUS AdcIfCollectSamples(ADC_INTERFACE_INFO *pInfo, uint32_t channelMask,
                                   uint32_t numSamplesRequired, int32_t *pSamples)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    uint8_t numSamplesInBlock = pInfo->adcCfg.numSamplesInBlock;
    uint32_t numSamplesToSend = 0;
    ADI_ADC_STATUS_OUTPUT *pAdcStatusOutput = &pInfo->adcStatusOutput[0];
    int32_t *pAdcSamples = &pInfo->adcSamples[0];
    int32_t waitCount = 0;

    do
    {
        pAdcSamples = &pInfo->adcSamples[0];
        /* Read one block of data from the ADC buffers if it is available */
        status = adi_adc_ReadBlock(pInfo->hAdc, pAdcSamples, pAdcStatusOutput);

        /* The output buffer will contain interleaved samples of all channels from all ADCs*/
        if (status == ADI_ADC_STATUS_SUCCESS)
        {
            waitCount = 0;
            numSamplesToSend += adi_adcutil_ExtractChannel(
                pAdcSamples, numSamplesInBlock, pInfo->runInfo.totalChannels, channelMask,
                &pSamples[numSamplesToSend]);
        }
        else if (status == ADI_ADC_STATUS_NO_DATA)
        {
            waitCount++;
            if (waitCount > APP_CFG_TIMEOUT_COUNT)
            {
                /* Update the stats to know why we are timing out*/
                adi_adc_GetRunData(pInfo->hAdc, &pInfo->runInfo);
                break;
            }
        }
        else
        {
            break;
        }
    } while (numSamplesToSend < numSamplesRequired);

    return status;
}

void UpdateOverflowErrorFlag(ADC_INTERFACE_INFO *pInfo, ADI_ADC_STATUS status)
{
    if (status == ADI_ADC_STATUS_BUFFER_OVERFLOW)
    {
        pInfo->overflowError = 1;
    }
}

/**
 * @}
 */
