/******************************************************************************
 Copyright (c) 2024 - 2025  Analog Devices Inc.
******************************************************************************/

/**
 * @file        adc_example_freertos_config.c
 * @brief       Interface file demonstrating ADC service callbacks for FreeRTOS.
 * @{
 */

/*============= I N C L U D E S =============*/

#include "adc_example_freertos_config.h"
#include "FreeRTOS.h"
#include "adc_service_interface.h"
#include "adi_adc.h"
#include "adi_evb.h"
#include "semphr.h"
#include <stdio.h>
#include <string.h>

/*============= F U N C T I O N S =============*/

/**
 * @brief Function to collect frames from ADC.
 *
 * This function runs as a FreeRTOS thread and is responsible for collecting
 * samples from the ADC and validates the frames. It continuously reads data from the ADC and
 * processes it as needed.
 *
 * @param pArg Pointer to arguments (not used in this implementation).
 */
static void CollectSamplesThread(void *pArg);

/**
 * @brief Function to update the error flag based on the dready time.
 * @param pInfo - Pointer to ADC interface info.
 */
static void UpdateDreadyErrorFlag(ADC_INTERFACE_INFO *pInfo);

/*=============  D A T A   =============*/

/** Semaphore for collect frames */
static SemaphoreHandle_t xSamplesAvailSemph;
/** Semaphore for Read/Write */
static SemaphoreHandle_t xReadWriteSemph;
/** Semaphore for SPI RX */
static SemaphoreHandle_t xSpiRxSemph;
/** Id for collect frames thread*/
static TaskHandle_t collectSamplesThreadId;

/*=============  C O D E  =============*/

int32_t AdcIfInitThreads(void)
{
    int32_t status = 0;
    BaseType_t threadStatus = pdPASS;
    // Assign the highest priority to the CollectSamplesThread than the example thread.
    threadStatus = xTaskCreate(CollectSamplesThread, "CollectSamplesThread", 512, NULL, 5,
                               &collectSamplesThreadId);
    if (threadStatus == pdPASS)
    {
        xSamplesAvailSemph = xSemaphoreCreateBinary();
        xSpiRxSemph = xSemaphoreCreateBinary();
        xReadWriteSemph = xSemaphoreCreateBinary();
    }
    else
    {
        status = -1;
    }
    return status;
}

ADI_ADC_STATUS AdcIfWaitAdcResponse(ADC_INTERFACE_INFO *pInfo)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    if (pInfo != NULL)
    {
        if (xSemaphoreTake(xReadWriteSemph, portMAX_DELAY) == pdTRUE)
        {
        }
    }
    return adcStatus;
}

void AdcIfSpiRxCallback()
{
    ADC_INTERFACE_INFO *pAdcIf = AdcIfGetInstance();
    pAdcIf->isSpiRunning = 0;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // Release the sempahore thats blocked to validate the frames in CollectSamplesThread
    xSemaphoreGiveFromISR(xSpiRxSemph, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void AdcIfDreadyCallback(uint32_t port, uint32_t pinFlag)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    ADC_INTERFACE_INFO *pAdcIf = AdcIfGetInstance();
    if (pAdcIf->enableRun == 1)
    {
        pAdcIf->prevDreadyTime = pAdcIf->currDreadyTime;
        pAdcIf->currDreadyTime = EvbGetTime();
        UpdateDreadyErrorFlag(pAdcIf);
        pAdcIf->isSpiRunning = 1;
        // Release the sempahore thats blocked in the CollectSamplesThread for Dready callback
        xSemaphoreGiveFromISR(xSamplesAvailSemph, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
    if (pAdcIf->isTdmCycleRunning == 1)
    {
        pAdcIf->tamperCnt++;
    }
}

ADI_ADC_STATUS AdcIfAdcCallback(void *hUser, uint32_t adcEvent)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    ADC_INTERFACE_INFO *pAdcIf;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (hUser != NULL)
    {
        pAdcIf = AdcIfGetInstance();
        if ((adcEvent & ADI_ADC_EVENT_BITM_RESPONSE_READY) != 0)
        {
            // Release the sempahore thats blocked in the WaitAdcResponse for read/write operation
            xSemaphoreGiveFromISR(xReadWriteSemph, &xHigherPriorityTaskWoken);
        }
        if ((adcEvent & ADI_ADC_EVENT_BITM_BLOCK_READY) != 0)
        {
            pAdcIf->blockReady = true;
        }
    }
    else
    {
        status = ADI_ADC_STATUS_NULL_PTR;
    }
    return status;
}

void CollectSamplesThread(void *pArg)
{
    ADC_INTERFACE_INFO *pAdcIf;
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;

    if (pArg == NULL)
    {

        while (1)
        {
            if (xSemaphoreTake(xSamplesAvailSemph, portMAX_DELAY) == pdTRUE)
            {
                pAdcIf = AdcIfGetInstance();
                status = adi_adc_CollectSamples(pAdcIf->hAdc, pAdcIf->currDreadyTime);
                if (status == ADI_ADC_STATUS_BUFFER_OVERFLOW)
                {
                    EvbLedOn(2);
                }
                if (xSemaphoreTake(xSpiRxSemph, portMAX_DELAY) == pdTRUE)
                {
                    pAdcIf = AdcIfGetInstance();
                    adi_adc_ValidateFrames(pAdcIf->hAdc);
                }
                else
                {
                    /* Error handling */
                }
            }
            else
            {
                /* Error handling */
            }
        }
    }
}

void UpdateDreadyErrorFlag(ADC_INTERFACE_INFO *pInfo)
{
    /* time is us -- Convert to second*/
    float timeFor1Sample = (float)(pInfo->currDreadyTime - pInfo->prevDreadyTime) / 1000000;
    float timeFor1sSamples = (float)pInfo->adcSamplingRate * timeFor1Sample;
    float error = (float)fabsf(timeFor1sSamples - 1.0f);

    if ((error > (float)0.25) && pInfo->dreadyCnt > 1)
    {
        pInfo->dreadyError = 1;
        EvbLedOn(5);
    }
}

/**
 * @}
 */
