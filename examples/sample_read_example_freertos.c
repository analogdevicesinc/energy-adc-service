/******************************************************************************
 Copyright (c) 2025  Analog Devices Inc.
******************************************************************************/
#include "FreeRTOS.h"
#include "adc_service_interface.h"
#include "adi_evb.h"
#include "app_cfg.h"
#include "queue.h"
#include "semphr.h"
#include <string.h>

/** Number of samples in the buffer */
#define NUM_BUFFER_SAMPLES 16000
/** Number of samples required */
#define ADC_NUM_SAMPELS_REQUIRED 64000

static uint8_t voltageSlots[APP_CFG_MAX_NUM_VOLTAGE_CHANNELS] = {3, 0};
static uint8_t currentSlots[APP_CFG_MAX_NUM_CURRENT_CHANNELS] = {2, 1};
/** Buffer to write and store the message */
static int32_t sampleBuffer0[NUM_BUFFER_SAMPLES];
/** Buffer to write and store the message */
static int32_t sampleBuffer1[NUM_BUFFER_SAMPLES];

static void PopulateAdcInterface(ADC_INTERFACE_INFO *pInfo);
static void SpiRxCallback(void);
static void DreadyCallback(void);
static void HostUartTxCallback(void);
static int32_t InitThreads(void);
void CollectSamplesThread(void *pArg);
void CmdThread(void *pArg);

static volatile int32_t dready = 0;
static volatile int32_t spiComplete = 0;
static ADC_INTERFACE_INFO adcsIf;
static void *hEvb;

/** Semaphore for collect frames */
static SemaphoreHandle_t xSamplesAvailSemph;
/** Semaphore for Read/Write */
static SemaphoreHandle_t xReadWriteSemph;
/** Semaphore for SPI RX */
static SemaphoreHandle_t xSpiRxSemph;
/** Semaphore for UART Data */
static SemaphoreHandle_t xTransmitSemph;
/** Id for collect frames thread*/
static TaskHandle_t collectSamplesThreadId;
/** Id for command thread*/
static TaskHandle_t cmdThreadId;
/** Max length of the queue to store the buffer information */
#define QUEUE_LENGTH 1
/** Queue to store the buffer information */
static QueueHandle_t xQueue;

/** Structure to store the buffer information */
typedef struct
{
    /** Pointer to the buffer */
    int32_t *pBuffer;
    /** Number of samples in the buffer */
    int32_t numSamples;
    /** Flag to indicate if the buffer is in use */
    volatile uint8_t inUse;
    /** Pointer to the inUse flag */
    uint8_t *pInUse;
} BUFFER_INFO;

int32_t main()
{
    int32_t status = 0;
    ADI_EVB_CONFIG evbConfig;
    ADI_EVB_CONFIG *pEvbConfig = &evbConfig;
    pEvbConfig->spiConfig.pfAdeSpiRxCallback = SpiRxCallback;
    pEvbConfig->gpioConfig.pfGpioCallback = DreadyCallback;
    pEvbConfig->uartConfig.pfHostUartTxCallback = HostUartTxCallback;

    EvbInit(&hEvb, pEvbConfig);

    status = InitThreads();
    vTaskStartScheduler();
    while (1)
    {
        // Loop indefinitely
    }
}

int32_t InitThreads(void)
{
    int32_t status = 0;
    BaseType_t threadStatus = pdPASS;

    threadStatus = xTaskCreate(CmdThread, "CmdThread", 512, NULL, 3, &cmdThreadId);

    threadStatus = xTaskCreate(CollectSamplesThread, "CollectSamplesThread", 512, NULL, 5,
                               &collectSamplesThreadId);

    if (threadStatus == pdPASS)
    {
        xSamplesAvailSemph = xSemaphoreCreateBinary();
        xSpiRxSemph = xSemaphoreCreateBinary();
        xReadWriteSemph = xSemaphoreCreateBinary();
        xTransmitSemph = xSemaphoreCreateBinary();
        xQueue = xQueueCreate(QUEUE_LENGTH, sizeof(BUFFER_INFO));

        if (xQueue == NULL)
        {
            // Queue creation failed
            status = -1;
        }
    }
    else
    {
        status = -1;
    }
    return status;
}

void CollectSamplesThread(void *pArg)
{
    int32_t i;
    int32_t numSamples = 0;
    int32_t numRequiredSamples = ADC_NUM_SAMPELS_REQUIRED;
    ADC_INTERFACE_INFO *pAdcsIf = &adcsIf;
    int32_t status = 0;
    ADI_ADC_CONFIG *pAdcCfg = &pAdcsIf->adcCfg;
    int32_t numAdc;
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    int32_t extractSamples = 0;
    int32_t totalSamples = 0;
    static BUFFER_INFO bufferInfo[2];
    int32_t idx = 0;
    uint32_t channelMask = 0x7F;
    // Initialize buffer states
    memset(bufferInfo, 0, sizeof(bufferInfo)); // Ensure all buffers start as free
    bufferInfo[0].pBuffer = &sampleBuffer0[0];
    bufferInfo[1].pBuffer = &sampleBuffer1[0];
    bufferInfo[0].pInUse = &bufferInfo[0].inUse;
    bufferInfo[1].pInUse = &bufferInfo[1].inUse;

    AdcIfCreateService(pAdcsIf);

    PopulateAdcInterface(pAdcsIf);
    status = AdcIfInitService(pAdcsIf, numAdc, &pAdcsIf->adcCfg.adcType[0]);
    status = AdcIfStartCapture(pAdcsIf);
    if (status != 0)
    {
        // Error handling
    }
    else
    {
        while (totalSamples < numRequiredSamples)
        {
            if (xSemaphoreTake(xSamplesAvailSemph, portMAX_DELAY) == pdTRUE)
            {
                adcStatus = adi_adc_CollectSamples(pAdcsIf->hAdc, 0);
                if (adcStatus != ADI_ADC_STATUS_SUCCESS)
                {
                    status = -1;
                    // Error handling
                }
                else
                {
                    if (xSemaphoreTake(xSpiRxSemph, portMAX_DELAY) == pdTRUE)
                    {
                        adi_adc_ValidateSamples(pAdcsIf->hAdc);
                        adcStatus = adi_adc_ReadBlock(pAdcsIf->hAdc, &pAdcsIf->adcSamples[0],
                                                      &pAdcsIf->adcStatusOutput[0]);

                        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
                        {
                            // If buffer not in use then store the samples in the buffer
                            if (bufferInfo[idx].inUse == 0)
                            {
                                /* Extract the required channels from the sample */
                                extractSamples = adi_adcutil_ExtractChannel(
                                    &pAdcsIf->adcSamples[0], pAdcCfg->numSamplesInBlock,
                                    pAdcsIf->runInfo.totalChannels, channelMask,
                                    &bufferInfo[idx].pBuffer[numSamples]);
                                numSamples += extractSamples;
                                if (numSamples == NUM_BUFFER_SAMPLES)
                                {
                                    bufferInfo[idx].numSamples = numSamples;
                                    if (xQueueSend(xQueue, &bufferInfo[idx], 0) != pdPASS)
                                    {
                                        // Queue send failed
                                        status = -1;
                                    }
                                    idx ^= 0x1;
                                    totalSamples += numSamples;
                                    numSamples = 0;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void CmdThread(void *pArg)
{
    BUFFER_INFO bufferInfo;
    while (1)
    {
        // Wait for the queue to have data
        if (xQueueReceive(xQueue, &bufferInfo, portMAX_DELAY) == pdTRUE)
        {
            // Wait for the semaphore to be available
            // Send samples through UART for the first time without waiting for the semaphore
            *bufferInfo.pInUse = 1; // Mark as busy
            EvbStartHostUartTxAsync(hEvb, (uint8_t *)bufferInfo.pBuffer,
                                    bufferInfo.numSamples * sizeof(uint32_t));
            if (xSemaphoreTake(xTransmitSemph, portMAX_DELAY) != pdTRUE)
            {
                // error handling
            }
            *bufferInfo.pInUse = 0; // Mark as free
        }
    }
}

void DreadyCallback()
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    ADC_INTERFACE_INFO *pAdcIf = AdcIfGetInstance();
    pAdcIf->dreadyFlag = 1;
    if (pAdcIf->enableRun == 1)
    {
        // Release the semaphore that's blocked in the CollectSamplesThread for Dready callback
        xSemaphoreGiveFromISR(xSamplesAvailSemph, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void SpiRxCallback()
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    ADC_INTERFACE_INFO *pAdcIf = AdcIfGetInstance();

    if (pAdcIf->enableRun == 1)
    {
        // Release the semaphore that's blocked to validate the frames in CollectSamplesThread
        xSemaphoreGiveFromISR(xSpiRxSemph, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void HostUartTxCallback(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // Release the semaphore that's blocked in the CmdThread for UART TX completion
    xSemaphoreGiveFromISR(xTransmitSemph, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief Populate ADC interface inputs
 */
static void PopulateAdcInterface(ADC_INTERFACE_INFO *pInfo)
{
    uint8_t i = 0;
    pInfo->numAdc = 1;
    for (i = 0; i < APP_CFG_MAX_NUM_VOLTAGE_CHANNELS; i++)
    {
        pInfo->adcBoardConfig.voltageSlots[i] = voltageSlots[i];
    }
    for (i = 0; i < APP_CFG_MAX_NUM_CURRENT_CHANNELS; i++)
    {
        pInfo->adcBoardConfig.currentSlots[i] = currentSlots[i];
    }

    for (i = 0; i < (pInfo->numAdc); i++)
    {
        pInfo->adcType[i] = ADI_ADC_TYPE_ADEMA127;
    }
    pInfo->adcStreamMode = ADI_ADC_STREAM_MODE_NORM;
    pInfo->adcSamplingRate = APP_CFG_ADC_SAMPLING_RATE;
    pInfo->clkIn = APP_CFG_ADC_MCLK;
    pInfo->decimateBy2 = APP_CFG_ADC_DECIMATION_BY2;

    pInfo->pfCallback = NULL;
}

ADI_ADC_STATUS AdcIfWaitAdcResponse(ADC_INTERFACE_INFO *pInfo)
{
    // Not required for this example
    (void)pInfo; // To resolve the warning for unused parameter
    return ADI_ADC_STATUS_SUCCESS;
}
