/******************************************************************************
 Copyright (c) 2025  Analog Devices Inc.
******************************************************************************/
#include "adc_datapath_cfg.h"
#include "adc_service_interface.h"
#include "adi_evb.h"
#include "app_cfg.h"
#include "message.h"
#include <string.h>

/* Max number of samples to buffer */
#define ADC_NUM_SAMPLES_REQUIRED 64000

#if BOARD_TYPE_BCM_ADEMA127
static uint8_t numVoltageChannels = 3;
static uint8_t numCurrentChannels = 25;
static uint8_t numTotalAdcChannels = 28;
static uint32_t channelMask = 0xFFFFFFF;

static ADC_BOARD_CONFIG adcBoardConfig = {
    .voltageSlots = {2, 1, 0},
    .currentSlots = {3,  7,  4,  8,  5,  9,  6,  10, 14, 11, 15, 12, 16,
                     13, 17, 21, 18, 22, 19, 23, 20, 24, 25, 26, 27},
    .numAdc = 4,
    .adcType = {ADI_ADC_TYPE_ADEMA127, ADI_ADC_TYPE_ADEMA127, ADI_ADC_TYPE_ADEMA127,
                ADI_ADC_TYPE_ADEMA127},
    .clkIn = 16384000,
    .adcSamplingRate = 8000,
    .decimateBy2 = 1,
    .adcInv = {0},
    .adcStreamMode = ADI_ADC_STREAM_MODE_NORM,
};

#elif BOARD_TYPE_BCM_ADE9113_ADEMA127
static uint8_t numVoltageChannels = 3;
static uint8_t numCurrentChannels = 21;
static uint8_t numTotalAdcChannels = 24;
static uint32_t channelMask = 0xFFFFFFF;

static ADC_BOARD_CONFIG adcBoardConfig = {
    .voltageSlots = {2, 1, 0},
    .currentSlots = {3, 7, 4, 8, 5, 9, 6, 10, 14, 11, 15, 12, 16, 13, 17, 21, 18, 22, 19, 23, 20},
    .numAdc = 4,
    .adcType = {ADI_ADC_TYPE_ADE91XX, ADI_ADC_TYPE_ADEMA127, ADI_ADC_TYPE_ADEMA127,
                ADI_ADC_TYPE_ADEMA127},
    .clkIn = 16384000,
    .adcSamplingRate = 8000,
    .decimateBy2 = 1,
    .adcInv = {0},
    .adcStreamMode = ADI_ADC_STREAM_MODE_NORM,
};

#else
static uint8_t numVoltageChannels = 3;
static uint8_t numCurrentChannels = 4;
static uint8_t numTotalAdcChannels = 7;
static uint32_t channelMask = 0x7F;

static ADC_BOARD_CONFIG adcBoardConfig = {
    .voltageSlots = {2, 3, 6},
    .currentSlots = {0, 1, 4, 5},
    .numAdc = 1,
    .adcType = {ADI_ADC_TYPE_ADEMA127},
    .clkIn = 16384000,
    .adcSamplingRate = 8000,
    .decimateBy2 = 1,
    .adcInv = {0},
    .adcStreamMode = ADI_ADC_STREAM_MODE_NORM,
};

#endif

static int32_t adcSamples[ADC_NUM_SAMPLES_REQUIRED];
static volatile int32_t dready = 0;
static volatile int32_t spiComplete = 0;
static ADC_INTERFACE_INFO adcsIf;
static ADI_EVB_CONFIG evbConfig;
static void *hEvb;

static void DreadyCallback(uint32_t port, uint32_t pin);
static void SpiRxCallback(void);
static void PrintOutput(int32_t *pSamples, int32_t numSamples);

int main()
{
    int32_t i;
    int32_t numSamples = 0;
    int32_t numRequiredSamples =
        (ADC_NUM_SAMPLES_REQUIRED / numTotalAdcChannels) * numTotalAdcChannels;
    int32_t numSamplesPerChannelToDisplay = 10;
    ADC_INTERFACE_INFO *pAdcsIf = &adcsIf;
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    int32_t status = 0;
    int32_t numAdc;
    int32_t *pSamples = &adcSamples[0];

    ADI_EVB_CONFIG *pEvbConfig = &evbConfig;
    pEvbConfig->spiConfig.pfAdeSpiRxCallback = SpiRxCallback;
    pEvbConfig->gpioConfig.pfGpioCallback = DreadyCallback;
    EvbInit(&hEvb, pEvbConfig);

    EvbInitMessageBuffer();
    printf("\n**************** ADC Service Sample Collection Example ******************\n");

    /* Create ADC Service */
    AdcIfCreateService(pAdcsIf);

    pAdcsIf->pfCallback = NULL;
    status = AdcIfInitService(pAdcsIf, &adcBoardConfig);
    if (status == 0)
    {
        status = AdcIfStartCapture(pAdcsIf);
    }

    if (status == 0)
    {
        while (numSamples < numRequiredSamples)
        {
            if (dready == 1)
            {
                dready = 0;
                spiComplete = 0;
                adi_adc_CollectSamples(pAdcsIf->hAdc, 0);
                while (spiComplete == 0)
                {
                    ;
                }
                adi_adc_ValidateSamples(pAdcsIf->hAdc);
                adcStatus = adi_adc_ReadBlock(pAdcsIf->hAdc, &pAdcsIf->adcSamples[0],
                                              &pAdcsIf->adcStatusOutput[0]);

                if (adcStatus == ADI_ADC_STATUS_SUCCESS)
                {
                    /* Extract the required channels from the sample */
                    numSamples += adi_adcutil_ExtractChannel(
                        &pAdcsIf->adcSamples[0], pAdcsIf->adcCfg.numSamplesInBlock,
                        pAdcsIf->runInfo.totalChannels, channelMask, &pSamples[numSamples]);
                }
            }
        }
    }

#ifdef DISABLE_ASCII_OUT
    printf("Collected %d samples\n", numSamples);
    printf("Displaying first %d samples per channel:\n", numSamplesPerChannelToDisplay);
    PrintOutput(pSamples, numSamplesPerChannelToDisplay);
    EvbFlushMessages();
#else
    /* Send samples to host */
    EvbHostCommTransmitAsync(hEvb, (uint8_t *)pSamples, numRequiredSamples * sizeof(int32_t));
#endif
    /* Check whether data contains proper output*/
    while (1)
    {
        ;
    }
}

/**
 * @brief Print the output samples for each channel
 *
 * @param pSamples Pointer to the samples array
 * @param numSamples Number of samples to print
 */
void PrintOutput(int32_t *pSamples, int32_t numSamples)
{
    int32_t i;
    for (i = 0; i < numSamples; i++)
    {
        // Print voltage channels
        for (int32_t j = 0; j < numVoltageChannels; j++)
        {
            printf("V%d:0x%04X ", j, pSamples[adcBoardConfig.voltageSlots[j]]);
        }
        // Print current channels
        for (int32_t j = 0; j < numCurrentChannels; j++)
        {
            printf("I%d:0x%04X ", j, pSamples[adcBoardConfig.currentSlots[j]]);
        }
        // Move pointer to next sample set
        pSamples += (numVoltageChannels + numCurrentChannels);
        printf("\n");
    }
}

/**
 * @brief GPIO callback for ADC data ready signal
 *
 * @param port GPIO port
 * @param pin GPIO pin
 */
void DreadyCallback(uint32_t port, uint32_t pin)
{
    ADC_INTERFACE_INFO *pAdcsIf = AdcIfGetInstance();
    // To resolve the warning for unused parameters
    (void)port;
    (void)pin;
    pAdcsIf->dreadyFlag = 1;
    dready = 1;
}

/**
 * @brief SPI receive complete callback
 */
void SpiRxCallback()
{
    spiComplete = 1;
}

ADI_ADC_STATUS AdcIfWaitAdcResponse(ADC_INTERFACE_INFO *pInfo)
{
    // Not required for this example
    (void)pInfo; // To resolve the warning for unused parameter
    return ADI_ADC_STATUS_SUCCESS;
}
