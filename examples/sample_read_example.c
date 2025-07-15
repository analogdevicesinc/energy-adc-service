/******************************************************************************
 Copyright (c) 2025  Analog Devices Inc.
******************************************************************************/
#include "adc_datapath_cfg.h"
#include "adc_service_interface.h"
#include "adi_evb.h"
#include "app_cfg.h"
#include "message.h"
#include <string.h>

/* Number of samples to buffer*/
#define ADC_NUM_SAMPLES_REQUIRED 64000

static uint8_t voltageSlots[APP_CFG_MAX_NUM_VOLTAGE_CHANNELS] = {3, 0};
static uint8_t currentSlots[APP_CFG_MAX_NUM_CURRENT_CHANNELS] = {2, 1};

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
    int32_t numRequiredSamples = ADC_NUM_SAMPLES_REQUIRED;
    int32_t numSamplesPerChannelToDisplay = 10;
    ADC_INTERFACE_INFO *pAdcsIf = &adcsIf;
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    int32_t status = 0;
    ADI_ADC_CONFIG *pAdcCfg = &pAdcsIf->adcCfg;
    int32_t numAdc;
    int32_t *pSamples = &adcSamples[0];
    uint32_t channelMask = 0x7F;

    ADI_EVB_CONFIG *pEvbConfig = &evbConfig;
    pEvbConfig->spiConfig.pfAdeSpiRxCallback = SpiRxCallback;
    pEvbConfig->gpioConfig.pfGpioCallback = DreadyCallback;
    EvbInit(&hEvb, pEvbConfig);

    EvbInitMessageBuffer();
    printf("\n**************** ADC Service Sample Collection Example ******************\n");

    /* Create ADC Service */
    AdcIfCreateService(pAdcsIf);
    for (i = 0; i < APP_CFG_MAX_NUM_VOLTAGE_CHANNELS; i++)
    {
        pAdcsIf->adcBoardConfig.voltageSlots[i] = voltageSlots[i];
    }
    for (i = 0; i < APP_CFG_MAX_NUM_CURRENT_CHANNELS; i++)
    {
        pAdcsIf->adcBoardConfig.currentSlots[i] = currentSlots[i];
    }
    pAdcCfg->numSamplesInBlock = APP_CFG_DEFAULT_SAMPLE_BLOCK_SIZE;
    numAdc = 1;
    pAdcCfg->adcType[0] = ADI_ADC_TYPE_ADEMA127;
    pAdcsIf->adcSamplingRate = APP_CFG_ADC_SAMPLING_RATE;
    pAdcsIf->clkIn = APP_CFG_ADC_MCLK;
    pAdcsIf->decimateBy2 = APP_CFG_ADC_DECIMATION_BY2;
    pAdcsIf->adcStreamMode = ADI_ADC_STREAM_MODE_NORM;

    status = AdcIfInitService(pAdcsIf, numAdc, &pAdcsIf->adcCfg.adcType[0]);
    status = AdcIfStartCapture(pAdcsIf);

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
                        &pAdcsIf->adcSamples[0], pAdcCfg->numSamplesInBlock,
                        pAdcsIf->runInfo.totalChannels, channelMask, &pSamples[numSamples]);
                }
            }
        }
    }
#ifdef DISABLE_ASCII_OUT
    printf("Collected %d samples\n", numSamples);
    printf("Displaying first %d samples per channel:\n", numSamplesPerChannelToDisplay);
    PrintOutput(pSamples, 10);
    EvbFlushMessages();
#else
    /* Send samples to host */
    EvbHostUartTransmitAsync(hEvb, (uint8_t *)pSamples, numRequiredSamples * sizeof(int32_t));
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
        for (int32_t j = 0; j < APP_CFG_MAX_NUM_VOLTAGE_CHANNELS; j++)
        {
            printf("V%d:0x%04X ", j, pSamples[voltageSlots[j]]);
        }
        // Print current channels
        for (int32_t j = 0; j < APP_CFG_MAX_NUM_CURRENT_CHANNELS; j++)
        {
            printf("I%d:0x%04X ", j, pSamples[currentSlots[j]]);
        }
        // Move pointer to next sample set
        pSamples += (APP_CFG_MAX_NUM_VOLTAGE_CHANNELS + APP_CFG_MAX_NUM_CURRENT_CHANNELS);
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