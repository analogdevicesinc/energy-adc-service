/******************************************************************************
 Copyright (c) 2025  Analog Devices Inc.
******************************************************************************/

/**
 * @file     adi_adc_utility.c
 * @brief    Utility functions for ADC Service.
 * @{
 */

/*============= I N C L U D E S =============*/
#include "ADE911X_addr_rdef.h"
#include "ADEMA127_addr_def.h"
#include "ADEMA127_addr_rdef.h"
#include "adi_adc.h"

/*=============  D E F I N I T I O N S  =============*/
/** Max value of Fmod */
#define FMOD_MAX 2048000
/** max value of Fdsp */
#define FDSP_MAX 64000000
/** Min value of clk input */
#define CLK_IN_MIN 3000000
/** Max value of clk input */
#define CLK_OUT_MIN 16384000

static ADI_ADC_STATUS PopulateStreamModeAde91xx(ADI_ADC_STREAM_MODE streamMode,
                                                uint8_t *pConfig0StreamDbg);
static ADI_ADC_STATUS PopulateStreamModeAdema12x(ADI_ADC_STREAM_MODE streamMode,
                                                 uint8_t *pConfig0StreamDbg);
static ADI_ADC_STATUS PopulateSamplingRateAde91xx(uint32_t samplingRate, uint8_t *pConfigFilt);
static ADI_ADC_STATUS PopulateSamplingRateAdema12x(uint32_t clkIn, uint32_t samplingRate,
                                                   uint8_t decimateBy2, uint8_t *pDatarate);

ADI_ADC_STATUS adi_adcutil_PopulateStreamMode(ADI_ADC_STREAM_MODE streamMode, uint8_t numAdc,
                                              ADI_ADC_TYPE *pAdcType,
                                              ADI_ADC_CONFIG_REGISTERS *pConfigReg)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    uint8_t i;

    if (numAdc <= 0)
    {
        status = ADI_ADC_STATUS_INVALID_NUM_ADC;
    }
    for (i = 0; i < numAdc; i++)
    {
        if (status == ADI_ADC_STATUS_SUCCESS)
        {
            if ((pAdcType[i] != ADI_ADC_TYPE_ADE91XX) && (pAdcType[i] != ADI_ADC_TYPE_ADEMA124) &&
                (pAdcType[i] != ADI_ADC_TYPE_ADEMA127))
            {
                status = ADI_ADC_STATUS_INVALID_ADC_TYPE;
            }
        }
    }

    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        for (i = 0; i < numAdc; i++)
        {
            if (status == ADI_ADC_STATUS_SUCCESS)
            {
                if (pAdcType[i] == ADI_ADC_TYPE_ADE91XX)
                {
                    status = PopulateStreamModeAde91xx(streamMode, &pConfigReg[i].config0StreamDbg);
                }
                else if (pAdcType[i] == ADI_ADC_TYPE_ADEMA124 ||
                         pAdcType[i] == ADI_ADC_TYPE_ADEMA127)
                {
                    status =
                        PopulateStreamModeAdema12x(streamMode, &pConfigReg[i].config0StreamDbg);
                }
            }
        }
    }

    return status;
}

ADI_ADC_STATUS adi_adcutil_PopulateSamplingRate(uint32_t clkIn, uint32_t samplingRate,
                                                uint8_t decimateBy2, uint8_t numAdc,
                                                ADI_ADC_TYPE *pAdcType,
                                                ADI_ADC_CONFIG_REGISTERS *pConfigReg)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    uint8_t i;

    if (numAdc <= 0)
    {
        status = ADI_ADC_STATUS_INVALID_NUM_ADC;
    }
    for (i = 0; i < numAdc; i++)
    {
        if (status == ADI_ADC_STATUS_SUCCESS)
        {
            if ((pAdcType[i] != ADI_ADC_TYPE_ADE91XX) && (pAdcType[i] != ADI_ADC_TYPE_ADEMA124) &&
                (pAdcType[i] != ADI_ADC_TYPE_ADEMA127))
            {
                status = ADI_ADC_STATUS_INVALID_ADC_TYPE;
            }
        }
    }
    for (i = 0; i < numAdc; i++)
    {
        if (status == ADI_ADC_STATUS_SUCCESS)
        {
            if (pAdcType[i] == ADI_ADC_TYPE_ADE91XX)
            {
                status = PopulateSamplingRateAde91xx(samplingRate, &pConfigReg[i].configFilt);
            }
            else if (pAdcType[i] == ADI_ADC_TYPE_ADEMA124 || pAdcType[i] == ADI_ADC_TYPE_ADEMA127)
            {
                status = PopulateSamplingRateAdema12x(clkIn, samplingRate, decimateBy2,
                                                      &pConfigReg[i].datarate);
            }
        }
    }

    return status;
}

ADI_ADC_STATUS PopulateStreamModeAde91xx(ADI_ADC_STREAM_MODE streamMode, uint8_t *pConfig0StreamDbg)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    if (streamMode == ADI_ADC_STREAM_MODE_INCR)
    {
        *pConfig0StreamDbg = ENUM_ADE911X_MAP0_CONFIG0_STREAM_DBG_COUNT_MODE;
    }
    else if (streamMode == ADI_ADC_STREAM_MODE_STATIC)
    {
        *pConfig0StreamDbg = ENUM_ADEMA127_MMR_RETAINED_CONFIG0_STREAM_DBG_STATIC_MODE;
    }
    else if (streamMode == ADI_ADC_STREAM_MODE_NORM)
    {
        *pConfig0StreamDbg = ENUM_ADE911X_MAP0_CONFIG0_STREAM_DBG_NORM_OP;
    }
    else
    {
        status = ADI_ADC_STATUS_INVALID_STREAM_MODE;
    }
    return status;
}

ADI_ADC_STATUS PopulateStreamModeAdema12x(ADI_ADC_STREAM_MODE streamMode,
                                          uint8_t *pConfig0StreamDbg)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    if (streamMode == ADI_ADC_STREAM_MODE_INCR)
    {
        *pConfig0StreamDbg = ENUM_ADEMA127_MMR_RETAINED_CONFIG0_STREAM_DBG_PROG_INCR;
    }
    else if (streamMode == ADI_ADC_STREAM_MODE_STATIC)
    {
        *pConfig0StreamDbg = ENUM_ADEMA127_MMR_RETAINED_CONFIG0_STREAM_DBG_STATIC_MODE;
    }
    else if (streamMode == ADI_ADC_STREAM_MODE_NORM)
    {
        *pConfig0StreamDbg = ENUM_ADEMA127_MMR_RETAINED_CONFIG0_STREAM_DBG_NORM_OP;
    }
    else
    {
        status = ADI_ADC_STATUS_INVALID_STREAM_MODE;
    }
    return status;
}

ADI_ADC_STATUS PopulateSamplingRateAde91xx(uint32_t samplingRate, uint8_t *pConfigFilt)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    if (samplingRate == 4000)
    {
        *pConfigFilt = ENUM_ADE911X_MAP0_CONFIG_FILT_DATAPATH_CONFIG_SINC3_LPF_4KHZ;
    }
    else if (samplingRate == 32000)
    {
        *pConfigFilt = ENUM_ADE911X_MAP0_CONFIG_FILT_DATAPATH_CONFIG_SINC3_LPF_32KHZ;
    }
    else if (samplingRate == 8000)
    {
        *pConfigFilt = ENUM_ADE911X_MAP0_CONFIG_FILT_DATAPATH_CONFIG_SINC3_COMP_LPF_8KHZ;
    }
    else
    {
        status = ADI_ADC_STATUS_INVALID_SAMPLING_RATE;
    }
    return status;
}

uint32_t adi_adcutil_ExtractChannel(int32_t *pSrc, uint32_t numSamples, uint32_t numChannels,
                                    uint32_t channelMask, int32_t *pDst)
{
    uint32_t extractedSamples = 0;
    uint32_t mask;
    uint32_t sampleIdx;
    uint32_t channelIdx;

    for (sampleIdx = 0; sampleIdx < numSamples; sampleIdx++)
    {
        for (channelIdx = 0; channelIdx < numChannels; channelIdx++)
        {
            mask = 1 << channelIdx;
            if (channelMask & mask)
            {
                *pDst++ = *pSrc;
                extractedSamples++;
            }
            pSrc++;
        }
    }
    return extractedSamples;
}

ADI_ADC_STATUS PopulateSamplingRateAdema12x(uint32_t clkIn, uint32_t samplingRate,
                                            uint8_t decimateBy2, uint8_t *pDatarate)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    uint32_t adcClkPrescaler = 0;
    uint32_t decimationRate = 0;
    uint32_t totalDivider;
    // Try to find a valid prescaler and decimation rate combination
    bool validConfigFound = false;
    uint32_t adcClk;
    uint32_t decimationFactor;
    uint32_t calculatedFs;
    uint8_t prescaler;
    uint16_t deci;

    if ((clkIn < CLK_IN_MIN) || (clkIn > CLK_OUT_MIN))
    {
        status = ADI_ADC_STATUS_INVALID_SAMPLING_RATE;
    }

    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        for (prescaler = 1; prescaler <= 5; prescaler++)
        {
            // Loop over 2, 4, 8, 16, 32
            adcClk = clkIn / (1 << prescaler); // Compute ADC clock after prescaler
            if (adcClk > FMOD_MAX)
            {
                // f_MOD = 2.048 MHz
                continue;
            }
            for (deci = 0; deci <= 8; deci++) // Loop over decimation rates (32, 64, ... 8192)
            {
                decimationFactor = 32 << deci; // 32, 64, 128, ..., 8192
                calculatedFs = adcClk / decimationFactor;

                if (decimateBy2)
                {
                    calculatedFs /= 2;
                }
                if (calculatedFs > FDSP_MAX)
                {
                    // f_DSP = 64 MHz
                    continue;
                }

                /* Valid fs configuration should have totalDivider <= 16 */
                totalDivider = 5 + prescaler + deci + decimateBy2;

                if (calculatedFs == samplingRate && totalDivider <= 16)
                {
                    adcClkPrescaler = prescaler;
                    decimationRate = deci;
                    validConfigFound = true;
                    break;
                }
            }
            if (validConfigFound)
            {
                break;
            }
        }

        if (!validConfigFound)
        {
            status = ADI_ADC_STATUS_INVALID_SAMPLING_RATE;
        }
        else
        {
            // Populate the config register with selected values
            *pDatarate = ((decimationRate << BITP_ADEMA127_MMR_DATARATE_DECIMATION_RATE) |
                          (adcClkPrescaler << BITP_ADEMA127_MMR_DATARATE_ADC_CLK_PRESCALER) |
                          (decimateBy2 << BITP_ADEMA127_MMR_DATARATE_DSP_DECIMATION_X2));
        }
    }

    return status;
}

/**
 * @}
 */
