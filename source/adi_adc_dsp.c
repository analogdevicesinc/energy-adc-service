/******************************************************************************
 Copyright (c) 2025  Analog Devices Inc.
******************************************************************************/

/**
 * @file     adi_adc_dsp.c
 * @brief    API definitions to configure ADC uDSP
 * @{
 */

/*============= I N C L U D E S =============*/
#include "adi_adc_dsp.h"
#include "ADEMA127_addr_def.h"
#include "ADEMA127_addr_rdef.h"
#include "adc_datapath_configure.h"
#include <string.h>

/*=============  D E F I N I T I O N S  =============*/
static uint32_t hpfCoeff[14][7] = {
    {0x400000E0, 0x800000EC, 0x400000E0, 0x7FFE9470, 0x0A000074, 0xC0016B60, 0xF20000BC},
    {0x400000E0, 0x800000EC, 0x400000E0, 0x7FFD2818, 0x14000014, 0xC002D708, 0xDC0000D0},
    {0x400000E0, 0x800000EC, 0x400000E0, 0x7FFA5004, 0x28000014, 0xC005AF14, 0x970000F4},
    {0x400000E0, 0x800000EC, 0x400000E0, 0x7FF4A0A0, 0x50000030, 0xC00B5EAC, 0xAD00008C},
    {0x3FFD938C, 0x8004DAF4, 0x3FFD938C, 0x7FE940C4, 0xA0000030, 0xC016BBE0, 0x56000048},
    {0x3FF2354C, 0x801B96DC, 0x3FF2354C, 0x7FD2817C, 0x4200005C, 0xC02D6E20, 0x980000F4},
    {0x3FDB7E60, 0x80490498, 0x3FDB7E60, 0x7FA502E4, 0x8F0000EC, 0xC05ABCB0, 0xF10000B0},
    {0x3FAE28D0, 0x80A3B0B8, 0x3FAE28D0, 0x7F4A059C, 0x79000094, 0xC0B4F9D4, 0x3C000000},
    {0x3F53DB5C, 0x81584AAC, 0x3F53DB5C, 0x7E940D0C, 0xC3000000, 0xC167F2EC, 0xBA000094},
    {0x3EA0B9D4, 0x82BE8E30, 0x3EA0B9D4, 0x7D283140, 0xA300003C, 0xC2C7FDEC, 0x080000C8},
    {0x3D403D1C, 0x857F86A4, 0x3D403D1C, 0x7A510DD0, 0x8100005C, 0xC571099C, 0x320000B0},
    {0x3A95D818, 0x8AD45068, 0x3A95D818, 0x74A70844, 0x9E00008C, 0xCA6B9DE8, 0xA70000F8},
    {0x35969944, 0x94D2CE0C, 0x35969944, 0x696FFBBC, 0xFD0000BC, 0xD324814C, 0xD4000018},
    {0x2CC9235C, 0xA66DBA84, 0x2CC9235C, 0x53AA8EB4, 0xD30000D0, 0xE0862508, 0xF70000C8}};

static void StoreDatapathScfEnable(ADI_ADC_INFO *pInfo,
                                   ADI_ADC_CHAN_DATAPATH_CONFIG *pDatapathEnConfig,
                                   uint8_t *pChanIdx, int8_t numChan, int8_t adcIdx);
static ADI_ADC_STATUS CheckAdcTypeValid(ADI_ADC_INFO *pInfo, int8_t adcIdx);

ADI_ADC_STATUS adi_adc_EnableDatapathConfig(ADI_ADC_HANDLE hAdc,
                                            ADI_ADC_CHAN_DATAPATH_CONFIG *pDatapathEnConfig,
                                            uint8_t *pChanIdx, int8_t numChan, int8_t adcIdx)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    ADI_ADC_INFO *pInfo;

    if (hAdc == NULL)
    {
        adcStatus = ADI_ADC_STATUS_NULL_PTR;
    }
    else
    {
        pInfo = (ADI_ADC_INFO *)hAdc;
        adcStatus = CheckAdcTypeValid(pInfo, adcIdx);
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = CheckChannelValid(pInfo, adcIdx, pChanIdx, numChan);
        }
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = AdcWriteDataPathEnable(pInfo, pDatapathEnConfig, pChanIdx, numChan, adcIdx);
            if (adcStatus == ADI_ADC_STATUS_SUCCESS)
            {
                StoreDatapathScfEnable(pInfo, pDatapathEnConfig, pChanIdx, numChan, adcIdx);
            }
        }
    }

    return adcStatus;
}

ADI_ADC_STATUS adi_adc_SetChannelParams(ADI_ADC_HANDLE hAdc,
                                        ADI_ADC_DSP_CHANNEL_PARAMS *pDspRegisters,
                                        uint8_t *pChanIdx, uint8_t numChannel, int8_t adcIdx)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    ADI_ADC_INFO *pInfo = (ADI_ADC_INFO *)hAdc;
    if (hAdc == NULL)
    {
        adcStatus = ADI_ADC_STATUS_NULL_PTR;
    }
    else
    {
        adcStatus =
            adi_adc_SetChannelGain(pInfo, &pDspRegisters->gain[0], pChanIdx, numChannel, adcIdx);
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = adi_adc_SetChannelOffset(pInfo, &pDspRegisters->offset[0], pChanIdx,
                                                 numChannel, adcIdx);
            if (adcStatus == ADI_ADC_STATUS_SUCCESS)
            {
                adcStatus =
                    adi_adc_SetShift(pInfo, &pDspRegisters->shift[0], pChanIdx, numChannel, adcIdx);
            }
            if (adcStatus == ADI_ADC_STATUS_SUCCESS)
            {
                adcStatus = adi_adc_SetChannelXtGain(pInfo, &pDspRegisters->xtGain[0], pChanIdx,
                                                     numChannel, adcIdx);
            }
            if (adcStatus == ADI_ADC_STATUS_SUCCESS)
            {
                adcStatus = adi_adc_SetChannelXtAggressor(pInfo, &pDspRegisters->xtAggressor[0],
                                                          pChanIdx, numChannel, adcIdx);
            }
        }
    }
    return adcStatus;
}

ADI_ADC_STATUS adi_adc_GetChannelParams(ADI_ADC_HANDLE hAdc, uint8_t *pChanIdx, uint8_t numChannel,
                                        int8_t adcIdx, ADI_ADC_DSP_CHANNEL_PARAMS *pDspRegisters)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    ADI_ADC_INFO *pInfo = (ADI_ADC_INFO *)hAdc;
    if (hAdc == NULL)
    {
        adcStatus = ADI_ADC_STATUS_NULL_PTR;
    }
    else
    {
        adcStatus =
            adi_adc_GetChannelGain(pInfo, pChanIdx, numChannel, adcIdx, &pDspRegisters->gain[0]);
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = adi_adc_GetChannelOffset(pInfo, pChanIdx, numChannel, adcIdx,
                                                 &pDspRegisters->offset[0]);
            if (adcStatus == ADI_ADC_STATUS_SUCCESS)
            {
                adcStatus =
                    adi_adc_GetShift(pInfo, pChanIdx, numChannel, adcIdx, &pDspRegisters->shift[0]);
            }
            if (adcStatus == ADI_ADC_STATUS_SUCCESS)
            {
                adcStatus = adi_adc_GetChannelXtGain(pInfo, pChanIdx, numChannel, adcIdx,
                                                     &pDspRegisters->xtGain[0]);
            }
            if (adcStatus == ADI_ADC_STATUS_SUCCESS)
            {
                adcStatus = adi_adc_GetChannelXtAggressor(pInfo, pChanIdx, numChannel, adcIdx,
                                                          &pDspRegisters->xtAggressor[0]);
            }
        }
    }
    return adcStatus;
}

ADI_ADC_STATUS adi_adc_SetDatapathParams(ADI_ADC_HANDLE hAdc,
                                         ADI_ADC_DSP_DATAPATH_PARAMS *pDspRegisters,
                                         uint8_t *pChanIdx, uint8_t numChannel, int8_t adcIdx)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    ADI_ADC_INFO *pInfo = (ADI_ADC_INFO *)hAdc;
    uint8_t decimationX2;
    uint8_t clkPreScalar;
    uint8_t decimationRate;
    if (hAdc == NULL)
    {
        adcStatus = ADI_ADC_STATUS_NULL_PTR;
    }
    else
    {

        decimationRate = pDspRegisters->datarate.decimationRate;
        clkPreScalar = pDspRegisters->datarate.clkPreScalar;
        decimationX2 = pDspRegisters->datarate.decimationX2;

        adcStatus = adi_adc_SetDataRate(pInfo, decimationRate, clkPreScalar, decimationX2, adcIdx);
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = adi_adc_SetDatapathAlpha(pInfo, (uint8_t *)&pDspRegisters->alpha, adcIdx);
            if (adcStatus == ADI_ADC_STATUS_SUCCESS)
            {
                adcStatus = adi_adc_EnableDatapathConfig(pInfo, &pDspRegisters->dataPathConfig[0],
                                                         pChanIdx, numChannel, adcIdx);
                if (adcStatus == ADI_ADC_STATUS_SUCCESS)
                {
                    adcStatus = adi_adc_SetChannelPhaseOffset(pInfo, &pDspRegisters->phaseOffset[0],
                                                              pChanIdx, numChannel, adcIdx);
                }
            }
        }
    }
    return adcStatus;
}

ADI_ADC_STATUS adi_adc_GetDatapathParams(ADI_ADC_HANDLE hAdc, uint8_t *pChanIdx, uint8_t numChannel,
                                         int8_t adcIdx, ADI_ADC_DSP_DATAPATH_PARAMS *pDspRegisters)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    ADI_ADC_INFO *pInfo = (ADI_ADC_INFO *)hAdc;
    uint8_t decimationX2;
    uint8_t clkPreScalar;
    uint8_t decimationRate;
    if (hAdc == NULL)
    {
        adcStatus = ADI_ADC_STATUS_NULL_PTR;
    }
    else
    {

        adcStatus = adi_adc_GetDatapathConfig(pInfo, pChanIdx, numChannel, adcIdx,
                                              &pDspRegisters->dataPathConfig[0]);
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus =
                adi_adc_GetDataRate(pInfo, adcIdx, &decimationRate, &clkPreScalar, &decimationX2);
            if (adcStatus == ADI_ADC_STATUS_SUCCESS)
            {
                adcStatus =
                    adi_adc_GetDatapathAlpha(pInfo, adcIdx, (uint8_t *)&pDspRegisters->alpha);
                if (adcStatus == ADI_ADC_STATUS_SUCCESS)
                {
                    adcStatus = adi_adc_GetChannelPhaseOffset(pInfo, pChanIdx, numChannel, adcIdx,
                                                              &pDspRegisters->phaseOffset[0]);
                }
            }
        }
    }
    return adcStatus;
}

ADI_ADC_STATUS adi_adc_SetChannelXtGain(ADI_ADC_HANDLE hAdc, float *pXtGainVal, uint8_t *pChanIdx,
                                        int8_t numChan, int8_t adcIdx)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    ADI_ADC_INFO *pInfo;

    if (hAdc == NULL)
    {
        adcStatus = ADI_ADC_STATUS_NULL_PTR;
    }
    else
    {
        pInfo = (ADI_ADC_INFO *)hAdc;
        adcStatus = CheckAdcTypeValid(pInfo, adcIdx);
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = CheckChannelValid(pInfo, adcIdx, pChanIdx, numChan);
        }
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = AdcSetChannelXtGain(pInfo, pXtGainVal, pChanIdx, numChan, adcIdx);
        }
    }

    return adcStatus;
}

ADI_ADC_STATUS adi_adc_SetChannelXtAggressor(ADI_ADC_HANDLE hAdc,
                                             ADI_ADC_CHAN_XT_AGGRESSOR *pAggrVal, uint8_t *pChanIdx,
                                             int8_t numChan, int8_t adcIdx)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    ADI_ADC_INFO *pInfo;

    if (hAdc == NULL)
    {
        adcStatus = ADI_ADC_STATUS_NULL_PTR;
    }
    else
    {
        pInfo = (ADI_ADC_INFO *)hAdc;
        adcStatus = CheckAdcTypeValid(pInfo, adcIdx);
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = CheckChannelValid(pInfo, adcIdx, pChanIdx, numChan);
        }
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = AdcSetChannelXtAggressor(pInfo, pAggrVal, pChanIdx, numChan, adcIdx);
        }
    }

    return adcStatus;
}

ADI_ADC_STATUS adi_adc_SetChannelGain(ADI_ADC_HANDLE hAdc, float *pGainVal, uint8_t *pChanIdx,
                                      int8_t numChan, int8_t adcIdx)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    ADI_ADC_INFO *pInfo;

    if (hAdc == NULL)
    {
        adcStatus = ADI_ADC_STATUS_NULL_PTR;
    }
    else
    {
        pInfo = (ADI_ADC_INFO *)hAdc;
        adcStatus = CheckAdcTypeValid(pInfo, adcIdx);
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = CheckChannelValid(pInfo, adcIdx, pChanIdx, numChan);
        }
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = AdcSetChannelGain(pInfo, pGainVal, pChanIdx, numChan, adcIdx);
        }
    }

    return adcStatus;
}

ADI_ADC_STATUS adi_adc_SetChannelOffset(ADI_ADC_HANDLE hAdc, int32_t *pOffsetVal, uint8_t *pChanIdx,
                                        int8_t numChan, int8_t adcIdx)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    ADI_ADC_INFO *pInfo;

    if (hAdc == NULL)
    {
        adcStatus = ADI_ADC_STATUS_NULL_PTR;
    }
    else
    {
        pInfo = (ADI_ADC_INFO *)hAdc;
        adcStatus = CheckAdcTypeValid(pInfo, adcIdx);
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = CheckChannelValid(pInfo, adcIdx, pChanIdx, numChan);
        }
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = AdcSetChannelOffset(pInfo, pOffsetVal, pChanIdx, numChan, adcIdx);
        }
    }

    return adcStatus;
}

ADI_ADC_STATUS adi_adc_SetChannelPhaseOffset(ADI_ADC_HANDLE hAdc, float *pPhOffsetVal,
                                             uint8_t *pChanIdx, int8_t numChan, int8_t adcIdx)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    ADI_ADC_INFO *pInfo;

    if (hAdc == NULL)
    {
        adcStatus = ADI_ADC_STATUS_NULL_PTR;
    }
    else
    {
        pInfo = (ADI_ADC_INFO *)hAdc;
        adcStatus = CheckAdcTypeValid(pInfo, adcIdx);
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = CheckChannelValid(pInfo, adcIdx, pChanIdx, numChan);
        }
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = AdcSetPhaseOffset(pInfo, pPhOffsetVal, pChanIdx, numChan, adcIdx);
        }
    }

    return adcStatus;
}

ADI_ADC_STATUS adi_adc_SetDataRate(ADI_ADC_HANDLE hAdc, uint8_t decimationRate,
                                   uint8_t clkPreScaler, uint8_t decimateX2, int8_t adcIdx)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    ADI_ADC_INFO *pInfo;

    if (hAdc == NULL)
    {
        adcStatus = ADI_ADC_STATUS_NULL_PTR;
    }
    else
    {
        pInfo = (ADI_ADC_INFO *)hAdc;
        adcStatus = CheckAdcTypeValid(pInfo, adcIdx);
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = AdcSetDataRate(pInfo, decimationRate, clkPreScaler, decimateX2, adcIdx);
        }
    }

    return adcStatus;
}

ADI_ADC_STATUS adi_adc_GetDataRate(ADI_ADC_HANDLE hAdc, int8_t adcIdx, uint8_t *pDecimationRate,
                                   uint8_t *pClkPreScaler, uint8_t *pDecimateX2)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    ADI_ADC_INFO *pInfo;

    if (hAdc == NULL)
    {
        adcStatus = ADI_ADC_STATUS_NULL_PTR;
    }
    else
    {
        pInfo = (ADI_ADC_INFO *)hAdc;
        adcStatus = CheckAdcTypeValid(pInfo, adcIdx);
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = AdcGetDataRate(pInfo, adcIdx, pDecimationRate, pClkPreScaler, pDecimateX2);
        }
    }

    return adcStatus;
}

ADI_ADC_STATUS adi_adc_SetDatapathAlpha(ADI_ADC_HANDLE hAdc, uint8_t *pAlphaVal, int8_t adcIdx)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    ADI_ADC_INFO *pInfo;

    if (hAdc == NULL)
    {
        adcStatus = ADI_ADC_STATUS_NULL_PTR;
    }
    else
    {
        pInfo = (ADI_ADC_INFO *)hAdc;
        adcStatus = CheckAdcTypeValid(pInfo, adcIdx);
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = AdcSetDatapathAlpha(pInfo, pAlphaVal, adcIdx);
        }
    }

    return adcStatus;
}

ADI_ADC_STATUS adi_adc_SetLpfCoeff(ADI_ADC_HANDLE hAdc, float *pCoeffs, int8_t adcIdx)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    ADI_ADC_INFO *pInfo;

    if (hAdc == NULL)
    {
        adcStatus = ADI_ADC_STATUS_NULL_PTR;
    }
    else
    {
        pInfo = (ADI_ADC_INFO *)hAdc;
        adcStatus = CheckAdcTypeValid(pInfo, adcIdx);
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = AdcSetLpfCoeff(pInfo, pCoeffs, adcIdx);
        }
    }

    return adcStatus;
}

ADI_ADC_STATUS adi_adc_SetScfCoeff(ADI_ADC_HANDLE hAdc, float *pCoeffs[], uint8_t *pChanIdx,
                                   int8_t numChan, int8_t adcIdx)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    ADI_ADC_INFO *pInfo;

    if (hAdc == NULL)
    {
        adcStatus = ADI_ADC_STATUS_NULL_PTR;
    }
    else
    {
        pInfo = (ADI_ADC_INFO *)hAdc;
        adcStatus = CheckAdcTypeValid(pInfo, adcIdx);
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = CheckChannelValid(pInfo, adcIdx, pChanIdx, numChan);
        }
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = AdcSetScfCoeff(pInfo, pCoeffs, pChanIdx, numChan, adcIdx);
        }
    }

    return adcStatus;
}

ADI_ADC_STATUS adi_adc_SetCompCoeff(ADI_ADC_HANDLE hAdc, float *pCoeffs[], uint8_t *pChanIdx,
                                    int8_t numChan, int8_t adcIdx)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    ADI_ADC_INFO *pInfo;

    if (hAdc == NULL)
    {
        adcStatus = ADI_ADC_STATUS_NULL_PTR;
    }
    else
    {
        pInfo = (ADI_ADC_INFO *)hAdc;
        adcStatus = CheckAdcTypeValid(pInfo, adcIdx);
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = CheckChannelValid(pInfo, adcIdx, pChanIdx, numChan);
        }
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = AdcSetCompCoeff(pInfo, pCoeffs, pChanIdx, numChan, adcIdx);
        }
    }

    return adcStatus;
}

ADI_ADC_STATUS adi_adc_SetHpfCoeff(ADI_ADC_HANDLE hAdc, float *pNumCoeffs, double *pDenCoeffs,
                                   int8_t adcIdx)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    ADI_ADC_INFO *pInfo;

    if (hAdc == NULL)
    {
        adcStatus = ADI_ADC_STATUS_NULL_PTR;
    }
    else
    {
        pInfo = (ADI_ADC_INFO *)hAdc;
        adcStatus = CheckAdcTypeValid(pInfo, adcIdx);
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = AdcSetHpfCoeff(pInfo, pNumCoeffs, pDenCoeffs, adcIdx);
        }
    }

    return adcStatus;
}

ADI_ADC_STATUS adi_adc_SetShift(ADI_ADC_HANDLE hAdc, uint8_t *pShiftVal, uint8_t *pChanIdx,
                                int8_t numChan, int8_t adcIdx)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    ADI_ADC_INFO *pInfo;
    int8_t chan = 0;

    if (hAdc == NULL)
    {
        adcStatus = ADI_ADC_STATUS_NULL_PTR;
    }
    else
    {
        pInfo = (ADI_ADC_INFO *)hAdc;
        adcStatus = CheckAdcTypeValid(pInfo, adcIdx);
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = CheckChannelValid(pInfo, adcIdx, pChanIdx, numChan);
        }
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = AdcSetShift(pInfo, pShiftVal, pChanIdx, numChan, adcIdx);
            for (chan = 0; chan < numChan; chan++)
            {
                pInfo->datapathShift[adcIdx][pChanIdx[chan]] = pShiftVal[chan];
            }
        }
    }

    return adcStatus;
}

ADI_ADC_STATUS adi_adc_SetHpfCutoff(ADI_ADC_HANDLE hAdc, uint32_t bwOption, int8_t adcIdx)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    ADI_ADC_INFO *pInfo;
    uint32_t lutIdx;
    uint8_t decimationRate[2] = {0, 0};
    uint32_t numBytes;

    if (hAdc == NULL)
    {
        adcStatus = ADI_ADC_STATUS_NULL_PTR;
    }
    else
    {
        pInfo = (ADI_ADC_INFO *)hAdc;
        adcStatus = CheckAdcTypeValid(pInfo, adcIdx);
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            // Read datarate config register
            adcStatus = AdcReadRegister(pInfo, ADDR_ADEMA127_MMR_DATARATE, adcIdx, decimationRate,
                                        &numBytes);
            decimationRate[1] = decimationRate[1] & BITM_ADEMA127_MMR_DATARATE_DECIMATION_RATE;
            lutIdx = 4 + decimationRate[1] - bwOption;
            if (lutIdx < 14)
            {
                adcStatus = AdcSetHpfCoeffFix(pInfo, hpfCoeff[lutIdx], adcIdx);
            }
            else
            {
                adcStatus = ADI_ADC_STATUS_DATAPATH_CONFIGURATION_FAILED;
            }
        }
    }

    return adcStatus;
}

ADI_ADC_STATUS adi_adc_GetChannelGain(ADI_ADC_HANDLE hAdc, uint8_t *pChanIdx, int8_t numChan,
                                      int8_t adcIdx, float *pGainVal)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    ADI_ADC_INFO *pInfo = (ADI_ADC_INFO *)hAdc;
    if (hAdc == NULL)
    {
        adcStatus = ADI_ADC_STATUS_NULL_PTR;
    }
    else
    {
        adcStatus = CheckAdcTypeValid(pInfo, adcIdx);
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = CheckChannelValid(pInfo, adcIdx, pChanIdx, numChan);
        }
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = AdcGetChannelGain(pInfo, pChanIdx, numChan, adcIdx, pGainVal);
        }
    }
    return adcStatus;
}

ADI_ADC_STATUS adi_adc_GetChannelXtGain(ADI_ADC_HANDLE hAdc, uint8_t *pChanIdx, int8_t numChan,
                                        int8_t adcIdx, float *pXtGainVal)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    ADI_ADC_INFO *pInfo = (ADI_ADC_INFO *)hAdc;
    if (hAdc == NULL)
    {
        adcStatus = ADI_ADC_STATUS_NULL_PTR;
    }
    else
    {
        adcStatus = CheckAdcTypeValid(pInfo, adcIdx);
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = CheckChannelValid(pInfo, adcIdx, pChanIdx, numChan);
        }
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = AdcGetChannelXtGain(pInfo, pChanIdx, numChan, adcIdx, pXtGainVal);
        }
    }
    return adcStatus;
}

ADI_ADC_STATUS adi_adc_GetChannelXtAggressor(ADI_ADC_HANDLE hAdc, uint8_t *pChanIdx, int8_t numChan,
                                             int8_t adcIdx, ADI_ADC_CHAN_XT_AGGRESSOR *pXtAggrVal)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    ADI_ADC_INFO *pInfo = (ADI_ADC_INFO *)hAdc;
    if (hAdc == NULL)
    {
        adcStatus = ADI_ADC_STATUS_NULL_PTR;
    }
    else
    {
        adcStatus = CheckAdcTypeValid(pInfo, adcIdx);
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = CheckChannelValid(pInfo, adcIdx, pChanIdx, numChan);
        }
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = AdcGetChannelXtAggressor(pInfo, pChanIdx, numChan, adcIdx, pXtAggrVal);
        }
    }
    return adcStatus;
}

ADI_ADC_STATUS adi_adc_GetChannelOffset(ADI_ADC_HANDLE hAdc, uint8_t *pChanIdx, int8_t numChan,
                                        int8_t adcIdx, int32_t *pOffsetVal)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    ADI_ADC_INFO *pInfo = (ADI_ADC_INFO *)hAdc;

    if (hAdc == NULL)
    {
        adcStatus = ADI_ADC_STATUS_NULL_PTR;
    }
    else
    {
        adcStatus = CheckAdcTypeValid(pInfo, adcIdx);
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = AdcGetChannelOffset(pInfo, pChanIdx, numChan, adcIdx, pOffsetVal);
        }
    }

    return adcStatus;
}

ADI_ADC_STATUS adi_adc_GetChannelPhaseOffset(ADI_ADC_HANDLE hAdc, uint8_t *pChanIdx, int8_t numChan,
                                             int8_t adcIdx, float *pPhOffsetVal)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    ADI_ADC_INFO *pInfo = (ADI_ADC_INFO *)hAdc;

    if (hAdc == NULL)
    {
        adcStatus = ADI_ADC_STATUS_NULL_PTR;
    }
    else
    {
        adcStatus = CheckAdcTypeValid(pInfo, adcIdx);
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = CheckChannelValid(pInfo, adcIdx, pChanIdx, numChan);
        }
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = AdcGetPhaseOffset(pInfo, pChanIdx, numChan, adcIdx, pPhOffsetVal);
        }
    }

    return adcStatus;
}

ADI_ADC_STATUS adi_adc_GetDatapathAlpha(ADI_ADC_HANDLE hAdc, int8_t adcIdx, uint8_t *pAlphaVal)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    ADI_ADC_INFO *pInfo = (ADI_ADC_INFO *)hAdc;

    if (hAdc == NULL)
    {
        adcStatus = ADI_ADC_STATUS_NULL_PTR;
    }
    else
    {
        adcStatus = CheckAdcTypeValid(pInfo, adcIdx);
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = AdcGetDatapathAlpha(pInfo, adcIdx, pAlphaVal);
        }
    }

    return adcStatus;
}

ADI_ADC_STATUS adi_adc_GetDatapathConfig(ADI_ADC_HANDLE hAdc, uint8_t *pChanIdx, int8_t numChan,
                                         int8_t adcIdx,
                                         ADI_ADC_CHAN_DATAPATH_CONFIG *pDatapathConfig)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    ADI_ADC_INFO *pInfo = (ADI_ADC_INFO *)hAdc;

    if (hAdc == NULL)
    {
        adcStatus = ADI_ADC_STATUS_NULL_PTR;
    }
    else
    {
        adcStatus = CheckAdcTypeValid(pInfo, adcIdx);
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = CheckChannelValid(pInfo, adcIdx, pChanIdx, numChan);
        }
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = AdcGetDataPathEnable(pInfo, pChanIdx, numChan, adcIdx, pDatapathConfig);
        }
    }

    return adcStatus;
}

ADI_ADC_STATUS adi_adc_GetShift(ADI_ADC_HANDLE hAdc, uint8_t *pChanIdx, int8_t numChan,
                                int8_t adcIdx, uint8_t *pShiftVal)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    ADI_ADC_INFO *pInfo;

    if (hAdc == NULL)
    {
        adcStatus = ADI_ADC_STATUS_NULL_PTR;
    }
    else
    {
        pInfo = (ADI_ADC_INFO *)hAdc;
        adcStatus = CheckAdcTypeValid(pInfo, adcIdx);
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = CheckChannelValid(pInfo, adcIdx, pChanIdx, numChan);
        }
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = AdcGetShift(pInfo, pChanIdx, numChan, adcIdx, pShiftVal);
        }
    }

    return adcStatus;
}

ADI_ADC_STATUS adi_adc_GetHpfCoeff(ADI_ADC_HANDLE hAdc, int8_t adcIdx, float *pNumCoeffs,
                                   double *pDenCoeffs)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    ADI_ADC_INFO *pInfo;

    if (hAdc == NULL)
    {
        adcStatus = ADI_ADC_STATUS_NULL_PTR;
    }
    else
    {
        pInfo = (ADI_ADC_INFO *)hAdc;
        adcStatus = CheckAdcTypeValid(pInfo, adcIdx);
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = AdcGetHpfCoeff(pInfo, adcIdx, pNumCoeffs, pDenCoeffs);
        }
    }

    return adcStatus;
}

ADI_ADC_STATUS adi_adc_GetScfCoeff(ADI_ADC_HANDLE hAdc, uint8_t *pChanIdx, uint8_t numChan,
                                   int8_t adcIdx, float *pCoeffs[])
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    ADI_ADC_INFO *pInfo;

    if (hAdc == NULL)
    {
        adcStatus = ADI_ADC_STATUS_NULL_PTR;
    }
    else
    {
        pInfo = (ADI_ADC_INFO *)hAdc;
        adcStatus = CheckAdcTypeValid(pInfo, adcIdx);
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = CheckChannelValid(pInfo, adcIdx, pChanIdx, numChan);
        }
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = AdcGetScfCoeff(pInfo, pChanIdx, numChan, adcIdx, pCoeffs);
        }
    }

    return adcStatus;
}

static ADI_ADC_STATUS CheckAdcTypeValid(ADI_ADC_INFO *pInfo, int8_t adcIdx)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;

    if (pInfo->adcCfg.numAdc > 0)
    {
        if (adcIdx == -1)
        {
            for (int i = 0; i < pInfo->adcCfg.numAdc; i++)
            {
                if (pInfo->adcCfg.adcType[i] == ADI_ADC_TYPE_ADE91XX)
                {
                    status = ADI_ADC_STATUS_INVALID_ADC_TYPE;
                    break;
                }
            }
        }
        else if (adcIdx >= 0 && adcIdx < pInfo->adcCfg.numAdc)
        {
            if (pInfo->adcCfg.adcType[adcIdx] == ADI_ADC_TYPE_ADE91XX)
            {
                status = ADI_ADC_STATUS_INVALID_ADC_TYPE;
            }
        }
    }
    else
    {
        status = ADI_ADC_STATUS_INVALID_NUM_ADC;
    }

    return status;
}

static void StoreDatapathScfEnable(ADI_ADC_INFO *pInfo,
                                   ADI_ADC_CHAN_DATAPATH_CONFIG *pDatapathEnConfig,
                                   uint8_t *pChanIdx, int8_t numChan, int8_t adcIdx)
{
    int8_t i, chan;

    if (adcIdx == -1)
    {
        for (i = 0; i < pInfo->adcCfg.numAdc; i++)
        {
            for (chan = 0; chan < numChan; chan++)
            {
                pInfo->datapathScfEn[i][pChanIdx[chan]] = pDatapathEnConfig[pChanIdx[chan]].scfEn;
            }
        }
    }
    else
    {
        for (chan = 0; chan < numChan; chan++)
        {
            pInfo->datapathScfEn[adcIdx][pChanIdx[chan]] = pDatapathEnConfig[pChanIdx[chan]].scfEn;
        }
    }
}

ADI_ADC_STATUS adi_adc_GetLpfCoeff(ADI_ADC_HANDLE hAdc, int8_t adcIdx, float *pCoeffs)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    ADI_ADC_INFO *pInfo;

    if (hAdc == NULL)
    {
        adcStatus = ADI_ADC_STATUS_NULL_PTR;
    }
    else
    {
        pInfo = (ADI_ADC_INFO *)hAdc;
        adcStatus = CheckAdcTypeValid(pInfo, adcIdx);
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = AdcGetLpfCoeff(pInfo, adcIdx, pCoeffs);
        }
    }

    return adcStatus;
}

ADI_ADC_STATUS adi_adc_GetCompCoeff(ADI_ADC_HANDLE hAdc, uint8_t *pChanIdx, uint8_t numChan,
                                    int8_t adcIdx, float *pCoeffs[])
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    ADI_ADC_INFO *pInfo;

    if (hAdc == NULL)
    {
        adcStatus = ADI_ADC_STATUS_NULL_PTR;
    }
    else
    {
        pInfo = (ADI_ADC_INFO *)hAdc;
        adcStatus = CheckAdcTypeValid(pInfo, adcIdx);
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = CheckChannelValid(pInfo, adcIdx, pChanIdx, numChan);
        }
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = AdcGetCompCoeff(pInfo, pChanIdx, numChan, adcIdx, pCoeffs);
        }
    }

    return adcStatus;
}

/**
 * @}
 */
