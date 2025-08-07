/******************************************************************************
 Copyright (c) 2025  Analog Devices Inc.
******************************************************************************/

/**
 * @file        adc_service_dsp_interface.c
 * @brief       Interface file demonstrating use of ADC service DSP APIs.
 * @{
 */

/*============= I N C L U D E S =============*/
#include "adc_service_dsp_interface.h"
#include "ADE911X_addr_def.h"
#include "ADE911X_addr_rdef.h"
#include "ADEMA127_addr_def.h"
#include "ADEMA127_addr_rdef.h"
#include "adc_datapath_cfg.h"
#include "adi_evb.h"
#include <stdint.h>
#include <string.h>

/*============= F U N C T I O N S =============*/

ADI_ADC_STATUS AdcIfDatapathSetVal(ADC_INTERFACE_INFO *pInfo, ADI_ADC_CONFIG *pConfig)
{

    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    int32_t i = 0;
    uint8_t channelNum;
    uint8_t adcIdx;
    uint8_t adcDatapathConfig;
    uint8_t numAdc = pConfig->numAdc;

    ADI_ADC_DSP_DATAPATH_PARAMS *pDatapathParams;
    ADI_ADC_DSP_CHANNEL_PARAMS *pChannelParams;

    for (i = 0; i < numAdc; i++)
    {
        pDatapathParams = &pInfo->adcRegParams[i].adcDatapathParams;
        pChannelParams = &pInfo->adcRegParams[i].adcChannelParams;
        pInfo->regCmiInvVal[i] = 0;

        memset(&pDatapathParams->alpha, 0, sizeof(pDatapathParams->alpha));
        memset(&pDatapathParams->pDataPathConfig[0], 0,
               sizeof(ADI_ADC_CHAN_DATAPATH_CONFIG) * APP_CFG_MAX_NUM_CHANNELS);
        memset(&pDatapathParams->datarate, 0, sizeof(pDatapathParams->datarate));
        memset(&pDatapathParams->pPhaseOffset[0], 0, sizeof(float) * APP_CFG_MAX_NUM_CHANNELS);
        memset(pChannelParams->pOffset, 0, sizeof(int32_t) * APP_CFG_MAX_NUM_CHANNELS);
        memset(pChannelParams->pXtGain, 0, sizeof(float) * APP_CFG_MAX_NUM_CHANNELS);
        memset(pChannelParams->pGain, 0, sizeof(float) * APP_CFG_MAX_NUM_CHANNELS);

        memset(pChannelParams->pXtAggressor, 0,
               sizeof(ADI_ADC_CHAN_XT_AGGRESSOR) * APP_CFG_MAX_NUM_CHANNELS);

        memset(pChannelParams->pShift, 0, sizeof(uint8_t) * APP_CFG_MAX_NUM_CHANNELS);

        pDatapathParams->datarate.decimationRate = APP_CFG_ADC_DECIMATION_RATE;
        pDatapathParams->datarate.clkPreScalar = APP_CFG_ADC_PRESCALER;
        pDatapathParams->datarate.decimationX2 = APP_CFG_ADC_DECIMATION_BY2;

        pInfo->adcRegParams[i].gain = APP_CFG_ADC_GAIN;
    }

    for (i = 0; i < APP_CFG_MAX_NUM_VOLTAGE_CHANNELS; i++)
    {
        adi_adc_GetAdcIdxAndChan(pInfo->hAdc, pInfo->adcBoardConfig.voltageSlots[i], &adcIdx,
                                 &channelNum);
        pDatapathParams = &pInfo->adcRegParams[adcIdx].adcDatapathParams;
        pChannelParams = &pInfo->adcRegParams[adcIdx].adcChannelParams;

        adcDatapathConfig = APP_CFG_ADC_DATAPATH_CONFIG_VOLTAGE;
        memcpy(&pDatapathParams->pDataPathConfig[channelNum], &adcDatapathConfig,
               sizeof(ADI_ADC_CHAN_DATAPATH_CONFIG));

        pDatapathParams->pPhaseOffset[channelNum] = APP_CFG_ADC_DATAPATH_PHASE_VOLTAGE;

        pChannelParams->pGain[channelNum] = APP_CFG_ADC_DATAPATH_GAIN_VOLTAGE;
        pChannelParams->pOffset[channelNum] = APP_CFG_ADC_DATAPATH_OFFSET_VOLTAGE;
        pChannelParams->pShift[channelNum] = APP_CFG_ADC_DATAPATH_SHIFT_VOLTAGE;
    }

    for (i = 0; i < APP_CFG_MAX_NUM_CURRENT_CHANNELS; i++)
    {
        adi_adc_GetAdcIdxAndChan(pInfo->hAdc, pInfo->adcBoardConfig.currentSlots[i], &adcIdx,
                                 &channelNum);
        pDatapathParams = &pInfo->adcRegParams[adcIdx].adcDatapathParams;
        pChannelParams = &pInfo->adcRegParams[adcIdx].adcChannelParams;
        pInfo->regCmiInvVal[adcIdx] |= (1 << channelNum);

        adcDatapathConfig = APP_CFG_ADC_DATAPATH_CONFIG_CURRENT;
        memcpy(&pDatapathParams->pDataPathConfig[channelNum], &adcDatapathConfig,
               sizeof(ADI_ADC_CHAN_DATAPATH_CONFIG));

        pDatapathParams->pPhaseOffset[channelNum] = APP_CFG_ADC_DATAPATH_PHASE_CURRENT;

        pChannelParams->pGain[channelNum] = APP_CFG_ADC_DATAPATH_GAIN_CURRENT;
        pChannelParams->pOffset[channelNum] = APP_CFG_ADC_DATAPATH_OFFSET_CURRENT;
        pChannelParams->pShift[channelNum] = APP_CFG_ADC_DATAPATH_SHIFT_CURRENT;
    }

    for (i = 0; i < numAdc; i++)
    {
        pInfo->adcRegParams[i].cmiReg = pInfo->regCmiInvVal[i];
#ifdef APP_CFG_ADC_INVERT_CURRENT_CHANNELS
        pInfo->adcRegParams[i].invReg = APP_CFG_ADC_INVERT_CURRENT_CHANNELS;
#else
        pInfo->adcRegParams[i].invReg = pInfo->regCmiInvVal[i];
#endif
    }

    return status;
}

ADI_ADC_STATUS AdcIfAccessDspMem(ADC_INTERFACE_INFO *pInfo, uint8_t value, int8_t adcIdx)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    int32_t attempts = 0;
    uint8_t status2Reg[2];
    uint32_t numBytesRead = 0;

    if (value == 1)
    {
        AdcIfStopCapture(pInfo);

        // Request DSP memory access - Set UDSP_MEM_ACCESS_REQ = 1
        status = AdcIfWriteRegister(pInfo, ADDR_ADEMA127_MMR_ACCESS_EXTENDED_MMAP, 1, adcIdx);

        if (status == ADI_ADC_STATUS_SUCCESS)
        {
            while (attempts < 5)
            {
                EvbDelayMs(1);
                status = AdcIfReadRegister(pInfo, ADDR_ADEMA127_MMR_STATUS2, adcIdx, status2Reg,
                                           &numBytesRead);
                if (status2Reg[1] & BITM_ADEMA127_MMR_STATUS2_DSP_MEM_ACCESS_READY)
                {
                    break;
                }
                if (status != ADI_ADC_STATUS_SUCCESS)
                {
                    break;
                }
                attempts++;
            }
            if (attempts >= 5)
            {
                status = ADI_ADC_STATUS_DATAPATH_CONFIGURATION_FAILED;
            }
        }
    }
    else
    {
        // Release DSP memory access - Set UDSP_MEM_ACCESS_REQ = 0
        status = AdcIfWriteRegister(pInfo, ADDR_ADEMA127_MMR_ACCESS_EXTENDED_MMAP, 0, adcIdx);
    }

    return status;
}

ADI_ADC_STATUS AdcIfConfigDspLock(ADC_INTERFACE_INFO *pInfo, uint8_t value, int8_t adcIdx)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;

    if (value == 0)
    {
        // Save DSP RAM region before datapath config is unlocked
        status = AdcIfGetDspRegisterStruct(pInfo, pInfo->adcCfg.numAdc, pInfo->adcCfg.pAdcType);
        AdcIfStopCapture(pInfo);
        if (status == ADI_ADC_STATUS_SUCCESS)
        {
            // Clear Datapath Config Lock
            status = AdcIfWriteRegister(pInfo, ADDR_ADEMA127_MMR_DATAPATH_CONFIG_LOCK, 0, adcIdx);
        }
    }
    else
    {

        // Clear Datapath Config Lock
        if (status == ADI_ADC_STATUS_SUCCESS)
        {
            status = AdcIfWriteRegister(pInfo, ADDR_ADEMA127_MMR_DATAPATH_CONFIG_LOCK, 1, adcIdx);
            pInfo->dreadyFlag = 0;
            EvbDelayMs(1);
            if (status == ADI_ADC_STATUS_SUCCESS)
            {
                // Reload DSP RAM region after datapath config is locked
                status = AdcIfPopulateDspRegisterStruct(pInfo, pInfo->adcCfg.numAdc,
                                                        pInfo->adcCfg.pAdcType);
            }
            // Wait for DREADY signal to resume
            while (pInfo->dreadyFlag == 0)
            {
                ;
            }
            if (status == ADI_ADC_STATUS_SUCCESS)
            {
                /* adcs become out of sync after writing to dsp lock registers as the dready stops
                   intermittently */
                status = adi_adc_Align(pInfo->hAdc);
            }
        }
    }

    return status;
}

ADI_ADC_STATUS AdcIfEnableDatapath(ADC_INTERFACE_INFO *pInfo,
                                   ADI_ADC_CHAN_DATAPATH_CONFIG *pDatapathEn, uint8_t *pChanIdx,
                                   int8_t numChan, int8_t adcIdx)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;

    status = AdcIfConfigDspLock(pInfo, 0, adcIdx);
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = adi_adc_EnableDatapathConfig(pInfo->hAdc, pDatapathEn, pChanIdx, numChan, adcIdx);
    }
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = AdcIfConfigDspLock(pInfo, 1, adcIdx);
    }

    return status;
}

ADI_ADC_STATUS AdcIfGetDatapathConfig(ADC_INTERFACE_INFO *pInfo, uint8_t *pChanIdx, int8_t numChan,
                                      int8_t adcIdx, ADI_ADC_CHAN_DATAPATH_CONFIG *pDatapathCfg)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    AdcIfStopCapture(pInfo);
    status = adi_adc_GetDatapathConfig(pInfo->hAdc, pChanIdx, numChan, adcIdx, pDatapathCfg);
    return status;
}

ADI_ADC_STATUS AdcIfGetDatarate(ADC_INTERFACE_INFO *pInfo, int8_t adcIdx, uint8_t *pDecimationRate,
                                uint8_t *pClkPreScaler, uint8_t *pSetDecimation)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    AdcIfStopCapture(pInfo);
    status =
        adi_adc_GetDataRate(pInfo->hAdc, adcIdx, pDecimationRate, pClkPreScaler, pSetDecimation);
    return status;
}

ADI_ADC_STATUS AdcIfSetGain(ADC_INTERFACE_INFO *pInfo, float *pGainVal, uint8_t *pChanIdx,
                            int8_t numChan, int8_t adcIdx)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;

    status = AdcIfAccessDspMem(pInfo, 1, adcIdx);
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = adi_adc_SetChannelGain(pInfo->hAdc, pGainVal, pChanIdx, numChan, adcIdx);
    }
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = AdcIfAccessDspMem(pInfo, 0, adcIdx);
    }

    return status;
}

ADI_ADC_STATUS AdcIfSetXtGain(ADC_INTERFACE_INFO *pInfo, float *pXtGainVal, uint8_t *pChanIdx,
                              int8_t numChan, int8_t adcIdx)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;

    status = AdcIfAccessDspMem(pInfo, 1, adcIdx);
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = adi_adc_SetChannelXtGain(pInfo->hAdc, pXtGainVal, pChanIdx, numChan, adcIdx);
    }
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = AdcIfAccessDspMem(pInfo, 0, adcIdx);
    }

    return status;
}

ADI_ADC_STATUS AdcIfSetXtAggressor(ADC_INTERFACE_INFO *pInfo, ADI_ADC_CHAN_XT_AGGRESSOR *pXtAggrVal,
                                   uint8_t *pChanIdx, int8_t numChan, int8_t adcIdx)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;

    status = AdcIfAccessDspMem(pInfo, 1, adcIdx);
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = adi_adc_SetChannelXtAggressor(pInfo->hAdc, pXtAggrVal, pChanIdx, numChan, adcIdx);
    }
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = AdcIfAccessDspMem(pInfo, 0, adcIdx);
    }

    return status;
}

ADI_ADC_STATUS AdcIfSetOffset(ADC_INTERFACE_INFO *pInfo, int32_t *pOffsetVal, uint8_t *pChanIdx,
                              int8_t numChan, int8_t adcIdx)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;

    status = AdcIfAccessDspMem(pInfo, 1, adcIdx);
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = adi_adc_SetChannelOffset(pInfo->hAdc, pOffsetVal, pChanIdx, numChan, adcIdx);
    }
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = AdcIfAccessDspMem(pInfo, 0, adcIdx);
    }

    return status;
}

ADI_ADC_STATUS AdcIfSetDatapathAlpha(ADC_INTERFACE_INFO *pInfo, uint8_t *pAlphaVal, int8_t adcIdx)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;

    status = AdcIfConfigDspLock(pInfo, 0, adcIdx);
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = adi_adc_SetDatapathAlpha(pInfo->hAdc, pAlphaVal, adcIdx);
    }
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = AdcIfConfigDspLock(pInfo, 1, adcIdx);
    }

    return status;
}

ADI_ADC_STATUS AdcIfSetPhaseOffset(ADC_INTERFACE_INFO *pInfo, float *pPhOffsetVal,
                                   uint8_t *pChanIdx, int8_t numChan, int8_t adcIdx)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = AdcIfConfigDspLock(pInfo, 0, adcIdx);
    }
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status =
            adi_adc_SetChannelPhaseOffset(pInfo->hAdc, pPhOffsetVal, pChanIdx, numChan, adcIdx);
    }
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = AdcIfConfigDspLock(pInfo, 1, adcIdx);
    }

    return status;
}

ADI_ADC_STATUS AdcIfSetDatarate(ADC_INTERFACE_INFO *pInfo, uint8_t decimationRate,
                                uint8_t clkPreScalar, uint8_t decimationByX2, int8_t adcIdx)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    status = AdcIfConfigDspLock(pInfo, 0, adcIdx);
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status =
            adi_adc_SetDataRate(pInfo->hAdc, decimationRate, clkPreScalar, decimationByX2, adcIdx);
    }
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = AdcIfConfigDspLock(pInfo, 1, adcIdx);
    }
    return status;
}

ADI_ADC_STATUS AdcIfSetShift(ADC_INTERFACE_INFO *pInfo, uint8_t *pVal, uint8_t *pChanIdx,
                             int8_t numChan, int8_t adcIdx)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;

    status = AdcIfAccessDspMem(pInfo, 1, adcIdx);
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = adi_adc_SetShift(pInfo->hAdc, pVal, pChanIdx, numChan, adcIdx);
    }
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = AdcIfAccessDspMem(pInfo, 0, adcIdx);
    }

    return status;
}

ADI_ADC_STATUS AdcIfSetScfCoeff(ADC_INTERFACE_INFO *pInfo, float *pCoeffs[], uint8_t *pChanIdx,
                                uint8_t numChan, int8_t adcIdx)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    status = AdcIfAccessDspMem(pInfo, 1, adcIdx);
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = adi_adc_SetScfCoeff(pInfo->hAdc, pCoeffs, pChanIdx, numChan, adcIdx);
    }
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = AdcIfAccessDspMem(pInfo, 0, adcIdx);
    }
    return status;
}

ADI_ADC_STATUS AdcIfSetHpfCoeff(ADC_INTERFACE_INFO *pInfo, float *pNumCoeffs, double *pDenCoeffs,
                                int8_t adcIdx)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    status = AdcIfAccessDspMem(pInfo, 1, adcIdx);
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = adi_adc_SetHpfCoeff(pInfo->hAdc, pNumCoeffs, pDenCoeffs, adcIdx);
    }
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = AdcIfAccessDspMem(pInfo, 0, adcIdx);
    }
    return status;
}

ADI_ADC_STATUS AdcIfSetLpfCoeff(ADC_INTERFACE_INFO *pInfo, float *pCoeffs, int8_t adcIdx)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    status = AdcIfAccessDspMem(pInfo, 1, adcIdx);
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = adi_adc_SetLpfCoeff(pInfo->hAdc, pCoeffs, adcIdx);
    }
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = AdcIfAccessDspMem(pInfo, 0, adcIdx);
    }
    return status;
}

ADI_ADC_STATUS AdcIfGetLpfCoeff(ADC_INTERFACE_INFO *pInfo, int8_t adcIdx, float *pCoeffs)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;

    status = AdcIfAccessDspMem(pInfo, 1, adcIdx);
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = adi_adc_GetLpfCoeff(pInfo->hAdc, adcIdx, pCoeffs);
    }
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = AdcIfAccessDspMem(pInfo, 0, adcIdx);
    }
    return status;
}

ADI_ADC_STATUS AdcIfGetGain(ADC_INTERFACE_INFO *pInfo, uint8_t *pChanIdx, int8_t numChan,
                            int8_t adcIdx, float *pGainVal)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;

    status = AdcIfAccessDspMem(pInfo, 1, adcIdx);
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = adi_adc_GetChannelGain(pInfo->hAdc, pChanIdx, numChan, adcIdx, pGainVal);
    }
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = AdcIfAccessDspMem(pInfo, 0, adcIdx);
    }

    return status;
}

ADI_ADC_STATUS AdcIfGetXtGain(ADC_INTERFACE_INFO *pInfo, uint8_t *pChanIdx, int8_t numChan,
                              int8_t adcIdx, float *pXtGainVal)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;

    status = AdcIfAccessDspMem(pInfo, 1, adcIdx);
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = adi_adc_GetChannelXtGain(pInfo->hAdc, pChanIdx, numChan, adcIdx, pXtGainVal);
    }
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = AdcIfAccessDspMem(pInfo, 0, adcIdx);
    }

    return status;
}

ADI_ADC_STATUS AdcIfGetXtAggressor(ADC_INTERFACE_INFO *pInfo, uint8_t *pChanIdx, int8_t numChan,
                                   int8_t adcIdx, ADI_ADC_CHAN_XT_AGGRESSOR *pXtAggrVal)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;

    status = AdcIfAccessDspMem(pInfo, 1, adcIdx);
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = adi_adc_GetChannelXtAggressor(pInfo->hAdc, pChanIdx, numChan, adcIdx, pXtAggrVal);
    }
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = AdcIfAccessDspMem(pInfo, 0, adcIdx);
    }

    return status;
}

ADI_ADC_STATUS AdcIfGetOffset(ADC_INTERFACE_INFO *pInfo, uint8_t *pChanIdx, int8_t numChan,
                              int8_t adcIdx, int32_t *pOffsetVal)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;

    status = AdcIfAccessDspMem(pInfo, 1, adcIdx);
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = adi_adc_GetChannelOffset(pInfo->hAdc, pChanIdx, numChan, adcIdx, pOffsetVal);
    }
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = AdcIfAccessDspMem(pInfo, 0, adcIdx);
    }

    return status;
}

ADI_ADC_STATUS AdcIfGetShift(ADC_INTERFACE_INFO *pInfo, uint8_t *pChanIdx, int8_t numChan,
                             int8_t adcIdx, uint8_t *pVal)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;

    status = AdcIfAccessDspMem(pInfo, 1, adcIdx);
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = adi_adc_GetShift(pInfo->hAdc, pChanIdx, numChan, adcIdx, pVal);
    }
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = AdcIfAccessDspMem(pInfo, 0, adcIdx);
    }

    return status;
}

ADI_ADC_STATUS AdcIfGetScfCoeff(ADC_INTERFACE_INFO *pInfo, uint8_t *pChanIdx, uint8_t numChan,
                                int8_t adcIdx, float *pCoeffs[])
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    status = AdcIfAccessDspMem(pInfo, 1, adcIdx);
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = adi_adc_GetScfCoeff(pInfo->hAdc, pChanIdx, numChan, adcIdx, pCoeffs);
    }
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = AdcIfAccessDspMem(pInfo, 0, adcIdx);
    }
    return status;
}

ADI_ADC_STATUS AdcIfGetHpfCoeff(ADC_INTERFACE_INFO *pInfo, int8_t adcIdx, float *pNumCoeffs,
                                double *pDenCoeffs)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;

    status = AdcIfAccessDspMem(pInfo, 1, adcIdx);
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = adi_adc_GetHpfCoeff(pInfo->hAdc, adcIdx, pNumCoeffs, pDenCoeffs);
    }
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = AdcIfAccessDspMem(pInfo, 0, adcIdx);
    }
    return status;
}

ADI_ADC_STATUS AdcIfSetHpfCutoff(ADC_INTERFACE_INFO *pInfo, uint32_t bwOption, int8_t adcIdx)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;

    status = AdcIfAccessDspMem(pInfo, 1, adcIdx);
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = adi_adc_SetHpfCutoff(pInfo->hAdc, bwOption, adcIdx);
        if (status == ADI_ADC_STATUS_SUCCESS)
        {
            pInfo->bwOption[adcIdx] = bwOption;
        }
    }
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = AdcIfAccessDspMem(pInfo, 0, adcIdx);
    }

    return status;
}

ADI_ADC_STATUS AdcIfGetHpfCutoff(ADC_INTERFACE_INFO *pInfo, int8_t adcIdx, uint32_t *pBwOption)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;

    if (pInfo->adcCfg.pAdcType[adcIdx] != ADI_ADC_TYPE_ADE91XX)
    {
        *pBwOption = pInfo->bwOption[adcIdx];
    }
    else
    {
        *pBwOption = 0;
        status = ADI_ADC_STATUS_INVALID_ADC_TYPE;
    }

    return status;
}

ADI_ADC_STATUS AdcIfGetDspConfig(ADC_INTERFACE_INFO *pInfo, uint8_t *pChanIdx, int8_t numChan,
                                 int8_t adcIdx, ADI_ADC_DSP_DATAPATH_PARAMS *pDatapathParams,
                                 ADI_ADC_DSP_CHANNEL_PARAMS *pChannelParams)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    uint32_t numBytesRead = 0;
    ADI_ADC_DSP_DATARATE *pDataRate = &pDatapathParams->datarate;

    adcStatus = AdcIfReadRegister(pInfo, ADDR_ADEMA127_MMR_RETAINED_ADC_CMI, adcIdx,
                                  &pInfo->adcRegBuff[0], &numBytesRead);
    pInfo->adcRegParams[adcIdx].cmiReg = pInfo->adcRegBuff[1];

    if (adcStatus == ADI_ADC_STATUS_SUCCESS)
    {
        adcStatus = AdcIfReadRegister(pInfo, ADDR_ADEMA127_MMR_RETAINED_ADC_INV, adcIdx,
                                      &pInfo->adcRegBuff[0], &numBytesRead);
        pInfo->adcRegParams[adcIdx].invReg = pInfo->adcRegBuff[1];
    }
    if (adcStatus == ADI_ADC_STATUS_SUCCESS)
    {
        adcStatus = AdcIfGetDatapathConfig(pInfo, pChanIdx, numChan, adcIdx,
                                           &pDatapathParams->pDataPathConfig[0]);
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = AdcIfGetPhaseOffset(pInfo, pChanIdx, numChan, adcIdx,
                                            &pDatapathParams->pPhaseOffset[0]);
        }
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = AdcIfGetDatarate(pInfo, adcIdx, &pDataRate->decimationRate,
                                         &pDataRate->clkPreScalar, &pDataRate->decimationX2);
        }
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = AdcIfGetDatapathAlpha(pInfo, adcIdx, (uint8_t *)&pDatapathParams->alpha);
        }
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = AdcIfGetDspRam(pInfo, pChanIdx, numChan, adcIdx, pChannelParams);
        }
    }
    return adcStatus;
}

ADI_ADC_STATUS AdcIfSetDspConfig(ADC_INTERFACE_INFO *pInfo,
                                 ADI_ADC_DSP_DATAPATH_PARAMS *pDatapathParams,
                                 ADI_ADC_DSP_CHANNEL_PARAMS *pChannelParams, uint8_t *pChanIdx,
                                 int8_t numChan, int8_t adcIdx)
{
    ADC_INTERFACE_INFO *pAdcIf = pInfo;
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    adcStatus = AdcIfConfigDspLock(pAdcIf, 0, adcIdx);
    if (adcStatus == ADI_ADC_STATUS_SUCCESS)
    {
        adcStatus =
            adi_adc_SetDatapathParams(pAdcIf->hAdc, pDatapathParams, pChanIdx, numChan, adcIdx);
    }

    if (adcStatus == ADI_ADC_STATUS_SUCCESS)
    {
        adcStatus = AdcIfWriteRegister(pInfo, ADDR_ADEMA127_MMR_RETAINED_ADC_CMI,
                                       pInfo->adcRegParams[adcIdx].cmiReg, (int8_t)adcIdx);
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = AdcIfWriteRegister(pInfo, ADDR_ADEMA127_MMR_RETAINED_ADC_GAIN,
                                           pInfo->adcRegParams[adcIdx].gain, (int8_t)adcIdx);
        }
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = AdcIfWriteRegister(pInfo, ADDR_ADEMA127_MMR_RETAINED_ADC_INV,
                                           pInfo->adcRegParams[adcIdx].invReg, (int8_t)adcIdx);
        }
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = AdcIfConfigDspLock(pInfo, 1, adcIdx);
        }
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = AdcIfSetDspRam(pInfo, pChannelParams, pChanIdx, numChan, adcIdx);
        }
    }
    return adcStatus;
}

ADI_ADC_STATUS AdcIfGetDspRam(ADC_INTERFACE_INFO *pInfo, uint8_t *pChanIdx, int8_t numChan,
                              int8_t adcIdx, ADI_ADC_DSP_CHANNEL_PARAMS *pChannelParams)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    if (adcStatus == ADI_ADC_STATUS_SUCCESS)
    {
        adcStatus = AdcIfAccessDspMem(pInfo, 1, adcIdx);
    }
    if (adcStatus == ADI_ADC_STATUS_SUCCESS)
    {
        adcStatus =
            adi_adc_GetChannelParams(pInfo->hAdc, pChanIdx, numChan, adcIdx, pChannelParams);
    }
    if (adcStatus == ADI_ADC_STATUS_SUCCESS)
    {
        adcStatus = AdcIfAccessDspMem(pInfo, 0, adcIdx);
    }
    return adcStatus;
}
ADI_ADC_STATUS AdcIfSetDspRam(ADC_INTERFACE_INFO *pInfo, ADI_ADC_DSP_CHANNEL_PARAMS *pChannelParams,
                              uint8_t *pChanIdx, int8_t numChan, int8_t adcIdx)
{
    ADC_INTERFACE_INFO *pAdcIf = pInfo;
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    if (adcStatus == ADI_ADC_STATUS_SUCCESS)
    {
        adcStatus = AdcIfAccessDspMem(pAdcIf, 1, adcIdx);
    }
    if (adcStatus == ADI_ADC_STATUS_SUCCESS)
    {
        adcStatus =
            adi_adc_SetChannelParams(pAdcIf->hAdc, pChannelParams, pChanIdx, numChan, adcIdx);
    }
    if (adcStatus == ADI_ADC_STATUS_SUCCESS)
    {
        adcStatus = AdcIfAccessDspMem(pAdcIf, 0, adcIdx);
    }
    return adcStatus;
}

ADI_ADC_STATUS AdcIfGetDatapathAlpha(ADC_INTERFACE_INFO *pInfo, int8_t adcIdx, uint8_t *pAlphaVal)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    AdcIfStopCapture(pInfo);
    status = adi_adc_GetDatapathAlpha(pInfo->hAdc, adcIdx, pAlphaVal);

    return status;
}

ADI_ADC_STATUS AdcIfGetPhaseOffset(ADC_INTERFACE_INFO *pInfo, uint8_t *pChanIdx, int8_t numChan,
                                   int8_t adcIdx, float *pPhOffsetVal)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;

    AdcIfStopCapture(pInfo);
    status = adi_adc_GetChannelPhaseOffset(pInfo->hAdc, pChanIdx, numChan, adcIdx, pPhOffsetVal);

    return status;
}

ADI_ADC_STATUS AdcIfSetAdcCmi(ADC_INTERFACE_INFO *pInfo, int8_t adcIdx, uint8_t cmiReg)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;

    if (pInfo->adcCfg.pAdcType[adcIdx] == ADI_ADC_TYPE_ADE91XX)
    {
        status = ADI_ADC_STATUS_INVALID_ADC_TYPE;
    }
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        // Write CMI register value
        status = AdcIfWriteRegister(pInfo, ADDR_ADEMA127_MMR_RETAINED_ADC_CMI, cmiReg, adcIdx);
    }

    return status;
}

ADI_ADC_STATUS AdcIfGetAdcCmi(ADC_INTERFACE_INFO *pInfo, int8_t adcIdx, uint8_t *pCmiReg)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    uint32_t numBytesRead = 0;
    uint8_t cmiReg[2];
    if (pInfo->adcCfg.pAdcType[adcIdx] == ADI_ADC_TYPE_ADE91XX)
    {
        status = ADI_ADC_STATUS_INVALID_ADC_TYPE;
    }
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        // Read CMI register value
        status = AdcIfReadRegister(pInfo, ADDR_ADEMA127_MMR_RETAINED_ADC_CMI, adcIdx, cmiReg,
                                   &numBytesRead);
        *pCmiReg = cmiReg[1]; // Extract the register value
    }
    return status;
}

ADI_ADC_STATUS AdcIfSetAdcInv(ADC_INTERFACE_INFO *pInfo, int8_t adcIdx, uint8_t invReg)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    uint32_t numBytesRead = 0;
    uint8_t invRegVal[2];

    if (pInfo->adcCfg.pAdcType[adcIdx] == ADI_ADC_TYPE_ADE91XX)
    {
        status = AdcIfReadRegister(pInfo, ADDR_ADE911X_MAP0_CONFIG_FILT, adcIdx, invRegVal,
                                   &numBytesRead);
        if (status == ADI_ADC_STATUS_SUCCESS)
        {
            // Clear the bits for ADC Invert
            invRegVal[1] &= ~(BITM_ADE911X_MAP0_CONFIG_FILT_I_ADC_INVERT |
                              BITM_ADE911X_MAP0_CONFIG_FILT_V1_ADC_INVERT |
                              BITM_ADE911X_MAP0_CONFIG_FILT_V2_ADC_INVERT);
            // Set the new invert value
            invRegVal[1] |= (invReg & 0x07) << BITP_ADE911X_MAP0_CONFIG_FILT_I_ADC_INVERT;

            // Write the updated register value
            status = AdcIfWriteRegister(pInfo, ADDR_ADE911X_MAP0_CONFIG_FILT, invRegVal[1], adcIdx);
        }
    }
    else if (pInfo->adcCfg.pAdcType[adcIdx] == ADI_ADC_TYPE_ADEMA127)
    {
        // For ADEMA127, write the ADC Invert register value
        status =
            AdcIfWriteRegister(pInfo, ADDR_ADEMA127_MMR_RETAINED_ADC_INV, (invReg & 0x7F), adcIdx);
    }
    else
    {
        // For ADEMA124, write the ADC Invert register value
        status =
            AdcIfWriteRegister(pInfo, ADDR_ADEMA127_MMR_RETAINED_ADC_INV, (invReg & 0x0F), adcIdx);
    }

    return status;
}

ADI_ADC_STATUS AdcIfGetAdcInv(ADC_INTERFACE_INFO *pInfo, int8_t adcIdx, uint8_t *pInvReg)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    uint32_t numBytesRead = 0;
    uint8_t invReg[2];

    if (pInfo->adcCfg.pAdcType[adcIdx] == ADI_ADC_TYPE_ADE91XX)
    {
        status =
            AdcIfReadRegister(pInfo, ADDR_ADE911X_MAP0_CONFIG_FILT, adcIdx, invReg, &numBytesRead);
        if (status == ADI_ADC_STATUS_SUCCESS)
        {
            *pInvReg = (invReg[1] >> BITP_ADE911X_MAP0_CONFIG_FILT_I_ADC_INVERT) & 0x07;
        }
    }
    else
    {
        status = AdcIfReadRegister(pInfo, ADDR_ADEMA127_MMR_RETAINED_ADC_INV, adcIdx, invReg,
                                   &numBytesRead);
        if (status == ADI_ADC_STATUS_SUCCESS)
        {
            *pInvReg = invReg[1]; // Extract the register value
        }
    }

    return status;
}
ADI_ADC_STATUS AdcIfGetCompCoeff(ADC_INTERFACE_INFO *pInfo, uint8_t *pChanIdx, uint8_t numChan,
                                 int8_t adcIdx, float *pCoeffs[])
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    status = AdcIfAccessDspMem(pInfo, 1, adcIdx);
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = adi_adc_GetCompCoeff(pInfo->hAdc, pChanIdx, numChan, adcIdx, pCoeffs);
    }
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = AdcIfAccessDspMem(pInfo, 0, adcIdx);
    }
    return status;
}

ADI_ADC_STATUS AdcIfSetCompCoeff(ADC_INTERFACE_INFO *pInfo, float *pCoeffs[], uint8_t *pChanIdx,
                                 uint8_t numChan, int8_t adcIdx)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    status = AdcIfAccessDspMem(pInfo, 1, adcIdx);
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = adi_adc_SetCompCoeff(pInfo->hAdc, pCoeffs, pChanIdx, numChan, adcIdx);
    }
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = AdcIfAccessDspMem(pInfo, 0, adcIdx);
    }
    return status;
}

/**
 * @}
 */
