/******************************************************************************
 Copyright (c) 2025  Analog Devices Inc.
******************************************************************************/

/**
 * @file        adc_example_attributes.c
 * @brief       The functions to set/get attributes.
 * @{
 */

/*============= I N C L U D E S =============*/
#if ENABLE_TEST_FIRMWARE == 1
#include "adc_example_ucomm.h"
#else
#include "adc_example_iio.h"
#include "adi_cli.h"
#endif
#include "ADEMA127_addr_def.h"
#include "ADEMA127_addr_rdef.h"
#include "adc_datapath_cfg.h"
#include "adc_example_attributes.h"
#include "adc_example_tdm.h"
#include "adc_service_dsp_interface.h"
#include "adc_service_interface.h"
#include "ade_crc.h"
#include "adi_adc.h"
#include "adi_adc_version.h"
#include "adi_circ_buf.h"
#include "adi_evb.h"
#include "app_cfg.h"
#include "board_cfg.h"
#include <errno.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*=============  D E F I N I T I O N S  =============*/
static void ProcessInitCommand(ADC_EXAMPLE *pExample);
static void ApplySettings(ADC_EXAMPLE *pExample);
static void SetChanGainAttr(uint8_t *pChanIdx, uint8_t *pValue, uint8_t valueSize);
static void SetChanXtGainAttr(uint8_t *pChanIdx, uint8_t *pValue, uint8_t valueSize);
static void SetChanXtAggressorAttr(uint8_t *pChanIdx, uint8_t *pValue, uint8_t valueSize);
static void SetChanOffsetAttr(uint8_t *pChanIdx, int8_t *pValue, uint8_t valueSize);
static void SetChanIntegerSampleDelayAttr(uint8_t *pChanIdx, int8_t *pValue, uint8_t valueSize);
static void SetChanShiftAttr(uint8_t *pChanIdx, uint8_t *pValue, uint8_t valueSize);
static ADI_ADC_STATUS SetSamplingRate(void);
static uint32_t ExtractDatapathConfig(ADI_ADC_CHAN_DATAPATH_CONFIG dataPathConfig);
static void ResetAdc(void);
static void ChooseSettings(uint8_t *pValue);
static void UpdateAdcErrorStatus(ADI_ADC_STATUS_OUTPUT *pAdcStatusOutput, uint8_t numAdc);

/** List of EVBs supported */
char *pEvbAvailable[] = {"EVAL-ADEMA127KTZ", "_"};

/** List of HPF Cutoff frequencies available */
char *pAdcExampleSettings[] = {"recommended", "default_adema127"};

/*=============  C O D E  =============*/

int32_t SetAttribute(int32_t attrId, uint8_t *pChanIdx, uint8_t *pValue, uint8_t valueSize)
{
    int32_t status = 0;
    ADC_EXAMPLE *pExample = GetAdcExampleInfo();
    ADC_EXAMPLE_ATTR_INFO *pExampleAttrInfo = &pExample->adcExampleAttrInfo;
    ADC_INTERFACE_INFO *pAdcIf = pExample->pAdcIf;
    double value;
    char boardName[18]; // A temp buffer to store boardName
    uint32_t writeVal;
    switch (attrId)
    {
    case EXAMPLE_ATTR_ID_EVB_TYPE:
        memcpy(boardName, pValue, valueSize);
        // boardname starts with \n so we skip the first character
        if (strcmp(&boardName[1], pEvbAvailable[0]) == 0)
        {
            pExampleAttrInfo->adcVariant = ADI_ADC_TYPE_ADEMA127;
            pExampleAttrInfo->adcIndex = 0;
            pExampleAttrInfo->chanConfig = 0xFFFFFFF;
        }
        break;

    case ADC_EXAMPLE_ATTR_ID_CHAN_GAIN:
        SetChanGainAttr(pChanIdx, pValue, valueSize);
        break;

    case ADC_EXAMPLE_ATTR_ID_CHAN_XT_GAIN:
        SetChanXtGainAttr(pChanIdx, pValue, valueSize);
        break;

    case ADC_EXAMPLE_ATTR_ID_CHAN_XT_AGGRESSOR:
        SetChanXtAggressorAttr(pChanIdx, pValue, valueSize);
        break;

    case ADC_EXAMPLE_ATTR_ID_CHAN_OFFSET:
        SetChanOffsetAttr(pChanIdx, (int8_t *)pValue, valueSize);
        break;

    case ADC_EXAMPLE_ATTR_ID_CHAN_SHIFT:
        SetChanShiftAttr(pChanIdx, pValue, valueSize);
        break;

    case ADC_EXAMPLE_ATTR_ID_DATAPATH_CONFIG:
        memcpy(&writeVal, pValue, valueSize);
        SetDatapathConfig(pAdcIf, pChanIdx, pExampleAttrInfo->adcIndex, writeVal);
        break;
    case ADC_EXAMPLE_ATTR_ID_CHAN_SCALE:
        memcpy(&value, pValue, valueSize);
        pExampleAttrInfo->adcChanScale[*pChanIdx] = value;
        break;

    case ADC_EXAMPLE_ATTR_ID_CHOOSE_SETTINGS:
        ChooseSettings(pValue);
        break;

    case ADC_EXAMPLE_ATTR_ID_START_DETECT:
        StartTamperDetection();
        break;

    case ADC_EXAMPLE_ATTR_ID_STOP_DETECT:
        StopTamperDetection();
        break;

    case ADC_EXAMPLE_ATTR_ID_APPLY_SETTING:
        ApplySettings(pExample);
        break;

    case ADC_EXAMPLE_ATTR_ID_CHAN_INTEGER_SAMPLE_DELAY:
        SetChanIntegerSampleDelayAttr(pChanIdx, (int8_t *)pValue, valueSize);
        break;

    case ADC_EXAMPLE_ATTR_ID_SAMPLING_RATE:
        status = (int32_t)SetSamplingRate();
        break;
    default:
        return -EINVAL;
    }

    return status;
}

int GetAttribute(int32_t attrId, int32_t *pChanIdx, uint32_t *pValue, uint8_t *pValueSize)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    float val;
    uint8_t chanShift;
    ADI_ADC_CHAN_XT_AGGRESSOR chanXtAggressor = {0};
    uint8_t xtAggressor;
    int32_t chanOffset;
    ADC_EXAMPLE *pExample = GetAdcExampleInfo();
    ADC_EXAMPLE_ATTR_INFO *pExampleAttrInfo = &pExample->adcExampleAttrInfo;
    ADC_INTERFACE_INFO *pAdcIf = pExample->pAdcIf;
    int8_t adcIdx = pExampleAttrInfo->adcIndex;
    ADI_ADC_DSP_DATAPATH_PARAMS *pDatapathParams = &pAdcIf->adcRegParams[adcIdx].adcDatapathParams;
    uint32_t chanDataPathVal;
    uint8_t len;
    uint8_t intSampDelay;
    switch (attrId)
    {
    case ADC_EXAMPLE_ATTR_ID_CHAN_GAIN:
        *pValueSize = sizeof(float);
        status = AdcIfGetGain(pAdcIf, (uint8_t *)pChanIdx, 1, pExampleAttrInfo->adcIndex, &val);
        memcpy(pValue, &val, sizeof(float));
        break;
    case ADC_EXAMPLE_ATTR_ID_CHAN_XT_GAIN:
        *pValueSize = sizeof(float);
        status = AdcIfGetXtGain(pAdcIf, (uint8_t *)pChanIdx, 1, pExampleAttrInfo->adcIndex, &val);
        memcpy(pValue, &val, sizeof(float));
        break;
    case ADC_EXAMPLE_ATTR_ID_CHAN_XT_AGGRESSOR:
        *pValueSize = sizeof(uint8_t);
        status = AdcIfGetXtAggressor(pAdcIf, (uint8_t *)pChanIdx, 1, pExampleAttrInfo->adcIndex,
                                     &chanXtAggressor);
        memcpy(&xtAggressor, &chanXtAggressor, sizeof(ADI_ADC_CHAN_XT_AGGRESSOR));
        memcpy(pValue, &xtAggressor, sizeof(uint8_t));
        break;
    case ADC_EXAMPLE_ATTR_ID_CHAN_OFFSET:
        *pValueSize = sizeof(int32_t);
        status =
            AdcIfGetOffset(pAdcIf, (uint8_t *)pChanIdx, 1, pExampleAttrInfo->adcIndex, &chanOffset);
        memcpy(pValue, &chanOffset, sizeof(int32_t));
        break;
    case ADC_EXAMPLE_ATTR_ID_CHAN_SHIFT:
        *pValueSize = sizeof(uint8_t);
        status =
            AdcIfGetShift(pAdcIf, (uint8_t *)pChanIdx, 1, pExampleAttrInfo->adcIndex, &chanShift);
        memcpy(pValue, &chanShift, sizeof(uint8_t));
        break;
    case ADC_EXAMPLE_ATTR_ID_DATAPATH_CONFIG:
        *pValueSize = sizeof(uint32_t);
        status = adi_adc_GetDatapathParams(pAdcIf->hAdc, (uint8_t *)pChanIdx, 1,
                                           pExampleAttrInfo->adcIndex, pDatapathParams);
        chanDataPathVal = ExtractDatapathConfig(pDatapathParams->pDataPathConfig[0]);
        memcpy(pValue, &chanDataPathVal, sizeof(uint32_t));
        break;
    case ADC_EXAMPLE_ATTR_ID_CHAN_SCALE:
        *pValueSize = sizeof(float);
        memcpy(pValue, &pExampleAttrInfo->adcChanScale[*pChanIdx], sizeof(float));
        break;
    case ADC_EXAMPLE_ATTR_ID_CHOOSE_SETTINGS:
        len = strlen(pAdcExampleSettings[pExampleAttrInfo->settings]) + 1; // +1 for null terminator
        *pValueSize = len;
        memcpy(pValue, pAdcExampleSettings[pExampleAttrInfo->settings], len);
        break;
    case ADC_EXAMPLE_ATTR_ID_TAMPER_CNT:
        *pValueSize = sizeof(uint32_t);
        memcpy(pValue, &pAdcIf->tamperCnt, sizeof(uint32_t));
        break;
    case ADC_EXAMPLE_ATTR_ID_CHAN_INTEGER_SAMPLE_DELAY:
        *pValueSize = sizeof(uint8_t);
        status = adi_adc_GetConfig(pAdcIf->hAdc, &pAdcIf->adcCfg);
        intSampDelay = pAdcIf->adcCfg.pIntegerSampleDelay[*pChanIdx];
        if (status != ADI_ADC_STATUS_SUCCESS)
        {
        }
        memcpy(pValue, &intSampDelay, sizeof(uint8_t));
        break;
    default:
        return -1;
    }

    return 0;
}

ADI_ADC_STATUS SetSamplingRate(void)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    ADC_EXAMPLE *pExample = GetAdcExampleInfo();
    ADC_INTERFACE_INFO *pAdcIf = pExample->pAdcIf;
    ADC_BOARD_CONFIG *pAdcBoardConfig = AdcExmGetBoardConfig();
    adcStatus = adi_adcutil_PopulateSamplingRate(
        pAdcBoardConfig->clkIn, pAdcBoardConfig->adcSamplingRate, pAdcBoardConfig->decimateBy2,
        pAdcIf->adcCfg.numAdc, &pAdcIf->adcCfg.pAdcType[0], &pAdcIf->configRegisters[0]);
    if (adcStatus == ADI_ADC_STATUS_SUCCESS)
    {
        adcStatus = adi_adc_SetSamplingRate(pAdcIf->hAdc, &pAdcIf->configRegisters[0]);
    }
    return adcStatus;
}

int32_t SetDatapathConfig(ADC_INTERFACE_INFO *pAdcIf, uint8_t *pChanIdx, uint8_t adcIndex,
                          uint32_t writeVal)
{
    int32_t status = 0;
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    pAdcIf->adcRegParams[adcIndex].adcDatapathParams.pDataPathConfig[*pChanIdx].gainOffsetEn =
        (writeVal >> 0) & 1;
    pAdcIf->adcRegParams[adcIndex].adcDatapathParams.pDataPathConfig[*pChanIdx].scfEn =
        (writeVal >> 1) & 1;
    pAdcIf->adcRegParams[adcIndex].adcDatapathParams.pDataPathConfig[*pChanIdx].hpfEn =
        (writeVal >> 2) & 1;
    pAdcIf->adcRegParams[adcIndex].adcDatapathParams.pDataPathConfig[*pChanIdx].compFiltEn =
        (writeVal >> 3) & 1;
    pAdcIf->adcRegParams[adcIndex].adcDatapathParams.pDataPathConfig[*pChanIdx].compFiltCfg =
        (writeVal >> 4) & 1;
    pAdcIf->adcRegParams[adcIndex].adcDatapathParams.pDataPathConfig[*pChanIdx].lpfEn =
        (writeVal >> 5) & 1;
    pAdcIf->adcRegParams[adcIndex].adcDatapathParams.pDataPathConfig[*pChanIdx].allPassEn =
        (writeVal >> 6) & 1;
    pAdcIf->adcRegParams[adcIndex].adcDatapathParams.pDataPathConfig[*pChanIdx].reserved =
        (writeVal >> 7) & 1;
    adcStatus = AdcIfSetDatapathConfig(
        pAdcIf, &pAdcIf->adcRegParams[adcIndex].adcDatapathParams.pDataPathConfig[0], pChanIdx, 1,
        adcIndex);
    if (adcStatus != ADI_ADC_STATUS_SUCCESS)
    {
        status = 1;
    }
    return status;
}

void ApplySettings(ADC_EXAMPLE *pExample)
{
    ADC_EXAMPLE_ATTR_INFO *pExampleAttrInfo = &pExample->adcExampleAttrInfo;
    if (pExampleAttrInfo->settings == ADC_EXAMPLE_SETTINGS_TYPE_RECOMMENDED_SETTINGS)
    {
        ProcessInitCommand(pExample);
    }
    else if (pExampleAttrInfo->settings == ADC_EXAMPLE_SETTINGS_TYPE_DEFAULT_ADEMA127)
    {
        ResetAdc();
    }
}

void ChooseSettings(uint8_t *pValue)
{
    uint8_t val;
    ADC_EXAMPLE *pExample = GetAdcExampleInfo();
    ADC_EXAMPLE_ATTR_INFO *pExampleAttrInfo = &pExample->adcExampleAttrInfo;

    char *pTrimmedSrc = (char *)pValue;
    while (*pTrimmedSrc == '\n' || *pTrimmedSrc == '\r' || *pTrimmedSrc == ' ')
    {
        pTrimmedSrc++;
    }
    for (val = ADC_EXAMPLE_SETTINGS_TYPE_RECOMMENDED_SETTINGS;
         val <= ADC_EXAMPLE_SETTINGS_TYPE_DEFAULT_ADEMA127; val++)
    {
        if (!strcmp(pTrimmedSrc, pAdcExampleSettings[val]))
        {
            break;
        }
    }
    pExampleAttrInfo->settings = val;
}

void SetChanGainAttr(uint8_t *pChanIdx, uint8_t *pValue, uint8_t valueSize)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    ADC_EXAMPLE *pExample = GetAdcExampleInfo();
    ADC_EXAMPLE_ATTR_INFO *pExampleAttrInfo = &pExample->adcExampleAttrInfo;
    ADC_INTERFACE_INFO *pAdcIf = pExample->pAdcIf;
    float value;
    // Copy the value from pValue to val
    memcpy(&value, pValue, valueSize);
    status = AdcIfSetGain(pAdcIf, &value, pChanIdx, 1, pExampleAttrInfo->adcIndex);
    if (status != ADI_ADC_STATUS_SUCCESS)
    {
    }
}

void SetChanXtGainAttr(uint8_t *pChanIdx, uint8_t *pValue, uint8_t valueSize)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    ADC_EXAMPLE *pExample = GetAdcExampleInfo();
    ADC_EXAMPLE_ATTR_INFO *pExampleAttrInfo = &pExample->adcExampleAttrInfo;
    ADC_INTERFACE_INFO *pAdcIf = pExample->pAdcIf;
    float value;
    memcpy(&value, pValue, valueSize);
    status = AdcIfSetXtGain(pAdcIf, &value, pChanIdx, 1, pExampleAttrInfo->adcIndex);
    if (status != ADI_ADC_STATUS_SUCCESS)
    {
    }
}
void SetChanXtAggressorAttr(uint8_t *pChanIdx, uint8_t *pValue, uint8_t valueSize)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    ADC_EXAMPLE *pExample = GetAdcExampleInfo();
    ADC_EXAMPLE_ATTR_INFO *pExampleAttrInfo = &pExample->adcExampleAttrInfo;
    ADC_INTERFACE_INFO *pAdcIf = pExample->pAdcIf;
    ADI_ADC_CHAN_XT_AGGRESSOR aggrValue;
    uint8_t value;
    memcpy(&value, pValue, valueSize);
    memcpy(&aggrValue, &value, sizeof(ADI_ADC_CHAN_XT_AGGRESSOR));
    status = AdcIfSetXtAggressor(pAdcIf, &aggrValue, pChanIdx, 1, pExampleAttrInfo->adcIndex);
    if (status != ADI_ADC_STATUS_SUCCESS)
    {
    }
}

void SetChanOffsetAttr(uint8_t *pChanIdx, int8_t *pValue, uint8_t valueSize)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    ADC_EXAMPLE *pExample = GetAdcExampleInfo();
    ADC_EXAMPLE_ATTR_INFO *pExampleAttrInfo = &pExample->adcExampleAttrInfo;
    ADC_INTERFACE_INFO *pAdcIf = pExample->pAdcIf;
    int32_t value;
    memcpy(&value, pValue, valueSize);
    status = AdcIfSetOffset(pAdcIf, &value, pChanIdx, 1, pExampleAttrInfo->adcIndex);
    if (status != ADI_ADC_STATUS_SUCCESS)
    {
    }
}

void SetChanShiftAttr(uint8_t *pChanIdx, uint8_t *pValue, uint8_t valueSize)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    ADC_EXAMPLE *pExample = GetAdcExampleInfo();
    ADC_EXAMPLE_ATTR_INFO *pExampleAttrInfo = &pExample->adcExampleAttrInfo;
    ADC_INTERFACE_INFO *pAdcIf = pExample->pAdcIf;
    uint8_t value;
    memcpy(&value, pValue, valueSize);
    status = AdcIfSetShift(pAdcIf, &value, pChanIdx, 1, pExampleAttrInfo->adcIndex);
    if (status != ADI_ADC_STATUS_SUCCESS)
    {
    }
}

void SetChanIntegerSampleDelayAttr(uint8_t *pChanIdx, int8_t *pValue, uint8_t valueSize)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    ADC_EXAMPLE *pExample = GetAdcExampleInfo();
    ADC_EXAMPLE_ATTR_INFO *pExampleAttrInfo = &pExample->adcExampleAttrInfo;
    ADC_INTERFACE_INFO *pAdcIf = pExample->pAdcIf;
    uint8_t value;
    memcpy(&value, pValue, valueSize);
    status = adi_adc_SetIntegerSampleDelay(pAdcIf->hAdc, &value, pChanIdx, 1,
                                           pExampleAttrInfo->adcIndex);
    if (status != ADI_ADC_STATUS_SUCCESS)
    {
    }
}

uint32_t ExtractDatapathConfig(ADI_ADC_CHAN_DATAPATH_CONFIG dataPathConfig)
{
    uint8_t result = 0;
    result |= (dataPathConfig.gainOffsetEn & 0x01);
    result |= (dataPathConfig.scfEn & 0x01) << 1;
    result |= (dataPathConfig.hpfEn & 0x01) << 2;
    result |= (dataPathConfig.compFiltEn & 0x01) << 3;
    result |= (dataPathConfig.compFiltCfg & 0x01) << 4;
    result |= (dataPathConfig.lpfEn & 0x01) << 5;
    result |= (dataPathConfig.allPassEn & 0x01) << 6;
    return result;
}

void ResetAdc()
{
    int32_t status = ADI_EVB_STATUS_SUCCESS;
    ADC_EXAMPLE *pExample = GetAdcExampleInfo();
    ADC_EXAMPLE_ATTR_INFO *pExampleAttrInfo = &pExample->adcExampleAttrInfo;
    ADC_INTERFACE_INFO *pAdcIf = pExample->pAdcIf;
    bool isSampleCaptureEnabled = pAdcIf->enableRun;
    // When ADC is reset,  all registers except DSP RAM addresses are reset.
    // Hence, lock datapath config to reset DSP RAM region.
    AdcIfStopCapture(pAdcIf);
    status = AdcIfWriteRegister(pAdcIf, ADDR_ADEMA127_MMR_DATAPATH_CONFIG_LOCK, 0,
                                pExampleAttrInfo->adcIndex);
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = AdcIfWriteRegister(pAdcIf, ADDR_ADEMA127_MMR_DATAPATH_CONFIG_LOCK, 1,
                                    pExampleAttrInfo->adcIndex);
        EvbDelayMs(1);
        if (status != ADI_ADC_STATUS_SUCCESS)
        {
            // Handle error
        }
    }
    else
    {
        // Handle error
    }
#ifdef USE_SIMUL_ADC
    ADI_ADC_CONFIG *pConfig = &pAdcIf->adcCfg;
    EvbConnectAdc(pConfig->numAdc, &pConfig->pAdcType[0]);
#else
    status = EvbResetAdcs();
#endif
    if (status != ADI_ADC_STATUS_SUCCESS)
    {
    }
    if (isSampleCaptureEnabled)
    {
        AdcIfStartCapture(pAdcIf);
    }
}

int32_t DebugRegRead(uint32_t address, uint32_t *pDst)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    ADC_EXAMPLE *pExample = GetAdcExampleInfo();
    ADC_EXAMPLE_ATTR_INFO *pExampleAttrInfo = &pExample->adcExampleAttrInfo;
    ADC_INTERFACE_INFO *pAdcIf = pExample->pAdcIf;
    int32_t adcIdx = pExampleAttrInfo->adcIndex;
    uint16_t regAddr = 0;
    uint32_t numBytesToSend = 0;
    regAddr = (uint16_t)address;

    if (address >= ADDR_ADEMA127_DSP_RAM_CH0_COMP_COEFF_B0_LO &&
        address <= ADDR_ADEMA127_DSP_RAM_ALL_HPF_COEFF_A2_2)
    {
        adcStatus = AdcIfAccessDspMem(pAdcIf, 1, adcIdx);
    }
    if (adcStatus != ADI_ADC_STATUS_SUCCESS)
    {
        EvbLedOn(2);
    }

    adcStatus = AdcIfReadRegister(pAdcIf, regAddr, adcIdx, (uint8_t *)&pAdcIf->adcRegBuff[0],
                                  (uint32_t *)&numBytesToSend);

    if (address >= ADDR_ADEMA127_DSP_RAM_CH0_COMP_COEFF_B0_LO &&
        address <= ADDR_ADEMA127_DSP_RAM_ALL_HPF_COEFF_A2_2)
    {
        adcStatus = AdcIfAccessDspMem(pAdcIf, 0, adcIdx);
    }
    if (adcStatus != ADI_ADC_STATUS_SUCCESS)
    {
        EvbLedOn(2);
    }
    else
    {
        // Send out the reg_val instead of [(reg + 1)_val and reg_val]
        *pDst = (uint8_t)pAdcIf->adcRegBuff[adcIdx * pAdcIf->adcCfg.numAdc + 1];
    }
    UpdateAdcErrorStatus(&pAdcIf->adcStatusOutput[0], 1);

    return 0;
}

int32_t DebugRegWrite(uint32_t address, uint32_t value)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    ADC_EXAMPLE *pExample = GetAdcExampleInfo();
    ADC_EXAMPLE_ATTR_INFO *pExampleAttrInfo = &pExample->adcExampleAttrInfo;
    ADC_INTERFACE_INFO *pAdcIf = pExample->pAdcIf;
    int32_t adcIdx = pExampleAttrInfo->adcIndex;

    if (address == ADDR_ADEMA127_MMR_DATAPATH_CONFIG_LOCK && value == 0)
    {
        // Save DSP RAM region before datapath config unlock
        adcStatus =
            AdcIfGetDspRegisterStruct(pAdcIf, pAdcIf->adcCfg.numAdc, &pAdcIf->adcCfg.pAdcType[0]);
    }
    if (adcStatus != ADI_ADC_STATUS_SUCCESS)
    {
        EvbLedOn(2);
    }

    /** It is recommended to update ADC_PD_CHx, ADC_GAIN_CHx, ADC_INV_CHx, ADC_CMI_CHx  while ADC
     * conversion is halted */
    if ((address >= ADDR_ADEMA127_MMR_DATARATE &&
         address <= ADDR_ADEMA127_MMR_PHASE_OFFSET_CH6_LO) ||
        (address >= ADDR_ADEMA127_MMR_RETAINED_ADC_PD &&
         address <= ADDR_ADEMA127_MMR_RETAINED_ADC_INV))
    {
        adcStatus = AdcIfConfigDspLock(pAdcIf, 0, adcIdx);
    }
    if (adcStatus != ADI_ADC_STATUS_SUCCESS)
    {
        EvbLedOn(2);
    }

    if (address >= ADDR_ADEMA127_DSP_RAM_CH0_COMP_COEFF_B0_LO &&
        address <= ADDR_ADEMA127_DSP_RAM_ALL_HPF_COEFF_A2_2)
    {
        adcStatus = AdcIfAccessDspMem(pAdcIf, 1, adcIdx);
    }

    if (adcStatus != ADI_ADC_STATUS_SUCCESS)
    {
        EvbLedOn(2);
    }

    if (address == ADDR_ADEMA127_MMR_RETAINED_ADC_CMI)
    {
        adcStatus = AdcIfSetAdcCmi(pAdcIf, adcIdx, (uint8_t)value);
    }
    else if (address == ADDR_ADEMA127_MMR_RETAINED_ADC_GAIN)
    {
        adcStatus = AdcIfSetAdcGain(pAdcIf, adcIdx, (uint8_t)value);
    }
    else
    {
        adcStatus = AdcIfWriteRegister(pAdcIf, (uint16_t)address, (uint8_t)value, adcIdx);
    }

    if (adcStatus != ADI_ADC_STATUS_SUCCESS)
    {
        EvbLedOn(2);
    }

    if ((address >= ADDR_ADEMA127_MMR_DATARATE &&
         address <= ADDR_ADEMA127_MMR_PHASE_OFFSET_CH6_LO) ||
        (address >= ADDR_ADEMA127_MMR_RETAINED_ADC_PD &&
         address <= ADDR_ADEMA127_MMR_RETAINED_ADC_INV))
    {
        adcStatus = AdcIfConfigDspLock(pAdcIf, 1, adcIdx);
    }

    if (address == ADDR_ADEMA127_MMR_DATAPATH_CONFIG_LOCK && value == 1)
    {
        // Reload DSP RAM after datapath config is locked
        adcStatus = AdcIfPopulateDspRegisterStruct(pAdcIf, pAdcIf->adcCfg.numAdc,
                                                   &pAdcIf->adcCfg.pAdcType[0]);
    }

    if (address >= ADDR_ADEMA127_DSP_RAM_CH0_COMP_COEFF_B0_LO &&
        address <= ADDR_ADEMA127_DSP_RAM_ALL_HPF_COEFF_A2_2)
    {
        adcStatus = AdcIfAccessDspMem(pAdcIf, 0, adcIdx);
    }

    if (adcStatus != ADI_ADC_STATUS_SUCCESS)
    {
        EvbLedOn(2);
    }

    return 0;
}

void ProcessInitCommand(ADC_EXAMPLE *pExample)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    adcStatus = AdcIfInitService(pExample->pAdcIf, &pExample->adcBoardConfig);
    if (adcStatus == ADI_ADC_STATUS_SUCCESS)
    {
        /* Toggle LED to indicate that ADC init is a success */
        EvbLedOn(1);
    }
}

static void UpdateAdcErrorStatus(ADI_ADC_STATUS_OUTPUT *pAdcStatusOutput, uint8_t numAdc)
{
    uint8_t i;

    for (i = 0; i < numAdc; i++)
    {
        /* Check if any frame had ADC CRC error */
        if (pAdcStatusOutput[i].crcError != 0)
        {
            EvbLedOn(2);
            break;
        }
    }
}

/**
 * @}
 */
