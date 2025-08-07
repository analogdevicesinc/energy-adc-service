/******************************************************************************
 Copyright (c) 2025  Analog Devices Inc.
******************************************************************************/

/**
 * @file        adc_example_attributes.c
 * @brief       The functions to set/get attributes.
 * @{
 */

/*============= I N C L U D E S =============*/
#include "adc_example_attributes.h"
#include "adc_service_dsp_interface.h"

/*=============  D E F I N I T I O N S  =============*/

/*=============  C O D E  =============*/

int32_t SetAttribute(ADC_INTERFACE_INFO *pAdcIf, uint32_t attribute)
{
    int32_t status = 0;
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;

    switch (attribute)
    {
    case ADC_ATTRIBUTE_SAMPLING_RATE:
        status = adi_adcutil_PopulateSamplingRate(
            pAdcIf->clkIn, pAdcIf->adcSamplingRate, pAdcIf->decimateBy2, pAdcIf->adcCfg.numAdc,
            &pAdcIf->adcCfg.pAdcType[0], &pAdcIf->configRegisters[0]);
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = adi_adc_SetSamplingRate(pAdcIf->hAdc, &pAdcIf->configRegisters[0]);
        }
        if (adcStatus != ADI_ADC_STATUS_SUCCESS)
        {
            status = 1;
        }
        break;
    default:
        status = 1;

        break;
    }

    return status;
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
    adcStatus = AdcIfEnableDatapath(
        pAdcIf, &pAdcIf->adcRegParams[adcIndex].adcDatapathParams.pDataPathConfig[0], pChanIdx, 1,
        adcIndex);
    if (adcStatus != ADI_ADC_STATUS_SUCCESS)
    {
        status = 1;
    }
    return status;
}

/**
 * @}
 */
