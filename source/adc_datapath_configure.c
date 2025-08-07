/******************************************************************************
 Copyright (c) 2024 - 2025  Analog Devices Inc.
******************************************************************************/

/*============= I N C L U D E S =============*/
#include "adc_datapath_configure.h"
#include "ADEMA127_addr_def.h"
#include "ADEMA127_addr_rdef.h"
#include "string.h"

/*============= D E F I N E S =============*/
/**
 * @brief Q format of ADC datapath configuration values.
 *
 * This constant represents the fixed point format for the ADC
 * datapath configuration values.
 */
#define ADEMA12x_CONFIG_FRAC_BITS 22
/** Q format of phase offset ADC registers */
#define ADEMA12x_PH_OFFSET_FRAC_BITS 13
/** Q format of HPF denominator coefficients */
#define ADEMA12x_HPF_DEN_FRAC_BITS 30
/** Maximum possible offset */
#define ADEMA12x_MAX_DSP_OFFSET ((1 << 23) - 1)

/**
 * @brief Write a calibration or filter coefficient value to ADC registers.
 * @param[in]  pAdcIf  		Pointer to ADC interface structure.
 * @param[in]  pAddress    	Starting address of registers to write.
 * @param[in]  value 		Calibration or filter coefficient value.
 * @param[in]  adcIdx		ADC number to write.
 * @return 0 for success or non zero error code
 */
static ADI_ADC_STATUS AdcWriteDataPathConfig(ADI_ADC_INFO *pInfo, uint16_t address, float value,
                                             int8_t adcIdx);

/**
 * @brief Write a fixed point calibration or filter coefficient value to ADC registers.
 * @param[in]  pAdcIf  		Pointer to ADC interface structure.
 * @param[in]  pAddress    	Starting address of registers to write.
 * @param[in]  value 		Calibration or filter coefficient value.
 * @param[in]  adcIdx		ADC number to write.
 * @return 0 for success or non zero error code
 */
static ADI_ADC_STATUS AdcWriteDataPathConfigFix(ADI_ADC_INFO *pInfo, uint16_t address,
                                                uint32_t value, int8_t adcIdx);
/**
 * @brief Write a channel phase offset value to ADC registers
 * (PHASE_OFFSET_CHx_LO/PHASE_OFFSET_CHx_HI).
 * @param[in]  pAdcIf  		Pointer to ADC interface structure.
 * @param[in]  pAddress    	Starting address of registers to write.
 * @param[in]  value 		Calibration or filter coefficient value.
 * @param[in]  adcIdx		ADC number to write.
 * @return 0 for success or non zero error code
 */
static ADI_ADC_STATUS AdcWriteChanPhOffset(ADI_ADC_INFO *pInfo, uint16_t address, float value,
                                           int8_t adcIdx);

static ADI_ADC_STATUS AdcWriteHpfDenCoeffs(ADI_ADC_INFO *pInfo, uint16_t address, double value,
                                           int8_t adcIdx);
static ADI_ADC_STATUS AdcReadDataPathConfig(ADI_ADC_INFO *pInfo, uint16_t address, int8_t adcIdx,
                                            float *pValue);
static ADI_ADC_STATUS AdcReadChanPhOffset(ADI_ADC_INFO *pInfo, uint16_t address, int8_t adcIdx,
                                          float *pValue);
static ADI_ADC_STATUS AdcReadHpfDenCoeffs(ADI_ADC_INFO *pInfo, uint16_t address, int8_t adcIdx,
                                          double *pValue);

ADI_ADC_STATUS AdcWriteDataPathEnable(ADI_ADC_INFO *pInfo,
                                      ADI_ADC_CHAN_DATAPATH_CONFIG *pDatapathConfig,
                                      uint8_t *pChanIdx, int8_t numChan, int8_t adcNum)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    int32_t i;
    uint16_t addr;
    uint16_t baseAddr = ADDR_ADEMA127_MMR_DATAPATH_CONFIG_CH0;
    uint16_t addrStep = 0x1;
    uint8_t configVal;

    for (i = 0; i < numChan; i++)
    {
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            addr = baseAddr + (pChanIdx[i] * addrStep);
            configVal = (uint8_t)(pDatapathConfig[pChanIdx[i]].gainOffsetEn
                                  << BITP_ADEMA127_MMR_DATAPATH_CONFIG_CH0_GAIN_OFFSET_XT_EN_CH0) |
                        (uint8_t)(pDatapathConfig[pChanIdx[i]].scfEn
                                  << BITP_ADEMA127_MMR_DATAPATH_CONFIG_CH0_SCF_EN_CH0) |
                        (uint8_t)(pDatapathConfig[pChanIdx[i]].hpfEn
                                  << BITP_ADEMA127_MMR_DATAPATH_CONFIG_CH0_HPF_EN_CH0) |
                        (uint8_t)(pDatapathConfig[pChanIdx[i]].compFiltEn
                                  << BITP_ADEMA127_MMR_DATAPATH_CONFIG_CH0_COMP_FLT_EN_CH0) |
                        (uint8_t)(pDatapathConfig[pChanIdx[i]].compFiltCfg
                                  << BITP_ADEMA127_MMR_DATAPATH_CONFIG_CH0_COMP_FLT_CFG_CH0) |
                        (uint8_t)(pDatapathConfig[pChanIdx[i]].lpfEn
                                  << BITP_ADEMA127_MMR_DATAPATH_CONFIG_CH0_LPF_EN_CH0) |
                        (uint8_t)(pDatapathConfig[pChanIdx[i]].allPassEn
                                  << BITP_ADEMA127_MMR_DATAPATH_CONFIG_CH0_ALLPASS_EN_CH0);

            // Write to channel datapath config register
            adcStatus = AdcWriteRegister(pInfo, addr, configVal, adcNum);
        }
    }

    return adcStatus;
}

ADI_ADC_STATUS AdcSetChannelGain(ADI_ADC_INFO *pInfo, float *pGainVal, uint8_t *pChanIdx,
                                 int8_t numChan, int8_t adcNum)
{
    int32_t i;
    uint16_t addr;
    uint16_t baseAddr = ADDR_ADEMA127_DSP_RAM_CH0_GAIN_LO;
    uint16_t addrStep = 0x40;
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;

    for (i = 0; i < numChan; i++)
    {
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            if (pGainVal[i] >= 2.0 || pGainVal[i] < -2.0)
            {
                adcStatus = ADI_ADC_STATUS_DATAPATH_CONFIGURATION_FAILED;
                break;
            }
            addr = baseAddr + (pChanIdx[i] * addrStep);
            // Write ADC Channel Gain value
            adcStatus = AdcWriteDataPathConfig(pInfo, addr, pGainVal[i], adcNum);
        }
    }

    return adcStatus;
}

ADI_ADC_STATUS AdcSetChannelXtGain(ADI_ADC_INFO *pInfo, float *pXtGainVal, uint8_t *pChanIdx,
                                   int8_t numChan, int8_t adcNum)
{
    int32_t i;
    uint16_t addr;
    uint16_t baseAddr = ADDR_ADEMA127_DSP_RAM_CH0_XT_GAIN_LO;
    uint16_t addrStep = 0x40;
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;

    for (i = 0; i < numChan; i++)
    {
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            if (pXtGainVal[i] >= 2.0 || pXtGainVal[i] < -2.0)
            {
                adcStatus = ADI_ADC_STATUS_DATAPATH_CONFIGURATION_FAILED;
                break;
            }
            addr = baseAddr + (pChanIdx[i] * addrStep);
            // Write ADC Channel XT Gain value
            adcStatus = AdcWriteDataPathConfig(pInfo, addr, pXtGainVal[i], adcNum);
        }
    }
    return adcStatus;
}

ADI_ADC_STATUS AdcSetChannelXtAggressor(ADI_ADC_INFO *pInfo, ADI_ADC_CHAN_XT_AGGRESSOR *pXtAggrVal,
                                        uint8_t *pChanIdx, int8_t numChan, int8_t adcNum)
{
    int32_t i;
    uint16_t addr;
    uint16_t baseAddr = ADDR_ADEMA127_DSP_RAM_CH0_XT_AGGRESSOR;
    uint16_t addrStep = 0x40;
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    uint8_t configVal;

    for (i = 0; i < numChan; i++)
    {
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            addr = baseAddr + (pChanIdx[i] * addrStep);
            configVal = (uint8_t)(pXtAggrVal[i].xtAggressor
                                  << BITP_ADEMA127_DSP_RAM_CH_XT_AGGRESSOR_XT_AGGRESSOR) |
                        (uint8_t)(pXtAggrVal[i].xtCompEn
                                  << BITP_ADEMA127_DSP_RAM_CH_XT_AGGRESSOR_XT_COMP_EN);
            // Write ADC Channel XT Aggressor value
            adcStatus = AdcWriteRegister(pInfo, addr, configVal, adcNum);
        }
    }

    return adcStatus;
}

ADI_ADC_STATUS AdcSetChannelOffset(ADI_ADC_INFO *pInfo, int32_t *pOffsetVal, uint8_t *pChanIdx,
                                   int8_t numChan, int8_t adcNum)
{
    int32_t i;
    uint16_t addr;
    uint16_t baseAddr = ADDR_ADEMA127_DSP_RAM_CH0_OFFSET_LO;
    uint16_t addrStep = 0x40;
    int32_t valFix;
    uint8_t valLo, valMd, valHi;
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;

    for (i = 0; i < numChan; i++)
    {
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            if (pOffsetVal[i] >= (int32_t)ADEMA12x_MAX_DSP_OFFSET ||
                pOffsetVal[i] < (int32_t)(-ADEMA12x_MAX_DSP_OFFSET))
            {
                adcStatus = ADI_ADC_STATUS_DATAPATH_CONFIGURATION_FAILED;
                break;
            }
            addr = baseAddr + (pChanIdx[i] * addrStep);
            // Write ADC Channel Gain value
            valFix = (pOffsetVal[i]);
            valLo = valFix & 0xFF;
            valMd = (valFix & 0xFF00) >> 8;
            valHi = (valFix & 0xFF0000) >> 16;

            /* Write LO value in the given ADC register address */
            adcStatus = AdcWriteRegister(pInfo, addr, valLo, adcNum);

            if (adcStatus == ADI_ADC_STATUS_SUCCESS)
            {
                /* Write MD value in the given ADC register address */
                adcStatus = AdcWriteRegister(pInfo, addr + 1, valMd, adcNum);
            }
            if (adcStatus == ADI_ADC_STATUS_SUCCESS)
            {
                /* Write HI value in the given ADC register address */
                adcStatus = AdcWriteRegister(pInfo, addr + 2, valHi, adcNum);
            }
        }
    }

    return adcStatus;
}

ADI_ADC_STATUS AdcSetPhaseOffset(ADI_ADC_INFO *pInfo, float *pPhOffsetVal, uint8_t *pChanIdx,
                                 int8_t numChan, int8_t adcNum)
{
    int32_t i;
    uint16_t addr;
    uint16_t baseAddr = ADDR_ADEMA127_MMR_PHASE_OFFSET_CH0_HI;
    uint16_t addrStep = 0x2;
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;

    for (i = 0; i < numChan; i++)
    {
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            if (pPhOffsetVal[i] >= 1.0)
            {
                adcStatus = ADI_ADC_STATUS_DATAPATH_CONFIGURATION_FAILED;
                break;
            }
            if (adcStatus == ADI_ADC_STATUS_SUCCESS)
            {
                addr = baseAddr + (pChanIdx[i] * addrStep);
                adcStatus = AdcWriteChanPhOffset(pInfo, addr, pPhOffsetVal[i], adcNum);
            }
        }
    }

    return adcStatus;
}

ADI_ADC_STATUS AdcSetDataRate(ADI_ADC_INFO *pInfo, uint8_t decimationRate, uint8_t clkPreScaler,
                              uint8_t setDecimation, int8_t adcNum)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    uint8_t datarateVal;
    if (adcStatus == ADI_ADC_STATUS_SUCCESS)
    {
        datarateVal =
            (uint8_t)(((uint16_t)setDecimation << BITP_ADEMA127_MMR_DATARATE_DSP_DECIMATION_X2) |
                      ((uint16_t)clkPreScaler << BITP_ADEMA127_MMR_DATARATE_ADC_CLK_PRESCALER) |
                      ((uint16_t)decimationRate << BITP_ADEMA127_MMR_DATARATE_DECIMATION_RATE));
        // Write to channel datarate config register
        adcStatus = AdcWriteRegister(pInfo, ADDR_ADEMA127_MMR_DATARATE, datarateVal, adcNum);
    }

    return adcStatus;
}

ADI_ADC_STATUS AdcSetDatapathAlpha(ADI_ADC_INFO *pInfo, uint8_t *pAlphaVal, int8_t adcNum)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    uint8_t alphaRegVal;
    uint8_t numAdcs;
    uint8_t maxNumAdcChans = 0;

    for (uint8_t i = 0; i < pInfo->adcCfg.numAdc; i++)
    {
        maxNumAdcChans = (pInfo->pTypeConfig[i].samplesPerFrame > maxNumAdcChans)
                             ? pInfo->pTypeConfig[i].samplesPerFrame
                             : maxNumAdcChans;
    }

    for (uint8_t i = 0; i < maxNumAdcChans; i++)
    {
        // Alpha is a 4-bit value; mask off upper bits to ensure only valid bits are written.
        pAlphaVal[i] &= BITM_ADEMA127_MMR_DATAPATH_ALPHA_CH0_1_ALPHA_CH0;
    }

    // Write ADC Channel 0 and Channel 1 Alpha value
    alphaRegVal = (uint8_t)(((uint32_t)pAlphaVal[1] << 4) | pAlphaVal[0]);
    adcStatus =
        AdcWriteRegister(pInfo, ADDR_ADEMA127_MMR_DATAPATH_ALPHA_CH0_1, alphaRegVal, adcNum);

    // Write ADC Channel 2 and Channel 3 Alpha value
    alphaRegVal = (uint8_t)(((uint32_t)pAlphaVal[3] << 4) | pAlphaVal[2]);
    adcStatus =
        AdcWriteRegister(pInfo, ADDR_ADEMA127_MMR_DATAPATH_ALPHA_CH2_3, alphaRegVal, adcNum);

    if (adcNum == -1)
    {
        adcNum = 0;
        numAdcs = pInfo->adcCfg.numAdc;
    }
    else
    {
        numAdcs = 1;
    }

    for (uint8_t i = 0; i < numAdcs; i++)
    {
        if (pInfo->adcCfg.pAdcType[i + adcNum] == ADI_ADC_TYPE_ADEMA127)
        {
            // Write ADC Channel 4 and Channel 5 Alpha value
            alphaRegVal = (uint8_t)(((uint32_t)pAlphaVal[5] << 4) | pAlphaVal[4]);
            adcStatus = AdcWriteRegister(pInfo, ADDR_ADEMA127_MMR_DATAPATH_ALPHA_CH4_5, alphaRegVal,
                                         i + adcNum);

            // Write ADC Channel 6
            adcStatus = AdcWriteRegister(pInfo, ADDR_ADEMA127_MMR_DATAPATH_ALPHA_CH6, pAlphaVal[6],
                                         i + adcNum);
        }
    }
    return adcStatus;
}

ADI_ADC_STATUS AdcSetLpfCoeff(ADI_ADC_INFO *pInfo, float *pCoeffs, int8_t adcNum)
{
    int32_t i;
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    uint16_t address = ADDR_ADEMA127_DSP_RAM_ALL_LPF_COEFF_B0_LO;

    for (i = 0; i < ADEMA12x_NUM_LPF_COEFFS; i++)
    {
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = AdcWriteDataPathConfig(pInfo, address, pCoeffs[i], adcNum);
            address = address + 4;
        }
    }

    return adcStatus;
}

ADI_ADC_STATUS AdcSetCompCoeff(ADI_ADC_INFO *pInfo, float *pCoeffs[], uint8_t *pChanIdx,
                               int8_t numChan, int8_t adcNum)
{
    int32_t i, chan;
    uint16_t addr;
    uint16_t baseAddr = ADDR_ADEMA127_DSP_RAM_CH0_COMP_COEFF_B0_LO;
    uint16_t addrStep = 0x40;
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;

    for (chan = 0; chan < numChan; chan++)
    {
        addr = baseAddr + (pChanIdx[chan] * addrStep);
        for (i = 0; i < ADEMA12x_NUM_COMP_COEFFS; i++)
        {
            if (adcStatus == ADI_ADC_STATUS_SUCCESS)
            {
                adcStatus = AdcWriteDataPathConfig(pInfo, addr, pCoeffs[chan][i], adcNum);
                addr = addr + 4;
            }
        }
    }

    return adcStatus;
}

ADI_ADC_STATUS AdcSetShift(ADI_ADC_INFO *pInfo, uint8_t *pShiftVal, uint8_t *pChanIdx,
                           int8_t numChan, int8_t adcNum)
{
    int32_t i;
    uint16_t addr;
    uint16_t baseAddr = ADDR_ADEMA127_DSP_RAM_CH0_SHIFT;
    uint16_t addrStep = 0x40;
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;

    for (i = 0; i < numChan; i++)
    {
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            // Shift is a 3-bit value. Mask off any invalid bits, keeping only the lower 3.
            pShiftVal[i] &= BITM_ADEMA127_DSP_RAM_CH_SHIFT_SHIFT;
            addr = baseAddr + (pChanIdx[i] * addrStep);
            // Write ADC Channel Gain value
            adcStatus = AdcWriteRegister(pInfo, addr, pShiftVal[i], adcNum);
        }
    }

    return adcStatus;
}

ADI_ADC_STATUS AdcSetHpfCoeff(ADI_ADC_INFO *pInfo, float *pNumCoeffs, double *pDenCoeffs,
                              int8_t adcNum)
{
    int32_t i;
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    uint16_t address = ADDR_ADEMA127_DSP_RAM_ALL_HPF_COEFF_B0_LO;

    for (i = 0; i < ADEMA12x_NUM_HPF_COEFFS_NUMERATOR; i++)
    {
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = AdcWriteDataPathConfig(pInfo, address, pNumCoeffs[i], adcNum);
            address = address + 4;
        }
    }

    for (i = 0; i < ADEMA12x_NUM_HPF_COEFFS_DENOMINATOR; i++)
    {
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = AdcWriteHpfDenCoeffs(pInfo, address, pDenCoeffs[i], adcNum);
            address = address + 8;
        }
    }

    return adcStatus;
}

ADI_ADC_STATUS AdcSetHpfCoeffFix(ADI_ADC_INFO *pInfo, uint32_t *pCoeffs, int8_t adcNum)
{
    int32_t i;
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    uint16_t address = ADDR_ADEMA127_DSP_RAM_ALL_HPF_COEFF_B0_LO;
    uint32_t *pDenCoeff = &pCoeffs[ADEMA12x_NUM_HPF_COEFFS_NUMERATOR];

    for (i = 0; i < ADEMA12x_NUM_HPF_COEFFS_NUMERATOR; i++)
    {
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = AdcWriteDataPathConfigFix(pInfo, address, pCoeffs[i], adcNum);
            address = address + 4;
        }
    }

    // Each denominator coefficient is split into two parts: AxL and AxH
    for (i = 0; i < ADEMA12x_NUM_HPF_COEFFS_DENOMINATOR * 2; i++)
    {
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = AdcWriteHpfDenCoeffs(pInfo, address, pDenCoeff[i], adcNum);
            address = address + 4;
        }
    }

    return adcStatus;
}

ADI_ADC_STATUS AdcWriteDataPathConfig(ADI_ADC_INFO *pInfo, uint16_t address, float value,
                                      int8_t adcIdx)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;

    int32_t valFix;
    uint8_t valLo, valMd, valHi;
    valFix = (int32_t)(value * ((int64_t)1 << ADEMA12x_CONFIG_FRAC_BITS));
    valLo = valFix & 0xFF;
    valMd = (valFix & 0xFF00) >> 8;
    valHi = (valFix & 0xFF0000) >> 16;

    /* Write LO value in the given ADC register address */
    status = AdcWriteRegister(pInfo, address, valLo, adcIdx);

    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        /* Write MD value in the given ADC register address */
        status = AdcWriteRegister(pInfo, address + 1, valMd, adcIdx);
    }
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        /* Write HI value in the given ADC register address */
        status = AdcWriteRegister(pInfo, address + 2, valHi, adcIdx);
    }

    return status;
}

ADI_ADC_STATUS AdcWriteDataPathConfigFix(ADI_ADC_INFO *pInfo, uint16_t address, uint32_t value,
                                         int8_t adcIdx)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;

    uint8_t valLo, valMd, valHi;
    valLo = (uint8_t)(value & 0xFF00);
    valMd = (uint8_t)((value & 0xFF0000) >> 8);
    valHi = (uint8_t)((value & 0xFF000000) >> 16);

    /* Write LO value in the given ADC register address */
    status = AdcWriteRegister(pInfo, address, valLo, adcIdx);

    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        /* Write MD value in the given ADC register address */
        status = AdcWriteRegister(pInfo, address + 1, valMd, adcIdx);
    }
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        /* Write HI value in the given ADC register address */
        status = AdcWriteRegister(pInfo, address + 2, valHi, adcIdx);
    }

    return status;
}

ADI_ADC_STATUS AdcWriteChanPhOffset(ADI_ADC_INFO *pInfo, uint16_t address, float value,
                                    int8_t adcIdx)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;

    int16_t valFix;
    uint8_t valLo, valHi;
    valFix = (int16_t)(value * ((int32_t)1 << ADEMA12x_PH_OFFSET_FRAC_BITS));
    valLo = valFix & 0xFF;
    valHi = (valFix & 0xFF00) >> 8;
    /* Write HI value in the given ADC register address */
    status = AdcWriteRegister(pInfo, address, valHi, adcIdx);

    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        /* Write LO value in the given ADC register address */
        status = AdcWriteRegister(pInfo, address + 1, valLo, adcIdx);
    }

    return status;
}

ADI_ADC_STATUS AdcWriteHpfDenCoeffs(ADI_ADC_INFO *pInfo, uint16_t address, double value,
                                    int8_t adcIdx)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;

    uint64_t valFix;
    uint8_t val0, val1, val2, val3, val4, val5;
    valFix = (uint64_t)(value * ((int64_t)1 << ADEMA12x_HPF_DEN_FRAC_BITS));
    val3 = (valFix >> 8) & 0xFF;
    val4 = (valFix >> 16) & 0xFF;
    val5 = (valFix >> 24) & 0xFF;
    val0 = 0;
    val1 = 0;
    val2 = valFix & 0xFF;

    /* Write upper 24 bits value in the given ADC register address */
    status = AdcWriteRegister(pInfo, address, val3, adcIdx);

    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = AdcWriteRegister(pInfo, address + 1, val4, adcIdx);
    }
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = AdcWriteRegister(pInfo, address + 2, val5, adcIdx);
    }

    /* Write lower 24 bits value in the given ADC register address */
    address += 4;
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = AdcWriteRegister(pInfo, address, val0, adcIdx);
    }
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = AdcWriteRegister(pInfo, address + 1, val1, adcIdx);
    }
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = AdcWriteRegister(pInfo, address + 2, val2, adcIdx);
    }

    return status;
}

ADI_ADC_STATUS AdcGetChannelGain(ADI_ADC_INFO *pInfo, uint8_t *pChanIdx, int8_t numChan,
                                 int8_t adcNum, float *pGainVal)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    int32_t i;
    uint16_t addr;
    uint16_t baseAddr = ADDR_ADEMA127_DSP_RAM_CH0_GAIN_LO;
    uint16_t addrStep = 0x40;

    for (i = 0; i < numChan; i++)
    {
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            addr = baseAddr + ((uint16_t)pChanIdx[i] * addrStep);
            // Write ADC Channel Gain value
            adcStatus = AdcReadDataPathConfig(pInfo, addr, adcNum, &pGainVal[i]);
        }
    }

    return adcStatus;
}

ADI_ADC_STATUS AdcGetChannelXtGain(ADI_ADC_INFO *pInfo, uint8_t *pChanIdx, int8_t numChan,
                                   int8_t adcNum, float *pXtGainVal)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    int32_t i;
    uint16_t addr;
    uint16_t baseAddr = ADDR_ADEMA127_DSP_RAM_CH0_XT_GAIN_LO;
    uint16_t addrStep = 0x40;

    for (i = 0; i < numChan; i++)
    {
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            addr = baseAddr + ((uint16_t)pChanIdx[i] * addrStep);
            // Read ADC Channel XT Gain value
            adcStatus = AdcReadDataPathConfig(pInfo, addr, adcNum, &pXtGainVal[i]);
        }
    }
    return adcStatus;
}

ADI_ADC_STATUS AdcGetChannelXtAggressor(ADI_ADC_INFO *pInfo, uint8_t *pChanIdx, int8_t numChan,
                                        int8_t adcNum, ADI_ADC_CHAN_XT_AGGRESSOR *pXtAggrVal)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    int32_t i;
    uint16_t addr;
    uint16_t baseAddr = ADDR_ADEMA127_DSP_RAM_CH0_XT_AGGRESSOR;
    uint16_t addrStep = 0x40;
    uint8_t *pReadVal = &pInfo->datapathReadVal.readVal[0];
    uint32_t numBytes;

    for (i = 0; i < numChan; i++)
    {
        addr = baseAddr + ((uint16_t)pChanIdx[i] * addrStep);
        // Read from XT Aggressor register
        adcStatus = AdcReadRegister(pInfo, addr, adcNum, &pReadVal[0], &numBytes);
        pXtAggrVal[i].xtAggressor =
            ((pReadVal[1] & BITM_ADEMA127_DSP_RAM_CH_XT_AGGRESSOR_XT_AGGRESSOR) >>
             BITP_ADEMA127_DSP_RAM_CH_XT_AGGRESSOR_XT_AGGRESSOR);
        pXtAggrVal[i].xtCompEn =
            ((pReadVal[1] & BITM_ADEMA127_DSP_RAM_CH_XT_AGGRESSOR_XT_COMP_EN) >>
             BITP_ADEMA127_DSP_RAM_CH_XT_AGGRESSOR_XT_COMP_EN);
    }
    return adcStatus;
}

ADI_ADC_STATUS AdcGetDataPathEnable(ADI_ADC_INFO *pInfo, uint8_t *pChanIdx, int8_t numChan,
                                    int8_t adcNum, ADI_ADC_CHAN_DATAPATH_CONFIG *pDatapathConfig)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    int32_t i;
    uint16_t addr;
    uint16_t baseAddr = ADDR_ADEMA127_MMR_DATAPATH_CONFIG_CH0;
    uint16_t addrStep = 0x1;
    uint8_t *pReadVal = &pInfo->datapathReadVal.readVal[0];
    uint32_t numBytes;

    for (i = 0; i < numChan; i++)
    {
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            addr = baseAddr + (pChanIdx[i] * addrStep);

            // Read from channel datapath config register
            adcStatus = AdcReadRegister(pInfo, addr, adcNum, &pReadVal[0], &numBytes);
            pDatapathConfig[i].gainOffsetEn =
                (pReadVal[1] >> BITP_ADEMA127_MMR_DATAPATH_CONFIG_CH0_GAIN_OFFSET_XT_EN_CH0) & 0x01;
            pDatapathConfig[i].scfEn =
                (pReadVal[1] >> BITP_ADEMA127_MMR_DATAPATH_CONFIG_CH0_SCF_EN_CH0) & 0x01;
            pDatapathConfig[i].hpfEn =
                (pReadVal[1] >> BITP_ADEMA127_MMR_DATAPATH_CONFIG_CH0_HPF_EN_CH0) & 0x01;
            pDatapathConfig[i].compFiltEn =
                (pReadVal[1] >> BITP_ADEMA127_MMR_DATAPATH_CONFIG_CH0_COMP_FLT_EN_CH0) & 0x01;
            pDatapathConfig[i].compFiltCfg =
                (pReadVal[1] >> BITP_ADEMA127_MMR_DATAPATH_CONFIG_CH0_COMP_FLT_CFG_CH0) & 0x01;
            pDatapathConfig[i].lpfEn =
                (pReadVal[1] >> BITP_ADEMA127_MMR_DATAPATH_CONFIG_CH0_LPF_EN_CH0) & 0x01;
            pDatapathConfig[i].allPassEn =
                (pReadVal[1] >> BITP_ADEMA127_MMR_DATAPATH_CONFIG_CH0_ALLPASS_EN_CH0) & 0x01;
        }
    }

    return adcStatus;
}

ADI_ADC_STATUS AdcGetDataRate(ADI_ADC_INFO *pInfo, int8_t adcNum, uint8_t *pDecimationRate,
                              uint8_t *pClkPreScaler, uint8_t *pSetDecimation)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    uint8_t *pReadVal = &pInfo->datapathReadVal.readVal[0];
    uint32_t numBytes;

    if (adcStatus == ADI_ADC_STATUS_SUCCESS)
    {

        // Write to channel datarate config register
        adcStatus =
            AdcReadRegister(pInfo, ADDR_ADEMA127_MMR_DATARATE, adcNum, &pReadVal[0], &numBytes);
        *pDecimationRate = (pReadVal[1] & BITM_ADEMA127_MMR_DATARATE_DECIMATION_RATE) >>
                           BITP_ADEMA127_MMR_DATARATE_DECIMATION_RATE;

        *pClkPreScaler = (pReadVal[1] & BITM_ADEMA127_MMR_DATARATE_ADC_CLK_PRESCALER) >>
                         BITP_ADEMA127_MMR_DATARATE_ADC_CLK_PRESCALER;

        *pSetDecimation = (pReadVal[1] & BITM_ADEMA127_MMR_DATARATE_DSP_DECIMATION_X2) >>
                          BITP_ADEMA127_MMR_DATARATE_DSP_DECIMATION_X2;
    }

    return adcStatus;
}

ADI_ADC_STATUS AdcGetChannelOffset(ADI_ADC_INFO *pInfo, uint8_t *pChanIdx, int8_t numChan,
                                   int8_t adcNum, int32_t *pOffsetVal)
{
    int32_t i;
    uint16_t addr;
    uint16_t baseAddr = ADDR_ADEMA127_DSP_RAM_CH0_OFFSET_LO;
    uint16_t addrStep = 0x40;
    uint32_t numBytes = 0;
    int32_t valFix = 0;
    uint8_t *pValHi = &pInfo->datapathReadVal.valHi[0];
    uint8_t *pValMd = &pInfo->datapathReadVal.valMd[0];
    uint8_t *pValLo = &pInfo->datapathReadVal.valLo[0];
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;

    for (i = 0; i < numChan; i++)
    {
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            addr = baseAddr + (pChanIdx[i] * addrStep);
            // Read ADC Channel Offset value
            /* Read LO value in the given ADC register address */
            adcStatus = AdcReadRegister(pInfo, addr, adcNum, &pValLo[0], &numBytes);

            if (adcStatus == ADI_ADC_STATUS_SUCCESS)
            {
                /* Read MD value in the given ADC register address */
                adcStatus = AdcReadRegister(pInfo, addr + 1, adcNum, &pValMd[0], &numBytes);
            }
            if (adcStatus == ADI_ADC_STATUS_SUCCESS)
            {
                /* Read HI value in the given ADC register address */
                adcStatus = AdcReadRegister(pInfo, addr + 2, adcNum, &pValHi[0], &numBytes);
            }
            if (adcStatus == ADI_ADC_STATUS_SUCCESS)
            {
                /* Reconstruct the fixed-point value */
                valFix = (int32_t)(((pValHi[1] << 16) | (pValMd[1] << 8) | (pValLo[1])));
                /** Extend the sign bit of of 24 bit number to 32 bit valFix  */
                valFix = (valFix << 8) >> 8;
                /* Convert back to int32_t */
                pOffsetVal[i] = valFix;
            }
            else
            {
                break;
            }
        }
    }

    return adcStatus;
}

ADI_ADC_STATUS AdcGetPhaseOffset(ADI_ADC_INFO *pInfo, uint8_t *pChanIdx, int8_t numChan,
                                 int8_t adcNum, float *pPhOffsetVal)
{
    int32_t i;
    uint16_t addr;
    uint16_t baseAddr = ADDR_ADEMA127_MMR_PHASE_OFFSET_CH0_HI;
    uint16_t addrStep = 0x2;
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;

    for (i = 0; i < numChan; i++)
    {
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            addr = baseAddr + (pChanIdx[i] * addrStep);
            adcStatus = AdcReadChanPhOffset(pInfo, addr, adcNum, &pPhOffsetVal[i]);
        }
    }
    return adcStatus;
}

ADI_ADC_STATUS AdcGetDatapathAlpha(ADI_ADC_INFO *pInfo, int8_t adcNum, uint8_t *pAlphaVal)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    uint8_t *pReadVal = &pInfo->datapathReadVal.readVal[0];
    uint32_t numBytes;

    adcStatus = AdcReadRegister(pInfo, ADDR_ADEMA127_MMR_DATAPATH_ALPHA_CH0_1, adcNum, &pReadVal[0],
                                &numBytes);
    pAlphaVal[0] = pReadVal[1] & 0x0F;        // Extract lower 4 bits
    pAlphaVal[1] = (pReadVal[1] >> 4) & 0x0F; // Extract upper 4 bits

    if (adcStatus == ADI_ADC_STATUS_SUCCESS)
    {
        // Read ADC Channel 2 and Channel 3 Alpha value
        adcStatus = AdcReadRegister(pInfo, ADDR_ADEMA127_MMR_DATAPATH_ALPHA_CH2_3, adcNum,
                                    &pReadVal[0], &numBytes);
        pAlphaVal[2] = pReadVal[1] & 0x0F;
        pAlphaVal[3] = (pReadVal[1] >> 4) & 0x0F;
    }

    if (pInfo->adcCfg.pAdcType[adcNum] == ADI_ADC_TYPE_ADEMA127)
    {
        // Read ADC Channel 4 and Channel 5 Alpha value
        adcStatus = AdcReadRegister(pInfo, ADDR_ADEMA127_MMR_DATAPATH_ALPHA_CH4_5, adcNum,
                                    &pReadVal[0], &numBytes);
        pAlphaVal[4] = pReadVal[1] & 0x0F;
        pAlphaVal[5] = (pReadVal[1] >> 4) & 0x0F;

        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            // Read ADC Channel 6 Alpha value
            adcStatus = AdcReadRegister(pInfo, ADDR_ADEMA127_MMR_DATAPATH_ALPHA_CH6, adcNum,
                                        &pReadVal[0], &numBytes);
            pAlphaVal[6] = pReadVal[1] & 0x0F;
        }
    }

    return adcStatus;
}

ADI_ADC_STATUS AdcReadDataPathConfig(ADI_ADC_INFO *pInfo, uint16_t address, int8_t adcIdx,
                                     float *pValue)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    uint8_t *pValHi = &pInfo->datapathReadVal.valHi[0];
    uint8_t *pValMd = &pInfo->datapathReadVal.valMd[0];
    uint8_t *pValLo = &pInfo->datapathReadVal.valLo[0];
    uint32_t numBytes = 0;
    int32_t valFix = 0;

    /* Read LO value in the given ADC register address */
    status = AdcReadRegister(pInfo, address, adcIdx, &pValLo[0], &numBytes);

    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        /* Read MD value in the given ADC register address */
        status = AdcReadRegister(pInfo, address + 1, adcIdx, &pValMd[0], &numBytes);
    }
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        /* Read HI value in the given ADC register address */
        status = AdcReadRegister(pInfo, address + 2, adcIdx, &pValHi[0], &numBytes);
    }
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        /* Reconstruct the fixed-point value */
        valFix = (int32_t)(((pValHi[1] << 16) | (pValMd[1] << 8) | (pValLo[1])) << 8) >> 8;
        /* Convert back to float */
        *pValue = ((float)valFix) / (float)((int64_t)1 << ADEMA12x_CONFIG_FRAC_BITS);
    }

    return status;
}

ADI_ADC_STATUS AdcReadChanPhOffset(ADI_ADC_INFO *pInfo, uint16_t address, int8_t adcIdx,
                                   float *pValue)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    int16_t valFix;
    uint8_t *pValHi = &pInfo->datapathReadVal.valHi[0];
    uint8_t *pValLo = &pInfo->datapathReadVal.valLo[0];
    uint32_t numBytes;
    /* Read HI value from the given ADC register address */
    status = AdcReadRegister(pInfo, address, adcIdx, &pValHi[0], &numBytes);

    if (status == ADI_ADC_STATUS_SUCCESS)
    {

        /* Read LO value from the given ADC register address */
        status = AdcReadRegister(pInfo, address + 1, adcIdx, &pValLo[0], &numBytes);
    }

    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        /* Reconstruct the fixed-point value */
        valFix = (int16_t)(pValHi[1] << 8) | pValLo[1];
        /* Convert back to float */
        *pValue = (float)((int16_t)valFix) / (float)((int32_t)1 << ADEMA12x_PH_OFFSET_FRAC_BITS);
    }
    return status;
}

ADI_ADC_STATUS AdcGetShift(ADI_ADC_INFO *pInfo, uint8_t *pChanIdx, int8_t numChan, int8_t adcNum,
                           uint8_t *pShiftVal)
{
    int32_t i;
    uint16_t addr;
    uint16_t baseAddr = ADDR_ADEMA127_DSP_RAM_CH0_SHIFT;
    uint16_t addrStep = 0x40;
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    uint32_t numBytes;
    uint8_t *pReadVal = &pInfo->datapathReadVal.readVal[0];

    for (i = 0; i < numChan; i++)
    {
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            addr = baseAddr + (pChanIdx[i] * addrStep);
            // Write ADC Channel Shift value
            adcStatus = AdcReadRegister(pInfo, addr, adcNum, &pReadVal[0], &numBytes);
            pShiftVal[i] = pReadVal[1];
        }
    }

    return adcStatus;
}

ADI_ADC_STATUS AdcGetHpfCoeff(ADI_ADC_INFO *pInfo, int8_t adcNum, float *pNumCoeffs,
                              double *pDenCoeffs)
{
    int32_t i;
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    uint16_t address = ADDR_ADEMA127_DSP_RAM_ALL_HPF_COEFF_B0_LO;

    for (i = 0; i < ADEMA12x_NUM_HPF_COEFFS_NUMERATOR; i++)
    {
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = AdcReadDataPathConfig(pInfo, address, adcNum, &pNumCoeffs[i]);
            address = address + 4;
        }
    }

    for (i = 0; i < ADEMA12x_NUM_HPF_COEFFS_DENOMINATOR; i++)
    {
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = AdcReadHpfDenCoeffs(pInfo, address, adcNum, &pDenCoeffs[i]);
            address = address + 8;
        }
    }

    return adcStatus;
}

ADI_ADC_STATUS AdcReadHpfDenCoeffs(ADI_ADC_INFO *pInfo, uint16_t address, int8_t adcIdx,
                                   double *pValue)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    uint64_t valFix = 0;
    uint8_t *pReadVal = &pInfo->datapathReadVal.readVal[0];
    uint8_t val2, val3, val4, val5;
    uint32_t numBytes;

    /* Read upper 24 bits value from the given ADC register address */
    status = AdcReadRegister(pInfo, address, adcIdx, &pReadVal[0], &numBytes);
    val3 = pReadVal[1];
    status = AdcReadRegister(pInfo, address + 1, adcIdx, &pReadVal[0], &numBytes);
    val4 = pReadVal[1];
    status = AdcReadRegister(pInfo, address + 2, adcIdx, &pReadVal[0], &numBytes);
    val5 = pReadVal[1];

    /* Read lower 24 bits value from the given ADC register address */
    address += 4;
    status = AdcReadRegister(pInfo, address + 2, adcIdx, &pReadVal[0], &numBytes);
    val2 = pReadVal[1];

    /* Reconstruct the fixed-point value */
    valFix =
        ((uint64_t)val3 << 8) | ((uint64_t)val4 << 16) | ((uint64_t)val5 << 24) | ((uint64_t)val2);

    /* Convert back to double */
    *pValue = (double)((int64_t)valFix) / ((int64_t)1 << ADEMA12x_HPF_DEN_FRAC_BITS);

    return status;
}

ADI_ADC_STATUS AdcSetScfCoeff(ADI_ADC_INFO *pInfo, float *pCoeffs[], uint8_t *pChanIdx,
                              int8_t numChan, int8_t adcNum)
{
    int32_t i, chan;
    uint16_t addr;
    uint16_t baseAddr = ADDR_ADEMA127_DSP_RAM_CH0_SCF_APF_COEFF_B0_LO;
    uint16_t addrStep = 0x40;
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;

    for (chan = 0; chan < numChan; chan++)
    {
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            addr = baseAddr + (pChanIdx[chan] * addrStep);
            for (i = 0; i < ADEMA12x_NUM_SCF_COEFFS; i++)
            {
                if (adcStatus == ADI_ADC_STATUS_SUCCESS)
                {
                    adcStatus = AdcWriteDataPathConfig(pInfo, addr, pCoeffs[chan][i], adcNum);
                    addr = addr + 4;
                }
            }
        }
    }

    return adcStatus;
}

ADI_ADC_STATUS AdcGetScfCoeff(ADI_ADC_INFO *pInfo, uint8_t *pChanIdx, uint8_t numChan,
                              int8_t adcNum, float *pCoeffs[])
{
    int32_t i, chan;
    uint16_t addr;
    uint16_t baseAddr = ADDR_ADEMA127_DSP_RAM_CH0_SCF_APF_COEFF_B0_LO;
    uint16_t addrStep = 0x40;
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;

    for (chan = 0; chan < numChan; chan++)
    {
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            addr = baseAddr + (pChanIdx[chan] * addrStep);
            for (i = 0; i < ADEMA12x_NUM_SCF_COEFFS; i++)
            {
                if (adcStatus == ADI_ADC_STATUS_SUCCESS)
                {
                    adcStatus = AdcReadDataPathConfig(pInfo, addr, adcNum, &pCoeffs[chan][i]);
                    addr = addr + 4;
                }
            }
        }
    }

    return adcStatus;
}

ADI_ADC_STATUS AdcGetLpfCoeff(ADI_ADC_INFO *pInfo, int8_t adcNum, float *pCoeffs)
{
    int32_t i;
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    uint16_t address = ADDR_ADEMA127_DSP_RAM_ALL_LPF_COEFF_B0_LO;

    for (i = 0; i < ADEMA12x_NUM_LPF_COEFFS; i++)
    {
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            adcStatus = AdcReadDataPathConfig(pInfo, address, adcNum, &pCoeffs[i]);
            address = address + 4;
        }
    }

    return adcStatus;
}

ADI_ADC_STATUS AdcGetCompCoeff(ADI_ADC_INFO *pInfo, uint8_t *pChanIdx, uint8_t numChan,
                               int8_t adcNum, float *pCoeffs[])
{
    int32_t i, chan;
    uint16_t addr;
    uint16_t baseAddr = ADDR_ADEMA127_DSP_RAM_CH0_COMP_COEFF_B0_LO;
    uint16_t addrStep = 0x40;
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;

    for (chan = 0; chan < numChan; chan++)
    {
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            addr = baseAddr + (pChanIdx[chan] * addrStep);
            for (i = 0; i < ADEMA12x_NUM_COMP_COEFFS; i++)
            {
                if (adcStatus == ADI_ADC_STATUS_SUCCESS)
                {
                    adcStatus = AdcReadDataPathConfig(pInfo, addr, adcNum, &pCoeffs[chan][i]);
                    addr = addr + 4;
                }
            }
        }
    }

    return adcStatus;
}
