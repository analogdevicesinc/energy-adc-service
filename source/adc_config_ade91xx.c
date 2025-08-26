/******************************************************************************
 Copyright (c) 2024 - 2025  Analog Devices Inc.
******************************************************************************/

/*============= I N C L U D E S =============*/
#include "adc_config_ade91xx.h"
#include "ADE911X_addr_def.h"
#include "ADE911X_addr_rdef.h"
#include "adc_private.h"

/*=============  C O D E  =============*/
static ADI_ADC_STATUS PopulateInitRegisters(ADC_CONFIG_REGISTERS *pConfigReg);
static ADI_ADC_STATUS PopulateUserConfigRegisters(ADC_CONFIG_REGISTERS *pConfigReg,
                                                  ADI_ADC_CONFIG_REGISTERS *pConfigRegisters);
static ADI_ADC_STATUS CheckRegisterStatusAde91xx(uint8_t status0, uint8_t status1);
static ADI_ADC_STATUS CheckRegisterStatusShortAde91xx(uint8_t status0, uint8_t status1);
static ADI_ADC_STATUS SetFrameFormatAde91xx(ADI_ADC_HANDLE hAdc, uint8_t format);

ADI_ADC_STATUS AdcInitAde91xx(ADC_TYPE_CONFIG *pTypeConfig)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;

    status = PopulateInitRegisters(&pTypeConfig->configReg);
    pTypeConfig->samplesPerFrame = 3;
    pTypeConfig->frameLength = ADI_ADC_LONG_FRAME_NBYTES_ADE91XX;
    pTypeConfig->cmdOffset = OFFSETOF(ADI_ADC_CMD_LONG_FRAME_ADE91XX, cmd);
    pTypeConfig->pfCheckStatus = CheckRegisterStatusAde91xx;
    pTypeConfig->status0Offset = OFFSETOF(ADI_ADC_RESPONSE_LONG_FRAME_ADE91XX, status0);
    pTypeConfig->status1Offset = OFFSETOF(ADI_ADC_RESPONSE_LONG_FRAME_ADE91XX, status1);
    pTypeConfig->dataOffset = OFFSETOF(ADI_ADC_RESPONSE_LONG_FRAME_ADE91XX, data);
    pTypeConfig->pfSetFrameFormat = SetFrameFormatAde91xx;
    pTypeConfig->pfPopulateUserCfgReg = PopulateUserConfigRegisters;

    return status;
}

static ADI_ADC_STATUS SetFrameFormatAde91xx(ADI_ADC_HANDLE hAdc, uint8_t format)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    ADI_ADC_INFO *pInfo = (ADI_ADC_INFO *)hAdc;
    ADC_TYPE_CONFIG *pTypeConfig = pInfo->pTypeConfig;

    if (format == ADI_ADC_FRAME_FORMAT_LONG)
    {
        pTypeConfig->frameLength = ADI_ADC_LONG_FRAME_NBYTES_ADE91XX;
        pTypeConfig->cmdOffset = OFFSETOF(ADI_ADC_CMD_LONG_FRAME_ADE91XX, cmd);
        pTypeConfig->status0Offset = OFFSETOF(ADI_ADC_RESPONSE_LONG_FRAME_ADE91XX, status0);
        pTypeConfig->status1Offset = OFFSETOF(ADI_ADC_RESPONSE_LONG_FRAME_ADE91XX, status1);
        pTypeConfig->dataOffset = OFFSETOF(ADI_ADC_RESPONSE_LONG_FRAME_ADE91XX, data);
        pTypeConfig->pfCheckStatus = CheckRegisterStatusAde91xx;
    }
    else
    {
        pTypeConfig->frameLength = ADI_ADC_SHORT_FRAME_NBYTES_ADE91XX;
        pTypeConfig->cmdOffset = OFFSETOF(ADI_ADC_CMD_SHORT_FRAME_ADE91XX, cmd);
        pTypeConfig->status0Offset = OFFSETOF(ADI_ADC_RESPONSE_SHORT_FRAME_ADE91XX, status0);
        pTypeConfig->dataOffset = OFFSETOF(ADI_ADC_RESPONSE_SHORT_FRAME_ADE91XX, data);
        pTypeConfig->pfCheckStatus = CheckRegisterStatusShortAde91xx;
    }

    return status;
}

static ADI_ADC_STATUS PopulateInitRegisters(ADC_CONFIG_REGISTERS *pConfigReg)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;

    pConfigReg->mask2.addr = ADDR_ADE911X_MAP0_MASK2;
    pConfigReg->mask2.value = 0xFF;

    pConfigReg->syncSnap.addr = ADDR_ADE911X_MAP0_SYNC_SNAP;
    pConfigReg->syncSnap.value = BITM_ADE911X_MAP0_SYNC_SNAP_ALIGN;

    pConfigReg->snapshotCountHi.addr = ADDR_ADE911X_MAP0_SNAPSHOT_COUNT_HI;
    pConfigReg->snapshotCountHi.value = 0;

    pConfigReg->configCrcMmrForce.addr = ADDR_ADE911X_MAP0_CONFIG_CRC;
    pConfigReg->configCrcMmrForce.value = BITM_ADE911X_MAP0_CONFIG_CRC_CRC_FORCE;

    pConfigReg->configCrcMmrDone.addr = ADDR_ADE911X_MAP0_CONFIG_CRC;
    pConfigReg->configCrcMmrDone.value = BITM_ADE911X_MAP0_CONFIG_CRC_CRC_DONE;

    pConfigReg->datapathConfigLock.addr = ADDR_ADE911X_MAP0_SCRATCH;

    pConfigReg->configCrcMmrRetainedDone.addr = ADDR_ADE911X_MAP0_SCRATCH;
    pConfigReg->configCrcMmrRetainedDone.value = 0;

    pConfigReg->configCrcMmrRetainedForce.addr = ADDR_ADE911X_MAP0_SCRATCH;
    pConfigReg->configCrcMmrRetainedForce.value = 0;

    pConfigReg->status0Crc.addr = ADDR_ADE911X_MAP0_STATUS0;
    pConfigReg->status0Crc.value = BITM_ADE911X_MAP0_STATUS0_SPI_CRC_ERR;

    pConfigReg->config0ClkOut.addr = ADDR_ADE911X_MAP0_CONFIG0;
    pConfigReg->config0ClkOut.value =
        BITM_ADE911X_MAP0_CONFIG0_CLKOUT_EN | BITM_ADE911X_MAP0_CONFIG0_CRC_EN_SPI_WRITE;

    pConfigReg->status1.addr = ADDR_ADE911X_MAP0_STATUS1;
    pConfigReg->status1.value = BITM_ADE911X_MAP0_STATUS1_ADC_SYNC_DONE;

    pConfigReg->status0.addr = ADDR_ADE911X_MAP0_STATUS0;
    pConfigReg->status0.value =
        (BITM_ADE911X_MAP0_STATUS0_CRC_CHG | BITM_ADE911X_MAP0_STATUS0_COM_UP |
         BITM_ADE911X_MAP0_STATUS0_RESET_DONE);

    pConfigReg->status2.addr = ADDR_ADE911X_MAP0_STATUS2;
    pConfigReg->status2.value = 0;

    return status;
}

static ADI_ADC_STATUS PopulateUserConfigRegisters(ADC_CONFIG_REGISTERS *pConfigReg,
                                                  ADI_ADC_CONFIG_REGISTERS *pConfigRegisters)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    pConfigReg->config0ClkOutStreamDbg.addr = ADDR_ADE911X_MAP0_CONFIG0;
    pConfigReg->config0ClkOutStreamDbg.value =
        (uint8_t)(((uint32_t)(pConfigRegisters->config0StreamDbg)
                   << BITP_ADE911X_MAP0_CONFIG0_STREAM_DBG) |
                  BITM_ADE911X_MAP0_CONFIG0_CLKOUT_EN | BITM_ADE911X_MAP0_CONFIG0_CRC_EN_SPI_WRITE);

    pConfigReg->config0Dready.addr = ADDR_ADE911X_MAP0_CONFIG0;
    pConfigReg->config0Dready.value = (uint8_t)(((uint32_t)(pConfigRegisters->config0StreamDbg)
                                                 << BITP_ADE911X_MAP0_CONFIG0_STREAM_DBG) |
                                                BITM_ADE911X_MAP0_CONFIG0_CRC_EN_SPI_WRITE);

    pConfigReg->datarate.addr = ADDR_ADE911X_MAP0_CONFIG_FILT;
    pConfigReg->datarate.value = pConfigRegisters->configFilt;
    return status;
}

static ADI_ADC_STATUS CheckRegisterStatusAde91xx(uint8_t status0, uint8_t status1)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    uint8_t status2;

    /* Check bits EFUSE_MEM_ERR, SPI_CRC_ERR, BITM_COMFLT_ERR */
    status0 =
        status0 & (BITM_ADE911X_MAP0_STATUS0_SPI_CRC_ERR | BITM_ADE911X_MAP0_STATUS0_EFUSE_MEM_ERR |
                   BITM_ADE911X_MAP0_STATUS0_COMFLT_ERR);
    /* Check bits V2_WAV_OVRNG, V1_WAV_OVRNG and I_WAV_OVRNG */
    status1 =
        status1 & (BITM_ADE911X_MAP0_STATUS1_I_WAV_OVRNG | BITM_ADE911X_MAP0_STATUS1_V1_WAV_OVRNG |
                   BITM_ADE911X_MAP0_STATUS1_V2_WAV_OVRNG);
    status2 = status0 & BITM_ADE911X_MAP0_STATUS0_STATUS2X;

    if ((status0 != 0) || (status1 != 0) || (status2 != 0))
    {
        adcStatus = ADI_ADC_STATUS_CONFIGURE_FAILED;
    }

    return adcStatus;
}

static ADI_ADC_STATUS CheckRegisterStatusShortAde91xx(uint8_t status0, uint8_t status1)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    uint8_t status2;
    // To remove unused parameter warning
    (void)status1;

    /* Check bits EFUSE_MEM_ERR, SPI_CRC_ERR, BITM_COMFLT_ERR */
    status0 =
        status0 & (BITM_ADE911X_MAP0_STATUS0_SPI_CRC_ERR | BITM_ADE911X_MAP0_STATUS0_EFUSE_MEM_ERR |
                   BITM_ADE911X_MAP0_STATUS0_COMFLT_ERR);
    status2 = status0 & BITM_ADE911X_MAP0_STATUS0_STATUS2X;

    if ((status0 != 0) || (status2 != 0))
    {
        adcStatus = ADI_ADC_STATUS_CONFIGURE_FAILED;
    }

    return adcStatus;
}
