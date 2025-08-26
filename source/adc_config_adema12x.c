/******************************************************************************
 Copyright (c) 2024 - 2025  Analog Devices Inc.
******************************************************************************/

/*============= I N C L U D E S =============*/
#include "adc_config_adema12x.h"
#include "ADEMA127_addr_def.h"
#include "ADEMA127_addr_rdef.h"
/*=============  C O D E  =============*/

static ADI_ADC_STATUS PopulateInitRegisters(ADC_CONFIG_REGISTERS *pConfigReg);
static ADI_ADC_STATUS PopulateUserConfigRegisters(ADC_CONFIG_REGISTERS *pConfigReg,
                                                  ADI_ADC_CONFIG_REGISTERS *pConfigRegisters);
static ADI_ADC_STATUS CheckRegisterStatusAdema124(uint8_t status0, uint8_t status1);
static ADI_ADC_STATUS CheckRegisterStatusAdema127(uint8_t status0, uint8_t status1);
static ADI_ADC_STATUS CheckRegisterStatusShortAdema12x(uint8_t status0, uint8_t status1);
static ADI_ADC_STATUS SetFrameFormatAdema127(ADI_ADC_HANDLE hAdc, uint8_t format);
static ADI_ADC_STATUS SetFrameFormatAdema124(ADI_ADC_HANDLE hAdc, uint8_t format);

ADI_ADC_STATUS AdcInitAdema127(ADC_TYPE_CONFIG *pTypeConfig)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;

    status = PopulateInitRegisters(&pTypeConfig->configReg);

    pTypeConfig->samplesPerFrame = 7;
    pTypeConfig->frameLength = ADI_ADC_LONG_FRAME_NBYTES_ADEMA127;

    pTypeConfig->cmdOffset =
        OFFSETOF(ADI_ADC_CMD_LONG_FRAME_ADEMA127,
                 cmd); // Offset calculated based on 4-byte alignment (uint32_t buffer)
    pTypeConfig->status0Offset = OFFSETOF(ADI_ADC_RESPONSE_LONG_FRAME_ADEMA127, status0);
    pTypeConfig->status1Offset = OFFSETOF(ADI_ADC_RESPONSE_LONG_FRAME_ADEMA127, status1);
    pTypeConfig->dataOffset = OFFSETOF(ADI_ADC_RESPONSE_LONG_FRAME_ADEMA127, data);

    pTypeConfig->pfSetFrameFormat = SetFrameFormatAdema127;
    pTypeConfig->pfCheckStatus = CheckRegisterStatusAdema127;
    pTypeConfig->pfPopulateUserCfgReg = PopulateUserConfigRegisters;

    return status;
}

ADI_ADC_STATUS AdcInitAdema124(ADC_TYPE_CONFIG *pTypeConfig)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    status = PopulateInitRegisters(&pTypeConfig->configReg);

    pTypeConfig->frameLength = ADI_ADC_LONG_FRAME_NBYTES_ADEMA124;
    pTypeConfig->samplesPerFrame = 4;
    pTypeConfig->cmdOffset = OFFSETOF(ADI_ADC_CMD_LONG_FRAME_ADEMA124, cmd);
    pTypeConfig->pfCheckStatus = CheckRegisterStatusAdema124;
    pTypeConfig->status0Offset = OFFSETOF(ADI_ADC_RESPONSE_LONG_FRAME_ADEMA124, status0);
    pTypeConfig->status1Offset = OFFSETOF(ADI_ADC_RESPONSE_LONG_FRAME_ADEMA124, status1);
    pTypeConfig->dataOffset = OFFSETOF(ADI_ADC_RESPONSE_LONG_FRAME_ADEMA124, data);
    pTypeConfig->pfSetFrameFormat = SetFrameFormatAdema124;
    pTypeConfig->pfPopulateUserCfgReg = PopulateUserConfigRegisters;

    return status;
}

ADI_ADC_STATUS PopulateInitRegisters(ADC_CONFIG_REGISTERS *pConfigReg)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;

    pConfigReg->mask2.addr = ADDR_ADEMA127_MMR_MASK2;
    pConfigReg->mask2.value = 0x2;

    pConfigReg->syncSnap.addr = ADDR_ADEMA127_MMR_SYNC_SNAP;
    pConfigReg->syncSnap.value = BITM_ADEMA127_MMR_SYNC_SNAP_ALIGN;

    pConfigReg->snapshotCountHi.addr = ADDR_ADEMA127_MMR_SNAPSHOT_COUNT_HI;
    pConfigReg->snapshotCountHi.value = 0;

    pConfigReg->configCrcMmrForce.addr = ADDR_ADEMA127_MMR_CONFIG_CRC_MMR;
    pConfigReg->configCrcMmrForce.value = BITM_ADEMA127_MMR_CONFIG_CRC_MMR_CRC_FORCE_MMR;

    pConfigReg->configCrcMmrDone.addr = ADDR_ADEMA127_MMR_CONFIG_CRC_MMR;
    pConfigReg->configCrcMmrDone.value = BITM_ADEMA127_MMR_CONFIG_CRC_MMR_CRC_DONE_MMR;

    pConfigReg->datapathConfigLock.addr = ADDR_ADEMA127_MMR_DATAPATH_CONFIG_LOCK;

    pConfigReg->configCrcMmrRetainedDone.addr = ADDR_ADEMA127_MMR_RETAINED_CONFIG_CRC_MMR_RETAINED;
    pConfigReg->configCrcMmrRetainedDone.value =
        BITM_ADEMA127_MMR_RETAINED_CONFIG_CRC_MMR_RETAINED_CRC_DONE_MMR_RETAINED;

    pConfigReg->configCrcMmrRetainedForce.addr = ADDR_ADEMA127_MMR_RETAINED_CONFIG_CRC_MMR_RETAINED;
    pConfigReg->configCrcMmrRetainedForce.value =
        BITM_ADEMA127_MMR_RETAINED_CONFIG_CRC_MMR_RETAINED_CRC_FORCE_MMR_RETAINED;

    pConfigReg->status0Crc.addr = ADDR_ADEMA127_MMR_STATUS0;
    pConfigReg->status0Crc.value = BITM_ADEMA127_MMR_STATUS0_SPI_CRC_ERR;

    pConfigReg->config0ClkOut.addr = ADDR_ADEMA127_MMR_RETAINED_CONFIG0;
    pConfigReg->config0ClkOut.value = BITM_ADEMA127_MMR_RETAINED_CONFIG0_ADC_POWER_MODE |
                                      BITM_ADEMA127_MMR_RETAINED_CONFIG0_CLKOUT_EN |
                                      BITM_ADEMA127_MMR_RETAINED_CONFIG0_CRC_EN_SPI_WRITE;

    pConfigReg->status1.addr = ADDR_ADEMA127_MMR_STATUS1;
    pConfigReg->status1.value = BITM_ADEMA127_MMR_STATUS1_SYNC_SNAP_REQUESTED;

    pConfigReg->status0.addr = ADDR_ADEMA127_MMR_STATUS0;
    pConfigReg->status0.value =
        (BITM_ADEMA127_MMR_STATUS0_CRC_CHG_MMR | BITM_ADEMA127_MMR_STATUS0_CRC_CHG_MMR_RETAINED |
         BITM_ADEMA127_MMR_STATUS0_RESET_DONE);

    pConfigReg->status2.addr = ADDR_ADEMA127_MMR_STATUS2;
    pConfigReg->status2.value = 0;

    pConfigReg->tdmConfig.addr = ADDR_ADEMA127_MMR_RETAINED_TDM_CONFIG;
    pConfigReg->tdmConfig.value = BITM_ADEMA127_MMR_RETAINED_TDM_CONFIG_TDM_DREADYB_EN | 4;
    pConfigReg->tdmThrshLsb.addr = ADDR_ADEMA127_MMR_RETAINED_TDM_THRSH_LSB;
    pConfigReg->tdmThrshLsb.value = 0;
    pConfigReg->tdmThrshMsb.addr = ADDR_ADEMA127_MMR_RETAINED_TDM_THRSH_MSB;
    pConfigReg->tdmThrshMsb.value = 4;
    return status;
}

static ADI_ADC_STATUS SetFrameFormatAdema127(ADI_ADC_HANDLE hAdc, uint8_t format)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    ADI_ADC_INFO *pInfo = (ADI_ADC_INFO *)hAdc;
    ADC_TYPE_CONFIG *pTypeConfig = pInfo->pTypeConfig;

    if (format == ADI_ADC_FRAME_FORMAT_LONG)
    {
        pTypeConfig->frameLength = ADI_ADC_LONG_FRAME_NBYTES_ADEMA127;
        pTypeConfig->cmdOffset = OFFSETOF(ADI_ADC_CMD_LONG_FRAME_ADEMA127, cmd);
        pTypeConfig->status0Offset = OFFSETOF(ADI_ADC_RESPONSE_LONG_FRAME_ADEMA127, status0);
        pTypeConfig->status1Offset = OFFSETOF(ADI_ADC_RESPONSE_LONG_FRAME_ADEMA127, status1);
        pTypeConfig->dataOffset = OFFSETOF(ADI_ADC_RESPONSE_LONG_FRAME_ADEMA127, data);
        pTypeConfig->pfCheckStatus = CheckRegisterStatusAdema127;
    }
    else
    {
        pTypeConfig->frameLength = ADI_ADC_SHORT_FRAME_NBYTES_ADEMA12X;
        pTypeConfig->cmdOffset = OFFSETOF(ADI_ADC_CMD_SHORT_FRAME_ADEMA12X, cmd);
        pTypeConfig->status0Offset = OFFSETOF(ADI_ADC_RESPONSE_SHORT_FRAME_ADEMA12X, status0);
        pTypeConfig->dataOffset = OFFSETOF(ADI_ADC_RESPONSE_SHORT_FRAME_ADEMA12X, data);
        pTypeConfig->pfCheckStatus = CheckRegisterStatusShortAdema12x;
    }

    return status;
}

static ADI_ADC_STATUS SetFrameFormatAdema124(ADI_ADC_HANDLE hAdc, uint8_t format)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    ADI_ADC_INFO *pInfo = (ADI_ADC_INFO *)hAdc;
    ADC_TYPE_CONFIG *pTypeConfig = pInfo->pTypeConfig;

    if (format == ADI_ADC_FRAME_FORMAT_LONG)
    {
        pTypeConfig->frameLength = ADI_ADC_LONG_FRAME_NBYTES_ADEMA124;
        pTypeConfig->cmdOffset = OFFSETOF(ADI_ADC_CMD_LONG_FRAME_ADEMA124, cmd);
        pTypeConfig->status0Offset = OFFSETOF(ADI_ADC_RESPONSE_LONG_FRAME_ADEMA124, status0);
        pTypeConfig->status1Offset = OFFSETOF(ADI_ADC_RESPONSE_LONG_FRAME_ADEMA124, status1);
        pTypeConfig->dataOffset = OFFSETOF(ADI_ADC_RESPONSE_LONG_FRAME_ADEMA124, data);
        pTypeConfig->pfCheckStatus = CheckRegisterStatusAdema124;
    }
    else
    {
        pTypeConfig->frameLength = ADI_ADC_SHORT_FRAME_NBYTES_ADEMA12X;
        pTypeConfig->cmdOffset = OFFSETOF(ADI_ADC_CMD_SHORT_FRAME_ADEMA12X, cmd);
        pTypeConfig->status0Offset = OFFSETOF(ADI_ADC_RESPONSE_SHORT_FRAME_ADEMA12X, status0);
        pTypeConfig->dataOffset = OFFSETOF(ADI_ADC_RESPONSE_SHORT_FRAME_ADEMA12X, data);
        pTypeConfig->pfCheckStatus = CheckRegisterStatusShortAdema12x;
    }

    return status;
}

static ADI_ADC_STATUS PopulateUserConfigRegisters(ADC_CONFIG_REGISTERS *pConfigReg,
                                                  ADI_ADC_CONFIG_REGISTERS *pConfigRegisters)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    pConfigReg->config0ClkOutStreamDbg.addr = ADDR_ADEMA127_MMR_RETAINED_CONFIG0;
    pConfigReg->config0ClkOutStreamDbg.value =
        (uint8_t)(BITM_ADEMA127_MMR_RETAINED_CONFIG0_ADC_POWER_MODE |
                  ((uint32_t)(pConfigRegisters->config0StreamDbg)
                   << BITP_ADEMA127_MMR_RETAINED_CONFIG0_STREAM_DBG) |
                  BITM_ADEMA127_MMR_RETAINED_CONFIG0_CLKOUT_EN |
                  BITM_ADEMA127_MMR_RETAINED_CONFIG0_CRC_EN_SPI_WRITE);

    pConfigReg->config0Dready.addr = ADDR_ADEMA127_MMR_RETAINED_CONFIG0;
    pConfigReg->config0Dready.value =
        (uint8_t)(BITM_ADEMA127_MMR_RETAINED_CONFIG0_ADC_POWER_MODE |
                  ((uint32_t)(pConfigRegisters->config0StreamDbg)
                   << BITP_ADEMA127_MMR_RETAINED_CONFIG0_STREAM_DBG) |
                  BITM_ADEMA127_MMR_RETAINED_CONFIG0_CRC_EN_SPI_WRITE);

    pConfigReg->datarate.addr = ADDR_ADEMA127_MMR_DATARATE;
    pConfigReg->datarate.value = pConfigRegisters->datarate;
    return status;
}

static ADI_ADC_STATUS CheckRegisterStatusAdema124(uint8_t status0, uint8_t status1)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    uint8_t status2;

    /* Check bits EFUSE_MEM_ERR, SPI_CRC_ERR, BITM_COMFLT_ERR */
    status0 =
        status0 & (BITM_ADEMA127_MMR_STATUS0_SPI_CRC_ERR | BITM_ADEMA127_MMR_STATUS0_EFUSE_MEM_ERR);
    /* Check bits V2_WAV_OVRNG, V1_WAV_OVRNG and I_WAV_OVRNG */
    status1 =
        status1 & (BITM_ADEMA127_MMR_STATUS1_V0_WAV_OVRNG | BITM_ADEMA127_MMR_STATUS1_V1_WAV_OVRNG |
                   BITM_ADEMA127_MMR_STATUS1_V2_WAV_OVRNG | BITM_ADEMA127_MMR_STATUS1_V3_WAV_OVRNG);
    status2 = status0 & BITM_ADEMA127_MMR_STATUS0_STATUS2X;

    if ((status0 != 0) || (status1 != 0) || (status2 != 0))
    {
        status = ADI_ADC_STATUS_CONFIGURE_FAILED;
    }

    return status;
}
static ADI_ADC_STATUS CheckRegisterStatusAdema127(uint8_t status0, uint8_t status1)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    uint8_t status2;

    /* Check bits EFUSE_MEM_ERR, SPI_CRC_ERR, BITM_COMFLT_ERR */
    status0 =
        status0 & (BITM_ADEMA127_MMR_STATUS0_SPI_CRC_ERR | BITM_ADEMA127_MMR_STATUS0_EFUSE_MEM_ERR);
    /* Check bits V2_WAV_OVRNG, V1_WAV_OVRNG and I_WAV_OVRNG */
    status1 =
        status1 & (BITM_ADEMA127_MMR_STATUS1_V0_WAV_OVRNG | BITM_ADEMA127_MMR_STATUS1_V1_WAV_OVRNG |
                   BITM_ADEMA127_MMR_STATUS1_V2_WAV_OVRNG | BITM_ADEMA127_MMR_STATUS1_V3_WAV_OVRNG |
                   BITM_ADEMA127_MMR_STATUS1_V4_WAV_OVRNG | BITM_ADEMA127_MMR_STATUS1_V5_WAV_OVRNG |
                   BITM_ADEMA127_MMR_STATUS1_V6_WAV_OVRNG);
    status2 = status0 & BITM_ADEMA127_MMR_STATUS0_STATUS2X;

    if ((status0 != 0) || (status1 != 0) || (status2 != 0))
    {
        status = ADI_ADC_STATUS_CONFIGURE_FAILED;
    }

    return status;
}

static ADI_ADC_STATUS CheckRegisterStatusShortAdema12x(uint8_t status0, uint8_t status1)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    uint8_t status2;
    // To remove unused parameter warning
    (void)status1;

    /* Check bits EFUSE_MEM_ERR, SPI_CRC_ERR, BITM_COMFLT_ERR */
    status0 =
        status0 & (BITM_ADEMA127_MMR_STATUS0_SPI_CRC_ERR | BITM_ADEMA127_MMR_STATUS0_EFUSE_MEM_ERR);

    status2 = status0 & BITM_ADEMA127_MMR_STATUS0_STATUS2X;

    if ((status0 != 0) || (status2 != 0))
    {
        status = ADI_ADC_STATUS_CONFIGURE_FAILED;
    }

    return status;
}
