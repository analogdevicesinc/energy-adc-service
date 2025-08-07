/******************************************************************************
 Copyright (c) 2024 - 2025  Analog Devices Inc.
******************************************************************************/

/**
 * @file     adc_configure.c
 * @brief    ADC daisy chain configuration
 * @{
 */

#include "adc_private.h"
#include "adi_adc.h"
#include "adi_adc_frame_assemble.h"
#include "adi_adc_frame_format.h"
#include <string.h>
/*============= I N C L U D E S =============*/

/**
 * @brief Enable clock out in first ADC. This is done by writing 1
 *        to bit 0 (CLKOUT_EN) of first ADC. Also configure the first
 *        ADC to provide ADC data in desired format. For getting incremental
 *        debug data bits [3:2] are set to 10. To enable CRC check on SPI
 *        writes, bit 1 is set to 1.
 *
 */
static ADI_ADC_STATUS EnableAdcClockOut(ADI_ADC_INFO *pInfo, uint8_t adcIdx);

/**
 * @brief Updates the REG_CONFIG of the ADC's other than first ADC.
 *        Configure the ADC's to provide ADC data in desired format.
 *        For getting incremental debug data bits [3:2] are set to 10.
 *        To enable CRC check on SPI writes, bit 1 is set to 1.
 *
 */
static ADI_ADC_STATUS UpdateRegisterConfig(ADI_ADC_INFO *pInfo);

/**
 * @brief Clears non-error bits of REG_STATUS in all the ADC.
 *        This ensures that we dont get status
 *        bits occured during startup during
 *        the regular run.
 *
 */
static ADI_ADC_STATUS ClearRegisterStatus(ADI_ADC_INFO *pInfo);

/**
 * @brief Read REG_STATUS from the ADC's. 5. Read the
 *        STATUS0-2 registers to see if the configurations
 *        have been done. Bit 5 (REST_DONE) of STATUS0
 *        should be set to 1.
 *
 */
static ADI_ADC_STATUS ReadRegisterStatus(ADI_ADC_INFO *pInfo);

/**
 * @brief Clears the CONFIG_CRC register.
 *        0x02 will clear the register since
 *        CRC_DONE bit is W1C.
 */
static ADI_ADC_STATUS ClearConfigCrc(ADI_ADC_INFO *pInfo);

/**
 * @brief Check ADC status registers are as expected
 *        and no error bits are set.
 */
static ADI_ADC_STATUS CheckRegisterStatus(ADI_ADC_INFO *pInfo);

/**
 * @brief Clear the SPI_CRC_ERR bit of STATUS0 register.
 *        Response for this command transmission is ignored,
 *        because this transmission is done to receive and
 *        discard any corrupted packets sent from the ADCs.
 */
static ADI_ADC_STATUS ClearStatus0Crc(ADI_ADC_INFO *pInfo);

/**
 * @brief Set all bits in MASK2 register so that all errors
 *        in STATUS2 register are enabled and reported in
 *        STATUS2X bit of STATUS0 register.
 */
static ADI_ADC_STATUS SetRegisterMask2(ADI_ADC_INFO *pInfo);

static ADI_ADC_STATUS PopulateAdcClockOutConfig(ADI_ADC_INFO *pInfo, uint8_t adcIdx);
static ADI_ADC_STATUS PopulateRegisterConfig(ADI_ADC_INFO *pInfo);
static ADI_ADC_STATUS PopulateRegisterMask2(ADI_ADC_INFO *pInfo);
static ADI_ADC_STATUS PopulateAlignAdc(ADI_ADC_INFO *pInfo);
static ADI_ADC_STATUS PopulateTdmConfig(ADI_ADC_INFO *pInfo);
static ADI_ADC_STATUS PopulateTdmThreshold(ADI_ADC_INFO *pInfo);
static ADI_ADC_STATUS PopulateRegisterStatus(ADI_ADC_INFO *pInfo);
static ADI_ADC_STATUS PopulateConfigFilt(ADI_ADC_INFO *pInfo);
static ADI_ADC_STATUS PopulateClearStatus0Crc(ADI_ADC_INFO *pInfo);
static ADI_ADC_STATUS PopulateConfigCrcMmrForce(ADI_ADC_INFO *pInfo);
static ADI_ADC_STATUS PopulateConfigCrcMmrRetainedForce(ADI_ADC_INFO *pInfo);
static ADI_ADC_STATUS PopulateConfigCrcMmrDone(ADI_ADC_INFO *pInfo);
static ADI_ADC_STATUS PopulateConfigCrcMmrRetainedDone(ADI_ADC_INFO *pInfo);
static ADI_ADC_STATUS PopulateDatapathConfigLock(ADI_ADC_INFO *pInfo, uint8_t value);
static ADI_ADC_STATUS PopulateClearStatus1(ADI_ADC_INFO *pInfo);
static ADI_ADC_STATUS PopulateClearStatus0(ADI_ADC_INFO *pInfo);

/*=============  D E F I N I T I O N S  =============*/
ADI_ADC_STATUS AdcEnableClockOut(ADI_ADC_INFO *pInfo, uint8_t adcIdx)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;

    uint8_t numAdc = pInfo->adcCfg.numAdc;

    if (numAdc != 1)
    {
        /* Clear STATUS0.SPI_CRC_ERR */
        status = ClearStatus0Crc(pInfo);
        if (status == ADI_ADC_STATUS_SUCCESS)
        {
            status = EnableAdcClockOut(pInfo, adcIdx);
        }
    }

    return status;
}

ADI_ADC_STATUS AdcConfigure(ADI_ADC_INFO *pInfo, ADI_ADC_CONFIG_REGISTERS *pConfigReg)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    uint8_t numAdc = pInfo->adcCfg.numAdc;

    int32_t idx;
    for (idx = 0; idx < numAdc; idx++)
    {
        if (status == ADI_ADC_STATUS_SUCCESS)
        {
            status = pInfo->pTypeConfig[idx].pfPopulateUserCfgReg(
                &pInfo->pTypeConfig[idx].configReg, &pConfigReg[idx]);
        }
    }

    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        /* Clear STATUS0.SPI_CRC_ERR */
        status = ClearStatus0Crc(pInfo);
    }
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = SetRegisterMask2(pInfo);
    }
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = UpdateRegisterConfig(pInfo);
    }
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = AdcSetSamplingRate(pInfo);
    }
    if ((status == ADI_ADC_STATUS_SUCCESS) && (numAdc != 1))
    {
        status = AlignAdc(pInfo);
    }
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = ReadRegisterStatus(pInfo);
    }
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = ClearConfigCrc(pInfo);
    }
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = ClearRegisterStatus(pInfo);
    }
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = UpdateTdmConfig(pInfo);
    }
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = SetTdmThreshold(pInfo);
    }
    AdcAssembleNopAllAdc(pInfo, &pInfo->pTxFramePtr[0]);

    memcpy(&pInfo->configReg, &pConfigReg, sizeof(ADI_ADC_CONFIG_REGISTERS));

    return status;
}

static ADI_ADC_STATUS EnableAdcClockOut(ADI_ADC_INFO *pInfo, uint8_t adcIdx)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;

    status = PopulateAdcClockOutConfig(pInfo, adcIdx);

    /* Write to the device chain, enabling clkout on specified device.
     * Other devices will not receive data until they receive clock.
     */

    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = TransferAdcData(pInfo);
        /* Ignore response since other ADCs are not ready */
        if (status != ADI_ADC_STATUS_SUCCESS)
        {
            /* Coverity Fix */
            status = ADI_ADC_STATUS_SUCCESS;
        }
    }

    return status;
}

static ADI_ADC_STATUS UpdateRegisterConfig(ADI_ADC_INFO *pInfo)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;

    status = PopulateRegisterConfig(pInfo);

    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = TransferAdcData(pInfo);
        if (status != ADI_ADC_STATUS_SUCCESS)
        {
            status = ADI_ADC_STATUS_CONFIGURE_FAILED;
        }
    }

    return status;
}

static ADI_ADC_STATUS ClearRegisterStatus(ADI_ADC_INFO *pInfo)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;

    status = PopulateClearStatus1(pInfo);
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = TransferAdcData(pInfo);
        if (status != ADI_ADC_STATUS_SUCCESS)
        {
            status = ADI_ADC_STATUS_CONFIGURE_FAILED;
        }
    }

    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = PopulateClearStatus0(pInfo);
        if (status == ADI_ADC_STATUS_SUCCESS)
        {
            status = TransferAdcData(pInfo);
            if (status != ADI_ADC_STATUS_SUCCESS)
            {
                status = ADI_ADC_STATUS_CONFIGURE_FAILED;
            }
        }
    }

    return status;
}

ADI_ADC_STATUS AlignAdc(ADI_ADC_INFO *pInfo)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;

    status = PopulateAlignAdc(pInfo);

    /* Synchronize with align operation */
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = TransferAdcData(pInfo);
        if (status == ADI_ADC_STATUS_SUCCESS)
        {
            AdcAssembleNopAllAdc(pInfo, &pInfo->pTxCmdFramePtr[0]);
            status = TransferAdcData(pInfo);
        }
    }

    return status;
}

ADI_ADC_STATUS UpdateTdmConfig(ADI_ADC_INFO *pInfo)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;

    status = PopulateTdmConfig(pInfo);

    /* Synchronize with align operation */
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = TransferAdcData(pInfo);
        if (status == ADI_ADC_STATUS_SUCCESS)
        {
            AdcAssembleNopAllAdc(pInfo, &pInfo->pTxCmdFramePtr[0]);
            status = TransferAdcData(pInfo);
        }
    }

    return status;
}

ADI_ADC_STATUS SetTdmThreshold(ADI_ADC_INFO *pInfo)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;

    status = PopulateTdmThreshold(pInfo);

    /* Synchronize with align operation */
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = TransferAdcData(pInfo);
        if (status == ADI_ADC_STATUS_SUCCESS)
        {
            AdcAssembleNopAllAdc(pInfo, &pInfo->pTxCmdFramePtr[0]);
            status = TransferAdcData(pInfo);
        }
    }

    return status;
}

static ADI_ADC_STATUS ReadRegisterStatus(ADI_ADC_INFO *pInfo)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;

    PopulateRegisterStatus(pInfo);

    /* Read STATUS0-2. STATUS0 and STATUS1 always included in LONG frame
     * response. */

    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = TransferAdcData(pInfo);
        if (status == ADI_ADC_STATUS_SUCCESS)
        {
            AdcAssembleNopAllAdc(pInfo, &pInfo->pTxCmdFramePtr[0]);
            status = TransferAdcData(pInfo);
        }
        if (status == ADI_ADC_STATUS_SUCCESS)
        {
            status = CheckRegisterStatus(pInfo);
        }
    }

    return status;
}

static ADI_ADC_STATUS CheckRegisterStatus(ADI_ADC_INFO *pInfo)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    int32_t adcIdx;
    ADI_ADC_CONFIG *pAdcCfg = &pInfo->adcCfg;
    volatile ADI_ADC_RX_BUFFER *pRxBuffer = &pInfo->rxBuffer;
    uint8_t numAdc = pAdcCfg->numAdc;
    ADC_TYPE_CONFIG *pTypeConfig = pInfo->pTypeConfig;
    volatile uint8_t *pRxFrames = &pRxBuffer->pAdcRxFrames[0];
    uint8_t status0;
    uint8_t status1;

    for (adcIdx = 0; adcIdx < numAdc; adcIdx++)
    {
        status0 = pRxFrames[pTypeConfig[adcIdx].status0Offset];
        status1 = pRxFrames[pTypeConfig[adcIdx].status1Offset];
        pRxFrames += pTypeConfig[adcIdx].frameLength;
        adcStatus = pTypeConfig[adcIdx].pfCheckStatus(status0, status1);
    }

    return adcStatus;
}

ADI_ADC_STATUS AdcSetSamplingRate(ADI_ADC_INFO *pInfo)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;

    status = PopulateDatapathConfigLock(pInfo, 0);

    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = TransferAdcData(pInfo);
        if (status != ADI_ADC_STATUS_SUCCESS)
        {
            status = ADI_ADC_STATUS_CONFIGURE_FAILED;
        }
    }

    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        PopulateConfigFilt(pInfo);
        if (status == ADI_ADC_STATUS_SUCCESS)
        {
            status = TransferAdcData(pInfo);
            if (status != ADI_ADC_STATUS_SUCCESS)
            {
                status = ADI_ADC_STATUS_CONFIGURE_FAILED;
            }
        }
    }

    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = PopulateDatapathConfigLock(pInfo, 1);

        if (status == ADI_ADC_STATUS_SUCCESS)
        {
            status = TransferAdcData(pInfo);
            if (status != ADI_ADC_STATUS_SUCCESS)
            {
                status = ADI_ADC_STATUS_CONFIGURE_FAILED;
            }
        }
    }

    return status;
}

static ADI_ADC_STATUS ClearConfigCrc(ADI_ADC_INFO *pInfo)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;

    status = PopulateConfigCrcMmrForce(pInfo);

    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        /* Status of the operation need not be checked because
         * it is expected that packets are corrupted if any
         * previous SPI transaction was not done correctly.
         * This write ensures that the corrupted packet is discarded
         * and status0.SPI_CRC_ERR bit is cleared.
         */
        TransferAdcData(pInfo);
    }

    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = PopulateConfigCrcMmrDone(pInfo);

        if (status == ADI_ADC_STATUS_SUCCESS)
        {
            /* Status of the operation need not be checked because
             * it is expected that packets are corrupted if any
             * previous SPI transaction was not done correctly.
             * This write ensures that the corrupted packet is discarded
             * and status0.SPI_CRC_ERR bit is cleared.
             */
            TransferAdcData(pInfo);
        }
    }

    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = PopulateConfigCrcMmrRetainedForce(pInfo);

        if (status == ADI_ADC_STATUS_SUCCESS)
        {
            /* Status of the operation need not be checked because
             * it is expected that packets are corrupted if any
             * previous SPI transaction was not done correctly.
             * This write ensures that the corrupted packet is discarded
             * and status0.SPI_CRC_ERR bit is cleared.
             */
            TransferAdcData(pInfo);
        }
    }

    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = PopulateConfigCrcMmrRetainedDone(pInfo);
        if (status == ADI_ADC_STATUS_SUCCESS)
        {
            /* Status of the operation need not be checked because
             * it is expected that packets are corrupted if any
             * previous SPI transaction was not done correctly.
             * This write ensures that the corrupted packet is discarded
             * and status0.SPI_CRC_ERR bit is cleared.
             */
            TransferAdcData(pInfo);
        }
    }

    return status;
}

static ADI_ADC_STATUS ClearStatus0Crc(ADI_ADC_INFO *pInfo)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;

    status = PopulateClearStatus0Crc(pInfo);

    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        /* Status of the operation need not be checked because
         * it is expected that packets are corrupted if any
         * previous SPI transaction was not done correctly.
         * This write ensures that the corrupted packet is discarded
         * and status0.SPI_CRC_ERR bit is cleared.
         */
        TransferAdcData(pInfo);
    }

    return status;
}

static ADI_ADC_STATUS SetRegisterMask2(ADI_ADC_INFO *pInfo)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    status = PopulateRegisterMask2(pInfo);

    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = TransferAdcData(pInfo);
    }

    return status;
}

static ADI_ADC_STATUS PopulateAdcClockOutConfig(ADI_ADC_INFO *pInfo, uint8_t adcIdx)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    int32_t idx;
    ADI_ADC_CONFIG *pAdcCfg = &pInfo->adcCfg;
    uint8_t numAdc = pAdcCfg->numAdc;
    ADC_TYPE_CONFIG *pTypeConfig = pInfo->pTypeConfig;
    uint8_t addr;
    uint8_t value;
    ADI_ADC_CMD *pCmd;

    for (idx = 0; idx < numAdc; idx++)
    {
        addr = pTypeConfig[idx].configReg.status2.addr;
        value = pTypeConfig[idx].configReg.status2.value;
        pCmd = (ADI_ADC_CMD *)(pInfo->pTxCmdFramePtr[idx] + pTypeConfig[idx].cmdOffset);
        status = adi_adc_AssembleCommand(pAdcCfg->pfCalcCmdCrc, ADI_ADC_RWB_READ, addr, value, pCmd,
                                         pAdcCfg->frameFormat);
        if (status != ADI_ADC_STATUS_SUCCESS)
        {
            break;
        }
    }
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        addr = pTypeConfig[adcIdx].configReg.config0ClkOut.addr;
        value = pTypeConfig[adcIdx].configReg.config0ClkOut.value;
        pCmd = (ADI_ADC_CMD *)(pInfo->pTxCmdFramePtr[adcIdx] + pTypeConfig[adcIdx].cmdOffset);
        status = adi_adc_AssembleCommand(pAdcCfg->pfCalcCmdCrc, ADI_ADC_RWB_WRITE, addr, value,
                                         pCmd, pAdcCfg->frameFormat);
    }

    return status;
}

static ADI_ADC_STATUS PopulateRegisterConfig(ADI_ADC_INFO *pInfo)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    int32_t idx;

    ADI_ADC_CONFIG *pAdcCfg = &pInfo->adcCfg;
    uint8_t numAdc = pAdcCfg->numAdc;
    ADC_TYPE_CONFIG *pTypeConfig = pInfo->pTypeConfig;
    uint8_t addr;
    uint8_t value;
    ADI_ADC_CMD *pCmd;

    if (numAdc != 1)
    {
        for (idx = 1; idx < numAdc; idx++)
        {
            addr = pTypeConfig[idx].configReg.config0ClkOutStreamDbg.addr;
            value = pTypeConfig[idx].configReg.config0ClkOutStreamDbg.value;
            pCmd = (ADI_ADC_CMD *)(pInfo->pTxCmdFramePtr[idx] + pTypeConfig[idx].cmdOffset);
            status = adi_adc_AssembleCommand(pAdcCfg->pfCalcCmdCrc, ADI_ADC_RWB_WRITE, addr, value,
                                             pCmd, pAdcCfg->frameFormat);
            if (status != ADI_ADC_STATUS_SUCCESS)
            {
                break;
            }
        }
        if (status == ADI_ADC_STATUS_SUCCESS)
        {
            addr = pTypeConfig[0].configReg.config0Dready.addr;
            value = pTypeConfig[0].configReg.config0Dready.value;
            pCmd = (ADI_ADC_CMD *)(pInfo->pTxCmdFramePtr[0] + pTypeConfig[0].cmdOffset);
            status = adi_adc_AssembleCommand(pAdcCfg->pfCalcCmdCrc, ADI_ADC_RWB_WRITE, addr, value,
                                             pCmd, pAdcCfg->frameFormat);
        }
    }
    else
    {
        idx = numAdc - 1;
        addr = pTypeConfig[idx].configReg.config0Dready.addr;
        value = pTypeConfig[idx].configReg.config0Dready.value;
        pCmd = (ADI_ADC_CMD *)(pInfo->pTxCmdFramePtr[idx] + pTypeConfig[idx].cmdOffset);
        status = adi_adc_AssembleCommand(pAdcCfg->pfCalcCmdCrc, ADI_ADC_RWB_WRITE, addr, value,
                                         pCmd, pAdcCfg->frameFormat);
    }

    return status;
}

static ADI_ADC_STATUS PopulateRegisterMask2(ADI_ADC_INFO *pInfo)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    int32_t idx;
    ADI_ADC_CONFIG *pAdcCfg = &pInfo->adcCfg;
    uint8_t numAdc = pAdcCfg->numAdc;
    ADC_TYPE_CONFIG *pTypeConfig = pInfo->pTypeConfig;
    uint8_t addr;
    uint8_t value;
    ADI_ADC_CMD *pCmd;

    for (idx = 0; idx < numAdc; idx++)
    {
        addr = pTypeConfig[idx].configReg.mask2.addr;
        value = pTypeConfig[idx].configReg.mask2.value;
        pCmd = (ADI_ADC_CMD *)(pInfo->pTxCmdFramePtr[idx] + pTypeConfig[idx].cmdOffset);
        status = adi_adc_AssembleCommand(pAdcCfg->pfCalcCmdCrc, ADI_ADC_RWB_WRITE, addr, value,
                                         pCmd, pAdcCfg->frameFormat);
        if (status != ADI_ADC_STATUS_SUCCESS)
        {
            break;
        }
    }

    return status;
}

static ADI_ADC_STATUS PopulateAlignAdc(ADI_ADC_INFO *pInfo)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    int32_t idx;
    ADI_ADC_CONFIG *pAdcCfg = &pInfo->adcCfg;
    ADC_TYPE_CONFIG *pTypeConfig = pInfo->pTypeConfig;
    uint8_t numAdc = pAdcCfg->numAdc;
    uint8_t addr;
    uint8_t value;
    ADI_ADC_CMD *pCmd;

    for (idx = 0; idx < numAdc; idx++)
    {
        addr = pTypeConfig[idx].configReg.syncSnap.addr;
        value = pTypeConfig[idx].configReg.syncSnap.value;
        pCmd = (ADI_ADC_CMD *)(pInfo->pTxCmdFramePtr[idx] + pTypeConfig[idx].cmdOffset);
        status = adi_adc_AssembleCommand(pAdcCfg->pfCalcCmdCrc, ADI_ADC_RWB_WRITE, addr, value,
                                         pCmd, pAdcCfg->frameFormat);
        if (status != ADI_ADC_STATUS_SUCCESS)
        {
            break;
        }
    }

    return status;
}

static ADI_ADC_STATUS PopulateTdmConfig(ADI_ADC_INFO *pInfo)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    int32_t idx;
    ADI_ADC_CONFIG *pAdcCfg = &pInfo->adcCfg;
    ADC_TYPE_CONFIG *pTypeConfig = pInfo->pTypeConfig;
    uint8_t numAdc = pAdcCfg->numAdc;
    uint8_t addr;
    uint8_t value;
    ADI_ADC_CMD *pCmd;

    for (idx = 0; idx < numAdc; idx++)
    {
        addr = pTypeConfig[idx].configReg.tdmConfig.addr;
        value = pTypeConfig[idx].configReg.tdmConfig.value;
        pCmd = (ADI_ADC_CMD *)(pInfo->pTxCmdFramePtr[idx] + pTypeConfig[idx].cmdOffset);
        status = adi_adc_AssembleCommand(pAdcCfg->pfCalcCmdCrc, ADI_ADC_RWB_WRITE, addr, value,
                                         pCmd, pAdcCfg->frameFormat);
        if (status != ADI_ADC_STATUS_SUCCESS)
        {
            break;
        }
    }
    return status;
}

static ADI_ADC_STATUS PopulateTdmThreshold(ADI_ADC_INFO *pInfo)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    int32_t idx;
    ADI_ADC_CONFIG *pAdcCfg = &pInfo->adcCfg;
    ADC_TYPE_CONFIG *pTypeConfig = pInfo->pTypeConfig;
    uint8_t numAdc = pAdcCfg->numAdc;
    uint8_t addr;
    uint8_t value;
    ADI_ADC_CMD *pCmd;

    for (idx = 0; idx < numAdc; idx++)
    {
        addr = pTypeConfig[idx].configReg.tdmThrshMsb.addr;
        value = pTypeConfig[idx].configReg.tdmThrshMsb.value;
        pCmd = (ADI_ADC_CMD *)(pInfo->pTxCmdFramePtr[idx] + pTypeConfig[idx].cmdOffset);
        status = adi_adc_AssembleCommand(pAdcCfg->pfCalcCmdCrc, ADI_ADC_RWB_WRITE, addr, value,
                                         pCmd, pAdcCfg->frameFormat);
        if (status != ADI_ADC_STATUS_SUCCESS)
        {
            break;
        }
    }
    return status;
}

static ADI_ADC_STATUS PopulateRegisterStatus(ADI_ADC_INFO *pInfo)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    int32_t idx;
    ADI_ADC_CONFIG *pAdcCfg = &pInfo->adcCfg;
    uint8_t numAdc = pAdcCfg->numAdc;
    uint8_t addr;
    uint8_t value;
    ADC_TYPE_CONFIG *pTypeConfig = pInfo->pTypeConfig;
    ADI_ADC_CMD *pCmd;

    for (idx = 0; idx < numAdc; idx++)
    {
        addr = pTypeConfig[idx].configReg.status2.addr;
        value = pTypeConfig[idx].configReg.status2.value;
        pCmd = (ADI_ADC_CMD *)(pInfo->pTxCmdFramePtr[idx] + pTypeConfig[idx].cmdOffset);
        status = adi_adc_AssembleCommand(pAdcCfg->pfCalcCmdCrc, ADI_ADC_RWB_READ, addr, value, pCmd,
                                         pAdcCfg->frameFormat);
        if (status != ADI_ADC_STATUS_SUCCESS)
        {
            break;
        }
    }
    return status;
}

static ADI_ADC_STATUS PopulateClearStatus0Crc(ADI_ADC_INFO *pInfo)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    int32_t idx;
    ADI_ADC_CONFIG *pAdcCfg = &pInfo->adcCfg;
    uint8_t numAdc = pAdcCfg->numAdc;
    uint8_t addr;
    uint8_t value;
    ADC_TYPE_CONFIG *pTypeConfig = pInfo->pTypeConfig;
    ADI_ADC_CMD *pCmd;

    for (idx = 0; idx < numAdc; idx++)
    {
        addr = pTypeConfig[idx].configReg.status0Crc.addr;
        value = pTypeConfig[idx].configReg.status0Crc.value;
        pCmd = (ADI_ADC_CMD *)(pInfo->pTxCmdFramePtr[idx] + pTypeConfig[idx].cmdOffset);
        status = adi_adc_AssembleCommand(pAdcCfg->pfCalcCmdCrc, ADI_ADC_RWB_WRITE, addr, value,
                                         pCmd, pAdcCfg->frameFormat);
        if (status != ADI_ADC_STATUS_SUCCESS)
        {
            break;
        }
    }

    return status;
}

static ADI_ADC_STATUS PopulateConfigCrcMmrRetainedForce(ADI_ADC_INFO *pInfo)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    int32_t idx;
    ADI_ADC_CONFIG *pAdcCfg = &pInfo->adcCfg;
    uint8_t numAdc = pAdcCfg->numAdc;
    uint8_t addr;
    uint8_t value;
    ADC_TYPE_CONFIG *pTypeConfig = pInfo->pTypeConfig;
    ADI_ADC_CMD *pCmd;

    for (idx = 0; idx < numAdc; idx++)
    {
        addr = pTypeConfig[idx].configReg.configCrcMmrRetainedForce.addr;
        value = pTypeConfig[idx].configReg.configCrcMmrRetainedForce.value;
        pCmd = (ADI_ADC_CMD *)(pInfo->pTxCmdFramePtr[idx] + pTypeConfig[idx].cmdOffset);
        status = adi_adc_AssembleCommand(pAdcCfg->pfCalcCmdCrc, ADI_ADC_RWB_WRITE, addr, value,
                                         pCmd, pAdcCfg->frameFormat);
        if (status != ADI_ADC_STATUS_SUCCESS)
        {
            break;
        }
    }

    return status;
}

static ADI_ADC_STATUS PopulateConfigCrcMmrRetainedDone(ADI_ADC_INFO *pInfo)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    int32_t idx;
    ADI_ADC_CONFIG *pAdcCfg = &pInfo->adcCfg;
    uint8_t numAdc = pAdcCfg->numAdc;
    uint8_t addr;
    uint8_t value;
    ADC_TYPE_CONFIG *pTypeConfig = pInfo->pTypeConfig;
    ADI_ADC_CMD *pCmd;

    for (idx = 0; idx < numAdc; idx++)
    {
        addr = pTypeConfig[idx].configReg.configCrcMmrRetainedDone.addr;
        value = pTypeConfig[idx].configReg.configCrcMmrRetainedDone.value;
        pCmd = (ADI_ADC_CMD *)(pInfo->pTxCmdFramePtr[idx] + pTypeConfig[idx].cmdOffset);
        status = adi_adc_AssembleCommand(pAdcCfg->pfCalcCmdCrc, ADI_ADC_RWB_WRITE, addr, value,
                                         pCmd, pAdcCfg->frameFormat);
        if (status != ADI_ADC_STATUS_SUCCESS)
        {
            break;
        }
    }

    return status;
}

static ADI_ADC_STATUS PopulateDatapathConfigLock(ADI_ADC_INFO *pInfo, uint8_t value)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    int32_t idx;
    ADI_ADC_CONFIG *pAdcCfg = &pInfo->adcCfg;
    uint8_t numAdc = pAdcCfg->numAdc;
    uint8_t addr;
    ADC_TYPE_CONFIG *pTypeConfig = pInfo->pTypeConfig;
    ADI_ADC_CMD *pCmd;

    for (idx = 0; idx < numAdc; idx++)
    {
        addr = pTypeConfig[idx].configReg.datapathConfigLock.addr;
        pCmd = (ADI_ADC_CMD *)(pInfo->pTxCmdFramePtr[idx] + pTypeConfig[idx].cmdOffset);
        status = adi_adc_AssembleCommand(pAdcCfg->pfCalcCmdCrc, ADI_ADC_RWB_WRITE, addr, value,
                                         pCmd, pAdcCfg->frameFormat);
        if (status != ADI_ADC_STATUS_SUCCESS)
        {
            break;
        }
    }

    return status;
}

static ADI_ADC_STATUS PopulateConfigFilt(ADI_ADC_INFO *pInfo)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    int32_t idx;
    ADI_ADC_CONFIG *pAdcCfg = &pInfo->adcCfg;
    uint8_t numAdc = pAdcCfg->numAdc;
    uint8_t addr;
    uint8_t value;
    ADC_TYPE_CONFIG *pTypeConfig = pInfo->pTypeConfig;
    ADI_ADC_CMD *pCmd;

    for (idx = 0; idx < numAdc; idx++)
    {
        addr = pTypeConfig[idx].configReg.datarate.addr;
        value = pTypeConfig[idx].configReg.datarate.value;
        pCmd = (ADI_ADC_CMD *)(pInfo->pTxCmdFramePtr[idx] + pTypeConfig[idx].cmdOffset);
        status = adi_adc_AssembleCommand(pAdcCfg->pfCalcCmdCrc, ADI_ADC_RWB_WRITE, addr, value,
                                         pCmd, pAdcCfg->frameFormat);
        if (status != ADI_ADC_STATUS_SUCCESS)
        {
            break;
        }
    }

    return status;
}

static ADI_ADC_STATUS PopulateConfigCrcMmrForce(ADI_ADC_INFO *pInfo)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    int32_t idx;
    ADI_ADC_CONFIG *pAdcCfg = &pInfo->adcCfg;
    uint8_t numAdc = pAdcCfg->numAdc;
    uint8_t addr;
    uint8_t value;
    ADC_TYPE_CONFIG *pTypeConfig = pInfo->pTypeConfig;
    ADI_ADC_CMD *pCmd;

    for (idx = 0; idx < numAdc; idx++)
    {
        addr = pTypeConfig[idx].configReg.configCrcMmrForce.addr;
        value = pTypeConfig[idx].configReg.configCrcMmrForce.value;
        pCmd = (ADI_ADC_CMD *)(pInfo->pTxCmdFramePtr[idx] + pTypeConfig[idx].cmdOffset);
        status = adi_adc_AssembleCommand(pAdcCfg->pfCalcCmdCrc, ADI_ADC_RWB_WRITE, addr, value,
                                         pCmd, pAdcCfg->frameFormat);
        if (status != ADI_ADC_STATUS_SUCCESS)
        {
            break;
        }
    }

    return status;
}

static ADI_ADC_STATUS PopulateConfigCrcMmrDone(ADI_ADC_INFO *pInfo)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    int32_t idx;
    ADI_ADC_CONFIG *pAdcCfg = &pInfo->adcCfg;
    uint8_t numAdc = pAdcCfg->numAdc;
    uint8_t addr;
    uint8_t value;
    ADC_TYPE_CONFIG *pTypeConfig = pInfo->pTypeConfig;
    ADI_ADC_CMD *pCmd;

    for (idx = 0; idx < numAdc; idx++)
    {
        addr = pTypeConfig[idx].configReg.configCrcMmrDone.addr;
        value = pTypeConfig[idx].configReg.configCrcMmrDone.value;
        pCmd = (ADI_ADC_CMD *)(pInfo->pTxCmdFramePtr[idx] + pTypeConfig[idx].cmdOffset);
        status = adi_adc_AssembleCommand(pAdcCfg->pfCalcCmdCrc, ADI_ADC_RWB_WRITE, addr, value,
                                         pCmd, pAdcCfg->frameFormat);
        if (status != ADI_ADC_STATUS_SUCCESS)
        {
            break;
        }
    }

    return status;
}

static ADI_ADC_STATUS PopulateClearStatus1(ADI_ADC_INFO *pInfo)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    int32_t idx;
    ADI_ADC_CONFIG *pAdcCfg = &pInfo->adcCfg;
    uint8_t numAdc = pAdcCfg->numAdc;
    uint8_t addr;
    uint8_t value;
    ADC_TYPE_CONFIG *pTypeConfig = pInfo->pTypeConfig;
    ADI_ADC_CMD *pCmd;

    for (idx = 0; idx < numAdc; idx++)
    {
        addr = pTypeConfig[idx].configReg.status1.addr;
        value = pTypeConfig[idx].configReg.status1.value;
        pCmd = (ADI_ADC_CMD *)(pInfo->pTxCmdFramePtr[idx] + pTypeConfig[idx].cmdOffset);
        status = adi_adc_AssembleCommand(pAdcCfg->pfCalcCmdCrc, ADI_ADC_RWB_WRITE, addr, value,
                                         pCmd, pAdcCfg->frameFormat);
        if (status != ADI_ADC_STATUS_SUCCESS)
        {
            break;
        }
    }
    return status;
}

static ADI_ADC_STATUS PopulateClearStatus0(ADI_ADC_INFO *pInfo)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    int32_t idx;
    ADI_ADC_CONFIG *pAdcCfg = &pInfo->adcCfg;
    uint8_t numAdc = pAdcCfg->numAdc;
    uint8_t addr;
    uint8_t value;
    ADC_TYPE_CONFIG *pTypeConfig = pInfo->pTypeConfig;
    ADI_ADC_CMD *pCmd;

    for (idx = 0; idx < numAdc; idx++)
    {
        addr = pTypeConfig[idx].configReg.status0.addr;
        value = pTypeConfig[idx].configReg.status0.value;
        pCmd = (ADI_ADC_CMD *)(pInfo->pTxCmdFramePtr[idx] + pTypeConfig[idx].cmdOffset);
        status = adi_adc_AssembleCommand(pAdcCfg->pfCalcCmdCrc, ADI_ADC_RWB_WRITE, addr, value,
                                         pCmd, pAdcCfg->frameFormat);
        if (status != ADI_ADC_STATUS_SUCCESS)
        {
            break;
        }
    }
    return status;
}

/**
 * @}
 */
