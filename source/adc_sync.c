/******************************************************************************
 Copyright (c) 2023 - 2025  Analog Devices Inc.
******************************************************************************/

/**
 * @file     adc_sync.c
 * @brief    ADC synchronization routines
 * @{
 */

/*============= I N C L U D E S =============*/
#include "adc_sync.h"
#include "ADE911X_addr_def.h"
#include "ADE911X_addr_rdef.h"
#include "adc_private.h"
#include "adi_adc_frame_assemble.h"

/*=============  D E F I N I T I O N S  =============*/
static ADI_ADC_STATUS PopulateAlignBit(ADI_ADC_INFO *pInfo);
static ADI_ADC_STATUS PopulateSnapshotCountHi(ADI_ADC_INFO *pInfo);

ADI_ADC_STATUS AdcSyncStart(volatile ADI_ADC_INFO *pInfo)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;

    pInfo->syncInProgress = true;
    pInfo->syncData.syncSnapState = ADI_ADC_SYNC_PREPARE_SNAP;

    return status;
}

ADI_ADC_STATUS AdcSyncAlign(ADI_ADC_INFO *pInfo)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    uint8_t syncState = (uint8_t)pInfo->syncData.syncSnapState;
    switch (syncState)
    {
    case ADI_ADC_SYNC_IDLE:
        break;
    case ADI_ADC_SYNC_PREPARE_SNAP:
        // prep_broadcast = 0, snapshot = 1, align = 0
        PopulateAlignBit(pInfo);
        pInfo->syncData.syncSnapState = ADI_ADC_SYNC_READ_SNAP_SHOT_CNT;
        break;
    case ADI_ADC_SYNC_READ_SNAP_SHOT_CNT:
        // read snap shop counter register
        PopulateSnapshotCountHi(pInfo);
        pInfo->syncData.syncSnapState = ADI_ADC_SYNC_WAIT;
        pInfo->syncData.waitCount = 0;
        break;
    case ADI_ADC_SYNC_WAIT:
        AdcAssembleNopAllAdc(pInfo, &pInfo->txCmdFramePtr[0]);
        pInfo->syncData.waitCount++;
        // wait at least 3 cycles of DREADY
        if (pInfo->syncData.waitCount > ADI_ADC_SYNC_NUM_DREADY_WAIT)
        {
            pInfo->syncData.waitCount = 0;
            AdcSyncAbort(pInfo);
        }
        break;
    }
    return status;
}

void AdcSyncAbort(volatile ADI_ADC_INFO *pInfo)
{
    pInfo->syncInProgress = false;
    pInfo->syncData.syncSnapState = ADI_ADC_SYNC_IDLE;
}

bool AdcSyncGetProgressStatus(volatile ADI_ADC_INFO *pInfo)
{
    return pInfo->syncInProgress;
}

static ADI_ADC_STATUS PopulateAlignBit(ADI_ADC_INFO *pInfo)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    int32_t idx;
    ADI_ADC_CONFIG *pAdcCfg = &pInfo->adcCfg;
    ADC_TYPE_CONFIG *pTypeConfig = &pInfo->typeConfig[0];
    uint8_t numAdc = pAdcCfg->numAdc;
    uint8_t addr;
    uint8_t value;
    ADI_ADC_CMD *pCmd;

    for (idx = 0; idx < numAdc; idx++)
    {
        if (status == ADI_ADC_STATUS_SUCCESS)
        {
            addr = pTypeConfig[idx].configReg.syncSnap.addr;
            value = pTypeConfig[idx].configReg.syncSnap.value;
            pCmd = (ADI_ADC_CMD *)(pInfo->txCmdFramePtr[idx] + pTypeConfig[idx].cmdOffset);
            status = adi_adc_AssembleCommand(pAdcCfg->pfCalcCmdCrc, ADI_ADC_RWB_WRITE, addr, value,
                                             pCmd, pAdcCfg->frameFormat);
        }
    }
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        pInfo->cmdStatus = ADI_ADC_READ_WRITE_CMD_STATUS_QUEUED;
    }

    return status;
}

static ADI_ADC_STATUS PopulateSnapshotCountHi(ADI_ADC_INFO *pInfo)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    int32_t idx;
    ADI_ADC_CONFIG *pAdcCfg = &pInfo->adcCfg;
    ADC_TYPE_CONFIG *pTypeConfig = &pInfo->typeConfig[0];
    uint8_t numAdc = pAdcCfg->numAdc;
    uint8_t addr;
    uint8_t value;
    ADI_ADC_CMD *pCmd;

    for (idx = 0; idx < numAdc; idx++)
    {
        if (status == ADI_ADC_STATUS_SUCCESS)
        {
            addr = pTypeConfig[idx].configReg.snapshotCountHi.addr;
            value = pTypeConfig[idx].configReg.snapshotCountHi.value;
            pCmd = (ADI_ADC_CMD *)(pInfo->txCmdFramePtr[idx] + pTypeConfig[idx].cmdOffset);
            status = adi_adc_AssembleCommand(pAdcCfg->pfCalcCmdCrc, ADI_ADC_RWB_READ, addr, value,
                                             pCmd, pAdcCfg->frameFormat);
        }
    }
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        pInfo->cmdStatus = ADI_ADC_READ_WRITE_CMD_STATUS_QUEUED;
    }

    return status;
}

/**
 * @}
 */
