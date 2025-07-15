/******************************************************************************
 Copyright (c) 2025  Analog Devices Inc.
******************************************************************************/

/**
 * @file        adc_service_adapter.c
 * @brief       Adapter file.
 * @{
 */

/*============= I N C L U D E S =============*/
#include "adc_service_adapter.h"
#include "ade_crc.h"
#include "adi_evb.h"
#include <stdint.h>
#include <string.h>

/*============= F U N C T I O N S =============*/
static int32_t CalcCmdCrc(uint8_t *pData, uint32_t numBytes, uint32_t *pCrc);
static int32_t CalcRespCrc(uint8_t *pData, uint32_t numBytes, uint32_t *pCrc);
static int32_t AdcAdaptTransceive(void *hEvb, uint8_t *pTxData, uint8_t *pRxData,
                                  uint32_t numBytes);
static int32_t AdcAdaptTransceiveAsync(void *hEvb, uint8_t *pTxData, uint8_t *pRxData,
                                       uint32_t numBytes);

int32_t AdcAdptPopulateConfig(ADI_ADC_CONFIG *pConfig)
{
    int32_t status = 0;

    pConfig->pfTransceive = AdcAdaptTransceive;
    pConfig->pfTransceiveAsync = AdcAdaptTransceiveAsync;

    pConfig->pfCalcCmdCrc = CalcCmdCrc;
    pConfig->pfCalcRespCrc = CalcRespCrc;

    return status;
}

int32_t CalcCmdCrc(uint8_t *pData, uint32_t numBytes, uint32_t *pCrc)
{
    int32_t status = 0;
    *pCrc = AdeCalculateCrc8(pData, (uint16_t)numBytes);

    return status;
}

int32_t CalcRespCrc(uint8_t *pData, uint32_t numBytes, uint32_t *pCrc)
{
    int32_t status = 0;
#if APP_CFG_ENABLE_HW_CRC == 1
    EvbCrcCalculate(NULL, pData, numBytes);
    EvbCrcGetValue(NULL, pCrc);
#else
    /*Calculate the 16 bit crc of the received frame*/
    *pCrc = AdeCalculateCrc16(pData, numBytes);
#endif

    return status;
}

int32_t AdcAdaptTransceive(void *hUser, uint8_t *pTxData, uint8_t *pRxData, uint32_t numBytes)
{
    int32_t status;
    uint32_t timeOut = 10000000;
    status = EvbAdeSpiTransceive(hUser, pTxData, pRxData, numBytes, timeOut);
    return status;
}

int32_t AdcAdaptTransceiveAsync(void *hUser, uint8_t *pTxData, uint8_t *pRxData, uint32_t numBytes)
{
    int32_t status;
    status = EvbAdeSpiTransceiveAsync(hUser, pTxData, pRxData, numBytes);
    return status;
}

/**
 * @}
 */
