/******************************************************************************
 Copyright (c) 2023 - 2025  Analog Devices Inc.
******************************************************************************/

/**
 * @file     adc_sync.h
 * @brief    Definitions for ADC synchronization process.
 * @addtogroup ADI_ADC
 * @{
 */

#ifndef __ADC_SYNC_H__
#define __ADC_SYNC_H__

/*============= I N C L U D E S =============*/
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#include "adc_private.h"
#include "adi_adc.h"

/** Number of dready cycle to wait after issuing ADC synchronization*/
#define ADI_ADC_SYNC_NUM_DREADY_WAIT 3

/**
 * @brief Function to start ADC synchronization process.
 * @param[in]  pInfo	- Pointer to adc info structure.
 * @return  One of the codes documented in #ADI_ADC_STATUS. Refer to
 * #ADI_ADC_STATUS for details.
 */
ADI_ADC_STATUS AdcSyncStart(volatile ADI_ADC_INFO *pInfo);

/**
 * @brief Function to perform ADC synchronization using align.
 * @return  One of the codes documented in #ADI_ADC_STATUS. Refer to
 * #ADI_ADC_STATUS for details.
 */
ADI_ADC_STATUS AdcSyncAlign(ADI_ADC_INFO *pInfo);

/**
 * @brief Function to check whether ADC synchronization is in progress or not.
 * @param[in]  pInfo	- Pointer to the ADC info structure.
 * @return  State of ADC synchronization.
 */
bool AdcSyncGetProgressStatus(volatile ADI_ADC_INFO *pInfo);

/**
 * @brief Function to abort the ADC synchronization.
 * @param[in]  pInfo	- Pointer to the ADC info structure.
 */
void AdcSyncAbort(volatile ADI_ADC_INFO *pInfo);

#ifdef __cplusplus
}
#endif

#endif /* __ADC_SYNC_H__ */
/** @} */
