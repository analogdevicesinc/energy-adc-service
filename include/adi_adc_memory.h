/******************************************************************************
 Copyright (c) 2024 - 2025  Analog Devices Inc.
******************************************************************************/

/**
 * @file
 * @brief     Defines for ADC service memory. This file includes
 * internal library defines to calculate memory size required. Include this
 * file only where #adi_adc_Create is called.
 * @{
 */

#ifndef __ADI_ADC_MEMORY__H_
#define __ADI_ADC_MEMORY__H_

/*============= I N C L U D E S =============*/
#include "adc_private.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*=============  D E F I N I T I O N S  =============*/

/** @addtogroup   ADCINIT Memory
 * @{
 */

/** State memory required in bytes for the library. Allocate a buffer aligned to
 * 32 bit boundary */
#define ADI_ADC_STATE_MEM_NUM_BYTES sizeof(ADI_ADC_INFO)
/** Temporary memory required in bytes for the library. Allocate a buffer
 * aligned to 32 bit boundary. */
#define ADI_ADC_TEMP_MEM_NUM_BYTES 32

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __ADI_ADC_MEMORY__H_ */

/**
 * @}
 */
