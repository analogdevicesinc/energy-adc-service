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

/** State memory required for 4 ADEMA127, block size = 4, Max sample delay= 4. */
#define ADI_ADC_STATE_MEM_NUM_BYTES_4XADEMA127_4XBLOCKSIZE                                         \
    ADI_ADC_STATE_MEM_NUM_BYTES(4, 7, 28, 4, 4)

/** State memory required for 1 ADEMA127, block size = 1, Max sample delay= 0. */
#define ADI_ADC_STATE_MEM_NUM_BYTES_1XADEMA127_1XBLOCKSIZE                                         \
    ADI_ADC_STATE_MEM_NUM_BYTES(1, 7, 7, 1, 0)

/** State memory required in bytes for the library. */
#define ADI_ADC_STATE_MEM_NUM_BYTES(numAdc, maxNumChannelPerAdc, maxNumChannel, blockSize,         \
                                    maxSampleDelay)                                                \
    (sizeof(ADI_ADC_INFO) +                                                                        \
     ((numAdc * 113) + (maxNumChannelPerAdc * 7) + (maxNumChannel * 6) +                           \
      (blockSize * numAdc * 1) + (numAdc * maxNumChannel * 2) + (numAdc * (blockSize + 2) * 35) +  \
      (maxNumChannel * (maxSampleDelay + 1))) *                                                    \
         sizeof(uint32_t) +                                                                        \
     3)

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
