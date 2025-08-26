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

/** State memory required for 1 ADEMA127, block size = 1, Max sample delay = 4. */
#define ADI_ADC_STATE_MEM_NUM_BYTES_1XADEMA127_1XBLOCKSIZE                                         \
    ADI_ADC_STATE_MEM_NUM_BYTES(1, 7, 7, 1, 4)

/** State memory required for 1 ADEMA124, block size = 1, Max sample delay = 4. */
#define ADI_ADC_STATE_MEM_NUM_BYTES_1XADEMA124_1XBLOCKSIZE                                         \
    ADI_ADC_STATE_MEM_NUM_BYTES(1, 4, 4, 1, 4)

/** State memory required for 1 ADE91XX, block size = 1, Max sample delay = 4. */
#define ADI_ADC_STATE_MEM_NUM_BYTES_1XADE91XX_1XBLOCKSIZE ADI_ADC_STATE_MEM_NUM_BYTES(1, 3, 3, 1, 4)

/** State memory required in bytes for the library. */
#define ADI_ADC_STATE_MEM_NUM_BYTES(numAdc, maxNumChannelPerAdc, maxNumChannel, blockSize,         \
                                    maxSampleDelay)                                                \
    (sizeof(ADI_ADC_INFO) + (3 * (numAdc)) * sizeof(uint32_t) +                                    \
     (maxNumChannelPerAdc) * sizeof(uint32_t) + (2 * (maxNumChannel)) * sizeof(uint32_t) +         \
     (3 * (ADI_ADC_LONG_FRAME_NBYTES_MAX * numAdc)) * sizeof(uint32_t) +                           \
     (2 * (numAdc * maxNumChannel)) * sizeof(uint32_t) +                                           \
     ((sizeof(ADC_TYPE_CONFIG) * numAdc + 3) / sizeof(uint32_t)) * sizeof(uint32_t) +              \
     ((4 + ADI_ADC_LONG_FRAME_NBYTES_MAX) * (numAdc * (blockSize + 2))) * sizeof(uint32_t) +       \
     (1 + ((sizeof(ADI_ADC_DELAY_BUFFER) + 3) / sizeof(uint32_t))) * maxNumChannel *               \
         sizeof(uint32_t) +                                                                        \
     (maxNumChannel * (maxSampleDelay + 1)) * sizeof(uint32_t) +                                   \
     (numAdc * blockSize) * sizeof(uint32_t) + 3 /* Alignment */                                   \
    )

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
