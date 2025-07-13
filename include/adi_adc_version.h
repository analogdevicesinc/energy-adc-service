/******************************************************************************
 Copyright (c) 2024 - 2025  Analog Devices Inc.
******************************************************************************/

/**
 * @file     adi_adc_version.h
 * @brief    Definitions of version numbers
 * @addtogroup ADI_ADC
 * @{
 */
#ifndef __ADI_ADC_VERSION_H__
#define __ADI_ADC_VERSION_H__

/*============= I N C L U D E S =============*/
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*=============  D E F I N I T I O N S  =============*/

/** Product name */
#define ADI_ADC_PRODUCT_NAME "ADC Service"
/** Major revision*/
#define ADI_ADC_MAJOR_REVISION 0
/** Minor revision*/
#define ADI_ADC_MINOR_REVISION 9
/** Patch number*/
#define ADI_ADC_PATCH_NUMBER 0
/** ID to differentiate different builds of same release.
 *  Updated through  automation*/
#define ADI_ADC_BUILD_HASH 0xfb2c27c1f
/** Build Number.
 *  Updated through  automation*/
#define ADI_ADC_BUILD_NUM 2507110

/** Release version */
#define ADI_ADC_VERSION                                                                            \
    ((ADI_ADC_MAJOR_REVISION << 24) | (ADI_ADC_MINOR_REVISION << 16) | (ADI_ADC_PATCH_NUMBER << 8))

#ifdef __cplusplus
}
#endif

#endif /* __ADI_ADC_VERSION_H__ */

/** @} */
