/******************************************************************************
 Copyright (c) 2025  Analog Devices Inc.
******************************************************************************/

/**
 * @file  adc_datapath_cfg.h
 * @brief Configuration file for ADC datapath settings.
 * @{
 */

#ifndef __ADC_DATAPATH_CFG_H__
#define __ADC_DATAPATH_CFG_H__

#ifdef __cplusplus
extern "C" {
#endif

/*============= D E F I N I T I O N S =============*/

/**
 * Master clock frequency for the ADC.
 */
#define APP_CFG_ADC_MCLK 16384000

/**
 * Prescaler value for the ADC clock.
 */
#define APP_CFG_ADC_PRESCALER 3

/**
 * Decimation rate for the ADC.
 */
#define APP_CFG_ADC_DECIMATION_RATE 2

/**
 * Decimation by 2 setting for the ADC.
 */
#define APP_CFG_ADC_DECIMATION_BY2 1

/**
 * Gain setting for the ADC.
 * 0 - 1
 * 1 - 2
 */
#define APP_CFG_ADC_GAIN 0x0

/**
 * Datapath configuration for ADC current channels.
 */
#define APP_CFG_ADC_DATAPATH_CONFIG_CURRENT 0x1F

/**
 * Datapath configuration for ADC voltage channels.
 */
#define APP_CFG_ADC_DATAPATH_CONFIG_VOLTAGE 0x4D

/**
 * Datapath shift value for ADC current channels.
 */
#define APP_CFG_ADC_DATAPATH_SHIFT_CURRENT 4

/**
 * Datapath shift value for ADC voltage channels.
 */
#define APP_CFG_ADC_DATAPATH_SHIFT_VOLTAGE 1

/**
 * Datapath gain value for ADC voltage channels.
 */
#define APP_CFG_ADC_DATAPATH_GAIN_VOLTAGE 1.0

/**
 * Datapath gain value for ADC current channels.
 */
#define APP_CFG_ADC_DATAPATH_GAIN_CURRENT 1.0

/**
 * Datapath offset value for ADC voltage channels.
 */
#define APP_CFG_ADC_DATAPATH_OFFSET_VOLTAGE 0

/**
 * Datapath offset value for ADC current channels.
 */
#define APP_CFG_ADC_DATAPATH_OFFSET_CURRENT 0

/**
 * Datapath phase value for ADC voltage channel.
 */
#define APP_CFG_ADC_DATAPATH_PHASE_VOLTAGE 0.0

/**
 * Datapath phase value for ADC current channels.
 */
#define APP_CFG_ADC_DATAPATH_PHASE_CURRENT 0.0

#ifdef __cplusplus
}
#endif

#endif /* __ADC_DATAPATH_CFG_H__ */

/** @} */
