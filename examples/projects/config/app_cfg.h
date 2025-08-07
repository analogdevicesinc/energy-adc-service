/******************************************************************************
 Copyright (c) 2025  Analog Devices Inc.
******************************************************************************/

/**
 * @file      app_cfg.h
 * @brief     ADC Service Application configuration file.
 * @{
 */

#ifndef __APP_CFG_H__
#define __APP_CFG_H__

#ifdef __cplusplus
extern "C" {
#endif

/*=============  I N C L U D E S   =============*/
#include "adc_datapath_cfg.h"

/*=============  D E F I N I T I O N S  =============*/

/** Ignore Rx Buffer Overflow */
#define APP_CFG_IGNORE_RX_BUFFER_OVERFLOW 1
/** Enable callback from ADC service */
#define APP_CFG_ENABLE_ADCS_CALLBACK 0
/** Priority level of ADC SPI interrupt */
#define APP_CFG_ADC_SPI_INTR_PRIORITY 0x3
/** Priority level of FRAM SPI interrupt */
#define APP_CFG_FRAM_SPI_INTR_PRIORITY 0x4
/** Prioirty level of DREADY GPIO interrupt */
#define APP_CFG_DREADY_INTR_PRIORITY 0x2
/** Priority level of timer interrupt */
#define APP_CFG_CF_TIMER_INTR_PRIORITY 0x5
/** Enable UART */
#define APP_CFG_ENABLE_HOST_UART 1
/** Enable Timer*/
#define APP_CFG_ENABLE_SYSTEM_TIMER 1
/** Enable Low Power Timer*/
#define APP_CFG_ENABLE_LP_TIMER 1
/** Enable GPIO */
#define APP_CFG_ENABLE_GPIO 1
/** UART speed */
#define APP_CFG_HOST_UART_SPEED 115200
/** Enable Spi*/
#define APP_CFG_ENABLE_SPI 1
/**  ADC SPI mode*/
#define APP_CFG_ADC_SPI_MODE 3
/** Priority level of ADC UART interrupt */
#define APP_CFG_UART_INTR_PRIORITY 6
/** sampling rate */
#define APP_CFG_ADC_SAMPLING_RATE 8000
/**
 * Qunatization noise in datapath
 */
#define APP_CFG_QUANTIZATION_NOISE 44
/** Enable Datapath */
#define APP_CFG_ENABLE_DATAPATH 1
/** max num adc */
#define APP_CFG_MAX_NUM_ADC 4
/** max block size */
#define APP_CFG_MAX_SAMPLE_BLOCK_SIZE 4
/** default block size */
#define APP_CFG_DEFAULT_SAMPLE_BLOCK_SIZE 1
/** Maximum number of voltage channels */
#define APP_CFG_MAX_NUM_VOLTAGE_CHANNELS 3
/** Maximum number of current channels */
#define APP_CFG_MAX_NUM_CURRENT_CHANNELS 4
/** max number of channels */
#define APP_CFG_MAX_NUM_CHANNELS 28
/** max number of channels per ADC */
#define APP_CFG_MAX_NUM_CHANNELS_PER_ADC 7
/** max delay in sample buffer.
    Valid delays: 0 (no delay), 1, 2, 3, 4 */
#define APP_CFG_MAX_SAMPLE_DELAY 4
/** Enable use of integer sample delay */
#define APP_CFG_USE_SAMPLE_DELAY
/** Size of sample delay buffer. */
#define APP_CFG_DELAY_BUFFER_SIZE (APP_CFG_MAX_SAMPLE_DELAY + 1)
/** Enable use store timestamp of samples */
#define APP_CFG_USE_TIMESTAMP 0

/** Enable display samples onto the terminal */
#define DISABLE_ASCII_OUT 1
/** Enable HW CRC */
#define APP_CFG_ENABLE_HW_CRC 1

#ifdef __cplusplus
}
#endif

#endif /* __APP_CFG_H__ */

/**
 * @}
 */