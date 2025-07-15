/******************************************************************************
 Copyright (c) 2024 - 2025  Analog Devices Inc.
******************************************************************************/

/**
 * @file      app_cfg.h
 * @brief     ADC Service Application configuration file
 * @defgroup EXAMPLE_CFG Configurations
 * @{
 */

#ifndef __APP_CFG_H__
#define __APP_CFG_H__

/*=============  I N C L U D E S   =============*/

#ifdef __cplusplus
extern "C" {
#endif

/*=============  D A T A  =============*/
/**
 * Device selector ID
 */
typedef enum
{
    ID_ADEMA12x,
    ID_ADE91xx
} ADC_ID;

/*=============  D E F I N I T I O N S  =============*/

/** Maximum timeout count to send command and get response */
#define APP_CFG_ENABLE_HOST_UART 1
/** Enable Wfs Uart*/
#define APP_CFG_ENABLE_WFS_UART 0
/** Enable Timer*/
#define APP_CFG_ENABLE_SYSTEM_TIMER 1
/** Enable Low Power Timer */
#define APP_CFG_ENABLE_LP_TIMER 1
/** Enable Spi*/
#define APP_CFG_ENABLE_SPI 1
/** Enable NUCLEOH563ZI SPI */
#define APP_CFG_ENABLE_ADE9178_SPI 1
/** Enable Gpio*/
#define APP_CFG_ENABLE_GPIO 1
/** timeout */
#define APP_CFG_TIMEOUT_COUNT 10000000
/** priority */
#define APP_CFG_PORT0_GPIO_INT_PRIO 5
/** priority */
#define APP_CFG_HOST_UART_TX_INT_PRIORITY 4
/** priority */
#define APP_CFG_HOST_UART_RX_INT_PRIORITY 4
/** baudrate */
#define APP_CFG_HOST_UART_SPEED 921600
/** priority */
#define APP_CFG_WFS_UART_INT_PRIORITY 4
/** priority */
#define APP_CFG_WFS_ERROR_INT_PRIORITY 4
/** baudrate */
#define APP_CFG_WFS_UART_SPEED 1152000
/**  ADC SPI bus speed*/
#define APP_CFG_ADC_SPI_SPEED 12500000
/**  ADC SPI mode*/
#define APP_CFG_ADC_SPI_MODE 3
/** Priority value for the GPIO interrupt */
#define APP_CFG_ADC_GPIO_DREADY_INT_PRIORITY 1
/** Priority value for the SPI Tx interrupt */
#define APP_CFG_ADC_SPI_TX_DMA_INT_PRIORITY 3
/** Priority value for the SPI Rx interrupt */
#define APP_CFG_ADC_SPI_RX_DMA_INT_PRIORITY 2
/** SPI interrupt priority */
#define APP_CFG_ADC_SPI_INT_PRIORITY 4
/** spi mode */
#define APP_CFG_ADC_SPI_MODE 3
/** max num adc */
#define APP_CFG_MAX_NUM_ADC 4
/** max block size */
#define APP_CFG_MAX_SAMPLE_BLOCK_SIZE 4
/** default block size */
#define APP_CFG_DEFAULT_SAMPLE_BLOCK_SIZE 1
/** Ignore Rx Buffer Overflow */
#define APP_CFG_IGNORE_RX_BUFFER_OVERFLOW 1
/** sampling rate */
#define APP_CFG_ADC_SAMPLING_RATE 8000
/** max number of channels */
#define APP_CFG_MAX_NUM_CHANNELS 28
/** max number of channels per ADC */
#define APP_CFG_MAX_NUM_CHANNELS_PER_ADC 7

/** no OS config macros */
/** ACTIVE DEVICE NAME */
#define APP_CFG_ACTIVE_DEVICE_NAME "adema12x"
/** active device id */
#define APP_CFG_ACTIVE_DEVICE_ID ID_ADEMA12x
/** Bytes per sample */
#define APP_CFG_BYTES_PER_SAMPLE sizeof(uint32_t) // For ADC resolution of 24-bits
/** Prioirty level of DREADY GPIO interrupt */
#define APP_CFG_DREADY_INTR_PRIORITY 2
/** Priority level of ADC SPI interrupt */
#define APP_CFG_ADC_SPI_INTR_PRIORITY 3
/** Priority level of FRAM SPI interrupt */
#define APP_CFG_FRAM_SPI_INTR_PRIORITY 4
/** Priority level of CF timer interrupt */
#define APP_CFG_CF_TIMER_INTR_PRIORITY 5
/** Priority level of ADC UART interrupt */
#define APP_CFG_UART_INTR_PRIORITY 4
/** Priority level of UART interrupt */
#define APP_CFG_CLI_UART_INTR_PRIORITY APP_CFG_UART_INTR_PRIORITY

/** @brief prompt string for ADSW-ADC */
#define APP_CFG_CLI_PROMPT "ADSW-ADC> "
/** maximum length of string */
#define APP_CFG_MAX_STRING_LENGTH 128
/** maximum block size of transmit buffer */
#define APP_CFG_MAX_BLOCKSIZE 0xFFFF
/** Enable callback from ADC service */
#define APP_CFG_ENABLE_ADCS_CALLBACK 1
/** Quantization noise */
#define APP_CFG_QUANTIZATION_NOISE 44
/** Enable Datapath */
#define APP_CFG_ENABLE_DATAPATH 1
/** Maximum number of voltage channels */
#define APP_CFG_MAX_NUM_VOLTAGE_CHANNELS 3
/** Maximum number of current channels */
#define APP_CFG_MAX_NUM_CURRENT_CHANNELS 25
/** max delay in sample buffer.
    Valid delays: 0 (no delay), 1, 2, 3, 4 */
#define APP_CFG_MAX_SAMPLE_DELAY 4
/** Enable use of integer sample delay */
#define APP_CFG_USE_SAMPLE_DELAY 1
/** Enable use store timestamp of samples */
#define APP_CFG_USE_TIMESTAMP 1

/** Maximum number of samples to store in example
 */
#define APP_CFG_EXM_MAX_SAMPLES_TO_STORE 30000

/** Maximum number of parameters that can be passed */
#define APP_CFG_CLI_MAX_PARAM_COUNT (24)
/** Maximum parameter length */
#define APP_CFG_CLI_MAX_PARAM_LENGTH (32)
/** Maximum command length */
#define APP_CFG_CLI_MAX_CMD_LENGTH (512)
/** Enable hardware CRC */
#define APP_CFG_ENABLE_HW_CRC 1
#ifdef __cplusplus
}
#endif

#endif /* __APP_CFG_H__ */

/**
 * @}
 */
