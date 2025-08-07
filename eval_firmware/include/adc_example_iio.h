/******************************************************************************
 Copyright (c) 2024 - 2025  Analog Devices Inc.
******************************************************************************/

/**
 * @file        adc_example_iio.h
 * @brief       ADC example definitions for IIO example
 * @{
 */

#ifndef __ADC_EXAMPLE_IIO_H__
#define __ADC_EXAMPLE_IIO_H__

#include "adc_example.h"
#include "adc_service_interface.h"
#include "adi_adc.h"
#include "adi_adc_memory.h"
#include "adi_cli.h"
#include "adi_cli_iiod_xml.h"
#include "adi_cli_memory.h"
#include "adi_evb.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Macros for stringification */
#define XSTR(s) #s

/** Macros for stringification */
#define STR(s) XSTR(s)

/**
 * @brief The full-scale code value for the ADC.
 *
 * This constant represents the full-scale code value for the ADC.
 * The full-scale code value is the maximum digital value that the
 * ADC can produce when the input voltage is at its maximum level.
 */
#define AD_ADC_FULL_SCALE_CODE 0x599999

/** ADC IC scale */
#define ADEMA127_IC_SCALE 1

/** IEC 3Phase Meter Board voltage scale */
#define IEC_3PHASE_METER_V_SCALE (((float)1190 / AD_ADC_FULL_SCALE_CODE) * ADEMA127_IC_SCALE)

/** IEC 3Phase Meter Board current scale */
#define IEC_3PHASE_METER_I_SCALE (((float)9398 / AD_ADC_FULL_SCALE_CODE) * ADEMA127_IC_SCALE)

/** Maximum number of samples to store
 */
#ifdef BOARD_CFG_USE_SMALL_BUFFER
#define ADC_EXM_MAX_SAMPLES_TO_STORE 28000
#else
#define ADC_EXM_MAX_SAMPLES_TO_STORE 64000
#endif

/** Number of data storage bits (needed for IIO client to plot ADC data) */
#define CHN_STORAGE_BITS (APP_CFG_BYTES_PER_SAMPLE * 8)

/** HW Mezzanine name */
#define HW_MEZZANINE_NAME "EVAL-ADEMA127"

/** HW Mezzanine name */
#define HW_CARRIER "EVAL-ADEMA127"

/** Num context attributes */
#define NUM_CTX_ATTRIBUTES 3

/** Firmware version - last 6 digits of git commit */
#define FIRMWARE_VERSION "abcedef"

/** No of supported EVBs in ADEMA127 Plugin */
#define NUM_EVB_AVAILABLE 2

/** Number of real bits (needed for IIO client to plot ADC data) */
#define CHN_REAL_BITS (32)

/** IIO version */
#define IIOD_VERSION "1.1.0000000"

/** Maximum number of adc channels*/
#define NUM_IIO_CHANNELS 7

/**	Number of IIO devices */
#define NUM_OF_IIO_DEVICES 1

/** Macro for the xml buffer size */
#define XML_DESC_BUFFER_SIZE 8000

/**
 * Enum holding the attribute IDs
 */
typedef enum
{
    /** ADC service version */
    ADC_EXAMPLE_ATTR_ID_VERSION,
    /** ADC service build id : git commit ID */
    ADC_EXAMPLE_ATR_ID_BUILD_ID,
    /** EVB Type */
    ADC_EXAMPLE_ATTR_ID_EVB_TYPE,
    /** Available EVB Type */
    ADC_EXAMPLE_ATTR_ID_EVB_TYPE_AVAIL,
    /** Channel Gain */
    ADC_EXAMPLE_ATTR_ID_CHAN_GAIN,
    /** Channel Offset */
    ADC_EXAMPLE_ATTR_ID_CHAN_OFFSET,
    /** ADC datapath config */
    ADC_EXAMPLE_ATTR_ID_DATAPATH_CONFIG,
    /** Adc channel scale */
    ADC_EXAMPLE_ATTR_ID_CHAN_SCALE,
    /** Adc channel shift */
    ADC_EXAMPLE_ATTR_ID_CHAN_SHIFT,
    /** Choose ADC example settings */
    ADC_EXAMPLE_ATTR_ID_CHOOSE_SETTINGS,
    /** Available settings */
    ADC_EXAMPLE_ATTR_ID_CHOOSE_SETTINGS_AVAIL,
    /** Apply settings */
    ADC_EXAMPLE_ATTR_ID_APPLY_SETTING,
    /** Available aplly settings */
    ADC_EXAMPLE_ATTR_ID_APPLY_SETTING_AVAIL,
    /** Channel integer sample delay */
    ADC_EXAMPLE_ATTR_ID_CHAN_INTEGER_SAMPLE_DELAY,
    /** Initiate Tamper detection */
    ADC_EXAMPLE_ATTR_ID_START_DETECT,
    /** Available Initiate Tamper detection */
    ADC_EXAMPLE_ATTR_ID_START_DETECT_AVAIL,
    /** Stop Tamper detection */
    ADC_EXAMPLE_ATTR_ID_STOP_DETECT,
    /** Available Stop Tamper detection */
    ADC_EXAMPLE_ATTR_ID_STOP_DETECT_AVAIL,
    /** Tamper detection count  */
    ADC_EXAMPLE_ATTR_ID_TAMPER_CNT,
    /** Channel XT Gain */
    ADC_EXAMPLE_ATTR_ID_CHAN_XT_GAIN,
    /** Channel XT Aggressor */
    ADC_EXAMPLE_ATTR_ID_CHAN_XT_AGGRESSOR,
} ADC_EXAMPLE_ATTR_ID;

/**
 * HPF Cutoff frequencies
 */
typedef enum
{
    ADC_EXAMPLE_HPF_CUTOFF_FREQ_10HZ,
    ADC_EXAMPLE_HPF_CUTOFF_FREQ_5HZ,
    ADC_EXAMPLE_HPF_CUTOFF_FREQ_2_5HZ,
    ADC_EXAMPLE_HPF_CUTOFF_FREQ_1_5HZ,
} ADC_EXAMPLE_HPF_CUTOFF_FREQ;

/**
 * HPF Cutoff frequencies
 */
typedef enum
{
    ADC_EXAMPLE_RECOMMENDED_SETTINGS,
    ADC_EXAMPLE_DEFAULT_ADEMA127,
} ADC_EXAMPLE_SETTINGS_TYPE;

/**
 * Structure for ADC example.
 */
typedef struct
{
    /** Buffer to hold samples to send */
    int32_t samplesToSend[ADC_EXM_MAX_SAMPLES_TO_STORE];
    /** Pointer to rx circular buffer instance */
    volatile ADI_CIRC_BUF *pCircBuff;
    /** Circular buffer for storing received data */
    volatile ADI_CIRC_BUF circBuff;
} EXAMPLE_SAMPLES_BUFFER;

/**
 * Structure to hold data for user handle.
 */
typedef struct
{
    /** CLI configurations */
    ADI_CLI_CONFIG config;
    /** handle for CLI */
    ADI_CLI_HANDLE hCli;
    /** Library memory for CLI Service */
    uint32_t stateMemory[ADI_CLI_STATE_MEM_NUM_BYTES / 4];
    /** temporary memory for CLI Service */
    uint32_t tempMemory[ADI_CLI_TEMP_MEM_NUM_BYTES / 4];
    /** Command buffer */
    char command[APP_CFG_CLI_MAX_CMD_LENGTH];
} EXAMPLE_CLI_INFO;

/**
 *
 * Structure for ADC example.
 */
typedef struct
{
    /** Handle to board */
    void *hEvb;
    /** evb type - an IIO example param */
    char *pEvbType;
    /** Board config */
    ADI_EVB_CONFIG evbConfig;
    /** Communication info */
    EXAMPLE_CLI_INFO *pCliInfo;
    /** Number of sampls collected */
    uint32_t numSamplesCollected;
    /**  ADC interface info */
    ADC_INTERFACE_INFO *pAdcIf;
    /** ADC types*/
    ADI_ADC_TYPE adcTypes[APP_CFG_MAX_NUM_ADC];
    /** Holds register value read from ADC*/
    uint8_t registerValue[2 * APP_CFG_MAX_NUM_ADC];
    /** Flag to indicate to start reading the adc samples */
    bool collectSamplesFlag;
    /** Structure to hold info on samples to send */
    EXAMPLE_SAMPLES_BUFFER samplesBuffer;
    /** channel config */
    uint32_t chanConfig;
    /** IIO Example params */
    /** reg address */
    uint16_t addr;
    /** value to write to a reg */
    uint8_t value;
    /** adc index */
    int8_t adcIndex;
    /** adc type */
    ADI_ADC_TYPE adcVariant;
    /** num of ADCs */
    uint8_t numAdc;
    /** num of samples required to be collected */
    uint32_t numSamplesRequired;
    /** Buffer to store block of samples from all channels and copy to no_os buffer */
    int32_t blockBuffer[APP_CFG_MAX_SAMPLE_BLOCK_SIZE * APP_CFG_MAX_NUM_CHANNELS];
    /** Hpf cutoff frequency in Hz */
    char *pHpfCutoffFreq;
    /** Example settings */
    char *pExampleSettings;
    /** ADC example settings type */
    ADC_EXAMPLE_SETTINGS_TYPE settings;
    /** Datapath params struct for 1 ADC */
    ADI_ADC_DSP_DATAPATH_PARAMS adcDatapathParams;
    /** Trigger Device */
    int32_t triggerDevice;
    /** value to be return when the DEBUG is enabled */
    int32_t debugAddress;
    /** Channel mask */
    uint32_t channelMask;
    /** sample count */
    int32_t sampleCount;
    /** channel scale */
    double adcChanScale[APP_CFG_MAX_NUM_CHANNELS];
    /** adc sampling rate */
    uint32_t samplingRate;
    /** IIO information */
    IioDesc iioDesc;
    /** XML length */
    char *pXmlSize;
    /** XML string */
    char *pXmlDescBuffer;
    /** Device attributes */
    DeviceAttributes deviceAttribute;
    /** Device parameters */
    DeviceParams deviceParams;
    /** Context attributes */
    CtxAttrType ctxAttribute;

} ADC_EXAMPLE;

/**
 * Collects the samples
 *
 * @param pExample -  Pointer to main example structure
 * @return status
 */
ADI_ADC_STATUS CollectSamples(ADC_EXAMPLE *pExample);

/**
 * Prints the IIO info
 *
 */
void PrintIIoInfo(void);

/**
 * Gets the ADC example info
 *
 * @return ADC_EXAMPLE pointer
 */
ADC_EXAMPLE *GetAdcExampleInfo(void);

/*!
 * @brief	Gets chan attr
 * @param	loReg- low reg addr
 * @param	mdReg- mid reg arr
 * @param	hiReg- hi reg addr
 * @return	Datapath config reg val
 */
uint32_t GetAdcChanAttr(uint16_t loReg, uint16_t mdReg, uint16_t hiReg);

/*!
 * @brief	Gets chan attr
 * @param	loReg- low reg addr
 * @param	mdReg- mid reg arr
 * @param	hiReg- hi reg addr
 * @param   writeVal- value to write to these reg
 * @return	Datapath config reg val
 */
ADI_ADC_STATUS SetAdcChanAttr(uint16_t loReg, uint16_t mdReg, uint16_t hiReg, uint32_t writeVal);

/**
 * @brief Write to a register
 *
 * @param address - Register address
 * @param value - Value to be written
 * @return int32_t - Status
 */
int32_t DebugRegWrite(uint32_t address, uint32_t value);

/**
 * @brief Get the attribute value
 *
 * @param attrId - Attribute ID
 * @param pChannel - pointer to channel number
 * @param pDst - Pointer to the destination buffer
 * @return int32_t - Status
 */
int IIoAttrGet(int32_t attrId, int32_t *pChannel, char *pDst);

/**
 * @brief Set the attribute value
 *
 * @param attrId - Attribute ID
 * @param pChanIdx - pointer to Channel number
 * @param pSrc - Value to be set
 * @return int32_t - Status
 */
int IioAttrSet(int32_t attrId, uint8_t *pChanIdx, char *pSrc);

/**
 * @brief Read from a register
 *
 * @param address - Register address
 * @param pDst - Pointer to the destination buffer
 * @return int32_t - Status
 */
int32_t DebugRegRead(uint32_t address, uint32_t *pDst);

/**
 * @brief Submit the buffer
 *
 * @param numBytes - Number of bytes
 * @return int32_t - Status
 */
int32_t IioSubmitBuffer(int32_t numBytes);
/**
 * @brief Get the channel attribute ID
 *
 * @param pAttrName - Attribute name
 * @return int32_t - Attribute ID
 */
int32_t GetChannelAttributeId(char *pAttrName);

/**
 * @brief Get the global attribute ID
 *
 * @param pAttrName - Attribute name
 * @return int32_t - Attribute ID
 */
int32_t GetGlobalAttributeId(char *pAttrName);

/**
 * @brief Close the device
 *
 * @return int32_t - Status
 */
int32_t Close(void);

#ifdef __cplusplus
}
#endif

#endif /* ADC_EXAMPLE_H_ */
/**
 * @}
 */
