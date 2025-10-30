/******************************************************************************
 Copyright (c) 2025  Analog Devices Inc.
******************************************************************************/

/**
 * @file        adc_example_attributes.h
 * @brief       The functions to set/get attributes.
 * @{
 */

#ifndef __ADC_EXAMPLE_ATTRIBUTES_H__
#define __ADC_EXAMPLE_ATTRIBUTES_H__

/*============= I N C L U D E S =============*/
#include "adc_service_interface.h"
#include "adi_adc.h"
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Macros for stringification */
#define XSTR(s) #s

/** Macros for stringification */
#define STR(s) XSTR(s)

/** No of supported EVBs in ADEMA127 Plugin */
#define NUM_EVB_AVAILABLE 2

/** Maximum number of samples to store
 */
#ifdef BOARD_CFG_USE_SMALL_BUFFER
#define ADC_EXM_MAX_SAMPLES_TO_STORE 28000
#else
#define ADC_EXM_MAX_SAMPLES_TO_STORE 64000
#endif

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
    EXAMPLE_ATTR_ID_EVB_TYPE,
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
    /** Command to set sampling rate */
    ADC_EXAMPLE_ATTR_ID_SAMPLING_RATE
} ADC_EXAMPLE_ATTR_ID;

/**
 * Settings type
 */
typedef enum
{
    ADC_EXAMPLE_SETTINGS_TYPE_RECOMMENDED_SETTINGS,
    ADC_EXAMPLE_SETTINGS_TYPE_DEFAULT_ADEMA127,
} ADC_EXAMPLE_SETTINGS_TYPE;

/**
 * Structure to hold the ADC example attributes
 */
typedef struct
{
    /** Evb type */
    char *pEvbType;
    /** ADC index */
    int8_t adcIndex;
    /** ADC type */
    ADI_ADC_TYPE adcVariant;
    /** Example settings */
    char *pExampleSettings;
    /** ADC example settings type */
    ADC_EXAMPLE_SETTINGS_TYPE settings;
    /** channel scale */
    float adcChanScale[APP_CFG_MAX_NUM_CHANNELS];
    /** address field for debug reg r/w attributes */
    int32_t debugAddress;
    /** channel config used for samples collection */
    uint32_t chanConfig;
} ADC_EXAMPLE_ATTR_INFO;

/*======= P U B L I C   P R O T O T Y P E S ========*/
/**
 * @brief Get the attribute value
 *
 * @param attrId - Attribute ID
 * @param pChanIdx - pointer to Channel number
 * @param pValue - pointer to value to be set
 * @param pValueSize - pointer to size of the value in bytes
 * @return int32_t - Status
 */
int GetAttribute(int32_t attrId, int32_t *pChanIdx, uint32_t *pValue, uint8_t *pValueSize);

/**
 * @brief Set the attribute value
 *
 * @param attrId - Attribute ID
 * @param pChanIdx - pointer to Channel number
 * @param pValue - pointer to value to be set
 * @param valueSize - Size of the value in bytes
 * @return int32_t - Status
 */
int32_t SetAttribute(int32_t attrId, uint8_t *pChanIdx, uint8_t *pValue, uint8_t valueSize);

/**
 * @brief Read from a register
 *
 * @param address - Register address
 * @param pDst - Pointer to the destination buffer
 * @return int32_t - Status
 */
int32_t DebugRegRead(uint32_t address, uint32_t *pDst);

/**
 * @brief Write to a register
 *
 * @param address - Register address
 * @param value - Register value to write
 * @return int32_t - Status
 */
int32_t DebugRegWrite(uint32_t address, uint32_t value);

/**
 * @brief Sets datapath config attribute.
 * @return status
 */
int32_t SetDatapathConfig(ADC_INTERFACE_INFO *pAdcIf, uint8_t *pChanIdx, uint8_t adcIndex,
                          uint32_t writeVal);

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
 * @brief Extract attribute value from string and output it's size in bytes
 *
 * @param pSrc - Attribute value in string
 * @param attrId - Attribute ID
 * @param pValueSize - Size of the value in bytes
 */
void ExtractAttributeValue(char *pSrc, int32_t attrId, uint8_t *pValueSize);

#ifdef __cplusplus
}
#endif

#endif /* __ADC_EXAMPLE_ATTRIBUTES_H__ */
/**
 * @}
 */
