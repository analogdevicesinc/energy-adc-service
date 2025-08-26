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
#include "adc_example_attributes.h"
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

/** List of EVBs supported */
extern char *pEvbAvailable[];

/** List of ADC Example settings available */
extern char *pAdcExampleSettings[];

/*============= D A T A  T Y P E S =============*/

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
    /** Pointer to board handle */
    void *hEvb;
    /** Board config */
    ADI_EVB_CONFIG evbConfig;
    /**  ADC interface info */
    ADC_INTERFACE_INFO *pAdcIf;
    /** Example attributes info */
    ADC_EXAMPLE_ATTR_INFO adcExampleAttrInfo;
    /** Structure to hold info on samples to send */
    EXAMPLE_SAMPLES_BUFFER samplesBuffer;
    /** Buffer to store block of samples from all channels from samplesBuffer and send over uart */
    int32_t blockBuffer[APP_CFG_MAX_SAMPLE_BLOCK_SIZE * APP_CFG_MAX_NUM_CHANNELS];
    /** Cli info */
    EXAMPLE_CLI_INFO *pCliInfo;
    /** Trigger Device */
    int32_t triggerDevice;
    /** Channel mask */
    uint32_t channelMask;
    /** sample count */
    int32_t sampleCount;
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
    /** Board-Related Configs */
    ADC_BOARD_CONFIG adcBoardConfig;

} ADC_EXAMPLE;

/**
 * @brief Initialises Com, Crc and Scomm services
 * @return status
 */
ADC_EXAMPLE_STATUS InitServices(void);

/**
 * @brief Process commands
 * @return status
 */
ADC_EXAMPLE_STATUS ProcessCommand(void);

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
 * @brief Submit the buffer
 *
 * @param numBytes - Number of bytes
 * @return int32_t - Status
 */
int32_t IioSubmitBuffer(int32_t numBytes);

/**
 * @brief Close the device
 *
 * @return int32_t - Status
 */
int32_t Close(void);

/**
 * @brief Gets pointer to ADC_EXAMPLE_IIO structure.
 * @return pointer to ADC_EXAMPLE_IIO structure.
 */
ADC_EXAMPLE *GetAdcExampleInfo(void);

/**
 * @brief Gets pointer to ADC_EXAMPLE_ATTR_INFO structure.
 * @return pointer to ADC_EXAMPLE_ATTR_INFO structure.
 */
ADC_EXAMPLE_ATTR_INFO *GetAdcExampleAttrInfo(void);

/**
 * @brief Gets IIO attribute value
 * @param attrId Attribute ID
 * @param pChanIdx Channel index
 * @param pDst Destination string buffer
 * @return status
 */
int32_t GetIioAttribute(int32_t attrId, int32_t *pChanIdx, char *pDst);

/**
 * @brief Gets string attribute value
 * @param attrId Attribute ID
 * @param pChanIdx Channel index
 * @param pDst Destination string buffer
 * @return status
 */
int32_t GetStringAttribute(int32_t attrId, int32_t *pChanIdx, char *pDst);

/**
 * @brief Gets attr data type
 * @param attrId ID
 * @return ADI_ATTR_TYPE datatype
 */
ADI_ATTR_TYPE GetAttributeDataType(int32_t attrId);

/**
 * @brief Gets avail attribute value
 * @param attrId Attribute ID
 * @param pDst Destination string buffer
 * @return status
 */
int32_t GetAvailableAttribute(uint32_t attrId, char *pDst);

/**
 * @brief Formats string
 * @param pDst Attribute name
 * @param pValue Pointer to value
 * @param attrType Attribute data type
 */
void FormatString(char *pDst, uint8_t *pValue, ADI_ATTR_TYPE attrType);

#ifdef __cplusplus
}
#endif

#endif /* ADC_EXAMPLE_H_ */
/**
 * @}
 */
