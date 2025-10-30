/******************************************************************************
 Copyright (c) 2024 - 2025  Analog Devices Inc.
******************************************************************************/

/**
 * @file        adc_example_iio.c
 * @brief       Interface which calls ADC APIs.
 * These functions can be used as an example for library API usage.
 * @defgroup    ADC_EXM ADC Example codes
 * @{
 */

/*============= I N C L U D E S =============*/
#include "adc_example_iio.h"
#include "adc_example.h"
#include "adc_example_attributes.h"
#include "adi_cli.h"
#include "board_cfg.h"
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "adc_datapath_cfg.h"
#include "adc_example.h"
#include "adc_example_attributes.h"
#include "adc_example_tdm.h"
#include "adc_service_dsp_interface.h"
#include "adc_service_interface.h"
#include "ade_crc.h"
#include "adi_adc.h"
#include "adi_adc_version.h"
#include "adi_circ_buf.h"
#include "adi_cli_iiod_xml.h"
#include "adi_evb.h"
#include "app_cfg.h"
#include "iiod_dispatch_table.h"
#include "message.h"
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/*=============  D E F I N I T I O N S  =============*/

/** IIOD channels scan parameters */
static scanType chanScan[NUM_IIO_CHANNELS];

/**
 *  Top level structure for ADC
 */
static ADC_EXAMPLE adcExample;

/** Uart Info */
int32_t uartInfo;
/** Uart Info */
static Args args;

/** Macro indicating the end of the array. */
#define END_ATTRIBUTES_ARRAY                                                                       \
    {                                                                                              \
        .pName = NULL                                                                              \
    }

/** Macro for the buffer size */
#define MAX_LINEAR_BUFFER_SIZE 256

/**
 *  ADC INTERFACE INFO structure
 */
SECTION(DMA_BUFFER) static ADC_INTERFACE_INFO adcIf;
/** Communication info */
SECTION(DMA_BUFFER) static EXAMPLE_CLI_INFO cliInfo;
/** XML length */
SECTION(DMA_BUFFER) static char xmlSize[20];
/** XML string */
SECTION(DMA_BUFFER) char xmlDescBuffer[XML_DESC_BUFFER_SIZE];
/** IIO Context Buffer */
SECTION(DMA_BUFFER) static CtxAttrType contextBuffer[XML_DESC_BUFFER_SIZE];
/** buffer to store the waveform data */
SECTION(DMA_BUFFER) static int32_t linearBuffer[MAX_LINEAR_BUFFER_SIZE];

/** Voltage slots */
static uint8_t vSlots[3] = {2, 6, 3};
/** Current slots */
static uint8_t iSlots[4] = {0, 1, 4, 5};
/*============= F U N C T I O N S =============*/

static void HostUartRxCallback(uint8_t *pData, uint32_t numBytes);
static void HostUartTxCallback(void);
static int32_t CliReceiveAsync(void *pInfo, char *pData, uint32_t numBytes);
static int32_t CliTransmitAsync(void *pInfo, uint8_t *pData, uint32_t numBytes);

static void InitCircBuff();
static int32_t InitDeviceAttributes(DeviceAttributes *pDeviceAttribute);
static float ConvertStrToFloat(const char *pStr);
static int32_t ConvertStrToInt32(const char *pStr);
static uint32_t ConvertStrtoUnit32(const char *pStr);
static uint8_t ConvertStrtoUnit8(const char *pStr);
static void PopulateExamplePointers(ADC_EXAMPLE *pExample);
static int32_t IsAvailAttr(int32_t attrId);
static int32_t IsButtonAttr(int32_t attrId);
static int32_t IsStringAttr(int32_t attrId);
/*
 * @brief Set the context attributes.
 * @param pCtxAttr - pointer to the context attributes
 * @param pAttrCount - pointer to number of context attributes
 */
static int32_t SetCtxAttributes(CtxAttrType **pCtxAttr, uint32_t *pAttrCount);
static int32_t InitIio(void);
static void PopulateBoardConfig(ADC_BOARD_CONFIG *pAdcBoardConfig);
/*=============  C O D E  =============*/

ADC_EXAMPLE_STATUS InitServices(void)
{
    ADC_EXAMPLE_STATUS status = ADC_EXAMPLE_STATUS_SUCCESS;
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    int32_t ifStatus;
    int32_t boardStatus = ADI_EVB_STATUS_SUCCESS;
    ADC_EXAMPLE *pExample = &adcExample;

    ADC_EXAMPLE_ATTR_INFO *pExampleAttrInfo = &pExample->adcExampleAttrInfo;
    ADI_EVB_CONFIG *pEvbConfig = &pExample->evbConfig;
    Args *pArgs = &args;
    ADI_CLI_STATUS cliStatus = ADI_CLI_STATUS_SUCCESS;
    uint8_t numAdc = 1;
    uint32_t i;
    uint32_t totalSampleBlockSize;
    uint32_t numMaxBlocks;
    void *pCmdInfo;
    pExample->pAdcIf = &adcIf;
    pExample->pCliInfo = &cliInfo;
    pExample->pXmlDescBuffer = &xmlDescBuffer[0];
    pExample->pXmlSize = &xmlSize[0];
    ADC_INTERFACE_INFO *pAdcIf = pExample->pAdcIf;
    PopulateExamplePointers(pExample);
    InitCircBuff();
    for (i = 0; i < sizeof(vSlots) / sizeof(vSlots[0]); i++)
    {
        // ACE expects scale in mV units
        pExampleAttrInfo->adcChanScale[vSlots[i]] = IEC_3PHASE_METER_V_SCALE * 1000;
    }
    for (i = 0; i < sizeof(iSlots) / sizeof(iSlots[0]); i++)
    {
        // ACE expects scale in mV units
        pExampleAttrInfo->adcChanScale[iSlots[i]] = IEC_3PHASE_METER_I_SCALE * 1000;
    }
    boardStatus = EvbInit(&pExample->hEvb, pEvbConfig);
    if (boardStatus != 0)
    {
        status = ADC_EXAMPLE_STATUS_BOARD_INIT_FAILED;
    }
    else
    {
        cliStatus =
            adi_cli_Create(&pExample->pCliInfo->hCli, pExample->pCliInfo->stateMemory,
                           sizeof(pExample->pCliInfo->stateMemory), pExample->pCliInfo->tempMemory,
                           sizeof(pExample->pCliInfo->tempMemory));
        if (cliStatus == ADI_CLI_STATUS_SUCCESS)
        {
            cliStatus = adi_cli_Init(pExample->pCliInfo->hCli, &pExample->pCliInfo->config);
            adi_cli_SetHandleTerminal(pExample->pCliInfo->hCli);
        }
        if (cliStatus != ADI_CLI_STATUS_SUCCESS)
        {
            status = ADC_EXAMPLE_STATUS_CLI_INIT_FAILED;
        }
        pCmdInfo = GetHandleForDispatchCommands(pExample->pCliInfo->hCli);
        pArgs->c = 1;
        pArgs->v[0].pS = "off";
        CliCmdEcho(pCmdInfo, dispatchTable, pArgs, NUM_COMMANDS);
        if (InitIio())
        {
            pExample->triggerDevice = 0;
            printf("IIO initialization failure!!\r\n");
        }
        if (status == ADC_EXAMPLE_STATUS_SUCCESS)
        {
            EvbLedOn(0);
            ifStatus = AdcIfCreateService(pAdcIf);
            if (ifStatus != 0)
            {
                status = ADC_EXAMPLE_STATUS_IF_CREATE_SERVICE_FAILED;
            }
        }

        if (status == 0)
        {
            PopulateBoardConfig(&pExample->adcBoardConfig);
            pExample->pAdcIf->pfCallback = AdcExmAdcCallback;
            EvbStartTimer();
            adcStatus = AdcIfInitService(pExample->pAdcIf, &pExample->adcBoardConfig);
            if (adcStatus != ADI_ADC_STATUS_SUCCESS)
            {
                status = ADC_EXAMPLE_STATUS_ADC_INIT_FAILED;
            }
            else
            {
                /* Toggle LED to indicate that ADC init is a success */
                EvbLedOn(1);
            }
            totalSampleBlockSize = pExample->pAdcIf->adcCfg.numSamplesInBlock *
                                   pExample->pAdcIf->runInfo.totalChannels;
            numMaxBlocks = ADC_EXM_MAX_SAMPLES_TO_STORE / totalSampleBlockSize;
            pExample->samplesBuffer.pCircBuff->nSize = pAdcIf->adcCfg.numSamplesInBlock *
                                                       pAdcIf->runInfo.totalChannels *
                                                       numMaxBlocks * APP_CFG_BYTES_PER_SAMPLE;
        }
        if (status == 0)
        {
            /* LED ON if ADC init success */
            EvbLedOn(1);
        }
        else
        {
            status = ADC_EXAMPLE_STATUS_ADC_INIT_FAILED;
        }
    }
    return status;
}

void PopulateExamplePointers(ADC_EXAMPLE *pExample)
{

    ADC_EXAMPLE_ATTR_INFO *pExampleAttrInfo = &pExample->adcExampleAttrInfo;
    ADI_EVB_CONFIG *pEvbConfig = &pExample->evbConfig;
    pExampleAttrInfo->pEvbType = pEvbAvailable[0];
    pExampleAttrInfo->adcIndex = 0;
    pExampleAttrInfo->pExampleSettings = pAdcExampleSettings[0];
    pExampleAttrInfo->chanConfig = 0xFFFFFFF;
    pEvbConfig->spiConfig.pfAdeSpiRxCallback = AdcSpiRxCallback;
    pEvbConfig->gpioConfig.pfGpioCallback = AdcDreadyCallback;
    pEvbConfig->hostCommConfig.pfRxCallback = HostUartRxCallback;
    pEvbConfig->hostCommConfig.pfTxCallback = HostUartTxCallback;
    pExample->pCliInfo->config.pfTransmitAsync = CliTransmitAsync;
    pExample->pCliInfo->config.pfReceiveAsync = CliReceiveAsync;
    pExample->pCliInfo->config.disableDisplayCtrlChars = true;
}

int32_t InitIio(void)
{
    int32_t status;
    ADC_EXAMPLE *pExample = &adcExample;
    IioDesc *pIioDescBuffer = &pExample->iioDesc;
    pIioDescBuffer->pCtxAttribute = &pExample->ctxAttribute;
    pIioDescBuffer->pDeviceParams = &pExample->deviceParams;
    pIioDescBuffer->pDeviceParams->pDeviceAttribute = &pExample->deviceAttribute;

    status = SetCtxAttributes(&pIioDescBuffer->pCtxAttribute, &pIioDescBuffer->numCtxAttribute);
    /* Initialize the IIO device */
    status = InitDeviceAttributes(pIioDescBuffer->pDeviceParams->pDeviceAttribute);
    pIioDescBuffer->numDevices = 1;
    pIioDescBuffer->pDeviceParams->pName = APP_CFG_ACTIVE_DEVICE_NAME;
    adi_cli_GenerateIiodXml(pIioDescBuffer, pExample->pXmlDescBuffer);
    return status;
}

void PrintIIoInfo(void)
{
    int32_t length;
    ADC_EXAMPLE *pExample = &adcExample;
    IioDesc *pIioDescBuffer = &pExample->iioDesc;
    sprintf(pExample->pXmlSize, "%" PRIi32, pIioDescBuffer->xmlSize);
    length = strlen(pExample->pXmlSize);
    pExample->pXmlSize[length] = '\n';
    EvbHostCommTransmitAsync(&uartInfo, (uint8_t *)&pExample->pXmlSize[0], length + 1);
    pIioDescBuffer->pXmlDesc[pIioDescBuffer->xmlSize] = '\n';
    EvbHostCommTransmitAsync(&uartInfo, (uint8_t *)pIioDescBuffer->pXmlDesc,
                             pIioDescBuffer->xmlSize + 1);
}
#if USE_FREERTOS == 1
int32_t IioSubmitBuffer(int32_t numBytes)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    // no of samples to collect from all channels
    uint32_t numSamplesRequired = numBytes / APP_CFG_BYTES_PER_SAMPLE;
    uint32_t samplesCopied = 0;
    uint32_t numSamplesSent = 0;
    ADC_EXAMPLE *pExample = &adcExample;
    ADC_EXAMPLE_ATTR_INFO *pExampleAttrInfo = &pExample->adcExampleAttrInfo;
    ADC_INTERFACE_INFO *pAdcIf = pExample->pAdcIf;
    uint8_t numSamplesInBlock = pAdcIf->adcCfg.numSamplesInBlock;
    uint32_t numSamplesPerIteration = numSamplesInBlock * pAdcIf->runInfo.totalChannels;
    int32_t *pBlockBuffer = &pExample->blockBuffer[0];
    int32_t txComplete;
    int32_t samplesRemaining;
    int32_t txBlockSize = numSamplesPerIteration;
    samplesRemaining = numSamplesRequired;
    while (numSamplesSent < numSamplesRequired)
    {

        // Transmits one block of samples for all channels.
        txComplete = EvbHostCommGetTxStatus();
        if (txComplete == 1)
        {
            samplesRemaining = numSamplesRequired - numSamplesSent;
            if (samplesRemaining >= txBlockSize)
            {
                txBlockSize = numSamplesPerIteration;
            }
            else
            {
                txBlockSize = samplesRemaining;
            }

            ADICircBufRead(pExample->samplesBuffer.pCircBuff, (uint8_t *)&linearBuffer[0],
                           txBlockSize * APP_CFG_BYTES_PER_SAMPLE);
            EvbHostCommTransmitAsync(&uartInfo, (uint8_t *)&linearBuffer[0],
                                     txBlockSize * APP_CFG_BYTES_PER_SAMPLE);
            numSamplesSent += txBlockSize;
        }
        samplesCopied += numSamplesPerIteration;
    }
    return 0;
}

#else
int32_t IioSubmitBuffer(int32_t numBytes)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    // no of samples to collect from all channels
    uint32_t numSamplesRequired = numBytes / APP_CFG_BYTES_PER_SAMPLE;
    uint32_t samplesCopied = 0;
    uint32_t numSamplesSent = 0;
    ADC_EXAMPLE *pExample = &adcExample;
    ADC_EXAMPLE_ATTR_INFO *pExampleAttrInfo = &pExample->adcExampleAttrInfo;
    ADC_INTERFACE_INFO *pAdcIf = pExample->pAdcIf;
    uint8_t numSamplesInBlock = pAdcIf->adcCfg.numSamplesInBlock;
    uint32_t numSamplesPerIteration = numSamplesInBlock * pAdcIf->runInfo.totalChannels;
    int32_t *pBlockBuffer = &pExample->blockBuffer[0];
    int32_t txComplete;
    int32_t samplesRemaining;
    int32_t txBlockSize = numSamplesPerIteration;
    AdcIfStartCapture(pAdcIf);
    samplesRemaining = numSamplesRequired;
    while (numSamplesSent < numSamplesRequired)
    {

        // Collect 1 block from all channels in each iteration
        adcStatus = AdcExmCollectSamples(pAdcIf, pExampleAttrInfo->chanConfig,
                                         numSamplesPerIteration, pBlockBuffer);
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            if (samplesCopied < numSamplesRequired)
            {
                ADICircBufWrite(pExample->samplesBuffer.pCircBuff, (uint8_t *)pBlockBuffer,
                                numSamplesPerIteration * APP_CFG_BYTES_PER_SAMPLE);
            }

            // Transmits one block of samples for all channels.
            txComplete = EvbHostCommGetTxStatus();
            if (txComplete == 1)
            {
                samplesRemaining = numSamplesRequired - numSamplesSent;
                if (samplesRemaining >= txBlockSize)
                {
                    txBlockSize = numSamplesPerIteration;
                }
                else
                {
                    txBlockSize = samplesRemaining;
                }

                ADICircBufRead(pExample->samplesBuffer.pCircBuff, (uint8_t *)&linearBuffer[0],
                               txBlockSize * APP_CFG_BYTES_PER_SAMPLE);
                EvbHostCommTransmitAsync(&uartInfo, (uint8_t *)&linearBuffer[0],
                                         txBlockSize * APP_CFG_BYTES_PER_SAMPLE);
                numSamplesSent += txBlockSize;
            }
            samplesCopied += numSamplesPerIteration;
        }
    }
    AdcIfStopCapture(pAdcIf);
    return 0;
}
#endif

ADC_EXAMPLE_STATUS ProcessCommand(void)
{
    ADC_EXAMPLE_STATUS status = ADC_EXAMPLE_STATUS_SUCCESS;
    ADC_EXAMPLE *pExample = &adcExample;
    EXAMPLE_CLI_INFO *pCliInfo = pExample->pCliInfo;
    ADI_CLI_STATUS cliStatus = 0;
    if (adi_cli_FlushMessages(pCliInfo->hCli) == 0)
    {
        cliStatus = adi_cli_GetCmd(pCliInfo->hCli, pCliInfo->command);
        if (cliStatus == ADI_CLI_STATUS_SUCCESS)
        {
            cliStatus =
                adi_cli_Dispatch(pCliInfo->hCli, pCliInfo->command, dispatchTable, NUM_COMMANDS);
        }
    }
    return status;
}

/** IIO Channel macro*/
#define IIO_CHAN(nm, chn)                                                                          \
    {                                                                                              \
        .pName = nm, .channelType = IIO_VOLTAGE, .ch_out = false, .indexed = true, .channel = chn, \
        .scan_index = chn, .pScanType = &chanScan[chn], .pAttributes = iioChanAttr                 \
    }

/** IIO Channel attribute macro*/
#define ADC_CHN_ATTR(_name, _priv, _dataType)                                                      \
    {                                                                                              \
        .pName = _name, .id = _priv, .dataType = _dataType                                         \
    }

/** IIO Channel attribute available macro*/
#define ADC_CHN_AVAIL_ATTR(_name, _priv, _dataType)                                                \
    {                                                                                              \
        .pName = _name, .id = _priv, .dataType = _dataType                                         \
    }

/** IIOD channels attributes list */
static AttributeType iioChanAttr[] = {
    ADC_CHN_ATTR("chn_gain", ADC_EXAMPLE_ATTR_ID_CHAN_GAIN, ADI_ATTR_TYPE_FLOAT),
    ADC_CHN_ATTR("chn_datapath_config", ADC_EXAMPLE_ATTR_ID_DATAPATH_CONFIG, ADI_ATTR_TYPE_UINT32),
    ADC_CHN_ATTR("chn_offset", ADC_EXAMPLE_ATTR_ID_CHAN_OFFSET, ADI_ATTR_TYPE_INT32),
    ADC_CHN_ATTR("chn_xt_gain", ADC_EXAMPLE_ATTR_ID_CHAN_XT_GAIN, ADI_ATTR_TYPE_FLOAT),
    ADC_CHN_ATTR("chn_xt_aggressor", ADC_EXAMPLE_ATTR_ID_CHAN_XT_AGGRESSOR, ADI_ATTR_TYPE_UINT8),
    ADC_CHN_ATTR("chn_shift", ADC_EXAMPLE_ATTR_ID_CHAN_SHIFT, ADI_ATTR_TYPE_UINT8),
    ADC_CHN_ATTR("chn_integer_sample_delay", ADC_EXAMPLE_ATTR_ID_CHAN_INTEGER_SAMPLE_DELAY,
                 ADI_ATTR_TYPE_UINT8),
    ADC_CHN_ATTR("scale", ADC_EXAMPLE_ATTR_ID_CHAN_SCALE, ADI_ATTR_TYPE_FLOAT),
    END_ATTRIBUTES_ARRAY};

/** IIOD device (global) attributes list */
static AttributeType iioGlobalAttributes[] = {
    ADC_CHN_ATTR("firmware_version", ADC_EXAMPLE_ATTR_ID_VERSION, ADI_ATTR_TYPE_STRING),
    ADC_CHN_ATTR("firmware_build_id", ADC_EXAMPLE_ATR_ID_BUILD_ID, ADI_ATTR_TYPE_STRING),
    ADC_CHN_ATTR("board_type", EXAMPLE_ATTR_ID_EVB_TYPE, ADI_ATTR_TYPE_STRING),
    ADC_CHN_AVAIL_ATTR("board_type_available", ADC_EXAMPLE_ATTR_ID_EVB_TYPE_AVAIL,
                       ADI_ATTR_TYPE_STRING),
    ADC_CHN_ATTR("choose_settings", ADC_EXAMPLE_ATTR_ID_CHOOSE_SETTINGS, ADI_ATTR_TYPE_STRING),
    ADC_CHN_AVAIL_ATTR("choose_settings_available", ADC_EXAMPLE_ATTR_ID_CHOOSE_SETTINGS_AVAIL,
                       ADI_ATTR_TYPE_STRING),
    ADC_CHN_ATTR("apply_settings", ADC_EXAMPLE_ATTR_ID_APPLY_SETTING, ADI_ATTR_TYPE_STRING),
    ADC_CHN_AVAIL_ATTR("apply_settings_available", ADC_EXAMPLE_ATTR_ID_APPLY_SETTING_AVAIL,
                       ADI_ATTR_TYPE_STRING),
    ADC_CHN_ATTR("start_tamper_detect", ADC_EXAMPLE_ATTR_ID_START_DETECT, ADI_ATTR_TYPE_STRING),
    ADC_CHN_AVAIL_ATTR("start_tamper_detect_available", ADC_EXAMPLE_ATTR_ID_START_DETECT_AVAIL,
                       ADI_ATTR_TYPE_STRING),
    ADC_CHN_ATTR("stop_tamper_detect", ADC_EXAMPLE_ATTR_ID_STOP_DETECT, ADI_ATTR_TYPE_STRING),
    ADC_CHN_AVAIL_ATTR("stop_tamper_detect_available", ADC_EXAMPLE_ATTR_ID_STOP_DETECT_AVAIL,
                       ADI_ATTR_TYPE_STRING),
    ADC_CHN_ATTR("tamper_count", ADC_EXAMPLE_ATTR_ID_TAMPER_CNT, ADI_ATTR_TYPE_STRING),
    END_ATTRIBUTES_ARRAY};

/** IIO Channels*/
static ChannelParams iioChannels[] = {
    IIO_CHAN("Chan0", 0), IIO_CHAN("Chan1", 1), IIO_CHAN("Chan2", 2), IIO_CHAN("Chan3", 3),
    IIO_CHAN("Chan4", 4), IIO_CHAN("Chan5", 5), IIO_CHAN("Chan6", 6)};

int32_t InitDeviceAttributes(DeviceAttributes *pDeviceAttribute)
{
    uint8_t chn;
    /* Update IIO device init parameters */
    for (chn = 0; chn < NUM_IIO_CHANNELS; chn++)
    {
        chanScan[chn].sign = 's';
        chanScan[chn].realbits = CHN_REAL_BITS;
        chanScan[chn].storagebits = CHN_STORAGE_BITS;
        chanScan[chn].shift = 0;
        chanScan[chn].isBigEndian = false;
    }

    pDeviceAttribute->numChannel = (sizeof(iioChannels) / sizeof((iioChannels)[0]));
    pDeviceAttribute->channels = iioChannels;
    pDeviceAttribute->pAttributes = iioGlobalAttributes;
    pDeviceAttribute->debugRegRWEnable = 1;

    return 0;
}

int32_t Close(void)
{
    return 0;
}

ADC_EXAMPLE *GetAdcExampleInfo(void)
{
    return &adcExample;
}

ADC_EXAMPLE_ATTR_INFO *GetAdcExampleAttrInfo(void)
{
    return &adcExample.adcExampleAttrInfo;
}

int32_t GetChannelAttributeId(char *pAttrName)
{
    uint32_t i;
    for (i = 0; i < sizeof(iioChanAttr) / sizeof(iioChanAttr[0]); i++)
    {
        if (strcmp(iioChanAttr[i].pName, pAttrName) == 0)
        {
            return iioChanAttr[i].id;
        }
    }
    return -1; // Return -1 if attribute not found
}

int32_t GetGlobalAttributeId(char *pAttrName)
{
    uint32_t i;
    for (i = 0; i < sizeof(iioGlobalAttributes) / sizeof(iioGlobalAttributes[0]); i++)
    {
        if (strcmp(iioGlobalAttributes[i].pName, pAttrName) == 0)
        {
            return iioGlobalAttributes[i].id;
        }
    }
    return -1; // Return -1 if attribute not found
}

void ExtractAttributeValue(char *pSrc, int32_t attrId, uint8_t *pValueSize)
{
    uint32_t i;
    ADI_ATTR_TYPE attrType;
    attrType = GetAttributeDataType(attrId);
    switch (attrType)
    {
    case ADI_ATTR_TYPE_INT32:
        ConvertStrToInt32(pSrc);
        *pValueSize = sizeof(int32_t);
        break;
    case ADI_ATTR_TYPE_UINT32:
        ConvertStrtoUnit32(pSrc);
        *pValueSize = sizeof(uint32_t);
        break;
    case ADI_ATTR_TYPE_FLOAT:
        ConvertStrToFloat(pSrc);
        *pValueSize = sizeof(float);
        break;
    case ADI_ATTR_TYPE_UINT8:
        ConvertStrtoUnit8(pSrc);
        *pValueSize = sizeof(uint8_t);
        break;
    case ADI_ATTR_TYPE_STRING:
        *pValueSize = sizeof(char) * (strlen(pSrc) + 1); // +1 for null terminator
        break;
    default:
        break;
    }
}

ADI_ATTR_TYPE GetAttributeDataType(int32_t attrId)
{
    uint32_t i;
    ;
    ADI_ATTR_TYPE attrType;
    bool attrFound = false;
    for (i = 0; i < sizeof(iioGlobalAttributes) / sizeof(iioGlobalAttributes[0]); i++)
    {
        if (iioGlobalAttributes[i].id == attrId)
        {
            attrType = iioGlobalAttributes[i].dataType;
            attrFound = true;
            break;
        }
    }
    if (!attrFound)
    {
        for (i = 0; i < sizeof(iioChanAttr) / sizeof(iioChanAttr[0]); i++)
        {
            if (iioChanAttr[i].id == attrId)
            {
                attrType = iioChanAttr[i].dataType;
                attrFound = true;
                break;
            }
        }
    }

    return attrType;
}
void HostUartRxCallback(uint8_t *pData, uint32_t numBytes)
{
    ADC_EXAMPLE *pExample = &adcExample;
    EXAMPLE_CLI_INFO *pCliInfo = pExample->pCliInfo;
    adi_cli_RxCallback(pCliInfo->hCli, pData, numBytes);
}

void HostUartTxCallback(void)
{
    ADC_EXAMPLE *pExample = &adcExample;
    EXAMPLE_CLI_INFO *pCliInfo = pExample->pCliInfo;
    adi_cli_TxCallback(pCliInfo->hCli);
}

int32_t CliReceiveAsync(void *pInfo, char *pData, uint32_t numBytes)
{
    int32_t status = 0;
    if (pInfo != NULL)
    {
        status = EvbHostCommReceiveAsync(pInfo, (uint8_t *)pData, numBytes);
    }
    return status;
}

int32_t CliTransmitAsync(void *pInfo, uint8_t *pData, uint32_t numBytes)
{
    int32_t status = 0;
    if (pInfo != NULL)
    {
        status = EvbHostCommTransmitAsync(pInfo, pData, numBytes);
    }
    return status;
}

static void InitCircBuff(void)
{
    ADC_EXAMPLE *pExample = &adcExample;
    EXAMPLE_SAMPLES_BUFFER *pSamplesBuffer = &pExample->samplesBuffer;
    // Initialize Circular Buffer
    pSamplesBuffer->pCircBuff = &pSamplesBuffer->circBuff;
    pSamplesBuffer->pCircBuff->pBase = (uint8_t *)&pSamplesBuffer->samplesToSend[0];
    pSamplesBuffer->pCircBuff->nSize = sizeof(pSamplesBuffer->pCircBuff);
    pSamplesBuffer->pCircBuff->nReadIndex = 0;
    pSamplesBuffer->pCircBuff->nWriteIndex = 0;
}

int32_t SetCtxAttributes(CtxAttrType **pCtxAttr, uint32_t *pAttrCount)
{
    CtxAttrType *pContextAttributes;
    uint8_t cnt = 0;
    pContextAttributes = contextBuffer;
    if (!pContextAttributes)
    {
        return -1;
    }
    (pContextAttributes + cnt)->name = "hw_mezzanine";
    (pContextAttributes + cnt)->value = HW_MEZZANINE_NAME;
    cnt++;

    (pContextAttributes + cnt)->name = "hw_carrier";
    (pContextAttributes + cnt)->value = HW_CARRIER;
    cnt++;

    (pContextAttributes + cnt)->name = "fw_version";
    (pContextAttributes + cnt)->value = FIRMWARE_VERSION;
    cnt++;

    *pCtxAttr = pContextAttributes;
    *pAttrCount = cnt;

    return 0;
}

static void PopulateBoardConfig(ADC_BOARD_CONFIG *pAdcBoardConfig)
{
    uint8_t i = 0;
    pAdcBoardConfig->numAdc = 1;

    for (i = 0; i < sizeof(vSlots) / sizeof(vSlots[0]); i++)
    {
        pAdcBoardConfig->voltageSlots[i] = vSlots[i];
    }
    for (i = 0; i < sizeof(iSlots) / sizeof(iSlots[0]); i++)
    {
        pAdcBoardConfig->currentSlots[i] = iSlots[i];
    }

    for (i = 0; i < (pAdcBoardConfig->numAdc); i++)
    {
        pAdcBoardConfig->adcType[i] = ADI_ADC_TYPE_ADEMA127;
    }
    pAdcBoardConfig->adcStreamMode = ADI_ADC_STREAM_MODE_NORM;
    pAdcBoardConfig->adcSamplingRate = APP_CFG_ADC_SAMPLING_RATE;
    pAdcBoardConfig->clkIn = APP_CFG_ADC_MCLK;
    pAdcBoardConfig->decimateBy2 = APP_CFG_ADC_DECIMATION_BY2;
}

ADC_BOARD_CONFIG *AdcExmGetBoardConfig(void)
{
    return &adcExample.adcBoardConfig;
}

int32_t GetIioAttribute(int32_t attrId, int32_t *pChanIdx, char *pDst)
{
    int32_t status = 0;
    uint8_t value[1024];
    uint8_t valueSize = 0;
    ADI_ATTR_TYPE attrType;
    attrType = GetAttributeDataType(attrId);
    if (IsAvailAttr(attrId))
    {
        GetAvailableAttribute(attrId, pDst);
    }
    else if (IsButtonAttr(attrId))
    {
        sprintf(pDst, "%s", "success");
    }
    else if (IsStringAttr(attrId))
    {
        GetStringAttribute(attrId, pChanIdx, pDst);
    }
    else
    {
        GetAttribute(attrId, pChanIdx, &value[0], &valueSize);
        FormatString(pDst, &value, attrType);
    }
    return 0;
}

int32_t GetStringAttribute(int32_t attrId, int32_t *pChanIdx, char *pDst)
{
    int32_t status = 0;
    ADC_EXAMPLE *pExample = &adcExample;
    ADC_EXAMPLE_ATTR_INFO *pExampleAttrInfo = &pExample->adcExampleAttrInfo;
    switch (attrId)
    {
    case ADC_EXAMPLE_ATTR_ID_VERSION:
        sprintf(pDst, "%d.%d.%d", ADI_ADC_MAJOR_REVISION, ADI_ADC_MINOR_REVISION,
                ADI_ADC_PATCH_NUMBER);
        break;
    case ADC_EXAMPLE_ATR_ID_BUILD_ID:
        // Slice '0x' from the macro
        sprintf(pDst, "%s", &STR(ADI_ADC_BUILD_HASH)[2]);

        break;
    case EXAMPLE_ATTR_ID_EVB_TYPE:
        sprintf(pDst, "%s", pExampleAttrInfo->pEvbType);
        break;
    default:
        return -1;
    }
    return 0;
}

int32_t GetAvailableAttribute(uint32_t attrId, char *pDst)
{
    uint8_t val;
    int32_t len;
    pDst[0] = '\0';
    switch (attrId)
    {
    case ADC_EXAMPLE_ATTR_ID_EVB_TYPE_AVAIL:
        for (val = 0; val < NUM_EVB_AVAILABLE; val++)
        {
            strcat(pDst, pEvbAvailable[val]);
            strcat(pDst, " ");
        }
        break;

    case ADC_EXAMPLE_ATTR_ID_CHOOSE_SETTINGS_AVAIL:
        for (val = ADC_EXAMPLE_SETTINGS_TYPE_RECOMMENDED_SETTINGS;
             val <= ADC_EXAMPLE_SETTINGS_TYPE_DEFAULT_ADEMA127; val++)
        {
            strcat(pDst, pAdcExampleSettings[val]);
            strcat(pDst, " ");
        }
        break;
    case ADC_EXAMPLE_ATTR_ID_APPLY_SETTING_AVAIL:
        sprintf(pDst, "%s", "Set_config");
        break;

    case ADC_EXAMPLE_ATTR_ID_START_DETECT_AVAIL:
        sprintf(pDst, "%s", "start_detect");
        break;

    case ADC_EXAMPLE_ATTR_ID_STOP_DETECT_AVAIL:
        sprintf(pDst, "%s", "stop_detect");
        break;
    }
    /* Remove extra trailing space at the end of the buffer string */
    len = strlen(pDst);
    pDst[len - 1] = '\0';

    return len;
}

float ConvertStrToFloat(const char *pStr)
{
    char *pEnd;
    float value = strtod(pStr, &pEnd);
    memcpy(pStr, &value, sizeof(float));
    if (pEnd == pStr)
    {
        return -EINVAL;
    }
    else
    {
        return value;
    }
}

int32_t ConvertStrToInt32(const char *pStr)
{
    char *pEnd;
    int32_t value = strtol(pStr, &pEnd, 0);
    memcpy(pStr, &value, sizeof(int32_t));
    if (pEnd == pStr)
    {
        return -EINVAL;
    }
    else
    {
        return value;
    }
}

uint32_t ConvertStrtoUnit32(const char *pStr)
{
    char *pEnd;
    uint32_t value = strtoul(pStr, &pEnd, 0);
    memcpy(pStr, &value, sizeof(uint32_t));
    if (pEnd == pStr)
    {
        return -1;
    }
    else
    {
        return value;
    }
}

uint8_t ConvertStrtoUnit8(const char *pStr)
{
    char *pEnd;
    uint8_t value = (uint8_t)strtoul(pStr, &pEnd, 0);
    memcpy(pStr, &value, sizeof(uint8_t));
    if (pEnd == pStr)
    {
        return -1;
    }
    else
    {
        return value;
    }
}

void FormatString(char *pDst, uint8_t *pValue, ADI_ATTR_TYPE attrType)
{
    int32_t val;
    uint32_t uval;
    float floatVal;
    uint8_t uval8;
    char *pStrVal;
    switch (attrType)
    {
    case ADI_ATTR_TYPE_INT32:
        val = *(int32_t *)pValue;
        sprintf(pDst, "%.2f", (float)val);
        break;
    case ADI_ATTR_TYPE_UINT32:
        uval = *(uint32_t *)pValue;
        sprintf(pDst, "%.2f", (float)uval);
        break;
    case ADI_ATTR_TYPE_FLOAT:
        floatVal = *(float *)pValue;
        sprintf(pDst, "%.5f", (float)floatVal);
        break;
    case ADI_ATTR_TYPE_UINT8:
        uval8 = *(uint8_t *)pValue;
        sprintf(pDst, "%.2f", (float)uval8);
        break;
    case ADI_ATTR_TYPE_STRING:
        pStrVal = (char *)pValue;
        sprintf(pDst, "%s", pStrVal);
        break;
    default:
        sprintf(pDst, "Unknown type");
        break;
    }
}

int32_t IsAvailAttr(int32_t attrId)
{
    return (attrId == ADC_EXAMPLE_ATTR_ID_EVB_TYPE_AVAIL ||
            attrId == ADC_EXAMPLE_ATTR_ID_CHOOSE_SETTINGS_AVAIL ||
            attrId == ADC_EXAMPLE_ATTR_ID_APPLY_SETTING_AVAIL ||
            attrId == ADC_EXAMPLE_ATTR_ID_START_DETECT_AVAIL ||
            attrId == ADC_EXAMPLE_ATTR_ID_STOP_DETECT_AVAIL);
}

int32_t IsButtonAttr(int32_t attrId)
{
    return (attrId == ADC_EXAMPLE_ATTR_ID_APPLY_SETTING ||
            attrId == ADC_EXAMPLE_ATTR_ID_START_DETECT ||
            attrId == ADC_EXAMPLE_ATTR_ID_STOP_DETECT);
}

int32_t IsStringAttr(int32_t attrId)
{
    return (attrId == ADC_EXAMPLE_ATTR_ID_VERSION || attrId == ADC_EXAMPLE_ATR_ID_BUILD_ID ||
            attrId == EXAMPLE_ATTR_ID_EVB_TYPE);
}

/**
 * @}
 */