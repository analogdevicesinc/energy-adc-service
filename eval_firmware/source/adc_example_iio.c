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
#include "adc_example_attributes.h"
#include "adi_cli.h"
#include "board_cfg.h"
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "ADEMA127_addr_def.h"
#include "ADEMA127_addr_rdef.h"
#include "adc_datapath_cfg.h"
#include "adc_example.h"
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
#include "cli_interface.h"
#include "iiod_dispatch_table.h"
#include "message.h"
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/*=============  D E F I N I T I O N S  =============*/

/** Number of active channels */
uint8_t numActiveChannels;

/** List of channels to be captured */
static volatile uint8_t activeChannels[NUM_IIO_CHANNELS];

/** IIOD channels scan parameters */
static scanType chanScan[NUM_IIO_CHANNELS];

/**
 *  Top level structure for ADC
 */
static ADC_EXAMPLE adcExample;
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

/** List of EVBs supported */
static char *pEvbAvailable[] = {"EVAL-ADEMA127KTZ", "_"};

/** List of HPF Cutoff frequencies available */
static char *pAdcHpfCutoffFreq[] = {"10Hz", "5Hz", "2.5Hz", "1.25Hz"};

/** List of HPF Cutoff frequencies available */
static char *pAdcExampleSettings[] = {"recommended", "default_adema127"};

/** Uart Info */
int32_t uartInfo;
static Args args;

/** Macro indicating the end of the array. */
#define END_ATTRIBUTES_ARRAY                                                                       \
    {                                                                                              \
        .pName = NULL                                                                              \
    }

/** Macro for the buffer size */
#define MAX_LINEAR_BUFFER_SIZE 256

/** IIO Context Buffer */
SECTION(DMA_BUFFER) static CtxAttrType contextBuffer[XML_DESC_BUFFER_SIZE];
/** buffer to store the waveform data */
SECTION(DMA_BUFFER) static int32_t linearBuffer[MAX_LINEAR_BUFFER_SIZE];
/** Voltage slots */
static uint8_t vSlots[3] = {2, 6, 3};
/** Current slots */
static uint8_t iSlots[4] = {0, 1, 4, 5};
static void PopulateExamplePointers(ADC_EXAMPLE *pExample);
/*============= F U N C T I O N S =============*/

static void UpdateAdcErrorStatus(ADI_ADC_STATUS_OUTPUT *pAdcStatusOutput, uint8_t numAdc);
static void ProcessInitCommand(ADC_EXAMPLE *pExample);
static void ApplySettings(ADC_EXAMPLE *pExample);
static double ConvertStrToDouble(const char *pStr);
static int32_t ConvertStrToInt32(const char *pStr);
static void SetChanGainAttr(char *pBuf, uint8_t *pChanIdx);
static void SetChanXtGainAttr(char *pBuf, uint8_t *pChanIdx);
static void SetChanXtAggressorAttr(char *pBuf, uint8_t *pChanIdx);
static void SetChanOffsetAttr(char *pBuf, uint8_t *pChanIdx);
static void SetChanIntegerSampleDelayAttr(char *pBuf, uint8_t *pChanIdx);
static void SetChanShiftAttr(char *pBuf, uint8_t *pChanIdx);
static void HostUartRxCallback(void);
static void HostUartTxCallback(void);
static int32_t CliReceiveAsync(void *pInfo, char *pData, uint32_t numBytes);
static int32_t CliTransmitAsync(void *pInfo, uint8_t *pData, uint32_t numBytes);
static uint32_t ConvertStrtoUnit32(const char *pSrc);
static uint8_t ConvertStrtoUnit8(const char *pSrc);
static int IioAttrAvailableGet(uint32_t attrId, char *pDst);
static void InitCircBuff();
static int32_t InitDeviceAttributes(DeviceAttributes *pDeviceAttribute);
static uint32_t ExtractDatapathConfig(ADI_ADC_CHAN_DATAPATH_CONFIG dataPathConfig);
static void ResetAdc(void);
static void ChooseSettings(char *pSrc);

static int32_t IsAvailAttr(int32_t attrId);
static int32_t IsButtonAttr(int32_t attrId);
/**
 * @brief Set the context attributes.
 * @param pCtxAttr - pointer to the context attributes
 * @param pAttrCount - pointer to number of context attributes
 */
static int32_t SetCtxAttributes(CtxAttrType **pCtxAttr, uint32_t *pAttrCount);
static int32_t InitIio(void);

/*=============  C O D E  =============*/

ADC_EXAMPLE_STATUS InitServices(void)
{
    ADC_EXAMPLE_STATUS status = ADC_EXAMPLE_STATUS_SUCCESS;
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    int32_t ifStatus;
    int32_t boardStatus = ADI_EVB_STATUS_SUCCESS;
    ADC_EXAMPLE *pExample = &adcExample;
    ADI_EVB_CONFIG *pEvbConfig = &pExample->evbConfig;
    Args *pArgs = &args;
    ADI_CLI_STATUS cliStatus = ADI_CLI_STATUS_SUCCESS;
    uint8_t numAdc = 1;
    uint32_t i;
    pExample->pAdcIf = &adcIf;
    pExample->pCliInfo = &cliInfo;
    pExample->pXmlDescBuffer = &xmlDescBuffer[0];
    pExample->pXmlSize = &xmlSize[0];
    PopulateExamplePointers(pExample);
    InitCircBuff();
    for (i = 0; i < sizeof(vSlots) / sizeof(vSlots[0]); i++)
    {
        // ACE expects scale in mV units
        pExample->adcChanScale[vSlots[i]] = IEC_3PHASE_METER_V_SCALE * 1000;
        pExample->pAdcIf->adcBoardConfig.voltageSlots[i] = vSlots[i];
    }
    for (i = 0; i < sizeof(iSlots) / sizeof(iSlots[0]); i++)
    {
        // ACE expects scale in mV units
        pExample->adcChanScale[iSlots[i]] = IEC_3PHASE_METER_I_SCALE * 1000;
        pExample->pAdcIf->adcBoardConfig.currentSlots[i] = iSlots[i];
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
            cliStatus = adi_cli_Init(&pExample->pCliInfo->config);
        }
        if (cliStatus != ADI_CLI_STATUS_SUCCESS)
        {
            status = ADC_EXAMPLE_STATUS_CLI_INIT_FAILED;
        }
        pArgs->c = 2;
        pArgs->v[0].pS = "off";
        pArgs->v[1].pS = "off";
        CliCmdManual(pArgs);
        if (InitIio())
        {
            pExample->triggerDevice = 0;
            printf("IIO initialization failure!!\r\n");
        }
        if (status == ADC_EXAMPLE_STATUS_SUCCESS)
        {
            EvbLedOn(0);
            ifStatus = AdcIfCreateService(pExample->pAdcIf);
            if (ifStatus != 0)
            {
                status = ADC_EXAMPLE_STATUS_IF_CREATE_SERVICE_FAILED;
            }
        }

        if (status == 0)
        {
            for (i = 0; i < numAdc; i++)
            {
                pExample->adcTypes[i] = ADI_ADC_TYPE_ADEMA127;
            }
            pExample->pAdcIf->adcStreamMode = ADI_ADC_STREAM_MODE_NORM;
            pExample->pAdcIf->adcSamplingRate = APP_CFG_ADC_SAMPLING_RATE;
            pExample->pAdcIf->clkIn = APP_CFG_ADC_MCLK;
            pExample->pAdcIf->decimateBy2 = APP_CFG_ADC_DECIMATION_BY2;

            adcStatus = AdcIfInitService(pExample->pAdcIf, numAdc, &pExample->adcTypes[0]);
            if (adcStatus != ADI_ADC_STATUS_SUCCESS)
            {
                status = ADC_EXAMPLE_STATUS_ADC_INIT_FAILED;
            }
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
    ADI_EVB_CONFIG *pEvbConfig = &pExample->evbConfig;
    pExample->pEvbType = pEvbAvailable[0];
    pExample->adcVariant = ADI_ADC_TYPE_ADEMA127;
    pExample->numAdc = 1;
    pExample->adcIndex = 0;
    pExample->chanConfig = 0xFFFFFFF;
    pEvbConfig->spiConfig.pfAdeSpiRxCallback = AdcIfSpiRxCallback;
    pEvbConfig->gpioConfig.pfGpioCallback = AdcIfDreadyCallback;
    pEvbConfig->uartConfig.pfHostUartRxCallback = HostUartRxCallback;
    pEvbConfig->uartConfig.pfHostUartTxCallback = HostUartTxCallback;
    pExample->pCliInfo->config.pfTransmitAsync = CliTransmitAsync;
    pExample->pCliInfo->config.pfReceiveAsync = CliReceiveAsync;
    pExample->pCliInfo->config.pDispatchTable = dispatchTable;
    pExample->pCliInfo->config.numRecords = NUM_COMMANDS;
    pExample->pHpfCutoffFreq = pAdcHpfCutoffFreq[0];
    pExample->pExampleSettings = pAdcExampleSettings[0];
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
    EvbHostUartTransmitAsync(&uartInfo, (uint8_t *)&pExample->pXmlSize[0], length + 1);
    pIioDescBuffer->pXmlDesc[pIioDescBuffer->xmlSize] = '\n';
    EvbHostUartTransmitAsync(&uartInfo, (uint8_t *)pIioDescBuffer->pXmlDesc,
                             pIioDescBuffer->xmlSize + 1);
}

static void UpdateAdcErrorStatus(ADI_ADC_STATUS_OUTPUT *pAdcStatusOutput, uint8_t numAdc)
{
    uint8_t i;

    for (i = 0; i < numAdc; i++)
    {
        /* Check if any frame had ADC CRC error */
        if (pAdcStatusOutput[i].crcError != 0)
        {
            EvbLedOn(2);
            break;
        }
    }
}

int32_t IioSubmitBuffer(int32_t numBytes)
{
    ADI_ADC_STATUS adcStatus = ADI_ADC_STATUS_SUCCESS;
    // no of samples to collect from all channels
    uint32_t numSamplesRequired = numBytes / APP_CFG_BYTES_PER_SAMPLE;
    uint32_t samplesCopied = 0;
    uint32_t numSamplesSent = 0;
    ADC_EXAMPLE *pExample = &adcExample;
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
        adcStatus =
            AdcIfCollectSamples(pAdcIf, pExample->chanConfig, numSamplesPerIteration, pBlockBuffer);
        if (adcStatus == ADI_ADC_STATUS_SUCCESS)
        {
            if (samplesCopied < numSamplesRequired)
            {
                ADICircBufWrite(pExample->samplesBuffer.pCircBuff, (uint8_t *)pBlockBuffer,
                                numSamplesPerIteration * APP_CFG_BYTES_PER_SAMPLE);
            }

            // Transmits one block of samples for all channels.
            txComplete = EvbGetTxStatus();
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
                EvbHostUartTransmitAsync(&uartInfo, (uint8_t *)&linearBuffer[0],
                                         txBlockSize * APP_CFG_BYTES_PER_SAMPLE);
                numSamplesSent += txBlockSize;
            }
            samplesCopied += numSamplesPerIteration;
        }
    }
    AdcIfStopCapture(pAdcIf);
    return 0;
}

int32_t DebugRegRead(uint32_t address, uint32_t *pDst)
{
    ADI_ADC_STATUS adcStatus;
    ADC_EXAMPLE *pExample = &adcExample;
    ADC_INTERFACE_INFO *pAdcIf = pExample->pAdcIf;
    int32_t adcIdx = pExample->adcIndex;
    uint16_t regAddr = 0;
    uint32_t numBytesToSend = 0;
    pExample->addr = (uint16_t)address;
    regAddr = pExample->addr;

    if (address >= ADDR_ADEMA127_DSP_RAM_CH0_COMP_COEFF_B0_LO &&
        address <= ADDR_ADEMA127_DSP_RAM_ALL_HPF_COEFF_A2_2)
    {
        AdcIfAccessDspMem(pAdcIf, 1, adcIdx);
    }

    adcStatus = AdcIfReadRegister(pAdcIf, regAddr, adcIdx, (uint8_t *)&pExample->registerValue[0],
                                  (uint32_t *)&numBytesToSend);

    if (address >= ADDR_ADEMA127_DSP_RAM_CH0_COMP_COEFF_B0_LO &&
        address <= ADDR_ADEMA127_DSP_RAM_ALL_HPF_COEFF_A2_2)
    {
        AdcIfAccessDspMem(pAdcIf, 0, adcIdx);
    }
    if (adcStatus != ADI_ADC_STATUS_SUCCESS)
    {
        EvbLedOn(2);
    }
    else
    {
        // Send out the reg_val instead of [(reg + 1)_val and reg_val]
        *pDst = (uint8_t)pExample->registerValue[adcIdx * pExample->numAdc + 1];
    }
    UpdateAdcErrorStatus(&pAdcIf->adcStatusOutput[0], 1);

    return 0;
}

int32_t DebugRegWrite(uint32_t address, uint32_t value)
{
    ADI_ADC_STATUS adcStatus;
    ADC_EXAMPLE *pExample = &adcExample;
    ADC_INTERFACE_INFO *pAdcIf = pExample->pAdcIf;
    int32_t adcIdx = pExample->adcIndex;

    if (address == ADDR_ADEMA127_MMR_DATAPATH_CONFIG_LOCK && value == 0)
    {
        // Save DSP RAM region before datapath config unlock
        adcStatus =
            AdcIfGetDspRegisterStruct(pAdcIf, pAdcIf->adcCfg.numAdc, pAdcIf->adcCfg.adcType);
    }
    /** It is recommended to update ADC_PD_CHx, ADC_GAIN_CHx, ADC_INV_CHx, ADC_CMI_CHx  while ADC
     * conversion is halted */
    if ((address >= ADDR_ADEMA127_MMR_DATARATE &&
         address <= ADDR_ADEMA127_MMR_PHASE_OFFSET_CH6_LO) ||
        (address >= ADDR_ADEMA127_MMR_RETAINED_ADC_PD &&
         address <= ADDR_ADEMA127_MMR_RETAINED_ADC_INV))
    {
        AdcIfConfigDspLock(pAdcIf, 0, adcIdx);
    }

    if (address >= ADDR_ADEMA127_DSP_RAM_CH0_COMP_COEFF_B0_LO &&
        address <= ADDR_ADEMA127_DSP_RAM_ALL_HPF_COEFF_A2_2)
    {
        AdcIfAccessDspMem(pAdcIf, 1, adcIdx);
    }

    adcStatus = AdcIfWriteRegister(pAdcIf, (uint16_t)address, (uint8_t)value, adcIdx);

    if ((address >= ADDR_ADEMA127_MMR_DATARATE &&
         address <= ADDR_ADEMA127_MMR_PHASE_OFFSET_CH6_LO) ||
        (address >= ADDR_ADEMA127_MMR_RETAINED_ADC_PD &&
         address <= ADDR_ADEMA127_MMR_RETAINED_ADC_INV))
    {
        AdcIfConfigDspLock(pAdcIf, 1, adcIdx);
    }

    if (address == ADDR_ADEMA127_MMR_DATAPATH_CONFIG_LOCK && value == 1)
    {
        // Reload DSP RAM after datapath config is locked
        adcStatus =
            AdcIfPopulateDspRegisterStruct(pAdcIf, pAdcIf->adcCfg.numAdc, pAdcIf->adcCfg.adcType);
    }

    if (address >= ADDR_ADEMA127_DSP_RAM_CH0_COMP_COEFF_B0_LO &&
        address <= ADDR_ADEMA127_DSP_RAM_ALL_HPF_COEFF_A2_2)
    {
        AdcIfAccessDspMem(pAdcIf, 0, adcIdx);
    }

    if (adcStatus != ADI_ADC_STATUS_SUCCESS)
    {
        EvbLedOn(2);
    }

    return 0;
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

int IIoAttrGet(int32_t attrId, int32_t *pChannel, char *pDst)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    float val;
    uint8_t chanShift;
    ADI_ADC_CHAN_XT_AGGRESSOR chanXtAggressor = {0};
    uint8_t xtAggressor;
    int32_t chanOffset;
    ADC_EXAMPLE *pExample = &adcExample;
    ADC_INTERFACE_INFO *pAdcIf = pExample->pAdcIf;
    ADI_ADC_DSP_DATAPATH_PARAMS *pDatapathParams = &pExample->adcDatapathParams;
    uint32_t chanDataPathVal;

    if (IsAvailAttr(attrId))
    {
        return IioAttrAvailableGet(attrId, pDst);
    }
    if (IsButtonAttr(attrId))
    {
        sprintf(pDst, "%s", "success");
        return status;
    }

    switch (attrId)
    {
    case ADC_EXAMPLE_ATTR_ID_VERSION:
        sprintf(pDst, "%d.%d.%d", ADI_ADC_MAJOR_REVISION, ADI_ADC_MINOR_REVISION,
                ADI_ADC_PATCH_NUMBER);
        break;
    case ADC_EXAMPLE_ATR_ID_BUILD_ID:
        // Slice '0x' from the macro
        sprintf(pDst, "%s", STR(ADI_ADC_BUILD_HASH) + 2);
        break;
    case ADC_EXAMPLE_ATTR_ID_EVB_TYPE:
        sprintf(pDst, "%s", pExample->pEvbType);
        break;
    case ADC_EXAMPLE_ATTR_ID_CHAN_GAIN:
        status = AdcIfGetGain(pAdcIf, (uint8_t *)pChannel, 1, pExample->adcIndex, &val);
        sprintf(pDst, "%.5f", val);
        break;
    case ADC_EXAMPLE_ATTR_ID_CHAN_XT_GAIN:
        status = AdcIfGetXtGain(pAdcIf, (uint8_t *)pChannel, 1, pExample->adcIndex, &val);
        sprintf(pDst, "%.5f", val);
        break;
    case ADC_EXAMPLE_ATTR_ID_CHAN_XT_AGGRESSOR:
        status = AdcIfGetXtAggressor(pAdcIf, (uint8_t *)pChannel, 1, pExample->adcIndex,
                                     &chanXtAggressor);
        memcpy(&xtAggressor, &chanXtAggressor, sizeof(ADI_ADC_CHAN_XT_AGGRESSOR));
        sprintf(pDst, "%.2f", (float)xtAggressor);
        break;
    case ADC_EXAMPLE_ATTR_ID_CHAN_OFFSET:
        status = AdcIfGetOffset(pAdcIf, (uint8_t *)pChannel, 1, pExample->adcIndex, &chanOffset);
        sprintf(pDst, "%.2f", (float)chanOffset);
        break;
    case ADC_EXAMPLE_ATTR_ID_CHAN_SHIFT:
        status = AdcIfGetShift(pAdcIf, (uint8_t *)pChannel, 1, pExample->adcIndex, &chanShift);
        sprintf(pDst, "%.2f", (float)chanShift);
        break;
    case ADC_EXAMPLE_ATTR_ID_DATAPATH_CONFIG:
        status = adi_adc_GetDatapathParams(pAdcIf->hAdc, (uint8_t *)pChannel, 1, pExample->adcIndex,
                                           pDatapathParams);
        chanDataPathVal = ExtractDatapathConfig(pDatapathParams->dataPathConfig[0]);
        sprintf(pDst, "%.2f", (float)chanDataPathVal);
        break;
    case ADC_EXAMPLE_ATTR_ID_CHAN_SCALE:
        sprintf(pDst, "%.5f", pExample->adcChanScale[*pChannel]);
        break;
    case ADC_EXAMPLE_ATTR_ID_CHOOSE_SETTINGS:
        sprintf(pDst, "%s", pExample->pExampleSettings);
        break;
    case ADC_EXAMPLE_ATTR_ID_TAMPER_CNT:
        sprintf(pDst, "%ld", pAdcIf->tamperCnt);
        break;
    case ADC_EXAMPLE_ATTR_ID_CHAN_INTEGER_SAMPLE_DELAY:
        status = adi_adc_GetConfig(pAdcIf->hAdc, &pAdcIf->adcCfg);
        val = pAdcIf->adcCfg.integerSampleDelay[*pChannel];
        if (status != ADI_ADC_STATUS_SUCCESS)
        {
        }
        sprintf(pDst, "%.8f", val);
        break;
    default:
        return -1;
    }

    return 0;
}

uint32_t ConvertStrtoUnit32(const char *pSrc)
{
    char *pEnd;
    uint32_t value = strtoul(pSrc, &pEnd, 0);

    if (pEnd == pSrc)
    {
        return -1;
    }
    else
    {
        return value;
    }
}

uint8_t ConvertStrtoUnit8(const char *pSrc)
{
    char *pEnd;
    uint8_t value = (uint8_t)strtoul(pSrc, &pEnd, 0);

    if (pEnd == pSrc)
    {
        return -1;
    }
    else
    {
        return value;
    }
}

int IioAttrSet(int32_t attrId, uint8_t *pChanIdx, char *pSrc)
{
    ADC_EXAMPLE *pExample = &adcExample;
    uint32_t writeVal;
    ADC_INTERFACE_INFO *pAdcIf = pExample->pAdcIf;
    double value;
    switch (attrId)
    {
    case ADC_EXAMPLE_ATTR_ID_EVB_TYPE:
        if (strcmp(pSrc, pEvbAvailable[0]) == 0)
        {
            pExample->adcVariant = ADI_ADC_TYPE_ADEMA127;
            pExample->numAdc = 1;
            pExample->adcIndex = 0;
            pExample->chanConfig = 0xFFFFFFF;
        }
        break;

    case ADC_EXAMPLE_ATTR_ID_CHAN_GAIN:
        SetChanGainAttr(pSrc, pChanIdx);
        break;

    case ADC_EXAMPLE_ATTR_ID_CHAN_XT_GAIN:
        SetChanXtGainAttr(pSrc, pChanIdx);
        break;

    case ADC_EXAMPLE_ATTR_ID_CHAN_XT_AGGRESSOR:
        SetChanXtAggressorAttr(pSrc, pChanIdx);
        break;

    case ADC_EXAMPLE_ATTR_ID_CHAN_OFFSET:
        SetChanOffsetAttr(pSrc, pChanIdx);
        break;

    case ADC_EXAMPLE_ATTR_ID_CHAN_SHIFT:
        SetChanShiftAttr(pSrc, pChanIdx);
        break;

    case ADC_EXAMPLE_ATTR_ID_DATAPATH_CONFIG:
        writeVal = ConvertStrtoUnit32(pSrc);
        SetDatapathConfig(pAdcIf, pChanIdx, pExample->adcIndex, writeVal);
        break;
    case ADC_EXAMPLE_ATTR_ID_CHAN_SCALE:
        value = ConvertStrToDouble(pSrc);
        pExample->adcChanScale[*pChanIdx] = value;
        break;

    case ADC_EXAMPLE_ATTR_ID_CHOOSE_SETTINGS:
        ChooseSettings(pSrc);
        break;

    case ADC_EXAMPLE_ATTR_ID_START_DETECT:
        StartTamperDetection();
        break;

    case ADC_EXAMPLE_ATTR_ID_STOP_DETECT:
        StopTamperDetection();
        break;

    case ADC_EXAMPLE_ATTR_ID_APPLY_SETTING:
        ApplySettings(pExample);
        break;

    case ADC_EXAMPLE_ATTR_ID_CHAN_INTEGER_SAMPLE_DELAY:
        SetChanIntegerSampleDelayAttr(pSrc, pChanIdx);
        break;

    default:
        return -EINVAL;
    }

    return 0;
}

int IioAttrAvailableGet(uint32_t attrId, char *pDst)
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
        for (val = ADC_EXAMPLE_RECOMMENDED_SETTINGS; val <= ADC_EXAMPLE_DEFAULT_ADEMA127; val++)
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

ADC_EXAMPLE_STATUS ProcessCommand(void)
{
    ADC_EXAMPLE_STATUS status = ADC_EXAMPLE_STATUS_SUCCESS;
    if (adi_cli_FlushMessages() == 0)
    {
        adi_cli_Interface();
    }
    return status;
}

void ProcessInitCommand(ADC_EXAMPLE *pExample)
{
    uint32_t totalSampleBlockSize;
    uint32_t numMaxBlocks;
    for (int i = 0; i < pExample->numAdc; i++)
    {
        // FIX ME: How to configure different adcTypes in IIO?
        pExample->adcTypes[i] = (ADI_ADC_TYPE)pExample->adcVariant;
    }
    pExample->samplesBuffer.pCircBuff->nReadIndex = 0;
    pExample->samplesBuffer.pCircBuff->nWriteIndex = 0;
    AdcIfInitService(pExample->pAdcIf, pExample->numAdc, &pExample->adcTypes[0]);
    totalSampleBlockSize =
        pExample->pAdcIf->adcCfg.numSamplesInBlock * pExample->pAdcIf->runInfo.totalChannels;
    numMaxBlocks = ADC_EXM_MAX_SAMPLES_TO_STORE / totalSampleBlockSize;
    pExample->samplesBuffer.pCircBuff->nSize = pExample->pAdcIf->adcCfg.numSamplesInBlock *
                                               pExample->pAdcIf->runInfo.totalChannels *
                                               numMaxBlocks * APP_CFG_BYTES_PER_SAMPLE;
}

void ApplySettings(ADC_EXAMPLE *pExample)
{

    if (pExample->settings == ADC_EXAMPLE_RECOMMENDED_SETTINGS)
    {
        ProcessInitCommand(pExample);
    }
    else if (pExample->settings == ADC_EXAMPLE_DEFAULT_ADEMA127)
    {
        ResetAdc();
    }
}

void ChooseSettings(char *pSrc)
{
    uint8_t val;
    ADC_EXAMPLE *pExample = &adcExample;
    char *pTrimmedSrc = pSrc;
    while (*pTrimmedSrc == '\n' || *pTrimmedSrc == '\r' || *pTrimmedSrc == ' ')
    {
        pTrimmedSrc++;
    }
    for (val = ADC_EXAMPLE_RECOMMENDED_SETTINGS; val <= ADC_EXAMPLE_DEFAULT_ADEMA127; val++)
    {
        if (!strcmp(pTrimmedSrc, pAdcExampleSettings[val]))
        {
            break;
        }
    }
    pExample->pExampleSettings = pTrimmedSrc;
    pExample->settings = (ADC_EXAMPLE_SETTINGS_TYPE)val;
}

void SetChanGainAttr(char *pBuf, uint8_t *pChanIdx)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    ADC_EXAMPLE *pExample = &adcExample;
    ADC_INTERFACE_INFO *pAdcIf = pExample->pAdcIf;
    float value;
    value = ConvertStrToDouble(pBuf);
    status = AdcIfSetGain(pAdcIf, &value, pChanIdx, 1, pExample->adcIndex);
    if (status != ADI_ADC_STATUS_SUCCESS)
    {
    }
}

void SetChanXtGainAttr(char *pBuf, uint8_t *pChanIdx)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    ADC_EXAMPLE *pExample = &adcExample;
    ADC_INTERFACE_INFO *pAdcIf = pExample->pAdcIf;
    float value;
    value = ConvertStrToDouble(pBuf);
    status = AdcIfSetXtGain(pAdcIf, &value, pChanIdx, 1, pExample->adcIndex);
    if (status != ADI_ADC_STATUS_SUCCESS)
    {
    }
}
void SetChanXtAggressorAttr(char *pBuf, uint8_t *pChanIdx)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    ADC_EXAMPLE *pExample = &adcExample;
    ADC_INTERFACE_INFO *pAdcIf = pExample->pAdcIf;
    ADI_ADC_CHAN_XT_AGGRESSOR aggrValue;
    uint8_t value;
    value = ConvertStrtoUnit32(pBuf);
    memcpy(&aggrValue, &value, sizeof(ADI_ADC_CHAN_XT_AGGRESSOR));
    status = AdcIfSetXtAggressor(pAdcIf, &aggrValue, pChanIdx, 1, pExample->adcIndex);
    if (status != ADI_ADC_STATUS_SUCCESS)
    {
    }
}

void SetChanOffsetAttr(char *pBuf, uint8_t *pChanIdx)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    ADC_EXAMPLE *pExample = &adcExample;
    ADC_INTERFACE_INFO *pAdcIf = pExample->pAdcIf;
    int32_t value;
    value = ConvertStrToInt32(pBuf);
    status = AdcIfSetOffset(pAdcIf, &value, pChanIdx, 1, pExample->adcIndex);
    if (status != ADI_ADC_STATUS_SUCCESS)
    {
    }
}

void SetChanShiftAttr(char *pBuf, uint8_t *pChanIdx)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    ADC_EXAMPLE *pExample = &adcExample;
    ADC_INTERFACE_INFO *pAdcIf = pExample->pAdcIf;
    uint8_t value;
    value = ConvertStrtoUnit32(pBuf);
    status = AdcIfSetShift(pAdcIf, &value, pChanIdx, 1, pExample->adcIndex);
    if (status != ADI_ADC_STATUS_SUCCESS)
    {
    }
}

void SetChanIntegerSampleDelayAttr(char *pBuf, uint8_t *pChanIdx)
{
    ADI_ADC_STATUS status = ADI_ADC_STATUS_SUCCESS;
    ADC_EXAMPLE *pExample = &adcExample;
    ADC_INTERFACE_INFO *pAdcIf = pExample->pAdcIf;
    uint8_t value;
    value = ConvertStrtoUnit8(pBuf);
    status = adi_adc_SetIntegerSampleDelay(pAdcIf->hAdc, &value, pChanIdx, 1, pExample->adcIndex);
    if (status != ADI_ADC_STATUS_SUCCESS)
    {
    }
}

double ConvertStrToDouble(const char *pStr)
{
    char *pEnd;
    double value = strtod(pStr, &pEnd);

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

    if (pEnd == pStr)
    {
        return -EINVAL;
    }
    else
    {
        return value;
    }
}

uint32_t ExtractDatapathConfig(ADI_ADC_CHAN_DATAPATH_CONFIG dataPathConfig)
{
    uint8_t result = 0;
    result |= (dataPathConfig.gainOffsetEn & 0x01);
    result |= (dataPathConfig.scfEn & 0x01) << 1;
    result |= (dataPathConfig.hpfEn & 0x01) << 2;
    result |= (dataPathConfig.compFiltEn & 0x01) << 3;
    result |= (dataPathConfig.compFiltCfg & 0x01) << 4;
    result |= (dataPathConfig.lpfEn & 0x01) << 5;
    result |= (dataPathConfig.allPassEn & 0x01) << 6;
    return result;
}

void ResetAdc()
{
    int32_t status = ADI_EVB_STATUS_SUCCESS;
    ADC_EXAMPLE *pExample = &adcExample;
    ADC_INTERFACE_INFO *pAdcIf = pExample->pAdcIf;
    // When ADC is reset,  all registers except DSP RAM addresses are reset.
    // Hence, lock datapath config to reset DSP RAM region.
    AdcIfStopCapture(pAdcIf);
    status =
        AdcIfWriteRegister(pAdcIf, ADDR_ADEMA127_MMR_DATAPATH_CONFIG_LOCK, 0, pExample->adcIndex);
    if (status == ADI_ADC_STATUS_SUCCESS)
    {
        status = AdcIfWriteRegister(pAdcIf, ADDR_ADEMA127_MMR_DATAPATH_CONFIG_LOCK, 1,
                                    pExample->adcIndex);
        EvbDelayMs(1);
    }
    status = EvbResetAdcs();
    if (status != ADI_ADC_STATUS_SUCCESS)
    {
    }
}

/** IIO Channel macro*/
#define IIO_CHAN(nm, chn)                                                                          \
    {                                                                                              \
        .pName = nm, .channelType = IIO_VOLTAGE, .ch_out = false, .indexed = true, .channel = chn, \
        .scan_index = chn, .pScanType = &chanScan[chn], .pAttributes = iioChanAttr                 \
    }

/** IIO Channel attribute macro*/
#define ADC_CHN_ATTR(_name, _priv)                                                                 \
    {                                                                                              \
        .pName = _name, .id = _priv                                                                \
    }

/** IIO Channel attribute available macro*/
#define ADC_CHN_AVAIL_ATTR(_name, _priv)                                                           \
    {                                                                                              \
        .pName = _name, .id = _priv                                                                \
    }

/** IIOD channels attributes list */
static AttributeType iioChanAttr[] = {
    ADC_CHN_ATTR("chn_gain", ADC_EXAMPLE_ATTR_ID_CHAN_GAIN),
    ADC_CHN_ATTR("chn_datapath_config", ADC_EXAMPLE_ATTR_ID_DATAPATH_CONFIG),
    ADC_CHN_ATTR("chn_offset", ADC_EXAMPLE_ATTR_ID_CHAN_OFFSET),
    ADC_CHN_ATTR("chn_xt_gain", ADC_EXAMPLE_ATTR_ID_CHAN_XT_GAIN),
    ADC_CHN_ATTR("chn_xt_aggressor", ADC_EXAMPLE_ATTR_ID_CHAN_XT_AGGRESSOR),
    ADC_CHN_ATTR("chn_shift", ADC_EXAMPLE_ATTR_ID_CHAN_SHIFT),
    ADC_CHN_ATTR("chn_integer_sample_delay", ADC_EXAMPLE_ATTR_ID_CHAN_INTEGER_SAMPLE_DELAY),
    ADC_CHN_ATTR("scale", ADC_EXAMPLE_ATTR_ID_CHAN_SCALE),
    END_ATTRIBUTES_ARRAY};

/** IIOD device (global) attributes list */
static AttributeType iioGlobalAttributes[] = {
    ADC_CHN_ATTR("firmware_version", ADC_EXAMPLE_ATTR_ID_VERSION),
    ADC_CHN_ATTR("firmware_build_id", ADC_EXAMPLE_ATR_ID_BUILD_ID),
    ADC_CHN_ATTR("board_type", ADC_EXAMPLE_ATTR_ID_EVB_TYPE),
    ADC_CHN_AVAIL_ATTR("board_type_available", ADC_EXAMPLE_ATTR_ID_EVB_TYPE_AVAIL),
    ADC_CHN_ATTR("choose_settings", ADC_EXAMPLE_ATTR_ID_CHOOSE_SETTINGS),
    ADC_CHN_AVAIL_ATTR("choose_settings_available", ADC_EXAMPLE_ATTR_ID_CHOOSE_SETTINGS_AVAIL),
    ADC_CHN_ATTR("apply_settings", ADC_EXAMPLE_ATTR_ID_APPLY_SETTING),
    ADC_CHN_AVAIL_ATTR("apply_settings_available", ADC_EXAMPLE_ATTR_ID_APPLY_SETTING_AVAIL),
    ADC_CHN_ATTR("start_tamper_detect", ADC_EXAMPLE_ATTR_ID_START_DETECT),
    ADC_CHN_AVAIL_ATTR("start_tamper_detect_available", ADC_EXAMPLE_ATTR_ID_START_DETECT_AVAIL),
    ADC_CHN_ATTR("stop_tamper_detect", ADC_EXAMPLE_ATTR_ID_STOP_DETECT),
    ADC_CHN_AVAIL_ATTR("stop_tamper_detect_available", ADC_EXAMPLE_ATTR_ID_STOP_DETECT_AVAIL),
    ADC_CHN_ATTR("tamper_count", ADC_EXAMPLE_ATTR_ID_TAMPER_CNT),
    END_ATTRIBUTES_ARRAY};

/** IIO Channels*/
static ChannelParams iioChannels[] = {
    IIO_CHAN("Chan0", 0), IIO_CHAN("Chan1", 1), IIO_CHAN("Chan2", 2), IIO_CHAN("Chan3", 3),
    IIO_CHAN("Chan4", 4), IIO_CHAN("Chan5", 5), IIO_CHAN("Chan6", 6), END_ATTRIBUTES_ARRAY};

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

void HostUartRxCallback()
{
    adi_cli_RxCallback();
}

void HostUartTxCallback(void)
{
    adi_cli_TxCallback();
}

int32_t CliReceiveAsync(void *pInfo, char *pData, uint32_t numBytes)
{
    int32_t status = 0;
    if (pInfo != NULL)
    {
        status = EvbHostUartReceiveAsync(pInfo, (uint8_t *)pData, numBytes);
    }
    return status;
}

int32_t CliTransmitAsync(void *pInfo, uint8_t *pData, uint32_t numBytes)
{
    int32_t status = 0;
    if (pInfo != NULL)
    {
        status = EvbHostUartTransmitAsync(pInfo, pData, numBytes);
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

/**
 * @}
 */
