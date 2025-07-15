/******************************************************************************
 Copyright (c) 2025  Analog Devices Inc.
******************************************************************************/

/*============= I N C L U D E S =============*/
#include "cli_commands.h"
#include "adc_example_iio.h"
#include "adi_cli.h"
#include "adi_evb.h"
#include "message.h"
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>

/** Available config option*/
static char *deviceTypes[] = {"iio:device0"};
/** Available channel option*/
static char *channelTypes[] = {"input", "output"};
/** Available debug option*/
static char *debugTypes[] = {"debug"};
#define MAX_PARAMETER_LENGTH 512
/** Buffer to store the command parameter */
static char commandParam[MAX_PARAMETER_LENGTH];
#define BUFFER_SIZE 1024

/** uart Info */
static int32_t uartInfo;
static int32_t ScanData(int32_t attrLength);
static int32_t WriteToDebugReg();

/** Buffer to store the data to transmit or receive*/
SECTION(DMA_BUFFER) static char buffer[BUFFER_SIZE];

int32_t CmdHelp(Args *pArgs)
{
    int32_t status = 0;
    if (pArgs->c == 0)
    {
    }

    return status;
}

int32_t CmdExit(Args *pArgs)
{
    int32_t status = 0;
    if (pArgs->c == 0)
    {
    }
    return status;
}

int32_t CmdPrint(Args *pArgs)
{
    int32_t status = 0;
    if (pArgs->c == 0)
    {
        PrintIIoInfo();
    }
    else
    {
        status = -1;
    }

    return status;
}

int32_t CmdVersion(Args *pArgs)
{
    int32_t status = 0;
    if (pArgs->c == 0)
    {
        INFO_MSG_RAW("%s\n", IIOD_VERSION)
    }
    else
    {
        status = -1;
    }

    return status;
}

int32_t CmdTimeout(Args *pArgs)
{
    int32_t status = 0;
    if (pArgs->c == 0)
    {
    }

    return status;
}

int32_t CmdOpen(Args *pArgs)
{
    int32_t status = 0;
    int32_t numChoices;
    int32_t choice;
    ADC_EXAMPLE *pExample = GetAdcExampleInfo();
    char *pParam = &commandParam[0];
    if (pArgs->c == 3 || pArgs->c == 4)
    {
        numChoices = sizeof(deviceTypes) / sizeof(deviceTypes[0]);
        choice = GetChoice(deviceTypes, pArgs->v[0].pS, numChoices, pParam);
        if (choice == 0)
        {
            sscanf(pArgs->v[1].pS, "%" PRIi32, &pExample->sampleCount);
            sscanf(pArgs->v[2].pS, "%lx", &pExample->channelMask);
        }
        INFO_MSG_RAW("%d\n", status)
    }
    else
    {
        status = -1;
    }

    return status;
}

int32_t CmdClose(Args *pArgs)
{
    int32_t status = 0;
    if (pArgs->c == 1)
    {
        status = Close();
    }
    else
    {
        status = -1;
    }
    INFO_MSG_RAW("%d\n", status) return status;
}

int32_t CmdRead(Args *pArgs)
{
    int32_t status = 0;
    int32_t numChoices;
    int32_t choice;
    int32_t chanNum;
    int32_t attrId = 0;
    uint32_t debugReadVal;
    int32_t length;
    char attr[20];
    char *pParam = &commandParam[0];
    ADC_EXAMPLE *pExample = GetAdcExampleInfo();
    if (pArgs->c >= 1 && pArgs->c <= 4)
    {
        buffer[0] = '\0';
        numChoices = sizeof(deviceTypes) / sizeof(deviceTypes[0]);
        choice = GetChoice(deviceTypes, pArgs->v[0].pS, numChoices, pParam);
        if (choice == 0)
        {
            // For the channel attributes.
            if (pArgs->c == 4)
            {
                numChoices = sizeof(channelTypes) / sizeof(channelTypes[0]);
                choice = GetChoice(channelTypes, pArgs->v[1].pS, numChoices, pParam);
                if (choice == 0)
                {
                    if (strstr(pArgs->v[2].pS, "voltage") != NULL)
                    {
                        sscanf(pArgs->v[2].pS, "voltage%ld", &chanNum);
                        attrId = GetChannelAttributeId(pArgs->v[3].pS);
                        IIoAttrGet(attrId, &chanNum, buffer);
                    }
                    else
                    {
                        status = -1;
                    }
                }
                else
                {
                    status = -1;
                }
            }
            // For the debug attributes.
            else if (pArgs->c == 3)
            {
                numChoices = sizeof(debugTypes) / sizeof(debugTypes[0]);
                choice = GetChoice(debugTypes, pArgs->v[1].pS, numChoices, pParam);
                if (choice == 0)
                {
                    sscanf(pArgs->v[2].pS, "%s", &attr[0]);
                    if (strcmp(&attr[0], "direct_reg_access") == 0)
                    {
                        DebugRegRead(pExample->debugAddress, &debugReadVal);
                        sprintf(buffer, "%ld", debugReadVal);
                    }
                    else
                    {
                        status = -1;
                    }
                }
                else
                {
                    status = -1;
                }
            }
            // For the global attributes.
            else if (pArgs->c == 2)
            {
                attrId = GetGlobalAttributeId(pArgs->v[1].pS);
                IIoAttrGet(attrId, &chanNum, buffer);
            }
        }
        length = strlen(buffer);
        INFO_MSG_RAW("%d\n", length) INFO_MSG_RAW("%s\n", buffer)
    }
    else
    {
        status = -1;
    }

    return status;
}

int32_t CmdWrite(Args *pArgs)
{
    int32_t status = 0;
    int32_t numChoices;
    int32_t choice;
    uint8_t chanNum;
    int32_t attrId = 0;
    int32_t attrLength;
    char attr[20];

    char *pParam = &commandParam[0];
    if (pArgs->c >= 2 && pArgs->c <= 5)
    {
        numChoices = sizeof(deviceTypes) / sizeof(deviceTypes[0]);
        choice = GetChoice(deviceTypes, pArgs->v[0].pS, numChoices, pParam);
        if (choice == 0)
        {
            // For the channel attributes.
            if (pArgs->c == 5)
            {
                numChoices = sizeof(channelTypes) / sizeof(channelTypes[0]);
                choice = GetChoice(channelTypes, pArgs->v[1].pS, numChoices, pParam);
                if (choice == 0)
                {
                    if (strstr(pArgs->v[2].pS, "voltage") != NULL)
                    {
                        sscanf(pArgs->v[2].pS, "voltage%hhu", &chanNum);
                        sscanf(pArgs->v[4].pS, "%" PRIi32, &attrLength);
                        attrId = GetChannelAttributeId(pArgs->v[3].pS);
                    }
                    else
                    {
                        status = -1;
                    }
                }
                else
                {
                    status = -1;
                }
            }
            // For the debug attributes.
            else if (pArgs->c == 4)
            {
                numChoices = sizeof(debugTypes) / sizeof(debugTypes[0]);
                choice = GetChoice(debugTypes, pArgs->v[1].pS, numChoices, pParam);
                if (choice == 0)
                {
                    sscanf(pArgs->v[2].pS, "%s", &attr[0]);
                    if (strcmp(&attr[0], "direct_reg_access") != 0)
                    {
                        status = -1;
                    }
                    else
                    {
                        sscanf(pArgs->v[3].pS, "%" PRIi32, &attrLength);
                    }
                }
                else
                {
                    status = -1;
                }
            }
            // For the global attributes.
            else if (pArgs->c == 3)
            {
                attrId = GetGlobalAttributeId(pArgs->v[1].pS);
                sscanf(pArgs->v[2].pS, "%" PRIi32, &attrLength);
            }

            // Read the values from the circular Buffer
            ScanData(attrLength);
            // For the Debug type, the [addr, val] prefixes with 0x. For the other attributes,
            // numbers is in decimal format.
            if (pArgs->c == 4)
            {
                WriteToDebugReg();
            }
            else
            {
                IioAttrSet(attrId, &chanNum, buffer);
            }
        }
        INFO_MSG_RAW("%d\n", attrLength)
    }
    else
    {
        status = -1;
    }

    return status;
}

int32_t ScanData(int32_t attrLength)
{
    int32_t status = 0;
    int32_t index = 0;
    int32_t ret = 0;
    while (index < attrLength)
    {
        ret = adi_cli_GetChar();
        if (ret != EOF)
        {
            sprintf(&buffer[index], "%c", (int)ret);
            index++;
        }
    }
    return status;
}

int32_t WriteToDebugReg()
{
    int32_t status = 0;
    int32_t numBytes;
    uint32_t address, value;
    ADC_EXAMPLE *pExample = GetAdcExampleInfo();
    numBytes = sscanf(buffer, "\n0x%" SCNx32 " 0x%" SCNx32, &address, &value);
    if (numBytes == 2)
    {
        DebugRegWrite(address, value);
    }
    else
    {
        numBytes = sscanf(buffer, "%" SCNd32, &pExample->debugAddress);
    }
    return status;
}

int32_t CmdReadBuf(Args *pArgs)
{
    int32_t status = 0;
    int32_t numChoices;
    int32_t choice;
    int32_t numBytes;
    int32_t length;
    char *pParam = &commandParam[0];
    ADC_EXAMPLE *pExample = GetAdcExampleInfo();
    if (pArgs->c == 2)
    {
        numChoices = sizeof(deviceTypes) / sizeof(deviceTypes[0]);
        choice = GetChoice(deviceTypes, pArgs->v[0].pS, numChoices, pParam);
        if (choice == 0)
        {
            sscanf(pArgs->v[1].pS, "%" PRIi32, &numBytes);
            sprintf(buffer, "%" PRIi32, numBytes);
            length = strlen(buffer);
            buffer[length] = '\n';
            EvbHostUartTransmitAsync(&uartInfo, (uint8_t *)buffer, length + 1);
            snprintf(buffer, 10, "%08" PRIx32, pExample->channelMask);
            length = strlen(buffer);
            buffer[length] = '\n';
            EvbHostUartTransmitAsync(&uartInfo, (uint8_t *)buffer, length + 1);
            IioSubmitBuffer(numBytes);
        }
    }
    else
    {
        status = -1;
    }

    return status;
}

int32_t CmdWriteBuf(Args *pArgs)
{
    int32_t status = 0;
    if (pArgs->c == 0)
    {
    }
    return status;
}

int32_t CmdGetRig(Args *pArgs)
{
    int32_t status = 0;
    int32_t value;
    ADC_EXAMPLE *pExample = GetAdcExampleInfo();

    if (pArgs->c == 1)
    {
        if (pExample->triggerDevice == 0)
        {
            value = -2;
            INFO_MSG_RAW("%d\n", value)
        }
    }
    else
    {
        status = -1;
    }

    return status;
}

int32_t CmdSetRig(Args *pArgs)
{
    int32_t status = 0;
    if (pArgs->c == 0)
    {
    }
    return status;
}

int32_t CmdSet(Args *pArgs)
{
    int32_t status = 0;
    if (pArgs->c == 0)
    {
    }
    return status;
}
