/******************************************************************************
 Copyright (c) 2023 - 2025  Analog Devices Inc.
******************************************************************************/

/**
 * @file        adc_example_freertos.c
 * @brief       Example of using ADC with FRTOS. This file contains the implementation of
 * the FreeRTOS tasks and their initialization.
 * @{
 */

/*============= I N C L U D E S =============*/
#include "adc_example_freertos.h"
#include "FreeRTOS.h"
#include "adc_example.h"
#include "adc_service_interface.h"
#include "adi_adc.h"
#include "adi_evb.h"
#include "semphr.h"
#include <stdio.h>
#include <string.h>

/*============= F U N C T I O N S =============*/

/**
 * @brief Function to receive commands and send responses through UART.
 * @param pArg - Pointer to arguments (not used).
 */
static void CmdThread(void *pArg);

/** Id for command Thread */
static TaskHandle_t cmdThreadId;

int32_t InitThreads(void)
{
    int32_t status = 0;
    BaseType_t threadStatus = pdPASS;

    threadStatus = xTaskCreate(CmdThread, "CmdThread", 512, NULL, 1, &cmdThreadId);
    if (threadStatus != pdPASS)
    {
        status = -1;
    }
    return status;
}

void CmdThread(void *pArg)
{
    if (pArg == NULL)
    {
        ADC_EXAMPLE_STATUS status;
        status = InitServices();
        while (status == ADC_EXAMPLE_STATUS_SUCCESS)
        {
            status = ProcessCommand();
        }
    }
}

/**
 * @}
 */
