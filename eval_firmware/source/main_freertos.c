/******************************************************************************
 Copyright (c) 2024 - 2025  Analog Devices Inc.
******************************************************************************/

/*============= I N C L U D E S =============*/

#include "FreeRTOS.h"
#include "adc_example_freertos.h"
#include "adc_example_freertos_config.h"
#include "task.h"
#include <stdint.h>

/*============= D A T A =============*/

/**
 * Entry point
 */
int main()
{
    int32_t status = 0;
    status = InitThreads();
    status = AdcIfInitThreads();
    if (status == 0)
    {
        vTaskStartScheduler();
        while (1)
        {
            // Loop indefinitely
        }
    }

    return 0;
}
