/******************************************************************************
 Copyright (c) 2022 - 2025  Analog Devices Inc.
******************************************************************************/

/*============= I N C L U D E S =============*/
#include "adc_example.h"

/*============= D A T A =============*/

/**
 * Entry point
 */
int main(void)
{
    ADC_EXAMPLE_STATUS adcStatus = ADC_EXAMPLE_STATUS_SUCCESS;
    int32_t status = 0;
    adcStatus = InitServices();
    while (adcStatus == ADC_EXAMPLE_STATUS_SUCCESS)
    {
        adcStatus = ProcessCommand();
    }

    return status;
}
