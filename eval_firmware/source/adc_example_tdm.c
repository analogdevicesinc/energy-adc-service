/******************************************************************************
 Copyright (c) 2024 - 2025  Analog Devices Inc.
******************************************************************************/

/**
 * @file        adc_example_tdm.c
 * @brief       Interface file demonstrating use of Tamper Detection Mode.
 * @{
 */

/*============= I N C L U D E S =============*/
#include "adc_example_tdm.h"
#include "adc_service_interface.h"
#include "adi_evb.h"
#include <stdint.h>

/*============= D E F I N E S =============*/

/*============= F U N C T I O N S =============*/

int32_t StartTamperDetection(void)
{
    int32_t status = 0;

    ADC_INTERFACE_INFO *pAdcIf = AdcIfGetInstance();
    EvbSetStandby(0);

    /*Set the LPTIM output pin to be used for TDM */
    EvbSetLptimOutput();

    /* Set the flag to indicate TDM cycle is running */
    pAdcIf->isTdmCycleRunning = 1;
    EvbStartLpTimer();

    return status;
}

int32_t StopTamperDetection(void)
{
    int32_t status = 0;

    ADC_INTERFACE_INFO *pAdcIf = AdcIfGetInstance();
    EvbStopLpTimer();

    /* Configure PA.4 ss pin*/
    EvbAdeSpiSetChipSelect();

    /*Clear the flag to indicate TDM cycle is not running*/
    pAdcIf->isTdmCycleRunning = 0;
    EvbSetStandby(1);

    return status;
}

/**
 * @}
 */
