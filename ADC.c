/*
 * ADC.c
 *
 *  Created on: 20/03/2019
 *      Author: Canale
 */

#include "ADC.h"

#include "fsl_debug_console.h"
#include "board.h"


#include "pin_mux.h"
#include "clock_config.h"

#include "FreeRTOS.h"
#include "semphr.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define ADC16_BASE ADC0
#define ADC16_CHANNEL_GROUP 0U
#define ADC16_USER_CHANNEL 12U /* PTB2, ADC0_SE12 */
/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile bool g_Adc16ConversionDoneFlag;
volatile uint32_t g_Adc16ConversionValue;
const uint32_t g_Adc16_12bitFullRange = 4096U;

SemaphoreHandle_t bin_sem;
adc_config_t conf;
/*******************************************************************************
 * Code
 ******************************************************************************/
void ADC0_IRQHandler(void)
{
//	TODO BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    g_Adc16ConversionDoneFlag = true;
    /* Read conversion result to clear the conversion completed flag. */
    g_Adc16ConversionValue = ADC16_GetChannelConversionValue(ADC16_BASE, ADC16_CHANNEL_GROUP);
//    g_Adc16InterruptCounter++;
/* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
  exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
//    TODO xSemaphoreGiveFromISR(bin_sem, &xHigherPriorityTaskWoken);
//    TODO portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void adc_init(adc_config_t config)
{
	EnableIRQ(ADC0_IRQn);
    ADC16_GetDefaultConfig(&conf.adc16ConfigStruct);
    ADC16_Init(config.base, &conf.adc16ConfigStruct);

//    bin_sem = xSemaphoreCreateBinary(); //TODO Debe estar antes del while(1) de la tarea

    ADC16_EnableHardwareTrigger(config.base, false); /* Make sure the software trigger is used. */
#if defined(FSL_FEATURE_ADC16_HAS_CALIBRATION) && FSL_FEATURE_ADC16_HAS_CALIBRATION
    if (kStatus_Success == ADC16_DoAutoCalibration(config.base))
    {
        PRINTF("ADC16_DoAutoCalibration() Done.\r\n");
    }
    else
    {
        PRINTF("ADC16_DoAutoCalibration() Failed.\r\n");
    }
#endif /* FSL_FEATURE_ADC16_HAS_CALIBRATION */

    PRINTF("ADC Full Range: %d\r\n", g_Adc16_12bitFullRange);
    PRINTF("Press any key to get user channel's ADC value ...\r\n");

    conf.adc16ChannelConfigStruct.channelNumber = ADC16_USER_CHANNEL;
    conf.adc16ChannelConfigStruct.enableInterruptOnConversionCompleted = true; /* Enable the interrupt. */
#if defined(FSL_FEATURE_ADC16_HAS_DIFF_MODE) && FSL_FEATURE_ADC16_HAS_DIFF_MODE
    conf.adc16ChannelConfigStruct.enableDifferentialConversion = false;
#endif /* FSL_FEATURE_ADC16_HAS_DIFF_MODE */

//    g_Adc16InterruptCounter = 0U;

}

void adc_SetChannelConfig(void)
{
	ADC16_SetChannelConfig(ADC16_BASE, ADC16_CHANNEL_GROUP, &conf.adc16ChannelConfigStruct);
}

uint32_t get_adc_value(void)
{
//	xSemaphoreTake(bin_sem,portMAX_DELAY);
//	TODO uint32_t adc_value = g_Adc16ConversionValue;
	return g_Adc16ConversionValue;
}

bool get_g_Adc16ConversionDoneFlag(void)
{
	return g_Adc16ConversionDoneFlag;
}
