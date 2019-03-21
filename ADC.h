/*
 * ADC.h
 *
 *  Created on: 20/03/2019
 *      Author: Canale
 */

#ifndef ADC_H_
#define ADC_H_

#include <stdint.h>
#include <stdbool.h>

#include "fsl_adc16.h"


typedef enum {adc_chnl0, adc_chnl1} adc_user_channel_t;


//volatile uint32_t g_Adc16InterruptCounter;

typedef struct
{
//	uint32_t  baudrate;
	ADC_Type * base;
//	adc_port_t port;
	adc_user_channel_t user_channel;  //pin user wants to read
//	uint32_t pin; //pin user wants to read
    adc16_config_t adc16ConfigStruct;
    adc16_channel_config_t adc16ChannelConfigStruct;
//	adc_direction_t pin_direction;//
//	uint8_t rx_pin;
//	uint8_t tx_pin;
//	uint8_t pin_mux;
}adc_config_t;


void adc_init(adc_config_t config);
void adc_SetChannelConfig(void);
uint32_t get_adc_value(void);
bool get_g_Adc16ConversionDoneFlag(void);

#endif /* ADC_H_ */
