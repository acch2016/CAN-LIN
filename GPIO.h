/*
 * GPIO.h
 *
 *  Created on: Mar 30, 2019
 *      Author: acc
 */

#ifndef GPIO_H_
#define GPIO_H_

#include <stdint.h>

typedef enum {rtos_gpioA,rtos_gpioB,rtos_gpioC,rtos_gpioD,rtos_gpioE} rtos_gpio_t;
typedef enum {rtos_gpio_portA,rtos_gpio_portB,rtos_gpio_portC,rtos_gpio_portD,rtos_gpio_portE} rtos_gpio_port_t;
typedef enum {rtos_gpio_input,rtos_gpio_output} rtos_gpio_direction_t;
typedef enum {rtos_gpio_sucess,rtos_gpio_fail} rtos_gpio_flag_t;

typedef struct
{
//	uint32_t  baudrate;
	rtos_gpio_t gpio;
	rtos_gpio_port_t port;
	uint32_t pin; //pin user wants to read
	rtos_gpio_direction_t pin_direction;//
//	uint8_t rx_pin;
//	uint8_t tx_pin;
//	uint8_t pin_mux;
}rtos_gpio_config_t;

rtos_gpio_flag_t rtos_gpio_init(rtos_gpio_config_t config);

//TODO Esta función recibira la estructura tipo rtos_gpio_config_t de donde obtendria el pin y el puerto
rtos_gpio_flag_t rtos_gpio_wait_pin(rtos_gpio_config_t config);
//TODO Declarar aqui la otra función que controlará al LED
uint8_t rtos_gpio_toogle_pin(rtos_gpio_config_t config, uint8_t modo_administrador);
rtos_gpio_flag_t rtos_gpio_LED_OFF(rtos_gpio_config_t config);
rtos_gpio_flag_t rtos_gpio_LED_ON(rtos_gpio_config_t config);

#endif /* GPIO_H_ */
