/*
 * GPIO.c
 *
 *  Created on: Mar 30, 2019
 *      Author: acc
 */

#include "GPIO.h"

#include "fsl_gpio.h"
#include "fsl_clock.h"
#include "fsl_port.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "event_groups.h"

/** Se necesitan 5 estructuras tipo rtos_gpio_hanlde_t para que haya una por GPIOx **/
#define NUMBER_OF_GPIOS (5)

#define EVENT_PORTA (1<<0)
#define EVENT_PORTB (1<<1)
#define EVENT_PORTC	(1<<2)
#define EVENT_PORTD (1<<3)
#define EVENT_PORTE (1<<4)

#define GET_ARGS(args,type) *((type*)args)

typedef struct
{
	uint8_t is_init;
	uint32_t interruptPin;
	EventGroupHandle_t events;
	SemaphoreHandle_t mutex;
	SemaphoreHandle_t bin_sem;
	TaskHandle_t blink_task_handle;
}rtos_gpio_hanlde_t;

TaskHandle_t blink_task_handle2;

static rtos_gpio_hanlde_t gpio_handles[NUMBER_OF_GPIOS] = {0};

static inline void nvic_enable_irq_nvic_set_priority(rtos_gpio_port_t);
static inline void enable_port_clock(rtos_gpio_port_t);
static inline GPIO_Type * get_gpio_base(rtos_gpio_t);
static inline PORT_Type * get_port_base(rtos_gpio_port_t);
//rtos_gpio_hanlde_t get_rtos_gpio_handle(rtos_gpio_config_t config);
void gpio_blinking_task(void * args);

void PORTA_IRQHandler(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	gpio_handles[rtos_gpio_portA].interruptPin = GPIO_GetPinsInterruptFlags(GPIOA);
	PORT_ClearPinsInterruptFlags( PORTA, gpio_handles[rtos_gpio_portA].interruptPin);

	xEventGroupSetBitsFromISR(gpio_handles[rtos_gpio_portA].events, EVENT_PORTA, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}
void PORTB_IRQHandler(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	gpio_handles[rtos_gpio_portB].interruptPin = GPIO_GetPinsInterruptFlags(GPIOB);
	PORT_ClearPinsInterruptFlags( PORTB, gpio_handles[rtos_gpio_portB].interruptPin);

	xEventGroupSetBitsFromISR(gpio_handles[rtos_gpio_portB].events, EVENT_PORTB, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}
void PORTC_IRQHandler(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	gpio_handles[rtos_gpio_portC].interruptPin = GPIO_GetPinsInterruptFlags(GPIOC);
	PORT_ClearPinsInterruptFlags( PORTC, gpio_handles[rtos_gpio_portC].interruptPin);

	xEventGroupSetBitsFromISR(gpio_handles[rtos_gpio_portC].events, EVENT_PORTC, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}
void PORTD_IRQHandler(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	gpio_handles[rtos_gpio_portD].interruptPin = GPIO_GetPinsInterruptFlags(GPIOD);
	PORT_ClearPinsInterruptFlags( PORTD, gpio_handles[rtos_gpio_portD].interruptPin);

	xEventGroupSetBitsFromISR(gpio_handles[rtos_gpio_portD].events, EVENT_PORTD, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}
void PORTE_IRQHandler(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	gpio_handles[rtos_gpio_portE].interruptPin = GPIO_GetPinsInterruptFlags(GPIOE);
	PORT_ClearPinsInterruptFlags( PORTE, gpio_handles[rtos_gpio_portE].interruptPin);

	xEventGroupSetBitsFromISR(gpio_handles[rtos_gpio_portE].events, EVENT_PORTE, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

rtos_gpio_flag_t rtos_gpio_init(rtos_gpio_config_t config)
{
	rtos_gpio_flag_t retval = rtos_gpio_fail;

	if(config.gpio < NUMBER_OF_GPIOS)
	{
		if(!gpio_handles[config.gpio].is_init)
		{
			gpio_handles[config.gpio].events =  xEventGroupCreate();
			gpio_handles[config.gpio].mutex = xSemaphoreCreateMutex();
			gpio_handles[config.gpio].bin_sem = xSemaphoreCreateBinary();

				port_pin_config_t port_output_config = { kPORT_PullDisable,
						kPORT_FastSlewRate, kPORT_PassiveFilterDisable,
						kPORT_OpenDrainDisable, kPORT_LowDriveStrength,
						kPORT_MuxAsGpio, kPORT_UnlockRegister, };
				port_pin_config_t port_input_config = { kPORT_PullDisable,
						kPORT_FastSlewRate, kPORT_PassiveFilterDisable,
						kPORT_OpenDrainDisable, kPORT_LowDriveStrength,
						kPORT_MuxAsGpio, kPORT_UnlockRegister, };
				gpio_pin_config_t gpio_output_config = { kGPIO_DigitalOutput, 1, };
				gpio_pin_config_t gpio_input_config = { kGPIO_DigitalInput, 1, };

			enable_port_clock(config.port);

			if( rtos_gpio_input == config.pin_direction)
			{
				PORT_SetPinConfig(get_port_base(config.port), config.pin, &port_input_config);
				PORT_SetPinInterruptConfig(get_port_base(config.port), config.pin, kPORT_InterruptRisingEdge);
				nvic_enable_irq_nvic_set_priority(config.port);
//			TODO En el rtos_uart.c Aldana hace un if para seleccionar uart0 o 1.
//			En este caso habria que ver si esto interfiere cuando se necesite inicializar varios pines
				GPIO_PinInit(get_gpio_base(config.gpio), config.pin, &gpio_input_config);
			}
			else
			{
				PORT_SetPinConfig(get_port_base(config.port), config.pin, &port_output_config);
				GPIO_PinInit(get_gpio_base(config.gpio), config.pin, &gpio_output_config);
			}

			gpio_handles[config.gpio].is_init = 1;
			retval = rtos_gpio_sucess;
		}
	}
	return retval;
}

rtos_gpio_flag_t rtos_gpio_LED_OFF(rtos_gpio_config_t config)
{
	rtos_gpio_flag_t flag = rtos_gpio_fail;
	if(gpio_handles[config.gpio].is_init)
	{
//		xSemaphoreTake(gpio_handles[config.gpio].mutex, portMAX_DELAY);
//
		GPIO_WritePinOutput(get_gpio_base(config.gpio), config.pin, 1);
//		xEventGroupWaitBits(gpio_handles[config.gpio].events,
//		EVENT_PORTA | EVENT_PORTB | EVENT_PORTC | EVENT_PORTD | EVENT_PORTE,
//		pdTRUE, pdFALSE, portMAX_DELAY);

//
//		xSemaphoreGive(gpio_handles[config.gpio].mutex);
		flag = rtos_gpio_sucess;
	}
	return flag;
}

rtos_gpio_flag_t rtos_gpio_LED_ON(rtos_gpio_config_t config)
{
	rtos_gpio_flag_t flag = rtos_gpio_fail;
	if(gpio_handles[config.gpio].is_init)
	{
//		xSemaphoreTake(gpio_handles[config.gpio].mutex, portMAX_DELAY);
//
		GPIO_WritePinOutput(get_gpio_base(config.gpio), config.pin, 0);
//		xEventGroupWaitBits(gpio_handles[config.gpio].events,
//		EVENT_PORTA | EVENT_PORTB | EVENT_PORTC | EVENT_PORTD | EVENT_PORTE,
//		pdTRUE, pdFALSE, portMAX_DELAY);
//
//		xSemaphoreGive(gpio_handles[config.gpio].mutex);
		flag = rtos_gpio_sucess;
	}
	return flag;
}

rtos_gpio_flag_t rtos_gpio_wait_pin(rtos_gpio_config_t config)
{
	rtos_gpio_flag_t flag = rtos_gpio_fail;
	if(gpio_handles[config.gpio].is_init)
	{
		xSemaphoreTake(gpio_handles[config.gpio].mutex, portMAX_DELAY);

		xEventGroupWaitBits(gpio_handles[config.gpio].events,
		EVENT_PORTA | EVENT_PORTB | EVENT_PORTC | EVENT_PORTD | EVENT_PORTE,
		pdTRUE, pdFALSE, portMAX_DELAY);

		xSemaphoreGive(gpio_handles[config.gpio].mutex);
		flag = rtos_gpio_sucess;
	}
	return flag;
}

uint8_t rtos_gpio_toogle_pin(rtos_gpio_config_t config, uint8_t modo_administrador)
{
//	rtos_gpio_flag_t flag = rtos_gpio_fail;
//	static rtos_gpio_hanlde_t args;

	if(gpio_handles[config.gpio].is_init)
	{
//		GPIO_TogglePinsOutput(GPIOB, 1 << 22); //RED_LED
//		flag = rtos_gpio_sucess;
		if(modo_administrador == 0)
		{
			modo_administrador = 1;
			xTaskCreate(gpio_blinking_task, "gpio_blinking_task", configMINIMAL_STACK_SIZE+110, (void *)&config, 1, NULL);
		}
		else
		{
			modo_administrador = 0;
//			vTaskDelete(gpio_handles[config.gpio].blink_task_handle);
			vTaskDelete(blink_task_handle2);
			GPIO_WritePinOutput(get_gpio_base(config.gpio), config.pin, 1); //R OFF
		}
	}
	return modo_administrador;
}

void gpio_blinking_task(void * arg)
{
	rtos_gpio_config_t config = GET_ARGS(arg,rtos_gpio_config_t);
	TickType_t xLastWakeTime;
	const TickType_t xPeriod = pdMS_TO_TICKS( 500 );
	xLastWakeTime = xTaskGetTickCount();
	gpio_handles[config.gpio].blink_task_handle = xTaskGetCurrentTaskHandle();
	blink_task_handle2 = xTaskGetCurrentTaskHandle();
	for(;;)
	{
		GPIO_TogglePinsOutput(GPIOB, 1 << 22);
		vTaskDelayUntil( &xLastWakeTime, xPeriod );

	}
}

static inline void nvic_enable_irq_nvic_set_priority(rtos_gpio_port_t port)
{
	switch(port)
	{
	case rtos_gpio_portA:
		NVIC_EnableIRQ(PORTA_IRQn);
		NVIC_SetPriority(PORTA_IRQn, 5);
		break;
	case rtos_gpio_portB:
		NVIC_EnableIRQ(PORTB_IRQn);
		NVIC_SetPriority(PORTB_IRQn, 5);
		break;
	case rtos_gpio_portC:
		NVIC_EnableIRQ(PORTC_IRQn);
		NVIC_SetPriority(PORTC_IRQn, 5);
		break;
	case rtos_gpio_portD:
		NVIC_EnableIRQ(PORTD_IRQn);
		NVIC_SetPriority(PORTD_IRQn, 5);
		break;
	case rtos_gpio_portE:
		NVIC_EnableIRQ(PORTE_IRQn);
		NVIC_SetPriority(PORTE_IRQn, 5);
		break;
	}
}

static inline void enable_port_clock(rtos_gpio_port_t port)
{
	switch(port)
	{
	case rtos_gpio_portA:
		CLOCK_EnableClock(kCLOCK_PortA);
		break;
	case rtos_gpio_portB:
		CLOCK_EnableClock(kCLOCK_PortB);
		break;
	case rtos_gpio_portC:
		CLOCK_EnableClock(kCLOCK_PortC);
		break;
	case rtos_gpio_portD:
		CLOCK_EnableClock(kCLOCK_PortD);
		break;
	case rtos_gpio_portE:
		CLOCK_EnableClock(kCLOCK_PortE);
		break;
	}
}

static inline GPIO_Type * get_gpio_base(rtos_gpio_t gpio)
{
	GPIO_Type * gpio_base = GPIOA;
	switch(gpio)
	{
	case rtos_gpioA:
		gpio_base = GPIOA;
		break;
	case rtos_gpioB:
		gpio_base = GPIOB;
		break;
	case rtos_gpioC:
		gpio_base = GPIOC;
		break;
	case rtos_gpioD:
		gpio_base = GPIOD;
		break;
	case rtos_gpioE:
		gpio_base = GPIOE;
		break;
	}
	return gpio_base;
}

static inline PORT_Type * get_port_base(rtos_gpio_port_t port)
{
	PORT_Type * port_base = PORTA;
	switch(port)
	{
	case rtos_gpio_portA:
		port_base = PORTA;
		break;
	case rtos_gpio_portB:
		port_base = PORTB;
		break;
	case rtos_gpio_portC:
		port_base = PORTC;
		break;
	case rtos_gpio_portD:
		port_base = PORTD;
		break;
	case rtos_gpio_portE:
		port_base = PORTE;
		break;
	}
	return port_base;
}
