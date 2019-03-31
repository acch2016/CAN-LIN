/*
 * lin_driver_test_main.c
 * Created on: Sep 15, 2018
 *     Author: Nico
 */

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"

#include "fsl_uart_freertos.h"
#include "fsl_uart.h"

#include "pin_mux.h"
#include "clock_config.h"
#include "lin1d3_driver.h"
#include "FreeRTOSConfig.h"

#include "ADC.h"
#include "GPIO.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define JUST_SLAVE //correr primero el slave

/* UART instance and clock */
#define MASTER_UART UART3
#define MASTER_UART_CLKSRC UART3_CLK_SRC
#define MASTER_UART_CLK_FREQ CLOCK_GetFreq(UART3_CLK_SRC)
#define MASTER_UART_RX_TX_IRQn UART3_RX_TX_IRQn

/* UART instance and clock */
#define SLAVE_UART UART4
#define SLAVE_UART_CLKSRC UART4_CLK_SRC
#define SLAVE_UART_CLK_FREQ CLOCK_GetFreq(UART4_CLK_SRC)
#define SLAVE_UART_RX_TX_IRQn UART4_RX_TX_IRQn

/* Task priorities. */
#define init_task_PRIORITY (configMAX_PRIORITIES - 2)
#define test_task_heap_size_d	(192)

#define app_message_id_1_d (0x01<<2|message_size_2_bytes_d)
#define app_message_id_2_d (0x02<<2|message_size_4_bytes_d)
#define app_message_id_3_d (0x03<<2|message_size_8_bytes_d)



/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void test_task(void *pvParameters);
#if !defined(JUST_SLAVE)
static void	message_1_callback_master(void* message);
static void	message_2_callback_master(void* message);
static void	message_3_callback_master(void* message);
#endif
static void	message_1_callback_slave(void* message);
static void	message_2_callback_slave(void* message);
static void	message_3_callback_slave(void* message);
/*******************************************************************************
 * Variables
 ******************************************************************************/
uint16_t adc_16value = 0;
uint8_t adc_value_H = 0;
rtos_gpio_config_t config_LED;

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Application entry point.
 */
int main(void)
{
    /* Init board hardware. */
    BOARD_InitPins();
    BOARD_BootClockRUN();
    NVIC_SetPriority(MASTER_UART_RX_TX_IRQn, 5);
    NVIC_SetPriority(SLAVE_UART_RX_TX_IRQn, 5);

	/**ADC**/
	adc_config_t config;
	config.base = ADC0;
	adc_init(config);
	/**GPIO**/
//	rtos_gpio_config_t config_LED;
	config_LED.gpio = rtos_gpioE;
	config_LED.port = rtos_gpio_portE;
	config_LED.pin = 26;
	config_LED.pin_direction = rtos_gpio_output;
	rtos_gpio_init(config_LED);

    if (xTaskCreate(test_task, "test_task", test_task_heap_size_d, NULL, init_task_PRIORITY, NULL) != pdPASS)
    {
        PRINTF("Init Task creation failed!.\r\n");
        while (1)
            ;
    }
    PRINTF(" *** LIN driver demo ***\r\n");
    vTaskStartScheduler();
    for (;;)
        ;
}

/*!
 * @brief Task responsible for loopback.
 */
static void test_task(void *pvParameters)
{
#if !defined(JUST_SLAVE)
	int error;
#endif
	lin1d3_nodeConfig_t node_config;
#if !defined(JUST_SLAVE)
	lin1d3_handle_t* master_handle;
#endif
	lin1d3_handle_t* slave_handle;
#if !defined(JUST_SLAVE)
	/* Set Master Config */
	node_config.type = lin1d3_master_nodeType;
	node_config.bitrate = 9600;
	node_config.uartBase = MASTER_UART;
	node_config.srcclk = MASTER_UART_CLK_FREQ;
	memset(node_config.messageTable,0, (sizeof(node_config.messageTable[0])*lin1d3_max_supported_messages_per_node_cfg_d));
	node_config.messageTable[0].ID = app_message_id_1_d;
	node_config.messageTable[0].handler = message_1_callback_master;
	node_config.messageTable[1].ID = app_message_id_2_d;
	node_config.messageTable[1].handler = message_2_callback_master;
	node_config.messageTable[2].ID = app_message_id_3_d;
	node_config.messageTable[2].handler = message_3_callback_master;
	/* Init Master node */
	master_handle = lin1d3_InitNode(node_config);
#endif
#if !defined(JUST_MASTER)
	/* Set Slave Config */
	node_config.type = lin1d3_slave_nodeType;
	node_config.bitrate = 9600;
	node_config.uartBase = SLAVE_UART;
	node_config.srcclk = SLAVE_UART_CLK_FREQ;
	memset(node_config.messageTable,0, (sizeof(node_config.messageTable[0])*lin1d3_max_supported_messages_per_node_cfg_d));
	node_config.messageTable[0].ID = app_message_id_1_d;
	node_config.messageTable[0].handler = message_1_callback_slave;
	node_config.messageTable[1].ID = app_message_id_2_d;
	node_config.messageTable[1].handler = message_2_callback_slave;
	node_config.messageTable[2].ID = app_message_id_3_d;
	node_config.messageTable[2].handler = message_3_callback_slave;
	/* Init Slave Node*/
	slave_handle = lin1d3_InitNode(node_config);
#endif
#if !defined(JUST_SLAVE)
	if((NULL == master_handle)
#if !defined(JUST_MASTER)
		|| (NULL == slave_handle)
#endif
	   ){
		error = kStatus_Fail;
	}
	else {
		error = kStatus_Success;
	}

	while (kStatus_Success == error)
    {
    	vTaskDelay(200);
    	lin1d3_masterSendMessage(master_handle, app_message_id_1_d);
    	vTaskDelay(200);
    	lin1d3_masterSendMessage(master_handle, app_message_id_2_d);
    	vTaskDelay(200);
    	lin1d3_masterSendMessage(master_handle, app_message_id_3_d);
    }
#endif
    vTaskSuspend(NULL);
}

#if !defined(JUST_SLAVE)
static void	message_1_callback_master(void* message)
{
	uint8_t* message_data = (uint8_t*)message;
	PRINTF("Master got response to message 1 %d,%d\r\n", message_data[0], message_data[1]);
}

static void	message_2_callback_master(void* message)
{
	uint8_t* message_data = (uint8_t*)message;
	PRINTF("Master got response to message 2 %d,%d,%d,%d\r\n", message_data[0], message_data[1], message_data[2], message_data[3]);
}

static void	message_3_callback_master(void* message)
{
	uint8_t* message_data = (uint8_t*)message;
	PRINTF("Master got response to message 3 %d,%d,%d,%d,%d,%d,%d,%d\r\n",
			message_data[0], message_data[1], message_data[2], message_data[3],
			message_data[4], message_data[5], message_data[6], message_data[7]);
}
#endif
static void	message_1_callback_slave(void* message)
{

	/*
	 When in software trigger mode, each conversion would be launched once calling the "ADC16_ChannelConfigure()"
	 function, which works like writing a conversion command and executing it. For another channel's conversion,
	 just to change the "channelNumber" field in channel configuration structure, and call the function
	 "ADC16_ChannelConfigure()"" again.
	 Also, the "enableInterruptOnConversionCompleted" inside the channel configuration structure is a parameter for
	 the conversion command. It takes affect just for the current conversion. If the interrupt is still required
	 for the following conversion, it is necessary to assert the "enableInterruptOnConversionCompleted" every time
	 for each command.
	 */
	adc_SetChannelConfig();
	while (!get_g_Adc16ConversionDoneFlag())
	{
	}
	PRINTF("ADC Value: %d\r\n", get_adc_value());

	uint8_t* message_data = (uint8_t*)message;
	PRINTF("Slave got message 1 request\r\n");
	adc_16value = get_adc_value();
	adc_value_H = adc_16value >> 8;
	message_data[0] = get_adc_value();
	message_data[1] = adc_value_H;
}

static void	message_2_callback_slave(void* message)
{
	rtos_gpio_LED_ON(config_LED);
	uint8_t* message_data = (uint8_t*)message;
	PRINTF("Slave got message 2 request\r\n");
	message_data[0] = 'L';
	message_data[1] = 'E';
	message_data[2] = 'D';
	message_data[3] = 'O';
	message_data[4] = 'N';
}

static void	message_3_callback_slave(void* message)
{
	rtos_gpio_LED_OFF(config_LED);
	uint8_t* message_data = (uint8_t*)message;
	PRINTF("Slave got message 3 request\r\n");
	message_data[0] = 'L';
	message_data[1] = 'E';
	message_data[2] = 'D';
	message_data[3] = 'O';
	message_data[4] = 'F';
	message_data[5] = 'F';
//	message_data[6] = 85;
//	message_data[7] = 86;
}

