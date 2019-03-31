///*
// * lin_driver_test_main.c
// * Created on: Sep 15, 2018
// *     Author: Nico
// */
//
///* FreeRTOS kernel includes. */
//#include "FreeRTOS.h"
//#include "task.h"
//#include "queue.h"
//#include "timers.h"
//#include "fsl_adc16.h"
//
///* Freescale includes. */
//#include "fsl_device_registers.h"
//#include "fsl_debug_console.h"
//#include "board.h"
//#include "fsl_gpio.h"
//#include "fsl_port.h"
//#include "fsl_flexcan.h"
//#include "fsl_uart_freertos.h"
//#include "fsl_uart.h"
//
//#include "pin_mux.h"
//#include "clock_config.h"
//#include "lin1d3_driver.h"
//#include "FreeRTOSConfig.h"
//
///*******************************************************************************
// * Definitions
// ******************************************************************************/
//#define xJUST_MASTER
//
///* UART instance and clock */
//#define MASTER_UART UART1
//#define MASTER_UART_CLKSRC UART1_CLK_SRC
//#define MASTER_UART_CLK_FREQ CLOCK_GetFreq(UART1_CLK_SRC)
//#define MASTER_UART_RX_TX_IRQn UART1_RX_TX_IRQn
//
///* UART instance and clock */
//#define SLAVE_UART UART2
//#define SLAVE_UART_CLKSRC UART2_CLK_SRC
//#define SLAVE_UART_CLK_FREQ CLOCK_GetFreq(UART2_CLK_SRC)
//#define SLAVE_UART_RX_TX_IRQn UART2_RX_TX_IRQn
//
///* Task priorities. */
//#define init_task_PRIORITY (configMAX_PRIORITIES - 2)
//#define test_task_heap_size_d	(192)
//
//#define app_message_id_1_d (0x01<<2|message_size_2_bytes_d)
//#define app_message_id_2_d (0x02<<2|message_size_4_bytes_d)
////#define app_message_id_3_d (0x03<<2|message_size_8_bytes_d)
//
//#define DEMO_ADC16_BASE ADC0
//#define DEMO_ADC16_CHANNEL_GROUP 0U
//#define DEMO_ADC16_USER_CHANNEL 12U
//
///*******************************************************************************
// * Prototypes
// ******************************************************************************/
//static void test_task(void *pvParameters);
//
//static void	message_1_callback_master(void* message);
//static void	message_2_callback_master(void* message);
//static void	message_1_callback_slave(void* message);
//static void	message_2_callback_slave(void* message);
//
//
//adc16_config_t adc16ConfigStruct;
//adc16_channel_config_t adc16ChannelConfigStruct;
//
///*******************************************************************************
// * Variables
// ******************************************************************************/
//#define BOARD_LED_GPIO BOARD_LED_RED_GPIO
//#define BOARD_LED_GPIO_PIN BOARD_LED_RED_GPIO_PIN
//
///*******************************************************************************
// * Code
// ******************************************************************************/
///*!
// * @brief Application entry point.
// */
//
//
//
//int main(void)
//{
//	BOARD_InitPins();
//	BOARD_BootClockRUN();
//	BOARD_InitDebugConsole();
//
//	/*ADC Configuration*/
//	ADC16_GetDefaultConfig(&adc16ConfigStruct);
//	ADC16_Init(DEMO_ADC16_BASE, &adc16ConfigStruct);
//	ADC16_EnableHardwareTrigger(DEMO_ADC16_BASE, false); /* Make sure the software trigger is used. */
//	if (kStatus_Success == ADC16_DoAutoCalibration(DEMO_ADC16_BASE))
//	{
//		PRINTF("ADC16_DoAutoCalibration() Done.\r\n");
//	}
//	else
//	{
//		PRINTF("ADC16_DoAutoCalibration() Failed.\r\n");
//	}
//	adc16ChannelConfigStruct.channelNumber = DEMO_ADC16_USER_CHANNEL;
//	adc16ChannelConfigStruct.enableInterruptOnConversionCompleted = false;
//	adc16ChannelConfigStruct.enableDifferentialConversion = false;
//
//	/* Init output LED GPIO. */
//	gpio_pin_config_t led_config = {
//			kGPIO_DigitalOutput,
//			1,
//	};
//
//	GPIO_PinInit(BOARD_LED_GPIO, BOARD_LED_GPIO_PIN, &led_config);
//
//	NVIC_SetPriority(MASTER_UART_RX_TX_IRQn, 5);
//	NVIC_SetPriority(SLAVE_UART_RX_TX_IRQn, 5);
//
//
//    if (xTaskCreate(test_task, "test_task", test_task_heap_size_d, NULL, init_task_PRIORITY, NULL) != pdPASS)
//    {
//        PRINTF("Init Task creation failed!.\r\n");
//        while (1)
//            ;
//    }
//    PRINTF(" *** LIN driver demo ***\r\n");
//    vTaskStartScheduler();
//    for (;;)
//        ;
//}
//
///*!
// * @brief Task responsible for loopback.
// */
//static void test_task(void *pvParameters)
//{
//	int error;
//	lin1d3_nodeConfig_t node_config;
//	lin1d3_handle_t* master_handle;
//	lin1d3_handle_t* slave_handle;
//	/* Set Master Config */
//	node_config.type = lin1d3_master_nodeType;
//	node_config.bitrate = 9600;
//	node_config.uartBase = MASTER_UART;
//	node_config.srcclk = MASTER_UART_CLK_FREQ;
//	memset(node_config.messageTable,0, (sizeof(node_config.messageTable[0])*lin1d3_max_supported_messages_per_node_cfg_d));
//	node_config.messageTable[0].ID = app_message_id_1_d;
//	node_config.messageTable[0].handler = message_1_callback_master;
//	node_config.messageTable[1].ID = app_message_id_2_d;
//	node_config.messageTable[1].handler = message_2_callback_master;
//	/* Init Master node */
//	master_handle = lin1d3_InitNode(node_config);
//#if !defined(JUST_MASTER)
//	/* Set Slave Config */
//	node_config.type = lin1d3_slave_nodeType;
//	node_config.bitrate = 9600;
//	node_config.uartBase = SLAVE_UART;
//	node_config.srcclk = SLAVE_UART_CLK_FREQ;
//	memset(node_config.messageTable,0, (sizeof(node_config.messageTable[0])*lin1d3_max_supported_messages_per_node_cfg_d));
//	node_config.messageTable[0].ID = app_message_id_1_d;
//	node_config.messageTable[0].handler = message_1_callback_slave;
//	node_config.messageTable[1].ID = app_message_id_2_d;
//	node_config.messageTable[1].handler = message_2_callback_slave;
//	/* Init Slave Node*/
//	slave_handle = lin1d3_InitNode(node_config);
//#endif
//
//	if((NULL == master_handle)
//#if !defined(JUST_MASTER)
//		|| (NULL == slave_handle)
//#endif
//	   ){
//		error = kStatus_Fail;
//	}
//	else {
//		error = kStatus_Success;
//	}
//
//	while (kStatus_Success == error)
//    {
//    	vTaskDelay(200);
//    	lin1d3_masterSendMessage(master_handle, app_message_id_1_d);
//    	vTaskDelay(200);
//    	lin1d3_masterSendMessage(master_handle, app_message_id_2_d);
//
//    }
//
//    vTaskSuspend(NULL);
//}
//
//
//static void	message_1_callback_master(void* message)
//{
//	PRINTF("Master got response to message 1\r\n");
//}
//
//static void	message_2_callback_master(void* message)
//{
//	PRINTF("Master got response to message 2\r\n");
//}
//
//
//static void	message_1_callback_slave(void* message)
//{
//	PORT_SetPinMux(BOARD_LED_RED_GPIO_PORT, BOARD_LED_GPIO_PIN, kPORT_MuxAsGpio);
//	GPIO_PortToggle(BOARD_LED_GPIO, 1u << BOARD_LED_GPIO_PIN);
//	PRINTF("Slave got message 1 request\r\n");
//}
//
//static void	message_2_callback_slave(void* message)
//{
//	PRINTF("Slave got message 2 request\r\n");
//
//	ADC16_SetChannelConfig(DEMO_ADC16_BASE, DEMO_ADC16_CHANNEL_GROUP, &adc16ChannelConfigStruct);
//
//	while (0U == (kADC16_ChannelConversionDoneFlag &
//			ADC16_GetChannelStatusFlags(DEMO_ADC16_BASE, DEMO_ADC16_CHANNEL_GROUP)))
//	{
//	}
//	PRINTF("ADC Value: %d\r\n", ADC16_GetChannelConversionValue(DEMO_ADC16_BASE, DEMO_ADC16_CHANNEL_GROUP));
//}
//
