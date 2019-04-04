/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "fsl_flexcan.h"
#include "board.h"

#include "fsl_uart_freertos.h" //lin
#include "fsl_uart.h" //lin

#include "pin_mux.h"
#include "clock_config.h"

#include "lin1d3_driver.h" //lin
#include "FreeRTOSConfig.h" //lin

#include "ADC.h"
#include "GPIO.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Period */
#define OS_TICK_PERIOD_100MS 100
#define OS_TICK_PERIOD_50MS 50
/* Task priorities. */
#define hello_task_PRIORITY (configMAX_PRIORITIES - 1)
/* CAN defines */
#define EXAMPLE_CAN CAN0
#define EXAMPLE_CAN_CLKSRC kCLOCK_BusClk
#define EXAMPLE_CAN_CLK_FREQ CLOCK_GetFreq(kCLOCK_BusClk)
#define RX1_MESSAGE_BUFFER_NUM (10)
#define RX2_MESSAGE_BUFFER_NUM (9)
#define TX100_MESSAGE_BUFFER_NUM (8)
#define TX50_MESSAGE_BUFFER_NUM (7)
#define CAN2
/* UART instance and clock */
#define MASTER_UART UART3
#define MASTER_UART_CLKSRC UART3_CLK_SRC
#define MASTER_UART_CLK_FREQ CLOCK_GetFreq(UART3_CLK_SRC)
#define MASTER_UART_RX_TX_IRQn UART3_RX_TX_IRQn
/* Task priorities. */
#define init_task_PRIORITY (configMAX_PRIORITIES - 2)
#define test_task_heap_size_d	(192)

#define app_message_id_1_d (0x01<<2|message_size_2_bytes_d)
#define app_message_id_2_d (0x02<<2|message_size_4_bytes_d)
#define app_message_id_3_d (0x03<<2|message_size_8_bytes_d)
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void task_100ms(void *pvParameters);
static void task_50ms(void *pvParameters);
static void task_rx(void *pvParameters);

//static void test_task(void *pvParameters); //lin

static void	message_1_callback_master(void* message); //lin
static void	message_2_callback_master(void* message); //lin
static void	message_3_callback_master(void* message); //lin
/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile bool txComplete = false;
volatile bool receiving = false;
volatile bool message_received = false;
volatile uint32_t received_mb_idx;
flexcan_handle_t flexcanHandle;
flexcan_mb_transfer_t tx100Xfer, tx50Xfer, rx1Xfer, rx2Xfer;
flexcan_frame_t tx100Frame, tx50Frame, rx1Frame, rx2Frame;
//uint32_t tx100Identifier = 0x20;
uint32_t tx50Identifier = 0x30;
uint32_t tx2Identifier = 0x31;
uint32_t rx1Identifier = 0x10;
uint32_t rx2Identifier = 0x11;


volatile TickType_t xFrequency_LIN = OS_TICK_PERIOD_100MS;

#ifdef CAN2
volatile TickType_t xFrequency = OS_TICK_PERIOD_50MS;
#endif

uint8_t adcLIN_L = 0;
uint8_t adcLIN_H = 0;
volatile bool lin_flag = false;
volatile lin1d3_handle_t* master_handle;
/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Application entry point.
 */

/*!
 * @brief FlexCAN Call Back function
 */
static void flexcan_callback(CAN_Type *base, flexcan_handle_t *handle, status_t status, uint32_t result, void *userData)
{
    switch (status)//indice del mailbox
    {
        /* Process FlexCAN Rx event. */
        case kStatus_FLEXCAN_RxIdle:
        	//receiving = false;
        	message_received = true;
        	received_mb_idx = result;

            break;

        /* Process FlexCAN Tx event. */
        case kStatus_FLEXCAN_TxIdle:
        	txComplete = true;
            break;

        default:
            break;
    }
}

void CAN_Init(void)
{
	flexcan_config_t flexcanConfig;
	flexcan_rx_mb_config_t mbConfig;

	PRINTF("\r\n==FlexCAN example -- Start.==\r\n\r\n");


	    /* Init FlexCAN module. */
	    /*
	     * flexcanConfig.clkSrc = kFLEXCAN_ClkSrcOsc;
	     * flexcanConfig.baudRate = 125000U;
	     * flexcanConfig.maxMbNum = 16;
	     * flexcanConfig.enableLoopBack = false;
	     * flexcanConfig.enableSelfWakeup = false;
	     * flexcanConfig.enableIndividMask = false;
	     * flexcanConfig.enableDoze = false;
	     */
	    FLEXCAN_GetDefaultConfig(&flexcanConfig);
	#if (!defined(FSL_FEATURE_FLEXCAN_SUPPORT_ENGINE_CLK_SEL_REMOVE)) || !FSL_FEATURE_FLEXCAN_SUPPORT_ENGINE_CLK_SEL_REMOVE
	    flexcanConfig.clkSrc = kFLEXCAN_ClkSrcPeri;
	#endif /* FSL_FEATURE_FLEXCAN_SUPPORT_ENGINE_CLK_SEL_REMOVE */
	    flexcanConfig.enableLoopBack = false;
	    FLEXCAN_Init(EXAMPLE_CAN, &flexcanConfig, EXAMPLE_CAN_CLK_FREQ);

	    /* Create FlexCAN handle structure and set call back function. */
	    FLEXCAN_TransferCreateHandle(EXAMPLE_CAN, &flexcanHandle, flexcan_callback, NULL);

	    /* Set Rx Masking mechanism. */
	    //FLEXCAN_SetRxMbGlobalMask(EXAMPLE_CAN, FLEXCAN_RX_MB_STD_MASK(rx1Identifier, 0, 0));

	    /* Setup Rx Message Buffers. */
	    mbConfig.format = kFLEXCAN_FrameFormatStandard;
	    mbConfig.type = kFLEXCAN_FrameTypeData;
	    mbConfig.id = FLEXCAN_ID_STD(rx1Identifier);
	    FLEXCAN_SetRxMbConfig(EXAMPLE_CAN, RX1_MESSAGE_BUFFER_NUM, &mbConfig, true);

	    mbConfig.id = FLEXCAN_ID_STD(rx2Identifier);
	    FLEXCAN_SetRxMbConfig(EXAMPLE_CAN, RX2_MESSAGE_BUFFER_NUM, &mbConfig, true);

	    /* Setup Tx Message Buffers. */
	    FLEXCAN_SetTxMbConfig(EXAMPLE_CAN, TX100_MESSAGE_BUFFER_NUM, true);
	    FLEXCAN_SetTxMbConfig(EXAMPLE_CAN, TX50_MESSAGE_BUFFER_NUM, true);
}

int main(void)
{

	/* Initialize board hardware. */
	BOARD_InitPins();
	BOARD_BootClockRUN();
	BOARD_InitDebugConsole();
	NVIC_SetPriority(MASTER_UART_RX_TX_IRQn, 5); //lin

	CAN_Init();

    xTaskCreate(task_100ms, "100ms Task", configMINIMAL_STACK_SIZE + 10, NULL, hello_task_PRIORITY, NULL);

#ifdef CAN2
    xTaskCreate(task_50ms, "50ms Task", configMINIMAL_STACK_SIZE + 10, NULL, hello_task_PRIORITY, NULL);
#endif
    xTaskCreate(task_rx, "rx Task", configMINIMAL_STACK_SIZE + 10, NULL, hello_task_PRIORITY, NULL);

//    if (xTaskCreate(test_task, "test_task", test_task_heap_size_d, NULL, init_task_PRIORITY, NULL) != pdPASS)
//    {
//        PRINTF("Init Task creation failed!.\r\n");
//        while (1)
//            ;
//    }
    vTaskStartScheduler();

    for (;;)
    	;
}

/*!
 * @brief Task responsible for sending the 100ms message.
 */
static void task_100ms(void *pvParameters)
{
	TickType_t xLastWakeTime;


	volatile uint32_t can_flags = 0;

	int error;
	lin1d3_nodeConfig_t node_config;
	lin1d3_handle_t* master_handle;
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
	if((NULL == master_handle))
	{
		error = kStatus_Fail;
	} else {
		error = kStatus_Success;
	}

	// Initialize the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {
    	if(0 == xFrequency_LIN)
    	{
    		vTaskDelay(10);
    		continue;
    	}
    	/* Get the flags from the can Driver */
    	can_flags = FLEXCAN_GetStatusFlags(EXAMPLE_CAN);
    	/** Transmitting by CAN to panel from global variables uint8
    	    	that will store what we'll get from callback 1
    	    	TODO bit shift **/

    	PRINTF("___________________request ADC LIN\r\n");
    	lin1d3_masterSendMessage(master_handle, app_message_id_1_d);
    	/** Waiting for response from master **/
    	while(false == lin_flag)
    	{
    		vTaskDelay(2);
    	}
    	lin_flag = false;
    	/* Send a Can message */
    	tx100Frame.id = FLEXCAN_ID_STD(tx2Identifier);
    	tx100Frame.format = kFLEXCAN_FrameFormatStandard;
    	tx100Frame.type = kFLEXCAN_FrameTypeData;
    	tx100Frame.length = 8;
    	tx100Xfer.frame = &tx100Frame;
    	tx100Xfer.mbIdx = TX100_MESSAGE_BUFFER_NUM;
        tx100Frame.dataByte0 = adcLIN_L;
    	tx100Frame.dataByte1 = adcLIN_H;
    	FLEXCAN_TransferSendNonBlocking(EXAMPLE_CAN, &flexcanHandle, &tx100Xfer);

        // Wait for the next cycle.
        vTaskDelayUntil( &xLastWakeTime, xFrequency_LIN);
    }
}
//static void	message_1_callback_master(void* message)
//{
//	uint8_t* message_data = (uint8_t*)message;
//	uint16_t adc_value = message_data[0] | (message_data[1] << 8);
//	PRINTF("Master got response to message 1 L:%d,H:%d  value:%d\r\n", message_data[0], message_data[1], adc_value);
//}

//#define I2C_APP_MEM


/*!
 * @brief Task responsible for sending the 50ms message.
 */
static void task_50ms(void *pvParameters)
{
	TickType_t xLastWakeTime;

	adc_config_t config;
	config.base = ADC0;
	adc_init(config);
	volatile bool g_Adc16ConversionDoneFlag;
	g_Adc16ConversionDoneFlag = false;
    uint16_t adc_16value = 0;
    uint8_t adc_value_H = 0;

//	const TickType_t xFrequency = OS_TICK_PERIOD_50MS;
	volatile uint32_t can_flags = 0;

	// Initialize the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {
    	if(0 == xFrequency)
    	{
    		vTaskDelay(10);
    		continue;
    	}
    	/* Get the flags from the can Driver */
    	can_flags = FLEXCAN_GetStatusFlags(EXAMPLE_CAN);

    	/* Send a Can message */
    	tx50Frame.id = FLEXCAN_ID_STD(tx50Identifier);
    	tx50Frame.format = kFLEXCAN_FrameFormatStandard;
    	tx50Frame.type = kFLEXCAN_FrameTypeData;
    	tx50Frame.length = 8;
    	tx50Xfer.frame = &tx50Frame;
    	tx50Xfer.mbIdx = TX50_MESSAGE_BUFFER_NUM;
    	if(0 != xFrequency)
    	{
			FLEXCAN_TransferSendNonBlocking(EXAMPLE_CAN, &flexcanHandle, &tx50Xfer);
    	}

    	g_Adc16ConversionDoneFlag = false;
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
//        PRINTF("ADC Interrupt Count: %d\r\n", g_Adc16InterruptCounter);
        adc_16value = get_adc_value();
        adc_value_H = adc_16value >> 8;
        tx50Frame.dataByte0 = get_adc_value();
    	tx50Frame.dataByte1 = adc_value_H;

//    	tx50Frame.dataByte0 = 50;
//    	tx50Frame.dataByte1++;

        // Wait for the next cycle.
        vTaskDelayUntil( &xLastWakeTime, xFrequency);
    }
}

/*!
 * @brief Task responsible for checking rx messages.
 */
static void task_rx(void *pvParameters)
{
	volatile uint32_t can_flags = 0;
	flexcan_frame_t* rxFrame;

	rtos_gpio_config_t config_LED;
	config_LED.gpio = rtos_gpioE;
	config_LED.port = rtos_gpio_portE;
	config_LED.pin = 26;
	config_LED.pin_direction = rtos_gpio_output;
	rtos_gpio_init(config_LED);

	// Initialize the xLastWakeTime variable with the current time.
    for (;;)
    {
    	/* Get the flags from the can Driver */
    	can_flags = FLEXCAN_GetStatusFlags(EXAMPLE_CAN);

    	if(!receiving) {
        	/* Start receive data through Rx Message Buffer. */
        	rx1Xfer.frame = &rx1Frame;
        	rx1Xfer.mbIdx = RX1_MESSAGE_BUFFER_NUM;
    		FLEXCAN_TransferReceiveNonBlocking(EXAMPLE_CAN, &flexcanHandle, &rx1Xfer);

        	/* Start receive data through Rx Message Buffer. */
        	rx2Xfer.frame = &rx2Frame;
        	rx2Xfer.mbIdx = RX2_MESSAGE_BUFFER_NUM;
    		FLEXCAN_TransferReceiveNonBlocking(EXAMPLE_CAN, &flexcanHandle, &rx2Xfer);

    		receiving = true;
    	}

    	if(message_received){
    		switch(received_mb_idx){
    		case RX1_MESSAGE_BUFFER_NUM:
    			/* Start the reception over */
            	rx1Xfer.frame = &rx1Frame;
            	rx1Xfer.mbIdx = RX1_MESSAGE_BUFFER_NUM;
        		FLEXCAN_TransferReceiveNonBlocking(EXAMPLE_CAN, &flexcanHandle, &rx1Xfer);
    			rxFrame = &rx1Frame;
    			break;
    		case RX2_MESSAGE_BUFFER_NUM:
    			/* Start the reception over */
            	rx2Xfer.frame = &rx2Frame;
            	rx2Xfer.mbIdx = RX2_MESSAGE_BUFFER_NUM;
        		FLEXCAN_TransferReceiveNonBlocking(EXAMPLE_CAN, &flexcanHandle, &rx2Xfer);
    			rxFrame = &rx2Frame;
    			break;
    		default:
    			break;
    		}
    		PRINTF("Received message: MB: %d, ID: 0x%x, data: %d,%d,%d,%d,%d,%d,%d,%d\r\n", received_mb_idx,
    				rxFrame->id>>18,
    				rxFrame->dataByte0, rxFrame->dataByte1, rxFrame->dataByte2, rxFrame->dataByte3,
					rxFrame->dataByte4, rxFrame->dataByte5, rxFrame->dataByte6, rxFrame->dataByte7
					);
    		if((rxFrame->id>>18) == 0x10)
    		{
    			if(rxFrame->dataByte0 & 0x02)
    			{
    				rtos_gpio_LED_ON(config_LED);
    			}
    			else
    			{
    				rtos_gpio_LED_OFF(config_LED);
    			}
    			if (rxFrame->dataByte0 & 0x04)
    			{
    				PRINTF("led LIN on");
    				lin1d3_masterSendMessage(master_handle, app_message_id_2_d);
				}
    			else
    			{
    				PRINTF("led LIN off");
    				lin1d3_masterSendMessage(master_handle, app_message_id_3_d);
				}
    		}
    		else if((rxFrame->id>>18) == 0x11)
			{
//    			if ((rxFrame->dataByte1) > 0)
    			{
    				xFrequency = (rxFrame->dataByte1)*100;
				}
//    			if ((rxFrame->dataByte2) > 0)
    			{
    				xFrequency_LIN = (rxFrame->dataByte2)*100;
				}

			}
    		message_received = false;
    	}
    	vTaskDelay(10);//2
    }
}

//static void test_task(void *pvParameters)
//{
//	int error;
//	lin1d3_nodeConfig_t node_config;
////	lin1d3_handle_t* master_handle;
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
//	node_config.messageTable[2].ID = app_message_id_3_d;
//	node_config.messageTable[2].handler = message_3_callback_master;
//	/* Init Master node */
//	master_handle = lin1d3_InitNode(node_config);
//	if((NULL == master_handle))
//	{
//		error = kStatus_Fail;
//	} else {
//		error = kStatus_Success;
//	}
//
//	while (kStatus_Success == error) {
//		vTaskDelay(200);
//		lin1d3_masterSendMessage(master_handle, app_message_id_1_d);
//		vTaskDelay(200);
//		lin1d3_masterSendMessage(master_handle, app_message_id_2_d);
//		vTaskDelay(200);
//		lin1d3_masterSendMessage(master_handle, app_message_id_3_d);
//	}
//
//	vTaskSuspend(NULL);
//}

static void	message_1_callback_master(void* message)
{
	uint8_t* message_data = (uint8_t*)message;
	adcLIN_L = message_data[0];
	adcLIN_H = message_data[1];
	uint16_t adc_value = message_data[0] | (message_data[1] << 8);
	PRINTF("Master got response to message 1 ADC value: %d\r\n", adc_value);
	lin_flag  = true;
}

static void	message_2_callback_master(void* message)
{
	uint8_t* message_data = (uint8_t*)message;
	PRINTF("Master got response to message 2 %d\r\n", message_data[0]);
}
static void	message_3_callback_master(void* message)
{
	uint8_t* message_data = (uint8_t*)message;
	PRINTF("Master got response to message 3 %d\r\n", message_data[0]);

}
