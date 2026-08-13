/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart_device.h"
#include "can_device.h"
#include "dbus_c620.h"
#include "chassis.h"
#include "serial_cmd.h"
#include "queue.h"
#include "stdio.h"
#include "string.h"
#include "sys_lock.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* printf 互斥锁 (全局, 供 sys_lock.h 的 SAFE_PRINTF 宏使用) */
SemaphoreHandle_t g_xPrintfMutex = NULL;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
	.name = "defaultTask",
	.stack_size = 128 * 4,
	.priority = (osPriority_t)osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

void CAN_TaskFunction(void *pvParameters)
{
	struct CAN_Device *pdev_can1 = GetCANDevice("can1");
	pdev_can1->Init(pdev_can1);

	/* 激活 USART6 DMA+IDLE 接收 (指令口); g_rx_stream 已在 SerialCmd_Init 中创建 */
	struct UART_Device *pdev_uart6 = GetUARTDevice("uart6");
	if (pdev_uart6) pdev_uart6->Init(pdev_uart6, 460800, 'N', 8, 1);

	CanMsg_t rx_msg = {0};
	CanMsg_t tx_msg = {0};
	uint8_t tx_data[8];

	/* 底盘模块初始化 (运动学参数 + 电机PID + 里程计归零) */
	static Chassis_t chassis;
	Chassis_Init(&chassis);

	static uint32_t tick = 0;
	ChassisCmd_t cmd;           /* 串口指令 (差速: vx=linear, vy=0, vw=angular) */
	SerialCmd_State_t state;    /* 运行状态 (STOP/RUN, 上电默认 STOP) */
	SAFE_PRINTF("CAN task start\n");

	tx_msg.id = CAN_CHASSIS_ALL_ID;
	tx_msg.ide = 0;
	tx_msg.rtr = 0;
	tx_msg.dlc = 8;

	while (1)
	{
		/* 状态机: STOP 强制停转 (上电默认); RUN 取差速指令 (500ms 无帧回零) */
		SerialCmd_GetState(&state);
		switch (state)
		{
		case SERIAL_CMD_STATE_RUN:
			SerialCmd_GetCmd(&cmd);
			Chassis_SetCmd(&chassis, cmd.vx, cmd.vy, cmd.vw);
			break;
		case SERIAL_CMD_STATE_STOP:
		default:
			/* STOP: 强制停转 */
			Chassis_SetCmd(&chassis, 0.0f, 0.0f, 0.0f);
			break;
		}

		/* 读取电机反馈, 填入底盘状态 */
		while (pdev_can1->Recv(pdev_can1, &rx_msg, 0) == 0)
		{
			if (rx_msg.id >= CAN_3508_M1_ID && rx_msg.id <= CAN_3508_M4_ID)
			{
				uint8_t idx = rx_msg.id - CAN_3508_M1_ID;
				C620_DecodeFeedback(rx_msg.data, &chassis.motor_fb[idx]);
			}
		}

		/* 逆解算 + PID 速度环 + 打包 → tx_data */
		Chassis_Control(&chassis, tx_data);

		for (int i = 0; i < 8; i++)
			tx_msg.data[i] = tx_data[i];
		uint32_t ret = pdev_can1->Send(pdev_can1, &tx_msg, 1);

		if (++tick >= 250)
		{
			tick = 0;
			SAFE_PRINTF("TGT:%d,%d,%d,%d SPD:%d,%d,%d,%d CUR:%d,%d,%d,%d OUT:%d,%d,%d,%d | TX:%04X %04X %04X %04X ret=%lu\r\n",
						(int)chassis.target_rpm[0], (int)chassis.target_rpm[1],
						(int)chassis.target_rpm[2], (int)chassis.target_rpm[3],
						chassis.motor_fb[0].speed_rpm, chassis.motor_fb[1].speed_rpm,
						chassis.motor_fb[2].speed_rpm, chassis.motor_fb[3].speed_rpm,
						chassis.motor_fb[0].given_current, chassis.motor_fb[1].given_current,
						chassis.motor_fb[2].given_current, chassis.motor_fb[3].given_current,
						chassis.target_current[0], chassis.target_current[1],
						chassis.target_current[2], chassis.target_current[3],
						((uint16_t)tx_data[0] << 8) | tx_data[1],
						((uint16_t)tx_data[2] << 8) | tx_data[3],
						((uint16_t)tx_data[4] << 8) | tx_data[5],
						((uint16_t)tx_data[6] << 8) | tx_data[7],
						ret);
		}
		osDelay(2);
	}
}

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	g_xPrintfMutex = xSemaphoreCreateMutex();
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of defaultTask */
	defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* 串口指令模块: 创建接收 StreamBuffer + 解析任务 (须在激活 USART1 RX 前调用) */
	SerialCmd_Init();

	xTaskCreate(
		CAN_TaskFunction,
		"CAN_task",
		256 * 4,
		NULL,
		osPriorityNormal,
		NULL);
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	/* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
	/* USER CODE BEGIN StartDefaultTask */
	/* Infinite loop */
	for (;;)
	{
		HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
		osDelay(500);
		HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);

		HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
		osDelay(500);
		HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);

		HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
		osDelay(500);
		HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
	}
	/* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
