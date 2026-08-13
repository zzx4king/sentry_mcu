#include "usart_device.h"
#include "usart.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "sys_lock.h"
#include "serial_cmd.h"
#include "stdio.h"
#include "string.h"

extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart6_rx;


uint8_t RX1_Offset = 0 ;  // 计算偏移量 , 用来计算接收数据长度的
static uint8_t g_uart1_rx_buf[DMA_BUF_SIZE]; // 接收数据存放的数组

static SemaphoreHandle_t g_UART1_TX_Semaphore;

uint8_t RX6_Offset = 0 ;  // USART6 接收偏移量
static uint8_t g_uart6_rx_buf[DMA_BUF_SIZE]; // USART6 接收缓冲

static SemaphoreHandle_t g_UART6_TX_Semaphore;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart1)
	{
		xSemaphoreGiveFromISR(g_UART1_TX_Semaphore, NULL);
	}
	else if (huart == &huart6)
	{
		xSemaphoreGiveFromISR(g_UART6_TX_Semaphore, NULL);
	}
}


// DMA 接收到一半的中断
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
		if(huart->Instance == USART1)
		{
			uint8_t Length  =  DMA_BUF_SIZE/2 - RX1_Offset ;
			//printf("HLength=%d\n",Length);
			//HAL_UART_Transmit(huart,g_uart1_rx_buf+RX1_Offset,Length,HAL_MAX_DELAY);
			BaseType_t xHigherPriorityTaskWoken = pdFALSE;
			SerialCmd_FeedFromISR(&g_uart1_rx_buf[RX1_Offset], Length, &xHigherPriorityTaskWoken);
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
			RX1_Offset += Length;
		}
		else if(huart->Instance == USART6)
		{
			uint8_t Length  =  DMA_BUF_SIZE/2 - RX6_Offset ;
			BaseType_t xHigherPriorityTaskWoken = pdFALSE;
			SerialCmd_FeedFromISR(&g_uart6_rx_buf[RX6_Offset], Length, &xHigherPriorityTaskWoken);
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
			RX6_Offset += Length;
		}
}

// DMA传输完成中断   , 就是接收满了的时候 触发中断
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
        uint8_t Length  =  DMA_BUF_SIZE - RX1_Offset ;
        //HAL_UART_Transmit(huart,g_uart1_rx_buf+RX1_Offset,Length,HAL_MAX_DELAY);

        //printf("CLength=%d\n",Length);
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        SerialCmd_FeedFromISR(&g_uart1_rx_buf[RX1_Offset], Length, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        RX1_Offset = 0 ; // 清空dma 位置基准值
    }
    else if(huart->Instance == USART6)
    {
        uint8_t Length  =  DMA_BUF_SIZE - RX6_Offset ;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        SerialCmd_FeedFromISR(&g_uart6_rx_buf[RX6_Offset], Length, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        RX6_Offset = 0 ; // 清空dma 位置基准值
    }
}

// 用户自定义的函数 ， 处理串口空闲中断
void USER_UART_IRQHandler(UART_HandleTypeDef *huart)
{

    if(huart->Instance == USART1)                                   //判断是否是串口1
    {
        if(__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) != RESET )   //判断是否是空闲中断
        {
            __HAL_UART_CLEAR_IDLEFLAG(huart);    //清除空闲中断标志（否则会一直不断进入中断）

            //计算接收到的数据长度 : BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx) - RX1_Offset
            uint8_t Length  =  DMA_BUF_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx) - RX1_Offset;
            //HAL_UART_Transmit(huart,g_uart1_rx_buf+RX1_Offset,Length,HAL_MAX_DELAY);
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            SerialCmd_FeedFromISR(&g_uart1_rx_buf[RX1_Offset], Length, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            RX1_Offset += Length;
				  //printf("ILength=%d\n",Length);
        }
    }
    else if(huart->Instance == USART6)                              //判断是否是串口6
    {
        if(__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) != RESET )
        {
            __HAL_UART_CLEAR_IDLEFLAG(huart);

            uint8_t Length  =  DMA_BUF_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart6_rx) - RX6_Offset;
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            SerialCmd_FeedFromISR(&g_uart6_rx_buf[RX6_Offset], Length, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            RX6_Offset += Length;
        }
    }
}

int UART1_Rx_Start(struct UART_Device *pDev, int baud, char parity, int data_bit, int stop_bit)
{
	if (!g_UART1_TX_Semaphore)
	{
		g_UART1_TX_Semaphore = xSemaphoreCreateBinary( );


	    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE); //使能串UART1 IDLE(空闲)中断
	    HAL_UART_Receive_DMA(&huart1, g_uart1_rx_buf, DMA_BUF_SIZE); //设置DMA传输，将uart1的数据搬运到g_uart1_rx_buf中
		  SAFE_PRINTF("Uart1 init is ok\n");
	}
	return 0;
}

int UART1_Send(struct UART_Device *pDev, uint8_t *datas, uint32_t len, int timeout)
{
	HAL_UART_Transmit_DMA(&huart1, datas, len);

	/* 等待1个信号量(为何不用mutex? 因为在中断里Give mutex会出锿) */
	if (pdTRUE == xSemaphoreTake(g_UART1_TX_Semaphore, timeout))
		return 0;
	else
		return -1;
}

int UART6_Rx_Start(struct UART_Device *pDev, int baud, char parity, int data_bit, int stop_bit)
{
	if (!g_UART6_TX_Semaphore)
	{
		g_UART6_TX_Semaphore = xSemaphoreCreateBinary( );


	    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE); //使能 USART6 IDLE(空闲)中断
	    HAL_UART_Receive_DMA(&huart6, g_uart6_rx_buf, DMA_BUF_SIZE); //设置DMA传输，将uart6的数据搬运到g_uart6_rx_buf中
		  SAFE_PRINTF("Uart6 init is ok\n");
	}
	return 0;
}

int UART6_Send(struct UART_Device *pDev, uint8_t *datas, uint32_t len, int timeout)
{
	HAL_UART_Transmit_DMA(&huart6, datas, len);

	/* 等待1个信号量(为何不用mutex? 因为在中断里Give mutex会出错) */
	if (pdTRUE == xSemaphoreTake(g_UART6_TX_Semaphore, timeout))
		return 0;
	else
		return -1;
}

struct UART_Device g_uart1_dev = {"uart1", UART1_Rx_Start, UART1_Send, NULL};
struct UART_Device g_uart6_dev = {"uart6", UART6_Rx_Start, UART6_Send, NULL};

static struct UART_Device *g_uart_devices[] = {&g_uart1_dev, &g_uart6_dev};

struct UART_Device *GetUARTDevice(char *name)
{
	int i = 0;
	for (i = 0; i < sizeof(g_uart_devices)/sizeof(g_uart_devices[0]); i++)
	{
		if (!strcmp(name, g_uart_devices[i]->name))
			return g_uart_devices[i];
	}

	return NULL;
}
