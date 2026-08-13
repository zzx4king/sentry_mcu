#include "can_device.h"
#include "can.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "cmsis_os.h"
#include "stdio.h"
#include "string.h"
#include "sys_lock.h"

static QueueHandle_t g_xCAN1_RX_Queue;
static SemaphoreHandle_t g_CAN1_TX_Semaphore;

/* CAN1 RX FIFO0 消息到达回调 (中断上下文) */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan != &hcan1 || !g_xCAN1_RX_Queue) return;

    CanMsg_t msg = {0};
    CAN_RxHeaderTypeDef rx_header;
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, msg.data);
    msg.id  = rx_header.IDE ? rx_header.ExtId : rx_header.StdId;
    msg.ide = rx_header.IDE;
    msg.rtr = rx_header.RTR;
    msg.dlc = rx_header.DLC;
    xQueueSendFromISR(g_xCAN1_RX_Queue, &msg, NULL);
}

/* CAN1 TX 邮箱空中断回调 */
static void CAN_TX_Cplt_Callback(CAN_HandleTypeDef *hcan)
{
    if (hcan != &hcan1 || !g_CAN1_TX_Semaphore) return;

    static BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(g_CAN1_TX_Semaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) { CAN_TX_Cplt_Callback(hcan); }
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan) { CAN_TX_Cplt_Callback(hcan); }
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan) { CAN_TX_Cplt_Callback(hcan); }

/* CAN 错误回调 (Bus-Off / Error Passive 等) */
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan != &hcan1) return;

    uint32_t err = HAL_CAN_GetError(hcan);

    /* Bus-Off: 硬件已自动恢复 (AutoBusOff=ENABLE) */
    if (err & HAL_CAN_ERROR_BOF)
    {
        /* 总线无对端设备时属正常现象, 不打印避免刷屏 */
    }
    /* Error Passive (TEC/REC > 127): 清零通知标志即可 */
    HAL_CAN_ResetError(hcan);
}

int CAN1_Init(struct CAN_Device *pDev)
{
    if (!g_xCAN1_RX_Queue)
    {
        g_xCAN1_RX_Queue = xQueueCreate(CAN_QUEUE_SIZE, sizeof(CanMsg_t));
        g_CAN1_TX_Semaphore = xSemaphoreCreateCounting(3, 3);  /* 3 个 TX 邮箱初始全空 */

        /* 创建失败则卡死在这里 (便于调试, 后续可改为返回错误) */
        if (!g_xCAN1_RX_Queue || !g_CAN1_TX_Semaphore)
        {
            SAFE_PRINTF("CAN1: queue or semaphore create FAILED!\n");
            for(;;) { osDelay(100); }
        }

        /* 配置 CAN 滤波器：不过滤，接收所有 ID */
        CAN_FilterTypeDef can_filter_st;
        can_filter_st.FilterActivation = ENABLE;
        can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
        can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
        can_filter_st.FilterIdHigh = 0x0000;
        can_filter_st.FilterIdLow = 0x0000;
        can_filter_st.FilterMaskIdHigh = 0x0000;
        can_filter_st.FilterMaskIdLow = 0x0000;
        can_filter_st.FilterBank = 0;
        can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
        HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);

        /* 先停止 CAN (确保从任何异常状态中恢复) */
        HAL_CAN_Stop(&hcan1);

        /* 强制复位 CAN 状态机 (清除 Bus-Off / Error Passive 等残留) */
        hcan1.State = HAL_CAN_STATE_READY;

        /* 先激活中断通知 (此时 CAN 在 INIT 模式, 不会触发中断) */
        HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
        HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY);

        /* 再启动 CAN (离开 INIT 模式, 开始参与总线) */
        if (HAL_CAN_Start(&hcan1) != HAL_OK)
        {
            SAFE_PRINTF("CAN1: Start FAILED!\n");
            for(;;) { osDelay(100); }
        }

        /* 清错误计数器并开启错误中断, 监控总线状态 */
        __HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_ERROR);
        __HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_BUSOFF);  /* Bus-Off 中断 */

        SAFE_PRINTF("CAN1 init is ok\n");
    }
    return 0;
}

int CAN1_Send(struct CAN_Device *pDev, CanMsg_t *pMsg, int timeout)
{
    CAN_TxHeaderTypeDef tx_header = {0};
    uint32_t tx_mailbox;

    tx_header.StdId = pMsg->id;
    tx_header.ExtId = pMsg->id;
    tx_header.IDE   = pMsg->ide ? CAN_ID_EXT : CAN_ID_STD;
    tx_header.RTR   = pMsg->rtr ? CAN_RTR_REMOTE : CAN_RTR_DATA;
    tx_header.DLC   = pMsg->dlc;
    tx_header.TransmitGlobalTime = DISABLE;

    /* 等待 TX 邮箱可用 */
    if (pdTRUE == xSemaphoreTake(g_CAN1_TX_Semaphore, timeout))
    {
        if (HAL_CAN_AddTxMessage(&hcan1, &tx_header, pMsg->data, &tx_mailbox) != HAL_OK)
        {
            xSemaphoreGive(g_CAN1_TX_Semaphore);  /* 归还信号量 */
            return -1;
        }
        return 0;
    }
    return -1;
}

int CAN1_Recv(struct CAN_Device *pDev, CanMsg_t *pMsg, int timeout)
{
    if (pdPASS == xQueueReceive(g_xCAN1_RX_Queue, pMsg, timeout))
        return 0;
    else
        return -1;
}

/* 设备实例 */
struct CAN_Device g_can1_dev = {"can1", CAN1_Init, CAN1_Send, CAN1_Recv};

static struct CAN_Device *g_can_devices[] = {&g_can1_dev};

struct CAN_Device *GetCANDevice(char *name)
{
    for (int i = 0; i < sizeof(g_can_devices) / sizeof(g_can_devices[0]); i++)
    {
        if (!strcmp(name, g_can_devices[i]->name))
            return g_can_devices[i];
    }
    return NULL;
}
