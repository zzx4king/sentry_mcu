/**
 * 串口指令协议解析模块
 *
 * 数据流:
 *   串口 RX → DMA/IDLE 中断 → SerialCmd_FeedFromISR → StreamBuffer
 *     → 解析任务(状态机: WAIT_HDR→TYPE→LEN→DATA→CRC) → 校验通过 → 全局 cmd
 *     → CAN 控制任务每周期 SerialCmd_GetCmd → Chassis_SetCmd(vx,0,vw)
 *
 * 协议帧: Header(0xA5) + Type + Len + Data[Len] + CRC8(MAXIM, poly 0x31)
 * CRC 范围: Header + Type + Len + Data (不含 CRC 自身)
 *
 * 注: 本模块与具体串口解耦。usart_device.c 的 IDLE/HalfCplt/Cplt 回调
 *     需改为调用 SerialCmd_FeedFromISR 投递字节, 并在某处调用
 *     UART1_Rx_Start 激活 DMA+IDLE 接收 (集成步骤, 不在本文件内)。
 */

#include "serial_cmd.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stream_buffer.h"
#include "semphr.h"
#include "cmsis_os.h"
#include "string.h"
#include "sys_lock.h"
#include "stdio.h"

/* 调试开关: 1 = 每 2s 打印一次统计。默认关闭, 避免与指令串口争用 TX */
#define SERIAL_CMD_DEBUG        0

#define SERIAL_CMD_RX_BUF_SIZE  256u            /* StreamBuffer 容量 */
#define SERIAL_CMD_TASK_STACK   (256u * 4u)     /* 解析任务栈 (字节) */
#define SERIAL_CMD_TASK_TIMEOUT_MS 100u         /* 解析任务阻塞超时 */

/*==============================================================
 *  解析状态机
 *============================================================*/
typedef enum {
    STATE_WAIT_HDR = 0,
    STATE_WAIT_TYPE,
    STATE_WAIT_LEN,
    STATE_WAIT_DATA,
    STATE_WAIT_CRC
} parse_state_t;

/*==============================================================
 *  模块全局状态
 *============================================================*/
static StreamBufferHandle_t g_rx_stream = NULL;
static SemaphoreHandle_t    g_cmd_mutex = NULL;

/* 全局指令 + 运行状态 (受 g_cmd_mutex 保护) */
static ChassisCmd_t     g_cmd          = {0.0f, 0.0f, 0.0f};
static SerialCmd_State_t g_state       = SERIAL_CMD_STATE_STOP;  /* 上电默认 STOP */
static uint32_t         g_last_rx_tick = 0;     /* 0 = 从未收到有效差速帧 */
static uint8_t          g_active        = 0;    /* 1 = 收到过且未超时 */

/* 统计计数 (仅解析任务写, 读取宽松) */
static volatile uint32_t g_good_frames = 0;
static volatile uint32_t g_crc_errors  = 0;
static volatile uint32_t g_len_errors  = 0;
static volatile uint32_t g_type_errors = 0;

/* 解析状态机变量 (仅解析任务内使用) */
static parse_state_t s_state    = STATE_WAIT_HDR;
static uint8_t       s_type     = 0;
static uint8_t       s_len      = 0;
static uint8_t       s_data[SERIAL_CMD_MAX_DATA];
static uint8_t       s_data_idx = 0;
static uint8_t       s_crc      = 0;   /* 运行中的 CRC 累加值 */

/*==============================================================
 *  CRC-8/MAXIM 单字节更新
 *  多项式 0x31, 初值 0x00, 输入/输出反射 (reversed poly = 0x8C)
 *  等效于标准 CRC-8/MAXIM, 校验 "123456789" = 0xA1
 *============================================================*/
static uint8_t crc8_maxim_update(uint8_t crc, uint8_t byte)
{
    crc ^= byte;
    for (int i = 0; i < 8; i++) {
        if (crc & 0x01u)
            crc = (uint8_t)((crc >> 1) ^ 0x8Cu);
        else
            crc = (uint8_t)(crc >> 1);
    }
    return crc;
}

/*==============================================================
 *  handle_frame - 处理一帧完整数据 (CRC 已校验通过)
 *
 *  Type 0x00 stop : 切换 STOP 状态, 清零 g_cmd (防切回 RUN 用旧指令)
 *  Type 0x01 run  : 切换 RUN  状态 (g_active 保持 0, 等新差速帧才动)
 *  Type 0x10 差速 : 仅 RUN 状态下更新 g_cmd; STOP 状态忽略 (安全)
 *============================================================*/
static void handle_frame(void)
{
    if (s_type == SERIAL_CMD_TYPE_STOP && s_len == 0) {
        /* stop 状态帧: 切换状态 + 清零指令 */
        xSemaphoreTake(g_cmd_mutex, portMAX_DELAY);
        g_state  = SERIAL_CMD_STATE_STOP;
        g_cmd.vx = 0.0f;
        g_cmd.vy = 0.0f;
        g_cmd.vw = 0.0f;
        g_active = 0;
        xSemaphoreGive(g_cmd_mutex);
        g_good_frames++;
        return;
    }

    if (s_type == SERIAL_CMD_TYPE_RUN && s_len == 0) {
        /* run 状态帧: 切换状态; g_active=0, 须等差速帧才驱动 */
        xSemaphoreTake(g_cmd_mutex, portMAX_DELAY);
        g_state  = SERIAL_CMD_STATE_RUN;
        g_active = 0;
        xSemaphoreGive(g_cmd_mutex);
        g_good_frames++;
        return;
    }

    if (s_type == SERIAL_CMD_TYPE_DIFFERENTIAL && s_len == 8) {
        float linear, angular;
        /* 小端序, STM32 原生, 直接 memcpy */
        memcpy(&linear,  &s_data[0], 4);
        memcpy(&angular, &s_data[4], 4);
        /* NaN 防护 (NaN != NaN), 防止 PID 被毒化 */
        if (linear != linear || angular != angular) {
            g_type_errors++;
            return;
        }
        xSemaphoreTake(g_cmd_mutex, portMAX_DELAY);
        /* STOP 状态下忽略差速指令 (安全: 即使收到也不驱动) */
        if (g_state == SERIAL_CMD_STATE_RUN) {
            g_cmd.vx = linear;       /* 前进速度 */
            g_cmd.vy = 0.0f;         /* 强制无横移 */
            g_cmd.vw = angular;      /* 旋转角速度 */
            g_last_rx_tick = xTaskGetTickCount();
            g_active = 1;
        }
        xSemaphoreGive(g_cmd_mutex);
        g_good_frames++;
        return;
    }

    /* 未知 Type 或 Len 与载荷不匹配 */
    g_type_errors++;
}

/*==============================================================
 *  parse_byte - 状态机喂入一个字节
 *============================================================*/
static void parse_byte(uint8_t b)
{
    switch (s_state) {
    case STATE_WAIT_HDR:
        if (b == SERIAL_CMD_HEADER) {
            s_crc = crc8_maxim_update(0x00u, b);   /* 初值 0x00, 喂入 Header */
            s_state = STATE_WAIT_TYPE;
        }
        break;

    case STATE_WAIT_TYPE:
        s_type = b;
        s_crc  = crc8_maxim_update(s_crc, b);
        s_state = STATE_WAIT_LEN;
        break;

    case STATE_WAIT_LEN:
        s_len = b;
        s_crc = crc8_maxim_update(s_crc, b);
        if (s_len > SERIAL_CMD_MAX_DATA) {
            g_len_errors++;
            s_state = STATE_WAIT_HDR;
        } else {
            s_data_idx = 0;
            s_state = (s_len == 0) ? STATE_WAIT_CRC : STATE_WAIT_DATA;
        }
        break;

    case STATE_WAIT_DATA:
        s_data[s_data_idx++] = b;
        s_crc = crc8_maxim_update(s_crc, b);
        if (s_data_idx >= s_len) {
            s_state = STATE_WAIT_CRC;
        }
        break;

    case STATE_WAIT_CRC:
        if (b == s_crc) {
            handle_frame();
        } else {
            g_crc_errors++;
        }
        s_state = STATE_WAIT_HDR;
        break;

    default:
        s_state = STATE_WAIT_HDR;
        break;
    }
}

/*==============================================================
 *  SerialCmd_Task - 解析任务: 阻塞等 StreamBuffer, 批量喂状态机
 *============================================================*/
static void SerialCmd_Task(void *pv)
{
    uint8_t buf[SERIAL_CMD_MAX_DATA + 4];
    size_t  n;
#if SERIAL_CMD_DEBUG
    TickType_t last_dbg = xTaskGetTickCount();
#endif
    (void)pv;

    for (;;) {
        n = xStreamBufferReceive(g_rx_stream, buf, sizeof(buf),
                                 pdMS_TO_TICKS(SERIAL_CMD_TASK_TIMEOUT_MS));
        for (size_t i = 0; i < n; i++) {
            parse_byte(buf[i]);
        }

#if SERIAL_CMD_DEBUG
        if ((xTaskGetTickCount() - last_dbg) >= pdMS_TO_TICKS(2000)) {
            last_dbg = xTaskGetTickCount();
            SAFE_PRINTF("SCMD: ok=%lu crc=%lu len=%lu type=%lu act=%u vx=%.2f vw=%.2f\r\n",
                        (unsigned long)g_good_frames, (unsigned long)g_crc_errors,
                        (unsigned long)g_len_errors,  (unsigned long)g_type_errors,
                        (unsigned int)g_active, (double)g_cmd.vx, (double)g_cmd.vw);
        }
#endif
    }
}

/*==============================================================
 *  SerialCmd_Init
 *============================================================*/
int SerialCmd_Init(void)
{
    if (g_rx_stream == NULL) {
        g_rx_stream = xStreamBufferCreate(SERIAL_CMD_RX_BUF_SIZE, 1);
    }
    if (g_cmd_mutex == NULL) {
        g_cmd_mutex = xSemaphoreCreateMutex();
    }
    if (g_rx_stream == NULL || g_cmd_mutex == NULL) {
        return -1;
    }
    if (xTaskCreate(SerialCmd_Task, "SerialCmd", SERIAL_CMD_TASK_STACK,
                    NULL, osPriorityBelowNormal, NULL) != pdPASS) {
        return -1;
    }
    return 0;
}

/*==============================================================
 *  SerialCmd_FeedFromISR
 *============================================================*/
void SerialCmd_FeedFromISR(const uint8_t *data, uint16_t len,
                           BaseType_t *pxHigherPriorityTaskWoken)
{
    if (g_rx_stream != NULL && data != NULL && len > 0) {
        xStreamBufferSendFromISR(g_rx_stream, (const void *)data, (size_t)len,
                                 pxHigherPriorityTaskWoken);
    }
}

/*==============================================================
 *  SerialCmd_GetCmd - 取当前生效指令, 超时自动回零
 *============================================================*/
void SerialCmd_GetCmd(ChassisCmd_t *out_cmd)
{
    if (out_cmd == NULL) return;

    TickType_t now = xTaskGetTickCount();
    xSemaphoreTake(g_cmd_mutex, portMAX_DELAY);
    /* 超时判定 (uint32 减法天然处理 tick 回绕) */
    if (g_active &&
        ((uint32_t)now - g_last_rx_tick) > (uint32_t)pdMS_TO_TICKS(SERIAL_CMD_TIMEOUT_MS)) {
        g_cmd.vx = 0.0f;
        g_cmd.vy = 0.0f;
        g_cmd.vw = 0.0f;
        g_active = 0;
    }
    *out_cmd = g_cmd;
    xSemaphoreGive(g_cmd_mutex);
}

/*==============================================================
 *  SerialCmd_GetState - 取当前运行状态 (STOP/RUN)
 *============================================================*/
void SerialCmd_GetState(SerialCmd_State_t *out_state)
{
    if (out_state == NULL) return;
    xSemaphoreTake(g_cmd_mutex, portMAX_DELAY);
    *out_state = g_state;
    xSemaphoreGive(g_cmd_mutex);
}

/*==============================================================
 *  SerialCmd_GetStatus
 *============================================================*/
void SerialCmd_GetStatus(SerialCmd_Status_t *st)
{
    if (st == NULL) return;

    st->good_frames = g_good_frames;
    st->crc_errors  = g_crc_errors;
    st->len_errors  = g_len_errors;
    st->type_errors = g_type_errors;

    xSemaphoreTake(g_cmd_mutex, portMAX_DELAY);
    st->last_rx_tick = g_last_rx_tick;
    st->active       = g_active;
    xSemaphoreGive(g_cmd_mutex);

    /* 实时超时复核 */
    if (st->active &&
        ((uint32_t)xTaskGetTickCount() - st->last_rx_tick) >
            (uint32_t)pdMS_TO_TICKS(SERIAL_CMD_TIMEOUT_MS)) {
        st->active = 0;
    }
}
