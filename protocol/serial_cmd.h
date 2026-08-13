#ifndef __SERIAL_CMD_H
#define __SERIAL_CMD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "FreeRTOS.h"   /* BaseType_t */
#include "stdint.h"
#include "chassis.h"    /* ChassisCmd_t */

/*==============================================================
 *  串口指令协议模块 (serial_cmd)
 *
 *  通过串口接收上位机的差速驱动指令 (线速度 + 角速度) 控制底盘。
 *  本模块只负责: 接收字节缓冲 + 帧解析 + 维护全局指令;
 *  串口硬件的 DMA/IDLE 激活由 usart_device 完成, 字节经
 *  SerialCmd_FeedFromISR 投递进来; CAN 控制任务每周期用
 *  SerialCmd_GetCmd 取最新指令驱动底盘。
 *
 *  协议帧格式 (变长, 小端):
 *    +--------+--------+--------+----------------+--------+
 *    | Header | Type   | Len    | Data[Len]      | CRC    |
 *    | 1 字节 | 1 字节 | 1 字节 | Len 字节       | 1 字节 |
 *    +--------+--------+--------+----------------+--------+
 *
 *  - Header = 0xA5 (帧头, 用于字节流同步)
 *  - Type   : 指令类型
 *  - Len    : Data 字段字节数 (0 ~ SERIAL_CMD_MAX_DATA)
 *  - Data   : 载荷, 含义由 Type 决定
 *  - CRC    : CRC-8/MAXIM, 多项式 0x31, 初值 0x00, 输入/输出反射
 *             校验字符串 "123456789" 的结果 = 0xA1 (供上位机对齐)
 *             CRC 计算范围: Header + Type + Len + Data (不含 CRC 自身)
 *
 *  已定义指令:
 *    Type 0x00 : stop 状态帧 (Len = 0)
 *                切换到 STOP 状态: 底盘强制停转, 同时清零 g_cmd
 *                (防止切回 RUN 时使用旧指令继续运动)
 *    Type 0x01 : run 状态帧 (Len = 0)
 *                切换到 RUN 状态: 允许接收底盘控制帧驱动底盘
 *    Type 0x10 : 差速驱动指令 (Len = 8, 仅 RUN 状态下生效)
 *                Data = linear_vel (float,4B,LE) + angular_vel (float,4B,LE)
 *                映射到底盘: vx = linear_vel, vw = angular_vel, vy = 0
 *                (强制无横移, 麦轮底盘按差速模型驱动)
 *
 *  状态机:
 *    MCU 上电默认 STOP。CAN 控制任务每周期读状态:
 *      STOP → Chassis_SetCmd(0,0,0) 强制停转
 *      RUN  → 取最新差速指令驱动底盘 (500ms 无帧自动回零)
 *============================================================*/

#define SERIAL_CMD_HEADER            0xA5u
#define SERIAL_CMD_MAX_DATA          64u     /* Data 字段最大字节数 (防溢出) */
#define SERIAL_CMD_TIMEOUT_MS        500u    /* RUN 状态下指令超时回零阈值 */

/* 指令类型 */
#define SERIAL_CMD_TYPE_STOP         0x00u   /* stop 状态帧 (Len=0) */
#define SERIAL_CMD_TYPE_RUN          0x01u   /* run  状态帧 (Len=0) */
#define SERIAL_CMD_TYPE_DIFFERENTIAL 0x10u   /* 差速驱动: linear + angular (Len=8) */

/* 运行状态 (受 g_cmd_mutex 保护, 由 stop/run 帧切换) */
typedef enum {
    SERIAL_CMD_STATE_STOP = 0,   /* 默认: 底盘强制停转 */
    SERIAL_CMD_STATE_RUN        /* 允许接收差速指令驱动底盘 */
} SerialCmd_State_t;

/*==============================================================
 *  SerialCmd_Init - 初始化串口指令模块
 *
 *  创建接收 StreamBuffer、指令互斥锁与解析任务。系统启动期间调用
 *  一次 (建议放在 MX_FREERTOS_Init 的 RTOS_THREADS 段)。
 *  @return 0 成功, -1 失败
 *============================================================*/
int SerialCmd_Init(void);

/*==============================================================
 *  SerialCmd_FeedFromISR - 从中断上下文投递接收到的字节
 *
 *  在 USART 的 DMA 半满/全满/IDLE 回调中调用, 将本次收到的一批
 *  字节投递到内部 StreamBuffer 供解析任务消费。
 *  (集成: 替换 usart_device.c 中 xQueueSendFromISR 的逐字节投递)
 *
 *  @param data     本次接收到的数据指针
 *  @param len      数据长度 (字节)
 *  @param pxHigherPriorityTaskWoken : FreeRTOS 标准出参, 传入 ISR 同名变量
 *============================================================*/
void SerialCmd_FeedFromISR(const uint8_t *data, uint16_t len,
                           BaseType_t *pxHigherPriorityTaskWoken);

/*==============================================================
 *  SerialCmd_GetCmd - 获取当前生效的底盘速度指令
 *
 *  CAN 控制任务在 RUN 状态下每个周期调用一次。若距离上一帧
 *  有效差速指令超过 SERIAL_CMD_TIMEOUT_MS, 自动返回零指令
 *  (安全停转, 防通信断线失控)。vy 固定为 0 (差速模型无横移)。
 *  STOP 状态下不应调用本接口 (由调用方直接 Chassis_SetCmd(0,0,0))。
 *
 *  典型用法 (freertos.c CAN_TaskFunction):
 *    SerialCmd_State_t st;
 *    SerialCmd_GetState(&st);
 *    if (st == SERIAL_CMD_STATE_RUN) {
 *        ChassisCmd_t cmd;
 *        SerialCmd_GetCmd(&cmd);
 *        Chassis_SetCmd(&chassis, cmd.vx, cmd.vy, cmd.vw);
 *    } else {
 *        Chassis_SetCmd(&chassis, 0, 0, 0);   // STOP 强制停转
 *    }
 *
 *  @param out_cmd : 输出, 当前生效指令 (vx=linear, vy=0, vw=angular)
 *============================================================*/
void SerialCmd_GetCmd(ChassisCmd_t *out_cmd);

/*==============================================================
 *  SerialCmd_GetState - 获取当前运行状态 (STOP / RUN)
 *
 *  状态由 stop/run 帧切换, MCU 上电默认 STOP。
 *  CAN 控制任务每周期调用, 据此决定是否驱动底盘。
 *  @param out_state : 输出, 当前状态
 *============================================================*/
void SerialCmd_GetState(SerialCmd_State_t *out_state);

/*==============================================================
 *  SerialCmd_GetStatus - 获取协议解析统计信息 (调试用)
 *============================================================*/
typedef struct {
    uint32_t good_frames;   /* 校验通过的有效帧数 */
    uint32_t crc_errors;    /* CRC 校验失败次数 */
    uint32_t len_errors;    /* Len 字段越界次数 */
    uint32_t type_errors;   /* Type 未知 / 载荷非法次数 */
    uint32_t last_rx_tick;  /* 最近一次有效帧的 tick */
    uint8_t  active;        /* 1 = 通信正常(未超时), 0 = 超时或从未收到 */
} SerialCmd_Status_t;

void SerialCmd_GetStatus(SerialCmd_Status_t *st);

#ifdef __cplusplus
}
#endif

#endif /* __SERIAL_CMD_H */
