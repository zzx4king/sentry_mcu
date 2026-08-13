#ifndef __DBUS_C620_H
#define __DBUS_C620_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

/*==============================================================
 *  CAN ID 定义 (参考大疆 C620 电调手册 + 14.CAN 官方例程)
 *============================================================*/

/* 底盘电机控制发送 ID：一个帧控制 4 个电机 (ID 1~4) */
#define CAN_CHASSIS_ALL_ID      0x200

/* 底盘电机反馈 ID：每个电调独立回传 */
#define CAN_3508_M1_ID          0x201
#define CAN_3508_M2_ID          0x202
#define CAN_3508_M3_ID          0x203
#define CAN_3508_M4_ID          0x204

/* 电流范围：-16384 ~ 16384 映射到 -20A ~ +20A */
#define C620_CURRENT_MIN        (-16384)
#define C620_CURRENT_MAX        16384

/*==============================================================
 *  电机反馈数据结构 (对应手册 8 字节反馈帧)
 *
 *  反馈数据帧格式 (大端)：
 *    Byte 0-1: 转子机械角度 ecd  (uint16, 0~8191)
 *    Byte 2-3: 转子转速 speed_rpm (int16)
 *    Byte 4-5: 实际转矩电流 given_current (int16)
 *    Byte 6:   电机温度 temperate (uint8)
 *    Byte 7:   保留
 *============================================================*/
typedef struct {
    uint16_t ecd;            /* 转子机械角度 0~8191 */
    int16_t  speed_rpm;      /* 转子转速 rpm */
    int16_t  given_current;  /* 实际转矩电流 */
    uint8_t  temperate;      /* 电机温度 */
    int16_t  last_ecd;       /* 上次角度 (用于计算转子总圈数) */
} Motor_Feedback_t;

/*==============================================================
 *  C620_PackChassisCmd - 将 4 个电机的目标电流打包为 8 字节 CAN 帧
 *
 *  @param m1~m4: 电机1~4 的目标电流，范围 [-16384, 16384]
 *  @param tx_data[8]: 输出，CAN 数据域 8 字节 (大端序)
 *
 *  数据帧格式 (ID=0x200)：
 *    Byte 0-1: 电机1 电流高8位 | 低8位
 *    Byte 2-3: 电机2 电流高8位 | 低8位
 *    Byte 4-5: 电机3 电流高8位 | 低8位
 *    Byte 6-7: 电机4 电流高8位 | 低8位
 *
 *  可在 FreeRTOS 任务中直接调用
 *============================================================*/
void C620_PackChassisCmd(int16_t m1, int16_t m2, int16_t m3, int16_t m4,
                         uint8_t tx_data[8]);

/*==============================================================
 *  C620_DecodeFeedback - 解析电机反馈帧
 *
 *  @param rx_data[8]: 接收到的 8 字节反馈数据
 *  @param motor: 输出，解析结果写入此结构体
 *
 *  可在 FreeRTOS 任务中直接调用
 *============================================================*/
void C620_DecodeFeedback(const uint8_t rx_data[8], Motor_Feedback_t *motor);

#ifdef __cplusplus
}
#endif

#endif /* __DBUS_C620_H */
