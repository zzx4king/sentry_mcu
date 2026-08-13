#ifndef __CHASSIS_H
#define __CHASSIS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "dbus_c620.h"

/*==============================================================
 *  底盘速度指令 (世界坐标系或底盘坐标系)
 *============================================================*/
typedef struct {
    float vx;   /* 前进速度 (m/s) */
    float vy;   /* 横移速度 (m/s) */
    float vw;   /* 旋转角速度 (rad/s) */
} ChassisCmd_t;

/*==============================================================
 *  底盘位姿 (里程计积分结果)
 *============================================================*/
typedef struct {
    float x;    /* X 坐标 (m) */
    float y;    /* Y 坐标 (m) */
    float yaw;  /* 偏航角 (rad) */
} ChassisPose_t;

/*==============================================================
 *  底盘状态 (单例, 持有全部运行时数据)
 *============================================================*/
typedef struct {
    /* 4 个电机的反馈数据 */
    Motor_Feedback_t motor_fb[4];

    /* 4 个电机的目标电流 (-16384 ~ +16384) */
    int16_t          target_current[4];

    /* 4 个电机的目标转速 (由运动学逆解算出) */
    float            target_rpm[4];

    /* 底盘速度指令 (来自遥控器/上位机) */
    ChassisCmd_t     cmd;

    /* 底盘里程计位姿 */
    ChassisPose_t    pose;

    /* 上次更新时间戳 (用于 dt 计算) */
    uint32_t         last_tick;

    /* 初始化标志 */
    uint8_t          inited;
} Chassis_t;

/*==============================================================
 *  Chassis_Init    - 底盘模块初始化
 *
 *  初始化电机 PID、运动学参数、里程计归零。
 *  应在 FreeRTOS 启动后、CAN 任务开始前调用一次。
 *============================================================*/
void Chassis_Init(Chassis_t *chassis);

/*==============================================================
 *  Chassis_SetCmd  - 设置底盘目标速度
 *
 *  @param chassis  底盘实例指针
 *  @param vx       前进速度 (m/s), 正值前进
 *  @param vy       横移速度 (m/s), 正值左移
 *  @param vw       旋转角速度 (rad/s), 正值逆时针
 *============================================================*/
void Chassis_SetCmd(Chassis_t *chassis, float vx, float vy, float vw);

/*==============================================================
 *  Chassis_Control - 底盘控制主循环 (每次 osDelay(2) 调用一次)
 *
 *  内部流程:
 *    1. 运动学逆解: cmd → 4 个电机目标 RPM
 *    2. PID 速度环: 目标 RPM → 电机电流
 *    3. 里程计更新: 正解算 + 积分
 *    4. 返回打包好的 CAN 数据供上层发送
 *
 *  @param chassis  底盘实例指针
 *  @param tx_data  输出, 8 字节 CAN 数据域 (可直接发到 0x200)
 *============================================================*/
void Chassis_Control(Chassis_t *chassis, uint8_t tx_data[8]);

#ifdef __cplusplus
}
#endif

#endif /* __CHASSIS_H */
