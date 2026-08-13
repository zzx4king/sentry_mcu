/**
 * 底盘状态管理
 *
 * 职责:
 *   - 持有 4 个电机反馈/目标数据
 *   - 接收底盘速度指令
 *   - 串联运动学 → PID → 里程计 → CAN 输出
 */

#include "chassis.h"
#include "kinematics.h"
#include "odometry.h"
#include "motor_control.h"
#include "dbus_c620.h"
#include "stddef.h"

/*==============================================================
 *  Chassis_Init
 *============================================================*/
void Chassis_Init(Chassis_t *chassis)
{
    if (!chassis) return;

    /* 归零所有状态 */
    for (int i = 0; i < 4; i++)
    {
        chassis->motor_fb[i]       = (Motor_Feedback_t){0};
        chassis->target_current[i] = 0;
        chassis->target_rpm[i]     = 0.0f;
    }
    chassis->cmd  = (ChassisCmd_t){0.0f, 0.0f, 0.0f};
    chassis->pose = (ChassisPose_t){0.0f, 0.0f, 0.0f};
    chassis->last_tick = 0;

    /* 初始化电机 PID 控制器 */
    MotorControl_Init();

    /* 初始化运动学参数 (实测值) */
    KinematicsParams_t kp;
    kp.wheel_radius     = 0.0765f;   /* 轮子半径 s = 153mm / 2 */
    kp.half_wheelbase_a = 0.218f;    /* 中心到轮子距离的 x/y 分量 a = 218mm */
    kp.half_wheelbase_b = 0.218f;    /* a = b (对称布局) */
    kp.reduction_ratio  = 15.76f;    /* 减速比 i = 268 / 17 */
    Kinematics_Init(&kp);

    /* 里程计归零 */
    Odometry_Init();

    chassis->inited = 1;
}

/*==============================================================
 *  Chassis_SetCmd
 *============================================================*/
void Chassis_SetCmd(Chassis_t *chassis, float vx, float vy, float vw)
{
    if (!chassis) return;

    chassis->cmd.vx = vx;
    chassis->cmd.vy = vy;
    chassis->cmd.vw = vw;
}

/*==============================================================
 *  Chassis_Control  (每 osDelay(2) = 2ms 调用一次)
 *
 *  流程:
 *    ③ 逆解算: cmd → 4 电机目标 RPM
 *    ④ PID 速度环: 目标 RPM → 电机电流
 *    ⑤ 打包 CAN 数据
 *    ⑦ 正解算: 实际 RPM → 底盘实际速度
 *    ⑧ 里程计积分: 实际速度 → 位姿
 *============================================================*/
void Chassis_Control(Chassis_t *chassis, uint8_t tx_data[8])
{
    if (!chassis || !tx_data || !chassis->inited) return;

    /*----------------------------------------------------------
     *  ③ 运动学逆解: 底盘速度 → 4 电机目标 RPM (CAN ID 顺序)
     *--------------------------------------------------------*/
    Kinematics_Inverse(chassis->cmd.vx, chassis->cmd.vy, chassis->cmd.vw,
                       chassis->target_rpm);

    /*----------------------------------------------------------
     *  ④ PID 速度环: 目标 RPM → 电机电流 (每个电机独立 PID)
     *--------------------------------------------------------*/
    for (int i = 0; i < 4; i++)
    {
        chassis->target_current[i] = MotorControl_GetCurrentByRPM(
            i,
            chassis->target_rpm[i],
            (float)chassis->motor_fb[i].speed_rpm);
    }

    /*----------------------------------------------------------
     *  ⑤ 打包 CAN 数据 (CAN ID 顺序: 0=0x201, 1=0x202, 2=0x203, 3=0x204)
     *--------------------------------------------------------*/
    C620_PackChassisCmd(chassis->target_current[0],
                        chassis->target_current[1],
                        chassis->target_current[2],
                        chassis->target_current[3],
                        tx_data);

    /*----------------------------------------------------------
     *  ⑦ 正解算: 4 电机实际 RPM → 底盘实际速度
     *--------------------------------------------------------*/
    float actual_rpm[4];
    for (int i = 0; i < 4; i++)
        actual_rpm[i] = (float)chassis->motor_fb[i].speed_rpm;

    float vx_act, vy_act, vw_act;
    Kinematics_Forward(actual_rpm, &vx_act, &vy_act, &vw_act);

    /*----------------------------------------------------------
     *  ⑧ 里程计积分: 实际速度 → 位姿 (dt = 2ms)
     *--------------------------------------------------------*/
    Odometry_Update(vx_act, vy_act, vw_act, 0.002f);
    Odometry_GetPose(&chassis->pose.x, &chassis->pose.y, &chassis->pose.yaw);
}
