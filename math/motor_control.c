#include "motor_control.h"
#include "pid.h"

/* PID 速度环参数 (前馈 + 轻量 PID 修正) */
#define PID_SPEED_KP            1.0f
#define PID_SPEED_KI            1.0f
#define PID_SPEED_KD            0.0f
#define PID_SPEED_TAU           0.02f
#define PID_SPEED_LIM_MIN     (-8000.0f)
#define PID_SPEED_LIM_MAX      8000.0f
#define PID_SPEED_LIM_MIN_INT (-8000.0f)
#define PID_SPEED_LIM_MAX_INT  8000.0f
#define PID_SPEED_SAMPLE_TIME  0.002f    /* 2ms 对应 osDelay(2) */

/* 目标转速最大值 (M3508 额定转速约 8000 rpm) */
#define MAX_SPEED_RPM           8000.0f
#define MOTOR_CURRENT_MAX       16384   /* 电机最大电流 对应 20A */

/* 4 个电机的速度环 PID 控制器 */
static PIDController g_speed_pid[4];
static uint8_t       g_pid_inited = 0;

void MotorControl_Init(void)
{
    for (uint8_t i = 0; i < 4; i++)
    {
        g_speed_pid[i].Kp        = PID_SPEED_KP;
        g_speed_pid[i].Ki        = PID_SPEED_KI;
        g_speed_pid[i].Kd        = PID_SPEED_KD;
        g_speed_pid[i].tau       = PID_SPEED_TAU;
        g_speed_pid[i].limMin    = PID_SPEED_LIM_MIN;
        g_speed_pid[i].limMax    = PID_SPEED_LIM_MAX;
        g_speed_pid[i].limMinInt = PID_SPEED_LIM_MIN_INT;
        g_speed_pid[i].limMaxInt = PID_SPEED_LIM_MAX_INT;
        g_speed_pid[i].T         = PID_SPEED_SAMPLE_TIME;

        PIDController_Init(&g_speed_pid[i]);
    }
    g_pid_inited = 1;
}

int16_t MotorControl_GetCurrentByRPM(uint8_t motor_idx, float target_rpm, float actual_rpm)
{
    if (motor_idx >= 4) return 0;
    if (!g_pid_inited) return 0;

    PIDController *pid = &g_speed_pid[motor_idx];

    /* 目标接近0时清积分器, 防止松杆后残留输出导致自移动 */
    if (target_rpm < 10.0f && target_rpm > -10.0f) {
        pid->integrator = 0.0f;
    }

    /* 前馈: 目标RPM → 基础电流 (立即响应, 对应电机反电动势 + 摩擦) */
    float ff_ratio = target_rpm / MAX_SPEED_RPM;
    if (ff_ratio >  1.0f) ff_ratio =  1.0f;
    if (ff_ratio < -1.0f) ff_ratio = -1.0f;
    float ff = ff_ratio * (float)MOTOR_CURRENT_MAX;

    /* PID 只修正前馈的残余误差 (限幅 ±8000, 防止过度干预) */
    float pid_out = PIDController_Update(pid, target_rpm, actual_rpm);

    float out = ff + pid_out;
    if (out >  MOTOR_CURRENT_MAX) out =  MOTOR_CURRENT_MAX;
    if (out < -MOTOR_CURRENT_MAX) out = -MOTOR_CURRENT_MAX;

    return (int16_t)out;
}
