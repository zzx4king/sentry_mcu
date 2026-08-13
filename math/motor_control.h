#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#include "stdint.h"

/**
 * @brief  初始化 4 个电机的 PID 速度环控制器
 */
void MotorControl_Init(void);

/**
 * @brief  直接指定目标转速，通过 PID 速度环计算电机目标电流
 * @param  motor_idx    电机索引 0~3
 * @param  target_rpm   目标转速 (rpm)
 * @param  actual_rpm   实际转速 (rpm)
 * @return 电机目标电流 (-16384 ~ +16384)
 */
int16_t MotorControl_GetCurrentByRPM(uint8_t motor_idx, float target_rpm, float actual_rpm);

#endif /* __MOTOR_CONTROL_H */
