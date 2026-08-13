#ifndef __KINEMATICS_H
#define __KINEMATICS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

/*==============================================================
 *  麦克纳姆轮底盘运动学 (法向投影法)
 *
 *  底盘坐标系 (俯视):
 *     前方 = +vx,  左方 = +vy,  逆时针 = +vw
 *
 *  轮子编号 (公式索引):
 *     3(左上) ──── 2(右上)
 *        │    车头    │
 *        │    → vx   │
 *     0(左下) ──── 1(右下)
 *
 *  CAN ID 映射:
 *     0x201 M1 = 右下(公式1),  0x202 M2 = 左下(公式0)
 *     0x203 M3 = 左上(公式3),  0x204 M4 = 右上(公式2)
 *============================================================*/

#define KINEMATICS_MAX_RPM  8000.0f

/*==============================================================
 *  运动学参数
 *============================================================*/
typedef struct {
    float wheel_radius;       /* 轮子半径 s (m) */
    float half_wheelbase_a;   /* 中心到轮子的 x/y 分量 a (m), 对称布局 a=b */
    float half_wheelbase_b;   /* 同上, a=b */
    float reduction_ratio;    /* 减速比 i */
} KinematicsParams_t;

void Kinematics_Init(const KinematicsParams_t *params);

/*==============================================================
 *  Kinematics_Inverse - 逆解算: 底盘速度 → 4 电机 RPM (CAN ID 顺序输出)
 *
 *  法向投影公式:
 *    RPM_i = K × (n̂_i · v_Pi)
 *    K = i/s × (60/2π)
 *    v_Pi = (vx − ω·y_i,  vy + ω·x_i)
 *
 *  展开后 (k = K × √2/2):
 *    RPM₀ = k × (−vx + vy − 2ωa)  左下(公式0)
 *    RPM₁ = k × (−vx − vy − 2ωa)  右下(公式1)
 *    RPM₂ = k × (+vx − vy − 2ωa)  右上(公式2)
 *    RPM₃ = k × (+vx + vy − 2ωa)  左上(公式3)
 *
 *  输出重排为 CAN ID 顺序: {右下, 左下, 左上, 右上}
 *
 *  @param vx      前进速度 (m/s)
 *  @param vy      横移速度 (m/s)
 *  @param vw      旋转角速度 (rad/s)
 *  @param rpm_out 输出, CAN ID 顺序: [M1右下, M2左下, M3左上, M4右上] (rpm)
 *============================================================*/
void Kinematics_Inverse(float vx, float vy, float vw, float rpm_out[4]);

/*==============================================================
 *  Kinematics_Forward - 正解算: 4 电机 RPM (CAN ID 顺序) → 底盘速度
 *
 *  ω_i = RPM_i × s/i × (2π/60)    (轮子角速度 rad/s)
 *
 *  vx    = s/(2√2) × (−ω₀ − ω₁ + ω₂ + ω₃)
 *  vy    = s/(2√2) × (+ω₀ − ω₁ − ω₂ + ω₃)
 *  omega = s/(4r)  × (+ω₀ + ω₁ + ω₂ + ω₃)    (r = √2·a)
 *
 *  @param rpm_in 输入, CAN ID 顺序: [M1右下, M2左下, M3左上, M4右上] (rpm)
 *  @param vx_out 输出, 前进速度 (m/s)
 *  @param vy_out 输出, 横移速度 (m/s)
 *  @param vw_out 输出, 旋转角速度 (rad/s)
 *============================================================*/
void Kinematics_Forward(const float rpm_in[4], float *vx_out, float *vy_out, float *vw_out);

#ifdef __cplusplus
}
#endif

#endif /* __KINEMATICS_H */
