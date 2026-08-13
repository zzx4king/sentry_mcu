/**
 * 全向轮底盘运动学 (法向投影法)
 *
 * 逆解算:  底盘速度 (vx, vy, vw) → 4 个电机 RPM (CAN ID 顺序)
 * 正解算:  4 个电机 RPM (CAN ID 顺序) → 底盘速度 (vx, vy, vw)
 *
 * 轮子映射:
 *   公式索引: 0=左下  1=右下  2=右上  3=左上
 *   CAN ID:  0x201=右下  0x202=左下  0x203=左上  0x204=右上
 *   输出数组:  [0]=右下   [1]=左下   [2]=左上   [3]=右上  (CAN ID 顺序)
 */

#include "kinematics.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define SQRT2_2  0.7071067811865475f   /* √2 / 2 */

static KinematicsParams_t g_kp;

/* 预计算常数 */
static float g_rpm_to_mps;     /* RPM → 轮面线速度 (m/s) */
static float g_mps_to_rpm;     /* 轮面线速度 → RPM */
static float g_inv_coeff;      /* 逆解算: K × √2/2 = mps_to_rpm × √2/2 */
static float g_fwd_lin;        /* 正解算 vx/vy: rpm_to_mps / (2√2) */
static float g_fwd_rot;        /* 正解算 omega: rpm_to_mps / (4√2·a) */

/*==============================================================
 *  Kinematics_Init
 *============================================================*/
void Kinematics_Init(const KinematicsParams_t *params)
{
    if (!params) return;

    g_kp = *params;

    float s = g_kp.wheel_radius;
    float a = g_kp.half_wheelbase_a;       /* a = b (对称布局) */
    float i = g_kp.reduction_ratio;

    /* rpm_to_mps = π × s / (30 × i):  电机 1 RPM 对应的轮面线速度 */
    g_rpm_to_mps  = (float)(M_PI * s) / (30.0f * i);
    g_mps_to_rpm  = 1.0f / g_rpm_to_mps;

    /* 逆解系数: K × √2/2 = mps_to_rpm × √2/2 */
    g_inv_coeff = g_mps_to_rpm * SQRT2_2;

    /* 正解系数 */
    g_fwd_lin = g_rpm_to_mps * 0.5f / SQRT2_2;             /* rpm_to_mps / (2√2) */
    g_fwd_rot = g_rpm_to_mps / (4.0f * SQRT2_2 * a);       /* rpm_to_mps / (4√2·a) */
}

/*==============================================================
 *  Kinematics_Inverse
 *
 *  公式 (k = g_inv_coeff):
 *    RPM₀ = k × (−vx + vy − 2ωa)  左下
 *    RPM₁ = k × (−vx − vy − 2ωa)  右下
 *    RPM₂ = k × (+vx − vy − 2ωa)  右上
 *    RPM₃ = k × (+vx + vy − 2ωa)  左上
 *
 *  输出 → CAN ID 顺序: [RPM₁, RPM₀, RPM₃, RPM₂]
 *============================================================*/
void Kinematics_Inverse(float vx, float vy, float vw, float rpm_out[4])
{
    if (!rpm_out) return;

    float k = g_inv_coeff;
    float a = g_kp.half_wheelbase_a;
    float two_omega_a = 2.0f * vw * a;

    /* 公式索引顺序: 0=左下, 1=右下, 2=右上, 3=左上 */
    float w[4];
    w[0] = k * (-vx + vy - two_omega_a);   /* 左下 */
    w[1] = k * (-vx - vy - two_omega_a);   /* 右下 */
    w[2] = k * (+vx - vy - two_omega_a);   /* 右上 */
    w[3] = k * (+vx + vy - two_omega_a);   /* 左上 */

    /* 限幅 */
    for (int i = 0; i < 4; i++)
    {
        if      (w[i] >  KINEMATICS_MAX_RPM) w[i] =  KINEMATICS_MAX_RPM;
        else if (w[i] < -KINEMATICS_MAX_RPM) w[i] = -KINEMATICS_MAX_RPM;
    }

    /* 重排为 CAN ID 顺序: 0=右下, 1=左下, 2=左上, 3=右上 */
    rpm_out[0] = w[1];   /* 右下 → 0x201 M1 */
    rpm_out[1] = w[0];   /* 左下 → 0x202 M2 */
    rpm_out[2] = w[3];   /* 左上 → 0x203 M3 */
    rpm_out[3] = w[2];   /* 右上 → 0x204 M4 */
}

/*==============================================================
 *  Kinematics_Forward
 *
 *  输入: CAN ID 顺序 → 映射回公式索引
 *    RPM₀ = rpm_in[1]  (左下)
 *    RPM₁ = rpm_in[0]  (右下)
 *    RPM₂ = rpm_in[3]  (右上)
 *    RPM₃ = rpm_in[2]  (左上)
 *
 *  公式:
 *    vx    = fwd_lin × (−RPM₀ − RPM₁ + RPM₂ + RPM₃)
 *    vy    = fwd_lin × (+RPM₀ − RPM₁ − RPM₂ + RPM₃)
 *    omega = fwd_rot × (+RPM₀ + RPM₁ + RPM₂ + RPM₃)
 *============================================================*/
void Kinematics_Forward(const float rpm_in[4], float *vx_out, float *vy_out, float *vw_out)
{
    if (!rpm_in) return;

    /* CAN ID 顺序 → 公式索引 */
    float w0 = rpm_in[1];   /* 左下 */
    float w1 = rpm_in[0];   /* 右下 */
    float w2 = rpm_in[3];   /* 右上 */
    float w3 = rpm_in[2];   /* 左上 */

    if (vx_out) *vx_out = g_fwd_lin * (-w0 - w1 + w2 + w3);
    if (vy_out) *vy_out = g_fwd_lin * (+w0 - w1 - w2 + w3);
    if (vw_out) *vw_out = g_fwd_rot * (+w0 + w1 + w2 + w3);
}
