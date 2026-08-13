/**
 * 里程计: 底盘速度积分 → 位姿 (x, y, yaw)
 *
 * 坐标系: 右手系, x 向前, y 向左, yaw 逆时针为正
 * 积分方法: 一阶欧拉
 */

#include "odometry.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* 模块级状态 (单例) */
static float g_odom_x   = 0.0f;
static float g_odom_y   = 0.0f;
static float g_odom_yaw = 0.0f;

/*==============================================================
 *  Odometry_Init
 *============================================================*/
void Odometry_Init(void)
{
    g_odom_x   = 0.0f;
    g_odom_y   = 0.0f;
    g_odom_yaw = 0.0f;
}

/*==============================================================
 *  Odometry_Update
 *
 *  积分公式 (一阶欧拉, 世界坐标系):
 *    x   += (vx * cos(yaw) - vy * sin(yaw)) * dt
 *    y   += (vx * sin(yaw) + vy * cos(yaw)) * dt
 *    yaw += vw * dt
 *============================================================*/
void Odometry_Update(float vx, float vy, float vw, float dt)
{
    float sin_yaw = sinf(g_odom_yaw);
    float cos_yaw = cosf(g_odom_yaw);

    /* 底盘坐标系速度 → 世界坐标系速度 */
    float dx = vx * cos_yaw - vy * sin_yaw;
    float dy = vx * sin_yaw + vy * cos_yaw;

    /* 积分 */
    g_odom_x   += dx * dt;
    g_odom_y   += dy * dt;
    g_odom_yaw += vw * dt;

    /* yaw 归一化到 [-PI, PI] */
    if      (g_odom_yaw >  M_PI) g_odom_yaw -= 2.0f * M_PI;
    else if (g_odom_yaw < -M_PI) g_odom_yaw += 2.0f * M_PI;
}

/*==============================================================
 *  Odometry_GetPose
 *============================================================*/
void Odometry_GetPose(float *x_out, float *y_out, float *yaw_out)
{
    if (x_out)   *x_out   = g_odom_x;
    if (y_out)   *y_out   = g_odom_y;
    if (yaw_out) *yaw_out = g_odom_yaw;
}
