#ifndef __ODOMETRY_H
#define __ODOMETRY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

/*==============================================================
 *  里程计: 对底盘速度积分得到位姿 (x, y, yaw)
 *
 *  坐标系: 右手系, x 向前, y 向左, yaw 逆时针为正
 *  积分方法: 一阶欧拉
 *============================================================*/

/*==============================================================
 *  Odometry_Init   - 里程计归零
 *
 *  将 (x, y, yaw) 全部设为 0。
 *============================================================*/
void Odometry_Init(void);

/*==============================================================
 *  Odometry_Update - 根据底盘速度和时间增量更新位姿
 *
 *  积分公式 (一阶欧拉):
 *    dx   = vx*cos(yaw) - vy*sin(yaw)
 *    dy   = vx*sin(yaw) + vy*cos(yaw)
 *    x   += dx * dt
 *    y   += dy * dt
 *    yaw += vw * dt
 *
 *  @param vx     底盘坐标系前进速度 (m/s)
 *  @param vy     底盘坐标系横移速度 (m/s)
 *  @param vw     旋转角速度 (rad/s)
 *  @param dt     时间增量 (s)
 *============================================================*/
void Odometry_Update(float vx, float vy, float vw, float dt);

/*==============================================================
 *  Odometry_GetPose - 读取当前位姿
 *
 *  @param x_out    输出, X 坐标 (m)
 *  @param y_out    输出, Y 坐标 (m)
 *  @param yaw_out  输出, 偏航角 (rad)
 *============================================================*/
void Odometry_GetPose(float *x_out, float *y_out, float *yaw_out);

#ifdef __cplusplus
}
#endif

#endif /* __ODOMETRY_H */
