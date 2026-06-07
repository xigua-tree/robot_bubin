#ifndef __KINEMATICS_H
#define __KINEMATICS_H

/* 全向轮底盘几何参数（米） */
#define LX         0.183f       /* 轮子到中心的 X 方向距离 */
#define LY         0.183f       /* 轮子到中心的 Y 方向距离 */

/* 单轮最大目标转速 (RPM) */
#define MAX_SPEED  200.0f

/* 摇杆映射系数：摇杆偏移量 → 目标速度 RPM */
#define ROCKER_SCALE  0.1f    /* 300 RPM / 2048 ≈ 0.146 */

/* 圆周率 */
#define PI          3.1415926f
#define PI_2        (2.0f * PI)

/**
 * @brief  全向轮运动学解算（4轮独立驱动）
 * @param  Vx   X 方向速度（正值 = 前进方向）
 * @param  Vy   Y 方向速度（正值 = 左移方向）
 * @param  omega 旋转角速度（正值 = 逆时针）
 * @param  wheel_rpm[4]  输出的 4 轮目标转速 (RPM)
 */
void OmniKinematics(int Vx, int Vy, int omega, float wheel_rpm[4]);

/**
 * @brief  角度归一化到 [-PI, PI]
 */
float normalizeAngleRad(float angle);

#endif
