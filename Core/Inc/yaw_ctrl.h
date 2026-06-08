#ifndef __YAW_CTRL_H
#define __YAW_CTRL_H

#include "pid.h"

/* yaw 角度环 PID 输出限幅 (RPM 等效值) */
#define YAW_OUT_MAX  200.0f

/* 目标 yaw 角度 (度)，由外部设置 */
extern float g_target_yaw_angle;
extern float yaw_err;
/**
 * @brief  初始化 yaw 角度环 PID（Kp/Ki/Kd = 0，用户自行调试）
 */
void YawCtrl_Init(void);

/**
 * @brief  执行一次 yaw 角度环 PID 计算
 * @param  measured_yaw  当前 yaw 角度 (度)
 * @param  dt            控制周期 (秒)
 * @return omega         旋转角速度输出 (RPM 等效值)
 */
float YawCtrl_Update(float measured_yaw, float dt);
void YawCtrl_SetPID(float Kp, float Ki, float Kd);


#endif
