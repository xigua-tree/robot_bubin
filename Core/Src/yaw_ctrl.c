#include "yaw_ctrl.h"

float g_target_yaw_angle = 0.0f;        /* 目标 yaw 角度 (度) */

static PID_t s_yaw_pid;                 /* yaw 角度环 PID 实例 */

/**
 * @brief  初始化 yaw 角度环 PID
 * @note   所有 PID 参数初始化为 0，用户后续调整
 */
void YawCtrl_Init(void)
{
    PID_Init(&s_yaw_pid, 0.0f, 0.0f, 0.0f, YAW_OUT_MAX);
}

/**
 * @brief  执行一次 yaw 角度环 PID 计算
 * @param  measured_yaw  当前 yaw 角度 (度)
 * @param  dt            控制周期 (秒)，暂未使用（保留接口）
 * @return omega         旋转角速度输出 (RPM 等效值)
 */
float YawCtrl_Update(float measured_yaw, float dt)
{
    (void)dt;  /* 增量式 PID 自身管理时序，dt 保留备用 */

    /* 计算最短角度误差，归一化到 [-180, 180] 度 */
    float err = g_target_yaw_angle - measured_yaw;
    while (err > 180.0f)  err -= 360.0f;
    while (err < -180.0f) err += 360.0f;

    /* 复用现有增量式 PID：
     * PID_Update 内部计算 error = target - measured
     * 设 target=0, measured=-err → error = 0-(-err) = err */
    s_yaw_pid.target = 0.0f;
    return PID_Update(&s_yaw_pid, -err);
}
