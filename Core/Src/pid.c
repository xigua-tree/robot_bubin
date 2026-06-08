#include "pid.h"
#include <string.h>
#include <math.h>

void PID_Init(PID_t *pid, float Kp, float Ki, float Kd, float out_max)
{
    memset(pid, 0, sizeof(PID_t));
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->out_max = out_max;
    pid->out_min = -out_max;
    pid->integral_max = out_max * 0.3f;  /* 积分限幅为输出的30% */
}

float PID_Update(PID_t *pid, float measured)
{
    /* 计算误差 */
    float error = pid->target - measured;
    pid->error[2] = pid->error[1];   /* e(k-2) = e(k-1) */
    pid->error[1] = pid->error[0];   /* e(k-1) = e(k)   */
    pid->error[0] = error;           /* e(k)   = error   */

    /* 死区：误差 < 10 RPM 时冻结积分 */
    if (fabsf(error) < 1.0f) {
        pid->integral = 0.0f;
        /* 目标=0 且速度≈0 → 强制停转；目标≠0 → 保持当前输出 */
        return pid->output;
    }
    if (fabsf(pid->target) < 10.0f) {
        pid->output = 0.0f;
        return 0.0f;
    }
    /* 积分分离：误差较大时不累加积分，防止饱和 */
    if (fabsf(error) < pid->out_max * 0.5f) {
        pid->integral += error;
        if (pid->integral > pid->integral_max)  pid->integral = pid->integral_max;
        if (pid->integral < -pid->integral_max) pid->integral = -pid->integral_max;
    } else {
        pid->integral = 0.0f;
    }

    /* 增量式 PID */
    /* Δu = Kp*(e(k)-e(k-1)) + Ki*e(k) + Kd*(e(k)-2e(k-1)+e(k-2)) */
    float delta = pid->Kp * (pid->error[0] - pid->error[1])
                + pid->Ki * pid->error[0]
                + pid->Kd * (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);

    pid->output += delta;

    /* 输出限幅 */
    if (pid->output > pid->out_max)  pid->output = pid->out_max;
    if (pid->output < pid->out_min)  pid->output = pid->out_min;

    return pid->output;
}

float PID_Update_Angle(PID_t *pid, float err)
{
    /* 计算误差 */
    float error = -err;
    pid->error[2] = pid->error[1];   /* e(k-2) = e(k-1) */
    pid->error[1] = pid->error[0];   /* e(k-1) = e(k)   */
    pid->error[0] = error;           /* e(k)   = error   */

    /* 死区：误差 < 10 RPM 时冻结积分 */
    if (fabsf(error) < 5.0f) {
        pid->integral = 0.0f;
        /* 目标=0 且速度≈0 → 强制停转；目标≠0 → 保持当前输出 */
        return 0;
    }

    // /* 积分分离：误差较大时不累加积分，防止饱和 */
    // if (fabsf(error) < pid->out_max * 0.5f) {
    //     pid->integral += error;
    //     if (pid->integral > pid->integral_max)  pid->integral = pid->integral_max;
    //     if (pid->integral < -pid->integral_max) pid->integral = -pid->integral_max;
    // } else {
    //     pid->integral = 0.0f;
    // }

    /* 增量式 PID */
    /* Δu = Kp*(e(k)-e(k-1)) + Ki*e(k) + Kd*(e(k)-2e(k-1)+e(k-2)) */
    float delta = pid->Kp * (pid->error[0] - pid->error[1])
                + pid->Ki * pid->error[0]
                + pid->Kd * (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);

    pid->output += delta;

    /* 输出限幅 */
    if (pid->output > pid->out_max)  pid->output = pid->out_max;
    if (pid->output < pid->out_min)  pid->output = pid->out_min;

    return pid->output;
}

void PID_SetTarget(PID_t *pid, float target)
{
    pid->target = target;
}

void PID_SetTunings(PID_t *pid, float Kp, float Ki, float Kd)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
}

void PID_Reset(PID_t *pid)
{
    pid->error[0] = 0.0f;
    pid->error[1] = 0.0f;
    pid->error[2] = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}
