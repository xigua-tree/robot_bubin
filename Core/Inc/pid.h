#ifndef __PID_H
#define __PID_H

#include <stdint.h>

typedef struct {
    float Kp, Ki, Kd;
    float target;          /* 目标速度 */
    float error[3];        /* e(k), e(k-1), e(k-2) */
    float output;          /* 当前输出值 */
    float out_max;         /* 输出上限 */
    float out_min;         /* 输出下限 */
    float integral;        /* 积分累积（带限幅） */
    float integral_max;
} PID_t;

void PID_Init(PID_t *pid, float Kp, float Ki, float Kd, float out_max);
float PID_Update(PID_t *pid, float measured);             /* 返回控制输出 */
void PID_SetTarget(PID_t *pid, float target);
void PID_SetTunings(PID_t *pid, float Kp, float Ki, float Kd);
void PID_Reset(PID_t *pid);
float PID_Update_Angle(PID_t *pid, float measured);

#endif
