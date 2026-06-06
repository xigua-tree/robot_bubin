#ifndef __SPEED_CTRL_H
#define __SPEED_CTRL_H

#include <stdint.h>
#include <stdbool.h>

#include "encoder.h"
#include "motor.h"
#include "pid.h"
#include "main.h"

#define SPEED_CTRL_MOTOR_COUNT 4

/* 控制参数 */
#define SPEED_CTRL_FREQ_HZ     1000.0f   /* 1kHz 控制频率 */
#define SPEED_CTRL_DT          0.05f    /* 1ms */

/* 默认 PID 参数 */
#define DEFAULT_KP  0.0f
#define DEFAULT_KI  0.0f
#define DEFAULT_KD  0.00f

/* 控制标志位（TIM8 ISR 置1，主循环清零） */
extern volatile uint8_t g_speed_ctrl_flag;
extern Encoder_t g_encoders[SPEED_CTRL_MOTOR_COUNT];

void SpeedCtrl_Init(void);
void SpeedCtrl_UpdateEncoders(void);     /* 只读编码器+算速度，不动电机 */
void SpeedCtrl_1kHz_Tick(void);          /* 编码器+PID+PWM 完整闭环 */
void SpeedCtrl_SetTarget(uint8_t id, float rpm);
void SpeedCtrl_SetPID(uint8_t id, float Kp, float Ki, float Kd);
float SpeedCtrl_GetSpeed(uint8_t id);    /* 获取实际速度（滤波后） */
float SpeedCtrl_GetError(uint8_t id);    /* 获取速度误差 (target - actual) */

#endif
