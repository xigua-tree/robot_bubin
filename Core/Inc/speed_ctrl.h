#ifndef __SPEED_CTRL_H
#define __SPEED_CTRL_H

#include <stdint.h>
#include <stdbool.h>

#include "encoder.h"
#include "motor.h"
#include "pid.h"

#define SPEED_CTRL_MOTOR_COUNT 4

/* 控制参数 */
#define SPEED_CTRL_FREQ_HZ     1000.0f   /* 1kHz 控制频率 */
#define SPEED_CTRL_DT          0.001f    /* 1ms */

/* 默认 PID 参数 */
#define DEFAULT_KP  0.5f
#define DEFAULT_KI  0.1f
#define DEFAULT_KD  0.01f

/* 控制标志位（TIM8 ISR 置1，主循环清零） */
extern volatile uint8_t g_speed_ctrl_flag;

void SpeedCtrl_Init(void);
void SpeedCtrl_1kHz_Tick(void);          /* 执行一次控制计算 */
void SpeedCtrl_SetTarget(uint8_t id, float rpm);
void SpeedCtrl_SetPID(uint8_t id, float Kp, float Ki, float Kd);
float SpeedCtrl_GetSpeed(uint8_t id);    /* 获取实际速度 */

#endif
