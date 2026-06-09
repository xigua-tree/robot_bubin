#ifndef __MOTOR_H
#define __MOTOR_H

#include <stdint.h>
#include "main.h"

/* TB6612 方向/刹车宏 */
#define MOTOR_CW   1
#define MOTOR_CCW -1
#define MOTOR_STOP 0

/* 最大 PWM 占空比（ARR = 16799） */
#define MOTOR_PWM_MAX 16000

typedef struct {
    /* 方向引脚 */
    GPIO_TypeDef *in1_port;
    uint16_t      in1_pin;
    GPIO_TypeDef *in2_port;
    uint16_t      in2_pin;
    /* PWM（TIM8 各通道） */
    TIM_HandleTypeDef *htim;
    uint32_t           channel;
    int8_t             invert;   /* 方向反转: 1=正常, -1=反向 */
} Motor_t;

extern Motor_t g_motors[4];

void Motor_InitAll(void);
/* duty: -1.0f ~ +1.0f，正=正转，负=反转 */
void Motor_SetDuty(Motor_t *m, int32_t duty);
void Motor_Stop(Motor_t *m);
void Motor_Brake(Motor_t *m);

#endif
