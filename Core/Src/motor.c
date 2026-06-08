#include "motor.h"
#include "tim.h"
#include <stdlib.h>

/* 电机1: PE14,PE15 + TIM8_CH1(PC6) */
/* 电机2: PE0,PE1   + TIM8_CH3(PC8) */
/* 电机3: PE12,PE13 + TIM8_CH2(PC7) */
/* 电机4: PB8,PB9   + TIM8_CH4(PC9) */
Motor_t g_motors[4];

void Motor_InitAll(void)
{
    /* 电机1 */
    g_motors[0].in1_port = GPIOE;
    g_motors[0].in1_pin  = GPIO_PIN_12;
    g_motors[0].in2_port = GPIOE;
    g_motors[0].in2_pin  = GPIO_PIN_13;
    g_motors[0].htim     = &htim8;
    g_motors[0].channel  = TIM_CHANNEL_2;
    g_motors[0].invert   = 1;

    /* 电机2 */
    g_motors[1].in1_port = GPIOE;
    g_motors[1].in1_pin  = GPIO_PIN_14;
    g_motors[1].in2_port = GPIOE;
    g_motors[1].in2_pin  = GPIO_PIN_15;
    g_motors[1].htim     = &htim8;
    g_motors[1].channel  = TIM_CHANNEL_1;
    g_motors[1].invert   = 1;

    /* 电机3 */
    g_motors[2].in1_port = GPIOE;
    g_motors[2].in1_pin  = GPIO_PIN_1;
    g_motors[2].in2_port = GPIOE;
    g_motors[2].in2_pin  = GPIO_PIN_0;
    g_motors[2].htim     = &htim8;
    g_motors[2].channel  = TIM_CHANNEL_3;
    g_motors[2].invert   = -1;

    /* 电机4 */
    g_motors[3].in1_port = GPIOB;
    g_motors[3].in1_pin  = GPIO_PIN_8;
    g_motors[3].in2_port = GPIOB;
    g_motors[3].in2_pin  = GPIO_PIN_9;
    g_motors[3].htim     = &htim8;
    g_motors[3].channel  = TIM_CHANNEL_4;
    g_motors[3].invert   = 1;

    /* 启动所有 PWM 通道 */
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);

    /* 初始停止 */
    for (int i = 0; i < 4; i++) {
        Motor_Stop(&g_motors[i]);
    }
}

void Motor_SetDuty(Motor_t *m, int32_t duty)
{
    /* 方向反转（软件层面，保持编码器极性一致） */
    duty *= m->invert;

    /* 限幅到 PWM 最大值 */
    if (duty > MOTOR_PWM_MAX)  duty = MOTOR_PWM_MAX;
    if (duty < -MOTOR_PWM_MAX) duty = -MOTOR_PWM_MAX;

    uint16_t pulse;

    if (duty > 0) {
        /* 正转: IN1=H, IN2=L */
        HAL_GPIO_WritePin(m->in1_port, m->in1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(m->in2_port, m->in2_pin, GPIO_PIN_RESET);
        pulse = (uint16_t)duty;
    } else if (duty < 0) {
        /* 反转: IN1=L, IN2=H */
        HAL_GPIO_WritePin(m->in1_port, m->in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(m->in2_port, m->in2_pin, GPIO_PIN_SET);
        pulse = (uint16_t)(-duty);
    } else {
        /* 停止: IN1=L, IN2=L */
        HAL_GPIO_WritePin(m->in1_port, m->in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(m->in2_port, m->in2_pin, GPIO_PIN_RESET);
        pulse = 0;
    }

    __HAL_TIM_SET_COMPARE(m->htim, m->channel, pulse);
}

void Motor_Stop(Motor_t *m)
{
    Motor_SetDuty(m, 0);
}

void Motor_Brake(Motor_t *m)
{
    /* TB6612: IN1=H, IN2=H = brake */
    HAL_GPIO_WritePin(m->in1_port, m->in1_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(m->in2_port, m->in2_pin, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(m->htim, m->channel, 0);
}

/* htim8 声明来自 tim.h */
