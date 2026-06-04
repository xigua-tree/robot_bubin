#include "motor.h"
#include "tim.h"

/* ============ Internal motor context ============ */
typedef struct {
    Motor_ID_t          id;
    TIM_HandleTypeDef  *htim;           /* &htim8 for PWM */
    uint32_t            tim_channel;    /* TIM_CHANNEL_1 ~ TIM_CHANNEL_4 */
    GPIO_TypeDef       *dir1_port;      /* TB6612 AIN1 — direction GPIO */
    uint16_t            dir1_pin;
    GPIO_TypeDef       *dir2_port;      /* TB6612 AIN2 — direction GPIO */
    uint16_t            dir2_pin;
    TIM_HandleTypeDef  *encoder_htim;   /* &htim1 ~ &htim4 */
} Motor_Context_t;

static Motor_Context_t g_motors[MOTOR_MAX];

/* ============ Public API ============ */

void Motor_InitAll(void)
{
    /* --- Motor 1: PE14→IN1, PE15→IN2, TIM8_CH1→PWMA, TIM1 encoder --- */
    g_motors[MOTOR_1].id            = MOTOR_1;
    g_motors[MOTOR_1].htim          = &htim8;
    g_motors[MOTOR_1].tim_channel   = TIM_CHANNEL_1;
    g_motors[MOTOR_1].dir1_port     = GPIOE;
    g_motors[MOTOR_1].dir1_pin      = GPIO_PIN_14;
    g_motors[MOTOR_1].dir2_port     = GPIOE;
    g_motors[MOTOR_1].dir2_pin      = GPIO_PIN_15;
    g_motors[MOTOR_1].encoder_htim  = &htim1;

    /* --- Motor 2: PE0→IN1, PE1→IN2, TIM8_CH3→PWMA, TIM2 encoder --- */
    g_motors[MOTOR_2].id            = MOTOR_2;
    g_motors[MOTOR_2].htim          = &htim8;
    g_motors[MOTOR_2].tim_channel   = TIM_CHANNEL_3;
    g_motors[MOTOR_2].dir1_port     = GPIOE;
    g_motors[MOTOR_2].dir1_pin      = GPIO_PIN_0;
    g_motors[MOTOR_2].dir2_port     = GPIOE;
    g_motors[MOTOR_2].dir2_pin      = GPIO_PIN_1;
    g_motors[MOTOR_2].encoder_htim  = &htim2;

    /* --- Motor 3: PE12→IN1, PE13→IN2, TIM8_CH2→PWMA, TIM3 encoder --- */
    g_motors[MOTOR_3].id            = MOTOR_3;
    g_motors[MOTOR_3].htim          = &htim8;
    g_motors[MOTOR_3].tim_channel   = TIM_CHANNEL_2;
    g_motors[MOTOR_3].dir1_port     = GPIOE;
    g_motors[MOTOR_3].dir1_pin      = GPIO_PIN_12;
    g_motors[MOTOR_3].dir2_port     = GPIOE;
    g_motors[MOTOR_3].dir2_pin      = GPIO_PIN_13;
    g_motors[MOTOR_3].encoder_htim  = &htim3;

    /* --- Motor 4: PB8→IN1, PB9→IN2, TIM8_CH4→PWMA, TIM4 encoder --- */
    g_motors[MOTOR_4].id            = MOTOR_4;
    g_motors[MOTOR_4].htim          = &htim8;
    g_motors[MOTOR_4].tim_channel   = TIM_CHANNEL_4;
    g_motors[MOTOR_4].dir1_port     = GPIOB;
    g_motors[MOTOR_4].dir1_pin      = GPIO_PIN_8;
    g_motors[MOTOR_4].dir2_port     = GPIOB;
    g_motors[MOTOR_4].dir2_pin      = GPIO_PIN_9;
    g_motors[MOTOR_4].encoder_htim  = &htim4;

    /* Start PWM on all 4 TIM8 channels */
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);

    /* Enable TIM8 update interrupt (UIE).
     * HAL_TIM_PWM_Start only starts the counter; it does NOT set UIE.
     * Without UIE the timer won't request interrupts, so SpeedCtrlTask
     * would block forever on ulTaskNotifyTake. */
    __HAL_TIM_ENABLE_IT(&htim8, TIM_IT_UPDATE);

    /* Start encoder counters (Init only configures, Start begins counting) */
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    /* All motors stopped */
    Motor_StopAll();
}

void Motor_DeInitAll(void)
{
    Motor_StopAll();
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_4);
    HAL_TIM_Encoder_Stop(&htim1, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Stop(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Stop(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Stop(&htim4, TIM_CHANNEL_ALL);
}

int Motor_SetDuty(Motor_ID_t id, int8_t duty)
{
    if (id >= MOTOR_MAX) {
        return -1;
    }

    /* Clamp to valid range */
    if (duty > 100)  duty = 100;
    if (duty < -100) duty = -100;

    Motor_Context_t *m   = &g_motors[id];
    uint32_t         arr = m->htim->Init.Period;   /* 16799 */
    uint32_t         pulse;

    if (duty > 0) {
        /* Forward: IN1=H, IN2=L, PWMA=PWM */
        HAL_GPIO_WritePin(m->dir1_port, m->dir1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(m->dir2_port, m->dir2_pin, GPIO_PIN_RESET);
        pulse = (arr + 1) * (uint32_t)duty / 100;
    } else if (duty < 0) {
        /* Reverse: IN1=L, IN2=H, PWMA=PWM */
        HAL_GPIO_WritePin(m->dir1_port, m->dir1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(m->dir2_port, m->dir2_pin, GPIO_PIN_SET);
        pulse = (arr + 1) * (uint32_t)(-duty) / 100;
    } else {
        /* Coast stop: IN1=L, IN2=L, PWMA=0 */
        HAL_GPIO_WritePin(m->dir1_port, m->dir1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(m->dir2_port, m->dir2_pin, GPIO_PIN_RESET);
        pulse = 0;
    }

    __HAL_TIM_SET_COMPARE(m->htim, m->tim_channel, pulse);
    return 0;
}

void Motor_Stop(Motor_ID_t id)
{
    Motor_SetDuty(id, 0);
}

void Motor_StopAll(void)
{
    for (int i = MOTOR_1; i < MOTOR_MAX; i++) {
        Motor_Stop((Motor_ID_t)i);
    }
}

int32_t Motor_GetEncoder(Motor_ID_t id)
{
    if (id >= MOTOR_MAX) {
        return 0;
    }
    return (int32_t)__HAL_TIM_GET_COUNTER(g_motors[id].encoder_htim);
}

void Motor_ResetEncoder(Motor_ID_t id)
{
    if (id >= MOTOR_MAX) {
        return;
    }
    __HAL_TIM_SET_COUNTER(g_motors[id].encoder_htim, 0);
}
