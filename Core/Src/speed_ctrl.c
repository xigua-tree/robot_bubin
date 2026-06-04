#include "speed_ctrl.h"
#include "tim.h"

/* 编码器 + PID + 电机 实例 */
static Encoder_t s_encoders[SPEED_CTRL_MOTOR_COUNT];
static PID_t     s_pids[SPEED_CTRL_MOTOR_COUNT];

/* 控制标志位：TIM8 ISR 每10次中断置1 */
volatile uint8_t g_speed_ctrl_flag = 0;

/* TIM 句柄来自 tim.h */

void SpeedCtrl_Init(void)
{
    /* 初始化编码器 */
    Encoder_Init(&s_encoders[0], &htim1);
    Encoder_Init(&s_encoders[1], &htim2);
    Encoder_Init(&s_encoders[2], &htim3);
    Encoder_Init(&s_encoders[3], &htim4);

    /* 初始化 PID */
    for (int i = 0; i < SPEED_CTRL_MOTOR_COUNT; i++) {
        PID_Init(&s_pids[i], DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, 1.0f);
        PID_SetTarget(&s_pids[i], 0.0f);
    }
}

void SpeedCtrl_1kHz_Tick(void)
{
    for (int i = 0; i < SPEED_CTRL_MOTOR_COUNT; i++) {
        /* 1. 更新编码器计数和速度 */
        Encoder_Update(&s_encoders[i]);

        /* 2. 计算速度 (RPM) */
        /* speed = delta_count / CPR / dt * 60 */
        /* 此处假设 Encoder_Update 被 1kHz 调用 */
        int32_t count = Encoder_GetCount(&s_encoders[i]);
        /* 速度计算由 Encoder_Update 内部累积，此处用差分法 */
        static int32_t last_count[SPEED_CTRL_MOTOR_COUNT];
        int32_t delta = count - last_count[i];
        last_count[i] = count;

        /* RPM = (delta / CPR) / dt * 60 */
        /* dt = 0.001s, CPR = ENCODER_PPR * 4 */
        float rpm = (float)delta / (float)(ENCODER_PPR * 4) / SPEED_CTRL_DT * 60.0f;

        /* 保存速度到 encoder 结构体 */
        s_encoders[i].speed_rpm = rpm;

        /* 3. PID 控制 */
        float output = PID_Update(&s_pids[i], rpm);

        /* 4. 更新 PWM */
        Motor_SetDuty(&g_motors[i], output);
    }
}

void SpeedCtrl_SetTarget(uint8_t id, float rpm)
{
    if (id < SPEED_CTRL_MOTOR_COUNT) {
        PID_SetTarget(&s_pids[id], rpm);
    }
}

void SpeedCtrl_SetPID(uint8_t id, float Kp, float Ki, float Kd)
{
    if (id < SPEED_CTRL_MOTOR_COUNT) {
        PID_SetTunings(&s_pids[id], Kp, Ki, Kd);
    }
}

float SpeedCtrl_GetSpeed(uint8_t id)
{
    if (id < SPEED_CTRL_MOTOR_COUNT) {
        return Encoder_GetSpeed(&s_encoders[id]);
    }
    return 0.0f;
}
