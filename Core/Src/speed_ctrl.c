#include "speed_ctrl.h"
#include "tim.h"

/* 编码器 + PID + 电机 实例 */
Encoder_t g_encoders[SPEED_CTRL_MOTOR_COUNT];
static PID_t     s_pids[SPEED_CTRL_MOTOR_COUNT];

/* 控制标志位：TIM8 ISR 每10次中断置1 */
volatile uint8_t g_speed_ctrl_flag = 0;

/* TIM 句柄来自 tim.h */

void SpeedCtrl_Init(void)
{
    /* 初始化编码器 */
    Encoder_Init(&g_encoders[0], &htim1);
    Encoder_Init(&g_encoders[1], &htim2);
    Encoder_Init(&g_encoders[2], &htim3);
    Encoder_Init(&g_encoders[3], &htim4);

    /* 初始化 PID */
    for (int i = 0; i < SPEED_CTRL_MOTOR_COUNT; i++) {
        PID_Init(&s_pids[i], DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, (float)MOTOR_PWM_MAX);
        PID_SetTarget(&s_pids[i], 0.0f);
    }
}

/* 只读取编码器并计算速度，不动电机（供开环测试用） */
void SpeedCtrl_UpdateEncoders(void)
{
    static int32_t last_count[SPEED_CTRL_MOTOR_COUNT];
    static float   s_filtered[SPEED_CTRL_MOTOR_COUNT];  /* 滤波后的 RPM */
    /* 滤波系数：越小越平滑但响应越慢，0.15~0.3 适合速度环 */
    #define RPM_FILTER_ALPHA 0.3f

    for (int i = 0; i < SPEED_CTRL_MOTOR_COUNT; i++) {
        /* 读取硬件编码器，更新累积值 */
        Encoder_Update(&g_encoders[i]);

        /* 差分计算瞬时转速 RPM */
        int32_t count = Encoder_GetCount(&g_encoders[i]);
        int32_t delta = count - last_count[i];
        last_count[i] = count;

        float rpm_raw = (float)delta / (float)(ENCODER_PPR * 4) / SPEED_CTRL_DT * 60.0f;

        /* 一阶低通滤波：filtered = filtered*(1-α) + raw*α */
        s_filtered[i] = s_filtered[i] * (1.0f - RPM_FILTER_ALPHA) + rpm_raw * RPM_FILTER_ALPHA;
        g_encoders[i].speed_rpm = s_filtered[i];
    }
}

/* 编码器读取 + PID 控制 + PWM 输出（完整的闭环控制） */
void SpeedCtrl_1kHz_Tick(void)
{
    SpeedCtrl_UpdateEncoders();

    for (int i = 0; i < SPEED_CTRL_MOTOR_COUNT; i++) {
        float output = PID_Update(&s_pids[i], g_encoders[i].speed_rpm);
        Motor_SetDuty(&g_motors[i], (int32_t)output);
    }
}

void SpeedCtrl_SetTarget(uint8_t id, float rpm)
{
    if (id < SPEED_CTRL_MOTOR_COUNT) {
        if (rpm >  340.0f) rpm =  340.0f;
        if (rpm < -340.0f) rpm = -340.0f;
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
        return Encoder_GetSpeed(&g_encoders[id]);
    }
    return 0.0f;
}

float SpeedCtrl_GetError(uint8_t id)
{
    if (id < SPEED_CTRL_MOTOR_COUNT) {
        return s_pids[id].target - g_encoders[id].speed_rpm;
    }
    return 0.0f;
}
