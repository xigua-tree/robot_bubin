#include "speed_ctrl.h"
#include "tim.h"

/* 编码器 + PID + 电机 实例 */
Encoder_t g_encoders[SPEED_CTRL_MOTOR_COUNT];
static PID_t     s_pids[SPEED_CTRL_MOTOR_COUNT];

/* 控制标志位：TIM8 ISR 每10次中断置1 */
volatile uint8_t g_speed_ctrl_flag = 0;

/* PID 输出低通滤波 */
static float s_out_filtered[SPEED_CTRL_MOTOR_COUNT];
#define OUTPUT_FILTER_ALPHA  0.5f

/* TIM 句柄来自 tim.h */

void SpeedCtrl_Init(void)
{
    /* 初始化编码器 */
    Encoder_Init(&g_encoders[0], &htim3);
    Encoder_Init(&g_encoders[1], &htim1);
    Encoder_Init(&g_encoders[2], &htim2);
    Encoder_Init(&g_encoders[3], &htim4);

    /* 电机3 编码器反向（与 Motor_t.invert 配对，保持 PID 负反馈） */
    g_encoders[2].invert = -1;

    /* 初始化 PID */
    for (int i = 0; i < SPEED_CTRL_MOTOR_COUNT; i++) {
        PID_Init(&s_pids[i], DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, (float)MOTOR_PWM_MAX);
        PID_SetTarget(&s_pids[i], 0.0f);
        s_out_filtered[i] = 0.0f;
    }

}

#define DELTA_AVG_SIZE  4   /* 编码器 delta 滑动平均窗口 */

/* 只读取编码器并计算速度，不动电机（供开环测试用） */
void SpeedCtrl_UpdateEncoders(void)
{
    static int32_t last_count[SPEED_CTRL_MOTOR_COUNT];
    static float   s_filtered[SPEED_CTRL_MOTOR_COUNT];
    static int32_t delta_buf[SPEED_CTRL_MOTOR_COUNT][DELTA_AVG_SIZE];
    static int32_t delta_sum[SPEED_CTRL_MOTOR_COUNT];
    static uint8_t delta_idx[SPEED_CTRL_MOTOR_COUNT];
    static bool    delta_ready[SPEED_CTRL_MOTOR_COUNT];
    #define RPM_FILTER_ALPHA 0.3f

    for (int i = 0; i < SPEED_CTRL_MOTOR_COUNT; i++) {
        /* 读取硬件编码器，更新累积值 */
        Encoder_Update(&g_encoders[i]);

        /* 差分计算 delta */
        int32_t count = Encoder_GetCount(&g_encoders[i]);
        int32_t delta = count - last_count[i];
        last_count[i] = count;

        /* 4 点滑动平均：用新 delta 替换最旧的，更新环 */
        delta_sum[i] -= delta_buf[i][delta_idx[i]];
        delta_buf[i][delta_idx[i]] = delta;
        delta_sum[i] += delta;
        delta_idx[i]++;
        if (delta_idx[i] >= DELTA_AVG_SIZE) {
            delta_idx[i]  = 0;
            delta_ready[i] = true;          /* 窗口填满后才使用平均值 */
        }

        float delta_avg;
        if (delta_ready[i]) {
            delta_avg = (float)delta_sum[i] / DELTA_AVG_SIZE;
        } else {
            /* 窗口未满时，用已填充样本的均值 */
            uint8_t n = delta_idx[i] > 0 ? delta_idx[i] : 1;
            delta_avg = (float)delta_sum[i] / n;
        }

        float rpm_raw = delta_avg / (float)(ENCODER_PPR * 4) / SPEED_CTRL_DT * 60.0f;

        /* 一阶低通滤波 */
        s_filtered[i] = s_filtered[i] * (1.0f - RPM_FILTER_ALPHA) + rpm_raw * RPM_FILTER_ALPHA;
        g_encoders[i].speed_rpm = s_filtered[i] * g_encoders[i].invert;
    }
}

/* 编码器读取 + PID 控制 + 输出低通 + PWM 输出 */
void SpeedCtrl_1kHz_Tick(void)
{
    SpeedCtrl_UpdateEncoders();

    for (int i = 0; i < SPEED_CTRL_MOTOR_COUNT; i++) {
        float output = PID_Update(&s_pids[i], g_encoders[i].speed_rpm);
        /* 一阶低通: y(k) = y(k-1)*(1-α) + x(k)*α */
        s_out_filtered[i] = s_out_filtered[i] * (1.0f - OUTPUT_FILTER_ALPHA)
                          + output * OUTPUT_FILTER_ALPHA;
        Motor_SetDuty(&g_motors[i], (int32_t)s_out_filtered[i]);
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
