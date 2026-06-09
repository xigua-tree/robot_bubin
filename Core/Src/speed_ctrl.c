#include "speed_ctrl.h"
#include "tim.h"

/* 编码器 + PID + 电机 实例 */
Encoder_t g_encoders[SPEED_CTRL_MOTOR_COUNT];
PID_t     s_pids[SPEED_CTRL_MOTOR_COUNT];

/* 控制标志位：TIM8 ISR 每10次中断置1 */
volatile uint8_t g_speed_ctrl_flag = 0;

/* PID 输出低通滤波 */
static float s_out_filtered[SPEED_CTRL_MOTOR_COUNT];
#define OUTPUT_FILTER_ALPHA  0.8f

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

/* 只读取编码器并计算速度*/
#define RPM_FILTER_ALPHA 0.8f   

/* 只读取编码器并计算速度 */
void SpeedCtrl_UpdateEncoders(void)
{
    static int32_t last_count[SPEED_CTRL_MOTOR_COUNT];
    static float   s_filtered[SPEED_CTRL_MOTOR_COUNT];

    for (int i = 0; i < SPEED_CTRL_MOTOR_COUNT; i++) {
        /* 读取硬件编码器，更新累积值 */
        Encoder_Update(&g_encoders[i]);

        /* 差分计算当前周期的脉冲增量 delta */
        int32_t count = Encoder_GetCount(&g_encoders[i]);
        int32_t delta = count - last_count[i];
        last_count[i] = count;

        /* 直接将原始 delta 转换为原始转速 rpm_raw */
        float rpm_raw = (float)delta / (float)(ENCODER_PPR * 4) / SPEED_CTRL_DT * 60.0f;

        /* 一阶低通滤波 */
        s_filtered[i] = s_filtered[i] * (1.0f - RPM_FILTER_ALPHA) + rpm_raw * RPM_FILTER_ALPHA;
        
        /* 输出滤波后的转速*/
        g_encoders[i].speed_rpm = s_filtered[i] * g_encoders[i].invert;
    }
}

/* 编码器读取 + PID 控制 + 输出低通 + PWM 输出 */
void SpeedCtrl_1kHz_Tick(void)
{
    SpeedCtrl_UpdateEncoders();
    chassis_control_task();
    for (int i = 0; i < SPEED_CTRL_MOTOR_COUNT; i++) {
        float output = PID_Update(&s_pids[i], g_encoders[i].speed_rpm);
        /* 一阶低通: y(k) = y(k-1)*(1-α) + x(k)*α */
        s_out_filtered[i] = s_out_filtered[i] * (1.0f - OUTPUT_FILTER_ALPHA)
                          + output * OUTPUT_FILTER_ALPHA;
        Motor_SetDuty(&g_motors[i], (int32_t)s_out_filtered[i]);
    }
}

float motor_rpm;

float map_0_100_to_20_80(float x) 
{
    // 限制输入范围在 0 到 100 之间（防止限幅外的数据导致输出超限）
    if (x < 0.0f)   x = 0.0f;
    if (x > 100.0f) x = 100.0f;
    
    // 0~100 缩放到 0~60，再加上 20 的基准偏移
    return (x * 0.6f) + 21.0f;
}

void SpeedCtrl_1kHz_uptest(void)//开环测试，转速为0-200，占空比为0-16000，
{
    for (int i = 0; i < SPEED_CTRL_MOTOR_COUNT; i++) {
        Motor_SetDuty(&g_motors[i], wheel_rpm[i]*motor_rpm);
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
