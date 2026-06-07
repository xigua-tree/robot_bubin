/**
 * @file    gyro_calib.c
 * @brief   陀螺仪零偏在线校准模块实现
 * @note    通过静止检测 + EMA 在线估计陀螺仪三轴零偏，动态补偿后传给 AHRS 滤波器。
 *          解决 6-DOF IMU（无磁力计）中 yaw 轴不可观测导致的漂移问题。
 */

#include "gyro_calib.h"
#include "main.h"       /* HAL_Delay, MPU6050 类型 */
#include "MPU6050.h"    /* MPU6050_GetData */
#include "i2c.h"        /* I2C 句柄 */
#include <math.h>       /* fabsf, sqrtf */

/* ==========================================================================
   参数配置
   ========================================================================== */

/* 静止检测阈值 */
#define GYRO_STATIC_THRESH     0.05f   /* 陀螺仪静止阈值 (rad/s, ≈ 2.9°/s) */
#define ACCEL_STATIC_THRESH    0.15f   /* 加速度计静止阈值 (g)              */
#define STATIONARY_CONFIRM     20      /* 连续静止帧数确认（20帧 @200Hz = 100ms） */

/* EMA 偏置更新系数 */
#define BIAS_EMA_ALPHA         0.0005f /* 偏置 EMA 系数，极慢更新确保只跟踪温漂 */

/* 陀螺仪刻度因子（与 MPU6050 配置一致：±2000°/s） */
#define GYRO_SCALE  ((2000.0f / 32768.0f) * (3.14159265359f / 180.0f))

/* ==========================================================================
   内部变量
   ========================================================================== */

static float g_bias_x = 0.0f;  /* X 轴陀螺仪零偏 (rad/s) */
static float g_bias_y = 0.0f;  /* Y 轴陀螺仪零偏 (rad/s) */
static float g_bias_z = 0.0f;  /* Z 轴陀螺仪零偏 (rad/s) */

static uint32_t g_stationary_cnt = 0;  /* 连续静止帧计数器 */

/* ==========================================================================
   初始化校准
   ========================================================================== */

/**
 * @brief  初始化零偏校准
 * @note   假设设备初始静止，采集 samples 个样本，取均值作为初始零偏。
 *         调用此函数前需确保 I2C 和 MPU6050 已初始化完毕。
 */
void GyroCalib_Init(int samples)
{
    if (samples <= 0) return;

    double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
    int16_t acc_raw, gyro_raw;

    for (int i = 0; i < samples; i++)
    {
        int16_t gx_r, gy_r, gz_r;
        MPU6050_GetData(&acc_raw, &acc_raw, &acc_raw,
                        &gx_r, &gy_r, &gz_r);

        sum_x += (double)gx_r;
        sum_y += (double)gy_r;
        sum_z += (double)gz_r;

        HAL_Delay(5);  /* 匹配 200Hz 采样间隔 */
    }

    /* 原始值均值 × 刻度因子 = 物理零偏 (rad/s) */
    g_bias_x = (float)(sum_x / samples) * GYRO_SCALE;
    g_bias_y = (float)(sum_y / samples) * GYRO_SCALE;
    g_bias_z = (float)(sum_z / samples) * GYRO_SCALE;

    g_stationary_cnt = 0;
}

/* ==========================================================================
   在线偏置更新与补偿
   ========================================================================== */

/**
 * @brief  在线偏置更新与补偿
 * @note   核心逻辑：
 *           1. 静止检测：陀螺仪三轴 < 阈值 且 加速度模长 ≈ 1g
 *           2. 连续 stationary 帧后，用 EMA 更新偏置估计
 *           3. 无论是否静止，都从 gyro 读数中减去当前偏置
 */
void GyroCalib_Update(float *gx, float *gy, float *gz,
                      float ax, float ay, float az)
{
    /* ---- 静止检测 ---- */
    int is_stationary = 0;

    /* 条件 1：陀螺仪三轴角速度均低于阈值（无旋转） */
    if (fabsf(*gx) < GYRO_STATIC_THRESH &&
        fabsf(*gy) < GYRO_STATIC_THRESH &&
        fabsf(*gz) < GYRO_STATIC_THRESH)
    {
        /* 条件 2：加速度计模长接近 1g（无线性加速度） */
        float acc_norm = sqrtf(ax * ax + ay * ay + az * az);
        if (fabsf(acc_norm - 1.0f) < ACCEL_STATIC_THRESH)
        {
            is_stationary = 1;
        }
    }

    /* ---- 连续静止帧计数器 ---- */
    if (is_stationary)
    {
        if (g_stationary_cnt < STATIONARY_CONFIRM)
        {
            g_stationary_cnt++;
        }
    }
    else
    {
        g_stationary_cnt = 0;
    }

    /* ---- 静止确认后，EMA 更新偏置 ---- */
    if (g_stationary_cnt >= STATIONARY_CONFIRM)
    {
        g_bias_x += BIAS_EMA_ALPHA * (*gx - g_bias_x);
        g_bias_y += BIAS_EMA_ALPHA * (*gy - g_bias_y);
        g_bias_z += BIAS_EMA_ALPHA * (*gz - g_bias_z);
    }

    /* ---- 补偿：始终从读数中减去当前偏置估计 ---- */
    *gx -= g_bias_x;
    *gy -= g_bias_y;
    *gz -= g_bias_z;
}

/* ==========================================================================
   获取当前偏置
   ========================================================================== */

/**
 * @brief  获取当前零偏估计值
 */
void GyroCalib_GetBias(float *bx, float *by, float *bz)
{
    *bx = g_bias_x;
    *by = g_bias_y;
    *bz = g_bias_z;
}
