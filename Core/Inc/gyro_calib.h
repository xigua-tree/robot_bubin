/**
 * @file    gyro_calib.h
 * @brief   陀螺仪零偏在线校准模块
 * @note    通过静止检测 + 指数移动平均，自动估计并补偿陀螺仪零偏，
 *           解决 6-DOF IMU 中 yaw 轴无法观测导致的漂移问题。
 */

#ifndef __GYRO_CALIB_H
#define __GYRO_CALIB_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief  初始化零偏校准
 * @note   采集指定数量的样本，假设设备初始静止，计算三轴陀螺仪均值作为初始零偏。
 *         应在传感器初始化完成后、进入主循环前调用。
 * @param  samples : 采样数量（如 500，约 2.5 秒 @ 200Hz）
 */
void GyroCalib_Init(int samples);

/**
 * @brief  在线偏置更新与补偿
 * @note   每次读取到新的 IMU 数据时调用。内部进行静止检测：
 *           - 三轴陀螺仪角速度均低于阈值
 *           - 加速度计模长接近 1g
 *         当连续多帧满足静止条件时，使用 EMA 更新零偏估计。
 *         无论是否静止，gyro 读数都会被就地补偿（减去当前零偏估计）。
 * @param  gx/gy/gz : 陀螺仪读数指针 (rad/s)，会就地修改为偏置补偿后的值
 * @param  ax/ay/az : 加速度计读数 (g)，只用于静止检测，不会被修改
 */
void GyroCalib_Update(float *gx, float *gy, float *gz,
                      float ax, float ay, float az);

/**
 * @brief  获取当前零偏估计值
 * @param  bx/by/bz : 三轴零偏输出 (rad/s)
 */
void GyroCalib_GetBias(float *bx, float *by, float *bz);

#ifdef __cplusplus
}
#endif

#endif /* __GYRO_CALIB_H */
