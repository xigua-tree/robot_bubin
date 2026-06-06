#ifndef __MPU6050_H
#define __MPU6050_H

#include <stdint.h>

/* MPU6050 I2C 从机地址 (7位地址0x68左移1位 = 0xD0) */
#define MPU6050_ADDRESS         0xD0

/* MPU6050 寄存器定义 */
#define MPU6050_SELF_TEST_X     0x0D    /* 自检寄存器X */
#define MPU6050_SELF_TEST_Y     0x0E    /* 自检寄存器Y */
#define MPU6050_SELF_TEST_Z     0x0F    /* 自检寄存器Z */
#define MPU6050_SELF_TEST_A     0x10    /* 自检寄存器A */
#define MPU6050_SMPLRT_DIV      0x19    /* 采样率分频寄存器 */
#define MPU6050_CONFIG          0x1A    /* 配置寄存器 */
#define MPU6050_GYRO_CONFIG     0x1B    /* 陀螺仪配置寄存器 */
#define MPU6050_ACCEL_CONFIG    0x1C    /* 加速度计配置寄存器 */
#define MPU6050_FIFO_EN         0x23    /* FIFO使能寄存器 */
#define MPU6050_INT_PIN_CFG     0x37    /* 中断引脚配置寄存器 */
#define MPU6050_INT_ENABLE      0x38    /* 中断使能寄存器 */
#define MPU6050_INT_STATUS      0x3A    /* 中断状态寄存器 */
#define MPU6050_ACCEL_XOUT_H    0x3B    /* 加速度计X轴高8位 */
#define MPU6050_ACCEL_XOUT_L    0x3C    /* 加速度计X轴低8位 */
#define MPU6050_ACCEL_YOUT_H    0x3D    /* 加速度计Y轴高8位 */
#define MPU6050_ACCEL_YOUT_L    0x3E    /* 加速度计Y轴低8位 */
#define MPU6050_ACCEL_ZOUT_H    0x3F    /* 加速度计Z轴高8位 */
#define MPU6050_ACCEL_ZOUT_L    0x40    /* 加速度计Z轴低8位 */
#define MPU6050_TEMP_OUT_H      0x41    /* 温度高8位 */
#define MPU6050_TEMP_OUT_L      0x42    /* 温度低8位 */
#define MPU6050_GYRO_XOUT_H     0x43    /* 陀螺仪X轴高8位 */
#define MPU6050_GYRO_XOUT_L     0x44    /* 陀螺仪X轴低8位 */
#define MPU6050_GYRO_YOUT_H     0x45    /* 陀螺仪Y轴高8位 */
#define MPU6050_GYRO_YOUT_L     0x46    /* 陀螺仪Y轴低8位 */
#define MPU6050_GYRO_ZOUT_H     0x47    /* 陀螺仪Z轴高8位 */
#define MPU6050_GYRO_ZOUT_L     0x48    /* 陀螺仪Z轴低8位 */
#define MPU6050_PWR_MGMT_1      0x6B    /* 电源管理寄存器1 */
#define MPU6050_PWR_MGMT_2      0x6C    /* 电源管理寄存器2 */
#define MPU6050_WHO_AM_I        0x75    /* WHO_AM_I寄存器 */

/* 函数声明 */
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data);
uint8_t MPU6050_ReadReg(uint8_t RegAddress);

void MPU6050_Init(void);
uint8_t MPU6050_GetID(void);
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                     int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ);

#endif
