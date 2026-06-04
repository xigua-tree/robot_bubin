#ifndef __ENCODER_H
#define __ENCODER_H

#include <stdint.h>
#include "main.h"

/* 编码器线数（电机旋转一圈的脉冲数，4倍频前） */
#define ENCODER_PPR 11

typedef struct {
    TIM_HandleTypeDef *htim;
    int32_t  accum;       /* 32位累计值，处理16位溢出 */
    int16_t  last_raw;    /* 上一次原始值 */
    float    speed_rpm;   /* 当前速度 RPM */
} Encoder_t;

void Encoder_Init(Encoder_t *enc, TIM_HandleTypeDef *htim);
void Encoder_Update(Encoder_t *enc);       /* 调用此函数计算速度（需周期性调用） */
int32_t Encoder_GetCount(Encoder_t *enc);
float Encoder_GetSpeed(Encoder_t *enc);

#endif
