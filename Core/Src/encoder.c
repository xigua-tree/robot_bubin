#include "encoder.h"

/* 编码器 4 倍频：每个脉冲产生 4 个计数边沿 */
#define ENCODER_CPR (ENCODER_PPR * 4)

void Encoder_Init(Encoder_t *enc, TIM_HandleTypeDef *htim)
{
    enc->htim = htim;
    enc->accum = 0;
    enc->last_raw = 0;
    enc->speed_rpm = 0.0f;
    enc->invert = 1;  /* 默认不反转 */

    /* 启动编码器模式 */
    HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);
}

void Encoder_Update(Encoder_t *enc)
{
    uint16_t raw = (uint16_t)__HAL_TIM_GET_COUNTER(enc->htim);

    int16_t delta = (int16_t)(raw - enc->last_raw);
    
    enc->accum += delta;
    
    enc->last_raw = raw;
}

int32_t Encoder_GetCount(Encoder_t *enc)
{
    return enc->accum;
}

float Encoder_GetSpeed(Encoder_t *enc)
{
    return enc->speed_rpm;
}
