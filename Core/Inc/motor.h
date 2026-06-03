#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "main.h"

typedef enum {
    MOTOR_1 = 0,
    MOTOR_2 = 1,
    MOTOR_3 = 2,
    MOTOR_4 = 3,
    MOTOR_MAX = 4
} Motor_ID_t;

/* Initialization */
void Motor_InitAll(void);
void Motor_DeInitAll(void);

/* Core control: duty range [-100, 100], returns 0=OK, -1=bad ID */
int  Motor_SetDuty(Motor_ID_t id, int8_t duty);
void Motor_Stop(Motor_ID_t id);
void Motor_StopAll(void);

/* Encoder */
int32_t Motor_GetEncoder(Motor_ID_t id);
void    Motor_ResetEncoder(Motor_ID_t id);

#endif /* __MOTOR_H__ */
