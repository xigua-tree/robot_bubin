#ifndef __SPEED_CTRL_H__
#define __SPEED_CTRL_H__

#include "motor.h"
#include "pid.h"
#include <stdint.h>

/* Control mode */
#define MODE_DUTY   0
#define MODE_SPEED  1

/* Default encoder CPR (Counts Per Revolution): measured 1338 per output revolution */
#define DEFAULT_CPR  1300

typedef struct {
    Motor_ID_t   motor_id;
    PID_t        pid;
    float        target_rpm;       /* desired RPM */
    float        current_rpm;      /* measured RPM */
    uint16_t     cpr;              /* encoder CPR */
    int32_t      last_encoder;     /* previous encoder reading */
    uint8_t      mode;             /* MODE_DUTY or MODE_SPEED */
} SpeedCtrl_t;

/* Global array (defined in speed_ctrl.c) */
extern SpeedCtrl_t g_speed_ctrl[4];

/* API called from serial_cmd.c */
void SpeedCtrl_Init(void);
void SpeedCtrl_SetMode(Motor_ID_t id, uint8_t mode);
void SpeedCtrl_SetTargetRPM(Motor_ID_t id, float rpm);       /* rpm ×10 via proto */
void SpeedCtrl_SetPID(Motor_ID_t id, float kp, float ki, float kd);
float SpeedCtrl_GetCurrentRPM(Motor_ID_t id);

/* FreeRTOS task function */
void SpeedCtrlTask(void *argument);

#endif /* __SPEED_CTRL_H__ */
