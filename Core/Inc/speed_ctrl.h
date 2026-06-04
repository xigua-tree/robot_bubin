#ifndef __SPEED_CTRL_H__
#define __SPEED_CTRL_H__

#include "motor.h"
#include "pid.h"
#include <stdint.h>

/* Default encoder CPR (Counts Per Revolution): measured per output revolution */
#define DEFAULT_CPR  1300

typedef struct {
    Motor_ID_t   motor_id;
    PID_t        pid;
    float        target_rpm;       /* desired RPM, 0 = stop */
    float        current_rpm;      /* raw measured RPM (for telemetry) */
    float        filtered_rpm;     /* low-pass filtered RPM → fed to PID */
    uint16_t     cpr;              /* encoder CPR */
    int32_t      last_encoder;     /* previous encoder reading */
} SpeedCtrl_t;

extern SpeedCtrl_t g_speed_ctrl[4];

void SpeedCtrl_Init(void);
void SpeedCtrl_SetTargetRPM(Motor_ID_t id, float rpm);
void SpeedCtrl_SetPID(Motor_ID_t id, float kp, float ki, float kd);
float SpeedCtrl_GetCurrentRPM(Motor_ID_t id);

void SpeedCtrlTask(void *argument);

/* Called from TIM8 update ISR (priority 5, can call FreeRTOS FromISR APIs).
 * Wakes up SpeedCtrlTask at exactly 1 ms intervals. */
void SpeedCtrl_NotifyFromISR(void);

#endif /* __SPEED_CTRL_H__ */
