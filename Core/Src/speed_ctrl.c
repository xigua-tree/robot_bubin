#include "speed_ctrl.h"
#include "cmsis_os.h"

SpeedCtrl_t g_speed_ctrl[4];

void SpeedCtrl_Init(void)
{
    for (int i = 0; i < 4; i++) {
        g_speed_ctrl[i].motor_id    = (Motor_ID_t)i;
        g_speed_ctrl[i].cpr         = DEFAULT_CPR;
        g_speed_ctrl[i].target_rpm  = 0.0f;
        g_speed_ctrl[i].current_rpm = 0.0f;

        PID_Init(&g_speed_ctrl[i].pid,
                 2.0f,    /* Kp */
                 0.1f,    /* Ki */
                 0.0f,    /* Kd */
                 -100.0f, /* out_min */
                  100.0f);/* out_max */
    }
}

void SpeedCtrl_SetTargetRPM(Motor_ID_t id, float rpm)
{
    if (id >= 4) return;
    SpeedCtrl_t *sc = &g_speed_ctrl[id];
    sc->target_rpm = rpm;
    PID_SetSetpoint(&sc->pid, rpm);
    /* Setpoint 0: let Ki accumulate braking naturally (PID must have Ki > 0) */
}

void SpeedCtrl_SetPID(Motor_ID_t id, float kp, float ki, float kd)
{
    if (id >= 4) return;
    PID_SetTunings(&g_speed_ctrl[id].pid, kp, ki, kd);
}

float SpeedCtrl_GetCurrentRPM(Motor_ID_t id)
{
    if (id >= 4) return 0.0f;
    return g_speed_ctrl[id].current_rpm;
}

/* ============ FreeRTOS 1kHz Speed Control Task ============ */

void SpeedCtrlTask(void *argument)
{
    (void)argument;

    int32_t last_enc[4];
    for (int i = 0; i < 4; i++) {
        last_enc[i] = Motor_GetEncoder((Motor_ID_t)i);
        g_speed_ctrl[i].last_encoder = last_enc[i];
    }

    for (;;) {
        for (int i = 0; i < 4; i++) {
            SpeedCtrl_t *sc = &g_speed_ctrl[i];
            if (sc->cpr == 0) continue;

            /* Speed measurement */
            int32_t enc_now = Motor_GetEncoder((Motor_ID_t)i);
            int16_t delta   = (int16_t)((uint16_t)enc_now - (uint16_t)last_enc[i]);
            last_enc[i] = enc_now;

            float rpm = (float)delta * 60000.0f / (float)sc->cpr;
            sc->current_rpm = rpm;

            /* PID update — always runs, for all 4 motors */
            float duty = PID_Update(&sc->pid, rpm);
            Motor_SetDuty((Motor_ID_t)i, (int8_t)duty);
        }
        osDelay(1);
    }
}
