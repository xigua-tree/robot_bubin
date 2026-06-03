#include "speed_ctrl.h"
#include "cmsis_os.h"

/* 4 motor speed controllers */
SpeedCtrl_t g_speed_ctrl[4];

/* ============ API ============ */

void SpeedCtrl_Init(void)
{
    for (int i = 0; i < 4; i++) {
        g_speed_ctrl[i].motor_id   = (Motor_ID_t)i;
        g_speed_ctrl[i].cpr        = DEFAULT_CPR;
        g_speed_ctrl[i].mode       = MODE_DUTY;
        g_speed_ctrl[i].target_rpm = 0.0f;
        g_speed_ctrl[i].current_rpm = 0.0f;

        PID_Init(&g_speed_ctrl[i].pid,
                 0.5f,    /* Kp */
                 0.02f,   /* Ki */
                 0.0f,    /* Kd */
                 -100.0f, /* out_min = -100% duty */
                  100.0f);/* out_max = +100% duty */
    }
}

void SpeedCtrl_SetMode(Motor_ID_t id, uint8_t mode)
{
    if (id >= 4) return;

    SpeedCtrl_t *sc = &g_speed_ctrl[id];

    if (mode == MODE_SPEED && sc->mode != MODE_SPEED) {
        /* Entering speed mode: init encoder baseline and reset PID */
        sc->mode = MODE_SPEED;
        sc->last_encoder = Motor_GetEncoder(id);
        PID_Reset(&sc->pid);
        PID_SetSetpoint(&sc->pid, sc->target_rpm);
    } else if (mode == MODE_DUTY && sc->mode != MODE_DUTY) {
        /* Exiting speed mode: stop motor to prevent runaway */
        sc->mode = MODE_DUTY;
        Motor_Stop(id);
    }
}

void SpeedCtrl_SetTargetRPM(Motor_ID_t id, float rpm)
{
    if (id >= 4) return;
    SpeedCtrl_t *sc = &g_speed_ctrl[id];
    sc->target_rpm = rpm;
    if (rpm == 0.0f) {
        /* Target zero: immediate stop, no PID regulation needed */
        Motor_Stop(id);
        PID_Reset(&sc->pid);
        PID_SetSetpoint(&sc->pid, 0.0f);
    } else {
        PID_SetSetpoint(&sc->pid, rpm);
    }
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

    /* Initialize last_encoder for all motors (in case InitAll wasn't called) */
    int32_t last_enc[4];
    for (int i = 0; i < 4; i++) {
        last_enc[i] = Motor_GetEncoder((Motor_ID_t)i);
        g_speed_ctrl[i].last_encoder = last_enc[i];
    }

    for (;;) {
        for (int i = 0; i < 4; i++) {
            SpeedCtrl_t *sc = &g_speed_ctrl[i];
            if (sc->mode != MODE_SPEED) continue;
            if (sc->cpr == 0) continue;  /* safety: prevent div-by-zero */

            /* == Speed measurement == */
            int32_t enc_now = Motor_GetEncoder((Motor_ID_t)i);
            int16_t delta   = (int16_t)((uint16_t)enc_now - (uint16_t)last_enc[i]);
            last_enc[i] = enc_now;

            /* RPM = delta × (1000 × 60) / CPR = delta × 60000 / CPR */
            float rpm = (float)delta * 60000.0f / (float)sc->cpr;
            sc->current_rpm = rpm;

            /* == PID update == */
            float duty = PID_Update(&sc->pid, rpm);
            Motor_SetDuty((Motor_ID_t)i, (int8_t)duty);
        }

        osDelay(1);   /* 1 kHz loop (configTICK_RATE_HZ=1000) */
    }
}
