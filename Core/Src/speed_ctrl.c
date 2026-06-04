#include "speed_ctrl.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"

/* Task handle (set by SpeedCtrlTask itself).
 * Used by SpeedCtrl_NotifyFromISR() to wake it via Task Notification. */
static TaskHandle_t xSpeedCtrlTaskHandle = NULL;

void SpeedCtrl_NotifyFromISR(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (xSpeedCtrlTaskHandle != NULL) {
        vTaskNotifyGiveFromISR(xSpeedCtrlTaskHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

SpeedCtrl_t g_speed_ctrl[4];

void SpeedCtrl_Init(void)
{
    for (int i = 0; i < 4; i++) {
        g_speed_ctrl[i].motor_id     = (Motor_ID_t)i;
        g_speed_ctrl[i].cpr          = DEFAULT_CPR;
        g_speed_ctrl[i].target_rpm   = 0.0f;
        g_speed_ctrl[i].current_rpm  = 0.0f;
        g_speed_ctrl[i].filtered_rpm = 0.0f;

        /* All gains start at 0 — tune via Bluetooth.
         * Send Kp/Ki/Kd in the protocol frame and the host will
         * call PID_SetTunings() with your values. */
        PID_Init(&g_speed_ctrl[i].pid,
                 0.0f,    /* Kp — set via Bluetooth */
                 0.0f,    /* Ki — set via Bluetooth */
                 0.0f,    /* Kd — set via Bluetooth */
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

    /* Register handle so ISR can wake us */
    xSpeedCtrlTaskHandle = xTaskGetCurrentTaskHandle();

    int32_t last_enc[4];
    for (int i = 0; i < 4; i++) {
        last_enc[i] = Motor_GetEncoder((Motor_ID_t)i);
        g_speed_ctrl[i].last_encoder = last_enc[i];
    }

    for (;;) {
        /* Block until TIM8 update ISR (1 ms) wakes us.
         * ulTaskNotifyTake clears the notification on exit (pdTRUE),
         * guaranteeing we don't accumulate stale wake-ups.       */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        for (int i = 0; i < 4; i++) {
            SpeedCtrl_t *sc = &g_speed_ctrl[i];
            if (sc->cpr == 0) continue;

            /* Speed measurement */
            int32_t enc_now = Motor_GetEncoder((Motor_ID_t)i);
            int16_t delta   = (int16_t)((uint16_t)enc_now - (uint16_t)last_enc[i]);
            last_enc[i] = enc_now;

            float rpm = (float)delta * 60000.0f / (float)sc->cpr;
            sc->current_rpm = rpm;

            /* Low-pass filter: smooth encoder quantisation and residual noise.
             * EMA with α=0.9 → time constant ≈ 10 ms @ 1 kHz. */
            #define EMA_ALPHA  0.9f
            sc->filtered_rpm = sc->filtered_rpm * EMA_ALPHA
                             + rpm * (1.0f - EMA_ALPHA);

            /* PID update — always runs, for all 4 motors */
            float duty = PID_Update(&sc->pid, sc->filtered_rpm);
            Motor_SetDuty((Motor_ID_t)i, (int8_t)duty);
        }
    }
}
