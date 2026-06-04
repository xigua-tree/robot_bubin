#include "pid.h"

static float clamp(float val, float lo, float hi)
{
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

void PID_Init(PID_t *pid, float kp, float ki, float kd, float out_min, float out_max)
{
    pid->kp       = kp;
    pid->ki       = ki;
    pid->kd       = kd;
    pid->setpoint = 0.0f;
    pid->output   = 0.0f;
    pid->out_min  = out_min;
    pid->out_max  = out_max;
    pid->e_prev1  = 0.0f;
    pid->e_prev2  = 0.0f;
    pid->first_run = 1;
}

float PID_Update(PID_t *pid, float measurement)
{
    float e = pid->setpoint - measurement;

    /* On first run, seed the error history with current error.
     * This makes Δu = Ki·e(k) for the very first step (P and D terms
     * cancel to zero), giving a smooth start without a spike.       */
    if (pid->first_run) {
        pid->e_prev1 = e;
        pid->e_prev2 = e;
        pid->first_run = 0;
    }

    /* ── Incremental PID calculation ─────────────────────────────
     * Δu(k) = Kp·[e(k)-e(k-1)] + Ki·e(k) + Kd·[e(k)-2e(k-1)+e(k-2)]
     * u(k)  = u(k-1) + Δu(k)                                     */
    float delta_u = pid->kp * (e - pid->e_prev1)
                  + pid->ki * e
                  + pid->kd * (e - 2.0f * pid->e_prev1 + pid->e_prev2);

    /* Shift error history for next iteration */
    pid->e_prev2 = pid->e_prev1;
    pid->e_prev1 = e;

    /* Accumulate and clamp the final output */
    pid->output += delta_u;
    pid->output = clamp(pid->output, pid->out_min, pid->out_max);

    return pid->output;
}

void PID_Reset(PID_t *pid)
{
    pid->e_prev1  = 0.0f;
    pid->e_prev2  = 0.0f;
    pid->first_run = 1;
    pid->output   = 0.0f;
}

void PID_SetTunings(PID_t *pid, float kp, float ki, float kd)
{
    /* Only reset PID state if the gains actually changed.
     * Avoids jerky output jumps when the host sends the same
     * parameters repeatedly (e.g. every speed-command frame). */
    if (pid->kp != kp || pid->ki != ki || pid->kd != kd) {
        PID_Reset(pid);
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void PID_SetSetpoint(PID_t *pid, float setpoint)
{
    pid->setpoint = setpoint;
}
