#include "pid.h"

static float clamp(float val, float lo, float hi)
{
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

void PID_Init(PID_t *pid, float kp, float ki, float kd, float out_min, float out_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->setpoint = 0.0f;
    pid->output   = 0.0f;
    pid->out_min  = out_min;
    pid->out_max  = out_max;
    pid->e_prev[0] = 0.0f;
    pid->e_prev[1] = 0.0f;
    pid->first_run = 1;
}

float PID_Update(PID_t *pid, float measurement)
{
    float e0 = pid->setpoint - measurement;

    if (pid->first_run) {
        /* First sample: store error, no delta yet */
        pid->e_prev[0] = e0;
        pid->e_prev[1] = e0;
        pid->first_run = 0;
        return pid->output;   /* return previous (initial) output */
    }

    float e1 = pid->e_prev[0];
    float e2 = pid->e_prev[1];

    /* Incremental PID: Δu = Kp×(e₀-e₁) + Ki×e₀ + Kd×(e₀-2×e₁+e₂) */
    float du = pid->kp * (e0 - e1)
             + pid->ki * e0
             + pid->kd * (e0 - 2.0f * e1 + e2);

    pid->output += du;
    pid->output = clamp(pid->output, pid->out_min, pid->out_max);

    /* Shift error history */
    pid->e_prev[1] = e1;
    pid->e_prev[0] = e0;

    return pid->output;
}

void PID_Reset(PID_t *pid)
{
    pid->e_prev[0] = 0.0f;
    pid->e_prev[1] = 0.0f;
    pid->first_run = 1;
    pid->output   = 0.0f;
}

void PID_SetTunings(PID_t *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    PID_Reset(pid);
}

void PID_SetSetpoint(PID_t *pid, float setpoint)
{
    pid->setpoint = setpoint;
}
