#include "pid.h"

static float clamp(float val, float lo, float hi)
{
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

void PID_Init(PID_t *pid, float kp, float ki, float kd, float out_min, float out_max)
{
    pid->kp           = kp;
    pid->ki           = ki;
    pid->kd           = kd;
    pid->setpoint     = 0.0f;
    pid->output       = 0.0f;
    pid->out_min      = out_min;
    pid->out_max      = out_max;
    pid->integral_max = 30.0f;   /* integral term max contributes ±30% duty */
    pid->integral     = 0.0f;
    pid->e_prev       = 0.0f;
    pid->first_run    = 1;
}

float PID_Update(PID_t *pid, float measurement)
{
    float e = pid->setpoint - measurement;

    /* Proportional */
    float p_term = pid->kp * e;

    /* Integral with anti-windup:
     * Only accumulate if output is not saturated in the same direction.
     * This prevents integral windup when the motor can't reach the target. */
    if ((pid->output > pid->out_min || e > 0.0f) &&
        (pid->output < pid->out_max || e < 0.0f)) {
        pid->integral += e;
    }
    /* Clamp integral term contribution to reasonable range */
    float i_max = pid->out_max * 2.0f;
    pid->integral = clamp(pid->integral, -i_max, i_max);
    float i_term = pid->ki * pid->integral;

    /* Derivative (skip on first sample) */
    float d_term = 0.0f;
    if (!pid->first_run) {
        d_term = pid->kd * (e - pid->e_prev);
    }
    pid->e_prev   = e;
    pid->first_run = 0;

    /* Compute output and clamp */
    pid->output = p_term + i_term + d_term;
    pid->output = clamp(pid->output, pid->out_min, pid->out_max);

    return pid->output;
}

void PID_Reset(PID_t *pid)
{
    pid->integral  = 0.0f;
    pid->e_prev    = 0.0f;
    pid->first_run = 1;
    pid->output    = 0.0f;
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
