#ifndef __PID_H__
#define __PID_H__

#include <stdint.h>

typedef struct {
    float   kp, ki, kd;       /* PID coefficients */
    float   setpoint;          /* target value */
    float   output;            /* controller output */
    float   out_min, out_max;  /* output clamping range */
    float   integral_max;      /* integral TERM limit (Ki×Σe clamped to ±this) */
    /* Internal state */
    float   integral;          /* accumulated integral (Σe) */
    float   e_prev;            /* previous error for D term */
    uint8_t first_run;         /* skip derivative on first sample */
} PID_t;

void PID_Init(PID_t *pid, float kp, float ki, float kd, float out_min, float out_max);

/**
 * u(k) = Kp×e(k) + Ki×Σe(i) + Kd×[e(k)-e(k-1)]
 * With output clamping + integral anti-windup.
 */
float PID_Update(PID_t *pid, float measurement);

void PID_Reset(PID_t *pid);
void PID_SetTunings(PID_t *pid, float kp, float ki, float kd);
void PID_SetSetpoint(PID_t *pid, float setpoint);

#endif /* __PID_H__ */
