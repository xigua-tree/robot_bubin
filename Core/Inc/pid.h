#ifndef __PID_H__
#define __PID_H__

#include <stdint.h>

typedef struct {
    float   kp, ki, kd;       /* PID coefficients */
    float   setpoint;          /* target value */
    float   output;            /* accumulated output u(k-1) */
    float   out_min, out_max;  /* output clamping range */
    /* Internal state — incremental form needs two historical errors */
    float   e_prev1;           /* e(k-1) */
    float   e_prev2;           /* e(k-2) */
    uint8_t first_run;         /* seed error history on first sample */
} PID_t;

void PID_Init(PID_t *pid, float kp, float ki, float kd, float out_min, float out_max);

/**
 * Incremental PID:
 *   Δu(k) = Kp·[e(k)-e(k-1)] + Ki·e(k) + Kd·[e(k)-2e(k-1)+e(k-2)]
 *   u(k)  = u(k-1) + Δu(k)
 *
 * Output is clamped to [out_min, out_max]. Integral anti-windup is
 * inherent — there is no accumulator, so you never need to unwind it.
 */
float PID_Update(PID_t *pid, float measurement);

void PID_Reset(PID_t *pid);
void PID_SetTunings(PID_t *pid, float kp, float ki, float kd);
void PID_SetSetpoint(PID_t *pid, float setpoint);

#endif /* __PID_H__ */
