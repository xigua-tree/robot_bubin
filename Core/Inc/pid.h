#ifndef __PID_H__
#define __PID_H__

#include <stdint.h>

typedef struct {
    float   kp, ki, kd;       /* PID coefficients (T=1ms baked in) */
    float   setpoint;          /* target value */
    float   output;            /* u(k) current controller output */
    float   out_min, out_max;  /* output clamping range */
    /* Internal state */
    float   e_prev[2];         /* e(k-1), e(k-2) */
    uint8_t first_run;         /* skip first update (no delta yet) */
} PID_t;

/**
 * @brief Initialize PID controller
 * @param out_min  Minimum output (e.g. -100 for -100% duty)
 * @param out_max  Maximum output (e.g. +100 for +100% duty)
 */
void PID_Init(PID_t *pid, float kp, float ki, float kd, float out_min, float out_max);

/**
 * @brief Update PID with current measurement, return control output
 * @param measurement  Current measured value
 * @return  Control output, clamped to [out_min, out_max]
 *
 * Uses incremental form:
 *   Δu = Kp×(e₀-e₁) + Ki×e₀ + Kd×(e₀-2×e₁+e₂)
 *   u(k) = u(k-1) + Δu
 */
float PID_Update(PID_t *pid, float measurement);

/**
 * @brief Reset PID state (clear error history, zero output)
 * Coefficients and limits are preserved.
 */
void PID_Reset(PID_t *pid);

/**
 * @brief Change tuning coefficients at runtime
 */
void PID_SetTunings(PID_t *pid, float kp, float ki, float kd);

/**
 * @brief Change setpoint at runtime
 */
void PID_SetSetpoint(PID_t *pid, float setpoint);

#endif /* __PID_H__ */
