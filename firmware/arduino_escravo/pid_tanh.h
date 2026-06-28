/**
 * EB-15 Robotic Arm — Escravo (Arduino UNO) — J4-J6
 * pid_tanh.h — MESMO controlador PID+tanh do mestre (ESP, J1-J3), para simetria
 * total entre as duas placas na co-simulação síncrona em malha fechada.
 *
 *   error    = target_deg - current_deg     (current vem do encoder AS5600)
 *   u        = Kp*error + Ki*integral + Kd*d_error
 *   f_step   = f_max * tanh(gamma * u)       [Hz, com sinal]
 *
 * Idêntico em forma a Códigos/firmware/esp32_mestre/main/pid_tanh.h; as CONSTANTES
 * (PID_KP[3], PID_FMAX_HZ[3], ...) vêm do config.h LOCAL do Uno (eixos 0-2 = J4-J6).
 */
#pragma once

#include <math.h>
#include "config.h"

struct PIDState {
    float integral;
    float prev_error;
};

static inline float pid_tanh_update(int axis, float target_deg, float current_deg,
                                    float dt, PIDState *state)
{
    const float error = target_deg - current_deg;

    if (fabsf(error) < PID_DEADBAND_DEG) {
        state->integral  = 0.0f;
        state->prev_error = 0.0f;
        return 0.0f;
    }

    const float d_error = (error - state->prev_error) / dt;

    state->integral += error * dt;
    if (state->integral >  PID_I_LIMIT) state->integral =  PID_I_LIMIT;
    if (state->integral < -PID_I_LIMIT) state->integral = -PID_I_LIMIT;

    const float u = PID_KP[axis] * error
                  + PID_KI[axis] * state->integral
                  + PID_KD[axis] * d_error;

    state->prev_error = error;

    // No avr-gcc double==float (32 bits); tanh() evita depender de tanhf em avr-libc.
    return PID_FMAX_HZ[axis] * (float)tanh((double)(PID_GAMMA[axis] * u));
}

static inline void pid_step_pulses(float f_hz, float dt, float *accum,
                                   int *pulses, bool *dir)
{
    *dir = (f_hz >= 0.0f);
    const float steps_ideal = fabsf(f_hz) * dt;
    *accum  += steps_ideal;
    *pulses  = (int)(*accum);
    *accum  -= (float)(*pulses);
}
