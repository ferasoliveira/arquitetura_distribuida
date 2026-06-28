/**
 * EB-15 Robotic Arm — Mestre (ESP32-S3)
 * pid_tanh.h — Controlador PID discreto com modulação tanh para J1-J3
 *
 * FORMULAÇÃO:
 *   error     = target_deg - current_deg
 *   d_error   = (error - prev_error) / dt
 *   integral  = clamp(integral + error * dt, -I_LIMIT, +I_LIMIT)
 *   u         = Kp*error + Ki*integral + Kd*d_error
 *   f_step    = f_max * tanh(gamma * u)       [Hz, com sinal]
 *
 * tanh(gamma * u):
 *   - Para erros grandes:  satura em ±f_max (movimento à velocidade máxima)
 *   - Para erros pequenos: velocidade proporcional ao erro (suave e estável)
 *   - Sinal negativo de u → direção inversa (tratado naturalmente pelo sinal de tanh)
 *
 * Zona morta: se |error| < DEADBAND, force f_step = 0 (encerra o movimento).
 *
 * Saída: f_step em Hz (positivo = avanço, negativo = recuo).
 *        O chamador converte para número de pulsos no período de amostragem.
 */
#pragma once

#include <math.h>
#include "config.h"

struct PIDState {
    float integral;
    float prev_error;
};

/**
 * Executa um ciclo do controlador PID+Tanh.
 *
 * @param axis       Índice da junta (0-2 para J1-J3)
 * @param target_deg Posição alvo em graus
 * @param current_deg Posição atual em graus (lida do encoder)
 * @param dt         Período de amostragem em segundos (tipicamente 0.005 s)
 * @param state      Estado persistente do controlador
 * @return           Frequência de step em Hz (positivo = avanço, negativo = recuo)
 */
static inline float pid_tanh_update(int axis, float target_deg, float current_deg,
                                    float dt, PIDState *state)
{
    const float error = target_deg - current_deg;

    /* Zona morta: encerra movimento suavemente */
    if (fabsf(error) < PID_DEADBAND_DEG) {
        state->integral  = 0.0f;
        state->prev_error = 0.0f;
        return 0.0f;
    }

    /* Derivada */
    const float d_error = (error - state->prev_error) / dt;

    /* Integral com anti-windup */
    state->integral += error * dt;
    if (state->integral >  PID_I_LIMIT) state->integral =  PID_I_LIMIT;
    if (state->integral < -PID_I_LIMIT) state->integral = -PID_I_LIMIT;

    /* Saída do controlador PID */
    const float u = PID_KP[axis] * error
                  + PID_KI[axis] * state->integral
                  + PID_KD[axis] * d_error;

    state->prev_error = error;

    /* Modulação não-linear: f = f_max * tanh(γ * u) */
    const float f_step = PID_FMAX_HZ[axis] * tanhf(PID_GAMMA[axis] * u);

    return f_step;
}

/**
 * Converte frequência de step (Hz) em número de pulsos para o período dt.
 * Usa acumulador DDA para evitar truncamento acumulado.
 *
 * @param f_hz      Frequência de step em Hz (positivo ou negativo)
 * @param dt        Período em segundos
 * @param accum     Acumulador fracionário persistente [0, 1)
 * @param pulses    Saída: número de pulsos a gerar neste ciclo
 * @param dir       Saída: direção (true = positivo)
 */
static inline void pid_step_pulses(float f_hz, float dt, float *accum,
                                   int *pulses, bool *dir)
{
    *dir = (f_hz >= 0.0f);
    const float steps_ideal = fabsf(f_hz) * dt;

    *accum  += steps_ideal;
    *pulses  = (int)(*accum);
    *accum  -= (float)(*pulses);
}
