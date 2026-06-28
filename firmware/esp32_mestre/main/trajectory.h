/**
 * EB-15 Robotic Arm — Mestre (ESP32-S3)
 * trajectory.h — Planejador de trajetória com perfil Curva-S de 7 segmentos
 *
 * PERFIL DE 7 SEGMENTOS (jerk finito):
 *   Seg 1: Jerk +J  → aceleração sobe de 0 a A_max
 *   Seg 2: Jerk  0  → aceleração constante A_max
 *   Seg 3: Jerk -J  → aceleração cai de A_max a 0 (v_peak atingida)
 *   Seg 4: Jerk  0  → velocidade constante V_max (cruzeiro)
 *   Seg 5: Jerk -J  → desaceleração começa
 *   Seg 6: Jerk  0  → desaceleração constante
 *   Seg 7: Jerk +J  → desaceleração termina suavemente (v=0)
 *
 * Garantias matemáticas:
 *   - Posição monotônica (sem reversão) para qualquer deslocamento > 0
 *   - Velocidade final exata = 0
 *   - Continuidade C² em todo o perfil
 *   - Deslocamentos curtos: reduz para perfil triangular puro (sem cruzeiro)
 *     ou perfil puramente limitado por jerk (sem aceleração constante)
 *
 * API pública:
 *   scurve_plan()          — calcula parâmetros para um eixo
 *   scurve_sample()        — amostra posição normalizada [0,1] em tempo τ ∈ [0,1]
 *   push_segment()         — insere segmento na fila circular
 *   plan_move_j_scurve()   — planeja trajetória no espaço de juntas
 *   plan_move_l_scurve()   — planeja trajetória linear cartesiana
 */
#pragma once

#include <math.h>
#include <string.h>
#include "config.h"
#include "kinematics.h"

/* Dependência da plataforma: delay mínimo para aguardar fila */
#ifndef EB15_DELAY_MS
  #ifdef EB15_NATIVE_TEST
    /* Teste nativo: sem delay real */
    static inline void eb15_delay_ms(uint32_t) {}
  #else
    /* ESP-IDF: delay via FreeRTOS */
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    static inline void eb15_delay_ms(uint32_t ms) {
        vTaskDelay(pdMS_TO_TICKS(ms ? ms : 1));
    }
  #endif
  #define EB15_DELAY_MS eb15_delay_ms
#endif

// ============================================================================
// Estruturas de Dados
// ============================================================================

/** Segmento atômico de trajetória (fatia de SEG_SLICE_MS ms). */
struct TrajectorySegment {
    float    q_deg[6];      /* posição alvo de todas as juntas em GRAUS */
    uint32_t duration_ms;   /* duração desta fatia                       */
    bool     active;
};

/** Parâmetros calculados do perfil S-Curve para um único eixo. */
struct SCurveProfile {
    float j_max;            /* jerk máximo aplicado (deg/s³)   */
    float a_max;            /* aceleração máxima (deg/s²)       */
    float v_max;            /* velocidade de pico com sinal     */
    float phase_dt[7];      /* duração de cada fase             */
    float t7;               /* duração total do movimento       */
    float total_dist;       /* deslocamento com sinal (deg)     */
    bool  valid;
};

// ============================================================================
// Estado global (definido em main.cpp)
// ============================================================================

extern TrajectorySegment g_segments[MAX_SEGMENTS];
extern volatile int g_seg_head;
extern volatile int g_seg_tail;
extern float g_joint_angles[6];     /* ângulos atuais em GRAUS */

/* Duração (ms) do último MoveJ planeado — exporta T_total p/ o despacho J4-J6. */
static uint32_t g_scurve_last_total_ms = 0;

/* Fatias S-curve de J4-J6 do último MoveJ, em PASSOS ABSOLUTOS, para streaming ao
   escravo (buffer look-ahead): J4-J6 reproduzem a MESMA curva S de J1-J3. */
#define J46_MAX_SLICES 64
static int16_t  g_j46_slices[J46_MAX_SLICES][3];   /* alvo ABSOLUTO em passos */
static uint16_t g_j46_dur[J46_MAX_SLICES];         /* duração da fatia (ms) */
static int      g_j46_nslices = 0;

struct CartesianPose { float x, y, z, rx, ry, rz; };
extern CartesianPose g_current_pose;

// ============================================================================
// Planejador S-Curve — scurve_plan
// ============================================================================

/**
 * Calcula os tempos dos 7 segmentos para um deslocamento.
 *
 * Três casos tratados:
 *   1. dist_deg == 0          → trivial, valid=false
 *   2. deslocamento curto sem atingir A_max:
 *        tj = cbrt(|d|/(2J)), ta=0 → perfil triangular puro de jerk
 *   3. deslocamento curto sem atingir V_max (mas atinge A_max):
 *        tj = A/J, ta resolvido de |d| = A*ta² + 2J*tj²*ta + 2J*tj³
 *   4. deslocamento longo (atinge V_max):
 *        cruzeiro tv = (|d| - ramp_dist) / v_peak
 */
static void scurve_plan(float dist_deg, int axis, float speed_pct,
                        SCurveProfile *out)
{
    memset(out, 0, sizeof(*out));
    const float d = fabsf(dist_deg);
    if (d < 1e-5f) { out->total_dist = dist_deg; return; }

    const float sf = constrainf(speed_pct / 100.0f, 0.05f, 1.0f);
    const float V  = MAX_SPEED_DEG_S[axis]  * sf;
    const float A  = MAX_ACCEL_DEG_S2[axis] * sf;
    const float J  = MAX_JERK_DEG_S3[axis];

    /* Tempo mínimo de rampa de jerk para atingir A */
    float tj = A / J;
    /* Mas se a velocidade pico atingível com tj=A/J excede V, reduz tj */
    {
        float tj_v = sqrtf(V / J);  /* tj tal que J*tj² = V (sem ta) */
        if (tj > tj_v) tj = tj_v;
    }

    float ta  = fmaxf(0.0f, V / A - tj);   /* tempo de aceleração constante */
    float vp  = J * tj * (tj + ta);         /* velocidade de pico            */
    /* Distância percorrida nas duas rampas (aceleração + desaceleração) */
    float ramp_dist = vp * (2.0f * tj + ta);
    float tv  = 0.0f;

    if (d < ramp_dist) {
        /* Não atinge V_max → reduz o perfil */
        const float d_amax = 2.0f * A * A * A / (J * J); /* |d| máximo sem cruzeiro, ta=0 */
        if (d <= d_amax) {
            /* Perfil triangular puro: só jerk, sem A_max */
            tj = cbrtf(d / (2.0f * J));
            ta = 0.0f;
        } else {
            /* Perfil trapezoidal: atinge A_max mas não V_max.
             * Distância total (aceleração + desaceleração simétricas):
             *   d = 2*J*tj³ + 3*J*tj²*ta + J*tj*ta²
             *   com tj = A/J:
             *   d = 2*A³/J² + 3*(A²/J)*ta + A*ta²
             * Isolando ta (quadrática, raiz positiva):
             *   ta = -1.5*(A/J) + sqrt(0.25*(A/J)² + d/A)
             *      = -1.5*tj   + sqrt(0.25*tj²      + d/A)   */
            tj = A / J;
            ta = -1.5f * tj + sqrtf(0.25f * tj * tj + d / A);
            if (ta < 0.0f) ta = 0.0f;
        }
        vp = J * tj * (tj + ta);
        tv = 0.0f;
    } else {
        tv = (d - ramp_dist) / vp;
    }

    /* Durações das 7 fases: [+J, 0, -J, 0, -J, 0, +J] */
    const float dt[7] = { tj, ta, tj, tv, tj, ta, tj };
    float cumul = 0.0f;
    for (int i = 0; i < 7; ++i) {
        out->phase_dt[i] = dt[i];
        cumul += dt[i];
    }
    out->t7         = cumul;
    out->j_max      = J;
    out->a_max      = J * tj;
    out->v_max      = (dist_deg >= 0.0f) ? vp : -vp;
    out->total_dist = dist_deg;
    out->valid      = (cumul > 1e-9f);
}

// ============================================================================
// Amostrador S-Curve — scurve_sample
// ============================================================================

/**
 * Retorna posição normalizada ∈ [0, 1] para τ ∈ [0, 1].
 *
 * Integra as equações cinemáticas de cada fase numericamente exata
 * (sem aproximação numérica — integração simbólica dentro de cada fase).
 *
 * Sequência de jerk: [+J, 0, -J, 0, -J, 0, +J]
 *   Fases 1-3: rampa de aceleração
 *   Fase  4:   cruzeiro
 *   Fases 5-7: rampa de desaceleração (espelho)
 */
static float scurve_sample(float tau, const SCurveProfile *prof)
{
    if (!prof->valid) return constrainf(tau, 0.0f, 1.0f);

    const float t_abs = constrainf(tau, 0.0f, 1.0f) * prof->t7;
    float remaining   = t_abs;
    float x = 0.0f, v = 0.0f, a = 0.0f;

    const float jerk_seq[7] = {
         prof->j_max,  0.0f, -prof->j_max,
         0.0f,
        -prof->j_max,  0.0f,  prof->j_max
    };

    for (int i = 0; i < 7 && remaining > 1e-12f; ++i) {
        const float dt = (remaining < prof->phase_dt[i]) ? remaining : prof->phase_dt[i];
        const float j  = jerk_seq[i];
        x += v * dt + 0.5f * a * dt * dt + j * dt * dt * dt / 6.0f;
        v += a * dt + 0.5f * j * dt * dt;
        a += j * dt;
        remaining -= dt;
    }

    const float d_abs = fabsf(prof->total_dist);
    if (d_abs < 1e-9f) return 0.0f;
    return constrainf(x / d_abs, 0.0f, 1.0f);
}

// ============================================================================
// Fila circular de segmentos
// ============================================================================

/**
 * Insere segmento na fila. Retorna false se fila cheia.
 * Thread-safe apenas se chamada de uma única tarefa produtora.
 */
static bool push_segment(const float q_deg[6], uint32_t duration_ms)
{
    const int next = (g_seg_head + 1) % MAX_SEGMENTS;
    if (next == g_seg_tail) return false;   /* fila cheia */

    TrajectorySegment *seg = &g_segments[g_seg_head];
    for (int i = 0; i < 6; ++i) seg->q_deg[i] = q_deg[i];
    seg->duration_ms = duration_ms;
    seg->active      = true;
    g_seg_head       = next;
    return true;
}

// ============================================================================
// plan_move_j_scurve — Trajetória no espaço de juntas
// ============================================================================

/**
 * Planeja e enfileira um movimento MoveJ com S-Curve.
 *
 * A junta de maior deslocamento absoluto (líder) define T_total.
 * Todas as outras juntas escalam seu perfil ao mesmo T_total para chegarem
 * simultaneamente (sincronismo temporal).
 */
static void plan_move_j_scurve(const float target_deg[6], float speed_pct)
{
    if (!check_soft_limits(target_deg)) return;

    float delta[6];
    int   leader = 0;
    for (int i = 0; i < 6; ++i) {
        delta[i] = target_deg[i] - g_joint_angles[i];
        if (fabsf(delta[i]) > fabsf(delta[leader])) leader = i;
    }
    if (fabsf(delta[leader]) < 0.01f) return;

    SCurveProfile prof;
    scurve_plan(delta[leader], leader, speed_pct, &prof);
    if (!prof.valid) return;

    const float T_total = (prof.t7 < 0.01f) ? 0.01f : prof.t7;
    const int   slices  = (int)ceilf(T_total * 1000.0f / (float)SEG_SLICE_MS);
    /* Duração total do movimento (ms): usada pelo despacho ABSOLUTO de J4-J6
       (um frame por MoveJ, DDA do escravo sincronizada com a S-curve de J1-J3). */
    g_scurve_last_total_ms = (uint32_t)(T_total * 1000.0f + 0.5f);

    const float start[6] = {
        g_joint_angles[0], g_joint_angles[1], g_joint_angles[2],
        g_joint_angles[3], g_joint_angles[4], g_joint_angles[5]
    };

    /* Reinicia coleta de fatias J4-J6 (passos absolutos) para streaming ao escravo.
       MERGE/COARSEN: emite uma fatia quando J4-J6 avança ≥ step_merge passos,
       carregando a duração acumulada → preserva o perfil de velocidade da curva S,
       evita fatias de 0 passo (travariam o encadeamento do DDA) E limita o número de
       fatias a ~J46_TARGET_SLICES (passo_merge dimensionado pelo tamanho do movimento)
       para nunca estourar o buffer do escravo (o que truncava o movimento). */
    g_j46_nslices = 0;
    auto clamp16 = [](int32_t v) -> int16_t {
        if (v > 32767)  return 32767;
        if (v < -32768) return -32768;
        return (int16_t)v;
    };
    int16_t last_abs[3];
    for (int i = 0; i < 3; ++i)
        last_abs[i] = clamp16((int32_t)(start[3 + i] * STEPS_PER_DEG[3 + i]));
    int16_t final_abs[3];
    for (int i = 0; i < 3; ++i)
        final_abs[i] = clamp16((int32_t)(target_deg[3 + i] * STEPS_PER_DEG[3 + i]));
    /* Passos do maior eixo J4-J6 → granularidade que mantém nº de fatias ≤ alvo. */
    const int J46_TARGET_SLICES = 48;
    int32_t max_steps_j46 = 0;
    for (int i = 0; i < 3; ++i) {
        int32_t ds = (int32_t)final_abs[i] - (int32_t)last_abs[i];
        if (ds < 0) ds = -ds;
        if (ds > max_steps_j46) max_steps_j46 = ds;
    }
    int32_t step_merge = (max_steps_j46 + J46_TARGET_SLICES - 1) / J46_TARGET_SLICES;
    if (step_merge < 1) step_merge = 1;
    uint32_t accum_ms = 0;

    for (int s = 1; s <= slices; ++s) {
        const float tau    = (float)s / (float)slices;
        const float factor = scurve_sample(tau, &prof);

        float q[6];
        for (int i = 0; i < 6; ++i)
            q[i] = start[i] + delta[i] * factor;

        accum_ms += (uint32_t)SEG_SLICE_MS;
        int16_t abs_j[3];
        for (int i = 0; i < 3; ++i)
            abs_j[i] = clamp16((int32_t)(q[3 + i] * STEPS_PER_DEG[3 + i]));
        int32_t dmax = 0;
        for (int i = 0; i < 3; ++i) {
            int32_t dd = (int32_t)abs_j[i] - (int32_t)last_abs[i];
            if (dd < 0) dd = -dd;
            if (dd > dmax) dmax = dd;
        }
        if (dmax >= step_merge && g_j46_nslices < J46_MAX_SLICES) {
            for (int i = 0; i < 3; ++i) g_j46_slices[g_j46_nslices][i] = abs_j[i];
            g_j46_dur[g_j46_nslices] = (uint16_t)(accum_ms > 60000U ? 60000U : accum_ms);
            g_j46_nslices++;
            last_abs[0] = abs_j[0]; last_abs[1] = abs_j[1]; last_abs[2] = abs_j[2];
            accum_ms = 0;
        }

        while (!push_segment(q, (uint32_t)SEG_SLICE_MS))
            EB15_DELAY_MS(2);
    }

    /* Garante que a ÚLTIMA fatia é o alvo final exato de J4-J6. */
    if ((last_abs[0] != final_abs[0] || last_abs[1] != final_abs[1] ||
         last_abs[2] != final_abs[2]) && g_j46_nslices < J46_MAX_SLICES) {
        for (int i = 0; i < 3; ++i) g_j46_slices[g_j46_nslices][i] = final_abs[i];
        uint32_t d = accum_ms ? accum_ms : (uint32_t)SEG_SLICE_MS;
        g_j46_dur[g_j46_nslices] = (uint16_t)(d > 60000U ? 60000U : d);
        g_j46_nslices++;
    }
}

// ============================================================================
// plan_move_l_scurve — Trajetória linear cartesiana
// ============================================================================

/**
 * Planeja um MoveL: interpola a pose TCP linearmente e resolve IK para cada fatia.
 * Fail-fast: aborta e limpa a fila se qualquer ponto intermediário falhar na IK.
 */
static bool plan_move_l_scurve(const CartesianPose *target, float speed_pct)
{
    /* Valida pose alvo */
    Mat3 R_target;
    euler_deg_to_rotation(target->rx, target->ry, target->rz, R_target);
    {
        float p_tgt[3] = { target->x, target->y, target->z };
        float pre_q[6];
        if (!inverse_kinematics(p_tgt, R_target, pre_q)) return false;
    }

    /* Distância euclidiana total */
    const float dx = target->x - g_current_pose.x;
    const float dy = target->y - g_current_pose.y;
    const float dz = target->z - g_current_pose.z;
    const float dist = sqrtf(dx*dx + dy*dy + dz*dz);
    if (dist < 0.1f) return true;

    /* Planeja S-Curve usando J1 como referência de limites (mais conservador) */
    SCurveProfile prof;
    scurve_plan(dist, 0, speed_pct, &prof);
    if (!prof.valid) return false;

    const float T_total = (prof.t7 < 0.01f) ? 0.01f : prof.t7;
    const int   slices  = (int)ceilf(T_total * 1000.0f / (float)SEG_SLICE_MS);
    const int   head_save = g_seg_head;

    const float sx = g_current_pose.x, sy = g_current_pose.y, sz = g_current_pose.z;
    const float srx= g_current_pose.rx,sry= g_current_pose.ry,srz= g_current_pose.rz;

    for (int s = 1; s <= slices; ++s) {
        const float tau    = (float)s / (float)slices;
        const float factor = scurve_sample(tau, &prof);

        const float px = sx  + (target->x  - sx)  * factor;
        const float py = sy  + (target->y  - sy)  * factor;
        const float pz = sz  + (target->z  - sz)  * factor;
        const float rrx= srx + (target->rx - srx) * factor;
        const float rry= sry + (target->ry - sry) * factor;
        const float rrz= srz + (target->rz - srz) * factor;

        Mat3 R_slice;
        euler_deg_to_rotation(rrx, rry, rrz, R_slice);

        float q_rad[6], p_slice[3] = { px, py, pz };
        if (!inverse_kinematics(p_slice, R_slice, q_rad)) {
            /* Desfaz segmentos já enfileirados (aborta trajetória parcial) */
            g_seg_head = head_save;
            return false;
        }

        float q_deg[6];
        for (int i = 0; i < 6; ++i)
            q_deg[i] = q_rad[i] * (180.0f / (float)M_PI);

        if (!check_soft_limits(q_deg)) {
            g_seg_head = head_save;
            return false;
        }

        while (!push_segment(q_deg, (uint32_t)SEG_SLICE_MS))
            EB15_DELAY_MS(2);
    }

    return true;
}
