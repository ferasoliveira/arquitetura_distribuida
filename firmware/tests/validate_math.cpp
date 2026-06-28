/**
 * validate_math.cpp — Teste nativo de corretude matemática (Parte A)
 *
 * Compila com g++ em qualquer host:
 *   g++ -O2 -std=c++14 -DEB15_NATIVE_TEST -I../esp32_mestre/main \
 *       -o validate_math validate_math.cpp && ./validate_math
 *
 * Ou via script: Códigos/firmware/tests/run_tests.ps1
 *
 * Critérios de aceite (plano_passo2.md):
 *   1. FK->IK->FK: 200 ciclos com erro < 1e-3 mm e < 1e-4 rad (matriz rotação)
 *   2. S-Curve monotônica e contínua para 7 distâncias
 *   3. scurve_sample(0)==0, scurve_sample(1)==1
 *   4. static_assert(sizeof(UnoFrame)==10)
 */

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Ativa modo de teste nativo: desativa dependências de plataforma */
#define EB15_NATIVE_TEST 1

/* config.h não depende de Arduino.h — inclui diretamente */
#include "../esp32_mestre/main/config.h"
#include "../esp32_mestre/main/kinematics.h"
#include "../esp32_mestre/main/trajectory.h"
#include "../esp32_mestre/main/pid_tanh.h"

/* Instâncias globais exigidas por trajectory.h */
TrajectorySegment g_segments[MAX_SEGMENTS];
volatile int g_seg_head = 0;
volatile int g_seg_tail = 0;
float g_joint_angles[6] = {};
CartesianPose g_current_pose = {};

// ============================================================================
// Utilidades de teste
// ============================================================================

static int failures = 0;
static int total    = 0;

static void check(bool ok, const char *desc) {
    ++total;
    if (!ok) {
        ++failures;
        fprintf(stderr, "  FAIL: %s\n", desc);
    }
}

static float mat3_max_diff(const Mat3 A, const Mat3 B) {
    float m = 0.f;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            m = std::max(m, fabsf(A[i][j] - B[i][j]));
    return m;
}

// ============================================================================
// Teste 1: FK -> IK -> FK  (200 ciclos)
// ============================================================================

static void test_fk_ik_fk(void) {
    printf("=== Teste 1: FK->IK->FK (200 amostras) ===\n");
    int local_fail = 0;

    for (int n = 0; n < 200; ++n) {
        /* Ângulos pseudo-aleatórios em radianos — cobre vários quadrantes */
        float q[6] = {
            -2.0f + 4.0f * n / 199.0f,
            -0.5f + 1.5f * ((n * 37) % 199) / 199.0f,
            -0.1f - 1.7f * ((n * 71) % 199) / 199.0f,
            -2.0f + 4.0f * ((n * 29) % 199) / 199.0f,
             0.15f + 2.5f * ((n * 43) % 199) / 199.0f,
            -2.5f + 5.0f * ((n * 61) % 199) / 199.0f,
        };

        float p[3], p2[3], qi[6];
        Mat3  R, R2;

        forward_kinematics(q, p, R);

        if (!inverse_kinematics(p, R, qi)) {
            ++local_fail;
            fprintf(stderr, "  [n=%d] IK falhou (ponto inalcancavel)\n", n);
            continue;
        }

        forward_kinematics(qi, p2, R2);

        const float pe = hypotf(hypotf(p[0]-p2[0], p[1]-p2[1]), p[2]-p2[2]);
        const float re = mat3_max_diff(R, R2);

        if (pe > 1e-3f) {
            ++local_fail;
            fprintf(stderr, "  [n=%d] Erro posicao: %.6f mm (limite 1e-3 mm)\n", n, pe);
        }
        if (re > 1e-4f) {
            ++local_fail;
            fprintf(stderr, "  [n=%d] Erro rotacao: %.6f (limite 1e-4)\n", n, re);
        }
    }

    char desc[80];
    snprintf(desc, sizeof(desc), "FK->IK->FK 200 ciclos sem erro (falhas=%d)", local_fail);
    check(local_fail == 0, desc);
}

// ============================================================================
// Teste 2: S-Curve monotônica e contínua
// ============================================================================

static void test_scurve(void) {
    printf("=== Teste 2: S-Curve monotonica e continua ===\n");
    const float dists[] = { 0.1f, 1.0f, 5.0f, 10.0f, 45.0f, 90.0f, 180.0f };
    const int N = 10001;

    for (int di = 0; di < 7; ++di) {
        const float d = dists[di];
        SCurveProfile prof;
        scurve_plan(d, 0, 100.0f, &prof);

        char desc[128];

        /* Deve ser válido */
        snprintf(desc, sizeof(desc), "S-curve valida para d=%.1f deg", d);
        check(prof.valid, desc);
        if (!prof.valid) continue;

        /* scurve_sample(0) == 0 */
        const float s0 = scurve_sample(0.0f, &prof);
        snprintf(desc, sizeof(desc), "sample(0)=0 para d=%.1f deg (got %.6f)", d, s0);
        check(fabsf(s0) < 1e-5f, desc);

        /* scurve_sample(1) == 1 */
        const float s1 = scurve_sample(1.0f, &prof);
        snprintf(desc, sizeof(desc), "sample(1)=1 para d=%.1f deg (got %.6f)", d, s1);
        check(fabsf(s1 - 1.0f) < 1e-4f, desc);

        /* Monotonicidade: sample[i+1] >= sample[i] */
        float last = -1.0f;
        int mono_fail = 0;
        for (int i = 0; i <= N; ++i) {
            const float tau = (float)i / (float)N;
            const float s   = scurve_sample(tau, &prof);
            if (!std::isfinite(s)) { ++mono_fail; break; }
            if (s < last - 1e-6f) { ++mono_fail; }
            last = s;
        }
        snprintf(desc, sizeof(desc), "S-curve monotonica d=%.1f deg (viol=%d)", d, mono_fail);
        check(mono_fail == 0, desc);
    }
}

// ============================================================================
// Teste 3: S-Curve casos especiais
// ============================================================================

static void test_scurve_edge(void) {
    printf("=== Teste 3: S-Curve casos especiais ===\n");
    SCurveProfile prof;

    /* Distância nula → inválida */
    scurve_plan(0.0f, 0, 100.0f, &prof);
    check(!prof.valid, "S-curve d=0 deg => invalid");

    /* Distância negativa → válida, total_dist negativo */
    scurve_plan(-45.0f, 0, 100.0f, &prof);
    check(prof.valid && prof.total_dist < 0.0f, "S-curve d=-45 deg => valid, total_dist<0");
    if (prof.valid) {
        const float s = scurve_sample(0.5f, &prof);
        check(s >= 0.0f && s <= 1.0f, "S-curve negativa: sample em [0,1]");
    }

    /* d=90 deg: duração positiva e finita */
    scurve_plan(90.0f, 0, 100.0f, &prof);
    check(prof.valid && prof.t7 > 0.0f && std::isfinite(prof.t7),
          "S-curve d=90 deg tem duracao finita positiva");
}

// ============================================================================
// Teste 4: static_assert do UnoFrame
// ============================================================================

static void test_frame_size(void) {
    printf("=== Teste 4: UnoFrame = 10 bytes ===\n");
    check(sizeof(UnoFrame) == 10, "sizeof(UnoFrame) == 10");
}

// ============================================================================
// Teste 5: S-Curve — limites dinâmicos respeitados
// Verifica que velocidade, aceleração e jerk nunca excedem os limites para
// todas as 7 distâncias de referência. Cobre o caso d=20° (bug intermediate).
// ============================================================================

static void test_scurve_dynamic_limits(void) {
    printf("=== Teste 5: S-Curve limites dinamicos ===\n");
    /* Inclui 20 deg — caso que revelou bug na fórmula intermediária (vp=50.29 > Vmax=45). */
    const float dists[] = { 0.1f, 1.0f, 5.0f, 10.0f, 20.0f, 30.0f, 45.0f, 90.0f, 180.0f };
    const int   NDIST   = 9;
    const int   N       = 10001;

    for (int di = 0; di < NDIST; ++di) {
        const float d_target = dists[di];
        SCurveProfile prof;
        scurve_plan(d_target, 0, 100.0f, &prof);
        if (!prof.valid) continue;

        const float V_max = MAX_SPEED_DEG_S[0];
        const float A_max = MAX_ACCEL_DEG_S2[0];
        const float J_max = MAX_JERK_DEG_S3[0];

        /* 1. Verificação analítica direta dos parâmetros do perfil.
         *    prof.a_max = J*tj (derivado da fórmula, sempre ≤ A se correto).
         *    prof.v_max com sinal = velocidade de pico assinada. */
        char desc[128];
        snprintf(desc, sizeof(desc), "d=%.1f: |v_max|=%.4f <= Vmax=%.1f", d_target,
                 fabsf(prof.v_max), V_max);
        check(fabsf(prof.v_max) <= V_max * 1.001f, desc);

        snprintf(desc, sizeof(desc), "d=%.1f: a_max=%.4f <= Amax=%.1f", d_target,
                 prof.a_max, A_max);
        check(prof.a_max <= A_max * 1.001f, desc);

        snprintf(desc, sizeof(desc), "d=%.1f: j_max=%.4f == Jmax=%.1f", d_target,
                 prof.j_max, J_max);
        check(fabsf(prof.j_max - J_max) < 1.0f, desc);

        /* 2. Verificação numérica via scurve_sample: velocidade de pico amostrada ≤ Vmax.
         *    Usa derivada de primeira ordem — menos ruidosa que segunda ordem. */
        float v_peak = 0.0f;
        float dt_fine = prof.t7 / (float)N;
        float x_prev = 0.0f;
        float x_final = 0.0f;
        for (int i = 1; i <= N; ++i) {
            const float x = scurve_sample((float)i / (float)N, &prof) * fabsf(prof.total_dist);
            const float v = fabsf(x - x_prev) / dt_fine;
            if (v > v_peak) v_peak = v;
            x_prev = x;
            x_final = x;
        }
        snprintf(desc, sizeof(desc), "d=%.1f: v_amostrada=%.4f <= Vmax=%.1f", d_target,
                 v_peak, V_max);
        check(v_peak <= V_max * 1.02f, desc);  /* 2% para arredondamento de float */

        /* 3. Distância final = d_target (±0.001 deg) */
        snprintf(desc, sizeof(desc), "d=%.1f: dist_final=%.5f (alvo=%.5f)", d_target,
                 x_final, d_target);
        check(fabsf(x_final - d_target) < 1e-3f, desc);
    }
}

// ============================================================================
// Teste 6: DDA zero-step → done imediato
// Simula o comportamento do Dda3Axis quando todos os passos são 0.
// ============================================================================

/* Stub mínimo do Dda3Axis para teste nativo (sem hardware AVR) */
struct DdaStub {
    uint16_t steps_[3], accum_[3], total_ticks_, tick_idx_;
    bool armed_, running_, done_;

    void clear() {
        for(int i=0;i<3;i++){steps_[i]=accum_[i]=0;}
        total_ticks_=tick_idx_=0;
        armed_=running_=done_=false;
    }
    DdaStub() { clear(); }

    void arm(int s0, int s1, int s2) {
        clear();
        steps_[0]=(uint16_t)(s0<0?-s0:s0);
        steps_[1]=(uint16_t)(s1<0?-s1:s1);
        steps_[2]=(uint16_t)(s2<0?-s2:s2);
        for(int i=0;i<3;i++) if(steps_[i]>total_ticks_) total_ticks_=steps_[i];
        for(int i=0;i<3;i++) accum_[i]=steps_[i]?(total_ticks_-steps_[i]):0;
        armed_=true;
    }

    /* Espelho exato do método Dda3Axis::start() corrigido */
    void start() {
        if (!armed_) return;
        if (total_ticks_ == 0) { armed_=false; done_=true; return; }
        running_=true;
    }

    bool takeDone() { bool r=done_; done_=false; return r; }
    bool busy()     { return armed_||running_; }
    bool running()  { return running_; }
};

static void test_dda_zero_step(void) {
    printf("=== Teste 6: DDA zero-step => done imediato ===\n");
    DdaStub dda;

    /* Arm com 0 passos em todos os eixos */
    dda.arm(0, 0, 0);
    check(dda.busy(), "DDA armado com 0 passos: busy()=true");
    check(!dda.running(), "DDA armado com 0 passos: running()=false");

    /* start() deve setar done_ e limpar armed_ */
    dda.start();
    check(!dda.running(), "DDA start() com 0 passos: running()=false");
    check(!dda.busy(),    "DDA start() com 0 passos: busy()=false (armed cleared)");
    check(dda.takeDone(), "DDA start() com 0 passos: takeDone()=true");

    /* Arm com passos reais: done_ não deve ser setado por start() */
    dda.arm(10, 5, 0);
    dda.start();
    check(dda.running(),   "DDA com passos: running()=true apos start()");
    check(!dda.takeDone(), "DDA com passos: done_ nao setado por start()");
}

// ============================================================================
// Teste 7: Simulação PID+Tanh — convergência sem oscilação permanente
// ============================================================================

static void test_pid_convergence(void) {
    printf("=== Teste 7: PID+Tanh convergencia ===\n");

    const float targets[] = { 0.5f, 1.0f, 10.0f, 45.0f, 90.0f };
    const int N_TARGETS = 5;
    const float dt = 1.0f / CONTROL_HZ;
    const float step_deg = 1.0f / STEPS_PER_DEG[0];  /* 1 passo em graus */

    for (int ti = 0; ti < N_TARGETS; ++ti) {
        const float target = targets[ti];
        PIDState state = {};
        float pos = 0.0f, accum = 0.0f;
        bool converged = false;
        int settle_tick = -1;
        int oscs = 0;
        float prev_err = target;

        for (int t = 0; t < 4000; ++t) {  /* max 20 s @ 200 Hz */
            const float f_hz = pid_tanh_update(0, target, pos, dt, &state);
            int pulses; bool dir;
            pid_step_pulses(f_hz, dt, &accum, &pulses, &dir);
            const float delta = (dir ? 1.0f : -1.0f) * (float)pulses * step_deg;
            pos += delta;

            const float err = target - pos;
            /* Conta inversões de sinal do erro → oscilações */
            if (err * prev_err < 0.0f) ++oscs;
            prev_err = err;

            if (fabsf(err) <= PID_DEADBAND_DEG && settle_tick < 0) {
                settle_tick = t;
                converged = true;
            }
        }

        char desc[128];
        snprintf(desc, sizeof(desc),
                 "PID converge para %.1f deg (settle=%d ticks, oscs=%d)", target, settle_tick, oscs);
        check(converged, desc);

        snprintf(desc, sizeof(desc),
                 "PID oscilacoes <= 4 para alvo %.1f deg (oscs=%d)", target, oscs);
        check(oscs <= 4, desc);
    }
}

// ============================================================================
// Teste 8: plan_move_j_scurve — segmentos gerados corretamente
// ============================================================================

static void test_plan_segments(void) {
    printf("=== Teste 8: plan_move_j_scurve segmentos ===\n");

    /* Reset da fila */
    g_seg_head = g_seg_tail = 0;
    for (int i = 0; i < 6; ++i) g_joint_angles[i] = 0.0f;

    const float target[6] = { 30.0f, 20.0f, -15.0f, 45.0f, 0.0f, 0.0f };
    plan_move_j_scurve(target, 50.0f);

    const int n_segs = (g_seg_head - g_seg_tail + MAX_SEGMENTS) % MAX_SEGMENTS;
    check(n_segs > 0, "plan_move_j_scurve gerou ao menos 1 segmento");
    check(n_segs <= MAX_SEGMENTS - 1, "plan_move_j_scurve nao estourou a fila");

    if (n_segs > 0) {
        /* Último segmento deve aproximar o alvo */
        int last_idx = (g_seg_head - 1 + MAX_SEGMENTS) % MAX_SEGMENTS;
        const TrajectorySegment *last = &g_segments[last_idx];
        for (int i = 0; i < 6; ++i) {
            char desc[128];
            snprintf(desc, sizeof(desc),
                     "Ultimo segmento J%d: %.3f (alvo %.3f)", i+1, last->q_deg[i], target[i]);
            check(fabsf(last->q_deg[i] - target[i]) < 0.01f, desc);
        }
        check(last->duration_ms == SEG_SLICE_MS, "Duracao do segmento = SEG_SLICE_MS");
    }
}

// ============================================================================
// main
// ============================================================================

int main(void) {
    printf("EB15 Mestre — validate_math\n");
    printf("============================\n");

    test_fk_ik_fk();
    test_scurve();
    test_scurve_edge();
    test_frame_size();
    test_scurve_dynamic_limits();
    test_dda_zero_step();
    test_pid_convergence();
    test_plan_segments();

    printf("============================\n");
    if (failures == 0) {
        printf("PASS: %d/%d verificacoes aprovadas\n", total, total);
        return 0;
    } else {
        printf("FAIL: %d/%d verificacoes falharam\n", failures, total);
        return 1;
    }
}
