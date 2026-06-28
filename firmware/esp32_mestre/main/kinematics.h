/**
 * EB-15 Robotic Arm — Mestre (ESP32-S3)
 * kinematics.h — Cinemática analítica fechada (Pieper)
 *
 * CONVENÇÃO ÚNICA usada por FK e IK:
 *   Base (3R planar rotacionado):
 *     - J1: rotação em torno de Z global (Rz)
 *     - J2: rotação em torno de Y (Ry) no plano vertical
 *     - J3: rotação em torno de Y relativo a J2 (Ry)
 *     Wrist center: wc = [cos(q1)*R, sin(q1)*R, L1 + L2*sin(q2) + L3*sin(q2+q3)]
 *       onde R = L2*cos(q2) + L3*cos(q2+q3)
 *
 *   Punho esférico (centro coincidente):
 *     R_total = Rz(q1) * Ry(q2+q3) * Rz(q4) * Ry(q5) * Rz(q6)
 *
 *   TCP: p = wc + L6 * R_total[:,2]
 *
 * IK: desacoplamento por Pieper (wrist center ← 3R ← Euler Z-Y-Z).
 *
 * Unidades: todas as funções operam em RADIANOS internamente.
 *           A conversão graus↔rad é responsabilidade do chamador.
 */
#pragma once

#include <math.h>
#include <stdbool.h>
#include "config.h"

typedef float Mat3[3][3];

/* --- Rotações elementares --- */

static inline void mat3_mul(const Mat3 A, const Mat3 B, Mat3 C) {
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            C[i][j] = 0.0f;
            for (int k = 0; k < 3; ++k) C[i][j] += A[i][k] * B[k][j];
        }
}

static inline void rot_z(float q, Mat3 R) {
    const float c = cosf(q), s = sinf(q);
    R[0][0]= c; R[0][1]=-s; R[0][2]=0;
    R[1][0]= s; R[1][1]= c; R[1][2]=0;
    R[2][0]= 0; R[2][1]= 0; R[2][2]=1;
}

static inline void rot_y(float q, Mat3 R) {
    const float c = cosf(q), s = sinf(q);
    R[0][0]= c; R[0][1]=0; R[0][2]= s;
    R[1][0]= 0; R[1][1]=1; R[1][2]= 0;
    R[2][0]=-s; R[2][1]=0; R[2][2]= c;
}

/* Extrai ângulos de Euler XYZ (rotação intrínseca) da matriz R.
 * Usado apenas para debug/log; a IK não precisa desta função. */
static inline void rotation_to_euler_deg(const Mat3 R, float *rx, float *ry, float *rz) {
    *ry = asinf(constrainf(-R[2][0], -1.0f, 1.0f));
    if (fabsf(cosf(*ry)) > 1e-6f) {
        *rz = atan2f( R[1][0],  R[0][0]);
        *rx = atan2f( R[2][1],  R[2][2]);
    } else {
        *rz = 0.0f;
        *rx = atan2f(-R[1][2],  R[1][1]);
    }
    const float k = 180.0f / (float)M_PI;
    *rx *= k; *ry *= k; *rz *= k;
}

/* Constrói R a partir de ângulos de Euler XYZ em graus. */
static inline void euler_deg_to_rotation(float rx_d, float ry_d, float rz_d, Mat3 R) {
    const float k = (float)M_PI / 180.0f;
    const float rx = rx_d*k, ry = ry_d*k, rz = rz_d*k;
    const float cx = cosf(rx), sx = sinf(rx);
    const float cy = cosf(ry), sy = sinf(ry);
    const float cz = cosf(rz), sz = sinf(rz);
    R[0][0]= cz*cy;        R[0][1]= cz*sy*sx - sz*cx;  R[0][2]= cz*sy*cx + sz*sx;
    R[1][0]= sz*cy;        R[1][1]= sz*sy*sx + cz*cx;  R[1][2]= sz*sy*cx - cz*sx;
    R[2][0]= -sy;          R[2][1]= cy*sx;              R[2][2]= cy*cx;
}

/* ============================================================================
 * Cinemática Direta (FK)
 * Entrada : q[6] em radianos
 * Saída   : p[3] posição TCP em mm, R orientação do TCP
 * ============================================================================ */
static void forward_kinematics(const float q[6], float p[3], Mat3 R) {
    const float q23    = q[1] + q[2];
    const float radial = LINK_L2 * cosf(q[1]) + LINK_L3 * cosf(q23);

    /* wrist center */
    const float wc[3] = {
        cosf(q[0]) * radial,
        sinf(q[0]) * radial,
        LINK_L1 + LINK_L2 * sinf(q[1]) + LINK_L3 * sinf(q23)
    };

    /* R_total = Rz(q0) * Ry(q1+q2) * Rz(q3) * Ry(q4) * Rz(q5) */
    Mat3 Rz1, Ry23, R03, Rz4, Ry5, Rz6, tmp, R36;
    rot_z(q[0], Rz1);  rot_y(q23, Ry23);
    mat3_mul(Rz1, Ry23, R03);

    rot_z(q[3], Rz4);  rot_y(q[4], Ry5);  rot_z(q[5], Rz6);
    mat3_mul(Rz4, Ry5, tmp);
    mat3_mul(tmp, Rz6, R36);
    mat3_mul(R03, R36, R);

    /* TCP = wc + L6 * terceira coluna de R */
    for (int i = 0; i < 3; ++i) p[i] = wc[i] + LINK_L6 * R[i][2];
}

/* ============================================================================
 * Cinemática Inversa (IK) — Pieper, cotovelo abaixo, punho esférico
 * Entrada : p[3] posição TCP mm, R orientação desejada
 * Saída   : q[6] em radianos
 * Retorna : true se solução encontrada; false se fora do espaço de trabalho
 * ============================================================================ */
static bool inverse_kinematics(const float p[3], const Mat3 R, float q[6]) {
    /* 1. Wrist center (desacoplamento de Pieper) */
    const float wx = p[0] - LINK_L6 * R[0][2];
    const float wy = p[1] - LINK_L6 * R[1][2];
    const float wz = p[2] - LINK_L6 * R[2][2];

    /* 2. Base: 3R planar */
    const float r = hypotf(wx, wy);
    const float z = wz - LINK_L1;

    if (r < 1e-6f) return false;   /* singularidade no eixo Z */

    const float c3_raw = (r*r + z*z - LINK_L2*LINK_L2 - LINK_L3*LINK_L3)
                         / (2.0f * LINK_L2 * LINK_L3);
    if (c3_raw < -1.00001f || c3_raw > 1.00001f) return false; /* fora do alcance */

    const float c3 = constrainf(c3_raw, -1.0f, 1.0f);

    q[0] = atan2f(wy, wx);
    q[2] = -acosf(c3);   /* cotovelo abaixo (q3 negativo) */
    q[1] = atan2f(z, r) - atan2f(LINK_L3 * sinf(q[2]),
                                   LINK_L2 + LINK_L3 * cosf(q[2]));

    /* 3. Wrist: R36 = R03^T * R (rotação do frame 3 ao TCP) */
    Mat3 Rz1, Ry23, R03, R36;
    rot_z(q[0], Rz1);
    rot_y(q[1] + q[2], Ry23);
    mat3_mul(Rz1, Ry23, R03);

    /* R36[i][j] = sum_k R03[k][i] * R[k][j]  (R03 transposta × R) */
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            R36[i][j] = 0.0f;
            for (int k = 0; k < 3; ++k) R36[i][j] += R03[k][i] * R[k][j];
        }

    /* 4. Euler Z-Y-Z do punho: Rz(q4)*Ry(q5)*Rz(q6)
     *   R36[2][2] = cos(q5)
     *   R36[1][2] = sin(q4)*sin(q5)
     *   R36[0][2] = cos(q4)*sin(q5)
     *   R36[2][1] = sin(q5)*sin(q6)
     *   R36[2][0] = -sin(q5)*cos(q6) */
    q[4] = acosf(constrainf(R36[2][2], -1.0f, 1.0f));

    if (fabsf(sinf(q[4])) > 1e-5f) {
        q[3] = atan2f( R36[1][2],  R36[0][2]);
        q[5] = atan2f( R36[2][1], -R36[2][0]);
    } else {
        /* singularidade do punho: q4=0 ou q4=π — distribui em q3 */
        q[3] = 0.0f;
        q[5] = atan2f(R36[1][0], R36[0][0]);
    }

    return true;
}

/* Verifica soft limits (recebe ângulos em GRAUS) */
static inline bool check_soft_limits(const float q_deg[6]) {
    for (int i = 0; i < 6; ++i)
        if (q_deg[i] < LIMIT_MIN_DEG[i] || q_deg[i] > LIMIT_MAX_DEG[i]) return false;
    return true;
}
