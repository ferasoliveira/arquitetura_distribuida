"""
trajectory.py — Trajetórias Canônicas do EB15
==============================================
TCC Fernando Oliveira — Passo 3: Ambiente Virtual, Pontes e Simulação

Define e exporta as trajetórias canônicas neutras utilizadas pelo
Adaptador Geral de Benchmark.

CONTRATO (Fase 1):
  - Uma única implementação de carregamento, validação e temporização.
  - As mesmas trajetórias são enviadas aos dois destinos (EB15/Webots e URSim).
  - Formato de saída: lista de dicionários com campos padronizados.
  - Unidades: graus para juntas, milissegundos para tempo.
  - Velocidades e acelerações são calculadas aqui (não nos drivers).

Trajetórias incluídas:
  1. SINGLE_JOINT  — Movimento isolado de J1 (±30°), validação de junta única
  2. ALL_JOINTS    — Movimento simultâneo de J1–J6 (excursão máxima segura)
  3. REPEATED_ABS  — Alvo absoluto repetido (test de repetibilidade)
  4. MOVE_J_SHORT  — Trajetória MoveJ curta (sequência de 5 posições)
  5. CANONICAL_1   — Trajetória canônica de benchmark (10 pontos, S-Curve)
"""

import math
import logging
from typing import List, Dict, Any, Optional
from copy import deepcopy

logger = logging.getLogger(__name__)

# ============================================================================
# Tipos e estruturas
# ============================================================================

# Um waypoint da trajetória
# {
#   "seq": int,          # Índice de sequência (0-based)
#   "t_planned": float,  # Tempo planejado desde t0 (ms)
#   "q": [f]*6,          # Ângulos alvo J1–J6 em graus
#   "qd": [f]*6,         # Velocidade em cada junta (deg/s)
#   "qdd": [f]*6,        # Aceleração em cada junta (deg/s²)
#   "duration_ms": int,  # Duração deste segmento (ms)
#   "label": str,        # Rótulo descritivo opcional
# }
Waypoint = Dict[str, Any]
Trajectory = Dict[str, Any]

# Limites de junta EB15 (graus)
_JMIN = [-170.0, -45.0, -120.0, -180.0, -90.0, -360.0]
_JMAX = [ 170.0, 180.0,  120.0,  180.0,  90.0,  360.0]

# Velocidades máximas (deg/s)
_VMAX = [45.0, 45.0, 45.0, 90.0, 90.0, 90.0]

# Acelerações máximas (deg/s²)
_AMAX = [90.0, 90.0, 90.0, 180.0, 180.0, 180.0]


# ============================================================================
# Utilitários internos
# ============================================================================

def _validate_waypoint(wp: Waypoint) -> bool:
    """Valida se um waypoint está dentro dos limites articulares do EB15."""
    q = wp.get("q", [])
    if len(q) != 6:
        return False
    for i, deg in enumerate(q):
        if not (_JMIN[i] <= deg <= _JMAX[i]):
            logger.warning(
                "Waypoint seq=%d: J%d=%.1f° fora do limite [%.1f, %.1f]",
                wp.get("seq", -1), i + 1, deg, _JMIN[i], _JMAX[i]
            )
            return False
    return True


def _estimate_duration(q_start: list, q_end: list, speed_pct: float = 50.0) -> int:
    """
    Estima a duração de um segmento baseado na maior excursão angular.

    speed_pct : float
        Percentual da velocidade máxima (1–100).

    Retorna
    -------
    int : Duração em milissegundos (mínimo 100 ms).
    """
    max_dur = 100
    for i in range(6):
        delta   = abs(q_end[i] - q_start[i])
        eff_spd = _VMAX[i] * (speed_pct / 100.0)
        if eff_spd > 0 and delta > 0:
            dur_ms = int((delta / eff_spd) * 1000.0)
            max_dur = max(max_dur, dur_ms)
    return max_dur


def _compute_derivatives(waypoints: List[Waypoint]) -> List[Waypoint]:
    """
    Calcula velocidades (qd) e acelerações (qdd) por diferenças finitas.

    Velocidade: diferença centrada (exceto nas extremidades).
    Aceleração: diferença centrada de segunda ordem.
    """
    n = len(waypoints)
    if n < 2:
        return waypoints

    result = deepcopy(waypoints)

    # Velocidade por diferença centrada
    for i in range(n):
        dt_ms = waypoints[i].get("duration_ms", 200)
        dt    = dt_ms / 1000.0  # segundos

        if i == 0:
            q0 = waypoints[0]["q"]
            q1 = waypoints[1]["q"]
            dt1 = waypoints[0].get("duration_ms", 200) / 1000.0
            result[i]["qd"] = [(q1[j] - q0[j]) / max(dt1, 0.001) for j in range(6)]
        elif i == n - 1:
            qn1 = waypoints[i - 1]["q"]
            qn  = waypoints[i]["q"]
            dt1 = waypoints[i].get("duration_ms", 200) / 1000.0
            result[i]["qd"] = [(qn[j] - qn1[j]) / max(dt1, 0.001) for j in range(6)]
        else:
            q_prev = waypoints[i - 1]["q"]
            q_next = waypoints[i + 1]["q"]
            dt_prev = waypoints[i - 1].get("duration_ms", 200) / 1000.0
            dt_next = waypoints[i].get("duration_ms", 200) / 1000.0
            total_dt = dt_prev + dt_next
            result[i]["qd"] = [
                (q_next[j] - q_prev[j]) / max(total_dt, 0.001)
                for j in range(6)
            ]

    # Aceleração por diferença centrada das velocidades
    for i in range(n):
        if i == 0:
            result[i]["qdd"] = [
                (result[1]["qd"][j] - result[0]["qd"][j])
                / max(result[0].get("duration_ms", 200) / 1000.0, 0.001)
                for j in range(6)
            ]
        elif i == n - 1:
            result[i]["qdd"] = [
                (result[i]["qd"][j] - result[i-1]["qd"][j])
                / max(result[i].get("duration_ms", 200) / 1000.0, 0.001)
                for j in range(6)
            ]
        else:
            dt_prev = result[i - 1].get("duration_ms", 200) / 1000.0
            dt_next = result[i].get("duration_ms", 200) / 1000.0
            total_dt = dt_prev + dt_next
            result[i]["qdd"] = [
                (result[i+1]["qd"][j] - result[i-1]["qd"][j])
                / max(total_dt, 0.001)
                for j in range(6)
            ]

    return result


def _build_trajectory(name: str, description: str,
                       waypoints_raw: List[List[float]],
                       speed_pct: float = 50.0) -> Trajectory:
    """
    Constrói uma trajetória a partir de uma lista de ângulos alvo.

    Parâmetros
    ----------
    name : str
        Identificador único da trajetória.
    description : str
        Descrição textual para documentação.
    waypoints_raw : list of list[6 floats]
        Lista de posições angulares [J1, J2, J3, J4, J5, J6] em graus.
    speed_pct : float
        Percentual da velocidade máxima.

    Retorna
    -------
    Trajectory dict com campos:
      name, description, speed_pct, waypoints (lista de Waypoint)
    """
    waypoints = []
    t_acc     = 0.0

    for idx, q in enumerate(waypoints_raw):
        q_prev = waypoints_raw[idx - 1] if idx > 0 else q
        dur    = _estimate_duration(q_prev, q, speed_pct) if idx > 0 else 0

        wp = {
            "seq":         idx,
            "t_planned":   t_acc,
            "q":           list(q),
            "qd":          [0.0] * 6,
            "qdd":         [0.0] * 6,
            "duration_ms": dur,
            "label":       f"wp_{idx:02d}",
        }

        if not _validate_waypoint(wp):
            raise ValueError(f"Trajetória '{name}': waypoint {idx} inválido (fora dos limites)")

        waypoints.append(wp)
        t_acc += dur

    # Calcula derivadas
    waypoints = _compute_derivatives(waypoints)

    total_duration = sum(wp["duration_ms"] for wp in waypoints)

    logger.info(
        "Trajetória '%s': %d waypoints | duração total = %.1f s",
        name, len(waypoints), total_duration / 1000.0
    )

    return {
        "name":           name,
        "description":    description,
        "speed_pct":      speed_pct,
        "total_ms":       total_duration,
        "waypoints":      waypoints,
    }


# ============================================================================
# Trajetórias canônicas
# ============================================================================

def traj_single_joint() -> Trajectory:
    """
    Trajetória 1 — Movimento isolado de J1 (±30°).

    Valida o mapeamento de sinal e unidade de J1 de forma isolada.
    Todas as outras juntas permanecem em 0°.
    """
    return _build_trajectory(
        name        = "SINGLE_JOINT_J1",
        description = "Movimento isolado de J1: 0→+30→0→-30→0 graus. "
                      "Valida mapeamento de sinal e unidade de J1.",
        waypoints_raw = [
            [  0.0,  0.0,  0.0,  0.0,  0.0,  0.0],  # Home
            [ 30.0,  0.0,  0.0,  0.0,  0.0,  0.0],  # +30°
            [  0.0,  0.0,  0.0,  0.0,  0.0,  0.0],  # Home
            [-30.0,  0.0,  0.0,  0.0,  0.0,  0.0],  # -30°
            [  0.0,  0.0,  0.0,  0.0,  0.0,  0.0],  # Home
        ],
        speed_pct = 40.0,
    )


def traj_all_joints() -> Trajectory:
    """
    Trajetória 2 — Movimento simultâneo de J1–J6.

    Excursão máxima dentro dos limites do EB15.
    Valida a sincronização entre J1–J3 (ESP32) e J4–J6 (Arduino).
    """
    return _build_trajectory(
        name        = "ALL_JOINTS_SYNC",
        description = "Movimento simultâneo J1–J6. "
                      "Valida sincronização entre ESP32 (J1–J3) e Arduino (J4–J6).",
        waypoints_raw = [
            [ 0.0,   0.0,   0.0,   0.0,   0.0,   0.0],
            [30.0,  30.0,  30.0,  45.0,  30.0,  90.0],
            [-30.0, -30.0, -30.0, -45.0, -30.0, -90.0],
            [  0.0,   0.0,   0.0,   0.0,   0.0,   0.0],
        ],
        speed_pct = 35.0,
    )


def traj_repeated_abs() -> Trajectory:
    """
    Trajetória 3 — Alvo absoluto repetido 5 vezes.

    Valida a repetibilidade e ausência de acumulação indevida de erro.
    O mesmo alvo absoluto é enviado N vezes consecutivas.
    """
    home   = [ 0.0,  0.0,  0.0,  0.0,  0.0,  0.0]
    target = [20.0, 20.0, 20.0, 45.0, 30.0, 60.0]

    pts = [home]
    for _ in range(5):
        pts.append(target)
        pts.append(home)

    return _build_trajectory(
        name        = "REPEATED_ABS",
        description = "Alvo absoluto repetido 5×. "
                      "Valida repetibilidade sem acumulação de erro.",
        waypoints_raw = pts,
        speed_pct   = 40.0,
    )


def traj_move_j_short() -> Trajectory:
    """
    Trajetória 4 — MoveJ curta de 5 posições.

    Trajetória compacta de referência para validação básica de benchmark.
    """
    return _build_trajectory(
        name        = "MOVE_J_SHORT",
        description = "Trajetória MoveJ de 5 posições (benchmark básico).",
        waypoints_raw = [
            [  0.0,   0.0,   0.0,   0.0,   0.0,   0.0],
            [ 20.0,  10.0,  15.0,  20.0,  10.0,  30.0],
            [-20.0,  20.0, -10.0, -20.0,  20.0, -30.0],
            [ 10.0, -10.0,  20.0,  10.0, -10.0,  45.0],
            [  0.0,   0.0,   0.0,   0.0,   0.0,   0.0],
        ],
        speed_pct = 45.0,
    )


def traj_canonical_benchmark() -> Trajectory:
    """
    Trajetória 5 — Canônica de benchmark (10 waypoints).

    Trajetória principal para comparação EB15/Webots vs. URSim.
    Cobre excursões moderadas de todas as juntas com retorno ao home.
    """
    return _build_trajectory(
        name        = "CANONICAL_BENCHMARK",
        description = "Trajetória canônica de 10 waypoints para benchmark "
                      "comparativo EB15/Webots vs. URSim via RTDE.",
        waypoints_raw = [
            [  0.0,   0.0,   0.0,   0.0,   0.0,   0.0],   # Home
            [ 10.0,   5.0,   5.0,  10.0,   5.0,  15.0],   # P1
            [ 20.0,  10.0,  10.0,  20.0,  10.0,  30.0],   # P2
            [ 30.0,  15.0,  15.0,  30.0,  15.0,  45.0],   # P3 (pico)
            [ 20.0,  10.0,  10.0,  20.0,  10.0,  30.0],   # P4
            [  0.0,   0.0,   0.0,   0.0,   0.0,   0.0],   # Home
            [-10.0,  -5.0,  -5.0, -10.0,  -5.0, -15.0],  # P5
            [-20.0, -10.0, -10.0, -20.0, -10.0, -30.0],  # P6
            [-30.0, -15.0, -15.0, -30.0, -15.0, -45.0],  # P7 (pico negativo)
            [  0.0,   0.0,   0.0,   0.0,   0.0,   0.0],   # Home
        ],
        speed_pct = 40.0,
    )


# ============================================================================
# Registro de trajetórias disponíveis
# ============================================================================

def traj_linear_x() -> Trajectory:
    """
    Trajetória T1 — Movimento linear em X (Ensaio 3, Passo 4).

    Aproximação no espaço de juntas: J1 varia de 0° → 30° → 0° (3 repetições),
    simulando varrimento cartesiano ao longo do eixo X do TCP.
    """
    pts = [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]
    for _ in range(3):
        pts.append([ 30.0,  10.0,  5.0,  0.0,  0.0,  0.0])
        pts.append([  0.0,   0.0,  0.0,  0.0,  0.0,  0.0])
    return _build_trajectory(
        name        = "LINEAR_X",
        description = "T1 Passo4: varrimento cartesiano linear em X via J1+J2+J3. "
                      "3 repetições J1: 0→30→0°. Avalia rastreamento de eixo dominante.",
        waypoints_raw = pts,
        speed_pct   = 10.0,
    )


def traj_circular_multijoint() -> Trajectory:
    """
    Trajetória T2 — Circular multi-junta (Ensaio 3, Passo 4).

    Movimentos coordenados de J1–J5 em 8 pontos distribuídos num círculo
    no espaço de juntas, simulando trajetória circular no espaço cartesiano.
    """
    import math as _math
    waypoints_raw = [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]
    n_points = 8
    for lap in range(2):
        for k in range(n_points):
            angle = 2 * _math.pi * k / n_points
            j1 = 25.0 * _math.cos(angle)
            j2 = 15.0 * _math.sin(angle)
            j3 = 10.0 * _math.cos(angle + _math.pi / 4)
            j4 = 20.0 * _math.sin(angle)
            j5 = 10.0 * _math.cos(angle)
            j6 = 0.0
            waypoints_raw.append([j1, j2, j3, j4, j5, j6])
    waypoints_raw.append([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    return _build_trajectory(
        name        = "CIRCULAR_MULTIJOINT",
        description = "T2 Passo4: trajetória circular multi-junta (2 voltas, 8 pts/volta). "
                      "Avalia sincronismo J1–J5 em movimento coordenado contínuo.",
        waypoints_raw = waypoints_raw,
        speed_pct   = 10.0,
    )


def traj_pick_and_place() -> Trajectory:
    """
    Trajetória T3 — Pick & Place 3D (Ensaio 3, Passo 4).

    Sequência: home → abaixar (J2+J3) → translação (J1) → levantar → rotação de punho (J6).
    """
    return _build_trajectory(
        name        = "PICK_AND_PLACE",
        description = "T3 Passo4: Pick & Place 3D. Descida (J2+J3), translação (J1), "
                      "subida e rotação de 45° no punho (J6).",
        waypoints_raw = [
            [  0.0,   0.0,   0.0,   0.0,  0.0,   0.0],  # Home
            [  0.0,  30.0,  30.0,   0.0,  0.0,   0.0],  # Descida vertical
            [ 20.0,  30.0,  30.0,   0.0,  0.0,   0.0],  # Translação J1 (pick)
            [ 20.0,   0.0,   0.0,   0.0,  0.0,   0.0],  # Subida
            [ 20.0,   0.0,   0.0,   0.0,  0.0,  45.0],  # Rotação punho (place)
            [ 20.0,  30.0,  30.0,   0.0,  0.0,  45.0],  # Descida com punho rotacionado
            [ 20.0,   0.0,   0.0,   0.0,  0.0,  45.0],  # Subida
            [  0.0,   0.0,   0.0,   0.0,  0.0,   0.0],  # Home
        ],
        speed_pct   = 10.0,
    )


TRAJECTORIES = {
    "single_joint":         traj_single_joint,
    "all_joints":           traj_all_joints,
    "repeated_abs":         traj_repeated_abs,
    "move_j_short":         traj_move_j_short,
    "canonical":            traj_canonical_benchmark,
    "linear_x":             traj_linear_x,
    "circular_multijoint":  traj_circular_multijoint,
    "pick_and_place":       traj_pick_and_place,
}


def load_trajectory(name: str) -> Trajectory:
    """
    Carrega uma trajetória pelo nome.

    Parâmetros
    ----------
    name : str
        Nome da trajetória (ver TRAJECTORIES).

    Retorna
    -------
    Trajectory

    Levanta
    -------
    KeyError se o nome não for encontrado.
    """
    if name not in TRAJECTORIES:
        available = list(TRAJECTORIES.keys())
        raise KeyError(f"Trajetória '{name}' não encontrada. Disponíveis: {available}")
    traj = TRAJECTORIES[name]()
    logger.info("Trajetória carregada: '%s' (%d waypoints, %.1f s)",
                name, len(traj["waypoints"]), traj["total_ms"] / 1000.0)
    return traj


# ============================================================================
# Validação rápida ao importar
# ============================================================================

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)s %(message)s")

    print("=== Validacao das trajetorias canonicas ===\n")

    for traj_name in TRAJECTORIES:
        traj = load_trajectory(traj_name)
        print(f"[OK] '{traj_name}': {len(traj['waypoints'])} waypoints | "
              f"{traj['total_ms']/1000:.1f} s")

        # Valida que todas as juntas estão dentro dos limites
        for wp in traj["waypoints"]:
            assert _validate_waypoint(wp), f"Limite violado: {wp}"

        # Valida que qd e qdd estão preenchidos
        for wp in traj["waypoints"]:
            assert len(wp["qd"])  == 6, "qd incompleto"
            assert len(wp["qdd"]) == 6, "qdd incompleto"

    print("\nTodos os testes de trajetoria passaram! [OK]")
