# Mapeamento de Juntas e Limitações do Proxy — EB15 vs. UR10e

**TCC Fernando Oliveira — Passo 3: Ambiente Virtual, Pontes e Simulação**

Este documento registra as correspondências cinemáticas, diferenças dimensionais e
limitações do proxy adotado (UR10e no Webots R2025a) em relação ao manipulador real
EB15. A identidade geométrica **não é alegada**; o UR10e é utilizado como proxy
dinâmico e cinemático por compartilhar a topologia de punho esférico (spherical wrist)
e estar disponível como PROTO oficial na biblioteca do Webots.

---

## 1. Tabela de Correspondência de Juntas

| Junta EB15 | Papel Cinemático        | Device UR10e (Webots) | Sentido Positivo |
|------------|-------------------------|-----------------------|------------------|
| J1         | Rotação da base         | `shoulder_pan_joint`  | Anti-horário visto de cima |
| J2         | Elevação do ombro       | `shoulder_lift_joint` | Frente (extensão) |
| J3         | Cotovelo                | `elbow_joint`         | Extensão |
| J4         | Punho 1 (yaw)           | `wrist_1_joint`       | Positivo = giro de referência |
| J5         | Punho 2 (pitch)         | `wrist_2_joint`       | Positivo = inclinação para cima |
| J6         | Punho 3 (roll)          | `wrist_3_joint`       | Anti-horário visto da ponta |

> **Sensor de posição:** cada junta possui um sensor de nome `<device>_sensor`
> (ex.: `shoulder_pan_joint_sensor`). O supervisor lê esses sensores como
> ground truth para emulação dos AS5600.

---

## 2. Limites Angulares — EB15 vs. UR10e

| Junta | EB15 Min (°) | EB15 Max (°) | UR10e Min (°) | UR10e Max (°) | Restrição Aplicada Em |
|-------|-------------|-------------|--------------|--------------|----------------------|
| J1    | −170        | +170        | −360         | +360         | `hil_bridge.py` (clamp) |
| J2    | −45         | +180        | −360         | +360         | `hil_bridge.py` (clamp) |
| J3    | −120        | +120        | −360         | +360         | `hil_bridge.py` (clamp) |
| J4    | −180        | +180        | −360         | +360         | `hil_bridge.py` (clamp) |
| J5    | −90         | +90         | −360         | +360         | `hil_bridge.py` (clamp) |
| J6    | −360        | +360        | −360         | +360         | `hil_bridge.py` (clamp) |

Os limites do UR10e são preservados no PROTO. Os limites do EB15 são aplicados
como restrição adicional na camada de adaptação (`hil_bridge.py`), sem modificar
o modelo do Webots.

---

## 3. Diferenças Dimensionais (Proxy ≠ Real)

| Parâmetro                        | EB15 (estimado)       | UR10e (real)          |
|----------------------------------|-----------------------|-----------------------|
| Comprimento do elo superior (a2) | ~150 mm               | 612,7 mm              |
| Comprimento do elo inferior (a3) | ~130 mm               | 571,6 mm              |
| Alcance máximo (aprox.)          | ~400 mm               | ~1.300 mm             |
| Massa nominal                    | ~1,5 kg               | ~33,5 kg              |
| Payload nominal                  | ~0,5 kg               | 12,5 kg               |
| Inércia dos elos                 | Valores PLA estimados | Alumínio industrial   |
| Acionadores                      | NEMA 17 (stepper)     | Servos industriais    |

**Consequência prática:** as forças de inércia e os torques gravitacionais no Webots
são calculados com os parâmetros do UR10e, **não** do EB15. O proxy valida a lógica
de controle e a arquitetura de software, mas os resultados dinâmicos (torque,
vibração, sobressinal) não devem ser extrapolados diretamente para o hardware real.

---

## 4. Pose Zero — Referência e Transformação

| Referencial          | Configuração zero                         |
|----------------------|-------------------------------------------|
| EB15 lógico          | Todas as juntas em 0° (posição de repouso vertical) |
| UR10e no Webots      | Todas as juntas em 0 rad (braço estendido verticalmente para cima) |
| Transformação        | Identidade (sem rotação de offset) na Fase 3 |

A pose zero do EB15 e do UR10e coincidem na configuração lógica de referência
adotada para este trabalho. Offsets físicos de montagem presentes no hardware real
devem ser calibrados separadamente no Passo 4.

---

## 5. Cinco Configurações Canônicas de Validação do Mapeamento

As poses abaixo foram utilizadas para verificar manualmente o mapeamento de ordem,
sinal e unidade entre o EB15 e o UR10e antes dos testes de trajetória.

| Config | J1 (°) | J2 (°) | J3 (°) | J4 (°) | J5 (°) | J6 (°) | Descrição |
|--------|--------|--------|--------|--------|--------|--------|-----------|
| C1     |   0    |   0    |   0    |   0    |   0    |   0    | Home (repouso) |
| C2     |   0    | +45    | +45    |   0    |   0    |   0    | Cotovelo elevado |
| C3     |   0    | +90    | −45    |   0    |   0    |   0    | Extensão frontal |
| C4     |   0    |   0    |   0    | +45    | +45    | +45    | Punho exercitado |
| C5     | +30    | +60    | −60    | +30    | +30    | +30    | Configuração combinada |

Para cada configuração, verificou-se:
- O ângulo reportado pelos sensores do UR10e corresponde ao ângulo comandado.
- Nenhuma junta ultrapassa os limites do EB15 definidos na Seção 2.
- O sentido positivo coincide com a convenção documentada na Tabela 1.

---

## 6. Limitações Conhecidas do Proxy

1. **Dinâmica não representativa:** inércia, torque e amortecimento refletem o UR10e
   industrial, não o EB15 acadêmico em PLA. Resultados de PVA dinâmico são qualitativos.

2. **Ausência de backlash:** o Webots não modela folgas nas caixas de redução poliméricas
   do EB15 presentes no hardware real.

3. **Sem modelo de flexibilidade de elos:** elos impressos em PLA têm rigidez menor do
   que o alumínio do UR10e; vibrações e ressonâncias do EB15 real não são capturadas.

4. **Alcance e espaço de trabalho distintos:** trajetórias projetadas para o UR10e podem
   ser geometricamente inalcançáveis no EB15 real, especialmente em extensões longas.

5. **PCINT físico não emulável em software:** o trigger elétrico via GPIO (INT0) opera em
   nível de hardware; a latência sub-microssegundo é reservada para medição em bancada física real.

6. **Offset de pose zero em hardware:** a pose zero lógica pode não coincidir com a posição
   física de repouso do hardware real; calibração de home é necessária no Passo 4.
