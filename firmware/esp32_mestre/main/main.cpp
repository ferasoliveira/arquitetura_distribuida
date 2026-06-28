/**
 * EB-15 Robotic Arm — Mestre (ESP32-S3)
 * main.cpp — Firmware principal com arquitetura dual-core FreeRTOS
 *
 * ARQUITETURA:
 *   Core 0 (rede, não crítico):
 *     - task_wifi       (prioridade 3): conexão Wi-Fi, reconexão automática
 *     - task_webserver  (prioridade 2): servidor HTTP REST (endpoints de controle)
 *     - task_rtde       (prioridade 4): servidor TCP RTDE-EB15 na porta 30003
 *
 *   Core 1 (tempo real, crítico):
 *     - task_control    (prioridade 10): laço 200 Hz via semáforo de timer
 *       → FK/IK, planejador Curva-S, PID+Tanh J1-J3, geração Step/Dir
 *     - task_uart_master (prioridade 8): despacha frames J4-J6 ao Arduino Uno,
 *       gerencia handshake e trigger, NÃO bloqueia task_control
 *
 * ISOLAMENTO TEMPORAL:
 *   - esp_timer dispara a cada 5 000 µs → ISR dá semáforo a task_control
 *   - task_control nunca espera UART nem rede — posta frame numa queue
 *   - task_uart_master consome a queue e gerencia o handshake bloqueante
 *
 * EMULAÇÃO QEMU (compilar com -DEB15_QEMU=1):
 *   - Wi-Fi e NVS desativados
 *   - Timer substitido por vTaskDelay(5 ms) dentro da task_control
 *   - GPIO logado via ESP_LOGD em vez de acionamento físico
 *   - UART2 redirecionada para TCP porta 30101 via port-forwarding QEMU
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <stdatomic.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/uart.h"

#include "esp_wifi.h"
#include "esp_eth.h"
#include "esp_eth_phy.h"
#include "esp_http_server.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"

/* ===== Co-simulação SÍNCRONA (lockstep) — EB15_LOCKSTEP =====
 * 1 = o ciclo de controle de J1-J3 roda UMA vez por passo do Webots, dirigido pela
 *     ponte2: o endpoint :30101 RECEBE o encoder medido → run_control_cycle (feedback
 *     fresco) → DEVOLVE o comando, tudo síncrono. Sem staleness; a simulação avança o
 *     mais rápido que o host permite e de forma determinística (sem espera de parede).
 * 0 = modo LIVRE histórico (task_control 200 Hz por timer; encoder lido em 30103). */
#define EB15_LOCKSTEP 1
/* Fusão de feedback no lockstep (filtro complementar / observador): o estimador
 * g_joint_angles é PREDITO pelo dead-reckoning (passos emitidos) e CORRIGIDO em
 * direção ao encoder medido com ganho LOCKSTEP_ENC_ALPHA por ciclo (5 ms / 200 Hz):
 *   est += alpha * (medido - est)
 * alpha=1 → sobrescreve tudo (malha de ganho total: instável/runaway em lockstep);
 * alpha=0 → puro dead-reckoning (malha aberta); alpha intermediário → MALHA FECHADA
 * de banda limitada (encoder corrige drift/passos perdidos, dead-reckoning prediz o
 * movimento intra-ciclo). Modela o stepper real (passos open-loop + AS5600 p/ drift)
 * e reproduz, de forma DETERMINÍSTICA, o passa-baixa que o modo livre tinha por acaso. */
#ifndef LOCKSTEP_ENC_ALPHA
#define LOCKSTEP_ENC_ALPHA 0.005f
#endif
#include "lwip/inet.h"
#include "esp_netif.h"
#include "esp_event.h"

#include "config.h"
#include "kinematics.h"
#include "trajectory.h"
#include "pid_tanh.h"
#include "uart_master.h"
#include "rtde_core.h"

// ============================================================================
// Tags de log
// ============================================================================
static const char *TAG_BOOT  = "BOOT";
static const char *TAG_CTRL  = "CTRL";
static const char *TAG_UART  = "UART";
static const char *TAG_WEB   = "WEB";
static const char *TAG_RTDE  = "RTDE";

// ============================================================================
// Estado global compartilhado (protegido por mutexes)
// ============================================================================

/* Ângulos atuais das juntas em GRAUS (lidos dos encoders AS5600) */
float g_joint_angles[6] = { 0.0f };

#ifdef EB15_QEMU
/* Referencias Step/Dir J1-J3 publicadas para a ponte HIL do Webots. */
static float s_qemu_command_deg[3] = { 0.0f, 0.0f, 0.0f };
#if EB15_LOCKSTEP
/* Último encoder MEDIDO bruto de J1-J3 no lockstep (referência do anti-windup do comando). */
static volatile float g_ls_meas[3] = { 0.0f, 0.0f, 0.0f };
/* Margem (graus) com que o comando dead-reckoned pode LIDERAR a junta física medida.
 * Modela um stepper que perde passos sob sobrecarga (não acumula passos-fantasma) → o
 * acumulador não dispara (windup) quando o atuador-proxy do Webots não acompanha. */
#ifndef LOCKSTEP_LEAD_DEG
#define LOCKSTEP_LEAD_DEG 12.0f
#endif
#endif
#endif

/* Pose cartesiana atual do TCP */
CartesianPose g_current_pose = { 0.0f, 0.0f, LINK_L1 + LINK_L2 + LINK_L3, 0.0f, 0.0f, 0.0f };

/* Fila circular de segmentos de trajetória */
TrajectorySegment g_segments[MAX_SEGMENTS];
volatile int g_seg_head = 0;
volatile int g_seg_tail = 0;

/* Mutex de proteção do estado das juntas */
static SemaphoreHandle_t s_joints_mutex  = NULL;
static float g_control_loop_jitter_us = 0.0f;

/* Buffer diagnóstico do relógio real da malha. Cada entrada corresponde a um
   ciclo executado; o endpoint /jitter exporta os timestamps sem estimá-los a
   partir da telemetria RTDE de 50 Hz. */
#define JITTER_BUFFER_SIZE 16384
static uint32_t s_cycle_timestamp_us[JITTER_BUFFER_SIZE] = {};
static volatile uint32_t s_cycle_write = 0;
static volatile uint32_t s_cycle_count = 0;
static portMUX_TYPE s_jitter_mux = portMUX_INITIALIZER_UNLOCKED;

/* Semáforo binário do timer de controle → task_control */
static SemaphoreHandle_t s_ctrl_sem      = NULL;

/* Queue de frames UART para task_uart_master */
typedef struct {
    int32_t  target_steps[3];  /* J4, J5, J6 absolutos em passos */
    uint16_t duration_ms;
    uint8_t  kind;             /* 0 = LOAD (bufferiza fatia no escravo); 1 = EXECUTE */
} UartJob;
#define UARTJOB_LOAD     0
#define UARTJOB_EXECUTE  1
static QueueHandle_t s_uart_queue = NULL;

/* Flag global de E-STOP — atomic para acesso seguro entre task_control (Core1) e
   task_uart_master (Core1) e futuras tasks de rede (Core0). */
static atomic_bool s_estop = ATOMIC_VAR_INIT(false);
static atomic_int s_operation_mode = ATOMIC_VAR_INIT(
    static_cast<int>(eb15_rtde::OperationMode::RTDE));

/* Estado PID para J1-J3 */
static PIDState s_pid[3] = {};

/* Acumuladores DDA para Step/Dir */
static float s_step_accum[3] = {};

/* Segmento de trajetória ativo (consumido a cada duration_ms, não a cada tick) */
static TrajectorySegment s_active_seg   = {};
static bool              s_seg_active   = false;
static int               s_seg_ticks_remaining = 0;
/* true após o primeiro segmento consumido: habilita HOLD do alvo final quando o
   buffer de trajetória esvazia (fecha erro residual da latência do relé HIL). */
static bool              s_have_target  = false;

/* Mutex do anel de segmentos: produtor (Core0) e consumidor (Core1 task_control) */
static SemaphoreHandle_t s_seg_mutex = NULL;

static bool motion_active(void) {
    bool active;
    xSemaphoreTake(s_seg_mutex, portMAX_DELAY);
    active = s_seg_active || (g_seg_head != g_seg_tail);
    xSemaphoreGive(s_seg_mutex);
    return active;
}

static void stop_motion(void) {
    atomic_store_explicit(&s_estop, true, memory_order_release);
    xSemaphoreTake(s_seg_mutex, portMAX_DELAY);
    g_seg_head = g_seg_tail = 0;
    s_seg_active = false;
    s_seg_ticks_remaining = 0;
    s_have_target = false;   /* não manter alvo após parada/E-STOP */
    xSemaphoreGive(s_seg_mutex);
    xQueueReset(s_uart_queue);
    uart_estop();
}

static void switch_operation_mode(eb15_rtde::OperationMode requested) {
    if (motion_active()) stop_motion();
    atomic_store_explicit(&s_operation_mode, static_cast<int>(requested),
                          memory_order_release);
    atomic_store_explicit(&s_estop, false, memory_order_release);
}

// ============================================================================
// push_segment protegido por mutex (para uso por task_webserver / task_rtde)
// trajectory.h::push_segment() é lock-free SPSC; esta wrapper garante que
// múltiplas tasks de Core 0 não corrompam g_seg_head.
// ============================================================================

static bool push_segment_locked(const float q_deg[6], uint32_t duration_ms)
{
    if (!s_seg_mutex) return push_segment(q_deg, duration_ms);
    if (xSemaphoreTake(s_seg_mutex, pdMS_TO_TICKS(10)) == pdFALSE) return false;
    const bool ok = push_segment(q_deg, duration_ms);
    xSemaphoreGive(s_seg_mutex);
    return ok;
}

// ============================================================================
// Timer de controle a 200 Hz
// ============================================================================

#ifndef EB15_QEMU
static void IRAM_ATTR ctrl_timer_cb(void *arg) {
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(s_ctrl_sem, &woken);
    if (woken) portYIELD_FROM_ISR();
}
#endif

// ============================================================================
// Geração de Step/Dir para J1-J3
// ============================================================================

static void emit_step(int gpio_pul, int gpio_dir, bool dir, int pulses) {
#ifndef EB15_QEMU
    gpio_set_level(gpio_dir, dir ? 1 : 0);
    for (int i = 0; i < pulses; ++i) {
        gpio_set_level(gpio_pul, 1);
        esp_rom_delay_us(2);
        gpio_set_level(gpio_pul, 0);
        esp_rom_delay_us(2);
    }
#else
    if (pulses > 0)
        ESP_LOGD(TAG_CTRL, "step J%d: dir=%d pulsos=%d", gpio_pul, dir, pulses);
#endif
}

static const int GPIO_PUL[3] = { PUL_J1, PUL_J2, PUL_J3 };
static const int GPIO_DIR[3] = { DIR_J1, DIR_J2, DIR_J3 };

// ============================================================================
// Ciclo de controle 200 Hz — executado dentro de task_control
// ============================================================================

static void run_control_cycle(void) {
    if (atomic_load_explicit(&s_estop, memory_order_relaxed)) return;

    static uint64_t last_time_us = 0;
    uint64_t now_us = esp_timer_get_time();
    g_control_loop_jitter_us = last_time_us == 0
        ? 0.0f : (float)((int64_t)(now_us - last_time_us) - 5000LL);
    last_time_us = now_us;

    portENTER_CRITICAL(&s_jitter_mux);
    s_cycle_timestamp_us[s_cycle_write] = (uint32_t)now_us;
    s_cycle_write = (s_cycle_write + 1U) % JITTER_BUFFER_SIZE;
    if (s_cycle_count < JITTER_BUFFER_SIZE) ++s_cycle_count;
    portEXIT_CRITICAL(&s_jitter_mux);

    const float dt = 1.0f / (float)CONTROL_HZ;  /* 0.005 s */

    /* --- Avança para o próximo segmento quando o atual expirar --- */
    bool holding = false;
    if (!s_seg_active || s_seg_ticks_remaining <= 0) {
        /* Tenta pegar novo segmento com proteção de mutex */
        if (xSemaphoreTake(s_seg_mutex, 0) == pdFALSE) return;  /* mutex ocupado: pula tick */
        const bool has_seg = (g_seg_head != g_seg_tail);
        if (has_seg) {
            s_active_seg = g_segments[g_seg_tail];
            g_seg_tail   = (g_seg_tail + 1) % MAX_SEGMENTS;
        }
        xSemaphoreGive(s_seg_mutex);

        if (has_seg) {
            s_have_target = true;
            /* Calcula quantos ticks de 5ms cabem neste segmento */
            const int ticks = (int)((float)s_active_seg.duration_ms / (1000.0f * dt) + 0.5f);
            s_seg_ticks_remaining = (ticks > 0) ? ticks : 1;
            s_seg_active = true;

            /* J4-J6 NÃO é despachado por slice: é enviado uma vez por MoveJ em
             * plan_move_j_locked (frame ABSOLUTO + duração total). Evita o atraso
             * por latência do handshake UART nos relés TCP do QEMU. Aqui só corre
             * a malha de J1-J3. */
        } else {
            /* Buffer de trajetória vazio: em vez de "soltar" a junta (retorno
             * antecipado deixava o PID sem regular), MANTÉM o último alvo
             * (s_active_seg) e segue regulando até convergir (fecha o resíduo da
             * latência do relé HIL). Sem alvo prévio → nada a manter. */
            s_seg_active = false;
            if (!s_have_target) return;
            s_seg_ticks_remaining = 0;
            holding = true;
        }
    }

    if (!holding && s_seg_ticks_remaining > 0)
        s_seg_ticks_remaining--;

    /* Lê ângulos atuais */
    float cur[6];
    xSemaphoreTake(s_joints_mutex, portMAX_DELAY);
    for (int i = 0; i < 6; ++i) cur[i] = g_joint_angles[i];
    xSemaphoreGive(s_joints_mutex);

    /* --- J1-J3: PID+Tanh + geração de Step/Dir a 200 Hz --- */
    for (int i = 0; i < 3; ++i) {
        /* Guarda anti-runaway: um erro implausível (> ANTISPIN_DEG) só acontece se o
         * encoder vier corrompido por um transitório de transporte do co-sim QEMU
         * (frames perdidos/valor espúrio). As trajetórias canônicas têm waypoints
         * « 60°, então |alvo - medido| jamais é tão grande em operação válida.
         * NÃO acionar o motor sobre feedback inválido: evita J1 disparar até o limite
         * (e o braço bater no chão). Retoma quando o encoder volta ao normal. */
        const float ANTISPIN_DEG = 100.0f;
        if (fabsf(s_active_seg.q_deg[i] - cur[i]) > ANTISPIN_DEG) {
            s_pid[i].integral = 0.0f;   /* zera integrador p/ não acumular windup */
            s_pid[i].prev_error = 0.0f;
            continue;
        }
        const float f_hz = pid_tanh_update(i, s_active_seg.q_deg[i], cur[i], dt, &s_pid[i]);
        int pulses; bool dir;
        pid_step_pulses(f_hz, dt, &s_step_accum[i], &pulses, &dir);
        emit_step(GPIO_PUL[i], GPIO_DIR[i], dir, pulses);

        /* Atualiza ângulo estimado (corrigido pelo encoder na leitura real) */
        const float delta_deg = (dir ? 1.0f : -1.0f) * (float)pulses / STEPS_PER_DEG[i];
        xSemaphoreTake(s_joints_mutex, portMAX_DELAY);
        g_joint_angles[i] += delta_deg;
#ifdef EB15_QEMU
        /* O encoder atualiza g_joint_angles assincronamente. O trem Step/Dir,
         * porém, é uma grandeza elétrica acumulada independente e não pode
         * ser zerado pelo feedback antes de a ponte 2 observá-lo. */
        s_qemu_command_deg[i] += delta_deg;
        /* CLAMP de soft-limit no comando: garante que a junta NUNCA seja comandada
         * além de ±limite. Sem isto, um transitório de transporte podia levar J1
         * além de 180°, o encoder AS5600 dava volta (wrap) e o PID perseguia a volta
         * → spin até o limite / braço no chão. Limitando o comando, o pior caso vira
         * "parada no limite" (SETTLE_TIMEOUT honesto), nunca um giro descontrolado. */
        if (s_qemu_command_deg[i] > LIMIT_MAX_DEG[i]) s_qemu_command_deg[i] = LIMIT_MAX_DEG[i];
        if (s_qemu_command_deg[i] < LIMIT_MIN_DEG[i]) s_qemu_command_deg[i] = LIMIT_MIN_DEG[i];
#if EB15_LOCKSTEP
        /* ANTI-WINDUP (lockstep): o comando dead-reckoned não pode LIDERAR a junta física
         * MEDIDA por mais que LOCKSTEP_LEAD_DEG. Quando o atuador-proxy do Webots não
         * acompanha um move agressivo, o acumulador pararia de disparar até o ±limite
         * (windup → satura → não recupera). Limitar a liderança modela um stepper que perde
         * passos sob sobrecarga: o comando segue colado à junta e o erro fecha quando ela
         * alcança. NÃO toca o PID/tanh/curva-S — é saturação do output (como o clamp acima). */
        {
            const float m = g_ls_meas[i];
            if (s_qemu_command_deg[i] > m + LOCKSTEP_LEAD_DEG) s_qemu_command_deg[i] = m + LOCKSTEP_LEAD_DEG;
            if (s_qemu_command_deg[i] < m - LOCKSTEP_LEAD_DEG) s_qemu_command_deg[i] = m - LOCKSTEP_LEAD_DEG;
        }
#endif
#endif
        xSemaphoreGive(s_joints_mutex);
    }
}

// ============================================================================
// task_control — Core 1, prioridade 10
// ============================================================================

static void task_control(void *pv) {
#if EB15_LOCKSTEP
    /* Em lockstep o ciclo de controle é dirigido pelo endpoint :30101 (1x por passo do
     * Webots). task_control fica ociosa para não rodar run_control_cycle em paralelo. */
    ESP_LOGI(TAG_CTRL, "[BOOT OK] task_control OCIOSA (lockstep: controle dirigido pela ponte2)");
    (void)pv;
    for (;;) vTaskDelay(portMAX_DELAY);
#else
    ESP_LOGI(TAG_CTRL, "[BOOT OK] task_control Core1 @ 200Hz");

#ifdef EB15_QEMU
    TickType_t last_wake = xTaskGetTickCount();
#endif

    for (;;) {
#ifdef EB15_QEMU
        xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(5));
#else
        xSemaphoreTake(s_ctrl_sem, portMAX_DELAY);
#endif
        run_control_cycle();
    }
#endif
}

// ============================================================================
// task_uart_master — Core 1, prioridade 8
// ============================================================================

static void task_uart_master(void *pv) {
    ESP_LOGI(TAG_UART, "task_uart_master iniciada");
    uart_reset();

    UartJob job;
    int pending = 0;   /* fatias LOAD escritas (pipelined) aguardando ACK na barreira */
    int64_t batch_t0 = 0;  /* instante da 1ª fatia do lote (p/ medir tempo de escrita) */
    for (;;) {
        if (xQueueReceive(s_uart_queue, &job, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (!atomic_load_explicit(&s_estop, memory_order_relaxed)) {
                bool ok = true;
                if (job.kind == UARTJOB_EXECUTE) {
                    const int64_t tb = esp_timer_get_time();
                    const int acks = uart_load_barrier(pending);   /* confirma cargas pipelined */
                    const int64_t te = esp_timer_get_time();
                    ok = uart_execute(job.duration_ms);  /* EXECUTE + DONE */
                    const int64_t td = esp_timer_get_time();
                    ESP_LOGI("TMG", "  UART escrita_lote=%lld ms | barreira acks=%d/%d dt=%lld ms | EXECUTE+DONE dt=%lld ms ok=%d",
                             (long long)((tb - batch_t0) / 1000), acks, pending,
                             (long long)((te - tb) / 1000),
                             (long long)((td - te) / 1000), (int)ok);
                    pending = 0;
                } else {
                    if (pending == 0) batch_t0 = esp_timer_get_time();
                    ok = uart_load_frame(job.target_steps, job.duration_ms);  /* só escreve */
                    if (ok) ++pending;
                }
                if (!ok) {
                    ESP_LOGE(TAG_UART, "Falha no despacho UART — E-STOP");
                    atomic_store_explicit(&s_estop, true, memory_order_release);
                    uart_estop();
                    pending = 0;
                }
            }
        }
    }
}

// ============================================================================
// Wrapper: enfileira segmentos com proteção de mutex (para uso por Core 0)
// ============================================================================

static void plan_move_j_locked(const float target[6], float speed_pct) {
    /* plan_move_j_scurve chama push_segment em loop; usa s_seg_mutex por chamada */
    const int64_t t_plan0 = esp_timer_get_time();
    plan_move_j_scurve(target, speed_pct);
    const int64_t t_plan1 = esp_timer_get_time();
    ESP_LOGI("TMG", "  scurve(J1-3 push) dt=%lld ms | j46_fatias=%d total_ms=%lu",
             (long long)((t_plan1 - t_plan0) / 1000), g_j46_nslices,
             (unsigned long)g_scurve_last_total_ms);

    /* --- Despacho J4-J6 (simavr/Uno): BUFFER LOOK-AHEAD ---
     * Em vez de 1 frame com o alvo final (degrau) ou 1 frame por fatia durante o
     * movimento (lento), CARREGAMOS todas as fatias da curva S de J4-J6 no escravo
     * (frames LOAD, só ACK, sem mover) e depois disparamos UM trigger (EXECUTE). O
     * escravo então reproduz o buffer encadeado, seguindo a MESMA curva S de J1-J3
     * → J4-J6 suave E sem latência por-fatia. A transferência (ACK rápido) ocorre
     * no início do waypoint; o movimento toca do buffer a toda velocidade. */
    if (s_uart_queue && g_j46_nslices > 0) {
        for (int s = 0; s < g_j46_nslices; ++s) {
            UartJob job;
            job.kind = UARTJOB_LOAD;
            job.target_steps[0] = g_j46_slices[s][0];
            job.target_steps[1] = g_j46_slices[s][1];
            job.target_steps[2] = g_j46_slices[s][2];
            job.duration_ms = g_j46_dur[s];
            xQueueSend(s_uart_queue, &job, portMAX_DELAY);
        }
        UartJob ex;
        ex.kind = UARTJOB_EXECUTE;
        ex.target_steps[0] = ex.target_steps[1] = ex.target_steps[2] = 0;
        /* duração total p/ o timeout do DONE (playback inteiro) */
        uint32_t tot = g_scurve_last_total_ms;
        if (tot > 60000U) tot = 60000U;
        ex.duration_ms = (uint16_t)tot;
        xQueueSend(s_uart_queue, &ex, portMAX_DELAY);
        ESP_LOGI("TMG", "  j46 enfileirado (%d LOAD + 1 EXECUTE) dt_enq=%lld ms",
                 g_j46_nslices, (long long)((esp_timer_get_time() - t_plan1) / 1000));
    }
}

// ============================================================================
// task_wifi — Core 0, prioridade 3
// ============================================================================

static SemaphoreHandle_t s_wifi_ready = NULL;

#ifdef EB15_QEMU
static esp_eth_handle_t s_qemu_eth_handle = NULL;
static esp_eth_netif_glue_handle_t s_qemu_eth_glue = NULL;

static void qemu_eth_got_ip(void *, esp_event_base_t, int32_t, void *event_data) {
    const ip_event_got_ip_t *event = static_cast<const ip_event_got_ip_t *>(event_data);
    ESP_LOGI(TAG_WEB, "[NETWORK OK] QEMU OpenEth ativo: " IPSTR ", gateway " IPSTR,
             IP2STR(&event->ip_info.ip), IP2STR(&event->ip_info.gw));
    if (s_wifi_ready) xSemaphoreGive(s_wifi_ready);
}
#endif

#ifndef CONFIG_EB15_WIFI_SSID
  #define CONFIG_EB15_WIFI_SSID "EB15-Robotics"
#endif
#ifndef CONFIG_EB15_WIFI_PASS
  #define CONFIG_EB15_WIFI_PASS "admin12345"
#endif

static void task_wifi(void *pv) {
    ESP_LOGI(TAG_WEB, "task_network iniciada");

    /* NVS necessário para Wi-Fi */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    /* Inicialização do netif e event loop */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

#ifdef EB15_QEMU
    esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t *netif = esp_netif_new(&netif_cfg);
    configASSERT(netif);

    eth_mac_config_t mac_cfg = ETH_MAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_cfg = ETH_PHY_DEFAULT_CONFIG();
    phy_cfg.phy_addr = 1;  /* DEFAULT_PHY do modelo OpenEth do QEMU Espressif */
    phy_cfg.reset_gpio_num = -1;
    phy_cfg.autonego_timeout_ms = 100;

    esp_eth_mac_t *mac = esp_eth_mac_new_openeth(&mac_cfg);
    esp_eth_phy_t *phy = esp_eth_phy_new_dp83848(&phy_cfg);
    configASSERT(mac && phy);
    esp_eth_config_t eth_cfg = ETH_DEFAULT_CONFIG(mac, phy);
    ESP_ERROR_CHECK(esp_eth_driver_install(&eth_cfg, &s_qemu_eth_handle));
    s_qemu_eth_glue = esp_eth_new_netif_glue(s_qemu_eth_handle);
    configASSERT(s_qemu_eth_glue);
    ESP_ERROR_CHECK(esp_netif_attach(netif, s_qemu_eth_glue));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP,
                                               qemu_eth_got_ip, NULL));
    ESP_ERROR_CHECK(esp_eth_start(s_qemu_eth_handle));
    ESP_LOGI(TAG_WEB, "QEMU OpenEth aguardando DHCP...");
#else
    esp_netif_create_default_wifi_sta();

    /* Configuração básica de estação Wi-Fi */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    wifi_config_t wcfg = {};
    /* Credenciais configuradas em sdkconfig (CONFIG_EB15_WIFI_SSID / PASSWORD) */
    strncpy((char *)wcfg.sta.ssid,     CONFIG_EB15_WIFI_SSID,     sizeof(wcfg.sta.ssid));
    strncpy((char *)wcfg.sta.password, CONFIG_EB15_WIFI_PASS,     sizeof(wcfg.sta.password));
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wcfg);
    esp_wifi_start();
    esp_wifi_connect();

    ESP_LOGI(TAG_WEB, "Wi-Fi conectando...");
    /* sinal para task_webserver começar (simplificado; Passo 3 usa event bits) */
    vTaskDelay(pdMS_TO_TICKS(3000));
    if (s_wifi_ready) xSemaphoreGive(s_wifi_ready);
#endif

    vTaskDelay(portMAX_DELAY);
}

// ============================================================================
// task_webserver — Core 0, prioridade 2
// HTTP REST mínimo:
//   GET  /status   → JSON com ângulos e estado
//   POST /move_j   → body: "q0,q1,q2,q3,q4,q5,speed" (floats CSV)
//   POST /estop    → E-STOP imediato
// ============================================================================

static esp_err_t http_status(httpd_req_t *req) {
    xSemaphoreTake(s_joints_mutex, portMAX_DELAY);
    float q[6]; for (int i=0; i<6; i++) q[i] = g_joint_angles[i];
    xSemaphoreGive(s_joints_mutex);
    char buf[256];
    snprintf(buf, sizeof(buf),
             "{\"q\":[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f],\"estop\":%s}",
             q[0],q[1],q[2],q[3],q[4],q[5],
             atomic_load_explicit(&s_estop, memory_order_relaxed) ? "true" : "false");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, buf);
    return ESP_OK;
}

static esp_err_t http_move_j(httpd_req_t *req) {
    if (atomic_load_explicit(&s_operation_mode, memory_order_acquire) !=
        static_cast<int>(eb15_rtde::OperationMode::USER)) {
        httpd_resp_set_status(req, "409 Conflict");
        httpd_resp_sendstr(req, "mode is not USER");
        return ESP_OK;
    }
    char body[128] = {};
    int  len = httpd_req_recv(req, body, sizeof(body) - 1);
    if (len <= 0) { httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "no body"); return ESP_OK; }
    float q[6] = {}, speed = 50.0f;
    /* formato: "q0,q1,q2,q3,q4,q5[,speed]" */
    sscanf(body, "%f,%f,%f,%f,%f,%f,%f", &q[0],&q[1],&q[2],&q[3],&q[4],&q[5],&speed);
    plan_move_j_locked(q, speed);
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

static esp_err_t http_estop(httpd_req_t *req) {
    stop_motion();
    httpd_resp_sendstr(req, "ESTOP");
    ESP_LOGW(TAG_WEB, "E-STOP via HTTP");
    return ESP_OK;
}

static esp_err_t http_mode(httpd_req_t *req) {
    char body[16] = {};
    const int len = httpd_req_recv(req, body, sizeof(body) - 1);
    if (len <= 0 || (strncmp(body, "user", 4) != 0 && strncmp(body, "rtde", 4) != 0)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "expected user or rtde");
        return ESP_OK;
    }
    const eb15_rtde::OperationMode requested = strncmp(body, "user", 4) == 0
        ? eb15_rtde::OperationMode::USER : eb15_rtde::OperationMode::RTDE;
    switch_operation_mode(requested);
    httpd_resp_sendstr(req, requested == eb15_rtde::OperationMode::USER ? "USER" : "RTDE");
    return ESP_OK;
}

static esp_err_t http_jitter_reset(httpd_req_t *req) {
    portENTER_CRITICAL(&s_jitter_mux);
    s_cycle_write = 0;
    s_cycle_count = 0;
    portEXIT_CRITICAL(&s_jitter_mux);
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

static esp_err_t http_jitter(httpd_req_t *req) {
    uint32_t count;
    uint32_t start;
    portENTER_CRITICAL(&s_jitter_mux);
    count = s_cycle_count;
    start = (s_cycle_write + JITTER_BUFFER_SIZE - count) % JITTER_BUFFER_SIZE;
    portEXIT_CRITICAL(&s_jitter_mux);

    httpd_resp_set_type(req, "text/csv");
    httpd_resp_send_chunk(req, "cycle,timestamp_us\n", HTTPD_RESP_USE_STRLEN);
    char line[48];
    for (uint32_t i = 0; i < count; ++i) {
        const uint32_t idx = (start + i) % JITTER_BUFFER_SIZE;
        const int n = snprintf(line, sizeof(line), "%lu,%lu\n",
                               (unsigned long)i,
                               (unsigned long)s_cycle_timestamp_us[idx]);
        if (httpd_resp_send_chunk(req, line, n) != ESP_OK) return ESP_FAIL;
    }
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

static void task_webserver(void *pv) {
    ESP_LOGI(TAG_WEB, "task_webserver aguardando Wi-Fi...");
    if (s_wifi_ready) xSemaphoreTake(s_wifi_ready, pdMS_TO_TICKS(10000));

    httpd_handle_t server = NULL;
    httpd_config_t hcfg   = HTTPD_DEFAULT_CONFIG();
    hcfg.server_port      = 80;
    if (httpd_start(&server, &hcfg) != ESP_OK) {
        ESP_LOGE(TAG_WEB, "httpd_start falhou");
        vTaskDelay(portMAX_DELAY);
    }

    static const httpd_uri_t uris[] = {
        { .uri="/status", .method=HTTP_GET,  .handler=http_status,  .user_ctx=NULL },
        { .uri="/move_j", .method=HTTP_POST, .handler=http_move_j,  .user_ctx=NULL },
        { .uri="/estop",  .method=HTTP_POST, .handler=http_estop,   .user_ctx=NULL },
        { .uri="/mode",   .method=HTTP_POST, .handler=http_mode,    .user_ctx=NULL },
        { .uri="/jitter", .method=HTTP_GET,  .handler=http_jitter,  .user_ctx=NULL },
        { .uri="/jitter/reset", .method=HTTP_POST, .handler=http_jitter_reset, .user_ctx=NULL },
    };
    for (int i = 0; i < 6; ++i) httpd_register_uri_handler(server, &uris[i]);

    ESP_LOGI(TAG_WEB, "HTTP server ativo: /status /move_j /estop");
    vTaskDelay(portMAX_DELAY);
}

// ============================================================================
// task_rtde — Core 0, prioridade 4
// ============================================================================

static bool recv_command_frame(int fd, eb15_rtde::CommandFrame *frame) {
    uint8_t *dst = reinterpret_cast<uint8_t *>(frame);
    size_t received = 0;
    while (received < sizeof(*frame)) {
        const int n = recv(fd, dst + received, sizeof(*frame) - received, 0);
        if (n <= 0) return false;
        received += static_cast<size_t>(n);
    }
    return true;
}

static void fill_telemetry(eb15_rtde::TelemetryFrame *frame) {
    xSemaphoreTake(s_joints_mutex, portMAX_DELAY);
    for (int i = 0; i < 6; ++i) {
        frame->q_deg[i] = g_joint_angles[i];
        frame->error_deg[i] = s_seg_active ? s_active_seg.q_deg[i] - g_joint_angles[i] : 0.0f;
    }
    xSemaphoreGive(s_joints_mutex);
    frame->temperature_c = g_control_loop_jitter_us;
}

static void task_rtde(void *pv) {
    ESP_LOGI(TAG_RTDE, "task_rtde iniciada (TCP :30003, telemetria 52 B @ 50 Hz)");
    const int listen_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (listen_fd < 0) { ESP_LOGE(TAG_RTDE, "socket falhou"); vTaskDelete(NULL); }
    int yes = 1;
    setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(30003);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(listen_fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0 ||
        listen(listen_fd, 1) != 0) {
        ESP_LOGE(TAG_RTDE, "bind/listen falhou"); close(listen_fd); vTaskDelete(NULL);
    }

    for (;;) {
        const int fd = accept(listen_fd, NULL, NULL);
        if (fd < 0) continue;
        timeval timeout{0, 200000};
        setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
        eb15_rtde::Session session;
        session.setMode(static_cast<eb15_rtde::OperationMode>(
            atomic_load_explicit(&s_operation_mode, memory_order_acquire)));
        ESP_LOGI(TAG_RTDE, "cliente conectado");

        int64_t tlm_t0 = esp_timer_get_time();
        int64_t tlm_last = tlm_t0, tlm_maxgap = 0;
        int tlm_n = 0;
        while (true) {
            eb15_rtde::TelemetryFrame telemetry{};
            fill_telemetry(&telemetry);
            if (send(fd, &telemetry, sizeof(telemetry), 0) != sizeof(telemetry)) break;
            {   /* taxa de telemetria + maior intervalo entre envios (detecta stall do loop) */
                const int64_t tnow = esp_timer_get_time();
                const int64_t gap = tnow - tlm_last; tlm_last = tnow;
                if (gap > tlm_maxgap) tlm_maxgap = gap;
                ++tlm_n;
                if (tnow - tlm_t0 >= 1000000) {
                    ESP_LOGI("TMG", "TELEM envios/s=%d maior_intervalo=%lld ms",
                             tlm_n, (long long)(tlm_maxgap / 1000));
                    tlm_n = 0; tlm_maxgap = 0; tlm_t0 = tnow;
                }
            }

            fd_set read_set;
            FD_ZERO(&read_set);
            FD_SET(fd, &read_set);
            timeval poll{0, 20000};
            const int ready = select(fd + 1, &read_set, NULL, NULL, &poll);
            if (ready > 0) {
                eb15_rtde::CommandFrame cmd{};
                if (!recv_command_frame(fd, &cmd)) break;
                eb15_rtde::CommandFrame decoded{};
                const uint32_t now_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);
                const auto result = session.decode(reinterpret_cast<uint8_t *>(&cmd),
                                                    sizeof(cmd), now_ms, decoded);
                if (result != eb15_rtde::DecodeResult::OK) continue;

                const auto type = static_cast<eb15_rtde::CommandType>(decoded.type);
                if (type == eb15_rtde::CommandType::MOVE_J) {
                    float q[6];
                    memcpy(q, decoded.q_deg, sizeof(q));
                    const int64_t t_rx = esp_timer_get_time();
                    ESP_LOGI("TMG", "RX MOVE_J q=[%.1f %.1f %.1f %.1f %.1f %.1f]",
                             q[0], q[1], q[2], q[3], q[4], q[5]);
                    plan_move_j_locked(q, 100.0f);
                    ESP_LOGI("TMG", "PLAN_LOCKED retornou: bloqueou RTDE/telemetria por %lld ms",
                             (long long)((esp_timer_get_time() - t_rx) / 1000));
                    session.setState(eb15_rtde::RobotState::MOVING);
                } else if (type == eb15_rtde::CommandType::ESTOP) {
                    stop_motion();
                } else if (type == eb15_rtde::CommandType::SET_MODE_USER ||
                           type == eb15_rtde::CommandType::SET_MODE_RTDE) {
                    const auto requested = type == eb15_rtde::CommandType::SET_MODE_USER
                        ? eb15_rtde::OperationMode::USER : eb15_rtde::OperationMode::RTDE;
                    switch_operation_mode(requested);
                    session.setState(eb15_rtde::RobotState::IDLE);
                    session.setMode(requested);
                }
            }

            if (session.state() == eb15_rtde::RobotState::MOVING && !motion_active())
                session.setState(eb15_rtde::RobotState::DONE);
            const uint32_t now_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);
            if (session.watchdogExpired(now_ms)) {
                ESP_LOGE(TAG_RTDE, "heartbeat >500 ms durante movimento: E-STOP");
                stop_motion();
            }
        }
        close(fd);
        if (motion_active()) stop_motion();
        ESP_LOGW(TAG_RTDE, "cliente desconectado");
    }
}

// ============================================================================
// Inicialização de GPIO
// ============================================================================

#ifdef EB15_QEMU
/* Abre uma conexão de encoder ao host (relé). Retorna fd ou -1. */
static int qemu_enc_connect(uint16_t port) {
    const int fd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (fd < 0) return -1;
    timeval timeout{0, 100000};
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    inet_pton(AF_INET, "10.0.2.2", &addr.sin_addr);
    if (connect(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
        close(fd);
        return -1;
    }
    return fd;
}

/* Leitura AS5600 ONE-SHOT (connect+ping+recv+close) — usada para J1-J3 (30103), cujo
 * caminho já capturava bem a rampa; manter idêntico ao histórico evita regressão. */
static bool qemu_read_as5600_oneshot(uint16_t port, uint16_t raw[3]) {
    const int fd = qemu_enc_connect(port);
    if (fd < 0) return false;
    const uint8_t ping = 1;
    if (send(fd, &ping, 1, 0) != 1) { close(fd); return false; }
    uint8_t *dst = reinterpret_cast<uint8_t *>(raw);
    size_t received = 0;
    while (received < 6) {
        const int n = recv(fd, dst + received, 6 - received, 0);
        if (n <= 0) { close(fd); return false; }
        received += static_cast<size_t>(n);
    }
    close(fd);
    return true;
}

/* Leitura AS5600 por conexão PERSISTENTE (*fd reusado entre ciclos; reconecta se
 * cair) — usada para J4-J6 (30104). Motivo: connect/close por leitura através do NIC
 * emulado do QEMU disputa o NIC com o tráfego UART do movimento e STARVA as leituras
 * — J4-J6 liam 0 durante todo o movimento e só o alvo no fim (degrau). Socket aberto
 * → cada leitura é ping+recv (rápida, ~40 Hz) → CAPTURA a rampa (itens 1/2/3). */
static bool qemu_read_as5600(int *fd, uint16_t port, uint16_t raw[3]) {
    if (*fd < 0) {
        *fd = qemu_enc_connect(port);
        if (*fd < 0) return false;
    }
    const uint8_t ping = 1;
    if (send(*fd, &ping, 1, 0) != 1) { close(*fd); *fd = -1; return false; }
    uint8_t *dst = reinterpret_cast<uint8_t *>(raw);
    size_t received = 0;
    while (received < 6) {
        const int n = recv(*fd, dst + received, 6 - received, 0);
        if (n <= 0) { close(*fd); *fd = -1; return false; }
        received += static_cast<size_t>(n);
    }
    return true;
}

static void task_qemu_encoder_feedback(void *) {
    ESP_LOGI(TAG_CTRL, "QEMU HIL: feedback AS5600 em 10.0.2.2:30103/30104 (conexao persistente)");
    int fd_uno = -1;
    int64_t rate_t0 = esp_timer_get_time();
    int rd_uno_ok = 0, rd_uno_fail = 0, rd_esp_ok = 0, rd_esp_fail = 0;
    for (;;) {
        uint16_t raw_esp[3]{};
        uint16_t raw_uno[3]{};
#if EB15_LOCKSTEP
        bool got_esp = false;  /* J1-J3 chega pelo endpoint lockstep :30101 (não ler 30103) */
        (void)raw_esp;
#else
        bool got_esp = qemu_read_as5600_oneshot(30103, raw_esp);  /* J1-J3: one-shot (persistente quebra o controle: leitura stale → PID dispara) */
#endif
        const bool got_uno = qemu_read_as5600(&fd_uno, 30104, raw_uno);  /* J4-J6: persistente (malha aberta, seguro) */
        if (got_uno) ++rd_uno_ok; else ++rd_uno_fail;
        if (got_esp) ++rd_esp_ok; else ++rd_esp_fail;
        const int64_t now_r = esp_timer_get_time();
        if (now_r - rate_t0 >= 1000000) {  /* taxa de leitura de encoder por segundo */
            ESP_LOGI("TMG", "ENC leituras/s: J1-3 ok=%d fail=%d | J4-6 ok=%d fail=%d | J1=%.1f",
                     rd_esp_ok, rd_esp_fail, rd_uno_ok, rd_uno_fail, g_joint_angles[0]);
            rd_uno_ok = rd_uno_fail = rd_esp_ok = rd_esp_fail = 0;
            rate_t0 = now_r;
        }
        static float s_prev_enc13[3] = {0.0f, 0.0f, 0.0f};
        static bool  s_seen_nonzero13 = false;

        /* Rejeição de FRAME-ZERO espúrio (J1-J3): o transporte de encoder do QEMU
         * às vezes devolve um frame com os 3 raw = 0 ("frame zero"), fazendo TODAS
         * as juntas caírem a 0 sincronizadamente e voltarem — artefato de CAPTAÇÃO,
         * não de controle (o firmware regula bem; error_code=0). Se já houve leitura
         * válida fora de casa e chega um frame todo-zero, descarta-o. Em casa (J1-J3
         * já ~0) um zero é legítimo e passa. */
        if (got_esp && raw_esp[0] == 0 && raw_esp[1] == 0 && raw_esp[2] == 0 &&
            s_seen_nonzero13 &&
            (fabsf(s_prev_enc13[0]) > 2.0f || fabsf(s_prev_enc13[1]) > 2.0f ||
             fabsf(s_prev_enc13[2]) > 2.0f)) {
            got_esp = false;  /* frame-zero espúrio → ignora J1-J3 neste ciclo */
        }

        if (got_esp || got_uno) {
            /* Rejeição de salto não-físico em J1-J3 (malha fechada): a junta move
             * no máximo ~1°/20ms; um salto > MAXJUMP_DEG indica leitura de encoder
             * corrompida por transitório de transporte do QEMU. Descartá-la mantém
             * a última leitura válida e impede o PID de disparar (J1 até o limite /
             * braço no chão). J4-J6 não filtram aqui: são malha aberta e J6 pode dar
             * volta legítima (±360°). */
            const float MAXJUMP_DEG = 45.0f;
            xSemaphoreTake(s_joints_mutex, portMAX_DELAY);
            for (int i = 0; i < 3; ++i) {
                if (got_esp) {
                    float deg = static_cast<float>(raw_esp[i] & 0x0FFFu) * 360.0f / 4096.0f;
                    if (deg > 180.0f) deg -= 360.0f;
                    if (fabsf(deg - s_prev_enc13[i]) <= MAXJUMP_DEG) {
                        g_joint_angles[i] = deg;
                        s_prev_enc13[i] = deg;
                        if (fabsf(deg) > 2.0f) s_seen_nonzero13 = true;
                    }   /* senão: salto espúrio → mantém leitura anterior */
                }
                if (got_uno) {
                    float deg = static_cast<float>(raw_uno[i] & 0x0FFFu) * 360.0f / 4096.0f;
                    g_joint_angles[3 + i] = deg > 180.0f ? deg - 360.0f : deg;
                }
            }
            xSemaphoreGive(s_joints_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static void task_qemu_stepdir_server(void *) {
    const int listen_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    configASSERT(listen_fd >= 0);
    int yes = 1;
    setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(30101);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    configASSERT(bind(listen_fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) == 0);
    configASSERT(listen(listen_fd, 2) == 0);
    ESP_LOGI(TAG_CTRL, "QEMU HIL: Step/Dir J1-J3 ativo em TCP :30101 (lockstep=%d)", EB15_LOCKSTEP);
    for (;;) {
        const int fd = accept(listen_fd, NULL, NULL);
        if (fd < 0) continue;
#if EB15_LOCKSTEP
        /* Endpoint LOCKSTEP (conexão persistente da ponte2). Por requisição:
         *   1. recebe o encoder MEDIDO de J1-J3 (3x uint16 AS5600, do Webots);
         *   2. fixa g_joint_angles[0..2] = medido (feedback fresco e sincronizado);
         *   3. roda UM ciclo de controle (PID+Tanh, dt=5ms);
         *   4. devolve o COMANDO (6 floats: passos+graus).
         * Assim o controle de J1-J3 corre EXATAMENTE 1x por passo do Webots — sem
         * staleness e sem espera de relógio de parede. */
        int one = 1; setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
        bool s_ls_seeded = false;   /* 1ª amostra adota o medido direto (seed do estimador) */
        for (;;) {
            /* Pedido: 6 bytes = 3x uint16 (J1-J3 encoder MEDIDO do Webots). J4-J6 segue
             * pelo caminho do modo livre (task_uart_master + ponte3), em malha fechada no
             * próprio Uno (sync por tempo-real); ver project_passo4_lockstep. */
            uint8_t meas[6];
            int got = 0; bool ok = true;
            while (got < 6) { int n = recv(fd, meas + got, 6 - got, 0); if (n <= 0) { ok = false; break; } got += n; }
            if (!ok) break;
            xSemaphoreTake(s_joints_mutex, portMAX_DELAY);
            for (int i = 0; i < 3; ++i) {
                uint16_t raw = (uint16_t)(meas[2*i] | ((uint16_t)meas[2*i+1] << 8)) & 0x0FFFu;
                float deg = (float)raw * 360.0f / 4096.0f;
                if (deg > 180.0f) deg -= 360.0f;
                g_ls_meas[i] = deg;
                if (!s_ls_seeded) g_joint_angles[i] = deg;
                else g_joint_angles[i] += LOCKSTEP_ENC_ALPHA * (deg - g_joint_angles[i]);
            }
            s_ls_seeded = true;
            xSemaphoreGive(s_joints_mutex);
            run_control_cycle();
            static int s_ls_cyc = 0;
            if ((++s_ls_cyc % 400) == 0) {
                const int qd = (g_seg_head - g_seg_tail + MAX_SEGMENTS) % MAX_SEGMENTS;
                ESP_LOGI("TMG", "LOCKSTEP cyc=%d meas0=%.1f segTgt0=%.1f cmd0=%.1f tks=%d qd=%d",
                         s_ls_cyc, g_joint_angles[0], s_active_seg.q_deg[0],
                         s_qemu_command_deg[0], s_seg_ticks_remaining, qd);
            }
            float payload[6];
            xSemaphoreTake(s_joints_mutex, portMAX_DELAY);
            for (int i = 0; i < 3; ++i) {
                payload[i] = s_qemu_command_deg[i] * STEPS_PER_DEG[i];
                payload[3 + i] = s_qemu_command_deg[i];
            }
            xSemaphoreGive(s_joints_mutex);
            uint8_t *p = (uint8_t *)payload; int sent = 0; const int tot = (int)sizeof(payload);
            while (sent < tot) { int n = send(fd, p + sent, tot - sent, 0); if (n <= 0) { ok = false; break; } sent += n; }
            if (!ok) break;
        }
        close(fd);
#else
        /* ONE-SHOT (modo livre): só serve o comando atual. */
        uint8_t ping = 0;
        if (recv(fd, &ping, 1, 0) == 1) {
            float payload[6]{};
            xSemaphoreTake(s_joints_mutex, portMAX_DELAY);
            for (int i = 0; i < 3; ++i) {
                payload[i] = s_qemu_command_deg[i] * STEPS_PER_DEG[i];
                payload[3 + i] = s_qemu_command_deg[i];
            }
            xSemaphoreGive(s_joints_mutex);
            send(fd, payload, sizeof(payload), 0);
        }
        close(fd);
#endif
    }
}
#endif

static void init_gpio(void) {
#ifndef EB15_QEMU
    gpio_config_t out_cfg = {
        .pin_bit_mask = (1ULL << PUL_J1) | (1ULL << DIR_J1) |
                        (1ULL << PUL_J2) | (1ULL << DIR_J2) |
                        (1ULL << PUL_J3) | (1ULL << DIR_J3) |
                        (1ULL << TRIGGER_PIN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&out_cfg);
    gpio_set_level(TRIGGER_PIN, 1);  /* TRIGGER inativo em HIGH */
#else
    ESP_LOGI(TAG_BOOT, "QEMU: GPIO init ignorado");
#endif
}

// ============================================================================
// Inicialização de UART2
// ============================================================================

static void init_uart2(void) {
    uart_config_t cfg = {
        .baud_rate  = UART2_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
    };
#ifdef EB15_QEMU
    /* QEMU: usa UART1 (2º -serial arg) — mais confiável que UART2 no emulador.
     * No hardware real, UART_PORT = UART_NUM_2 (GPIO19/20). */
    uart_driver_install(UART_NUM_1, 512, 512, 0, NULL, ESP_INTR_FLAG_IRAM);
    uart_param_config(UART_NUM_1, &cfg);
    ESP_LOGI(TAG_BOOT, "QEMU: UART1 (bridge) driver instalado, pinos GPIO ignorados");
#else
    uart_driver_install(UART_NUM_2, 256, 256, 0, NULL, 0);
    uart_param_config(UART_NUM_2, &cfg);
    uart_set_pin(UART_NUM_2, UART2_TX_PIN, UART2_RX_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
#endif
}

// ============================================================================
// app_main — Ponto de entrada ESP-IDF
// ============================================================================

extern "C" void app_main(void) {
    ESP_LOGI(TAG_BOOT, "EB15 Mestre ESP32-S3 iniciando...");

#ifdef EB15_QEMU
    ESP_LOGI(TAG_BOOT, "Modo QEMU ativo");
#endif

    /* --- Recursos de sincronização --- */
    s_ctrl_sem     = xSemaphoreCreateBinary();
    s_joints_mutex = xSemaphoreCreateMutex();
    s_seg_mutex    = xSemaphoreCreateMutex();
    s_uart_queue   = xQueueCreate(80, sizeof(UartJob));  /* comporta as fatias S-curve de um MoveJ */
    configASSERT(s_ctrl_sem && s_joints_mutex && s_seg_mutex && s_uart_queue);

    /* --- Hardware --- */
    init_gpio();
    init_uart2();

    /* --- Timer de controle 200 Hz (Core 1, hardware) --- */
#ifndef EB15_QEMU
    esp_timer_handle_t ctrl_timer;
    esp_timer_create_args_t targs = {
        .callback        = ctrl_timer_cb,
        .arg             = NULL,
        .dispatch_method = ESP_TIMER_ISR,
        .name            = "ctrl_200hz",
        .skip_unhandled_events = true,
    };
    esp_timer_create(&targs, &ctrl_timer);
    esp_timer_start_periodic(ctrl_timer, TIMER_PERIOD_US);
    ESP_LOGI(TAG_BOOT, "Timer de controle 200 Hz iniciado");
#endif

    /* --- Tarefas Core 1 (tempo real) --- */
    xTaskCreatePinnedToCore(task_control,      "ctrl",      4096, NULL, 10, NULL, 1);
    xTaskCreatePinnedToCore(task_uart_master,  "uart_mst",  4096, NULL,  8, NULL, 1);

    /* --- Tarefas Core 0 (rede) --- */
    s_wifi_ready = xSemaphoreCreateBinary();
    xTaskCreatePinnedToCore(task_wifi,      "wifi",    4096, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(task_webserver, "http",    8192, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(task_rtde,      "rtde",    4096, NULL, 4, NULL, 0);
#ifdef EB15_QEMU
    xTaskCreatePinnedToCore(task_qemu_stepdir_server, "hil_step", 4096, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(task_qemu_encoder_feedback, "hil_enc", 4096, NULL, 3, NULL, 0);
#endif

    ESP_LOGI(TAG_BOOT, "[BOOT OK] Todas as tarefas criadas");

#if 0  /* Trajetorias QEMU entram somente por RTDE/User; nao mover no boot. */
    /* Demo: enfileira um MoveJ de teste para exercitar o laço de controle.
     * Executado ANTES das tasks iniciarem para evitar race na fila. */
    {
        const float demo[6] = { 30.0f, 20.0f, -15.0f, 45.0f, 0.0f, 0.0f };
        plan_move_j_scurve(demo, 50.0f);
        const int ns = (g_seg_head - g_seg_tail + MAX_SEGMENTS) % MAX_SEGMENTS;
        ESP_LOGI(TAG_BOOT, "QEMU: demo MoveJ enfileirado — %d segmentos de %d ms", ns, SEG_SLICE_MS);
    }
#endif

    /* app_main pode retornar — FreeRTOS continua as tarefas */
}
