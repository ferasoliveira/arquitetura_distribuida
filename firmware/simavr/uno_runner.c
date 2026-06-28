/*
 * uno_runner.c — Harness do firmware escravo (Arduino UNO .ino REAL) no simavr.
 *
 * Arquitetura canônica v2 (wiki/plano_acao_v2.md): o .elf compilado por arduino-cli
 * roda cycle-accurate no simavr; este harness liga libsimavr e EXPOE as interfaces
 * elétricas do ATmega328P para as pontes Python (relés) via TCP:
 *
 *   - UART  (USART0, D0/D1 @115200) ........ bytes bidirecionais  -> porta TCP --uart-port
 *   - Trigger (D9 = PB1 = PCINT1) ........... linha digital de disparo (drive) -> --ctrl-port
 *   - E-STOP  (A0 = PC0 = PCINT8, ativo LOW)  drive pela mesma --ctrl-port
 *   - Step/Dir J4-J6 (STEP D2/D4/D6, DIR D3/D5/D7) .. eventos de passo -> --step-port
 *   - Encoder AS5600 (I2C bit-bang em PC2/PC3) ...... escravo I2C alimentado -> --enc-port
 *
 * NENHUM mock/reimplementação de firmware: este programa apenas executa o .elf real
 * e repassa níveis elétricos. A lógica de protocolo (handshake, DDA, ISR, I2C) é
 * inteiramente do firmware dentro do simavr.
 *
 * Modos:
 *   --smoke                 self-test (Etapa 2): injeta frame válido -> ACK + passos;
 *                           frame com checksum inválido -> NAK. Exit 0 = PASS.
 *   --serve                 abre os servidores TCP para as pontes (Etapa 3+).
 *
 * Build (dentro do container eb15-simavr):
 *   gcc -O2 -o uno_runner uno_runner.c -lsimavr -lsimavrparts -lpthread -lelf
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
#include <pthread.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <fcntl.h>

#include <simavr/sim_avr.h>
#include <simavr/sim_elf.h>
#include <simavr/avr_uart.h>
#include <simavr/avr_ioport.h>
#include <simavr/sim_irq.h>

/* ------------------------------------------------------------------ */
/* Mapa de pinos do firmware (config.h do arduino_escravo)            */
/*   STEP J4/J5/J6 = D2/D4/D6 = PD2/PD4/PD6                            */
/*   DIR  J4/J5/J6 = D3/D5/D7 = PD3/PD5/PD7                            */
/*   TRIGGER       = D9       = PB1 (PCINT1 -> PCINT0_vect)            */
/*   ESTOP         = A0       = PC0 (PCINT8 -> PCINT1_vect, ativo LOW) */
/*   I2C SDA/SCL   = A2/A3    = PC2/PC3 (bit-bang)                     */
/* ------------------------------------------------------------------ */
#define PORT_B 'B'
#define PORT_C 'C'
#define PORT_D 'D'
#define PIN_TRIGGER 1   /* PB1 */
#define PIN_ESTOP   0   /* PC0 */
#define PIN_SDA     2   /* PC2 */
#define PIN_SCL     3   /* PC3 */

static const uint8_t STEP_BITS[3] = {2, 4, 6}; /* PD2 PD4 PD6 */
static const uint8_t DIR_BITS[3]  = {3, 5, 7}; /* PD3 PD5 PD7 */

/* Bytes de protocolo (protocol_core.hpp) */
#define UART_ACK   0x06
#define UART_NAK   0x15
#define UART_BUSY  0x12
#define UART_DONE  0x04
#define UART_ESTOP 0x05
#define FRAME_PREAMBLE 0xAA

/* ------------------------------------------------------------------ */
/* Estado global                                                      */
/* ------------------------------------------------------------------ */
static avr_t *g_avr = NULL;
static avr_irq_t *g_uart_in_irq = NULL;   /* injeta byte no RX do AVR  */
static avr_irq_t *g_trig_irq = NULL;      /* drive D9 (PB1)            */
static avr_irq_t *g_estop_irq = NULL;     /* drive A0 (PC0)            */
static avr_irq_t *g_sda_irq = NULL;       /* barramento bit-bang PC2    */
static avr_irq_t *g_scl_irq = NULL;       /* barramento bit-bang PC3    */

static volatile int g_uart_ready = 1;     /* XON/XOFF do UART do simavr */
static double g_pace = 0.0;                /* >0 = ritma o AVR ao tempo real (fator) */

/* Bytes que o AVR transmitiu (TX), capturados pelo callback OUTPUT.   */
#define TXCAP 4096
static uint8_t g_avr_tx[TXCAP];
static volatile int g_avr_tx_n = 0;

/* FIFO de bytes a injetar no RX do AVR (respeitando XON).             */
#define RXFIFO 1024
static uint8_t g_rx_fifo[RXFIFO];
static volatile int g_rx_head = 0, g_rx_tail = 0;
static pthread_mutex_t g_rx_lock = PTHREAD_MUTEX_INITIALIZER;

/* Contagem de passos observados por eixo (rising edge do STEP).      */
static volatile uint32_t g_steps[3] = {0, 0, 0};
static volatile uint8_t  g_dir[3]   = {1, 1, 1};
static uint8_t g_step_prev[3] = {0, 0, 0};

/* Captura de tempo para VCD (modo --vcd, ensaio elétrico isolado T3/J_start):
 * ciclos das bordas de SUBIDA do STEP J4 (PD2) e da borda de DESCIDA do
 * trigger D9. Cycle-accurate: o relógio do AVR é a única referência (62,5 ns). */
static volatile int g_vcd_rec = 0;
#define VCDCAP 8192
static uint64_t g_vcd_step_cycle[VCDCAP];
static volatile int g_vcd_step_n = 0;
static uint64_t g_vcd_trig_cycle = 0;

/* Ground truth recebido da ponte 3. Sem amostra válida, o escravo I2C
 * permanece indisponível: nunca convertemos o zero de inicialização em
 * uma medição fictícia. */
static uint16_t g_encoder_raw[3] = {0, 0, 0};
static volatile int g_encoder_valid = 0;
static pthread_mutex_t g_encoder_lock = PTHREAD_MUTEX_INITIALIZER;
static volatile int g_i2c_enabled = 0;
static volatile uint32_t g_i2c_reads = 0;
static uint8_t g_master_sda_low = 0;
static uint8_t g_slave_sda_low = 0;

/* Escravo GPIO open-drain TCA9548A + AS5600. O firmware é o mestre e usa
 * PC2/PC3 por bit-bang; portanto o periférico TWI do simavr não participa. */
typedef enum { I2C_RX, I2C_TX, I2C_MASTER_ACK } i2c_phase_t;
static struct {
    int active;
    uint8_t scl;
    uint8_t sda;
    uint8_t rx_byte;
    uint8_t bit_count;
    uint8_t ack_active;
    uint8_t ack_value;
    uint8_t start_tx_after_ack;
    uint8_t address;
    uint8_t selected_axis;
    uint8_t selected_reg;
    uint8_t tx_bytes[2];
    uint8_t tx_index;
    i2c_phase_t phase;
} g_i2c = {0};

/* ------------------------------------------------------------------ */
/* Callbacks simavr                                                   */
/* ------------------------------------------------------------------ */
static void tx2s_push(uint8_t b);     /* fwd */
static void stp2s_push(uint8_t rec);  /* fwd */
static volatile int g_serving;        /* def. tentativa; inicializada adiante */

static void uart_out_cb(struct avr_irq_t *irq, uint32_t value, void *p) {
    (void)irq; (void)p;
    if (g_avr_tx_n < TXCAP) g_avr_tx[g_avr_tx_n++] = (uint8_t)value;
    if (g_serving) tx2s_push((uint8_t)value);  /* AVR TX -> socket UART (ponte 1) */
}

static void uart_xon_cb(struct avr_irq_t *irq, uint32_t value, void *p) {
    (void)irq; (void)value; (void)p; g_uart_ready = 1;
}
static void uart_xoff_cb(struct avr_irq_t *irq, uint32_t value, void *p) {
    (void)irq; (void)value; (void)p; g_uart_ready = 0;
}

static void step_cb(struct avr_irq_t *irq, uint32_t value, void *p) {
    (void)irq;
    intptr_t axis = (intptr_t)p;
    uint8_t v = value ? 1 : 0;
    if (v && !g_step_prev[axis]) {                  /* rising edge */
        g_steps[axis]++;
        if (g_serving) stp2s_push((uint8_t)(axis | (g_dir[axis] ? 0x10 : 0x00)));
        if (g_vcd_rec && axis == 0 && g_vcd_step_n < VCDCAP && g_avr)
            g_vcd_step_cycle[g_vcd_step_n++] = (uint64_t)g_avr->cycle;
    }
    g_step_prev[axis] = v;
}
static void dir_cb(struct avr_irq_t *irq, uint32_t value, void *p) {
    intptr_t axis = (intptr_t)p;
    g_dir[axis] = value ? 1 : 0;
}

/* Timer de ciclo: drena a FIFO de RX para o AVR respeitando XON.     */
static avr_cycle_count_t rx_pump(avr_t *avr, avr_cycle_count_t when, void *p) {
    (void)p;
    if (g_uart_ready) {
        pthread_mutex_lock(&g_rx_lock);
        if (g_rx_head != g_rx_tail) {
            uint8_t b = g_rx_fifo[g_rx_tail];
            g_rx_tail = (g_rx_tail + 1) % RXFIFO;
            pthread_mutex_unlock(&g_rx_lock);
            avr_raise_irq(g_uart_in_irq, b);
        } else {
            pthread_mutex_unlock(&g_rx_lock);
        }
    }
    /* re-arma a cada ~80us (1280 ciclos @16MHz): ~1 byte/80us << 115200bps cap */
    return when + 1280;
}

static void rx_push(uint8_t b) {
    pthread_mutex_lock(&g_rx_lock);
    int nh = (g_rx_head + 1) % RXFIFO;
    if (nh != g_rx_tail) { g_rx_fifo[g_rx_head] = b; g_rx_head = nh; }
    pthread_mutex_unlock(&g_rx_lock);
}

/* ------------------------------------------------------------------ */
/* Drive de linhas digitais (trigger / estop)                         */
/* ------------------------------------------------------------------ */
static void trigger_set(int high) { if (g_trig_irq) avr_raise_irq(g_trig_irq, high ? 1 : 0); }
static void estop_set(int high)   { if (g_estop_irq) avr_raise_irq(g_estop_irq, high ? 1 : 0); }

/* ------------------------------------------------------------------ */
/* Avanço do tempo simulado (em us)                                   */
/* ------------------------------------------------------------------ */
static int run_us(avr_t *avr, uint32_t us) {
    avr_cycle_count_t target = avr->cycle + (avr_cycle_count_t)((uint64_t)us * avr->frequency / 1000000ULL);
    while (avr->cycle < target) {
        int st = avr_run(avr);
        if (st == cpu_Done || st == cpu_Crashed) return st;
    }
    return avr->state;
}

/* ------------------------------------------------------------------ */
/* Escravo I2C em GPIO (PC2 SDA / PC3 SCL)                            */
/* ------------------------------------------------------------------ */
static void i2c_drive_sda(int high) {
    if (!g_sda_irq) return;
    g_slave_sda_low = high ? 0 : 1;
    avr_raise_irq(g_sda_irq, high ? 1 : 0); /* HIGH=release/pull-up, LOW=drive */
    if (!g_master_sda_low) g_i2c.sda = high ? 1 : 0;
}

static int i2c_axis_from_mask(uint8_t mask) {
    if (mask == (1u << 3)) return 0;
    if (mask == (1u << 4)) return 1;
    if (mask == (1u << 5)) return 2;
    return -1;
}

static void i2c_begin(void) {
    g_i2c.active = 1;
    g_i2c.phase = I2C_RX;
    g_i2c.rx_byte = 0;
    g_i2c.bit_count = 0;
    g_i2c.ack_active = 0;
    g_i2c.start_tx_after_ack = 0;
    g_i2c.address = 0;
    i2c_drive_sda(1);
}

static void i2c_end(void) {
    g_i2c.active = 0;
    g_i2c.phase = I2C_RX;
    g_i2c.bit_count = 0;
    g_i2c.ack_active = 0;
    i2c_drive_sda(1);
}

static void i2c_prepare_tx(void) {
    uint16_t raw = 0;
    pthread_mutex_lock(&g_encoder_lock);
    raw = g_encoder_raw[g_i2c.selected_axis] & 0x0FFFu;
    pthread_mutex_unlock(&g_encoder_lock);
    g_i2c.tx_bytes[0] = (uint8_t)(raw >> 8);
    g_i2c.tx_bytes[1] = (uint8_t)raw;
    g_i2c.tx_index = 0;
    g_i2c.start_tx_after_ack = 1;
}

static int i2c_accept_byte(uint8_t byte) {
    if (g_i2c.address == 0) {
        g_i2c.address = byte;
        const uint8_t addr = byte >> 1;
        const int read = byte & 1;
        if (addr == 0x70 && !read) return g_encoder_valid;
        if (addr == 0x36 && !read) return g_encoder_valid && g_i2c.selected_axis < 3;
        if (addr == 0x36 && read && g_encoder_valid &&
            g_i2c.selected_axis < 3 && g_i2c.selected_reg == 0x0C) {
            i2c_prepare_tx();
            return 1;
        }
        return 0;
    }
    if ((g_i2c.address >> 1) == 0x70) {
        const int axis = i2c_axis_from_mask(byte);
        if (axis < 0) return 0;
        g_i2c.selected_axis = (uint8_t)axis;
        return 1;
    }
    if ((g_i2c.address >> 1) == 0x36 && !(g_i2c.address & 1)) {
        g_i2c.selected_reg = byte;
        return byte == 0x0C;
    }
    return 0;
}

static void i2c_sda_cb(struct avr_irq_t *irq, uint32_t value, void *p) {
    (void)irq; (void)p;
    const uint8_t next = value ? 1 : 0;
    if (!g_i2c_enabled) { g_i2c.sda = next; return; }
    if (g_i2c.scl) {
        if (g_i2c.sda && !next) i2c_begin();      /* START/repeated START */
        else if (!g_i2c.sda && next) i2c_end();   /* STOP */
    }
    g_i2c.sda = next;
}

static void i2c_set_tx_bit(void) {
    const uint8_t byte = g_i2c.tx_bytes[g_i2c.tx_index];
    const uint8_t bit = (byte >> (7 - g_i2c.bit_count)) & 1u;
    i2c_drive_sda(bit ? 1 : 0);
}

static void i2c_scl_cb(struct avr_irq_t *irq, uint32_t value, void *p) {
    (void)irq; (void)p;
    const uint8_t next = value ? 1 : 0;
    if (!g_i2c_enabled) { g_i2c.scl = next; return; }
    if (!g_i2c.active || next == g_i2c.scl) { g_i2c.scl = next; return; }

    if (next) { /* rising edge: mestre amostra SDA */
        if (g_i2c.phase == I2C_RX && !g_i2c.ack_active) {
            g_i2c.rx_byte = (uint8_t)((g_i2c.rx_byte << 1) | (g_i2c.sda ? 1 : 0));
            if (++g_i2c.bit_count == 8) {
                g_i2c.ack_value = i2c_accept_byte(g_i2c.rx_byte) ? 1 : 0;
            }
        } else if (g_i2c.phase == I2C_TX) {
            g_i2c.bit_count++;
        } else if (g_i2c.phase == I2C_MASTER_ACK) {
            g_i2c.ack_value = g_i2c.sda ? 0 : 1;
        }
    } else { /* falling edge: escravo pode mudar SDA */
        if (g_i2c.phase == I2C_RX) {
            if (g_i2c.bit_count == 8 && !g_i2c.ack_active) {
                g_i2c.ack_active = 1;
                i2c_drive_sda(g_i2c.ack_value ? 0 : 1);
            } else if (g_i2c.ack_active) {
                g_i2c.ack_active = 0;
                g_i2c.bit_count = 0;
                g_i2c.rx_byte = 0;
                i2c_drive_sda(1);
                if (g_i2c.start_tx_after_ack) {
                    g_i2c.start_tx_after_ack = 0;
                    g_i2c.phase = I2C_TX;
                    i2c_set_tx_bit();
                }
            }
        } else if (g_i2c.phase == I2C_TX) {
            if (g_i2c.bit_count < 8) {
                i2c_set_tx_bit();
            } else {
                i2c_drive_sda(1);
                g_i2c.phase = I2C_MASTER_ACK;
            }
        } else if (g_i2c.phase == I2C_MASTER_ACK) {
            if (g_i2c.ack_value && g_i2c.tx_index == 0) {
                g_i2c.tx_index = 1;
                g_i2c.bit_count = 0;
                g_i2c.phase = I2C_TX;
                i2c_set_tx_bit();
            } else {
                if (g_i2c.tx_index == 1) {
                    uint32_t count = ++g_i2c_reads;
                    if (count <= 6 || count % 1000 == 0) {
                        uint16_t raw = (uint16_t)((g_i2c.tx_bytes[0] << 8) | g_i2c.tx_bytes[1]);
                        fprintf(stderr, "[i2c] AS5600 J%d raw=%u leitura=%u\n",
                                g_i2c.selected_axis + 4, raw, count);
                    }
                }
                g_i2c.phase = I2C_RX;
                g_i2c.bit_count = 0;
                i2c_drive_sda(1);
            }
        }
    }
    g_i2c.scl = next;
}

/* O bit-bang do firmware mantém PORTC=0 e alterna DDRC para obter open-drain.
 * A IRQ de direção é, portanto, a fonte fiel das bordas do mestre. */
static void i2c_direction_cb(struct avr_irq_t *irq, uint32_t value, void *p) {
    (void)irq; (void)p;
    if (!g_i2c_enabled) return;
    const uint8_t ddr = (uint8_t)value;
    g_master_sda_low = (ddr & (1u << PIN_SDA)) ? 1 : 0;
    const uint8_t new_sda = g_master_sda_low ? 0 : (g_slave_sda_low ? 0 : 1);
    const uint8_t new_scl = (ddr & (1u << PIN_SCL)) ? 0 : 1;
    /* A notificação de DDR ocorre antes de o simavr gravar DDRC. Sem
     * SET_EXTERNAL, o update subsequente preserva estes níveis quando o pino
     * vira entrada; isto modela pull-up + wired-AND sem sobrescrever o ACK. */
    avr_raise_irq(g_sda_irq, new_sda);
    avr_raise_irq(g_scl_irq, new_scl);
    if (new_sda != g_i2c.sda) i2c_sda_cb(NULL, new_sda, NULL);
    if (new_scl != g_i2c.scl) i2c_scl_cb(NULL, new_scl, NULL);
}

/* ------------------------------------------------------------------ */
/* Setup do simavr                                                    */
/* ------------------------------------------------------------------ */
static avr_t *make_avr(const char *elf_path) {
    elf_firmware_t fw;
    memset(&fw, 0, sizeof(fw));
    if (elf_read_firmware(elf_path, &fw) != 0) {
        fprintf(stderr, "[uno_runner] ERRO: nao consegui ler elf %s\n", elf_path);
        return NULL;
    }
    avr_t *avr = avr_make_mcu_by_name("atmega328p");
    if (!avr) { fprintf(stderr, "[uno_runner] ERRO: mcu atmega328p indisponivel\n"); return NULL; }
    avr_init(avr);
    avr->frequency = 16000000;
    fw.frequency = 16000000;
    avr_load_firmware(avr, &fw);

    /* UART '0': desliga dump para stdio; conecta OUTPUT/INPUT/XON/XOFF */
    uint32_t flags = 0;
    avr_ioctl(avr, AVR_IOCTL_UART_GET_FLAGS('0'), &flags);
    flags &= ~AVR_UART_FLAG_STDIO;
    avr_ioctl(avr, AVR_IOCTL_UART_SET_FLAGS('0'), &flags);

    avr_irq_register_notify(avr_io_getirq(avr, AVR_IOCTL_UART_GETIRQ('0'), UART_IRQ_OUTPUT), uart_out_cb, NULL);
    avr_irq_register_notify(avr_io_getirq(avr, AVR_IOCTL_UART_GETIRQ('0'), UART_IRQ_OUT_XON),  uart_xon_cb,  NULL);
    avr_irq_register_notify(avr_io_getirq(avr, AVR_IOCTL_UART_GETIRQ('0'), UART_IRQ_OUT_XOFF), uart_xoff_cb, NULL);
    g_uart_in_irq = avr_io_getirq(avr, AVR_IOCTL_UART_GETIRQ('0'), UART_IRQ_INPUT);

    /* Step/Dir J4-J6: observa PORTD */
    for (intptr_t i = 0; i < 3; i++) {
        avr_irq_register_notify(avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ(PORT_D), STEP_BITS[i]), step_cb, (void*)i);
        avr_irq_register_notify(avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ(PORT_D), DIR_BITS[i]),  dir_cb,  (void*)i);
    }

    /* Trigger (PB1) e E-STOP (PC0) como entradas dirigidas externamente */
    g_trig_irq  = avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ(PORT_B), PIN_TRIGGER);
    g_estop_irq = avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ(PORT_C), PIN_ESTOP);

    /* I2C bit-bang: observar e dirigir os próprios pinos GPIO. O drive externo
     * representa apenas pull-up/open-drain dos escravos físicos. */
    g_sda_irq = avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ(PORT_C), PIN_SDA);
    g_scl_irq = avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ(PORT_C), PIN_SCL);
    g_i2c.sda = g_i2c.scl = 1;
    avr_irq_register_notify(
        avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ(PORT_C), IOPORT_IRQ_DIRECTION_ALL),
        i2c_direction_cb, NULL);

    /* Bomba de RX por timer de ciclo */
    avr_cycle_timer_register(avr, 1280, rx_pump, NULL);

    return avr;
}

/* ================================================================== */
/* MODO SMOKE (Etapa 2)                                               */
/* ================================================================== */
static int wait_for_byte(avr_t *avr, uint8_t want, uint32_t timeout_us, const char **gotseq) {
    /* roda em janelas curtas ate aparecer 'want' na captura TX, ou timeout */
    uint32_t step = 200; /* us por janela */
    uint32_t elapsed = 0;
    int from = g_avr_tx_n;
    while (elapsed < timeout_us) {
        run_us(avr, step);
        elapsed += step;
        for (int i = from; i < g_avr_tx_n; i++) {
            if (g_avr_tx[i] == want) return 1;
        }
    }
    (void)gotseq;
    return 0;
}

static void build_frame(uint8_t *f, int16_t j4, int16_t j5, int16_t j6, uint16_t dur, int corrupt) {
    f[0] = FRAME_PREAMBLE;
    f[1] = j4 & 0xFF;  f[2] = (j4 >> 8) & 0xFF;
    f[3] = j5 & 0xFF;  f[4] = (j5 >> 8) & 0xFF;
    f[5] = j6 & 0xFF;  f[6] = (j6 >> 8) & 0xFF;
    f[7] = dur & 0xFF; f[8] = (dur >> 8) & 0xFF;
    uint8_t xor = 0;
    for (int i = 0; i < 9; i++) xor ^= f[i];
    f[9] = corrupt ? (uint8_t)(xor ^ 0xFF) : xor;
}

static void hexdump_tx(int from) {
    fprintf(stderr, "[smoke]   TX do AVR:");
    for (int i = from; i < g_avr_tx_n; i++) fprintf(stderr, " %02X", g_avr_tx[i]);
    fprintf(stderr, "\n");
}

static int smoke(const char *elf_path) {
    g_avr = make_avr(elf_path);
    if (!g_avr) return 2;
    int fails = 0;

    /* Linhas em repouso: trigger HIGH (pull-up), estop HIGH (inativo) */
    trigger_set(1);
    estop_set(1);

    /* 1) Boot: deixa setup() rodar e timers iniciarem (~300ms) */
    fprintf(stderr, "[smoke] boot 300ms...\n");
    run_us(g_avr, 300000);
    fprintf(stderr, "[smoke] apos boot: TX_n=%d (esperado 0 ou ESTOP se encoder falhar)\n", g_avr_tx_n);
    int estop_seen = 0;
    for (int i = 0; i < g_avr_tx_n; i++) if (g_avr_tx[i] == UART_ESTOP) estop_seen = 1;
    if (estop_seen) {
        fprintf(stderr, "[smoke] AVISO: ESTOP (0x05) emitido no boot -> encoder I2C precisa de escravo (Etapa 4)\n");
        hexdump_tx(0);
    }

    /* 2) Teste NAK: frame com checksum corrompido */
    fprintf(stderr, "[smoke] === teste NAK (checksum invalido) ===\n");
    int from = g_avr_tx_n;
    uint8_t frame[10];
    build_frame(frame, 10, 0, 0, 100, /*corrupt=*/1);
    for (int i = 0; i < 10; i++) rx_push(frame[i]);
    int got_nak = wait_for_byte(g_avr, UART_NAK, 50000, NULL);
    hexdump_tx(from);
    if (got_nak) fprintf(stderr, "[smoke]   -> NAK (0x15) recebido: OK\n");
    else { fprintf(stderr, "[smoke]   -> NAK NAO recebido: FALHA\n"); fails++; }

    /* 3) Teste ACK: frame valido J4=+10 passos, 100ms */
    fprintf(stderr, "[smoke] === teste ACK (frame valido J4=+10) ===\n");
    from = g_avr_tx_n;
    uint32_t steps0[3] = {g_steps[0], g_steps[1], g_steps[2]};
    build_frame(frame, 10, 0, 0, 100, /*corrupt=*/0);
    for (int i = 0; i < 10; i++) rx_push(frame[i]);
    int got_ack = wait_for_byte(g_avr, UART_ACK, 50000, NULL);
    hexdump_tx(from);
    if (got_ack) fprintf(stderr, "[smoke]   -> ACK (0x06) recebido: OK\n");
    else { fprintf(stderr, "[smoke]   -> ACK NAO recebido: FALHA\n"); fails++; }

    /* 4) Trigger: borda de descida em D9 inicia o DDA */
    fprintf(stderr, "[smoke] === trigger (borda de descida em D9) ===\n");
    trigger_set(1); run_us(g_avr, 50);
    trigger_set(0); run_us(g_avr, 20);   /* low ~20us */
    trigger_set(1);
    /* 5) roda a duracao do segmento + margem e conta passos */
    run_us(g_avr, 250000);
    uint32_t dj4 = g_steps[0] - steps0[0];
    uint32_t dj5 = g_steps[1] - steps0[1];
    uint32_t dj6 = g_steps[2] - steps0[2];
    fprintf(stderr, "[smoke]   passos observados J4=%u J5=%u J6=%u (esperado 10/0/0)\n", dj4, dj5, dj6);
    if (dj4 == 10 && dj5 == 0 && dj6 == 0) fprintf(stderr, "[smoke]   -> passos OK\n");
    else { fprintf(stderr, "[smoke]   -> passos divergentes: FALHA\n"); fails++; }

    /* 6) DONE */
    int got_done = 0;
    for (int i = from; i < g_avr_tx_n; i++) if (g_avr_tx[i] == UART_DONE) got_done = 1;
    if (got_done) fprintf(stderr, "[smoke]   -> DONE (0x04) recebido: OK\n");
    else { fprintf(stderr, "[smoke]   -> DONE NAO recebido: FALHA\n"); fails++; }

    fprintf(stderr, "[smoke] ============================\n");
    fprintf(stderr, "[smoke] RESULT: %s (%d falha(s))\n", fails ? "FAIL" : "PASS", fails);
    return fails ? 1 : 0;
}

/* ================================================================== */
/* MODO VCD (Etapa 7) — ensaio elétrico ISOLADO de T3/J_start         */
/* Injeta um frame válido + trigger e grava um VCD cycle-accurate das  */
/* bordas REAIS do firmware: trigger D9 (descida) e STEP J4 (subidas). */
/* O relógio do AVR (62,5 ns/ciclo) é a única referência de tempo.     */
/* ================================================================== */
static int vcd_dump(const char *elf_path, const char *out_path) {
    g_avr = make_avr(elf_path);
    if (!g_avr) return 2;
    trigger_set(1); estop_set(1);
    run_us(g_avr, 300000);  /* boot do firmware */

    uint8_t frame[10];
    build_frame(frame, 20, 0, 0, 200, /*corrupt=*/0);  /* J4=+20 passos, 200ms */
    for (int i = 0; i < 10; i++) rx_push(frame[i]);
    if (!wait_for_byte(g_avr, UART_ACK, 50000, NULL)) {
        fprintf(stderr, "[vcd] ACK nao recebido — abortando\n");
        return 1;
    }

    g_vcd_rec = 1;
    trigger_set(1); run_us(g_avr, 50);
    g_vcd_trig_cycle = (uint64_t)g_avr->cycle;   /* instante da borda de descida */
    trigger_set(0); run_us(g_avr, 20);
    trigger_set(1);
    run_us(g_avr, 400000);   /* roda o segmento; captura todas as bordas de passo */
    g_vcd_rec = 0;

    if (g_vcd_step_n < 2) {
        fprintf(stderr, "[vcd] passos insuficientes capturados (%d)\n", g_vcd_step_n);
        return 1;
    }

    FILE *f = fopen(out_path, "w");
    if (!f) { fprintf(stderr, "[vcd] nao consegui abrir %s\n", out_path); return 1; }
    const double NS_PER_CYCLE = 1.0e9 / 16.0e6;   /* 62.5 ns @16 MHz */
#define CY2NS(c) ((uint64_t)((double)(c) * NS_PER_CYCLE + 0.5))
    fprintf(f, "$timescale 1ns $end\n");
    fprintf(f, "$var wire 1 a trigger_d9 $end\n");
    fprintf(f, "$var wire 1 b step_j4 $end\n");
    fprintf(f, "$enddefinitions $end\n");
    fprintf(f, "#0\n1a\n0b\n");
    fprintf(f, "#%llu\n0a\n", (unsigned long long)CY2NS(g_vcd_trig_cycle));
    for (int i = 0; i < g_vcd_step_n; i++) {
        uint64_t rise = CY2NS(g_vcd_step_cycle[i]);
        fprintf(f, "#%llu\n1b\n", (unsigned long long)rise);
        fprintf(f, "#%llu\n0b\n", (unsigned long long)(rise + 125)); /* pulso ~2 ciclos */
    }
    fclose(f);
    fprintf(stderr, "[vcd] gravado %s: %d passos J4; T3(trig->1o passo)=%.3f us\n",
            out_path, g_vcd_step_n,
            (CY2NS(g_vcd_step_cycle[0]) - CY2NS(g_vcd_trig_cycle)) / 1000.0);
    return 0;
}

/* ================================================================== */
/* MODO SERVE — sockets TCP para as pontes Python (Etapa 3+)          */
/* ================================================================== */
/* (g_serving definido no topo, junto aos callbacks)                  */

/* AVR TX -> socket UART (preenchido em uart_out_cb, drenado pela thread UART) */
#define TX2S 4096
static uint8_t g_tx2s[TX2S];
static volatile int g_tx2s_head = 0, g_tx2s_tail = 0;
static pthread_mutex_t g_tx2s_lock = PTHREAD_MUTEX_INITIALIZER;
static void tx2s_push(uint8_t b) {
    pthread_mutex_lock(&g_tx2s_lock);
    int nh = (g_tx2s_head + 1) % TX2S;
    if (nh != g_tx2s_tail) { g_tx2s[g_tx2s_head] = b; g_tx2s_head = nh; }
    pthread_mutex_unlock(&g_tx2s_lock);
}

/* Eventos de passo -> socket step (preenchido em step_cb, drenado pela thread step) */
#define STP2S 8192
static uint8_t g_stp2s[STP2S];
static volatile int g_stp2s_head = 0, g_stp2s_tail = 0;
static pthread_mutex_t g_stp2s_lock = PTHREAD_MUTEX_INITIALIZER;
static void stp2s_push(uint8_t rec) {
    pthread_mutex_lock(&g_stp2s_lock);
    int nh = (g_stp2s_head + 1) % STP2S;
    if (nh != g_stp2s_tail) { g_stp2s[g_stp2s_head] = rec; g_stp2s_head = nh; }
    pthread_mutex_unlock(&g_stp2s_lock);
}

/* Comando de trigger/estop vindo da ponte 1 (aplicado na thread do AVR) */
static volatile int g_trig_cmd = 0;       /* 'F','0','1','E', ou 0 */

/* ----- helpers TCP ----- */
static int tcp_listen(int port) {
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) return -1;
    int yes = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    struct sockaddr_in a; memset(&a, 0, sizeof(a));
    a.sin_family = AF_INET; a.sin_addr.s_addr = htonl(INADDR_ANY); a.sin_port = htons(port);
    if (bind(fd, (struct sockaddr*)&a, sizeof(a)) != 0) { close(fd); return -1; }
    if (listen(fd, 1) != 0) { close(fd); return -1; }
    return fd;
}
static void set_nonblock(int fd) { int fl = fcntl(fd, F_GETFL, 0); fcntl(fd, F_SETFL, fl | O_NONBLOCK); }
static void set_nodelay(int fd) { int yes = 1; setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes)); }

/* Thread UART: ponte 1 (cliente) <-> UART do AVR. Bytes bidirecionais crus. */
static void *uart_thread(void *p) {
    int port = (int)(intptr_t)p;
    int lfd = tcp_listen(port);
    if (lfd < 0) { fprintf(stderr, "[serve] ERRO bind UART %d\n", port); return NULL; }
    fprintf(stderr, "[serve] UART  ouvindo em :%d\n", port);
    for (;;) {
        int c = accept(lfd, NULL, NULL);
        if (c < 0) { usleep(10000); continue; }
        set_nonblock(c); set_nodelay(c);
        fprintf(stderr, "[serve] UART  cliente conectado\n");
        for (;;) {
            uint8_t buf[256];
            int n = recv(c, buf, sizeof(buf), 0);
            if (n == 0) break;
            if (n > 0) for (int i = 0; i < n; i++) rx_push(buf[i]);
            /* drena TX do AVR -> socket */
            pthread_mutex_lock(&g_tx2s_lock);
            while (g_tx2s_tail != g_tx2s_head) {
                uint8_t b = g_tx2s[g_tx2s_tail];
                g_tx2s_tail = (g_tx2s_tail + 1) % TX2S;
                pthread_mutex_unlock(&g_tx2s_lock);
                if (send(c, &b, 1, 0) < 0) { /* ignora EAGAIN */ }
                pthread_mutex_lock(&g_tx2s_lock);
            }
            pthread_mutex_unlock(&g_tx2s_lock);
            usleep(200);
        }
        close(c);
        fprintf(stderr, "[serve] UART  cliente desconectado\n");
    }
}

/* Thread de controle: ponte 1 envia 1-byte commands p/ trigger/estop. */
static void *ctrl_thread(void *p) {
    int port = (int)(intptr_t)p;
    int lfd = tcp_listen(port);
    if (lfd < 0) { fprintf(stderr, "[serve] ERRO bind CTRL %d\n", port); return NULL; }
    fprintf(stderr, "[serve] CTRL  ouvindo em :%d\n", port);
    for (;;) {
        int c = accept(lfd, NULL, NULL);
        if (c < 0) { usleep(10000); continue; }
        fprintf(stderr, "[serve] CTRL  cliente conectado\n");
        for (;;) {
            uint8_t b;
            int n = recv(c, &b, 1, 0);
            if (n <= 0) break;
            if (b=='F'||b=='0'||b=='1'||b=='E') g_trig_cmd = b;  /* aplicado na thread do AVR */
        }
        close(c);
    }
}

/* Thread step: stream de eventos de passo p/ ponte 3 (J4-J6). 1 byte/evento:
 * bit0-1 = eixo (0=J4,1=J5,2=J6); bit4 = direcao (1=positiva). */
static void *step_thread(void *p) {
    int port = (int)(intptr_t)p;
    int lfd = tcp_listen(port);
    if (lfd < 0) { fprintf(stderr, "[serve] ERRO bind STEP %d\n", port); return NULL; }
    fprintf(stderr, "[serve] STEP  ouvindo em :%d\n", port);
    for (;;) {
        int c = accept(lfd, NULL, NULL);
        if (c < 0) { usleep(10000); continue; }
        set_nodelay(c);
        fprintf(stderr, "[serve] STEP  cliente conectado\n");
        for (;;) {
            pthread_mutex_lock(&g_stp2s_lock);
            int empty = (g_stp2s_tail == g_stp2s_head);
            uint8_t rec = 0;
            if (!empty) { rec = g_stp2s[g_stp2s_tail]; g_stp2s_tail = (g_stp2s_tail + 1) % STP2S; }
            pthread_mutex_unlock(&g_stp2s_lock);
            if (empty) { usleep(200); continue; }
            if (send(c, &rec, 1, 0) < 0) break;
        }
        close(c);
    }
}

/* Thread encoder: ponte 3 injeta continuamente o ground truth do Webots como
 * 3 x uint16 little-endian. Estes valores alimentam o escravo AS5600 GPIO. */
static void *encoder_thread(void *p) {
    int port = (int)(intptr_t)p;
    int lfd = tcp_listen(port);
    if (lfd < 0) { fprintf(stderr, "[serve] ERRO bind ENC %d\n", port); return NULL; }
    fprintf(stderr, "[serve] ENC   ouvindo em :%d\n", port);
    for (;;) {
        int c = accept(lfd, NULL, NULL);
        if (c < 0) { usleep(10000); continue; }
        set_nodelay(c);
        fprintf(stderr, "[serve] ENC   ponte 3 conectada\n");
        uint8_t buf[6];
        size_t have = 0;
        for (;;) {
            int n = recv(c, buf + have, sizeof(buf) - have, 0);
            if (n <= 0) break;
            have += (size_t)n;
            if (have == sizeof(buf)) {
                pthread_mutex_lock(&g_encoder_lock);
                for (int i = 0; i < 3; i++)
                    g_encoder_raw[i] = (uint16_t)(buf[2*i] | ((uint16_t)buf[2*i+1] << 8)) & 0x0FFFu;
                g_encoder_valid = 1;
                pthread_mutex_unlock(&g_encoder_lock);
                have = 0;
            }
        }
        pthread_mutex_lock(&g_encoder_lock);
        g_encoder_valid = 0;
        pthread_mutex_unlock(&g_encoder_lock);
        close(c);
        fprintf(stderr, "[serve] ENC   ponte 3 desconectada; AS5600 indisponivel\n");
    }
}

/* Timer de ciclo: aplica comandos de trigger/estop (na thread do AVR). */
static avr_cycle_count_t trig_pump(avr_t *avr, avr_cycle_count_t when, void *p) {
    (void)p;
    static int restore_high = 0;      /* contagem p/ restaurar D9 em HIGH */
    int cmd = g_trig_cmd;
    if (cmd) {
        g_trig_cmd = 0;
        if (cmd == 'F') { avr_raise_irq(g_trig_irq, 0); restore_high = 10; }  /* low; restaura em ~ ciclos */
        else if (cmd == '0') avr_raise_irq(g_trig_irq, 0);
        else if (cmd == '1') avr_raise_irq(g_trig_irq, 1);
        else if (cmd == 'E') { avr_raise_irq(g_estop_irq, 0); }               /* E-STOP ativo LOW */
    }
    if (restore_high > 0 && --restore_high == 0) avr_raise_irq(g_trig_irq, 1);
    return when + 16000;  /* ~1ms @16MHz; low do trigger dura ~10ms (suficiente p/ PCINT) */
}

static int serve(const char *elf_path, int uart_port, int ctrl_port, int step_port, int enc_port) {
    g_avr = make_avr(elf_path);
    if (!g_avr) return 2;
    g_serving = 1;
    g_i2c_enabled = 1;
    g_i2c.sda = g_i2c.scl = 1;
    i2c_drive_sda(1);
    avr_raise_irq(g_scl_irq, 1);
    trigger_set(1); estop_set(1);

    avr_cycle_timer_register(g_avr, 16000, trig_pump, NULL);

    pthread_t tu, tc, ts, te;
    pthread_create(&tu, NULL, uart_thread, (void*)(intptr_t)uart_port);
    pthread_create(&tc, NULL, ctrl_thread, (void*)(intptr_t)ctrl_port);
    pthread_create(&ts, NULL, step_thread, (void*)(intptr_t)step_port);
    pthread_create(&te, NULL, encoder_thread, (void*)(intptr_t)enc_port);

    /* Não inicia setup()/pollEncoders() antes de existir ground truth real. */
    fprintf(stderr, "[serve] aguardando primeira amostra real da ponte 3 em :%d...\n", enc_port);
    fflush(stderr);
    for (int waited = 0; waited < 3000 && !g_encoder_valid; waited++) usleep(10000);
    if (!g_encoder_valid) {
        fprintf(stderr, "[serve] ERRO: ponte 3 nao forneceu encoder em 30 s; abortando sem fallback\n");
        return 3;
    }

    fprintf(stderr, "[serve] encoder real recebido; simavr rodando o .ino REAL. [READY]\n");
    if (g_pace > 0.0)
        fprintf(stderr, "[serve] PACING tempo real ativo (fator=%.2f): o AVR e ritmado ao "
                "relogio de parede para que os passos do DDA cheguem ao longo do movimento "
                "(captura a rampa em vez de rajada).\n", g_pace);
    fflush(stderr);

    /* Pacing opcional: o simavr roda o AVR o mais rapido possivel; sem ritmo, um
     * movimento de varios segundos de tempo-AVR sai em rajada de wall-clock e a
     * telemetria registra um degrau. Com g_pace>0 ritmamos a execucao ao relogio de
     * parede (g_pace=1.0 -> tempo real; >1.0 -> mais lento), espalhando os eventos de
     * passo ao longo do movimento. So FREIA (nunca acelera): se o host nao alcanca o
     * tempo real, roda o mais rapido que conseguir. Ancora absoluta evita drift. */
    struct timespec t_anchor;
    clock_gettime(CLOCK_MONOTONIC, &t_anchor);
    const avr_cycle_count_t c_anchor = g_avr->cycle;
    const avr_cycle_count_t check_every = 16000;  /* ~1 ms de tempo-AVR @16MHz */
    avr_cycle_count_t next_check = c_anchor + check_every;
    const double freq = (double)(g_avr->frequency ? g_avr->frequency : 16000000UL);

    for (;;) {
        int st = avr_run(g_avr);
        if (st == cpu_Done || st == cpu_Crashed) { fprintf(stderr, "[serve] AVR parou (state=%d)\n", st); return 1; }
        if (g_pace > 0.0 && g_avr->cycle >= next_check) {
            const double avr_s = (double)(g_avr->cycle - c_anchor) / freq * g_pace;
            struct timespec now;
            clock_gettime(CLOCK_MONOTONIC, &now);
            const double wall_s = (double)(now.tv_sec - t_anchor.tv_sec) +
                                  (double)(now.tv_nsec - t_anchor.tv_nsec) / 1e9;
            const double ahead = avr_s - wall_s;
            if (ahead > 0.0005) usleep((useconds_t)(ahead * 1e6));  /* freia se >0,5 ms adiantado */
            next_check = g_avr->cycle + check_every;
        }
    }
}

/* ------------------------------------------------------------------ */
int main(int argc, char **argv) {
    const char *elf = NULL;
    const char *vcd_out = NULL;
    int mode_smoke = 0, mode_serve = 0;
    int uart_port = 30200, ctrl_port = 30201, step_port = 30202, enc_port = 30203;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--smoke")) mode_smoke = 1;
        else if (!strcmp(argv[i], "--serve")) mode_serve = 1;
        else if (!strcmp(argv[i], "--vcd") && i+1 < argc) vcd_out = argv[++i];
        else if (!strcmp(argv[i], "--elf") && i+1 < argc) elf = argv[++i];
        else if (!strcmp(argv[i], "--uart-port") && i+1 < argc) uart_port = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--ctrl-port") && i+1 < argc) ctrl_port = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--step-port") && i+1 < argc) step_port = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--enc-port")  && i+1 < argc) enc_port  = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--pace") && i+1 < argc) g_pace = atof(argv[++i]);
        else if (argv[i][0] != '-') elf = argv[i];
    }
    if (g_pace == 0.0) { const char *e = getenv("EB15_SIMAVR_PACE"); if (e) g_pace = atof(e); }
    if (!elf) { fprintf(stderr, "uso: uno_runner --elf <arquivo.elf> [--smoke | --serve | --vcd <out>] [--pace <fator>]\n"); return 2; }
    if (mode_smoke) return smoke(elf);
    if (vcd_out) return vcd_dump(elf, vcd_out);
    if (mode_serve) return serve(elf, uart_port, ctrl_port, step_port, enc_port);
    fprintf(stderr, "informe --smoke, --serve ou --vcd <out>\n");
    return 2;
}
