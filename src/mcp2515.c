/*
 * mcp2515.c — MCP2515 CAN controller driver via PIO SPI
 *
 * PIO2 SM0 = SPI Master Mode 0 (CPOL=0, CPHA=0)
 * GP6=SCK, GP7=MOSI, GP8=MISO, GP27=CS, GP9=INT
 * Assumes 8MHz crystal on MCP2515 module.
 */
#include "mcp2515.h"
#include "config.h"
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"
#include "spi_pio.pio.h"
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#define MCP_PIO         pio2
#define MCP_SM          0
#define MCP_SPI_FREQ    4000000   /* 4 MHz SPI clock — was 10 MHz (datasheet max, zero margin);
                                    * dropped for signal-integrity headroom over breadboard/long
                                    * wiring, where 10 MHz edges are prone to ringing/reflection. */

/* ── MCP2515 SPI instructions ────────────────────────────────────────── */
#define MCP_RESET       0xC0
#define MCP_READ        0x03
#define MCP_WRITE       0x02
#define MCP_RTS_TXB0    0x81
#define MCP_READ_STATUS 0xA0
#define MCP_RX_STATUS   0xB0
#define MCP_BIT_MODIFY  0x05
#define MCP_READ_RX0    0x90
#define MCP_READ_RX1    0x94
#define MCP_LOAD_TX0    0x40

/* ── MCP2515 registers ───────────────────────────────────────────────── */
#define REG_CANSTAT     0x0E
#define REG_CANCTRL     0x0F
#define REG_CNF3        0x28
#define REG_CNF2        0x29
#define REG_CNF1        0x2A
#define REG_CANINTE     0x2B
#define REG_CANINTF     0x2C
#define REG_TXB0CTRL    0x30
#define REG_TXB0SIDH    0x31
#define REG_TXB0SIDL    0x32
#define REG_TXB0DLC     0x35
#define REG_TXB0D0      0x36
#define REG_RXB0CTRL    0x60
#define REG_RXB0SIDH    0x61
#define REG_RXB0SIDL    0x62
#define REG_RXB0DLC     0x65
#define REG_RXB0D0      0x66
#define REG_EFLG        0x2D
#define REG_TEC         0x1C
#define REG_REC         0x1D

/* CANCTRL modes */
#define MODE_NORMAL     0x00
#define MODE_SLEEP      0x20
#define MODE_LOOPBACK   0x40
#define MODE_LISTEN     0x60
#define MODE_CONFIG     0x80

/* CANINTE / CANINTF bits */
#define INT_RX0         0x01
#define INT_RX1         0x02
#define INT_TX0         0x04
#define INT_ERR         0x20
#define INT_MERR        0x80

/* EFLG bits (REG_EFLG) */
#define EFLG_RX0OVR     0x40
#define EFLG_RX1OVR     0x80

/* ── State ───────────────────────────────────────────────────────────── */
static mcp2515_rx_cb_t s_rx_cb;
static uint s_pio_offset;
static volatile bool s_initialized;
static volatile uint8_t s_last_canstat = 0xFF;  /* raw CANSTAT after last reset — for debug */
static volatile TaskHandle_t s_notify_task = NULL;
static volatile uint8_t s_last_txb0ctrl = 0;    /* refreshed once per mcp2515_poll() call */
static volatile uint8_t s_live_canstat = 0xFF;  /* refreshed once per mcp2515_poll() call */
static volatile uint32_t s_rx0_overflow = 0;
static volatile uint32_t s_rx1_overflow = 0;

/* ── PIO SPI low-level ───────────────────────────────────────────────── */
static inline void cs_low(void)  { gpio_put(MCP2515_CS_PIN, 0); }
static inline void cs_high(void) {
    gpio_put(MCP2515_CS_PIN, 1);
    /* Delay at least 100ns (TCSH) for MCP2515. At 240MHz, 1 cycle = ~4.16ns.
     * 30 volatile iterations (~120-200ns) guarantees we satisfy the limit. */
    for (volatile int i = 0; i < 30; i++) {
        __asm volatile("nop");
    }
}

static uint8_t spi_xfer_byte(uint8_t tx) {
    pio_sm_put_blocking(MCP_PIO, MCP_SM, (uint32_t)tx << 24);
    return (uint8_t)pio_sm_get_blocking(MCP_PIO, MCP_SM);
}

static void spi_write_buf(const uint8_t *buf, int len) {
    for (int i = 0; i < len; i++)
        spi_xfer_byte(buf[i]);
}

static void spi_read_buf(uint8_t *buf, int len) {
    for (int i = 0; i < len; i++)
        buf[i] = spi_xfer_byte(0xFF);
}

/* ── MCP2515 register access ─────────────────────────────────────────── */
static uint8_t mcp_read_reg(uint8_t addr) {
    cs_low();
    spi_xfer_byte(MCP_READ);
    spi_xfer_byte(addr);
    uint8_t val = spi_xfer_byte(0xFF);
    cs_high();
    return val;
}

static void mcp_write_reg(uint8_t addr, uint8_t val) {
    cs_low();
    spi_xfer_byte(MCP_WRITE);
    spi_xfer_byte(addr);
    spi_xfer_byte(val);
    cs_high();
}

static void mcp_bit_modify(uint8_t addr, uint8_t mask, uint8_t val) {
    cs_low();
    spi_xfer_byte(MCP_BIT_MODIFY);
    spi_xfer_byte(addr);
    spi_xfer_byte(mask);
    spi_xfer_byte(val);
    cs_high();
}

static void mcp_reset(void) {
    cs_low();
    spi_xfer_byte(MCP_RESET);
    cs_high();
    busy_wait_ms(100);  /* 100ms — allow 8MHz crystal to stabilise after reset */
}

/* ── Baud rate config (8MHz crystal) ─────────────────────────────────── */
static bool mcp_set_baud(uint32_t baud) {
    uint8_t cnf1, cnf2, cnf3;
    switch (baud) {
    case 20000:   cnf1 = 0x18; cnf2 = 0xB4; cnf3 = 0x04; break;
    case 50000:   cnf1 = 0x09; cnf2 = 0xB4; cnf3 = 0x04; break;
    case 125000:  cnf1 = 0x03; cnf2 = 0xB4; cnf3 = 0x04; break;
    case 250000:  cnf1 = 0x01; cnf2 = 0xB4; cnf3 = 0x04; break;
    case 500000:  cnf1 = 0x00; cnf2 = 0xB4; cnf3 = 0x04; break;
    case 1000000: cnf1 = 0x00; cnf2 = 0x90; cnf3 = 0x02; break;
    default: return false;
    }
    mcp_write_reg(REG_CNF1, cnf1);
    mcp_write_reg(REG_CNF2, cnf2);
    mcp_write_reg(REG_CNF3, cnf3);
    return true;
}

/* ── PIO SPI init ────────────────────────────────────────────────────── */
static void pio_spi_init(void) {
    s_pio_offset = pio_add_program(MCP_PIO, &spi_master_program);

    pio_sm_config c = spi_master_program_get_default_config(s_pio_offset);

    /* OUT pin = MOSI */
    sm_config_set_out_pins(&c, MCP2515_MOSI_PIN, 1);
    /* IN pin = MISO */
    sm_config_set_in_pins(&c, MCP2515_MISO_PIN);
    /* Sideset = SCK */
    sm_config_set_sideset_pins(&c, MCP2515_SCK_PIN);

    sm_config_set_out_shift(&c, false, true, 8);   /* MSB first, autopull 8 */
    sm_config_set_in_shift(&c, false, true, 8);    /* MSB first, autopush 8 */

    float div = (float)clock_get_hz(clk_sys) / (MCP_SPI_FREQ * 4.0f);
    sm_config_set_clkdiv(&c, div);

    /* GPIO init */
    pio_gpio_init(MCP_PIO, MCP2515_SCK_PIN);
    pio_gpio_init(MCP_PIO, MCP2515_MOSI_PIN);
    pio_gpio_init(MCP_PIO, MCP2515_MISO_PIN);
    gpio_pull_up(MCP2515_SCK_PIN);   /* pull-up SCK */
    gpio_pull_up(MCP2515_MOSI_PIN);  /* pull-up MOSI */
    gpio_pull_up(MCP2515_MISO_PIN);  /* pull-up MISO — prevent float when MCP2515 not driving */

    pio_sm_set_consecutive_pindirs(MCP_PIO, MCP_SM, MCP2515_SCK_PIN, 1, true);
    pio_sm_set_consecutive_pindirs(MCP_PIO, MCP_SM, MCP2515_MOSI_PIN, 1, true);
    pio_sm_set_consecutive_pindirs(MCP_PIO, MCP_SM, MCP2515_MISO_PIN, 1, false);

    pio_sm_init(MCP_PIO, MCP_SM, s_pio_offset, &c);
    pio_sm_set_enabled(MCP_PIO, MCP_SM, true);
}

/* ── INT pin ISR ─────────────────────────────────────────────────────── */
static void int_isr(uint gpio, uint32_t events) {
    (void)gpio; (void)events;
    if (s_notify_task) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(s_notify_task, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/* ── Public API ──────────────────────────────────────────────────────── */
bool mcp2515_init(uint32_t can_baud_hz, mcp2515_rx_cb_t rx_cb) {
    s_rx_cb = rx_cb;

    /* CS pin — GPIO, not PIO */
    gpio_init(MCP2515_CS_PIN);
    gpio_set_dir(MCP2515_CS_PIN, GPIO_OUT);
    gpio_pull_up(MCP2515_CS_PIN);     /* pull-up CS */
    cs_high();

    /* INT pin — input, pull-up, falling edge IRQ */
    gpio_init(MCP2515_INT_PIN);
    gpio_set_dir(MCP2515_INT_PIN, GPIO_IN);
    gpio_pull_up(MCP2515_INT_PIN);
    gpio_set_irq_enabled_with_callback(MCP2515_INT_PIN, GPIO_IRQ_EDGE_FALL, true, int_isr);

    /* PIO SPI */
    pio_spi_init();

    /* Flush any stale data in RX FIFO before first transaction */
    while (!pio_sm_is_rx_fifo_empty(MCP_PIO, MCP_SM))
        (void)pio_sm_get(MCP_PIO, MCP_SM);

    /* MCP2515 reset + config — retry up to 3 times
     * If RESET cmd (0xC0) didn't land cleanly (PIO glitch on first enable),
     * force CONFIG mode by writing CANCTRL directly. */
    uint8_t stat = 0;
    bool config_ok = false;
    for (int attempt = 0; attempt < 3; attempt++) {
        mcp_reset();
        stat = mcp_read_reg(REG_CANSTAT);
        s_last_canstat = stat;
        if ((stat & 0xE0) == MODE_CONFIG) { config_ok = true; break; }

        /* Not in config mode — force it explicitly via CANCTRL */
        mcp_write_reg(REG_CANCTRL, MODE_CONFIG);
        busy_wait_ms(10);
        stat = mcp_read_reg(REG_CANSTAT);
        s_last_canstat = stat;
        if ((stat & 0xE0) == MODE_CONFIG) { config_ok = true; break; }

        busy_wait_ms(50);
    }
    if (!config_ok) {
        s_initialized = false;
        return false;
    }

    /* Baud rate */
    if (!mcp_set_baud(can_baud_hz)) return false;

    /* RXB0 and RXB1: receive all messages (no filter) */
    mcp_write_reg(REG_RXB0CTRL, 0x60);
    mcp_write_reg(0x70, 0x60);          /* REG_RXB1CTRL = 0x70 */

    /* Enable RX0 and RX1 interrupts */
    mcp_write_reg(REG_CANINTE, INT_RX0 | INT_RX1);

    /* Switch to normal mode */
    mcp_bit_modify(REG_CANCTRL, 0xE0, MODE_NORMAL);
    busy_wait_ms(10);

    stat = mcp_read_reg(REG_CANSTAT);
    if ((stat & 0xE0) != MODE_NORMAL) {
        s_initialized = false;
        return false;
    }

    s_initialized = true;
    return true;
}

bool mcp2515_send(const mcp2515_msg_t *msg) {
    if (!s_initialized) return false;

    /* Check TX buffer free */
    uint8_t ctrl = mcp_read_reg(REG_TXB0CTRL);
    if (ctrl & 0x08) return false;  /* TXREQ still pending */

    /* Load TX buffer via SPI instruction */
    cs_low();
    spi_xfer_byte(MCP_LOAD_TX0);
    /* SIDH, SIDL, EID8, EID0, DLC */
    spi_xfer_byte((uint8_t)(msg->id >> 3));
    spi_xfer_byte((uint8_t)((msg->id & 0x07) << 5));
    spi_xfer_byte(0x00);  /* EID8 */
    spi_xfer_byte(0x00);  /* EID0 */
    uint8_t dlc = msg->dlc > 8 ? 8 : msg->dlc;
    spi_xfer_byte(dlc);
    spi_write_buf(msg->data, dlc);
    cs_high();

    /* Request to send */
    cs_low();
    spi_xfer_byte(MCP_RTS_TXB0);
    cs_high();
    return true;
}

void mcp2515_poll(void) {
    if (!s_initialized) return;

    /* Loop to empty both RX0 and RX1 as long as there is data.
     * This ensures the MCP2515 INT pin returns to high,
     * so we will receive subsequent falling edge interrupts. */
    int loop_limit = 0;
    while (loop_limit++ < 10) {
        uint8_t intf = mcp_read_reg(REG_CANINTF);
        if (intf == 0xFF) {
            break; /* chip not responding or disconnected */
        }
        if (!(intf & (INT_RX0 | INT_RX1 | INT_ERR | INT_MERR))) {
            break;
        }

        if (intf & INT_RX0) {
            cs_low();
            spi_xfer_byte(MCP_READ_RX0);
            uint8_t sidh = spi_xfer_byte(0xFF);
            uint8_t sidl = spi_xfer_byte(0xFF);
            spi_xfer_byte(0xFF);  /* EID8 */
            spi_xfer_byte(0xFF);  /* EID0 */
            uint8_t dlc_raw = spi_xfer_byte(0xFF);
            uint8_t dlc = dlc_raw & 0x0F;
            if (dlc > 8) dlc = 8;
            uint8_t data[8] = {0};
            spi_read_buf(data, dlc);
            cs_high();

            /* Clear RX0 interrupt flag */
            mcp_bit_modify(REG_CANINTF, INT_RX0, 0x00);

            if (s_rx_cb) {
                mcp2515_msg_t msg;
                msg.id = ((uint32_t)sidh << 3) | ((sidl >> 5) & 0x07);
                msg.dlc = dlc;
                memcpy(msg.data, data, dlc);
                s_rx_cb(&msg);
            }
        }

        if (intf & INT_RX1) {
            cs_low();
            spi_xfer_byte(MCP_READ_RX1);
            uint8_t sidh = spi_xfer_byte(0xFF);
            uint8_t sidl = spi_xfer_byte(0xFF);
            spi_xfer_byte(0xFF);  /* EID8 */
            spi_xfer_byte(0xFF);  /* EID0 */
            uint8_t dlc_raw = spi_xfer_byte(0xFF);
            uint8_t dlc = dlc_raw & 0x0F;
            if (dlc > 8) dlc = 8;
            uint8_t data[8] = {0};
            spi_read_buf(data, dlc);
            cs_high();

            /* Clear RX1 interrupt flag */
            mcp_bit_modify(REG_CANINTF, INT_RX1, 0x00);

            if (s_rx_cb) {
                mcp2515_msg_t msg;
                msg.id = ((uint32_t)sidh << 3) | ((sidl >> 5) & 0x07);
                msg.dlc = dlc;
                memcpy(msg.data, data, dlc);
                s_rx_cb(&msg);
            }
        }

        /* Clear error flags if any */
        if (intf & (INT_ERR | INT_MERR)) {
            mcp_bit_modify(REG_CANINTF, INT_ERR | INT_MERR, 0x00);
        }
    }

    /* RX0OVR/RX1OVR (EFLG bits 6/7) latch in hardware and are *not*
     * self-clearing like CANINTF's error bits above — left alone they'd
     * stay set forever after the first overflow, so this has to explicitly
     * BIT MODIFY them back to 0 once seen. Counted rather than just
     * latched/cleared so a transient overflow is still visible later even
     * after the flag itself has been cleared. */
    uint8_t eflg = mcp_read_reg(REG_EFLG);
    if (eflg & (EFLG_RX0OVR | EFLG_RX1OVR)) {
        if (eflg & EFLG_RX0OVR) s_rx0_overflow++;
        if (eflg & EFLG_RX1OVR) s_rx1_overflow++;
        mcp_bit_modify(REG_EFLG, EFLG_RX0OVR | EFLG_RX1OVR, 0x00);
    }

    /* Snapshot for mcp2515_get_txb0ctrl()/mcp2515_get_canstat() — cheap
     * enough to read every poll, and this is the only place that already
     * unconditionally talks to the chip every cycle regardless of whether a
     * frame was pending. Read live here (not lazily on demand from the
     * getters) because task_can is the *only* task allowed to touch the
     * MCP2515 over SPI/CS — task_oled just reads these cached values. */
    s_last_txb0ctrl = mcp_read_reg(REG_TXB0CTRL);
    s_live_canstat  = mcp_read_reg(REG_CANSTAT);
}

void mcp2515_stop(void) {
    if (!s_initialized) return;
    mcp_bit_modify(REG_CANCTRL, 0xE0, MODE_CONFIG);
    gpio_set_irq_enabled(MCP2515_INT_PIN, GPIO_IRQ_EDGE_FALL, false);
    pio_sm_set_enabled(MCP_PIO, MCP_SM, false);
    s_initialized = false;
}

void mcp2515_start(uint32_t can_baud_hz) {
    pio_sm_set_enabled(MCP_PIO, MCP_SM, true);
    gpio_set_irq_enabled(MCP2515_INT_PIN, GPIO_IRQ_EDGE_FALL, true);
    mcp_reset();
    mcp_set_baud(can_baud_hz);
    mcp_write_reg(REG_RXB0CTRL, 0x60);
    mcp_write_reg(0x70, 0x60);          /* REG_RXB1CTRL = 0x70 */
    mcp_write_reg(REG_CANINTE, INT_RX0 | INT_RX1);
    mcp_bit_modify(REG_CANCTRL, 0xE0, MODE_NORMAL);
    busy_wait_ms(10);
    s_initialized = (mcp_read_reg(REG_CANSTAT) & 0xE0) == MODE_NORMAL;
}

bool    mcp2515_is_ok(void)          { return s_initialized; }
uint8_t mcp2515_last_canstat(void)   { return s_last_canstat; }

void mcp2515_get_errors(uint8_t *tec, uint8_t *rec, uint8_t *eflg) {
    if (!s_initialized) {
        if (tec) *tec = 0;
        if (rec) *rec = 0;
        if (eflg) *eflg = 0;
        return;
    }
    if (tec) *tec = mcp_read_reg(0x1C); /* TEC */
    if (rec) *rec = mcp_read_reg(0x1D); /* REC */
    if (eflg) *eflg = mcp_read_reg(0x2D); /* EFLG */
}

void mcp2515_set_notify_task(void *task_handle) {
    s_notify_task = (TaskHandle_t)task_handle;
}

uint8_t mcp2515_get_canstat(void) {
    return s_initialized ? s_live_canstat : 0xFF;
}

uint8_t mcp2515_get_txb0ctrl(void) {
    return s_last_txb0ctrl;
}

void mcp2515_get_rx_overflow(uint32_t *rx0_overflow, uint32_t *rx1_overflow) {
    if (rx0_overflow) *rx0_overflow = s_rx0_overflow;
    if (rx1_overflow) *rx1_overflow = s_rx1_overflow;
}

void mcp2515_set_listen_only(bool enable) {
    if (!s_initialized) return;
    uint8_t mode = enable ? MODE_LISTEN : MODE_NORMAL;
    mcp_bit_modify(REG_CANCTRL, 0xE0, mode);
    busy_wait_ms(10);
    s_initialized = (mcp_read_reg(REG_CANSTAT) & 0xE0) == mode;
}
