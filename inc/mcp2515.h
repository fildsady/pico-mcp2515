#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint32_t id;
    uint8_t  dlc;
    uint8_t  data[8];
} mcp2515_msg_t;

typedef void (*mcp2515_rx_cb_t)(const mcp2515_msg_t *msg);

bool mcp2515_init(uint32_t can_baud_hz, mcp2515_rx_cb_t rx_cb);
bool mcp2515_send(const mcp2515_msg_t *msg);
void mcp2515_poll(void);
void mcp2515_stop(void);
void mcp2515_start(uint32_t can_baud_hz);
bool    mcp2515_is_ok(void);
uint8_t mcp2515_last_canstat(void);
void    mcp2515_get_errors(uint8_t *tec, uint8_t *rec, uint8_t *eflg);
void    mcp2515_set_notify_task(void *task_handle);

/* CANSTAT as of the last mcp2515_poll() call (opmode in bits 7-5, ICOD
 * interrupt-source code in bits 3-1) — distinct from mcp2515_last_canstat(),
 * which only ever reflects the one-time read right after mcp2515_init()'s
 * reset/config sequence. Cached rather than a live SPI read on demand,
 * since task_can is the only task allowed to touch the MCP2515 over SPI. */
uint8_t mcp2515_get_canstat(void);

/* TXB0CTRL as of the last mcp2515_poll() call — bit3 TXERR (error during
 * last TX attempt), bit5 MLOA (lost arbitration), bit4 ABTF (aborted). */
uint8_t mcp2515_get_txb0ctrl(void);

/* RX0OVR/RX1OVR (EFLG bits 6/7) latch on hardware and don't self-clear, so
 * mcp2515_poll() clears them itself and counts occurrences here — these are
 * cumulative since mcp2515_init()/mcp2515_start(), not "currently set". */
void mcp2515_get_rx_overflow(uint32_t *rx0_overflow, uint32_t *rx1_overflow);

/* Switches between Normal (TX enabled, ACKs frames) and Listen-only (RX
 * only, never drives the bus — not even an ACK bit) without a full
 * stop/start or losing the configured baud rate. Only task_can may call
 * this (same SPI/CS ownership rule as everything else in this driver). */
void mcp2515_set_listen_only(bool enable);
