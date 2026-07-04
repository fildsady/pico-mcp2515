# pico-mcp2515

MCP2515 CAN controller driver for the Raspberry Pi Pico (RP2040/RP2350) SDK, extracted from [can-audio-remote](https://github.com/fildsady/can-audio-remote).

- Talks to the MCP2515 over SPI implemented in PIO (`src/spi_pio.pio`) rather than a hardware SPI peripheral — frees up the board's hardware SPI block for other use
- Covers init, register read/write/bit-modify, TX/RX (standard + extended frames), Listen-only mode, RX overflow counters, TEC/REC error counters, CANSTAT/TXB0CTRL diagnostics

## Dependencies

- Pico SDK (`hardware/pio.h`, `hardware/gpio.h`, `hardware/irq.h`, `hardware/clocks.h`)
- FreeRTOS (`FreeRTOS.h`, `task.h`) — this driver assumes an RTOS is present. Not intended for bare-metal use.

## Wiring / config

Unlike `ssd1306.h`, this driver does **not** define its own pins — it expects the consuming project's own `config.h` (on the include path) to provide:

```c
#define MCP2515_SCK_PIN   <your SCK pin>
#define MCP2515_MOSI_PIN  <your MOSI pin>
#define MCP2515_MISO_PIN  <your MISO pin>
#define MCP2515_CS_PIN    <your CS pin>
#define MCP2515_INT_PIN   <your INT pin>
```

PIO instance/state machine (`pio2`, state machine 0) are fixed in `src/mcp2515.c` — change those `#define`s directly if they collide with something else already using `pio2` on your board.

## Usage

```c
#include "mcp2515.h"

static void on_rx(const mcp2515_msg_t *msg) {
    /* called from mcp2515_poll() below when a frame arrives */
}

mcp2515_init(500000, on_rx); /* baud, RX callback */

mcp2515_msg_t tx = { .id = 0x123, .dlc = 8, .data = {0} };
mcp2515_send(&tx);

/* call periodically (e.g. once per task loop) — polls for RX, updates
 * cached diagnostics (CANSTAT/TXB0CTRL/overflow counters), and fires
 * on_rx() for any frame received since the last call */
mcp2515_poll();
```

Switch to Listen-only (RX/monitoring only, never ACKs or transmits) with `mcp2515_set_listen_only(true)`, back to Normal with `false`. See [inc/mcp2515.h](inc/mcp2515.h) for the full API (diagnostics: `mcp2515_get_canstat()`, `mcp2515_get_txb0ctrl()`, `mcp2515_get_errors()`, `mcp2515_get_rx_overflow()`).

## License

MIT — see [LICENSE](LICENSE).
