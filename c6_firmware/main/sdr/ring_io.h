/* ring_io.h — c6_sdr transport shim replacing ESP-SDR's UART ring_io.
 *
 * ring_capture.c was written for a byte-stream UART: txq_pump() drains the
 * on-chip frame queue through ring_write(), and host_input() polls
 * ring_input_available()/ring_read_byte() for the '\n' stop byte.
 *
 * Here the "host" is the ESP32-P4 over the hosted SDIO RPC, so:
 *   - ring_write() appends to a flat capture buffer in DRAM; a full buffer
 *     stops accepting, txq backs up, drops are counted, and ~2 s later the
 *     run ends via the engine's own host-stall watchdog;
 *   - there is no input stream: ring_input_available() is always 0, stops
 *     come from the SdrStop RPC via burst_serial_stop_requested().
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

int  ring_write(const uint8_t *data, size_t len); /* bytes consumed, 0..len */
int  ring_input_available(void);
int  ring_read_byte(uint8_t *b);
