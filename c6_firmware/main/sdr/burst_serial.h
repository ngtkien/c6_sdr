/* burst_serial.h — c6_sdr shim for the upstream UART transport API.
 * Only the stop-request flag does real work (SdrStop RPC); the serial
 * send/port accessors are inert stubs satisfied in sdr_engine.c. */
#pragma once

#include <stdbool.h>
#include <stddef.h>

typedef enum { BURST_SERIAL_USB, BURST_SERIAL_UART, BURST_SERIAL_COUNT } burst_serial_port_t;

bool burst_serial_send(const void *data, size_t size);
burst_serial_port_t burst_serial_port(void);
unsigned burst_serial_baud(void);

/* Consume a stream stop request (set by the SdrStop RPC handler). */
bool burst_serial_stop_requested(void);
