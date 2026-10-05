/* SPDX-License-Identifier: Apache-2.0
 * SDIO link layer — esp_payload_header framing over the sdio_slave driver.
 */
#pragma once

#include <stdint.h>
#include <zephyr/kernel.h>

typedef void (*c6_serial_cb_t)(const uint8_t *ep, uint8_t ep_len,
			       const uint8_t *data, uint16_t data_len);
typedef void (*c6_open_cb_t)(void);

int  c6_link_init(c6_serial_cb_t serial_cb, c6_open_cb_t open_cb);
/* Send a protobuf blob wrapped in a serial TLV on the given endpoint.
 * Fragments into <=1500-byte payload frames automatically. */
int  c6_link_tx_serial(const char *ep, const uint8_t *pb, uint16_t pb_len);

bool c6_link_datapath(void);
