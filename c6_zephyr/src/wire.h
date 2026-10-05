/* SPDX-License-Identifier: Apache-2.0
 * ESP-Hosted wire protocol constants — shared between the P4 host driver
 * (esp_hosted_ng) and this Zephyr slave. Keep in sync with
 * projects/c6_sdr/common/ and zephyr-esp32p4-v1-bsp esp_hosted_ng.h.
 */
#pragma once

#include <stdint.h>
#include <zephyr/sys/util.h>

/* 12-byte payload header (same layout as esp_payload_header). */
struct c6_hdr {
	uint8_t  if_type;      /* if_type[3:0], if_num[7:4] */
	uint8_t  flags;
	uint16_t len;          /* payload length */
	uint16_t offset;       /* offset of payload inside frame */
	uint16_t checksum;
	uint16_t seq_num;
	uint8_t  throttle;
	uint8_t  reserved2;
} __packed;

#define C6_HDR_LEN        sizeof(struct c6_hdr)
#define C6_DATA_OFFSET    C6_HDR_LEN
#define C6_FLAG_MORE_FRAG BIT(0)

/* interface type nibble values (esp_hosted_if_type_t) */
#define C6_IF_STA    1
#define C6_IF_AP     2
#define C6_IF_SERIAL 3
#define C6_IF_HCI    4
#define C6_IF_PRIV   5
#define C6_IF_TEST   6

/* serial-channel TLV */
#define C6_TLV_EP_NAME 0x01
#define C6_TLV_DATA    0x02
#define C6_EP_RSP      "RPCRsp"
#define C6_EP_EVT      "RPCEvt"

/* host->slave interrupt numbers (esp_hosted_transport_init.h): the host
 * writes BIT(n) to SCRATCH7 (card addr 0x8C); the slave receives slave
 * interrupt bit n.
 */
#define C6_EV_OPEN_DATA_PATH    0
#define C6_EV_CLOSE_DATA_PATH   1
#define C6_EV_RESET             2

#define C6_MAX_FRAME      1600   /* max payload per esp frame */
#define C6_RX_BUFSZ       (C6_MAX_FRAME + C6_HDR_LEN + 64)
#define C6_TX_BUFSZ       (C6_MAX_FRAME + C6_HDR_LEN + 64)
#define C6_SDIO_BUF       1536   /* ESSL rx buffer size (must match host) */
#define C6_SDIO_NUM_BUFS  20
