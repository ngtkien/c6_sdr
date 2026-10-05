/* SPDX-License-Identifier: Apache-2.0
 * SDIO link layer — esp_payload_header framing over the ported
 * sdio_slave driver. Mirrors the ESP-Hosted slave data path.
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>
#include <drivers/esp32c6_sdio_slave.h>

#include <esp_system.h>

#include "wire.h"
#include "link.h"

LOG_MODULE_DECLARE(c6_sdr, LOG_LEVEL_INF);

#define RX_THREAD_STACK 4096
#define RX_THREAD_PRIO  K_PRIO_COOP(5)

static c6_serial_cb_t serial_cb;
static c6_open_cb_t   open_cb;
static volatile bool  datapath;

/* slave -> host */
static uint16_t tx_seq;
static struct k_mutex tx_lock;

/* host -> slave serial reassembly */
static uint8_t  ser_acc[C6_RX_BUFSZ];
static uint32_t ser_len;
static uint16_t ser_seq;
static bool     ser_inframe;

/* rx buffers registered with the slave DMA */
static uint8_t rx_buf[C6_SDIO_NUM_BUFS][C6_SDIO_BUF] __aligned(4);

static K_THREAD_STACK_DEFINE(rx_stack, RX_THREAD_STACK);
static struct k_thread rx_thread;

static uint16_t esp_checksum(const uint8_t *buf, uint32_t len)
{
	uint32_t c = 0;

	for (uint32_t i = 0; i < len; i++) {
		c += buf[i];
	}
	return c & 0xFFFF;
}

static struct k_work hostint_work;
static volatile uint32_t hostint_bits;

/* Runs in work-queue context — callbacks may sleep/lock. */
static void hostint_worker(struct k_work *work)
{
	uint32_t bits;

	ARG_UNUSED(work);
	bits = hostint_bits;
	hostint_bits = 0;

	if (bits & BIT(C6_EV_OPEN_DATA_PATH)) {
		if (!datapath) {
			datapath = true;
			LOG_INF("data path open");
			if (open_cb) {
				open_cb();
			}
		}
	}
	if (bits & BIT(C6_EV_CLOSE_DATA_PATH)) {
		datapath = false;
		LOG_INF("data path closed");
	}
	if (bits & BIT(C6_EV_RESET)) {
		LOG_WRN("host requested reset");
		k_msleep(20);          /* let the log out */
		esp_restart();
	}
}

/* Called from ISR context with the host->slave interrupt number (slvint
 * bit index). Bit n fires when the host writes BIT(n) to SCRATCH7.
 */
static void host_int_cb(uint8_t pos)
{
	if (pos < 8) {
		hostint_bits |= BIT(pos);
		k_work_submit(&hostint_work);
	}
}

/* Parse the serial TLV stream accumulated in ser_acc; call serial_cb for
 * each complete "endpoint + data" unit. Returns consumed length. */
static void serial_drain(void)
{
	uint32_t pos = 0;

	while (ser_len - pos >= 3) {
		uint16_t ep_len, data_len;
		uint32_t p = pos;

		if (ser_acc[p] != C6_TLV_EP_NAME) {
			LOG_WRN("serial tlv: bad ep tag %02x", ser_acc[p]);
			break;
		}
		ep_len = ser_acc[p + 1] | (ser_acc[p + 2] << 8);
		if (ep_len == 0 || ep_len >= 16) {
			break;
		}
		p += 3;
		if (p + ep_len + 3 > ser_len) {
			break;                    /* incomplete */
		}
		if (ser_acc[p + ep_len] != C6_TLV_DATA) {
			LOG_WRN("serial tlv: bad data tag");
			break;
		}
		data_len = ser_acc[p + ep_len + 1] |
			   (ser_acc[p + ep_len + 2] << 8);
		if (p + ep_len + 3 + data_len > ser_len) {
			break;                    /* incomplete */
		}
		if (serial_cb) {
			serial_cb(&ser_acc[p], ep_len,
				  &ser_acc[p + ep_len + 3], data_len);
		}
		pos = p + ep_len + 3 + data_len;
	}
	if (pos) {
		ser_len -= pos;
		memmove(ser_acc, ser_acc + pos, ser_len);
	}
}

/* Accumulate a serial-frame payload; handles esp-frame fragmentation via
 * seq_num + MORE_FRAGMENT (same as process_serial_rx_pkt in the IDF fw). */
static void serial_rx(const struct c6_hdr *hdr, const uint8_t *payload,
		      uint16_t plen)
{
	uint16_t seq = sys_le16_to_cpu(hdr->seq_num);
	uint8_t flags = hdr->flags;

	if (!ser_inframe) {
		ser_seq = seq;
		ser_inframe = true;
		ser_len = 0;
	} else if (seq != ser_seq) {
		/* desync — drop stale partial and start fresh */
		ser_len = 0;
		ser_seq = seq;
	}

	if (plen > sizeof(ser_acc) - ser_len) {
		LOG_WRN("serial acc overrun %u+%u", ser_len, plen);
		ser_len = 0;
		ser_inframe = false;
		return;
	}
	memcpy(ser_acc + ser_len, payload, plen);
	ser_len += plen;

	if (!(flags & C6_FLAG_MORE_FRAG)) {
		serial_drain();
		ser_inframe = false;
	}
}

static void rx_entry(void *p1, void *p2, void *p3)
{
	sdio_slave_buf_handle_t h;
	uint8_t *addr;
	size_t len;
	int ret;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	for (;;) {
		ret = sdio_slave_recv(&h, &addr, &len, K_FOREVER);
		if (ret != 0 || !addr) {
			continue;
		}
		if (len >= C6_HDR_LEN) {
			const struct c6_hdr *hdr = (const struct c6_hdr *)addr;
			uint8_t if_type = hdr->if_type & 0x0F;
			uint16_t plen = sys_le16_to_cpu(hdr->len);
			uint16_t off = sys_le16_to_cpu(hdr->offset);

			if (if_type == C6_IF_SERIAL && off <= len &&
			    plen <= len - off) {
				serial_rx(hdr, addr + off, plen);
			} else if (if_type != C6_IF_SERIAL) {
				LOG_DBG("rx if %u len %u", if_type, plen);
			}
		}
		sdio_slave_recv_load_buf(h);
	}
}

int c6_link_init(c6_serial_cb_t scb, c6_open_cb_t ocb)
{
	sdio_slave_config_t cfg = {
		.sending_mode     = SDIO_SLAVE_SEND_STREAM,
		.timing           = SDIO_SLAVE_TIMING_PSEND_PSAMPLE,
		.flags            = 0,          /* SDIO_SLAVE_FLAG_HIGH_SPEED */
		.send_queue_size  = 20,
		.recv_buffer_size = C6_SDIO_BUF,
		.event_cb         = host_int_cb,
	};
	int ret;

	serial_cb = scb;
	open_cb = ocb;
	k_mutex_init(&tx_lock);
	k_work_init(&hostint_work, hostint_worker);

	ret = sdio_slave_initialize(&cfg);
	if (ret) {
		LOG_ERR("sdio_slave_initialize: %d", ret);
		return ret;
	}
	for (int i = 0; i < C6_SDIO_NUM_BUFS; i++) {
		sdio_slave_buf_handle_t h =
			sdio_slave_recv_register_buf(rx_buf[i]);

		if (!h || sdio_slave_recv_load_buf(h)) {
			LOG_ERR("recv buf %d", i);
			return -ENOMEM;
		}
	}
	ret = sdio_slave_start();
	if (ret) {
		LOG_ERR("sdio_slave_start: %d", ret);
		return ret;
	}
	LOG_INF("sdio slave up");

	k_thread_create(&rx_thread, rx_stack, RX_THREAD_STACK, rx_entry,
			NULL, NULL, NULL, RX_THREAD_PRIO, 0, K_NO_WAIT);
	k_thread_name_set(&rx_thread, "c6_sdio_rx");
	return 0;
}

/* drain completed tx frames — buffers were malloc'd per fragment */
static void tx_reclaim(void)
{
	void *arg;

	while (sdio_slave_send_get_finished(&arg, K_NO_WAIT) == 0) {
		k_free(arg);
	}
}

int c6_link_tx_serial(const char *ep, const uint8_t *pb, uint16_t pb_len)
{
	uint8_t *tlv, *p;
	uint16_t ep_len = strlen(ep);
	uint32_t tlv_len = 3 + ep_len + 3 + pb_len;
	uint32_t left, off;
	int ret = 0;

	tx_reclaim();
	k_mutex_lock(&tx_lock, K_FOREVER);

	tlv = k_malloc(tlv_len);
	if (!tlv) {
		k_mutex_unlock(&tx_lock);
		return -ENOMEM;
	}

	/* assemble TLV stream: {0x01, ep_len(2), ep, 0x02, data_len(2), pb} */
	p = tlv;
	*p++ = C6_TLV_EP_NAME;
	*p++ = ep_len & 0xFF;
	*p++ = ep_len >> 8;
	memcpy(p, ep, ep_len);
	p += ep_len;
	*p++ = C6_TLV_DATA;
	*p++ = pb_len & 0xFF;
	*p++ = pb_len >> 8;
	memcpy(p, pb, pb_len);

	left = tlv_len;
	off = 0;
	while (left) {
		uint16_t frag = left > C6_MAX_FRAME ? C6_MAX_FRAME : left;
		uint8_t *frame = k_malloc(C6_HDR_LEN + frag);
		struct c6_hdr *hdr;
		int r;

		if (!frame) {
			ret = -ENOMEM;
			break;
		}
		hdr = (struct c6_hdr *)frame;
		memset(hdr, 0, C6_HDR_LEN);
		hdr->if_type = C6_IF_SERIAL;
		hdr->len = sys_cpu_to_le16(frag);
		hdr->offset = sys_cpu_to_le16(C6_DATA_OFFSET);
		hdr->seq_num = sys_cpu_to_le16(++tx_seq);
		hdr->flags = (left > C6_MAX_FRAME) ? C6_FLAG_MORE_FRAG : 0;
		memcpy(frame + C6_HDR_LEN, tlv + off, frag);
		hdr->checksum = sys_cpu_to_le16(
			esp_checksum(frame, C6_HDR_LEN + frag));

		r = sdio_slave_send_queue(frame, C6_HDR_LEN + frag, frame,
					  K_SECONDS(10));
		if (r) {
			LOG_ERR("tx queue: %d", r);
			k_free(frame);
			ret = r;
			break;
		}
		off += frag;
		left -= frag;
	}

	k_free(tlv);
	k_mutex_unlock(&tx_lock);
	return ret;
}

bool c6_link_datapath(void)
{
	return datapath;
}
