/* SPDX-License-Identifier: Apache-2.0 */

#include <string.h>
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>

#include "sdr.h"

LOG_MODULE_REGISTER(sdr, LOG_LEVEL_INF);

int sdr_pull_all(uint32_t total_len, uint8_t *buf, uint32_t buf_cap,
		 uint32_t *out_len)
{
	uint32_t off = 0;
	int ret = 0, idle = 0;

	(void)total_len;
	while (off < buf_cap) {
		uint32_t got = 0, total = 0, roff = 0;

		ret = esp_ng_sdr_iq_read(off, buf + off, buf_cap - off,
					 &got, &total, &roff);
		if (ret == -ENOENT) {
			ret = 0;
			break;
		}
		if (ret == -EAGAIN) {
			if (++idle > 100) {
				ret = 0;
				break;
			}
			k_msleep(2);
			continue;
		}
		if (ret) {
			return ret;
		}
		idle = 0;
		if (roff != off) {
			off = roff; /* lagging read lost stale bytes — resync */
		}
		if (!got) {
			break; /* slave has no more staged data */
		}
		off += got;
	}
	*out_len = off;
	return ret;
}

int sdr_spec_pull(uint32_t freq_hz, uint16_t nfft, uint8_t stride,
		  uint8_t units_per_frame, uint8_t max_hold,
		  uint32_t gain, uint32_t dcap, uint32_t duration_ms,
		  uint8_t *buf, uint32_t buf_cap, uint32_t *out_len,
		  struct esp_ng_sdr_run_res *res)
{
	struct esp_ng_sdr_spec_req req = {
		.freq_hz = freq_hz,
		.nfft = nfft,
		.stride = stride,
		.units_per_frame = units_per_frame,
		.max_hold = max_hold,
		.stats = 0,
		.gain = gain,
		.dcap = dcap,
		.duration_ms = duration_ms,
	};
	int ret = esp_ng_sdr_spec(&req, res);

	if (ret) {
		return ret;
	}
	return sdr_pull_all(res->total_len, buf, buf_cap, out_len);
}

int sdr_iq_burst_pull(uint32_t freq_hz, uint16_t n_pairs, uint8_t fmt,
		      uint32_t gain, uint32_t dcap,
		      uint8_t *buf, uint32_t buf_cap, uint32_t *out_len,
		      struct esp_ng_sdr_run_res *res)
{
	struct esp_ng_sdr_iq_req req = {
		.freq_hz = freq_hz,
		.mode = 0,
		.n_words = n_pairs,
		.units = 0,
		.format = fmt,
		.gain = gain,
		.dcap = dcap,
		.duration_ms = 0,
	};
	int ret = esp_ng_sdr_iq_start(&req, res);

	if (ret) {
		return ret;
	}
	return sdr_pull_all(res->total_len, buf, buf_cap, out_len);
}

/* Consume complete SPC1/SPS1 records from the front of buf; returns
 * bytes consumed (a partial trailing record stays for the next chunk).
 * Sets *halt if cb vetoes the stream. */
static uint32_t spc1_consume(const uint8_t *buf, uint32_t len,
			     bool (*cb)(const struct sdr_spc1 *,
					const uint8_t *, void *),
			     void *arg, int *frames, bool *halt)
{
	uint32_t off = 0;

	while (off + 4 <= len) {
		uint32_t magic;
		uint32_t flen;

		memcpy(&magic, buf + off, 4);
		if (magic == SDR_SPC1_MAGIC) {
			struct sdr_spc1 h;
			uint32_t nfft;

			if (off + SDR_SPC1_HDR > len) {
				break;
			}
			memcpy(&h, buf + off, sizeof(h));
			nfft = 1u << (h.nfft_log2 & 0x0f);
			if (!nfft || nfft > 4096) {
				break;
			}
			flen = SDR_SPC1_HDR + nfft + 4;
			if (off + flen > len) {
				break;
			}
			(*frames)++;
			if (cb && !cb(&h, buf + off + SDR_SPC1_HDR, arg)) {
				*halt = true;
				off += flen;
				break;
			}
			off += flen;
		} else if (magic == SDR_SPS1_MAGIC) {
			if (off + 40 > len) {
				break;
			}
			off += 40;
		} else {
			break;
		}
	}
	return off;
}

int sdr_walk_spc1(const uint8_t *buf, uint32_t len,
		  bool (*cb)(const struct sdr_spc1 *h, const uint8_t *bins,
			     void *arg),
		  void *arg)
{
	int frames = 0;
	bool halt = false;

	spc1_consume(buf, len, cb, arg, &frames, &halt);
	return frames;
}

/* Stream a SPEC run: the slave captures asynchronously while we drain
 * the ring live — cb() fires per frame as it lands. Bounded runs end
 * via -ENOENT when drained; unbounded runs (duration_ms = 0) stop after
 * max_frames (esp_ng_sdr_stop) or when cb returns false. */
int sdr_spec_stream(uint32_t freq_hz, uint16_t nfft, uint8_t stride,
		    uint8_t units_per_frame, uint8_t max_hold,
		    uint32_t gain, uint32_t dcap, uint32_t duration_ms,
		    uint32_t max_frames,
		    bool (*cb)(const struct sdr_spc1 *, const uint8_t *,
			       void *),
		    void *arg, uint32_t *out_bytes, int *out_frames)
{
	struct esp_ng_sdr_spec_req req = {
		.freq_hz = freq_hz,
		.nfft = nfft,
		.stride = stride,
		.units_per_frame = units_per_frame,
		.max_hold = max_hold,
		.stats = 0,
		.gain = gain,
		.dcap = dcap,
		.duration_ms = duration_ms,
	};
	struct esp_ng_sdr_run_res res;
	uint32_t consumed = 0, fill = 0;
	static uint8_t acc[2048]; /* read chunk + partial-frame carry */
	int frames = 0, idle = 0, ret;
	bool halt = false;

	ret = esp_ng_sdr_spec(&req, &res);
	if (ret) {
		return ret;
	}
	if (res.resp) {
		return -EIO;
	}
	*out_bytes = 0;
	uint64_t t_data = 0, t_again = 0;
	uint32_t n_data = 0, n_again = 0;
	for (;;) {
		uint32_t got = 0, total = 0, roff = 0;
		uint32_t c0 = k_cycle_get_32();

		ret = esp_ng_sdr_iq_read(consumed, acc + fill,
					 sizeof(acc) - fill,
					 &got, &total, &roff);
		uint32_t us = k_cyc_to_us_floor32(k_cycle_get_32() - c0);
		if (ret == -ENOENT) {
			printk("rd: %u data %u.%ums avg | %u again %u.%ums avg\n",
			       n_data, (uint32_t)(t_data / (n_data ?: 1) / 10),
			       (uint32_t)(t_data / (n_data ?: 1) % 10),
			       n_again, (uint32_t)(t_again / (n_again ?: 1) / 10),
			       (uint32_t)(t_again / (n_again ?: 1) % 10));
			break; /* run over, ring drained */
		}
		if (ret == -EAGAIN) {
			t_again += us; n_again++;
			if (++idle > 300) {
				break; /* run died silently */
			}
			k_msleep(2);
			continue;
		}
		if (ret) {
			esp_ng_sdr_stop();
			return ret;
		}
		t_data += us; n_data++;
		idle = 0;
		if (roff != consumed) {
			consumed = roff; /* overrun gap — drop partial record */
			fill = 0;
		}
		fill += got;
		consumed += got;
		*out_bytes += got;
		uint32_t used = spc1_consume(acc, fill, cb, arg,
					     &frames, &halt);
		if (used) {
			memmove(acc, acc + used, fill - used);
			fill -= used;
		}
		if (halt ||
		    (max_frames && frames >= (int)max_frames)) {
			esp_ng_sdr_stop();
			break;
		}
	}
	*out_frames = frames;
	return frames ? 0 : -ENODATA;
}
