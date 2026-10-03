/* SPDX-License-Identifier: Apache-2.0 */

#include <string.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>

#include "sdr.h"

LOG_MODULE_REGISTER(sdr, LOG_LEVEL_INF);

int sdr_pull_all(uint32_t total_len, uint8_t *buf, uint32_t buf_cap,
		 uint32_t *out_len)
{
	uint32_t off = 0;
	int ret;

	while (off < total_len && off < buf_cap) {
		uint32_t got = 0, total = 0;

		ret = esp_ng_sdr_iq_read(off, buf + off, buf_cap - off,
					 &got, &total);
		if (ret) {
			return ret;
		}
		if (!got) {
			break; /* slave has no more staged data */
		}
		off += got;
	}
	*out_len = off;
	return 0;
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

int sdr_walk_spc1(const uint8_t *buf, uint32_t len,
		  bool (*cb)(const struct sdr_spc1 *h, const uint8_t *bins,
			     void *arg),
		  void *arg)
{
	uint32_t off = 0;
	int frames = 0;

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
			frames++;
			if (cb && !cb(&h, buf + off + SDR_SPC1_HDR, arg)) {
				break;
			}
			off += flen;
		} else if (magic == SDR_SPS1_MAGIC) {
			/* stats frame: 36 B body + crc */
			if (off + 40 > len) {
				break;
			}
			off += 40;
		} else {
			break;
		}
	}
	return frames;
}
