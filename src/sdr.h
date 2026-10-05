/* SPDX-License-Identifier: Apache-2.0
 *
 * c6_sdr host-side helpers: pull staged captures off the C6 via the
 * esp_ng_sdr_* RPCs and parse the ESP-SDR SPC1 spectrum frames.
 */
#pragma once

#include <stdint.h>
#include <stddef.h>
#include "esp_hosted_ng.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ESP-SDR "SPC1" spectrum frame (wire, little-endian, packed) */
struct __attribute__((packed)) sdr_spc1 {
	uint32_t magic;        /* 0x31435053 "SPC1" */
	uint32_t frame_seq;
	uint64_t pair_index;
	uint32_t pairs;
	uint16_t ffts;
	uint8_t  flags;        /* bit0 max-hold, bit1 abandoned, bit2 dropped */
	uint8_t  gain;
	uint16_t drops;
	uint8_t  nfft_log2;
	uint8_t  db_step;
};
#define SDR_SPC1_MAGIC 0x31435053u
#define SDR_SPS1_MAGIC 0x31535053u /* stats frame, 36+4 B */
#define SDR_SPC1_HDR   28u

/* Run one bounded SPEC capture on the C6 and pull every staged frame into
 * buf. Returns 0 on a clean run (res->status == 0 as well), negative errno
 * otherwise; *out_len receives staged bytes copied. */
int sdr_spec_pull(uint32_t freq_hz, uint16_t nfft, uint8_t stride,
		  uint8_t units_per_frame, uint8_t max_hold,
		  uint32_t gain, uint32_t dcap, uint32_t duration_ms,
		  uint8_t *buf, uint32_t buf_cap, uint32_t *out_len,
		  struct esp_ng_sdr_run_res *res);

/* Pull staged bytes after a Spec/IqStart call; returns copied length. */
int sdr_pull_all(uint32_t total_len, uint8_t *buf, uint32_t buf_cap,
		 uint32_t *out_len);

/* Stream a SPEC capture: the slave runs asynchronously and cb() fires
 * per SPC1 frame as it lands on the host. duration_ms bounds the run
 * (0 = run until max_frames / cb false / slave stall); max_frames = 0
 * drains to the natural end. Returns 0 with *out_frames/*out_bytes set,
 * -ENODATA if nothing arrived, or a negative transport error. */
int sdr_spec_stream(uint32_t freq_hz, uint16_t nfft, uint8_t stride,
		    uint8_t units_per_frame, uint8_t max_hold,
		    uint32_t gain, uint32_t dcap, uint32_t duration_ms,
		    uint32_t max_frames,
		    bool (*cb)(const struct sdr_spc1 *, const uint8_t *,
			       void *),
		    void *arg, uint32_t *out_bytes, int *out_frames);

/* Iterate SPC1 frames in a pulled blob. Returns parsed frame count;
 * cb() runs per frame with header + bins (NULL-safe to skip SPS1/other
 * records). cb return false stops the walk. */
int sdr_walk_spc1(const uint8_t *buf, uint32_t len,
		  bool (*cb)(const struct sdr_spc1 *h, const uint8_t *bins,
			     void *arg),
		  void *arg);

/* One-shot raw-IQ burst into buf (mode 0). fmt: 0 = raw u32 words,
 * 16 = iq8 (2 B/pair), 20 = iq10 (5 B/2 pairs). */
int sdr_iq_burst_pull(uint32_t freq_hz, uint16_t n_pairs, uint8_t fmt,
		      uint32_t gain, uint32_t dcap,
		      uint8_t *buf, uint32_t buf_cap, uint32_t *out_len,
		      struct esp_ng_sdr_run_res *res);

/* Factory RF-test CW tone on/off (IqStart mode 2/3) — emits real RF.
 * backoff_qdb: attenuation in 0.25 dB units, 0 = full PA power. */
int sdr_tone(uint32_t freq_hz, bool on, uint32_t backoff_qdb,
	     struct esp_ng_sdr_run_res *res);

#ifdef __cplusplus
}
#endif
