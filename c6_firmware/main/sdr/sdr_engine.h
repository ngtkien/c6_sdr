/* SDR engine for the c6_sdr hosted slave firmware.
 *
 * Radio-side glue around the ESP-SDR capture engine (ring_capture.c /
 * spectrum.c / chip.h), driven over the hosted SDIO RPC instead of the
 * ESP-SDR UART protocol. After esp_wifi_init() the radio is parked in
 * WIFI_MODE_NULL + promiscuous (same trick as esp-sdr's app_main) so
 * prepare_rx() owns the RF frontend while the hosted link stays up.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Mirror of ring_result_t fields the host cares about (status/detail/
 * pairs/elapsed/frames/drops/abandoned) — kept loose so the RPC layer
 * doesn't need ring_capture internals. */
typedef struct {
	uint32_t status, detail;
	uint32_t units;
	uint64_t pairs;
	uint64_t elapsed_us;
	uint32_t frames, drops, abandoned, ffts;
	uint32_t total_len;   /* bytes in the capture buffer */
	bool     stopped;
} sdr_run_result_t;

int  sdr_engine_prealloc(void);  /* reserve the capture sink early (pre-wifi heap) */
int  sdr_engine_init(void);      /* radio → SDR mode; call once after esp_wifi_init */
bool sdr_engine_ready(void);
int  sdr_set_freq(uint32_t hz);  /* retune; also applied per-call */
int  sdr_set_gain(int code);     /* -1 = hardware AGC, else PHY gain index 0..54 */
int  sdr_set_analog_bw(uint32_t mhz); /* 0 = auto, 12..54 MHz → dcap code */

/* Capture output buffer shared by all runs; sdr_iq_read() serves RPCs. */
uint32_t sdr_iq_read(uint32_t offset, uint8_t *dst, uint32_t max_len,
                     uint32_t *total_len);

/* SPEC: run ring_capture in RING_MODE_SPEC for duration_ms (0 = until
 * sdr_stop). Frames land in the capture buffer. */
int  sdr_spec_run(uint32_t freq_hz, uint8_t rate_code, uint16_t nfft,
                  uint8_t stride, uint8_t units_per_frame, bool max_hold,
                  bool stats, uint32_t duration_ms, sdr_run_result_t *res);

/* Async variant: returns immediately, run executes in a dedicated task
 * while the host drains the 32 KiB ring via sdr_stream_read. Bounded
 * (duration_ms>0) runs stop on their own; unbounded runs need sdr_stop
 * or they die when the undrained output stalls for 2 s. */
int  sdr_spec_start(uint32_t freq_hz, uint8_t rate_code, uint16_t nfft,
                    uint8_t stride, uint8_t units_per_frame, bool max_hold,
                    bool stats, uint32_t duration_ms);

/* Absolute-offset read of the stream ring. *pos = absolute position of
 * dst[0] (> offset when stale data was overwritten — host resyncs),
 * *produced = total bytes produced so far. */
uint32_t sdr_stream_read(uint32_t offset, uint8_t *dst, uint32_t max_len,
                         uint32_t *pos, uint32_t *produced);
bool sdr_read_is_ring(void);    /* last run's output lives in the stream ring */
uint32_t sdr_produced(void);    /* absolute bytes produced by the current run */

/* IQ mode 0: single-shot burst into IQ_BUFFER (raw u32 words / packed).
 * fmt: 0 = raw u32 (4 B/pair), 16 = iq8 (2 B/pair), 20 = iq10 (5 B/2 pairs). */
int  sdr_iq_burst(uint32_t freq_hz, uint8_t rate_code, uint16_t n_pairs,
                  uint8_t fmt, sdr_run_result_t *res);

/* IQ mode 1: ring decimated-IQ run into the capture buffer. */
int  sdr_iq_run(uint32_t freq_hz, uint8_t rate_code, uint16_t dec,
                uint8_t bits, uint8_t shift, bool rot,
                uint32_t duration_ms, sdr_run_result_t *res);

void sdr_stop(void);             /* abort an in-flight ring run */
bool sdr_busy(void);             /* a run is in progress */
