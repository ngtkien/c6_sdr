/* SDR engine — ESP-SDR radio path driven by hosted RPC instead of UART.
 *
 * Extracted from esp-sdr/main/families/c5_c6_c61/receiver.c (C6 branches
 * only): the same PHY debug externs, analog filter save/apply/restore,
 * single-shot stock_capture() into IQ_BUFFER, and ring_capture_run() for
 * SPEC / raw CAPTURE modes. Output goes to a flat DRAM buffer (or stays in
 * the reserved SRAM banks) and is pulled by the SdrIqRead RPC.
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_rom_crc.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "heap_memory_layout.h"
#include "soc/soc.h"

#ifndef CONFIG_IDF_TARGET_ESP32C6
#define CONFIG_IDF_TARGET_ESP32C6 0
#endif
#include "chip.h"
#include "ring_capture.h"
#include "rx_tuning.h"
#include "sdr_engine.h"
#include "ring_io.h"
#include "burst_serial.h"

static const char *TAG = "c6_sdr";

/* ---- PHY debug hooks (same externs esp-sdr uses; chip.h remaps the names) */
extern void phy_stop_tx_tone(unsigned);
extern void phy_pbus_workmode(void);
extern void phy_pbus_xpd_rx_on(unsigned);
extern void phy_pbus_xpd_tx_off(void);
extern void phy_set_rxclk_en(unsigned);
extern void phy_chip_set_chan(unsigned, unsigned);
extern void phy_rx_filter_mode(unsigned);
extern unsigned phy_chip_i2c_readReg(unsigned, unsigned, unsigned);
extern void phy_i2c_writeReg(unsigned, unsigned, unsigned, unsigned);

static unsigned frequency_mhz = 2412;
static bool rx_ready;

#define RX_FILTER_REG 4u /* C6/C61 */
static int rx_analog_filter = -1; /* -1: keep PHY-calibrated automatic */

/* RX-only analog capacitance: preserve PHY calibration between snapshots. */
static void rx_analog_apply(unsigned saved[2]) {
    for (unsigned j = 0; j < 2; j++) {
        saved[j] = phy_chip_i2c_readReg(0x67, 1, RX_FILTER_REG + j);
        if (rx_analog_filter >= 0)
            phy_i2c_writeReg(0x67, 1, RX_FILTER_REG + j,
                             (saved[j] & ~63u) | (unsigned)rx_analog_filter);
    }
}
static void rx_analog_restore(const unsigned saved[2]) {
    if (rx_analog_filter >= 0)
        for (unsigned j = 0; j < 2; j++)
            phy_i2c_writeReg(0x67, 1, RX_FILTER_REG + j, saved[j]);
}

/* Required by the stock RF test archive; no shell is exposed. */
int cmd_parse(char *cmd, char *name, int *argc, char **argv) {
    (void)cmd; (void)name; (void)argc; (void)argv;
    return -1;
}
#include "burst_gain.h"

static void prepare_rx(void) {
    if (rx_ready) return;
    phy_chip_set_chan(frequency_mhz, 0);
    phy_stop_tx_tone(1);
    phy_pbus_workmode();
    phy_pbus_xpd_tx_off();
    phy_pbus_xpd_rx_on(1);
    phy_set_rxclk_en(1);
    gain_apply();
    rx_ready = true;
}

/* ---- capture output ------------------------------------------------------
 * SPEC frames and packed bursts append to sdr_cap; raw ring-CAPTURE units
 * stay in the SRAM banks and are read back through the unit table.
 *
 * sdr_cap is a true ring with absolute byte positions: the capture task
 * produces at sdr_wr_abs, the host consumes via SdrIqRead at sdr_rd_abs.
 * Reads reference absolute offsets, so the host can drain while the run
 * still captures (stream mode) and a lagging reader resyncs to the
 * oldest byte still held.
 */
/* Hosted heap tops out around ~120 KiB; transport ~47 KiB + wifi ~35 KiB
 * leaves <40 KiB — 96 KiB was never reachable, 48 KiB starves sdio_init.
 * With live host draining the ring only needs to cover pull latency
 * (~50 ms at ~95 KiB/s); 24 KiB keeps ~250 ms of slack and leaves the
 * SDIO send-path a usable DMA block. */
#define SDR_CAP_SIZE (24 * 1024)
/* Heap-allocated at engine init: the .bss region must stay below the
 * 0x40820000 RF-dump guard, so the sink lives in the heap above the
 * reserved ring banks instead. */
static uint8_t *sdr_cap;
static volatile uint32_t sdr_wr_abs; /* total bytes produced */
static volatile uint32_t sdr_rd_abs; /* bytes consumed by host reads */
static volatile bool sdr_stop_flag;
static volatile bool sdr_busy_flag;

static enum { SRC_NONE, SRC_BUF, SRC_UNITS, SRC_RING } read_src;
static const uint8_t *read_base;
static uint32_t read_len;
static ring_result_t last_run;

/* async run task (see sdr_spec_start below); stack + TCB allocated in
 * sdr_engine_prealloc — heap, not .bss, which is hard-capped by the
 * 0x40820000 RF-dump guard */
static StaticTask_t *sdr_task_tcb;
static StackType_t *sdr_task_stack;
static TaskHandle_t sdr_task;
static ring_config_t sdr_pending_cfg;
static sdr_run_result_t sdr_async_res;
static void sdr_run_task(void *arg);

int ring_write(const uint8_t *data, size_t len) {
    uint32_t used = sdr_wr_abs - sdr_rd_abs;
    uint32_t space = SDR_CAP_SIZE - (used > SDR_CAP_SIZE ? SDR_CAP_SIZE : used);
    size_t n = len < space ? len : space;
    if (n) {
        uint32_t off = sdr_wr_abs % SDR_CAP_SIZE;
        uint32_t first = SDR_CAP_SIZE - off;
        if (first > n) first = n;
        memcpy(sdr_cap + off, data, first);
        if (n > first) memcpy(sdr_cap, data + first, n - first);
        sdr_wr_abs += (uint32_t)n;
    }
    return (int)n;
}

/* Absolute-position read for SRC_RING: returns bytes copied; *pos is the
 * absolute offset of dst[0] (advanced past the requested offset when the
 * host fell behind and stale bytes were overwritten), *produced is the
 * running total. Consumed bytes advance sdr_rd_abs so ring_write sees
 * the freed space. */
uint32_t sdr_stream_read(uint32_t offset, uint8_t *dst, uint32_t max_len,
                         uint32_t *pos, uint32_t *produced) {
    uint32_t wr = sdr_wr_abs;
    uint32_t used = wr - sdr_rd_abs;
    uint32_t avail = used > SDR_CAP_SIZE ? SDR_CAP_SIZE : used;
    uint32_t oldest = wr - avail;
    uint32_t p = offset < oldest ? oldest : offset;
    uint32_t n = wr - p;

    if (n > max_len) n = max_len;
    if (n) {
        uint32_t off = p % SDR_CAP_SIZE;
        uint32_t first = SDR_CAP_SIZE - off;
        if (first > n) first = n;
        memcpy(dst, sdr_cap + off, first);
        if (n > first) memcpy(dst + first, sdr_cap, n - first);
        uint32_t rd = p + n;
        if ((int32_t)(rd - sdr_rd_abs) > 0) sdr_rd_abs = rd;
    }
    *pos = p;
    *produced = wr;
    return n;
}
int ring_input_available(void) { return 0; }
int ring_read_byte(uint8_t *b) { (void)b; return -1; }

bool burst_serial_stop_requested(void) {
    if (!sdr_stop_flag) return false;
    sdr_stop_flag = false;
    return true;
}

/* Inert serial stubs: only spectrum.c's unused command path touches them. */
bool burst_serial_send(const void *data, size_t size) {
    (void)data; (void)size;
    return false;
}
burst_serial_port_t burst_serial_port(void) { return BURST_SERIAL_UART; }
unsigned burst_serial_baud(void) { return 0; }

/* ---- single-shot burst into IQ_BUFFER ------------------------------------ */

static size_t packed_size(unsigned n) { return (n * 20u + 7u) / 8u; }

/* Two complete IQ10 samples occupy five bytes; odd tail takes three. */
static void pack_iq(unsigned n) {
    uint8_t *p = (uint8_t *)IQ_BUFFER;
    for (unsigned j = 0; j < n; j += 2, p += 5) {
        uint32_t a = IQ_BUFFER[j] & 0xfffffu;
        uint32_t b = j + 1 < n ? IQ_BUFFER[j + 1] & 0xfffffu : 0;
        p[0] = a; p[1] = a >> 8; p[2] = (a >> 16) | (b << 4);
        if (j + 1 < n) { p[3] = b >> 4; p[4] = b >> 12; }
    }
}

/* IQ8: signed I then Q, upper eight bits of each IQ10 field. */
static void pack_iq8(unsigned n) {
    uint8_t *p = (uint8_t *)IQ_BUFFER;
    for (unsigned j = 0; j < n; j++) {
        uint32_t w = IQ_BUFFER[j];
        p[2 * j] = (w >> 2) & 255;
        p[2 * j + 1] = (w >> 12) & 255;
    }
}

static size_t wire_size(unsigned n, unsigned format) {
    return format == 16 ? n * 2 : format == 20 ? packed_size(n) : n * 4;
}

static bool acquire_iq(unsigned n, unsigned *capture_us) {
    prepare_rx();

    for (unsigned j = 0; j < n; j++) IQ_BUFFER[j] = 0xa5a0055au;
    for (unsigned j = 0; j < 4; j++) IQ_BUFFER[n + j] = 0x5a5aa5a5u ^ j;
    unsigned analog_saved[2];
    rx_analog_apply(analog_saved);
    uint32_t owner = REG_READ(SRAM_OWNER_REG);
    int64_t start = esp_timer_get_time();
    bool done = stock_capture(n, 0); /* C6: only the 80 MS/s source-15 path */
    uint32_t elapsed = (uint32_t)(esp_timer_get_time() - start);
    REG_WRITE(SRAM_OWNER_REG, owner);
    rx_analog_restore(analog_saved);
    if (!done) return false;
    for (unsigned j = 0; j < n; j++)
        if (IQ_BUFFER[j] == 0xa5a0055au) return false;
    for (unsigned j = 0; j < 4; j++)
        if (IQ_BUFFER[n + j] != (0x5a5aa5a5u ^ j)) return false;
    *capture_us = elapsed;
    return true;
}

/* ---- gain / tuning -------------------------------------------------------- */

int sdr_set_gain(int code) { /* -1 = hardware AGC, else 0..gain_max() */
    if (code < 0) {
        gain_mode = GAIN_HARDWARE;
    } else {
        if ((unsigned)code > gain_max()) return ESP_ERR_INVALID_ARG;
        gain_mode = GAIN_MANUAL;
        gain_code = (unsigned)code;
    }
    rx_ready = false;
    return 0;
}

int sdr_set_analog_bw(uint32_t mhz) { /* 0 = PHY auto, else dcap code 0..63 */
    if (mhz == 0) {
        rx_analog_filter = -1;
        return 0;
    }
    if (mhz > 63) return ESP_ERR_INVALID_ARG;
    rx_analog_filter = (int)mhz;
    return 0;
}

int sdr_set_freq(uint32_t hz) {
    unsigned mhz = hz / 1000000u;
    unsigned rem = hz % 1000000u;
    if (rem >= 500000u) mhz++;
    if (!rx_frequency_valid(mhz)) return ESP_ERR_INVALID_ARG;
    if (mhz != frequency_mhz) {
        frequency_mhz = mhz;
        rx_ready = false;
    }
    return 0;
}

/* ---- init ----------------------------------------------------------------- */

/* Take the capture sink before wifi/RPC buffers fragment the heap.
 * Call from app_main — by the time the first SDR RPC arrives the wifi
 * pool has eaten ~35 KiB and a 96 KiB block no longer fits. */
int sdr_engine_prealloc(void) {
    if (sdr_cap) {
        return ESP_OK;
    }
    sdr_cap = heap_caps_malloc(SDR_CAP_SIZE,
                             MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!sdr_cap) {
        ESP_LOGE(TAG, "capture buffer alloc failed (%u B)", SDR_CAP_SIZE);
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "capture sink %u KiB reserved", SDR_CAP_SIZE / 1024);
    /* runner-task stack + TCB while the pre-wifi heap is still roomy;
     * the task itself is only kicked by sdr_spec_start(). Stack bytes
     * come out of the same DMA pool the SDIO send path needs, so keep
     * it tight: 3 KiB covers FFT + frame assembly. */
    sdr_task_stack = heap_caps_malloc(3072,
                                      MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    sdr_task_tcb = heap_caps_malloc(sizeof(StaticTask_t),
                                    MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (sdr_task_stack && sdr_task_tcb) {
        sdr_task = xTaskCreateStatic(sdr_run_task, "sdr_run", 3072,
                                   NULL, 5, sdr_task_stack, sdr_task_tcb);
    }
    if (!sdr_task) {
        ESP_LOGW(TAG, "no run task — async spec streaming unavailable");
    }
    return ESP_OK;
}

int sdr_engine_init(void) {
    /* Caller has already run esp_wifi_init/set_storage/set_mode(NULL)/
     * start/ps(NONE)/promiscuous(1)/set_channel — same sequence esp-sdr's
     * app_main uses to park the radio for raw receive. */
    esp_err_t ret = sdr_engine_prealloc();
    if (ret != ESP_OK) {
        return ret;
    }
    ring_capture_init();
    prepare_rx();
    ESP_LOGI(TAG, "sdr engine ready @%u MHz, gain_max=%u", frequency_mhz,
             gain_max());
    return 0;
}

bool sdr_engine_ready(void) { return rx_ready; }
bool sdr_busy(void) { return sdr_busy_flag; }
bool sdr_read_is_ring(void) { return read_src == SRC_RING; }
uint32_t sdr_produced(void) { return sdr_wr_abs; }

/* ---- run wrappers ---------------------------------------------------------- */

static void run_translate(const ring_result_t *r, sdr_run_result_t *res) {
    res->status = r->status;
    res->detail = r->detail;
    res->units = r->units;
    res->pairs = r->pairs;
    res->elapsed_us = r->elapsed_us;
    res->frames = r->frames;
    res->drops = r->drops;
    res->abandoned = r->abandoned;
    res->ffts = r->ffts;
    res->total_len = sdr_wr_abs;
    res->stopped = r->stopped_by_host;
}

static int ring_run(const ring_config_t *cfg, sdr_run_result_t *res) {
    sdr_wr_abs = 0;
    sdr_rd_abs = 0;
    sdr_stop_flag = false;
    sdr_busy_flag = true;
    ring_capture_init();
    prepare_rx();
    unsigned saved[2];
    rx_analog_apply(saved);
    ring_result_t r;
    ring_capture_run(cfg, &r);
    rx_analog_restore(saved);
    sdr_busy_flag = false;
    last_run = r;
    run_translate(&r, res);
    return 0;
}

int sdr_spec_run(uint32_t freq_hz, uint8_t rate_code, uint16_t nfft,
                 uint8_t stride, uint8_t units_per_frame, bool max_hold,
                 bool stats, uint32_t duration_ms, sdr_run_result_t *res) {
    if (sdr_busy_flag) return ESP_ERR_INVALID_STATE;
    if (freq_hz && sdr_set_freq(freq_hz)) return ESP_ERR_INVALID_ARG;
    if (rate_code != 0 || nfft != 256 || !stride || stride > 64 ||
        !units_per_frame || duration_ms > 86400000u)
        return ESP_ERR_INVALID_ARG;
    ring_config_t cfg = {
        .mode = RING_MODE_SPEC, .rate = rate_code,
        .duration_ms = duration_ms, .nfft = nfft, .stride = stride,
        .units_per_frame = units_per_frame, .max_hold = max_hold,
        .stats = stats,
    };
    int rc = ring_run(&cfg, res);
    if (!rc) {
        res->total_len = sdr_wr_abs;
        read_src = SRC_RING;
        read_len = sdr_wr_abs;
    }
    return rc;
}

/* ---- async runs: capture in a dedicated task so the RPC handler can
 * answer immediately and the host drains via SdrIqRead while the run is
 * still producing. */
static void sdr_run_task(void *arg) {
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        ring_run(&sdr_pending_cfg, &sdr_async_res);
        ESP_LOGI(TAG, "run done: st=%u units=%lu frames=%lu drops=%lu len=%lu stk=%lu",
                 (unsigned)last_run.status, (unsigned long)last_run.units,
                 (unsigned long)last_run.frames,
                 (unsigned long)last_run.drops, (unsigned long)sdr_wr_abs,
                 (unsigned long)uxTaskGetStackHighWaterMark(NULL));
    }
}

int sdr_spec_start(uint32_t freq_hz, uint8_t rate_code, uint16_t nfft,
                   uint8_t stride, uint8_t units_per_frame, bool max_hold,
                   bool stats, uint32_t duration_ms) {
    if (sdr_busy_flag) return ESP_ERR_INVALID_STATE;
    if (freq_hz && sdr_set_freq(freq_hz)) return ESP_ERR_INVALID_ARG;
    if (rate_code != 0 || nfft != 256 || !stride || stride > 64 ||
        !units_per_frame || duration_ms > 86400000u)
        return ESP_ERR_INVALID_ARG;
    if (!sdr_task) return ESP_ERR_INVALID_STATE;
    sdr_pending_cfg = (ring_config_t){
        .mode = RING_MODE_SPEC, .rate = rate_code,
        .duration_ms = duration_ms, .nfft = nfft, .stride = stride,
        .units_per_frame = units_per_frame, .max_hold = max_hold,
        .stats = stats,
    };
    read_src = SRC_RING;
    sdr_wr_abs = 0;
    sdr_rd_abs = 0;
    sdr_busy_flag = true; /* reads see TIMEOUT, not drained, before task wakes */
    xTaskNotifyGive(sdr_task);
    return 0;
}

int sdr_iq_burst(uint32_t freq_hz, uint8_t rate_code, uint16_t n_pairs,
                 uint8_t fmt, sdr_run_result_t *res) {
    if (sdr_busy_flag) return ESP_ERR_INVALID_STATE;
    if (freq_hz && sdr_set_freq(freq_hz)) return ESP_ERR_INVALID_ARG;
    if (rate_code != 0 || n_pairs < 256 || n_pairs > IQ_WORDS ||
        (fmt != 0 && fmt != 16 && fmt != 20))
        return ESP_ERR_INVALID_ARG;
    sdr_busy_flag = true;
    memset(res, 0, sizeof(*res));
    unsigned elapsed = 0;
    bool ok = acquire_iq(n_pairs, &elapsed);
    sdr_busy_flag = false;
    if (!ok) {
        res->status = 1;
        res->detail = 1;
        return 0;
    }
    if (fmt == 16) pack_iq8(n_pairs);
    else if (fmt == 20) pack_iq(n_pairs);
    res->pairs = n_pairs;
    res->elapsed_us = elapsed;
    res->units = 1;
    res->total_len = wire_size(n_pairs, fmt);
    read_src = SRC_BUF;
    read_base = (const uint8_t *)IQ_BUFFER;
    read_len = res->total_len;
    return 0;
}

int sdr_iq_run(uint32_t freq_hz, uint8_t rate_code, uint16_t dec,
               uint8_t bits, uint8_t shift, bool rot,
               uint32_t duration_ms, sdr_run_result_t *res) {
    (void)dec; (void)bits; (void)shift; (void)rot;
    /* C6 has no decimated-IQ path (RING_MODE_IQ is S3-only); use
     * RING_MODE_CAPTURE for a gapless raw grab instead. dec is reused as
     * the unit count (1..RING_BANKS). */
    if (sdr_busy_flag) return ESP_ERR_INVALID_STATE;
    if (freq_hz && sdr_set_freq(freq_hz)) return ESP_ERR_INVALID_ARG;
    if (rate_code != 0 || dec < 1 || dec > RING_BANKS ||
        duration_ms > 86400000u)
        return ESP_ERR_INVALID_ARG;
    ring_config_t cfg = {
        .mode = RING_MODE_CAPTURE, .rate = rate_code,
        .duration_ms = duration_ms, .capture_units = dec,
    };
    int rc = ring_run(&cfg, res);
    if (!rc) {
        read_src = SRC_UNITS;
        uint32_t total = 0;
        for (unsigned u = 0; u < res->units && u < RING_BANKS; u++)
            total += last_run.cap[u].count * 4u;
        read_len = total;
        res->total_len = total;
    }
    return rc;
}

void sdr_stop(void) { sdr_stop_flag = true; }

uint32_t sdr_iq_read(uint32_t offset, uint8_t *dst, uint32_t max_len,
                     uint32_t *total_len) {
    *total_len = read_len;
    if (offset >= read_len) return 0;
    uint32_t n = read_len - offset;
    if (n > max_len) n = max_len;
    if (read_src == SRC_BUF) {
        memcpy(dst, read_base + offset, n);
    } else if (read_src == SRC_UNITS) {
        /* Walk the captured unit table: offset is a byte offset into the
         * concatenation of cap[u].count 32-bit words at each bank. */
        uint32_t pos = 0, out = 0;
        for (unsigned u = 0; u < last_run.units && u < RING_BANKS && out < n; u++) {
            uint32_t bytes = last_run.cap[u].count * 4u;
            if (offset < pos + bytes) {
                uint32_t in_off = offset - pos;
                uint32_t chunk = bytes - in_off;
                if (chunk > n - out) chunk = n - out;
                const uint32_t *src =
                    ring_capture_bank(last_run.cap[u].bank) +
                    last_run.cap[u].first + in_off / 4;
                memcpy(dst + out, (const uint8_t *)src + (in_off & 3), chunk);
                out += chunk;
            }
            pos += bytes;
        }
        return out;
    } else {
        return 0;
    }
    return n;
}
