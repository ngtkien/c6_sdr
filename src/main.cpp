/* SPDX-License-Identifier: Apache-2.0
 *
 * ESP32-P4 driving the c6_sdr hybrid firmware on the ESP32-C6:
 * spectrum capture + raw I/Q over the existing hosted SDIO RPC link.
 *
 * Demo outputs (Kconfig):
 *   - ASCII waterfall on the serial console
 *   - live waterfall on the EK79007 DSI panel
 *   - capture dumps to /SD:/c6_sdr_*.bin
 *
 * Boot flow: hosted link check (GetMacAddress + fw version) -> periodic
 * bounded SdrSpec runs (P4 pulls SPC1 frames via SdrIqRead) -> optional
 * single-shot I/Q burst dump.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/cache.h>
#include <zephyr/drivers/display.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/fs/fs.h>
#include <ff.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include <bsp/esp32p4_bsp.h>

#include "esp_hosted_ng.h"
#include "sdr.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* staged captures: 96 KiB slave sink, pulled in 1.4 KB RPC chunks */
#define CAP_MAX  (96 * 1024)
static uint8_t cap_buf[CAP_MAX] __attribute__((section(".ext_ram.bss")));

/* ---------- SD dump ---------- */
static FATFS fat_fs;
static struct fs_mount_t mp = {
	.type = FS_FATFS,
	.mnt_point = "/SD:",
};
static bool sd_ok;
static int sd_fail_streak;

static void sd_mount_try(void)
{
	mp.fs_data = &fat_fs;
	if (disk_access_ioctl("SD", DISK_IOCTL_CTRL_INIT, NULL) == 0 &&
	    fs_mount(&mp) == 0) {
		sd_ok = true;
	}
}

/* card-side writes fail hard on some cards — give up after a few */
static void sd_rc(int rc)
{
	if (rc < 0 && ++sd_fail_streak >= 4) {
		sd_ok = false;
		printk("sd: writes failing (%d), disabling SD dump\n", rc);
	} else if (rc >= 0) {
		sd_fail_streak = 0;
	}
}

static void sd_write(const char *path, const uint8_t *buf, uint32_t len)
{
	struct fs_file_t f;
	int rc;

	if (!sd_ok) {
		return;
	}
	fs_file_t_init(&f);
	rc = fs_open(&f, path, FS_O_CREATE | FS_O_WRITE);
	if (rc) {
		LOG_WRN("sd open %s: %d", path, rc);
		sd_rc(rc);
		return;
	}
	rc = fs_write(&f, buf, len);
	fs_close(&f);
	sd_rc(rc);
	if (rc < 0)
		LOG_WRN("sd: %s write failed: %d", path, rc);
	else
		LOG_INF("sd: wrote %d B to %s", rc, path);
}

static void sd_append(const char *path, const uint8_t *buf, uint32_t len)
{
	struct fs_file_t f;
	int rc;

	if (!sd_ok) {
		return;
	}
	fs_file_t_init(&f);
	rc = fs_open(&f, path, FS_O_CREATE | FS_O_APPEND);
	if (rc) {
		sd_rc(rc);
		return;
	}
	rc = fs_write(&f, buf, len);
	fs_close(&f);
	sd_rc(rc);
}

/* ---------- DSI waterfall + HUD ---------- */
#define PANEL_W   1024
#define PANEL_H   600
#define WF_TOP    64   /* rows above this are the HUD */
#define WF_GUTTER 56   /* left column reserved for freq labels */
#define WF_W      (PANEL_W - WF_GUTTER)

static uint8_t fb[PANEL_W * PANEL_H * 3] __aligned(64)
	__attribute__((section(".ext_ram.bss")));
static const struct device *disp;
static int wf_row = WF_TOP;
static bool dsi_ok;

/* HUD live stats, refreshed once per spec call */
static char hud_link[96], hud_stat[96], hud_iq[96];
static uint32_t st_rows, st_drops, st_staged, st_sweep, st_fails;
static uint32_t st_freq_mhz;
static bool st_label_pending;
static int label_end_row = -1; /* gutter text occupies [row,row+14) */

/* scroll-rate meter: rows painted per second, EMA over ~1 s window */
static uint32_t fps_rows, fps_x10;
static int64_t fps_t0;

static void fps_tick(void)
{
	int64_t now = k_uptime_get();
	int64_t dt = now - fps_t0;

	if (dt >= 500) {
		uint32_t inst = (uint32_t)(fps_rows * 10000 / dt);

		fps_x10 = fps_x10 ? (fps_x10 * 3 + inst) / 4 : inst;
		fps_rows = 0;
		fps_t0 = now;
	}
}

/* 5x7 font, ASCII 32..126 (classic public-domain lcd font) */
static const uint8_t font5x7[][5] = {
	{0x00,0x00,0x00,0x00,0x00},{0x00,0x00,0x5f,0x00,0x00},
	{0x00,0x07,0x00,0x07,0x00},{0x14,0x7f,0x14,0x7f,0x14},
	{0x24,0x2a,0x7f,0x2a,0x12},{0x23,0x13,0x08,0x64,0x62},
	{0x36,0x49,0x55,0x22,0x50},{0x00,0x05,0x03,0x00,0x00},
	{0x00,0x1c,0x22,0x41,0x00},{0x00,0x41,0x22,0x1c,0x00},
	{0x14,0x08,0x3e,0x08,0x14},{0x08,0x08,0x3e,0x08,0x08},
	{0x00,0x50,0x30,0x00,0x00},{0x08,0x08,0x08,0x08,0x08},
	{0x00,0x60,0x60,0x00,0x00},{0x20,0x10,0x08,0x04,0x02},
	{0x3e,0x51,0x49,0x45,0x3e},{0x00,0x42,0x7f,0x40,0x00},
	{0x42,0x61,0x51,0x49,0x46},{0x21,0x41,0x45,0x4b,0x31},
	{0x18,0x14,0x12,0x7f,0x10},{0x27,0x45,0x45,0x45,0x39},
	{0x3c,0x4a,0x49,0x49,0x30},{0x01,0x71,0x09,0x05,0x03},
	{0x36,0x49,0x49,0x49,0x36},{0x06,0x49,0x49,0x29,0x1e},
	{0x00,0x36,0x36,0x00,0x00},{0x00,0x56,0x36,0x00,0x00},
	{0x08,0x14,0x22,0x41,0x00},{0x14,0x14,0x14,0x14,0x14},
	{0x00,0x41,0x22,0x14,0x08},{0x02,0x01,0x51,0x09,0x06},
	{0x32,0x49,0x79,0x41,0x3e},{0x7e,0x11,0x11,0x11,0x7e},
	{0x7f,0x49,0x49,0x49,0x36},{0x3e,0x41,0x41,0x41,0x22},
	{0x7f,0x41,0x41,0x22,0x1c},{0x7f,0x49,0x49,0x49,0x41},
	{0x7f,0x09,0x09,0x09,0x01},{0x3e,0x41,0x49,0x49,0x7a},
	{0x7f,0x08,0x08,0x08,0x7f},{0x00,0x41,0x7f,0x41,0x00},
	{0x20,0x40,0x41,0x3f,0x01},{0x7f,0x08,0x14,0x22,0x41},
	{0x7f,0x40,0x40,0x40,0x40},{0x7f,0x02,0x0c,0x02,0x7f},
	{0x7f,0x04,0x08,0x10,0x7f},{0x3e,0x41,0x41,0x41,0x3e},
	{0x7f,0x09,0x09,0x09,0x06},{0x3e,0x41,0x51,0x21,0x5e},
	{0x7f,0x09,0x19,0x29,0x46},{0x46,0x49,0x49,0x49,0x31},
	{0x01,0x01,0x7f,0x01,0x01},{0x3f,0x40,0x40,0x40,0x3f},
	{0x1f,0x20,0x40,0x20,0x1f},{0x3f,0x40,0x38,0x40,0x3f},
	{0x63,0x14,0x08,0x14,0x63},{0x07,0x08,0x70,0x08,0x07},
	{0x61,0x51,0x49,0x45,0x43},{0x00,0x7f,0x41,0x41,0x00},
	{0x02,0x04,0x08,0x10,0x20},{0x00,0x41,0x41,0x7f,0x00},
	{0x04,0x02,0x01,0x02,0x04},{0x40,0x40,0x40,0x40,0x40},
	{0x00,0x01,0x02,0x04,0x00},{0x20,0x54,0x54,0x54,0x78},
	{0x7f,0x48,0x44,0x44,0x38},{0x38,0x44,0x44,0x44,0x20},
	{0x38,0x44,0x44,0x48,0x7f},{0x38,0x54,0x54,0x54,0x18},
	{0x08,0x7e,0x09,0x01,0x02},{0x0c,0x52,0x52,0x52,0x3e},
	{0x7f,0x08,0x04,0x04,0x78},{0x00,0x44,0x7d,0x40,0x00},
	{0x20,0x40,0x44,0x3d,0x00},{0x7f,0x10,0x28,0x44,0x00},
	{0x00,0x41,0x7f,0x40,0x00},{0x7c,0x04,0x18,0x04,0x78},
	{0x7c,0x08,0x04,0x04,0x78},{0x38,0x44,0x44,0x44,0x38},
	{0x7c,0x14,0x14,0x14,0x08},{0x08,0x14,0x14,0x18,0x7c},
	{0x7c,0x08,0x04,0x04,0x08},{0x48,0x54,0x54,0x54,0x20},
	{0x04,0x3f,0x44,0x40,0x20},{0x3c,0x40,0x40,0x20,0x7c},
	{0x1c,0x20,0x40,0x20,0x1c},{0x3c,0x40,0x30,0x40,0x3c},
	{0x44,0x28,0x10,0x28,0x44},{0x0c,0x50,0x50,0x50,0x3c},
	{0x44,0x64,0x54,0x4c,0x44},{0x00,0x08,0x36,0x41,0x00},
	{0x00,0x00,0x7f,0x00,0x00},{0x00,0x41,0x36,0x08,0x00},
	{0x10,0x08,0x08,0x10,0x08},
};

static void fb_fill(int x, int y, int w, int h,
		    uint8_t r, uint8_t g, uint8_t b)
{
	for (int yy = y; yy < y + h && yy < PANEL_H; yy++) {
		if (yy < 0) {
			continue;
		}
		for (int xx = x; xx < x + w && xx < PANEL_W; xx++) {
			if (xx < 0) {
				continue;
			}
			uint8_t *p = fb + (yy * PANEL_W + xx) * 3;

			p[0] = r; p[1] = g; p[2] = b;
		}
	}
}

static void fb_glyph(int x, int y, char c, int scale,
		     uint8_t r, uint8_t g, uint8_t b)
{
	const uint8_t *gl;

	if (c < 32 || c > 126) {
		c = '?';
	}
	gl = font5x7[c - 32];
	for (int cx = 0; cx < 5; cx++) {
		for (int cy = 0; cy < 7; cy++) {
			if (gl[cx] & (1 << cy)) {
				fb_fill(x + cx * scale, y + cy * scale,
					scale, scale, r, g, b);
			}
		}
	}
}

static void fb_text(int x, int y, const char *s, int scale,
		    uint8_t r, uint8_t g, uint8_t b)
{
	for (; *s; s++, x += 6 * scale) {
		fb_glyph(x, y, *s, scale, r, g, b);
	}
}

/* dB-like bin code -> RGB888 heat ramp (blue->cyan->green->amber->red) */
static void db_rgb(uint8_t v, uint8_t *r, uint8_t *g, uint8_t *b)
{
	uint32_t x = v; /* 0..255 */

	if (x < 64) {        /* deep blue -> blue */
		*r = 0; *g = x; *b = 96 + x;
	} else if (x < 128) { /* blue -> cyan */
		*r = 0; *g = 64 + (x - 64); *b = 160;
	} else if (x < 192) { /* cyan -> green/amber */
		*r = (x - 128) * 2; *g = 128 + (x - 128); *b = 128 - (x - 128);
	} else {             /* amber -> red */
		*r = 160 + (x - 192); *g = 255 - (x - 192) * 3; *b = 0;
	}
}

/* repaint the HUD region in fb (title/link/stats/colorbar) */
static void hud_draw(void)
{
	fb_fill(0, 0, PANEL_W, WF_TOP, 8, 8, 16);
	fb_text(8, 4, "C6-SDR LIVE SPECTRUM", 2, 0, 220, 255);
	fb_text(8, 22, hud_link, 2, 160, 200, 255);
	fb_text(8, 40, hud_stat, 2, 255, 220, 0);
	if (hud_iq[0]) {
		fb_text(560, 40, hud_iq, 2, 120, 255, 160);
	}
	/* dB colorbar + anchor labels */
	for (int i = 0; i < 256; i++) {
		uint8_t r, g, b;

		db_rgb((uint8_t)i, &r, &g, &b);
		fb_fill(560 + i * 2, 4, 2, 12, r, g, b);
	}
	fb_text(560, 18, "QUIET", 1, 120, 160, 220);
	fb_text(1024 - 6 * 6, 18, "LOUD", 1, 255, 120, 0);
	fb_fill(0, WF_TOP - 1, PANEL_W, 1, 40, 40, 60);
}

/* zero-copy mode: fb is scanned out continuously by the DSI GDMA, so a
 * repaint only needs the touched span written back to PSRAM — no
 * display_write, no copy, next video frame picks it up (~56 Hz). */
static void dsi_hud(void)
{
	if (!dsi_ok) {
		return;
	}
	hud_draw();
	sys_cache_data_flush_range(fb, WF_TOP * PANEL_W * 3);
}

static void dsi_init(void)
{
	disp = bsp_get_display();
	if (!disp || !device_is_ready(disp)) {
		LOG_WRN("no DSI panel");
		return;
	}
	memset(fb, 0, sizeof(fb));
	hud_draw();
	/* one full-frame write on an aligned PSRAM buffer takes the
	 * zero-copy path: the driver retargets the scanout at fb itself */
	struct display_buffer_descriptor d = {
		.buf_size = sizeof(fb),
		.width = PANEL_W, .height = PANEL_H, .pitch = PANEL_W,
	};
	display_write(disp, 0, 0, &d, fb);
	display_blanking_off(disp);
	dsi_ok = true;
	LOG_INF("DSI panel up (zero-copy fb)");
}

/* paint one spectrum row at wf_row; bins are dB-like codes */
static void dsi_spec_row(const uint8_t *bins, uint32_t nfft)
{
	uint8_t *row = fb + wf_row * PANEL_W * 3;

	/* gutter: clear unless inside the live label span */
	if (wf_row < label_end_row - 14 || wf_row >= label_end_row) {
		memset(row, 0, WF_GUTTER * 3);
	}
	if (st_label_pending) {
		char lab[8];

		snprintf(lab, sizeof(lab), "%u", st_freq_mhz);
		memset(row, 0, WF_GUTTER * 3);
		/* clear the rows the 14px-tall label will occupy */
		for (int i = 1; i < 14 && wf_row + i < PANEL_H; i++) {
			memset(fb + (wf_row + i) * PANEL_W * 3, 0,
			       WF_GUTTER * 3);
		}
		fb_text(3, wf_row + 1, lab, 2, 255, 255, 255);
		label_end_row = wf_row + 14;
		st_label_pending = false;
	}
	/* waterfall: resample nfft bins across WF_W columns */
	for (uint32_t c = 0; c < WF_W; c++) {
		uint8_t r, g, b;
		uint8_t *px = row + (WF_GUTTER + c) * 3;

		db_rgb(bins[c * nfft / WF_W], &r, &g, &b);
		px[0] = r; px[1] = g; px[2] = b;
	}
	sys_cache_data_flush_range(row, PANEL_W * 3);
	if (++wf_row >= PANEL_H) {
		wf_row = WF_TOP;
	}
}

/* ---------- row output: console ASCII and/or DSI ---------- */
static const char ascii[] = " .:-=+*#%@";

static bool sdr_row_cb(const struct sdr_spc1 *h, const uint8_t *bins,
		       void *arg)
{
	uint32_t nfft = 1u << (h->nfft_log2 & 0x0f);
	uint32_t freq = (uint32_t)(uintptr_t)arg;

	if (IS_ENABLED(CONFIG_C6_SDR_ASCII_WF)) {
		uint32_t cols = 96;
		char line[97];

		for (uint32_t c = 0; c < cols; c++) {
			uint8_t g = (uint8_t)((uint32_t)bins[c * nfft / cols] *
				    (sizeof(ascii) - 2) / 255);
			line[c] = ascii[g];
		}
		line[cols] = 0;
		printk("%4u %s f%u g%u d%u\n", freq / 1000000, line,
		       h->ffts, h->gain, h->drops);
	}
	st_rows++;
	st_drops += h->drops;
	fps_rows++;
	fps_tick();
	if (dsi_ok) {
		dsi_spec_row(bins, nfft);
	}
	return true;
}

static void hud_update(void)
{
	snprintf(hud_stat, sizeof(hud_stat),
		 "FPS%u.%u SWP%u ROWS%u DRP%u %uKB F%uMHZ%s",
		 fps_x10 / 10, fps_x10 % 10,
		 st_sweep, st_rows, st_drops, st_staged / 1024,
		 st_freq_mhz, st_fails ? "  *FAIL*" : "");
}

/* ---------- link + demo ---------- */
static int link_check(uint8_t mac[6])
{
	const struct device *dev = esp_ng_dev();
	uint32_t tries = 0;

	if (!dev || !device_is_ready(dev)) {
		/* first boot may lose the race against C6 init — retry
		 * the EN-cycle reinit a few times */
		while (tries++ < 4) {
			if (esp_ng_slave_reinit() == 0 &&
			    esp_ng_wait_slave_init(K_SECONDS(20)) == 0 &&
			    esp_ng_dev() && device_is_ready(esp_ng_dev())) {
				dev = esp_ng_dev();
				break;
			}
			k_sleep(K_SECONDS(2));
		}
		if (!dev || !device_is_ready(dev)) {
			printk("link: hosted SDIO down\n");
			return -ENODEV;
		}
	}
	if (esp_ng_slave_mac(mac)) {
		printk("link: MAC RPC failed\n");
		return -EIO;
	}
	printk("link: hosted SDIO up, C6 STA MAC %02x:%02x:%02x:%02x:%02x:%02x\n",
	       mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
	return 0;
}

int main(void)
{
	uint8_t mac[6];
	uint32_t major, minor, patch;
	uint32_t f_lo = CONFIG_C6_SDR_SWEEP_LO_MHZ;
	uint32_t f_hi = CONFIG_C6_SDR_SWEEP_HI_MHZ;
	uint32_t f_step = CONFIG_C6_SDR_SWEEP_STEP_MHZ;
	uint32_t dur = CONFIG_C6_SDR_SPEC_MS;
	uint32_t n_runs = CONFIG_C6_SDR_RUNS;
	uint32_t gain = CONFIG_C6_SDR_GAIN;
	int fail = 0;

	bsp_init();
	printk("\n=== c6_sdr: P4 -> C6 raw-I/Q + spectrum demo ===\n");
	sd_mount_try();

	if (link_check(mac)) {
		fail++;
		goto done;
	}
	if (esp_ng_slave_fw_version(&major, &minor, &patch) == 0) {
		printk("link: C6 slave fw v%u.%u.%u\n", major, minor, patch);
		snprintf(hud_link, sizeof(hud_link),
			 "C6 %02X:%02X:%02X:%02X:%02X:%02X  FW%u.%u.%u  %u-%uMHZ",
			 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
			 major, minor, patch, f_lo, f_hi);
		if (!(major == 1 && minor == 4 && patch >= 90)) {
			printk("link: warn — fw is not c6_sdr; SDR RPCs will "
			       "not be answered (restore/flash c6-sdr.bin)\n");
		}
	} else {
		printk("link: fw version unsupported (stock slave?)\n");
		snprintf(hud_link, sizeof(hud_link),
			 "C6 %02X:%02X:%02X:%02X:%02X:%02X  FW?  %u-%uMHZ",
			 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
			 f_lo, f_hi);
	}
	hud_update();

#ifdef CONFIG_C6_SDR_DSI
	dsi_init();
#endif

	/* ---- single-shot I/Q burst (once, feeds the HUD + SD dump) ---- */
	if (IS_ENABLED(CONFIG_C6_SDR_IQ_BURST)) {
		uint32_t len = 0;
		struct esp_ng_sdr_run_res res;
		int ret = sdr_iq_burst_pull(CONFIG_C6_SDR_IQ_FREQ_MHZ * 1000000u,
					    16380, 16 /* iq8 */, gain,
					    ESP_NG_SDR_DCAP_AUTO,
					    cap_buf, CAP_MAX, &len, &res);
		if (ret) {
			printk("iq burst: rpc err %d\n", ret);
			fail++;
			st_fails = fail;
		} else if (!len || res.status) {
			printk("iq burst: empty capture (status %u)\n",
			       res.status);
			fail++;
			st_fails = fail;
		} else {
			int16_t i_min = 127, i_max = -128, q_min = 127,
				q_max = -128;
			for (uint32_t j = 0; j + 1 < len; j += 2) {
				int8_t I = (int8_t)cap_buf[j];
				int8_t Q = (int8_t)cap_buf[j + 1];
				if (I < i_min) { i_min = I; }
				if (I > i_max) { i_max = I; }
				if (Q < q_min) { q_min = Q; }
				if (Q > q_max) { q_max = Q; }
			}
			printk("iq burst: %u B (%u pairs iq8) in %u us "
			       "I[%d..%d] Q[%d..%d]\n",
			       len, len / 2, res.elapsed_us,
			       i_min, i_max, q_min, q_max);
			snprintf(hud_iq, sizeof(hud_iq),
				 "IQ%uKB I[%d..%d] Q[%d..%d]",
				 len / 1024, i_min, i_max, q_min, q_max);
			if (dsi_ok) {
				dsi_hud();
			}
			if (IS_ENABLED(CONFIG_C6_SDR_SD_DUMP) && sd_ok && len) {
				sd_write("/SD:/c6_sdr_iq.bin", cap_buf, len);
			}
		}
	}

	/* ---- spectrum sweep: one bounded SPEC run per step ---- */
	for (uint32_t run = 0; n_runs == 0 || run < n_runs; run++) {
		uint32_t total = 0;
		struct esp_ng_sdr_run_res res;
		int ret;

		st_sweep = run + 1;
		for (uint32_t f = f_lo; f <= f_hi; f += f_step) {
			uint32_t len = 0;

			st_freq_mhz = f;
			st_label_pending = true;
			ret = sdr_spec_pull(f * 1000000u, 256,
					    CONFIG_C6_SDR_STRIDE,
					    CONFIG_C6_SDR_UNITS_PER_FRAME,
					    IS_ENABLED(CONFIG_C6_SDR_MAX_HOLD),
					    gain, CONFIG_C6_SDR_DCAP,
					    dur, cap_buf, CAP_MAX, &len, &res);
			if (ret) {
				printk("spec %u MHz: rpc err %d\n", f, ret);
				fail++;
				st_fails = fail;
				continue;
			}
			total += len;
			int frames = sdr_walk_spc1(cap_buf, len, sdr_row_cb,
						   (void *)(uintptr_t)(f *
								       1000000u));
			if (res.status && !frames) {
				printk("spec %u MHz: ring status %u/%u\n",
				       f, res.status, res.detail);
				fail++;
				st_fails = fail;
				continue;
			}
			if (res.status) {
				/* sink overflowed mid-run — partial frames
				 * are still valid spectrum; drops counted */
				printk("spec %u MHz: %u/%u frames, ring "
				       "st %u\n", f, frames, res.detail,
				       res.status);
			}
			if (!frames) {
				printk("spec %u MHz: %u B, no SPC1 frames\n",
				       f, len);
				fail++;
				st_fails = fail;
				continue;
			}
			st_staged += len;
			hud_update();
			if (dsi_ok) {
				dsi_hud();
			}
			if (IS_ENABLED(CONFIG_C6_SDR_SD_DUMP) && sd_ok && len) {
				sd_append("/SD:/c6_sdr_spec.bin", cap_buf,
					  len);
			}
		}
		printk("sweep %u done: %u B staged total\n", run + 1, total);
	}

done:
	st_fails = fail;
	hud_update();
	if (dsi_ok) {
		dsi_hud();
	}
	printk("--- c6_sdr demo %s (%d fail) ---\n",
	       fail ? "FAIL" : "PASS", fail);
	return 0;
}
