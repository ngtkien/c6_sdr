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
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/input/input.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/fs/fs.h>
#include <ff.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include <bsp/esp32p4_bsp.h>

#include <math.h>
#include "esp_hosted_ng.h"
#include "sdr.h"
#include "audio.h"

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
static bool sd_fmt_tried;

static void sd_mount_try(void)
{
	mp.fs_data = &fat_fs;
	if (disk_access_ioctl("SD", DISK_IOCTL_CTRL_INIT, NULL) == 0 &&
	    fs_mount(&mp) == 0) {
		sd_ok = true;
	}
}

/* card-side writes fail hard on some cards — reformat once, then give
 * up after a few */
static void sd_rc(int rc)
{
	if (rc >= 0) {
		sd_fail_streak = 0;
		return;
	}
	if (!sd_fmt_tried) {
		/* corrupt FAT can mount fine but fail every write —
		 * reformat once and remount before giving up */
		sd_fmt_tried = true;
		printk("sd: write err %d — mkfs FAT + remount\n", rc);
		fs_unmount(&mp);
		mp.fs_data = &fat_fs;
		if (fs_mkfs(FS_FATFS, (uintptr_t)"SD:", NULL, 0) == 0 &&
		    fs_mount(&mp) == 0) {
			printk("sd: reformatted — writes re-enabled\n");
			sd_fail_streak = 0;
			return;
		}
		printk("sd: mkfs/remount failed\n");
	}
	if (++sd_fail_streak >= 4) {
		sd_ok = false;
		printk("sd: writes failing (%d), disabling SD dump\n", rc);
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

static volatile bool cap_req;
static volatile bool cap_busy;   /* sweep loop idles while a dump runs */

/* ---------- DSI waterfall + HUD ---------- */
#define PANEL_W   1024
#define PANEL_H   600
#define WF_TOP    64   /* rows above this are the HUD */
#define WF_SCROLL 4    /* rows to shift when the waterfall wraps */
#define WF_GUTTER 56   /* left column reserved for freq labels */
#define WF_W      (PANEL_W - WF_GUTTER)
/* on-screen view button in the HUD (GT911 touch, see touch_cb) */
#define TBTN_X 930
#define TBTN_Y 34
#define TBTN_W 90
#define TBTN_H 26
#define HBTN_X 800   /* HOLD: freeze/pause the chart updates */
#define CBTN_X 864   /* CLR:  clear max-hold/pano + marker  */
#define PBTN_X 736   /* CAP:  screenshot to SD + UART dump  */
#define FBTN_X 940   /* FRQ:  IQ page freq-lock button (drawn in page) */
#define FBTN_Y 552
#define FBTN_W 80
#define FBTN_H 26
#define SBTN_Y 34
#define SBTN_W 58
#define SBTN_H 26

/* display modes — cycled by the BOOT button (sw0); each owns the full
 * area below the HUD — no split views on a 600 px panel */
enum dsi_view { VIEW_WF = 0, VIEW_SPEC, VIEW_PANO, VIEW_IQ, VIEW_N };

static uint8_t fb[PANEL_W * PANEL_H * 3] __aligned(64)
	__attribute__((section(".ext_ram.bss")));
static const struct device *disp;
static int wf_row = WF_TOP;
static int wf_top = WF_TOP; /* view-dependent waterfall start row */
static bool dsi_ok;
static int dsi_view;

/* live-trace / panorama state */
static int tr_top, tr_hgt;
static bool tr_pano;
static bool disp_hold;      /* HOLD button: freeze chart updates */
static uint8_t tr_h[WF_W];
static uint8_t tr_lv[WF_W]; /* live-dot height per column */
static int tr_flush;        /* flush-throttle counter */
static int marker_x = -1;   /* tap-to-tune marker column, -1 off */
static uint32_t tr_freq;       /* block the SPLIT trace belongs to */
static uint32_t sw_lo_mhz, sw_hi_mhz, sw_step_mhz;

/* HUD live stats, refreshed once per spec call */
static char hud_link[96], hud_stat[96], hud_iq[96];
static uint32_t st_rows, st_drops, st_staged, st_sweep, st_fails;
static uint32_t link_recovers;
static uint32_t st_freq_mhz;
static bool st_label_pending;
static uint32_t paint_freq_mhz; /* display-thread's row freq for labels */
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

static const char *view_name(void)
{
	switch (dsi_view) {
	case VIEW_SPEC:  return "SPEC";
	case VIEW_PANO:  return "PANO";
	case VIEW_IQ:    return "IQ";
	default:         return "WFL";
	}
}

static const char *view_title(void)
{
	switch (dsi_view) {
	case VIEW_SPEC:  return "PSD MAX-HOLD";
	case VIEW_PANO:  return "SWEPT SURVEY";
	case VIEW_IQ:    return "IQ SCOPE";
	default:         return "SPECTROGRAM";
	}
}

static void view_color(uint8_t *r, uint8_t *g, uint8_t *b)
{
	switch (dsi_view) {
	case VIEW_SPEC:  *r = 120; *g = 220; *b = 255; break;
	case VIEW_PANO:  *r = 255; *g = 170; *b = 60;  break;
	case VIEW_IQ:    *r = 255; *g = 110; *b = 255; break;
	default:         *r = 80;  *g = 255; *b = 160; break;
	}
}

/* dB-like bin code -> RGB888 heat ramp (blue->cyan->green->amber->red) */
/* ice ramp: deep blue -> blue -> cyan -> white (strong = bright) */
static void db_rgb(uint8_t v, uint8_t *r, uint8_t *g, uint8_t *b)
{
	uint32_t x = v; /* 0..255 */

	if (x < 64) {         /* near-black -> deep blue */
		*r = 0; *g = x / 2; *b = 64 + x;
	} else if (x < 128) { /* blue -> cyan */
		*r = 0; *g = 32 + (x - 64); *b = 200;
	} else if (x < 192) { /* cyan -> pale cyan */
		*r = (x - 128); *g = 96 + (x - 128) * 2; *b = 230;
	} else {              /* pale -> white */
		*r = 96 + (x - 192) * 2; *g = 230; *b = 255;
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
	/* chart name, right-aligned in the gap before the VIEW> button */
	{
		const char *n = view_title();
		uint8_t r, g, b;

		view_color(&r, &g, &b);
		fb_text(920 - 12 * strlen(n), 20, n, 2, r, g, b);
	}
	/* on-screen view button (GT911 touch, see touch_cb) */
	fb_fill(TBTN_X, TBTN_Y, TBTN_W, TBTN_H, 20, 40, 90);
	fb_fill(TBTN_X, TBTN_Y, TBTN_W, 1, 90, 160, 255);
	fb_fill(TBTN_X, TBTN_Y + TBTN_H - 1, TBTN_W, 1, 90, 160, 255);
	fb_fill(TBTN_X, TBTN_Y, 1, TBTN_H, 90, 160, 255);
	fb_fill(TBTN_X + TBTN_W - 1, TBTN_Y, 1, TBTN_H, 90, 160, 255);
	fb_text(TBTN_X + 8, TBTN_Y + 6, "VIEW>", 2, 160, 220, 255);
	/* HOLD + CLR buttons */
	fb_fill(HBTN_X, SBTN_Y, SBTN_W, SBTN_H, disp_hold ? 70 : 20,
		disp_hold ? 30 : 40, 90);
	fb_fill(HBTN_X, SBTN_Y, SBTN_W, 1, 90, 160, 255);
	fb_fill(HBTN_X, SBTN_Y + SBTN_H - 1, SBTN_W, 1, 90, 160, 255);
	fb_fill(HBTN_X, SBTN_Y, 1, SBTN_H, 90, 160, 255);
	fb_fill(HBTN_X + SBTN_W - 1, SBTN_Y, 1, SBTN_H, 90, 160, 255);
	fb_text(HBTN_X + 5, SBTN_Y + 6, "HOLD", 2,
		disp_hold ? 255 : 160, disp_hold ? 120 : 220, 255);
	fb_fill(CBTN_X, SBTN_Y, SBTN_W, SBTN_H, 20, 40, 90);
	fb_fill(CBTN_X, SBTN_Y, SBTN_W, 1, 90, 160, 255);
	fb_fill(CBTN_X, SBTN_Y + SBTN_H - 1, SBTN_W, 1, 90, 160, 255);
	fb_fill(CBTN_X, SBTN_Y, 1, SBTN_H, 90, 160, 255);
	fb_fill(CBTN_X + SBTN_W - 1, SBTN_Y, 1, SBTN_H, 90, 160, 255);
	fb_text(CBTN_X + 10, SBTN_Y + 6, "CLR", 2, 160, 220, 255);
	/* CAP button */
	fb_fill(PBTN_X, SBTN_Y, SBTN_W, SBTN_H, 20, 40, 90);
	fb_fill(PBTN_X, SBTN_Y, SBTN_W, 1, 90, 160, 255);
	fb_fill(PBTN_X, SBTN_Y + SBTN_H - 1, SBTN_W, 1, 90, 160, 255);
	fb_fill(PBTN_X, SBTN_Y, 1, SBTN_H, 90, 160, 255);
	fb_fill(PBTN_X + SBTN_W - 1, SBTN_Y, 1, SBTN_H, 90, 160, 255);
	fb_text(PBTN_X + 12, SBTN_Y + 6, "CAP", 2, 160, 220, 255);
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

/* paint (grow) the max-hold trace bar for column c to height hh */
static void tr_grow(int c, int hh)
{
	uint8_t r, g, b;
	int base = tr_top + tr_hgt - 1;
	uint8_t *px = fb + ((base - tr_h[c]) * PANEL_W + WF_GUTTER + c) * 3;

	db_rgb((uint8_t)(hh * 255 / tr_hgt), &r, &g, &b);
	for (int y = tr_h[c]; y < hh; y++) {
		px[0] = r; px[1] = g; px[2] = b;
		px -= PANEL_W * 3;
	}
	tr_h[c] = hh;
}

/* live-value marker: erase the old dot (restoring whatever was under
 * it — max-hold color inside the bar, background above) and draw the
 * new one. Bright white so the live level reads against the max bars. */
static void tr_live_dot(int c, int hh)
{
	int base = tr_top + tr_hgt - 1;
	uint8_t *px;
	uint8_t r, g, b;

	if (tr_lv[c] != hh && tr_lv[c] > 0) {
		px = fb + ((base - tr_lv[c] + 1) * PANEL_W + WF_GUTTER + c) * 3;
		if (tr_lv[c] <= tr_h[c]) {
			db_rgb((uint8_t)(tr_lv[c] * 255 / tr_hgt), &r, &g, &b);
		} else {
			r = g = 0; b = 8;
		}
		px[0] = r; px[1] = g; px[2] = b;
	}
	if (hh > 0) {
		px = fb + ((base - hh + 1) * PANEL_W + WF_GUTTER + c) * 3;
		px[0] = 255; px[1] = 255; px[2] = 255;
	}
	tr_lv[c] = hh;
}

/* slow decay for the panorama: erase one pixel from the top of each
 * accumulated bar so old peaks fade instead of pinning at max */
static void tr_decay(void)
{
	int base = tr_top + tr_hgt - 1;

	for (int c = 0; c < WF_W; c++) {
		uint8_t *px;

		if (!tr_h[c]) {
			continue;
		}
		px = fb + ((base - tr_h[c] + 1) * PANEL_W + WF_GUTTER + c) * 3;
		px[0] = 0; px[1] = 0; px[2] = 8;
		tr_h[c]--;
	}
}

/* eSpDR-style spectrum trace.
 * SPLIT: max-hold bar chart of the current block's nfft bins.
 * PANO:  each bin lands on the sweep-frequency axis (80 MHz span per
 *        block) — a full-band panorama that accumulates over the sweep. */
static void dsi_trace(const uint8_t *bins, uint32_t nfft, uint32_t freq_hz)
{
	if (!tr_hgt) {
		return;
	}
	if (!tr_pano) {
		/* per-block trace: reset when the tuned freq changes */
		if (freq_hz != tr_freq) {
			tr_freq = freq_hz;
			memset(tr_h, 0, sizeof(tr_h));
			memset(tr_lv, 0, sizeof(tr_lv));
			fb_fill(WF_GUTTER, tr_top, WF_W, tr_hgt, 0, 0, 8);
		}
		for (int c = 0; c < WF_W; c++) {
			uint8_t v = bins[c * nfft / WF_W];
			int hh = v * (tr_hgt - 1) / 255;

			if (hh > tr_h[c]) {
				tr_grow(c, hh);
			}
			tr_live_dot(c, hh);
		}
	} else {
		/* panorama: bin frequency = tune + (b - nfft/2) * 80MHz/nfft */
		uint32_t lo_hz = (sw_lo_mhz - 40) * 1000000u;
		uint32_t span_hz = (sw_hi_mhz - sw_lo_mhz + 80) * 1000000u;
		int64_t bin_hz = 80000000ll / nfft;
		static int dec;

		for (uint32_t b = 0; b < nfft; b++) {
			int64_t fbq = (int64_t)freq_hz +
				((int32_t)b - (int32_t)nfft / 2) * bin_hz;
			int px = (int)((fbq - lo_hz) * WF_W / span_hz);
			int hh;

			if (px < 0 || px >= WF_W) {
				continue;
			}
			hh = bins[b] * (tr_hgt - 1) / 255;
			if (hh > tr_h[px]) {
				tr_grow(px, hh);
			}
			tr_live_dot(px, hh);
		}
		if (++dec >= 20) { /* ~1px every 170ms at 120f/s */
			dec = 0;
			tr_decay();
		}
	}
	/* the region flush is ~1.6MB of cache writeback — at ~176 f/s that
	 * dominates the frame cost. The scanout only refreshes at ~56 Hz,
	 * so flushing every 4th frame loses nothing on screen. */
	if (++tr_flush >= 4) {
		tr_flush = 0;
		sys_cache_data_flush_range(fb + tr_top * PANEL_W * 3,
					   tr_hgt * PANEL_W * 3);
	}
}

/* ---------- IQ page: phosphor constellation + I/Q waveforms ----------
 * Burst iq8 pairs land in iq_pts; the phosphor map accumulates each
 * burst with 7/8 decay so signal structure persists like a CRT.
 * Auto-scale tracks a smoothed peak; DC offset is removed for display
 * and reported in the stats line. */
#define IQ_PTS_CAP 16384           /* iq8 pairs kept per burst        */
#define SC_W 512
#define SC_H 512
#define SC_X 24
#define SC_Y (WF_TOP + 12)
#define WV_X 560                   /* I/Q waveform strips             */
#define WV_W 440
#define WI_Y (SC_Y)
#define WI_H 140
#define WQ_Y (SC_Y + 150)
#define WQ_H 140
#define WP_Y (SC_Y + 300)          /* inst-frequency strip            */
#define WP_H 140
static int8_t iq_pts[2 * IQ_PTS_CAP];
static uint32_t iq_npairs;
static uint8_t iq_den[SC_W * SC_H] __attribute__((section(".ext_ram.bss")));
static volatile uint32_t iq_seq, iq_seen;
static uint32_t iq_peak = 128;     /* smoothed peak |LSB| for scale   */
static int iq_mean_i, iq_mean_q;
static bool iq_lock;               /* FRQ button: park on one freq    */
static uint32_t iq_lock_hz;

static void iq_circle(int cx, int cy, int r, uint8_t cr, uint8_t cg,
		      uint8_t cb)
{
	/* midpoint circle, 8 symmetric dots per step */
	for (int x = 0, y = r, d = 1 - r; x <= y;) {
		static const int8_t sx[8] = {1, -1, 1, -1, 1, -1, 1, -1};
		static const int8_t sy[8] = {1, 1, -1, -1, 1, 1, -1, -1};
		static const bool sw[8] = {0, 0, 0, 0, 1, 1, 1, 1};

		for (int i = 0; i < 8; i++) {
			int px = cx + (sw[i] ? y : x) * sx[i];
			int py = cy + (sw[i] ? x : y) * sy[i];

			if ((unsigned)(px - SC_X) < SC_W &&
			    (unsigned)(py - SC_Y) < SC_H) {
				uint8_t *p = fb + (py * PANEL_W + px) * 3;

				p[0] = cr; p[1] = cg; p[2] = cb;
			}
		}
		if (d < 0) {
			d += 2 * x + 3;
		} else {
			d += 2 * (x - y) + 5;
			y--;
		}
		x++;
	}
}

/* clipped dim segment between two scope points (trajectory overlay) */
static void iq_seg(int x0, int y0, int x1, int y1,
		   uint8_t cr, uint8_t cg, uint8_t cb)
{
	int dx = x1 - x0, dy = y1 - y0;
	int steps = MAX(dx < 0 ? -dx : dx, dy < 0 ? -dy : dy);

	if (steps > 1024) {
		steps = 1024;
	}
	for (int i = 0; i <= steps; i++) {
		int x = x0 + dx * i / (steps ? steps : 1);
		int y = y0 + dy * i / (steps ? steps : 1);

		if ((unsigned)(x - SC_X) < SC_W &&
		    (unsigned)(y - SC_Y) < SC_H) {
			uint8_t *p = fb + (y * PANEL_W + x) * 3;

			p[0] = cr; p[1] = cg; p[2] = cb;
		}
	}
}

/* instantaneous-frequency strip: diff-phase per pair = FM demod trace */
static void iq_dev(int x0, int y0, int w, int h)
{
	int mid = y0 + h / 2;
	uint32_t n = iq_npairs > 1 ? iq_npairs - 1 : 1;

	fb_fill(x0, y0, w, h, 4, 4, 12);
	fb_fill(x0, mid, w, 1, 50, 50, 70);
	fb_text(x0 + 4, y0 + 4, "dF", 1, 60, 90, 110);
	for (int c = 0; c < w; c++) {
		uint32_t k = (uint32_t)c * n / w + 1;
		int dI = (int)iq_pts[2 * k] * iq_pts[2 * k - 2] +
			 (int)iq_pts[2 * k + 1] * iq_pts[2 * k - 1];
		int dQ = (int)iq_pts[2 * k + 1] * iq_pts[2 * k - 2] -
			 (int)iq_pts[2 * k] * iq_pts[2 * k - 1];
		float ph = atan2f((float)dQ, (float)dI);
		int y = mid - (int)(ph * (h / 2 - 8) / 3.14159f);
		uint8_t *p;

		if (y < y0 + 2) {
			y = y0 + 2;
		} else if (y > y0 + h - 3) {
			y = y0 + h - 3;
		}
		p = fb + (y * PANEL_W + x0 + c) * 3;
		p[0] = 120; p[1] = 160; p[2] = 255;
	}
}

/* one waveform strip: first np pairs decimated to WV_W columns */
static void iq_wave(int x0, int y0, int w, int h, int which, int peak,
		   uint8_t cr, uint8_t cg, uint8_t cb, const char *tag)
{
	int mid = y0 + h / 2;
	uint32_t n = iq_npairs ? iq_npairs : 1;

	fb_fill(x0, y0, w, h, 4, 4, 12);
	fb_fill(x0, mid, w, 1, 50, 50, 70);
	fb_text(x0 + 4, y0 + 4, tag, 1, cr / 2, cg / 2, cb / 2);
	for (int c = 0; c < w; c++) {
		uint32_t k = (uint32_t)c * n / w;
		int v = iq_pts[2 * k + which];
		int y = mid - v * (h / 2 - 8) / (peak ? (int)peak : 1);
		uint8_t *p;

		if (y < y0 + 2) {
			y = y0 + 2;
		} else if (y > y0 + h - 3) {
			y = y0 + h - 3;
		}
		p = fb + (y * PANEL_W + x0 + c) * 3;
		p[0] = cr; p[1] = cg; p[2] = cb;
	}
}

static void dsi_scope(void)
{
	int cx = SC_X + SC_W / 2, cy = SC_Y + SC_H / 2;
	uint32_t n = iq_npairs;
	int pk = 8;
	int64_t isum = 0, qsum = 0;
	uint8_t *p;
	char lab[40];

	if (iq_seen == iq_seq || !n) {
		return;
	}
	iq_seen = iq_seq;

	/* burst stats → DC + smoothed autoscale peak + cloud RMS */
	int64_t sumsq = 0;

	for (uint32_t i = 0; i < n; i++) {
		int vi = iq_pts[i * 2], vq = iq_pts[i * 2 + 1];

		isum += vi; qsum += vq;
		sumsq += (int64_t)vi * vi + (int64_t)vq * vq;
		if (vi < 0) {
			vi = -vi;
		}
		if (vq < 0) {
			vq = -vq;
		}
		if (vi > pk) {
			pk = vi;
		}
		if (vq > pk) {
			pk = vq;
		}
	}
	iq_mean_i = (int)(isum / n);
	iq_mean_q = (int)(qsum / n);
	iq_peak = MAX((uint32_t)pk, (iq_peak * 15) >> 4);
	int scale = (SC_W / 2 - 24) * 256 / (int)iq_peak; /* Q8 px/LSB */
	/* cloud RMS around the DC point — tight ring vs blob measure */
	uint32_t iq_rms = (uint32_t)sqrtf(
		(float)(sumsq / (int64_t)n) -
		(float)(iq_mean_i * iq_mean_i + iq_mean_q * iq_mean_q));

	/* phosphor decay, then accumulate the burst */
	for (int i = 0; i < SC_W * SC_H; i++) {
		iq_den[i] = (uint8_t)((iq_den[i] * 7) >> 3);
	}
	for (uint32_t i = 0; i < n; i++) {
		int x = cx + (((iq_pts[i * 2] - iq_mean_i) * scale) >> 8);
		int y = cy - (((iq_pts[i * 2 + 1] - iq_mean_q) * scale) >> 8);

		if ((unsigned)(x - SC_X) < SC_W && (unsigned)(y - SC_Y) < SC_H) {
			uint8_t *d = &iq_den[(y - SC_Y) * SC_W + (x - SC_X)];

			*d = *d > 223 ? 255 : *d + 32;
		}
	}

	/* render phosphor → fb through the heat ramp */
	for (int yy = 0; yy < SC_H; yy++) {
		const uint8_t *d = &iq_den[yy * SC_W];

		p = fb + ((SC_Y + yy) * PANEL_W + SC_X) * 3;
		for (int xx = 0; xx < SC_W; xx++) {
			uint8_t v = d[xx];

			if (v) {
				db_rgb(v, &p[0], &p[1], &p[2]);
			} else {
				p[0] = 0; p[1] = 0; p[2] = 8;
			}
			p += 3;
		}
	}
	/* latest-burst trajectory — dim cyan polyline over the phosphor */
	{
		uint32_t step = n > 4096 ? n / 4096 : 1;
		int px_prev = -1, py_prev = 0;

		for (uint32_t i = 0; i < n; i += step) {
			int x = cx + (((iq_pts[i * 2] - iq_mean_i) * scale) >> 8);
			int y = cy -
				(((iq_pts[i * 2 + 1] - iq_mean_q) * scale) >> 8);

			if (px_prev >= 0) {
				iq_seg(px_prev, py_prev, x, y, 30, 130, 160);
			}
			px_prev = x;
			py_prev = y;
		}
	}
	/* reticle: cross + three radius guides */
	fb_fill(cx - SC_W / 2 + 4, cy, SC_W - 8, 1, 45, 45, 65);
	fb_fill(cx, cy - SC_H / 2 + 4, 1, SC_H - 8, 45, 45, 65);
	iq_circle(cx, cy, SC_W / 6, 40, 40, 55);
	iq_circle(cx, cy, SC_W / 3, 40, 40, 55);
	iq_circle(cx, cy, SC_W / 2 - 8, 55, 55, 75);
	/* DC position marker (LO leakage) — small magenta cross */
	{
		int mx = cx - (iq_mean_i * scale >> 8);
		int my = cy + (iq_mean_q * scale >> 8);

		if ((unsigned)(mx - SC_X) < SC_W &&
		    (unsigned)(my - SC_Y) < SC_H) {
			fb_fill(mx - 4, my, 9, 1, 255, 60, 200);
			fb_fill(mx, my - 4, 1, 9, 255, 60, 200);
		}
	}
	/* scale + stats */
	snprintf(lab, sizeof(lab), "+-%u LSB", iq_peak);
	fb_text(SC_X + 6, SC_Y + SC_H - 14, lab, 1, 140, 140, 180);
	snprintf(lab, sizeof(lab), "I u%d  Q u%d  RMS%u", iq_mean_i,
		 iq_mean_q, iq_rms);
	fb_text(SC_X + 6, SC_Y + 6, lab, 1, 140, 140, 180);

	/* right column: I(t), Q(t), inst-freq strips + stats block */
	iq_wave(WV_X, WI_Y, WV_W, WI_H, 0, iq_peak, 80, 220, 140, "I(t)");
	iq_wave(WV_X, WQ_Y, WV_W, WQ_H, 1, iq_peak, 255, 180, 60, "Q(t)");
	iq_dev(WV_X, WP_Y, WV_W, WP_H);
	snprintf(lab, sizeof(lab), "F %lu.%lu MHz%s  %lu pairs",
		 (unsigned long)((iq_lock ? iq_lock_hz
					   : st_freq_mhz * 1000000u) / 1000000),
		 (unsigned long)((iq_lock ? iq_lock_hz
					   : st_freq_mhz * 1000000u) / 100000 % 10),
		 iq_lock ? " LOCK" : "",
		 (unsigned long)iq_npairs);
	fb_text(WV_X + 4, WP_Y + WP_H + 10, lab, 1, 120, 120, 160);

	/* FRQ lock button (IQ page only) */
	fb_fill(FBTN_X, FBTN_Y, FBTN_W, FBTN_H,
		iq_lock ? 90 : 20, iq_lock ? 50 : 40, 90);
	fb_fill(FBTN_X, FBTN_Y, FBTN_W, 1, 90, 160, 255);
	fb_fill(FBTN_X, FBTN_Y + FBTN_H - 1, FBTN_W, 1, 90, 160, 255);
	fb_fill(FBTN_X, FBTN_Y, 1, FBTN_H, 90, 160, 255);
	fb_fill(FBTN_X + FBTN_W - 1, FBTN_Y, 1, FBTN_H, 90, 160, 255);
	fb_text(FBTN_X + 8, FBTN_Y + 6, iq_lock ? "FRQ*" : "FRQ", 2,
		iq_lock ? 255 : 160, iq_lock ? 200 : 220, 255);

	sys_cache_data_flush_range(fb + WF_TOP * PANEL_W * 3,
				   (PANEL_H - WF_TOP) * PANEL_W * 3);
}

/* FM discriminator on the iq8 burst -> stretched PCM blip (~40 ms).
 * diff-phase per pair = instantaneous frequency; resampled 4096->2048
 * samples ≈ 51us of signal played back ~400x slower. Honest blip. */
static void iq_blip(const int8_t *xy, uint32_t n)
{
	static int16_t pcm[2048];

	if (!audio_ready() || n < 64) {
		return;
	}
	for (uint32_t i = 0; i < 2048; i++) {
		uint32_t k = 1 + i * (n - 2) / 2048;
		int dI = (int)xy[k * 2] * xy[k * 2 - 2] +
			 (int)xy[k * 2 + 1] * xy[k * 2 - 1];
		int dQ = (int)xy[k * 2 + 1] * xy[k * 2 - 2] -
			 (int)xy[k * 2] * xy[k * 2 - 1];

		pcm[i] = (int16_t)(atan2f((float)dQ, (float)dI) * 3000);
	}
	audio_play(pcm, 2048);
}

/* ---------- screen capture: BMP to SD + RGB565 dump over UART ----------
 * Triggered by the CAP button or 'c' on the console. UART frame:
 * "@@SCR w h len\n" + raw RGB565 payload + crc16 — a host script can
 * reassemble a PNG without touching the SD card. */
static void put32le(uint8_t *p, uint32_t v)
{
	p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24;
}

static void cap_uart_dump(const struct device *con)
{
	enum { DW = 512, DH = 300 };
	static uint8_t pix[DW * 2];
	uint16_t crc = 0xffff;

	printk("@@SCR %u %u %lu\n", (uint32_t)DW, (uint32_t)DH,
	       (unsigned long)(DW * DH * 2));
	for (int y = 0; y < DH; y++) {
		const uint8_t *row = fb + (y * 2) * PANEL_W * 3;

		for (int x = 0; x < DW; x++) {
			const uint8_t *s = row + x * 6;
			uint16_t v = ((s[0] & 0xf8) << 8) |
				     ((s[1] & 0xfc) << 3) | (s[2] >> 3);

			pix[x * 2] = v & 0xff;
			pix[x * 2 + 1] = v >> 8;
		}
		for (uint32_t i = 0; i < DW * 2; i++) {
			crc ^= (uint16_t)pix[i] << 8;
			for (int b = 0; b < 8; b++) {
				crc = crc & 0x8000 ?
					(crc << 1) ^ 0x1021 : crc << 1;
			}
			uart_poll_out(con, pix[i]);
		}
	}
	uart_poll_out(con, crc & 0xff);
	uart_poll_out(con, crc >> 8);
	uart_poll_out(con, '\n');
	printk("cap: uart frame done (crc %04x)\n", crc);
}

static void cap_sd_bmp(void)
{
	static uint8_t hdr[54];
	static uint8_t row[PANEL_W * 3];
	struct fs_file_t f;
	char path[32];
	uint32_t imgsz = PANEL_W * PANEL_H * 3;
	int rc = 0;

	if (!sd_ok) {
		return;
	}
	snprintf(path, sizeof(path), "/SD:/scr_%s.bmp", view_name());
	fs_file_t_init(&f);
	if (fs_open(&f, path, FS_O_CREATE | FS_O_WRITE) != 0) {
		return;
	}
	memset(hdr, 0, sizeof(hdr));
	hdr[0] = 'B'; hdr[1] = 'M';
	put32le(hdr + 2, 54 + imgsz);       /* file size           */
	put32le(hdr + 10, 54);              /* pixel data offset   */
	put32le(hdr + 14, 40);              /* info header size    */
	put32le(hdr + 18, PANEL_W);
	put32le(hdr + 22, PANEL_H);
	hdr[26] = 1;                        /* planes              */
	hdr[28] = 24;                       /* bpp                 */
	put32le(hdr + 34, imgsz);
	put32le(hdr + 38, 2835);            /* ~72 DPI             */
	put32le(hdr + 42, 2835);
	fs_write(&f, hdr, 54);
	for (int y = PANEL_H - 1; y >= 0; y--) {
		const uint8_t *s = fb + y * PANEL_W * 3;

		for (int x = 0; x < PANEL_W; x++) {
			row[x * 3] = s[x * 3 + 2];
			row[x * 3 + 1] = s[x * 3 + 1];
			row[x * 3 + 2] = s[x * 3];
		}
		rc = fs_write(&f, row, sizeof(row));
		if (rc < 0) {
			break;
		}
	}
	fs_close(&f);
	sd_rc(rc);
	if (rc >= 0) {
		printk("cap: %s written\n", path);
	}
}

static void cap_dump(void)
{
	const struct device *con =
		DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

	cap_busy = true;
	cap_sd_bmp();
	if (con && device_is_ready(con)) {
		cap_uart_dump(con);
	}
	cap_busy = false;
}

/* freq grid for the panorama: vertical line + label every sweep step */
static void pano_grid(void)
{
	uint32_t lo_hz = (sw_lo_mhz - 40) * 1000000u;
	uint32_t span_hz = (sw_hi_mhz - sw_lo_mhz + 80) * 1000000u;
	int base = tr_top + tr_hgt - 1;

	for (uint32_t f = sw_lo_mhz; f <= sw_hi_mhz; f += sw_step_mhz) {
		int px = (int)(((int64_t)f * 1000000 - lo_hz) * WF_W / span_hz);

		fb_fill(WF_GUTTER + px, tr_top, 1, tr_hgt, 30, 30, 50);
		char lab[8];
		snprintf(lab, sizeof(lab), "%u", f);
		fb_text(WF_GUTTER + px + 2, base - 8, lab, 1,
			100, 100, 140);
	}
}

/* switch display mode: relayout regions, clear, reset state */
static void dsi_view_apply(int v)
{
	dsi_view = v;
	memset(tr_h, 0, sizeof(tr_h));
	memset(tr_lv, 0, sizeof(tr_lv));
	tr_freq = 0;
	label_end_row = -1;
	if (v == VIEW_SPEC) {
		tr_top = WF_TOP;
		tr_hgt = PANEL_H - WF_TOP - 12;
		tr_pano = false;
		wf_top = PANEL_H; /* trace owns the frame, no waterfall */
	} else if (v == VIEW_PANO) {
		tr_top = WF_TOP;
		tr_hgt = PANEL_H - WF_TOP - 12;
		tr_pano = true;
		wf_top = PANEL_H;
	} else if (v == VIEW_IQ) {
		tr_top = tr_hgt = 0;
		wf_top = PANEL_H; /* scope owns the frame */
		memset(iq_den, 0, sizeof(iq_den));
		iq_peak = 128;
	} else {
		tr_top = 0;
		tr_hgt = 0;
		wf_top = WF_TOP;
	}
	fb_fill(0, WF_TOP, PANEL_W, PANEL_H - WF_TOP, 0, 0, 8);
	if (tr_hgt) {
		/* baseline + quarter-height gridlines */
		int base = tr_top + tr_hgt - 1;

		for (int i = 0; i < 4; i++) {
			fb_fill(WF_GUTTER, base - tr_hgt * i / 4, WF_W, 1,
				30, 30, 50);
		}
		/* relative level tags in the free left gutter (bins are
		 * ~log codes 0-255; quarters ≈ 25% of scale each) */
		fb_text(8, base - tr_hgt / 4 - 9, "75%", 1, 90, 90, 120);
		fb_text(8, base - tr_hgt / 2 - 9, "50%", 1, 90, 90, 120);
		fb_text(8, base - tr_hgt * 3 / 4 - 9, "25%", 1, 90, 90, 120);
		if (tr_pano) {
			pano_grid();
		}
	}
	fb_fill(0, wf_top - 1, PANEL_W, 1, 40, 40, 60);
	wf_row = wf_top;
	sys_cache_data_flush_range(fb + WF_TOP * PANEL_W * 3,
				   (PANEL_H - WF_TOP) * PANEL_W * 3);
}

/* touch-button rects inside the HUD (drawn by hud_draw) */
static volatile bool touch_req;
static volatile bool hold_req, clr_req, frq_req;
static volatile int tap_x = -1, tap_y;

/* GT911 touch via Zephyr input: track position, flag presses inside the
 * on-screen button — dsi_view_apply runs on the app thread instead. */
static void touch_cb(struct input_event *evt, void *user_data)
{
	static int32_t tx, ty;

	(void)user_data;
	if (evt->type == INPUT_EV_ABS) {
		if (evt->code == INPUT_ABS_X) {
			tx = evt->value;
		} else if (evt->code == INPUT_ABS_Y) {
			ty = evt->value;
		}
	} else if (evt->type == INPUT_EV_KEY &&
		   evt->code == INPUT_BTN_TOUCH && evt->value) {
		if (tx >= TBTN_X && tx < TBTN_X + TBTN_W &&
		    ty >= TBTN_Y && ty < TBTN_Y + TBTN_H) {
			touch_req = true;
		} else if (tx >= HBTN_X && tx < HBTN_X + SBTN_W &&
			   ty >= SBTN_Y && ty < SBTN_Y + SBTN_H) {
			hold_req = true;
		} else if (tx >= CBTN_X && tx < CBTN_X + SBTN_W &&
			   ty >= SBTN_Y && ty < SBTN_Y + SBTN_H) {
			clr_req = true;
		} else if (tx >= PBTN_X && tx < PBTN_X + SBTN_W &&
			   ty >= SBTN_Y && ty < SBTN_Y + SBTN_H) {
			cap_req = true;
		} else if (dsi_view == VIEW_IQ &&
			   tx >= FBTN_X && tx < FBTN_X + FBTN_W &&
			   ty >= FBTN_Y && ty < FBTN_Y + FBTN_H) {
			frq_req = true;
		} else if (ty >= WF_TOP && tx >= WF_GUTTER &&
			   dsi_view != VIEW_IQ) {
			tap_x = tx - WF_GUTTER;
			tap_y = ty;
		}
	}
}
INPUT_CALLBACK_DEFINE(NULL, touch_cb, NULL);

/* marker: MHz*10 at the tapped column for the active view */
static uint32_t marker_mhz10(void)
{
	if (tr_pano) {
		uint32_t lo = (sw_lo_mhz - 40) * 10u;
		uint32_t span = (sw_hi_mhz - sw_lo_mhz + 80) * 10u;

		return lo + (uint32_t)marker_x * span / WF_W;
	}
	/* SPEC / WFL: columns span 80 MHz around the current tune freq */
	return (uint32_t)((int32_t)st_freq_mhz * 10 +
	       ((int64_t)marker_x - WF_W / 2) * 800 / WF_W);
}

static void marker_draw(void)
{
	int top = tr_hgt ? tr_top : wf_top;
	uint8_t *px;
	char lab[16];

	if (marker_x < 0 || top >= PANEL_H) {
		return;
	}
	px = fb + (top * PANEL_W + WF_GUTTER + marker_x) * 3;
	for (int y = top; y < PANEL_H; y++) {
		px[0] = 200; px[1] = 40; px[2] = 255;
		px += PANEL_W * 3;
	}
	if (tr_hgt) { /* freq label: only where it won't scroll away */
		snprintf(lab, sizeof(lab), "%u.%uM",
			 marker_mhz10() / 10, marker_mhz10() % 10);
		fb_text(WF_GUTTER + marker_x + 3, top + 2, lab, 1,
			255, 160, 255);
	}
}

/* BOOT button edge-detect; call frequently from the frame path */
static const struct gpio_dt_spec boot_btn =
	GPIO_DT_SPEC_GET_OR(DT_ALIAS(sw0), gpios, {0});
static int btn_prev = -1; /* -1 = unread */
static int64_t btn_ms;
static void view_button_poll(void)
{
	int now;
	int64_t t;

	if (touch_req) { /* on-screen VIEW> button */
		touch_req = false;
		dsi_view_apply((dsi_view + 1) % VIEW_N);
		printk("view: %s (touch)\n", view_name());
	}
	if (hold_req) { /* HOLD: freeze chart, HUD + stats keep running */
		hold_req = false;
		disp_hold = !disp_hold;
		printk("hold: %s\n", disp_hold ? "on" : "off");
		dsi_hud();
	}
	if (clr_req) { /* CLR: wipe accumulated trace + marker */
		clr_req = false;
		marker_x = -1;
		dsi_view_apply(dsi_view);
	}
	if (frq_req) { /* IQ page: lock bursts to one freq */
		frq_req = false;
		iq_lock = !iq_lock;
		if (iq_lock) {
			iq_lock_hz = marker_x >= 0 ?
				marker_mhz10() * 100000u :
				st_freq_mhz * 1000000u;
		}
		printk("iq lock: %s %u.%u MHz\n", iq_lock ? "on" : "off",
		       iq_lock_hz / 1000000, (iq_lock_hz / 100000) % 10);
		iq_seq++; /* force a scope repaint for the button state */
	}
	if (tap_x >= 0) { /* tap-to-tune marker on the chart */
		marker_x = tap_x < WF_W ? tap_x : WF_W - 1;
		tap_x = -1;
		marker_draw();
		printk("marker: %u.%u MHz\n",
		       marker_mhz10() / 10, marker_mhz10() % 10);
	}
	if (!boot_btn.port) {
		return;
	}
	now = gpio_pin_get_dt(&boot_btn);
	if (btn_prev < 0) {
		btn_prev = now;
		btn_ms = k_uptime_get();
		return;
	}
	t = k_uptime_get();
	if (!now && btn_prev && t - btn_ms > 400) { /* pressed edge */
		btn_ms = t;
		dsi_view_apply((dsi_view + 1) % VIEW_N);
		printk("view: %s\n", view_name());
	}
	btn_prev = now;
}

/* paint one spectrum row at wf_row; bins are dB-like codes */
static void dsi_spec_row(const uint8_t *bins, uint32_t nfft)
{
	uint8_t *row;

	if (wf_top >= PANEL_H) {
		return; /* PANO: no waterfall region */
	}
	row = fb + wf_row * PANEL_W * 3;

	/* gutter: clear unless inside the live label span */
	if (wf_row < label_end_row - 14 || wf_row >= label_end_row) {
		memset(row, 0, WF_GUTTER * 3);
	}
	if (st_label_pending) {
		char lab[8];

		snprintf(lab, sizeof(lab), "%u", paint_freq_mhz);
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
	if (++wf_row >= PANEL_H) {
		/* bottom reached: scroll the waterfall up a few rows instead
		 * of jumping back to the top — newest rows stay at the
		 * bottom and history slides up continuously */
		uint8_t *base = fb + wf_top * PANEL_W * 3;

		memmove(base, base + WF_SCROLL * PANEL_W * 3,
			(PANEL_H - wf_top - WF_SCROLL) * PANEL_W * 3);
		wf_row = PANEL_H - WF_SCROLL;
		label_end_row -= MIN(label_end_row, WF_SCROLL);
		/* the whole region was rewritten — flush it once */
		sys_cache_data_flush_range(base,
				(PANEL_H - wf_top) * PANEL_W * 3);
	} else {
		sys_cache_data_flush_range(row, PANEL_W * 3);
	}
}

/* ---------- row output: console ASCII and/or DSI ---------- */
static const char ascii[] = " .:-=+*#%@";

/* display worker: the pull path just enqueues rows; painting (db_rgb,
 * memmove scrolls, cache flushes) runs here so a slow frame can't
 * throttle the RPC drain. Queue-full rows are counted, not dropped
 * silently. */
#define DISP_MAX_NFFT 256
struct disp_row {
	uint32_t freq;
	uint32_t nfft;
	uint8_t  drops;
	uint8_t  bins[DISP_MAX_NFFT];
};
K_MSGQ_DEFINE(disp_q, sizeof(struct disp_row), 24, 4);
static uint32_t disp_drops;

static void disp_thread(void *a, void *b, void *c)
{
	struct disp_row r;
	uint32_t lab_freq = 0;

	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	for (;;) {
		if (k_msgq_get(&disp_q, &r, K_MSEC(250))) {
			view_button_poll();
			continue;
		}
		view_button_poll(); /* per-row too — else buttons starve
				     * while the queue never empties */
		if (disp_hold) {
			continue;
		}
		if (r.freq != lab_freq) {
			lab_freq = r.freq;
			paint_freq_mhz = r.freq / 1000000;
			st_label_pending = true;
		}
		dsi_trace(r.bins, r.nfft, r.freq);
		dsi_spec_row(r.bins, r.nfft);
		marker_draw();
	}
}
K_THREAD_DEFINE(disp_th, 4096, disp_thread, NULL, NULL, NULL,
		K_PRIO_COOP(9), 0, 0);

/* scope + blip pump: runs alongside the display thread; the sweep loop
 * drops iq8 bursts into iq_pts and this repaints + plays them */
static void scope_thread(void *a, void *b, void *c)
{
	const struct device *con =
		DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	uint8_t ch;

	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	for (;;) {
		/* console remote: 'v' cycles view, 'c' captures screen */
		if (con && device_is_ready(con)) {
			while (uart_poll_in(con, &ch) == 0) {
				if (ch == 'v' || ch == 'V') {
					touch_req = true;
				} else if (ch == 'c' || ch == 'C') {
					cap_req = true;
				}
			}
		}
		if (cap_req) {
			cap_req = false;
			cap_dump();
		}
		if (dsi_ok && dsi_view == VIEW_IQ && !disp_hold &&
		    iq_seen != iq_seq) {
			dsi_scope();
		}
		k_msleep(20);
	}
}
K_THREAD_DEFINE(scope_th, 2048, scope_thread, NULL, NULL, NULL,
		K_PRIO_COOP(10), 0, 0);

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
	if (dsi_ok && nfft <= DISP_MAX_NFFT) {
		struct disp_row r = {
			.freq = freq, .nfft = nfft, .drops = h->drops,
		};

		memcpy(r.bins, bins, nfft);
		if (k_msgq_put(&disp_q, &r, K_NO_WAIT)) {
			disp_drops++;
		}
	}
	return true;
}

static void hud_update(void)
{
	snprintf(hud_stat, sizeof(hud_stat),
		 "FPS%u.%u %s SWP%u ROWS%u DRP%u %uKB F%uMHZ%s%s",
		 fps_x10 / 10, fps_x10 % 10, view_name(),
		 st_sweep, st_rows, st_drops, st_staged / 1024,
		 st_freq_mhz, st_fails ? "  *FAIL*" : "",
		 link_recovers ? "  RCV" : "");
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

/* Mid-run recovery: the C6 occasionally self-reboots mid-burst and the
 * P4 then sees rpc tx -116 until the bus re-enumerates. Don't wait for
 * the transport to notice on its own — pulse EN, wait for slave init,
 * verify the link answers. */
static int link_recover(void)
{
	for (int i = 0; i < 3; i++) {
		if (esp_ng_slave_reinit() == 0 &&
		    esp_ng_wait_slave_init(K_SECONDS(20)) == 0 &&
		    esp_ng_dev() && device_is_ready(esp_ng_dev()) &&
		    esp_ng_slave_mac((uint8_t[6]){0}) == 0) {
			link_recovers++;
			printk("link: recovered after C6 reset (#%u)\n",
			       link_recovers);
			hud_update();
			if (dsi_ok) {
				dsi_hud();
			}
			return 0;
		}
		k_sleep(K_SECONDS(2));
	}
	printk("link: recover failed\n");
	return -EIO;
}

int main(void)
{
	uint8_t mac[6];
	uint32_t major, minor, patch;
	uint32_t f_lo = CONFIG_C6_SDR_SWEEP_LO_MHZ;
	uint32_t f_hi = CONFIG_C6_SDR_SWEEP_HI_MHZ;
	uint32_t f_step = CONFIG_C6_SDR_SWEEP_STEP_MHZ;

	sw_lo_mhz = f_lo;
	sw_hi_mhz = f_hi;
	sw_step_mhz = f_step;
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
	if (boot_btn.port) {
		gpio_pin_configure_dt(&boot_btn, GPIO_INPUT);
	}
	dsi_view_apply(CONFIG_C6_SDR_DSI_VIEW);
#endif
	audio_init();

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

	/* ---- spectrum sweep: one async SPEC stream per step; rows paint
	 * live in sdr_row_cb while the C6 is still capturing ---- */
	for (uint32_t run = 0; n_runs == 0 || run < n_runs; run++) {
		uint32_t total = 0;
		uint32_t consec_fail = 0;
		int ret;

		st_sweep = run + 1;
		for (uint32_t f = f_lo; f <= f_hi; f += f_step) {
			uint32_t nbytes = 0;
			int frames = 0;

			while (cap_busy) { /* UART dump owns the console */
				k_msleep(50);
			}
			st_freq_mhz = f;
			st_label_pending = true;
			if (dsi_ok && dsi_view == VIEW_IQ) {
				/* IQ view: bursts instead of sweeps — the
				 * scope + demod blip own the slot */
				struct esp_ng_sdr_run_res ires;
				uint32_t ilen = 0;

				ret = sdr_iq_burst_pull(
							iq_lock ? iq_lock_hz
								: f * 1000000u,
							IQ_PTS_CAP / 4, 16,
							gain,
							ESP_NG_SDR_DCAP_AUTO,
							cap_buf, CAP_MAX,
							&ilen, &ires);
				if (ret) {
					fail++;
					st_fails = fail;
					if (++consec_fail >= 3 &&
					    link_recover() == 0) {
						consec_fail = 0;
					}
				} else {
					consec_fail = 0;
				}
				if (!ret && ilen >= 8 && !ires.status) {
					uint32_t n = MIN(ilen / 2,
							 IQ_PTS_CAP);
					memcpy(iq_pts, cap_buf, n * 2);
					iq_npairs = n;
					iq_seq++;
					iq_blip(iq_pts, n);
					total += ilen;
					st_staged += ilen;
					hud_update();
					if (dsi_ok) {
						dsi_hud();
					}
				}
				continue;
			}
			ret = sdr_spec_stream(f * 1000000u, 256,
					      CONFIG_C6_SDR_STRIDE,
					      CONFIG_C6_SDR_UNITS_PER_FRAME,
					      IS_ENABLED(CONFIG_C6_SDR_MAX_HOLD),
					      gain, CONFIG_C6_SDR_DCAP,
					      dur, 0,
					      sdr_row_cb,
					      (void *)(uintptr_t)(f *
								  1000000u),
					      &nbytes, &frames);
			if (ret == -ENODATA || (ret == 0 && !frames)) {
				printk("spec %u MHz: no SPC1 frames\n", f);
				fail++;
				st_fails = fail;
				if (++consec_fail >= 3 &&
				    link_recover() == 0) {
					consec_fail = 0;
				}
				continue;
			}
			if (ret) {
				printk("spec %u MHz: rpc err %d\n", f, ret);
				fail++;
				st_fails = fail;
				if (++consec_fail >= 3 &&
				    link_recover() == 0) {
					consec_fail = 0;
				}
				continue;
			}
			consec_fail = 0;
			total += nbytes;
			st_staged += nbytes;
			hud_update();
			if (dsi_ok) {
				dsi_hud();
			}
		}
		printk("sweep %u done: %u B streamed\n", run + 1, total);
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
