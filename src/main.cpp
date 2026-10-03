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

static void sd_mount_try(void)
{
	mp.fs_data = &fat_fs;
	if (disk_access_ioctl("SD", DISK_IOCTL_CTRL_INIT, NULL) == 0 &&
	    fs_mount(&mp) == 0) {
		sd_ok = true;
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
		return;
	}
	rc = fs_write(&f, buf, len);
	fs_close(&f);
	LOG_INF("sd: wrote %u B to %s", len, path);
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
		return;
	}
	fs_write(&f, buf, len);
	fs_close(&f);
}

/* ---------- DSI waterfall ---------- */
#define PANEL_W 1024
#define PANEL_H 600
static uint8_t fb[PANEL_W * PANEL_H * 3] __aligned(64)
	__attribute__((section(".ext_ram.bss")));
static const struct device *disp;
static int wf_row = 60;
static bool dsi_ok;

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

static void dsi_init(void)
{
	disp = bsp_get_display();
	if (!disp || !device_is_ready(disp)) {
		LOG_WRN("no DSI panel");
		return;
	}
	memset(fb, 0, sizeof(fb));
	display_blanking_off(disp);
	dsi_ok = true;
	LOG_INF("DSI panel up");
}

/* paint one spectrum row at wf_row; bins are dB-like codes */
static void dsi_spec_row(const uint8_t *bins, uint32_t nfft,
			 uint32_t lo_hz, uint32_t span_hz)
{
	uint8_t *row = fb + wf_row * PANEL_W * 3;
	uint32_t px_per_bin = PANEL_W / nfft;

	if (!px_per_bin) {
		px_per_bin = 1;
	}
	for (uint32_t k = 0; k < nfft; k++) {
		uint8_t r, g, b;

		db_rgb(bins[k], &r, &g, &b);
		for (uint32_t p = 0; p < px_per_bin; p++) {
			uint8_t *px = row + (k * px_per_bin + p) * 3;
			px[0] = r; px[1] = g; px[2] = b;
		}
	}
	if (px_per_bin * nfft < PANEL_W) {
		memset(row + px_per_bin * nfft * 3, 0,
		       (PANEL_W - px_per_bin * nfft) * 3);
	}
	if (++wf_row >= PANEL_H) {
		wf_row = 60;
	}
	(void)lo_hz; (void)span_hz;
}

static void dsi_flush(void)
{
	struct display_buffer_descriptor d = {
		.buf_size = sizeof(fb),
		.width = PANEL_W, .height = PANEL_H, .pitch = PANEL_W,
	};
	sys_cache_data_flush_range(fb, sizeof(fb));
	display_write(disp, 0, 0, &d, fb);
}

/* ---------- serial waterfall ---------- */
static const char ascii[] = " .:-=+*#%@";
static uint8_t spec_lo, spec_hi;

static bool ascii_row_cb(const struct sdr_spc1 *h, const uint8_t *bins,
			 void *arg)
{
	uint32_t nfft = 1u << (h->nfft_log2 & 0x0f);
	uint32_t cols = 96;
	char line[97];
	uint32_t freq = (uint32_t)(uintptr_t)arg;

	for (uint32_t c = 0; c < cols; c++) {
		uint32_t k = c * nfft / cols;
		uint8_t v = bins[k];
		uint8_t g;

		g = (uint8_t)((uint32_t)v * (sizeof(ascii) - 2) / 255);
		line[c] = ascii[g];
	}
	line[cols] = 0;
	printk("%4u %s f%u g%u d%u\n", freq / 1000000, line, h->ffts,
	       h->gain, h->drops);
	if (dsi_ok) {
		dsi_spec_row(bins, nfft, 0, 0);
	}
	if (bins[0] < spec_lo) {
		spec_lo = bins[0];
	}
	if (bins[nfft / 2] > spec_hi) {
		spec_hi = bins[nfft / 2];
	}
	return true;
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
		if (!(major == 1 && minor == 4 && patch >= 90)) {
			printk("link: warn — fw is not c6_sdr; SDR RPCs will "
			       "not be answered (restore/flash c6-sdr.bin)\n");
		}
	} else {
		printk("link: fw version unsupported (stock slave?)\n");
	}

#ifdef CONFIG_C6_SDR_DSI
	dsi_init();
#endif

	/* ---- spectrum sweep: one bounded SPEC run per step ---- */
	for (uint32_t run = 0; run < n_runs; run++) {
		uint32_t total = 0;
		struct esp_ng_sdr_run_res res;
		int ret;

		for (uint32_t f = f_lo; f <= f_hi; f += f_step) {
			uint32_t len = 0;

			ret = sdr_spec_pull(f * 1000000u, 256,
					    CONFIG_C6_SDR_STRIDE,
					    CONFIG_C6_SDR_UNITS_PER_FRAME,
					    IS_ENABLED(CONFIG_C6_SDR_MAX_HOLD),
					    gain, CONFIG_C6_SDR_DCAP,
					    dur, cap_buf, CAP_MAX, &len, &res);
			if (ret) {
				printk("spec %u MHz: rpc err %d\n", f, ret);
				fail++;
				continue;
			}
			total += len;
			if (res.status) {
				printk("spec %u MHz: ring status %u/%u\n",
				       f, res.status, res.detail);
				continue;
			}
			int frames = sdr_walk_spc1(cap_buf, len, ascii_row_cb,
						   (void *)(uintptr_t)(f *
								       1000000u));
			if (IS_ENABLED(CONFIG_C6_SDR_SD_DUMP) && sd_ok) {
				sd_append("/SD:/c6_sdr_spec.bin", cap_buf,
					  len);
			}
			if (!frames) {
				printk("spec %u MHz: %u B, no SPC1 frames\n",
				       f, len);
			}
		}
#ifdef CONFIG_C6_SDR_DSI
		if (dsi_ok) {
			dsi_flush();
		}
#endif
		printk("sweep %u done: %u B staged total\n", run + 1, total);
	}

	/* ---- single-shot I/Q burst demo ---- */
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
			if (IS_ENABLED(CONFIG_C6_SDR_SD_DUMP) && sd_ok) {
				sd_write("/SD:/c6_sdr_iq.bin", cap_buf, len);
			}
		}
	}

done:
	printk("--- c6_sdr demo %s (%d fail) ---\n",
	       fail ? "FAIL" : "PASS", fail);
	return 0;
}
