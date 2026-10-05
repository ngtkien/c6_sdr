# c6_sdr — ESP32-P4 spectrum / raw-IQ front-end over the C6

Software-defined-radio demo pair for the ESP32-P4 Function EV board:

- `c6_firmware/` — hybrid ESP32-C6 slave firmware: ESP-Hosted-MCU
  (hosted v1.4.7, SDIO transport, RPC + OTA intact) **plus** the
  [ESP-SDR](https://github.com/codelabs-ch/esp-sdr) RF-capture engine.
  The C6 keeps answering hosted RPCs while also exposing SDR commands —
  so the P4 can call it like a "scan" API and the C6 can still be
  updated over the air (no ESP-Prog needed to leave this firmware).
- `src/` — Zephyr app on the P4: drives the SDR RPCs over the existing
  SDIO link and renders/dumps the results.

Unlike the standalone ESP-SDR port (UART I/Q out on PROG_C6), everything
here flows through the same SDIO hosted link already wired between the
P4 and the C6 — no extra cabling.

## Radio path on the C6

- PHY tuned directly (`phy_set_freq`, `chip_v7_set_chan`),
  `WIFI_MODE_NULL` + RX-only; Bluetooth disabled to reclaim SRAM.
- Verified path: 80 MS/s raw RF dump into the two SRAM banks at
  `0x40820000`/`0x40840000` (16 384 IQ pairs each, 10-bit packed).
- SPEC mode runs a 256-point Q15 FFT on-chip (esp-dsp twiddles) and emits
  `SPC1` frames: 28-byte header + `nfft` 8-bit log-power bins + CRC32 —
  compact enough for a single RPC response payload.
- Raw-IQ burst mode captures up to 16 380 pairs and packs them as raw
  u32 words, iq8 (2 B/pair) or iq10, staged for chunked readback.
- Captured output lives in a ~96 KiB slave-side ring sink; the host
  pulls it with `SdrIqRead` chunks (≤ 1400 B each, fits the SDIO/TLV
  frame budget).

## Host RPC API (new, ids 380–383 / 680–683)

Declared in `zephyr-esp32p4-v1-bsp/drivers/wifi/esp_hosted_ng/esp_hosted_ng.h`:

```c
int esp_ng_sdr_spec(const struct esp_ng_sdr_spec_req *, struct esp_ng_sdr_run_res *);
int esp_ng_sdr_iq_start(const struct esp_ng_sdr_iq_req *, struct esp_ng_sdr_run_res *);
int esp_ng_sdr_iq_read(uint32_t offset, uint8_t *buf, uint32_t buf_len,
                       uint32_t *got, uint32_t *total);
int esp_ng_sdr_stop(void);
```

- `SdrSpec` is synchronous on the slave: tune → run the ring for
  `duration_ms` → respond `{status, total_len, frames, pairs, rate_hz}`.
- Then pull `total_len` bytes via `SdrIqRead(offset, …)` and parse
  `SPC1` frames (`sdr_walk_spc1` in `src/sdr.cpp`).
- `SdrIqStart` mode 0 = single-shot burst (`n_words` = 256..16380 pairs);
  mode 1 = ring run (C6: raw ring data, no S3-style decimation).
- `gain`: 255 = hardware AGC, 256 = keep, else manual code 0–54.
  `dcap`: 254 = PHY auto, 255 = keep, else filter code 0–63.
- Note: a `duration_ms = 0` SPEC run blocks the slave RPC task until the
  sink fills (≈96 KiB) + host-stall timeout (~2 s), then finishes on its
  own — use bounded durations in normal operation.

## Zephyr app

Boot → hosted link check (MAC + `GetCoprocessorFwVersion`, warns if the
C6 is still stock) → single-shot I/Q burst → `C6_SDR_RUNS` sweeps
(`0` = forever) of bounded `SdrSpec` runs from `C6_SDR_SWEEP_LO_MHZ` to
`C6_SDR_SWEEP_HI_MHZ`. Outputs (Kconfig):

- `C6_SDR_DSI=y` (default) — EK79007 DSI panel, 1024×600: scrolling
  RGB888 heat-map waterfall (one row pushed per SPC1 frame, so the
  display updates live), per-frequency MHz labels in the left gutter,
  and a HUD with C6 MAC/fw version, live counters (sweep, rows, drops,
  staged KB, current freq) and a dB colorbar.
- `C6_SDR_ASCII_WF` — 96-column ASCII waterfall on the console
  (`freq bin-row ffts gain drops`). Defaults to on only when DSI is
  off; sweep summaries + errors always print either way.
- `C6_SDR_SD_DUMP` — staged bytes appended to `/SD:/c6_sdr_spec.bin`,
  IQ burst to `/SD:/c6_sdr_iq.bin`. A card that rejects writes is
  disabled automatically after 4 failures.

```bash
# 1. build + flash the C6 hybrid firmware (ESP-IDF ≥ 5.3)
cd c6_firmware
idf.py set-target esp32c6 && idf.py build
# then flash via ESP-Prog on PROG_C6 (see below), OR OTA it from the
# c6_ota project if the C6 still runs hosted firmware:
#   C6_FW_FILE=c6_firmware/build/network_adapter.bin ../c6_ota/build.sh

# 2. build + flash the P4 app
./build.sh
./flash.sh /dev/ttyUSB2     # P4 console port
```

`network_adapter.bin` (~868 KiB) fits the 1536 KiB OTA slot, so the whole
cycle is reachable without opening the case.

## Flashing / restoring the C6

- **From hosted firmware (stock or v1.4.7):** plain OTA through
  `projects/c6_ota` works — the hybrid answers the same RPCs.
- **From standalone ESP-SDR or any non-hosted image:** the hosted link
  is gone — use `c6_ota`'s recovery mode (`CONFIG_C6_RECOVERY=y` or a
  `/SD:/c6_download.flg` file): the P4 drives C6 BOOT via GPIO47 and
  EN via GPIO54 into ROM download mode, then flash over the PROG_C6
  UART:

  ```bash
  esptool --chip esp32c6 -p <PORT> -b 460800 write-flash \
      --flash-mode dio --flash-freq 80m --flash-size 4MB \
      0x0     build/bootloader/bootloader.bin \
      0x8000  build/partition_table/partition-table.bin \
      0xd000  build/ota_data_initial.bin \
      0x10000 build/network_adapter.bin
  ```

- The hybrid itself still implements `OTABegin/Write/End`, so once it is
  on the C6 you can OTA back to a stock hosted image any time.

## Caveats

- The SDR capture path uses undocumented PHY/RF-register behavior from
  the ESP-SDR project — **hardware validation is required** before
  trusting absolute power levels; bin values are log-power codes
  (`db_step` in the frame header), not calibrated dBm.
- While a capture runs, the C6 masks interrupts on both cores — hosted
  RPC/SDIO traffic stalls for the run duration. Keep `duration_ms`
  modest; chunk pulls happen after the run.
- 100–6000 MHz tuning range; the useful instantaneous view is the
  ~40 MHz analog front-end bandwidth around the tuned frequency.
