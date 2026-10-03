/* spectrum.c — c6_sdr trim: only the shared FFT-table init survives.
 *
 * Upstream spectrum.c carries the UART snapshot command path plus ~26 KiB
 * of static workspace (fft_data/window/powers/frame). The hybrid slave
 * serves spectrum through ring_capture's RING_MODE_SPEC instead, which
 * does its own sliced radix-2 work in ring_scalar.h against
 * dsps_fft_w_table_sc16. Keeping just the init saves the workspace and
 * keeps .bss below the 0x40820000 RF-dump guard on ESP32-C6. */
#include "spectrum.h"
#include "ring_capture.h"
#include "dsps_fft2r.h"

static int16_t twiddles[RING_SPEC_NFFT_MAX] __attribute__((aligned(16)));
static bool ready;

bool spectrum_fft_init(void) {
    if (!ready)
        ready = dsps_fft2r_init_sc16(twiddles, RING_SPEC_NFFT_MAX) == ESP_OK;
    return ready;
}

/* Unused in the RPC build; kept so the header's contract still links. */
spectrum_workspace_t spectrum_workspace(void) {
    return (spectrum_workspace_t){0, 0, 0, 0};
}

bool spectrum_command(const char *line, unsigned frequency_mhz,
                      spectrum_acquire_fn acquire) {
    (void)line; (void)frequency_mhz; (void)acquire;
    return false;
}
