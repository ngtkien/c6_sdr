/* SPDX-License-Identifier: Apache-2.0
 * ESP-IDF-compatible OTA control: otadata select + hosted OTA RPCs.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

/* Must run early at boot so the fallback path knows the running slot. */
void c6_ota_detect(void);
/* Current OTA slot this image booted from (0=ota_0, 1=ota_1, -1=unknown). */
int  c6_ota_current_slot(void);
/* Write otadata so the IDF bootloader selects `slot` next boot. */
int  c6_ota_select_boot(int slot);

/* Hosted OTA RPC handlers (slave side). */
int  c6_ota_begin(void);
int  c6_ota_write(const uint8_t *data, uint32_t len);
int  c6_ota_end(void);
/* Reboot into whatever otadata says — called shortly after c6_ota_end. */
void c6_ota_reboot_later(void);
