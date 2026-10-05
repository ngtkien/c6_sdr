/* SPDX-License-Identifier: Apache-2.0
 * ESP-IDF-compatible OTA control over the raw flash device.
 *
 * otadata @ 0xd000 holds two 32-byte esp_ota_select_entry_t records
 * (sector 0 and sector 1). The bootloader picks the record with a valid
 * crc and the highest ota_seq; slot = (ota_seq - 1) % ota_app_count.
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/crc.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <esp_system.h>

#include "ota_boot.h"

LOG_MODULE_DECLARE(c6_sdr, LOG_LEVEL_INF);

#define OTADATA_BASE    0xD000
#define OTADATA_OFF0    0x0000
#define OTADATA_OFF1    0x1000
#define OTA_SLOT_BASE0  0x10000
#define OTA_SLOT_BASE1  0x190000
#define OTA_SLOT_SIZE   0x180000     /* 1536K */
#define OTA_SLOT_COUNT  2
#define OTA_IMG_VALID   2            /* ESP_OTA_IMG_VALID */
#define OTA_IMG_INVALID 3            /* ESP_OTA_IMG_INVALID — never selectable */
#define OTA_IMG_ABORTED 4            /* ESP_OTA_IMG_ABORTED — never selectable */

struct ota_select_entry {
	uint32_t ota_seq;
	uint8_t  seq_label[20];
	uint32_t ota_state;
	uint32_t crc;
};

static const struct device *flash_dev =
	DEVICE_DT_GET(DT_CHOSEN(zephyr_flash_controller));
static const uint32_t slot_base[OTA_SLOT_COUNT] = {
	OTA_SLOT_BASE0, OTA_SLOT_BASE1
};
static int current_slot = -1;

/* ota write state */
static uint32_t ota_off;
static int      ota_target = -1;
static bool     ota_done;

static uint32_t entry_crc(uint32_t seq)
{
	return crc32_ieee_update(UINT32_MAX, (const uint8_t *)&seq, 4);
}

static bool entry_valid(const struct ota_select_entry *e)
{
	return e->crc == entry_crc(e->ota_seq);
}

/* bootloader_common_ota_select_invalid(): an aborted/invalid entry is
 * never selected, whatever its crc. */
static bool entry_invalid(const struct ota_select_entry *e)
{
	return e->ota_seq == UINT32_MAX || e->ota_state == OTA_IMG_INVALID ||
	       e->ota_state == OTA_IMG_ABORTED;
}

static int read_entry(uint32_t off, struct ota_select_entry *e)
{
	return flash_read(flash_dev, OTADATA_BASE + off, e, sizeof(*e));
}


/* Running slot = the slot otadata would have selected this boot. */
static void detect_current_slot(void)
{
	struct ota_select_entry e[2];
	bool v[2];
	uint32_t seq;

	if (current_slot >= 0) {
		return;
	}
	/* Mirror the IDF bootloader: when both records are "invalid"
	 * (seq 0xFFFFFFFF / INVALID / ABORTED) it prints
	 * "No factory image, trying OTA 0" and boots slot 0 whatever
	 * their crc says. Otherwise the highest-seq *selectable* entry
	 * wins. */
	bool inv0 = read_entry(OTADATA_OFF0, &e[0]) != 0 ||
		    entry_invalid(&e[0]);
	bool inv1 = read_entry(OTADATA_OFF1, &e[1]) != 0 ||
		    entry_invalid(&e[1]);

	if (inv0 && inv1) {
		current_slot = 0;
		LOG_INF("running from ota_0 (otadata fallback)");
		return;
	}
	v[0] = !inv0 && entry_valid(&e[0]);
	v[1] = !inv1 && entry_valid(&e[1]);
	if (!v[0] && !v[1]) {
		current_slot = 0;
		LOG_INF("running from ota_0 (no valid otadata)");
		return;
	}
	seq = !v[0] ? e[1].ota_seq :
	      !v[1] ? e[0].ota_seq :
	      (e[0].ota_seq > e[1].ota_seq ? e[0].ota_seq : e[1].ota_seq);
	current_slot = (int)((seq - 1) % OTA_SLOT_COUNT);
	LOG_INF("running from ota_%d (ota_seq %u)", current_slot, seq);
}

void c6_ota_detect(void)
{
	detect_current_slot();
}

int c6_ota_current_slot(void)
{
	return current_slot;
}

int c6_ota_select_boot(int slot)
{
	struct ota_select_entry e[2], nw;
	uint32_t seq, new_off;
	bool v0, v1;
	int active, ret;

	v0 = read_entry(OTADATA_OFF0, &e[0]) == 0 && entry_valid(&e[0]);
	v1 = read_entry(OTADATA_OFF1, &e[1]) == 0 && entry_valid(&e[1]);
	seq = v0 ? e[0].ota_seq : 0;
	if (v1 && e[1].ota_seq > seq) {
		seq = e[1].ota_seq;
	}
	/* smallest seq > current that maps to target slot */
	while (((seq - 1 + 1) % OTA_SLOT_COUNT) != slot) {
		seq++;
	}
	seq++;  /* now (seq-1)%2 == slot */

	memset(&nw, 0xFF, sizeof(nw));
	nw.ota_seq = seq;
	nw.ota_state = OTA_IMG_VALID;
	nw.crc = entry_crc(seq);

	/* write to the sector opposite the currently-active (newest) entry */
	active = (v1 && (!v0 || e[1].ota_seq > e[0].ota_seq)) ? 1 : 0;
	new_off = active ? OTADATA_OFF0 : OTADATA_OFF1;

	ret = flash_erase(flash_dev, OTADATA_BASE + new_off, 0x1000);
	if (!ret) {
		ret = flash_write(flash_dev, OTADATA_BASE + new_off,
				  &nw, sizeof(nw));
	}
	if (ret) {
		LOG_ERR("otadata write: %d", ret);
		return ret;
	}
	LOG_INF("otadata -> ota_%d (seq %u)", slot, seq);
	return 0;
}

int c6_ota_begin(void)
{
	int ret;

	detect_current_slot();
	ota_target = current_slot < 0 ? 1 : (current_slot ^ 1);
	ota_off = 0;
	ota_done = false;
	LOG_INF("ota begin -> ota_%d", ota_target);
	ret = flash_erase(flash_dev, slot_base[ota_target], OTA_SLOT_SIZE);
	if (ret) {
		LOG_ERR("slot erase: %d", ret);
		return ret;
	}
	return 0;
}

int c6_ota_write(const uint8_t *data, uint32_t len)
{
	uint32_t pad;
	int ret;

	if (ota_target < 0 || ota_done) {
		return -EINVAL;
	}
	pad = (4 - (len & 3)) & 3;
	ret = flash_write(flash_dev, slot_base[ota_target] + ota_off,
			  data, len);
	if (!ret && pad) {
		uint8_t z[3] = {0};

		ret = flash_write(flash_dev,
				  slot_base[ota_target] + ota_off + len,
				  z, pad);
	}
	if (ret) {
		LOG_ERR("ota write @%u: %d", ota_off, ret);
		return ret;
	}
	ota_off += len;
	return 0;
}

int c6_ota_end(void)
{
	int ret;

	if (ota_target < 0) {
		return -EINVAL;
	}
	/* The IDF bootloader validates the image before jumping and
	 * auto-falls-back to the other slot if it fails — that's the
	 * recovery path for a bad write. */
	ret = c6_ota_select_boot(ota_target);
	if (!ret) {
		ota_done = true;
	}
	return ret;
}

void c6_ota_reboot_later(void)
{
	if (!ota_done) {
		return;
	}
	LOG_INF("ota done — rebooting");
	k_sleep(K_MSEC(100));
	esp_restart();
}
