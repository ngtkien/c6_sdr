/* SPDX-License-Identifier: Apache-2.0
 * Zephyr ESP32-C6 hosted slave — SDIO link + hosted RPC + OTA.
 *
 * Runs from an IDF OTA slot; keeps the stock ESP-IDF 2nd-stage
 * bootloader + otadata. If the hosted data path never opens (driver
 * regression, bad image), a watchdog rewrites otadata back to the other
 * slot and reboots — the IDF bootloader then brings up the known-good
 * hybrid firmware again.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/fatal.h>
#include <esp_system.h>
#include "esp_hosted_rpc.pb.h"

#include "wire.h"
#include "link.h"
#include "rpc.h"
#include "ota_boot.h"

LOG_MODULE_REGISTER(c6_sdr, LOG_LEVEL_INF);

#define FALLBACK_TIMEOUT_S   90
#define HEARTBEAT_PERIOD_S   30

static struct k_work_delayable fallback_work;
static struct k_work_delayable hb_work;
static uint32_t hb_num;

static void fallback_revert(void)
{
	int slot = c6_ota_current_slot();

	if (slot >= 0) {
		LOG_ERR("no datapath after %us — reverting to ota_%d",
			FALLBACK_TIMEOUT_S, slot ^ 1);
		c6_ota_select_boot(slot ^ 1);
	} else {
		LOG_ERR("no datapath — rebooting (slot unknown)");
	}
	esp_restart();
}

static void fallback_handler(struct k_work *work)
{
	if (!c6_link_datapath()) {
		fallback_revert();
	}
}

static void hb_handler(struct k_work *work)
{
	if (c6_link_datapath()) {
		c6_rpc_send_event(RpcId_Event_Heartbeat, hb_num);
		hb_num++;
	}
	k_work_schedule(&hb_work, K_SECONDS(HEARTBEAT_PERIOD_S));
}

static void serial_rx(const uint8_t *ep, uint8_t ep_len,
		      const uint8_t *data, uint16_t data_len)
{
	if (ep_len == 6 && memcmp(ep, C6_EP_RSP, 6) == 0) {
		c6_rpc_dispatch(data, data_len);
	}
}

static void on_open_datapath(void)
{
	/* cancel the fallback — link is coming up */
	k_work_cancel_delayable(&fallback_work);
	/* hosted-ng expects the ESPInit protobuf event on RPCEvt */
	c6_rpc_send_event(RpcId_Event_ESPInit, 0);
	k_work_schedule(&hb_work, K_SECONDS(5));
}

int main(void)
{
	int ret;

	k_work_init_delayable(&fallback_work, fallback_handler);
	k_work_init_delayable(&hb_work, hb_handler);

	LOG_INF("c6_zephyr: sdio hosted slave");
	c6_ota_detect();
	ret = c6_link_init(serial_rx, on_open_datapath);
	if (ret) {
		LOG_ERR("link init: %d", ret);
		fallback_revert();
		return ret;
	}

	/* If the host never opens the data path, revert to the other slot
	 * so we don't leave the C6 stranded on a broken image. */
	k_work_schedule(&fallback_work, K_SECONDS(FALLBACK_TIMEOUT_S));

	LOG_INF("sdio slave ready, waiting for host");
	for (;;) {
		c6_ota_reboot_later();
		k_msleep(50);
	}
	return 0;
}

/* Fatal-error escape: try to leave a bootable fallback before dying. */
void k_sys_fatal_error_handler(unsigned int reason,
			       const struct arch_esf *esf)
{
	ARG_UNUSED(esf);
	int slot = c6_ota_current_slot();

	(void)reason;
	if (slot >= 0) {
		c6_ota_select_boot(slot ^ 1);
	}
	esp_restart();
}
