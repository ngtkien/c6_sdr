/* SPDX-License-Identifier: Apache-2.0
 * ESP-Hosted-NG RPC dispatch + minimal handler set:
 *   ESPInit event, GetMacAddress, GetCoprocessorFwVersion, OTA stubs.
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <pb_decode.h>
#include <pb_encode.h>
#include <esp_efuse.h>
#include <esp_efuse_table.h>

#include "esp_hosted_rpc.pb.h"
#include "wire.h"
#include "link.h"
#include "rpc.h"
#include "ota_boot.h"

LOG_MODULE_DECLARE(c6_sdr, LOG_LEVEL_INF);

/* Rpc is ~4.4 KB — static, rpc is serialized anyway. */
static Rpc tx_msg;
static uint8_t pb_buf[1600];

/* MAC address: factory-programmed efuse base MAC (ESP_EFUSE_MAC in
 * EFUSE_BLK1). Falls back to a locally-administered OUI if unread. */
static void get_base_mac(uint8_t mac[6])
{
	if (esp_efuse_read_field_blob(ESP_EFUSE_MAC_FACTORY, mac, 48) != 0 ||
	    (mac[0] == 0 && mac[1] == 0 && mac[2] == 0)) {
		mac[0] = 0x02; mac[1] = 0xC6; mac[2] = 0x5D;
		mac[3] = 0x00; mac[4] = 0x00; mac[5] = 0x01;
	}
}

static int rpc_tx(Rpc *msg, const char *ep)
{
	pb_ostream_t os = pb_ostream_from_buffer(pb_buf, sizeof(pb_buf));
	int ret;

	if (!pb_encode(&os, Rpc_fields, msg)) {
		LOG_ERR("pb encode %u: %s", msg->msg_id, PB_GET_ERROR(&os));
		return -EINVAL;
	}
	ret = c6_link_tx_serial(ep, pb_buf, os.bytes_written);
	return ret;
}

/* Send an event. hb_num is ignored for non-heartbeat ids. */
int c6_rpc_send_event(uint32_t msg_id, uint32_t hb_num)
{
	memset(&tx_msg, 0, sizeof(tx_msg));
	tx_msg.msg_type = RpcType_Event;
	tx_msg.msg_id = msg_id;
	tx_msg.uid = 0;
	switch (msg_id) {
	case RpcId_Event_ESPInit:
		tx_msg.which_payload = Rpc_event_esp_init_tag;
		break;
	case RpcId_Event_Heartbeat:
		tx_msg.which_payload = Rpc_event_heartbeat_tag;
		tx_msg.payload.event_heartbeat.hb_num = hb_num;
		break;
	default:
		return -EINVAL;
	}
	return rpc_tx(&tx_msg, C6_EP_EVT);
}

static int rpc_resp(Rpc *req)
{
	req->msg_type = RpcType_Resp;
	req->msg_id = req->which_payload;
	return rpc_tx(req, C6_EP_RSP);
}

int c6_rpc_dispatch(const uint8_t *pb, uint16_t pb_len)
{
	static Rpc req;
	pb_istream_t is = pb_istream_from_buffer(pb, pb_len);

	memset(&req, 0, sizeof(req));
	if (!pb_decode(&is, Rpc_fields, &req)) {
		LOG_WRN("pb decode: %s", PB_GET_ERROR(&is));
		return -EINVAL;
	}
	if (req.msg_type != RpcType_Req) {
		return 0;
	}

	switch (req.msg_id) {
	case RpcId_Req_GetMACAddress: {
		uint8_t mac[6];

		get_base_mac(mac);
		req.which_payload = Rpc_resp_get_mac_address_tag;
		req.payload.resp_get_mac_address.resp = 0;
		req.payload.resp_get_mac_address.mac.size = 6;
		memcpy(req.payload.resp_get_mac_address.mac.bytes, mac, 6);
		LOG_INF("resp mac %02x:%02x:%02x:%02x:%02x:%02x",
			mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
		return rpc_resp(&req);
	}
	case RpcId_Req_GetCoprocessorFwVersion:
		req.which_payload = Rpc_resp_get_coprocessor_fwversion_tag;
		req.payload.resp_get_coprocessor_fwversion.resp = 0;
		req.payload.resp_get_coprocessor_fwversion.major1 = 1;
		req.payload.resp_get_coprocessor_fwversion.minor1 = 4;
		req.payload.resp_get_coprocessor_fwversion.patch1 = 90;
		return rpc_resp(&req);
	/* Transport-lifecycle RPCs the host issues during bring-up. This
	 * firmware has no Wi-Fi data path; acknowledging them is what lets
	 * the hosted link come up so control/OTA/SDR RPCs can run. */
	case RpcId_Req_WifiInit:
		req.which_payload = Rpc_resp_wifi_init_tag;
		req.payload.resp_wifi_init.resp = 0;
		return rpc_resp(&req);
	case RpcId_Req_SetWifiMode:
		req.which_payload = Rpc_resp_set_wifi_mode_tag;
		req.payload.resp_set_wifi_mode.resp = 0;
		return rpc_resp(&req);
	case RpcId_Req_WifiStart:
		req.which_payload = Rpc_resp_wifi_start_tag;
		req.payload.resp_wifi_start.resp = 0;
		return rpc_resp(&req);
	case RpcId_Req_OTABegin:
		req.which_payload = Rpc_resp_ota_begin_tag;
		req.payload.resp_ota_begin.resp = c6_ota_begin();
		return rpc_resp(&req);
	case RpcId_Req_OTAWrite:
		req.which_payload = Rpc_resp_ota_write_tag;
		req.payload.resp_ota_write.resp =
			c6_ota_write(req.payload.req_ota_write.ota_data.bytes,
				     req.payload.req_ota_write.ota_data.size);
		return rpc_resp(&req);
	case RpcId_Req_OTAEnd:
		req.which_payload = Rpc_resp_ota_end_tag;
		req.payload.resp_ota_end.resp = c6_ota_end();
		return rpc_resp(&req);
	case RpcId_Req_ConfigHeartbeat:
		/* accept and ack — heartbeat events not yet generated */
		req.which_payload = Rpc_resp_config_heartbeat_tag;
		req.payload.resp_config_heartbeat.resp = 0;
		return rpc_resp(&req);
	default:
		/* Drop the request — the host times out and treats it as
		 * unsupported, which is safer than a fake resp=0 success. */
		LOG_WRN("unhandled rpc %u — dropped", req.msg_id);
		return 0;
	}
}
