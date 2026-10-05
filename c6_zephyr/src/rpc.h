/* SPDX-License-Identifier: Apache-2.0
 * ESP-Hosted-NG RPC dispatch (nanopb) + minimal handler set.
 */
#pragma once

#include <stdint.h>

int  c6_rpc_dispatch(const uint8_t *pb, uint16_t pb_len);
int  c6_rpc_send_event(uint32_t msg_id, uint32_t hb_num);
