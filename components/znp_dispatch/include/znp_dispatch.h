// SPDX-FileCopyrightText: 2025-2026 Evgenij Cjura and project contributors
// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include "mt_proto.h"
#ifdef __cplusplus
extern "C" {
#endif

/* Identity/version reported to the host. Host logs VERSION/PING (not gated),
 * so these values are free; keep them stable. */
#define ZNP_TRANSPORT_REV 0x02
#define ZNP_PRODUCT_ID    0x00
#define ZNP_VER_MAJOR     0x02
#define ZNP_VER_MINOR     0x07
#define ZNP_VER_MAINT     0x01
/* Advertised MT capability bitmap (SYS_PING SRSP, 16-bit LE).
 * def 5 (LOW): the old 0x0179 advertised SAPI (MT_CAP_SAPI 0x0020)
 * and UTIL (MT_CAP_UTIL 0x0040) — but neither subsystem is routed in
 * znp_dispatch (a capability-probing host such as zigbee-herdsman, whose startup
 * issues UTIL_GET_DEVICE_INFO, would trust a dead subsystem and then time out).
 * Trimmed (not stubbed — trimming is the safe, honest choice for now) to ONLY
 * the subsystems this NCP actually dispatches:
 *   MT_CAP_SYS 0x0001 | MT_CAP_AF 0x0008 | MT_CAP_ZDO 0x0010 | MT_CAP_APP 0x0100
 * (APP_CNF answers under the MT_CAP_APP bit.) = 0x0119. */
#define ZNP_PING_CAPS     0x0119

/* ── NV item IDs (mirrors z-stack-3.x / zigbee_mgr.cpp) ─────────────────── */
#define ZNP_NV_STARTUP_OPTION  0x0003
#define ZNP_NV_LOGICAL_TYPE    0x0087
#define ZNP_NV_PANID           0x0083
#define ZNP_NV_EXTENDED_PANID  0x002D
#define ZNP_NV_CHANLIST        0x0084
#define ZNP_NV_PRECFGKEY       0x0062

/* ── TI ZCD_STARTOPT_* bits for NV STARTUP_OPTION (id 0x0003) ─────────────
 * The host writes 0x03 (CLEAR_CONFIG|CLEAR_STATE) before a SYS_RESET to demand
 * a blank coordinator (see zigbee_mgr.cpp do_commissioning ~:299). On TI these
 * bits make the boot loader wipe NV at startup. On this NCP esp_restart()
 * preserves the zb_storage NVRAM, so we must reproduce that wipe ourselves —
 * see ZNP_FACTORY_NEW_* below and znp_ezb.c. */
#define ZNP_STARTOPT_CLEAR_CONFIG  0x01   /* TI ZCD_STARTOPT_CLEAR_CONFIG */
#define ZNP_STARTOPT_CLEAR_STATE   0x02   /* TI ZCD_STARTOPT_CLEAR_STATE  */

/* ── TI OSAL NV operation statuses (returned in NV SRSP status byte) ──────── */
#define ZNP_NV_SUCCESS       0x00   /* ZSuccess                         */
#define ZNP_NV_OPER_FAILED   0x0A   /* NV_OPER_FAILED — write rejected  */
#define ZNP_NV_ITEM_UNINIT   0x02   /* NV_ITEM_UNINIT — item not present */

/* ── Factory-new flag (def 1 / hardening CRIT) ─────────────────────────
 * When the host writes STARTUP_OPTION with a clear bit set, the dispatcher
 * raises this latch in the dispatch ctx. The chip backend persists it to NVS
 * (znp_ezb.c) so that on the NEXT boot, BEFORE esp_zigbee_init, the zb_storage
 * NVRAM partition is erased — giving the host the blank radio it asked for. */

/* ── Network-config buffer (filled by buffered NV writes) ────────────────── */
typedef struct {
    uint16_t pan_id;         bool have_pan_id;
    uint8_t  ext_pan_id[8];  bool have_ext_pan_id;
    uint32_t chan_mask;       bool have_chan_mask;
    uint8_t  nwk_key[16];    bool have_nwk_key;
    uint8_t  logical_type;   bool have_logical_type;
} znp_netcfg_t;

/* Apply one NV write to cfg. Returns true if accepted, false if REJECTED
 * (a known id whose value is too short to be valid — e.g. an 8-byte PRECFGKEY
 * when 16 are required). Unknown/untracked ids are accepted (return true) and
 * ignored. Guards against short val buffers — never over-reads.
 * Def 3 (MED): the rejection path is real now; a false result must
 * be surfaced as an NV-failure status in the write SRSP (callers must not lie
 * 0x00-success when apply was rejected). */
bool znp_netcfg_apply_nv(znp_netcfg_t *cfg, uint16_t id,
                          const uint8_t *val, uint8_t len);

/* ── EZB BDB mode constants (local defines — do NOT include esp-zigbee headers
 * in the pure dispatcher; real calls happen in znp_ezb / Task 4.5). ─────── */
#define ZNP_EZB_BDB_FORMATION 0x08   /* ESP-Zigbee ESP_ZB_BDB_MODE_NETWORK_FORMATION */
#define ZNP_EZB_BDB_STEERING  0x04   /* ESP-Zigbee ESP_ZB_BDB_MODE_NETWORK_STEERING  */

/* TI Z-Stack BDB mode byte values (from wire: zigbee_mgr.cpp do_commissioning) */
#define ZNP_TI_BDB_FORMATION  0x04
#define ZNP_TI_BDB_STEERING   0x02

/* Thin seam between pure protocol logic and chip-specific actions, so the
 * dispatcher is host-testable with a fake backend. */
typedef struct {
    void (*get_ieee)(uint8_t out[8]);   /* 802.15.4 EUI64, little-endian (wire order) */
    void (*request_reset)(void);        /* trigger a reset; RESET_IND follows on boot */
    void (*apply_config)(const znp_netcfg_t *cfg);   /* push buffered cfg into the stack */
    bool (*start_stack)(void);                        /* init coordinator + launch */
    bool (*bdb_commission)(uint8_t ezb_mode_mask);   /* ezb_bdb_start_top_level_commissioning */
    bool (*get_nwk_info)(uint16_t *panid, uint16_t *short_addr, uint8_t *dev_state); /* (4.3) */
    bool (*permit_join)(uint8_t duration_s);
    /* def 1: latch a persistent "factory-new" request. The dispatcher calls this
     * when the host writes STARTUP_OPTION with a clear bit set. The backend MUST
     * persist it across the imminent SYS_RESET (esp_restart) so the next boot
     * erases the Zigbee NVRAM before esp_zigbee_init. May be NULL on host tests. */
    void (*request_factory_new)(void);
    /* Phase 5: trigger an over-the-air ZDO interview request. `req_cmd1` is the
     * MT ZDO cmd1 (0x02 NODE_DESC, 0x04 ACTIVE_EP, 0x05 SIMPLE_DESC); `endpoint`
     * is only meaningful for SIMPLE_DESC. The response returns asynchronously as
     * the matching AREQ (built by znp_build_{node,active_ep,simple}_desc_rsp and
     * sent via znp_uart_send_raw). Returns false if the request can't be issued.
     * Nullable — a NULL hook makes the ZDO SREQ answer status=success but never
     * produce an AREQ (the host's interview step then times out and falls back). */
    bool (*zdo_request)(uint8_t req_cmd1, uint16_t nwk, uint8_t endpoint);
    /* Phase 6: transmit a ZCL frame to a device (AF_DATA_REQUEST → ezb APSDE).
     * `data`/`len` is the fully-formed ZCL body. Nullable. */
    bool (*af_data_request)(uint16_t nwk, uint8_t dst_ep, uint8_t src_ep,
                            uint16_t cluster_id, uint8_t trans_id,
                            const uint8_t *data, uint8_t len);
} znp_backend_t;

/* ── Dispatch context (carries config buffer + backend pointer) ──────────── */
typedef struct {
    znp_netcfg_t         cfg;
    const znp_backend_t *be;
} znp_dispatch_ctx;

/* Handle one received request frame. If a SRSP must be sent, encodes the full
 * MT response into buf and returns its byte length (>0). Returns 0 when there is
 * no response to send (AREQ-style side-effects like RESET_REQ, or unhandled cmd). */
size_t znp_dispatch(const mt_frame_t *req, znp_dispatch_ctx *ctx,
                    uint8_t *buf, size_t cap);

/* Build a SYS_RESET_IND (AREQ 0x41/0x80) into buf. reason 0x00 = power-up.
 * Returns encoded length. */
size_t znp_build_reset_ind(uint8_t reason, uint8_t *buf, size_t cap);

/* ── Unsolicited AREQ builders (Task 4.4) ────────────────────────────────
 * Each encodes a full MT frame (SOF…FCS) into buf[0..n-1].
 * Returns total bytes written, or 0 on overflow.
 * Task 4.5 signal handler calls these then znp_uart_send_raw(). */

/* ZDO_STATE_CHANGE_IND  AREQ 0x45/0xC0 — 1-byte ZDO state.
 * dev_state: e.g. 0x09 = DEV_ZB_COORD (zigbee_mgr.cpp on_state_change). */
size_t znp_build_state_change_ind(uint8_t dev_state, uint8_t *buf, size_t cap);

/* ZDO_TC_DEV_IND  AREQ 0x45/0xCA — device joined/rejoined.
 * Payload: nwk(2 LE) + ieee(8 LE) + capabilities(1) = 11 bytes.
 * P4 parser reads nwk at [0..1], ieee at [2..9]; capabilities passed through.
 * (zigbee_interview.cpp on_tc_dev_ind, payload_len<11 guard). */
size_t znp_build_tc_dev_ind(uint16_t nwk_addr, uint64_t ieee,
                             uint8_t capabilities,
                             uint8_t *buf, size_t cap);

/* ZDO_LEAVE_IND  AREQ 0x45/0xC9 — device left.
 * Payload: nwk(2 LE) + ieee(8 LE) + remove(1) + rejoin(1) = 12 bytes.
 * P4 parser: payload_len<10 guard; reads ieee at [2..9].
 * (zigbee_mgr.cpp on_zdo_leave_ind comment + body). */
size_t znp_build_leave_ind(uint16_t src_addr, uint64_t ieee,
                            uint8_t remove, uint8_t rejoin,
                            uint8_t *buf, size_t cap);

/* ZDO_MGMT_PERMIT_JOIN_RSP  AREQ 0x45/0xB6 — companion to the 0x36 SRSP (def 1).
 * Payload: srcaddr(2 LE) + status(1) = 3 bytes. The 0x36 case in znp_dispatch
 * already returns the SRSP the host blocks on; this optional unsolicited RSP is
 * what a faithful TI Z-Stack NCP additionally emits. The chip backend's
 * permit_join path may build it here then znp_uart_send_raw() it (mirrors the
 * other AREQ builders). Harmless if the host registers no 0xB6 handler. */
size_t znp_build_permit_join_rsp(uint16_t src_addr, uint8_t status,
                                 uint8_t *buf, size_t cap);

/* Phase 5 (interview) — ZDO ACTIVE_EP_RSP AREQ (cmd0 MT_AREQ(ZNP_ZDO), cmd1
 * 0x85). Emitted after the over-the-air ZDO Active-EP exchange completes, so
 * the host's zigbee_interview.cpp wait_rsp(0x85, nwk) unblocks. Payload:
 *   SrcAddr(2 LE) Status(1) NWKAddr(2 LE) ActiveEPCount(1) ActiveEPList(count)
 * SrcAddr and NWKAddr are both the interviewed device's short address (what
 * the host matches on). `ep_count` endpoints are copied from `ep_list`.
 * Returns encoded length, or 0 if the frame would overflow `cap`. */
size_t znp_build_active_ep_rsp(uint16_t nwk, uint8_t status,
                               const uint8_t *ep_list, uint8_t ep_count,
                               uint8_t *buf, size_t cap);

/* Phase 5 — ZDO NODE_DESC_RSP AREQ (cmd1 0x82). Payload:
 *   SrcAddr(2 LE) Status(1) NWKAddr(2 LE) NodeDescriptor(13)
 * The host reads only the logical device type (node-desc is non-fatal on the
 * ZHAC side). `logical_type` (0=coord,1=router,2=end-device) goes in the low 3
 * bits of NodeDescriptor byte 0; the remaining descriptor bytes are left as a
 * benign default. Returns encoded length or 0 on overflow. */
size_t znp_build_node_desc_rsp(uint16_t nwk, uint8_t status,
                               uint8_t logical_type,
                               uint8_t *buf, size_t cap);

/* Phase 5 — ZDO SIMPLE_DESC_RSP AREQ (cmd1 0x84). TI MT layout (host parses
 * profile@+7, device@+9, in-count@+12): SrcAddr(2) Status(1) NWKAddr(2)
 * Len(1) Endpoint(1) ProfileID(2 LE) DeviceID(2 LE) DevVer(1)
 * NumIn(1) InClusters(2*NumIn LE) NumOut(1) OutClusters(2*NumOut LE).
 * `Len` = the SimpleDescriptor byte count (everything after it). Returns
 * encoded length or 0 on overflow / bad counts. */
size_t znp_build_simple_desc_rsp(uint16_t nwk, uint8_t status, uint8_t endpoint,
                                 uint16_t profile_id, uint16_t device_id,
                                 uint8_t dev_ver,
                                 const uint16_t *in_clusters,  uint8_t in_count,
                                 const uint16_t *out_clusters, uint8_t out_count,
                                 uint8_t *buf, size_t cap);

/* Phase 6 — AF_INCOMING_MSG AREQ (cmd0 MT_AREQ(ZNP_AF), cmd1 0x81): a device's
 * ZCL frame delivered up to the host. TI MT layout (host reads group@0,
 * cluster@2, src_nwk@4, src_ep@6, lqi@9, len@16, data@17):
 *   GroupID(2) ClusterID(2) SrcAddr(2) SrcEndpoint(1) DstEndpoint(1)
 *   WasBroadcast(1) LinkQuality(1) SecurityUse(1) TimeStamp(4) TransSeqNum(1)
 *   Len(1) Data(Len)
 * Timestamp/securityuse/transseq are emitted as 0 (the host ignores them for
 * decode). Returns encoded length or 0 on overflow. */
size_t znp_build_af_incoming_msg(uint16_t group_id, uint16_t cluster_id,
                                 uint16_t src_nwk, uint8_t src_ep, uint8_t dst_ep,
                                 uint8_t lqi, const uint8_t *data, uint8_t data_len,
                                 uint8_t *buf, size_t cap);

/* Phase 6 — AF_DATA_CONFIRM AREQ (cmd0 MT_AREQ(ZNP_AF), cmd1 0x80): confirms a
 * host AF_DATA_REQUEST. Payload: Status(1) Endpoint(1) TransID(1). */
size_t znp_build_af_data_confirm(uint8_t status, uint8_t endpoint, uint8_t trans_id,
                                 uint8_t *buf, size_t cap);

#ifdef __cplusplus
}
#endif
