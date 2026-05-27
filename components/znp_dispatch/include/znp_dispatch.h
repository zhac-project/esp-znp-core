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
#define ZNP_PING_CAPS     0x0179   /* advertised MT capability bitmap (16-bit LE) */

/* ── NV item IDs (mirrors z-stack-3.x / zigbee_mgr.cpp) ─────────────────── */
#define ZNP_NV_STARTUP_OPTION  0x0003
#define ZNP_NV_LOGICAL_TYPE    0x0087
#define ZNP_NV_PANID           0x0083
#define ZNP_NV_EXTENDED_PANID  0x002D
#define ZNP_NV_CHANLIST        0x0084
#define ZNP_NV_PRECFGKEY       0x0062

/* ── Network-config buffer (filled by buffered NV writes) ────────────────── */
typedef struct {
    uint16_t pan_id;         bool have_pan_id;
    uint8_t  ext_pan_id[8];  bool have_ext_pan_id;
    uint32_t chan_mask;       bool have_chan_mask;
    uint8_t  nwk_key[16];    bool have_nwk_key;
    uint8_t  logical_type;   bool have_logical_type;
} znp_netcfg_t;

/* Apply one NV write to cfg.  Returns true if accepted (unknown ids silently
 * accepted+ignored).  Guards against short val buffers — never over-reads. */
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

#ifdef __cplusplus
}
#endif
