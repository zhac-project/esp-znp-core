// SPDX-FileCopyrightText: 2025-2026 Evgenij Cjura and project contributors
// SPDX-License-Identifier: AGPL-3.0-or-later
#include "znp_dispatch.h"

/* ── helpers ─────────────────────────────────────────────────────────────── */

static size_t encode_srsp_sub(uint8_t subsys, uint8_t cmd1,
                               const uint8_t *pl, uint8_t pl_len,
                               uint8_t *buf, size_t cap) {
    mt_frame_t f = { MT_SRSP(subsys), cmd1, pl_len, pl };
    return mt_encode(&f, buf, cap);
}

/* def 6 (LOW): the SYS subsystem is by far the most common SRSP
 * source, so keep the terse 4-arg spelling — but route it through the generic
 * encoder rather than a second hardwired copy. */
static size_t encode_srsp(uint8_t cmd1, const uint8_t *pl, uint8_t pl_len,
                          uint8_t *buf, size_t cap) {
    return encode_srsp_sub(ZNP_SYS, cmd1, pl, pl_len, buf, cap);
}

/* def 6: serialize a 64-bit value little-endian (wire order) into out[0..7].
 * Used for the IEEE EUI64 in tc_dev_ind + leave_ind (was duplicated inline). */
static void put_le64(uint8_t *out, uint64_t v) {
    for (int i = 0; i < 8; i++) out[i] = (uint8_t)(v >> (i * 8));
}

/* def 2 (HIGH): MT RPC-error SRSP for an unhandled SREQ.
 * A TI host blocks on every SREQ's SRSP with a timeout; an unrouted SREQ that
 * returns 0 = silence = 2-3 s × N retries of dead air (AF_DATA_REQUEST,
 * MGMT_LEAVE, MSG_CB_REGISTER, the ZDO interview reqs, …). Z-Stack answers the
 * MONITOR-TEST RPC-error frame instead: SRSP of the RPC-subsystem 0 (cmd0=0x60,
 * cmd1=0x00) with payload {errorcode, jcmd0, jcmd1} echoing the offending
 * header. errorcode 0x02 = MT_RPC_ERR_COMMAND_ID (subsystem/cmd not supported).
 * This turns triple-timeout dead-air into one immediate honest error FOR HOSTS
 * THAT CORRELATE IT (generic/herdsman-class); the current znp_driver host
 * matches SRSPs by subsystem+cmd1 and does not yet correlate the 0x60/0x00
 * RPC-error — see CHANGELOG. Honest error until the command is actually
 * implemented (out of scope here). DEFAULT fall-through ONLY — it must never
 * shadow a real handler (incl. the new 0x36). */
#define ZNP_RPC_ERR_INVALID_CMD  0x02   /* MT_RPC_ERR_COMMAND_ID */
static size_t encode_rpc_error(const mt_frame_t *req, uint8_t errcode,
                               uint8_t *buf, size_t cap) {
    const uint8_t pl[3] = { errcode, req->cmd0, req->cmd1 };
    /* cmd0 = MT_SRSP(0) = 0x60 (RPC subsystem 0 / MONITOR-boot res), cmd1 = 0x00 */
    mt_frame_t f = { (uint8_t)0x60, 0x00, 3, pl };
    return mt_encode(&f, buf, cap);
}

/* True when the frame is a synchronous request the host blocks on (SREQ type
 * bits 0x20). Only SREQs warrant an RPC-error reply; an unrouted AREQ is
 * fire-and-forget and must stay silent. */
static bool is_sreq(uint8_t cmd0) { return (cmd0 & 0xE0) == 0x20; }

/* Map a TI Z-Stack BDB mode byte to the ESP-Zigbee (EZB) mode mask.
 * TI FORMATION=0x04 → EZB 0x08, TI STEERING=0x02 → EZB 0x04.
 * Unknown TI bits are silently dropped. */
static uint8_t ti_bdb_to_ezb(uint8_t ti_mode) {
    uint8_t ezb = 0;
    if (ti_mode & ZNP_TI_BDB_FORMATION) ezb |= ZNP_EZB_BDB_FORMATION;
    if (ti_mode & ZNP_TI_BDB_STEERING)  ezb |= ZNP_EZB_BDB_STEERING;
    return ezb;
}

/* Decode a 2-byte little-endian uint16 */
static uint16_t le16(const uint8_t *p) {
    return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

/* Decode a 4-byte little-endian uint32 */
static uint32_t le32(const uint8_t *p) {
    return (uint32_t)(p[0] | ((uint32_t)p[1] << 8) |
                      ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}

/* ── znp_netcfg_apply_nv ─────────────────────────────────────────────────── */

bool znp_netcfg_apply_nv(znp_netcfg_t *cfg, uint16_t id,
                          const uint8_t *val, uint8_t len) {
    /* def 3 (MED): a known id whose value is too short to be a
     * valid setting is REJECTED (return false) rather than silently dropped.
     * The header promised accept/reject; before this fix the function could
     * never return false, so a truncated PRECFGKEY/PANID/etc. was dropped yet
     * the write SRSP still answered 0x00-success and the host formed a wrong
     * network. The caller must surface false as an NV-failure status. */
    switch (id) {
        case ZNP_NV_PANID:
            if (len < 2) return false;   /* reject short val */
            cfg->pan_id = le16(val);
            cfg->have_pan_id = true;
            break;
        case ZNP_NV_EXTENDED_PANID:
            if (len < 8) return false;
            memcpy(cfg->ext_pan_id, val, 8);
            cfg->have_ext_pan_id = true;
            break;
        case ZNP_NV_CHANLIST:
            if (len < 4) return false;
            cfg->chan_mask = le32(val);
            cfg->have_chan_mask = true;
            break;
        case ZNP_NV_PRECFGKEY:
            if (len < 16) return false;
            memcpy(cfg->nwk_key, val, 16);
            cfg->have_nwk_key = true;
            break;
        case ZNP_NV_LOGICAL_TYPE:
            if (len < 1) return false;
            cfg->logical_type = val[0];
            cfg->have_logical_type = true;
            break;
        default:
            /* unknown / untracked id — silently accept */
            break;
    }
    return true;
}

/* ── NV write handler (def 1 + def 3) ─────────────────────────────────────
 * Single entry for both osalNvWrite (0x09) and osalNvWriteExt (0x1D). Captures
 * the STARTUP_OPTION factory-reset request (def 1, CRIT) and stages
 * tracked config via znp_netcfg_apply_nv (def 3 — returns false on reject).
 *
 * Returns true when the write is accepted (so the SRSP can answer success),
 * false when it must be reported as an NV failure. */
static bool handle_nv_write(znp_dispatch_ctx *ctx, uint16_t nv_id,
                            const uint8_t *val, uint8_t len) {
    if (nv_id == ZNP_NV_STARTUP_OPTION) {
        /* CRIT: the host writes STARTUP_OPTION (then SYS_RESET) to demand a
         * blank coordinator. esp_restart() preserves the zb_storage NVRAM, so
         * the old PAN/network-key would resurrect against the new config and the
         * radio could never be factory-reset. When a clear bit is set, latch a
         * persistent factory-new request; znp_ezb.c erases zb_storage BEFORE
         * esp_zigbee_init on the next boot. STARTUP_OPTION=0x00 is a no-op. */
        if (len < 1) return false;             /* malformed → honest failure */
        uint8_t opt = val[0];
        if (opt & (ZNP_STARTOPT_CLEAR_STATE | ZNP_STARTOPT_CLEAR_CONFIG)) {
            if (ctx->be && ctx->be->request_factory_new) {
                ctx->be->request_factory_new();
            }
        }
        return true;   /* STARTUP_OPTION itself is not staged into netcfg */
    }
    return znp_netcfg_apply_nv(&ctx->cfg, nv_id, val, len);
}

/* ── NV SRSP helpers ─────────────────────────────────────────────────────── */

/* NV_WRITE_EXT (0x1D) / NV_WRITE (0x09): SRSP = status(1).
 * def 3: status is now honest — 0x00=success only when the write was actually
 * applied; ZNP_NV_OPER_FAILED (0x0A) when validation/apply rejected it. */
static size_t nv_write_srsp(uint8_t cmd1, uint8_t status, uint8_t *buf, size_t cap) {
    const uint8_t pl[1] = { status };
    return encode_srsp(cmd1, pl, 1, buf, cap);
}

/* NV_ITEM_INIT (0x07): SRSP = status(1), 0x00=success */
static size_t nv_item_init_srsp(uint8_t *buf, size_t cap) {
    const uint8_t pl[1] = { 0x00 };
    return encode_srsp(0x07, pl, 1, buf, cap);
}

/* NV_LENGTH (0x13): SRSP = len(2 LE), return 0 for unknown */
static size_t nv_length_srsp(uint8_t *buf, size_t cap) {
    const uint8_t pl[2] = { 0x00, 0x00 };
    return encode_srsp(0x13, pl, 2, buf, cap);
}

/* NV_DELETE (0x12): SRSP = status(1) */
static size_t nv_delete_srsp(uint8_t *buf, size_t cap) {
    const uint8_t pl[1] = { 0x00 };
    return encode_srsp(0x12, pl, 1, buf, cap);
}

/* NV_READ_EXT (0x1C) / NV_READ (0x08): SRSP = status(1) + len(1) + data[len].
 * def 4 (LOW): TI returns NV_OPER_FAILED for a missing item, not
 * SUCCESS+len0. We serve a readback from the staged cfg for the ids we track
 * (so a foreign host — e.g. z2m — can verify what it wrote) and answer
 * ZNP_NV_ITEM_UNINIT (0x02) for everything else instead of lying SUCCESS. */
static size_t nv_read_srsp(uint8_t cmd1, const znp_netcfg_t *cfg,
                            uint16_t nv_id, uint8_t *buf, size_t cap) {
    uint8_t pl[2 + 16];   /* status + len + up to a 16-byte network key */
    uint8_t dlen = 0;
    bool    have = false;

    if (cfg) {
        switch (nv_id) {
            case ZNP_NV_PANID:
                if (cfg->have_pan_id) {
                    pl[2] = (uint8_t)(cfg->pan_id & 0xFF);
                    pl[3] = (uint8_t)(cfg->pan_id >> 8);
                    dlen = 2; have = true;
                }
                break;
            case ZNP_NV_EXTENDED_PANID:
                if (cfg->have_ext_pan_id) {
                    memcpy(pl + 2, cfg->ext_pan_id, 8); dlen = 8; have = true;
                }
                break;
            case ZNP_NV_CHANLIST:
                if (cfg->have_chan_mask) {
                    pl[2] = (uint8_t)(cfg->chan_mask & 0xFF);
                    pl[3] = (uint8_t)((cfg->chan_mask >> 8) & 0xFF);
                    pl[4] = (uint8_t)((cfg->chan_mask >> 16) & 0xFF);
                    pl[5] = (uint8_t)((cfg->chan_mask >> 24) & 0xFF);
                    dlen = 4; have = true;
                }
                break;
            case ZNP_NV_PRECFGKEY:
                if (cfg->have_nwk_key) {
                    memcpy(pl + 2, cfg->nwk_key, 16); dlen = 16; have = true;
                }
                break;
            case ZNP_NV_LOGICAL_TYPE:
                if (cfg->have_logical_type) {
                    pl[2] = cfg->logical_type; dlen = 1; have = true;
                }
                break;
            default:
                break;
        }
    }

    if (have) {
        pl[0] = ZNP_NV_SUCCESS;
        pl[1] = dlen;
        return encode_srsp(cmd1, pl, (uint8_t)(2 + dlen), buf, cap);
    }
    /* Unknown / un-staged id: report uninitialised, not a false success. */
    pl[0] = ZNP_NV_ITEM_UNINIT;
    pl[1] = 0;
    return encode_srsp(cmd1, pl, 2, buf, cap);
}

/* ── znp_dispatch ────────────────────────────────────────────────────────── */

/* ── ZDO subsystem dispatcher ────────────────────────────────────────────── */

static size_t dispatch_zdo(const mt_frame_t *req, znp_dispatch_ctx *ctx,
                           uint8_t *buf, size_t cap, bool *matched) {
    const znp_backend_t *be = ctx ? ctx->be : NULL;
    *matched = true;   /* cleared in default: so the top level can RPC-error */

    switch (req->cmd1) {
        /* ZDO_STARTUP_FROM_APP (0x40):
         * P4 sends: MT_SREQ(ZNP_ZDO)/0x40, payload={0x00,0x00} (startup delay LE16)
         * We: apply_config → start_stack, then reply the SRSP status.
         *
         * def 4 (MED): TI ZDO_STARTUP_FROM_APP status semantics are
         *   0x00 = RESTORED_NETWORK_STATE
         *   0x01 = NEW_NETWORK_STATE
         *   0x02 = NOT_STARTED  (the failure value)
         * The old code returned 0x01 on failure — but 0x01 is a SUCCESS variant
         * to TI, so a generic host (zigbee-herdsman) read a failed start_stack as
         * "new network started" and proceeded over a dead stack. Now: 0x02 on
         * failure; 0x00 on success (we treat a started stack as restored — the
         * host only branches success-vs-NOT_STARTED). NULL op → success (0x00). */
        case 0x40: {
            uint8_t status = 0x00;   /* success (restored) */
            if (be && be->apply_config) be->apply_config(ctx ? &ctx->cfg : NULL);
            /* T36 / hardening MED (def 4): the network key (PRECFGKEY) was
             * staged into ctx->cfg by an earlier osalNvWrite and has now been
             * consumed by apply_config (the backend copied it into its own
             * pending buffer + handed it to the stack). Zeroize this dispatch-ctx
             * copy (== app_main's s_ctx.cfg) so the cleartext key does not linger
             * in RAM for the process lifetime, and clear the have-flag so a later
             * re-read of NV doesn't echo a wiped key. The backend zeroizes its
             * own copy at its apply site (znp_ezb.c). */
            if (ctx) {
                memset(ctx->cfg.nwk_key, 0, sizeof(ctx->cfg.nwk_key));
                ctx->cfg.have_nwk_key = false;
            }
            if (be && be->start_stack && !be->start_stack()) status = 0x02; /* NOT_STARTED */
            return encode_srsp_sub(ZNP_ZDO, 0x40, &status, 1, buf, cap);
        }

        /* ZDO_EXT_NWK_INFO (0x50):
         * P4 polls this to learn the formed network (zigbee_mgr.cpp:424-430).
         * SRSP payload layout (P4 reads exactly these offsets):
         *   [0..1] = shortaddr LE
         *   [2]    = dev_state  (DEV_ZB_COORD = 0x09)
         *   [3..4] = pan_id LE
         *   [5..6] = zeros (remaining fields, P4 only checks payload_len>=7)
         * Call get_nwk_info if available; fall through with zeros if NULL. */
        case 0x50: {
            uint16_t panid      = 0;
            uint16_t short_addr = 0;
            uint8_t  dev_state  = 0;
            if (be && be->get_nwk_info) be->get_nwk_info(&panid, &short_addr, &dev_state);
            uint8_t pl[7] = {
                (uint8_t)(short_addr & 0xFF),        /* [0] shortaddr low  */
                (uint8_t)(short_addr >> 8),           /* [1] shortaddr high */
                dev_state,                            /* [2] dev_state      */
                (uint8_t)(panid & 0xFF),              /* [3] panid low      */
                (uint8_t)(panid >> 8),                /* [4] panid high     */
                0x00,                                 /* [5] zeros          */
                0x00,                                 /* [6] zeros          */
            };
            return encode_srsp_sub(ZNP_ZDO, 0x50, pl, sizeof(pl), buf, cap);
        }

        /* ZDO_MGMT_PERMIT_JOIN_REQ (0x36) — def 1 (HIGH):
         * pairing was DEAD because 0x36 fell into default:return 0, so the host
         * (zigbee_mgr zcl_commands.cpp:58 zigbee_permit_join) saw NO SRSP and
         * burned 3× the 2000 ms znp_sreq_retry timeout per open-network attempt.
         *
         * Host wire layout (zcl_commands.cpp:62-71, exact bytes):
         *   pl[0]   AddrMode      (0x0F broadcast)
         *   pl[1:2] DstAddr LE    (0xFFFC = routers+coord)
         *   pl[3]   Duration (s)  (0=close, 255=open indefinitely)  ← what we want
         *   pl[4]   TCSignificance(0x01)
         * The host only reads SRSP payload[0] as a status (0x00=success); the
         * Z-Stack MGMT_PERMIT_JOIN_RSP AREQ (0x45/0xB6) is NOT registered by the
         * host (no znp_register_areq for 0xB6), but a faithful TI NCP emits it,
         * so we also queue it defensively — harmless if the host ignores it. */
        case 0x36: {
            uint8_t duration = (req->payload && req->payload_len >= 4)
                               ? req->payload[3] : 0;
            uint8_t status = 0x00;
            if (be && be->permit_join && !be->permit_join(duration)) status = 0x01;
            /* SRSP (the byte the host blocks on) — this alone unblocks pairing. */
            return encode_srsp_sub(ZNP_ZDO, 0x36, &status, 1, buf, cap);
        }

        default:
            *matched = false;
            return 0;
    }
}

/* ── APP_CNF subsystem dispatcher ────────────────────────────────────────── */

static size_t dispatch_app_cnf(const mt_frame_t *req, znp_dispatch_ctx *ctx,
                               uint8_t *buf, size_t cap, bool *matched) {
    const znp_backend_t *be = ctx ? ctx->be : NULL;
    const uint8_t status_ok[1] = { 0x00 };
    *matched = true;

    switch (req->cmd1) {
        /* BDB_SET_CHANNEL (0x08):
         * payload: isPrimary(1) + chan_mask(4 LE).
         * Channel was already captured via NV CHANLIST write; ack only. */
        case 0x08:
            return encode_srsp_sub(ZNP_APP_CNF, 0x08, status_ok, 1, buf, cap);

        /* BDB_START_COMMISSIONING (0x05):
         * payload: 1 byte TI BDB mode (0x04=FORMATION, 0x02=STEERING).
         * Map TI→EZB and call bdb_commission. Status 0x01 on failure,
         * 0x00 on success or NULL op. */
        case 0x05: {
            uint8_t ti_mode = (req->payload && req->payload_len >= 1)
                              ? req->payload[0] : 0;
            uint8_t ezb_mode = ti_bdb_to_ezb(ti_mode);
            uint8_t status = 0x00;
            if (be && be->bdb_commission && !be->bdb_commission(ezb_mode)) status = 0x01;
            return encode_srsp_sub(ZNP_APP_CNF, 0x05, &status, 1, buf, cap);
        }
        default:
            *matched = false;
            return 0;
    }
}

/* ── AF subsystem dispatcher ─────────────────────────────────────────────── */

static size_t dispatch_af(const mt_frame_t *req, znp_dispatch_ctx *ctx,
                          uint8_t *buf, size_t cap, bool *matched) {
    (void)ctx;
    const uint8_t status_ok[1] = { 0x00 };
    *matched = true;

    switch (req->cmd1) {
        /* AF_REGISTER (0x00):
         * P4 sends: endpoint(1) + profile(2) + device_id(2) + version(1) +
         *           latency(1) + in_cluster_count(1) + out_cluster_count(1) + clusters
         * (zigbee_mgr.cpp:895: MT_SREQ(ZNP_AF)/0x00)
         * SRSP: 1-byte status=0x00 (success). NCP registers real endpoint at stack
         * start (Task 4.5); here just ack. status 0xB8=ZApsDuplicateEntry is also
         * OK in real use but we always return 0x00 from the NCP side. */
        case 0x00:
            return encode_srsp_sub(ZNP_AF, 0x00, status_ok, 1, buf, cap);

        default:
            *matched = false;
            return 0;
    }
}

/* ── SYS subsystem dispatcher ────────────────────────────────────────────── */

static size_t dispatch_sys(const mt_frame_t *req, znp_dispatch_ctx *ctx,
                           uint8_t *buf, size_t cap, bool *matched) {
    const znp_backend_t *be = ctx ? ctx->be : NULL;
    *matched = true;

    switch (req->cmd1) {

        /* ── existing commands ───────────────────────────────────────────── */
        case 0x01: {   /* SYS_PING -> 2B capability bitmap (LE) */
            const uint8_t pl[2] = { (uint8_t)(ZNP_PING_CAPS & 0xFF),
                                    (uint8_t)(ZNP_PING_CAPS >> 8) };
            return encode_srsp(0x01, pl, 2, buf, cap);
        }
        case 0x02: {   /* SYS_VERSION */
            const uint8_t pl[5] = { ZNP_TRANSPORT_REV, ZNP_PRODUCT_ID,
                                    ZNP_VER_MAJOR, ZNP_VER_MINOR, ZNP_VER_MAINT };
            return encode_srsp(0x02, pl, 5, buf, cap);
        }
        case 0x04: {   /* SYS_GET_EXTADDR -> 8B IEEE (LE) */
            uint8_t ieee[8] = {0};
            if (be && be->get_ieee) be->get_ieee(ieee);
            return encode_srsp(0x04, ieee, 8, buf, cap);
        }
        case 0x00:     /* SYS_RESET_REQ: reset, no SRSP (host waits for RESET_IND) */
            if (be && be->request_reset) be->request_reset();
            return 0;

        /* ── NV commands ─────────────────────────────────────────────────── */

        /* osalNvWriteExt (SYS 0x1D):
         * payload = id(2 LE) + offset(2 LE) + len(2 LE) + data[len]
         * Mirrors nv_write_raw() in zigbee_mgr.cpp exactly. */
        case 0x1D: {
            const uint8_t *pl = req->payload;
            uint8_t plen = req->payload_len;
            /* def 3: assume failure until a write actually lands. A malformed or
             * truncated frame must NOT answer 0x00-success (the host would then
             * believe PAN/key/channel were stored and form a wrong network). */
            uint8_t status = ZNP_NV_OPER_FAILED;
            if (ctx && plen >= 6) {
                uint16_t nv_id  = le16(pl);
                /* offset at pl[2..3] — we ignore offset (always 0 from P4) */
                uint16_t dlen16 = le16(pl + 4);
                /* hardening: 128 B clamp. Every NV item the P4 writes
                 * (network key 16 B, PAN id, channel, ...) is far below this, so
                 * the clamp never truncates a real write — it only bounds the
                 * copy into znp_netcfg_apply_nv. A >128 B osalNvWriteExt (which
                 * Z-Stack would accept) is silently truncated: an intentional,
                 * documented divergence for this NCP, not a Z-Stack-faithful path. */
                uint8_t  dlen   = (dlen16 > 128) ? 128 : (uint8_t)dlen16;
                /* def 2 (HIGH): size_t arithmetic — never let
                 * (uint8_t)(6 + dlen) wrap for dlen near 0xFF and pass the guard
                 * while apply_nv over-reads the payload. (Here dlen is already
                 * clamped to ≤128 so the cast was accidentally safe; size_t makes
                 * it robust regardless of the clamp.) */
                if ((size_t)plen >= 6u + (size_t)dlen) {
                    status = handle_nv_write(ctx, nv_id, pl + 6, dlen)
                             ? ZNP_NV_SUCCESS : ZNP_NV_OPER_FAILED;
                }
            }
            return nv_write_srsp(0x1D, status, buf, cap);
        }

        /* osalNvWrite (SYS 0x09):
         * payload = id(2 LE) + offset(2 LE) + len(1) + data[len]
         * Shorter legacy variant (1-byte len). */
        case 0x09: {
            const uint8_t *pl = req->payload;
            uint8_t plen = req->payload_len;
            uint8_t status = ZNP_NV_OPER_FAILED;   /* def 3: fail unless applied */
            if (ctx && plen >= 5) {
                uint16_t nv_id = le16(pl);
                uint8_t  dlen  = pl[4];
                /* def 2 (HIGH): dlen=pl[4] can be up to 0xFF. The old
                 * guard `plen >= (uint8_t)(5 + dlen)` wrapped to ≤4 for dlen in
                 * [251..255], passing the check and letting apply_nv over-read up
                 * to 255 B from a ≤245 B payload (stale parser-buffer bytes into
                 * nwk_key/PAN/cfg with a SUCCESS SRSP). size_t arithmetic cannot
                 * wrap, so the guard now holds for every dlen. */
                if ((size_t)plen >= 5u + (size_t)dlen) {
                    status = handle_nv_write(ctx, nv_id, pl + 5, dlen)
                             ? ZNP_NV_SUCCESS : ZNP_NV_OPER_FAILED;
                }
            }
            return nv_write_srsp(0x09, status, buf, cap);
        }

        /* osalNvItemInit (SYS 0x07):
         * payload = id(2) + itemlen(2) + initlen(1) + init[initlen]
         * Always reply success (0x00). */
        case 0x07:
            return nv_item_init_srsp(buf, cap);

        /* osalNvLength (SYS 0x13): payload = id(2). Reply len=0 (unknown). */
        case 0x13:
            return nv_length_srsp(buf, cap);

        /* osalNvDelete (SYS 0x12): payload = id(2). Reply success. */
        case 0x12:
            return nv_delete_srsp(buf, cap);

        /* osalNvReadExt (SYS 0x1C):
         * payload = id(2 LE) + offset(2 LE). Reply status=0 + len=0. */
        case 0x1C: {
            uint16_t nv_id = (req->payload_len >= 2)
                             ? le16(req->payload) : 0;
            return nv_read_srsp(0x1C, ctx ? &ctx->cfg : NULL, nv_id, buf, cap);
        }

        /* osalNvRead (SYS 0x08):
         * payload = id(2 LE) + offset(2 LE). Reply status=0 + len=0. */
        case 0x08: {
            uint16_t nv_id = (req->payload_len >= 2)
                             ? le16(req->payload) : 0;
            return nv_read_srsp(0x08, ctx ? &ctx->cfg : NULL, nv_id, buf, cap);
        }

        default:
            *matched = false;
            return 0;  /* unhandled SYS command */
    }
}

/* ── znp_dispatch (top-level router) ─────────────────────────────────────── */

size_t znp_dispatch(const mt_frame_t *req, znp_dispatch_ctx *ctx,
                    uint8_t *buf, size_t cap) {
    const znp_backend_t *be = ctx ? ctx->be : NULL;

    /* def 3 (MED): AREQ-typed SYS_RESET_REQ routing.
     * The current host sends reset as SREQ (zigbee_mgr.cpp:307, MT_SREQ(ZNP_SYS)
     * /0x00) — handled below in dispatch_sys. But real Z-Stack hosts and
     * zigbee-herdsman send SYS_RESET_REQ as an AREQ (0x41/0x00), which the old
     * SREQ-only routing dropped → the chip never reset. Route the AREQ form to
     * the SAME request_reset path (T33). No SRSP either way: the host waits for
     * the RESET_IND that follows on boot. */
    if (req->cmd0 == MT_AREQ(ZNP_SYS) && req->cmd1 == 0x00) {
        if (be && be->request_reset) be->request_reset();
        return 0;
    }

    /* Route SREQs by subsystem. matched=false means the subsystem dispatcher had
     * no handler for this cmd1 → fall through to the RPC-error (def 2). */
    bool matched = false;
    size_t n = 0;
    if (req->cmd0 == MT_SREQ(ZNP_ZDO)) {
        n = dispatch_zdo(req, ctx, buf, cap, &matched);
    } else if (req->cmd0 == MT_SREQ(ZNP_AF)) {
        n = dispatch_af(req, ctx, buf, cap, &matched);
    } else if (req->cmd0 == MT_SREQ(ZNP_APP_CNF)) {
        n = dispatch_app_cnf(req, ctx, buf, cap, &matched);
    } else if (req->cmd0 == MT_SREQ(ZNP_SYS)) {
        n = dispatch_sys(req, ctx, buf, cap, &matched);
    }

    if (matched) return n;   /* a real handler ran (incl. silent SYS_RESET) */

    /* def 2 (HIGH): no handler. For a SREQ the host is blocking on
     * an SRSP — answer the MT RPC-error frame so it fails immediately instead of
     * burning N × the sreq timeout in dead air. Unrouted AREQs stay silent. */
    if (is_sreq(req->cmd0)) {
        return encode_rpc_error(req, ZNP_RPC_ERR_INVALID_CMD, buf, cap);
    }
    return 0;
}

size_t znp_build_reset_ind(uint8_t reason, uint8_t *buf, size_t cap) {
    const uint8_t pl[6] = { reason, ZNP_TRANSPORT_REV, ZNP_PRODUCT_ID,
                            ZNP_VER_MAJOR, ZNP_VER_MINOR, ZNP_VER_MAINT };
    mt_frame_t f = { MT_AREQ(ZNP_SYS), 0x80, 6, pl };
    return mt_encode(&f, buf, cap);
}

/* ── Task 4.4: unsolicited AREQ builders ────────────────────────────────── */

/* ZDO_STATE_CHANGE_IND  AREQ 0x45/0xC0
 * Payload: 1 byte ZDO state.
 * P4 reads: state = f.payload[0]  (zigbee_mgr.cpp:82). */
size_t znp_build_state_change_ind(uint8_t dev_state, uint8_t *buf, size_t cap) {
    const uint8_t pl[1] = { dev_state };
    mt_frame_t f = { MT_AREQ(ZNP_ZDO), 0xC0, 1, pl };
    return mt_encode(&f, buf, cap);
}

/* ZDO_TC_DEV_IND  AREQ 0x45/0xCA
 * Payload: nwk(2 LE) + ieee(8 LE) + capabilities(1) = 11 bytes.
 * P4 reads: ev.nwk  = le16(payload[0..1])
 *           ev.ieee = le64(payload[2..9])
 * (zigbee_interview.cpp:590-592, payload_len<11 guard). */
size_t znp_build_tc_dev_ind(uint16_t nwk_addr, uint64_t ieee,
                             uint8_t capabilities,
                             uint8_t *buf, size_t cap) {
    uint8_t pl[11];
    pl[0] = (uint8_t)(nwk_addr & 0xFF);
    pl[1] = (uint8_t)(nwk_addr >> 8);
    put_le64(pl + 2, ieee);   /* IEEE 8 bytes little-endian (def 6 helper) */
    pl[10] = capabilities;
    mt_frame_t f = { MT_AREQ(ZNP_ZDO), 0xCA, 11, pl };
    return mt_encode(&f, buf, cap);
}

/* ZDO_LEAVE_IND  AREQ 0x45/0xC9
 * Payload: nwk(2 LE) + ieee(8 LE) + remove(1) + rejoin(1) = 12 bytes.
 * P4 reads: ieee = le64(payload[2..9])  (zigbee_mgr.cpp:761).
 * payload_len<10 guard (nwk+ieee mandatory; remove/rejoin always present). */
size_t znp_build_leave_ind(uint16_t src_addr, uint64_t ieee,
                            uint8_t remove, uint8_t rejoin,
                            uint8_t *buf, size_t cap) {
    uint8_t pl[12];
    pl[0] = (uint8_t)(src_addr & 0xFF);
    pl[1] = (uint8_t)(src_addr >> 8);
    put_le64(pl + 2, ieee);   /* IEEE 8 bytes little-endian (def 6 helper) */
    pl[10] = remove;
    pl[11] = rejoin;
    mt_frame_t f = { MT_AREQ(ZNP_ZDO), 0xC9, 12, pl };
    return mt_encode(&f, buf, cap);
}

/* Phase 5: ZDO ACTIVE_EP_RSP (0x85). See znp_dispatch.h. */
size_t znp_build_active_ep_rsp(uint16_t nwk, uint8_t status,
                               const uint8_t *ep_list, uint8_t ep_count,
                               uint8_t *buf, size_t cap) {
    /* A device can advertise at most 32 endpoints (1..240 valid, but the pool
     * caps far below); bound the local payload buffer regardless of the count
     * the stack hands us. */
    if (ep_count > 32) ep_count = 32;
    uint8_t pl[6 + 32];
    pl[0] = (uint8_t)(nwk & 0xFF);   /* SrcAddr LE  */
    pl[1] = (uint8_t)(nwk >> 8);
    pl[2] = status;
    pl[3] = (uint8_t)(nwk & 0xFF);   /* NWKAddr LE (== SrcAddr for a device rsp) */
    pl[4] = (uint8_t)(nwk >> 8);
    pl[5] = ep_count;
    if (ep_count && ep_list) memcpy(pl + 6, ep_list, ep_count);
    mt_frame_t f = { MT_AREQ(ZNP_ZDO), 0x85, (uint8_t)(6 + ep_count), pl };
    return mt_encode(&f, buf, cap);
}

/* ZDO_MGMT_PERMIT_JOIN_RSP  AREQ 0x45/0xB6  (def 1 — companion to case 0x36)
 * A faithful TI Z-Stack NCP follows the 0x36 SRSP with this unsolicited RSP.
 * Payload: srcaddr(2 LE) + status(1) = 3 bytes. The current host does not
 * register an 0xB6 handler, so this is emitted defensively for generic hosts
 * (herdsman) that do; it never replaces the SRSP that actually unblocks
 * zigbee_permit_join. The backend's permit_join path may call this then
 * znp_uart_send_raw(), mirroring the other unsolicited AREQ builders. */
size_t znp_build_permit_join_rsp(uint16_t src_addr, uint8_t status,
                                 uint8_t *buf, size_t cap) {
    const uint8_t pl[3] = {
        (uint8_t)(src_addr & 0xFF),
        (uint8_t)(src_addr >> 8),
        status,
    };
    mt_frame_t f = { MT_AREQ(ZNP_ZDO), 0xB6, 3, pl };
    return mt_encode(&f, buf, cap);
}
