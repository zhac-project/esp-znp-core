#include "znp_dispatch.h"

/* ── helpers ─────────────────────────────────────────────────────────────── */

static size_t encode_srsp(uint8_t cmd1, const uint8_t *pl, uint8_t pl_len,
                          uint8_t *buf, size_t cap) {
    mt_frame_t f = { MT_SRSP(ZNP_SYS), cmd1, pl_len, pl };
    return mt_encode(&f, buf, cap);
}

static size_t encode_srsp_sub(uint8_t subsys, uint8_t cmd1,
                               const uint8_t *pl, uint8_t pl_len,
                               uint8_t *buf, size_t cap) {
    mt_frame_t f = { MT_SRSP(subsys), cmd1, pl_len, pl };
    return mt_encode(&f, buf, cap);
}

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
    switch (id) {
        case ZNP_NV_PANID:
            if (len < 2) return true;   /* guard short val */
            cfg->pan_id = le16(val);
            cfg->have_pan_id = true;
            break;
        case ZNP_NV_EXTENDED_PANID:
            if (len < 8) return true;
            memcpy(cfg->ext_pan_id, val, 8);
            cfg->have_ext_pan_id = true;
            break;
        case ZNP_NV_CHANLIST:
            if (len < 4) return true;
            cfg->chan_mask = le32(val);
            cfg->have_chan_mask = true;
            break;
        case ZNP_NV_PRECFGKEY:
            if (len < 16) return true;
            memcpy(cfg->nwk_key, val, 16);
            cfg->have_nwk_key = true;
            break;
        case ZNP_NV_LOGICAL_TYPE:
            if (len < 1) return true;
            cfg->logical_type = val[0];
            cfg->have_logical_type = true;
            break;
        default:
            /* unknown / untracked id — silently accept */
            break;
    }
    return true;
}

/* ── NV SRSP helpers ─────────────────────────────────────────────────────── */

/* NV_WRITE_EXT (0x1D) / NV_WRITE (0x09): SRSP = status(1), 0x00=success */
static size_t nv_write_srsp(uint8_t cmd1, uint8_t *buf, size_t cap) {
    const uint8_t pl[1] = { 0x00 };
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

/* NV_READ_EXT (0x1C) / NV_READ (0x08): SRSP = status(1) + len(1) + data[len]
 * We return zeros for the data since we buffer writes but don't expose readback. */
static size_t nv_read_srsp(uint8_t cmd1, const znp_netcfg_t *cfg,
                            uint16_t nv_id, uint8_t *buf, size_t cap) {
    (void)cfg; (void)nv_id;   /* readback of arbitrary items not needed yet */
    /* status=0x00, len=0, no data */
    const uint8_t pl[2] = { 0x00, 0x00 };
    return encode_srsp(cmd1, pl, 2, buf, cap);
}

/* ── znp_dispatch ────────────────────────────────────────────────────────── */

/* ── ZDO subsystem dispatcher ────────────────────────────────────────────── */

static size_t dispatch_zdo(const mt_frame_t *req, znp_dispatch_ctx *ctx,
                           uint8_t *buf, size_t cap) {
    const znp_backend_t *be = ctx ? ctx->be : NULL;
    const uint8_t status_ok[1] = { 0x00 };

    switch (req->cmd1) {
        /* ZDO_STARTUP_FROM_APP (0x40):
         * P4 sends: MT_SREQ(ZNP_ZDO)/0x40, payload={0x00,0x00} (startup delay LE16)
         * We: apply_config → start_stack, reply SRSP status 0x00 on success,
         * 0x01 on failure. NULL backend op → treat as success. */
        case 0x40: {
            uint8_t status = 0x00;
            if (be && be->apply_config) be->apply_config(ctx ? &ctx->cfg : NULL);
            if (be && be->start_stack && !be->start_stack()) status = 0x01;
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

        default:
            return 0;
    }
}

/* ── APP_CNF subsystem dispatcher ────────────────────────────────────────── */

static size_t dispatch_app_cnf(const mt_frame_t *req, znp_dispatch_ctx *ctx,
                               uint8_t *buf, size_t cap) {
    const znp_backend_t *be = ctx ? ctx->be : NULL;
    const uint8_t status_ok[1] = { 0x00 };

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
            return 0;
    }
}

/* ── AF subsystem dispatcher ─────────────────────────────────────────────── */

static size_t dispatch_af(const mt_frame_t *req, znp_dispatch_ctx *ctx,
                          uint8_t *buf, size_t cap) {
    (void)ctx;
    const uint8_t status_ok[1] = { 0x00 };

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
            return 0;
    }
}

/* ── znp_dispatch ────────────────────────────────────────────────────────── */

size_t znp_dispatch(const mt_frame_t *req, znp_dispatch_ctx *ctx,
                    uint8_t *buf, size_t cap) {
    const znp_backend_t *be = ctx ? ctx->be : NULL;

    /* Route by subsystem */
    if (req->cmd0 == MT_SREQ(ZNP_ZDO))     return dispatch_zdo(req, ctx, buf, cap);
    if (req->cmd0 == MT_SREQ(ZNP_AF))      return dispatch_af(req, ctx, buf, cap);
    if (req->cmd0 == MT_SREQ(ZNP_APP_CNF)) return dispatch_app_cnf(req, ctx, buf, cap);
    if (req->cmd0 != MT_SREQ(ZNP_SYS))     return 0;   /* unhandled subsystem */

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
            if (ctx && plen >= 6) {
                uint16_t nv_id  = le16(pl);
                /* offset at pl[2..3] — we ignore offset (always 0 from P4) */
                uint16_t dlen16 = le16(pl + 4);
                uint8_t  dlen   = (dlen16 > 128) ? 128 : (uint8_t)dlen16;
                if (plen >= (uint8_t)(6 + dlen)) {
                    znp_netcfg_apply_nv(&ctx->cfg, nv_id, pl + 6, dlen);
                }
            }
            return nv_write_srsp(0x1D, buf, cap);
        }

        /* osalNvWrite (SYS 0x09):
         * payload = id(2 LE) + offset(2 LE) + len(1) + data[len]
         * Shorter legacy variant (1-byte len). */
        case 0x09: {
            const uint8_t *pl = req->payload;
            uint8_t plen = req->payload_len;
            if (ctx && plen >= 5) {
                uint16_t nv_id = le16(pl);
                uint8_t  dlen  = pl[4];
                if (plen >= (uint8_t)(5 + dlen)) {
                    znp_netcfg_apply_nv(&ctx->cfg, nv_id, pl + 5, dlen);
                }
            }
            return nv_write_srsp(0x09, buf, cap);
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
            return 0;  /* unhandled SYS command */
    }
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
    /* IEEE 8 bytes little-endian */
    pl[2] = (uint8_t)(ieee);
    pl[3] = (uint8_t)(ieee >> 8);
    pl[4] = (uint8_t)(ieee >> 16);
    pl[5] = (uint8_t)(ieee >> 24);
    pl[6] = (uint8_t)(ieee >> 32);
    pl[7] = (uint8_t)(ieee >> 40);
    pl[8] = (uint8_t)(ieee >> 48);
    pl[9] = (uint8_t)(ieee >> 56);
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
    /* IEEE 8 bytes little-endian */
    pl[2] = (uint8_t)(ieee);
    pl[3] = (uint8_t)(ieee >> 8);
    pl[4] = (uint8_t)(ieee >> 16);
    pl[5] = (uint8_t)(ieee >> 24);
    pl[6] = (uint8_t)(ieee >> 32);
    pl[7] = (uint8_t)(ieee >> 40);
    pl[8] = (uint8_t)(ieee >> 48);
    pl[9] = (uint8_t)(ieee >> 56);
    pl[10] = remove;
    pl[11] = rejoin;
    mt_frame_t f = { MT_AREQ(ZNP_ZDO), 0xC9, 12, pl };
    return mt_encode(&f, buf, cap);
}
