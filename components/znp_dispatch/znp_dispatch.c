#include "znp_dispatch.h"

/* ── helpers ─────────────────────────────────────────────────────────────── */

static size_t encode_srsp(uint8_t cmd1, const uint8_t *pl, uint8_t pl_len,
                          uint8_t *buf, size_t cap) {
    mt_frame_t f = { MT_SRSP(ZNP_SYS), cmd1, pl_len, pl };
    return mt_encode(&f, buf, cap);
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

size_t znp_dispatch(const mt_frame_t *req, znp_dispatch_ctx *ctx,
                    uint8_t *buf, size_t cap) {
    if (req->cmd0 != MT_SREQ(ZNP_SYS)) return 0;   /* only SYS this phase */

    const znp_backend_t *be = ctx ? ctx->be : NULL;

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
