// SPDX-FileCopyrightText: 2025-2026 Evgenij Cjura and project contributors
// SPDX-License-Identifier: AGPL-3.0-or-later
#include "mt_proto.h"

uint8_t mt_fcs(uint8_t len, uint8_t cmd0, uint8_t cmd1,
               const uint8_t *payload, uint8_t payload_len) {
    uint8_t f = (uint8_t)(len ^ cmd0 ^ cmd1);
    for (uint8_t i = 0; i < payload_len; i++) f ^= payload[i];
    return f;
}

size_t mt_encode(const mt_frame_t *f, uint8_t *buf, size_t buf_size) {
    if (f->payload_len > MT_MAX_PAYLOAD) return 0;
    const size_t total = (size_t)MT_OVERHEAD + f->payload_len;
    if (buf_size < total) return 0;
    buf[0] = MT_SOF;
    buf[1] = f->payload_len;
    buf[2] = f->cmd0;
    buf[3] = f->cmd1;
    for (uint8_t i = 0; i < f->payload_len; i++) buf[4 + i] = f->payload[i];
    buf[4 + f->payload_len] = mt_fcs(f->payload_len, f->cmd0, f->cmd1,
                                     f->payload, f->payload_len);
    return total;
}

mt_decode_result_t mt_decode(const uint8_t *buf, size_t len, mt_frame_t *out) {
    if (len < MT_OVERHEAD)                  return MT_DECODE_TRUNCATED;
    if (buf[0] != MT_SOF)                   return MT_DECODE_BAD_SOF;
    const uint8_t plen = buf[1];
    if (plen > MT_MAX_PAYLOAD)              return MT_DECODE_OVERFLOW;
    if (len < (size_t)MT_OVERHEAD + plen)   return MT_DECODE_TRUNCATED;
    const uint8_t expect = mt_fcs(plen, buf[2], buf[3], buf + 4, plen);
    if (expect != buf[4 + plen])            return MT_DECODE_FCS_ERROR;
    out->cmd0        = buf[2];
    out->cmd1        = buf[3];
    out->payload_len = plen;
    out->payload     = buf + 4;
    return MT_DECODE_OK;
}

void mt_parser_reset(mt_parser_t *p) {
    p->state = 0; p->len = 0; p->idx = 0;
}

int mt_parser_feed(mt_parser_t *p, uint8_t b, mt_frame_t *out) {
    switch (p->state) {
        case 0: if (b == MT_SOF) p->state = 1; return 0;            /* SOF */
        case 1:                                                     /* LEN */
            p->len = b;
            if (p->len > MT_MAX_PAYLOAD) { mt_parser_reset(p); return 0; }
            p->state = 2; return 0;
        case 2: p->cmd0 = b; p->state = 3; return 0;                /* CMD0 */
        case 3: p->cmd1 = b; p->idx = 0; p->state = (p->len ? 4 : 5); return 0; /* CMD1 */
        case 4:                                                     /* DATA */
            p->data[p->idx++] = b;
            if (p->idx >= p->len) p->state = 5;
            return 0;
        case 5: {                                                   /* FCS */
            const uint8_t expect = mt_fcs(p->len, p->cmd0, p->cmd1, p->data, p->len);
            const int ok = (expect == b);
            if (ok && out) {
                out->cmd0 = p->cmd0; out->cmd1 = p->cmd1;
                out->payload_len = p->len; out->payload = p->data;
            }
            mt_parser_reset(p);
            return ok ? 1 : 0;
        }
        default: mt_parser_reset(p); return 0;
    }
}
