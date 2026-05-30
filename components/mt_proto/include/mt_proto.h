// SPDX-FileCopyrightText: 2025-2026 Evgenij Cjura and project contributors
// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include <stdint.h>
#include <stddef.h>
#ifdef __cplusplus
extern "C" {
#endif

#define MT_SOF          0xFE
#define MT_MAX_PAYLOAD  250
#define MT_OVERHEAD     5     /* SOF + LEN + CMD0 + CMD1 + FCS */

/* type|subsystem helpers — identical to znp_driver */
#define MT_SREQ(sub)  ((uint8_t)(0x20 | (sub)))
#define MT_AREQ(sub)  ((uint8_t)(0x40 | (sub)))
#define MT_SRSP(sub)  ((uint8_t)(0x60 | (sub)))
#define ZNP_SYS       0x01
#define ZNP_AF        0x04
#define ZNP_ZDO       0x05
#define ZNP_UTIL      0x07
#define ZNP_APP_CNF   0x0F

typedef struct {
    uint8_t        cmd0;
    uint8_t        cmd1;
    uint8_t        payload_len;
    const uint8_t *payload;       /* non-owning */
} mt_frame_t;

typedef enum {
    MT_DECODE_OK = 0,
    MT_DECODE_BAD_SOF,
    MT_DECODE_FCS_ERROR,
    MT_DECODE_TRUNCATED,
    MT_DECODE_OVERFLOW,
} mt_decode_result_t;

/* Compute the MT frame check sequence: len ^ cmd0 ^ cmd1 ^ all payload bytes.
 * `payload` may be NULL only when `payload_len == 0`. */
uint8_t mt_fcs(uint8_t len, uint8_t cmd0, uint8_t cmd1,
               const uint8_t *payload, uint8_t payload_len);

/* Encode frame into buf. Returns total bytes written, or 0 on overflow/too-small buf. */
size_t  mt_encode(const mt_frame_t *f, uint8_t *buf, size_t buf_size);

/* Decode one complete buffer (buf[0]==SOF .. includes trailing FCS).
 * On OK, out->payload points into buf+4. */
mt_decode_result_t mt_decode(const uint8_t *buf, size_t len, mt_frame_t *out);

typedef struct {
    uint8_t state;                  /* 0=SOF 1=LEN 2=CMD0 3=CMD1 4=DATA 5=FCS */
    uint8_t len;
    uint8_t cmd0;
    uint8_t cmd1;
    uint8_t idx;
    uint8_t data[MT_MAX_PAYLOAD];
} mt_parser_t;

void mt_parser_reset(mt_parser_t *p);
/* Feed one byte. Returns 1 and fills *out when a complete, FCS-valid frame is
 * received; returns 0 otherwise (incl. FCS mismatch -> resets to SOF-hunt).
 * LIFETIME: out->payload aliases p->data and is valid only until the next call
 * to mt_parser_feed() or mt_parser_reset(). Copy the payload bytes before the
 * next feed if you need them to outlive this call. */
int  mt_parser_feed(mt_parser_t *p, uint8_t b, mt_frame_t *out);

#ifdef __cplusplus
}
#endif
