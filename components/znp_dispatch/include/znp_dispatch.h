#pragma once
#include <stddef.h>
#include <stdint.h>
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

/* Thin seam between pure protocol logic and chip-specific actions, so the
 * dispatcher is host-testable with a fake backend. */
typedef struct {
    void (*get_ieee)(uint8_t out[8]);   /* 802.15.4 EUI64, little-endian (wire order) */
    void (*request_reset)(void);        /* trigger a reset; RESET_IND follows on boot */
} znp_backend_t;

/* Handle one received request frame. If a SRSP must be sent, encodes the full
 * MT response into buf and returns its byte length (>0). Returns 0 when there is
 * no response to send (AREQ-style side-effects like RESET_REQ, or unhandled cmd). */
size_t znp_dispatch(const mt_frame_t *req, const znp_backend_t *be,
                    uint8_t *buf, size_t cap);

/* Build a SYS_RESET_IND (AREQ 0x41/0x80) into buf. reason 0x00 = power-up.
 * Returns encoded length. */
size_t znp_build_reset_ind(uint8_t reason, uint8_t *buf, size_t cap);

#ifdef __cplusplus
}
#endif
