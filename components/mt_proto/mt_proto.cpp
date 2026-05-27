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

mt_decode_result_t mt_decode(const uint8_t *, size_t, mt_frame_t *) {
    return MT_DECODE_TRUNCATED;   /* implemented in a later task */
}
