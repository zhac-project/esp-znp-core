#include "znp_dispatch.h"

static size_t encode_srsp(uint8_t cmd1, const uint8_t *pl, uint8_t pl_len,
                          uint8_t *buf, size_t cap) {
    mt_frame_t f = { MT_SRSP(ZNP_SYS), cmd1, pl_len, pl };
    return mt_encode(&f, buf, cap);
}

size_t znp_dispatch(const mt_frame_t *req, const znp_backend_t *be,
                    uint8_t *buf, size_t cap) {
    if (req->cmd0 != MT_SREQ(ZNP_SYS)) return 0;   /* only SYS this phase */
    switch (req->cmd1) {
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
        default:
            return 0;  /* unhandled SYS command */
    }
}

size_t znp_build_reset_ind(uint8_t reason, uint8_t *buf, size_t cap) {
    (void)reason; (void)buf; (void)cap; return 0;   /* implemented in Task 3.2 */
}
