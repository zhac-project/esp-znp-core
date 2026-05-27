#pragma once
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "mt_proto.h"
#ifdef __cplusplus
extern "C" {
#endif

/* Called from the UART RX task when a complete, FCS-valid MT frame arrives.
 * frame->payload aliases an internal parser buffer (see mt_parser_feed lifetime
 * rule): consume/copy synchronously inside the callback. */
typedef void (*znp_frame_cb_t)(const mt_frame_t *frame);

/* Configure the UART (115200 8N1, no flow control, pins from Kconfig) and spawn
 * the RX task. cb is invoked per received frame. Call once at startup. */
void znp_uart_init(znp_frame_cb_t cb);

/* Write already-encoded MT bytes to the UART. Returns true if all bytes queued.
 * (Encoding lives in znp_dispatch so the wire output stays host-testable.) */
bool znp_uart_send_raw(const uint8_t *buf, size_t len);

#ifdef __cplusplus
}
#endif
