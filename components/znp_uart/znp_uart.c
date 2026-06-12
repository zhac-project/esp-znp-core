// SPDX-FileCopyrightText: 2025-2026 Evgenij Cjura and project contributors
// SPDX-License-Identifier: AGPL-3.0-or-later
#include "znp_uart.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include <string.h>

static const char *TAG = "znp_uart";
static const uart_port_t PORT = CONFIG_ZNP_UART_PORT;
static znp_frame_cb_t s_cb = NULL;
static SemaphoreHandle_t s_tx_mutex = NULL;
static QueueHandle_t s_uart_q = NULL;   /* F32: UART event queue (overrun detection) */

#define ZNP_UART_RX_BUF 512
/* F32 (FINDINGS.md): bounded TX so a stalled host (not draining our TX FIFO)
 * can't pin the Zigbee mainloop forever on the mutex. */
#define ZNP_UART_TX_TIMEOUT_MS 200

static void rx_task(void *arg);

/* P6-T37 (FINDINGS §12 def 3): SINGLETON CONTRACT.
 * This driver is intentionally a process-wide singleton with no deinit. The
 * co-processor firmware brings the UART up exactly once at boot (from app_main)
 * and keeps it for the lifetime of the device; nothing ever tears it down, so a
 * znp_uart_deinit (vTaskDelete + uart_driver_delete + vSemaphoreDelete) would be
 * dead machinery that only adds teardown-race surface. We therefore DOCUMENT the
 * singleton rather than build deinit, and guard against accidental double-init
 * below — a second znp_uart_init() would leak the TX mutex, spawn a second RX
 * task racing the first on the same event queue, and re-install the driver.
 * The guard is the TX mutex handle: it is NULL only before the first init. */
static bool s_inited = false;

void znp_uart_init(znp_frame_cb_t cb) {
    if (s_inited) {
        ESP_LOGW(TAG, "znp_uart_init called twice — ignoring (singleton, init-once)");
        return;
    }
    s_inited = true;
    s_cb = cb;
    const uart_config_t cfg = {
        .baud_rate = CONFIG_ZNP_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    /* F32: install WITH an event queue (20 slots) so the RX task can observe
     * UART_FIFO_OVF / UART_BUFFER_FULL and resync, instead of silently dropping
     * bytes at 115200 with no flow control. */
    ESP_ERROR_CHECK(uart_driver_install(PORT, ZNP_UART_RX_BUF, ZNP_UART_RX_BUF, 20, &s_uart_q, 0));
    ESP_ERROR_CHECK(uart_param_config(PORT, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(PORT, CONFIG_ZNP_UART_TX_GPIO, CONFIG_ZNP_UART_RX_GPIO,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    s_tx_mutex = xSemaphoreCreateMutex();
    /* P6-T35 (FINDINGS §12 HIGH, def 3): RX task stack raised 4096 → 8192 B.
     * The RX task is TWDT-subscribed and was previously running the full
     * dispatch chain incl. the ZBOSS stack bring-up on a thin 4 KB margin;
     * bring-up is now off this task (see znp_ezb.c ezb_worker_task), but the
     * dispatch + MT builders still need comfortable headroom, so keep ≥8 KB.
     * Also: the create return is now CHECKED — an ignored failure here left a
     * silently RX-dead NCP (host sees no SRSP, commissioning never starts). */
    BaseType_t rc = xTaskCreate(rx_task, "znp_uart_rx", 8192, NULL, 10, NULL);
    if (rc != pdPASS) {
        ESP_LOGE(TAG, "xTaskCreate(znp_uart_rx) failed: %d — NCP RX is DEAD", (int)rc);
        /* P6-T37: clear the singleton flag so a caller may retry init and get a
         * working RX task (the driver+mutex are already up; the retry guard
         * above tolerates the already-installed driver via this reset path). */
        s_inited = false;
        return;
    }
    ESP_LOGI(TAG, "UART%d up @ %d 8N1 (tx=%d rx=%d)", PORT, CONFIG_ZNP_UART_BAUD,
             CONFIG_ZNP_UART_TX_GPIO, CONFIG_ZNP_UART_RX_GPIO);
}

bool znp_uart_send_raw(const uint8_t *buf, size_t len) {
    if (!buf || len == 0) return false;
    /* F32: bounded mutex wait — drop + log rather than block the caller (the
     * stack mainloop / signal handler) indefinitely under TX contention. */
    if (s_tx_mutex &&
        xSemaphoreTake(s_tx_mutex, pdMS_TO_TICKS(ZNP_UART_TX_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGW(TAG, "tx mutex timeout — dropping %u B frame", (unsigned)len);
        return false;
    }
    int w = uart_write_bytes(PORT, (const char *)buf, len);
    bool ok = (w == (int)len);
    if (s_tx_mutex) xSemaphoreGive(s_tx_mutex);
    return ok;
}

static void rx_task(void *arg) {
    (void)arg;
    static mt_parser_t parser;
    mt_parser_reset(&parser);
    uint8_t chunk[64];
    mt_frame_t frame;
    uart_event_t ev;

    /* F32: subscribe to the task watchdog so a wedged RX loop reboots the
     * co-processor. Best effort — no-op if the TWDT isn't enabled in sdkconfig. */
    esp_task_wdt_add(NULL);

    for (;;) {
        if (xQueueReceive(s_uart_q, &ev, pdMS_TO_TICKS(1000)) != pdTRUE) {
            esp_task_wdt_reset();   /* idle but alive */
            continue;
        }
        switch (ev.type) {
        case UART_DATA: {
            int remaining = (int)ev.size;
            while (remaining > 0) {
                int want = remaining < (int)sizeof(chunk) ? remaining : (int)sizeof(chunk);
                int n = uart_read_bytes(PORT, chunk, want, pdMS_TO_TICKS(20));
                if (n <= 0) break;
                remaining -= n;
                for (int i = 0; i < n; i++) {
                    if (mt_parser_feed(&parser, chunk[i], &frame) == 1 && s_cb) {
                        s_cb(&frame);   /* payload valid only during this call */
                    }
                }
            }
            break;
        }
        case UART_FIFO_OVF:
        case UART_BUFFER_FULL:
            /* F32 + P6-T37 (FINDINGS §12 def 1): RX overrun — bytes were lost,
             * so the in-flight MT frame is unrecoverable.
             *
             * ORDER MATTERS: xQueueReset() FIRST, then uart_flush_input().
             * If we flushed the driver ring before draining the event queue,
             * any bytes that arrive in the window between flush and reset stay
             * in the ring while their UART_DATA event is dropped by the reset —
             * a permanent read-accounting lag where every later event then
             * reads those stale bytes first, pushing SRSPs past the host's
             * 2-3 s budget. Draining the queue first, then flushing, guarantees
             * the ring and the event accounting are both empty on resync.
             *
             * Also reset the parser so a half-consumed frame doesn't poison the
             * next parse — we resync on the next SOF, not on a corrupt partial. */
            ESP_LOGW(TAG, "UART RX overrun (event %d) — queue-reset + flush + parser resync", (int)ev.type);
            xQueueReset(s_uart_q);
            uart_flush_input(PORT);
            mt_parser_reset(&parser);
            break;
        case UART_FRAME_ERR:
        case UART_PARITY_ERR:
            /* P6-T37 (FINDINGS §12 def 2): a framing or parity error means the
             * byte stream is corrupt at the wire level. Previously these fell
             * into default and were ignored, letting noise bytes stream into the
             * parser guarded only by the weak XOR-8 FCS (1/256 false-accept — a
             * false-accepted SYS RESET 0x21/0x00 would trigger a spurious reboot).
             * Treat them like an overrun: flush the corrupt input + reset the
             * parser so we resync on the next clean SOF. */
            ESP_LOGW(TAG, "UART framing/parity error (event %d) — flush + parser resync", (int)ev.type);
            uart_flush_input(PORT);
            mt_parser_reset(&parser);
            break;
        default:
            break;
        }
        esp_task_wdt_reset();
    }
}
