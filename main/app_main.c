// SPDX-FileCopyrightText: 2025-2026 Evgenij Cjura and project contributors
// SPDX-License-Identifier: AGPL-3.0-or-later
#include "esp_log.h"
#include "nvs_flash.h"
#include "znp_uart.h"
#include "znp_dispatch.h"
#include "znp_ezb.h"

static const char *TAG = "znp_core";

/* Single dispatch context: carries buffered NV config + chip backend.
 * Lives for the lifetime of app_main; on_frame runs in the UART RX task. */
static znp_dispatch_ctx s_ctx;

/* RX task hands each received frame here; dispatch builds an encoded response
 * (or 0) and we write it straight back. Runs in the UART RX task context. */
static void on_frame(const mt_frame_t *f) {
    uint8_t buf[260];
    size_t n = znp_dispatch(f, &s_ctx, buf, sizeof(buf));
    /* T36 / FINDINGS §12 LOW (def 5): a dropped SRSP leaves the host's
     * synchronous request waiting until its retry/timeout — surface the failure
     * instead of silently ignoring znp_uart_send_raw's result. */
    if (n > 0 && !znp_uart_send_raw(buf, n)) {
        ESP_LOGW(TAG, "on_frame: SRSP send failed (%u B) — host may time out / retry",
                 (unsigned)n);
    }
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ret = nvs_flash_init_partition("zb_storage");
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase_partition("zb_storage"));
        ret = nvs_flash_init_partition("zb_storage");
    }
    ESP_ERROR_CHECK(ret);

    s_ctx.be = znp_ezb_backend();

    znp_uart_init(on_frame);

    /* Announce we just booted — the host toggles NRESET and waits for this. */
    uint8_t buf[16];
    size_t n = znp_build_reset_ind(0x00, buf, sizeof(buf));
    /* T36 / FINDINGS §12 LOW (def 5): the host waits for this RESET_IND after
     * toggling NRESET. Check the builder length (0 = encode overflow) and the
     * send result instead of firing blind — a missed RESET_IND stalls the host's
     * NCP-up handshake with no operator-visible cause. */
    if (n == 0) {
        ESP_LOGE(TAG, "znp_build_reset_ind encode failed — RESET_IND NOT sent");
    } else if (!znp_uart_send_raw(buf, n)) {
        ESP_LOGE(TAG, "RESET_IND send failed — host NCP-up handshake may stall");
    } else {
        ESP_LOGI(TAG, "esp-znp-core MT-NCP up: SYS link ready, RESET_IND sent");
    }
}
