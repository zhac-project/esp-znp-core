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
    if (n > 0) znp_uart_send_raw(buf, n);
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
    znp_uart_send_raw(buf, n);

    ESP_LOGI(TAG, "esp-znp-core MT-NCP up: SYS link ready, RESET_IND sent");
}
