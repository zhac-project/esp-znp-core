#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_zigbee.h"   // v2.x all-in-one header (was esp_zigbee_core.h in v1.x)

static const char *TAG = "znp_core";

void app_main(void)
{
    // Default NVS partition — erase + retry if unformatted / version-bumped
    // (otherwise ESP_ERROR_CHECK panics on a fresh/erased device).
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // v2.x: zb_storage is an NVS partition the Zigbee stack will use later.
    ret = nvs_flash_init_partition("zb_storage");
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase_partition("zb_storage"));
        ret = nvs_flash_init_partition("zb_storage");
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "esp-znp-core boot");
    ESP_LOGI(TAG, "esp-zigbee-lib version: %s", esp_zigbee_get_version_string());
    ESP_LOGI(TAG, "MT-NCP foundation ready (stack not started)");
}
