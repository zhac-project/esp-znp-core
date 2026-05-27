#include "znp_ezb.h"
#include "esp_mac.h"
#include "esp_system.h"
#include <string.h>

static void ezb_get_ieee(uint8_t out[8]) {
    uint8_t mac[8] = {0};
    /* 802.15.4 EUI64 from efuse. esp_read_mac returns it MSB-first; the MT
     * ExtAddr field is little-endian, so reverse. VERIFY this orientation on
     * first P4 integration (compare the IEEE the P4 logs vs the chip label). */
    if (esp_read_mac(mac, ESP_MAC_IEEE802154) == ESP_OK) {
        for (int i = 0; i < 8; i++) out[i] = mac[7 - i];
    } else {
        memset(out, 0, 8);   /* P4 tolerates IEEE=0 (bind falls back) */
    }
}

static void ezb_request_reset(void) {
    esp_restart();   /* reboots -> app_main emits SYS_RESET_IND on next boot */
}

const znp_backend_t *znp_ezb_backend(void) {
    static const znp_backend_t b = { ezb_get_ieee, ezb_request_reset };
    return &b;
}
