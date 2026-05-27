#include "znp_uart.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "znp_uart";
static const uart_port_t PORT = CONFIG_ZNP_UART_PORT;
static znp_frame_cb_t s_cb = NULL;
static SemaphoreHandle_t s_tx_mutex = NULL;

#define ZNP_UART_RX_BUF 512

static void rx_task(void *arg);

void znp_uart_init(znp_frame_cb_t cb) {
    s_cb = cb;
    const uart_config_t cfg = {
        .baud_rate = CONFIG_ZNP_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(PORT, ZNP_UART_RX_BUF, ZNP_UART_RX_BUF, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(PORT, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(PORT, CONFIG_ZNP_UART_TX_GPIO, CONFIG_ZNP_UART_RX_GPIO,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    s_tx_mutex = xSemaphoreCreateMutex();
    xTaskCreate(rx_task, "znp_uart_rx", 4096, NULL, 10, NULL);
    ESP_LOGI(TAG, "UART%d up @ %d 8N1 (tx=%d rx=%d)", PORT, CONFIG_ZNP_UART_BAUD,
             CONFIG_ZNP_UART_TX_GPIO, CONFIG_ZNP_UART_RX_GPIO);
}

bool znp_uart_send_raw(const uint8_t *buf, size_t len) {
    if (!buf || len == 0) return false;
    if (s_tx_mutex) xSemaphoreTake(s_tx_mutex, portMAX_DELAY);
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
    for (;;) {
        int n = uart_read_bytes(PORT, chunk, sizeof(chunk), pdMS_TO_TICKS(100));
        for (int i = 0; i < n; i++) {
            if (mt_parser_feed(&parser, chunk[i], &frame) == 1 && s_cb) {
                s_cb(&frame);   /* payload valid only during this call */
            }
        }
    }
}
