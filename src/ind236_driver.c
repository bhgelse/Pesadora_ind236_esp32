#include "ind236_driver.h"
#include <string.h>
#include <stdlib.h>
#include "driver/uart.h"
#include "driver/gpio.h"        // <-- FALTABA: define GPIO_NUM_xx
#include "esp_log.h"
#include "esp_check.h"          // <-- FALTABA: define ESP_RETURN_ON_ERROR

#define UART_SICS       UART_NUM_1
#define PIN_TX_SICS     GPIO_NUM_17
#define PIN_RX_SICS     GPIO_NUM_16
#define BAUD_SICS       9600
#define BUF_SIZE        512

static const char *TAG = "ind236";
static uint8_t s_buf[BUF_SIZE];
static int s_idx = 0;

esp_err_t ind236_init(void) {
    uart_config_t cfg = {
        .baud_rate = BAUD_SICS,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_RETURN_ON_ERROR(uart_driver_install(UART_SICS, BUF_SIZE, 0, 0, NULL, 0), TAG, "install");
    ESP_RETURN_ON_ERROR(uart_param_config(UART_SICS, &cfg), TAG, "config");
    ESP_RETURN_ON_ERROR(uart_set_pin(UART_SICS, PIN_TX_SICS, PIN_RX_SICS,
                                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE), TAG, "pins");
    ESP_LOGI(TAG, "UART1 listo: 9600 8N1 TX=%d RX=%d", PIN_TX_SICS, PIN_RX_SICS);
    return ESP_OK;
}

static bool parse_peso(const uint8_t *buf, int len, float *out) {
    char tmp[128];
    int j = 0;
    for (int i = 0; i < len && j < (int)sizeof(tmp) - 1; i++) {
        uint8_t c = buf[i];
        if ((c >= '0' && c <= '9') || c == '.' || c == '-' || c == '+' ||
            c == ' ' || c == 'k' || c == 'g' || c == 'l' || c == 'b') {
            tmp[j++] = c;
        } else if (c == '\r' || c == '\n' || c == 0x02) {
            tmp[j++] = ' ';
        }
    }
    tmp[j] = '\0';

    int n = strlen(tmp);
    int i = 0;
    while (i < n) {
        if ((tmp[i] >= '0' && tmp[i] <= '9') || tmp[i] == '-' || tmp[i] == '+') {
            int start = i;
            if (tmp[i] == '-' || tmp[i] == '+') i++;
            while (i < n && ((tmp[i] >= '0' && tmp[i] <= '9') || tmp[i] == '.')) i++;
            char num[32];
            int l = i - start;
            if (l > 0 && l < (int)sizeof(num)) {
                memcpy(num, &tmp[start], l);
                num[l] = '\0';
                float v = strtof(num, NULL);
                if (v > -100000.0f && v < 100000.0f) {
                    *out = v;
                    return true;
                }
            }
        } else {
            i++;
        }
    }
    return false;
}

esp_err_t ind236_read_continuous(float *peso) {
    uint8_t chunk[64];
    int len = uart_read_bytes(UART_SICS, chunk, sizeof(chunk), pdMS_TO_TICKS(20));
    if (len <= 0) return ESP_ERR_TIMEOUT;

    for (int i = 0; i < len; i++) {
        if (s_idx < BUF_SIZE) s_buf[s_idx++] = chunk[i];
        if (chunk[i] == '\r' || s_idx >= BUF_SIZE) {
            float v;
            if (parse_peso(s_buf, s_idx, &v)) {
                *peso = v;
                s_idx = 0;
                return ESP_OK;
            }
            s_idx = 0;
        }
    }
    return ESP_ERR_NOT_FOUND;
}