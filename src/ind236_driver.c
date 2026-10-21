#include "ind236_driver.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_check.h"

#define UART_SICS       UART_NUM_1
#define PIN_TX_SICS     GPIO_NUM_17
#define PIN_RX_SICS     GPIO_NUM_16
#define BAUD_SICS       19200
#define BUF_SIZE        512

/* ============================================================
 *  FACTOR_DECIMALES
 *  1   -> báscula muestra 50        (sin decimales)
 *  10  -> báscula muestra 50.0
 *  100 -> báscula muestra 50.00
 *  1000-> báscula muestra 50.000
 *  Ajusta según la pantalla del IND236.
 * ============================================================ */
#define FACTOR_DECIMALES 1.0f

/* Activa el hexdump para depurar la trama cruda */
#define DEBUG_HEXDUMP 0

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
    ESP_LOGI(TAG, "UART1 listo: 9600 8N1 TX=%d RX=%d | FACTOR_DECIMALES=%.0f",
             PIN_TX_SICS, PIN_RX_SICS, FACTOR_DECIMALES);
    return ESP_OK;
}

/* Parsea trama Toledo Continuous:
 * [STX 0x02][S1][S2][S3][peso 6 ASCII][tara 6 ASCII][CR 0x0D][CHK]
 * Devuelve true si encontró trama válida.
 */
static bool parse_toledo(const uint8_t *buf, int len, float *out) {
    int i = 0;
    while (i < len && buf[i] != 0x02) i++;
    if (i >= len) return false;

    if (i + 17 > len) return false;

    int p_start = i + 4;   // saltar STX + S1 + S2 + S3
    int p_end   = p_start + 6;

    char num[8];
    int j = 0;
    for (int k = p_start; k < p_end; k++) {
        uint8_t c = buf[k];
        if ((c >= '0' && c <= '9') || c == '-' || c == '+') {
            num[j++] = c;
        } else if (c == ' ') {
            // ignorar
        } else {
            return false;
        }
    }
    num[j] = '\0';
    if (j == 0) return false;

    float v = strtof(num, NULL) / FACTOR_DECIMALES;
    if (v < -100000.0f || v > 100000.0f) return false;

    *out = v;
    return true;
}

esp_err_t ind236_read_continuous(float *peso) {
    uint8_t chunk[64];
    int len = uart_read_bytes(UART_SICS, chunk, sizeof(chunk), pdMS_TO_TICKS(20));
    if (len <= 0) return ESP_ERR_TIMEOUT;

#if DEBUG_HEXDUMP
    ESP_LOG_BUFFER_HEXDUMP("ind236", chunk, len, ESP_LOG_INFO);
#endif

    for (int i = 0; i < len; i++) {
        if (s_idx < BUF_SIZE) s_buf[s_idx++] = chunk[i];
        if (chunk[i] == 0x0D || s_idx >= BUF_SIZE) {
            float v;
            if (parse_toledo(s_buf, s_idx, &v)) {
                *peso = v;
                s_idx = 0;
                return ESP_OK;
            }
            s_idx = 0;
        }
    }
    return ESP_ERR_NOT_FOUND;
}
