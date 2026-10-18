#include "sics_driver.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

static uart_port_t s_uart_num;

esp_err_t sics_init(uart_port_t uart_num, int baud_rate, int tx_pin, int rx_pin) {
    s_uart_num = uart_num;
    uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_RETURN_ON_ERROR(uart_driver_install(uart_num, 1024, 0, 0, NULL, 0), "sics", "install fail");
    ESP_RETURN_ON_ERROR(uart_param_config(uart_num, &uart_config), "sics", "config fail");
    ESP_RETURN_ON_ERROR(uart_set_pin(uart_num, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE), "sics", "pin fail");
    return ESP_OK;
}

esp_err_t sics_read_weight(float *peso_actual) {
    // Enviar comando "S" + CR LF
    const char *cmd = "S\r\n";
    uart_write_bytes(s_uart_num, cmd, strlen(cmd));

    // Leer respuesta
    uint8_t data[128];
    int len = uart_read_bytes(s_uart_num, data, sizeof(data) - 1, pdMS_TO_TICKS(500));
    if (len <= 0) return ESP_ERR_TIMEOUT;
    data[len] = '\0';

    // Análisis simple: buscar el primer carácter después del segundo espacio y antes del espacio de unidad
    // Formato de respuesta de ejemplo: "S S     12.345 kg"
    char *p = strchr((char*)data, ' ');
    if (!p) return ESP_ERR_INVALID_RESPONSE;
    p = strchr(p + 1, ' '); // Saltar estado
    if (!p) return ESP_ERR_INVALID_RESPONSE;
    p++; // Apuntar al inicio del valor numérico

    char *endptr;
    float val = strtof(p, &endptr);
    if (endptr == p) return ESP_ERR_INVALID_RESPONSE;

    *peso_actual = val;
    return ESP_OK;
}