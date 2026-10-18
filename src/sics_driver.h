#pragma once

#include "esp_err.h"
#include "driver/uart.h"

// Inicializar UART y configurar pines (TX, RX)
esp_err_t sics_init(uart_port_t uart_num, int baud_rate, int tx_pin, int rx_pin);

// Enviar comando "S" para solicitar peso estable, analizar respuesta y escribir en peso_actual
esp_err_t sics_read_weight(float *peso_actual);