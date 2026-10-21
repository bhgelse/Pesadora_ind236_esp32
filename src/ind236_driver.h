#pragma once
#include "esp_err.h"

// Inicializa UART1 a 9600 8N1 con pines GPIO4 (TX), GPIO5 (RX)
esp_err_t ind236_init(void);

// Lee el siguiente peso válido del flujo continuo.
// Devuelve ESP_OK y escribe en *peso si encontró una trama completa.
// NO envía comandos (el IND236 está en modo continuo).
esp_err_t ind236_read_continuous(float *peso);