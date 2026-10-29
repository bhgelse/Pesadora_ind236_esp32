#pragma once
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

esp_err_t ind236_init(void);
/* Bloquea hasta una trama válida, o hasta 'espera' ticks sin recibir bytes */
esp_err_t ind236_read_continuous(float *peso, TickType_t espera);
