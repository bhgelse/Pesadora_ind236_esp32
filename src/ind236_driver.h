#pragma once
#include "esp_err.h"

esp_err_t ind236_init(void);
esp_err_t ind236_read_continuous(float *peso);
