#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_random.h"
#include "sics_driver.h"
#include "control_logic.h"
#include "ui_hardware.h"      // <-- NUEVO
#include "driver/i2c.h"

#define MODO_SIMULADO   1
#define UART_SICS       UART_NUM_1
#define PIN_TX_SICS     GPIO_NUM_4
#define PIN_RX_SICS     GPIO_NUM_5
#define PESO_OBJETIVO   100.0f
#define BAUD_SICS       9600

static bool s_permiso          = true;
static bool s_manual_grueso    = false;
static bool s_manual_fino      = false;
static bool s_manual_descarga  = false;

static float s_peso_simulado = 0.0f;

static float simular_peso(void) {
    float incremento = 3.0f + (float)(esp_random() % 500) / 100.0f;
    s_peso_simulado += incremento;
    if (s_peso_simulado >= PESO_OBJETIVO) s_peso_simulado = 0.0f;
    return s_peso_simulado;
}

void app_main(void) {
    control_init();
    ui_init();                           // <-- NUEVO

    printf("Escaneando bus I2C...\n");
for (uint8_t addr = 1; addr < 127; addr++) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    esp_err_t res = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    if (res == ESP_OK) {
        printf("Dispositivo I2C encontrado en: 0x%02X\n", addr);
    }
}

    ui_mostrar_mensaje("Pesadora IND236", "Iniciando..."); // <-- NUEVO

#if MODO_SIMULADO
    printf("*** MODO SIMULADO ACTIVO ***\n");
#else
    if (sics_init(UART_SICS, BAUD_SICS, PIN_TX_SICS, PIN_RX_SICS) != ESP_OK) {
        ui_mostrar_mensaje("Error UART", "Revisar MAX3232");
        return;
    }
#endif

    float peso = 0.0f;

    while (1) {
#if MODO_SIMULADO
        peso = simular_peso();

        const char *etapa;
        if (peso < PESO_OBJETIVO * 0.8f)        etapa = "GRUESO";
        else if (peso < PESO_OBJETIVO * 0.99f)  etapa = "FINO";
        else                                    etapa = "META";

        printf("[SIM] Peso: %6.2f kg | Etapa: %s\n", peso, etapa);
        ui_mostrar_peso(peso, etapa);    // <-- NUEVO

        control_update(peso, PESO_OBJETIVO, s_permiso,
                       s_manual_grueso, s_manual_fino, s_manual_descarga);
#else
        if (sics_read_weight(&peso) == ESP_OK) {
            const char *etapa = (peso < PESO_OBJETIVO * 0.8f)  ? "GRUESO" :
                                (peso < PESO_OBJETIVO * 0.99f) ? "FINO"   : "META";
            printf("[IND236] Peso: %.2f kg\n", peso);
            ui_mostrar_peso(peso, etapa);    // <-- NUEVO
            control_update(peso, PESO_OBJETIVO, s_permiso,
                           s_manual_grueso, s_manual_fino, s_manual_descarga);
        } else {
            printf("[IND236] Timeout. Apagando válvulas.\n");
            ui_mostrar_mensaje("SIN COMUNIC.", "IND236 TIMEOUT"); // <-- NUEVO
            control_update(0.0f, 0.0f, false, false, false, false);
        }
#endif

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}