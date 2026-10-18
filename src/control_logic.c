#include "control_logic.h"
#include "driver/gpio.h"

#define PIN_GRUESO    GPIO_NUM_18
#define PIN_FINO      GPIO_NUM_19
#define PIN_DESCARGA  GPIO_NUM_21

// Pines de LEDs (Salidas)
#define PIN_LED_GRUESO   GPIO_NUM_25
#define PIN_LED_FINO     GPIO_NUM_26
#define PIN_LED_DESCARGA GPIO_NUM_27

void control_init(void) {
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << PIN_GRUESO) | (1ULL << PIN_FINO) | (1ULL << PIN_DESCARGA) |
                        (1ULL << PIN_LED_GRUESO) | (1ULL << PIN_LED_FINO) | (1ULL << PIN_LED_DESCARGA),
        .pull_up_en = false, .pull_down_en = false, .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    // Apagar todo al inicio
    gpio_set_level(PIN_GRUESO, 0); gpio_set_level(PIN_FINO, 0); gpio_set_level(PIN_DESCARGA, 0);
    gpio_set_level(PIN_LED_GRUESO, 0); gpio_set_level(PIN_LED_FINO, 0); gpio_set_level(PIN_LED_DESCARGA, 0);
}

void control_update(float peso_actual, float peso_objetivo, bool permiso, 
                    bool manual_grueso, bool manual_fino, bool manual_descarga) {
    
    bool activar_grueso = false;
    bool activar_fino = false;
    bool activar_descarga = false;

    // 1. Prioridad: Modo Manual (si el interruptor está en ON)
    if (manual_grueso || manual_fino || manual_descarga) {
        activar_grueso = manual_grueso;
        activar_fino = manual_fino;
        activar_descarga = manual_descarga;
    } 
    // 2. Modo Automático (si hay permiso)
    else if (permiso) {
        if (peso_actual < peso_objetivo * 0.8f) {
            activar_grueso = true;
        } else if (peso_actual < peso_objetivo * 0.99f) {
            activar_fino = true;
        } else if (peso_actual >= peso_objetivo) {
            // Opcional: activar descarga automática aquí si lo deseas
             activar_descarga = true; 
        }
    }
    // 3. Si no hay permiso y no hay manual, todo apagado (else implícito)

    // Aplicar a Relés y LEDs
    gpio_set_level(PIN_GRUESO, activar_grueso);
    gpio_set_level(PIN_LED_GRUESO, activar_grueso);
    
    gpio_set_level(PIN_FINO, activar_fino);
    gpio_set_level(PIN_LED_FINO, activar_fino);
    
    gpio_set_level(PIN_DESCARGA, activar_descarga);
    gpio_set_level(PIN_LED_DESCARGA, activar_descarga);
}