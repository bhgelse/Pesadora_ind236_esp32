#include "control_logic.h"
#include "driver/gpio.h"

// Salidas
#define PIN_GRUESO     GPIO_NUM_18
#define PIN_FINO       GPIO_NUM_19
#define PIN_DESCARGA   GPIO_NUM_21
#define PIN_LED_GRUESO GPIO_NUM_25
#define PIN_LED_FINO   GPIO_NUM_26
#define PIN_LED_DESC   GPIO_NUM_27

// Entradas (ajusta según tu cableado)
#define PIN_BTN_INICIO GPIO_NUM_32
#define PIN_BTN_PARO   GPIO_NUM_33
#define PIN_PEDAL      GPIO_NUM_34   // INPUT only, requiere pull-up externo

void control_init(void) {
    gpio_config_t out = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << PIN_GRUESO) | (1ULL << PIN_FINO) | (1ULL << PIN_DESCARGA) |
                        (1ULL << PIN_LED_GRUESO) | (1ULL << PIN_LED_FINO) | (1ULL << PIN_LED_DESC),
        .pull_up_en = false, .pull_down_en = false, .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&out);

    gpio_config_t in = {
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << PIN_BTN_INICIO) | (1ULL << PIN_BTN_PARO) | (1ULL << PIN_PEDAL),
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&in);

    gpio_set_level(PIN_GRUESO, 0);
    gpio_set_level(PIN_FINO, 0);
    gpio_set_level(PIN_DESCARGA, 0);
    gpio_set_level(PIN_LED_GRUESO, 0);
    gpio_set_level(PIN_LED_FINO, 0);
    gpio_set_level(PIN_LED_DESC, 0);
}

void control_update(bool rel_grueso, bool rel_fino, bool rel_descarga) {
    gpio_set_level(PIN_GRUESO, rel_grueso);
    gpio_set_level(PIN_LED_GRUESO, rel_grueso);
    gpio_set_level(PIN_FINO, rel_fino);
    gpio_set_level(PIN_LED_FINO, rel_fino);
    gpio_set_level(PIN_DESCARGA, rel_descarga);
    gpio_set_level(PIN_LED_DESC, rel_descarga);
}

// Activos en LOW (con pull-up interno): presionado = 0
bool control_boton_inicio(void)  { return gpio_get_level(PIN_BTN_INICIO) == 0; }
bool control_boton_paro(void)    { return gpio_get_level(PIN_BTN_PARO)   == 0; }
bool control_pedal_descarga(void){ return gpio_get_level(PIN_PEDAL)      == 0; }