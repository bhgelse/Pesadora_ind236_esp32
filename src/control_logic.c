#include "control_logic.h"
#include "driver/gpio.h"

/* Salidas */
#define PIN_GRUESO     GPIO_NUM_18
#define PIN_FINO       GPIO_NUM_19
#define PIN_DESCARGA   GPIO_NUM_21
#define PIN_LED_GRUESO GPIO_NUM_25
#define PIN_LED_FINO   GPIO_NUM_26
#define PIN_LED_DESC   GPIO_NUM_27

/* Entradas (Pedal cambiado al GPIO15 por tener pull-up interno) */
#define PIN_BTN_INICIO GPIO_NUM_32
#define PIN_BTN_PARO   GPIO_NUM_33
#define PIN_PEDAL      GPIO_NUM_15
#define PIN_BTN_RESET  GPIO_NUM_13            /* junto a los otros defines de entradas */ 

void control_init(void) {
    /* ---- Salidas (relés + LEDs) ---- */
    gpio_config_t out = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << PIN_GRUESO) | (1ULL << PIN_FINO) | (1ULL << PIN_DESCARGA) |
                        (1ULL << PIN_LED_GRUESO) | (1ULL << PIN_LED_FINO) | (1ULL << PIN_LED_DESC),
        .pull_up_en = false, .pull_down_en = false, .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&out);

    /* ---- Entradas (Botones y Pedal) ---- */
    gpio_config_t in = {
        .mode = GPIO_MODE_INPUT,
                .pin_bit_mask = (1ULL << PIN_BTN_INICIO) | (1ULL << PIN_BTN_PARO) |
                        (1ULL << PIN_PEDAL) | (1ULL << PIN_BTN_RESET),
        .pull_up_en = GPIO_PULLUP_ENABLE, // Pull-up interno activado
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&in);

    /* Apagar todo al inicio */
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

/* Lectura de entradas (Presionado = 0 por el pull-up) */
bool control_boton_inicio(void)  { return gpio_get_level(PIN_BTN_INICIO) == 0; }
bool control_boton_paro(void)    { return gpio_get_level(PIN_BTN_PARO)   == 0; }
bool control_pedal_descarga(void) { return gpio_get_level(PIN_PEDAL)      == 0; }
bool control_boton_reset(void) { return gpio_get_level(PIN_BTN_RESET) == 0; }
