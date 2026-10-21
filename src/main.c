/* ============================================================
 *  SISTEMA DE PESAJE IND236 + ESP32
 *  MODO: IND236 REAL, entradas físicas 32/33/34
 * ------------------------------------------------------------
 *  GPIO18 -> Relé GRUESO
 *  GPIO19 -> Relé FINO
 *  GPIO21 -> Relé DESCARGA
 *  GPIO25 -> LED GRUESO
 *  GPIO26 -> LED FINO
 *  GPIO27 -> LED DESCARGA
 *
 *  GPIO32 -> Botón INICIO (pull-up interno)
 *  GPIO33 -> Botón PARO   (pull-up interno)
 *  GPIO34 -> Pedal        (pull-up EXTERNO 10k a 3V3)
 *
 *  UART1: GPIO16 (RX) <- TX del MAX3232
 *         GPIO17 (TX) -> RX del MAX3232
 * ============================================================ */

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "ind236_driver.h"
#include "control_logic.h"
#include "ui_hardware.h"

/* ============================================================
 *  PARÁMETROS DEL PROCESO (ajustables)
 * ============================================================ */
#define PESO_OBJETIVO       50.0f
#define UMBRAL_GRUESO       45.0f
#define COLA_GRUESA         0.5f
#define COLA_FINA           0.3f
#define PESO_MIN_DESCARGA   1.0f
#define TIMEOUT_DESCARGA    15000     /* ms */
#define TIMEOUT_LLENADO     120000    /* ms */

static const char *TAG = "main";

/* ============================================================
 *  PESO COMPARTIDO (protegido por mutex)
 * ============================================================ */
static float g_peso_actual = 0.0f;
static SemaphoreHandle_t g_mutex_peso;

float peso_get(void) {
    float v;
    xSemaphoreTake(g_mutex_peso, portMAX_DELAY);
    v = g_peso_actual;
    xSemaphoreGive(g_mutex_peso);
    return v;
}

void peso_set(float v) {
    xSemaphoreTake(g_mutex_peso, portMAX_DELAY);
    g_peso_actual = v;
    xSemaphoreGive(g_mutex_peso);
}

/* ============================================================
 *  TASK UART: lee IND236 continuo en Core 1
 * ============================================================ */
static void task_uart(void *arg) {
    ind236_init();
    while (1) {
        float p;
        if (ind236_read_continuous(&p) == ESP_OK) {
            peso_set(p);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* ============================================================
 *  MÁQUINA DE ESTADOS
 * ============================================================ */
typedef enum {
    EST_IDLE,
    EST_LLENO_GRUESO,
    EST_LLENO_FINO,
    EST_ESPERANDO_PEDAL,
    EST_DESCARGANDO,
    EST_ERROR
} estado_t;

static const char* estado_str(estado_t e) {
    switch (e) {
        case EST_IDLE:            return "IDLE";
        case EST_LLENO_GRUESO:    return "LLENO_GRUESO";
        case EST_LLENO_FINO:      return "LLENO_FINO";
        case EST_ESPERANDO_PEDAL: return "ESPERANDO_PEDAL";
        case EST_DESCARGANDO:     return "DESCARGANDO";
        case EST_ERROR:           return "ERROR";
        default:                  return "?";
    }
}

// Enviar estado por USB Serial en formato JSON para la interfaz web
static void enviar_estado_web(float peso, estado_t estado, bool rel_grueso, bool rel_fino, bool rel_desc, bool pedal) {
    // Mapear el estado a un texto legible
    const char* estado_txt = estado_str(estado);

    // Formato JSON simple (una línea terminada en \n)
    printf("{\"peso\":%.2f,\"estado\":\"%s\",\"grueso\":%d,\"fino\":%d,\"descarga\":%d,\"pedal\":%d}\n",
           peso, estado_txt, rel_grueso, rel_fino, rel_desc, pedal);
}

/* ============================================================
 *  APP PRINCIPAL
 * ============================================================ */
void app_main(void) {
    g_mutex_peso = xSemaphoreCreateMutex();
    peso_set(0.0f);

    control_init();
    ui_init();
    ui_mostrar_mensaje("Pesadora IND236", "Iniciando...");

    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, " Sistema iniciado (IND236 real)");
    ESP_LOGI(TAG, " Peso objetivo: %.2f kg", PESO_OBJETIVO);
    ESP_LOGI(TAG, " Umbral grueso: %.2f kg | Cola gruesa: %.2f",
             UMBRAL_GRUESO, COLA_GRUESA);
    ESP_LOGI(TAG, " Cola fina: %.2f kg | Descarga: %.2f",
             COLA_FINA, PESO_MIN_DESCARGA);
    ESP_LOGI(TAG, " Entradas: INICIO=GPIO32 PARO=GPIO33 PEDAL=GPIO34");
    ESP_LOGI(TAG, "==========================================");

    /* Task UART del IND236 en Core 1 */
    xTaskCreatePinnedToCore(task_uart, "uart", 4096, NULL, 10, NULL, 1);
    ESP_LOGI(TAG, "Task UART (IND236) lanzada en Core 1");

    vTaskDelay(pdMS_TO_TICKS(1000));
    ui_mostrar_mensaje("Listo", "Pulse INICIO");

    estado_t estado      = EST_IDLE;
    estado_t estado_prev = EST_IDLE;
    TickType_t t_inicio = 0;
    TickType_t t_ultimo_log_peso = 0;

    while (1) {
        float peso = peso_get();

        /* ====================================================
         *  LECTURA DE ENTRADAS FÍSICAS
         * ==================================================== */
        bool btn_inicio = control_boton_inicio();
        bool btn_paro   = control_boton_paro();
        bool pedal      = control_pedal_descarga();

        /* Logs de flanco (solo cuando cambia) */
        static bool btn_inicio_prev = false;
        static bool btn_paro_prev   = false;
        static bool pedal_prev      = false;

        if (btn_inicio && !btn_inicio_prev) ESP_LOGI(TAG, ">>> INICIO presionado");
        if (btn_paro   && !btn_paro_prev)   ESP_LOGI(TAG, ">>> PARO presionado");
        if (pedal != pedal_prev)
            ESP_LOGI(TAG, ">>> Pedal -> %s", pedal ? "PISADO" : "LIBERADO");

        btn_inicio_prev = btn_inicio;
        btn_paro_prev   = btn_paro;
        pedal_prev      = pedal;

        /* ====================================================
         *  MÁQUINA DE ESTADOS
         * ==================================================== */
        switch (estado) {

        case EST_IDLE:
            if (btn_inicio) {
                estado = EST_LLENO_GRUESO;
                t_inicio = xTaskGetTickCount();
                ESP_LOGI(TAG, "INICIO llenado. Peso inicial: %.2f kg", peso);
            }
            break;

        case EST_LLENO_GRUESO:
            if (btn_paro) {
                ESP_LOGW(TAG, "PARO en GRUESO");
                estado = EST_IDLE;
                break;
            }
            if (peso >= (UMBRAL_GRUESO - COLA_GRUESA)) {
                ESP_LOGI(TAG, "Corte GRUESO a %.2f kg (umbral %.2f - cola %.2f)",
                         peso, UMBRAL_GRUESO, COLA_GRUESA);
                estado = EST_LLENO_FINO;
            }
            if ((xTaskGetTickCount() - t_inicio) > pdMS_TO_TICKS(TIMEOUT_LLENADO)) {
                ESP_LOGE(TAG, "TIMEOUT llenado en GRUESO a %.2f kg", peso);
                estado = EST_ERROR;
            }
            break;

        case EST_LLENO_FINO:
            if (btn_paro) {
                ESP_LOGW(TAG, "PARO en FINO");
                estado = EST_IDLE;
                break;
            }
            if (peso >= (PESO_OBJETIVO - COLA_FINA)) {
                ESP_LOGI(TAG, "Corte FINO a %.2f kg (objetivo %.2f - cola %.2f)",
                         peso, PESO_OBJETIVO, COLA_FINA);
                ESP_LOGI(TAG, "Esperando asentamiento 1.5 s...");
                vTaskDelay(pdMS_TO_TICKS(1500));
                ESP_LOGI(TAG, "Peso final llenado: %.2f kg", peso_get());
                estado = EST_ESPERANDO_PEDAL;
            }
            if ((xTaskGetTickCount() - t_inicio) > pdMS_TO_TICKS(TIMEOUT_LLENADO)) {
                ESP_LOGE(TAG, "TIMEOUT llenado en FINO a %.2f kg", peso);
                estado = EST_ERROR;
            }
            break;

        case EST_ESPERANDO_PEDAL:
            if (btn_paro) {
                ESP_LOGW(TAG, "PARO en ESPERANDO_PEDAL");
                estado = EST_IDLE;
                break;
            }
            if (pedal && peso > PESO_MIN_DESCARGA) {
                ESP_LOGI(TAG, "Pedal PISADO. Iniciando descarga desde %.2f kg", peso);
                estado = EST_DESCARGANDO;
                t_inicio = xTaskGetTickCount();
            }
            break;

        case EST_DESCARGANDO:
            if (peso <= PESO_MIN_DESCARGA) {
                ESP_LOGI(TAG, "Descarga completa a %.2f kg", peso);
                estado = EST_IDLE;
            } else if ((xTaskGetTickCount() - t_inicio) > pdMS_TO_TICKS(TIMEOUT_DESCARGA)) {
                ESP_LOGW(TAG, "TIMEOUT descarga a %.2f kg", peso);
                estado = EST_IDLE;
            }
            break;

        case EST_ERROR:
            if (btn_paro) {
                ESP_LOGI(TAG, "Reset desde ERROR");
                estado = EST_IDLE;
            }
            break;
        }

        /* Log de cambio de estado */
        if (estado != estado_prev) {
            ESP_LOGI(TAG, "*** ESTADO: %s -> %s (peso=%.2f) ***",
                     estado_str(estado_prev), estado_str(estado), peso);
            estado_prev = estado;
        }

        /* ====================================================
         *  SALIDAS FÍSICAS (relés + LEDs)
         * ==================================================== */
        bool rel_grueso = (estado == EST_LLENO_GRUESO);
        bool rel_fino   = (estado == EST_LLENO_FINO);
        bool rel_desc   = (estado == EST_DESCARGANDO);

        /* Log solo cuando cambia */
        static bool rg_prev = false, rf_prev = false, rd_prev = false;
        if (rel_grueso != rg_prev) ESP_LOGI(TAG, "RELE GRUESO  -> %s", rel_grueso ? "ON" : "OFF");
        if (rel_fino   != rf_prev) ESP_LOGI(TAG, "RELE FINO    -> %s", rel_fino   ? "ON" : "OFF");
        if (rel_desc   != rd_prev) ESP_LOGI(TAG, "RELE DESCARGA-> %s", rel_desc   ? "ON" : "OFF");
        rg_prev = rel_grueso; rf_prev = rel_fino; rd_prev = rel_desc;

        control_update(rel_grueso, rel_fino, rel_desc);

                // Enviar datos a la interfaz web cada 200 ms
        static TickType_t t_web = 0;
        if ((xTaskGetTickCount() - t_web) > pdMS_TO_TICKS(200)) {
            t_web = xTaskGetTickCount();
            enviar_estado_web(peso, estado, rel_grueso, rel_fino, rel_desc, pedal);
        }

        /* ====================================================
         *  LOG PERIÓDICO (cada 500 ms)
         * ==================================================== */
        if ((xTaskGetTickCount() - t_ultimo_log_peso) > pdMS_TO_TICKS(500)) {
            t_ultimo_log_peso = xTaskGetTickCount();
            ESP_LOGI(TAG, "Peso:%7.2f kg | %-16s | G=%d F=%d D=%d | Pedal=%d",
                     peso, estado_str(estado), rel_grueso, rel_fino, rel_desc, pedal);
        }

        /* ====================================================
         *  LCD (cada 200 ms)
         * ==================================================== */
        static TickType_t t_ui = 0;
        if ((xTaskGetTickCount() - t_ui) > pdMS_TO_TICKS(200)) {
            t_ui = xTaskGetTickCount();
            const char *etapa =
                (estado == EST_IDLE)            ? "LISTO"   :
                (estado == EST_LLENO_GRUESO)    ? "GRUESO"  :
                (estado == EST_LLENO_FINO)      ? "FINO"    :
                (estado == EST_ESPERANDO_PEDAL) ? "PEDAL"   :
                (estado == EST_DESCARGANDO)     ? "DESCARGA":
                                                  "ERROR";
            ui_mostrar_peso(peso, etapa);
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
