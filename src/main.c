/* ============================================================
 *  SISTEMA DE PESAJE IND236 + ESP32
 *  MODO ACTUAL: SIMULACION (sin hardware IND236)
 * ------------------------------------------------------------
 *  Lo que es SIMULACION está marcado con  // [SIM]
 *  Lo que es LÓGICA REAL (se usará con IND236) está sin marca
 * ============================================================ */

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_random.h"
#include "esp_log.h"
#include "ind236_driver.h"
#include "control_logic.h"
#include "ui_hardware.h"

/* ============================================================
 *  INTERRUPTORES DE MODO
 * ------------------------------------------------------------
 *  MODO_SIMULADO     1 = usa simulador interno (sin IND236)
 *                    0 = usa IND236 real por UART
 *
 *  SIM_FORZAR_INICIO 1 = pulsa INICIO automatico cada 8 s
 *  SIM_FORZAR_PEDAL  1 = pisa el pedal automaticamente
 *                     (solo tienen efecto si MODO_SIMULADO=1)
 * ============================================================ */
#define MODO_SIMULADO       0
//#define SIM_FORZAR_INICIO   1
//#define SIM_FORZAR_PEDAL    1

/* ============================================================
 *  PARÁMETROS DEL PROCESO (ajustables)
 * ============================================================ */
#define PESO_OBJETIVO       50.0f    // kg
#define UMBRAL_GRUESO       45.0f    // kg
#define COLA_GRUESA         0.5f     // kg de compensación tras cerrar gruesa
#define COLA_FINA           0.3f     // kg de compensación tras cerrar fina
#define PESO_MIN_DESCARGA   1.0f     // kg (balde vacío)
#define TIMEOUT_DESCARGA    15000    // ms
#define TIMEOUT_LLENADO     120000   // ms

static const char *TAG = "main";

/* ============================================================
 *  PESO COMPARTIDO (protegido por mutex)
 *  Lo usa tanto el simulador como el driver real del IND236.
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
 *  [SIM] SIMULADOR DE BÁSCULA
 * ------------------------------------------------------------
 *  Simula el caudal de cada válvula en kg/segundo.
 *  Solo se compila si MODO_SIMULADO == 1.
 * ============================================================ */
#if MODO_SIMULADO
static float s_peso_sim = 0.0f;
static bool  s_sim_valvula_gruesa = false;
static bool  s_sim_valvula_fina   = false;
static bool  s_sim_valvula_desc   = false;

/* Caudales simulados (kg/s) — ajusta si quieres otra velocidad */
#define CAUDAL_GRUESA   12.0f
#define CAUDAL_FINA      3.0f
#define CAUDAL_DESCARGA 20.0f

static void sim_tick(float dt) {
    if (s_sim_valvula_gruesa) s_peso_sim += CAUDAL_GRUESA   * dt;
    if (s_sim_valvula_fina)   s_peso_sim += CAUDAL_FINA     * dt;
    if (s_sim_valvula_desc)   s_peso_sim -= CAUDAL_DESCARGA * dt;
    if (s_peso_sim < 0.0f) s_peso_sim = 0.0f;

    /* Ruido de báscula ±0.02 kg */
    float ruido = ((float)(esp_random() % 400) / 10000.0f) - 0.02f;
    peso_set(s_peso_sim + ruido);
}

void sim_set_valvulas(bool g, bool f, bool d) {
    if (g != s_sim_valvula_gruesa)
        ESP_LOGI(TAG, "[SIM] Valvula GRUESA  -> %s", g ? "ABIERTA" : "CERRADA");
    if (f != s_sim_valvula_fina)
        ESP_LOGI(TAG, "[SIM] Valvula FINA    -> %s", f ? "ABIERTA" : "CERRADA");
    if (d != s_sim_valvula_desc)
        ESP_LOGI(TAG, "[SIM] Valvula DESCARGA-> %s", d ? "ABIERTA" : "CERRADA");
    s_sim_valvula_gruesa = g;
    s_sim_valvula_fina   = f;
    s_sim_valvula_desc   = d;
}
#else
/* ============================================================
 *  LÓGICA REAL: TASK UART que lee el IND236 en modo continuo
 *  Solo se compila si MODO_SIMULADO == 0
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
#endif

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

static void log_estado(estado_t anterior, estado_t nuevo, float peso) {
    ESP_LOGI(TAG, "*** ESTADO: %s -> %s (peso=%.2f kg) ***",
             estado_str(anterior), estado_str(nuevo), peso);
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
    ESP_LOGI(TAG, " Sistema iniciado");
    ESP_LOGI(TAG, " MODO_SIMULADO = %d  (1=sim, 0=IND236)", MODO_SIMULADO);
    #if MODO_SIMULADO
    ESP_LOGI(TAG, " [SIM] Forzar INICIO = %d", SIM_FORZAR_INICIO);
    ESP_LOGI(TAG, " [SIM] Forzar PEDAL  = %d", SIM_FORZAR_PEDAL);
    #endif
    ESP_LOGI(TAG, "==========================================");

    #if !MODO_SIMULADO
    /* LÓGICA REAL: inicia la lectura del IND236 en Core 1 */
    xTaskCreatePinnedToCore(task_uart, "uart", 4096, NULL, 10, NULL, 1);
    ESP_LOGI(TAG, "Task UART (IND236) lanzada en Core 1");
    #endif

    vTaskDelay(pdMS_TO_TICKS(1000));
    ui_mostrar_mensaje("Listo", "Pulse INICIO");

    estado_t estado      = EST_IDLE;
    estado_t estado_prev = EST_IDLE;
    TickType_t t_inicio          = 0;
    TickType_t t_ultimo_tick_sim = xTaskGetTickCount();
    TickType_t t_ultimo_log_peso = 0;

    while (1) {
        float peso = peso_get();

        /* ========================================================
         *  [SIM] Tick del simulador: avanza el peso según válvulas
         * ======================================================== */
        #if MODO_SIMULADO
        TickType_t ahora_tick = xTaskGetTickCount();
        float dt = (ahora_tick - t_ultimo_tick_sim) * portTICK_PERIOD_MS / 1000.0f;
        if (dt > 0.2f) dt = 0.2f;
        t_ultimo_tick_sim = ahora_tick;
        sim_tick(dt);
        peso = peso_get();
        #endif

        /* ========================================================
         *  LECTURA DE ENTRADAS FÍSICAS (botones y pedal)
         *  En modo sim se pueden forzar por software.
         * ======================================================== */
        bool btn_inicio = control_boton_inicio();
        bool btn_paro   = control_boton_paro();
        bool pedal      = control_pedal_descarga();

        #if MODO_SIMULADO && SIM_FORZAR_INICIO
        {
            static TickType_t t_auto = 0;
            if (estado == EST_IDLE &&
                (xTaskGetTickCount() - t_auto) > pdMS_TO_TICKS(8000)) {
                btn_inicio = true;
                t_auto = xTaskGetTickCount();
                ESP_LOGI(TAG, "[SIM] Pulsacion automatica de INICIO");
            }
        }
        #endif

        #if MODO_SIMULADO && SIM_FORZAR_PEDAL
        if (estado == EST_ESPERANDO_PEDAL) {
            pedal = true;
        }
        #endif

        /* Detección de flancos para logs claros */
        static bool btn_inicio_prev = false;
        static bool btn_paro_prev   = false;
        static bool pedal_prev      = false;

        if (btn_inicio && !btn_inicio_prev) ESP_LOGI(TAG, "Boton INICIO presionado");
        if (btn_paro   && !btn_paro_prev)   ESP_LOGI(TAG, "Boton PARO presionado");
        if (pedal != pedal_prev)
            ESP_LOGI(TAG, "Pedal de descarga -> %s", pedal ? "PISADO" : "LIBERADO");

        btn_inicio_prev = btn_inicio;
        btn_paro_prev   = btn_paro;
        pedal_prev      = pedal;

        /* ========================================================
         *  MÁQUINA DE ESTADOS (lógica real del proceso)
         * ======================================================== */
        switch (estado) {

        case EST_IDLE:
            if (btn_inicio) {
                estado = EST_LLENO_GRUESO;
                t_inicio = xTaskGetTickCount();
                ESP_LOGI(TAG, "INICIO llenado. Peso inicial: %.2f kg", peso);
            }
            break;

        case EST_LLENO_GRUESO:
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

        /* Log al cambiar de estado */
        if (estado != estado_prev) {
            log_estado(estado_prev, estado, peso);
            estado_prev = estado;
        }

        /* ========================================================
         *  SALIDAS FÍSICAS (relés)
         * ======================================================== */
        bool rel_grueso = (estado == EST_LLENO_GRUESO);
        bool rel_fino   = (estado == EST_LLENO_FINO);
        bool rel_desc   = (estado == EST_DESCARGANDO);

        static bool rg_prev = false, rf_prev = false, rd_prev = false;
        if (rel_grueso != rg_prev) ESP_LOGI(TAG, "RELE GRUESO  -> %s", rel_grueso ? "ON" : "OFF");
        if (rel_fino   != rf_prev) ESP_LOGI(TAG, "RELE FINO    -> %s", rel_fino   ? "ON" : "OFF");
        if (rel_desc   != rd_prev) ESP_LOGI(TAG, "RELE DESCARGA-> %s", rel_desc   ? "ON" : "OFF");
        rg_prev = rel_grueso; rf_prev = rel_fino; rd_prev = rel_desc;

        control_update(rel_grueso, rel_fino, rel_desc);

        /* [SIM] Refleja en el simulador el estado de las válvulas */
        #if MODO_SIMULADO
        sim_set_valvulas(rel_grueso, rel_fino, rel_desc);
        #endif

        /* Log periódico de peso cada 500 ms */
        if ((xTaskGetTickCount() - t_ultimo_log_peso) > pdMS_TO_TICKS(500)) {
            t_ultimo_log_peso = xTaskGetTickCount();
            ESP_LOGI(TAG, "Peso: %6.2f kg | Estado: %-16s | G=%d F=%d D=%d | Pedal=%d",
                     peso, estado_str(estado), rel_grueso, rel_fino, rel_desc, pedal);
        }

        /* LCD cada 200 ms */
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