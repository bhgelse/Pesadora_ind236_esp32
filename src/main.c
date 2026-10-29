#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "ind236_driver.h"
#include "control_logic.h"

/* ================= PARÁMETROS AJUSTABLES ================= */
#define SCAN_MS               10      /* ciclo máximo de control (además corre al llegar cada peso) */
#define PESO_TIMEOUT_MS       500     /* sin trama del IND236 => peso inválido */
#define PESO_VACIO            1.0f    /* kg: balde considerado vacío */
#define LLENADO_MAX_MS        60000
#define DESC_MIN_MS           1500
#define DESC_MAX_MS           15000
#define HMI_PERIODO_MS        100
#define MANUAL_REQUIERE_PESO  0       /* 0 = permite probar compuertas sin IND236; poner 1 en producción */

typedef struct { float obj, corte, cg, cf, cero, span; } cfg_t;
static cfg_t cfg = { 50.0f, 40.0f, 0.5f, 0.3f, 0.0f, 1.0f };

typedef enum { EST_IDLE, EST_LLENANDO, EST_LLENO, EST_DESCARGANDO } estado_t;
enum { ALM_NONE, ALM_SOBRE, ALM_SIN_PESO, ALM_T_LLENADO, ALM_DESC_INC };

/* ---- datos compartidos entre tareas ---- */
static volatile float    g_raw = 0;        /* peso crudo del IND236 */
static volatile uint32_t g_raw_ms = 0;
static volatile bool     g_raw_ok = false;
static volatile bool     cfg_dirty = false;
static TaskHandle_t  h_ctl;
static QueueHandle_t q_cmd;

/* ---- comandos de la HMI (se ejecutan dentro del ciclo de control) ---- */
typedef enum { C_MODO, C_MAN, C_CFG, C_START, C_STOP, C_ACK, C_PEDAL,
               C_BYP_PED, C_BYP_INI, C_ZERO, C_SPAN } ctype_t;
typedef struct { ctype_t t; int i; bool v; float a, b, c, d; } cmd_t;

/* ---- memoria no volátil ---- */
static void cfg_save(void) {
    nvs_handle_t h; cfg_t t = cfg;
    if (nvs_open("emp", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_blob(h, "cfg", &t, sizeof t); nvs_commit(h); nvs_close(h);
    }
}
static void cfg_load(void) {
    nvs_handle_t h; cfg_t t; size_t n = sizeof t;
    if (nvs_open("emp", NVS_READONLY, &h) == ESP_OK) {
        if (nvs_get_blob(h, "cfg", &t, &n) == ESP_OK && n == sizeof t &&
            t.obj > 0 && t.corte <= t.obj && t.span > 0) cfg = t;
        nvs_close(h);
    }
}

/* ---- entradas con antirrebote ---- */
typedef struct { bool st, raw; uint32_t t; uint16_t on_ms, off_ms; } din_t;
static bool din(din_t *d, bool raw, uint32_t now) {
    if (raw != d->raw) { d->raw = raw; d->t = now; }
    if (raw != d->st && (now - d->t) >= (raw ? d->on_ms : d->off_ms)) d->st = raw;
    return d->st;
}

/* ---- alarmas: se enganchan por flanco; los enclavamientos son por nivel ---- */
static uint8_t  alarma = ALM_NONE;
static uint32_t cprev = 0;
static void alm_check(int bit, bool cond, uint8_t code) {
    bool pv = (cprev >> bit) & 1u;
    if (cond && !pv && !alarma) alarma = code;
    if (cond) cprev |= (1u << bit); else cprev &= ~(1u << bit);
}

/* ---- salida hacia la HMI (una sola escritura, sin mezclar logs) ---- */
static void hmi_send(float peso, bool g, bool f, bool d, bool aut, int est, bool ini,
                     bool par, bool ped, bool pok, int man, bool run) {
    char b[320];
    int n = snprintf(b, sizeof b,
        "{\"peso\":%.2f,\"grueso\":%d,\"fino\":%d,\"descarga\":%d,\"auto\":%d,\"estado\":%d,"
        "\"bIni\":%d,\"bPar\":%d,\"pedal\":%d,\"alarma\":%d,\"run\":%d,\"pOk\":%d,\"man\":%d,"
        "\"obj\":%.2f,\"corte\":%.2f,\"cg\":%.2f,\"cf\":%.2f}\n",
        peso, g, f, d, aut, est, ini, par, ped, alarma, run, pok, man,
        cfg.obj, cfg.corte, cfg.cg, cfg.cf);
    if (n > (int)sizeof b - 1) n = sizeof b - 1;
    uart_write_bytes(UART_NUM_0, b, n);
}

/* ================= TAREA: lectura del IND236 ================= */
static void task_ind(void *arg) {
    ind236_init();
    for (;;) {
        float p;
        if (ind236_read_continuous(&p, pdMS_TO_TICKS(100)) == ESP_OK) {
            g_raw = p;
            g_raw_ms = (uint32_t)(esp_timer_get_time() / 1000);
            g_raw_ok = true;
            xTaskNotifyGive(h_ctl);              /* despierta el control al instante */
        }
    }
}

/* ================= TAREA: comandos de la HMI ================= */
static bool jnum(const char *s, const char *k, float *o) {
    char p[24]; snprintf(p, sizeof p, "\"%s\":", k);
    const char *q = strstr(s, p); if (!q) return false;
    *o = strtof(q + strlen(p), NULL); return true;
}
static bool jtrue(const char *s, const char *k) {
    char p[24]; snprintf(p, sizeof p, "\"%s\":true", k); return strstr(s, p) != NULL;
}
static bool jhas(const char *s, const char *k, const char *v) {
    char p[48]; snprintf(p, sizeof p, "\"%s\":\"%s\"", k, v); return strstr(s, p) != NULL;
}
static void enviar(cmd_t c) { xQueueSend(q_cmd, &c, 0); }

static void parse_cmd(const char *s) {
    cmd_t c; memset(&c, 0, sizeof c);
    if (jhas(s, "cmd", "mode"))        { c.t = C_MODO;  c.v = jhas(s, "value", "auto"); enviar(c); }
    else if (jhas(s, "cmd", "manual")) { c.t = C_MAN;   c.v = jtrue(s, "value");
        c.i = jhas(s, "valvula", "grueso") ? 0 : jhas(s, "valvula", "fino") ? 1 : 2; enviar(c); }
    else if (jhas(s, "cmd", "config")) { c.t = C_CFG;
        if (jnum(s, "objetivo", &c.a) && jnum(s, "grueso", &c.b) &&
            jnum(s, "colaG", &c.c) && jnum(s, "colaF", &c.d)) enviar(c); }
    else if (jhas(s, "cmd", "start"))  { c.t = C_START; enviar(c); }
    else if (jhas(s, "cmd", "stop"))   { c.t = C_STOP;  enviar(c); }
    else if (jhas(s, "cmd", "ack"))    { c.t = C_ACK;   enviar(c); }
    else if (jhas(s, "cmd", "pedal"))  { c.t = C_PEDAL; c.v = jtrue(s, "value"); enviar(c); }
    else if (jhas(s, "cmd", "bypass")) {
        if (strstr(s, "\"pedal\":"))  { c.t = C_BYP_PED; c.v = jtrue(s, "pedal");  enviar(c); }
        if (strstr(s, "\"inicio\":")) { c.t = C_BYP_INI; c.v = jtrue(s, "inicio"); enviar(c); } }
    else if (jhas(s, "cmd", "zero"))   { c.t = C_ZERO;  enviar(c); }
    else if (jhas(s, "cmd", "span"))   { if (jnum(s, "patron", &c.a)) { c.t = C_SPAN; enviar(c); } }
}

static void task_hmi_rx(void *arg) {
    char l[200]; int n = 0;
    for (;;) {
        uint8_t ch;
        if (uart_read_bytes(UART_NUM_0, &ch, 1, pdMS_TO_TICKS(50)) == 1) {   /* 1 byte: sin latencia */
            if (ch == '\n' || ch == '\r') { if (n > 0) { l[n] = 0; parse_cmd(l); } n = 0; }
            else if (n < (int)sizeof l - 1) l[n++] = (char)ch;
        } else if (cfg_dirty) { cfg_dirty = false; cfg_save(); }  /* guarda en flash sin molestar al control */
    }
}

/* ================= TAREA: CONTROL (ciclo tipo PLC) ================= */
static void task_control(void *arg) {
    din_t d_ped = {0,0,0,20,40}, d_ini = {0,0,0,20,20}, d_par = {0,0,0,0,60}, d_rst = {0,0,0,20,20};
    bool modo_auto = true, run_hmi = false, byp_ped = false, byp_ini = false, ped_virt = false;
    bool man[3] = {false, false, false}, fase_fino = false;
    bool ini_prev = false, par_prev = false, rst_prev = false, ped_prev = false;
    uint8_t alm_prev = ALM_NONE;
    estado_t estado = EST_IDLE;
    uint32_t t_est = 0, t_hmi = 0, t_pv = 0, sig_prev = 0xFFFFFFFFu;

    for (;;) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(SCAN_MS));
        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
        bool start_evt = false, stop_evt = false, ack_evt = false;

        /* 1) comandos de la HMI */
        cmd_t c;
        while (xQueueReceive(q_cmd, &c, 0) == pdTRUE) {
            switch (c.t) {
            case C_MODO:  modo_auto = c.v; estado = EST_IDLE; run_hmi = false;
                          man[0] = man[1] = man[2] = false; break;
            case C_MAN:   man[c.i] = c.v; break;
            case C_CFG:   if (c.a > 0 && c.b >= 0 && c.b <= c.a && c.c >= 0 && c.d >= 0) {
                              cfg.obj = c.a; cfg.corte = c.b; cfg.cg = c.c; cfg.cf = c.d; cfg_dirty = true; } break;
            case C_START: start_evt = true; break;
            case C_STOP:  stop_evt = true; break;
            case C_ACK:   ack_evt = true; break;
            case C_PEDAL: ped_virt = c.v; if (c.v) t_pv = now; break;
            case C_BYP_PED: byp_ped = c.v; break;
            case C_BYP_INI: byp_ini = c.v; break;
            case C_ZERO:  cfg.cero = g_raw; cfg_dirty = true; break;
            case C_SPAN:  { float dd = g_raw - cfg.cero;
                            if (fabsf(dd) > 1.0f && c.a > 0) { cfg.span = c.a / dd; cfg_dirty = true; } } break;
            }
        }
        if (ped_virt && (now - t_pv) > 3000) ped_virt = false;   /* si la HMI se cuelga, se suelta solo */

        /* 2) entradas */
        bool ini = din(&d_ini, control_boton_inicio(), now) || byp_ini;
        bool par = din(&d_par, control_boton_paro(), now);
        bool rst = din(&d_rst, control_boton_reset(), now);
        bool ped = din(&d_ped, control_pedal_descarga(), now) || ped_virt || byp_ped;
        bool ini_rise = ini && !ini_prev, par_rise = par && !par_prev;
        bool rst_rise = rst && !rst_prev, ped_rise = ped && !ped_prev;

        /* 3) peso */
        bool peso_ok = g_raw_ok && ((now - g_raw_ms) < PESO_TIMEOUT_MS);
        float peso = (g_raw - cfg.cero) * cfg.span;
        float lim = cfg.obj + fmaxf(2.0f, 0.1f * cfg.obj);
        bool ovf = peso_ok && peso > lim;
        bool need_peso = modo_auto || MANUAL_REQUIERE_PESO;

        /* 4) PARO: actúa en este mismo ciclo, en cualquier modo y estado */
        if (par_rise || stop_evt) {
            estado = EST_IDLE; run_hmi = false; man[0] = man[1] = man[2] = false;
        }
        if (ack_evt || rst_rise) alarma = ALM_NONE;

        /* 5) alarmas */
        bool fill_act = modo_auto ? (estado == EST_LLENANDO) : (man[0] || man[1]);
        alm_check(0, ovf, ALM_SOBRE);
        alm_check(1, need_peso && !peso_ok && fill_act, ALM_SIN_PESO);

        /* 6) secuencia */
        bool rg = false, rf = false, rd = false;
        if (start_evt && !alarma && !par) run_hmi = true;
        if (modo_auto) {
            switch (estado) {
            case EST_IDLE:
                if ((start_evt || ini_rise) && !alarma && !par) { estado = EST_LLENANDO; fase_fino = false; t_est = now; }
                break;
            case EST_LLENANDO:
                if (!fase_fino && peso >= cfg.corte - cfg.cg) fase_fino = true;       /* cierra grueso */
                if (fase_fino && peso >= cfg.obj - cfg.cf) { estado = EST_LLENO; break; }
                rg = !fase_fino; rf = true;                                          /* grueso+fino, luego solo fino */
                if ((now - t_est) > LLENADO_MAX_MS) { alarma = ALM_T_LLENADO; estado = EST_IDLE; }
                break;
            case EST_LLENO:
                if (ped_rise || (byp_ped && ped)) { estado = EST_DESCARGANDO; t_est = now; }
                break;
            case EST_DESCARGANDO:
                rd = true;                                       /* sigue aunque se suelte el pedal */
                if ((now - t_est) >= DESC_MIN_MS && peso_ok && peso <= PESO_VACIO) {
                    rd = false;
                    estado = ((ini || run_hmi) && !alarma && !par) ? EST_LLENANDO : EST_IDLE;
                    fase_fino = false; t_est = now;
                } else if ((now - t_est) >= DESC_MAX_MS) {
                    alarma = ALM_DESC_INC; estado = EST_IDLE; rd = false;
                }
                break;
            }
        } else {
            rg = man[0]; rf = man[1]; rd = man[2] && ped;        /* manual: la descarga exige pedal */
        }

        /* alarma nueva: cierra el llenado y corta el ciclo (la descarga en curso se respeta) */
        if (alarma && !alm_prev) {
            run_hmi = false; man[0] = man[1] = false; rg = rf = false;
            if (alarma != ALM_SOBRE || estado == EST_LLENANDO) estado = EST_IDLE;
        }
        alm_prev = alarma;

        /* 7) ENCLAVAMIENTOS (prioridad absoluta, por nivel) y salidas */
        bool inhibe = (need_peso && !peso_ok) || ovf || alarma || par;
        bool og = rg && !inhibe, of = rf && !inhibe, od = rd && !par;
        if (od) og = of = false;                                  /* nunca llenar y descargar a la vez */
        control_update(og, of, od);

        /* 8) HMI: al cambiar algo y cada 100 ms */
        int manbits = man[0] | (man[1] << 1) | (man[2] << 2);
        uint32_t sig = (uint32_t)og | ((uint32_t)of << 1) | ((uint32_t)od << 2) | ((uint32_t)estado << 3) |
                       ((uint32_t)alarma << 6) | ((uint32_t)modo_auto << 9) | ((uint32_t)ped << 10) |
                       ((uint32_t)peso_ok << 11) | ((uint32_t)par << 12) | ((uint32_t)manbits << 13);
        if (sig != sig_prev || (now - t_hmi) >= HMI_PERIODO_MS) {
            sig_prev = sig; t_hmi = now;
            hmi_send(peso, og, of, od, modo_auto, (int)estado, ini, par, ped, peso_ok, manbits, estado != EST_IDLE);
        }

        ini_prev = ini; par_prev = par; rst_prev = rst; ped_prev = ped;
    }
}

/* ================= APP MAIN ================= */
void app_main(void) {
    esp_log_level_set("*", ESP_LOG_NONE);          /* el puerto serial queda solo para JSON */

    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) { nvs_flash_erase(); nvs_flash_init(); }
    cfg_load();

    control_init();                                 /* todo apagado al arrancar */

    uart_config_t uc = { .baud_rate = 115200, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE,
                         .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
                         .source_clk = UART_SCLK_DEFAULT };
    uart_param_config(UART_NUM_0, &uc);
    uart_driver_install(UART_NUM_0, 1024, 1024, 0, NULL, 0);   /* buffer de TX: la escritura no bloquea el control */

    q_cmd = xQueueCreate(16, sizeof(cmd_t));
    xTaskCreatePinnedToCore(task_control, "ctl", 6144, NULL, 11, &h_ctl, 1);
    xTaskCreatePinnedToCore(task_ind, "ind", 4096, NULL, 12, NULL, 1);
    xTaskCreate(task_hmi_rx, "hmi_rx", 4096, NULL, 5, NULL);
}
