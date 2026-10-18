#include "ui_hardware.h"
#include <stdio.h>
#include <string.h>
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ===== Configuración I2C del LCD =====
#define I2C_MASTER_NUM      I2C_NUM_0
#define I2C_MASTER_SDA      GPIO_NUM_23   // <-- cámbialo si usas otro pin
#define I2C_MASTER_SCL      GPIO_NUM_22   // <-- cámbialo si usas otro pin
#define I2C_MASTER_FREQ_HZ  100000

// Dirección típica del PCF8574 (0x27 o 0x3F)
#define LCD_ADDR            0x27

// Bits del PCF8574 -> LCD
#define LCD_BACKLIGHT       0x08
#define LCD_EN              0x04
#define LCD_RW              0x02
#define LCD_RS              0x01

static void lcd_write_byte(uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LCD_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, data | LCD_BACKLIGHT, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
}

static void lcd_pulse_enable(uint8_t data) {
    lcd_write_byte(data | LCD_EN);
    vTaskDelay(pdMS_TO_TICKS(1));
    lcd_write_byte(data & ~LCD_EN);
    vTaskDelay(pdMS_TO_TICKS(1));
}

static void lcd_send_nibble(uint8_t nibble, bool rs) {
    uint8_t data = (nibble << 4) | (rs ? LCD_RS : 0);
    lcd_write_byte(data);
    lcd_pulse_enable(data);
}

static void lcd_send_byte(uint8_t byte, bool rs) {
    lcd_send_nibble((byte >> 4) & 0x0F, rs);
    lcd_send_nibble(byte & 0x0F, rs);
}

static void lcd_cmd(uint8_t cmd) {
    lcd_send_byte(cmd, false);
    vTaskDelay(pdMS_TO_TICKS(2));
}

static void lcd_data(uint8_t data) {
    lcd_send_byte(data, true);
}

static void lcd_set_cursor(uint8_t col, uint8_t row) {
    uint8_t addr = (row == 0) ? 0x80 : 0xC0;
    lcd_cmd(addr | col);
}

static void lcd_print(const char *str) {
    while (*str) lcd_data((uint8_t)*str++);
}

// ===== API pública =====
void ui_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA,
        .scl_io_num = I2C_MASTER_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);

    vTaskDelay(pdMS_TO_TICKS(50));
    // Secuencia de inicialización en modo 4 bits
    lcd_send_nibble(0x03, false); vTaskDelay(pdMS_TO_TICKS(5));
    lcd_send_nibble(0x03, false); vTaskDelay(pdMS_TO_TICKS(5));
    lcd_send_nibble(0x03, false); vTaskDelay(pdMS_TO_TICKS(1));
    lcd_send_nibble(0x02, false); // modo 4 bits

    lcd_cmd(0x28); // 4 bits, 2 líneas, 5x8
    lcd_cmd(0x0C); // display ON, cursor OFF
    lcd_cmd(0x06); // incremento automático
    lcd_cmd(0x01); // clear
    vTaskDelay(pdMS_TO_TICKS(2));
}

void ui_mostrar_peso(float peso, const char *etapa) {
    char linea1[17];
    char linea2[17];
    snprintf(linea1, sizeof(linea1), "P:%7.2f kg", peso);
    snprintf(linea2, sizeof(linea2), "ETAPA:%-9s", etapa);

    lcd_set_cursor(0, 0);
    lcd_print("                "); // limpiar línea
    lcd_set_cursor(0, 0);
    lcd_print(linea1);

    lcd_set_cursor(0, 1);
    lcd_print("                ");
    lcd_set_cursor(0, 1);
    lcd_print(linea2);
}

void ui_mostrar_mensaje(const char *linea1, const char *linea2) {
    lcd_set_cursor(0, 0);
    lcd_print("                ");
    lcd_set_cursor(0, 0);
    lcd_print(linea1);

    lcd_set_cursor(0, 1);
    lcd_print("                ");
    lcd_set_cursor(0, 1);
    lcd_print(linea2);
}