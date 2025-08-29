// Nota: Configuración aproximada basada en hoja de datos SSM2518 (Analog Devices).
// Ajusta si tu hardware requiere otros pines o modo TDM.

#include "ssm2518.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_MASTER_NUM       I2C_NUM_0
static uint8_t s_ssm2518_addr7 = 0x34;

static const char *TAG = "SSM2518";

static esp_err_t ssm2518_write_reg(uint8_t page, uint8_t reg, uint8_t val)
{
    // Selección de página
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    uint8_t addr8 = (s_ssm2518_addr7 << 1) | I2C_MASTER_WRITE;
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, addr8, true);
    i2c_master_write_byte(cmd, 0x00, true); // Page Select Register
    i2c_master_write_byte(cmd, page, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 100 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) return ret;

    // Escritura del registro
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, addr8, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, val, true);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 100 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t ssm2518_read_reg(uint8_t page, uint8_t reg, uint8_t *out_val)
{
    if (!out_val) return ESP_ERR_INVALID_ARG;
    // Selección de página
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    uint8_t addr8 = (s_ssm2518_addr7 << 1) | I2C_MASTER_WRITE;
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, addr8, true);
    i2c_master_write_byte(cmd, 0x00, true); // Page Select Register
    i2c_master_write_byte(cmd, page, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 100 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) return ret;

    // Lectura del registro
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, addr8, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (s_ssm2518_addr7 << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, out_val, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 100 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static void ssm2518_dump_page0(void)
{
    uint8_t val = 0;
    for (uint8_t r = 0x00; r <= 0x10; ++r) {
        if (ssm2518_read_reg(0x00, r, &val) == ESP_OK) {
            ESP_LOGI(TAG, "P0[%02X]=0x%02X", r, val);
        } else {
            ESP_LOGW(TAG, "P0[%02X] read failed", r);
        }
    }
}

void ssm2518_set_address(uint8_t addr7)
{
    s_ssm2518_addr7 = addr7 & 0x7F;
}

void ssm2518_configure(void)
{
    esp_err_t ret;

    // Software reset si aplica (Page 0, Reg 0x01 bit0)
    ret = ssm2518_write_reg(0x00, 0x01, 0x01);
    if (ret != ESP_OK) ESP_LOGW(TAG, "Reset write failed: %s", esp_err_to_name(ret));
    vTaskDelay(pdMS_TO_TICKS(10));

    // Power management / enable blocks (valores típicos, revisar hardware)
    // Page 0 Reg 0x02: Power Control
    ssm2518_write_reg(0x00, 0x02, 0x00); // Desmute, power-up bloques por defecto

    // Formato serie: I2S Philips, 16-bit datos, slots de 16-bit (más seguro por defecto)
    // Page 0 Reg 0x03: Serial Interface #1 (modo I2S, polaridades por defecto)
    ssm2518_write_reg(0x00, 0x03, 0x00);
    // Page 0 Reg 0x04: Serial Interface #2 (word/slot length). 0x00 típico: 16-bit
    ssm2518_write_reg(0x00, 0x04, 0x00);

    // Sample Rate / PLL: si no hay MCLK, usar PLL interna desde BCLK.
    // TODO: Programar registros PLL exactos según datasheet si se opera sin MCLK.

    // Volumen/ganancia: Page 0 Reg 0x05/0x06 (L/R)
    ssm2518_write_reg(0x00, 0x05, 0x00); // 0 dB aprox (depende de formato del registro)
    ssm2518_write_reg(0x00, 0x06, 0x00);

    // Desmutear salidas si es necesario (ej. Reg 0x09)
    ssm2518_write_reg(0x00, 0x09, 0x00);

    ssm2518_dump_page0();
    ESP_LOGI(TAG, "SSM2518 configurado (modo I2S 16-bit)");
}
