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

    // Formato serie: I2S, 16-bit, MSB a segundo flanco (Philips)
    // Page 0 Reg 0x03: Serial Interface (I2S)
    ssm2518_write_reg(0x00, 0x03, 0x00); // I2S estándar, 16-bit

    // Sample Rate / PLL: si no hay MCLK, usar PLL interna desde BCLK.
    // Muchos diseños operan bien sin configurar explícitamente; el PLL se ajusta por autodetección.
    // Si necesitas forzar 44.1k/48k, colocar registros de PLL según datasheet.

    // Volumen/ganancia: Page 0 Reg 0x05/0x06 (L/R)
    ssm2518_write_reg(0x00, 0x05, 0x00); // 0 dB aprox (depende de formato del registro)
    ssm2518_write_reg(0x00, 0x06, 0x00);

    // Desmutear salidas si es necesario (ej. Reg 0x09)
    ssm2518_write_reg(0x00, 0x09, 0x00);

    ESP_LOGI(TAG, "SSM2518 configurado (modo I2S 16-bit)");
}
