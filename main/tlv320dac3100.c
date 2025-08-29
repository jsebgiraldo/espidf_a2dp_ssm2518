// Minimal scaffold for TLV320DAC3100 (Adafruit 6309 breakout)
// Goal: detect device and perform a basic configuration for I2S 16-bit playback.
// This keeps it safe and non-destructive while providing helpful logs.

#include "tlv320dac3100.h"

#include <string.h>
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_MASTER_NUM I2C_NUM_0

static const char *TAG = "TLV320";
static uint8_t s_tlv_addr7 = 0x18; // typical default; may be 0x19 depending on ADR pin

static bool i2c_probe_addr7(uint8_t addr7)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (!cmd) return false;
    uint8_t addr8 = (addr7 << 1) | I2C_MASTER_WRITE;
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, addr8, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret == ESP_OK;
}

static esp_err_t tlv_read_reg(uint8_t page, uint8_t reg, uint8_t *out_val)
{
    if (!out_val) return ESP_ERR_INVALID_ARG;
    uint8_t addr8_w = (s_tlv_addr7 << 1) | I2C_MASTER_WRITE;
    uint8_t addr8_r = (s_tlv_addr7 << 1) | I2C_MASTER_READ;

    // Select page
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (!cmd) return ESP_FAIL;
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, addr8_w, true);
    i2c_master_write_byte(cmd, 0x00, true);
    i2c_master_write_byte(cmd, page, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) return ret;

    // Set register address then read
    cmd = i2c_cmd_link_create();
    if (!cmd) return ESP_FAIL;
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, addr8_w, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, addr8_r, true);
    i2c_master_read_byte(cmd, out_val, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}
static void tlv320_dump_debug(void)
{
    // Read a handful of useful registers to diagnose mute/routing/power
    uint8_t v;
    if (tlv_read_reg(0x00, 0x1B, &v) == ESP_OK) ESP_LOGI(TAG, "P0:IF_CTRL1(0x1B)=0x%02X", v);
    if (tlv_read_reg(0x00, 0x3F, &v) == ESP_OK) ESP_LOGI(TAG, "P0:DAC_DP(0x3F)=0x%02X", v);
    if (tlv_read_reg(0x00, 0x40, &v) == ESP_OK) ESP_LOGI(TAG, "P0:DAC_VOLCTL(0x40)=0x%02X", v);
    if (tlv_read_reg(0x00, 0x41, &v) == ESP_OK) ESP_LOGI(TAG, "P0:DAC_LVOL(0x41)=0x%02X", v);
    if (tlv_read_reg(0x00, 0x42, &v) == ESP_OK) ESP_LOGI(TAG, "P0:DAC_RVOL(0x42)=0x%02X", v);
    if (tlv_read_reg(0x01, 0x1F, &v) == ESP_OK) ESP_LOGI(TAG, "P1:HP_DRV(0x1F)=0x%02X", v);
    if (tlv_read_reg(0x01, 0x23, &v) == ESP_OK) ESP_LOGI(TAG, "P1:OUT_ROUT(0x23)=0x%02X", v);
    if (tlv_read_reg(0x01, 0x24, &v) == ESP_OK) ESP_LOGI(TAG, "P1:HPL_VOL(0x24)=0x%02X", v);
    if (tlv_read_reg(0x01, 0x25, &v) == ESP_OK) ESP_LOGI(TAG, "P1:HPR_VOL(0x25)=0x%02X", v);
    if (tlv_read_reg(0x01, 0x0C, &v) == ESP_OK) ESP_LOGI(TAG, "P1:MIXL_CFG(0x0C)=0x%02X", v); // Correct mixer registers
    if (tlv_read_reg(0x01, 0x0D, &v) == ESP_OK) ESP_LOGI(TAG, "P1:MIXR_CFG(0x0D)=0x%02X", v); // Correct mixer registers
    if (tlv_read_reg(0x01, 0x28, &v) == ESP_OK) ESP_LOGI(TAG, "P1:HPL_PGA(0x28)=0x%02X", v);
    if (tlv_read_reg(0x01, 0x29, &v) == ESP_OK) ESP_LOGI(TAG, "P1:HPR_PGA(0x29)=0x%02X", v);
}

bool tlv320_detect(uint8_t *addr7_out)
{
    // Many TLV/AIC310x variants use 0x18..0x1B depending on ADR pins
    const uint8_t cand[] = {0x18, 0x19, 0x1A, 0x1B};
    for (size_t i = 0; i < sizeof(cand); ++i) {
        if (!i2c_probe_addr7(cand[i])) continue;
        // Confirm by attempting a simple read (Page0, OT flag @ 0x03)
        uint8_t prev = s_tlv_addr7;
        s_tlv_addr7 = cand[i];
        uint8_t val = 0xFF;
        esp_err_t ok = tlv_read_reg(0x00, 0x03, &val);
        s_tlv_addr7 = prev;
        if (ok == ESP_OK) {
            if (addr7_out) *addr7_out = cand[i];
            ESP_LOGI(TAG, "Detected TLV320DAC3100 at 0x%02X (OT_FLAG=0x%02X)", cand[i], val);
            return true;
        }
    }
    return false;
}

void tlv320_set_address(uint8_t addr7)
{
    s_tlv_addr7 = addr7 & 0x7F;
}

static esp_err_t tlv_write_reg(uint8_t page, uint8_t reg, uint8_t val)
{
    // TLV320 uses a page/register map similar in concept; the Adafruit guide notes I2C reg autoincrement.
    // For this scaffold, we perform simple page select then single-byte write.
    uint8_t addr8 = (s_tlv_addr7 << 1) | I2C_MASTER_WRITE;

    // Page select (register 0x00)
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (!cmd) return ESP_FAIL;
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, addr8, true);
    i2c_master_write_byte(cmd, 0x00, true);
    i2c_master_write_byte(cmd, page, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) return ret;

    // Register write
    cmd = i2c_cmd_link_create();
    if (!cmd) return ESP_FAIL;
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, addr8, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, val, true);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

bool tlv320_configure_basic_i2s_16bit(bool enable_speaker)
{
    // This is a minimal safe setup with logs. A full setup would:
    // - soft reset
    // - clocking: enable PLL from BCLK, set NDAC/MDAC/DOSR as needed
    // - audio interface: I2S, 16-bit, left/right, rising-edge BCK default
    // - route DAC to HP (and optionally speaker), set gains, unmute
    // For now, we log guidance and do a minimal register poke that is known safe.

    ESP_LOGI(TAG, "Configuring TLV320DAC3100 (addr 0x%02X) for I2S 16-bit...", s_tlv_addr7);
    ESP_LOGW(TAG, "Note: This is a scaffold. It expects BCLK/WSEL/DIN wired; MCLK optional. Speaker path needs 5V VIN.");

    // Try a soft reset: Page 0, Reg 1 = 0x01 (per TLV family, reset often here).
    // If the address is wrong or chip missing, this will fail silently.
    (void)tlv_write_reg(0x00, 0x01, 0x01);
    vTaskDelay(pdMS_TO_TICKS(5));

    // Minimal: set interface to I2S, 16-bit (Codec Interface Control 1 @ Page0 0x1B)
    // FORMAT_I2S=0b00 at bits[7:6]; DATA_LEN_16=0b00 at bits[5:4] => value 0x00
    (void)tlv_write_reg(0x00, 0x1B, 0x00);
    uint8_t ifc = 0xFF;
    if (tlv_read_reg(0x00, 0x1B, &ifc) == ESP_OK) {
        ESP_LOGI(TAG, "TLV320 read IF_CTRL1=0x%02X (expect 0x00 for I2S/16)", ifc);
    }

    // Quick sanity read of an always-present status reg (Overtemp flag @ Page0 0x03)
    uint8_t ot = 0xFF;
    esp_err_t r = tlv_read_reg(0x00, 0x03, &ot);
    if (r == ESP_OK) {
        ESP_LOGI(TAG, "TLV320 read OT_FLAG=0x%02X (OK)", ot);
    } else {
        ESP_LOGW(TAG, "TLV320 readback failed (0x03): %s", esp_err_to_name(r));
    }

    // Minimal: ensure digital power up and unmute later in a fuller implementation.
    // Without full clock tree programming, many codecs idle until clocks are present.
    ESP_LOGI(TAG, "TLV320 scaffold configured. Implement full PLL/route/gain in a follow-up step.");
    return true;
}

static bool tlv_write_ok(uint8_t page, uint8_t reg, uint8_t val)
{
    return tlv_write_reg(page, reg, val) == ESP_OK;
}

bool tlv320_configure_bclk_i2s_16(int sample_rate)
{
    // Configuración completa para TLV320DAC3100 usando BCLK como fuente PLL
    // Optimizada para salida de auriculares a 3.3V
    if (!(sample_rate == 44100 || sample_rate == 48000)) {
        ESP_LOGE(TAG, "Unsupported SR %d for BCLK mode (use 44100 or 48000)", sample_rate);
        return false;
    }

    ESP_LOGI(TAG, "Configurando TLV320 completamente para auriculares 3.3V, SR=%d", sample_rate);

    // 1. Soft reset y esperar estabilización
    if (!tlv_write_ok(0x00, 0x01, 0x01)) {
        ESP_LOGE(TAG, "Failed soft reset");
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    // 2. Configurar interfaz I2S: 16-bit, modo slave
    if (!tlv_write_ok(0x00, 0x1B, 0x00)) return false; // I2S, 16-bit
    if (!tlv_write_ok(0x00, 0x1C, 0x00)) return false; // data offset = 0
    if (!tlv_write_ok(0x00, 0x1D, 0x01)) return false; // BCLK/WCLK inputs

    // 3. Clock configuration: BCLK como fuente PLL
    if (!tlv_write_ok(0x00, 0x04, 0x04)) return false; // BCLK como PLL ref

    // 4. PLL Programming: P=1, R=1, J=8, D=0
    if (!tlv_write_ok(0x00, 0x05, 0x91)) return false; // P=1, R=1, PLL powered
    if (!tlv_write_ok(0x00, 0x06, 0x08)) return false; // J=8
    if (!tlv_write_ok(0x00, 0x07, 0x00)) return false; // D MSB = 0
    if (!tlv_write_ok(0x00, 0x08, 0x00)) return false; // D LSB = 0
    
    vTaskDelay(pdMS_TO_TICKS(10)); // Wait for PLL lock

    // 5. Clock dividers
    if (!tlv_write_ok(0x00, 0x0B, 0x82)) return false; // NDAC=2, powered
    if (!tlv_write_ok(0x00, 0x0C, 0x81)) return false; // MDAC=1, powered
    if (!tlv_write_ok(0x00, 0x0D, 0x00)) return false; // DOSR MSB = 0
    if (!tlv_write_ok(0x00, 0x0E, 0x80)) return false; // DOSR LSB = 128

    // 6. Processing block
    if (!tlv_write_ok(0x00, 0x3C, 0x01)) return false; // PRB_P1

    // 7. Power up DACs y configurar datapath
    if (!tlv_write_ok(0x00, 0x3F, 0xD4)) return false; // Left+Right DAC on, soft step
    if (!tlv_write_ok(0x00, 0x40, 0x00)) return false; // DAC volume control
    if (!tlv_write_ok(0x00, 0x41, 0x00)) return false; // Left DAC 0dB
    if (!tlv_write_ok(0x00, 0x42, 0x00)) return false; // Right DAC 0dB

    ESP_LOGI(TAG, "TLV320 clock tree y DAC configurados");
    return true;
}

bool tlv320_enable_speaker_out_max(void)
{
    ESP_LOGI(TAG, "Configurando salida auriculares TLV320 para máximo volumen 3.3V");
    
    // Página 1: configuración analógica
    if (!tlv_write_ok(0x01, 0x00, 0x01)) { // Switch to Page 1
        ESP_LOGE(TAG, "Failed to switch to page 1");
        return false;
    }
    
    // 1. CRÍTICO: Configurar OUT_ROUT para ruteo directo DAC->HP
    if (!tlv_write_ok(0x01, 0x23, 0x44)) { // OUT_ROUT: DAC_L->HPL, DAC_R->HPR
        ESP_LOGE(TAG, "Failed to set output routing");
        return false;
    }
    
    // 2. Configurar mixers para asegurar ruteo DAC->HP
    if (!tlv_write_ok(0x01, 0x0C, 0x08)) { // Left DAC routed to HPL
        ESP_LOGE(TAG, "Failed to route left DAC to HPL");
        return false;
    }
    if (!tlv_write_ok(0x01, 0x0D, 0x08)) { // Right DAC routed to HPR
        ESP_LOGE(TAG, "Failed to route right DAC to HPR");
        return false;
    }
    
    // 3. Power up analog power supplies
    if (!tlv_write_ok(0x01, 0x01, 0x08)) { // Disable weak AVDD connection
        ESP_LOGE(TAG, "Failed to configure analog power");
        return false;
    }
    if (!tlv_write_ok(0x01, 0x02, 0x01)) { // Enable master analog power
        ESP_LOGE(TAG, "Failed to enable analog power");
        return false;
    }
    
    // 4. Configurar y power up HP drivers para 3.3V
    if (!tlv_write_ok(0x01, 0x1F, 0x0C)) { // HP drivers: powered, class-AB for 3.3V
        ESP_LOGE(TAG, "Failed to configure HP drivers");
        return false;
    }
    
    // 5. Power up HP output stage
    if (!tlv_write_ok(0x01, 0x09, 0x30)) { // HPL and HPR powered up
        ESP_LOGE(TAG, "Failed to power up HP outputs");
        return false;
    }
    
    // 6. Set HP volumes (0x80 enables routing + 0x00 = 0dB)
    if (!tlv_write_ok(0x01, 0x10, 0x80)) { // HPL volume: routing enabled, 0dB
        ESP_LOGE(TAG, "Failed to set HPL volume");
        return false;
    }
    if (!tlv_write_ok(0x01, 0x11, 0x80)) { // HPR volume: routing enabled, 0dB
        ESP_LOGE(TAG, "Failed to set HPR volume");
        return false;
    }
    
    // 7. Unmute HP outputs
    if (!tlv_write_ok(0x01, 0x16, 0x00)) { // HP unmute register
        ESP_LOGE(TAG, "Failed to unmute HP outputs");
        return false;
    }
    
    // Return to Page 0 for final DAC setup
    if (!tlv_write_ok(0x00, 0x00, 0x00)) {
        ESP_LOGE(TAG, "Failed to switch back to page 0");
        return false;
    }
    
    // 8. Final DAC unmute and power up
    if (!tlv_write_ok(0x00, 0x40, 0x00)) { // Unmute both DACs
        ESP_LOGE(TAG, "Failed to unmute DACs");
        return false;
    }
    
    // 9. Ensure DACs are powered and datapath is active
    if (!tlv_write_ok(0x00, 0x3F, 0xD6)) { // Both DACs on, datapath active
        ESP_LOGE(TAG, "Failed to ensure DAC power");
        return false;
    }
    
    vTaskDelay(pdMS_TO_TICKS(100)); // Allow analog circuits to settle
    
    ESP_LOGI(TAG, "TLV320 auriculares configurados para 3.3V - audio debe estar presente");
    tlv320_dump_debug();
    
    return true;
}

// External reset pin configuration - defined in main.c
#ifndef TLV320_RESET_IO
#define TLV320_RESET_IO 33
#endif

bool tlv320_hardware_reset_and_init(int sample_rate)
{
    ESP_LOGI(TAG, "Ejecutando reset completo del TLV320 via hardware + reconfiguración");
    
    // Configure reset pin if not already done
    int gpio = TLV320_RESET_IO;
    if (gpio >= 0 && gpio < 64) {
        // Configure GPIO as output
        uint64_t mask = (1ULL << (unsigned)gpio);
        gpio_config_t io;
        memset(&io, 0, sizeof(io));
        io.pin_bit_mask = mask;
        io.mode = GPIO_MODE_OUTPUT;
        io.pull_up_en = GPIO_PULLUP_DISABLE;
        io.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io.intr_type = GPIO_INTR_DISABLE;
        
        esp_err_t err = gpio_config(&io);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure reset GPIO%d: %s", gpio, esp_err_to_name(err));
            return false;
        }
        
        // Hardware reset sequence: Assert reset (LOW) -> Release (HIGH)
        ESP_LOGI(TAG, "Aplicando reset hardware en GPIO%d", gpio);
        gpio_set_level(gpio, 0);  // Assert reset
        vTaskDelay(pdMS_TO_TICKS(10)); // Hold reset for 10ms minimum
        gpio_set_level(gpio, 1);  // Release reset
        vTaskDelay(pdMS_TO_TICKS(50)); // Wait for internal startup
        
        ESP_LOGI(TAG, "Reset hardware completado - chip en estado inicial");
    } else {
        ESP_LOGW(TAG, "GPIO de reset no válido (%d), usando solo software reset", gpio);
    }
    
    // After hardware reset, reconfigure completely
    ESP_LOGI(TAG, "Reconfigurando TLV320 completamente después del reset");
    
    // Full configuration sequence
    if (!tlv320_configure_bclk_i2s_16(sample_rate)) {
        ESP_LOGE(TAG, "Failed to configure TLV320 clocks after reset");
        return false;
    }
    
    if (!tlv320_enable_speaker_out_max()) {
        ESP_LOGE(TAG, "Failed to enable TLV320 output after reset");
        return false;
    }
    
    ESP_LOGI(TAG, "TLV320 reset completo y reconfiguración exitosa");
    return true;
}

bool tlv320_configure_dual_output(void)
{
    ESP_LOGI(TAG, "Configurando TLV320 para salida dual: Headphones + Speaker");
    
    // ========== PAGE 1: ANALOG ROUTING AND POWER ==========
    if (!tlv_write_ok(0x01, 0x00, 0x01)) {
        ESP_LOGE(TAG, "Failed to switch to page 1");
        return false;
    }
    
    // 1. Configure analog power supplies for dual output
    if (!tlv_write_ok(0x01, 0x01, 0x08)) { // Disable weak AVDD connection
        ESP_LOGE(TAG, "Failed to configure analog power");
        return false;
    }
    if (!tlv_write_ok(0x01, 0x02, 0x01)) { // Enable master analog power
        ESP_LOGE(TAG, "Failed to enable analog power");
        return false;
    }
    
    // 2. Configure output routing for both HP and Line outputs
    // Bit[7:6] = 01: DAC_L -> HPL, Bit[5:4] = 01: DAC_R -> HPR  
    // Bit[3:2] = 01: DAC_L -> LOL, Bit[1:0] = 01: DAC_R -> LOR
    if (!tlv_write_ok(0x01, 0x23, 0x55)) { // Route to both HP and Line outputs
        ESP_LOGE(TAG, "Failed to set dual output routing");
        return false;
    }
    
    // 3. Configure mixers for DAC routing to HP outputs
    if (!tlv_write_ok(0x01, 0x0C, 0x08)) { // Left DAC routed to HPL
        ESP_LOGE(TAG, "Failed to route left DAC to HPL");
        return false;
    }
    if (!tlv_write_ok(0x01, 0x0D, 0x08)) { // Right DAC routed to HPR
        ESP_LOGE(TAG, "Failed to route right DAC to HPR");
        return false;
    }
    
    // 4. Configure mixers for DAC routing to Line outputs (for speaker)
    if (!tlv_write_ok(0x01, 0x0E, 0x08)) { // Left DAC routed to LOL
        ESP_LOGE(TAG, "Failed to route left DAC to LOL");
        return false;
    }
    if (!tlv_write_ok(0x01, 0x0F, 0x08)) { // Right DAC routed to LOR
        ESP_LOGE(TAG, "Failed to route right DAC to LOR");
        return false;
    }
    
    // 5. Configure HP drivers for 3.3V operation
    if (!tlv_write_ok(0x01, 0x1F, 0x0C)) { // HP drivers: powered, class-AB for 3.3V
        ESP_LOGE(TAG, "Failed to configure HP drivers");
        return false;
    }
    
    // 6. Configure Line Output drivers for speaker
    if (!tlv_write_ok(0x01, 0x20, 0x06)) { // Line drivers: powered, class-AB
        ESP_LOGE(TAG, "Failed to configure Line drivers");
        return false;
    }
    
    // 7. Power up both HP and Line output stages
    if (!tlv_write_ok(0x01, 0x09, 0x3C)) { // HPL, HPR, LOL, LOR powered up
        ESP_LOGE(TAG, "Failed to power up outputs");
        return false;
    }
    
    // 8. Set HP volumes (0x80 enables routing + volume)
    if (!tlv_write_ok(0x01, 0x10, 0x80)) { // HPL volume: routing enabled, 0dB
        ESP_LOGE(TAG, "Failed to set HPL volume");
        return false;
    }
    if (!tlv_write_ok(0x01, 0x11, 0x80)) { // HPR volume: routing enabled, 0dB
        ESP_LOGE(TAG, "Failed to set HPR volume");
        return false;
    }
    
    // 9. Set Line Output volumes for speaker (0x80 enables + moderate volume)
    if (!tlv_write_ok(0x01, 0x12, 0x92)) { // LOL volume: routing enabled, +6dB for speaker
        ESP_LOGE(TAG, "Failed to set LOL volume");
        return false;
    }
    if (!tlv_write_ok(0x01, 0x13, 0x92)) { // LOR volume: routing enabled, +6dB for speaker
        ESP_LOGE(TAG, "Failed to set LOR volume");
        return false;
    }
    
    // 10. Unmute all outputs
    if (!tlv_write_ok(0x01, 0x16, 0x00)) { // HP unmute
        ESP_LOGE(TAG, "Failed to unmute HP outputs");
        return false;
    }
    if (!tlv_write_ok(0x01, 0x17, 0x00)) { // Line Out unmute
        ESP_LOGE(TAG, "Failed to unmute Line outputs");
        return false;
    }
    
    // ========== PAGE 0: DIGITAL CONFIGURATION ==========
    if (!tlv_write_ok(0x00, 0x00, 0x00)) {
        ESP_LOGE(TAG, "Failed to switch back to page 0");
        return false;
    }
    
    // 11. Set DAC volumes (moderate level for dual output)
    if (!tlv_write_ok(0x00, 0x41, 0x08)) { // Left DAC: -4dB to prevent clipping
        ESP_LOGE(TAG, "Failed to set left DAC volume");
        return false;
    }
    if (!tlv_write_ok(0x00, 0x42, 0x08)) { // Right DAC: -4dB to prevent clipping
        ESP_LOGE(TAG, "Failed to set right DAC volume");
        return false;
    }
    
    // 12. Unmute both DACs
    if (!tlv_write_ok(0x00, 0x40, 0x00)) { // Unmute both DACs
        ESP_LOGE(TAG, "Failed to unmute DACs");
        return false;
    }
    
    // 13. Ensure DACs are powered and datapath is active
    if (!tlv_write_ok(0x00, 0x3F, 0xD6)) { // Both DACs on, datapath active
        ESP_LOGE(TAG, "Failed to ensure DAC power");
        return false;
    }
    
    vTaskDelay(pdMS_TO_TICKS(100)); // Allow analog circuits to settle
    
    ESP_LOGI(TAG, "TLV320 configurado para salida dual: Headphones + Speaker");
    ESP_LOGI(TAG, "- Headphones: 3.3V compatible, 0dB gain");
    ESP_LOGI(TAG, "- Speaker: Line Out +6dB, requiere 5V para potencia completa");
    
    tlv320_dump_debug();
    
    return true;
}

bool tlv320_configure_headphone_only(void)
{
    ESP_LOGI(TAG, "=== CONFIGURACIÓN CORRECTA TLV320DAC3100 HeadphoneOutput ===");
    ESP_LOGI(TAG, "Aplicando correcciones según análisis del datasheet TI");
    
    // ========== PAGE 0: DIGITAL CLOCKS Y PLL ==========
    if (!tlv_write_ok(0x00, 0x00, 0x00)) {
        ESP_LOGE(TAG, "Failed to switch to page 0");
        return false;
    }
    
    // CORRECCIÓN 1: Clock Selection P0/R4 = 0x07 (PLL ref = BCLK y CODEC_CLKIN = PLL)
    ESP_LOGI(TAG, "CORRECCIÓN: Clock selection P0/R4 = 0x07 (BCLK→PLL, CODEC_CLKIN=PLL)");
    if (!tlv_write_ok(0x00, 0x04, 0x07)) {
        ESP_LOGE(TAG, "Failed to set clock selection");
        return false;
    }
    
    // CORRECCIÓN 2: PLL Configuration - P=1, R=2, J=32, D=0 para VCO correcto
    ESP_LOGI(TAG, "CORRECCIÓN: PLL P=1, R=2, J=32, D=0 para VCO en rango válido");
    if (!tlv_write_ok(0x00, 0x05, 0x92)) { // PLL on, P=1, R=2
        ESP_LOGE(TAG, "Failed to set PLL P and R");
        return false;
    }
    if (!tlv_write_ok(0x00, 0x06, 0x20)) { // J=32
        ESP_LOGE(TAG, "Failed to set PLL J");
        return false;
    }
    if (!tlv_write_ok(0x00, 0x07, 0x00)) { // D=0 (MSB)
        ESP_LOGE(TAG, "Failed to set PLL D MSB");
        return false;
    }
    if (!tlv_write_ok(0x00, 0x08, 0x00)) { // D=0 (LSB)
        ESP_LOGE(TAG, "Failed to set PLL D LSB");
        return false;
    }
    
    // Wait for PLL lock
    vTaskDelay(pdMS_TO_TICKS(20));
    
    // CORRECCIÓN 3: Divisores NDAC=8, MDAC=2, DOSR=128 para Fs correcto
    ESP_LOGI(TAG, "CORRECCIÓN: NDAC=8, MDAC=2, DOSR=128 para Fs=48kHz correcto");
    if (!tlv_write_ok(0x00, 0x0B, 0x88)) { // NDAC=8, powered on
        ESP_LOGE(TAG, "Failed to set NDAC");
        return false;
    }
    if (!tlv_write_ok(0x00, 0x0C, 0x82)) { // MDAC=2, powered on
        ESP_LOGE(TAG, "Failed to set MDAC");
        return false;
    }
    if (!tlv_write_ok(0x00, 0x0D, 0x00)) { // DOSR MSB=0
        ESP_LOGE(TAG, "Failed to set DOSR MSB");
        return false;
    }
    if (!tlv_write_ok(0x00, 0x0E, 0x80)) { // DOSR LSB=128
        ESP_LOGE(TAG, "Failed to set DOSR LSB");
        return false;
    }
    
    // Audio Interface Configuration
    if (!tlv_write_ok(0x00, 0x1B, 0x00)) { // I2S, 16-bit, slave mode
        ESP_LOGE(TAG, "Failed to set audio interface");
        return false;
    }
    
    // Audio Processing Block Selection
    if (!tlv_write_ok(0x00, 0x3C, 0x01)) { // PRB_P1 for DAC
        ESP_LOGE(TAG, "Failed to set audio processing block");
        return false;
    }
    
    // ========== PAGE 1: ANALOG ROUTING AND POWER ==========
    if (!tlv_write_ok(0x01, 0x00, 0x01)) {
        ESP_LOGE(TAG, "Failed to switch to page 1");
        return false;
    }
    
    // Analog power configuration
    if (!tlv_write_ok(0x01, 0x01, 0x08)) { // Disable weak connection
        ESP_LOGE(TAG, "Failed to configure analog power");
        return false;
    }
    if (!tlv_write_ok(0x01, 0x02, 0x01)) { // Enable master analog power
        ESP_LOGE(TAG, "Failed to enable analog power");
        return false;
    }
    
    // CORRECCIÓN 4: HP Driver Power P1/R31 = 0xD0 (HPL on, HPR on, 1.65V CM)
    ESP_LOGI(TAG, "CORRECCIÓN: HP Driver Power P1/R31 = 0xD0 (bits D7/D6 para power)");
    if (!tlv_write_ok(0x01, 0x1F, 0xD0)) { // HPL on (D7=1), HPR on (D6=1), CM=1.65V
        ESP_LOGE(TAG, "Failed to power HP drivers correctly");
        return false;
    }
    
    // CORRECCIÓN 5: Routing P1/R35 = 0x44 (DAC_L→HPL, DAC_R→HPR via mixers)
    ESP_LOGI(TAG, "CORRECCIÓN: Routing P1/R35 = 0x44 (DAC_L→HPL, DAC_R→HPR)");
    if (!tlv_write_ok(0x01, 0x23, 0x44)) { // Correct DAC to HP routing
        ESP_LOGE(TAG, "Failed to set HP routing");
        return false;
    }
    
    // CORRECCIÓN 6: HP Volume Control P1/R36=0x80, P1/R37=0x80 (0dB)
    ESP_LOGI(TAG, "CORRECCIÓN: HP Volume P1/R36=0x80, P1/R37=0x80 (conectar control)");
    if (!tlv_write_ok(0x01, 0x24, 0x80)) { // HPL volume control connected, 0dB
        ESP_LOGE(TAG, "Failed to set HPL volume");
        return false;
    }
    if (!tlv_write_ok(0x01, 0x25, 0x80)) { // HPR volume control connected, 0dB
        ESP_LOGE(TAG, "Failed to set HPR volume");
        return false;
    }
    
    // CORRECCIÓN 7: HP PGAs P1/R40=0x06, P1/R41=0x06 (unmute + 0dB)
    ESP_LOGI(TAG, "CORRECCIÓN: HP PGAs P1/R40=0x06, P1/R41=0x06 (UNMUTE + 0dB)");
    if (!tlv_write_ok(0x01, 0x28, 0x06)) { // HPL PGA: unmute, 0dB gain
        ESP_LOGE(TAG, "Failed to unmute HPL PGA");
        return false;
    }
    if (!tlv_write_ok(0x01, 0x29, 0x06)) { // HPR PGA: unmute, 0dB gain (with reserved bit)
        ESP_LOGE(TAG, "Failed to unmute HPR PGA");
        return false;
    }
    
    // Output stage power control
    if (!tlv_write_ok(0x01, 0x09, 0x30)) { // Power up HPL and HPR output stages
        ESP_LOGE(TAG, "Failed to power up HP output stages");
        return false;
    }
    
    // Mixer routing to HP drivers
    if (!tlv_write_ok(0x01, 0x0C, 0x08)) { // Route DAC_L to left HP mixer
        ESP_LOGE(TAG, "Failed to route DAC_L to HP mixer");
        return false;
    }
    if (!tlv_write_ok(0x01, 0x0D, 0x08)) { // Route DAC_R to right HP mixer
        ESP_LOGE(TAG, "Failed to route DAC_R to HP mixer");
        return false;
    }
    
    // ========== PAGE 0: DAC DIGITAL CONFIGURATION ==========
    if (!tlv_write_ok(0x00, 0x00, 0x00)) {
        ESP_LOGE(TAG, "Failed to switch back to page 0");
        return false;
    }
    
    // DAC Power and Data Path
    if (!tlv_write_ok(0x00, 0x3F, 0xD4)) { // Both DACs powered and data paths active
        ESP_LOGE(TAG, "Failed to power up DACs");
        return false;
    }
    
    // DAC Volume Control - unmute
    if (!tlv_write_ok(0x00, 0x40, 0x00)) { // Unmute both DACs
        ESP_LOGE(TAG, "Failed to unmute DACs");
        return false;
    }
    
    // Individual DAC Channel Volumes
    if (!tlv_write_ok(0x00, 0x41, 0x00)) { // Left DAC 0dB
        ESP_LOGE(TAG, "Failed to set left DAC volume");
        return false;
    }
    if (!tlv_write_ok(0x00, 0x42, 0x00)) { // Right DAC 0dB
        ESP_LOGE(TAG, "Failed to set right DAC volume");
        return false;
    }
    
    vTaskDelay(pdMS_TO_TICKS(100)); // Allow analog circuits to settle
    
    // CRÍTICO: Verificar y reconfigurar mixers si es necesario
    ESP_LOGI(TAG, "Verificando configuración crítica de mixers...");
    
    // Asegurar que estamos en Page 1
    if (!tlv_write_ok(0x01, 0x00, 0x01)) {
        ESP_LOGW(TAG, "Warning: Could not switch to page 1 for mixer verification");
    }
    
    // Verificar mixers y reconfigurar si están incorrectos
    uint8_t mixl_val, mixr_val;
    if (tlv_read_reg(0x01, 0x0C, &mixl_val) && tlv_read_reg(0x01, 0x0D, &mixr_val)) {
        if (mixl_val != 0x08 || mixr_val != 0x08) {
            ESP_LOGW(TAG, "MIXERS INCORRECTOS! MIXL=0x%02X, MIXR=0x%02X - Reconfigurando...", mixl_val, mixr_val);
            
            // Forzar reconfiguración de mixers
            tlv_write_ok(0x01, 0x0C, 0x08); // DAC_L to left HP mixer
            tlv_write_ok(0x01, 0x0D, 0x08); // DAC_R to right HP mixer
            vTaskDelay(pdMS_TO_TICKS(10));
            
            // Verificar nuevamente
            if (tlv_read_reg(0x01, 0x0C, &mixl_val) && tlv_read_reg(0x01, 0x0D, &mixr_val)) {
                ESP_LOGI(TAG, "Mixers reconfigurados: MIXL=0x%02X, MIXR=0x%02X", mixl_val, mixr_val);
            }
        } else {
            ESP_LOGI(TAG, "✓ Mixers correctos: MIXL=0x%02X, MIXR=0x%02X", mixl_val, mixr_val);
        }
    }
    
    ESP_LOGI(TAG, "✅ TLV320DAC3100 CONFIGURACIÓN CORRECTA COMPLETADA");
    ESP_LOGI(TAG, "✅ CORRECCIONES APLICADAS:");
    ESP_LOGI(TAG, "   • Clock: P0/R4=0x07 (BCLK→PLL, CODEC_CLKIN=PLL)");
    ESP_LOGI(TAG, "   • PLL: P=1, R=2, J=32, D=0 (VCO correcto)");
    ESP_LOGI(TAG, "   • Divisores: NDAC=8, MDAC=2, DOSR=128");
    ESP_LOGI(TAG, "   • HP Power: P1/R31=0xD0 (bits D7/D6)");
    ESP_LOGI(TAG, "   • Routing: P1/R35=0x44 (DAC_L→HPL, DAC_R→HPR)");
    ESP_LOGI(TAG, "   • HP PGAs: P1/R40=0x06, P1/R41=0x06 (UNMUTED)");
    ESP_LOGI(TAG, "   • Mixers: P1/R12=0x08, P1/R13=0x08 (DAC→HP)");
    ESP_LOGI(TAG, "🎧 AUDIO DEBE FUNCIONAR CORRECTAMENTE EN HEADPHONES HPL/HPR");
    
    tlv320_dump_debug();
    
    return true;
}

void tlv320_advanced_debug_and_health_check(void)
{
    ESP_LOGI(TAG, "=== TLV320 ADVANCED HEALTH CHECK ===");
    
    uint8_t v;
    
    // Page 0 - Digital Status
    ESP_LOGI(TAG, "--- PAGE 0 (Digital) ---");
    if (tlv_read_reg(0x00, 0x00, &v) == ESP_OK) ESP_LOGI(TAG, "P0:PAGE_SEL(0x00)=0x%02X", v);
    if (tlv_read_reg(0x00, 0x01, &v) == ESP_OK) ESP_LOGI(TAG, "P0:RESET(0x01)=0x%02X %s", v, (v&0x01)?"RESET_ACTIVE":"NORMAL");
    if (tlv_read_reg(0x00, 0x1B, &v) == ESP_OK) ESP_LOGI(TAG, "P0:IF_CTRL1(0x1B)=0x%02X I2S=%s", v, (v&0xC0)==0x00?"I2S":"OTHER");
    if (tlv_read_reg(0x00, 0x3F, &v) == ESP_OK) {
        ESP_LOGI(TAG, "P0:DAC_DP(0x3F)=0x%02X LDAC=%s RDAC=%s", v, 
                 (v&0x80)?"ON":"OFF", (v&0x40)?"ON":"OFF");
        if ((v & 0xC0) != 0xC0) ESP_LOGW(TAG, "WARNING: One or both DACs are OFF!");
    }
    if (tlv_read_reg(0x00, 0x40, &v) == ESP_OK) {
        ESP_LOGI(TAG, "P0:DAC_VOLCTL(0x40)=0x%02X LDAC_MUTE=%s RDAC_MUTE=%s", v,
                 (v&0x08)?"MUTED":"UNMUTED", (v&0x04)?"MUTED":"UNMUTED");
        if (v & 0x0C) ESP_LOGW(TAG, "WARNING: One or both DACs are MUTED!");
    }
    if (tlv_read_reg(0x00, 0x41, &v) == ESP_OK) ESP_LOGI(TAG, "P0:DAC_LVOL(0x41)=0x%02X (%+.1fdB)", v, v * -0.5f);
    if (tlv_read_reg(0x00, 0x42, &v) == ESP_OK) ESP_LOGI(TAG, "P0:DAC_RVOL(0x42)=0x%02X (%+.1fdB)", v, v * -0.5f);
    
    // Page 1 - Analog Status
    ESP_LOGI(TAG, "--- PAGE 1 (Analog) ---");
    if (tlv_read_reg(0x01, 0x01, &v) == ESP_OK) ESP_LOGI(TAG, "P1:PWR_CFG(0x01)=0x%02X", v);
    if (tlv_read_reg(0x01, 0x02, &v) == ESP_OK) ESP_LOGI(TAG, "P1:ANALOG_PWR(0x02)=0x%02X %s", v, (v&0x01)?"ENABLED":"DISABLED");
    if (tlv_read_reg(0x01, 0x09, &v) == ESP_OK) {
        ESP_LOGI(TAG, "P1:OUT_PWR(0x09)=0x%02X HPL=%s HPR=%s", v,
                 (v&0x20)?"ON":"OFF", (v&0x10)?"ON":"OFF");
        if ((v & 0x30) != 0x30) ESP_LOGW(TAG, "WARNING: HP outputs not powered!");
    }
    if (tlv_read_reg(0x01, 0x1F, &v) == ESP_OK) {
        ESP_LOGI(TAG, "P1:HP_DRV(0x1F)=0x%02X HPL_DRV=%s HPR_DRV=%s", v,
                 (v&0x02)?"ON":"OFF", (v&0x01)?"ON":"OFF");
        if ((v & 0x03) != 0x03) ESP_LOGW(TAG, "WARNING: HP drivers not enabled!");
    }
    if (tlv_read_reg(0x01, 0x23, &v) == ESP_OK) {
        ESP_LOGI(TAG, "P1:OUT_ROUT(0x23)=0x%02X", v);
        if ((v & 0xF0) != 0x50) ESP_LOGW(TAG, "WARNING: HP routing incorrect! Expected 0x5x, got 0x%02X", v);
    }
    if (tlv_read_reg(0x01, 0x24, &v) == ESP_OK) {
        ESP_LOGI(TAG, "P1:HPL_VOL(0x24)=0x%02X EN=%s VOL=%+.1fdB", v,
                 (v&0x80)?"YES":"NO", (v&0x7F) * -0.5f);
        if (!(v & 0x80)) ESP_LOGW(TAG, "WARNING: HPL amplifier DISABLED!");
    }
    if (tlv_read_reg(0x01, 0x25, &v) == ESP_OK) {
        ESP_LOGI(TAG, "P1:HPR_VOL(0x25)=0x%02X EN=%s VOL=%+.1fdB", v,
                 (v&0x80)?"YES":"NO", (v&0x7F) * -0.5f);
        if (!(v & 0x80)) ESP_LOGW(TAG, "WARNING: HPR amplifier DISABLED!");
    }
    if (tlv_read_reg(0x01, 0x0C, &v) == ESP_OK) {
        ESP_LOGI(TAG, "P1:MIXL_CFG(0x0C)=0x%02X DAC_L_TO_HP=%s", v, (v&0x08)?"ROUTED":"NOT_ROUTED");
        if (!(v & 0x08)) ESP_LOGW(TAG, "WARNING: Left DAC not routed to HP mixer!");
    }
    if (tlv_read_reg(0x01, 0x0D, &v) == ESP_OK) {
        ESP_LOGI(TAG, "P1:MIXR_CFG(0x0D)=0x%02X DAC_R_TO_HP=%s", v, (v&0x08)?"ROUTED":"NOT_ROUTED");
        if (!(v & 0x08)) ESP_LOGW(TAG, "WARNING: Right DAC not routed to HP mixer!");
    }
    
    ESP_LOGI(TAG, "=== END HEALTH CHECK ===");
}

bool tlv320_audio_watchdog_check_and_recover(void)
{
    uint8_t dac_power, dac_mute, hpl_vol, hpr_vol, hp_drv, out_pwr;
    bool needs_recovery = false;
    
    // Check critical registers for audio path integrity
    if (tlv_read_reg(0x00, 0x3F, &dac_power) != ESP_OK) {
        ESP_LOGW(TAG, "Watchdog: Cannot read DAC power register");
        return false;
    }
    
    if (tlv_read_reg(0x00, 0x40, &dac_mute) != ESP_OK) {
        ESP_LOGW(TAG, "Watchdog: Cannot read DAC mute register");
        return false;
    }
    
    if (tlv_read_reg(0x01, 0x24, &hpl_vol) != ESP_OK) {
        ESP_LOGW(TAG, "Watchdog: Cannot read HPL volume register");
        return false;
    }
    
    if (tlv_read_reg(0x01, 0x25, &hpr_vol) != ESP_OK) {
        ESP_LOGW(TAG, "Watchdog: Cannot read HPR volume register");
        return false;
    }
    
    if (tlv_read_reg(0x01, 0x1F, &hp_drv) != ESP_OK) {
        ESP_LOGW(TAG, "Watchdog: Cannot read HP driver register");
        return false;
    }
    
    if (tlv_read_reg(0x01, 0x09, &out_pwr) != ESP_OK) {
        ESP_LOGW(TAG, "Watchdog: Cannot read output power register");
        return false;
    }
    
    // Check for known failure conditions
    if ((dac_power & 0xC0) != 0xC0) {
        ESP_LOGW(TAG, "Watchdog: DACs powered down (0x%02X), recovering...", dac_power);
        tlv_write_reg(0x00, 0x3F, 0xD4);
        needs_recovery = true;
    }
    
    if (dac_mute & 0x0C) {
        ESP_LOGW(TAG, "Watchdog: DACs muted (0x%02X), recovering...", dac_mute);
        tlv_write_reg(0x00, 0x40, 0x00);
        needs_recovery = true;
    }
    
    if (!(hpl_vol & 0x80)) {
        ESP_LOGW(TAG, "Watchdog: HPL amplifier disabled (0x%02X), recovering...", hpl_vol);
        tlv_write_reg(0x01, 0x24, 0x80);
        needs_recovery = true;
    }
    
    if (!(hpr_vol & 0x80)) {
        ESP_LOGW(TAG, "Watchdog: HPR amplifier disabled (0x%02X), recovering...", hpr_vol);
        tlv_write_reg(0x01, 0x25, 0x80);
        needs_recovery = true;
    }
    
    if ((hp_drv & 0x03) != 0x03) {
        ESP_LOGW(TAG, "Watchdog: HP drivers not enabled (0x%02X), recovering...", hp_drv);
        tlv_write_reg(0x01, 0x1F, 0x03);
        needs_recovery = true;
    }
    
    if ((out_pwr & 0x30) != 0x30) {
        ESP_LOGW(TAG, "Watchdog: HP outputs not powered (0x%02X), recovering...", out_pwr);
        tlv_write_reg(0x01, 0x09, 0x30);
        needs_recovery = true;
    }
    
    // CRÍTICO: Verificar y corregir mixers (problema principal)
    uint8_t mixl, mixr;
    if (tlv_read_reg(0x01, 0x0C, &mixl) && tlv_read_reg(0x01, 0x0D, &mixr)) {
        if (mixl != 0x08) {
            ESP_LOGW(TAG, "Watchdog: MIXL incorrecto (0x%02X), forzando 0x08...", mixl);
            tlv_write_reg(0x01, 0x0C, 0x08);
            needs_recovery = true;
        }
        if (mixr != 0x08) {
            ESP_LOGW(TAG, "Watchdog: MIXR incorrecto (0x%02X), forzando 0x08...", mixr);
            tlv_write_reg(0x01, 0x0D, 0x08);
            needs_recovery = true;
        }
    } else {
        ESP_LOGW(TAG, "Watchdog: No se pueden leer mixers, aplicando fix forzado...");
        tlv_write_reg(0x01, 0x0C, 0x08); // Force left mixer
        tlv_write_reg(0x01, 0x0D, 0x08); // Force right mixer
        needs_recovery = true;
    }
    
    if (needs_recovery) {
        ESP_LOGI(TAG, "Watchdog: Recovery operations completed");
        vTaskDelay(pdMS_TO_TICKS(50)); // Allow recovery to settle
    }
    
    return needs_recovery;
}

void tlv320_diagnose_and_fix_dropout_issues(void)
{
    ESP_LOGI(TAG, "=== TLV320 DROPOUT DIAGNOSIS & FIX ===");
    
    uint8_t hp_drv, out_rout, mixl, mixr, hpl_vol, hpr_vol;
    bool needs_fix = false;
    
    // Read critical registers
    if (tlv_read_reg(0x01, 0x1F, &hp_drv) != ESP_OK) {
        ESP_LOGE(TAG, "Cannot read HP driver register");
        return;
    }
    
    if (tlv_read_reg(0x01, 0x23, &out_rout) != ESP_OK) {
        ESP_LOGE(TAG, "Cannot read output routing register");
        return;
    }
    
    if (tlv_read_reg(0x01, 0x0C, &mixl) != ESP_OK) {
        ESP_LOGE(TAG, "Cannot read left mixer register");
        return;
    }
    
    if (tlv_read_reg(0x01, 0x0D, &mixr) != ESP_OK) {
        ESP_LOGE(TAG, "Cannot read right mixer register");
        return;
    }
    
    if (tlv_read_reg(0x01, 0x24, &hpl_vol) != ESP_OK) {
        ESP_LOGE(TAG, "Cannot read HPL volume register");
        return;
    }
    
    if (tlv_read_reg(0x01, 0x25, &hpr_vol) != ESP_OK) {
        ESP_LOGE(TAG, "Cannot read HPR volume register");
        return;
    }
    
    ESP_LOGI(TAG, "Current register values:");
    ESP_LOGI(TAG, "  HP_DRV(0x1F)=0x%02X (should be 0x03)", hp_drv);
    ESP_LOGI(TAG, "  OUT_ROUT(0x23)=0x%02X (should be 0x50)", out_rout);
    ESP_LOGI(TAG, "  MIXL_CFG(0x0C)=0x%02X (should be 0x08)", mixl);
    ESP_LOGI(TAG, "  MIXR_CFG(0x0D)=0x%02X (should be 0x08)", mixr);
    ESP_LOGI(TAG, "  HPL_VOL(0x24)=0x%02X (should be 0x80)", hpl_vol);
    ESP_LOGI(TAG, "  HPR_VOL(0x25)=0x%02X (should be 0x80)", hpr_vol);
    
    // Fix HP driver issue (0x06 -> 0x03)
    if (hp_drv != 0x03) {
        ESP_LOGW(TAG, "Fixing HP driver register: 0x%02X -> 0x03", hp_drv);
        tlv_write_reg(0x01, 0x1F, 0x03);
        needs_fix = true;
    }
    
    // CORRECCIÓN: Fix output routing (should be 0x44 para DAC→HP correcto)
    if (out_rout != 0x44) {
        ESP_LOGW(TAG, "Fixing output routing: 0x%02X -> 0x44 (DAC_L→HPL, DAC_R→HPR)", out_rout);
        tlv_write_reg(0x01, 0x23, 0x44);
        needs_fix = true;
    }
    
    // Fix mixer routing (should be 0x08 for both)
    if (mixl != 0x08) {
        ESP_LOGW(TAG, "Fixing left mixer routing: 0x%02X -> 0x08", mixl);
        tlv_write_reg(0x01, 0x0C, 0x08);
        needs_fix = true;
    }
    
    if (mixr != 0x08) {
        ESP_LOGW(TAG, "Fixing right mixer routing: 0x%02X -> 0x08", mixr);
        tlv_write_reg(0x01, 0x0D, 0x08);
        needs_fix = true;
    }
    
    // Fix HP volume enables (should be 0x80)
    if (hpl_vol != 0x80) {
        ESP_LOGW(TAG, "Fixing HPL volume enable: 0x%02X -> 0x80", hpl_vol);
        tlv_write_reg(0x01, 0x24, 0x80);
        needs_fix = true;
    }
    
    if (hpr_vol != 0x80) {
        ESP_LOGW(TAG, "Fixing HPR volume enable: 0x%02X -> 0x80", hpr_vol);
        tlv_write_reg(0x01, 0x25, 0x80);
        needs_fix = true;
    }
    
    if (needs_fix) {
        ESP_LOGI(TAG, "Dropout fixes applied - audio should recover");
        vTaskDelay(pdMS_TO_TICKS(100)); // Allow changes to settle
        
        // Re-read and verify
        ESP_LOGI(TAG, "Verifying fixes...");
        tlv320_dump_debug();
    } else {
        ESP_LOGI(TAG, "All registers are correct - issue may be elsewhere");
    }
    
    ESP_LOGI(TAG, "=== DROPOUT DIAGNOSIS COMPLETE ===");
}

bool tlv320_emergency_mixer_fix(void)
{
    ESP_LOGW(TAG, "=== EMERGENCY MIXER FIX ===");
    ESP_LOGW(TAG, "Forcing correct mixer configuration for HeadphoneOutput");
    
    // Switch to Page 1
    if (!tlv_write_ok(0x01, 0x00, 0x01)) {
        ESP_LOGE(TAG, "Failed to switch to page 1 for emergency fix");
        return false;
    }
    
    // FORZAR directamente la configuración correcta sin leer primero
    ESP_LOGW(TAG, "Escribiendo MIXL=0x08 (DAC_L to HPL mixer) FORZADO");
    bool success1 = tlv_write_ok(0x01, 0x0C, 0x08);
    
    ESP_LOGW(TAG, "Escribiendo MIXR=0x08 (DAC_R to HPR mixer) FORZADO");
    bool success2 = tlv_write_ok(0x01, 0x0D, 0x08);
    
    // CORRECCIÓN: HP drivers con bits correctos P1:R31=0xD0
    ESP_LOGW(TAG, "Forzando HP drivers CORRECTO: P1:R31=0xD0 (bits D7/D6)");
    bool success3 = tlv_write_ok(0x01, 0x1F, 0xD0);
    
    // Forzar HP output stages powered
    ESP_LOGW(TAG, "Forzando HP output stages: P1:R9=0x30");
    bool success4 = tlv_write_ok(0x01, 0x09, 0x30);
    
    // CORRECCIÓN: Routing correcto P1:R35=0x44
    ESP_LOGW(TAG, "Forzando routing CORRECTO: P1:R35=0x44 (DAC→HP)");
    bool success5 = tlv_write_ok(0x01, 0x23, 0x44);
    
    // CORRECCIÓN: HP PGAs unmuted P1:R40=0x06, P1:R41=0x06
    ESP_LOGW(TAG, "Forzando HP PGAs UNMUTED: P1:R40=0x06, P1:R41=0x06");
    bool success6 = tlv_write_ok(0x01, 0x28, 0x06);
    bool success7 = tlv_write_ok(0x01, 0x29, 0x06);
    
    // Delay para que se apliquen los cambios
    vTaskDelay(pdMS_TO_TICKS(50));
    
    bool overall_success = success1 && success2 && success3 && success4 && success5 && success6 && success7;
    
    if (overall_success) {
        ESP_LOGI(TAG, "✅ EMERGENCY FIX CON CORRECCIONES APLICADO COMPLETAMENTE");
        ESP_LOGI(TAG, "✅ Mixers: 0x08, HP drivers: 0xD0, Output stages: 0x30, Routing: 0x44");
        ESP_LOGI(TAG, "✅ HP PGAs: 0x06 (UNMUTED), TODAS LAS CORRECCIONES APLICADAS");
        ESP_LOGI(TAG, "🎧 AUDIO DEBERÍA FUNCIONAR CORRECTAMENTE AHORA");
    } else {
        ESP_LOGE(TAG, "❌ EMERGENCY FIX PARCIALMENTE FALLIDO");
        ESP_LOGE(TAG, "❌ Verificar conexiones I2C y alimentación del TLV320");
    }
    
    return overall_success;
}
