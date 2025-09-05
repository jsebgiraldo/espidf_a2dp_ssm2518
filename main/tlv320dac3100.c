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

// --- Public wrappers for CLI ---
esp_err_t tlv320_reg_read(uint8_t page, uint8_t reg, uint8_t *val_out)
{
    return tlv_read_reg(page, reg, val_out);
}

esp_err_t tlv320_reg_write(uint8_t page, uint8_t reg, uint8_t val)
{
    return tlv_write_reg(page, reg, val);
}

void tlv320_dump_debug_public(void)
{
    tlv320_dump_debug();
}

// Ensure this symbol is emitted and never inlined away
void __attribute__((noinline)) tlv320_print_status(void)
{
    ESP_LOGI(TAG, "tlv320_print_status: collecting registers...");
    uint8_t r4=0, r5=0, r6=0, r7=0, r8=0, rb=0, rc=0, rd=0, re=0, r1b=0, r1c=0, r1d=0, r3c=0, r3f=0, r40=0, r41=0, r42=0;
    uint8_t p1_0c=0, p1_0d=0, p1_23=0, p1_1f=0, p1_24=0, p1_25=0, p1_28=0, p1_29=0, p1_21=0, p1_20=0, p1_26=0, p1_2a=0;
    tlv_read_reg(0x00, 0x04, &r4);
    tlv_read_reg(0x00, 0x05, &r5);
    tlv_read_reg(0x00, 0x06, &r6);
    tlv_read_reg(0x00, 0x07, &r7);
    tlv_read_reg(0x00, 0x08, &r8);
    tlv_read_reg(0x00, 0x0B, &rb);
    tlv_read_reg(0x00, 0x0C, &rc);
    tlv_read_reg(0x00, 0x0D, &rd);
    tlv_read_reg(0x00, 0x0E, &re);
    tlv_read_reg(0x00, 0x1B, &r1b);
    tlv_read_reg(0x00, 0x1C, &r1c);
    tlv_read_reg(0x00, 0x1D, &r1d);
    tlv_read_reg(0x00, 0x3C, &r3c);
    tlv_read_reg(0x00, 0x3F, &r3f);
    tlv_read_reg(0x00, 0x40, &r40);
    tlv_read_reg(0x00, 0x41, &r41);
    tlv_read_reg(0x00, 0x42, &r42);
    tlv_read_reg(0x01, 0x0C, &p1_0c);
    tlv_read_reg(0x01, 0x0D, &p1_0d);
    tlv_read_reg(0x01, 0x21, &p1_21);
    tlv_read_reg(0x01, 0x23, &p1_23);
    tlv_read_reg(0x01, 0x1F, &p1_1f);
    tlv_read_reg(0x01, 0x24, &p1_24);
    tlv_read_reg(0x01, 0x25, &p1_25);
    tlv_read_reg(0x01, 0x28, &p1_28);
    tlv_read_reg(0x01, 0x29, &p1_29);
    tlv_read_reg(0x01, 0x20, &p1_20);
    tlv_read_reg(0x01, 0x26, &p1_26);
    tlv_read_reg(0x01, 0x2A, &p1_2a);

    ESP_LOGI(TAG, "=== TLV STATUS ===");
    ESP_LOGI(TAG, "P0 R4=0x%02X R5=0x%02X R6=0x%02X R7=0x%02X R8=0x%02X", r4,r5,r6,r7,r8);
    ESP_LOGI(TAG, "P0 R11..R14 (NDAC/MDAC/DOSR)= 0x%02X 0x%02X 0x%02X 0x%02X", rb,rc,rd,re);
    ESP_LOGI(TAG, "P0 IF: 1B=0x%02X 1C=0x%02X 1D=0x%02X, PRB(3C)=0x%02X", r1b,r1c,r1d,r3c);
    ESP_LOGI(TAG, "P0 DAC: 3F=0x%02X 40=0x%02X 41=0x%02X 42=0x%02X", r3f,r40,r41,r42);
    ESP_LOGI(TAG, "P1 MIX: 0C=0x%02X 0D=0x%02X, OUT_ROUT(23)=0x%02X, BASE(21)=0x%02X", p1_0c,p1_0d,p1_23,p1_21);
    ESP_LOGI(TAG, "P1 HP: 1F=0x%02X, VOL 24=0x%02X 25=0x%02X, PGA 28=0x%02X 29=0x%02X", p1_1f,p1_24,p1_25,p1_28,p1_29);
    ESP_LOGI(TAG, "P1 CLSD: 20=0x%02X 26=0x%02X 2A=0x%02X", p1_20,p1_26,p1_2a);

    // Simple interpretation
    int fmt = (r1b >> 6) & 0x3; // 0=I2S
    int wl  = (r1b >> 4) & 0x3; // 0=16bit
    ESP_LOGI(TAG, "IF fmt=%s, wordlen=%s, BCLK/WCLK as %s", (fmt==0?"I2S":"other"), (wl==0?"16":"!=16"), (r1d==0x01?"inputs(slave)":"other"));
    if (fmt!=0 || wl!=0 || r1d!=0x01) {
        ESP_LOGW(TAG, "La interfaz no es I2S 16-bit slave — ajusta 0x1B..0x1D");
    }
    if (r4 != 0x07) {
        ESP_LOGW(TAG, "Clock sel (0x04) esperado=0x07 (PLL<-BCLK, CODEC<-PLL)");
    }
    if (rb != 0x82 || rc != 0x81 || re != 0x80) {
        ESP_LOGW(TAG, "Divisores NDAC/MDAC/DOSR no son (2/1/128) — revisa fs");
    }
    if (p1_0c != 0x08 || p1_0d != 0x08) {
        ESP_LOGW(TAG, "MIXERS HP no rutean DAC (P1:0C/0D deberian ser 0x08)");
    }
    if (p1_23 != 0x44) {
        ESP_LOGW(TAG, "OUT_ROUT (0x23) deberia ser 0x44 (DAC_L->HPL, DAC_R->HPR)");
    }
    if ((p1_1f & 0xC0) != 0xC0) {
        ESP_LOGW(TAG, "HP drivers no estan ON (P1:1F bits D7/D6)");
    }
    ESP_LOGI(TAG, "Esperado I2S16: BCLK/LRCLK = 32; verificalo con analizador");
}

bool tlv320_autofix_page0_for_i2s16(int sample_rate, bool verbose)
{
    if (!(sample_rate == 44100 || sample_rate == 48000)) {
        if (verbose) ESP_LOGE(TAG, "AutoFix: SR %d no soportado (44100/48000)", sample_rate);
        return false;
    }

    // 1) Muteo suave antes de tocar clocks
    (void)tlv_write_ok(0x00, 0x40, 0x60); // ejemplo: set soft-stepping + mute bits si aplica

    // 2) Configurar interfaz I2S 16-bit, slave
    tlv_write_ok(0x00, 0x1B, 0x00);
    tlv_write_ok(0x00, 0x1C, 0x00);
    tlv_write_ok(0x00, 0x1D, 0x01);

    // 3) Selección de reloj: PLL desde BCLK, CODEC_CLKIN=PLL
    tlv_write_ok(0x00, 0x04, 0x07);

    // 4) PLL: P=1, R=1, J=8, D=0 (para BCLK = Fs*32)
    tlv_write_ok(0x00, 0x05, 0x91);
    tlv_write_ok(0x00, 0x06, 0x08);
    tlv_write_ok(0x00, 0x07, 0x00);
    tlv_write_ok(0x00, 0x08, 0x00);
    vTaskDelay(pdMS_TO_TICKS(10));

    // 5) Divisores DAC: NDAC=2, MDAC=1, DOSR=128
    tlv_write_ok(0x00, 0x0B, 0x82);
    tlv_write_ok(0x00, 0x0C, 0x81);
    tlv_write_ok(0x00, 0x0D, 0x00);
    tlv_write_ok(0x00, 0x0E, 0x80);

    // 6) Processing block PRB_P1
    tlv_write_ok(0x00, 0x3C, 0x01);

    // 7) Data path ON y unmute DACs de forma segura
    tlv_write_ok(0x00, 0x3F, 0xD4);
    tlv_write_ok(0x00, 0x40, 0x00);

    if (verbose) {
        uint8_t v;
        tlv_read_reg(0x00, 0x04, &v); ESP_LOGI(TAG, "AF: P0/R4=0x%02X", v);
        tlv_read_reg(0x00, 0x05, &v); ESP_LOGI(TAG, "AF: P0/R5=0x%02X", v);
        tlv_read_reg(0x00, 0x06, &v); ESP_LOGI(TAG, "AF: P0/R6=0x%02X", v);
        tlv_read_reg(0x00, 0x0B, &v); ESP_LOGI(TAG, "AF: P0/R11=0x%02X", v);
        tlv_read_reg(0x00, 0x0C, &v); ESP_LOGI(TAG, "AF: P0/R12=0x%02X", v);
        tlv_read_reg(0x00, 0x0E, &v); ESP_LOGI(TAG, "AF: P0/R14=0x%02X", v);
    }
    ESP_LOGI(TAG, "AutoFix Page0 completado para I2S16 SR=%d", sample_rate);
    return true;
}

bool tlv320_configure_bclk_i2s_16(int sample_rate)
{
    // Configuración completa para TLV320DAC3100 usando BCLK como fuente PLL
    // Asume I2S 16-bit en slots de 16-bit (BCLK = Fs * 32). Si usas slots de 32-bit, BCLK=Fs*64 y el DAC oirá Fs' = BCLK/32 = 2*Fs.
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
    if (!tlv_write_ok(0x00, 0x04, 0x07)) return false; // BCLK como PLL ref

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
    
    ESP_LOGI(TAG, "TLV320 reset completo y reconfiguración exitosa");
    return true;
}

bool tlv320_configure_dual_output(void)
{
    ESP_LOGI(TAG, "TLV320: Dual = HP estéreo + SPK (mono desde mixer izq)");

    // --- PAGE 0: configurar interfaz, relojes y luego habilitar DAC ---
    // I2S format: 16-bit word length, I²S (Philips), codec is slave for BCLK/LRCLK
    tlv_write_ok(0x00, 0x1B, 0x00); // P0/R27: I²S + 16-bit
    tlv_write_ok(0x00, 0x1C, 0x00); // P0/R28: data offset = 0 (I²S adds the 1-bit delay)
    tlv_write_ok(0x00, 0x1D, 0x01); // P0/R29: BCLK/WCLK as inputs

    // Clock source: use PLL referenced from BCLK, and use PLL as CODEC_CLKIN
    tlv_write_ok(0x00, 0x04, 0x07); // P0/R4: PLL ref=BCLK, CODEC_CLKIN=PLL

    // PLL = 8 × BCLK  (P=1, R=1, J=8, D=0) para slots de 16-bit (BCLK = Fs×32)
    tlv_write_ok(0x00, 0x05, 0x91); // P0/R5: power PLL, P=1, R=1
    tlv_write_ok(0x00, 0x06, 0x08); // P0/R6: J=8
    tlv_write_ok(0x00, 0x07, 0x00); // P0/R7: D MSB
    tlv_write_ok(0x00, 0x08, 0x00); // P0/R8: D LSB
    vTaskDelay(pdMS_TO_TICKS(10));  // Esperar lock de PLL

    // Divisores DAC para Fs exacto (Fs = PLL / (NDAC*MDAC*DOSR))
    tlv_write_ok(0x00, 0x0B, 0x82); // NDAC=2, powered
    tlv_write_ok(0x00, 0x0C, 0x81); // MDAC=1, powered
    tlv_write_ok(0x00, 0x0D, 0x00); // DOSR MSB = 0
    tlv_write_ok(0x00, 0x0E, 0x80); // DOSR LSB = 128

    // Processing block
    tlv_write_ok(0x00, 0x3C, 0x01);  // PRB_P1

    // Volumen digital inicial conservador
    tlv_write_ok(0x00, 0x41, 0x10);  // -8 dB digital (arranque)
    tlv_write_ok(0x00, 0x42, 0x10);

    // Habilitar DACs y datapath después de clocks estables
    tlv_write_ok(0x00, 0x3F, 0xD4);  // LDAC/RDAC ON + data path
    tlv_write_ok(0x00, 0x40, 0x00);  // Unmute DACs

    // --- PAGE 1: potencia analógica y ruteos ---
    tlv_write_ok(0x01, 0x01, 0x08);          // Disable weak AVDD
    tlv_write_ok(0x01, 0x02, 0x01);          // Master analog power ON

    // 1) Ruteo de DAC a mezcladores HP (necesario para HPL/HPR)
    tlv_write_ok(0x01, 0x0C, 0x08);          // DAC_L -> mixer L (HP path)
    tlv_write_ok(0x01, 0x0D, 0x08);          // DAC_R -> mixer R (HP path)
    // 2) Ruteo base que alimenta los mixers analógicos (de aquí toma señal el Class-D)
    tlv_write_ok(0x01, 0x21, 0x44);          // LDAC→Left mixer, RDAC→Right mixer
    //    Si quieres SPK MONO (L+R en el mixer izq): usa 0x4C en vez de 0x44.

    // 3) Conectar mezcladores a salidas HP (¡esto te faltaba!)
    tlv_write_ok(0x01, 0x23, 0x44);          // DAC_L→HPL, DAC_R→HPR

    // 4) HP drivers, PGAs y power stage
    tlv_write_ok(0x01, 0x1F, 0xD0);          // HP drivers ON (CM apropiado)
    tlv_write_ok(0x01, 0x28, 0x06);          // HPL PGA 0 dB (unmute)
    tlv_write_ok(0x01, 0x29, 0x06);          // HPR PGA 0 dB (unmute)
    tlv_write_ok(0x01, 0x24, 0x80);          // HPL vol enable (0 dB)
    tlv_write_ok(0x01, 0x25, 0x80);          // HPR vol enable (0 dB)
    tlv_write_ok(0x01, 0x09, 0x30);          // Power stage HPL/HPR ON

    // 5) Class-D: enganchar al volumen analógico izq + encender
    tlv_write_ok(0x01, 0x26, 0x80);          // SPK_VOL: bit7=1 (conectado), 0 dB
    tlv_write_ok(0x01, 0x20, 0x00);          // Class-D OFF para limpiar fault
    vTaskDelay(pdMS_TO_TICKS(5));
    tlv_write_ok(0x01, 0x20, 0x80);          // Class-D ON

    //tlv_write_ok(0x01, 0x2A, 0x1C);          // Class-D unmute, ganancia mínima no-mute, 24dB
    //tlv_write_ok(0x01, 0x2A, 0b00011100);          // Class-D unmute, ganancia mínima no-mute, 24dB
    tlv_write_ok(0x01, 0x2A, 0b00000100);          // Class-D unmute, ganancia mínima no-mute , 6dB

    vTaskDelay(pdMS_TO_TICKS(200));
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
    if (tlv_read_reg(0x01, 0x0C, &mixl_val) == ESP_OK && tlv_read_reg(0x01, 0x0D, &mixr_val) == ESP_OK) {
        if (mixl_val != 0x08 || mixr_val != 0x08) {
            ESP_LOGW(TAG, "MIXERS INCORRECTOS! MIXL=0x%02X, MIXR=0x%02X - Reconfigurando...", mixl_val, mixr_val);
            
            // Forzar reconfiguración de mixers
            tlv_write_ok(0x01, 0x0C, 0x08); // DAC_L to left HP mixer
            tlv_write_ok(0x01, 0x0D, 0x08); // DAC_R to right HP mixer
            vTaskDelay(pdMS_TO_TICKS(10));
            
            // Verificar nuevamente
            if (tlv_read_reg(0x01, 0x0C, &mixl_val) == ESP_OK && tlv_read_reg(0x01, 0x0D, &mixr_val) == ESP_OK) {
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

// --- Default baseline: minimal, conservative setup for quick audio on HP ---
bool tlv320_configure_default(void)
{
    ESP_LOGI(TAG, "TLV320 DEFAULT: baseline I2S16 + HP path at 0 dB");

    // Page 0: set interface first (I2S 16-bit, slave BCLK/WCLK)
    tlv_write_ok(0x00, 0x1B, 0x00); // I2S, 16-bit
    tlv_write_ok(0x00, 0x1C, 0x00); // data offset = 0
    tlv_write_ok(0x00, 0x1D, 0x01); // BCLK/WCLK inputs

    // Clocks: CODEC_CLKIN=PLL, PLL ref=BCLK
    tlv_write_ok(0x00, 0x04, 0x07);
    // PLL: P=1, R=1, J=8, D=0 -> PLL = 8*BCLK when BCLK=Fs*32 (I2S16 slots)
    tlv_write_ok(0x00, 0x05, 0x91);
    tlv_write_ok(0x00, 0x06, 0x08);
    tlv_write_ok(0x00, 0x07, 0x00);
    tlv_write_ok(0x00, 0x08, 0x00);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Dividers: NDAC=2, MDAC=1, DOSR=128 -> Fs = PLL/(2*1*128) = BCLK/32
    tlv_write_ok(0x00, 0x0B, 0x82);
    tlv_write_ok(0x00, 0x0C, 0x81);
    tlv_write_ok(0x00, 0x0D, 0x00);
    tlv_write_ok(0x00, 0x0E, 0x80);

    // Processing block PRB_P1
    tlv_write_ok(0x00, 0x3C, 0x01);

    // Set digital volumes to 0 dB initially and enable datapath
    tlv_write_ok(0x00, 0x41, 0x00); // LDAC 0 dB
    tlv_write_ok(0x00, 0x42, 0x00); // RDAC 0 dB
    tlv_write_ok(0x00, 0x3F, 0xD4); // LDAC/RDAC on + datapath
    tlv_write_ok(0x00, 0x40, 0x00); // Unmute

    // Page 1: power analog and route DAC -> HP (HPL/HPR) at 0 dB
    tlv_write_ok(0x01, 0x01, 0x08); // Disable weak AVDD
    tlv_write_ok(0x01, 0x02, 0x01); // Master analog power on

    // Mixers: route DAC_L to left HP mixer, DAC_R to right HP mixer
    tlv_write_ok(0x01, 0x0C, 0x08);
    tlv_write_ok(0x01, 0x0D, 0x08);
    // Output routing: DAC_L->HPL, DAC_R->HPR
    tlv_write_ok(0x01, 0x23, 0x44);

    // Base routing feeding analog mixers (used by Class-D)
    tlv_write_ok(0x01, 0x21, 0x44); // LDAC->Left mixer, RDAC->Right mixer

    // HP drivers and PGAs
    tlv_write_ok(0x01, 0x1F, 0xD0); // HP drivers on, CM ~1.65V
    tlv_write_ok(0x01, 0x28, 0x06); // HPL PGA 0 dB
    tlv_write_ok(0x01, 0x29, 0x06); // HPR PGA 0 dB
    tlv_write_ok(0x01, 0x24, 0x80); // HPL vol connect, 0 dB
    tlv_write_ok(0x01, 0x25, 0x80); // HPR vol connect, 0 dB
    tlv_write_ok(0x01, 0x09, 0x30); // Power up HPL/HPR outputs

    // Minimal Class-D speaker enable (requires 5V SPKVDD on board)
    tlv_write_ok(0x01, 0x26, 0x80); // SPK_VOL connect, 0 dB
    tlv_write_ok(0x01, 0x20, 0x00); // Toggle off to clear faults
    vTaskDelay(pdMS_TO_TICKS(5));
    tlv_write_ok(0x01, 0x20, 0x80); // Class-D ON
    tlv_write_ok(0x01, 0x2A, 0x04); // Unmute, low gain (~6 dB)

    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_LOGI(TAG, "DEFAULT listo: audio por HPL/HPR y Class-D (si 5V) a 0 dB con I2S16");
    return true;
}



// 6dB
 // tlv wr 1 2a 5 
 // 12dB
// tlv wr 1 2a d 
// 18dB
// tlv wr 1 2a 15 
// 24dB
 // tlv wr 1 2a 1d 