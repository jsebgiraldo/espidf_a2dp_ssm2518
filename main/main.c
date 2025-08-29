// std
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
// FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/ringbuf.h"
// ESP-IDF
#include "esp_log.h"
#include "esp_err.h"
#include "esp_pm.h"  // Para PM locks y control de frecuencia
// sdkconfig (Kconfig macros)
#include "sdkconfig.h"
// NVS
#include "nvs_flash.h"
#include "nvs.h"
// I2C
#include "driver/i2c.h"
// GPIO for optional codec reset and line checks
#include "driver/gpio.h"
// I2S (STD driver v2)
#include "driver/i2s_std.h"
// Bluetooth A2DP Sink
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
// SSM2518 driver (simple)
#include "ssm2518.h"
// TLV320DAC3100 driver (scaffold)
#include "tlv320dac3100.h"

// Tamaños de buffers y agua para suavizar jitter
#ifndef PCM_RB_SIZE_BYTES
#define PCM_RB_SIZE_BYTES   (32 * 1024)  // 32 KB para mayor colchón
#endif
#define PCM_LOW_WATER_BYTES  (12 * 1024)  // 12 KB
#define PCM_HIGH_WATER_BYTES (28 * 1024)  // 28 KB

// --------- Configuración de pines ---------
// I2C
#define I2C_MASTER_NUM           I2C_NUM_0
#define I2C_MASTER_SDA_IO        21
#define I2C_MASTER_SCL_IO        22
#define I2C_MASTER_FREQ_HZ       100000
#define SSM2518_I2C_ADDR_7BIT    0x34
// TLV320 reset GPIO (set to a valid OUTPUT-capable GPIO if wired; -1 means not used)
#ifndef TLV320_RESET_IO
#define TLV320_RESET_IO          33
#endif

// Tag de logs
static const char *TAG = "MAIN";

// ========== I2S INIT ==========
// Configuración optimizada para usar BCLK como fuente de reloj del TLV320
#define I2S_BCLK_IO              26   // BCLK
#define I2S_LRCK_IO              25   // LRCLK / WS  
#define I2S_DOUT_IO              27   // SDATA hacia codec
#define I2S_MCLK_IO              -1   // MCLK deshabilitado - modo BCLK optimizado

// Inversiones: fijado a polaridad normal (BCLK y WS no invertidos)

// I2S handle
static i2s_chan_handle_t s_tx_chan = NULL;
static uint32_t s_current_sample_rate = 44100; // valor por defecto
static SemaphoreHandle_t s_i2s_mutex;
static RingbufHandle_t s_pcm_rb = NULL;
static TaskHandle_t s_i2s_writer_task = NULL;
static uint8_t s_silence[2048] = {0};
static volatile size_t s_rx_bytes = 0;
static volatile size_t s_tx_bytes = 0;
static TaskHandle_t s_audio_stats_task = NULL;
static bool s_tlv_active = false;
static bool s_tlv_configured = false;
static bool s_have_last_bda = false;
static esp_bd_addr_t s_last_bda = {0};

// PM lock para evitar cambios de frecuencia durante audio
static esp_pm_lock_handle_t s_pm_lock = NULL;

// (sin duplicado L/R por software)

// --------------- I2C -----------------
static void i2c_master_init(void)
{
    i2c_config_t conf;
    memset(&conf, 0, sizeof(conf));
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    // Deshabilitar pull-ups internos; usar pull-ups externos del breakout si están presentes
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = 0;
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));
}

static bool ssm2518_detected(void)
{
    // Enviar sólo dirección para comprobar ACK
    uint8_t addr8 = (SSM2518_I2C_ADDR_7BIT << 1) | I2C_MASTER_WRITE;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, addr8, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 100 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return (ret == ESP_OK);
}

static bool tlv320_present(uint8_t *addr_out)
{
    return tlv320_detect(addr_out);
}

static void tlv320_reset_if_configured(void)
{
    int gpio = TLV320_RESET_IO;
    if (gpio >= 0 && gpio < 64) {
        ESP_LOGI(TAG, "Configurando GPIO%d como reset del TLV320", gpio);
        uint64_t mask = (1ULL << (unsigned)gpio);
        gpio_config_t io;
        memset(&io, 0, sizeof(io));
        io.pin_bit_mask = mask;
        io.mode = GPIO_MODE_OUTPUT;
        io.pull_up_en = GPIO_PULLUP_DISABLE;
        io.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io.intr_type = GPIO_INTR_DISABLE;
        ESP_ERROR_CHECK(gpio_config(&io));
        
        // Secuencia de reset del TLV320: LOW -> HIGH
        ESP_LOGI(TAG, "Ejecutando reset del TLV320...");
        gpio_set_level(gpio, 0);  // Assert reset (LOW)
        vTaskDelay(pdMS_TO_TICKS(10)); // Hold reset for 10ms
        gpio_set_level(gpio, 1);  // Release reset (HIGH)
        vTaskDelay(pdMS_TO_TICKS(50)); // Wait for TLV320 to initialize after reset
        ESP_LOGI(TAG, "Reset del TLV320 completado - chip listo para configuración");
        s_tlv_configured = false; // Reset configuration flag after hardware reset
    } else {
        ESP_LOGW(TAG, "Pin de reset TLV320 no configurado (GPIO%d inválido)", gpio);
    }
}

static void i2c_debug_scan(void)
{
    // Quick bus-level check
    int sda_level = gpio_get_level(I2C_MASTER_SDA_IO);
    int scl_level = gpio_get_level(I2C_MASTER_SCL_IO);
    ESP_LOGI(TAG, "I2C lines: SDA(GPIO%d)=%d, SCL(GPIO%d)=%d", I2C_MASTER_SDA_IO, sda_level, I2C_MASTER_SCL_IO, scl_level);

    ESP_LOGI(TAG, "I2C scan start...");
    int found = 0;
    for (uint8_t addr = 0x03; addr <= 0x77; ++addr) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        if (!cmd) break;
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(200));
        i2c_cmd_link_delete(cmd);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, " - Found I2C device at 0x%02X", addr);
            found++;
        }
    }
    if (found == 0) {
        ESP_LOGW(TAG, "I2C scan found no devices. Check power, wiring, pull-ups, and reset line.");
    }
}

// --------------- I2S -----------------
static void i2s_init(uint32_t sample_rate)
{
    s_current_sample_rate = sample_rate;

    i2s_chan_config_t chan_cfg;
    memset(&chan_cfg, 0, sizeof(chan_cfg));
    chan_cfg.id = I2S_NUM_0;
    chan_cfg.role = I2S_ROLE_MASTER;
    // 1) DMA más profundo (≈ 24 KB en anillo DMA): 12 desc × 2048 B/desc
    chan_cfg.dma_desc_num = 12;     // antes 8
    chan_cfg.dma_frame_num = 256;   // antes 256 → 512 frames * 4 B/frame = 2048 B por desc (16b estéreo)
    chan_cfg.auto_clear = true;
    chan_cfg.intr_priority = 2;     // eleva prioridad de la ISR del I2S
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &s_tx_chan, NULL));

    // Philips, 16-bit de datos y 16-bit por slot → BCLK = 32×Fs
    i2s_std_slot_config_t slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO);
    
    // 🔧 CONFIGURACIÓN EXPLÍCITA DE SLOTS para evitar alineación I2S incorrecta
    slot_cfg.slot_bit_width = I2S_SLOT_BIT_WIDTH_16BIT; // 32×Fs
    slot_cfg.bit_shift      = true;      // Philips (1-bit delay), igual que antes
    // Nota: left_align, big_endian, bit_order_lsb no existen en ESP-IDF 5.4.1
    // Estos se configuran automáticamente en el modo Philips estándar
    
    ESP_LOGI(TAG, "🔧 I2S Slot config: 16-bit slots, bit_shift=true, Philips mode");
    
    i2s_std_config_t std_cfg;
    memset(&std_cfg, 0, sizeof(std_cfg));
    std_cfg.clk_cfg = (i2s_std_clk_config_t) I2S_STD_CLK_DEFAULT_CONFIG(sample_rate);
    std_cfg.slot_cfg = slot_cfg;
    
    // � MODO BCLK: MCLK deshabilitado, TLV320 usa BCLK→PLL optimizado
    std_cfg.gpio_cfg.mclk = I2S_GPIO_UNUSED;  // Sin MCLK - modo BCLK puro
    std_cfg.gpio_cfg.bclk = I2S_BCLK_IO;
    std_cfg.gpio_cfg.ws   = I2S_LRCK_IO;
    std_cfg.gpio_cfg.dout = I2S_DOUT_IO;
    std_cfg.gpio_cfg.din  = I2S_GPIO_UNUSED;
    std_cfg.gpio_cfg.invert_flags.mclk_inv = false;
    std_cfg.gpio_cfg.invert_flags.bclk_inv = false;
    std_cfg.gpio_cfg.invert_flags.ws_inv   = false;
    ESP_LOGI(TAG, "🔧 I2S Inversiones: BCLK=NORMAL, WS=NORMAL");
    
    // APLL se queda
    std_cfg.clk_cfg.clk_src = I2S_CLK_SRC_APLL;
    std_cfg.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;
    
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(s_tx_chan, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(s_tx_chan));

    // Ajuste de drive strength para mejorar integridad de señal en BCLK/WS/DOUT
    (void)gpio_set_drive_capability((gpio_num_t)I2S_BCLK_IO, GPIO_DRIVE_CAP_3);
    (void)gpio_set_drive_capability((gpio_num_t)I2S_LRCK_IO, GPIO_DRIVE_CAP_3);
    (void)gpio_set_drive_capability((gpio_num_t)I2S_DOUT_IO,  GPIO_DRIVE_CAP_3);

    // Ring buffer unificado (B1): 32 KB para mayor holgura en ráfagas A2DP
    if (s_pcm_rb == NULL) {
        s_pcm_rb = xRingbufferCreate(PCM_RB_SIZE_BYTES, RINGBUF_TYPE_BYTEBUF);
        ESP_LOGI(TAG, "📦 Ring buffer: %u KB (unificado, 32×Fs)", (unsigned)(PCM_RB_SIZE_BYTES/1024));
    }

    // Logging: 32×Fs
    ESP_LOGI(TAG, "🎵 I2S BCLK estable: SR=%lu, BCLK=%lu Hz, DMA=%lu×%lu",
             (unsigned long)sample_rate,
             (unsigned long)(sample_rate * 32),
             (unsigned long)chan_cfg.dma_desc_num,
             (unsigned long)chan_cfg.dma_frame_num);
    
    if (!s_i2s_mutex) {
        s_i2s_mutex = xSemaphoreCreateMutex();
    }

    // Nota: No se crea un segundo buffer; tamaño unificado arriba
}

static void audio_stats_task(void *arg)
{
    size_t last_rx = 0, last_tx = 0;
    
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        size_t rx = s_rx_bytes;
        size_t tx = s_tx_bytes;
        size_t drx = rx - last_rx;
        size_t dtx = tx - last_tx;
        last_rx = rx;
        last_tx = tx;
        
        ESP_LOGI(TAG, "Audio stats: RX %.1f kB/s, TX %.1f kB/s", drx / 1024.0f, dtx / 1024.0f);
        
        // TLV320 audio watchdog SOLO cuando detecta dropout de audio
        bool audio_dropped = (drx > 1000 && dtx < 1000); // Receiving but not transmitting
        
        if (s_tlv_active && s_tlv_configured && audio_dropped) {
            ESP_LOGW(TAG, "🔥 AUDIO DROPOUT DETECTED! RX=%.1f kB/s, TX=%.1f kB/s", drx / 1024.0f, dtx / 1024.0f);
            ESP_LOGW(TAG, "🔧 Ejecutando fix de emergencia para mixers...");
            
            // Ejecutar fix inmediato para problema de mixers
            if (tlv320_emergency_mixer_fix()) {
                ESP_LOGI(TAG, "✅ Fix de emergencia aplicado - audio debería funcionar");
            } else {
                ESP_LOGE(TAG, "❌ Fix de emergencia falló - problema de hardware");
            }
            
            ESP_LOGW(TAG, "Running TLV320 full health check...");
            tlv320_advanced_debug_and_health_check();
            ESP_LOGW(TAG, "Running dropout-specific diagnosis and fixes...");
            tlv320_diagnose_and_fix_dropout_issues();
            
            if (tlv320_audio_watchdog_check_and_recover()) {
                ESP_LOGI(TAG, "TLV320 watchdog performed recovery operations");
                
                // Aplicar emergency fix adicional para asegurar que los mixers estén correctos
                ESP_LOGW(TAG, "🚨 Aplicando emergency fix adicional post-watchdog...");
                if (tlv320_emergency_mixer_fix()) {
                    ESP_LOGI(TAG, "✅ Emergency fix post-watchdog exitoso");
                } else {
                    ESP_LOGE(TAG, "❌ Emergency fix post-watchdog falló");
                }
            }
        }
    }
}

static void i2s_set_sample_rate(uint32_t sample_rate)
{
    if (s_tx_chan == NULL) return;
    if (sample_rate == s_current_sample_rate) return;
    
    ESP_LOGI(TAG, "🔄 Cambiando sample rate I2S: %lu -> %lu", (unsigned long)s_current_sample_rate, (unsigned long)sample_rate);
    
    // Reconfigurar clock manteniendo APLL para BCLK estable
    i2s_std_clk_config_t clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate);
    clk_cfg.clk_src = I2S_CLK_SRC_APLL;
    clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256; // Interno solamente
    
    ESP_ERROR_CHECK(i2s_channel_disable(s_tx_chan));
    ESP_ERROR_CHECK(i2s_channel_reconfig_std_clock(s_tx_chan, &clk_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(s_tx_chan));
    
    s_current_sample_rate = sample_rate;
    ESP_LOGI(TAG, "📡 BCLK actualizado: %lu Hz (32×%lu)",
             (unsigned long)(sample_rate * 32), (unsigned long)sample_rate);
}

static size_t i2s_write_pcm(const uint8_t *data, size_t len, uint32_t timeout_ms)
{
    if (s_tx_chan == NULL) return 0;
    size_t bytes_written = 0;
    // Proteger contra concurrencia desde callback BT
    if (s_i2s_mutex && xSemaphoreTake(s_i2s_mutex, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
        (void)i2s_channel_write(s_tx_chan, data, len, &bytes_written, pdMS_TO_TICKS(timeout_ms));
        xSemaphoreGive(s_i2s_mutex);
    }
    return bytes_written;
}

static void i2s_writer_task(void *arg)
{
    // B2: Escritor DMA-alineado a bloques fijos de 2048 bytes
    const size_t BLK = 2048;          // antes 1024
    uint8_t blk_buf[2048];
    size_t fill = 0;
    const TickType_t recv_timeout = pdMS_TO_TICKS(10);
    static bool started = false;

    for (;;) {
        // Espera a tener ~16 KB de datos antes de arrancar para evitar underruns iniciales
        if (!started) {
            size_t free_now = xRingbufferGetCurFreeSize(s_pcm_rb);
            if ((PCM_RB_SIZE_BYTES - free_now) < (16*1024)) {
                vTaskDelay(pdMS_TO_TICKS(2));
                continue;
            }
            started = true;
        }
        size_t item_size = 0;
        uint8_t *item = (uint8_t *)xRingbufferReceive(s_pcm_rb, &item_size, recv_timeout);

        if (item && item_size) {
            size_t off = 0;
            while (off < item_size) {
                size_t copy_n = BLK - fill;
                if (copy_n > (item_size - off)) copy_n = item_size - off;
                memcpy(&blk_buf[fill], item + off, copy_n);
                fill += copy_n;
                off  += copy_n;

                if (fill == BLK) {
                    (void)i2s_write_pcm(blk_buf, BLK, 30);
                    s_tx_bytes += BLK;
                    fill = 0;
                }
            }
            vRingbufferReturnItem(s_pcm_rb, item);
        } else {
            // No hay más datos; si tenemos parcial, completar con ceros y enviar
            if (fill > 0) {
                memset(&blk_buf[fill], 0, BLK - fill);
                (void)i2s_write_pcm(blk_buf, BLK, 20);
                s_tx_bytes += BLK;
                fill = 0;
            } else {
                // Mantener DMA alimentado con silencio
                (void)i2s_write_pcm(s_silence, BLK, 20);
            }
            vTaskDelay(1);
        }
    }
}

// --------------- Auto-Pairing (GAP) y Auto-Reconnect ---------------
static void save_last_bda_nvs(const esp_bd_addr_t bda)
{
    nvs_handle h;
    if (nvs_open("a2dp", NVS_READWRITE, &h) == ESP_OK) {
        (void)nvs_set_blob(h, "last_bda", bda, sizeof(esp_bd_addr_t));
        (void)nvs_commit(h);
        nvs_close(h);
    }
}

static bool load_last_bda_nvs(esp_bd_addr_t bda_out)
{
    nvs_handle h;
    size_t len = sizeof(esp_bd_addr_t);
    if (nvs_open("a2dp", NVS_READONLY, &h) == ESP_OK) {
        esp_err_t err = nvs_get_blob(h, "last_bda", bda_out, &len);
        nvs_close(h);
        return (err == ESP_OK && len == sizeof(esp_bd_addr_t));
    }
    return false;
}

static void gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_BT_GAP_PIN_REQ_EVT: {
        ESP_LOGI(TAG, "GAP PIN_REQ: respondiendo con PIN 0000");
        esp_bt_pin_code_t pin_code;
        pin_code[0] = '0'; pin_code[1] = '0'; pin_code[2] = '0'; pin_code[3] = '0';
        esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
        break;
    }
    case ESP_BT_GAP_CFM_REQ_EVT:
        ESP_LOGI(TAG, "GAP CFM_REQ: aceptando SSP Just Works, num_val=%lu", (unsigned long)param->cfm_req.num_val);
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;
    case ESP_BT_GAP_AUTH_CMPL_EVT:
        ESP_LOGI(TAG, "GAP AUTH_CMPL: status=%d, name=%s", param->auth_cmpl.stat, (const char *)param->auth_cmpl.device_name);
        break;
    default:
        break;
    }
}

// --------------- Bluetooth A2DP Sink -----------------
static void bt_app_a2d_data_cb(const uint8_t *data, uint32_t len)
{
    // 'data' es PCM 16-bit LE estéreo decodificado por la pila A2DP
    if (len == 0 || data == NULL || !s_pcm_rb) return;

    // 2) Jitter buffer real: objetivo de ocupación y poda suave
    const size_t RB_TOTAL   = PCM_RB_SIZE_BYTES;
    size_t free_now         = xRingbufferGetCurFreeSize(s_pcm_rb);
    size_t fill_now         = RB_TOTAL - free_now;
    const size_t LOW_WATER  = PCM_LOW_WATER_BYTES;
    const size_t HIGH_WATER = PCM_HIGH_WATER_BYTES;

    // Si vamos muy llenos, podar sólo lo justo para mantenernos ≤ HIGH_WATER
    if (fill_now > HIGH_WATER) {
        size_t need_free = (fill_now + len) - HIGH_WATER;
        while (need_free > 0) {
            size_t old_sz = 0;
            size_t take = (need_free < 1024) ? need_free : 1024; // poda en rebanadas de 1 KB
            uint8_t *old = (uint8_t*) xRingbufferReceiveUpTo(s_pcm_rb, &old_sz, 0, take);
            if (!old || old_sz == 0) break;
            vRingbufferReturnItem(s_pcm_rb, old);
            need_free = (old_sz >= need_free) ? 0 : (need_free - old_sz);
        }
    }

    // Intenta NO perder el bloque: permite 2 ms de espera
    if (xRingbufferSend(s_pcm_rb, (void*)data, len, pdMS_TO_TICKS(2)) == pdFALSE) {
        // Último recurso: descarta 1 KB y reintenta
        size_t old_sz = 0;
        uint8_t *old = (uint8_t*) xRingbufferReceiveUpTo(s_pcm_rb, &old_sz, 0, 1024);
        if (old) vRingbufferReturnItem(s_pcm_rb, old);
        (void)xRingbufferSend(s_pcm_rb, (void*)data, len, 0);
    }
    s_rx_bytes += len;
}

static void bt_app_a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param)
{
    switch (event) {
    case ESP_A2D_CONNECTION_STATE_EVT:
        ESP_LOGI(TAG, "A2DP estado conexión: %d", param->conn_stat.state);
        if (param->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
            // Guardar BDA del último dispositivo conectado
#ifdef CONFIG_IDF_TARGET
            // Use field name that's available in current IDF; try remote_bda first
#endif
            const uint8_t *bda = (const uint8_t *)param->conn_stat.remote_bda;
            if (bda) {
                memcpy(s_last_bda, bda, sizeof(esp_bd_addr_t));
                s_have_last_bda = true;
                save_last_bda_nvs(s_last_bda);
                ESP_LOGI(TAG, "A2DP conectado a %02X:%02X:%02X:%02X:%02X:%02X (guardado)",
                         bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
            }
        }
        break;
    case ESP_A2D_AUDIO_STATE_EVT:
        ESP_LOGI(TAG, "A2DP estado audio: %d", param->audio_stat.state);
        
        // 🎵 Configurar TLV320 cuando comience la reproducción de audio
        if (param->audio_stat.state == ESP_A2D_AUDIO_STATE_STARTED && s_tlv_active && !s_tlv_configured) {
            ESP_LOGI(TAG, "Audio iniciado - configurando TLV320 EXCLUSIVAMENTE para HeadphoneOutput");
            if (tlv320_configure_bclk_i2s_16(44100)) {            // ← 32×Fs
                if (tlv320_configure_headphone_only()) {
                    s_tlv_configured = true;
                    ESP_LOGI(TAG, "✓ TLV320 configurado EXCLUSIVAMENTE para HPL/HPR");
                    ESP_LOGI(TAG, "✓ Routing: DAC_L -> HPL, DAC_R -> HPR");
                    ESP_LOGI(TAG, "✓ Line outputs DESHABILITADOS");
                    ESP_LOGI(TAG, "🔧 Aplicando fix preventivo de mixers...");
                    tlv320_emergency_mixer_fix();
                } else {
                    ESP_LOGE(TAG, "✗ FALLO en configuración HeadphoneOutput");
                }
            } else {
                ESP_LOGE(TAG, "✗ FALLO en configuración de clocks I2S (BCLK=32×Fs)");
            }
        }
        
        // 🔒 PM Lock: Fijar frecuencia CPU durante audio
        if (param->audio_stat.state == ESP_A2D_AUDIO_STATE_STARTED) {
            // Reducir verbosidad global durante reproducción; mantener este TAG en INFO
            esp_log_level_set("*", ESP_LOG_WARN);
            esp_log_level_set(TAG, ESP_LOG_INFO);
            if (s_pm_lock && esp_pm_lock_acquire(s_pm_lock) == ESP_OK) {
                ESP_LOGI(TAG, "🔒 CPU frequency locked para audio estable");
            }
        } else if (param->audio_stat.state == ESP_A2D_AUDIO_STATE_STOPPED) {
            // Restaurar verbosidad razonable al detenerse el audio
            esp_log_level_set("*", ESP_LOG_INFO);
            esp_log_level_set(TAG, ESP_LOG_INFO);
            if (s_pm_lock) {
                esp_pm_lock_release(s_pm_lock);
                ESP_LOGI(TAG, "🔓 CPU frequency unlocked");
            }
        }
        break;
    case ESP_A2D_AUDIO_CFG_EVT:
        ESP_LOGI(TAG, "A2DP audio cfg (codec=%d)", param->audio_cfg.mcc.type);
        if (param->audio_cfg.mcc.type == ESP_A2D_MCT_SBC) {
            int sample_rate = 44100;
            uint8_t oct0 = param->audio_cfg.mcc.cie.sbc[0];
            if (oct0 & (1 << 6)) sample_rate = 32000;
            else if (oct0 & (1 << 5)) sample_rate = 44100;
            else if (oct0 & (1 << 4)) sample_rate = 48000;

            ESP_LOGI(TAG, "A2DP SBC sample_rate=%d", sample_rate);
            i2s_set_sample_rate(sample_rate);

            if (s_tlv_active) {
                if (s_tlv_configured) {
                    ESP_LOGI(TAG, "Actualizando sample rate del TLV320: %d", sample_rate);
                    if (!tlv320_configure_bclk_i2s_16(sample_rate)) {
                        ESP_LOGW(TAG, "Failed to update TLV320 sample rate");
                    }
                } else {
                    ESP_LOGI(TAG, "Configurando TLV320 para sample rate: %d", sample_rate);
                    if (!tlv320_configure_bclk_i2s_16(sample_rate)) {
                        ESP_LOGW(TAG, "Failed to configure TLV320 clocks for new sample rate");
                    } else {
                        ESP_LOGI(TAG, "Reconfigurando TLV320 EXCLUSIVAMENTE para HeadphoneOutput");
                        if (tlv320_configure_headphone_only()) {
                            s_tlv_configured = true;
                            ESP_LOGI(TAG, "✓ HeadphoneOutput reconfigurado exitosamente");
                            ESP_LOGI(TAG, "🔧 Aplicando fix preventivo de mixers post-reconfiguración...");
                            tlv320_emergency_mixer_fix();
                        } else {
                            ESP_LOGE(TAG, "✗ FALLO en reconfiguración HeadphoneOutput");
                            s_tlv_configured = false;
                        }
                    }
                }
            }
        }
        break;
    default:
        break;
    }
}

static void bluetooth_a2dp_sink_init(void)
{
    // Verificar configuración del controlador en sdkconfig
#if CONFIG_BTDM_CTRL_MODE_BLE_ONLY
    ESP_LOGE(TAG, "El controlador BT está configurado como BLE_ONLY en sdkconfig; habilita BR/EDR o BTDM para usar A2DP Classic BT");
    return;
#endif

    // Liberar memoria BLE solo cuando no estamos en modo BTDM (dual-mode)
#if !CONFIG_BTDM_CTRL_MODE_BTDM
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));
#endif

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    // Inicializar/habilitar el controlador de forma segura según su estado actual
    esp_bt_controller_status_t st = esp_bt_controller_get_status();
    ESP_LOGI(TAG, "BT controller status: %d", st);
    if (st == ESP_BT_CONTROLLER_STATUS_IDLE) {
        ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
        st = esp_bt_controller_get_status();
    }

    // Seleccionar modo de habilitación según sdkconfig
#if CONFIG_BTDM_CTRL_MODE_BR_EDR_ONLY
    const esp_bt_mode_t bt_mode = ESP_BT_MODE_CLASSIC_BT;
#elif CONFIG_BTDM_CTRL_MODE_BTDM
    const esp_bt_mode_t bt_mode = ESP_BT_MODE_BTDM;
#else
    const esp_bt_mode_t bt_mode = ESP_BT_MODE_CLASSIC_BT; // valor por defecto
#endif

    if (st != ESP_BT_CONTROLLER_STATUS_ENABLED) {
        esp_err_t err = esp_bt_controller_enable(bt_mode);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_bt_controller_enable(modo=%d) falló: %s", (int)bt_mode, esp_err_to_name(err));
            ESP_ERROR_CHECK(err);
        }
    }

    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    // GAP: auto-pair SSP (Just Works) y PIN fallback
    ESP_ERROR_CHECK(esp_bt_gap_register_callback(gap_cb));
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_NONE; // Just Works
    (void)esp_bt_gap_set_security_param(ESP_BT_SP_IOCAP_MODE, &iocap, sizeof(iocap));
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    ESP_ERROR_CHECK(esp_bt_gap_set_pin(pin_type, 0, NULL));

    // A2DP Sink
    ESP_ERROR_CHECK(esp_a2d_register_callback(&bt_app_a2d_cb));
    ESP_ERROR_CHECK(esp_a2d_sink_register_data_callback(&bt_app_a2d_data_cb));
    ESP_ERROR_CHECK(esp_a2d_sink_init());

    const char *dev_name = "ESP32 A2DP SEBAS MAX";
    ESP_ERROR_CHECK(esp_bt_gap_set_device_name(dev_name));
    ESP_ERROR_CHECK(esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE));

    // Intentar reconectar automáticamente al último dispositivo emparejado
    if (!s_have_last_bda) {
        s_have_last_bda = load_last_bda_nvs(s_last_bda);
    }
    if (s_have_last_bda) {
        ESP_LOGI(TAG, "Intentando reconectar a %02X:%02X:%02X:%02X:%02X:%02X...",
                 s_last_bda[0], s_last_bda[1], s_last_bda[2], s_last_bda[3], s_last_bda[4], s_last_bda[5]);
        esp_a2d_sink_connect(s_last_bda);
    }
}

void app_main(void)
{
    // Inicializar NVS antes de usar WiFi/BT o PHY
    esp_err_t nvs_ret = nvs_flash_init();
    if (nvs_ret == ESP_ERR_NVS_NO_FREE_PAGES || nvs_ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_ret);

    ESP_LOGI(TAG, "Inicializando I2C...");
    i2c_master_init();
    vTaskDelay(pdMS_TO_TICKS(50));

    // Opcional: reset del TLV320 si hay pin de reset cableado
    tlv320_reset_if_configured();

    // Escaneo de bus para diagnóstico
    i2c_debug_scan();

    // Intentar detectar TLV320 primero (si está conectado)
    uint8_t tlv_addr = 0;
    if (tlv320_present(&tlv_addr)) {
        tlv320_set_address(tlv_addr);
        ESP_LOGI(TAG, "TLV320DAC3100 detectado en 0x%02X - configuración después de I2S", tlv_addr);
        s_tlv_active = true;
        // NO configurar aquí - esperar hasta que I2S genere BCLK
    } else {
        // Si no hay TLV, intentar SSM2518
        bool i2c_ok = ssm2518_detected();
        if (i2c_ok) {
            ESP_LOGI(TAG, "SSM2518 detectado por I2C (0x%02X). Modo programable.", SSM2518_I2C_ADDR_7BIT);
            ssm2518_set_address(SSM2518_I2C_ADDR_7BIT);
            ssm2518_configure();
        } else {
            ESP_LOGW(TAG, "No se detectó TLV320 ni SSM2518 por I2C. Continuando: SSM2518 Stand Alone / sólo I2S.");
        }
    }

    // Inicializar I2S (salida) antes del Bluetooth
    ESP_LOGI(TAG, "Inicializando I2S...");
    i2s_init(44100);

    // Con BCLK estable activo, configurar TLV completamente
    if (s_tlv_active) {
        vTaskDelay(pdMS_TO_TICKS(50)); // Dar tiempo al I2S para generar BCLK estable
        
        ESP_LOGI(TAG, "🚀 Ejecutando configuración BCLK OPTIMIZADA del TLV320...");
        if (!tlv320_hardware_reset_and_init(44100)) {
            ESP_LOGE(TAG, "❌ Failed to reset and configure TLV320");
        } else {
            ESP_LOGI(TAG, "✅ TLV320 configurado exitosamente en modo BCLK optimizado");
            ESP_LOGI(TAG, "🎯 Clock path: BCLK→PLL→NDAC/8→MDAC/2→DOSR/128");
        }
    }

    // Lanzar tarea escritora I2S en Core 1 con prioridad ALTA
    if (s_i2s_writer_task == NULL) {
        // B3: Prefill ring buffer ~16 KB (8×2048B) para estabilidad inicial
        if (s_pcm_rb) {
            for (int i = 0; i < 8; ++i) {
                (void)xRingbufferSend(s_pcm_rb, s_silence, sizeof(s_silence), 0);
            }
        }
        BaseType_t ok = xTaskCreatePinnedToCore(
            i2s_writer_task,
            "i2s_writer",
            3072,
            NULL,
            configMAX_PRIORITIES - 1, // Prioridad alta recomendada para escritura de audio
            &s_i2s_writer_task,
            1 /* CPU1 - dedicado para audio */);
        if (ok != pdPASS) {
            ESP_LOGE(TAG, "No se pudo crear i2s_writer_task");
        } else {
            ESP_LOGI(TAG, "🎵 I2S writer task: Core 1, prioridad máxima-1");
        }
        
        // Tarea de estadísticas en Core 0 con prioridad baja
        if (s_audio_stats_task == NULL) {
            xTaskCreatePinnedToCore(audio_stats_task, "audio_stats", 2048, NULL, tskIDLE_PRIORITY+1, &s_audio_stats_task, 0);
        }
    }

    // 🔒 Inicializar PM lock para estabilidad de frecuencia durante audio
    if (s_pm_lock == NULL) {
        esp_err_t ret = esp_pm_lock_create(ESP_PM_CPU_FREQ_MAX, 0, "audio", &s_pm_lock);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "🔒 PM lock creado para frecuencia CPU estable durante audio");
        } else {
            ESP_LOGW(TAG, "⚠️  No se pudo crear PM lock: %s", esp_err_to_name(ret));
        }
    }

    // Inicializar Bluetooth y A2DP Sink
    ESP_LOGI(TAG, "Inicializando Bluetooth A2DP Sink...");
    bluetooth_a2dp_sink_init();

    // Bucle mínimo
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
