// std
#include <stdio.h>
// FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/ringbuf.h"
// ESP-IDF
#include "esp_log.h"
#include "esp_err.h"
// sdkconfig (Kconfig macros)
#include "sdkconfig.h"
// NVS
#include "nvs_flash.h"
// I2C
#include "driver/i2c.h"
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

// --------- Configuración de pines ---------
// I2C
#define I2C_MASTER_NUM           I2C_NUM_0
#define I2C_MASTER_SDA_IO        21
#define I2C_MASTER_SCL_IO        22
#define I2C_MASTER_FREQ_HZ       100000
#define SSM2518_I2C_ADDR_7BIT    0x34

// I2S salida hacia SSM2518 (I2S Philips, 16-bit, estéreo)
// Ajusta estos pines según tu cableado hacia el PMOD AMP3
#define I2S_BCLK_IO              26   // BCLK
#define I2S_LRCK_IO              25   // LRCLK / WS
#define I2S_DOUT_IO              27   // SDATA hacia SSM2518
#define I2S_MCLK_IO              I2S_GPIO_UNUSED // SSM2518 puede operar sin MCLK usando PLL interna

static const char *TAG = "MAIN";

// I2S handle
static i2s_chan_handle_t s_tx_chan = NULL;
static uint32_t s_current_sample_rate = 44100; // valor por defecto
static SemaphoreHandle_t s_i2s_mutex;
static RingbufHandle_t s_pcm_rb = NULL;
static TaskHandle_t s_i2s_writer_task = NULL;
static uint8_t s_silence[1024] = {0};
static volatile size_t s_rx_bytes = 0;
static volatile size_t s_tx_bytes = 0;
static TaskHandle_t s_audio_stats_task = NULL;

// --------------- I2C -----------------
static void i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        .clk_flags = 0,
    };
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

// --------------- I2S -----------------
static void i2s_init(uint32_t sample_rate)
{
    s_current_sample_rate = sample_rate;

    i2s_chan_config_t chan_cfg = {
        .id = I2S_NUM_0,
        .role = I2S_ROLE_MASTER,
    // Balance entre latencia y robustez: varios desc y frames moderados
    .dma_desc_num = 8,
    .dma_frame_num = 256,
        .auto_clear = true,
        .intr_priority = 0,
    };
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &s_tx_chan, NULL));

    i2s_std_config_t std_cfg = {
    .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_16BIT,
            I2S_SLOT_MODE_STEREO
        ),
        .gpio_cfg = {
            .mclk = I2S_MCLK_IO,
            .bclk = I2S_BCLK_IO,
            .ws = I2S_LRCK_IO,
            .dout = I2S_DOUT_IO,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            }
        }
    };
    // Preferir APLL como fuente de reloj para reducir jitter
    std_cfg.clk_cfg.clk_src = I2S_CLK_SRC_APLL;
    std_cfg.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(s_tx_chan, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(s_tx_chan));

    if (!s_i2s_mutex) {
        s_i2s_mutex = xSemaphoreCreateMutex();
    }

    // Crear ring buffer para desacoplar callback BT de escritura I2S
    if (s_pcm_rb == NULL) {
        // 12 KB de buffer (~60ms a 48kHz 16-bit estéreo)
        s_pcm_rb = xRingbufferCreate(12 * 1024, RINGBUF_TYPE_BYTEBUF);
    }
}

static void i2s_set_sample_rate(uint32_t sample_rate)
{
    if (s_tx_chan == NULL) return;
    if (sample_rate == s_current_sample_rate) return;
    ESP_LOGI(TAG, "Cambiando sample rate I2S: %lu -> %lu", (unsigned long)s_current_sample_rate, (unsigned long)sample_rate);
    // Reconfigurar clock a nueva tasa
    i2s_std_clk_config_t clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate);
    // Mantener APLL como fuente y MCLK x256 para mayor estabilidad
    clk_cfg.clk_src = I2S_CLK_SRC_APLL;
    clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;
    ESP_ERROR_CHECK(i2s_channel_disable(s_tx_chan));
    ESP_ERROR_CHECK(i2s_channel_reconfig_std_clock(s_tx_chan, &clk_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(s_tx_chan));
    s_current_sample_rate = sample_rate;
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
    const TickType_t recv_timeout = pdMS_TO_TICKS(20);
    for (;;) {
        size_t item_size = 0;
        uint8_t *chunk = (uint8_t *)xRingbufferReceive(s_pcm_rb, &item_size, recv_timeout);
        if (chunk && item_size) {
            // Escribir el bloque recibido a I2S
            (void)i2s_write_pcm(chunk, item_size, 50);
            s_tx_bytes += item_size;
            vRingbufferReturnItem(s_pcm_rb, chunk);
        } else {
            // Si no hay datos, alimentar silencio para evitar underrun audible
            (void)i2s_write_pcm(s_silence, sizeof(s_silence), 20);
        }
        // Ceder CPU para evitar disparar WDT
        vTaskDelay(1);
    }
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
    }
}

// --------------- Bluetooth A2DP Sink -----------------
static void bt_app_a2d_data_cb(const uint8_t *data, uint32_t len)
{
    // 'data' es PCM 16-bit LE estéreo decodificado por la pila A2DP
    if (len == 0 || data == NULL) return;
    if (s_pcm_rb) {
        // Encolar datos; si está lleno, descartar el bloque más viejo para evitar lag
        while (xRingbufferSend(s_pcm_rb, (void *)data, len, 0) == pdFALSE) {
            // Descartar datos antiguos hasta poder encolar
            size_t old_sz = 0;
            uint8_t *old = (uint8_t *)xRingbufferReceiveUpTo(s_pcm_rb, &old_sz, 0, len);
            if (!old) break;
            vRingbufferReturnItem(s_pcm_rb, old);
        }
    s_rx_bytes += len;
    }
}

static void bt_app_a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param)
{
    switch (event) {
    case ESP_A2D_CONNECTION_STATE_EVT:
        ESP_LOGI(TAG, "A2DP estado conexión: %d", param->conn_stat.state);
        break;
    case ESP_A2D_AUDIO_STATE_EVT:
        ESP_LOGI(TAG, "A2DP estado audio: %d", param->audio_stat.state);
        break;
    case ESP_A2D_AUDIO_CFG_EVT:
        ESP_LOGI(TAG, "A2DP audio cfg (codec=%d)", param->audio_cfg.mcc.type);
        // Ajustar sample rate según SBC CIE
        if (param->audio_cfg.mcc.type == ESP_A2D_MCT_SBC) {
            int sample_rate = 44100; // default
            uint8_t oct0 = param->audio_cfg.mcc.cie.sbc[0];
            if (oct0 & (1 << 6)) sample_rate = 32000;
            else if (oct0 & (1 << 5)) sample_rate = 44100;
            else if (oct0 & (1 << 4)) sample_rate = 48000;
            ESP_LOGI(TAG, "A2DP SBC sample_rate=%d", sample_rate);
            i2s_set_sample_rate(sample_rate);
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

    ESP_ERROR_CHECK(esp_a2d_register_callback(&bt_app_a2d_cb));
    ESP_ERROR_CHECK(esp_a2d_sink_register_data_callback(&bt_app_a2d_data_cb));

    ESP_ERROR_CHECK(esp_a2d_sink_init());

    const char *dev_name = "ESP32 A2DP SSM2518";
    ESP_ERROR_CHECK(esp_bt_gap_set_device_name(dev_name));
    ESP_ERROR_CHECK(esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE));
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

    bool i2c_ok = ssm2518_detected();
    if (i2c_ok) {
        ESP_LOGI(TAG, "SSM2518 detectado por I2C (0x%02X). Modo programable.", SSM2518_I2C_ADDR_7BIT);
    ssm2518_set_address(SSM2518_I2C_ADDR_7BIT);
        ssm2518_configure();
    } else {
        ESP_LOGW(TAG, "SSM2518 no detectado por I2C. Modo Stand Alone.");
    }

    // Inicializar I2S (salida) antes del Bluetooth
    ESP_LOGI(TAG, "Inicializando I2S...");
    i2s_init(44100);

    // Lanzar tarea escritora I2S en Core 1 con prioridad alta
    if (s_i2s_writer_task == NULL) {
        // Prefill ring buffer con silencio mínimo para reducir latencia inicial
        if (s_pcm_rb) {
            for (int i = 0; i < 2; ++i) {
                (void)xRingbufferSend(s_pcm_rb, s_silence, sizeof(s_silence), 0);
            }
        }
        BaseType_t ok = xTaskCreatePinnedToCore(
            i2s_writer_task,
            "i2s_writer",
            3072,
            NULL,
            configMAX_PRIORITIES - 3,
            &s_i2s_writer_task,
            1 /* CPU1 */);
        if (ok != pdPASS) {
            ESP_LOGE(TAG, "No se pudo crear i2s_writer_task");
        }
        // Tarea de estadísticas (opcional)
        if (s_audio_stats_task == NULL) {
            xTaskCreatePinnedToCore(audio_stats_task, "audio_stats", 2048, NULL, tskIDLE_PRIORITY+1, &s_audio_stats_task, 0);
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
