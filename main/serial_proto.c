#include "serial_proto.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "tlv320dac3100.h"
#include "a2dp_status.h"

#define SYNC_A 0xAA
#define SYNC_B 0x55

// Commands
enum {
    CMD_WRITE_REG  = 0x01,
    CMD_READ_REG   = 0x02,
    CMD_WRITE_BURST= 0x03, // optional
    CMD_READ_BURST = 0x04, // optional
    CMD_PING       = 0x05,
    CMD_TLV_DETECT = 0x06,
    CMD_A2DP_STATUS= 0x07,
    CMD_PRESET_APPLY = 0x08,
    CMD_PRESET_LIST  = 0x09,
    RSP_WRITE_ACK  = 0x81,
    RSP_READ_VAL   = 0x82,
    RSP_WRITE_BURST_ACK = 0x83,
    RSP_READ_BURST_VAL  = 0x84,
    RSP_PONG       = 0x85,
    RSP_TLV_DETECT = 0x86,
    RSP_A2DP_STATUS= 0x87,
    RSP_PRESET_ACK = 0x88,
    RSP_PRESET_LIST= 0x89,
    RSP_ERROR      = 0xE0,
};

static const char *s_tag = "SERPROT";
static TaskHandle_t s_task = NULL;
static uart_port_t s_uart = UART_NUM_0;

static uint8_t cks(uint8_t cmd, uint8_t len, const uint8_t *p) {
    uint32_t s = cmd + len;
    for (uint8_t i = 0; i < len; ++i) s += p[i];
    return (uint8_t)(- (int32_t)s);
}

static inline void send_bytes(const void *data, size_t len) {
    uart_write_bytes(s_uart, (const char*)data, (size_t)len);
}

static void send_frame(uint8_t cmd, const uint8_t *p, uint8_t len) {
    uint8_t hdr[4] = { SYNC_A, SYNC_B, cmd, len };
    uint8_t sum = cks(cmd, len, p);
    send_bytes(hdr, 4);
    if (len) send_bytes(p, len);
    send_bytes(&sum, 1);
}

static void send_error(uint8_t code, uint8_t last_cmd, const uint8_t *info, uint8_t info_len) {
    uint8_t tmp[3 + 16];
    if (info_len > 16) info_len = 16;
    tmp[0] = code; tmp[1] = last_cmd; tmp[2] = info_len;
    if (info_len) memcpy(&tmp[3], info, info_len);
    send_frame(RSP_ERROR, tmp, (uint8_t)(3 + info_len));
}

static void handle_cmd(uint8_t cmd, const uint8_t *pl, uint8_t len) {
    switch (cmd) {
    case CMD_WRITE_REG:
        if (len != 3) { send_error(3, cmd, NULL, 0); return; }
        {
            uint8_t page = pl[0], reg = pl[1], val = pl[2];
            bool ok = (tlv320_reg_write(page, reg, val) == ESP_OK);
            uint8_t rsp[3] = { page, reg, ok ? 1 : 0 };
            send_frame(RSP_WRITE_ACK, rsp, sizeof(rsp));
        }
        return;
    case CMD_READ_REG:
        if (len != 2) { send_error(3, cmd, NULL, 0); return; }
        {
            uint8_t page = pl[0], reg = pl[1], val = 0xFF;
            esp_err_t r = tlv320_reg_read(page, reg, &val);
            if (r == ESP_OK) {
                uint8_t rsp[3] = { page, reg, val };
                send_frame(RSP_READ_VAL, rsp, sizeof(rsp));
            } else {
                uint8_t info[2] = { page, reg };
                send_error(1, cmd, info, 2);
            }
        }
        return;
    case CMD_WRITE_BURST:
        // payload: [page, start_reg, count, values...]
        if (len < 3) { send_error(3, cmd, NULL, 0); return; }
        {
            uint8_t page = pl[0], start = pl[1], count = pl[2];
            if (len != (uint8_t)(3 + count) || count == 0) { send_error(3, cmd, NULL, 0); return; }
            bool ok = true;
            for (uint8_t i = 0; i < count; ++i) {
                if (tlv320_reg_write(page, (uint8_t)(start + i), pl[3 + i]) != ESP_OK) { ok = false; break; }
            }
            uint8_t rsp[3] = { page, start, ok ? count : 0 };
            send_frame(RSP_WRITE_BURST_ACK, rsp, sizeof(rsp));
        }
        return;
    case CMD_READ_BURST:
        // payload: [page, start_reg, count]
        if (len != 3) { send_error(3, cmd, NULL, 0); return; }
        {
            uint8_t page = pl[0], start = pl[1], count = pl[2];
            if (count == 0 || count > 32) { send_error(3, cmd, NULL, 0); return; }
            uint8_t buf[3 + 32];
            buf[0] = page; buf[1] = start; buf[2] = count;
            for (uint8_t i = 0; i < count; ++i) {
                uint8_t v = 0xFF;
                if (tlv320_reg_read(page, (uint8_t)(start + i), &v) != ESP_OK) {
                    send_error(1, cmd, &page, 1);
                    return;
                }
                buf[3 + i] = v;
            }
            send_frame(RSP_READ_BURST_VAL, buf, (uint8_t)(3 + count));
        }
        return;
    case CMD_PING: {
        // payload: optional [vmaj, vmin] from PC; respond with version of firmware and feature bitmap
        uint8_t ver[4];
        ver[0] = 1; // proto major
        ver[1] = 1; // proto minor
        ver[2] = 0x03; // features bit0=bursts, bit1=detect
        ver[3] = 0x00;
        send_frame(RSP_PONG, ver, sizeof(ver));
        return;
    }
    case CMD_PRESET_LIST: {
        // Return a compact list of available preset IDs
        // IDs: 0x01 Default, 0x02 HP-only, 0x03 Speaker-only, 0x04 Dual, 0x05 Clocks-only
        uint8_t ids[] = { 0x01, 0x02, 0x03, 0x04, 0x05 };
        uint8_t out[1 + sizeof(ids)];
        out[0] = (uint8_t)sizeof(ids);
        memcpy(&out[1], ids, sizeof(ids));
        send_frame(RSP_PRESET_LIST, out, (uint8_t)sizeof(out));
        return;
    }
    case CMD_PRESET_APPLY: {
        if (len < 1) { send_error(3, cmd, NULL, 0); return; }
        uint8_t id = pl[0];
        bool ok = false;
        switch (id) {
            case 0x01: ok = tlv320_configure_default(); break;
            case 0x02: ok = tlv320_configure_headphone_only(); break;
            case 0x03: ok = tlv320_configure_speaker_only(); break;
            case 0x04: ok = tlv320_configure_dual_output(); break;
            case 0x05: ok = tlv320_configure_bclk_i2s_16(48000); break; // clocks-only @48k
            default: ok = false; break;
        }
        uint8_t rsp[2] = { id, (uint8_t)(ok ? 1 : 0) };
        send_frame(RSP_PRESET_ACK, rsp, sizeof(rsp));
        return;
    }
    case CMD_TLV_DETECT: {
        uint8_t addr = 0;
        uint8_t rsp[2] = { 0, 0 };
        rsp[0] = tlv320_detect(&addr) ? 1 : 0;
        rsp[1] = addr;
        send_frame(RSP_TLV_DETECT, rsp, sizeof(rsp));
        return;
    }
    case CMD_A2DP_STATUS: {
        a2dp_status_t st; a2dp_get_status(&st);
        uint8_t pl[1+1+4+4+4+1+6];
        size_t o = 0;
        pl[o++] = st.connected;
        pl[o++] = st.audio_state;
        pl[o++] = (uint8_t)(st.sample_rate & 0xFF);
        pl[o++] = (uint8_t)((st.sample_rate >> 8) & 0xFF);
        pl[o++] = (uint8_t)((st.sample_rate >> 16) & 0xFF);
        pl[o++] = (uint8_t)((st.sample_rate >> 24) & 0xFF);
        pl[o++] = (uint8_t)(st.rx_bytes & 0xFF);
        pl[o++] = (uint8_t)((st.rx_bytes >> 8) & 0xFF);
        pl[o++] = (uint8_t)((st.rx_bytes >> 16) & 0xFF);
        pl[o++] = (uint8_t)((st.rx_bytes >> 24) & 0xFF);
        pl[o++] = (uint8_t)(st.tx_bytes & 0xFF);
        pl[o++] = (uint8_t)((st.tx_bytes >> 8) & 0xFF);
        pl[o++] = (uint8_t)((st.tx_bytes >> 16) & 0xFF);
        pl[o++] = (uint8_t)((st.tx_bytes >> 24) & 0xFF);
        pl[o++] = st.have_last_bda;
        memcpy(&pl[o], st.last_bda, 6); o += 6;
        send_frame(RSP_A2DP_STATUS, pl, (uint8_t)o);
        return;
    }
    default:
        send_error(2, cmd, NULL, 0);
        return;
    }
}

static void serial_task(void *arg) {
    uint8_t buf[1024];
    size_t n = 0;
    for (;;) {
        int r = uart_read_bytes(s_uart, buf + n, sizeof(buf) - n, pdMS_TO_TICKS(50));
        if (r <= 0) {
            // yield
            vTaskDelay(1);
            continue;
        }
        n += (size_t)r;
        size_t i = 0;
        while (i + 4 <= n) {
            if (!(buf[i] == SYNC_A && buf[i+1] == SYNC_B)) { i++; continue; }
            if (i + 4 > n) break;
            uint8_t cmd = buf[i+2];
            uint8_t len = buf[i+3];
            size_t total = 4u + (size_t)len + 1u;
            if (i + total > n) break;
            const uint8_t *pl = &buf[i+4];
            uint8_t sum = buf[i+4 + len];
            if (sum == cks(cmd, len, pl)) {
                handle_cmd(cmd, pl, len);
            } else {
                ESP_LOGW(s_tag, "Bad CKS: cmd=0x%02X len=%u", cmd, len);
                uint8_t info[2] = { cmd, len };
                send_error(4, cmd, info, 2);
            }
            i += total;
        }
        if (i > 0) {
            memmove(buf, buf + i, n - i);
            n -= i;
        }
        if (n == sizeof(buf)) n = 0; // safety
    }
}

bool serial_proto_start(uart_port_t uart_num) {
    if (s_task) return true;
    s_uart = uart_num;
    // Ensure driver is installed with sane defaults
    uart_config_t cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // Try to get existing config; if not installed, install
    // Install RX buffer only; TX can share
    if (uart_is_driver_installed(s_uart) == false) {
        if (uart_driver_install(s_uart, 2048, 0, 0, NULL, 0) != ESP_OK) return false;
        (void)uart_param_config(s_uart, &cfg);
        (void)uart_set_pin(s_uart, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    }
    if (xTaskCreatePinnedToCore(serial_task, "serial_proto", 3072, NULL, tskIDLE_PRIORITY+3, &s_task, 0) != pdPASS) {
        s_task = NULL;
        return false;
    }
    ESP_LOGI(s_tag, "Binary protocol started on UART%u", (unsigned)s_uart);
    return true;
}

bool serial_proto_stop(void) {
    if (!s_task) return true;
    TaskHandle_t t = s_task;
    s_task = NULL;
    vTaskDelete(t);
    ESP_LOGI(s_tag, "Binary protocol stopped");
    return true;
}

bool serial_proto_is_running(void) {
    return s_task != NULL;
}

void serial_proto_set_log_tag(const char *tag) {
    if (tag && *tag) s_tag = tag;
}
