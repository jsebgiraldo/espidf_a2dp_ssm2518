#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t connected;       // 0/1
    uint8_t audio_state;     // ESP_A2D_AUDIO_STATE_* value
    uint32_t sample_rate;    // Hz
    uint32_t rx_bytes;       // bytes received from A2DP (PCM)
    uint32_t tx_bytes;       // bytes written to I2S
    uint8_t have_last_bda;   // 0/1
    uint8_t last_bda[6];     // last connected device address
} a2dp_status_t;

// Fills the status snapshot.
void a2dp_get_status(a2dp_status_t *out);

#ifdef __cplusplus
}
#endif
