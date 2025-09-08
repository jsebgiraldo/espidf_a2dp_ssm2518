# Protocolo Serial PC ↔ ESP32 para TLV320DAC3100 (v1.1)

Este documento especifica el protocolo binario sobre UART entre la app Qt (PC) y el firmware ESP32, cuyo objetivo es exponer operaciones de lectura/escritura de registros del TLV320DAC3100, además de permitir extensiones futuras (burst, ping, etc.).

La aplicación Qt ya implementa el lado PC conforme a esta especificación mínima: WRITE_REG y READ_REG. El ESP32 implementa esos dos comandos y añade mejoras opcionales: ráfagas (burst), PING/PONG y TLV_DETECT para descubrimiento.

## 1. Capa física (UART)

- Velocidad por defecto: 115200 bps (configurable)
- Formato: 8 data bits, sin paridad, 1 stop bit (8N1)
- Control de flujo: ninguno
- Tiempo de respuesta esperado: ≤ 30 ms por comando (tolerante a ±100 ms). En ráfagas de 32 bytes típicas, ≤ 10 ms.
- Tamaño de buffer recomendado: ≥ 1024 bytes RX
    - Firmware usa 2048 bytes en ESP32 para robustez.

## 2. Estructura de trama

Cada mensaje (comando o respuesta) es una trama binaria, sin escape/byte-stuffing. Se resíncroniza buscando cabecera.

```
[SYNC_A, SYNC_B, CMD, LEN, PAYLOAD[0..LEN-1], CKS]
```

- `SYNC_A = 0xAA`, `SYNC_B = 0x55`
- `CMD`: código de comando/respuesta (1 byte)
- `LEN`: longitud del payload (0..255)
- `PAYLOAD`: bytes del comando (puede ser 0)
- `CKS`: checksum de 8 bits (ver sección 3)

No se usa escape/byte-stuffing. La recuperación ante errores se hace por resincronización buscando la cabecera 0xAA 0x55. Los frames inválidos (checksum incorrecto) generan opcionalmente un `ERROR` con código BADCKS.

Límite práctico de `LEN`: 0..250 para dejar margen a cabecera y simplificar buffers (recomendado). Para los comandos mínimos (read/write) `LEN` ≤ 3. Para ráfagas se sugiere `count` ≤ 32 por frame.

## 3. Checksum (8 bits)

Checksum simple con suma en 8 bits de `CMD + LEN + sum(PAYLOAD)` y complemento a dos para forzar suma total ≡ 0 (mod 256).

- Definición: `CKS = (uint8_t)(- (int32_t)(CMD + LEN + ΣPAYLOAD))`
- Propiedad: `(CMD + LEN + ΣPAYLOAD + CKS) & 0xFF == 0`

Ejemplo C:

```c
static uint8_t proto_checksum(uint8_t cmd, uint8_t len, const uint8_t *payload) {
    uint32_t s = cmd + len;
    for (uint8_t i = 0; i < len; ++i) s += payload[i];
    return (uint8_t)(- (int32_t)s);
}
```

## 4. Comandos (requeridos y opcionales)

Códigos PC→ESP32 y ESP32→PC. Todos los campos multi-byte se envían en orden natural de bytes (aquí todo es byte a byte, sin enteros multi-byte en los comandos mínimos).

- `0x01 WRITE_REG` (PC→ESP32)
  - Payload: `[page (1), reg (1), value (1)]`
  - Acción ESP32: `tlv320_reg_write(page, reg, value)`
  - Respuesta: `0x81 WRITE_ACK` con payload `[page, reg, ok]` donde `ok=0x01` si éxito, `0x00` si fallo.

- `0x02 READ_REG` (PC→ESP32)
  - Payload: `[page (1), reg (1)]`
  - Acción ESP32: `tlv320_reg_read(page, reg, &value)`
  - Respuesta: `0x82 READ_VAL` con payload `[page, reg, value]` si éxito.
  - En caso de error de I2C, ver “Errores” más abajo: puede enviarse un frame `ERROR (0xE0)` opcional.

Opcionales (recomendados):
- `0x03 WRITE_BURST` → `0x83`
    - Payload: `[page, start_reg, count, values[0..count-1]]`
    - Acción: escritura secuencial de `count` bytes empezando en `start_reg`.
    - Respuesta: `[page, start_reg, written_count]` (si falla alguna escritura, `written_count=0`).
- `0x04 READ_BURST` → `0x84`
    - Payload: `[page, start_reg, count]`
    - Respuesta: `[page, start_reg, count, values...]` (hasta 32 bytes típicamente).
- `0x05 PING` → `0x85 PONG`
    - Payload PC: vacío o `[vmaj, vmin]`.
    - Respuesta: `[proto_maj, proto_min, feature_lo, feature_hi]`.
        - `feature_lo` bit0=BURST, bit1=DETECT.
- `0x06 TLV_DETECT` → `0x86`
    - Payload: vacío. Respuesta: `[present(0/1), i2c_addr]`.

- `0x07 A2DP_STATUS` → `0x87`
    - Payload: vacío
    - Respuesta: `[connected(1), audio_state(1), sample_rate(4 LE), rx_bytes(4 LE), tx_bytes(4 LE), have_last_bda(1), last_bda(6)]`
        - `audio_state` sigue los valores de `esp_a2d_audio_state_t` (p.ej., 0=STOPPED, 1=STARTED, 2=SUSPENDED).

Notas:
- Siempre responder a cada comando válido con su respuesta (o con `ERROR` si aplica).
- No se incluye ID de transacción; el protocolo es secuencial simple.

## 5. Errores y resincronización

Para mantener el lado PC simple, los errores de parser pueden ignorarse silenciosamente (drop). Para reportar fallos de operación (p. ej., I2C), se define un frame recomendado:

- `0xE0 ERROR` (ESP32→PC) — Opcional pero recomendado
  - Payload: `[code (1), last_cmd (1), info_len (1), info[0..info_len-1]]`
    - `code`: 1=I2C_FAIL, 2=BADCMD, 3=BADLEN, 4=BADCKS, 5=INTERNAL
  - `last_cmd`: último `CMD` recibido que causó el error (p.ej., 0x02)
  - `info`: datos adicionales opcionales (p.ej., `page, reg`)

Resincronización RX y control de buffers:
- Buscar `0xAA 0x55`; si checksum no coincide, descartar el primer byte y volver a buscar cabecera.
- Limpiar buffer si crece demasiado (p. ej., > 1 KiB) para evitar desbordes ante ruido. El firmware resetea el parseo si el buffer alcanza 1024 bytes sin extraer frames válidos.

## 6. Ejemplos de tramas (HEX)

WRITE_REG page=0x01, reg=0x23, val=0x44
- Comando: `AA 55 01 03 01 23 44 94`  (CKS=0x94)
- Respuesta OK: `AA 55 81 03 01 23 01 57`

READ_REG page=0x00, reg=0x1B
- Comando: `AA 55 02 02 00 1B E1`
- Respuesta (valor=0x00): `AA 55 82 03 00 1B 00 60`

Ejemplo ERROR (I2C_FAIL al leer P0/R0x1B):
Ejemplo A2DP_STATUS (conectado, audio STARTED, 44100 Hz):
- Comando: `AA 55 07 00 F4`
- Respuesta (valores ficticios): `AA 55 87 12 01 01 44 AC 00 00 10 27 00 00 20 4E 00 00 01 11 22 33 44 55 7F`
    - connected=1, audio_state=1, sample_rate=44100 (0x0000AC44), rx=0x00002710, tx=0x00004E20, have_last_bda=1, bda=11:22:33:44:55:7F
- `AA 55 E0 05 01 02 02 00 1B FB`
  - code=0x01 (I2C_FAIL), last_cmd=0x02, info_len=0x02, info=[0x00, 0x1B]

## 7. Esqueleto de implementación (ESP-IDF)

A continuación un esqueleto auto-contenido. Completar con tu inicialización UART (pines y puerto usado) y enlazar con las funciones del TLV ya existentes en tu proyecto.

```c
#include <string.h>
#include "driver/uart.h"
#include "esp_log.h"
#include "tlv320dac3100.h"  // tus wrappers tlv320_reg_read/write

#define UARTX            UART_NUM_0
#define TAG              "SERIAL"
#define SYNC_A           0xAA
#define SYNC_B           0x55

// Comandos
enum {
    CMD_WRITE_REG = 0x01,
    CMD_READ_REG  = 0x02,
    RSP_WRITE_ACK = 0x81,
    RSP_READ_VAL  = 0x82,
    RSP_ERROR     = 0xE0,
};

static uint8_t cks(uint8_t cmd, uint8_t len, const uint8_t *p) {
    uint32_t s = cmd + len;
    for (uint8_t i = 0; i < len; ++i) s += p[i];
    return (uint8_t)(- (int32_t)s);
}

static void send_frame(uint8_t cmd, const uint8_t *p, uint8_t len) {
    uint8_t hdr[4] = { SYNC_A, SYNC_B, cmd, len };
    uint8_t sum = cks(cmd, len, p);
    uart_write_bytes(UARTX, (const char*)hdr, 4);
    if (len) uart_write_bytes(UARTX, (const char*)p, len);
    uart_write_bytes(UARTX, (const char*)&sum, 1);
}

static void send_write_ack(uint8_t page, uint8_t reg, bool ok) {
    uint8_t pl[3] = { page, reg, ok ? 1 : 0 };
    send_frame(RSP_WRITE_ACK, pl, sizeof(pl));
}

static void send_read_val(uint8_t page, uint8_t reg, uint8_t val) {
    uint8_t pl[3] = { page, reg, val };
    send_frame(RSP_READ_VAL, pl, sizeof(pl));
}

static void send_error(uint8_t code, uint8_t last_cmd, const uint8_t *info, uint8_t info_len) {
    uint8_t tmp[3 + 8]; // info máx 8 por simplicidad
    if (info_len > 8) info_len = 8;
    tmp[0] = code; tmp[1] = last_cmd; tmp[2] = info_len;
    memcpy(&tmp[3], info, info_len);
    send_frame(RSP_ERROR, tmp, 3 + info_len);
}

static void handle_cmd(uint8_t cmd, const uint8_t *pl, uint8_t len) {
    if (cmd == CMD_WRITE_REG && len == 3) {
        uint8_t page = pl[0], reg = pl[1], val = pl[2];
        bool ok = (tlv320_reg_write(page, reg, val) == ESP_OK);
        send_write_ack(page, reg, ok);
        return;
    }
    if (cmd == CMD_READ_REG && len == 2) {
        uint8_t page = pl[0], reg = pl[1], val = 0xFF;
        esp_err_t r = tlv320_reg_read(page, reg, &val);
        if (r == ESP_OK) send_read_val(page, reg, val);
        else { uint8_t info[2] = {page, reg}; send_error(1, cmd, info, 2); }
        return;
    }
    // Comando no soportado o longitud inválida
    send_error((len < 2 || len > 3) ? 3 : 2, cmd, NULL, 0);
}

void serial_task(void *arg) {
    // Configurar UART (ejemplo). Ajusta pines/baud según tu placa.
    uart_config_t cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    uart_param_config(UARTX, &cfg);
    uart_set_pin(UARTX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UARTX, 2048, 0, 0, NULL, 0);

    uint8_t buf[1024];
    size_t n = 0;

    while (1) {
        int r = uart_read_bytes(UARTX, buf + n, sizeof(buf) - n, pdMS_TO_TICKS(50));
        if (r <= 0) continue;
        n += r;

        // Procesar buffer: buscar cabecera y extraer frames
        size_t i = 0;
        while (i + 4 <= n) {
            // Buscar SYNC
            if (!(buf[i] == SYNC_A && buf[i+1] == SYNC_B)) { i++; continue; }
            if (i + 4 > n) break; // espera cmd/len
            uint8_t cmd = buf[i+2];
            uint8_t len = buf[i+3];
            size_t total = 2 + 1 + 1 + len + 1; // AA 55 cmd len payload cks
            if (i + total > n) break; // frame incompleto
            const uint8_t *pl = &buf[i+4];
            uint8_t c = buf[i+4+len];
            if (c == cks(cmd, len, pl)) {
                handle_cmd(cmd, pl, len);
            } else {
                ESP_LOGW(TAG, "Bad checksum: cmd=0x%02X len=%u", cmd, len);
                // opcional: send_error(4, cmd, NULL, 0);
            }
            i += total; // avanzar al siguiente posible frame
        }
        // Compactar buffer
        if (i > 0) {
            memmove(buf, buf + i, n - i);
            n -= i;
        }
        if (n == sizeof(buf)) {
            // Buffer lleno sin encontrar frame completo: reset
            n = 0;
        }
    }
}
```

## 8. Integración con tu driver TLV

Reutiliza las funciones que ya tienes:
- `esp_err_t tlv320_reg_read(uint8_t page, uint8_t reg, uint8_t *val_out);`
- `esp_err_t tlv320_reg_write(uint8_t page, uint8_t reg, uint8_t val);`

El esqueleto anterior ya las invoca, y mapea sus resultados en `WRITE_ACK` y `READ_VAL` o en un `ERROR` opcional.

## 9. Extensiones (reservado para futuro)

Para mantener compatibilidad, define pero no implementes todavía (si no lo necesitas) los siguientes IDs:
- `0x03 WRITE_BURST` → `0x83` (p.ej.: `[page, start_reg, count, values...]`)
- `0x04 READ_BURST`  → `0x84` (p.ej.: `[page, start_reg, count]` → valores)
- `0x05 PING`        → `0x85` (pong con versión)
- `0x06 TLV_DETECT`  → `0x86` (dirección I2C detectada)

La aplicación Qt actual solo usa `0x01/0x02` y sus respuestas `0x81/0x82`.

## 10. Recomendaciones

- Responder siempre algo: éxito o error. Eso simplifica timeouts en PC.
- Limitar el trabajo en ISR; realizar el parseo en una tarea dedicada.
- Validar `page` y `reg` en rangos 0..255. Para `WRITE_BURST`, validar tamaño total.
- Loguear con `ESP_LOG*` los eventos clave; no saturar el UART.
- Si compartes UART0 entre consola humana y protocolo binario, alterna con un comando local (`proto on/off`). El firmware expone este conmutador en la CLI.

## 11. Compatibilidad con la app Qt

La app Qt de este repo ya implementa:
- Envío de `WRITE_REG` y `READ_REG`.
- Recepción de `WRITE_ACK` y `READ_VAL`.
- Cualquier frame desconocido (incluyendo `ERROR`) se muestran en el log sin romper el flujo.

Con implementar lo descrito en este documento, PC y ESP32 se comunicarán correctamente desde el primer build.

## 12. Mapa de características (firmware v1.1)

- Requeridos: WRITE_REG, READ_REG.
- Disponibles: WRITE_BURST, READ_BURST, PING/PONG, TLV_DETECT, ERROR frames.
- UART: 115200-8N1; buffer RX 2048 en ESP32; checksum sumatoria complemento a dos.
