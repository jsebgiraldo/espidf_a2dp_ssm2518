# Comandos soportados por Qt ↔ ESP32 (v1.1)

Resumen práctico para integrar desde la app Qt. Todos los frames:
AA 55 CMD LEN PAYLOAD... CKS

- Velocidad: 115200 bps, 8N1. Timeout sugerido por comando: 100 ms.
- Checksum: complemento a dos de CMD+LEN+ΣPAYLOAD.

IDs y payloads:

- 0x01 WRITE_REG → 0x81 WRITE_ACK
  - PC→ESP32: [page, reg, value]
  - ESP32→PC: [page, reg, ok]

- 0x02 READ_REG → 0x82 READ_VAL
  - PC→ESP32: [page, reg]
  - ESP32→PC: [page, reg, value]

- 0x03 WRITE_BURST → 0x83 WRITE_BURST_ACK
  - PC→ESP32: [page, start_reg, count, values...]
  - ESP32→PC: [page, start_reg, written_count]

- 0x04 READ_BURST → 0x84 READ_BURST_VAL
  - PC→ESP32: [page, start_reg, count]
  - ESP32→PC: [page, start_reg, count, values...]

- 0x05 PING → 0x85 PONG
  - PC→ESP32: vacío o [vmaj, vmin]
  - ESP32→PC: [proto_maj, proto_min, feature_lo, feature_hi]
  - feature_lo bit0=BURST, bit1=DETECT, bit2=PRESETS

- 0x06 TLV_DETECT → 0x86
  - PC→ESP32: vacío
  - ESP32→PC: [present(0/1), i2c_addr]

- 0x07 A2DP_STATUS → 0x87
  - PC→ESP32: vacío
  - ESP32→PC: [connected(1), audio_state(1), sample_rate(4 LE), rx_bytes(4 LE), tx_bytes(4 LE), have_last_bda(1), last_bda(6)]

- 0x08 PRESET_APPLY → 0x88
  - PC→ESP32: [id]
  - ESP32→PC: [id, ok]
  - IDs expuestos: 0x01 Default, 0x02 HP-only, 0x03 Speaker-only, 0x04 Dual, 0x05 Clocks-only@48k

- 0x09 PRESET_LIST → 0x89
  - PC→ESP32: vacío
  - ESP32→PC: [count, ids[count]]
  - IDs actuales: 0x01 Default, 0x02 HP-only, 0x03 Speaker-only, 0x04 Dual, 0x05 Clocks-only@48k
  - IDs reservados (si el firmware los publica): 0x21 Bass+, 0x22 Mid+, 0x23 Treble+

- 0xE0 ERROR
  - ESP32→PC: [code, last_cmd, info_len, info...]
  - code: 1=I2C_FAIL, 2=BADCMD, 3=BADLEN, 4=BADCKS, 5=INTERNAL

Notas de implementación Qt:
- Mantén un buffer circular y busca 0xAA 0x55 para resincronizar.
- Verifica CKS antes de despachar; descarta frames inválidos y opcionalmente muéstralos en log.
- Limita READ/WRITE_BURST a 32 bytes por frame para baja latencia.
- Secuencia típica de inicio:
  1) Enviar PING; verificar features.
  2) Enviar TLV_DETECT; si present=1, usar addr como referencia.
  3) Operar con READ/WRITE (y BURST si disponible) sobre el TLV.

Compatibilidad:

Sugerencias de comandos Qt (UI):
- Ping + Status A2DP (poll cada 1s) para mostrar: conectado, bitrate, dirección del último dispositivo, bytes RX/TX.
- Detect TLV (botón) y mostrar dirección I2C.
- Lectura/escritura de registros (inputs hex) y ráfagas.
- Presets TLV (combos) que envíen ráfagas conocidas.
- La app Qt existente (v1.0) que usa solo WRITE_REG/READ_REG sigue funcionando sin cambios.
- Las respuestas ERROR son opcionales de manejar; si se reciben, loguearlas.
