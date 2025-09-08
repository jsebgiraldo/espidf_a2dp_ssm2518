Objective: Implement Qt6 (Widgets or QML) client to control ESP32 TLV320DAC3100 via UART protocol v1.1.

Key tasks:
1) Serial transport
- QSerialPort @ 115200-8N1, no flow control
- Frame: AA 55 CMD LEN PAYLOAD CKS; CKS = two's complement of sum(CMD+LEN+PAYLOAD)
- Parser with resync: scan for AA 55, validate CKS, drop invalid bytes
- Single in-flight request; 100 ms timeout per command

2) Core commands (must)
- PING (0x05) → PONG (0x85): parse proto version + feature bits
- TLV_DETECT (0x06) → 0x86: presence + i2c addr
- READ_REG (0x02) → 0x82: [page, reg, value]
- WRITE_REG (0x01) → 0x81: [page, reg, ok]
- A2DP_STATUS (0x07) → 0x87: [connected, audio_state, sample_rate u32 LE, rx u32, tx u32, have_bda, bda[6]]

3) Optional commands
- WRITE_BURST (0x03)/READ_BURST (0x04) for efficient block operations

4) UI ideas
- Status panel: connected, audio_state name, sample_rate, bytes RX/TX, last BDA (if any)
- Detect TLV: show present + i2c addr
- Reg RW: inputs (page/reg/value), buttons Read/Write
- Presets combo (future): send known TLV sequences using BURST

5) Testing flow
- Button: Ping → show proto/features
- Timer (1s): poll A2DP_STATUS and update status
- Button: TLV Detect
- Reg read/write interactions

6) C++ structure
- SerialClient class: open(port), send(cmd,payload), awaitResponse(expectedCmd)
- High-level helpers: ping(), detect(), readReg(p,r), writeReg(p,r,v), getA2dpStatus()
- Use signals/slots to update UI

Specs reference
- See docs/serial-protocol.md (v1.1) and docs/qt-commands.md for IDs/payloads
 - See docs/tlv320-presets.md for predefined TLV presets (HP-only, Speaker-only, Dual, Quality HP) and suggested sequences.
