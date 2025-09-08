# TLV320DAC3100 Presets (UART protocol v1.1)

This document defines ready-to-use configuration presets for the TLV320DAC3100, exposed via the binary UART protocol and intended to be consumed by the Qt client. Each preset is a sequence of [page, reg, value] writes (often grouped as a burst) targeting safe, high-quality audio at 44.1/48 kHz using I2S 16-bit clocks sourced from BCLK.

Notes
- MCLK not required. PLL runs from BCLK with stable dividers. I2S slots assumed 16-bit (BCLK = Fs × 32).
- Headphone path runs off 3.3 V. Class-D speaker requires 5 V SPKVDD on the Adafruit 6309 board.
- All presets follow the safe order: interface → PLL/clocks → DAC path → analog routing → power stages → unmute.
- Use WRITE_BURST (0x03) where shown; otherwise apply single WRITE_REG (0x01).

Common Blocks (recommended to send first)
- Interface (I2S 16-bit, slave):
  - P0 1B = 0x00
  - P0 1C = 0x00
  - P0 1D = 0x01
- PLL from BCLK; CODEC_CLKIN=PLL; P=1 R=1 J=8 D=0 (Fs = BCLK/32):
  - P0 04 = 0x07
  - P0 05 = 0x91
  - P0 06 = 0x08
  - P0 07 = 0x00
  - P0 08 = 0x00
- DAC dividers (Fs exact):
  - P0 0B = 0x82   (NDAC=2)
  - P0 0C = 0x81   (MDAC=1)
  - P0 0D = 0x00
  - P0 0E = 0x80   (DOSR=128)
- Processing block and DAC base:
  - P0 3C = 0x01   (PRB_P1)
  - P0 41 = 0x00   (LDAC 0 dB)
  - P0 42 = 0x00   (RDAC 0 dB)
  - P0 3F = 0xD4   (LDAC/RDAC on + datapath)
  - P0 40 = 0x00   (unmute)

Preset A: Headphones only (HPL/HPR, 0 dB)
- Page 1 analog power:
  - P1 01 = 0x08   (disable weak AVDD)
  - P1 02 = 0x01   (master analog power)
- Mixers and routing:
  - P1 0C = 0x08   (DAC_L → left HP mixer)
  - P1 0D = 0x08   (DAC_R → right HP mixer)
  - P1 23 = 0x44   (DAC_L→HPL, DAC_R→HPR)
  - P1 21 = 0x44   (LDAC→Left, RDAC→Right for analog base)
- HP drivers, PGA, volumes, outputs:
  - P1 1F = 0xD0   (HP drivers ON)
  - P1 28 = 0x06   (HPL PGA 0 dB, unmute)
  - P1 29 = 0x06   (HPR PGA 0 dB, unmute)
  - P1 24 = 0x80   (HPL volume connect, 0 dB)
  - P1 25 = 0x80   (HPR volume connect, 0 dB)
  - P1 09 = 0x30   (power HPL/HPR)
- Class-D is left OFF by default (no SPK path).

Preset B: Speaker only (mono L+R → Class‑D)
- Page 1 analog power:
  - P1 01 = 0x08
  - P1 02 = 0x01
- Analog base routing for Class‑D:
  - P1 21 = 0x4C   (LDAC + RDAC summed into left analog mixer)
- Ensure HP path is OFF:
  - P1 1F = 0x00   (HP drivers OFF)
  - P1 23 = 0x00   (no DAC→HP routing)
- Speaker path (requires 5 V):
  - P1 26 = 0x80   (SPK_VOL connect, 0 dB)
  - P1 20 = 0x00   (toggle off)
  - P1 20 = 0x80   (Class‑D ON)
  - P1 2A = 0x04   (unmute, ~6 dB)

Preset C: Dual output (HP stereo + Speaker mono)
- Start from Preset A blocks for HP, then add Class‑D:
  - P1 26 = 0x80   (SPK_VOL connect)
  - P1 20 = 0x00; wait 5 ms; P1 20 = 0x80 (Class‑D ON)
  - P1 2A = 0x04   (unmute, ~6 dB)
- Optionally switch P1 21 to 0x4C for mono sum on speaker while keeping HP stereo.

Preset D: Quality-focused HP (reduced digital volume, soft ramp)
- Same clocks as Common Blocks.
- Digital volumes −8 dB at start:
  - P0 41 = 0x10; P0 42 = 0x10
- HP path as Preset A.
- After 200 ms, ramp P0 41/42 to 0x00 in small steps for pop reduction.

Optional knobs (for Qt UI)
- Digital DAC volume: P0 41/42 (0x00=0 dB … higher = attenuation). Step in 0.5 dB.
- HP analog PGA: P1 28/29 (0x06 ≈ 0 dB; see datasheet for mapping). Avoid large jumps.
- Class‑D gain/unmute: P1 2A (bits set gain; 0x04 ≈ 6 dB, 0x0C ≈ 12 dB, 0x14 ≈ 18 dB, 0x1C ≈ 24 dB). 0x00 mutes.
- Speaker volume connect: P1 26 bit7 (1=connect).

Protocol mapping (for Qt)
- Two integration paths:
  1) High-level preset commands (recommended):
     - PRESET_LIST (0x09) → 0x89: payload [count, ids[count]]
     - PRESET_APPLY (0x08) → 0x88: payload [id], response [id, ok]
       • IDs exposed by firmware:
         - 0x01: Default (HP + speaker minimal)
         - 0x02: Headphones only
         - 0x03: Speaker only
         - 0x04: Dual output
         - 0x05: Clocks-only @ 48 kHz (BCLK mode)
  2) Low-level register sequences (manual):
     - Use WRITE_BURST (0x03) for grouped writes: [page, start_reg, count, values…]
     - Use WRITE_REG (0x01) for scattered writes. Read back with READ_REG (0x02) as needed.

Testing sequence suggestion
1) Ping, detect TLV address.
2) Apply Common Blocks.
3) Apply one preset (A/B/C/D) depending on desired mode.
4) Verify with READ_REG that key registers match.
5) Check audio; tweak volumes/gains via knobs.

Tone presets (reserved for future EQ support)
These presets aim to boost frequency bands independently. The TLV320DAC3100 lacks a built-in parametric EQ; recommended implementation is a light DSP on ESP32 (IIR biquads) before I2S. Until firmware support lands, treat these IDs as reserved and only show them if listed by PRESET_LIST.

- Preset E: Bass boost (ID 0x21)
  - Target: low-shelf +6 dB at ~120 Hz, Q≈0.7
  - Implementation (firmware, future): apply biquad LS on both channels.

- Preset F: Mid boost (ID 0x22)
  - Target: peaking +3 dB at ~1 kHz, Q≈1.0
  - Implementation (firmware, future): apply peaking EQ on both channels.

- Preset G: Treble boost (ID 0x23)
  - Target: high-shelf +6 dB at ~8 kHz, Q≈0.7
  - Implementation (firmware, future): apply biquad HS on both channels.

Qt integration tips
- Query PRESET_LIST; only enable these tone presets if their IDs appear.
- If supported, add a small “Tone” group with three buttons or a combo for Bass+/Mid+/Treble+ and call PRESET_APPLY with the chosen ID.
- Optionally add a “Reset tone” action that reapplies the current base preset (e.g., Default or Headphones only).
