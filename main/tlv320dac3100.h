#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

// Minimal TLV320DAC3100 scaffold
// - Detect typical I2C addresses
// - Allow setting selected 7-bit address
// - Provide a basic configure stub (logs only for now)

// Try to detect the TLV320DAC3100 at common addresses (0x18, 0x19).
// If found, writes the found address to addr7_out (if not NULL) and returns true.
bool tlv320_detect(uint8_t *addr7_out);

// Set the active 7-bit I2C address for subsequent operations.
void tlv320_set_address(uint8_t addr7);

// Configure the device for I2S 16-bit stereo playback.
// Note: This is a scaffold; it currently performs minimal actions and logs guidance.
// The speaker path requires 5V VIN on the Adafruit 6309 board.
bool tlv320_configure_basic_i2s_16bit(bool enable_speaker);

// Configure clocks to use BCLK as PLL input (no external MCLK) for I2S 16-bit.
// Supports sample_rate 44100 or 48000 (both use J=8, P=1, R=1 with NDAC=2, MDAC=1, DOSR=128).
// Returns true if register writes succeed.
bool tlv320_configure_bclk_i2s_16(int sample_rate);

// Enable HP/Line-Out path on the TLV320DAC3100 and set maximum analog volume (0 dB).
// Routes both DAC channels to HPL/HPR and powers the HP drivers. Speaker path depends on board design.
bool tlv320_enable_speaker_out_max(void);

// Perform hardware reset via GPIO pin (if configured) followed by full re-initialization
// This is useful when the TLV320 gets into an unknown state
bool tlv320_hardware_reset_and_init(int sample_rate);

// Configure both headphone and speaker outputs with proper routing and volumes
// Enables simultaneous output to both HP jack and Class-AB line outputs
// For speaker: requires 5V supply on Adafruit 6309 board for adequate volume
bool tlv320_configure_dual_output(void);

// Configure ONLY headphone output HPL/HPR with correct register mapping
// Based on TLV320DAC3100 datasheet specifications for proper HP routing
bool tlv320_configure_headphone_only(void);

// Default baseline configuration: conservative, minimal setup to get audio out
// of headphone outputs (HPL/HPR) using I2S 16-bit clocks from BCLK.
// It programs Page 0 clocks (PLL from BCLK, NDAC/MDAC/DOSR), selects PRB_P1,
// enables DAC datapath and powers headphone analog path at 0 dB.
// Returns true on success.
bool tlv320_configure_default(void);

// Enhanced configuration for best audio quality at 44.1/48 kHz, I2S 16-bit.
// - PLL from BCLK with stable dividers (slots=16-bit => BCLK=Fs*32)
// - Processing block PRB_P1
// - Smooth pop-free ramp, DAC at 0 dB, HP PGAs at 0 dB
// - Stereo headphones (HPL/HPR) + mono speaker (L+R summed to left analog path)
// Returns true on success.
// (reserved) bool tlv320_configure_enhanced(int sample_rate);

// Print a summarized status: clock source, PLL params, NDAC/MDAC/DOSR, I2S format,
// derived BCLK/LRCLK ratio expectation and warnings if mismatched.
void tlv320_print_status(void);

// --- UART/Debug helpers ---
// Public wrappers to read/write a TLV register (page/reg) and dump a small debug set.
// These are thin wrappers around the internal I2C accessors used by the driver.
esp_err_t tlv320_reg_read(uint8_t page, uint8_t reg, uint8_t *val_out);
esp_err_t tlv320_reg_write(uint8_t page, uint8_t reg, uint8_t val);
void tlv320_dump_debug_public(void);

// Auto-fix Page 0 clocks for I2S 16-bit (BCLK = Fs * 32). Reads current regs and
// reprograms 0x04, 0x05..0x08, 0x0B..0x0E, 0x1B..0x1D, 0x3C in a safe order.
// sample_rate supported: 44100 or 48000. Returns true on success.
bool tlv320_autofix_page0_for_i2s16(int sample_rate, bool verbose);

