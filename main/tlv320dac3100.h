#pragma once

#include <stdbool.h>
#include <stdint.h>

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
// Routes both DAC channels to HPL/HPR and powers the HP drivers. DAC3100 has no Class-D.
bool tlv320_enable_speaker_out_max(void);

// Perform hardware reset via GPIO pin (if configured) followed by full re-initialization
// This is useful when the TLV320 gets into an unknown state
bool tlv320_hardware_reset_and_init(int sample_rate);

// Configure both headphone and speaker outputs with proper routing and volumes
// Enables simultaneous output to both HP jack and Class-AB line outputs
// For speaker: requires 5V supply on Adafruit 6309 board for adequate volume
// Configure both headphone and speaker outputs.
// Headphones: DAC_L->HPL, DAC_R->HPR at 0 dB.
// Class-D: sums L+R in analog by enabling both routes with 0 dB and powers Class-D with gentle de-pop.
bool tlv320_configure_dual_output(void);

// Configure ONLY headphone output HPL/HPR with correct register mapping
// Based on TLV320DAC3100 datasheet specifications for proper HP routing
// Deprecated: Headphone-only path with conflicting clock assumptions. Prefer tlv320_configure_dual_output().
bool tlv320_configure_headphone_only(void);

