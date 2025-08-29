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
bool tlv320_configure_dual_output(void);

// Configure ONLY headphone output HPL/HPR with correct register mapping
// Based on TLV320DAC3100 datasheet specifications for proper HP routing
bool tlv320_configure_headphone_only(void);

// Advanced debugging and health check function
// Monitors critical registers and provides detailed diagnostics
void tlv320_advanced_debug_and_health_check(void);

// Watchdog function to monitor and recover from audio dropouts
// Call periodically to detect and fix audio path issues
bool tlv320_audio_watchdog_check_and_recover(void);

// Diagnose and fix specific problems based on logs
// Specifically addresses dropout issues seen in user logs
void tlv320_diagnose_and_fix_dropout_issues(void);

// Emergency fix for mixer configuration problem
// Forces correct mixer routing when DAC->HP path is broken
bool tlv320_emergency_mixer_fix(void);

// 🔧 CRITICAL: Force correct clock configuration for BCLK=64×Fs
// Call this after ESP32 I2S is generating BCLK with 32-bit slots
// Overwrites previous clock config with mathematically correct values
void tlv_force_clock_for_64fs(void);

// ========== CLOCK VERIFICATION ==========
// Verify clock math for debugging - call after I2S config
void tlv320_verify_clock_math(int sample_rate, bool is_32bit_slots);

// Read and verify registers after forcing 64×Fs configuration
void tlv320_readback_clock(int sample_rate);

// Dump current TLV320 clock configuration registers
void tlv320_dump_clock_config(void);

// ========== MCLK OPTIMIZATIONS ==========
// Configure TLV320 to use MCLK directly (no PLL) for best jitter performance
// MCLK = 256*Fs, NDAC=8, MDAC=2, DOSR=128 → internal clock = Fs
bool tlv320_configure_mclk_direct(int sample_rate);

// Initialize TLV320 for headphone output using MCLK instead of BCLK→PLL
// Best audio quality with ESP32 APLL-generated stable MCLK
bool tlv320_init_hp_from_mclk(int sample_rate);
