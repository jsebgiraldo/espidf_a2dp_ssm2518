#pragma once

#include "stdint.h"

// Configuración básica del SSM2518 mediante I2C
// Asume formato de audio I2S, 16 bits, estéreo, mute desactivado, ganancia por defecto.
void ssm2518_configure(void);

// Configurar dirección I2C (7-bit). Por defecto 0x34.
void ssm2518_set_address(uint8_t addr7);
