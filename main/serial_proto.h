#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "driver/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

// Start the binary serial protocol task on the given UART.
// If the UART driver is not installed, this function will install it with 115200-8N1 and a 2KB RX buffer.
// Returns true if the task is running (started or already active), false on failure.
bool serial_proto_start(uart_port_t uart_num);

// Stop the binary serial protocol task if running. Returns true if stopped or not running.
bool serial_proto_stop(void);

// Query whether the protocol task is currently running.
bool serial_proto_is_running(void);

// Optional: set a custom log tag to use inside the module (default: "SERPROT").
void serial_proto_set_log_tag(const char *tag);

#ifdef __cplusplus
}
#endif
