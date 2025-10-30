#ifndef DEFINES_H
#define DEFINES_H

#ifdef __cplusplus
extern "C" {
#endif

#include "mcp_error.h"

// Speed Defines
#define MCP_I2C_SPEED_SLOW      100000u     // 100 kHz
#define MCP_I2C_SPEED_DEFAULT   400000u     // 400 kHz
#define MCP_I2C_SPEED_FAST      3400000u    // 3.4 MHz
#define MCP_SPI_SPEED_SLOW      100000u     // 100 kHz
#define MCP_SPI_SPEED_DEFAULT   1000000u    // 1 MHz
#define MCP_SPI_SPEED_FAST      4000000u    // 4 MHz
#define MCP_SPI_SPEED_MAX       10000000u   // 10 MHz
// MCP23 Register Addresses
#define MCP_IODIR               0x00        // I/O Direction Register
#define MCP_IPOL                0x01        // Input Polarity Port Register
#define MCP_GPINTEN             0x02        // Interrupt-on-Change Control Register
#define MCP_DEFVAL              0x03        // Default Compare Register
#define MCP_INTCON              0x04        // Interrupt Control Register
#define MCP_IOCON               0x05        // Configuration Register
#define MCP_GPPU                0x06        // Pull-up Resistor Register
#define MCP_INTF                0x07        // Interrupt Flag Register
#define MCP_INTCAP              0x08        // Interrupt Captured Register
#define MCP_GPIO                0x09        // General Purpose I/O Register
#define MCP_OLAT                0x0A        // Output Latch Register
// MCP23 Interrupt Commands
#define MCP_INT_DISABLE         0x00
#define MCP_INT_ENABLE          0x01
#define MCP_CHANGE_ANY          0x00
#define MCP_COMPARE_DEFVAL      0x01
// MCP23 HAEN Defines
#define MCP_HAEN_DISABLE        0x00
#define MCP_HAEN_ENABLE         0x01
// MCP23 Device Info
#define MCP_PIN_MAX             0x07
#define MCP_REG_MAX             0x0A
#define MCP_DELAY               0x00
#define MCP_WDELAY              0x00
#define MCP_BPW                 0x08
// MCP23 Convenience Commands
#define MCP_LED_DISABLE         0x00
#define MCP_LED_ENABLE          0x01

#ifdef __cplusplus
}
#endif

#endif /* DEFINES_H */