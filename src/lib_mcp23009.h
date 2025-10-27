#ifndef MCP23009_H
#define MCP23009_H

#include <stdint.h>
#include <linux/i2c-dev.h>

#ifdef __cplusplus
extern "C" {
#endif

// MCP23009 Defines
#define MCP_009_SPEED_SLOW      100000u     // 100 kHz
#define MCP_009_SPEED_DEFAULT   400000u     // 400 kHz
#define MCP_009_SPEED_FAST      3400000u    // 3.4 MHz
// MCP23009 Device
#define MCP_009_PIN_MAX         0x07
#define MCP_009_REG_MAX         0x0A
#define MCP_009_ADDR_MAX        0x07
#define MCP_009_BASE            0x20
#define MCP_009_WRITE_CMD       0x00
#define MCP_009_READ_CMD        0x01
// MCP23009 Interrupt Commands
#define MCP_009_INT_DISABLE     0x00
#define MCP_009_INT_ENABLE      0x01
#define MCP_009_CHANGE_ANY      0x00
#define MCP_009_COMPARE_DEFVAL  0x01
// MCP23009 Register Addresses
#define MCP_009_IODIR           0x00        // I/O Direction Register
#define MCP_009_IPOL            0x01        // Input Polarity Port Register
#define MCP_009_GPINTEN         0x02        // Interrupt-on-Change Control Register
#define MCP_009_DEFVAL          0x03        // Default Compare Register
#define MCP_009_INTCON          0x04        // Interrupt Control Register
#define MCP_009_IOCON           0x05        // Configuration Register
#define MCP_009_GPPU            0x06        // Pull-up Resistor Register
#define MCP_009_INTF            0x07        // Interrupt Flag Register
#define MCP_009_INTCAP          0x08        // Interrupt Captured Register
#define MCP_009_GPIO            0x09        // General Purpose I/O Register
#define MCP_009_OLAT            0x0A        // Output Latch Register

typedef struct {
    int fd;
    uint32_t speed_hz;
    uint8_t  address;
} mcp23009_t;

/**
 * @brief Opens and configures an I2C device file for MCP23009 communication.
 *
 * @param dev       Pointer to the MCP23009 device handle.
 * @param bus       I2C bus number.
 * @param speed_hz  I2C communication speed in Hz.
 * @param address   MCP23009 device address (0-7).
 * 
 * @return int      File descriptor for the opened I2C device, or -1 on failure.
 */
int mcp23009_init(mcp23009_t *dev, uint8_t bus, uint32_t speed_hz, uint8_t address);

/**
 * @brief Writes a byte of data to a specified register of the MCP23009 via I2C.
 *
 * @param dev       Pointer to the MCP23009 device handle.
 * @param reg       Register address to write to.
 * @param data      Byte of data to write.
 * 
 * @return int8_t   Returns 0 on success, or -1 on failure.
 */
int8_t mcp23009_write(mcp23009_t *dev, uint8_t reg, uint8_t data);

/**
 * @brief Reads a byte from a specified register of the MCP23009 via I2C.
 *
 * @param dev       Pointer to the MCP23009 device handle.
 * @param reg       Register address to read from.
 * 
 * @return int16_t  Returns the read byte (0-255) on success, or -1 on failure.
 */
int16_t mcp23009_read(mcp23009_t *dev, uint8_t reg);

/**
 * @brief Writes a value to a specific pin within a register of the MCP23009.
 *
 * @param dev       Pointer to the MCP23009 device handle.
 * @param reg       Register address to write to.
 * @param pin       Pin number (0-7) within the register.
 * @param data      Value to write to the pin (0 or 1).
 * 
 * @return int8_t   Returns 0 on success, or -1 on failure.
 */
int8_t mcp23009_write_pin(mcp23009_t *dev, uint8_t reg, uint8_t pin, uint8_t data);

/**
 * @brief Reads the value of a specific pin within a register of the MCP23009.
 *
 * @param dev       Pointer to the MCP23009 device handle.
 * @param reg       Register address to read from.
 * @param pin       Pin number (0-7) within the register.
 * 
 * @return int8_t   Returns the pin value (0 or 1) on success, or -1 on failure.
 */
int8_t mcp23009_read_pin(mcp23009_t *dev, uint8_t reg, uint8_t pin);

/**
 * @brief Configures interrupt settings for specified pins on the MCP23009.
 *
 * @param dev               Pointer to the MCP23009 device handle.
 * @param enable            Enable or disable interrupts (MCP_009_INT_ENABLE or MCP_009_INT_DISABLE).
 * @param bitmask           Bitmask specifying which pins to configure.
 * @param interrupt_mode    Interrupt mode (MCP_009_CHANGE_ANY or MCP_009_COMPARE_DEFVAL).
 * 
 * @return int8_t           Returns 0 on success, or -1 on failure.
 */
int8_t mcp23009_interrupt(mcp23009_t *dev, uint8_t enable, uint8_t bitmask, uint8_t interrupt_mode);

#ifdef __cplusplus
}
#endif

#endif /* MCP23009_H */