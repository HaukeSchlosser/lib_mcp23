#ifndef MCP23S08_H
#define MCP23S08_H

#include <stdint.h>
#include <linux/spi/spidev.h>

#ifdef __cplusplus
extern "C" {
#endif

// MCP23S08 SPI Defines
#define MCP_08S_DELAY           0x00
#define MCP_08S_WDELAY          0x00
#define MCP_08S_BPW             0x08
#define MCP_08S_SPEED_SLOW      100000u     // 100 kHz
#define MCP_08S_SPEED_DEFAULT   1000000u    // 1 MHz
#define MCP_08S_SPEED_FAST      4000000u    // 4 MHz
#define MCP_08S_SPEED_MAX       10000000u   // 10 MHz
// MCP23S08 Device
#define MCP_08S_PIN_MAX         0x07
#define MCP_08S_REG_MAX         0x0A
#define MCP_08S_ADDR_MAX        0x04
#define MCP_08S_BASE            0x40
#define MCP_08S_WRITE_CMD       0x00
#define MCP_08S_READ_CMD        0x01
// MSCP23S08 Interrupt Commands
#define MCP_08S_INT_DISABLE     0x00
#define MCP_08S_INT_ENABLE      0x01
#define MCP_08S_CHANGE_ANY      0x00
#define MCP_08S_COMPARE_DEFVAL  0x01
// MSCP23S08 Register Addresses
#define MCP_08S_IODIR           0x00        // I/O Direction Register
#define MCP_08S_IPOL            0x01        // Input Polarity Port Register
#define MCP_08S_GPINTEN         0x02        // Interrupt-on-Change Control Register
#define MCP_08S_DEFVAL          0x03        // Default Compare Register
#define MCP_08S_INTCON          0x04        // Interrupt Control Register
#define MCP_08S_IOCON           0x05        // Configuration Register
#define MCP_08S_GPPU            0x06        // Pull-up Resistor Register
#define MCP_08S_INTF            0x07        // Interrupt Flag Register
#define MCP_08S_INTCAP          0x08        // Interrupt Captured Register
#define MCP_08S_GPIO            0x09        // General Purpose I/O Register
#define MCP_08S_OLAT            0x0A        // Output Latch Register

#define MCP_08S_HAEN_DISABLE    0x00
#define MCP_08S_HAEN_ENABLE     0x01

typedef struct {
    int fd;
    uint32_t speed_hz;
    uint8_t  haen_enabled;
    uint8_t  mode;
    uint8_t  address;
} mcp23s08_t;

/**
 * @brief Opens and configures an SPI device file for MCP23S08 communication.
 *
 * @param dev           Pointer to the MCP23S08 device handle.
 * @param bus           SPI bus number (0 or 1).
 * @param cs            Chip select line (0 or 1).
 * @param spi_mode      SPI mode (SPI_MODE_0 or SPI_MODE_3).
 * @param speed_hz      SPI communication speed in Hz.
 * @param address       Address of device
 * @param haen_enabled  If HAEN is enabled
 * 
 * @return int      File descriptor for the opened SPI device, or -1 on failure.
 */
int mcp23s08_init(mcp23s08_t *dev, uint8_t bus, uint8_t cs, 
                 uint8_t spi_mode, uint32_t speed_hz, uint8_t 
                 address, uint8_t haen_enabled);

/**
 * @brief Writes a byte of data to a specified register of the MCP23S08 via SPI.
 *
 * @param dev       Pointer to the MCP23S08 device handle.
 * @param reg       Register address to write to.
 * @param data      Byte of data to write.
 * 
 * @return int8_t   Returns 0 on success, or -1 on failure.
 */
int8_t mcp23s08_write(mcp23s08_t *dev, uint8_t reg, uint8_t data);

/**
 * @brief Reads a byte from a specified register of the MCP23S08 via SPI.
 *
 * @param dev       Pointer to the MCP23S08 device handle.
 * @param reg       Register address to read from.
 * 
 * @return int16_t  The received byte from the SPI response, or -1 on failure.
 */
int16_t mcp23s08_read(mcp23s08_t *dev, uint8_t reg);

/**
 * @brief Writes a value to a specific pin of the MCP23S08.
 *
 * @param dev       Pointer to the MCP23S08 device handle.
 * @param reg       Register address to write to.
 * @param pin       Pin number to write (0-7).
 * @param data      Value to write to the pin (0 or 1).
 * 
 * @return int8_t   Returns 0 on success, or -1 on failure.
 */
int8_t mcp23s08_write_pin(mcp23s08_t *dev, uint8_t reg, uint8_t pin, uint8_t data);

/**
 * @brief Reads the value of a specific pin from the MCP23S08.
 *
 * @param dev       Pointer to the MCP23S08 device handle.
 * @param reg       Register address to read from.
 * @param pin       Pin number to read (0-7).
 * 
 * @return int8_t   The value of the pin (0 or 1), or -1 on failure.
 */
int8_t mcp23s08_read_pin(mcp23s08_t *dev, uint8_t reg, uint8_t pin);

/**
 * @brief Configures interrupt settings for specified pins on the MCP23S08.
 *
 * @param dev               Pointer to the MCP23S08 device handle.
 * @param enable            Enable or disable interrupts (MCP_08S_INT_ENABLE or MCP_08S_INT_DISABLE).
 * @param bitmask           Bitmask specifying which pins to configure for interrupts.
 * @param interrupt_mode    Interrupt mode (MCP_08S_CHANGE_ANY or MCP_08S_COMPARE_DEFVAL).
 * 
 * @return int8_t           Returns 0 on success, or -1 on failure.
 */
int8_t mcp23s08_interrupt(mcp23s08_t *dev, uint8_t enable, uint8_t bitmask, uint8_t interrupt_mode);

#ifdef __cplusplus
}
#endif

#endif /* MCP23S08_H */