#ifndef MCP23S08_H
#define MCP23S08_H

#include "mcp23.h"
#include "defines.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MCP_08S_ADDR_MAX    0x03
#define MCP_08S_BASE        0x40
#define MCP_08S_WRITE_CMD   0x00
#define MCP_08S_READ_CMD    0x01

struct mcp23s08 {
    int fd;
    uint32_t speed_hz;
    uint8_t  haen_enabled;
    uint8_t  mode;
    uint8_t  address;
};

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