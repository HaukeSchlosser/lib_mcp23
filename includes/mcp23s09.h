#ifndef MCP23S09_H
#define MCP23S09_H

#include "mcp23.h"
#include "defines.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MCP_09S_WRITE_CMD   0x40
#define MCP_09S_READ_CMD    0x41

struct mcp23s09 {
    int fd;
    uint32_t speed_hz;
    uint8_t  mode;
};

/**
 * @brief Opens and configures an SPI device file for MCP23S09 communication.
 *
 * @param dev       Pointer to the MCP23S09 device handle.
 * @param bus       SPI bus number (0 or 1).
 * @param cs        Chip select line (0 or 1).
 * @param spi_mode  SPI mode (SPI_MODE_0 or SPI_MODE_3).
 * @param speed_hz  SPI communication speed in Hz.
 * 
 * @return int      File descriptor for the opened SPI device, or -1 on failure.
 */
int mcp23s09_init(mcp23s09_t *dev, uint8_t bus, uint8_t cs, uint8_t spi_mode, uint32_t speed_hz);

/**
 * @brief Writes a byte of data to a specified register of the MCP23S09 via SPI.
 *
 * @param dev       Pointer to the MCP23S09 device handle.
 * @param reg       Register address to write to.
 * @param data      Byte of data to write.
 * 
 * @return int8_t   Returns 0 on success, or -1 on failure.
 */
int8_t mcp23s09_write(mcp23s09_t *dev, uint8_t reg, uint8_t data);

/**
 * @brief Reads a byte from a specified register of the MCP23S09 via SPI.
 *
 * @param dev       Pointer to the MCP23S09 device handle.
 * @param reg       Register address to read from.
 * 
 * @return int16_t  The received byte from the SPI response, or -1 on failure.
 */
int16_t mcp23s09_read(mcp23s09_t *dev, uint8_t reg);

/**
 * @brief Writes a value to a specific pin of the MCP23S09.
 *
 * @param dev       Pointer to the MCP23S09 device handle.
 * @param reg       Register address to write to.
 * @param pin       Pin number to write (0-7).
 * @param data      Value to write to the pin (0 or 1).
 * 
 * @return int8_t   Returns 0 on success, or -1 on failure.
 */
int8_t mcp23s09_write_pin(mcp23s09_t *dev, uint8_t reg, uint8_t pin, uint8_t data);

/**
 * @brief Reads the value of a specific pin from the MCP23S09.
 *
 * @param dev       Pointer to the MCP23S09 device handle.
 * @param reg       Register address to read from.
 * @param pin       Pin number to read (0-7).
 * 
 * @return int8_t   The value of the pin (0 or 1), or -1 on failure.
 */
int8_t mcp23s09_read_pin(mcp23s09_t *dev, uint8_t reg, uint8_t pin);

/**
 * @brief Configures interrupt settings for specified pins on the MCP23S09.
 *
 * @param dev               Pointer to the MCP23S09 device handle.
 * @param enable            Enable or disable interrupts (MCP_09S_INT_ENABLE or MCP_09S_INT_DISABLE).
 * @param bitmask           Bitmask specifying which pins to configure for interrupts.
 * @param interrupt_mode    Interrupt mode (MCP_09S_CHANGE_ANY or MCP_09S_COMPARE_DEFVAL).
 * 
 * @return int8_t           Returns 0 on success, or -1 on failure.
 */
int8_t mcp23s09_interrupt(mcp23s09_t *dev, uint8_t enable, uint8_t bitmask, uint8_t interrupt_mode);

#ifdef __cplusplus
}
#endif

#endif /* MCP23S09_H */