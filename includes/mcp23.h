#ifndef MCP23_H
#define MCP23_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "defines.h"

struct mcp23s08;  typedef struct mcp23s08 mcp23s08_t;
struct mcp23s09;  typedef struct mcp23s09 mcp23s09_t;
struct mcp23009;  typedef struct mcp23009 mcp23009_t;

typedef enum {
    MCP_VARIANT_23S08,
    MCP_VARIANT_23S09,
    MCP_VARIANT_23009
} mcp_variant_t;

struct mcp_dev_base {
    mcp_error_t   last_error;
    int           fd;
    mcp_variant_t variant;
};

typedef enum { 
    SPI, 
    I2C 
} mcp_bus_t;

typedef struct mcp_dev {
    mcp_dev_base_t base;
    union {
        mcp23s08_t *s08;
        mcp23s09_t *s09;
        mcp23009_t *i09;
    } u;
} mcp_dev_t;

typedef struct {
    mcp_variant_t variant;
    mcp_bus_t     bus;
    union {
        struct {
            uint8_t  bus;
            uint8_t  cs;
            uint8_t  mode;
            uint32_t speed_hz;
            uint8_t  address;
            uint8_t  haen_enabled;
        } spi;
        struct {
            uint8_t  bus;
            uint32_t speed_hz;
            uint8_t  address;
        } i2c;
    } u;
} mcp_cfg_t;

/**
 * @brief Opens and configures an device file for MCP23 communication.
 *
 * @param dev           Pointer to the MCP23 device handle.
 * @param cfg           Pointer to the configuration structure.
 * 
 * @return mcp_err_t    Returns MCP_OK on success, or a negative error code on failure.
 */
mcp_err_t mcp_init(mcp_dev_t *dev, const void *cfg);

/**
 * @brief Closes the device file associated with the MCP23 device.
 *
 * @param dev           Pointer to the MCP23 device handle.
 * 
 * @return mcp_err_t    Returns MCP_OK on success, or a negative error code on failure.
 */
mcp_err_t mcp_close(mcp_dev_t *dev);

/**
 * @brief Writes a byte of data to a specified register of the MCP23 device.
 *
 * @param dev           Pointer to the MCP23 device handle.
 * @param reg           Register address to write to.
 * @param data          Byte of data to write.
 * 
 * @return mcp_err_t    Returns MCP_OK on success, or a negative error code on failure.
 */
mcp_err_t mcp_write(mcp_dev_t *dev, uint8_t reg, uint8_t data);

/**
 * @brief Reads a byte from a specified register of the MCP23 device.
 *
 * @param dev       Pointer to the MCP23 device handle.
 * @param reg       Register address to read from.
 * 
 * @return int16_t  The received byte from the device, or a negative error code on failure.
 */
int16_t mcp_read(mcp_dev_t *dev, uint8_t reg);

/**
 * @brief Writes a value to a specific pin of the MCP23 device.
 *
 * @param dev           Pointer to the MCP23 device handle.
 * @param reg           Register address to write to.
 * @param pin           Pin number to write (0-7).
 * @param data          Value to write to the pin (0 or 1).
 * 
 * @return mcp_err_t    Returns MCP_OK on success, or a negative error code on failure.
 */
mcp_err_t mcp_write_pin(mcp_dev_t *dev, uint8_t reg, uint8_t pin, uint8_t data);

/*
 * @brief Reads the value of a specific pin from the MCP23 device.
 * 
 * @param dev       Pointer to the MCP23 device handle.
 * @param reg       Register address to read from.
 * @param pin       Pin number to read (0-7).
 * 
 * @return int8_t   The value of the pin (0 or 1), or a negative error code on failure.
 */
int8_t mcp_read_pin(mcp_dev_t *dev, uint8_t reg, uint8_t pin);

/**
 * @brief Configures interrupt settings for the MCP23 device.
 *
 * @param dev               Pointer to the MCP23 device handle.
 * @param enable            Enable or disable interrupts (MCP_INT_ENABLE or MCP_INT_DISABLE).
 * @param bitmask           Bitmask specifying which pins to configure for interrupts.
 * @param interrupt_mode    Interrupt mode (MCP_CHANGE_ANY or MCP_COMPARE_DEFVAL).
 * 
 * @return mcp_err_t        Returns MCP_OK on success, or a negative error code on failure.
 */
mcp_err_t mcp_interrupt(mcp_dev_t *dev, uint8_t enable, uint8_t bitmask, uint8_t interrupt_mode);

/**
 * @brief Enables or disables LED on all pins of the MCP23 device.
 *
 * @param dev           Pointer to the MCP23 device handle.
 * @param enable        Enable or disable LED (MCP_LED_ENABLE or MCP_LED_DISABLE).
 * 
 * @return mcp_err_t    Returns MCP_OK on success, or a negative error code on failure.
 */
mcp_err_t mcp_led(mcp_dev_t *dev, uint8_t enable);

/*
 * @brief Retrieves the last error code for the MCP23 device.
 *
 * @param dev           Pointer to the MCP23 device handle.
 * 
 * @return mcp_err_t    The last error code of the device, or NULL on failure.
 */
const mcp_error_t* mcp_get_error(const mcp_dev_t *dev);

#define MCP_CFG_SPI_23S08(_bus,_cs,_mode,_speed,_addr,_haen) \
    ((mcp_cfg_t){ .variant=MCP_VARIANT_23S08, .bus=SPI, \
        .u.spi={.bus=_bus,.cs=_cs,.mode=_mode,.speed_hz=_speed,.address=_addr,.haen_enabled=_haen}})

#define MCP_CFG_SPI_23S09(_bus,_cs,_mode,_speed) \
    ((mcp_cfg_t){ .variant=MCP_VARIANT_23S09, .bus=SPI, \
        .u.spi={.bus=_bus,.cs=_cs,.mode=_mode,.speed_hz=_speed,.address=0,.haen_enabled=0}})

#define MCP_CFG_I2C_23009(_bus,_speed,_addr) \
    ((mcp_cfg_t){ .variant=MCP_VARIANT_23009, .bus=I2C, \
        .u.i2c={.bus=_bus,.speed_hz=_speed,.address=_addr}})

#ifdef __cplusplus
}
#endif

#endif /* MCP23_H */