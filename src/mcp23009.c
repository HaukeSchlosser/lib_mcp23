#include <stdint.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <string.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include "../includes/mcp23009.h"

/*
 * @brief Helper function to set IOCON register for interrupt configuration.
 */
static mcp_err_t set_iocon_for_interrupt(mcp23009_t *dev) {

    if (!dev) return MCP_EPARAM;

    uint8_t iocon = 0;
    iocon |= (0 << 5);
    iocon |= (1 << 2);
    iocon |= (0 << 1);
    iocon |= (1 << 0);

    return mcp23009_write(dev, MCP_IOCON, iocon);
}

/*
* @brief Helper function to set specified pins as input and enable pull-ups.
*/
static mcp_err_t set_pins_input_and_pullups(mcp23009_t *dev, uint8_t bitmask) {

    if (!dev) return MCP_EPARAM;

    int16_t r = mcp23009_read(dev, MCP_IODIR);
    if (r < 0) return (mcp_err_t)r;

    uint8_t iodir = (uint8_t)r | bitmask;
    int rc = mcp23009_write(dev, MCP_IODIR, iodir);
    if (rc < 0) return (mcp_err_t)rc;

    r = mcp23009_read(dev, MCP_GPPU);
    if (r < 0) return (mcp_err_t)r;

    uint8_t gppu = (uint8_t)r | bitmask;
    rc = mcp23009_write(dev, MCP_GPPU, gppu);
    if (rc < 0) return (mcp_err_t)rc;

    return 0;
}

/*
 * @brief Helper function to configure INTCON and DEFVAL registers for interrupts.
 */
static mcp_err_t configure_interrupt_mode(mcp23009_t *dev, uint8_t bitmask, uint8_t interrupt_mode) {

    if (!dev) return MCP_EPARAM;

    int rc;
    int16_t r = mcp23009_read(dev, MCP_INTCON);
    if (r < 0) return (mcp_err_t)r;

    uint8_t intcon = (uint8_t)r;
    if (interrupt_mode == MCP_CHANGE_ANY) {
        intcon &= (uint8_t)~bitmask;
        rc = mcp23009_write(dev, MCP_INTCON, intcon);
        if (rc < 0) return (mcp_err_t)rc;
    } else {
        intcon |= bitmask;
        rc = mcp23009_write(dev, MCP_INTCON, intcon);
        if (rc < 0) return (mcp_err_t)rc;

        r = mcp23009_read(dev, MCP_DEFVAL);
        if (r < 0) return (mcp_err_t)r;

        uint8_t defval = (uint8_t)r;
        defval = (uint8_t)((defval & ~bitmask) | bitmask);
        rc = mcp23009_write(dev, MCP_DEFVAL, defval);
        if (rc < 0) return (mcp_err_t)rc;
    }

    return MCP_OK;
}

/*
 * @brief Helper function to update GPINTEN register for enabling/disabling interrupts.
 */
static mcp_err_t update_gpinten(mcp23009_t *dev, uint8_t bitmask, uint8_t enable) {

    if (!dev) return MCP_EPARAM;

    int16_t r = mcp23009_read(dev, MCP_GPINTEN);
    if (r < 0) return (mcp_err_t)r;

    uint8_t gpinten = (uint8_t)r;
    if (enable == MCP_INT_ENABLE) {
        gpinten |= bitmask;
    } else {
        gpinten &= (uint8_t)~bitmask;
    }

    int rc = mcp23009_write(dev, MCP_GPINTEN, gpinten);
    if (rc < 0) return (mcp_err_t)rc;

    return MCP_OK;
}

/*
 * @brief Helper function to clear pending interrupts by reading INTCAP register.
 */
static mcp_err_t clear_pending_int(mcp23009_t *dev) {

    if (!dev) return MCP_EPARAM;

    int16_t r = mcp23009_read(dev, MCP_INTCAP);
    if (r < 0) return (mcp_err_t)r;

    return MCP_OK;
}

mcp_err_t mcp23009_init(mcp23009_t *dev, uint8_t bus, uint32_t speed_hz, uint8_t address) {

    if (!dev) return MCP_EPARAM;
    if (address > MCP_009_ADDR_MAX) return MCP_FAIL(dev, MCP_ECONFIG);

    // clamp
    if (speed_hz < MCP_I2C_SPEED_SLOW) speed_hz = MCP_I2C_SPEED_SLOW;
    if (speed_hz > MCP_I2C_SPEED_FAST) speed_hz = MCP_I2C_SPEED_FAST;

    char bus_name[16];
    int n = snprintf(bus_name, sizeof bus_name, "/dev/i2c-%u", (unsigned)bus);
    if (n < 0 || n >= (int)sizeof bus_name) return MCP_FAIL(dev, MCP_EIO);

    int fd = open(bus_name, O_RDWR);
    if (fd < 0) return MCP_FAIL(dev, mcp_map_errno());

   uint8_t i2c_addr = (uint8_t)(MCP_009_BASE | (address & MCP_009_ADDR_MAX));
    if (ioctl(fd, I2C_SLAVE, i2c_addr) < 0) {
        mcp_err_t rc = MCP_FAIL(dev, mcp_map_errno());
        close(fd);
        return rc;
    }

    memset(dev, 0, sizeof *dev);
    dev->base.fd  = fd;
    dev->speed_hz = speed_hz;
    dev->address  = i2c_addr;

    return MCP_OK;
}

mcp_err_t mcp23009_write(mcp23009_t *dev, uint8_t reg, uint8_t data) {
    if (!dev)               return MCP_EPARAM;
    if (dev->base.fd < 0)   return MCP_FAIL(dev, MCP_ESTATE);
    if (reg > MCP_REG_MAX)  return MCP_FAIL(dev, MCP_EPARAM);

    uint8_t buf[2] = { reg, data };
    ssize_t n = write(dev->base.fd, buf, sizeof buf);

    if (n < 0) return MCP_FAIL(dev, mcp_map_errno());
    if (n != (ssize_t)sizeof buf) return MCP_FAIL(dev, MCP_EIO);

    return MCP_OK;
}

int16_t mcp23009_read(mcp23009_t *dev, uint8_t reg) {

    if (!dev)               return MCP_EPARAM;
    if (dev->base.fd < 0)   return MCP_FAIL(dev, MCP_ESTATE);
    if (reg > MCP_REG_MAX)  return MCP_FAIL(dev, MCP_EPARAM);

    ssize_t n = write(dev->base.fd, &reg, 1);
    if (n < 0) return MCP_FAIL(dev, mcp_map_errno());
    if (n != 1) return MCP_FAIL(dev, MCP_EIO);

    uint8_t data = 0;
    n = read(dev->base.fd, &data, 1);
    if (n < 0) return MCP_FAIL(dev, mcp_map_errno());
    if (n != 1) return MCP_FAIL(dev, MCP_EIO);

    return (int16_t)data;
}

mcp_err_t mcp23009_write_pin(mcp23009_t *dev, uint8_t reg, uint8_t pin, uint8_t data) {

    if (!dev)               return MCP_EPARAM;
    if (dev->base.fd < 0)   return MCP_FAIL(dev, MCP_ESTATE);
    if (reg > MCP_REG_MAX)  return MCP_FAIL(dev, MCP_EPARAM);
    if (pin > MCP_PIN_MAX)  return MCP_FAIL(dev, MCP_EPARAM);

    int16_t r = mcp23009_read(dev, reg);
    if (r < 0) return (mcp_err_t)r;

    uint8_t val = (uint8_t)r;
    uint8_t mask = (uint8_t)(1u << pin);

    if (data) {
        val = (uint8_t)(val |  mask);
    } else {
        val = (uint8_t)(val & (uint8_t)~mask);
    }

    mcp_err_t rc = mcp23009_write(dev, reg, val);
    if (rc < 0) return rc;

    return MCP_OK;
}

int8_t mcp23009_read_pin(mcp23009_t *dev, uint8_t reg, uint8_t pin) {
    
    if (!dev)               return MCP_EPARAM;
    if (dev->base.fd < 0)   return MCP_FAIL(dev, MCP_ESTATE);
    if (reg > MCP_REG_MAX)  return MCP_FAIL(dev, MCP_EPARAM);
    if (pin > MCP_PIN_MAX)  return MCP_FAIL(dev, MCP_EPARAM);

    int16_t r = mcp23009_read(dev, reg);
    if (r < 0) return (mcp_err_t)r;

    return ((uint8_t)r >> pin) & 0x1;
}

mcp_err_t mcp23009_interrupt(mcp23009_t *dev, uint8_t enable, uint8_t bitmask, uint8_t interrupt_mode) {

    if (!dev)                          return MCP_EPARAM;
    if (dev->base.fd < 0)              return MCP_FAIL(dev, MCP_ESTATE);
    if (enable != MCP_INT_ENABLE && enable != MCP_INT_DISABLE)
                                       return MCP_FAIL(dev, MCP_EPARAM);
    if (interrupt_mode != MCP_CHANGE_ANY &&
        interrupt_mode != MCP_COMPARE_DEFVAL)
                                       return MCP_FAIL(dev, MCP_EPARAM);
    if (bitmask == 0)                  return MCP_FAIL(dev, MCP_EPARAM);

    mcp_err_t rc = set_iocon_for_interrupt(dev);
    if (rc < 0) return rc;

    rc = set_pins_input_and_pullups(dev, bitmask);
    if (rc < 0) return rc;

    rc = configure_interrupt_mode(dev, bitmask, interrupt_mode);
    if (rc < 0) return rc;

    rc = update_gpinten(dev, bitmask, enable);
    if (rc < 0) return rc;

    rc = clear_pending_int(dev);
    if (rc < 0) return rc;

    return MCP_OK;
}

mcp_err_t mcp23009_led(mcp23009_t *dev, uint8_t enable) {

    if (!dev)                    return MCP_EPARAM;
    if (dev->base.fd < 0)        return MCP_FAIL(dev, MCP_ESTATE);

    if (enable != MCP_LED_ENABLE && enable != MCP_LED_DISABLE)
                                 return MCP_FAIL(dev, MCP_EPARAM);

    // Set all pins as outputs and turn off all LEDs
    mcp_err_t rc = mcp23009_write(dev, MCP_IODIR, 0xFF); // reset
    if (rc < 0) return rc;

    rc = mcp23009_write(dev, MCP_IODIR, 0x00); // IODIR: all output
    if (rc < 0) return rc;

    rc = mcp23009_write(dev, MCP_OLAT, 0xFF); // OLAT: all high (LEDs off)
    if (rc < 0) return rc;
    
    if (enable == MCP_LED_ENABLE) {
        rc = mcp23009_write(dev, MCP_OLAT, 0x00);
        if (rc < 0) return rc;
        return MCP_OK;
    }
    if (enable == MCP_LED_DISABLE) {
        rc = mcp23009_write(dev, MCP_OLAT, 0xFF);
        if (rc < 0) return rc;
        return MCP_OK;
    }

    return MCP_OK;
}