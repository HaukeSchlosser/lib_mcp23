#include <stdint.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <sys/ioctl.h>

#include "../includes/mcp23s08.h"
#include "../includes/mcp_spi.h"

static int8_t get_control_byte(mcp23s08_t *dev, uint8_t cmd) {

    if (!dev)                                                           return MCP_EPARAM;
    if (dev->address > MCP_08S_ADDR_MAX)                                return MCP_FAIL(dev, MCP_ECONFIG);
    if (dev->haen_enabled != MCP_HAEN_ENABLE && dev->address != 0x00)   return MCP_FAIL(dev, MCP_ECONFIG);
    if (cmd != MCP_08S_WRITE_CMD && cmd != MCP_08S_READ_CMD)            return MCP_FAIL(dev, MCP_EPARAM);

    return (MCP_08S_BASE | ((dev->address & MCP_08S_ADDR_MAX) << 1) | (cmd & 0x01));
}

/*
 * @brief Helper function to set IOCON register for interrupt configuration.
 */
static mcp_err_t set_iocon_for_interrupt(mcp23s08_t *dev) {

    if (!dev) return MCP_EPARAM;

    uint8_t iocon = 0;
    iocon |= (0 << 5);
    iocon |= (0 << 4);
    iocon |= (dev->haen_enabled << 3);
    iocon |= (1 << 2);
    iocon |= (0 << 1);

    return mcp23s08_write(dev, MCP_IOCON, iocon);
}

/*
* @brief Helper function to set specified pins as input and enable pull-ups.
*/
static mcp_err_t set_pins_input_and_pullups(mcp23s08_t *dev, uint8_t bitmask) {

    if (!dev) return MCP_EPARAM;

    int16_t r = mcp23s08_read(dev, MCP_IODIR);
    if (r < 0) return (mcp_err_t)r;

    uint8_t iodir = (uint8_t)r | bitmask;
    int rc = mcp23s08_write(dev, MCP_IODIR, iodir);
    if (rc < 0) return (mcp_err_t)rc;

    r = mcp23s08_read(dev, MCP_GPPU);
    if (r < 0) return (mcp_err_t)r;

    uint8_t gppu = (uint8_t)r | bitmask;
    rc = mcp23s08_write(dev, MCP_GPPU, gppu);
    if (rc < 0) return (mcp_err_t)rc;

    return MCP_OK;
}

/*
 * @brief Helper function to configure INTCON and DEFVAL registers for interrupts.
 */
static mcp_err_t configure_interrupt_mode(mcp23s08_t *dev, uint8_t bitmask, uint8_t interrupt_mode) {

    if (!dev) return MCP_EPARAM;

    int rc;
    int16_t r = mcp23s08_read(dev, MCP_INTCON);
    if (r < 0) return (mcp_err_t)r;

    uint8_t intcon = (uint8_t)r;
    if (interrupt_mode == MCP_CHANGE_ANY) {
        intcon &= (uint8_t)~bitmask;
        rc = mcp23s08_write(dev, MCP_INTCON, intcon);
        if (rc < 0) return (mcp_err_t)rc;
    } else {
        intcon |= bitmask;
        rc = mcp23s08_write(dev, MCP_INTCON, intcon);
        if (rc < 0) return (mcp_err_t)rc;

        r = mcp23s08_read(dev, MCP_DEFVAL);
        if (r < 0) return (mcp_err_t)r;

        uint8_t defval = (uint8_t)r;
        defval = (uint8_t)((defval & ~bitmask) | bitmask);
        rc = mcp23s08_write(dev, MCP_DEFVAL, defval);
        if (rc < 0) return (mcp_err_t)rc;
    }

    return MCP_OK;
}

/*
 * @brief Helper function to update GPINTEN register for enabling/disabling interrupts.
 */
static mcp_err_t update_gpinten(mcp23s08_t *dev, uint8_t bitmask, uint8_t enable) {

    if (!dev) return MCP_EPARAM;

    int16_t r = mcp23s08_read(dev, MCP_GPINTEN);
    if (r < 0) return (mcp_err_t)r;

    uint8_t gpinten = (uint8_t)r;
    if (enable == MCP_INT_ENABLE) {
        gpinten |= bitmask;
    } else {
        gpinten &= (uint8_t)~bitmask;
    }

    int rc = mcp23s08_write(dev, MCP_GPINTEN, gpinten);
    if (rc < 0) return (mcp_err_t)rc;

    return MCP_OK;
}

/*
 * @brief Helper function to clear pending interrupts by reading INTCAP register.
 */
static mcp_err_t clear_pending_int(mcp23s08_t *dev) {

    if (!dev) return MCP_EPARAM;

    int16_t r = mcp23s08_read(dev, MCP_INTCAP);
    if (r < 0) return (mcp_err_t)r;

    return MCP_OK;
}

mcp_err_t mcp23s08_init(mcp23s08_t *dev, uint8_t bus, uint8_t cs, 
                 uint8_t spi_mode, uint32_t speed_hz, uint8_t 
                 address, uint8_t haen_enabled) {

    if (!dev) return MCP_EPARAM;
    if (bus >= 2)  return MCP_FAIL(dev, MCP_EPARAM);
    if (cs  >= 2)  return MCP_FAIL(dev, MCP_EPARAM);

    if (address > MCP_08S_ADDR_MAX)                 return MCP_FAIL(dev, MCP_ECONFIG);
    if (spi_mode != SPI_MODE_0 && spi_mode != SPI_MODE_3)
                                                    return MCP_FAIL(dev, MCP_EPARAM);
    if (haen_enabled != MCP_HAEN_DISABLE && haen_enabled != MCP_HAEN_ENABLE)
                                                    return MCP_FAIL(dev, MCP_ECONFIG);

    // clamp
    if (speed_hz < MCP_SPI_SPEED_SLOW) speed_hz = MCP_SPI_SPEED_SLOW;
    if (speed_hz > MCP_SPI_SPEED_MAX) speed_hz = MCP_SPI_SPEED_MAX;

    static const char * spidev[2][2] = {
        {"/dev/spidev0.0", "/dev/spidev0.1"},
        {"/dev/spidev1.0", "/dev/spidev1.1"},
    };

    int fd = open(spidev[bus][cs], O_RDWR);
    if (fd < 0) return MCP_FAIL(dev, mcp_map_errno());

    memset(dev, 0, sizeof *dev);
    dev->haen_enabled = haen_enabled;
    dev->speed_hz = speed_hz;
    dev->mode = spi_mode;
    dev->address = address;

    uint8_t req_mode8 = (spi_mode == SPI_MODE_3) ? (SPI_CPOL | SPI_CPHA) : 0u;
    if (ioctl(fd, SPI_IOC_WR_MODE, &req_mode8) < 0) {
        mcp_err_t rc = MCP_FAIL(dev, mcp_map_errno());
        close(fd);
        return rc;
    }

    uint8_t apm = 0;
    if (ioctl(fd, SPI_IOC_RD_MODE, &apm) < 0) {
        mcp_err_t rc = MCP_FAIL(dev, mcp_map_errno());
        close(fd);
        return rc;
    }
    if (apm != req_mode8) {
        mcp_err_t rc = MCP_FAIL(dev, MCP_EIO);
        close(fd);
        return rc;
    }

    uint8_t spi_bpw = MCP_BPW;
    if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &spi_bpw) < 0) {
        mcp_err_t rc = MCP_FAIL(dev, mcp_map_errno());
        close(fd);
        return rc;
    }

    uint8_t rb_bpw = 0;
    if (ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &rb_bpw) < 0) {
        mcp_err_t rc = MCP_FAIL(dev, mcp_map_errno());
        close(fd);
        return rc;
    }
    if (rb_bpw != spi_bpw) {
        mcp_err_t rc = MCP_FAIL(dev, MCP_EIO);
        close(fd);
        return rc;
    }

    uint32_t req_speed = dev->speed_hz;
    if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &req_speed) < 0) {
        mcp_err_t rc = MCP_FAIL(dev, mcp_map_errno());
        close(fd);
        return rc;
    }

    uint32_t aps = 0;
    if (ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &aps) == 0 && aps) {
        if (aps != dev->speed_hz) {
            printf("[mcp23::mcp23s08_init] INFO: driver applied %u Hz (requested %u Hz)\n", aps, dev->speed_hz);
        }
        dev->speed_hz = aps;
    }

    dev->base.fd = fd;
    return MCP_OK;
}

mcp_err_t mcp23s08_write(mcp23s08_t *dev, uint8_t reg, uint8_t data) {

    if (!dev)                 return MCP_EPARAM;
    if (dev->base.fd < 0)     return MCP_FAIL(dev, MCP_ESTATE);
    if (reg > MCP_REG_MAX)    return MCP_FAIL(dev, MCP_EPARAM);
    if (dev->address > MCP_08S_ADDR_MAX)
                              return MCP_FAIL(dev, MCP_ECONFIG);

    uint8_t ctr_byte = get_control_byte(dev, MCP_08S_WRITE_CMD);
    uint8_t tx_buf[3] = {ctr_byte, reg, data};

    int rc = mcp23_spi_transfer(dev->base.fd, tx_buf, NULL, sizeof tx_buf);
    if (rc < 0) return MCP_FAIL(dev, rc);

    return MCP_OK;
}

int16_t mcp23s08_read(mcp23s08_t *dev, uint8_t reg)
{
    if (!dev)                      return MCP_EPARAM;
    if (dev->base.fd < 0)          return MCP_FAIL(dev, MCP_ESTATE);
    if (dev->address > MCP_08S_ADDR_MAX)
                                   return MCP_FAIL(dev, MCP_ECONFIG);
    if (reg > MCP_REG_MAX)         return MCP_FAIL(dev, MCP_EPARAM);

    uint8_t ctr = get_control_byte(dev, MCP_08S_READ_CMD);
    uint8_t tx[3] = {ctr, reg, 0x00};
    uint8_t rx[sizeof tx];

    int rc = mcp23_spi_transfer(dev->base.fd, tx, rx, sizeof tx);
    if (rc < 0) return MCP_FAIL(dev, rc);

    return (int16_t)rx[2];
}

mcp_err_t mcp23s08_write_pin(mcp23s08_t *dev, uint8_t reg, uint8_t pin, uint8_t data) {

    if (!dev)                  return MCP_EPARAM;
    if (dev->base.fd < 0)      return MCP_FAIL(dev, MCP_ESTATE);
    if (reg > MCP_REG_MAX)     return MCP_FAIL(dev, MCP_EPARAM);
    if (pin > MCP_PIN_MAX)     return MCP_FAIL(dev, MCP_EPARAM);

    int16_t r = mcp23s08_read(dev, reg);
    if (r < 0) return (mcp_err_t)r;

    uint8_t val = (uint8_t)r;
    if (data) {
        val = (uint8_t)(val | (uint8_t)(1u << pin));
    } else {
        val = (uint8_t)(val & (uint8_t)~(uint8_t)(1u << pin));
    }

    mcp_err_t rc = mcp23s08_write(dev, reg, val);
    if (rc < 0) return rc;

    return MCP_OK;
}

int8_t mcp23s08_read_pin(mcp23s08_t *dev, uint8_t reg, uint8_t pin) {

    if (!dev)                return MCP_EPARAM;
    if (dev->base.fd < 0)    return MCP_FAIL(dev, MCP_ESTATE);
    if (reg > MCP_REG_MAX)   return MCP_FAIL(dev, MCP_EPARAM);
    if (pin > MCP_PIN_MAX)   return MCP_FAIL(dev, MCP_EPARAM);

    int16_t r = mcp23s08_read(dev, reg);
    if (r < 0) return (mcp_err_t)r;

    return ((uint8_t)r >> pin) & 0x1;
}

mcp_err_t mcp23s08_interrupt(mcp23s08_t *dev, uint8_t enable, uint8_t bitmask, uint8_t interrupt_mode) {

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

mcp_err_t mcp23s08_led(mcp23s08_t *dev, uint8_t enable) {

    if (!dev)                    return MCP_EPARAM;
    if (dev->base.fd < 0)        return MCP_FAIL(dev, MCP_ESTATE);

    if (enable != MCP_LED_ENABLE && enable != MCP_LED_DISABLE)
                                  return MCP_FAIL(dev, MCP_EPARAM);

    // Set all pins as outputs and turn off all LEDs
    mcp_err_t rc = mcp23s08_write(dev, MCP_IODIR, 0xFF); // reset
    if (rc < 0) return rc;

    rc = mcp23s08_write(dev, MCP_IODIR, 0x00); // IODIR: all output
    if (rc < 0) return rc;

    rc = mcp23s08_write(dev, MCP_OLAT, 0xFF); // OLAT: all high (LEDs off)
    if (rc < 0) return rc;

    if (enable == MCP_LED_ENABLE) {
        rc = mcp23s08_write(dev, MCP_OLAT, 0x00);
        if (rc < 0) return rc;
        return MCP_OK;
    }
    if (enable == MCP_LED_DISABLE) {
        rc = mcp23s08_write(dev, MCP_OLAT, 0xFF);
        if (rc < 0) return rc;
        return MCP_OK;
    }

    return MCP_OK;
}