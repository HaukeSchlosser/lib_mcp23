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
static int set_iocon_for_interrupt(mcp23009_t *dev) {

    if (!dev) {
        fprintf(stderr, "[mcp23s09::set_iocon_for_interrupt] ERROR: Invalid device handle\n");
        return -1;
    }

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
static int set_pins_input_and_pullups(mcp23009_t *dev, uint8_t bitmask) {

    if (!dev) {
        fprintf(stderr, "[mcp23009::set_pins_input_and_pullups] ERROR: Invalid device handle\n");
        return -1;
    }

    int16_t r = mcp23009_read(dev, MCP_IODIR);
    if (r < 0) {
        fprintf(stderr, "[mcp23009::set_pins_input_and_pullups] ERROR: Failed to read IODIR register\n");
        return -1;
    }

    uint8_t iodir = (uint8_t)r | bitmask;
    if (mcp23009_write(dev, MCP_IODIR, iodir) < 0) {
        fprintf(stderr, "[mcp23009::set_pins_input_and_pullups] ERROR: Failed to write IODIR register\n");
        return -1;
    }

    r = mcp23009_read(dev, MCP_GPPU);
    if (r < 0) {
        fprintf(stderr, "[mcp23009::set_pins_input_and_pullups] ERROR: Failed to read GPPU register\n");
        return -1;
    }

    uint8_t gppu = (uint8_t)r | bitmask;
    if (mcp23009_write(dev, MCP_GPPU, gppu) < 0) {
        return -1;
    }

    return 0;
}

/*
 * @brief Helper function to configure INTCON and DEFVAL registers for interrupts.
 */
static int configure_interrupt_mode(mcp23009_t *dev, uint8_t bitmask, uint8_t interrupt_mode) {

    if (!dev) {
        fprintf(stderr, "[mcp23009::configure_interrupt_mode] ERROR: Invalid device handle\n");
        return -1;
    }

    int16_t r = mcp23009_read(dev, MCP_INTCON);
    if (r < 0) {
        fprintf(stderr, "[mcp23009::configure_interrupt_mode] ERROR: Failed to read INTCON register\n");
        return -1;
    }

    uint8_t intcon = (uint8_t)r;
    if (interrupt_mode == MCP_CHANGE_ANY) {
        intcon &= ~bitmask;
        if (mcp23009_write(dev, MCP_INTCON, intcon) < 0) {
            fprintf(stderr, "[mcp23009::configure_interrupt_mode] ERROR: Failed to write INTCON register\n");
            return -1;
        }
    } else {
        intcon |= bitmask;
        if (mcp23009_write(dev, MCP_INTCON, intcon) < 0) {
            fprintf(stderr, "[mcp23009::configure_interrupt_mode] ERROR: Failed to write INTCON register\n");
            return -1;
        }

        r = mcp23009_read(dev, MCP_DEFVAL);
        if (r < 0) {
            fprintf(stderr, "[mcp23009::configure_interrupt_mode] ERROR: Failed to read DEFVAL register\n");
            return -1;
        }

        uint8_t defval = (uint8_t)r;
        defval = (uint8_t)((defval & ~bitmask) | bitmask);
        if (mcp23009_write(dev, MCP_DEFVAL, defval) < 0) {
            fprintf(stderr, "[mcp23009::configure_interrupt_mode] ERROR: Failed to write DEFVAL register\n");
            return -1;
        }
    }

    return 0;
}

/*
 * @brief Helper function to update GPINTEN register for enabling/disabling interrupts.
 */
static int update_gpinten(mcp23009_t *dev, uint8_t bitmask, uint8_t enable) {

    if (!dev) {
        fprintf(stderr, "[mcp23009::update_gpinten] ERROR: Invalid device handle\n");
        return -1;
    }

    int16_t r = mcp23009_read(dev, MCP_GPINTEN);
    if (r < 0) {
        fprintf(stderr, "[mcp23009::update_gpinten] ERROR: Failed to read GPINTEN register\n");
        return -1;
    }

    uint8_t gpinten = (uint8_t)r;
    if (enable == MCP_INT_ENABLE) {
        gpinten |= bitmask;
    } else {
        gpinten &= (uint8_t)~bitmask;
    }

    return mcp23009_write(dev, MCP_GPINTEN, gpinten);
}

/*
 * @brief Helper function to clear pending interrupts by reading INTCAP register.
 */
static int clear_pending_int(mcp23009_t *dev) {

    if (!dev) {
        fprintf(stderr, "[mcp23009::clear_pending_int] ERROR: Invalid device handle\n");
        return -1;
    }

    return mcp23009_read(dev, MCP_INTCAP);
}

int mcp23009_init(mcp23009_t *dev, uint8_t bus, uint32_t speed_hz, uint8_t address) {
    int fd;

    if (!dev) {
        fprintf(stderr, "[mcp23::mcp23009_init] ERROR Invalid device handle\n");
        return -1;
    }

    if (address > MCP_009_ADDR_MAX) {
        fprintf(stderr, "[mcp23::mcp23009_init] ERROR Invalid address input: 0x%02X\n", address);
        fprintf(stderr, "[mcp23::mcp23009_init] ERROR Expected range: 0x00 - 0x%02X\n", MCP_009_ADDR_MAX);
        return -1;
    }

    if (speed_hz < MCP_I2C_SPEED_SLOW) {
        printf("[mcp23::mcp23009_init] INFO: SPI speed too low, setting to minimum %u Hz\n", MCP_I2C_SPEED_SLOW);
        speed_hz = MCP_I2C_SPEED_SLOW;
    }
    if (speed_hz > MCP_I2C_SPEED_FAST) {
        printf("[mcp23::mcp23009_init] INFO: SPI speed too high, setting to maximum %u Hz\n", MCP_I2C_SPEED_FAST);
        speed_hz = MCP_I2C_SPEED_FAST;
    }

    char bus_name[16];
    int n = snprintf(bus_name, sizeof bus_name, "/dev/i2c-%u", (unsigned)bus);
    if (n < 0 || n >= (int)sizeof bus_name) {
        fprintf(stderr, "[mcp23::mcp23009_init] ERROR Constructing bus name\n");
        return -1;
    }

    if ((fd = open(bus_name, O_RDWR)) < 0) {
        fprintf(stderr, "[mcp23::mcp23009_init] ERROR Could not open I2C device: %s\n", bus_name);
        perror("System Message");
        return -1;
    }

    uint8_t i2c_addr = (uint8_t)(MCP_009_BASE | (address & MCP_009_ADDR_MAX));
    if (ioctl(fd, I2C_SLAVE, i2c_addr) <0){
        fprintf(stderr, "[mcp23::mcp23009_init] ERROR Could not select I2C slave 0x%02X on %s\n", 
            i2c_addr, bus_name);
        perror("I2C_SLAVE");
        close(fd);
        return -1;
    }

    memset(dev, 0, sizeof *dev);
    dev->speed_hz = speed_hz;
    dev->address = i2c_addr;

    dev->fd = fd;
    return fd;
}

int8_t mcp23009_write(mcp23009_t *dev, uint8_t reg, uint8_t data) {

    if (!dev) {
        fprintf(stderr, "[mcp23::mcp23009_write] ERROR Invalid device handle\n");
        return -1;
    }

    if (dev->fd < 0) {
        fprintf(stderr, "[mcp23::mcp23009_write] ERROR Invalid file descriptor\n");
        return -1;
    }

    if (reg > MCP_REG_MAX) {
        fprintf(stderr, "[mcp23::mcp23009_write] ERROR Invalid register address: 0x%02X\n", reg);
        fprintf(stderr, "[mcp23::mcp23009_write] ERROR Expected range: 0x00 - 0x%02X\n", MCP_REG_MAX);
        return -1;
    }

    uint8_t buf[2] = {reg, data};
    ssize_t written = write(dev->fd, buf, 2);
    if (written != 2) {
        fprintf(stderr, "[mcp23::mcp23009_write] ERROR Writing to register 0x%02X\n", reg);
        perror(NULL);
        return -1;
    }

    return 0;

}

int16_t mcp23009_read(mcp23009_t *dev, uint8_t reg) {

    if (!dev) {
        fprintf(stderr, "[mcp23009_read] ERROR Invalid device handle\n");
        return -1;
    }

    if (dev->fd < 0) {
        fprintf(stderr, "[mcp23009_read] ERROR Invalid file descriptor\n");
        return -1;
    }

    if (reg > MCP_REG_MAX) {
        fprintf(stderr, "[mcp23009_read] ERROR Invalid register address: 0x%02X\n", reg);
        fprintf(stderr, "[mcp23009_read] ERROR Expected range: 0x00 - 0x%02X\n", MCP_REG_MAX);
        return -1;
    }

    uint8_t data = 0;
    ssize_t n;

    n = write(dev->fd, &reg, 1);
    if (n != 1) { 
        char msg[64];
        snprintf(msg, sizeof msg, "[mcp23009_read] write reg=0x%02X", reg);
        perror(msg); 
        return -1;
    }

    n = read(dev->fd, &data, 1);
    if (n != 1) { 
        char msg[64];
        snprintf(msg, sizeof msg, "[mcp23009_read] read reg=0x%02X", reg);
        perror(msg);
        return -1;
    }

    return (int16_t)data;

}

int8_t mcp23009_write_pin(mcp23009_t *dev, uint8_t reg, uint8_t pin, uint8_t data) {

    if (!dev) {
        fprintf(stderr, "[mcp23009_write_pin] ERROR Invalid device handle\n");
        return -1;
    }

    if (pin > MCP_PIN_MAX) {
        fprintf(stderr, "[mcp23009_write_pin] ERROR Invalid pin: %u\n", pin);
        fprintf(stderr, "[mcp23009_write_pin] ERROR Expected range: 0x00 - %d\n", MCP_PIN_MAX);
        return -1;
    }

    int16_t reg_data = mcp23009_read(dev, reg);
    if (reg_data < 0) {
        fprintf(stderr, "[mcp23009_write_pin] ERROR: Failed to read register: 0x%02X\n", reg);
        return -1;
    }

    uint8_t reg_v = (uint8_t)reg_data;
    uint8_t bitmask = (uint8_t)(1 << pin);
    if (data) {
        reg_v |=  bitmask;
    } else {
        reg_v &= (uint8_t)~bitmask;
    }

    return mcp23009_write(dev, reg, reg_v);

}

int8_t mcp23009_read_pin(mcp23009_t *dev, uint8_t reg, uint8_t pin) {
    
    if (!dev) {
        fprintf(stderr, "[mcp23009_read_pin] ERROR Invalid device handle\n");
        return -1;
    }

    if (pin > MCP_PIN_MAX) {
        fprintf(stderr, "[mcp23009_read_pin] ERROR Invalid pin: %u\n", pin);
        fprintf(stderr, "[mcp23009_read_pin] ERROR Expected range: 0x00 - %d\n", MCP_PIN_MAX);
        return -1;
    }

    int16_t reg_data = mcp23009_read(dev, reg);
    if (reg_data < 0) {
        fprintf(stderr, "[mcp23009_read_pin] ERROR: Failed to read register: 0x%02X\n", reg);
        return -1;
    }

    uint8_t v = (uint8_t)reg_data;
    return (int8_t)((v >> pin) & 1u);

}

int8_t mcp23009_interrupt(mcp23009_t *dev, uint8_t enable, uint8_t bitmask, uint8_t interrupt_mode) {

    if (!dev) {
        fprintf(stderr, "[mcp23009_interrupt] ERROR Invalid device handle\n");
        return -1;
    }

    if (enable != MCP_INT_ENABLE && enable != MCP_INT_DISABLE) {
        fprintf(stderr, "[mcp23009_interrupt] ERROR Invalid enable value: %u\n", enable);
        fprintf(stderr, "[mcp23009_interrupt] ERROR Expected values: MCP_009_INT_ENABLE (%u) or MCP_009_INT_DISABLE (%u)\n",
            MCP_INT_ENABLE, MCP_INT_DISABLE);
        return -1;
    }

    if (interrupt_mode != MCP_CHANGE_ANY && interrupt_mode != MCP_COMPARE_DEFVAL) {
        fprintf(stderr, "[mcp23009_interrupt] ERROR Invalid interrupt mode value: %u\n", interrupt_mode);
        fprintf(stderr, "[mcp23009_interrupt] ERROR Expected values: MCP_009_CHANGE_ANY (%u) or MCP_009_COMPARE_DEFVAL (%u)\n",
            MCP_CHANGE_ANY, MCP_COMPARE_DEFVAL);
        return -1;
    }

    if (set_iocon_for_interrupt(dev) < 0) {
        fprintf(stderr, "[mcp23009_interrupt] ERROR Failed to write to IOCON register\n");
        return -1;
    }

    if (set_pins_input_and_pullups(dev, bitmask) < 0) {
        fprintf(stderr, "[mcp23009_interrupt] ERROR Failed to set bitmask\n");
        return -1;
    }

    if (configure_interrupt_mode(dev, bitmask, interrupt_mode) < 0) {
        fprintf(stderr, "[mcp23009_interrupt] ERROR Failed to configure interrupt mode\n");
        return -1;
    }

    if (update_gpinten(dev, bitmask, enable) < 0) {
        fprintf(stderr, "[mcp23009_interrupt] ERROR Failed to update GPINTEN register\n");
        return -1;
    }

    if (clear_pending_int(dev) < 0) {
        fprintf(stderr, "[mcp23009_interrupt] ERROR Failed to clear pending interrupts\n");
        return -1;
    }

    return 0;

}