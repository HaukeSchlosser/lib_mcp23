#include "../includes/mcp23.h"
#include "../includes/mcp23s08.h"
#include "../includes/mcp23s09.h"
#include "../includes/mcp23009.h"

#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h> 
#include <sys/ioctl.h>
#include <string.h>

mcp_err_t mcp_init(mcp_dev_t *dev, const void *cfg_i) {

    if (!dev)   return MCP_EPARAM;
    if (!cfg_i) MCP_FAIL(dev, MCP_EPARAM);

    const mcp_cfg_t *cfg = (const mcp_cfg_t*)cfg_i;
    memset(&dev->u, 0, sizeof(dev->u));
    dev->base.variant = cfg->variant;

    switch (cfg->variant) {
        case MCP_VARIANT_23S08: {
            dev->u.s08 = malloc(sizeof *dev->u.s08);
            if (!dev->u.s08) return MCP_FAIL(dev, MCP_ENOMEM);
            int fd = mcp23s08_init(dev->u.s08,
                cfg->u.spi.bus, cfg->u.spi.cs, cfg->u.spi.mode,
                cfg->u.spi.speed_hz, cfg->u.spi.address, cfg->u.spi.haen_enabled);
            if (fd < 0) {
                return MCP_FAIL(dev, mcp_map_errno());
                free(dev->u.s08); 
                dev->u.s08 = NULL; 
            }
            return MCP_OK;
        }
        case MCP_VARIANT_23S09: {
            dev->u.s09 = malloc(sizeof *dev->u.s09);
            if (!dev->u.s09) return MCP_FAIL(dev, MCP_ENOMEM);
            int fd = mcp23s09_init(dev->u.s09,
                cfg->u.spi.bus, cfg->u.spi.cs, cfg->u.spi.mode,
                cfg->u.spi.speed_hz);
            if (fd < 0) {
                return MCP_FAIL(dev, mcp_map_errno());
                free(dev->u.s09); 
                dev->u.s09 = NULL; 
            }
            return MCP_OK;
        }
        case MCP_VARIANT_23009: {
            dev->u.i09 = malloc(sizeof *dev->u.i09);
            if (!dev->u.i09) return MCP_FAIL(dev, MCP_ENOMEM);
            int fd = mcp23009_init(dev->u.i09,
                cfg->u.i2c.bus, cfg->u.i2c.speed_hz, cfg->u.i2c.address);
            if (fd < 0) {
                return MCP_FAIL(dev, mcp_map_errno());
                free(dev->u.i09); 
                dev->u.i09 = NULL; 
            }
            return MCP_OK;
        }
        default: return MCP_FAIL(dev, MCP_ENOTSUP);
    }
}

mcp_err_t mcp_write(mcp_dev_t *dev, uint8_t reg, uint8_t data) {

    if (!dev) return MCP_EPARAM;

    switch (dev->base.variant) {
        case MCP_VARIANT_23S08:
            return mcp23s08_write(dev->u.s08, reg, data);
        case MCP_VARIANT_23S09:
            return mcp23s09_write(dev->u.s09, reg, data);
        case MCP_VARIANT_23009:
            return mcp23009_write(dev->u.i09, reg, data);
        default: return MCP_FAIL(dev, MCP_ENOTSUP);
    }
}

int16_t mcp_read(mcp_dev_t *dev, uint8_t reg) {

    if (!dev) return MCP_EPARAM;

    switch (dev->base.variant) {
        case MCP_VARIANT_23S08:
            return mcp23s08_read(dev->u.s08, reg);
        case MCP_VARIANT_23S09:
            return mcp23s09_read(dev->u.s09, reg);
        case MCP_VARIANT_23009:
            return mcp23009_read(dev->u.i09, reg);
        default: return MCP_FAIL(dev, MCP_ENOTSUP);
    }
}

mcp_err_t mcp_write_pin(mcp_dev_t *dev, uint8_t reg, uint8_t pin, uint8_t data) {
    
    if (!dev) return MCP_EPARAM;

    switch (dev->base.variant) {
        case MCP_VARIANT_23S08:
            return mcp23s08_write_pin(dev->u.s08, reg, pin, data);
        case MCP_VARIANT_23S09:
            return mcp23s09_write_pin(dev->u.s09, reg, pin, data);
        case MCP_VARIANT_23009:
            return mcp23009_write_pin(dev->u.i09, reg, pin, data);
        default: return MCP_FAIL(dev, MCP_ENOTSUP);
    }
}

int8_t mcp_read_pin(mcp_dev_t *dev, uint8_t reg, uint8_t pin) {
    
    if (!dev) return MCP_EPARAM;

    switch (dev->base.variant) {
        case MCP_VARIANT_23S08:
            return mcp23s08_read_pin(dev->u.s08, reg, pin);
        case MCP_VARIANT_23S09:
            return mcp23s09_read_pin(dev->u.s09, reg, pin);
        case MCP_VARIANT_23009:
            return mcp23009_read_pin(dev->u.i09, reg, pin);
        default: return MCP_FAIL(dev, MCP_ENOTSUP);
    }
}

mcp_err_t mcp_interrupt(mcp_dev_t *dev, uint8_t enable, uint8_t bitmask, uint8_t interrupt_mode) {
    
    if (!dev) return MCP_EPARAM;

    switch (dev->base.variant) {
        case MCP_VARIANT_23S08:
            return mcp23s08_interrupt(dev->u.s08, enable, bitmask, interrupt_mode);
        case MCP_VARIANT_23S09:
            return mcp23s09_interrupt(dev->u.s09, enable, bitmask, interrupt_mode);
        case MCP_VARIANT_23009:
            return mcp23009_interrupt(dev->u.i09, enable, bitmask, interrupt_mode);
        default: return MCP_FAIL(dev, MCP_ENOTSUP);
    }
}

mcp_err_t mcp_led(mcp_dev_t *dev, uint8_t enable) {
    
    if (!dev) return MCP_EPARAM;

    switch (dev->base.variant) {
        case MCP_VARIANT_23S08:
            return mcp23s08_led(dev->u.s08, enable);
        case MCP_VARIANT_23S09:
            return mcp23s09_led(dev->u.s09, enable);
        case MCP_VARIANT_23009:
            return mcp23009_led(dev->u.i09, enable);
        default: return MCP_FAIL(dev, MCP_ENOTSUP);
    }
}

mcp_err_t mcp_close(mcp_dev_t *dev) {

    if (!dev) return MCP_EPARAM;

    switch (dev->base.variant) {
        case MCP_VARIANT_23S08:
            if (dev->u.s08) { 
                free(dev->u.s08); 
                dev->u.s08 = NULL; 
            }
            return MCP_OK;
        case MCP_VARIANT_23S09:
            if (dev->u.s09) { 
                free(dev->u.s09); 
                dev->u.s09 = NULL; 
            }
            return MCP_OK;
        case MCP_VARIANT_23009:
            if (dev->u.i09) {
                if (dev->u.i09->base.fd >= 0) {
                    close(dev->u.i09->base.fd);
                }
                free(dev->u.i09);
                dev->u.i09 = NULL;
            }
            return MCP_OK;
        default: return MCP_FAIL(dev, MCP_ENOTSUP);
    }
}

const mcp_error_t* mcp_get_error(const mcp_dev_t *dev) {

    if (!dev) return NULL;
    return &dev->base.last_error;

}

uint8_t mcp_build_bitmask(const unsigned *pins, unsigned n) {

    if (!pins || n == 0) return 0;

    uint8_t m = 0;
    for (unsigned i = 0; i < n; ++i) {
        if (pins[i] < 8) m |= (uint8_t)(1u << pins[i]);
    }
    return m;
}