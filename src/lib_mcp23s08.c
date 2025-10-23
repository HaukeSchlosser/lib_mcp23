#include <stdint.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <string.h>

#include "lib_mcp23s08.h"

static int8_t get_control_byte(mcp23s08_t *dev, uint8_t cmd) {

    if (!dev) {
        fprintf(stderr, "[mcp23s08::get_control_byte] ERROR: Invalid device handle\n");
        return -1;
    }

    if (dev->address > MCP_08S_ADDR_MAX) {
        fprintf(stderr, "[mcp23s08::get_control_byte] ERROR: Invalid address: 0x%02X (valid 0x00–0x%02X)\n",
                dev->address, MCP_08S_ADDR_MAX);
        return -1;
    }

    if (dev->haen_enabled != MCP_08S_HAEN_ENABLE && dev->address != 0x00) {
        fprintf(stderr, "[mcp23s08::get_control_byte] WARNING: HAEN is disabled; address bits are ignored by the device\n");
    }

    if (cmd != MCP_08S_WRITE_CMD && cmd != MCP_08S_READ_CMD) {
        fprintf(stderr, "[mcp23s08::get_control_byte] ERROR: Invalid cmd: 0x%02X (use 0x%02X=WRITE, 0x%02X=READ)\n",
                cmd, MCP_08S_WRITE_CMD, MCP_08S_READ_CMD);
        return -1;
    }

    return (MCP_08S_BASE | ((dev->address & MCP_08S_ADDR_MAX) << 1) | (cmd & 0x01));
}

/*
 * @brief Performs an SPI transaction using SPI ioctl interface.
 *
 * @param dev       Pointer to the MCP23S08 device handle.
 * @param tx_buf    Pointer to the transmit buffer (data to send).
 * @param rx_buf    Pointer to the receive buffer (data received).
 * @param len       Length of the data to be transmitted/received (in bytes).
 *
 * @return int      Returns 0 on success, or -1 if the ioctl call fails.
 */
static int spi_transfer(mcp23s08_t *dev, uint8_t *tx_buf, uint8_t *rx_buf, unsigned int len) {

    if (!dev) {
        fprintf(stderr, "[mcp23s08::spi_transfer] ERROR: Invalid device handle\n");
        return -1;
    }

    if (dev->fd < 0) {
        fprintf(stderr, "[mcp23s08::spi_transfer] ERROR: Invalid file descriptor: %d\n", dev->fd);
        return -1;
    }

    if (len == 0) {
        fprintf(stderr, "[mcp23s08::spi_transfer] ERROR: Invalid length: %u\n", len);
        return -1;
    }

    if (len > UINT32_MAX) {
        fprintf(stderr, "[mcp23s08::spi_transfer] ERROR: len too large for spi_ioc_transfer (%u)\n", len);
        return -1;
    }

    struct spi_ioc_transfer spi;
    memset(&spi, 0, sizeof(spi));
    spi.tx_buf = (uintptr_t) tx_buf;
    spi.rx_buf = (uintptr_t) rx_buf;
    spi.len = len;
    spi.bits_per_word = MCP_08S_BPW;
    spi.delay_usecs = MCP_08S_DELAY;

    return ioctl(dev->fd, SPI_IOC_MESSAGE(1), &spi);
}

/*
 * @brief Helper function to set IOCON register for interrupt configuration.
 */
static int set_iocon_for_interrupt(mcp23s08_t *dev) {

    if (!dev) {
        fprintf(stderr, "[mcp23s08::set_iocon_for_interrupt] ERROR: Invalid device handle\n");
        return -1;
    }

    uint8_t iocon = 0;
    iocon |= (0 << 5);
    iocon |= (0 << 4);
    iocon |= (dev->haen_enabled << 3);
    iocon |= (1 << 2);
    iocon |= (0 << 1);

    return mcp23s08_write(dev, MCP_08S_IOCON, iocon);
}

/*
* @brief Helper function to set specified pins as input and enable pull-ups.
*/
static int set_pins_input_and_pullups(mcp23s08_t *dev, uint8_t bitmask) {

    if (!dev) {
        fprintf(stderr, "[mcp23s08::set_pins_input_and_pullups] ERROR: Invalid device handle\n");
        return -1;
    }

    int16_t r = mcp23s08_read(dev, MCP_08S_IODIR);
    if (r < 0) {
        fprintf(stderr, "[mcp23s08::set_pins_input_and_pullups] ERROR: Failed to read IODIR register\n");
        return -1;
    }

    uint8_t iodir = (uint8_t)r | bitmask;
    if (mcp23s08_write(dev, MCP_08S_IODIR, iodir) < 0) {
        fprintf(stderr, "[mcp23s08::set_pins_input_and_pullups] ERROR: Failed to write IODIR register\n");
        return -1;
    }

    r = mcp23s08_read(dev, MCP_08S_GPPU);
    if (r < 0) {
        fprintf(stderr, "[mcp23s08::set_pins_input_and_pullups] ERROR: Failed to read GPPU register\n");
        return -1;
    }

    uint8_t gppu = (uint8_t)r | bitmask;
    if (mcp23s08_write(dev, MCP_08S_GPPU, gppu) < 0) {
        return -1;
    }

    return 0;
}

/*
 * @brief Helper function to configure INTCON and DEFVAL registers for interrupts.
 */
static int configure_interrupt_mode(mcp23s08_t *dev, uint8_t bitmask, uint8_t interrupt_mode) {

    if (!dev) {
        fprintf(stderr, "[mcp23s08::configure_interrupt_mode] ERROR: Invalid device handle\n");
        return -1;
    }

    int16_t r = mcp23s08_read(dev, MCP_08S_INTCON);
    if (r < 0) {
        fprintf(stderr, "[mcp23s08::configure_interrupt_mode] ERROR: Failed to read INTCON register\n");
        return -1;
    }

    uint8_t intcon = (uint8_t)r;
    if (interrupt_mode == MCP_08S_CHANGE_ANY) {
        intcon &= ~bitmask;
        if (mcp23s08_write(dev, MCP_08S_INTCON, intcon) < 0) {
            fprintf(stderr, "[mcp23s08::configure_interrupt_mode] ERROR: Failed to write INTCON register\n");
            return -1;
        }
    } else {
        intcon |= bitmask;
        if (mcp23s08_write(dev, MCP_08S_INTCON, intcon) < 0) {
            fprintf(stderr, "[mcp23s08::configure_interrupt_mode] ERROR: Failed to write INTCON register\n");
            return -1;
        }

        r = mcp23s08_read(dev, MCP_08S_DEFVAL);
        if (r < 0) {
            fprintf(stderr, "[mcp23s08::configure_interrupt_mode] ERROR: Failed to read DEFVAL register\n");
            return -1;
        }

        uint8_t defval = (uint8_t)r;
        defval = (uint8_t)((defval & ~bitmask) | bitmask);
        if (mcp23s08_write(dev, MCP_08S_DEFVAL, defval) < 0) {
            fprintf(stderr, "[mcp23s08::configure_interrupt_mode] ERROR: Failed to write DEFVAL register\n");
            return -1;
        }
    }

    return 0;
}

/*
 * @brief Helper function to update GPINTEN register for enabling/disabling interrupts.
 */
static int update_gpinten(mcp23s08_t *dev, uint8_t bitmask, uint8_t enable) {

    if (!dev) {
        fprintf(stderr, "[mcp23s08::update_gpinten] ERROR: Invalid device handle\n");
        return -1;
    }

    int16_t r = mcp23s08_read(dev, MCP_08S_GPINTEN);
    if (r < 0) {
        fprintf(stderr, "[mcp23s08::update_gpinten] ERROR: Failed to read GPINTEN register\n");
        return -1;
    }

    uint8_t gpinten = (uint8_t)r;
    if (enable == MCP_08S_INT_ENABLE) {
        gpinten |= bitmask;
    } else {
        gpinten &= (uint8_t)~bitmask;
    }

    return mcp23s08_write(dev, MCP_08S_GPINTEN, gpinten);
}

/*
 * @brief Helper function to clear pending interrupts by reading INTCAP register.
 */
static int clear_pending_int(mcp23s08_t *dev) {

    if (!dev) {
        fprintf(stderr, "[mcp23s08::clear_pending_int] ERROR: Invalid device handle\n");
        return -1;
    }

    return mcp23s08_read(dev, MCP_08S_INTCAP);
}

int mcp23s08_init(mcp23s08_t *dev, uint8_t bus, uint8_t cs, 
                 uint8_t spi_mode, uint32_t speed_hz, uint8_t 
                 address, uint8_t haen_enabled) {
    int fd;

    if (!dev) {
        fprintf(stderr, "[mcp23s08_init] ERROR Invalid device handle\n");
        return -1;
    }

    if (bus > 1) {
        fprintf(stderr, "[mcp23s08_init] ERROR Invalid bus input: %d\n", bus);
        return -1;
    }

    if (cs > 1) {
        fprintf(stderr, "[mcp23s08_init] ERROR Invalid chip select input: %d\n", cs);
        return -1;
    }

    if (spi_mode != SPI_MODE_0 && spi_mode != SPI_MODE_3) {
        fprintf(stderr, "[mcp23s08_init] ERROR Invalid SPI mode input: %d\n", spi_mode);
        fprintf(stderr, "[mcp23s08_init] ERROR Expected values: SPI_MODE_0 (0x%02X) or SPI_MODE_3 (0x%02lX)\n",
                SPI_MODE_0, SPI_MODE_3);
        return -1;
    }

    if (address > MCP_08S_ADDR_MAX) {
        fprintf(stderr, "[mcp23s08_init] ERROR Invalid address input: 0x%02X\n", address);
        fprintf(stderr, "[mcp23s08_init] ERROR Expected range: 0x00 - 0x%02X\n", MCP_08S_ADDR_MAX);
        return -1;
    }

    if (haen_enabled != MCP_08S_HAEN_DISABLE && haen_enabled != MCP_08S_HAEN_ENABLE) {
        fprintf(stderr, "[mcp23s08_init] ERROR Invalid HAEN input: %d\n", haen_enabled);
        fprintf(stderr, "[mcp23s08_init] ERROR Expected values: MCP_08S_HAEN_DISABLE (0) or MCP_08S_HAEN_ENABLE (1)\n");
        return -1;
    }

    // clamp
    if (speed_hz < MCP_08S_SPEED_SLOW) {
        printf("[mcp23s08_init] INFO: SPI speed too low, setting to minimum %u Hz\n", MCP_08S_SPEED_SLOW);
        speed_hz = MCP_08S_SPEED_SLOW;
    }
    if (speed_hz > MCP_08S_SPEED_MAX) {
        printf("[mcp23s08_init] INFO: SPI speed too high, setting to maximum %u Hz\n", MCP_08S_SPEED_MAX);
        speed_hz = MCP_08S_SPEED_MAX;
    }

    static const char * spidev[2][2] = {
        {"/dev/spidev0.0", "/dev/spidev0.1"},
        {"/dev/spidev1.0", "/dev/spidev1.1"},
    };

    if ((fd = open(spidev[bus][cs], O_RDWR)) < 0) {
        fprintf(stderr, "[mcp23s08_init] ERROR Could not open SPI device: %s\n", spidev[bus][cs]);
        perror("System Message");
        return -1;
    }

    memset(dev, 0, sizeof *dev);
    dev->haen_enabled = haen_enabled;
    dev->speed_hz = speed_hz;
    dev->mode = spi_mode;
    dev->address = address;

    uint8_t req_mode8 = (spi_mode == SPI_MODE_3) ? (SPI_CPOL | SPI_CPHA) : 0u;
    if (ioctl(fd, SPI_IOC_WR_MODE, &req_mode8) < 0) {
        fprintf(stderr, "[mcp23s08_init] ERROR Could not set SPI_MODE: %u\n", spi_mode);
        perror("System Message");
        close(fd);
        return -1;
    }

    uint8_t apm = 0;
    if (ioctl(fd, SPI_IOC_RD_MODE, &apm) < 0) {
        fprintf(stderr, "[mcp23s08_init] ERROR Could not read SPI_MODE\n");
        perror("System Message");
        close(fd);
        return -1;
    }

    if (apm != req_mode8) {
        fprintf(stderr, "[mcp23s08_init] WARNING: SPI_MODE mismatch: requested=0x%02X, applied=0x%02X\n",
                req_mode8, apm);
        return -1;
    }

    uint8_t spi_bpw = MCP_08S_BPW;
    if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &spi_bpw) < 0) {
        fprintf(stderr, "[mcp23s08_init] ERROR Could not set SPI_BPW: %u\n", spi_bpw);
        perror("System Message");
	    close(fd);
        return -1;
    }

    uint8_t rb_bpw = 0;
    if (ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &rb_bpw) < 0) {
        fprintf(stderr, "[mcp23s08_init] ERROR Could not read SPI_BPW\n");
        perror("System Message");
        close(fd);
        return -1;
    }

    if (rb_bpw != spi_bpw) {
        fprintf(stderr, "[mcp23s08_init] ERROR: BPW mismatch: requested=%u, applied=%u\n",
                spi_bpw, rb_bpw);
        close(fd);
        return -1;
    }

    if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &dev->speed_hz) < 0) {
        fprintf(stderr, "[mcp23s08_init] ERROR Could not set SPI_SPEED: %u\n", dev->speed_hz);
        perror("System Message");
        close(fd);
        return -1;
    }

    uint32_t aps = 0;
    if (ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &aps) == 0 && aps) {
        if (aps != dev->speed_hz) {
            printf("[mcp23s08_init] INFO: driver applied %u Hz (requested %u Hz)\n", aps, dev->speed_hz);
        }
        dev->speed_hz = aps;
    }

    dev->fd = fd;
    return fd;
}

int8_t mcp23s08_write(mcp23s08_t *dev, uint8_t reg, uint8_t data) {

    if (!dev) {
        fprintf(stderr, "[mcp23s08_write] ERROR Invalid device handle\n");
        return -1;
    }

    if (reg > MCP_08S_REG_MAX) {
        fprintf(stderr, "[mcp23s08_write] ERROR Invalid register: 0x%02X\n", reg);
        fprintf(stderr, "[mcp23s08_write] ERROR Expected range: 0x00 - %d\n", MCP_08S_REG_MAX);
        return -1;
    }

    if (dev->address > MCP_08S_ADDR_MAX) {
        fprintf(stderr, "[mcp23s08_write] ERROR Invalid address input: 0x%02X\n", dev->address);
        fprintf(stderr, "[mcp23s08_write] ERROR Expected range: 0x00 - 0x%02X\n", MCP_08S_ADDR_MAX);
        return -1;
    }

    uint8_t ctr_byte = get_control_byte(dev, MCP_08S_WRITE_CMD);
    uint8_t tx_buf[3] = {ctr_byte, reg, data};
    if (spi_transfer(dev,  tx_buf, NULL, sizeof tx_buf) < 0) {
        fprintf(stderr, "[mcp23s08_write] ERROR: SPI transaction failed\n");
        perror("System Message");
        return -1;
    }

    return 0;
}

int16_t mcp23s08_read(mcp23s08_t *dev, uint8_t reg) {

    if (!dev) {
        fprintf(stderr, "[mcp23s08_read] ERROR Invalid device handle\n");
        return -1;
    }

    if (dev->address > MCP_08S_ADDR_MAX) {
        fprintf(stderr, "[mcp23s08_read] ERROR Invalid address input: 0x%02X\n", dev->address);
        fprintf(stderr, "[mcp23s08_read] ERROR Expected range: 0x00 - 0x%02X\n", MCP_08S_ADDR_MAX);
        return -1;
    }

    if (reg > MCP_08S_REG_MAX) {
        fprintf(stderr, "[mcp23s08_read] ERROR Invalid register: 0x%02X\n", reg);
        fprintf(stderr, "[mcp23s08_read] ERROR Expected range: 0x00 - %d\n", MCP_08S_REG_MAX);
        return -1;
    }

    uint8_t ctr_byte = get_control_byte(dev, MCP_08S_READ_CMD);
    uint8_t tx_buf[3] = {ctr_byte, reg, 0x00};
    uint8_t rx_buf[sizeof tx_buf];

    if (spi_transfer(dev, tx_buf, rx_buf, sizeof tx_buf) < 0) {
        fprintf(stderr, "[mcp23s08_read] ERROR: SPI transaction failed\n");
        perror("System Message");
        return -1;
    }

    return rx_buf[2];
}

int8_t mcp23s08_write_pin(mcp23s08_t *dev, uint8_t reg, uint8_t pin, uint8_t data) {

    if (!dev) {
        fprintf(stderr, "[mcp23s08_write_pin] ERROR Invalid device handle\n");
        return -1;
    }

    if (reg > MCP_08S_REG_MAX) {
        fprintf(stderr, "[mcp23s08_write_pin] ERROR Invalid register: 0x%02X\n", reg);
        fprintf(stderr, "[mcp23s08_write_pin] ERROR Expected range: 0x00 - %d\n", MCP_08S_REG_MAX);
        return -1;
    }

    if (pin > MCP_08S_PIN_MAX) {
        fprintf(stderr, "[mcp23s08_write_pin] ERROR Invalid pin: %u\n", pin);
        fprintf(stderr, "[mcp23s08_write_pin] ERROR Expected range: 0x00 - %d\n", MCP_08S_PIN_MAX);
        return -1;
    }

    int16_t reg_data = mcp23s08_read(dev, reg);
    if (reg_data < 0) {
        fprintf(stderr, "[mcp23s08_write_pin] ERROR: Failed to read register: 0x%02X\n", reg);
        return -1;
    }

    if (data) {
        reg_data |= 1 << pin;
    } else {
        reg_data &= ~(1 << pin);
    }

    return mcp23s08_write(dev, reg, reg_data);
}

int8_t mcp23s08_read_pin(mcp23s08_t *dev, uint8_t reg, uint8_t pin) {

    if (!dev) {
        fprintf(stderr, "[mcp23s08_read_pin] ERROR Invalid device handle\n");
        return -1;
    }

    if (reg > MCP_08S_REG_MAX) {
        fprintf(stderr, "[mcp23s08_read_pin] ERROR Invalid register: 0x%02X\n", reg);
        fprintf(stderr, "[mcp23s08_read_pin] ERROR Expected range: 0x00 - %d\n", MCP_08S_REG_MAX);
        return -1;
    }

    if (pin > MCP_08S_PIN_MAX) {
        fprintf(stderr, "[mcp23s08_read_pin] ERROR Invalid pin: %u\n", pin);
        fprintf(stderr, "[mcp23s08_read_pin] ERROR Expected range: 0x00 - %d\n", MCP_08S_PIN_MAX);
        return -1;
    }

    return ((mcp23s08_read(dev, reg) >> pin) & 1);
}

int8_t mcp23s08_interrupt(mcp23s08_t *dev, uint8_t enable, uint8_t bitmask, uint8_t interrupt_mode) {

    if (!dev) {
        fprintf(stderr, "[mcp23s08_interrupt] ERROR Invalid device handle\n");
        return -1;
    }

    if (enable != MCP_08S_INT_ENABLE && enable != MCP_08S_INT_DISABLE) {
        fprintf(stderr, "[mcp23s08_interrupt] ERROR Invalid enable value: %u\n", enable);
        fprintf(stderr, "[mcp23s08_interrupt] ERROR Expected values: INT_ENABLE (%d) or INT_DISABLE (%d)\n",
                MCP_08S_INT_ENABLE, MCP_08S_INT_DISABLE);
        return -1;
    }

    if (interrupt_mode != MCP_08S_CHANGE_ANY && interrupt_mode != MCP_08S_COMPARE_DEFVAL) {
        fprintf(stderr, "[mcp23s08_interrupt] ERROR Invalid interrupt_mode value: %u\n", interrupt_mode);
        fprintf(stderr, "[mcp23s08_interrupt] ERROR Expected values: CHANGE_ANY (%d) or COMPARE_DEFVAL (%d)\n",
                MCP_08S_CHANGE_ANY, MCP_08S_COMPARE_DEFVAL);
        return -1;
    }

    if (set_iocon_for_interrupt(dev) < 0) {
        fprintf(stderr, "[mcp23s08_interrupt] ERROR Failed to write to IOCON register\n");
        return -1;
    }

    if (set_pins_input_and_pullups(dev, bitmask) < 0) {
        fprintf(stderr, "[mcp23s08_interrupt] ERROR Failed to set bitmask\n");
        return -1;
    }

    if (configure_interrupt_mode(dev, bitmask, interrupt_mode) < 0) {
        fprintf(stderr, "[mcp23s08_interrupt] ERROR Failed to configure interrupt mode\n");
        return -1;
    }

    if (update_gpinten(dev, bitmask, enable) < 0) {
        fprintf(stderr, "[mcp23s08_interrupt] ERROR Failed to update GPINTEN register\n");
        return -1;
    }

    if (clear_pending_int(dev) < 0) {
        fprintf(stderr, "[mcp23s08_interrupt] ERROR Failed to clear pending interrupts\n");
        return -1;
    }

    return 0;
}