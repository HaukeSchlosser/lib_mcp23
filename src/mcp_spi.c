#include "../includes/mcp_spi.h"

#include "../includes/defines.h"
#include <stdint.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <sys/ioctl.h>

int mcp23_spi_transfer(int fd, uint8_t *tx_buf, uint8_t *rx_buf, unsigned int len) {

    if (fd < 0) {
        fprintf(stderr, "[mcp23::mcp23_spi_transfer] ERROR: Invalid file descriptor: %d\n", fd);
        return -1;
    }

    if (!tx_buf) {
        fprintf(stderr, "[mcp23::mcp23_spi_transfer] ERROR: Invalid tx_buf pointer\n");
        return -1;
    }

    if (!rx_buf) {
        fprintf(stderr, "[mcp23::mcp23_spi_transfer] ERROR: Invalid rx_buf pointer\n");
        return -1;
    }

    if (len == 0) {
        fprintf(stderr, "[mcp23::mcp23_spi_transfer] ERROR: Invalid length: %u\n", len);
        return -1;
    }

    if (len > UINT32_MAX) {
        fprintf(stderr, "[mcp23::mcp23_spi_transfer] ERROR: len too large for spi_ioc_transfer (%u)\n", len);
        return -1;
    }

    struct spi_ioc_transfer spi;
    memset(&spi, 0, sizeof(spi));
    spi.tx_buf = (uintptr_t) tx_buf;
    spi.rx_buf = (uintptr_t) rx_buf;
    spi.len = len;
    spi.bits_per_word = MCP_BPW;
    spi.delay_usecs = MCP_DELAY;

    return ioctl(fd, SPI_IOC_MESSAGE(1), &spi);
}