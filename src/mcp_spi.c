#include "../includes/mcp_spi.h"
#include "../includes/defines.h"

#include <string.h>
#include <sys/ioctl.h>

int mcp23_spi_transfer(int fd, uint8_t *tx_buf, uint8_t *rx_buf, unsigned int len) {

    if (fd < 0)             return MCP_EPARAM;
    if (!tx_buf)            return MCP_EPARAM;
    if (!rx_buf)            return MCP_EPARAM;
    if (len == 0)           return MCP_EPARAM;
    if (len > UINT32_MAX)   return MCP_ECONFIG;

    struct spi_ioc_transfer spi;
    memset(&spi, 0, sizeof(spi));
    spi.tx_buf          = (uintptr_t) tx_buf;
    spi.rx_buf          = (uintptr_t) rx_buf;
    spi.len             = (__u32)len;
    spi.bits_per_word   = MCP_BPW;
    spi.delay_usecs     = MCP_DELAY;

    if (ioctl(fd, SPI_IOC_MESSAGE(1), &spi) < 0) {
        int code = mcp_map_errno();
        return code;
    }

    return (int)len;
}