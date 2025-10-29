#ifndef MCPSPI_H
#define MCPSPI_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <linux/spi/spidev.h>

/*
 * @brief Performs an SPI transaction using SPI ioctl interface.
 *
 * @param dev       File descriptor of the SPI device.
 * @param tx_buf    Pointer to the transmit buffer (data to send).
 * @param rx_buf    Pointer to the receive buffer (data received).
 * @param len       Length of the data to be transmitted/received (in bytes).
 *
 * @return int      Returns 0 on success, or -1 if the ioctl call fails.
 */
int mcp23_spi_transfer(int fd, uint8_t *tx_buf, uint8_t *rx_buf, unsigned int len);

#ifdef __cplusplus
}
#endif

#endif /* MCPSPI_H */