/*
 * @file mcp23s09_test_read.c
 * @brief Test program for reading from MCP23S09 GPIO expander
 *
 * This program initializes the MCP23S09 device and continuously reads
 * the GPIO register and individual pin states, printing the results to
 * the console.
 *
 * @author Hauke Schlosser
 * @date 10/17/2025
**/

#include <unistd.h> 
#include <stdio.h>

#include <../src/lib_mcp23s09.h>

int main() {

    mcp23s09_t dev;

    int fd = mcp23s09_init(&dev, 0, 0, SPI_MODE_0, MCP_09S_SPEED_MAX);
    if (fd < 0) {
        fprintf(stderr, "init failed\n");
        return 1;
    }

    if (mcp23s09_write(&dev, MCP_09S_IPOL, 0x00) < 0) return 1;
    if (mcp23s09_write(&dev, MCP_09S_IOCON, 0x00) < 0) return 1;
    if (mcp23s09_write(&dev, MCP_09S_IODIR, 0xFF) < 0) return 1;
    if (mcp23s09_write(&dev, MCP_09S_GPPU,  0xFF) < 0) return 1;

    printf("[mcp23s09_test_read] Run test program...\n");
    printf("[mcp23s09_test_read] Press crtl + c to stop...\n");

    for (;;) {
        printf("[mcp23s09_test_read] Reading all GPIOs...\n");
        int v = mcp23s09_read(&dev, MCP_09S_GPIO);
        if (v < 0) return 1;
        printf("[mcp23s09_test_read] GPIO = 0x%02X\n", v);
        sleep(1);

        printf("[mcp23s09_test_read] Reading pin 1...\n");
        int p0 = mcp23s09_read_pin(&dev, MCP_09S_GPIO, 1);
        if (p0 < 0) return 1;
        printf("[mcp23s09_test_read] Pin 1 = %d\n", p0);
        sleep(1);
    }

    return 0;

}