/*
 * MCP23S09 Test Write
 *
 * @brief Test program for writing to an MCP23S09 GPIO expander via SPI.
 * 
 * This program initializes the MCP23S09, configures its registers for output,
 * and continuously toggles the GPIO pin states every second. The program runs
 * indefinitely until manually stopped (Ctrl + C).
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

    if (mcp23s09_write(&dev, MCP_09S_IOCON, 0x00) < 0) return 1;
    if (mcp23s09_write(&dev, MCP_09S_IODIR, 0x00) < 0) return 1;
    if (mcp23s09_write(&dev, MCP_09S_OLAT, 0xFF) < 0) return 1;

    printf("[mcp23s09_test_write] Run test program...\n");
    printf("[mcp23s09_test_write] Press crtl + c to stop...\n");

    for (;;) {
        printf("[mcp23s09_test_write] All Pins - OFF\n");
        if (mcp23s09_write(&dev, MCP_09S_OLAT, 0x00) < 0) return 1;
        sleep(1);
        printf("[mcp23s09_test_write] All Pins - ON\n"); 
        if (mcp23s09_write(&dev, MCP_09S_OLAT, 0xFF) < 0) return 1;
        sleep(1);
        printf("[mcp23s09_test_write] Pin GP1 - OFF\n");
        if (mcp23s09_write_pin(&dev, MCP_09S_OLAT, 1, 0) < 0) return 1;
        sleep(1);
        printf("[mcp23s09_test_write] Pin GP1 - ON\n");
        if (mcp23s09_write_pin(&dev, MCP_09S_OLAT, 1, 1) < 0) return 1;
        sleep(1);
    }

    return 0;

}