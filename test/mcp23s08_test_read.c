#include <unistd.h> 
#include <stdio.h>

#include <../src/lib_mcp23s08.h>

int main() {

    mcp23s08_t dev;

    uint8_t addr = 0x00;
    int fd = mcp23s08_init(&dev, 0, 0, SPI_MODE_0, MCP_08S_SPEED_MAX, addr, MCP_08S_HAEN_DISABLE);
    if (fd < 0) {
        fprintf(stderr, "init failed\n");
        return 1;
    }

    if (mcp23s08_write(&dev, MCP_08S_IPOL, 0x00) < 0) return 1;
    if (mcp23s08_write(&dev, MCP_08S_IOCON, 0x00) < 0) return 1;
    if (mcp23s08_write(&dev, MCP_08S_IODIR, 0xFF) < 0) return 1;
    if (mcp23s08_write(&dev, MCP_08S_GPPU,  0xFF) < 0) return 1;

    printf("[mcp23s08_test_read] Run test program...\n");
    printf("[mcp23s08_test_read] Press crtl + c to stop...\n");

    for (;;) {
        printf("[mcp23s08_test_read] Reading all GPIOs...\n");
        int v = mcp23s08_read(&dev, MCP_08S_GPIO);
        if (v < 0) return 1;
        printf("[mcp23s08_test_read] GPIO = 0x%02X\n", v);
        sleep(1);

        printf("[mcp23s08_test_read] Reading pin 1...\n");
        int p0 = mcp23s08_read_pin(&dev, MCP_08S_GPIO, 1);
        if (p0 < 0) return 1;
        printf("[mcp23s08_test_read] Pin 1 = %d\n", p0);
        sleep(1);
    }

    return 0;

}