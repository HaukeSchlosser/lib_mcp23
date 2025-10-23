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

    if (mcp23s08_write(&dev, MCP_08S_IOCON, 0x00) < 0) return 1;
    if (mcp23s08_write(&dev, MCP_08S_IODIR, 0x00) < 0) return 1;
    if (mcp23s08_write(&dev, MCP_08S_OLAT, 0xFF) < 0) return 1;

    printf("[mcp23s08_test_write] Run test program...\n");
    printf("[mcp23s08_test_write] Press crtl + c to stop...\n");

    for (;;) {
        printf("[mcp23s08_test_write] All Pins - OFF\n");
        if (mcp23s08_write(&dev, MCP_08S_OLAT, 0x00) < 0) return 1;
        sleep(1);
        printf("[mcp23s08_test_write] All Pins - ON\n"); 
        if (mcp23s08_write(&dev, MCP_08S_OLAT, 0xFF) < 0) return 1;
        sleep(1);
        printf("[mcp23s08_test_write] Pin GP1 - OFF\n");
        if (mcp23s08_write_pin(&dev, MCP_08S_OLAT, 1, 0) < 0) return 1;
        sleep(1);
        printf("[mcp23s08_test_write] Pin GP1 - ON\n");
        if (mcp23s08_write_pin(&dev, MCP_08S_OLAT, 1, 1) < 0) return 1;
        sleep(1);
    }

    return 0;
}