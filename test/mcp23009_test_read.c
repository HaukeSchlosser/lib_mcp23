#include <unistd.h> 
#include <stdio.h>

#include <../src/lib_mcp23009.h>

int main() {

    mcp23009_t dev;

    uint8_t bus = 1;
    uint8_t addr = 0;
    int fd = mcp23009_init(&dev, bus, MCP_009_SPEED_FAST, addr);
    if (fd < 0) {
        fprintf(stderr, "init failed\n");
        return 1;
    }

    if (mcp23009_write(&dev, MCP_009_IODIR, 0xFF) < 0) { // IODIR: all output
        fprintf(stderr, "write IODIR failed\n");
        return 1;
    }

    if (mcp23009_write(&dev, MCP_009_GPPU, 0xFF) < 0) {  // GPPU: all pull-ups enabled
        fprintf(stderr, "write GPPU failed\n");
        return 1;
    }

    for (;;) {

        int16_t input = mcp23009_read(&dev, MCP_009_GPIO);
        if (input < 0) {
            fprintf(stderr, "read GPIO failed\n");
            return 1;
        }
        printf("GPIO: 0x%02X\n", (uint8_t)input);
        usleep(500000);

        int8_t pin0 = mcp23009_read_pin(&dev, MCP_009_GPIO, 0);
        if (pin0 < 0) {
            fprintf(stderr, "read_pin GPIO pin 0 failed\n");
            return 1;
        }
        printf("GPIO pin 0: %d\n", pin0);
        usleep(500000);
    }

    return 0;
}