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

    if (mcp23009_write(&dev, MCP_009_IODIR, 0x00) < 0) { // IODIR: all output
        fprintf(stderr, "write IODIR failed\n");
        return 1;
    }

    if (mcp23009_write(&dev, MCP_009_OLAT, 0xFF) < 0) {  // OLAT: all high (LEDs off)
        fprintf(stderr, "write OLAT failed\n");
        return 1;
    }

    for (;;) {
        printf("Blinking all pins\n");
        if (mcp23009_write(&dev, MCP_009_OLAT, 0x00) < 0) {  // OLAT: all low (LEDs on)
            fprintf(stderr, "write OLAT failed\n");
            return 1;
        }
        usleep(500000);
        printf("Turning off all pins\n");
        if (mcp23009_write(&dev, MCP_009_OLAT, 0xFF) < 0) {  // OLAT: all high (LEDs off)
            fprintf(stderr, "write OLAT failed\n");
            return 1;
        }
        usleep(500000);

        printf("Blinking pin 0\n");
        if (mcp23009_write_pin(&dev, MCP_009_OLAT, 0, 0) < 0) { // OLAT: pin 0 low
            fprintf(stderr, "write_pin OLAT pin 0 failed\n");
            return 1;
        }
        usleep(500000);
        printf("Turning off pin 0\n");
        if (mcp23009_write_pin(&dev, MCP_009_OLAT, 0, 1) < 0) { // OLAT: pin 0 high
            fprintf(stderr, "write_pin OLAT pin 0 failed\n");
            return 1;
        }
        usleep(500000);
    }

    return 0;
}