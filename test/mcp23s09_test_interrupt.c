#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <linux/gpio.h>

#include "../src/lib_mcp23s09.h"

#define CHIPDEV   "/dev/gpiochip0"
#define INT_LINE  25

int main() {

    mcp23s09_t dev;
    int fd = mcp23s09_init(&dev, 0, 0, SPI_MODE_0, MCP_09S_SPEED_MAX);
    if (fd < 0) return 1;

    uint8_t bitmask = 0xFF;
    if (mcp23s09_interrupt(&dev, MCP_09S_INT_ENABLE, bitmask, MCP_09S_CHANGE_ANY) < 0) return 1;
    if (mcp23s09_read(&dev, MCP_09S_INTCAP) < 0) return 1;

    int fd_chip = open(CHIPDEV, O_RDONLY);
    if (fd_chip < 0) return 1; 

    struct gpioevent_request req;
    memset(&req, 0, sizeof(req));
    req.lineoffset   = INT_LINE;
    req.handleflags  = GPIOHANDLE_REQUEST_INPUT;
    req.eventflags   = GPIOEVENT_REQUEST_FALLING_EDGE;
    strncpy(req.consumer_label, "mcp23s09-int", sizeof(req.consumer_label)-1);

    if (ioctl(fd_chip, GPIO_GET_LINEEVENT_IOCTL, &req) < 0) {
        close(fd_chip);
        close(fd);
        return 1;
    }
    close(fd_chip);

    printf("[mcp23s09_test_interrupt] Run test program...\n");
    printf("[mcp23s09_test_interrupt] Press crtl + c to stop...\n");

    while(1) {
        struct gpioevent_data ev;
        ssize_t n = read(req.fd, &ev, sizeof(ev));
        if (n != (ssize_t)sizeof(ev)) {
            if (n < 0) perror("read event");
            else fprintf(stderr, "short read on event fd\n");
            break;
        }

        int16_t inf = mcp23s09_read(&dev, MCP_09S_INTF);
        int16_t cap = mcp23s09_read(&dev, MCP_09S_INTCAP);
        if (inf >= 0 && cap >= 0) {
            uint8_t f = (uint8_t)inf, c = (uint8_t)cap;
            if (!f) {
                printf("[mcp23s09_test_interrupt] No flags (INTF=%02X INTCAP=%02X)\n", f, c);
                continue;
            }
            while (f) {
                int pin = __builtin_ctz(f);
                printf("[mcp23s09_test_interrupt]  pin=%d state=%d\n", pin, (c >> pin) & 1);
                f &= (f - 1);
            }
        }
    }

    close(req.fd);
    close(fd);
    return 0;

}