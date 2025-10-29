# MCP23 GPIO Expander Library

A C library for interfacing with Microchip's MCP23 series GPIO expanders on Linux systems. Supports MCP23S08 (SPI), MCP23S09 (SPI), and MCP23009 (I2C) devices.

## Features

- Unified API for different MCP23 variants
- Support for both SPI and I2C interfaces
- GPIO operations (read/write individual pins or full port)
- Interrupt handling capabilities
- Configurable bus speeds and addresses
- Thread-safe implementation

## Supported Devices

- **MCP23S08**: 8-bit SPI GPIO expander
- **MCP23S09**: 8-bit SPI GPIO expander
- **MCP23009**: 8-bit I2C GPIO expander

## Requirements  

### System  
- Linux kernel ≥ 5.4 (GPIO UAPI, spidev, i2c-dev supported)  
- Root or udev access to:
  - `/dev/spidevX.Y` (for SPI variants)  
  - `/dev/i2c-X` (for I²C variants)  
  - `/dev/gpiochipN` (for interrupt handling)  

### Build dependencies  
- `gcc` or `clang` (C11 compatible)  
- `make` (GNU Make 3.8+)  
- Linux kernel headers (for `<linux/spi/spidev.h>` and `<linux/gpio.h>`)  
- Standard C headers (`stdio.h`, `stdlib.h`, `string.h`, `unistd.h`, etc.)

## Installation

```bash
git clone https://github.com/HaukeSchlosser/lib_mcp23.git
cd lib_mcp23
make
sudo make install
```

The library will be installed to `/usr/local/lib` and headers to `/usr/local/include/mcp23`.

### Compilation

```bash
gcc -o your_program your_program.c -lmcp23
```

## Usage

### Including the Library

```c
#include <mcp23/mcp23.h>
```

### Basic Example

```c
#include <mcp23/mcp23.h>

// Initialize device configuration
mcp_cfg_t cfg = MCP_CFG_I2C_23009(1, 400000, 0x20); // I2C bus 1, 400kHz, addr 0x20
mcp_dev_t dev;

// Initialize device
if (mcp_init(&dev, &cfg) < 0) {
    fprintf(stderr, "Failed to initialize device\n");
    return 1;
}

// Write to GPIO port
mcp_write(&dev, MCP_GPIO, 0xFF);

// Read from GPIO port
int16_t value = mcp_read(&dev, MCP_GPIO);

// Cleanup
mcp_close(&dev);
```

### Configuration Macros

- `MCP_CFG_SPI_23S08(bus, cs, mode, speed, addr, haen)`
- `MCP_CFG_SPI_23S09(bus, cs, mode, speed)`
- `MCP_CFG_I2C_23009(bus, speed, addr)`

### API Functions

- `mcp_init()`: Initialize device
- `mcp_close()`: Close device connection
- `mcp_read()`: Read from register
- `mcp_write()`: Write to register
- `mcp_read_pin()`: Read single pin
- `mcp_write_pin()`: Write to single pin
- `mcp_interrupt()`: Configure interrupts

## Testing

The project includes a test program demonstrating basic functionality:

```bash
cd test
make
./build/mcp_test_read <variant> <operation>
```

Supported variants:
- mcp23s08
- mcp23s09
- mcp23009

Operations:
- read: Continuous GPIO read
- write: LED blinking demo
- interrupt: Interrupt handling demo

## Register Map

| Address | Register | Description |
|---------|----------|-------------|
| 0x00    | IODIR    | I/O Direction |
| 0x01    | IPOL     | Input Polarity |
| 0x02    | GPINTEN  | Interrupt-on-change |
| 0x03    | DEFVAL   | Default Compare |
| 0x04    | INTCON   | Interrupt Control |
| 0x05    | IOCON    | Configuration |
| 0x06    | GPPU     | Pull-up Resistor |
| 0x07    | INTF     | Interrupt Flag |
| 0x08    | INTCAP   | Interrupt Capture |
| 0x09    | GPIO     | Port Register |
| 0x0A    | OLAT     | Output Latch |

## License

This project is licensed under the GPL-3.0 License. 

## Contributing

Contributions are welcome! Please feel free to submit pull requests.

## Authors

Hauke Schlosser