# MCP23S0x SPI Library

A lightweight, high-performance, and robust C/C++ library for the **Microchip MCP23S08** and **MCP23S09** 8-bit GPIO expanders over **SPI**.

Designed for embedded systems, Raspberry Pi, and Linux-based microcontrollers — providing a clean abstraction layer and shared libraries for seamless integration.

---

## Features

- Supports **MCP23S08** and **MCP23S09** devices  
- Built as modular shared libraries (`libmcp23s08.so`, `libmcp23s09.so`)  
- Clean and minimal **C API** for easy integration into existing projects  
- Compatible with **C and C++** applications  
- Includes example test programs to verify SPI communication  
- Fully compatible with **Linux SPI driver (spidev)**

---

## Installation

#### 1. Clone the Repository

```bash
git clone https://github.com/HaukeSchlosser/lib_mcp23s0x.git
cd lib_mcp23s0x
```

#### 2. Build the Libraries
Builds the shared libraries under build/ and example test programs under test/out/:
```bash
make
```

#### 3. Install System-Wide
Installs headers and shared libraries into /usr/local:
```bash
sudo make install
```

Result:
```bash
/usr/local/lib/libmcp23s08.so
/usr/local/lib/libmcp23s09.so
/usr/local/include/lib_mcp23s08.h
/usr/local/include/lib_mcp23s09.h
```

---

## Usage

#### Compile own Program
Link application against one or both MCP23S0x libraries:
```bash
gcc -o <program_name> <program_name>.c -I/usr/local/include -L/usr/local/lib -lmcp23s08 -lmcp23s09
```

If system does not automatically load libraries from /usr/local/lib, add:
```bash
-Wl,-rpath,/usr/local/lib
```

Full example:
```bash
gcc -o my_app main.c -I/usr/local/include -L/usr/local/lib -lmcp23s08 -lmcp23s09 -Wl,-rpath,/usr/local/lib
```

----

## Example
```c
#include <stdio.h>
#include "lib_mcp23s08.h"

int main(void) {
    printf("Initializing MCP23S08...\n");
    if (mcp23s08_init(0) == 0)
        printf("MCP23S08 initialized successfully!\n");
    else
        printf("Initialization failed.\n");
    return 0;
}
```

Compile and run:
```bash
gcc -o test_mcp23s08 test_mcp23s08.c -lmcp23s08
./test_mcp23s08
```

---

## Maintenance

#### Uninstall
```bash
sudo make uninstall
```

#### Clean Build Files
```bash
make clean
```

---

#### License
This project is licensed under the MIT License.

---

#### Author
Developed and maintained by Hauke Schlosser

--- 

#### Keywords
SPI, MCP23S08, MCP23S09, Microchip, GPIO Expander, C Library, Raspberry Pi, Embedded Linux, spidev, I/O Control, Shared Library, Device Driver, Hardware Abstraction Layer, HAL