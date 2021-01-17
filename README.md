# O2

O2 is a cycle-accurate emulator of the 6202 microprocessor in just ~1000 lines of code. The library only consists of a header and a source file, which makes it easy to integrate into other projects which rely on a 6502 CPU (like a NES emulator for example). libO2 can also be configured and compiled automatically if the project uses CMake.

## Building

### Standalone

Since libO2 only has one file, it can be compiled without any overlying toolchain; simply compile with `CXX -o O2.o O2.cpp -I /path/where/O2.hpp/resides -c -O2`. This will create the object file `O2.o` that you can link to your program.

### CMake

Start by adding O2 to your project: `add_subdirectory(path/to/O2)`. The project can then link with libO2 with for example `target_include_directories(TARGET PRIVATE ${O2_DIRECTORIES})` and `target_link_libraries(TARGET ${O2_LIBRARIES})`.

## Usage

### Initialization

Using libO2 is very simple since everything is contained in the O2::CPU class. Creating a new instance is as easy as creating a new class:

```cpp
#include <O2.hpp>

int main()
{
    O2::CPU processor;
}
```

This will initialize a new instance with default values.

To keep libO2 as flexible as possible, the emulator does not handle any of the memory and outsources it for the program to handle. Because of that, the program needs to configure read and write functions for the CPU. This can be done with the `setRW` functions or the constructor itself.

```cpp
#include <O2.hpp>

uint8_t read(uint16_t address)
{
    // * Return the correct value based on the address *
}
void write(uint16_t address, uint8_t value)
{
    // * Write [value] to [address] *
}

int main()
{
    // Create new CPU class with default values
    O2::CPU processor0();
    // Set the correct read-write functions
    processor0.setRW(read, write);
    // Or set them directly in the constructor
    O2::CPU processor1(read, write);
}
```

std::bind can be used to pass a member function to setRW:

```cpp
#include <O2.hpp>

class emulatorWrapper
{
    O2::CPU cpu;

    void config()
    {
        cpu.setRW(std::bind(&emulatorWrapper::read, this, _1), std::bind(&emulatorWrapper::write, this, _1, _2));
    }
    uint8_t read(uint16_t address)
    {
        // * Return the correct value based on the address *
    }
    void write(uint16_t address, uint8_t value)
    {
        // * Write [value] to [address] *
    }
}

```

### Execute

Since libO2 emulates the O2 down to the cycle, the program can run the processor either cycle-by-cycle, instruction-by-instruction, or millisecond-by-millisecond. The library comes with functions for all these types:

```cpp
#include <O2.hpp>

unsigned char read(unsigned short address)
{
    // * Return the correct value based on the address *
}
void write(unsigned short address, unsigned char value)
{
    // * Write [value] to [address] *
}

int main()
{
    O2::CPU processor(read, write);

    // Execute a single cycle
    processor.cycle();
    // Execute 10 cycles
    processor.cycles(10);
    // Complete a full instruction
    processor.step();
    // Execute 10 instructions
    processor.steps(10);

    // Execute indefinitely at the default speed of ~1.79 MHz
    processor.run();
    // Execute indefinitely at a speed of 100 Hz
    processor.run(100UL);
    // Execute for 100 milliseconds at the default speed of ~1.79 MHz
    processor.run(100ULL);
    // Execute for 100 milliseconds at a speed of 100 Hz
    processor.run(100ULL, 100UL);
}
```

### Interrupts

Interrupts can be raised by using the `raise` function:

```cpp
#include <O2.hpp>

unsigned char read(unsigned short address)
{
    // * Return the correct value based on the address *
}
void write(unsigned short address, unsigned char value)
{
    // * Write [value] to [address] *
}

int main()
{
    O2::CPU processor(read, write);

    // Raise a NMI interrupt
    processor.raise(O2::NMI);
    // Raise a RESET interrupt
    processor.raise(O2::RESET);
    // Raise a IRQ interrupt
    processor.raise(O2::IRQ);
}
```
