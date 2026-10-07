# Lab 3 - SPI Flash Memory

Let consider this lab as a product that needs some additional features of.
Our research team has reported differences in performance based on flash memory
sizes, so they have asked that we include logs of the flash memory manufacturer
ID as well as its capacity is. It is your job to implement the needed parts to
communicate with the flash memory and to report the necessary information.

1. Implement an SPI driver from scratch
2. Use it to talk to a NOR flash memory chip over SPI.

## Overview

Write an SPI driver against the `spi` interface (`spi.hpp`), then use it
to:

1. Read and print the flash chip's manufacturer ID.
2. Read and print its memory capacity.
3. Read and print the first 32-byte block.
4. Take user input and write up to 32 bytes to that same block.

Each of these should be its own class function (or set of class functions)
added to the `flash_memory` driver within the `lab3_spi.cpp` file.

## Learning Objectives

- Implement an SPI driver from scratch, with a configurable bit rate.
- Drive a chip-select line with a plain GPIO output pin.
- Talk to a real SPI device (NOR flash) using your own driver.

## Hardware

Board: [LP-MSPM0G3507 LaunchPad](https://www.ti.com/tool/LP-MSPM0G3507).
Which SPI instance/pins you use is up to you - check the datasheet's
pin-to-peripheral function table for which pins carry SPI0/SPI1 signals,
and drive chip-select with a plain `output_pin`.

NOR flash part: Winbond W25Q32JV - 32 Mbit SPI NOR flash or similar.

## Grading Rubric (60 pt)

1. **SPI signaling - 15 pts.** Produces correct signals on the
   out/in/clock lines at the bit rate the application configures.
2. **ID + capacity - 15 pts.** Reads and prints both the manufacturer ID
   and memory capacity.
3. **Block read - 10 pts.** Reads and prints the first 32 bytes of the
   device.
4. **Block write - 10 pts.** Takes user input and writes up to 32 bytes
   to that first block.
5. **Code submission - 10 pts.** Submit a PR to your git repo targeting
   your `main` branch.

## Reference material

- **[MSPM0G3507 datasheet](https://www.ti.com/lit/ds/symlink/mspm0g3507.pdf)** -
  pin-to-peripheral function table for SPI pins.
- **[MSPM0G350x Technical Reference Manual](https://www.ti.com/lit/ug/slau846e/slau846e.pdf)** -
  SPI chapter.
- **[Winbond W25Q32JV datasheet](https://www.mouser.com/datasheet/2/949/w25q32jv_revg_03272018_plus-1489806.pdf)** -
  the NOR flash chip for this lab. Look for the JEDEC ID command
  (`0x9F`) for manufacturer/capacity, and the standard read/page-program
  commands.

## Running this implementation

The driver uses SPI0 at **1 MHz, mode 0**, with a GPIO chip select. SPI and
chip-select registers are configured directly in `lab3_spi.cpp`.

| LaunchPad connection | W25Q32JV signal (8-pin package) |
| --- | --- |
| PA12 | CLK (6) |
| PA13 | DO / MISO (2) |
| PA14 | DI / MOSI (5) |
| PA8 | /CS (1) |
| 3.3 V | VCC (8), /WP (3), /HOLD (7) |
| GND | GND (4) |

Use the module's signal labels if its connector differs from the bare chip.
The pin functions are listed in the TI datasheet linked above.

1. Open the repository workspace in CCS and build `lab3_spi`. If CCS reports
   a missing SDK or SysConfig version, select the installed versions in the
   project's **Properties > General > Products**. This implementation was
   compiled and linked with TI Arm Clang 5.1.1.LTS, MSPM0 SDK 2.11.00.07, and
   SysConfig 1.28.1. `startup.c` includes the SDK's startup/vector table.
2. After connecting the module, start a CCS debug session and run. Use the
   **CIO console** for `printf`/`getchar` input and output; keep the debugger
   attached. This does not use a serial terminal.
3. Expect manufacturer `0xEF`, capacity `4194304 bytes (32 Mbit)`, and a hex
   dump of the first 32 bytes. Enter a line of text; only its first 32 bytes
   are written, without a newline or null terminator. An empty line skips
   writing. Bytes beyond the input length retain their previous contents.
4. The program erases and restores the first 4 KiB sector, preserving bytes
   outside the input and verifying every restored page. Keep power connected
   until it finishes; the erase/restore sequence is not power-loss safe.
5. Rerun to check persistence. A logic analyzer should show idle-low SCLK at
   1 MHz, MSB-first bytes, and /CS low across each complete command and data
   transfer. The SPI driver also accepts modes 1-3 and rates from 15625 Hz
   upward, rounding down (maximum 16 MHz with the default 32 MHz clock).

With no module attached, the program reports that no supported flash was
detected and exits. Compilation/linking has been checked; physical SPI and
flash read/write verification remain pending until the module is connected.
