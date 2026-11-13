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

## Local implementation and wiring

The implementation uses SPI1 at **1 MHz, mode 0, 8-bit, MSB first**. SPI and
chip select are configured by direct register accesses; SysConfig supplies only
clocks and debug pins. The named register constants come from TI's SDK. These
C++17 projects do not include `third_party/libhal-util/bit.hpp`, which requires
C++20 concepts and class-type template arguments.

Use these **LaunchPad signal labels**, not the chip's physical pin numbers:

| LaunchPad | Flash signal | Common 8-pin W25Q package pin |
| --- | --- | --- |
| PB6 | /CS | 1 |
| PB7 | DO / MISO / IO1 | 2 |
| PB8 | DI / MOSI / IO0 | 5 |
| PB9 | CLK | 6 |
| 3V3 | VCC | 8 |
| GND | GND | 4 |
| 3V3 | /WP and /HOLD, if not already pulled high on the module | 3 and 7 |

Check the full flash part number and module pin labels before wiring. The
W25Q32JV is a **2.7–3.6 V** device; other W25Q variants may require **1.8 V**.
The table's package numbers apply only to the common 8-pin package. A bare
surface-mount chip needs an adapter/breakout and a 100 nF bypass capacitor near
VCC/GND. A 10 kOhm pull-up on /CS keeps it inactive during MCU reset if the
module does not already provide one. Follow the
[Winbond datasheet](https://www.winbond.com/resource-files/w25q32jv%20revg%2003272018%20plus.pdf)
and the
[LaunchPad schematic](https://www.ti.com/lit/pdf/slau873)
for your board revision.

Female-to-female jumpers work with male headers on both boards. To attach the
logic analyzer at the same time, use grabber probes, spare connected header
positions, or a breadboard and suitable male jumper ends. Connect analyzer
GND to circuit GND. No separate flash programmer is needed.

### Run and demonstrate

1. Open the repository workspace in CCS with MSPM0 SDK 2.10.00.04, SysConfig,
   and TI ARM Clang 5.1.x installed. Build `lab3_spi` and debug it through the
   LaunchPad's USB data connection.
2. Use the **CCS CIO console** for `printf` and keyboard input. This program
   uses debugger-assisted standard I/O; it does not configure a UART terminal.
   Reserve at least **2048 bytes of stack** and **1024 bytes of heap** for the
   C/C++ runtime in the CCS/SysConfig project configuration, then verify the
   generated link map. The sector backup itself uses static storage.
3. For a W25Q32JV, expect manufacturer `0xEF` and capacity `4194304` bytes.
   The driver accepts Winbond W25Q IDs with memory type `0x40`, `0x60`, or
   `0x70` and capacity exponents `0x10` through `0x18`. Larger or unidentified
   parts are rejected before any erase/program operation.
4. Capture CS, CLK, MOSI, and MISO at 10 MS/s or faster. Decode mode 0, MSB
   first. Check a 1 us clock period, `0x9F` identification, and a `0x03`
   read starting at address `0x000000` with CS low across command and data.
5. Enter 1–32 text bytes and press Enter. The program verifies the write and
   prints the block again. Empty input skips the write; longer input is
   rejected in full. Newlines and the string terminator are not stored.
   Bytes beyond the entered length retain their previous contents.
6. Write different text twice, then reset the MCU and check persistence. Keep
   the flash powered during each update. The software preserves the rest of
   the first sector, but interruption during erase/program can lose that
   sector; use a lab chip with no valuable contents.

### Be able to explain the code

- `mspm0_spi::configure()` chooses clock dividers that do not exceed the
  requested rate. At the default 32 MHz clock, `CLKDIV=0` and `SCR=15` give
  `32 MHz / (1 * 2 * 16) = 1 MHz`. Polarity and phase select modes 0–3.
- Each transmitted byte also receives a byte. `transfer()` drains RX even
  for write-only commands, sends filler for reads, and waits until the final
  clock edge before the flash driver releases CS.
- JEDEC ID is manufacturer, memory type, capacity exponent. `2^0x16` bytes
  is 4 MiB, which is 32 Mbit; bits and bytes differ by a factor of eight.
- NOR programming changes bits from 1 to 0. Erasing restores 1s in a whole
  4096-byte sector. `write_first_block()` backs up the sector, changes only
  the requested bytes, erases, restores 256-byte pages, and verifies all
  4096 bytes. A write-enable command is needed before every erase/program.
- The status register's busy bit must clear before the next operation.
  Polls are bounded so missing or unresponsive hardware produces an error.

Target compilation and simulated-device tests do not replace the logic
analyzer captures and real flash demonstration required by the rubric.

## Validation and submission evidence

Run the host test from the repository root:

```sh
c++ -std=c++17 -Wall -Wextra -Werror -DLAB_HOST_TEST \
  tests/lab3_device_test.cpp -o /tmp/lab3-device-tests
/tmp/lab3-device-tests
```

The host model checks identification, erase/program sequencing, sector
preservation, repeated and short writes, size limits, write protection,
verification failures, and bounded busy polling.

The November 10 board test passed 17 checks with the logic analyzer attached:
ten repeated reads, full and short writes, erase/rewrite, empty input, oversized
input rejection, a captured final write, and reset persistence. The startup
capture shows JEDEC `EF 40 16` (4 MiB). The write capture sampled at 12 MS/s shows
approximately 1.01 MHz SPI and verifies all 4096 sector bytes, with bytes 32
through 4095 preserved. The saved text is `LAB3 PASS: MSPM0G3507 SPI FLASH!`.

See the [test summary](evidence/2026-11-10/summary.json),
[functional results](evidence/2026-11-10/functional-tests.json), and
[waveform verification](evidence/2026-11-10/waveform-verification.json).
Open the [startup capture](evidence/2026-11-10/startup-verified.sr) or
[write capture](evidence/2026-11-10/write-verified.sr) in PulseView with
D0=CS, D1=CLK, D2=MOSI, D3=MISO, mode 0, MSB first. UART logs, decoded
transactions, and sector dumps are stored beside the captures.

Earlier reads and write verification were inconsistent. The failures stopped
after analyzer hookup/reconnection without a source change; their exact cause
was not isolated. These passing results apply to that physical setup. Operation
after probe removal and a complete power cycle with the final text were not
retested. The IMU remained connected, and module supply measured 3.25 V.

The physical test used the optional [UART runtime](../../tools/board/README.md)
with the same lab implementation, TI ARM Clang 5.1.1.LTS, and MSPM0 SDK
2.10.00.04. Both the CCS project and UART image built successfully. Physical
operation of the CCS CIO/debugger console was not tested. Evidence includes the
flashed image hash; generated firmware and machine-specific CCS setup stay out
of source control.
