# Build and flash the lab UART demos

The optional tools in this folder build a lab with a USB serial console instead
of CCS CIO. They use the same lab source and leave CCS project metadata and
SysConfig files unchanged. The implemented lab and its test instructions are
listed in that lab's README.

## Build

Install TI ARM Clang 5.1.x and MSPM0 SDK 2.10.00.04. From the repository root:

```sh
export TI_COMPILER_ROOT=/path/to/ti-cgt-armllvm_5.1.1.LTS
export MSPM0_SDK_ROOT=/path/to/mspm0_sdk_2_10_00_04
bash tools/board/build.sh 3  # Use 4 for the Lab 4 implementation.
```

The `.out` and `.bin` files go into the lab's ignored `Debug/uart/` directory.
This runtime sets the system/bus clock to 32 MHz, reserves 4 KiB each for stack
and heap, and restores initialized data from flash on reset.

## Console and programming

UART0 uses PA10 TX and PA11 RX at **115200 baud, 8N1, no flow control**. Keep
LaunchPad jumpers J21/J22 in their default 1-2 positions for the XDS110 USB
serial bridge. See the [TI board guide](https://www.ti.com/lit/pdf/slau873).
The CCS project builds continue to use their original CIO console.

Install Python packages `pyocd` and `pyserial`, and download TI's
[MSPM0 device pack](https://software-dl.ti.com/msp430/esd/MSPM0-CMSIS/MSPM0G1X0X_G3X0X/latest/exports/TexasInstruments.MSPM0G1X0X_G3X0X_DFP.1.3.1.pack).
Use `python3 -m pyocd list` to find the probe serial number. Linux must grant
access to the USB debug interface and serial device.

Open the serial monitor before programming to retain startup messages:

```sh
python3 -m serial.tools.miniterm /dev/ttyACM0 115200 --raw --eol LF
# In another terminal, substituting your pack path and probe serial number:
python3 tools/board/flash.py 3 --pack /path/to/device.pack --uid MG350001
```

Use `4` to program Lab 4. Programming replaces the lab running on the MCU.
The script validates the image and device identity, saves the previous 128 KiB
main flash, erases affected main-flash sectors, writes and verifies every image
byte, and resets the MCU. It does not program device-security configuration or
automatically erase a locked device.

For a logged console, run `console_bridge.py /dev/ttyACM0` and open the monitor
on `/tmp/lab34-console` instead. The bridge owns the physical serial port and
saves received bytes in `/tmp/lab34-console.log`. Stop and restart both programs
after unplugging the board; do not open the physical port in another reader.

## Wiring and evidence

Follow the relevant [Lab 3](../../lab3_spi/README.md) or
[Lab 4](../../lab4_i2c/README.md) instructions. Disconnect USB before moving
wires. Verify module voltage ratings; some W25Q parts require 1.8 V.
The two labs use separate signal pins and can share compatible 3V3/GND power.
Only one lab runs on the MCU at a time.

Each submission includes its host test and the saved hardware evidence under
that lab's `evidence/` directory. These records describe the tests actually
performed and their limitations. Running the host test or build script does
not flash the connected board.
