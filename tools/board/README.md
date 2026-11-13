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
bash tools/board/build.sh 5 sensor   # Lab 5 needs a role:
bash tools/board/build.sh 5 display  # sensor or display.
```

The `.out` and `.bin` files go into the lab's ignored `Debug/uart/` directory.
This runtime sets the system/bus clock to 32 MHz, reserves 4 KiB each for stack
and heap, and restores initialized data from flash on reset.

Lab 5 also compiles the FreeRTOS kernel from the SDK (heap_4, the TI ARM Clang
CM0 port, TI's static-allocation hooks) with `freertos_kernel/` as the config
directory, and writes `lab5_adc_uart_crc_<role>.out/.bin`. The display image
is built with `-DLAB5_DEVICE_ROLE_DISPLAY`.

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

## Two boards with the same probe serial (Lab 5)

LaunchPad XDS110s can share a serial number (both boards used for Lab 5
report `MG350001`), and pyocd opens probes by serial number, so `--uid` cannot
tell them apart. Select each board by its USB port path instead:

```sh
for tty in /dev/ttyACM*; do echo "$tty $(udevadm info -q path -n $tty)"; done
python3 tools/board/flash.py 5 --role sensor --pack /path/to/device.pack --usb-port 3-1.1.4
python3 tools/board/flash.py 5 --role display --pack /path/to/device.pack --usb-port 3-1.1.2
```

The port path is the `3-1.1.4` part of the sysfs path; that board's console is
the `ttyACM*` whose path ends in `3-1.1.4:1.0`. The `ttyACM` numbers can swap
after replugging, while the port path stays with the physical USB port. Every
new debugger connection resets the board. Before programming, `flash.py`
resets and halts the core (with an application running, the first sector
erase could report failure even though it erased, so one failed erase is also
retried), and it leaves the new image running afterwards.

## Wiring and evidence

Follow the relevant [Lab 3](../../lab3_spi/README.md),
[Lab 4](../../lab4_i2c/README.md), or [Lab 5](../../lab5_adc_uart_crc/README.md)
instructions. Disconnect USB before moving
wires. Verify module voltage ratings; some W25Q parts require 1.8 V.
The two labs use separate signal pins and can share compatible 3V3/GND power.
Only one lab runs on the MCU at a time.

Each submission includes its host test and the saved hardware evidence under
that lab's `evidence/` directory. These records describe the tests actually
performed and their limitations. Running the host test or build script does
not flash the connected board.
