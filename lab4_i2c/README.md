# Lab 4 - I2C IMU

For this lab, you will:

1. Implement an I2C driver from scratch.
2. Use it to talk to an I2C accelerometer/IMU chip.
3. Use the accelerometer to determine the orientation of the chip

## Use cases

Consider this a component of a great product. Such products where this
technology would be used is:

- Smart phone orientation detection
- Drone orientation and balance
- Auto leveling robotics
- Level measurement device

## Overview

Write an I2C driver against the `i2c` interface (`i2c.hpp`), then use it to
implement the `accelerometer` interface (`accelerometer.hpp`) for the BMI160:

1. Read the X, Y, and Z acceleration values from the IMU.
2. Compute roll and pitch angles from the acceleration vector.
3. Continuously print the roll and pitch values to stdout.

Each of these should be its own class function (or set of class functions)
added to the `bmi160` driver within the `lab4_i2c.cpp` file.

DriverLib is allowed for this lab's I2C driver implementation — this is your
first chance to use a vendor-supplied API instead of writing register-level
code, so use it to get comfortable working with a vendor SDK to implement an
interface.

## Learning Objectives

- Implement an I2C driver, with SDA/SCL signal generation and device
  read/write, using DriverLib.
- Talk to a real I2C device (accelerometer/IMU) using your own driver.
- Convert raw accelerometer readings into roll and pitch angles.

## Hardware

Board: [LP-MSPM0G3507 LaunchPad](https://www.ti.com/tool/LP-MSPM0G3507).
Which I2C instance/pins you use is up to you - check the datasheet's
pin-to-peripheral function table for which pins carry I2C0/I2C1 signals.

IMU part: [Bosch BMI160](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi160-ds000.pdf)
6-axis accelerometer/gyroscope. This is the chip that will be provided to you
for this lab.

## Grading Rubric (60 pt)

1. **I2C implementation - 20 pts.** Produces correct SDA/SCL signals at the
   configured clock rate and can read from and write to a device on the bus.
2. **Accelerometer implementation - 20 pts.** Implements the `accelerometer`
   interface for the BMI160 and returns acceleration values for the X, Y, and
   Z axes.
3. **Roll and pitch - 10 pts.** Continuously computes and prints roll and
   pitch from the acceleration values.
4. **Code submission - 10 pts.** Submit a PR to your git repo targeting your
   `main` branch.

## Reference material

- **[MSPM0G3507 datasheet](https://www.ti.com/lit/ds/symlink/mspm0g3507.pdf)** -
  pin-to-peripheral function table for I2C pins.
- **[MSPM0G350x Technical Reference Manual](https://www.ti.com/lit/ug/slau846e/slau846e.pdf)** -
  I2C chapter.
- **[BMI160 datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi160-ds000.pdf)** -
  command/power-mode, configuration, and acceleration output registers.

## Local implementation and wiring

The implementation uses **I2C1 at 100 kHz**, with DriverLib inside the concrete
`mspm0_i2c` class. The BMI160 is configured for a **100 Hz** sample rate and
**+/-2 g** range. Only its accelerometer is needed for this lab.

| LaunchPad | BMI160 breakout |
| --- | --- |
| PB2 | SCL / SCK |
| PB3 | SDA / SDI |
| 3V3 | 3.3 V-compatible module supply; VDD and VDDIO if exposed separately |
| GND | GND |
| 3V3 | CS / CSB, unless already tied high on the module |
| GND | SDO / SA0, selecting address `0x68` |

If SDO is tied high, the address is `0x69`; change the address passed to
`bmi160` in `main()`. Do not leave address selection floating. CSB must stay
high for I2C. No interrupt-pin wiring is needed. See the
[BMI160 datasheet, primary I2C interface and connection diagrams](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi160-ds000.pdf).

SDA and SCL each need a pull-up to **3.3 V**. Many breakout boards include
these. If yours does not, start with **two 4.7 kOhm resistors**, one from SDA
to 3.3 V and one from SCL to 3.3 V, using short wires. Check the module's
schematic or measure its resistors before adding parallel pull-ups. The
code uses open-drain signals and does not rely on internal weak pull-ups.

With a BMI160 module, the LaunchPad, a USB data cable, a logic analyzer, and
female-to-female jumpers, the major parts are covered if both boards have
male headers. A breadboard plus male-to-female/male-to-male wires is useful
for adding resistors and connecting analyzer probes. A bare BMI160 chip
requires a breakout board; loose jumpers cannot connect directly to it.

### Run and demonstrate

1. Build `lab4_i2c` in the repository CCS workspace using SDK 2.10.00.04 and
   TI ARM Clang 5.1.x. Use the LaunchPad's USB debug connection and **CCS CIO
   console**. No separate USB-to-UART adapter is needed for this program.
   Reserve at least **2048 bytes of stack** and **1024 bytes of heap** for
   standard I/O and floating-point formatting, and verify the link map.
2. Decode SDA/SCL as I2C with a logic analyzer. The default clock period is
   10 us. For a register read at address `0x68`, expect START, `0xD0` on the
   wire (address plus write bit), register number, repeated START, `0xD1`
   (address plus read bit), data, final NACK, and STOP. Decoders that display
   7-bit addresses show `0x68` for both phases.
3. The chip-ID register at `0x00` should return `0xD1`. Initialization retries
   with an error message if identity, bus access, power mode, or data-ready
   checks fail. An absent device must not appear as valid zero acceleration.
4. Hold the module still with its marked +Z axis upward. Expect about
   `(0, 0, +1) g` and near-zero roll/pitch, allowing sensor offset. Tilt about
   each marked axis and watch the sign and angle change. The printed axes
   refer to the sensor, not necessarily the orientation of the breakout text.
5. Capture one six-byte burst starting at `0x12`. Record the measured bus
   clock, ACKs, repeated START, and angle readings for the lab demonstration.

### Be able to explain the code

- `mspm0_i2c` implements write, read, and combined write/read transactions.
  A combined read omits STOP after the register address, then sends a
  repeated START. Waiting for bus-idle between those phases would hang
  because the controller deliberately still owns the bus.
- The I2C hardware's low/high timing factors total 10. With 32 MHz BUSCLK,
  divider 1 and timer period 31 produce `32 MHz / (10 * 32) = 100 kHz`.
  DriverLib configures the hardware; the class handles FIFO movement,
  completion, NACKs, arbitration loss, and bounded waits. The short delay
  after starting a transfer follows TI's I2C_ERR_13 workaround.
- The BMI160 starts suspended. Initialization checks ID, resets it, writes
  acceleration configuration/range, sends command `0x11` to register
  `0x7E`, waits for startup, and checks power mode and data-ready.
- Acceleration registers `0x12..0x17` contain signed, little-endian X/Y/Z.
  At +/-2 g, divide each signed 16-bit value by 16384 to obtain g.
- `roll = atan2(y, z)` and `pitch = atan2(-x, sqrt(y*y + z*z))`, converted
  from radians to degrees. Gravity gives tilt while still; acceleration
  during motion affects the estimate, and gravity alone cannot give yaw.
  Roll is undefined when gravity is exactly along X; the code reports NaN.
- A failed sensor read returns NaNs and a failure flag. `main()` prints a
  connection error instead of displaying an invented level orientation.

If a target physically holds SDA low, resetting the MCU's I2C controller
may not release it. Check the wiring and power-cycle the sensor before
retrying. Hardware waveform and orientation checks remain necessary even
when compilation and simulated-device tests pass.

## Validation and submission evidence

Run the host test from the repository root:

```sh
c++ -std=c++17 -Wall -Wextra -Werror -DLAB_HOST_TEST \
  tests/lab4_device_test.cpp -o /tmp/lab4-device-tests
/tmp/lab4-device-tests
```

The host model checks sensor initialization and configuration, a six-byte
sample burst, signed decoding and scaling, alternate/invalid addresses,
communication and identity failures, and roll/pitch including undefined angles.

The November 10 board capture recorded 2253 acceleration/tilt samples over
240 seconds with no read or initialization failures. Physical motion changed
roll from approximately -92 to +95 degrees and pitch from -84 to +49 degrees.
After an MCU reset, initialization and readings resumed. See the
[test summary](evidence/2026-11-10/summary.json),
[UART log](evidence/2026-11-10/uart.log), and
[parsed samples](evidence/2026-11-10/samples.jsonl).

This demonstrates communication, motion response, and reset recovery. Sensor
calibration, a controlled flat-orientation check, and logic-analyzer verification
of I2C clock timing/repeated START were not performed. The 100 kHz value in the
console is the configured rate, not a waveform measurement.

The physical test used the optional [UART runtime](../../tools/board/README.md)
with the same lab implementation, TI ARM Clang 5.1.1.LTS, and MSPM0 SDK
2.10.00.04. Both the CCS project and UART image built successfully. Physical
operation of the CCS CIO/debugger console was not tested. Evidence includes the
flashed image hash; generated firmware and machine-specific CCS setup stay out
of source control.
