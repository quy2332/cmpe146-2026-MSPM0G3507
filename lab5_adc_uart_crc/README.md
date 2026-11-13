# Lab 5 - ADC + UART + CRC (FreeRTOS)

For this lab, you will:

1. Implement an ADC driver, with DMA-driven sampling of the on-board
   internal temperature sensor.
2. Implement a UART/serial driver with an independent DMA-backed circular
   receive buffer.
3. Implement a hardware-accelerated CRC-32 driver, register-level against
   the on-chip CRC accelerator, and benchmark it against a supplied software
   CRC-32 implementation.
4. Write the ADC-to-temperature conversion yourself.
5. Use FreeRTOS to build a request/response system between two boards
   playing different roles: a **sensor** board samples its internal
   temperature sensor and a **display** board requests readings and shows
   them on an RGB LED.

## Use cases

Consider this a component of a larger product. Products where a sensor node
answering on-demand requests over a CRC-protected UART link (or one like it)
would be used include:

- A central controller polling one or more remote sensor nodes on demand
  instead of being flooded with a constant stream of readings
- Environmental monitoring systems where a display/control unit is
  physically separate from the sensing hardware
- Any point-to-point serial link where corrupted data must be detected
  before it's acted on

## Overview

This lab requires FreeRTOS. Two boards play two different roles:

- The **sensor** board has its internal temperature sensor wired to its
  ADC. It waits for a one-byte request from the display board, then
  samples, converts, and replies with a batch of readings.
- The **display** board has the RGB LED and the S1/S2 buttons. Pressing
  either button asks the sensor board for a fresh batch of readings, then
  shows the result as a color: blue for cold, yellow for warm, red for hot
  (the exact temperature bands are your call).

Both roles are implemented in the same `lab5_adc_uart_crc.cpp` file. A
`constexpr device_role this_device` near the top of the file picks which
role's tasks `main()` creates, and an `if constexpr` in `main()` compiles in
only that role's task. The other role's code is discarded at compile time,
so a single codebase produces either board's firmware. For this lab, flip
`this_device` by hand and rebuild before flashing each board; the comment
above it explains how you'd instead drive this from a compiler define on
the build system's command line if you wanted, say, a CI job to build both
roles from one invocation without hand-editing source.

The exchange itself:

1. Display board: wait for S1 or S2 to be pressed, then transmit a one-byte
   command to the sensor board.
2. Sensor board: on receiving that command, fill a 64-sample buffer from
   its internal temperature sensor in a single DMA-backed ADC transfer,
   convert each raw sample to a temperature value, compute a CRC-32 over the
   batch with both your `crc32_hardware` driver and the supplied
   `crc32_software` reference (timing each with a `lab2::steady_clock` and
   printing both durations, so you can see which is faster), and transmit
   the 64 temperature values with the CRC appended over UART. Before
   transmitting, poll one of its own buttons (S1 or S2, your choice). If
   it's being held down, flip one bit somewhere in the outgoing packet
   before sending it. This doesn't need an interrupt; a plain level read at
   transmit time is enough. It gives you an on-demand way to demonstrate the
   CRC check catching corrupted data without editing and reflashing code.
3. Display board: receive the batch, validate its CRC, and set the RGB LED
   color from the readings. If the CRC is invalid, drop the packet, print an
   appropriate message, and turn the RGB LED magenta instead of updating it
   with a color derived from the (untrustworthy) readings.

The wire format for a single temperature value is your choice, as long as
it can resolve at least 0.5 degC steps. A plain whole-degree integer
cannot. Both boards need to agree on whatever encoding you pick, since one
side writes it and the other reads it back.

Each of these should be its own class function (or set of class functions)
added to the `adc_driver`, `serial_driver`, and `crc32_hardware` classes
within the `lab5_adc_uart_crc.cpp` file. All three drivers are written from
scratch, register-level - no DriverLib anywhere in this lab. That includes
reading the internal temperature sensor's factory calibration constants -
look for how they're stored in the MSPM0G350x Technical Reference Manual
and datasheet, the same way you'd track down any other register.

`adc16_stream` fills an entire buffer per call, rather than returning one
sample at a time - the ADC driver must use DMA to configure a single
transfer that fills the whole buffer, put the calling task to sleep using
FreeRTOS for the duration of that transfer, and let the DMA
transfer-complete interrupt wake it once the buffer is full (one sleep per
buffer, not one per sample). The UART driver's receive side must similarly
be backed by a circular buffer that a **separate** DMA transfer (or
interrupt) fills continuously in the background - see `receive_cursor()`'s
docs in `serial.hpp` for how a caller is expected to consume it. That means
the sensor board runs two independent DMA-backed transfers at once: one for
the ADC, one for UART reception. `crc32_software` is supplied and already
implemented; use it as-is to check your `crc32_hardware` driver's output
and as the baseline for the timing comparison.

> [!NOTE]
> Strictly speaking, this lab doesn't require FreeRTOS to work. The
> DMA-transfer-complete wake-up could just as well be done synchronously -
> the driver itself blocking on a completion flag - without an RTOS in the
> picture at all. FreeRTOS is used here anyway, mainly to get you writing
> against it before the final project.

## Learning Objectives

- Implement a buffer-filling ADC driver, register-level with DMA, that
  frees the CPU (via putting its calling task to sleep) for the duration of
  a whole buffered transfer rather than per sample.
- Implement a UART driver with a DMA-backed circular receive buffer.
- Implement a register-level driver for the on-chip CRC-32 accelerator, and
  measure how much it outperforms a software CRC-32 implementation.
- Convert a raw ADC sample from the internal temperature sensor into an
  actual temperature value.
- Design a simple request/response protocol between two independently
  running devices with different roles.
- Use `if constexpr` on a compile-time constant to build one codebase into
  either of two different firmware images.
- Detect corrupted data using a checksum computed on both ends of a link.

## Hardware

Board: [LP-MSPM0G3507 LaunchPad](https://www.ti.com/tool/LP-MSPM0G3507). You
will need two boards, one per role. Which UART instance/pins you use is up
to you - check the datasheet's pin-to-peripheral function table for which
pins carry UART signals, and cross-connect the two boards' TX/RX lines (and
grounds). The display board also needs the on-board RGB LED and S1/S2, and
the sensor board needs one of S1/S2 (your choice which) for the corrupted-
data demonstration - see
[lab1_gpio/README.md](../lab1_gpio/README.md#hardware) for that wiring.

The internal temperature sensor requires no external wiring - it's an ADC
input channel built into the chip. Which channel it's mapped to, how to
enable it, and where its factory calibration constants live are all in the
MSPM0G350x Technical Reference Manual and datasheet; finding that is part
of the assignment.

## Grading Rubric (100 pt)

1. **ADC + DMA - 15 pts.** `adc_driver` correctly implements the
   `adc16_stream` interface register-level with DMA, filling the whole given
   buffer in a single transfer and putting its calling task to sleep until
   the DMA transfer-complete interrupt wakes it.
2. **UART + DMA - 15 pts.** Serial driver transmits data at the configured
   baud rate, and its receive side is a circular buffer that works with its
   own independent DMA transfer.
3. **CRC driver - 15 pts.** `crc32_hardware` produces a CRC that matches the
   supplied `crc32_software` reference implementation, and is measurably
   faster.
4. **ADC-to-temperature conversion - 10 pts.** Raw ADC samples are
   correctly converted to temperature values that resolve at least 0.5 degC.
5. **Sensor role - 10 pts.** On receiving the request byte, the sensor
   board samples, converts, and transmits a 64-value batch with a valid CRC
   appended.
6. **Display role - 10 pts.** Pressing S1 or S2 requests a batch and, once
   a valid batch arrives, sets the RGB LED color based on the readings.
   Invalid CRCs are dropped with a printed message and the LED turns
   magenta instead of updating with a color derived from the readings.
7. **FreeRTOS application - 10 pts.** Each board runs the correct role's
   task, selected via the `if constexpr` in `main()`.
8. **Corrupted data demonstration - 5 pts.** Holding S1 or S2 (your choice)
   on the sensor board while it transmits flips one bit in the outgoing
   packet, and the display board correctly detects and reports the
   resulting CRC mismatch.
9. **Code submission - 10 pts.** Submit a PR to your git repo targeting your
   `main` branch.

## Reference material

- **[MSPM0G3507 datasheet](https://www.ti.com/lit/ds/symlink/mspm0g3507.pdf)** -
  pin-to-peripheral function table for UART pins, and internal temperature
  sensor electrical characteristics.
- **[MSPM0G350x Technical Reference Manual](https://www.ti.com/lit/ug/slau846e/slau846e.pdf)** -
  ADC12, DMA, UART, CRC accelerator chapters, and the internal temperature
  sensor's calibration constants.

## Local implementation and wiring

All three drivers are register-level against the MSPM0G3507 register maps in
`ti/devices/msp/msp.h` (the names match the TRM). No DriverLib function is
called in `lab5_adc_uart_crc.cpp`.

| Piece | Choice |
| --- | --- |
| ADC | ADC0 channel 11 (internal temperature sensor), 12-bit, VDDA reference, 20 us sample time, repeat-single conversion |
| ADC DMA | DMA channel 1, triggered by MEMRES0; one block of 64 transfers per `read()`. The calling task sleeps on a task notification that the DMA interrupt gives |
| UART | UART1, PA8 = TX, PA9 = RX, 115200 8N1 |
| UART receive | DMA channel 0 in repeated-single mode filling a 256-byte ring forever; `receive_cursor()` = 256 - DMASZ |
| CRC | CRC peripheral in CRC-32 mode with BITREVERSE (the reflected/zlib CRC-32), seed `0xFFFFFFFF`, final XOR in software, 32-bit word writes |
| Timer | TIMG12 at 32 MHz, extended to 64 bits in its zero-event interrupt, for the CRC timing |
| Sample encoding | `int16_t` in 0.01 degC, little-endian |
| Reply packet | `0x5A` marker, 64 samples (128 bytes), CRC-32 of the samples (4 bytes, little-endian) |

Temperature conversion (TRM "Temperature Sensor", datasheet 7.14 and 8.13):
`TEMP_SENSE0` at `0x41C4003C` holds the 12-bit code at 30 degC measured with
the 3.3 V VDDA reference (both test chips read 792 and 793). With the same
reference for the samples, T = 30 + (code - TEMP_SENSE0) x (3.3 V / 4096) /
(-1.8 mV/degC). One code is 0.45 degC, so each 0.01 degC sample resolves
better than the required 0.5 degC.

### Wiring between the two boards

Both PA8 and PA9 are on header J1, the same column as PB9 (pin 7) and the
Lab 4 I2C pins PB2/PB3 (pins 9 and 10). Count from the 3V3 end: pin 1 3V3,
pin 2 PA25, **pin 3 PA9**, **pin 4 PA8**.

| Board A | Board B |
| --- | --- |
| PA8 (J1.4, UART1 TX) | PA9 (J1.3, UART1 RX) |
| PA9 (J1.3, UART1 RX) | PA8 (J1.4, UART1 TX) |
| GND | GND |

PB8/PB9 cannot be used for this link: they have no UART function, and on a
board with the Lab 3 flash they are its SPI MOSI/clock lines. Keep J21 and
J22 on pins 1-2 on both boards; they route UART0 (PA10/PA11) to the USB
console.

### Run and demonstrate

1. Build each role. In CCS, set `this_device` in the `#else` branch and
   rebuild. With the standalone tools, `bash tools/board/build.sh 5 sensor`
   and `bash tools/board/build.sh 5 display` (the script passes
   `-DLAB5_DEVICE_ROLE_DISPLAY` for the display image).
2. Flash one board per role. If both XDS110s report the same serial number
   (`MG350001` here), select them by USB port; see
   [the board tools](../tools/board/README.md).
3. Open both USB consoles at 115200. The sensor prints its `TEMP_SENSE0` and a
   CRC self-test on `123456789` (`0xCBF43926` from both engines).
4. Press S1 or S2 on the display board. The sensor prints the batch average,
   the ADC+DMA time, and both CRC times; the display prints the average and
   sets the LED: blue below 20 degC, yellow up to 26 degC, red from 26 degC.
   A fingertip on the sensor board's MCU turns it red within a few presses.
5. Hold S2 on the sensor board and press a button on the display board. The
   sensor flips one payload bit after computing the CRC; the display prints
   `CRC MISMATCH ... Packet dropped` and turns the LED magenta.

### Hardware results (November 13, 2026)

Both roles were flashed to two LP-MSPM0G3507 boards and run end to end.
Button presses were simulated with the debugger (IOMUX input inversion on
S1/S2, with [`simulate_presses.py`](evidence/2026-11-13/simulate_presses.py)); the
console logs are in [`evidence/2026-11-13/`](evidence/2026-11-13/).

- Normal request: CRC OK, average 23.02 degC, LED yellow.
- Sensor S2 held: bit 74 flipped, display reported the mismatch, LED magenta.
- Released: CRC OK again, LED yellow.
- ADC+DMA: 1364 us for 64 samples (one DMA block, one task sleep).
- CRC-32 over the 128-byte payload: hardware 27.0 us, software 309.4 us
  (11x faster), identical results on every batch.

Host tests for the conversion, packet format, and CRC bit-flip detection:

```sh
c++ -std=c++17 -Wall -Wextra -Werror -DLAB_HOST_TEST \
  tests/lab5_device_test.cpp -o /tmp/lab5-device-tests
/tmp/lab5-device-tests
```

Not verified: physical button presses (simulated only), the blue band (needs
real cooling), and absolute temperature accuracy beyond the factory
single-point trim.
