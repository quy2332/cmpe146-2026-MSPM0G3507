# Lab 2 - Timers and PWM

In this lab you will:

1. Implement a steady clock and a PWM driver from scratch - TI DriverLib is
   not allowed.
2. Use the steady clock to build your own blocking delay function.
3. Use the PWM driver to animate a continuous RGB "color wheel" on the
   on-board LED.

This document is the complete assignment: what to build, how it's graded,
and how to get started.

## Overview

Drive the on-board RGB LED as a smooth, continuously animating color wheel:
each of the red, green, and blue channels gets its own PWM signal, and their
duty cycles rise and fall out of phase with each other (for example, 120
degrees apart) so the LED sweeps through colors forever. The whole lab is
getting a real time base and real PWM working, then combining them.

- **Timer -> delay.** Implement `lab2::steady_clock` (declared in
  `hal/timer.hpp`) on top of one of the device's hardware timers. Use its
  `uptime()` and `frequency()` to write your own blocking delay function,
  e.g. `delay_ms(clock, milliseconds)`. This is what paces your animation
  loop.
- **PWM -> brightness.** Implement `lab2::pwm` (also in `hal/timer.hpp`) on
  top of a timer's compare/capture channel. Use `duty_cycle()` to control
  how bright each LED channel is.
- **Combine.** In `main()`, continuously update the three PWM duty cycles -
  from a lookup table, a computed waveform, or whatever scheme you like -
  and call your delay function between updates so the animation runs at a
  visible, steady rate.

Encode the color sequence however you like (a lookup table, a formula,
three phase-shifted counters) - just make it visibly smooth and continuous.

## Learning Objectives

- Implement a steady clock (time base) driver from scratch.
- Implement a PWM driver from scratch.
- Convert a desired frequency into the correct timer register values
  (period/load and compare), instead of hardcoding them.
- Use a hardware timer to build a blocking delay function.
- Drive multiple PWM channels out of phase to animate a color transition.

## Hardware

Board: [LP-MSPM0G3507 LaunchPad](https://www.ti.com/tool/LP-MSPM0G3507).
Same LED wiring as `lab1_gpio`:

| Color | Pin  |
| ----- | ---- |
| Red   | PB26 |
| Green | PB27 |
| Blue  | PB22 |

## Requirements

1. **Steady clock.** Implement `lab2::steady_clock` using a GPTIMER.
   `frequency()` must return the timer's actual operating frequency, not a
   placeholder - you may pick whatever operating frequency you like, just
   make sure the driver reports it correctly.
2. **PWM.** Implement `lab2::pwm` using a GPTIMER's compare/capture channel.
   Give your PWM class a way to configure its frequency (a constructor
   parameter is fine) and actually use that frequency to compute the period
   (load) register value - don't hardcode it. Default the timer's input
   clock to BUSCLK (32 MHz) unless you have a specific reason to pick
   something else. You'll want frequency control on this driver again in
   later labs, so build it in now instead of bolting it on later.
3. **Delay function.** Using your steady clock, write a delay function that
   blocks the caller for a specified duration.
4. **Color wheel.** Combine the two drivers above to continuously animate
   the RGB LED as described in Overview.

## Grading Rubric (50 pt)

1. **Steady clock - 15 pts.** Implemented, running, and reports the correct
   frequency.
2. **PWM - 20 pts.** Implemented, running, frequency is configurable (not
   hardcoded), and is correctly translated into register values.
3. **Color wheel animation - 15 pts.** Smooth, continuous, and visible on
   hardware; the instructor/TA will observe it running.

## Reference material

- **[MSPM0G350x Technical Reference Manual](https://www.ti.com/lit/ug/slau846e/slau846e.pdf)** -
  General Purpose Input/Output (GPIO), Input/Output Multiplexer (IOMUX),
  and General Purpose Timer (GPTIMER) chapters.
- **[MSPM0G3507 datasheet](https://www.ti.com/lit/ds/symlink/mspm0g3507.pdf)** -
  pin-to-peripheral function table (which pins carry which timer's PWM
  output).
- **[LP-MSPM0G3507 User's Guide](https://www.ti.com/lit/slau873)** -
  on-board LED wiring.
- **`style.md` Ignore §S.10 (Memory-Mapped I/O)** - use offsets from the base address
  to access specific registers.

## HINT: FREQUENCY MATH

The PWM and steady clock drivers share the same timer hardware, so the
frequency math has the same shape for both:

1. Pick an input clock source for the timer (BUSCLK - 32 MHz - by default).
2. Pick a divider/prescaler if you want a different tick rate than the raw
   input clock.
3. For the steady clock, that resulting tick rate is exactly what
   `frequency()` must return.
4. For PWM, the timer's period register sets how many ticks make up one
   waveform cycle - which sets the frequency. The compare register sets how
   many of those ticks are HIGH - which sets the duty cycle. Both should be
   computed from the frequency and duty cycle you're given, not hardcoded.

Work this out from the GPTIMER chapter now - it's the same math you'll
reuse for every PWM/timer peripheral for the rest of the course.
