#include <cinttypes>
#include <cstdint>
#include <cstdio>

#include "../hal/gpio.hpp"
#include "../hal/timer.hpp"
#include "../hal/timer_util.hpp"

#include "ti_msp_dl_config.h"


namespace {

[[nodiscard]] volatile std::uint32_t& timer_register(
    std::uintptr_t p_base,
    std::uintptr_t p_offset)
{
  return *reinterpret_cast<volatile std::uint32_t*>(
      p_base + p_offset);
}

constexpr std::uintptr_t timg12_base = 0x4087'0000;

constexpr std::uintptr_t power_enable_offset = 0x0800;
constexpr std::uintptr_t reset_control_offset = 0x0804;
constexpr std::uintptr_t clock_divider_offset = 0x1000;
constexpr std::uintptr_t clock_select_offset = 0x1008;
constexpr std::uintptr_t counter_offset = 0x1800;
constexpr std::uintptr_t counter_control_offset = 0x1804;
constexpr std::uintptr_t load_offset = 0x1808;

constexpr std::uint32_t reset_command = 0xB100'0003;
constexpr std::uint32_t power_enable_command = 0x2600'0001;

constexpr std::uint32_t select_bus_clock = 1U << 3;

// CLKDIV stores the divider minus one:
// 0 means divide by 1; 7 means divide by 8.
constexpr std::uint32_t divide_clock_by_8 = 7U;

constexpr std::uint32_t bus_clock_frequency = 32'000'000U;
constexpr std::uint32_t timer_clock_frequency =
    bus_clock_frequency / 8U;

constexpr std::uint32_t counter_enable = 1U << 0;
constexpr std::uint32_t counter_repeat = 1U << 1;
constexpr std::uint32_t counter_count_up = 1U << 5;
constexpr std::uint32_t counter_start_at_zero = 1U << 29;

constexpr std::uintptr_t timg6_base = 0x4086'8000;
constexpr std::uintptr_t iomux_base = 0x4042'8000;

constexpr std::uintptr_t ccp_direction_offset = 0x1100;
constexpr std::uintptr_t output_disable_offset = 0x1104;

constexpr std::uintptr_t compare_0_offset = 0x1810;
constexpr std::uintptr_t compare_control_0_offset = 0x1830;
constexpr std::uintptr_t output_control_0_offset = 0x1850;
constexpr std::uintptr_t compare_action_0_offset = 0x1870;

constexpr std::uintptr_t channel_register_spacing = 4U;

constexpr std::uint32_t pwm_timer_frequency = 32'000'000U;

constexpr std::uint32_t output_high_at_zero = 0x0000'0001U;
constexpr std::uint32_t output_low_at_zero = 0x0000'0002U;
constexpr std::uint32_t output_low_at_compare_up = 0x0000'0400U;

constexpr std::uint32_t peripheral_connected = 1U << 7;

constexpr std::uintptr_t timg8_base = 0x4009'0000;

[[nodiscard]] volatile std::uint32_t& iomux_pin_control(
    std::size_t p_iomux_index)
{
  constexpr std::uintptr_t first_pin_control_offset = 0x0004;

  return *reinterpret_cast<volatile std::uint32_t*>(
      iomux_base +
      first_pin_control_offset +
      (p_iomux_index * sizeof(std::uint32_t)));
}

[[nodiscard]] std::uint16_t interpolate_duty_cycle(
    std::uint16_t p_start,
    std::uint16_t p_end,
    std::uint32_t p_step,
    std::uint32_t p_total_steps)
{
  std::int32_t const start = p_start;
  std::int32_t const difference =
      static_cast<std::int32_t>(p_end) - start;

  std::int32_t const value =
      start +
      ((difference * static_cast<std::int32_t>(p_step)) /
       static_cast<std::int32_t>(p_total_steps));

  return static_cast<std::uint16_t>(value);
}

void transition_color(
    lab2::pwm& p_red,
    lab2::pwm& p_green,
    lab2::pwm& p_blue,
    lab2::steady_clock& p_clock,
    std::uint16_t p_red_start,
    std::uint16_t p_green_start,
    std::uint16_t p_blue_start,
    std::uint16_t p_red_end,
    std::uint16_t p_green_end,
    std::uint16_t p_blue_end)
{
  constexpr std::uint32_t transition_steps = 128U;
  constexpr auto step_delay = std::chrono::milliseconds(5);

  for (std::uint32_t step = 0;
       step <= transition_steps;
       ++step) {
    p_red.duty_cycle(
        interpolate_duty_cycle(
            p_red_start,
            p_red_end,
            step,
            transition_steps));

    p_green.duty_cycle(
        interpolate_duty_cycle(
            p_green_start,
            p_green_end,
            step,
            transition_steps));

    p_blue.duty_cycle(
        interpolate_duty_cycle(
            p_blue_start,
            p_blue_end,
            step,
            transition_steps));

    lab2::delay(p_clock, step_delay);
  }
}

class mspm0_pwm_timer
{
public:
  mspm0_pwm_timer(std::uintptr_t p_base,
                  std::uint32_t p_requested_frequency)
      : base_(p_base)
  {
    // Number of timer ticks in one PWM cycle.
    period_ticks_ =
        pwm_timer_frequency / p_requested_frequency;

    // Report the actual resulting frequency. This remains correct even
    // when the requested frequency does not divide 32 MHz evenly.
    frequency_ =
        pwm_timer_frequency / period_ticks_;

    timer_register(base_, reset_control_offset) =
        reset_command;

    timer_register(base_, power_enable_offset) =
        power_enable_command;

    for (volatile std::uint32_t delay = 0; delay < 32U; ++delay) {
    }

    timer_register(base_, clock_select_offset) =
        select_bus_clock;

    // Divide by one, so the timer receives the complete 32 MHz BUSCLK.
    timer_register(base_, clock_divider_offset) = 0U;

    // The counter visits values 0 through LOAD, so LOAD is period - 1.
    timer_register(base_, load_offset) =
        period_ticks_ - 1U;

    // Configure repeat mode, upward counting, and starting from zero.
    // EN remains clear until start() is called.
    timer_register(base_, counter_control_offset) =
        counter_repeat |
        counter_count_up |
        counter_start_at_zero;
  }

  void start()
  {
    timer_register(base_, counter_control_offset) =
        counter_enable |
        counter_repeat |
        counter_count_up |
        counter_start_at_zero;
  }

  [[nodiscard]] std::uintptr_t base() const
  {
    return base_;
  }

  [[nodiscard]] std::uint32_t frequency() const
  {
    return frequency_;
  }

  [[nodiscard]] std::uint32_t period_ticks() const
  {
    return period_ticks_;
  }

private:
  std::uintptr_t base_;
  std::uint32_t frequency_;
  std::uint32_t period_ticks_;
};

class mspm0_pwm final : public lab2::pwm
{
public:
  mspm0_pwm(mspm0_pwm_timer& p_timer,
            std::uint8_t p_channel,
            std::size_t p_iomux_index,
            std::uint32_t p_iomux_function)
      : timer_(p_timer),
        channel_(p_channel)
  {
    std::uintptr_t const channel_offset =
        channel_ * channel_register_spacing;

    // Route the selected timer channel to the physical LED pin.
    iomux_pin_control(p_iomux_index) =
        peripheral_connected | p_iomux_function;

    // Configure the capture/compare channel as an output.
    timer_register(timer_.base(), ccp_direction_offset) |=
        1U << channel_;

    // Allow the timer output to reach the pin.
    timer_register(timer_.base(), output_disable_offset) &=
        ~(1U << channel_);

    // Compare mode. The reset value of zero selects compare rather
    // than capture.
    timer_register(
        timer_.base(),
        compare_control_0_offset + channel_offset) = 0U;

    // Initial output LOW, no inversion, signal-generator output.
    timer_register(
        timer_.base(),
        output_control_0_offset + channel_offset) = 0U;

    // Begin with a 0% duty cycle.
    duty_cycle(0U);
  }

private:
  std::uint32_t driver_frequency() override
  {
    return timer_.frequency();
  }

  void driver_duty_cycle(std::uint16_t p_duty_cycle) override
  {
    std::uintptr_t const channel_offset =
        channel_ * channel_register_spacing;

    volatile std::uint32_t& compare =
        timer_register(
            timer_.base(),
            compare_0_offset + channel_offset);

    volatile std::uint32_t& action =
        timer_register(
            timer_.base(),
            compare_action_0_offset + channel_offset);

    if (p_duty_cycle == 0U) {
      // Keep the output LOW for the complete cycle.
      action = output_low_at_zero;
      compare = 0U;
      return;
    }

    if (p_duty_cycle == 65'535U) {
      // Keep the output HIGH for the complete cycle.
      action = output_high_at_zero;
      compare = timer_.period_ticks() - 1U;
      return;
    }

    std::uint32_t high_ticks =
        static_cast<std::uint32_t>(
            (static_cast<std::uint64_t>(
                 timer_.period_ticks()) *
             p_duty_cycle) /
            65'535U);

    // Preserve a visible pulse for every nonzero, non-100% value.
    if (high_ticks == 0U) {
      high_ticks = 1U;
    }

    if (high_ticks >= timer_.period_ticks()) {
      high_ticks = timer_.period_ticks() - 1U;
    }

    compare = high_ticks;

    // HIGH at zero, LOW when the upward counter reaches compare.
    action =
        output_high_at_zero |
        output_low_at_compare_up;
  }

  mspm0_pwm_timer& timer_;
  std::uint8_t channel_;
};

class mspm0_steady_clock final : public lab2::steady_clock
{
public:
  mspm0_steady_clock()
  {
    timer_register(timg12_base, reset_control_offset) =
        reset_command;

    timer_register(timg12_base, power_enable_offset) =
        power_enable_command;

    // Give the timer power domain time to become ready.
    for (volatile std::uint32_t delay = 0; delay < 32U; ++delay) {
    }

    timer_register(timg12_base, clock_select_offset) =
        select_bus_clock;

    timer_register(timg12_base, clock_divider_offset) =
        divide_clock_by_8;

    // Count upward through the entire 32-bit range.
    timer_register(timg12_base, load_offset) =
        0xFFFF'FFFFU;

    timer_register(timg12_base, counter_control_offset) =
        counter_enable |
        counter_repeat |
        counter_count_up |
        counter_start_at_zero;
  }

private:
  std::uint32_t driver_frequency() override
  {
    return timer_clock_frequency;
  }

  std::uint64_t driver_uptime() override
  {
    std::uint32_t const current =
        timer_register(timg12_base, counter_offset);

    // Extend the hardware's 32-bit count into a 64-bit uptime.
    if (current < previous_counter_) {
      upper_ticks_ += (1ULL << 32);
    }

    previous_counter_ = current;

    return upper_ticks_ | current;
  }

  std::uint32_t previous_counter_ = 0;
  std::uint64_t upper_ticks_ = 0;
};

}  // namespace


// Test double for lab2::steady_clock. Does not touch real hardware - it just
// increments a counter every time uptime() is called, as if a timer tick had
// elapsed. Useful for exercising delay logic before your real Timer_A-backed
// steady_clock exists.

class fake_steady_clock : public lab2::steady_clock
{
public:
  fake_steady_clock() = default;

private:
  std::uint32_t driver_frequency() override
  {
    // Use a small frequency to reduce the amount of ticks needed
    // for the delay function.
    return 1'000'000;
  }

  std::uint64_t driver_uptime() override
  {
    return m_count++;
  }

  std::uint64_t m_count = 0;
};

int main()
{
  SYSCFG_DL_init();
  std::printf("Hello, World\n");

  mspm0_steady_clock clock;

  constexpr std::uint32_t requested_pwm_frequency = 1'000U;
  constexpr std::uint16_t off = 0U;
  constexpr std::uint16_t full = 65'535U;

  // Red and green share TIMG6's period and frequency.
  mspm0_pwm_timer timer_6{
      timg6_base,
      requested_pwm_frequency,
  };

  // Blue uses TIMG8 but is configured for the same frequency.
  mspm0_pwm_timer timer_8{
      timg8_base,
      requested_pwm_frequency,
  };

  mspm0_pwm red_led{
      timer_6,
      0,   // TIMG6_C0
      56,  // PB26 uses PINCM57
      5,   // IOMUX function 5
  };

  mspm0_pwm green_led{
      timer_6,
      1,   // TIMG6_C1
      57,  // PB27 uses PINCM58
      5,   // IOMUX function 5
  };

  mspm0_pwm blue_led{
      timer_8,
      1,   // TIMG8_C1
      49,  // PB22 uses PINCM50
      3,   // IOMUX function 3
  };

  // Start only after every channel has been configured.
  timer_6.start();
  timer_8.start();

  std::printf(
      "PWM frequency: %" PRIu32 " Hz\n",
      red_led.frequency());

  while (true) {
    // Red → yellow
    transition_color(
        red_led,
        green_led,
        blue_led,
        clock,
        full, off, off,
        full, full, off);

    // Yellow → green
    transition_color(
        red_led,
        green_led,
        blue_led,
        clock,
        full, full, off,
        off, full, off);

    // Green → cyan
    transition_color(
        red_led,
        green_led,
        blue_led,
        clock,
        off, full, off,
        off, full, full);

    // Cyan → blue
    transition_color(
        red_led,
        green_led,
        blue_led,
        clock,
        off, full, full,
        off, off, full);

    // Blue → magenta
    transition_color(
        red_led,
        green_led,
        blue_led,
        clock,
        off, off, full,
        full, off, full);

    // Magenta → red
    transition_color(
        red_led,
        green_led,
        blue_led,
        clock,
        full, off, full,
        full, off, off);
  }
}
