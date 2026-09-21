#include <cstddef>
#include <cstdint>
#include <cstdio>

#include "ti_msp_dl_config.h"

#include "../hal/gpio.hpp"

namespace {

struct gpio_register_map
{
  std::uint32_t reserved0[512];              // 0x0000–0x07FC
  volatile std::uint32_t power_enable;       // 0x0800
  volatile std::uint32_t reset_control;      // 0x0804
  std::uint32_t reserved1[3];                // 0x0808–0x0810
  volatile std::uint32_t status;             // 0x0814
  std::uint32_t reserved2[666];              // 0x0818–0x127C
  volatile std::uint32_t output;             // 0x1280
  std::uint32_t reserved3[3];
  volatile std::uint32_t output_set;         // 0x1290
  std::uint32_t reserved4[3];
  volatile std::uint32_t output_clear;       // 0x12A0
  std::uint32_t reserved5[7];
  volatile std::uint32_t output_enable;      // 0x12C0
  std::uint32_t reserved6[3];
  volatile std::uint32_t output_enable_set;  // 0x12D0
  std::uint32_t reserved7[3];
  volatile std::uint32_t output_enable_clear; // 0x12E0
  std::uint32_t reserved8[39];
  volatile std::uint32_t input;              // 0x1380
};

struct iomux_register_map
{
  std::uint32_t reserved;
  volatile std::uint32_t pin_cm[251];
};

static_assert(offsetof(gpio_register_map, power_enable) == 0x0800);
static_assert(offsetof(gpio_register_map, output) == 0x1280);
static_assert(offsetof(gpio_register_map, output_set) == 0x1290);
static_assert(offsetof(gpio_register_map, output_clear) == 0x12A0);
static_assert(offsetof(gpio_register_map, output_enable) == 0x12C0);
static_assert(offsetof(gpio_register_map, input) == 0x1380);
static_assert(offsetof(iomux_register_map, pin_cm) == 0x0004);

constexpr std::uintptr_t gpio_a_base = 0x400A'0000;
constexpr std::uintptr_t gpio_b_base = 0x400A'2000;
constexpr std::uintptr_t iomux_base  = 0x4042'8000;

gpio_register_map* const gpio_a =
    reinterpret_cast<gpio_register_map*>(gpio_a_base);

gpio_register_map* const gpio_b =
    reinterpret_cast<gpio_register_map*>(gpio_b_base);

iomux_register_map* const iomux =
    reinterpret_cast<iomux_register_map*>(iomux_base);

constexpr std::uint32_t gpio_function        = 1U;
constexpr std::uint32_t peripheral_connected = 1U << 7;
constexpr std::uint32_t pull_down_enable     = 1U << 16;
constexpr std::uint32_t pull_up_enable       = 1U << 17;
constexpr std::uint32_t input_enable         = 1U << 18;
constexpr std::uint32_t open_drain_enable    = 1U << 25;

[[nodiscard]] constexpr std::uint32_t resistor_bits(
    lab1::pin_resistor p_resistor)
{
  switch (p_resistor) {
    case lab1::pin_resistor::none:
      return 0U;

    case lab1::pin_resistor::pull_down:
      return pull_down_enable;

    case lab1::pin_resistor::pull_up:
      return pull_up_enable;
  }

  return 0U;
}

class mspm0_input_pin final : public lab1::input_pin
{
public:
  mspm0_input_pin(gpio_register_map* p_port,
                  std::uint8_t p_pin,
                  std::size_t p_iomux_index)
      : port_(p_port),
        pin_mask_(1U << p_pin),
        iomux_index_(p_iomux_index)
  {
  }

private:
  bool driver_configure(settings const& p_settings) override
  {
    // Ensure this pin is not driving an output.
    port_->output_enable_clear = pin_mask_;

    iomux->pin_cm[iomux_index_] =
        peripheral_connected |
        gpio_function |
        input_enable |
        resistor_bits(p_settings.resistor);

    return true;
  }

  bool driver_level() override
  {
    return (port_->input & pin_mask_) != 0U;
  }

  gpio_register_map* port_;
  std::uint32_t pin_mask_;
  std::size_t iomux_index_;
};

class mspm0_output_pin final : public lab1::output_pin
{
public:
  mspm0_output_pin(gpio_register_map* p_port,
                   std::uint8_t p_pin,
                   std::size_t p_iomux_index)
      : port_(p_port),
        pin_mask_(1U << p_pin),
        iomux_index_(p_iomux_index)
  {
  }

private:
  bool driver_configure(settings const& p_settings) override
  {
    std::uint32_t configuration =
        peripheral_connected |
        gpio_function |
        input_enable |
        resistor_bits(p_settings.resistor);

    if (p_settings.open_drain) {
      configuration |= open_drain_enable;
    }

    // Establish a known LOW state before enabling the output.
    port_->output_clear = pin_mask_;

    iomux->pin_cm[iomux_index_] = configuration;
    port_->output_enable_set = pin_mask_;

    return true;
  }

  void driver_level(bool p_high) override
  {
    if (p_high) {
      port_->output_set = pin_mask_;
    } else {
      port_->output_clear = pin_mask_;
    }
  }

  bool driver_level() override
  {
    // Read the physical pin voltage, not a cached software value.
    return (port_->input & pin_mask_) != 0U;
  }

  gpio_register_map* port_;
  std::uint32_t pin_mask_;
  std::size_t iomux_index_;
};

void initialize_gpio()
{
  constexpr std::uint32_t reset_command = 0xB100'0003;
  constexpr std::uint32_t power_enable_command = 0x2600'0001;

  gpio_a->reset_control = reset_command;
  gpio_b->reset_control = reset_command;

  gpio_a->power_enable = power_enable_command;
  gpio_b->power_enable = power_enable_command;

  // Allow time for the peripheral power to stabilize.
  for (volatile std::uint32_t delay = 0; delay < 32U; ++delay) {
  }
}

}  // namespace

int main()
{

  SYSCFG_DL_init();
  std::printf("Hello, World\n");

  initialize_gpio();

  // RGB LED2 outputs
  mspm0_output_pin blue_led{
      gpio_b,
      22,
      49,  // PB22 uses PINCM50
  };

  mspm0_output_pin red_led{
      gpio_b,
      26,
      56,  // PB26 uses PINCM57
  };

  mspm0_output_pin green_led{
      gpio_b,
      27,
      57,  // PB27 uses PINCM58
  };

  // Push-button inputs
  mspm0_input_pin switch_1{
      gpio_a,
      18,
      39,  // PA18 uses PINCM40
  };

  mspm0_input_pin switch_2{
      gpio_b,
      21,
      48,  // PB21 uses PINCM49
  };

  lab1::output_pin::settings output_settings{};

  // S1 has an external pull-down and reads HIGH when pressed.
  lab1::input_pin::settings switch_1_settings{
      .resistor = lab1::pin_resistor::pull_down,
  };

  // S2 reads LOW when pressed, so it uses a pull-up.
  lab1::input_pin::settings switch_2_settings{
      .resistor = lab1::pin_resistor::pull_up,
  };

  blue_led.configure(output_settings);
  red_led.configure(output_settings);
  green_led.configure(output_settings);

  switch_1.configure(switch_1_settings);
  switch_2.configure(switch_2_settings);

  while (true) {
    bool const switch_1_pressed = switch_1.level();
    bool const switch_2_pressed = !switch_2.level();

    if (switch_1_pressed && switch_2_pressed) {
      // Both switches: blue
      red_led.level(true);
      green_led.level(true);
      blue_led.level(true);
    } else if (switch_1_pressed) {
      // S1 only: red
      red_led.level(true);
      green_led.level(false);
      blue_led.level(false);
    } else if (switch_2_pressed) {
      // S2 only: green
      red_led.level(false);
      green_led.level(true);
      blue_led.level(false);
    } else {
      // Neither switch: LED off
      red_led.level(false);
      green_led.level(false);
      blue_led.level(false);
    }
  }
}