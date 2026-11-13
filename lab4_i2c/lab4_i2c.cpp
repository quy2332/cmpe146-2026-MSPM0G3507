#include <cstddef>
#include <cstdint>
#include <cstdio>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#include "../hal/accelerometer.hpp"
#include "../hal/i2c.hpp"

#if !defined(LAB_HOST_TEST)
#include <ti/driverlib/dl_gpio.h>
#include <ti/driverlib/dl_i2c.h>
#include <ti/driverlib/m0p/dl_core.h>

#include "ti_msp_dl_config.h"
#endif

namespace lab4 {

/** @brief BMI160 accelerometer with a +/-2 g range and 100 Hz sample rate. */
class bmi160 final : public accelerometer
{
public:
  /// Delay callback accepts milliseconds.
  using delay_function = void (*)(std::uint32_t);
  /** @brief Tilt angles in degrees in the sensor's coordinate system. */
  struct angles
  {
    float roll;
    float pitch;
  };

  /** @brief Bind a bus, a delay function, and address 0x68 or 0x69. */
  bmi160(i2c& p_bus, delay_function p_delay, std::uint8_t p_address = 0x68);
  /** @brief Check chip identity, configure the range, and start measurement. */
  [[nodiscard]] bool initialize();
  /** @brief Whether the latest read succeeded; failed reads return NaNs. */
  [[nodiscard]] bool last_read_ok() const;
  /**
   * @brief Calculate roll/pitch from gravity; no yaw information is available.
   *
   * Assumes a stationary or slowly moving sensor. A zero or non-finite vector
   * produces NaN angles. Roll is undefined when gravity is exactly on X.
   */
  [[nodiscard]] static angles roll_pitch(read_t const& p_acceleration);

private:
  bool write_register(std::uint8_t p_register, std::uint8_t p_value);
  bool read_registers(std::uint8_t p_register, std::span<std::uint8_t> p_data);
  read_t driver_read() override;
  static float decode_axis(std::uint8_t p_low, std::uint8_t p_high);

  i2c& m_bus;
  delay_function m_delay;
  std::uint8_t m_address;
  // Two state flags use a single byte; bit 0 = initialized, bit 1 = good read.
  std::uint8_t m_state = 0;
};

bmi160::bmi160(i2c& p_bus, delay_function p_delay, std::uint8_t p_address)
  : m_bus(p_bus)
  , m_delay(p_delay)
  , m_address(p_address)
{
}

bool bmi160::write_register(std::uint8_t p_register, std::uint8_t p_value)
{
  std::array<std::uint8_t, 2> const data{ p_register, p_value };
  return m_bus.transaction(m_address, data, {});
}

bool bmi160::read_registers(std::uint8_t p_register,
                            std::span<std::uint8_t> p_data)
{
  // A combined transaction holds the bus for write-address + repeated START.
  return m_bus.transaction(m_address, { &p_register, 1 }, p_data);
}

bool bmi160::initialize()
{
  m_state = 0;
  if (m_address != 0x68 && m_address != 0x69) {
    return false;
  }
  m_delay(10);
  std::uint8_t value = 0;
  if (!read_registers(0x00, { &value, 1 }) || value != 0xD1) {
    return false;
  }
  if (!write_register(0x7E, 0xB6)) {  // Soft reset to a known configuration.
    return false;
  }
  m_delay(10);
  if (!write_register(0x40, 0x28)) {  // ACC_CONF: normal filter, 100 Hz.
    return false;
  }
  // In suspend mode allow at least 450 us between register writes.
  m_delay(1);
  if (!write_register(0x41, 0x03)) {  // ACC_RANGE: +/-2 g, 16384 LSB/g.
    return false;
  }
  m_delay(1);
  if (!write_register(0x7E, 0x11)) {  // Accelerometer normal power mode.
    return false;
  }
  m_delay(5);  // Mode transition can take 3.8 ms.
  constexpr std::uint8_t accelerometer_power = 0x30;
  constexpr std::uint8_t normal_power = 0x10;
  if (!read_registers(0x03, { &value, 1 }) ||
      (value & accelerometer_power) != normal_power) {
    return false;
  }
  constexpr std::uint8_t fatal_or_configuration_error = 0x1F;
  if (!read_registers(0x02, { &value, 1 }) ||
      (value & fatal_or_configuration_error) != 0) {
    return false;
  }
  constexpr std::uint8_t data_ready = 0x80;
  for (unsigned attempt = 0; attempt < 100; ++attempt) {
    if (!read_registers(0x1B, { &value, 1 })) {
      return false;
    }
    if ((value & data_ready) != 0) {
      m_state = 0x01;
      return true;
    }
    m_delay(1);
  }
  return false;
}

bool bmi160::last_read_ok() const
{
  constexpr std::uint8_t good_read = 0x02;
  return (m_state & good_read) != 0;
}

float bmi160::decode_axis(std::uint8_t p_low, std::uint8_t p_high)
{
  // Concatenate little-endian bytes, then explicitly sign-extend 16 bits.
  auto value =
    static_cast<std::int32_t>((std::uint32_t{ p_high } << 8) | p_low);
  if (value >= 0x8000) {
    value -= 0x1'0000;
  }
  return static_cast<float>(value) / 16384.0F;
}

accelerometer::read_t bmi160::driver_read()
{
  constexpr std::uint8_t initialized = 0x01;
  m_state &= initialized;
  std::array<std::uint8_t, 6> data{};
  if (m_state == 0 || !read_registers(0x12, data)) {
    auto const invalid = std::numeric_limits<float>::quiet_NaN();
    return { invalid, invalid, invalid };
  }
  m_state = 0x03;
  return { decode_axis(data[0], data[1]),
           decode_axis(data[2], data[3]),
           decode_axis(data[4], data[5]) };
}

bmi160::angles bmi160::roll_pitch(read_t const& p_acceleration)
{
  auto const x = p_acceleration.x;
  auto const y = p_acceleration.y;
  auto const z = p_acceleration.z;
  auto const invalid = std::numeric_limits<float>::quiet_NaN();
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
      x * x + y * y + z * z < 1.0e-12F) {
    return { invalid, invalid };
  }
  constexpr float degrees_per_radian = 57.295779513F;
  auto const yz_length = std::sqrt(y * y + z * z);
  return { yz_length < 1.0e-6F ? invalid
                               : std::atan2(y, z) * degrees_per_radian,
           std::atan2(-x, yz_length) * degrees_per_radian };
}

#if !defined(LAB_HOST_TEST)
namespace {

static_assert(CPUCLK_FREQ == 32'000'000, "Update bus timing for this clock");
constexpr std::uint32_t bus_clock = CPUCLK_FREQ;
constexpr std::uint32_t poll_limit = 1'000'000;
constexpr std::uint32_t transfer_errors =
  DL_I2C_INTERRUPT_CONTROLLER_NACK |
  DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST;
constexpr std::uint32_t transfer_events = transfer_errors |
                                          DL_I2C_INTERRUPT_CONTROLLER_TX_DONE |
                                          DL_I2C_INTERRUPT_CONTROLLER_RX_DONE;

void delay_ms(std::uint32_t p_milliseconds)
{
  while (p_milliseconds-- != 0) {
    delay_cycles(bus_clock / 1000);
  }
}

}  // namespace

/** @brief Polling DriverLib I2C1 controller on PB2 (SCL) and PB3 (SDA). */
class mspm0_i2c final : public i2c
{
public:
  /** @brief Connect the open-drain I2C pins; external pull-ups are required. */
  mspm0_i2c();
  /** @brief Return the actual configured SCL frequency in hertz. */
  [[nodiscard]] std::uint32_t clock_rate() const;

private:
  bool driver_configure(settings const& p_settings) override;
  bool driver_transaction(std::uint8_t p_address,
                          std::span<std::uint8_t const> p_data_out,
                          std::span<std::uint8_t> p_data_in) override;
  bool transfer(std::uint8_t p_address,
                std::span<std::uint8_t const> p_data_out,
                std::span<std::uint8_t> p_data_in,
                DL_I2C_CONTROLLER_STOP p_stop);
  bool wait_idle();
  void reset_controller();

  std::uint32_t m_clock_rate = 0;
  std::uint32_t m_divider = 1;
  std::uint8_t m_period = 31;
};

mspm0_i2c::mspm0_i2c()
{
  DL_GPIO_initPeripheralInputFunction(IOMUX_PINCM15, IOMUX_PINCM15_PF_I2C1_SCL);
  DL_GPIO_initPeripheralInputFunction(IOMUX_PINCM16, IOMUX_PINCM16_PF_I2C1_SDA);
  DL_GPIO_enableHiZ(IOMUX_PINCM15);
  DL_GPIO_enableHiZ(IOMUX_PINCM16);
}

std::uint32_t mspm0_i2c::clock_rate() const
{
  return m_clock_rate;
}

void mspm0_i2c::reset_controller()
{
  DL_I2C_reset(I2C1);
  DL_I2C_enablePower(I2C1);
  delay_cycles(32);
  DL_I2C_ClockConfig const clock{
    DL_I2C_CLOCK_BUSCLK, static_cast<DL_I2C_CLOCK_DIVIDE>(m_divider - 1)
  };
  DL_I2C_setClockConfig(I2C1, &clock);
  DL_I2C_setTimerPeriod(I2C1, m_period);
  DL_I2C_enableController(I2C1);
  DL_I2C_clearInterruptStatus(I2C1, transfer_events);
}

bool mspm0_i2c::driver_configure(settings const& p_settings)
{
  if (p_settings.clock_rate == 0 || p_settings.clock_rate > 400'000) {
    return false;
  }
  std::uint32_t best_divisor = 0;
  for (std::uint32_t divider = 1; divider <= 8; ++divider) {
    auto const denominator =
      std::uint64_t{ 10 } * divider * p_settings.clock_rate;
    auto const count = (bus_clock + denominator - 1) / denominator;
    auto const divisor =
      static_cast<std::uint32_t>(std::uint64_t{ 10 } * divider * count);
    if (count >= 2 && count <= 128 &&
        (best_divisor == 0 || divisor < best_divisor)) {
      m_divider = divider;
      m_period = static_cast<std::uint8_t>(count - 1);
      best_divisor = divisor;
    }
  }
  if (best_divisor == 0) {
    return false;
  }
  m_clock_rate = bus_clock / best_divisor;
  reset_controller();
  return true;
}

bool mspm0_i2c::wait_idle()
{
  for (std::uint32_t attempt = 0; attempt < poll_limit; ++attempt) {
    auto const status = DL_I2C_getControllerStatus(I2C1);
    if ((status & DL_I2C_CONTROLLER_STATUS_IDLE) != 0 &&
        (status & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) == 0) {
      return true;
    }
  }
  return false;
}

bool mspm0_i2c::transfer(std::uint8_t p_address,
                         std::span<std::uint8_t const> p_data_out,
                         std::span<std::uint8_t> p_data_in,
                         DL_I2C_CONTROLLER_STOP p_stop)
{
  auto const receiving = p_data_out.empty();
  auto const length = receiving ? p_data_in.size() : p_data_out.size();
  auto const done = receiving ? DL_I2C_INTERRUPT_CONTROLLER_RX_DONE
                              : DL_I2C_INTERRUPT_CONTROLLER_TX_DONE;
  auto const direction =
    receiving ? DL_I2C_CONTROLLER_DIRECTION_RX : DL_I2C_CONTROLLER_DIRECTION_TX;
  DL_I2C_clearInterruptStatus(I2C1, transfer_events);
  std::size_t count = 0;
  if (!receiving) {
    count = DL_I2C_fillControllerTXFIFO(
      I2C1, p_data_out.data(), static_cast<std::uint16_t>(length));
  }
  DL_I2C_startControllerTransferAdvanced(I2C1,
                                         p_address,
                                         direction,
                                         static_cast<std::uint16_t>(length),
                                         DL_I2C_CONTROLLER_START_ENABLE,
                                         p_stop,
                                         DL_I2C_CONTROLLER_ACK_DISABLE);
  // I2C_ERR_13: status needs at least three peripheral clock cycles to settle.
  delay_cycles(3 * m_divider);
  std::uint32_t stalled = 0;
  while (stalled < poll_limit) {
    auto const events = DL_I2C_getRawInterruptStatus(I2C1, transfer_events);
    if ((events & transfer_errors) != 0 ||
        (DL_I2C_getControllerStatus(I2C1) & DL_I2C_CONTROLLER_STATUS_ERROR) !=
          0) {
      return false;
    }
    auto const before = count;
    if (receiving) {
      while (count < length && !DL_I2C_isControllerRXFIFOEmpty(I2C1)) {
        p_data_in[count++] = DL_I2C_receiveControllerData(I2C1);
      }
    } else if (count < length) {
      count +=
        DL_I2C_fillControllerTXFIFO(I2C1,
                                    p_data_out.data() + count,
                                    static_cast<std::uint16_t>(length - count));
    }
    if ((events & done) != 0) {
      return count == length;
    }
    stalled = before == count ? stalled + 1 : 0;
  }
  return false;
}

bool mspm0_i2c::driver_transaction(std::uint8_t p_address,
                                   std::span<std::uint8_t const> p_data_out,
                                   std::span<std::uint8_t> p_data_in)
{
  if (m_clock_rate == 0 || p_address > 0x7F || p_data_out.size() > 0xFFF ||
      p_data_in.size() > 0xFFF) {
    return false;
  }
  if (p_data_out.empty() && p_data_in.empty()) {
    return true;
  }
  bool success = wait_idle();
  if (success) {
    DL_I2C_flushControllerTXFIFO(I2C1);
    DL_I2C_flushControllerRXFIFO(I2C1);
    if (!p_data_out.empty()) {
      auto const stop = p_data_in.empty() ? DL_I2C_CONTROLLER_STOP_ENABLE
                                          : DL_I2C_CONTROLLER_STOP_DISABLE;
      success = transfer(p_address, p_data_out, {}, stop);
    }
    if (success && !p_data_in.empty()) {
      // Do not wait for bus-idle here: we own it until the repeated START.
      success =
        transfer(p_address, {}, p_data_in, DL_I2C_CONTROLLER_STOP_ENABLE);
    }
    success = success && wait_idle();
  }
  if (!success) {
    // Release controller state/FIFOs for a retry after NACK or timeout.
    // A target holding SDA low may still require a physical power cycle.
    reset_controller();
  }
  return success;
}
#endif
}  // namespace lab4

#if !defined(LAB_HOST_TEST)
int main()
{
  SYSCFG_DL_init();
  lab4::mspm0_i2c bus;
  if (!bus.configure({ 100'000 })) {
    std::puts("Unsupported I2C clock settings.");
    while (true) {
      __WFI();
    }
  }
  std::printf("I2C: %lu Hz\n", static_cast<unsigned long>(bus.clock_rate()));
  // SDO grounded selects 0x68. Change to 0x69 if SDO is tied to 3.3 V.
  lab4::bmi160 sensor(bus, lab4::delay_ms, 0x68);
  while (!sensor.initialize()) {
    std::puts("BMI160 initialization failed: check 0x68 address, CSB, wiring,"
              " and pull-ups. Retrying...");
    lab4::delay_ms(1000);
  }
  while (true) {
    auto const acceleration = sensor.read();
    if (sensor.last_read_ok()) {
      auto const tilt = lab4::bmi160::roll_pitch(acceleration);
      std::printf("X=%+.3f g Y=%+.3f g Z=%+.3f g  roll=%+.1f pitch=%+.1f deg\n",
                  static_cast<double>(acceleration.x),
                  static_cast<double>(acceleration.y),
                  static_cast<double>(acceleration.z),
                  static_cast<double>(tilt.roll),
                  static_cast<double>(tilt.pitch));
    } else {
      std::puts("BMI160 read failed; check the I2C connection.");
    }
    lab4::delay_ms(100);
  }
}
#endif
