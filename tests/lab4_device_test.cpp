#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>

#include <algorithm>
#include <array>

// Compile the same device implementation used on the LaunchPad.
#include "../lab4_i2c/lab4_i2c.cpp"

namespace {
void no_delay(std::uint32_t)
{
}

class imu_model : public lab4::i2c
{
public:
  std::array<std::uint8_t, 256> registers{};
  bool acknowledge = true;
  bool correct_id = true;
  unsigned sample_reads = 0;

  imu_model()
  {
    registers[0] = 0xD1;
  }

private:
  bool driver_configure(settings const&) override
  {
    return true;
  }
  bool driver_transaction(std::uint8_t p_address,
                          std::span<std::uint8_t const> p_out,
                          std::span<std::uint8_t> p_in) override
  {
    assert(p_address == 0x68 || p_address == 0x69);
    if (!acknowledge) {
      return false;
    }
    assert(!p_out.empty());
    auto const reg = p_out[0];
    if (p_in.empty()) {
      assert(p_out.size() == 2);
      if (reg == 0x7E && p_out[1] == 0xB6) {
        registers.fill(0);
        registers[0] = 0xD1;
      } else if (reg == 0x7E && p_out[1] == 0x11) {
        registers[3] = 0x10;
        registers[0x1B] = 0x80;
      } else {
        registers[reg] = p_out[1];
      }
    } else {
      assert(p_out.size() == 1);
      if (reg == 0x12) {
        assert(p_in.size() == 6);  // A single burst keeps XYZ coherent.
        ++sample_reads;
      }
      std::copy_n(registers.begin() + reg, p_in.size(), p_in.begin());
      if (reg == 0 && !correct_id) {
        p_in[0] = 0;
      }
    }
    return true;
  }
};

void test_imu()
{
  imu_model bus;
  lab4::bmi160 sensor(bus, no_delay);
  assert(std::isnan(sensor.read().x));
  assert(!sensor.last_read_ok());
  assert(sensor.initialize());
  assert(bus.registers[0x40] == 0x28 && bus.registers[0x41] == 0x03);
  std::array<std::uint8_t, 6> const raw{ 0, 0x40, 0, 0xC0, 0, 0x20 };
  std::copy(raw.begin(), raw.end(), bus.registers.begin() + 0x12);
  auto const sample = sensor.read();
  assert(sensor.last_read_ok() && bus.sample_reads == 1);
  assert(sample.x == 1.0F && sample.y == -1.0F && sample.z == 0.5F);
  bus.registers[0x12] = 0;
  bus.registers[0x13] = 0x80;
  assert(sensor.read().x == -2.0F);
  bus.registers[0x12] = 0xFF;
  bus.registers[0x13] = 0x7F;
  assert(sensor.read().x == 32767.0F / 16384.0F);
  bus.acknowledge = false;
  assert(std::isnan(sensor.read().x) && !sensor.last_read_ok());
  assert(!sensor.initialize());
  bus.acknowledge = true;
  bus.correct_id = false;
  assert(!sensor.initialize());
  bus.correct_id = true;
  lab4::bmi160 alternate(bus, no_delay, 0x69);
  assert(alternate.initialize());
  lab4::bmi160 invalid(bus, no_delay, 0x67);
  assert(!invalid.initialize());

  auto tilt = lab4::bmi160::roll_pitch({ 0, 0, 1 });
  assert(tilt.roll == 0 && tilt.pitch == 0);
  tilt = lab4::bmi160::roll_pitch({ 0, 1, 0 });
  assert(std::abs(tilt.roll - 90) < 0.001F && tilt.pitch == 0);
  tilt = lab4::bmi160::roll_pitch({ -1, 0, 1 });
  assert(std::abs(tilt.pitch - 45) < 0.001F);
  tilt = lab4::bmi160::roll_pitch({ 1, 0, 0 });
  assert(std::isnan(tilt.roll) && std::abs(tilt.pitch + 90) < 0.001F);
  tilt = lab4::bmi160::roll_pitch({ 0, 0, 0 });
  assert(std::isnan(tilt.roll) && std::isnan(tilt.pitch));
}
}  // namespace

int main()
{
  test_imu();
  std::puts("Lab 4 device tests passed.");
}
