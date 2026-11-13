#include <cassert>
#include <cstdint>
#include <cstdio>

#include <algorithm>
#include <array>
#include <vector>

// Compile the same device implementation used on the LaunchPad.
#include "../lab3_spi/lab3_spi.cpp"

namespace {
void no_delay(std::uint32_t)
{
}

// Model NOR constraints: separate write-enable command, erase granularity,
// page boundaries, 1->0 programming, and busy status after mutations.
class flash_model
  : public lab3::spi
  , public lab1::output_pin
{
public:
  std::array<std::uint8_t, 8192> memory{};
  std::array<std::uint8_t, 3> id{ 0xEF, 0x40, 0x16 };
  bool stuck_busy = false;
  bool deny_write = false;
  bool drop_program = false;
  unsigned erases = 0;
  unsigned status_reads = 0;

private:
  bool driver_configure(lab3::spi::settings const&) override
  {
    return true;
  }
  bool driver_configure(lab1::output_pin::settings const&) override
  {
    return true;
  }
  bool driver_level() override
  {
    return !m_selected;
  }
  std::size_t address() const
  {
    assert(m_frame.size() >= 4);
    return (std::size_t{ m_frame[1] } << 16) |
           (std::size_t{ m_frame[2] } << 8) | m_frame[3];
  }
  void driver_level(bool p_high) override
  {
    if (!p_high) {
      assert(!m_selected);
      m_selected = true;
      m_frame.clear();
      return;
    }
    if (!m_selected) {
      return;  // Constructor sets CS high before the first transaction.
    }
    m_selected = false;
    assert(!m_frame.empty());
    if (m_frame[0] == 0x06) {
      assert(m_frame.size() == 1);
      m_write_enabled = !deny_write;
    } else if (m_frame[0] == 0x20) {
      assert(m_write_enabled && m_frame.size() == 4);
      auto const start = address();
      assert(start % 4096 == 0 && start + 4096 <= memory.size());
      std::fill_n(memory.begin() + start, 4096, 0xFF);
      ++erases;
      m_write_enabled = false;
      m_busy_polls = 2;
    } else if (m_frame[0] == 0x02) {
      assert(m_write_enabled && m_frame.size() > 4);
      auto const start = address();
      auto const length = m_frame.size() - 4;
      assert(start / 256 == (start + length - 1) / 256);
      assert(start + length <= memory.size());
      if (!drop_program) {
        for (std::size_t index = 0; index < length; ++index) {
          memory[start + index] &= m_frame[index + 4];
        }
      }
      m_write_enabled = false;
      m_busy_polls = 1;
    }
  }
  void driver_transfer(std::span<std::uint8_t const> p_out,
                       std::span<std::uint8_t> p_in,
                       std::uint8_t p_filler) override
  {
    assert(m_selected);
    auto const length = std::max(p_out.size(), p_in.size());
    for (std::size_t index = 0; index < length; ++index) {
      auto const position = m_frame.size();
      std::uint8_t result = 0xFF;
      if (position != 0) {
        if (m_frame[0] == 0x9F && position <= id.size()) {
          result = id[position - 1];
        } else if (m_frame[0] == 0x05) {
          ++status_reads;
          result = static_cast<std::uint8_t>(
            (stuck_busy || m_busy_polls != 0 ? 0x01 : 0) |
            (m_write_enabled ? 0x02 : 0));
          if (m_busy_polls != 0) {
            --m_busy_polls;
          }
        } else if (m_frame[0] == 0x03 && position >= 4) {
          result = memory.at(address() + position - 4);
        }
      }
      m_frame.push_back(index < p_out.size() ? p_out[index] : p_filler);
      if (index < p_in.size()) {
        p_in[index] = result;
      }
    }
  }
  bool m_selected = false;
  bool m_write_enabled = false;
  unsigned m_busy_polls = 0;
  std::vector<std::uint8_t> m_frame;
};

void test_flash()
{
  flash_model model;
  for (std::size_t index = 0; index < model.memory.size(); ++index) {
    model.memory[index] = static_cast<std::uint8_t>(index * 37);
  }
  auto const original = model.memory;
  lab3::flash_memory flash(model, model, no_delay);
  assert(flash.initialize());
  assert(flash.manufacturer_id() == 0xEF);
  assert(flash.capacity_bytes() == 4 * 1024 * 1024);
  lab3::flash_memory::block block{};
  assert(flash.read_first_block(block));
  assert(std::equal(block.begin(), block.end(), original.begin()));
  block.fill(0);
  assert(flash.write_first_block(block));
  block.fill(0xFF);  // Must erase to restore zero bits to ones.
  assert(flash.write_first_block(block));
  assert(model.erases == 2);
  assert(std::equal(block.begin(), block.end(), model.memory.begin()));
  assert(std::equal(
    original.begin() + 32, original.end(), model.memory.begin() + 32));
  std::array<std::uint8_t, 3> const short_write{ 1, 2, 3 };
  assert(flash.write_first_block(short_write));
  assert(model.memory[3] == 0xFF && model.memory[31] == 0xFF);
  auto const erases = model.erases;
  assert(flash.write_first_block(short_write));
  assert(flash.write_first_block({}));
  std::array<std::uint8_t, 33> oversized{};
  assert(!flash.write_first_block(oversized));
  assert(model.erases == erases);
  model.deny_write = true;
  assert(!flash.write_first_block(block));
  assert(model.erases == erases);
  model.deny_write = false;
  model.drop_program = true;
  block.fill(0x5A);
  assert(
    !flash.write_first_block(block));  // Verify detects failed programming.

  for (auto const capacity : { 0x00, 0x19, 0xFF }) {
    model.id[2] = static_cast<std::uint8_t>(capacity);
    assert(!flash.initialize());
    assert(flash.capacity_bytes() == 0);
  }
  model.stuck_busy = true;
  auto const reads = model.status_reads;
  assert(!flash.initialize());
  assert(model.status_reads - reads == 1000);
}

}  // namespace

int main()
{
  test_flash();
  std::puts("Lab 3 device tests passed.");
}
