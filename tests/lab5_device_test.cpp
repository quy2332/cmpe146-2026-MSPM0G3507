#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include <array>

// Build with -DLAB_HOST_TEST; the same conversion and packet code runs on
// the LaunchPads. Drivers and FreeRTOS tasks are compiled out.
#include "../lab5_adc_uart_crc/lab5_adc_uart_crc.cpp"

namespace {
std::uint32_t software_crc(std::uint8_t const* p_data, std::size_t p_size)
{
  crc32_software crc;
  crc.reset();
  crc.update({ p_data, p_size });
  return crc.checksum();
}

void test_upscale()
{
  assert(upscale_12_to_16(0x000) == 0x0000);
  assert(upscale_12_to_16(0xFFF) == 0xFFFF);
  assert(upscale_12_to_16(0x800) == 0x8008);
  for (std::uint16_t code = 0; code < 4096; code++) {
    assert((upscale_12_to_16(code) >> 4) == code);  // exact inverse
  }
}

void test_temperature_conversion()
{
  constexpr std::uint16_t trim = 792;  // TEMP_SENSE0 read from both boards
  // The factory code is 30.00 degC by definition.
  assert(to_temperature(upscale_12_to_16(trim), trim) == 3000);
  // One 12-bit code at 3.3 V is 0.806 mV; at -1.8 mV/degC that is 0.448
  // degC, and a higher code means a colder die.
  assert(to_temperature(upscale_12_to_16(trim + 1), trim) == 2955);
  assert(to_temperature(upscale_12_to_16(trim - 1), trim) == 3045);
  // Code 806 is what the boards read on the desk: about 23.7 degC.
  assert(to_temperature(upscale_12_to_16(806), trim) == 2373);
  // Resolution requirement: one code step is finer than 0.5 degC.
  for (std::uint16_t code = 600; code < 1000; code++) {
    auto const step = to_temperature(upscale_12_to_16(code), trim) -
                      to_temperature(upscale_12_to_16(code + 1), trim);
    assert(step > 0 && step < 50);
  }
  // Out-of-range codes saturate instead of wrapping.
  assert(to_temperature(0, 4095) == 32767);
}

void test_rounding_and_average()
{
  assert(divide_rounded(5, 2) == 3);
  assert(divide_rounded(-5, 2) == -3);
  assert(divide_rounded(5, -2) == -3);
  assert(divide_rounded(4, 3) == 1);
  sample_batch samples{};
  samples.fill(2300);
  samples[0] = 2364;  // +64 total -> average +1
  assert(average_temperature(samples) == 2301);
}

void test_packet_round_trip()
{
  sample_batch samples{};
  for (std::size_t index = 0; index < samples.size(); index++) {
    samples[index] =
      static_cast<temperature_sample_t>(static_cast<int>(index) * 517 - 16000);
  }
  auto const payload = encode_samples(samples);
  assert(payload[0] == 0x80 && payload[1] == 0xC1);  // -16000 little-endian
  assert(decode_samples(payload) == samples);

  auto const crc = software_crc(payload.data(), payload.size());
  auto const packet = build_response(payload, crc);
  assert(packet.size() == 133);
  assert(packet[0] == response_marker);
  assert(std::memcmp(packet.data() + 1, payload.data(), payload.size()) == 0);
  assert(load_u32(packet.data() + 1 + payload_size) == crc);
}

void test_crc_detects_every_single_bit_flip()
{
  constexpr std::array<std::uint8_t, 9> check{ '1', '2', '3', '4', '5',
                                               '6', '7', '8', '9' };
  assert(software_crc(check.data(), check.size()) == 0xCBF4'3926U);

  sample_payload payload{};
  for (std::size_t index = 0; index < payload.size(); index++) {
    payload[index] = static_cast<std::uint8_t>(index * 37 + 11);
  }
  auto const good = software_crc(payload.data(), payload.size());
  for (std::size_t bit = 0; bit < payload.size() * 8; bit++) {
    auto corrupted = payload;
    corrupted[bit / 8] ^= static_cast<std::uint8_t>(1U << (bit % 8));
    assert(software_crc(corrupted.data(), corrupted.size()) != good);
  }
}

void test_colors_and_text()
{
  assert(color_for(1599) == led_color::blue);
  assert(color_for(1600) == led_color::yellow);
  assert(color_for(2499) == led_color::yellow);
  assert(color_for(2500) == led_color::red);
  assert(std::strcmp(color_name(led_color::magenta), "magenta (CRC error)") ==
         0);
  assert(std::strcmp(format_centi_degrees(2373).text.data(), "23.73") == 0);
  assert(std::strcmp(format_centi_degrees(-5).text.data(), "-0.05") == 0);
}
}  // namespace

int main()
{
  test_upscale();
  test_temperature_conversion();
  test_rounding_and_average();
  test_packet_round_trip();
  test_crc_detects_every_single_bit_flip();
  test_colors_and_text();
  std::puts("Lab 5 device tests passed.");
}
