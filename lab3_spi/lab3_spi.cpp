#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "ti_msp_dl_config.h"

#include "../hal/gpio.hpp"
#include "../hal/spi.hpp"

/// GPIO PA8, used only as an active-low flash chip select.
class chip_select : public lab1::output_pin
{
private:
  bool driver_configure(settings const& p_settings) override
  {
    if (p_settings.open_drain ||
        p_settings.resistor != lab1::pin_resistor::none) {
      return false;
    }
    // SYSCFG_DL_init() has already powered GPIOA. Start deselected.
    GPIOA->DOUTSET31_0 = pin_mask;
    IOMUX->SECCFG.PINCM[IOMUX_PINCM19] = IOMUX_PINCM19_PF_GPIOA_DIO08 |
                                         IOMUX_PINCM_PC_CONNECTED |
                                         IOMUX_PINCM_INENA_ENABLE;
    GPIOA->DOESET31_0 = pin_mask;
    return true;
  }

  void driver_level(bool p_high) override
  {
    if (p_high) {
      GPIOA->DOUTSET31_0 = pin_mask;
    } else {
      GPIOA->DOUTCLR31_0 = pin_mask;
    }
  }

  bool driver_level() override
  {
    return (GPIOA->DIN31_0 & pin_mask) != 0;
  }

  static constexpr std::uint32_t pin_mask = 0x100;
};

/// Register-level SPI0: PA12 clock, PA13 input, PA14 output; GPIO owns CS.
class spi_driver : public lab3::spi
{
private:
  bool driver_configure(settings const& p_settings) override
  {
    if (p_settings.clock_rate == 0 || p_settings.bus_mode > mode::m3) {
      return false;
    }
    // SCLK = SYSCLK / (2 * (SCR + 1)). Round down to the requested rate.
    constexpr auto half_clock = CPUCLK_FREQ / 2;
    auto divider = half_clock / p_settings.clock_rate +
                   (half_clock % p_settings.clock_rate != 0);
    if (divider > 1024) {
      return false;
    }

    SPI0->GPRCM.RSTCTL = SPI_RSTCTL_KEY_UNLOCK_W | SPI_RSTCTL_RESETSTKYCLR_CLR |
                         SPI_RSTCTL_RESETASSERT_ASSERT;
    SPI0->GPRCM.PWREN = SPI_PWREN_KEY_UNLOCK_W | SPI_PWREN_ENABLE_ENABLE;
    for (unsigned count = 0; count < 16; ++count) {
      __NOP();
    }
    SPI0->CLKSEL = SPI_CLKSEL_SYSCLK_SEL_ENABLE;
    SPI0->CLKDIV = SPI_CLKDIV_RATIO_DIV_BY_1;
    SPI0->CLKCTL = divider - 1;
    SPI0->CTL0 =
      SPI_CTL0_DSS_DSS_8 | SPI_CTL0_FRF_MOTOROLA_3WIRE |
      (p_settings.bus_mode >= mode::m2 ? SPI_CTL0_SPO_HIGH : 0) |
      (p_settings.bus_mode == mode::m1 || p_settings.bus_mode == mode::m3
         ? SPI_CTL0_SPH_SECOND
         : 0);
    IOMUX->SECCFG.PINCM[IOMUX_PINCM34] =
      IOMUX_PINCM34_PF_SPI0_SCLK | IOMUX_PINCM_PC_CONNECTED;
    IOMUX->SECCFG.PINCM[IOMUX_PINCM35] =
      IOMUX_PINCM35_PF_SPI0_POCI | IOMUX_PINCM_PC_CONNECTED |
      IOMUX_PINCM_INENA_ENABLE | IOMUX_PINCM_PIPU_ENABLE;
    IOMUX->SECCFG.PINCM[IOMUX_PINCM36] =
      IOMUX_PINCM36_PF_SPI0_PICO | IOMUX_PINCM_PC_CONNECTED;
    SPI0->CTL1 =
      SPI_CTL1_CP_ENABLE | SPI_CTL1_MSB_ENABLE | SPI_CTL1_ENABLE_ENABLE;
    return true;
  }

  void driver_transfer(std::span<std::uint8_t const> p_data_out,
                       std::span<std::uint8_t> p_data_in,
                       std::uint8_t p_filler) override
  {
    for (std::size_t index = 0;
         index < p_data_out.size() || index < p_data_in.size();
         ++index) {
      while ((SPI0->STAT & SPI_STAT_TNF_MASK) == 0) {
      }
      SPI0->TXDATA = index < p_data_out.size() ? p_data_out[index] : p_filler;
      while ((SPI0->STAT & SPI_STAT_RFE_MASK) != 0) {
      }
      // Drain every received byte, including during write-only transfers.
      auto data = static_cast<std::uint8_t>(SPI0->RXDATA);
      if (index < p_data_in.size()) {
        p_data_in[index] = data;
      }
    }
    // CS must stay low through the final clock edge.
    while ((SPI0->STAT & SPI_STAT_BUSY_MASK) != 0) {
    }
  }
};

/// W25Q-compatible flash with 256-byte pages and 4 KiB erase sectors.
class flash_memory
{
public:
  /**
   * @brief Construct a new flash memory object
   *
   * @param p_spi - spi port connected to the spi flash memory
   * @param p_chip_select - pin connected to spi flash memory chip select
   */
  flash_memory(lab3::spi* p_spi, lab1::output_pin* p_chip_select)
    : m_spi(p_spi)
    , m_chip_select(p_chip_select)
  {
  }

  /// Read the manufacturer byte of the JEDEC ID (Winbond is 0xEF).
  std::uint8_t manufacturer_id()
  {
    std::uint8_t id[3];
    exchange(0x9F, id);
    return id[0];
  }

  /// Decode JEDEC capacity as 2^N bytes; reject unsupported/absent devices.
  std::uint32_t capacity_bytes()
  {
    std::uint8_t id[3];
    exchange(0x9F, id);
    return id[2] >= 12 && id[2] <= 24 ? std::uint32_t{ 1 } << id[2] : 0;
  }

  /// Read the first 32 bytes; false means the device remained busy.
  bool read_block(std::uint8_t (&p_data)[32])
  {
    if (!wait_ready()) {
      return false;
    }
    read(0, p_data);
    return true;
  }

  /// Write up to 32 bytes at address zero, preserving all other sector bytes.
  bool write_block(std::span<std::uint8_t const> p_data)
  {
    if (p_data.size() > 32) {
      return false;
    }
    if (p_data.empty()) {
      return true;
    }
    if (!wait_ready()) {
      return false;
    }
    // Erasing is necessary to change zero bits back to ones. Keep the sector
    // in static RAM so it does not overflow the small default CCS stack.
    static std::uint8_t sector[4096];
    static std::uint8_t verify[256];
    read(0, sector);
    std::memcpy(sector, p_data.data(), p_data.size());
    if (!modify(0x20, 0, {})) {
      return false;
    }
    for (std::size_t offset = 0; offset < sizeof(sector); offset += 256) {
      if (!modify(0x02, offset, { sector + offset, 256 })) {
        return false;
      }
      read(offset, verify);
      if (std::memcmp(sector + offset, verify, sizeof(verify)) != 0) {
        return false;
      }
    }
    return true;
  }

private:
  void exchange(std::uint8_t p_command, std::span<std::uint8_t> p_response)
  {
    m_chip_select->level(false);
    m_spi->transfer({ &p_command, 1 }, {});
    m_spi->transfer({}, p_response);
    m_chip_select->level(true);
  }

  bool wait_ready()
  {
    // At <=16 MHz, a million 16-clock status reads allow at least 1 second
    // (sector erase max: 400 ms). Also bounds a disconnected/stuck MISO.
    constexpr std::uint8_t busy = 0x01;
    for (unsigned attempt = 0; attempt < 1'000'000; ++attempt) {
      std::uint8_t status;
      exchange(0x05, { &status, 1 });
      if ((status & busy) == 0) {
        return true;
      }
    }
    return false;
  }

  void begin_address(std::uint8_t p_command, std::uint32_t p_address)
  {
    std::uint8_t header[] = { p_command,
                              static_cast<std::uint8_t>(p_address >> 16),
                              static_cast<std::uint8_t>(p_address >> 8),
                              static_cast<std::uint8_t>(p_address) };
    m_chip_select->level(false);
    m_spi->transfer(header, {});
  }

  void read(std::uint32_t p_address, std::span<std::uint8_t> p_data)
  {
    begin_address(0x03, p_address);
    m_spi->transfer({}, p_data);
    m_chip_select->level(true);
  }

  bool modify(std::uint8_t p_command,
              std::uint32_t p_address,
              std::span<std::uint8_t const> p_data)
  {
    constexpr std::uint8_t write_enabled = 0x02;
    exchange(0x06, {});
    std::uint8_t status;
    exchange(0x05, { &status, 1 });
    if ((status & write_enabled) == 0) {
      return false;
    }
    begin_address(p_command, p_address);
    m_spi->transfer(p_data, {});
    m_chip_select->level(true);
    return wait_ready();
  }

  lab3::spi* m_spi = nullptr;
  lab1::output_pin* m_chip_select = nullptr;
};

void print_block(std::uint8_t const (&p_data)[32])
{
  for (std::size_t index = 0; index < sizeof(p_data); ++index) {
    std::printf("%02X%c",
                static_cast<unsigned>(p_data[index]),
                index % 16 == 15 ? '\n' : ' ');
  }
}

int main()
{
  SYSCFG_DL_init();
  chip_select select;
  spi_driver spi;
  if (!select.configure({}) ||
      !spi.configure({ 1'000'000, lab3::spi::mode::m0 })) {
    std::puts("SPI configuration failed.");
    return 1;
  }
  flash_memory flash(&spi, &select);
  auto manufacturer = flash.manufacturer_id();
  auto capacity = flash.capacity_bytes();
  if (manufacturer == 0 || manufacturer == 0xFF || capacity == 0) {
    std::puts("No supported flash detected. Check power and SPI wiring.");
    return 1;
  }
  std::printf("Manufacturer ID: 0x%02X\nCapacity: %lu bytes (%lu Mbit)\n",
              static_cast<unsigned>(manufacturer),
              static_cast<unsigned long>(capacity),
              static_cast<unsigned long>(capacity / 131072));

  std::uint8_t block[32];
  if (!flash.read_block(block)) {
    std::puts("Flash busy timeout.");
    return 1;
  }
  std::puts("First 32 bytes:");
  print_block(block);
  std::puts(
    "Enter up to 32 characters, then Enter (empty line skips writing):");
  std::fflush(stdout);
  std::size_t length = 0;
  int character;
  // Consume the whole line, keeping only its first 32 bytes; no terminator
  // is stored in flash. Accept spaces and both LF and CRLF console input.
  while ((character = std::getchar()) != EOF && character != '\n') {
    if (character != '\r' && length < sizeof(block)) {
      block[length++] = static_cast<std::uint8_t>(character);
    }
  }
  if (length != 0) {
    if (!flash.write_block({ block, length }) || !flash.read_block(block)) {
      std::puts(
        "Flash write/verify failed. Check wiring and write protection.");
      return 1;
    }
    std::puts("Verified first 32 bytes:");
    print_block(block);
  }
  return 0;
}
