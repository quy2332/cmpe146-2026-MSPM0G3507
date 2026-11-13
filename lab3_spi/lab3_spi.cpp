#include <cstddef>
#include <cstdint>
#include <cstdio>

#include <algorithm>
#include <array>

#include "../hal/gpio.hpp"
#include "../hal/spi.hpp"

// Host tests exercise the device protocol without memory-mapped peripherals.
#if !defined(LAB_HOST_TEST)
#include "ti_msp_dl_config.h"
#endif

namespace lab3 {

/** @brief W25Q SPI NOR driver using 24-bit addresses and 4 KiB sectors. */
class flash_memory
{
public:
  /// Delay callback takes milliseconds; tests can replace the hardware delay.
  using delay_function = void (*)(std::uint32_t);
  /// The assignment's logical block is smaller than a hardware erase sector.
  using block = std::array<std::uint8_t, 32>;

  /** @brief Bind the SPI bus, active-low chip select, and delay function. */
  flash_memory(spi& p_spi,
               lab1::output_pin& p_chip_select,
               delay_function p_delay);
  /** @brief Wake and identify a Winbond flash of at most 16 MiB. */
  [[nodiscard]] bool initialize();
  /** @brief Return the manufacturer byte from the JEDEC ID. */
  [[nodiscard]] std::uint8_t manufacturer_id() const;
  /** @brief Return capacity in bytes, or zero before initialization. */
  [[nodiscard]] std::uint32_t capacity_bytes() const;
  /** @brief Read addresses 0 through 31; false means not ready. */
  [[nodiscard]] bool read_first_block(block& p_data);
  /**
   * @brief Replace up to 32 bytes at address zero and verify the entire sector.
   *
   * Other bytes are preserved in RAM across the erase. Empty input is a no-op.
   * Keep power connected until this returns; the update is not power-fail safe.
   */
  [[nodiscard]] bool write_first_block(std::span<std::uint8_t const> p_data);

private:
  void command(std::uint8_t p_command);
  std::uint8_t status();
  bool wait_ready();
  bool write_enable();
  void read_bytes(std::uint32_t p_address, std::span<std::uint8_t> p_data);
  bool program_page(std::uint32_t p_address,
                    std::span<std::uint8_t const> p_data);

  spi& m_spi;
  lab1::output_pin& m_chip_select;
  delay_function m_delay;
  std::uint8_t m_manufacturer = 0;
  std::uint32_t m_capacity = 0;
  std::array<std::uint8_t, 4096> m_sector{};
};

flash_memory::flash_memory(spi& p_spi,
                           lab1::output_pin& p_chip_select,
                           delay_function p_delay)
  : m_spi(p_spi)
  , m_chip_select(p_chip_select)
  , m_delay(p_delay)
{
  m_chip_select.level(true);
}

void flash_memory::command(std::uint8_t p_command)
{
  m_chip_select.level(false);
  m_spi.transfer({ &p_command, 1 }, {});
  m_chip_select.level(true);
}

std::uint8_t flash_memory::status()
{
  constexpr std::uint8_t read_status = 0x05;
  std::uint8_t result = 0;
  m_chip_select.level(false);
  m_spi.transfer({ &read_status, 1 }, {});
  m_spi.transfer({}, { &result, 1 });
  m_chip_select.level(true);
  return result;
}

bool flash_memory::wait_ready()
{
  constexpr std::uint8_t busy = 0x01;
  // W25Q32JV's maximum 4 KiB erase time is 400 ms. Allow margin.
  for (unsigned attempt = 0; attempt < 1000; ++attempt) {
    if ((status() & busy) == 0) {
      return true;
    }
    m_delay(1);
  }
  return false;
}

bool flash_memory::initialize()
{
  m_manufacturer = 0;
  m_capacity = 0;
  command(0xAB);  // Release power-down; rising CS completes the command.
  m_delay(1);
  if (!wait_ready()) {
    return false;
  }
  constexpr std::uint8_t read_jedec_id = 0x9F;
  std::array<std::uint8_t, 3> id{};
  m_chip_select.level(false);
  m_spi.transfer({ &read_jedec_id, 1 }, {});
  m_spi.transfer({}, id);
  m_chip_select.level(true);
  // Restrict writes to W25Q commands and the three-byte address range.
  if (id[0] != 0xEF || (id[1] != 0x40 && id[1] != 0x60 && id[1] != 0x70) ||
      id[2] < 0x10 || id[2] > 0x18) {
    return false;
  }
  m_manufacturer = id[0];
  m_capacity = std::uint32_t{ 1 } << id[2];
  return true;
}

std::uint8_t flash_memory::manufacturer_id() const
{
  return m_manufacturer;
}

std::uint32_t flash_memory::capacity_bytes() const
{
  return m_capacity;
}

void flash_memory::read_bytes(std::uint32_t p_address,
                              std::span<std::uint8_t> p_data)
{
  std::array<std::uint8_t, 4> const header{
    0x03,
    static_cast<std::uint8_t>(p_address >> 16),
    static_cast<std::uint8_t>(p_address >> 8),
    static_cast<std::uint8_t>(p_address)
  };
  m_chip_select.level(false);
  m_spi.transfer(header, {});
  m_spi.transfer({}, p_data);
  m_chip_select.level(true);
}

bool flash_memory::read_first_block(block& p_data)
{
  if (m_capacity == 0 || !wait_ready()) {
    return false;
  }
  read_bytes(0, p_data);
  return true;
}

bool flash_memory::write_enable()
{
  constexpr std::uint8_t write_enabled = 0x02;
  command(0x06);
  return (status() & write_enabled) != 0;
}

bool flash_memory::program_page(std::uint32_t p_address,
                                std::span<std::uint8_t const> p_data)
{
  if (!write_enable()) {
    return false;
  }
  std::array<std::uint8_t, 4> const header{
    0x02,
    static_cast<std::uint8_t>(p_address >> 16),
    static_cast<std::uint8_t>(p_address >> 8),
    static_cast<std::uint8_t>(p_address)
  };
  m_chip_select.level(false);
  m_spi.transfer(header, {});
  m_spi.transfer(p_data, {});
  m_chip_select.level(true);
  return wait_ready();
}

bool flash_memory::write_first_block(std::span<std::uint8_t const> p_data)
{
  if (m_capacity == 0 || p_data.size() > block{}.size()) {
    return false;
  }
  if (p_data.empty()) {
    return true;
  }
  if (!wait_ready()) {
    return false;
  }
  read_bytes(0, m_sector);
  if (std::equal(p_data.begin(), p_data.end(), m_sector.begin())) {
    return true;
  }
  std::copy(p_data.begin(), p_data.end(), m_sector.begin());
  if (!write_enable()) {
    return false;
  }
  constexpr std::array<std::uint8_t, 4> erase_sector{ 0x20, 0, 0, 0 };
  m_chip_select.level(false);
  m_spi.transfer(erase_sector, {});
  m_chip_select.level(true);
  if (!wait_ready()) {
    return false;
  }
  constexpr std::size_t page_size = 256;
  for (std::size_t offset = 0; offset < m_sector.size(); offset += page_size) {
    std::span<std::uint8_t const> const page{ m_sector.data() + offset,
                                              page_size };
    // Erased pages already contain 0xFF; avoid unnecessary programming.
    if (!std::all_of(page.begin(),
                     page.end(),
                     [](std::uint8_t p_byte) { return p_byte == 0xFF; }) &&
        !program_page(static_cast<std::uint32_t>(offset), page)) {
      return false;
    }
  }
  std::array<std::uint8_t, page_size> verify{};
  for (std::size_t offset = 0; offset < m_sector.size(); offset += page_size) {
    read_bytes(static_cast<std::uint32_t>(offset), verify);
    if (!std::equal(verify.begin(), verify.end(), m_sector.begin() + offset)) {
      return false;
    }
  }
  return true;
}

#if !defined(LAB_HOST_TEST)
namespace {

static_assert(CPUCLK_FREQ == 32'000'000, "Update bus timing for this clock");
constexpr std::uint32_t bus_clock = CPUCLK_FREQ;

void delay_ms(std::uint32_t p_milliseconds)
{
  // NOP takes at least one cycle; loop overhead adds conservative margin.
  while (p_milliseconds-- != 0) {
    for (std::uint32_t cycle = 0; cycle < bus_clock / 1000; ++cycle) {
      __NOP();
    }
  }
}

[[noreturn]] void stop(char const* p_message)
{
  std::puts(p_message);
  std::fflush(stdout);
  while (true) {
    __WFI();
  }
}

void wait_spi(std::uint32_t p_mask, std::uint32_t p_value)
{
  for (std::uint32_t attempt = 0; attempt < 1'000'000; ++attempt) {
    if ((SPI1->STAT & p_mask) == p_value) {
      return;
    }
  }
  stop("SPI controller timeout; check peripheral clock/configuration.");
}

}  // namespace

/** @brief Register-level PB6 output used exclusively for flash chip select. */
class chip_select_pin final : public lab1::output_pin
{
public:
  /** @brief Power GPIOB and start with chip select inactive (high). */
  chip_select_pin();

private:
  bool driver_configure(settings const& p_settings) override;
  void driver_level(bool p_high) override;
  bool driver_level() override;
};

chip_select_pin::chip_select_pin()
{
  GPIOB->GPRCM.PWREN = GPIO_PWREN_KEY_UNLOCK_W | GPIO_PWREN_ENABLE_ENABLE;
  delay_ms(1);
  configure({});
}

bool chip_select_pin::driver_configure(settings const& p_settings)
{
  if (p_settings.open_drain ||
      p_settings.resistor != lab1::pin_resistor::none) {
    return false;
  }
  constexpr std::uint32_t pin = 0x0000'0040;
  GPIOB->DOUTSET31_0 = pin;  // Set latch before enabling output.
  IOMUX->SECCFG.PINCM[IOMUX_PINCM23] = IOMUX_PINCM23_PF_GPIOB_DIO06 |
                                       IOMUX_PINCM_PC_CONNECTED |
                                       IOMUX_PINCM_INENA_ENABLE;
  GPIOB->DOESET31_0 = pin;
  return true;
}

void chip_select_pin::driver_level(bool p_high)
{
  constexpr std::uint32_t pin = 0x0000'0040;
  if (p_high) {
    GPIOB->DOUTSET31_0 = pin;
  } else {
    GPIOB->DOUTCLR31_0 = pin;
  }
}

bool chip_select_pin::driver_level()
{
  constexpr std::uint32_t pin = 0x0000'0040;
  return (GPIOB->DIN31_0 & pin) != 0;
}

/** @brief Register-level SPI1: PB7 MISO, PB8 MOSI, PB9 clock. */
class mspm0_spi final : public spi
{
public:
  /** @brief Power/reset SPI1 and connect the three signal pins. */
  mspm0_spi();
  /** @brief Return the actual configured clock rate in hertz. */
  [[nodiscard]] std::uint32_t clock_rate() const;

private:
  bool driver_configure(settings const& p_settings) override;
  void driver_transfer(std::span<std::uint8_t const> p_data_out,
                       std::span<std::uint8_t> p_data_in,
                       std::uint8_t p_filler) override;
  std::uint32_t m_clock_rate = 0;
};

mspm0_spi::mspm0_spi()
{
  SPI1->GPRCM.RSTCTL = SPI_RSTCTL_KEY_UNLOCK_W | SPI_RSTCTL_RESETASSERT_ASSERT |
                       SPI_RSTCTL_RESETSTKYCLR_CLR;
  SPI1->GPRCM.PWREN = SPI_PWREN_KEY_UNLOCK_W | SPI_PWREN_ENABLE_ENABLE;
  delay_ms(1);
  IOMUX->SECCFG.PINCM[IOMUX_PINCM24] = IOMUX_PINCM24_PF_SPI1_POCI |
                                       IOMUX_PINCM_PC_CONNECTED |
                                       IOMUX_PINCM_INENA_ENABLE;
  IOMUX->SECCFG.PINCM[IOMUX_PINCM25] =
    IOMUX_PINCM25_PF_SPI1_PICO | IOMUX_PINCM_PC_CONNECTED;
  IOMUX->SECCFG.PINCM[IOMUX_PINCM26] =
    IOMUX_PINCM26_PF_SPI1_SCLK | IOMUX_PINCM_PC_CONNECTED;
}

std::uint32_t mspm0_spi::clock_rate() const
{
  return m_clock_rate;
}

bool mspm0_spi::driver_configure(settings const& p_settings)
{
  if (p_settings.clock_rate == 0 || p_settings.bus_mode > mode::m3) {
    return false;
  }
  // SCLK = SYSCLK / (CLKDIV + 1) / (2 * (SCR + 1)). Never exceed request.
  std::uint32_t divider = 0;
  std::uint32_t serial_divider = 0;
  std::uint32_t best_divisor = 0;
  for (std::uint32_t candidate = 1; candidate <= 8; ++candidate) {
    auto const denominator =
      std::uint64_t{ 2 } * candidate * p_settings.clock_rate;
    auto const count =
      std::max<std::uint64_t>(1, (bus_clock + denominator - 1) / denominator);
    auto const divisor =
      static_cast<std::uint32_t>(std::uint64_t{ 2 } * candidate * count);
    if (count <= 1024 && (best_divisor == 0 || divisor < best_divisor)) {
      divider = candidate - 1;
      serial_divider = static_cast<std::uint32_t>(count - 1);
      best_divisor = divisor;
    }
  }
  if (best_divisor == 0) {
    return false;
  }
  wait_spi(SPI_STAT_BUSY_MASK, SPI_STAT_BUSY_IDLE);
  SPI1->CTL1 = 0;
  SPI1->CLKSEL = SPI_CLKSEL_SYSCLK_SEL_ENABLE;
  SPI1->CLKDIV = divider;
  SPI1->CLKCTL = serial_divider;
  std::uint32_t format = SPI_CTL0_DSS_DSS_8 | SPI_CTL0_FRF_MOTOROLA_3WIRE;
  if (p_settings.bus_mode == mode::m2 || p_settings.bus_mode == mode::m3) {
    format |= SPI_CTL0_SPO_HIGH;
  }
  if (p_settings.bus_mode == mode::m1 || p_settings.bus_mode == mode::m3) {
    format |= SPI_CTL0_SPH_SECOND;
  }
  SPI1->CTL0 = format;
  SPI1->CTL1 =
    SPI_CTL1_CP_ENABLE | SPI_CTL1_MSB_ENABLE | SPI_CTL1_ENABLE_ENABLE;
  m_clock_rate = bus_clock / best_divisor;
  return true;
}

void mspm0_spi::driver_transfer(std::span<std::uint8_t const> p_data_out,
                                std::span<std::uint8_t> p_data_in,
                                std::uint8_t p_filler)
{
  if (m_clock_rate == 0) {
    stop("Configure SPI before transferring data.");
  }
  auto const length = std::max(p_data_out.size(), p_data_in.size());
  for (std::size_t index = 0; index < length; ++index) {
    wait_spi(SPI_STAT_TNF_MASK, SPI_STAT_TNF_NOT_FULL);
    SPI1->TXDATA = index < p_data_out.size() ? p_data_out[index] : p_filler;
    wait_spi(SPI_STAT_RFE_MASK, SPI_STAT_RFE_NOT_EMPTY);
    auto const received = static_cast<std::uint8_t>(SPI1->RXDATA);
    if (index < p_data_in.size()) {
      p_data_in[index] = received;
    }
    // Consume RX even during writes, so the receive FIFO cannot overflow.
  }
  // The caller may release CS as soon as transfer() returns.
  wait_spi(SPI_STAT_BUSY_MASK, SPI_STAT_BUSY_IDLE);
}

namespace {
void print_block(flash_memory::block const& p_data)
{
  for (std::size_t index = 0; index < p_data.size(); ++index) {
    std::printf("%02X%c",
                static_cast<unsigned>(p_data[index]),
                index % 16 == 15 ? '\n' : ' ');
  }
}
}  // namespace
#endif
}  // namespace lab3

#if !defined(LAB_HOST_TEST)
int main()
{
  SYSCFG_DL_init();
  lab3::chip_select_pin chip_select;
  lab3::mspm0_spi bus;
  if (!bus.configure({ 1'000'000, lab3::spi::mode::m0 })) {
    lab3::stop("Unsupported SPI settings.");
  }
  // Static storage keeps the 4 KiB sector backup off the small default stack.
  static lab3::flash_memory flash(bus, chip_select, lab3::delay_ms);
  lab3::delay_ms(10);
  if (!flash.initialize()) {
    lab3::stop("Flash not ready/supported. Check power, wiring, and part ID.");
  }
  std::printf("SPI: %lu Hz\nManufacturer: 0x%02X\nCapacity: %lu bytes\n",
              static_cast<unsigned long>(bus.clock_rate()),
              static_cast<unsigned>(flash.manufacturer_id()),
              static_cast<unsigned long>(flash.capacity_bytes()));
  while (true) {
    lab3::flash_memory::block block{};
    if (!flash.read_first_block(block)) {
      lab3::stop("Flash read timed out.");
    }
    lab3::print_block(block);
    std::printf("Enter up to 32 text bytes in the console, then Enter\n"
                "(empty line skips writing): ");
    std::fflush(stdout);
    std::size_t size = 0;
    bool too_long = false;
    int character = 0;
    while ((character = std::getchar()) != '\n' && character != EOF) {
      if (character == '\r') {
        continue;
      }
      if (size == block.size()) {
        too_long = true;
      } else {
        block[size++] = static_cast<std::uint8_t>(character);
      }
    }
    if (character == EOF) {
      lab3::stop("No console input. Check the UART or CCS CIO connection.");
    }
    if (too_long) {
      std::puts("More than 32 bytes: nothing written. Try again.");
    } else if (size != 0) {
      if (!flash.write_first_block({ block.data(), size })) {
        lab3::stop("Flash write/verification failed. Check write protection.");
      }
      std::puts("Write verified; next dump shows the stored bytes.");
    }
  }
}
#endif
