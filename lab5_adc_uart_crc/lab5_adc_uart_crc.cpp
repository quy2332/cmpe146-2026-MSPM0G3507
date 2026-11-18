// cspell:ignore CCLKCTL centi CLKDIV CLKEN CLKSEL CRCCTRL CRCIN CRCOUT CRCSEED
// cspell:ignore CTRCTL CVAE DDEVICE DMAEN DOECLR DOESET DOUTCLR FACTORYREGION
// cspell:ignore FBRD GPIOA GPIOB Gprcm GPRCM gprcm IBRD IIDX INENA LCRH MCLK
// cspell:ignore microvolts NOLINTNEXTLINE PIPD PIPU PWREN RESETSTKYCLR RSTCTL
// cspell:ignore RXDATA RXFE RXIFLSEL RXINT SLASEX SYSOSC TSTRIM TXFF ULPCLK
// cspell:ignore VDDA VRSEL

// ADC + DMA: DriverLib. UART RX: interrupts. CRC: register-level.
// Sensor: request_task -> sample_request semaphore -> sampler_task ->
// batch_queue -> transmit_task.
// Default role is SENSOR. Define LAB5_DEVICE_ROLE_DISPLAY for the display
// board. UART wiring: sensor PA8 TX -> display PA9 RX, and vice versa; common
// GND.

#include <algorithm>
#include <array>
#include <cinttypes>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "../hal/adc.hpp"
#include "../hal/crc.hpp"
#include "../hal/gpio.hpp"
#include "../hal/serial.hpp"
#include "../hal/timer.hpp"
#include <ti/driverlib/driverlib.h>

namespace {
constexpr std::uint16_t upscale_12_to_16(std::uint16_t p_code);
}

namespace {
// ---------------------------------------------------------------------------
// Register-level helpers shared by the drivers below
// ---------------------------------------------------------------------------

// Every MSPM0 peripheral's PWREN/RSTCTL register takes the same unlock keys
// in its top byte (TRM "Peripheral Power Enable Control").
constexpr std::uint32_t power_enable =
  GPIO_PWREN_KEY_UNLOCK_W | GPIO_PWREN_ENABLE_ENABLE;
constexpr std::uint32_t reset_assert = GPIO_RSTCTL_KEY_UNLOCK_W |
                                       GPIO_RSTCTL_RESETSTKYCLR_CLR |
                                       GPIO_RSTCTL_RESETASSERT_ASSERT;
// Peripherals need a few bus cycles after PWREN before they accept writes.
constexpr std::uint32_t power_up_cycles = 32;
constexpr std::uint32_t mcu_clock = 32'000'000;  // SYSOSC, MCLK = ULPCLK

void wait_cycles(std::uint32_t p_cycles)
{
  // Each iteration is at least 4 cycles on a Cortex-M0+.
  for (std::uint32_t count = 0; count < p_cycles / 4 + 1; count++) {
    __NOP();
  }
}

template<class Gprcm>
void reset_and_power(Gprcm& p_gprcm)
{
  p_gprcm.RSTCTL = reset_assert;
  p_gprcm.PWREN = power_enable;
  wait_cycles(power_up_cycles);
}

template<class Gprcm>
void power_if_needed(Gprcm& p_gprcm)
{
  // GPIO ports are shared with the startup console, so don't reset them if
  // something already turned them on.
  if ((p_gprcm.PWREN & GPIO_PWREN_ENABLE_MASK) == 0) {
    reset_and_power(p_gprcm);
  }
}

constexpr std::uint32_t pin_function_gpio = 0x1;  // IOMUX PF value for GPIO

void set_pin_mux(std::uint8_t p_pincm, std::uint32_t p_value)
{
  // PINCMn in the datasheet pin table is PINCM[n - 1] in the register map.
  IOMUX->SECCFG.PINCM[p_pincm - 1U] = p_value;
}

constexpr std::uint32_t pull_bits(lab1::pin_resistor p_resistor)
{
  switch (p_resistor) {
    case lab1::pin_resistor::pull_down:
      return IOMUX_PINCM_PIPD_ENABLE;
    case lab1::pin_resistor::pull_up:
      return IOMUX_PINCM_PIPU_ENABLE;
    case lab1::pin_resistor::none:
    default:
      return 0;
  }
}

/// RAII interrupt mask, safe to nest and to use from an ISR.
class interrupt_lock
{
public:
  interrupt_lock()
    : m_primask(__get_PRIMASK())
  {
    __disable_irq();
  }
  interrupt_lock(interrupt_lock const&) = delete;
  interrupt_lock& operator=(interrupt_lock const&) = delete;
  ~interrupt_lock()
  {
    __set_PRIMASK(m_primask);
  }

private:
  std::uint32_t m_primask;
};

std::uint32_t address_of(void const volatile* p_pointer)
{
  return static_cast<std::uint32_t>(
    reinterpret_cast<std::uintptr_t>(const_cast<void const*>(p_pointer)));
}

// ---------------------------------------------------------------------------
// GPIO
// ---------------------------------------------------------------------------

enum class gpio_port : std::uint8_t
{
  a,
  b,
};

/// A pin's GPIO port/bit plus its IOMUX PINCM number from the datasheet.
struct pin_location
{
  gpio_port port;
  std::uint8_t pin;
  std::uint8_t pincm;
};

// LP-MSPM0G3507 user guide (SLAU873): S1 pulls PA18 high when pressed, S2
// pulls PB21 low when pressed; the RGB LED is active high.
constexpr pin_location button_s1{ gpio_port::a, 18, 40 };
constexpr pin_location button_s2{ gpio_port::b, 21, 49 };
constexpr pin_location led_red{ gpio_port::b, 26, 57 };
constexpr pin_location led_green{ gpio_port::b, 27, 58 };
constexpr pin_location led_blue{ gpio_port::b, 22, 50 };

GPIO_Regs* port_registers(gpio_port p_port)
{
  return p_port == gpio_port::a ? GPIOA : GPIOB;
}

class mspm0_input_pin final : public lab1::input_pin
{
public:
  explicit mspm0_input_pin(pin_location p_location)
    : m_location(p_location)
  {
    power_if_needed(port_registers(m_location.port)->GPRCM);
  }

private:
  bool driver_configure(settings const& p_settings) override
  {
    port_registers(m_location.port)->DOECLR31_0 = mask();
    set_pin_mux(m_location.pincm,
                IOMUX_PINCM_PC_CONNECTED | IOMUX_PINCM_INENA_ENABLE |
                  pull_bits(p_settings.resistor) | pin_function_gpio);
    return true;
  }

  bool driver_level() override
  {
    return (port_registers(m_location.port)->DIN31_0 & mask()) != 0;
  }

  [[nodiscard]] std::uint32_t mask() const
  {
    return 1UL << m_location.pin;
  }

  pin_location m_location;
};

class mspm0_output_pin final : public lab1::output_pin
{
public:
  explicit mspm0_output_pin(pin_location p_location)
    : m_location(p_location)
  {
    power_if_needed(port_registers(m_location.port)->GPRCM);
  }

private:
  bool driver_configure(settings const& p_settings) override
  {
    auto const open_drain =
      p_settings.open_drain ? IOMUX_PINCM_HIZ1_ENABLE : 0U;
    // INENA lets level() read the pin back from hardware.
    set_pin_mux(m_location.pincm,
                IOMUX_PINCM_PC_CONNECTED | IOMUX_PINCM_INENA_ENABLE |
                  open_drain | pull_bits(p_settings.resistor) |
                  pin_function_gpio);
    port_registers(m_location.port)->DOESET31_0 = mask();
    return true;
  }

  void driver_level(bool p_high) override
  {
    auto* const port = port_registers(m_location.port);
    if (p_high) {
      port->DOUTSET31_0 = mask();
    } else {
      port->DOUTCLR31_0 = mask();
    }
  }

  bool driver_level() override
  {
    return (port_registers(m_location.port)->DIN31_0 & mask()) != 0;
  }

  [[nodiscard]] std::uint32_t mask() const
  {
    return 1UL << m_location.pin;
  }

  pin_location m_location;
};

// ---------------------------------------------------------------------------
// Steady clock: TIMG12 (32-bit) counting BUSCLK, extended to 64 bits in
// software by counting wraps in its zero-event interrupt.
// ---------------------------------------------------------------------------

class timg12_steady_clock;
timg12_steady_clock* active_clock = nullptr;

class timg12_steady_clock final : public lab2::steady_clock
{
public:
  timg12_steady_clock()
  {
    reset_and_power(TIMG12->GPRCM);
    TIMG12->CLKSEL = GPTIMER_CLKSEL_BUSCLK_SEL_ENABLE;
    TIMG12->CLKDIV = GPTIMER_CLKDIV_RATIO_DIV_BY_1;
    TIMG12->COMMONREGS.CPS = 0;  // prescale by 1
    TIMG12->COMMONREGS.CCLKCTL = GPTIMER_CCLKCTL_CLKEN_ENABLED;
    TIMG12->COUNTERREGS.LOAD = 0xFFFF'FFFFU;
    TIMG12->COUNTERREGS.CTRCTL =
      GPTIMER_CTRCTL_CM_UP | GPTIMER_CTRCTL_REPEAT_REPEAT_1 |
      GPTIMER_CTRCTL_CVAE_ZEROVAL | GPTIMER_CTRCTL_EN_ENABLED;
    // Zero event == the counter wrapping from 0xFFFFFFFF back to 0.
    TIMG12->CPU_INT.ICLR = GPTIMER_CPU_INT_IMASK_Z_SET;
    active_clock = this;
    TIMG12->CPU_INT.IMASK = GPTIMER_CPU_INT_IMASK_Z_SET;
    NVIC_ClearPendingIRQ(TIMG12_INT_IRQn);
    NVIC_EnableIRQ(TIMG12_INT_IRQn);
  }
  timg12_steady_clock(timg12_steady_clock const&) = delete;
  timg12_steady_clock& operator=(timg12_steady_clock const&) = delete;

  void handle_interrupt()
  {
    if ((TIMG12->CPU_INT.MIS & GPTIMER_CPU_INT_IMASK_Z_SET) != 0) {
      TIMG12->CPU_INT.ICLR = GPTIMER_CPU_INT_IMASK_Z_SET;
      m_wraps = m_wraps + 1;
    }
  }

private:
  std::uint32_t driver_frequency() override
  {
    return mcu_clock;
  }

  std::uint64_t driver_uptime() override
  {
    interrupt_lock const lock;
    // With interrupts masked, a wrap can still happen between reading the
    // counter and checking the flag. Re-read the counter if the flag
    // appears in between so the pair is always consistent.
    auto wrap_pending =
      (TIMG12->CPU_INT.RIS & GPTIMER_CPU_INT_IMASK_Z_SET) != 0;
    std::uint32_t counter = TIMG12->COUNTERREGS.CTR;
    if (!wrap_pending &&
        (TIMG12->CPU_INT.RIS & GPTIMER_CPU_INT_IMASK_Z_SET) != 0) {
      wrap_pending = true;
      counter = TIMG12->COUNTERREGS.CTR;
    }
    std::uint64_t const wraps = m_wraps + (wrap_pending ? 1U : 0U);
    return (wraps << 32U) | counter;
  }

  std::uint32_t volatile m_wraps = 0;
};

// Only the ADC uses DMA. UART reception uses its own RX interrupt.
constexpr std::uint8_t adc_dma_channel = 0;
}  // namespace

// ===========================================================================
// ROLE 1: ADC with DriverLib and one DMA transfer per buffer
// ===========================================================================
class adc_driver final : public lab5::adc16_stream
{
public:
  adc_driver();
  adc_driver(adc_driver const&) = delete;
  adc_driver& operator=(adc_driver const&) = delete;
  [[nodiscard]] static std::uint16_t calibration_code();
  void handle_interrupt();

private:
  void driver_read(std::span<std::uint16_t> p_buffer) override;
  void read_block(std::uint16_t* p_destination, std::uint16_t p_count);
  TaskHandle_t volatile m_waiting_task = nullptr;
};

namespace {
adc_driver* active_adc = nullptr;
// SYSOSC 32 MHz / 8 = 4 MHz; 80 sample cycles = 20 us (minimum 12.5 us).
constexpr std::uint32_t sample_clock = mcu_clock / 8;
constexpr std::uint32_t sample_time_cycles = sample_clock / 50'000;
}  // namespace

adc_driver::adc_driver()
{
  DL_ADC12_reset(ADC0);
  DL_ADC12_enablePower(ADC0);
  wait_cycles(power_up_cycles);

  DL_ADC12_ClockConfig const clock_config{
    DL_ADC12_CLOCK_SYSOSC,
    DL_ADC12_CLOCK_FREQ_RANGE_24_TO_32,
    DL_ADC12_CLOCK_DIVIDE_8,
  };
  DL_ADC12_setClockConfig(ADC0, &clock_config);
  DL_ADC12_initSingleSample(ADC0,
                            DL_ADC12_REPEAT_MODE_ENABLED,
                            DL_ADC12_SAMPLING_SOURCE_AUTO,
                            DL_ADC12_TRIG_SRC_SOFTWARE,
                            DL_ADC12_SAMP_CONV_RES_12_BIT,
                            DL_ADC12_SAMP_CONV_DATA_FORMAT_UNSIGNED);
  DL_ADC12_configConversionMem(ADC0,
                               DL_ADC12_MEM_IDX_0,
                               DL_ADC12_INPUT_CHAN_11,
                               DL_ADC12_REFERENCE_VOLTAGE_VDDA,
                               DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0,
                               DL_ADC12_AVERAGING_MODE_DISABLED,
                               DL_ADC12_BURN_OUT_SOURCE_DISABLED,
                               DL_ADC12_TRIGGER_MODE_AUTO_NEXT,
                               DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
  DL_ADC12_setPowerDownMode(ADC0, DL_ADC12_POWER_DOWN_MODE_MANUAL);
  DL_ADC12_setSampleTime0(ADC0, sample_time_cycles);
  DL_ADC12_setDMASamplesCnt(ADC0, 1);
  DL_ADC12_enableDMATrigger(ADC0, DL_ADC12_DMA_MEM0_RESULT_LOADED);

  DL_DMA_Config const dma_config{
    DMA_ADC0_EVT_GEN_BD_TRIG,    DL_DMA_TRIGGER_TYPE_EXTERNAL,
    DL_DMA_SINGLE_TRANSFER_MODE, DL_DMA_NORMAL_MODE,
    DL_DMA_WIDTH_HALF_WORD,      DL_DMA_WIDTH_HALF_WORD,
    DL_DMA_ADDR_UNCHANGED,       DL_DMA_ADDR_INCREMENT,
  };
  DL_DMA_initChannel(DMA, adc_dma_channel, &dma_config);
  DL_DMA_setSrcAddr(DMA,
                    adc_dma_channel,
                    DL_ADC12_getMemResultAddress(ADC0, DL_ADC12_MEM_IDX_0));

  active_adc = this;
  DL_ADC12_clearInterruptStatus(ADC0, DL_ADC12_INTERRUPT_DMA_DONE);
  DL_ADC12_enableInterrupt(ADC0, DL_ADC12_INTERRUPT_DMA_DONE);
  NVIC_ClearPendingIRQ(ADC0_INT_IRQn);
  NVIC_SetPriority(ADC0_INT_IRQn, 3);
  NVIC_EnableIRQ(ADC0_INT_IRQn);
  DL_ADC12_enableConversions(ADC0);
}

std::uint16_t adc_driver::calibration_code()
{
  return static_cast<std::uint16_t>(DL_SYSCTL_getTempCalibrationConstant() &
                                    0x0FFFU);
}

void adc_driver::read_block(std::uint16_t* p_destination, std::uint16_t p_count)
{
  // Clear old notifications before starting a new transfer. Record the task
  // first, so even a very fast completion cannot lose its wake-up.
  (void)ulTaskNotifyTake(pdTRUE, 0);
  m_waiting_task = xTaskGetCurrentTaskHandle();
  DL_DMA_disableChannel(DMA, adc_dma_channel);
  DL_DMA_setSrcAddr(DMA,
                    adc_dma_channel,
                    DL_ADC12_getMemResultAddress(ADC0, DL_ADC12_MEM_IDX_0));
  DL_DMA_setDestAddr(DMA, adc_dma_channel, address_of(p_destination));
  DL_DMA_setTransferSize(DMA, adc_dma_channel, p_count);
  DL_ADC12_clearInterruptStatus(ADC0, DL_ADC12_INTERRUPT_DMA_DONE);
  DL_DMA_enableChannel(DMA, adc_dma_channel);
  // The ADC clears DMAEN after each block, so re-arm it for every read.
  DL_ADC12_enableDMA(ADC0);
  DL_ADC12_startConversion(ADC0);
  (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  DL_ADC12_stopConversion(ADC0);
  DL_ADC12_disableDMA(ADC0);
  DL_DMA_disableChannel(DMA, adc_dma_channel);
  m_waiting_task = nullptr;
}

void adc_driver::driver_read(std::span<std::uint16_t> p_buffer)
{
  // The lab's 64 samples fit in ONE DMA block and cause ONE task sleep.
  // Larger callers are supported up to the hardware's 16-bit block limit
  // per transfer. An empty span needs no transfer and no wait.
  constexpr std::size_t max_block = 0xFFFF;
  for (std::size_t offset = 0; offset < p_buffer.size(); offset += max_block) {
    auto const count = std::min(max_block, p_buffer.size() - offset);
    read_block(p_buffer.data() + offset, static_cast<std::uint16_t>(count));
  }
  for (auto& sample : p_buffer) {
    sample = upscale_12_to_16(sample);
  }
}

void adc_driver::handle_interrupt()
{
  // DriverLib calls remain inside the driver class. The vector wrapper only
  // forwards to this method. ADC DMA completion is an ADC interrupt.
  if (DL_ADC12_getPendingInterrupt(ADC0) == DL_ADC12_IIDX_DMA_DONE) {
    BaseType_t higher_priority_woken = pdFALSE;
    if (m_waiting_task != nullptr) {
      vTaskNotifyGiveFromISR(m_waiting_task, &higher_priority_woken);
    }
    portYIELD_FROM_ISR(higher_priority_woken);
  }
}

// NOLINTNEXTLINE(readability-identifier-naming): vector table symbol.
extern "C" void ADC0_IRQHandler(void)
{
  if (active_adc != nullptr) {
    active_adc->handle_interrupt();
  }
}

// ROLE 2: UART1 on PA8 (TX, J1.4) / PA9 (RX, J1.3).
// Register-level driver with an interrupt-filled 256-byte receive ring.
class serial_driver final : public lab6::serial
{
public:
  serial_driver();
  serial_driver(serial_driver const&) = delete;
  serial_driver& operator=(serial_driver const&) = delete;
  void handle_receive_interrupt();

private:
  bool driver_configure(settings const& p_settings) override;
  void driver_write(std::span<std::uint8_t const> p_data) override;
  std::span<std::uint8_t const> driver_receive_buffer() override;
  std::size_t driver_receive_cursor() override;

  std::array<std::uint8_t, 256> m_receive_buffer{};
  std::size_t volatile m_receive_cursor = 0;
};

namespace {
serial_driver* active_serial = nullptr;
constexpr std::uint8_t uart1_tx_pincm = 19;        // PA8
constexpr std::uint8_t uart1_rx_pincm = 20;        // PA9
constexpr std::uint32_t uart1_pin_function = 0x2;  // IOMUX_PINCM19/20_PF_UART1
constexpr std::uint32_t uart_clock = mcu_clock;    // ULPCLK (BUSCLK for PD0)
constexpr std::uint32_t uart_idle_poll_limit = 1'000'000;
// UART1 sits in the ULP (PD0) domain, where only RXIFLSEL 0 (>= 1/4 of the
// 4-entry FIFO, i.e. one byte) and 4 (full) are implemented (TRM IFLS).
constexpr std::uint32_t receive_level_one_byte = UART_IFLS_RXIFLSEL_LVL_1_4_RES;

struct baud_divisor
{
  std::uint32_t oversampling;
  std::uint32_t integer;
  std::uint32_t fraction;
};

/// IBRD/FBRD for p_baud_rate, preferring 16x oversampling (TRM 24.2).
bool compute_baud_divisor(std::uint32_t p_baud_rate, baud_divisor& p_result)
{
  if (p_baud_rate == 0) {
    return false;
  }
  for (std::uint32_t const oversampling : { 16U, 8U }) {
    // divisor * 64 = clock * 64 / (oversampling * baud), rounded.
    auto const scaled = (std::uint64_t{ uart_clock } * 64 +
                         std::uint64_t{ oversampling } * p_baud_rate / 2) /
                        (std::uint64_t{ oversampling } * p_baud_rate);
    auto const integer = scaled >> 6U;
    if (integer >= 1 && integer <= 0xFFFF) {
      p_result = { oversampling,
                   static_cast<std::uint32_t>(integer),
                   static_cast<std::uint32_t>(scaled & 0x3FU) };
      return true;
    }
  }
  return false;
}
}  // namespace

serial_driver::serial_driver()
{
  reset_and_power(UART1->GPRCM);
  set_pin_mux(uart1_tx_pincm, IOMUX_PINCM_PC_CONNECTED | uart1_pin_function);
  // Pull RX up so an unplugged link idles high instead of reading noise.
  set_pin_mux(uart1_rx_pincm,
              IOMUX_PINCM_PC_CONNECTED | IOMUX_PINCM_INENA_ENABLE |
                IOMUX_PINCM_PIPU_ENABLE | uart1_pin_function);
  UART1->CLKSEL = UART_CLKSEL_BUSCLK_SEL_ENABLE;
  UART1->CLKDIV = UART_CLKDIV_RATIO_DIV_BY_1;
  (void)driver_configure(settings{});

  // RX threshold = one byte. Drain the FIFO on every RX interrupt.
  // No UART DriverLib calls and no DMA channel are used here.
  active_serial = this;
  UART1->CPU_INT.ICLR = UART_CPU_INT_ICLR_RXINT_CLR;
  UART1->CPU_INT.IMASK = UART_CPU_INT_IMASK_RXINT_SET;
  NVIC_ClearPendingIRQ(UART1_INT_IRQn);
  NVIC_SetPriority(UART1_INT_IRQn, 3);
  NVIC_EnableIRQ(UART1_INT_IRQn);
}

void serial_driver::handle_receive_interrupt()
{
  UART1->CPU_INT.ICLR = UART_CPU_INT_ICLR_RXINT_CLR;
  while ((UART1->STAT & UART_STAT_RXFE_MASK) == 0) {
    auto const byte = static_cast<std::uint8_t>(UART1->RXDATA);
    auto const cursor = m_receive_cursor;
    m_receive_buffer[cursor] = byte;
    // Publish the cursor only after the new byte has been stored.
    __DMB();
    m_receive_cursor = (cursor + 1U) % m_receive_buffer.size();
  }
}

bool serial_driver::driver_configure(settings const& p_settings)
{
  // Validate everything before touching the peripheral (serial.hpp).
  baud_divisor divisor{};
  if (!compute_baud_divisor(p_settings.baud_rate, divisor)) {
    return false;
  }
  std::uint32_t line_control = UART_LCRH_WLEN_DATABIT8;
  switch (p_settings.parity) {
    case settings::parity::none:
      break;
    case settings::parity::odd:
      line_control |= UART_LCRH_PEN_ENABLE | UART_LCRH_EPS_ODD;
      break;
    case settings::parity::even:
      line_control |= UART_LCRH_PEN_ENABLE | UART_LCRH_EPS_EVEN;
      break;
    case settings::parity::forced1:  // stick parity, EPS = 0 sends 1
      line_control |= UART_LCRH_PEN_ENABLE | UART_LCRH_SPS_ENABLE;
      break;
    case settings::parity::forced0:  // stick parity, EPS = 1 sends 0
      line_control |=
        UART_LCRH_PEN_ENABLE | UART_LCRH_SPS_ENABLE | UART_LCRH_EPS_EVEN;
      break;
    default:
      return false;
  }
  switch (p_settings.stop) {
    case settings::stop_bits::one:
      break;
    case settings::stop_bits::two:
      line_control |= UART_LCRH_STP2_ENABLE;
      break;
    default:
      return false;
  }

  // Let any byte in flight finish, then reconfigure with the UART disabled.
  for (std::uint32_t poll = 0;
       poll < uart_idle_poll_limit && (UART1->STAT & UART_STAT_BUSY_MASK) != 0;
       poll++) {
  }
  UART1->CTL0 = UART1->CTL0 & ~UART_CTL0_ENABLE_MASK;
  UART1->IBRD = divisor.integer;
  UART1->FBRD = divisor.fraction;
  UART1->LCRH = line_control;  // writing LCRH latches IBRD/FBRD
  UART1->IFLS =
    (UART1->IFLS & ~UART_IFLS_RXIFLSEL_MASK) | receive_level_one_byte;
  UART1->CTL0 =
    (divisor.oversampling == 16 ? UART_CTL0_HSE_OVS16 : UART_CTL0_HSE_OVS8) |
    UART_CTL0_MODE_UART | UART_CTL0_FEN_ENABLE | UART_CTL0_TXE_ENABLE |
    UART_CTL0_RXE_ENABLE | UART_CTL0_ENABLE_ENABLE;
  return true;
}

void serial_driver::driver_write(std::span<std::uint8_t const> p_data)
{
  for (auto const byte : p_data) {
    while ((UART1->STAT & UART_STAT_TXFF_MASK) != 0) {
      continue;  // 4-byte TX FIFO full; drains at the baud rate
    }
    UART1->TXDATA = byte;
  }
}

std::span<std::uint8_t const> serial_driver::driver_receive_buffer()
{
  return m_receive_buffer;
}

std::size_t serial_driver::driver_receive_cursor()
{
  auto const cursor = m_receive_cursor;
  __DMB();
  return cursor;
}

// NOLINTNEXTLINE(readability-identifier-naming): vector table symbol.
extern "C" void UART1_IRQHandler(void)
{
  if (active_serial != nullptr) {
    active_serial->handle_receive_interrupt();
  }
}

// ROLE 3: the on-chip CRC accelerator in CRC-32 mode.
//
// The reference is the reflected ("LSB-first") CRC-32, while the accelerator
// natively shifts MSB-first. BITREVERSE reverses each input byte and the
// output, which is exactly the reflected algorithm. The 0xFFFFFFFF seed is
// its own bit-reverse, and the final XOR is done here in software.
//
// There is one CRC peripheral, so every crc32_hardware object shares it.
class crc32_hardware final : public lab6::crc32
{
public:
  crc32_hardware()
  {
    if ((CRC->GPRCM.PWREN & CRC_PWREN_ENABLE_MASK) == 0) {
      reset_and_power(CRC->GPRCM);
    }
    driver_reset();
  }

private:
  static constexpr std::uint32_t seed = 0xFFFF'FFFFU;
  static constexpr std::uint32_t final_xor = 0xFFFF'FFFFU;

  void driver_reset() override
  {
    // The polynomial must be chosen before the seed is written.
    CRC->CRCCTRL = CRC_CRCCTRL_POLYSIZE_CRC32 |
                   CRC_CRCCTRL_BITREVERSE_REVERSED |
                   CRC_CRCCTRL_INPUT_ENDIANNESS_LITTLE_ENDIAN |
                   CRC_CRCCTRL_OUTPUT_BYTESWAP_DISABLE;
    CRC->CRCSEED = seed;
  }

  void driver_update(std::span<std::uint8_t const> p_data) override
  {
    auto const* data = p_data.data();
    auto remaining = p_data.size();
    // An 8-bit store to CRCIN feeds exactly one byte.
    auto* const byte_input = reinterpret_cast<std::uint8_t volatile*>(
      const_cast<std::uint32_t*>(&CRC->CRCIN));

    while (remaining != 0 && (reinterpret_cast<std::uintptr_t>(data) & 3U)) {
      *byte_input = *data++;
      remaining--;
    }
    // Little-endian word writes feed the lowest-addressed byte first, so
    // four bytes go in per bus write.
    for (; remaining >= 4; remaining -= 4, data += 4) {
      std::uint32_t word;
      std::memcpy(&word, data, sizeof(word));
      CRC->CRCIN = word;
    }
    while (remaining != 0) {
      *byte_input = *data++;
      remaining--;
    }
  }

  std::uint32_t driver_checksum() override
  {
    return CRC->CRCOUT ^ final_xor;
  }
};

// ---------------------------------------------------------------------------
// Interrupt handlers. The names must match the vector table in TI's startup
// file (startup_mspm0g350x_ticlang.c).
// ---------------------------------------------------------------------------

// NOLINTNEXTLINE(readability-identifier-naming): vector table symbol.
extern "C" void TIMG12_IRQHandler(void)
{
  if (active_clock != nullptr) {
    active_clock->handle_interrupt();
  }
}

// Supplied reference implementation - do not modify. This is the standard
// reflected CRC-32 (IEEE 802.3 / zlib) algorithm, computed bit-by-bit in
// software. Your crc32_hardware driver's output must match this for the
// same input, and should be measurably faster.
class crc32_software : public lab6::crc32
{
private:
  void driver_reset() override
  {
    m_crc = initial_value;
  }

  void driver_update(std::span<std::uint8_t const> p_data) override
  {
    for (auto const byte : p_data) {
      m_crc ^= byte;
      for (int bit = 0; bit < 8; bit++) {
        std::uint32_t const mask = -(m_crc & 1U);
        m_crc = (m_crc >> 1) ^ (polynomial & mask);
      }
    }
  }

  std::uint32_t driver_checksum() override
  {
    return m_crc ^ initial_value;
  }

  static constexpr std::uint32_t polynomial = 0xEDB8'8320U;
  static constexpr std::uint32_t initial_value = 0xFFFF'FFFFU;

  std::uint32_t m_crc = initial_value;
};

namespace {
// ---------------------------------------------------------------------------
// Wire protocol and temperature math. Nothing in this section touches
// hardware. These helpers convert samples and encode the shared packet.
// ---------------------------------------------------------------------------

// The one-byte message the display board sends to ask the sensor board for
// a fresh batch of readings.
constexpr std::uint8_t sample_request_command = 0xA5;

constexpr std::size_t samples_per_batch = 64;

// One temperature sample on the wire: a signed count of 0.01 degC steps
// ("centi-degrees"), little-endian. 2534 means 25.34 degC. That resolves far
// finer than the required 0.5 degC and still covers -327.68 to +327.67 degC,
// well past the sensor's -40 to 125 degC range.
using temperature_sample_t = std::int16_t;

// Response layout (132 bytes):
// [0..127]   64 signed centi-degree samples, each little-endian.
// [128..131] CRC-32 of the 128 sample bytes, little-endian.
// Both boards must use this same encoding. There is no extra header byte.
constexpr std::size_t payload_size =
  samples_per_batch * sizeof(temperature_sample_t);
[[maybe_unused]] constexpr std::size_t crc_size = sizeof(std::uint32_t);
using temperature_batch = std::array<temperature_sample_t, samples_per_batch>;
constexpr std::size_t packet_size =
  sizeof(temperature_batch) + sizeof(std::uint32_t);

using sample_payload = std::array<std::uint8_t, payload_size>;
using response_packet = std::array<std::uint8_t, packet_size>;

// Datasheet (SLASEX6C) section 7.14 and 8.13: TEMP_SENSE0 holds the 12-bit
// ADC code of the sensor at TSTRIM = 30 degC, measured with VRSEL = VDDA
// (3.3 V). The sensor slope TSc is -1.8 mV/degC (typical).
constexpr std::int32_t trim_temperature = 3000;  // centi-degC
constexpr std::int64_t sensor_slope_microvolts_per_degree = -1800;
constexpr std::int64_t reference_microvolts = 3'300'000;
constexpr std::int64_t adc_12_bit_codes = 4096;

/// Divide, rounding halves away from zero (C++ division truncates).
constexpr std::int64_t divide_rounded(std::int64_t p_numerator,
                                      std::int64_t p_denominator)
{
  auto const negative = (p_numerator < 0) != (p_denominator < 0);
  auto const numerator = p_numerator < 0 ? -p_numerator : p_numerator;
  auto const denominator = p_denominator < 0 ? -p_denominator : p_denominator;
  auto const magnitude = (numerator + denominator / 2) / denominator;
  return negative ? -magnitude : magnitude;
}

/**
 * @brief Convert one adc16_stream sample of the internal temperature sensor
 * into centi-degrees Celsius.
 *
 * Implements TRM (SLAU846E) equations 17 and 18:
 *
 *   V      = (VREF / 2^12) * (code - 0.5)
 *   T      = (1 / TSc) * (V_sample - V_trim) + TSTRIM
 *
 * The samples and the factory code both use the same VDDA reference, so the
 * 0.5 code offsets cancel and only the code difference matters.
 *
 * @param p_sample - 16-bit sample from adc_driver (12-bit code bit-duplicated
 * up to 16 bits, as adc16_stream requires)
 * @param p_trim_code - FACTORYREGION TEMP_SENSE0 (12-bit code at 30 degC)
 */
constexpr temperature_sample_t to_temperature(std::uint16_t p_sample,
                                              std::uint16_t p_trim_code)
{
  // Bit duplication put the 12-bit code in the top 12 bits; shifting it back
  // down recovers it exactly.
  auto const code = static_cast<std::int64_t>(p_sample >> 4U);
  auto const code_delta = code - static_cast<std::int64_t>(p_trim_code);
  // centi-degC = code_delta * (VREF / 4096) / TSc * 100
  auto const temperature_delta =
    divide_rounded(code_delta * reference_microvolts * 100,
                   adc_12_bit_codes * sensor_slope_microvolts_per_degree);
  auto const temperature = trim_temperature + temperature_delta;
  return static_cast<temperature_sample_t>(
    std::clamp<std::int64_t>(temperature,
                             std::numeric_limits<temperature_sample_t>::min(),
                             std::numeric_limits<temperature_sample_t>::max()));
}

/// Upscale a 12-bit code to 16 bits by bit duplication (adc.hpp contract).
constexpr std::uint16_t upscale_12_to_16(std::uint16_t p_code)
{
  auto const code = static_cast<std::uint16_t>(p_code & 0x0FFFU);
  return static_cast<std::uint16_t>((code << 4U) | (code >> 8U));
}

void store_u32(std::uint32_t p_value, std::uint8_t* p_destination)
{
  for (std::size_t index = 0; index < sizeof(p_value); index++) {
    p_destination[index] = static_cast<std::uint8_t>(p_value >> (8 * index));
  }
}

std::uint32_t load_u32(std::uint8_t const* p_source)
{
  std::uint32_t value = 0;
  for (std::size_t index = 0; index < sizeof(value); index++) {
    value |= std::uint32_t{ p_source[index] } << (8 * index);
  }
  return value;
}

sample_payload encode_samples(temperature_batch const& p_samples)
{
  sample_payload payload{};
  for (std::size_t index = 0; index < p_samples.size(); index++) {
    auto const bits = static_cast<std::uint16_t>(p_samples[index]);
    payload[2 * index] = static_cast<std::uint8_t>(bits);
    payload[2 * index + 1] = static_cast<std::uint8_t>(bits >> 8U);
  }
  return payload;
}

temperature_batch decode_samples(sample_payload const& p_payload)
{
  temperature_batch samples{};
  for (std::size_t index = 0; index < samples.size(); index++) {
    auto const bits = static_cast<std::uint16_t>(
      p_payload[2 * index] | (p_payload[2 * index + 1] << 8U));
    samples[index] = static_cast<temperature_sample_t>(bits);
  }
  return samples;
}

response_packet build_response(sample_payload const& p_payload,
                               std::uint32_t p_crc)
{
  response_packet packet{};
  std::copy(p_payload.begin(), p_payload.end(), packet.begin());
  store_u32(p_crc, packet.data() + payload_size);
  return packet;
}

/// Mean of a batch in centi-degC, rounded to nearest.
std::int32_t average_temperature(temperature_batch const& p_samples)
{
  std::int64_t sum = 0;
  for (auto const sample : p_samples) {
    sum += sample;
  }
  return static_cast<std::int32_t>(
    divide_rounded(sum, static_cast<std::int64_t>(p_samples.size())));
}

/// Print centi-degC as "-12.34" without pulling in floating-point printf.
struct centi_degrees_text
{
  std::array<char, 12> text{};
};

centi_degrees_text format_centi_degrees(std::int32_t p_value)
{
  centi_degrees_text result;
  auto const negative = p_value < 0;
  auto const magnitude =
    static_cast<std::uint32_t>(negative ? -p_value : p_value);
  std::snprintf(result.text.data(),
                result.text.size(),
                "%s%" PRIu32 ".%02" PRIu32,
                negative ? "-" : "",
                magnitude / 100,
                magnitude % 100);
  return result;
}

enum class led_color : std::uint8_t
{
  off,
  blue,
  yellow,
  red,
  magenta,
};

// Display color bands, in centi-degC. These are die temperatures. The
// conversion uses the VDDA (USB 3.3 V) reference that TEMP_SENSE0 was
// measured with, so a supply that sags by 0.1 V reads about 12 degC cold;
// the boards read 20-23 degC at rest on a healthy supply. The wide yellow
// band keeps a resting board yellow, a fingertip on the MCU pushes it to
// red, and a cold pack is needed for blue.
constexpr std::int32_t cold_below = 1600;  // < 16.00 degC is blue
constexpr std::int32_t hot_from = 2500;    // >= 25.00 degC is red

led_color color_for(std::int32_t p_centi_degrees)
{
  if (p_centi_degrees < cold_below) {
    return led_color::blue;
  }
  if (p_centi_degrees >= hot_from) {
    return led_color::red;
  }
  return led_color::yellow;
}

char const* color_name(led_color p_color)
{
  switch (p_color) {
    case led_color::blue:
      return "blue (cold)";
    case led_color::yellow:
      return "yellow (warm)";
    case led_color::red:
      return "red (hot)";
    case led_color::magenta:
      return "magenta (CRC error)";
    case led_color::off:
    default:
      return "off";
  }
}
}  // namespace

namespace {
// ---------------------------------------------------------------------------
// Application
// ---------------------------------------------------------------------------

constexpr lab6::serial::settings link_settings{ 115200 };

/// Bytes the ring holds that the application hasn't consumed yet.
std::size_t available(lab6::serial& p_port, std::size_t p_read_cursor)
{
  auto const size = p_port.receive_buffer().size();
  return (p_port.receive_cursor() + size - p_read_cursor) % size;
}

/// Pop one received byte. The caller checks available() first.
std::uint8_t take_byte(lab6::serial& p_port, std::size_t& p_read_cursor)
{
  auto const buffer = p_port.receive_buffer();
  auto const byte = buffer[p_read_cursor];
  p_read_cursor = (p_read_cursor + 1) % buffer.size();
  return byte;
}

std::uint32_t ticks_to_nanoseconds(std::uint64_t p_ticks,
                                   std::uint32_t p_frequency)
{
  return static_cast<std::uint32_t>(p_ticks * 1'000'000'000ULL / p_frequency);
}

struct timed_crc
{
  std::uint32_t value;
  std::uint64_t ticks;
};

timed_crc time_crc(lab6::crc32& p_crc,
                   lab2::steady_clock& p_clock,
                   std::span<std::uint8_t const> p_data)
{
  auto const start = p_clock.uptime();
  p_crc.reset();
  p_crc.update(p_data);
  auto const value = p_crc.checksum();
  auto const end = p_clock.uptime();
  return { value, end - start };
}

/// Startup self-test: both CRC engines on the standard "123456789" check
/// string (expected 0xCBF43926) and on split updates.
bool crc_self_test(lab6::crc32& p_hardware, lab6::crc32& p_software)
{
  constexpr std::array<std::uint8_t, 9> check{ '1', '2', '3', '4', '5',
                                               '6', '7', '8', '9' };
  constexpr std::uint32_t expected = 0xCBF4'3926U;
  p_hardware.reset();
  p_hardware.update(std::span<std::uint8_t const>(check).first(3));
  p_hardware.update(std::span<std::uint8_t const>(check).subspan(3));
  auto const hardware = p_hardware.checksum();
  p_software.reset();
  p_software.update(check);
  auto const software = p_software.checksum();
  std::printf("CRC self-test \"123456789\": hardware 0x%08" PRIX32
              ", software 0x%08" PRIX32 ", expected 0x%08" PRIX32 "\n",
              hardware,
              software,
              expected);
  return hardware == expected && software == expected;
}

// ROLE 4: each queue item is a complete COPY of the 64 converted samples.
SemaphoreHandle_t sample_request = nullptr;
QueueHandle_t batch_queue = nullptr;
[[maybe_unused]] StaticSemaphore_t sample_request_storage;
[[maybe_unused]] StaticQueue_t batch_queue_control;
[[maybe_unused]] std::array<std::uint8_t, sizeof(temperature_batch)>
  batch_queue_storage{};

// request_task only receives from the link; transmit_task only sends.
// main() configures the shared driver before either task starts.
[[maybe_unused]] void request_task(void* p_context)
{
  auto& link = *static_cast<serial_driver*>(p_context);
  std::size_t read_cursor = link.receive_cursor();
  while (true) {
    while (available(link, read_cursor) != 0) {
      if (take_byte(link, read_cursor) == sample_request_command) {
        // A binary semaphore coalesces requests if one is already pending.
        (void)xSemaphoreGive(sample_request);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// ROLE 1: wait for a request, acquire one batch, convert, and queue a copy.
[[maybe_unused]] void sampler_task(void*)
{
  adc_driver adc;
  auto const trim = adc_driver::calibration_code();
  std::array<std::uint16_t, samples_per_batch> raw{};
  temperature_batch batch{};
  while (true) {
    (void)xSemaphoreTake(sample_request, portMAX_DELAY);
    adc.read(raw);
    for (std::size_t index = 0; index < batch.size(); index++) {
      batch[index] = to_temperature(raw[index], trim);
    }
    // xQueueSend copies all 128 bytes; the next read cannot change the item
    // that transmit_task receives. Queue depth is one, with backpressure.
    (void)xQueueSend(batch_queue, &batch, portMAX_DELAY);
  }
}

// ROLE 3: consume a copied batch, benchmark both CRC engines, and transmit.
[[maybe_unused]] void transmit_task(void* p_context)
{
  auto& link = *static_cast<serial_driver*>(p_context);
  timg12_steady_clock clock;
  crc32_hardware hardware_crc;
  crc32_software software_crc;
  mspm0_input_pin corrupt_button(button_s2);
  (void)corrupt_button.configure({ lab1::pin_resistor::pull_up });
  if (!crc_self_test(hardware_crc, software_crc)) {
    std::puts("CRC self-test FAILED; check CRC settings before the demo");
  }
  temperature_batch batch{};
  std::uint32_t batch_number = 0;
  while (true) {
    (void)xQueueReceive(batch_queue, &batch, portMAX_DELAY);
    batch_number++;
    auto payload = encode_samples(batch);
    auto const hardware = time_crc(hardware_crc, clock, payload);
    auto const software = time_crc(software_crc, clock, payload);

    // Read S2 after CRC computation and flip exactly one payload bit.
    auto const corrupt = !corrupt_button.level();
    if (corrupt) {
      auto const bit = (batch_number * 37U) % (payload.size() * 8U);
      payload[bit / 8U] ^= static_cast<std::uint8_t>(1U << (bit % 8U));
    }
    auto const packet = build_response(payload, hardware.value);
    link.write(packet);

    auto const average = average_temperature(batch);
    std::printf("Batch %" PRIu32 ": avg %s degC, %u samples%s\n",
                batch_number,
                format_centi_degrees(average).text.data(),
                static_cast<unsigned>(batch.size()),
                corrupt ? " (S2 held: one transmitted bit flipped)" : "");
    auto const frequency = clock.frequency();
    std::printf("  CRC hw 0x%08" PRIX32 " in %" PRIu32 " ns; sw 0x%08" PRIX32
                " in %" PRIu32 " ns; %s\n",
                hardware.value,
                ticks_to_nanoseconds(hardware.ticks, frequency),
                software.value,
                ticks_to_nanoseconds(software.ticks, frequency),
                hardware.value == software.value ? "match" : "MISMATCH");
  }
}

void show_color(led_color p_color,
                lab1::output_pin& p_red,
                lab1::output_pin& p_green,
                lab1::output_pin& p_blue)
{
  p_red.level(p_color == led_color::red || p_color == led_color::yellow ||
              p_color == led_color::magenta);
  p_green.level(p_color == led_color::yellow);
  p_blue.level(p_color == led_color::blue || p_color == led_color::magenta);
}

enum class receive_result : std::uint8_t
{
  complete,
  timed_out,
};

/// Collect exactly one response after the saved receive cursor.
receive_result receive_response(lab6::serial& p_link,
                                std::size_t& p_read_cursor,
                                response_packet& p_packet)
{
  constexpr TickType_t poll_period = pdMS_TO_TICKS(2);
  constexpr TickType_t timeout = pdMS_TO_TICKS(500);
  auto const started = xTaskGetTickCount();
  std::size_t received = 0;
  while (static_cast<TickType_t>(xTaskGetTickCount() - started) < timeout) {
    while (available(p_link, p_read_cursor) != 0 &&
           received < p_packet.size()) {
      auto const byte = take_byte(p_link, p_read_cursor);
      p_packet[received++] = byte;
    }
    if (received == p_packet.size()) {
      return receive_result::complete;
    }
    vTaskDelay(poll_period);
  }
  return receive_result::timed_out;
}

[[maybe_unused]] void display_task(void*)
{
  serial_driver link;
  crc32_hardware hardware_crc;
  // S1 (PA18) drives high when pressed; S2 (PB21) pulls low when pressed.
  mspm0_input_pin s1(button_s1);
  mspm0_input_pin s2(button_s2);
  (void)s1.configure({ lab1::pin_resistor::pull_down });
  (void)s2.configure({ lab1::pin_resistor::pull_up });
  mspm0_output_pin red(led_red);
  mspm0_output_pin green(led_green);
  mspm0_output_pin blue(led_blue);
  for (lab1::output_pin* led : { &red, &green, &blue }) {
    (void)led->configure({});
  }
  show_color(led_color::off, red, green, blue);

  if (!link.configure(link_settings)) {
    std::puts("Link settings rejected");
  }
  std::printf("Role: DISPLAY. UART1 %" PRIu32 " baud on PA8 (TX) / PA9 (RX)\n"
              "Press S1 or S2 to request a batch. Bands: blue < %s, "
              "yellow < %s, red otherwise (degC)\n",
              link_settings.baud_rate,
              format_centi_degrees(cold_below).text.data(),
              format_centi_degrees(hot_from).text.data());

  std::size_t read_cursor = link.receive_cursor();
  auto was_pressed = false;
  std::uint32_t request_number = 0;
  response_packet packet{};

  while (true) {
    // 1. Wait for a press (rising edge of "either button down"), polling
    // every 10 ms, which also debounces.
    auto const pressed = s1.level() || !s2.level();
    auto const new_press = pressed && !was_pressed;
    was_pressed = pressed;
    if (!new_press) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    request_number++;

    // 2. Discard anything stale, then ask.
    read_cursor = link.receive_cursor();
    constexpr std::array<std::uint8_t, 1> request{ sample_request_command };
    link.write(request);

    // 3. Wait for the reply.
    if (receive_response(link, read_cursor, packet) !=
        receive_result::complete) {
      std::printf("Request %" PRIu32
                  ": no reply from the sensor board within 500 ms "
                  "(check TX/RX/GND wiring and that it runs the sensor "
                  "role)\n",
                  request_number);
      show_color(led_color::off, red, green, blue);
      continue;
    }

    // 4. Check the CRC.
    sample_payload payload{};
    std::copy_n(packet.begin(), payload.size(), payload.begin());
    auto const received_crc = load_u32(packet.data() + payload_size);
    hardware_crc.reset();
    hardware_crc.update(payload);
    auto const computed_crc = hardware_crc.checksum();
    if (computed_crc != received_crc) {
      std::printf("Request %" PRIu32 ": CRC MISMATCH (received 0x%08" PRIX32
                  ", computed 0x%08" PRIX32 "). Packet dropped; LED %s\n",
                  request_number,
                  received_crc,
                  computed_crc,
                  color_name(led_color::magenta));
      show_color(led_color::magenta, red, green, blue);
      continue;
    }

    // 5. Valid: average and show.
    auto const samples = decode_samples(payload);
    auto const average = average_temperature(samples);
    auto const color = color_for(average);
    show_color(color, red, green, blue);
    std::printf(
      "Request %" PRIu32 ": CRC 0x%08" PRIX32
      " OK, avg %s degC over %u samples (min %s, max %s); "
      "LED %s\n",
      request_number,
      received_crc,
      format_centi_degrees(average).text.data(),
      static_cast<unsigned>(samples.size()),
      format_centi_degrees(*std::min_element(samples.begin(), samples.end()))
        .text.data(),
      format_centi_degrees(*std::max_element(samples.begin(), samples.end()))
        .text.data(),
      color_name(color));
  }
}

// Static allocation: each sensor task has its own stack and control block.
// The request task needs little stack; printf users get 4 KiB each.
constexpr std::uint32_t request_stack_words = 256;
constexpr std::uint32_t sampler_stack_words = 512;
constexpr std::uint32_t transmit_stack_words = 1024;
constexpr std::uint32_t task_stack_words = 1024;
[[maybe_unused]] StaticTask_t request_task_tcb, sampler_task_tcb,
  transmit_task_tcb, display_task_tcb;
[[maybe_unused]] StackType_t request_task_stack[request_stack_words];
[[maybe_unused]] StackType_t sampler_task_stack[sampler_stack_words];
[[maybe_unused]] StackType_t transmit_task_stack[transmit_stack_words];
[[maybe_unused]] StackType_t display_task_stack[task_stack_words];

// Select the board role here, or set a compiler define for each build.
enum class device_role : std::uint8_t
{
  sensor,
  display
};
#if defined(LAB5_DEVICE_ROLE_DISPLAY) && defined(LAB5_DEVICE_ROLE_SENSOR)
#error "Select only one Lab 5 device role"
#elif defined(LAB5_DEVICE_ROLE_DISPLAY)
constexpr device_role this_device = device_role::display;
#elif defined(LAB5_DEVICE_ROLE_SENSOR)
constexpr device_role this_device = device_role::sensor;
#else
constexpr device_role this_device = device_role::sensor;  // Change for display.
#endif

}  // namespace

// The supplied SysConfig file configures only the clock tree/debug pins.
// Driver constructors configure the ADC, DMA, UART, and CRC peripherals.
extern "C" void SYSCFG_DL_init(void);

int main()
{
  SYSCFG_DL_init();
  std::printf("\nLab 5: ADC + UART + CRC (FreeRTOS)\n");

  if constexpr (this_device == device_role::sensor) {
    // Construct the shared UART driver once; its lifetime is the whole run.
    static serial_driver link;
    if (!link.configure(link_settings)) {
      std::puts("Link settings rejected");
      while (true) {
      }
    }
    sample_request = xSemaphoreCreateBinaryStatic(&sample_request_storage);
    batch_queue = xQueueCreateStatic(1,
                                     sizeof(temperature_batch),
                                     batch_queue_storage.data(),
                                     &batch_queue_control);
    configASSERT(sample_request != nullptr);
    configASSERT(batch_queue != nullptr);
    std::puts("Role: SENSOR; UART1 115200 baud, PA8 TX / PA9 RX");
    std::puts("Hold sensor S2 during a request to corrupt one payload bit.");
    auto const request_handle = xTaskCreateStatic(request_task,
                                                  "request_task",
                                                  request_stack_words,
                                                  &link,
                                                  tskIDLE_PRIORITY + 1,
                                                  request_task_stack,
                                                  &request_task_tcb);
    auto const sampler_handle = xTaskCreateStatic(sampler_task,
                                                  "sampler_task",
                                                  sampler_stack_words,
                                                  nullptr,
                                                  tskIDLE_PRIORITY + 1,
                                                  sampler_task_stack,
                                                  &sampler_task_tcb);
    auto const transmit_handle = xTaskCreateStatic(transmit_task,
                                                   "transmit_task",
                                                   transmit_stack_words,
                                                   &link,
                                                   tskIDLE_PRIORITY + 1,
                                                   transmit_task_stack,
                                                   &transmit_task_tcb);
    configASSERT(request_handle != nullptr);
    configASSERT(sampler_handle != nullptr);
    configASSERT(transmit_handle != nullptr);
  } else {
    auto const display_handle = xTaskCreateStatic(display_task,
                                                  "display_task",
                                                  task_stack_words,
                                                  nullptr,
                                                  tskIDLE_PRIORITY + 1,
                                                  display_task_stack,
                                                  &display_task_tcb);
    configASSERT(display_handle != nullptr);
  }

  vTaskStartScheduler();

  // Only reached if vTaskStartScheduler() itself fails to start. The
  // idle/timer task memory callbacks required by
  // configSUPPORT_STATIC_ALLOCATION are already supplied by the freertos_kernel
  // project's dpl/StaticAllocs_freertos.c - don't redefine them here.
  while (true) {
  }
}
