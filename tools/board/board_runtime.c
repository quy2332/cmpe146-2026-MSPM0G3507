#include "ti_msp_dl_config.h"

#include <stdio.h>

// These names are entry points required by TI's generated-config and stdio ABI.
void SYSCFG_DL_init(void)
{
  DL_SYSCTL_switchMCLKfromHSCLKtoSYSOSC();
  DL_SYSCTL_setMCLKDivider(DL_SYSCTL_MCLK_DIVIDER_DISABLE);
  DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);
  DL_SYSCTL_setULPCLKDivider(DL_SYSCTL_ULPCLK_DIV_1);

  DL_GPIO_enablePower(GPIOA);
  DL_GPIO_enablePower(GPIOB);
  DL_UART_reset(UART0);
  DL_UART_enablePower(UART0);
  delay_cycles(32);
  DL_GPIO_initPeripheralOutputFunction(IOMUX_PINCM21,
                                       IOMUX_PINCM21_PF_UART0_TX);
  DL_GPIO_initPeripheralInputFunction(IOMUX_PINCM22, IOMUX_PINCM22_PF_UART0_RX);
  DL_UART_ClockConfig const clock = { DL_UART_CLOCK_BUSCLK,
                                      DL_UART_CLOCK_DIVIDE_RATIO_1 };
  DL_UART_Config const config = {
    DL_UART_MODE_NORMAL, DL_UART_DIRECTION_TX_RX,    DL_UART_FLOW_CONTROL_NONE,
    DL_UART_PARITY_NONE, DL_UART_WORD_LENGTH_8_BITS, DL_UART_STOP_BITS_ONE
  };
  DL_UART_setClockConfig(UART0, &clock);
  DL_UART_init(UART0, &config);
  DL_UART_setOversampling(UART0, DL_UART_OVERSAMPLING_RATE_16X);
  DL_UART_configBaudRate(UART0, CPUCLK_FREQ, 115200);
  DL_UART_enableFIFOs(UART0);
  DL_UART_enable(UART0);
  setvbuf(stdout, NULL, _IONBF, 0);
  setvbuf(stdin, NULL, _IONBF, 0);
  puts("\nLaunchPad UART console: 115200 baud, 8N1");
}

// Override the TI runtime's host I/O hooks so no CCS CIO debugger is needed.
int HOSTwrite(int p_descriptor, char const* p_buffer, unsigned p_count)
{
  (void)p_descriptor;
  for (unsigned index = 0; index < p_count; ++index) {
    if (p_buffer[index] == '\n') {
      DL_UART_transmitDataBlocking(UART0, '\r');
    }
    DL_UART_transmitDataBlocking(UART0, (unsigned char)p_buffer[index]);
  }
  return (int)p_count;
}

int HOSTread(int p_descriptor, char* p_buffer, unsigned p_count)
{
  (void)p_descriptor;
  if (p_count == 0) {
    return 0;
  }
  unsigned char const character = DL_UART_receiveDataBlocking(UART0);
  p_buffer[0] = character == '\r' ? '\n' : (char)character;
  return 1;
}
