#ifndef LAB_BOARD_CONFIG_H
#define LAB_BOARD_CONFIG_H

// Standalone UART build only; CCS continues to use its generated configuration.
#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>

#define CPUCLK_FREQ 32000000UL

#ifdef __cplusplus
extern "C"
{
#endif
  void SYSCFG_DL_init(void);
#ifdef __cplusplus
}
#endif

#endif
