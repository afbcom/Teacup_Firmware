
/** \file
  \brief CPU initialisation, ARM specific part.

  To be included from cpu.c, for details see there.
*/

#if defined TEACUP_C_INCLUDE && defined __ARM_STM32F4HAL__
  
#include "config_wrapper.h"
#include "stm32f4xx_hal_conf.h"    

void cpu_init() {
  /**
    Enable all periphals.
  */
    // Enable power and clocking for all GPIO
	GPIO_CLK_ENABLE(GPIO_CLK_ENABLE_ALL);
}

#endif /* defined TEACUP_C_INCLUDE && defined __ARMEL__ */
