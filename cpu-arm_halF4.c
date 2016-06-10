
/** \file
  \brief CPU initialisation, ARM specific part.

  To be included from cpu.c, for details see there.
*/

#if defined TEACUP_C_INCLUDE && defined __ARM_HALF4__
  
#include "config_wrapper.h"
#include "stm32f4xx_hal_conf.h"    

void cpu_init() {
  /**
    Enable all periphals.
  */
    // Enable power and clocking for all GPIO
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();

}

#endif /* defined TEACUP_C_INCLUDE && defined __ARMEL__ */
