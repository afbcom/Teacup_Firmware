
/** \file
  \brief Manage heaters, including PID and PWM, ARM specific part.
*/

#if defined TEACUP_C_INCLUDE && defined __ARM_STM32F4HAL__

#include "stm32f4xx.h"
#include "pinio.h"
#include "sersendf.h"
#include "debug.h"
#include "stm32f4xx_hal_conf.h"         

/**
  Test configuration.
*/
#ifdef EECONFIG
  #error EEPROM handling (EECONFIG) not yet supported on ARM.
#endif

#ifdef BANG_BANG
  #error BANG_BANG not supported on ARM. You may set PWM frequency of one \
         or all heater(s) to zero, which gives similar, better behaviour.
#endif

/** \def PWM_SCALE

  G-code standard (if such a thing exists at all) gives a heater setting
  range between 0 (off) and 255 (full on), so we let the PWM timers count up
  to 255. Doing so allows to set the prescaler for these frequencies (all on
  a 96 MHz CPU clock):

    prescaler     frequency         prescaler     frequency

            0     367.5 kHz                 4      75.3 kHz
            1     188.2 kHz                 5      62.7 kHz
            2     125.5 kHz               ...       ...
            3      94.1 kHz             65535      5.74 Hz

  As one can see, frequency steps are rather coarse on the high end and
  become finer grained the lower it gets.

  If this range is generally too high for your purposes, you can set PWM_SCALE
  to multiples of 255 to lower the range. Doubling it to 510 moves the
  frequency range to 2.9 Hz...183.8 kHz, quadrupling it to 1020 moves the range
  to 1.5 Hz...91.9 kHz and so on. The highest allowed number is 65535.

  That said, code below calculates the best prescaler value for a configured
  frequency, so you should bother about PWM_SCALE only of you need frequencies
  below 6 Hz.
*/
#define PWM_SCALE 255

// some helper macros
#define _EXPANDER(pre, val, post) pre ## val ## post
#define EXPANDER(pre, val, post) _EXPANDER(pre, val, post)

/** \struct heater_definition_t

  Holds pinout data to allow changing PWM output after initialisation. Port,
  pin, PWM channel if used. After inititalisation we can no longer do the
  #include "config_wrapper.h" trick.
*/
typedef struct {
  union {
    /// Pointer to the capture compare register which changes PWM duty.
    __IO uint32_t* ccr;
    /// Pointer to the port for non-PWM pins.
    __IO uint32_t* bsrr;
  };
  uint16_t masked_pin;
  uint8_t uses_pwm;
} heater_definition_t;

#undef DEFINE_HEATER
#define DEFINE_HEATER(name, pin, invert, pwm) \
  { \
    { pwm && pin ## _TIMER ? \
      &(pin ## _TIMER-> EXPANDER(CCR, pin ## _CHANNEL,)) : \
      &(pin ## _PORT->BSRR) }, \
    MASK(pin ## _PIN), \
    pwm && pin ## _TIMER \
  },
static const heater_definition_t heaters[NUM_HEATERS] = {
  #include "config_wrapper.h"
};
#undef DEFINE_HEATER

/** Initialise heater subsystem.

  Initialise PWM timers, etc. Inspired by heater-arm_lpc11xx.c (pwm.c in LPC1343CodeBase):

    https://github.com/microbuilder/LPC1343CodeBase

  Note that PWM is inversed, pins start at Low by chip design. When the pin's
  counter is reached, they're set to High. Reaching the timer reset counter is
  programmed to reset everything and start over. Thus, having both counter
  matches similar gives a low duty, having the pin counter match zero gives
  full on.

  For simplicity we reset all timer counters always on Match 3 and always
  at PWM_SCALE (255 per default), so setting a pin match to PWM_SCALE / 2
  gives 50% duty, setting it to PWM_SCALE gives full off. This choice disallows
  using a pin connected to a Match 3, working around this would make code much
  more complicated (and still not allow to use more than 3 pins per timer).

  On ARM we can define a PWM frequency pretty fine grained, so we take the
  'pwm' value of DEFINE_HEATER() not only wether to use PWM at all, but also
  to define the PWM frequency. Float values are allowed.

  If there's more than one pin on a timer, they share the same PWM frequency;
  the frequency choosen is the one of the pin defined last.
*/
void heater_init() {
  /**
    Pins on the STM32F411RE are usable as following, N are negated pin (active low)
    some pins are commented out (-) because they are shared. You can change this 
    in arduino_stm32f4xx. But take care! You could pwm two pins simultanious or disable
    other important functions (serial connection).
    PWM5 = TIM5 = Stepper timer.

      pin      timer/channel      alt func for PWM   other uses

      PIOA_0   PWM2/1, 5/1        01, 02            AD0
      PIOA_1   PWM2/2, 5/2        01, 02            MOSI4, AD1
      - PIOA_2   PWM2/3, 5/3, 9/1   01, 02, 03        TX2, AD2, UART!
      - PIOA_3   PWM2/4, 5/4, 9/2   01, 02, 03        RX2, AD3, UART!
      - PIOA_5   PWM2/1             01                LED1, SCK1, AD5
      - PIOA_6   PWM3/1             02                MISO1, AD6
      - PIOA_7   PWM1/1N, 3/2       01, 02            MOSI1, AD7
      PIOA_8   PWM1/1             01                SCL1
      PIOA_9   PWM1/2             01                TX1
      PIOA_10  PWM1/3             01                MOSI5, RX1
      PIOA_11  PWM1/4             01                TX6, MISO4
      - PIOA_15  PWM2/1             01                NSS1, TX1
      
      PIOB_0   PWM1/2N, 3/3       01, 02            SCK5, CK5, AD8
      PIOB_1   PWM1/3N, 3/4       01, 02            NSS4, WS5, AD9
      - PIOB_3   PWM2/2             01                SDA2, SCK3
      PIOB_4   PWM3/1             02                SDA3, MISO3
      PIOB_5   PWM3/2             02                MOSI3
      PIOB_6   PWM4/1             02                SCL1, TX1
      PIOB_7   PWM4/2             02                SDA1, RX1
      PIOB_8   PWM4/3, 10/1       02, 03            SCL1, MOSI5
      PIOB_9   PWM4/4, 11/1       02, 03            SDA1, NSS2
      PIOB_10  PWM2/3             01                SCL3
      - PIOB_13  PWM1/1N            01                SCK2
      - PIOB_14  PWM1/2N            01                MISO2
      - PIOB_15  PWM1/3N            01                MOSI2

      - PIOC_6   PWM3/1             02                MCK2, TX6
      - PIOC_7   PWM3/2             02                SCK2, RX6
      - PIOC_8   PWM3/3             02                SDA3
      - PIOC_9   PWM3/4             02                SDA3
      
  */

  GPIO_InitTypeDef  igpio;
  TIM_HandleTypeDef htim;
  // TIM_OC_InitTypeDef sConfigOC;

  // Auto-generate pin setup.
  #undef DEFINE_HEATER 
  #define DEFINE_HEATER(name, pin, invert, pwm) 					 \
    TIM_OC_InitTypeDef sConfigOC;									 \
    if (pwm && pin ## _TIMER) {                                      \
      uint32_t freq;                                                 \
      uint8_t macro_mask;                                            \
      if (pin ## _TIMER == TIM1) {                                   \
        __HAL_RCC_TIM1_CLK_ENABLE();								 \
        igpio.Alternate = GPIO_AF1_TIM1;}   /* turn on TIM1     */   \
      else if (pin ## _TIMER == TIM2){                               \
       __HAL_RCC_TIM2_CLK_ENABLE();									 \
       igpio.Alternate = GPIO_AF1_TIM2;								 \
        }         /* turn on TIM2     */   							 \
      else if (pin ## _TIMER == TIM3) {                              \
       __HAL_RCC_TIM3_CLK_ENABLE();									 \
       igpio.Alternate = GPIO_AF2_TIM3;}  /* turn on TIM3     */     \
      else if (pin ## _TIMER == TIM4) {                              \
       __HAL_RCC_TIM4_CLK_ENABLE();           /* turn on TIM4     */ \
       igpio.Alternate = GPIO_AF2_TIM4;}							 \
      /* Pin set-up */												 	\
      igpio.Pin = 0x01 << pin ## _PIN;                               	\
	  igpio.Mode = GPIO_MODE_AF_PP; 								 	\
	  igpio.Pull = GPIO_PULLUP; 									 	\
	  igpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH; 							\
	  HAL_GPIO_Init( pin ## _PORT , &igpio); 							\
      freq = F_CPU / PWM_SCALE / (pwm ? pwm : 1); /* Figure PWM freq. */\
	  	if (freq > 65535) freq = 65535;                                 \
	    if (freq < 1) freq = 1;   /*  Timer Base config */              \
	  htim.Instance = pin ## _TIMER;									\
	  htim.Init.Prescaler = freq - 1;  /* 1kHz   */						\
	  htim.Init.CounterMode = TIM_COUNTERMODE_UP;						\
	  htim.Init.Period = 0x0000;										\
	  htim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;					\
	  HAL_TIM_PWM_Init(&htim);											\
    /* Output Control Config*/                                          \
	  sConfigOC.OCMode = TIM_OCMODE_PWM1;								\
	  sConfigOC.Pulse = 0;												\
	  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;						\
	  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;						\
	  HAL_TIM_PWM_ConfigChannel(&htim, &sConfigOC, pin ## _CHANNEL);	\
	  HAL_TIM_PWM_Start(&htim, TIM_CHANNEL_1);							\
    }                                                                  \
    else {                                                             \
      SET_OUTPUT(pin);                                                 \
      WRITE(pin, invert ? 1 : 0);                                      \
    }								

    #include "config_wrapper.h"
    #undef DEFINE_HEATER

  pid_init();
}

/** Set PWM output.

  \param index The heater we're setting the output for.

  \param value The PWM value to write, range 0 (off) to 255 (full on).

  This function is called by M106 or, if a temp sensor is connected to the
  heater, every few milliseconds by its PID handler. Using M106 on an output
  with a sensor changes its setting only for a short moment.
*/
void heater_set(heater_t index, uint8_t value) {

  if (index < NUM_HEATERS) {

    heaters_runtime[index].heater_output = value;

    if (heaters[index].uses_pwm) {
      // Remember, we scale, and duty cycle is inverted.
      *heaters[index].ccr = (uint32_t)value * (PWM_SCALE / 255);

      if (DEBUG_PID && (debug_flags & DEBUG_PID))
        sersendf_P(PSTR("PWM %su = %lu\n"), index, *heaters[index].ccr);
    }
    else {
      *(heaters[index].bsrr) =
        heaters[index].masked_pin << ((value >= HEATER_THRESHOLD) ? 0 : 16);
    }
    if (value)
      power_on();
  }
}

#endif /* defined TEACUP_C_INCLUDE && defined __ARMEL__ */
