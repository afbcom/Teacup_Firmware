
/** \file
  \brief Analog subsystem, ARM specific part.
  STM32F4 goes a different way. The ADC don't have a register for
  each channel. We are using DMA instead.
*/

#if defined TEACUP_C_INCLUDE && defined __ARM_STM32F4HAL__

#include "stm32f4xx.h"
#include "arduino.h"
#include "delay.h"
#include "temp.h"
#include "stm32f4xx_hal_conf.h"  


// DMA ADC-buffer
#define OVERSAMPLE 6
#define SELECT_FROM_DOUBLE_BUFFER !(DMA2_Stream4->CR & DMA_SxCR_CT)  

volatile uint16_t BSS adc_buffer[2][NUM_TEMP_SENSORS * OVERSAMPLE];

static ADC_HandleTypeDef hadc;
static DMA_HandleTypeDef hdma;
static ADC_ChannelConfTypeDef sConfig;

// Private functions
void init_analog(void);
void init_analog_pins(void);
void init_dma(void);

static void DMA_MultiBuff_Start_IT(void);
static void DMA_Base_Init();


/** Initialize the analog subsystem.

  Initialize the ADC and start hardware scan for all sensors.
*/
void analog_init() {

  if (NUM_TEMP_SENSORS) {                       // At least one channel in use.
    init_analog();
    init_dma();

  }

}

/** Initialize all analog pins from config
 
  Initialize the pins to analog mode, no pullup/no pulldown, highspeed
*/
void init_analog() {

  __HAL_RCC_ADC1_CLK_ENABLE();

    GPIO_InitTypeDef  igpio;

  /*
   config analog pins
   1. analog mode
   2. no pullup
   3. high speed
  */
  #undef DEFINE_TEMP_SENSOR
  #define DEFINE_TEMP_SENSOR(name, type, pin, additional)   \
    igpio.Pin =  0x01 << pin ## _PIN;                       \
    igpio.Mode = GPIO_MODE_ANALOG;                          \
    igpio.Pull = GPIO_NOPULL;                               \
    igpio.Speed = GPIO_SPEED_FREQ_HIGH;                     \
    HAL_GPIO_Init( pin ## _PORT , &igpio);                  \
    hadc.Instance = ADC1;                                   \
    hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;    \
    hadc.Init.ScanConvMode = ENABLE;                        \
    hadc.Init.Resolution = ADC_RESOLUTION_10B;              \
    hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;              \
    hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;        \
    hadc.Init.ContinuousConvMode = ENABLE;                  \
    hadc.Init.DiscontinuousConvMode = DISABLE;              \
    hadc.Init.NbrOfConversion = NUM_TEMP_SENSORS;           \
    hadc.Init.DMAContinuousRequests = ENABLE;               \
    hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;              \
    hadc.DMA_Handle = &hdma;                                \
    HAL_ADC_Init( &hadc );                                  \
    sConfig.Channel = 1;                          \
    sConfig.Rank = 1 ;                   \
    sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;         \
    HAL_ADC_ConfigChannel( &hadc, &sConfig );                            
  #include "config_wrapper.h"               
  #undef DEFINE_TEMP_SENSOR



  // for loop over each channel (0..15) for sequence
  // // for PIO ## ADC >= 10 SRPR1 and ADC -10, else SMPR 2
  // // 0x03 = 28 cycles
  // #define DEFINE_TEMP_SENSOR(name, type, pin, additional) \
                                              
  // #include "config_wrapper.h"
  // #undef DEFINE_TEMP_SENSOR



 

  // #define DEFINE_TEMP_SENSOR(name, type, pin, additional) \
  // if (NUM_TEMP_SENSORS) { \
  //   uint8_t subt = (pin ## _ADC > 9) ? 10 : 0; \
  //   if (pin ## _ADC > 9) { \
  //     ADC1->SMPR1 |= (uint32_t)0x03 << (3 * ((pin ## _ADC) - subt)); \
  //   } else { \
  //     ADC1->SMPR2 |= (uint32_t)0x03 << (3 * ((pin ## _ADC) - subt)); \
  //   } \
  //   subt = (TEMP_SENSOR_ ## name < 7) ? 0 : (TEMP_SENSOR_ ## name < 13) ? 7 : 13; \
  //   if (TEMP_SENSOR_ ## name < 7) { \
  //     ADC1->SQR3 |= pin ## _ADC << (5 * TEMP_SENSOR_ ## name - subt); \
  //   } else \
  //   if (TEMP_SENSOR_ ## name < 13) { \
  //     ADC1->SQR2 |= pin ## _ADC << (5 * (TEMP_SENSOR_ ## name - subt)); \
  //   } else { \
  //     ADC1->SQR1 |= pin ## _ADC << (5 * (TEMP_SENSOR_ ## name - subt)); \
  //   } \
  // }
  // #include "config_wrapper.h"
  // #undef DEFINE_TEMP_SENSOR


  /*
   *  In order to use HAL with a double buffered DMA stream
   *  we need to do a bit of tweaking. Only HAL_ADC_Start_DMA
   *  sets the ADC_CR2_DMA bits. So we need to reset the DMA
   *  to operate in double buffered mode after starting ADC
   */
  HAL_ADC_Start_DMA( &hadc , adc_buffer, NUM_TEMP_SENSORS ); 


}

/** Init the DMA for ADC
*/

void init_dma() {

  // Enable clock
  __HAL_RCC_DMA2_CLK_ENABLE();

  DMA_Base_Init();
  // 10. Enable DMA-Stream
  DMA_MultiBuff_Start_IT();

  // while(!(DMA2_Stream4->CR & DMA_SxCR_EN));

  HAL_NVIC_SetPriority(DMA2_Stream4_IRQn,3,1);

}

static void DMA_Base_Init(){
 /**
    We have two DMA streams for ADC1. (DMA2_Stream0 and DMA2_Stream4)
    We take DMA2 Stream4.
    See reference manual 9.3.3 channel selection (p. 166)
  */
  hdma.Instance = DMA2_Stream4;
  hdma.Init.Channel = DMA_CHANNEL_0;
  hdma.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma.Init.PeriphInc= DMA_PINC_DISABLE;
  hdma.Init.MemInc = DMA_MINC_ENABLE;
  hdma.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma.Init.Mode = DMA_CIRCULAR;
  hdma.Init.Priority = DMA_PRIORITY_VERY_HIGH;
  hdma.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  hdma.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_1QUARTERFULL;
  hdma.Init.MemBurst = DMA_MBURST_SINGLE;
  hdma.Init.PeriphBurst = DMA_PBURST_SINGLE;

  HAL_DMA_Init(&hdma);

}

static void DMA_MultiBuff_Start_IT(){

  // 4. total number of data items
  DMA2_Stream4->NDTR = NUM_TEMP_SENSORS * OVERSAMPLE;

  
  // 2. perihperal port register address
  DMA2_Stream4->PAR = (uint32_t)&ADC1->DR;

  // 3. memory address
  DMA2_Stream4->M0AR = (uint32_t)&adc_buffer[0];
  DMA2_Stream4->M1AR = (uint32_t)&adc_buffer[1];

  DMA2_Stream4->CR |= DMA_SxCR_TCIE | DMA_SxCR_DBM | DMA_SxCR_EN;


  /*
    The HAL functions for ouble buffer look identical but
    do not produce the same results. It is possible (likely)
    that HAL the error handling in HAL functions is changing
    how things are set
  */
  // HAL_DMAEx_MultiBufferStart_IT( &hdma, 
  //                              (uint32_t)&ADC1->DR,
  //                              (uint32_t)&adc_buffer[0],  
  //                              (uint32_t)&adc_buffer[1],
  //                              NUM_TEMP_SENSORS * OVERSAMPLE );

}



/** Read analog value.

  \param channel Channel to be read.

  \return Analog reading, 10-bit right aligned.

*/



uint16_t analog_read(uint8_t index) {
  if (NUM_TEMP_SENSORS > 0) {
    // uint16_t buffer[NUM_TEMP_SENSORS * OVERSAMPLE];
    uint16_t r = 0;
    uint16_t temp;
    uint16_t max_temp = 0;
    uint16_t min_temp = 0xffff;

    uint8_t i;
    // take a copy of the last adc_buffer
    // ATOMIC_START
    // ATOMIC_END

    for (i = 0; i < OVERSAMPLE; i++) {
      temp = adc_buffer[SELECT_FROM_DOUBLE_BUFFER][index + NUM_TEMP_SENSORS * i];
      max_temp = max_temp > temp ? max_temp : temp;
      min_temp = min_temp < temp ? min_temp : temp;
      r += temp;
    }

    r = (r - max_temp - min_temp) / (OVERSAMPLE - 2);
    // this is nearly the same like the freerun under other systems
    // ADC1->CR2 |= ADC_CR2_SWSTART | ADC_CR2_CONT;
    return r;
  } else {
    return 0;
  }
}

/**
  Restart ADC every tick.

  On the STM32F4 this function may not be necessary since
  we can initialize our ADC to operate in continous conbersion mode.
*/

void analog_tick(void) {
  // ADC1->CR2 |= ADC_CR2_SWSTART | ADC_CR2_CONT;
}

/**
  DMA2 Stream4 interrupt.

  Happens every time the complete stream is written.
  In that case, we can turn of the ADC until we read it.

  Must have the same name as in cmsis-startup_stm32f411xe.s.
*/
void DMA2_Stream4_Handler(void) {

  if(__HAL_DMA_GET_FLAG( &hdma, __HAL_DMA_GET_TC_FLAG_INDEX(&hdma)) == SET ){
    __HAL_DMA_CLEAR_FLAG(&hdma, RESET);
    HAL_ADC_Stop(&hadc);
  }
  
}

#endif /* defined TEACUP_C_INCLUDE && defined __ARMEL__ */
