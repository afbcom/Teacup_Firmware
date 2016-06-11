
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
volatile uint16_t BSS adc_buffer[2][NUM_TEMP_SENSORS * OVERSAMPLE];
// uint16_t BSS adc_result[2][NUM_TEMP_SENSORS];

static ADC_HandleTypeDef hadc;
static DMA_HandleTypeDef hdma;
static ADC_ChannelConfTypeDef sConfig;


// Private functions
void init_analog(void);
void init_analog_pins(void);
void init_dma(void);


/** Initialize the analog subsystem.

  Initialize the ADC and start hardware scan for all sensors.
*/
void analog_init() {

  if (NUM_TEMP_SENSORS) { // At least one channel in use.
    init_analog_pins();  
    init_dma();                     
    init_analog();
  }

}


void init_analog_pins(){

  GPIO_InitTypeDef  igpio;

  #undef DEFINE_TEMP_SENSOR
  /*
   config analog pins
   1. analog mode
   2. no pullup
   3. high speed
  */
  #define DEFINE_TEMP_SENSOR(name, type, pin, additional) \
    igpio.Pin = GPIO_PIN_ ## ( pin ## _PIN);              \
    igpio.Mode = GPIO_MODE_ANALOG;                        \
    igpio.Pull = GPIO_NOPULL;                             \
    igpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;              \
    HAL_GPIO_Init( pin ## _PORT , &igpio);                \
  #include "config_wrapper.h"               
  #undef DEFINE_TEMP_SENSOR
}


void init_analog() {

  __HAL_RCC_ADC1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc.Init.Resolution = ADC_RESOLUTION_10B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.NbrOfConversion = NUM_TEMP_SENSORS;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.DMAContinuousRequests = ENABLE;

   hadc->DMA_Handle = &hdma;  

  // for loop over each channel (0..15) for sequence
  #undef DEFINE_TEMP_SENSOR
  // for PIO ## ADC >= 10 SRPR1 and ADC -10, else SMPR 2
  // 0x03 = 28 cycles
  // subt line is to keep compiler happy
  #define DEFINE_TEMP_SENSOR(name, type, pin, additional) \
    if (NUM_TEMP_SENSORS) {                               \
       sConfig.Channel = pin ## _ADC;                     \
       sConfig.Rank = TEMP_SENSOR_ ## name;               \
       sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;    \
       HAL_ADC_ConfigChannel( &hadc, &sConfig );          \
    }                                                     \
  #include "config_wrapper.h"
  #undef DEFINE_TEMP_SENSOR

  HAL_ADC_Init( &hadc );
  HAL_ADC_Start( &hadc );

}

/** Init the DMA for ADC
*/

void init_dma() {
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

  hdma.XferM1CpltCallback = &DMA2_Stream4_Handler;

  HAL_DMAEx_MultiBufferStart_IT( &hdma, 
                                 (uint32_t)&ADC1->DR,
                                 (uint32_t)&adc_buffer[0],  
                                 (uint32_t)&adc_buffer[1],
                                 NUM_TEMP_SENSORS * OVERSAMPLE );


  NVIC_SetPriority(DMA2_Stream4_IRQn,
  NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 3, 1));

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
      temp = adc_buffer[!(DMA2_Stream4->CR & DMA_SxCR_CT)][index + NUM_TEMP_SENSORS * i];
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
*/

void analog_tick(void) {
  HAL_ADC_Start(&hadc);
}

#endif /* defined TEACUP_C_INCLUDE && defined __ARMEL__ */
