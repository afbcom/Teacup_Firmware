
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
#include "stm32f4xx_hal_def.h"
#include "hal_config.h"


// DMA ADC-buffer
#define OVERSAMPLE 6
#define SELECT_FROM_DOUBLE_BUFFER !(DMA2_Stream4->CR & DMA_SxCR_CT)  


volatile uint16_t BSS adc_buffer[2][NUM_TEMP_SENSORS * OVERSAMPLE];

static ADC_HandleTypeDef hadc;
static DMA_HandleTypeDef hdma;
static ADC_ChannelConfTypeDef sConfig;
static GPIO_InitTypeDef igpio;
static uint32_t channel;
static uint32_t rank;

// Private functions
void init_analog(void);
void init_analog_pins(void);
void init_dma(void);

static void DMA_MultiBuff_Start_IT(void);
static void DMA_Base_Init();
void custom_HAL_DMA_Init();



/** Initialize the analog subsystem.

  Initialize the ADC and start hardware scan for all sensors.
*/
void analog_init() {

  if (NUM_TEMP_SENSORS) {                       // At least one channel in use.
    init_analog();
    init_dma();

  }

}

void init_analog() {

  ANALOG_ADC_CLK_ENABLE(ANALOG_ADC);

  #undef DEFINE_TEMP_SENSOR
  #define DEFINE_TEMP_SENSOR(name, type, pin, additional)   \
    GPIO_CONFIGURE_PIN(igpio, additional);                  \
    ADC_BASE_CONFIG( hadc, hdma, ANALOG_ADC);               \
    if (NUM_TEMP_SENSORS) {                                 \
     ADC_CHANNEL_CONFIG(hadc, sConfig, ANALOG_ADC );        \
    }   
        
  #include "config_wrapper.h"               
  #undef DEFINE_TEMP_SENSOR

   /*
    *  In order to use HAL with a double buffered DMA stream
    *  we need to do a bit of tweaking. Only HAL_ADC_Start_DMA
    *  sets the ADC_CR2_DMA bits. So we need to reset the DMA
    *  to operate in double buffered mode after starting ADC
    */
  HAL_ADC_Start_DMA(&hadc, (uint32_t*)adc_buffer, NUM_TEMP_SENSORS); 

}

/** Init the DMA for ADC
*/

void init_dma() {

  // Enable clck
  DMA_CLK_ENABLE(ANALOG_DMA);

  DMA_BASE_CONFIG(hdma, ANALOG_DMA);

  DMA_MultiBuff_Start_IT();

  HAL_NVIC_SetPriority(ANALOG_DMA_IRQN,3,1);
}



void DMA_MultiBuff_Start_IT(){

  __HAL_DMA_SET_COUNTER(&hdma, NUM_TEMP_SENSORS * OVERSAMPLE);

  DMA_SET_PERH_ADDR(&hdma,(uint32_t)&ANALOG_ADC_INSTANCE->DR );

  HAL_DMAEx_ChangeMemory(&hdma, (uint32_t)&adc_buffer[0] , MEMORY0);
  HAL_DMAEx_ChangeMemory(&hdma, (uint32_t)&adc_buffer[1] , MEMORY1);

  DMA_ENABLE_MULTIBUFFER(&hdma);

  __HAL_DMA_ENABLE_IT(&hdma, DMA_IT_TC);
  __HAL_DMA_ENABLE(&hdma);

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
void ANALOG_DMA_IRQHandler(void) {

  if(__HAL_DMA_GET_FLAG( &hdma, __HAL_DMA_GET_TC_FLAG_INDEX(&hdma)) == SET ){
    __HAL_DMA_CLEAR_FLAG(&hdma, RESET);
    // HAL_ADC_Stop(&hadc);
  }
  
}

#endif /* defined TEACUP_C_INCLUDE && defined __ARMEL__ */
