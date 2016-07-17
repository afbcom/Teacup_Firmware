#ifndef  TEACUP_HAL_CONFIG
#define TEACUP_HAL_CONFIG

#include "stm32f4xx.h"
#include "arduino.h"
#include "arduino_stm32f411.h"
#include "stm32f4xx_hal_conf.h"
#include "stm32f4xx_hal_def.h"

 /*************************************************************************
 *          BOARD CONFIGURATION PARAMETERS
 *************************************************************************/

#define STM32F401xE
#define WORD_LENGTH 32
#define HALFWORD_LENGTH 16
#define BYTE_LENGTH 8
	
/*************************************************************************
 *					STEPPER CONFIGURATION
 ***********************************************************************/
#define STEPPER_TIMER 		TIM5
#define STEPPER_TIMER_IRQN 	TIM5_IRQn
#define STEPPER_TIMER_NVIC  NVIC_PRIORITYGROUP_0

/*************************************************************************
 *          SERIAL CONFIGURATION PARAMETERS
 ***********************************************************************/

 
#define SERIAL_UART_INSTANCE        USART2
#define SERIAL_UART_BAUD            115200
#define SERIAL_UART_MODE            UART_MODE_TX_RX
#define SERIAL_UART_PARITY          UART_PARITY_NONE
#define SERIAL_UART_WORDLENGTH      8
#define SERIAL_UART_STOPBITS        1
#define SERIAL_UART_OVERSAMPLING    16
#define SERIAL_UART_HWCTRL          UART_HWCONTROL_NONE

#define SERIAL_TX_PIN         PA_2_PIN 
#define SERIAL_TX_PORT        PA_2_PORT
#define SERIAL_TX_MODE        GPIO_MODE_AF_PP
#define SERIAL_TX_PULL        GPIO_NOPULL
#define SERIAL_TX_SPEED       GPIO_SPEED_FREQ_VERY_HIGH

#define SERIAL_RX_PIN         PA_3_PIN
#define SERIAL_RX_PORT        PA_3_PORT
#define SERIAL_RX_MODE        GPIO_MODE_AF_PP
#define SERIAL_RX_PULL        GPIO_NOPULL
#define SERIAL_RX_SPEED       GPIO_SPEED_FREQ_VERY_HIGH

/*************************************************************************
*          ADC CONFIGURATION PARAMETERS
***********************************************************************/ 

#define ANALOG_ADC_INSTANCE             ADC1
#define ANALOG_ADC_PRESCALER            ADC_CLOCK_SYNC_PCLK_DIV8
#define ANALOG_ADC_SCANCONVMODE         ENABLE
#define ANALOG_ADC_EXTERNALTRIGGER      ADC_SOFTWARE_START
#define ANALOG_ADC_RESOLUTION           ADC_RESOLUTION_10B
#define ANALOG_ADC_DATAALIGN            ADC_DATAALIGN_RIGHT
#define ANALOG_ADC_CONTCONVMODE         ENABLE
#define ANALOG_ADC_DMACONTREQ           ENABLE
#define ANALOG_ADC_EOCSEL               ADC_EOC_SEQ_CONV

#define ANALOG_ADC_CHANNEL              PB_0_ADC
#define ANALOG_ADC_RANK                 1   
#define ANALOG_ADC_SAMPLINGTIME_CYCLES  56
#define ANALOG_ADC_OFFSET               0

#define THERMISTOR_EXTRUDER_PIN         PB_0_PIN
#define THERMISTOR_EXTRUDER_PORT        PB_0_PORT
#define THERMISTOR_EXTRUDER_MODE        GPIO_MODE_ANALOG
#define THERMISTOR_EXTRUDER_PULL        GPIO_NOPULL
#define THERMISTOR_EXTRUDER_SPEED       GPIO_SPEED_FREQ_HIGH

/*************************************************************************
*          DMA  CONFIGURATION PARAMETERS
***********************************************************************/ 
#define  ANALOG_DMA_NUMBER              2
#define  ANALOG_DMA_STREAM              4
#define  ANALOG_DMA_CHANNEL             0

#define ANALOG_DMA_DIRECTION            DMA_PERIPH_TO_MEMORY
#define ANALOG_DMA_PINC                 DMA_PINC_DISABLE
#define ANALOG_DMA_MINC                 DMA_MINC_ENABLE
#define ANALOG_DMA_PDATAALIGN           DMA_PDATAALIGN_HALFWORD
#define ANALOG_DMA_MEMDATAALIGN         DMA_MDATAALIGN_HALFWORD
#define ANALOG_DMA_MODE                 DMA_CIRCULAR
#define ANALOG_DMA_PRIORITY             DMA_PRIORITY_VERY_HIGH
#define ANALOG_DMA_FIFOTHRESHOLD        DMA_FIFO_THRESHOLD_1QUARTERFULL
#define ANALOG_MEMBURST                 DMA_MBURST_SINGLE
#define ANALOG_MPERIPHBURST             DMA_PBURST_SINGLE
#define ANALOG_DMA_BUFFER_TYPE          HALF_WORD


/*************************************************************************
*          ADC CONFIGURATION MACROS
***********************************************************************/ 

#define ADC_SET_CLOCKPRESCALER( adc_handle, name)                                                                      \
  if ( name ## _PRESCALER == ADC_CLOCK_SYNC_PCLK_DIV2) adc_handle.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;       \
  else if ( name ## _PRESCALER == ADC_CLOCK_SYNC_PCLK_DIV4) adc_handle.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;  \
  else if ( name ## _PRESCALER == ADC_CLOCK_SYNC_PCLK_DIV6) adc_handle.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV6;  \
  else adc_handle.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;                              

#define ADC_SET_SCANCONVMODE( adc_handle, name)                                  \
  if( name ## _SCANCONVMODE == DISABLE) adc_handle.Init.ScanConvMode = DISABLE;  \
  else adc_handle.Init.ScanConvMode = ENABLE;

#define ADC_SET_RESOLUTION( adc_handle, name)                                                             \
  if( name ## _RESOLUTION == ADC_RESOLUTION_6B ) adc_handle.Init.Resolution = ADC_RESOLUTION_6B;          \
  else if( name ## _RESOLUTION == ADC_RESOLUTION_8B ) adc_handle.Init.Resolution = ADC_RESOLUTION_8B;     \
  else if( name ## _RESOLUTION == ADC_RESOLUTION_10B ) adc_handle.Init.Resolution = ADC_RESOLUTION_10B;   \
  else  adc_handle.Init.Resolution = ADC_RESOLUTION_12B;  


#define ADC_SET_EXTERNALTRIGGER( adc_handle, name)                        \
  if ( name ## _EXTERNALTRIGGER == ADC_EXTERNALTRIGCONV_T1_CC1 ) {        \
    adc_handle.Init.ExternalTrigConv =  ADC_EXTERNALTRIGCONV_T1_CC1;}     \
  else if ( name ## _EXTERNALTRIGGER == ADC_EXTERNALTRIGCONV_T1_CC2 ){    \
     adc_handle.Init.ExternalTrigConv =  ADC_EXTERNALTRIGCONV_T1_CC2;}    \
  else if ( name ## _EXTERNALTRIGGER == ADC_EXTERNALTRIGCONV_T1_CC3 ){    \
     adc_handle.Init.ExternalTrigConv =  ADC_EXTERNALTRIGCONV_T1_CC3;}    \
  else if ( name ## _EXTERNALTRIGGER == ADC_EXTERNALTRIGCONV_T2_CC2 ){    \
     adc_handle.Init.ExternalTrigConv =  ADC_EXTERNALTRIGCONV_T2_CC2;}    \
  else if ( name ## _EXTERNALTRIGGER == ADC_EXTERNALTRIGCONV_T2_CC3 ){    \
     adc_handle.Init.ExternalTrigConv =  ADC_EXTERNALTRIGCONV_T2_CC3;}    \
  else if ( name ## _EXTERNALTRIGGER == ADC_EXTERNALTRIGCONV_T2_CC4 ){    \
     adc_handle.Init.ExternalTrigConv =  ADC_EXTERNALTRIGCONV_T2_CC4;}    \
  else if ( name ## _EXTERNALTRIGGER == ADC_EXTERNALTRIGCONV_T2_TRGO ){   \
     adc_handle.Init.ExternalTrigConv =  ADC_EXTERNALTRIGCONV_T2_TRGO;}   \
  else if ( name ## _EXTERNALTRIGGER == ADC_EXTERNALTRIGCONV_T3_CC1 ){    \
     adc_handle.Init.ExternalTrigConv =  ADC_EXTERNALTRIGCONV_T3_CC1;}    \
  else if ( name ## _EXTERNALTRIGGER == ADC_EXTERNALTRIGCONV_T3_TRGO ){   \
     adc_handle.Init.ExternalTrigConv =  ADC_EXTERNALTRIGCONV_T3_TRGO;}   \
  else if ( name ## _EXTERNALTRIGGER == ADC_EXTERNALTRIGCONV_T4_CC4 ){    \
     adc_handle.Init.ExternalTrigConv =  ADC_EXTERNALTRIGCONV_T4_CC4;}    \
  else if ( name ## _EXTERNALTRIGGER == ADC_EXTERNALTRIGCONV_T5_CC1 ){    \
     adc_handle.Init.ExternalTrigConv =  ADC_EXTERNALTRIGCONV_T5_CC1;}    \
  else if ( name ## _EXTERNALTRIGGER == ADC_EXTERNALTRIGCONV_T5_CC2 ){    \
     adc_handle.Init.ExternalTrigConv =  ADC_EXTERNALTRIGCONV_T5_CC2;}    \
  else if ( name ## _EXTERNALTRIGGER == ADC_EXTERNALTRIGCONV_T5_CC3 ){    \
     adc_handle.Init.ExternalTrigConv =  ADC_EXTERNALTRIGCONV_T5_CC3;}    \
  else if ( name ## _EXTERNALTRIGGER == ADC_EXTERNALTRIGCONV_T8_CC1 ){    \
     adc_handle.Init.ExternalTrigConv =  ADC_EXTERNALTRIGCONV_T8_CC1;}    \
  else if ( name ## _EXTERNALTRIGGER == ADC_EXTERNALTRIGCONV_T8_TRGO ){   \
     adc_handle.Init.ExternalTrigConv =  ADC_EXTERNALTRIGCONV_T8_TRGO;}   \
  else if ( name ## _EXTERNALTRIGGER == ADC_EXTERNALTRIGCONV_Ext_IT11 ){  \
     adc_handle.Init.ExternalTrigConv =  ADC_EXTERNALTRIGCONV_Ext_IT11;}  \
  else adc_handle.Init.ExternalTrigConv = ADC_SOFTWARE_START;

#define ADC_SET_DATAALIGN( adc_handle, name )                                                   \
  if( name ## _DATAALIGN == ADC_DATAALIGN_LEFT) adc_handle.Init.DataAlign = ADC_DATAALIGN_LEFT; \
  else adc_handle.Init.DataAlign = ADC_DATAALIGN_RIGHT;

#define ADC_SET_CONTCONVMODE( adc_handle, name )                                                  \
  if( name ## _CONTCONVMODE == DISABLE ) adc_handle.Init.ContinuousConvMode = DISABLE;  \
  else adc_handle.Init.ContinuousConvMode = ENABLE;  

#define ADC_SET_DMACONTREQ( adc_handle, name )                                          \
  if( name ## _DMACONTREQ == DISABLE) adc_handle.Init.DMAContinuousRequests = DISABLE;  \
  else adc_handle.Init.DMAContinuousRequests = ENABLE; 

#define ADC_SET_EOCSEL( adc_handle, name)                                                             \
  if( name ## _EOCSEL == ADC_EOC_SINGLE_CONV ) adc_handle.Init.EOCSelection = ADC_EOC_SINGLE_CONV;    \
  else if ( name ## _EOCSEL == ADC_EOC_SEQ_CONV ) adc_handle.Init.EOCSelection = ADC_EOC_SEQ_CONV;       \
  else adc_handle.Init.EOCSelection = ADC_EOC_SINGLE_SEQ_CONV;


#define ADC_BASE_CONFIG( adc_handle , dma_handle, name) \
  hadc.Instance = name ## _INSTANCE;                    \
  ADC_SET_CLOCKPRESCALER(adc_handle, name);             \
  ADC_SET_SCANCONVMODE( adc_handle, name);              \
  ADC_SET_RESOLUTION( adc_handle, name);                \
  ADC_SET_EXTERNALTRIGGER( adc_handle, name);           \
  ADC_SET_DATAALIGN( adc_handle, name );                \
  ADC_SET_CONTCONVMODE( adc_handle, name );             \
  ADC_SET_DMACONTREQ( adc_handle, name );               \
  ADC_SET_EOCSEL( adc_handle, name);                    \
  adc_handle.Init.NbrOfConversion = NUM_TEMP_SENSORS;   \
  adc_handle.DMA_Handle = &dma_handle;                  \
  HAL_ADC_Init( &adc_handle ); 


#define ADC_SET_CHANNEL( channel_config_handle, name)                                  \
  if (name ## _CHANNEL < 0 )  channel_config_handle.Channel = ADC_CHANNEL_0;           \
  else if (name ## _CHANNEL  == 0)  channel_config_handle.Channel = ADC_CHANNEL_0;     \
  else if (name ## _CHANNEL  == 1)  channel_config_handle.Channel = ADC_CHANNEL_1;     \
  else if (name ## _CHANNEL  == 2)  channel_config_handle.Channel = ADC_CHANNEL_2;     \
  else if (name ## _CHANNEL  == 3)  channel_config_handle.Channel = ADC_CHANNEL_3;     \
  else if (name ## _CHANNEL  == 4)  channel_config_handle.Channel = ADC_CHANNEL_4;     \
  else if (name ## _CHANNEL  == 5)  channel_config_handle.Channel = ADC_CHANNEL_5;     \
  else if (name ## _CHANNEL  == 6)  channel_config_handle.Channel = ADC_CHANNEL_6;     \
  else if (name ## _CHANNEL  == 7)  channel_config_handle.Channel = ADC_CHANNEL_7;     \
  else if (name ## _CHANNEL  == 8)  channel_config_handle.Channel = ADC_CHANNEL_8;     \
  else if (name ## _CHANNEL  == 9)  channel_config_handle.Channel = ADC_CHANNEL_9;     \
  else if (name ## _CHANNEL  == 10)  channel_config_handle.Channel = ADC_CHANNEL_10;   \
  else if (name ## _CHANNEL  == 11)  channel_config_handle.Channel = ADC_CHANNEL_11;   \
  else if (name ## _CHANNEL  == 12)  channel_config_handle.Channel = ADC_CHANNEL_12;   \
  else if (name ## _CHANNEL  == 13)  channel_config_handle.Channel = ADC_CHANNEL_13;   \
  else if (name ## _CHANNEL  == 14)  channel_config_handle.Channel = ADC_CHANNEL_14;   \
  else if (name ## _CHANNEL  == 15)  channel_config_handle.Channel = ADC_CHANNEL_15;   \
  else if (name ## _CHANNEL  == 16)  channel_config_handle.Channel = ADC_CHANNEL_16;   \
  else if (name ## _CHANNEL  == 17)  channel_config_handle.Channel = ADC_CHANNEL_17;   \
  else if (name ## _CHANNEL  == 18)  channel_config_handle.Channel = ADC_CHANNEL_18;   \
  else if (name ## _CHANNEL > 18)  channel_config_handle.Channel = 18;                 \
  else  channel_config_handle.Rank = name ## _RANK;

#define ADC_SET_RANK( channel_config_handle, name)                  \
  if( name ## _RANK < 1 ) channel_config_handle.Rank = 1;           \
  else if (name ## _RANK > 16)  channel_config_handle.Rank = 16;    \
  else  channel_config_handle.Rank = name ## _RANK;

#define ADC_SET_SAMPLINGTIME_CYCLES( channel_config_handle, name)                                                \
  if( name ## _SAMPLINGTIME_CYCLES < 4 ) channel_config_handle.SamplingTime = ADC_SAMPLETIME_3CYCLES;            \
  else if( name ## _SAMPLINGTIME_CYCLES < 16) channel_config_handle.SamplingTime = ADC_SAMPLETIME_15CYCLES;      \
  else if( name ## _SAMPLINGTIME_CYCLES < 29 ) channel_config_handle.SamplingTime = ADC_SAMPLETIME_28CYCLES;     \
  else if( name ## _SAMPLINGTIME_CYCLES < 57 ) channel_config_handle.SamplingTime = ADC_SAMPLETIME_56CYCLES;     \
  else if( name ## _SAMPLINGTIME_CYCLES < 85 ) channel_config_handle.SamplingTime = ADC_SAMPLETIME_84CYCLES;     \
  else if( name ## _SAMPLINGTIME_CYCLES < 113 ) channel_config_handle.SamplingTime = ADC_SAMPLETIME_112CYCLES;   \
  else if( name ## _SAMPLINGTIME_CYCLES < 145 ) channel_config_handle.SamplingTime = ADC_SAMPLETIME_144CYCLES;   \
  else if( name ## _SAMPLINGTIME_CYCLES < 481  ) channel_config_handle.SamplingTime = ADC_SAMPLETIME_480CYCLES;  \
  else  channel_config_handle.SamplingTime =  ADC_SAMPLETIME_480CYCLES;

#define ADC_CHANNEL_CONFIG( adc_handle, channel_config_handle, name) \
  ADC_SET_CHANNEL( channel_config_handle, name);                     \
  ADC_SET_RANK( channel_config_handle, name);                        \
  ADC_SET_SAMPLINGTIME_CYCLES( channel_config_handle, name);         \
  HAL_ADC_ConfigChannel(&adc_handle,&channel_config_handle);

#define ANALOG_ADC_CLK_ENABLE( name )                               \
  if( name ## _INSTANCE == ADC1 ) __HAL_RCC_ADC1_CLK_ENABLE();      
  // else if( name ## _INSTANCE == ADC2 ) __HAL_RCC_ADC2_CLK_ENABLE(); \
  // else if( name ## _INSTANCE == ADC3 ) __HAL_RCC_ADC3_CLK_ENABLE(); 

#define ANALOG_ADC_CLK_DISABLE( name )                               \
  if( name ## _INSTANCE == ADC1 ) __HAL_RCC_ADC1_CLK_DISABLE();      
  // else if( name ## _INSTANCE == ADC2 ) __HAL_RCC_ADC2_CLK_DISABLE(); \
  // else if( name ## _INSTANCE == ADC3 ) __HAL_RCC_ADC3_CLK_DISABLE(); 

  /*************************************************************************
 *          DMA CONFIGURATION MACROS
 ***********************************************************************/ 

#if  ( ANALOG_DMA_NUMBER == 1   &&   ANALOG_DMA_STREAM == 0 )
  #define ANALOG_DMA_INSTANCE     DMA1_Stream0
  #define ANALOG_DMA_IRQN         DMA1_Stream0_IRQn 
  #define ANALOG_DMA_IRQHandler   DMA1_Stream0_IRQHandle
#elif  ( ANALOG_DMA_NUMBER == 1  &&   ANALOG_DMA_STREAM == 1 )
  #define ANALOG_DMA_INSTANCE     DMA1_Stream1
  #define ANALOG_DMA_IRQN         DMA1_Stream1_IRQn 
  #define ANALOG_DMA_IRQHandler   DMA1_Stream1_IRQHandle
#elif  ( ANALOG_DMA_NUMBER == 1  &&   ANALOG_DMA_STREAM == 2 )
  #define ANALOG_DMA_INSTANCE     DMA1_Stream2
  #define ANALOG_DMA_IRQN         DMA1_Stream2_IRQn 
  #define ANALOG_DMA_IRQHandler   DMA1_Stream2_IRQHandle
#elif  ( ANALOG_DMA_NUMBER == 1  &&   ANALOG_DMA_STREAM == 3 )
  #define ANALOG_DMA_INSTANCE     DMA1_Stream3
  #define ANALOG_DMA_IRQN         DMA1_Stream3_IRQn 
  #define ANALOG_DMA_IRQHandler   DMA1_Stream3_IRQHandle
#elif  ( ANALOG_DMA_NUMBER == 1  &&   ANALOG_DMA_STREAM == 4 )
  #define ANALOG_DMA_INSTANCE     DMA1_Stream4
  #define ANALOG_DMA_IRQN         DMA1_Stream4_IRQn 
  #define ANALOG_DMA_IRQHandler   DMA1_Stream4_IRQHandle
#elif  ( ANALOG_DMA_NUMBER == 1  &&   ANALOG_DMA_STREAM == 5 )
  #define ANALOG_DMA_INSTANCE     DMA1_Stream5
  #define ANALOG_DMA_IRQN         DMA1_Stream5_IRQn 
  #define ANALOG_DMA_IRQHandler   DMA1_Stream5_IRQHandle
#elif  ( ANALOG_DMA_NUMBER == 1  &&   ANALOG_DMA_STREAM == 6 )
  #define ANALOG_DMA_INSTANCE     DMA1_Stream6
  #define ANALOG_DMA_IRQN         DMA1_Stream6_IRQn 
  #define ANALOG_DMA_IRQHandler   DMA1_Stream6_IRQHandle
#elif  ( ANALOG_DMA_NUMBER == 2  &&   ANALOG_DMA_STREAM == 0 )
  #define ANALOG_DMA_INSTANCE     DMA2_Stream0
  #define ANALOG_DMA_IRQN         DMA2_Stream0_IRQn 
  #define ANALOG_DMA_IRQHandler   DMA2_Stream0_IRQHandle
#elif  ( ANALOG_DMA_NUMBER == 2  &&   ANALOG_DMA_STREAM == 1 )
  #define ANALOG_DMA_INSTANCE     DMA2_Stream1
  #define ANALOG_DMA_IRQN         DMA2_Stream1_IRQn 
  #define ANALOG_DMA_IRQHandler   DMA2_Stream1_IRQHandle
#elif  ( ANALOG_DMA_NUMBER == 2  &&   ANALOG_DMA_STREAM == 2 )
  #define ANALOG_DMA_INSTANCE     DMA2_Stream2
  #define ANALOG_DMA_IRQN         DMA2_Stream2_IRQn 
  #define ANALOG_DMA_IRQHandler   DMA2_Stream2_IRQHandle
#elif  ( ANALOG_DMA_NUMBER == 2  &&   ANALOG_DMA_STREAM == 3 )
  #define ANALOG_DMA_INSTANCE     DMA2_Stream3
  #define ANALOG_DMA_IRQN         DMA2_Stream3_IRQn 
  #define ANALOG_DMA_IRQHandler   DMA2_Stream3_IRQHandle
#elif  ( ANALOG_DMA_NUMBER == 2  &&   ANALOG_DMA_STREAM == 4 )
  #define ANALOG_DMA_INSTANCE     DMA2_Stream4
  #define ANALOG_DMA_IRQN         DMA2_Stream4_IRQn 
  #define ANALOG_DMA_IRQHandler   DMA2_Stream4_IRQHandle
#endif



 #define DMA_SET_DIRECTION( dma_handle, name )                                                            \
  if( name ## _DIRECTION == DMA_MEMORY_TO_MEMORY)  dma_handle.Init.Direction = DMA_MEMORY_TO_MEMORY;      \
  else if (name ## _DIRECTION  == DMA_MEMORY_TO_PERIPH) dma_handle.Init.Direction = DMA_MEMORY_TO_PERIPH; \
  else dma_handle.Init.Direction = DMA_PERIPH_TO_MEMORY;

#define DMA_SET_PINC( dma_handle, name )                                            \
  if( name ## _PINC == DMA_PINC_ENABLE) dma_handle.Init.PeriphInc= DMA_PINC_ENABLE; \
  else dma_handle.Init.PeriphInc= DMA_PINC_DISABLE;

#define DMA_SET_MINC( dma_handle, name )                                                \
  if( name ## _MINC == DMA_MINC_DISABLE) dma_handle.Init.PeriphInc= DMA_MINC_DISABLE; \
  else dma_handle.Init.PeriphInc= DMA_MINC_ENABLE;

#define DMA_SET_PDATAALIGN( dma_handle, name)                                                  \
  if( name ## _PDATAALIGN  ==  BYTE_LENGTH )  dma_handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;       \
  else if  ( name ## _PDATAALIGN == WORD_LENGTH )  dma_handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD; \
  else dma_handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;


#define DMA_SET_MDATAALIGN( dma_handle, name)                                                                           \
  if( name ## _BUFFER_TYPE ==  BYTE_LENGTH )  dma_handle.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;       \
  else if ( name ## _BUFFER_TYPE == WORD_LENGTH ) dma_handle.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;\
  else dma_handle.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;


#define DMA_SET_MDATAALIGN( dma_handle, name)                   \
   dma_handle.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;

#define DMA_SET_MODE( dma_handle, name)                                      \
  if( name ## _MODE ==  DMA_NORMAL)  dma_handle.Init.Mode = DMA_NORMAL;      \
  else if ( name ## _MODE ==  DMA_PFCTRL) dma_handle.Init.Mode = DMA_PFCTRL; \
  else dma_handle.Init.Mode = DMA_CIRCULAR;

#define DMA_SET_PRIORITY( dma_handle, name)                                                        \
  if( name ## _PRIORITY ==  DMA_PRIORITY_LOW)  dma_handle.Init.Priority = DMA_PRIORITY_LOW;        \
  else if ( name ## _PRIORITY ==  DMA_PRIORITY_HIGH) dma_handle.Init.Priority = DMA_PRIORITY_HIGH; \
  else dma_handle.Init.Priority = DMA_PRIORITY_VERY_HIGH;

#define DMA_SET_FIFOTHRESHOLD( dma_handle, name)                                                                                   \
  if( name ## _FIFOTHRESHOLD == DMA_FIFO_THRESHOLD_FULL) dma_handle.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;                        \
  else if( name ## _FIFOTHRESHOLD == DMA_FIFO_THRESHOLD_3QUARTERSFULL) dma_handle.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_3QUARTERSFULL; \
  else if( name ## _FIFOTHRESHOLD == DMA_FIFO_THRESHOLD_HALFFULL) dma_handle.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;           \
  else  dma_handle.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_1QUARTERFULL;

#define DMA_SET_CHANNEL( dma_handle, name)                                    \
  if( name ## _CHANNEL < 0 ) dma_handle.Init.Channel = DMA_CHANNEL_0;         \
  else if( name ## _CHANNEL == 0 ) dma_handle.Init.Channel = DMA_CHANNEL_0;   \
  else if( name ## _CHANNEL == 1 ) dma_handle.Init.Channel = DMA_CHANNEL_1;   \
  else if( name ## _CHANNEL == 2 ) dma_handle.Init.Channel = DMA_CHANNEL_2;   \
  else if( name ## _CHANNEL == 3 ) dma_handle.Init.Channel = DMA_CHANNEL_3;   \
  else if( name ## _CHANNEL == 4 ) dma_handle.Init.Channel = DMA_CHANNEL_4;   \
  else if( name ## _CHANNEL == 5 ) dma_handle.Init.Channel = DMA_CHANNEL_5;   \
  else if( name ## _CHANNEL == 6 ) dma_handle.Init.Channel = DMA_CHANNEL_6;   \
  else if( name ## _CHANNEL == 7 ) dma_handle.Init.Channel = DMA_CHANNEL_7;   \
  else if( name ## _CHANNEL > 7 ) dma_handle.Init.Channel = DMA_CHANNEL_7; 

#define DMA_SET_PERH_ADDR(__HANDLE__, __ADDRESS_) ((__HANDLE__)->Instance->PAR = __ADDRESS_ )
#define DMA_ENABLE_MULTIBUFFER(__HANDLE__)((__HANDLE__)->Instance->CR |= DMA_SxCR_DBM )

#define DMA_BASE_CONFIG(dma_handle, name )     \
  dma_handle.Instance = name ## _INSTANCE;     \
  DMA_SET_DIRECTION(dma_handle, name );        \
  DMA_SET_PINC(dma_handle, name );             \
  DMA_SET_MINC(dma_handle, name );             \
  DMA_SET_PDATAALIGN(dma_handle, name );       \
  DMA_SET_MDATAALIGN(dma_handle, name );       \
  DMA_SET_MODE(dma_handle, name );             \
  DMA_SET_PRIORITY(dma_handle, name );         \
  DMA_SET_FIFOTHRESHOLD(dma_handle, name );    \
  DMA_SET_CHANNEL( dma_handle, name);          \
  HAL_DMA_Init(&dma_handle);

#define DMA_CLK_ENABLE(name)                                     \
  if( name ## _NUMBER == 1) __HAL_RCC_DMA1_CLK_ENABLE();         \
  else if (name ## _NUMBER == 2)  __HAL_RCC_DMA2_CLK_ENABLE();

/*************************************************************************
 *          SERIAL CONFIGURATION MACROS
 ***********************************************************************/

#define UART_SET_INSTANCE( UART_HANDLE, INSTANCE  )                                             \
  if ( INSTANCE == USART1 ) UART_HANDLE.Instance = USART1;                                      \
  else UART_HANDLE.Instance = USART2;

#define UART_SET_BAUD( UART_HANDLE, NUMBER )                                                    \
  UART_HANDLE.Init.BaudRate = (uint32_t)(NUMBER); 

#define UART_SET_STOPBITS( UART_HANDLE, NUMBER )                                                \
  if (NUMBER == 2) UART_HANDLE.Init.StopBits = UART_STOPBITS_2;                                 \
  else UART_HANDLE.Init.StopBits = UART_STOPBITS_1;           

#define UART_SET_WORDLENGTH( UART_HANDLE, NUMBER)                                               \
  if (NUMBER == 9) huart.Init.WordLength = UART_WORDLENGTH_9B;                                  \
  else huart.Init.WordLength = UART_WORDLENGTH_8B;        

#define UART_SET_PARITY( UART_HANDLE, MODE )                                                    \
  if( MODE == UART_PARITY_EVEN )  UART_HANDLE.Init.Parity = UART_PARITY_EVEN;                   \
  else if(MODE == UART_PARITY_ODD) UART_HANDLE.Init.Parity = UART_PARITY_ODD;                   \
  else UART_HANDLE.Init.Parity = UART_PARITY_NONE;            

#define UART_SET_MODE( UART_HANDLE, MODE)                                                       \
  if( MODE == UART_MODE_TX ) UART_HANDLE.Init.Mode = UART_MODE_TX;                              \
  else if(MODE == UART_MODE_RX) UART_HANDLE.Init.Mode = UART_MODE_RX;                           \
  else UART_HANDLE.Init.Mode = UART_MODE_TX_RX;

#define UART_SET_OVERSAMPLING( UART_HANDLE, NUMBER)                                             \
  if( NUMBER == 8 ) UART_HANDLE.Init.OverSampling = UART_OVERSAMPLING_8;                        \
  else UART_HANDLE.Init.OverSampling = UART_OVERSAMPLING_16;

#define UART_SET_HWCTRL( UART_HANDLE, MODE)                                                      \
  if( MODE == UART_HWCONTROL_RTS ) UART_HANDLE.Init.HwFlowCtl = UART_HWCONTROL_RTS;              \
  else if( MODE == UART_HWCONTROL_CTS ) UART_HANDLE.Init.HwFlowCtl = UART_HWCONTROL_CTS;         \
  else if( MODE == UART_HWCONTROL_RTS_CTS ) UART_HANDLE.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS; \
  else UART_HANDLE.Init.HwFlowCtl = UART_HWCONTROL_NONE;

#define UART_CONFIGURE( UART_HANDLE, NAME  )                                               \
    UART_SET_INSTANCE( UART_HANDLE, NAME ## _INSTANCE );                                   \
    UART_SET_BAUD( UART_HANDLE, NAME ## _BAUD );                                           \
    UART_SET_STOPBITS( UART_HANDLE, NAME ## _STOPBITS);                                    \
    UART_SET_WORDLENGTH( UART_HANDLE, NAME ## _WORDLENGTH);                                \
    UART_SET_PARITY( UART_HANDLE, NAME ## _PARITY);                                        \
    UART_SET_MODE( UART_HANDLE, NAME ## _MODE);                                            \
    UART_SET_OVERSAMPLING( UART_HANDLE, NAME ## _OVERSAMPLING);                            \
    UART_SET_HWCTRL( UART_HANDLE, NAME ## _HWCTRL);                                        \
    HAL_UART_Init(&UART_HANDLE);
  
#define UART_CLK_ENABLE( name )                                                  \
  if( name ## _INSTANCE  == USART1 )  __HAL_RCC_USART1_CLK_ENABLE();             \
  else if ( name ## _INSTANCE  == USART2 ) __HAL_RCC_USART2_CLK_ENABLE();        
  // else if ( name ## _INSTANCE  == USART3 ) __HAL_RCC_USART3_CLK_ENABLE();        \
  // else if ( name ## _INSTANCE  == UART4 ) __HAL_RCC_UART4_CLK_ENABLE();          \
  // else if ( name ## _INSTANCE  == UART5 ) __HAL_RCC_UART5_CLK_ENABLE();          \
  // else if ( name ## _INSTANCE  == USART6 ) __HAL_RCC_USART6_CLK_ENABLE();        \
  // else if ( name ## _INSTANCE  == UART7 ) __HAL_RCC_UART7_CLK_ENABLE();          \
  // else if ( name ## _INSTANCE  == UART8 ) __HAL_RCC_UART8_CLK_ENABLE();          

#define UART_CLK_DISABLE( name )                                                  \
  if( name ## _INSTANCE  == USART1 )  __HAL_RCC_USART1_CLK_DISABLE();             \
  else if ( name ## _INSTANCE  == USART2 ) __HAL_RCC_USART2_CLK_DISABLE();        
  // else if ( name ## _INSTANCE  == USART3 ) __HAL_RCC_USART3_CLK_DISABLE();        \
  // else if ( name ## _INSTANCE  == UART4 ) __HAL_RCC_UART4_CLK_DISABLE();          \
  // else if ( name ## _INSTANCE  == UART5 ) __HAL_RCC_UART5_CLK_DISABLE();          \
  // else if ( name ## _INSTANCE  == USART6 ) __HAL_RCC_USART6_CLK_DISABLE();        \
  // else if ( name ## _INSTANCE  == UART7 ) __HAL_RCC_UART7_CLK_DISABLE();          \
  // else if ( name ## _INSTANCE  == UART8 ) __HAL_RCC_UART8_CLK_DISABLE();  




/*************************************************************************
 *          GPIO CONFIGURATION
 ***********************************************************************/
 /* IF the  ALternate Function is needed the GPIO_SET_PERIPH 
  * macro must be called before this function
  */
#define GPIO_CONFIGURE_PIN( gpio_handle, name )                                                \
  GPIO_SET_PIN( gpio_handle, name );                                                           \
  GPIO_SET_MODE(gpio_handle, name);                                                            \
  GPIO_SET_SPEED(gpio_handle, name);                                                           \
  GPIO_SET_PULL(gpio_handle, name);                                                            \
  HAL_GPIO_Init( name ## _PORT, &gpio_handle);

#define GPIO_SET_PIN( gpio_handle, name )  \
   gpio_handle.Pin = 1 << name ## _PIN;                                  

#define GPIO_SET_MODE(gpio_handle, name)                                                       \
  if (name ## _MODE == GPIO_MODE_OUTPUT_PP) gpio_handle.Mode = GPIO_MODE_OUTPUT_PP;            \
  else if (name ## _MODE == GPIO_MODE_OUTPUT_OD) gpio_handle.Mode = GPIO_MODE_OUTPUT_OD;       \
  else if (name ## _MODE == GPIO_MODE_AF_PP) gpio_handle.Mode = GPIO_MODE_AF_PP;               \
  else if (name ## _MODE == GPIO_MODE_AF_OD) gpio_handle.Mode = GPIO_MODE_AF_OD;               \
  else if (name ## _MODE == GPIO_MODE_ANALOG) gpio_handle.Mode = GPIO_MODE_ANALOG;             \
  else if (name ## _MODE == GPIO_MODE_IT_RISING) gpio_handle.Mode = GPIO_MODE_IT_RISING;       \
  else if (name ## _MODE == GPIO_MODE_IT_FALLING) gpio_handle.Mode = GPIO_MODE_IT_FALLING;     \
  else if (name ## _MODE == GPIO_MODE_IT_RISING_FALLING) {                                     \
      gpio_handle.Mode = GPIO_MODE_IT_RISING_FALLING;}                                         \
  else if (name ## _MODE == GPIO_MODE_EVT_RISING) gpio_handle.Mode = GPIO_MODE_EVT_RISING;     \
  else if (name ## _MODE == GPIO_MODE_EVT_FALLING) gpio_handle.Mode = GPIO_MODE_EVT_FALLING;   \
  else if (name ## _MODE == GPIO_MODE_EVT_RISING_FALLING){                                     \
      gpio_handle.Mode = GPIO_MODE_EVT_RISING_FALLING;}                                        \
  else gpio_handle.Mode = GPIO_MODE_INPUT;

#define GPIO_SET_SPEED(gpio_handle, name)                                                         \
  if (name ## _SPEED == GPIO_SPEED_FREQ_LOW) gpio_handle.Speed = GPIO_SPEED_FREQ_LOW;             \
  else if (name ## _SPEED == GPIO_SPEED_FREQ_MEDIUM) gpio_handle.Speed = GPIO_SPEED_FREQ_MEDIUM;  \
  else if (name ## _SPEED == GPIO_SPEED_FREQ_HIGH) gpio_handle.Speed = GPIO_SPEED_FREQ_HIGH;      \
  else gpio_handle.Speed = GPIO_SPEED_FREQ_HIGH;

#define GPIO_SET_PULL(gpio_handle, name)\
  if ( name ## _PULL == GPIO_PULLUP ) gpio_handle.Pull = GPIO_PULLUP;             \
  else if ( name ## _PULL == GPIO_PULLDOWN ) gpio_handle.Pull = GPIO_PULLDOWN;    \
  else gpio_handle.Pull = GPIO_NOPULL; 


#define GPIO_CLK_ENABLE( name )                                                   \
  if ( name ## _PORT == GPIOA)  __HAL_RCC_GPIOA_CLK_ENABLE();                     \
  else if ( name ## _PORT == GPIOB) __HAL_RCC_GPIOB_CLK_ENABLE();                 \
  else if ( name ## _PORT == GPIOC) __HAL_RCC_GPIOC_CLK_ENABLE();                 \
  else if ( name ## _PORT == GPIOD) __HAL_RCC_GPIOD_CLK_ENABLE();                 \
  else if ( name ## _PORT == GPIOH) __HAL_RCC_GPIOH_CLK_ENABLE();                    
  // else if ( name ## _PORT == GPIOE) __HAL_RCC_GPIOE_CLK_ENABLE();                 \
  // else if ( name ## _PORT == GPIOF) __HAL_RCC_GPIOF_CLK_ENABLE();                 \
  // else if ( name ## _PORT == GPIOG) __HAL_RCC_GPIOG_CLK_ENABLE();                 \

#define GPIO_CLK_DISBLE( name )                                                   \
  if ( name ## _PORT == GPIOA) __HAL_RCC_GPIOA_CLK_DISABLE();                     \
  else if ( name ## _PORT == GPIOB) __HAL_RCC_GPIOB_CLK_DISABLE();                \
  else if ( name ## _PORT == GPIOC) __HAL_RCC_GPIOC_CLK_DISABLE();                \
  else if ( name ## _PORT == GPIOD) __HAL_RCC_GPIOD_CLK_DISABLE();                \
  else if ( name ## _PORT == GPIOH) __HAL_RCC_GPIOH_CLK_DISABLE();       

  // else if ( name ## _PORT == GPIOE) __HAL_RCC_GPIOE_CLK_DISABLE();                \
  // else if ( name ## _PORT == GPIOF) __HAL_RCC_GPIOF_CLK_DISABLE();                \
  // else if ( name ## _PORT == GPIOG) __HAL_RCC_GPIOf_CLK_DISABLE();                \              

/*--------------------------------------------------------------------------------------------------*/

/*------------------------------GPIO ALTERNATE FUNCTION-----------------------------------------*/

// #if defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F427xx) || defined(STM32F437xx) || \
//     defined(STM32F407xx) || defined(STM32F417xx) || defined(STM32F405xx) || defined(STM32F415xx) || \
//     defined(STM32F446xx) || defined(STM32F469xx) || defined(STM32F479xx) 
//   #define GPIO_SET_TIMER_ALT( gpio_handle,  name )                                        \
//     if( name ## _TIMER_INSTANCE == TIM1 ) gpio_handle.Alternate = GPIO_AF1_TIM1;          \
//     else if( name ## _TIMER_INSTANCE == TIM2 ) gpio_handle.Alternate = GPIO_AF1_TIM2;     \
//     else if( name ## _TIMER_INSTANCE == TIM3 ) gpio_handle.Alternate = GPIO_AF2_TIM3;     \
//     else if( name ## _TIMER_INSTANCE == TIM4 ) gpio_handle.Alternate = GPIO_AF2_TIM4;     \
//     else if( name ## _TIMER_INSTANCE == TIM5 ) gpio_handle.Alternate = GPIO_AF2_TIM5;     \
//     else if( name ## _TIMER_INSTANCE == TIM8 ) gpio_handle.Alternate = GPIO_AF3_TIM8;     \
//     else if( name ## _TIMER_INSTANCE == TIM9 ) gpio_handle.Alternate = GPIO_AF3_TIM9;     \
//     else if( name ## _TIMER_INSTANCE == TIM10 ) gpio_handle.Alternate = GPIO_AF3_TIM10;   \
//     else if( name ## _TIMER_INSTANCE == TIM11 ) gpio_handle.Alternate = GPIO_AF3_TIM11;   \
//     else if( name ## _TIMER_INSTANCE == TIM12 ) gpio_handle.Alternate = GPIO_AF9_TIM12;   \
//     else if( name ## _TIMER_INSTANCE == TIM13 ) gpio_handle.Alternate = GPIO_AF9_TIM13;   \
//     else if( name ## _TIMER_INSTANCE == TIM14 ) gpio_handle.Alternate = GPIO_AF9_TIM14;   \
// #endif 


#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F411xE) 
  #define GPIO_SET_TIMER_ALT( gpio_handle,  name )                                        \
    if( name ## _TIMER_INSTANCE == TIM1 ) gpio_handle.Alternate = GPIO_AF1_TIM1;          \
    else if( name ## _TIMER_INSTANCE == TIM2 ) gpio_handle.Alternate = GPIO_AF1_TIM2;     \
    else if( name ## _TIMER_INSTANCE == TIM3 ) gpio_handle.Alternate = GPIO_AF2_TIM3;     \
    else if( name ## _TIMER_INSTANCE == TIM4 ) gpio_handle.Alternate = GPIO_AF2_TIM4;     \
    else if( name ## _TIMER_INSTANCE == TIM5 ) gpio_handle.Alternate = GPIO_AF2_TIM5;     \
    else if( name ## _TIMER_INSTANCE == TIM9 ) gpio_handle.Alternate = GPIO_AF3_TIM9;     \
    else if( name ## _TIMER_INSTANCE == TIM10 ) gpio_handle.Alternate = GPIO_AF3_TIM10;   \
    else if( name ## _TIMER_INSTANCE == TIM11 ) gpio_handle.Alternate = GPIO_AF3_TIM11;   
#endif 

#if defined(STM32F410Tx) || defined(STM32F410Cx) || defined(STM32F410Rx)
  #define GPIO_SET_TIMER_ALT( gpio_handle,  name )                                        \
    if( name ## _TIMER_INSTANCE == TIM1 ) gpio_handle.Alternate = GPIO_AF1_TIM1;          \
    else if( name ## _TIMER_INSTANCE == TIM5 ) gpio_handle.Alternate = GPIO_AF2_TIM5;     \
    else if( name ## _TIMER_INSTANCE == TIM9 ) gpio_handle.Alternate = GPIO_AF3_TIM9;     \
    else if( name ## _TIMER_INSTANCE == TIM11 ) gpio_handle.Alternate = GPIO_AF3_TIM11;  
#endif 

/*-------------------------GPIO UART ALTERNATE FUNCTION-----------------------------------------*/
#if defined(STM32F429xx) || defined(STM32F439xx) || defined(STM32F427xx) || defined(STM32F437xx) || \
    defined(STM32F469xx) || defined(STM32F479xx) || defined(STM32F469xx) || defined(STM32F479xx)
  #define GPIO_SET_UART_ALT(gpio_handle,  name )                                             \
    if( name ## _UART_INSTANCE == USART1 ) gpio_handle.Alternate = GPIO_AF7_USART1;          \
    else if( name ## _INSTANCE == USART2 ) gpio_handle.Alternate = GPIO_AF7_USART2;     \
    else if( name ## _INSTANCE == USART3 ) gpio_handle.Alternate = GPIO_AF7_USART3;     \
    else if( name ## _INSTANCE == UART4 ) gpio_handle.Alternate = GPIO_AF8_UART4;       \
    else if( name ## _INSTANCE == UART5 ) gpio_handle.Alternate = GPIO_AF8_UART5;       \
    else if( name ## _INSTANCE == USART6 ) gpio_handle.Alternate = GPIO_AF8_USART6;     \
    else if( name ## _INSTANCE == UART7 ) gpio_handle.Alternate = GPIO_AF8_UART7;       \
    else if( name ## _INSTANCE == UART8 ) gpio_handle.Alternate = GPIO_AF8_UART8;       
#endif 

#if defined(STM32F407xx) || defined(STM32F417xx) || defined(STM32F405xx) || defined(STM32F415xx) || defined(STM32F446xx)
  #define GPIO_SET_UART_ALT( gpio_handle,  name )                                            \
    if( name ## _INSTANCE == USART1 ) gpio_handle.Alternate = GPIO_AF7_USART1;          \
    else if( name ## _INSTANCE == USART2 ) gpio_handle.Alternate = GPIO_AF7_USART2;     \
    else if( name ## _INSTANCE == USART3 ) gpio_handle.Alternate = GPIO_AF7_USART3;     \
    else if( name ## _INSTANCE == UART4 ) gpio_handle.Alternate = GPIO_AF8_UART4;       \
    else if( name ## _INSTANCE == UART5 ) gpio_handle.Alternate = GPIO_AF8_UART5;       \
    else if( name ## _INSTANCE == USART6 ) gpio_handle.Alternate = GPIO_AF8_USART6;       
#endif 


#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F411xE) || defined(STM32F410Tx) || defined(STM32F410Cx) || defined(STM32F410Rx)
  #define GPIO_SET_UART_ALT( gpio_handle, name )                                             \
    if( name ## _INSTANCE == USART1 ) gpio_handle.Alternate = GPIO_AF7_USART1;          \
    else if( name ## _INSTANCE == USART2 ) gpio_handle.Alternate = GPIO_AF7_USART2;     \
    else if( name ## _INSTANCE == USART6 ) gpio_handle.Alternate = GPIO_AF8_USART6;       
#endif


/*************************************************************************
 *          TIMER CONFIGURATION
 ***********************************************************************/

#define TIMER_CLK_ENABLE(TIMER)                     \
      if (TIMER == TIM1) {                          \
        __HAL_RCC_TIM1_CLK_ENABLE();}               \
      else if (TIMER == TIM2){                      \
       __HAL_RCC_TIM2_CLK_ENABLE();}                \
      else if (TIMER == TIM3) {                     \
       __HAL_RCC_TIM3_CLK_ENABLE();}                \
      else if (TIMER == TIM4) {                     \
       __HAL_RCC_TIM4_CLK_ENABLE();}                \
      else if (TIMER == TIM5) {                     \
       __HAL_RCC_TIM5_CLK_ENABLE();}      


#define TIMER_CLK_DISABLE(TIMER)          \
      if (TIMER == TIM1) {                          \
        __HAL_RCC_TIM1_CLK_DISABLE();}          \
      else if (TIMER == TIM2){                      \
       __HAL_RCC_TIM2_CLK_DISABLE();}             \
      else if (TIMER == TIM3) {                     \
       __HAL_RCC_TIM3_CLK_DISABLE();}         \
      else if (TIMER == TIM4) {                     \
       __HAL_RCC_TIM4_CLK_DISABLE();}       \
      else if (TIMER == TIM5) {                     \
       __HAL_RCC_TIM5_CLK_DISABLE();}       

#endif /* END TEACUP_HAL_CONFIG  */
