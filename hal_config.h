#ifndef  TEACUP_HAL_CONFIG
#define TEACUP_HAL_CONFIG

#include "arduino_stm32f411.h"
#include "stm32f4xx_hal_conf.h"

#define GPIO_CLK_ENABLE_ALL 1
	
/*************************************************************************
 *					STEPPER CONFIGURATION
 ***********************************************************************/
#define STEPPER_TIMER 		TIM5
#define STEPPER_TIMER_IRQN 	TIM5_IRQn
#define STEPPER_TIMER_NVIC  NVIC_PRIORITYGROUP_0

/*************************************************************************
 *					SERIAL CONFIGURATION
 ***********************************************************************/

 
#define SERIAL_UART 		    USART2
#define SERIAL_BAUD 		    115200
#define SERIAL_MODE 		    UART_MODE_TX_RX
#define SERIAL_PARITY 		  UART_PARITY_NONE
#define SERIAL_WORDLENGTH 	8
#define SERIAL_STOPBITS 	  1
#define SERIAL_OVERSAMPLING 16
#define SERIAL_CTRL         UART_HWCONTROL_NONE

#define SERIAL_TX_PIN 		  GPIO_PIN_2 
#define SERIAL_TX_PORT 		  PA_2_PORT
#define SERIAL_TX_MODE 		  GPIO_MODE_AF_PP
#define SERIAL_TX_PULL 		  GPIO_NOPULL
#define SERIAL_TX_SPEED		  GPIO_SPEED_FREQ_VERY_HIGH
#define SERIAL_TX_ALT	      GPIO_AF7_USART2

#define SERIAL_RX_PIN 		  GPIO_PIN_3
#define SERIAL_RX_PORT		  PA_3_PORT
#define SERIAL_RX_MODE 		  GPIO_MODE_AF_PP
#define SERIAL_RX_PULL 		  GPIO_NOPULL
#define SERIAL_RX_SPEED		  GPIO_SPEED_FREQ_VERY_HIGH
#define SERIAL_RX_ALT	      GPIO_AF7_USART2


#define UART_SET_INSTANCE( UART )	\
  if ( UART == USART1 ) huart.Instance = USART1;		\
  else huart.Instance = USART2;

#define UART_SET_BAUD( NUMBER)								  		\
	huart.Init.BaudRate = (uint32_t)(NUMBER);	

#define UART_SET_STOPBITS(NUMBER)								  	\
	if (NUMBER == 2) huart.Init.StopBits = UART_STOPBITS_2;   			\
	else huart.Init.StopBits = UART_STOPBITS_1;			  	  

#define UART_SET_WORDLENGTH(NUMBER) 								\
	if (NUMBER == 9) huart.Init.WordLength = UART_WORDLENGTH_9B;			\
	else huart.Init.WordLength = UART_WORDLENGTH_8B; 				

#define UART_SET_PARITY(MODE)										\
	if( MODE == UART_PARITY_EVEN )  huart.Init.Parity = UART_PARITY_EVEN;   	 		\
	else if(MODE == UART_PARITY_ODD) huart.Init.Parity = UART_PARITY_ODD;  	 		\
	else huart.Init.Parity = UART_PARITY_NONE;						

#define UART_SET_MODE(MODE)										\
	if( MODE == UART_MODE_TX ) huart.Init.Mode = UART_MODE_TX;				\
	else if(MODE == UART_MODE_RX) huart.Init.Mode = UART_MODE_RX; 		\
	else huart.Init.Mode = UART_MODE_TX_RX;

#define UART_SET_OVERSAMPLING(NUMBER)								\
	if( NUMBER == 8 ) huart.Init.OverSampling = UART_OVERSAMPLING_8;		\
	else huart.Init.OverSampling = UART_OVERSAMPLING_16;

#define UART_SET_HWCTRL(MODE)               \
  if( MODE == UART_HWCONTROL_RTS ) huart.Init.HwFlowCtl = UART_HWCONTROL_RTS;    \
  else if( MODE == UART_HWCONTROL_CTS ) huart.Init.HwFlowCtl = UART_HWCONTROL_CTS; \
  else if( MODE == UART_HWCONTROL_RTS_CTS ) huart.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS; \
  else huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;

#define UART_CONFIGURE( NAME  )                     \
    UART_CLK_ENABLE( NAME ##_UART ); \
    UART_SET_INSTANCE( NAME ##_UART ); \
    UART_SET_BAUD( NAME ##_BAUD ); \
    UART_SET_STOPBITS( NAME ## _STOPBITS); \
    UART_SET_WORDLENGTH( NAME ## _WORDLENGTH);   \
    UART_SET_PARITY( NAME ##_PARITY);         \
    UART_SET_MODE(NAME ##_MODE);          \
    UART_SET_OVERSAMPLING( NAME ##_OVERSAMPLING);  \
    UART_SET_HWCTRL(NAME ## _CTRL); \
    HAL_UART_Init(&huart);


#define UART_CLK_ENABLE( UART ) 		\
  if(UART  == USART1 ){ 				\
  __HAL_RCC_USART1_CLK_ENABLE();}		\
  else{	__HAL_RCC_USART2_CLK_ENABLE();}	

#define UART_CLK_DISABLE(UART) 		\
  if(UART  == USART2 ){ 				\
  __HAL_RCC_USART2_CLK_DISABLE();}		\
  else if (UART == USART1 ){			\
  	__HAL_RCC_USART1_CLK_DISABLE();}		


#define TIMER_CLK_ENABLE(TIMER) 					          \
      if (TIMER == TIM1) {                          \
        __HAL_RCC_TIM1_CLK_ENABLE();}     			    \
      else if (TIMER == TIM2){                      \
       __HAL_RCC_TIM2_CLK_ENABLE();}           		  \
      else if (TIMER == TIM3) {                     \
       __HAL_RCC_TIM3_CLK_ENABLE();}    			      \
      else if (TIMER == TIM4) {                     \
       __HAL_RCC_TIM4_CLK_ENABLE();}				        \
      else if (TIMER == TIM5) {                     \
       __HAL_RCC_TIM5_CLK_ENABLE();}			


#define TIMER_CLK_DISABLE(TIMER) 					\
      if (TIMER == TIM1) {                          \
        __HAL_RCC_TIM1_CLK_DISABLE();}     			\
      else if (TIMER == TIM2){                      \
       __HAL_RCC_TIM2_CLK_DISABLE();}           	\
      else if (TIMER == TIM3) {                     \
       __HAL_RCC_TIM3_CLK_DISABLE();}    			\
      else if (TIMER == TIM4) {                     \
       __HAL_RCC_TIM4_CLK_DISABLE();}				\
      else if (TIMER == TIM5) {                     \
       __HAL_RCC_TIM5_CLK_DISABLE();}				

/*************************************************************************
 *					GPIO CONFIGURATION
 ***********************************************************************/

#define GPIO_CONFIGURE_PIN( name)				\
	igpio.Pin =  name  ##_PIN;			\
	igpio.Mode = name ## _MODE ;				  \
	igpio.Speed = name ## _SPEED;				  \
  igpio.Pull = name ## _PULL;           \
	igpio.Alternate = name ## _ALT; 			\
	HAL_GPIO_Init( name ## _PORT, &igpio);


#define GPIO_CLK_ENABLE(PORT) 					    \
  if (PORT == GPIOA) {                      \
    __HAL_RCC_GPIOA_CLK_ENABLE();}     			\
  else if (PORT == GPIOB){                  \
   __HAL_RCC_GPIOB_CLK_ENABLE();}           \
  else if (PORT == GPIOC) {                 \
   __HAL_RCC_GPIOC_CLK_ENABLE();}    			  \
  else if (PORT == GPIOD) {                 \
   __HAL_RCC_GPIOD_CLK_ENABLE();}				    \
  else if (PORT == GPIOH) {                 \
   __HAL_RCC_GPIOH_CLK_ENABLE();}				    \
  else if (PORT == GPIO_CLK_ENABLE_ALL){		\
  __HAL_RCC_GPIOA_CLK_ENABLE();					    \
  __HAL_RCC_GPIOB_CLK_ENABLE();					    \
  __HAL_RCC_GPIOC_CLK_ENABLE();					    \
  __HAL_RCC_GPIOD_CLK_ENABLE();					    \
  __HAL_RCC_GPIOH_CLK_ENABLE();}

#define GPIO_CLK_DISBLE(PORT) 					    \
  if (PORT == GPIOA) {                      \
    __HAL_RCC_GPIOA_CLK_DISABLE();}     		\
  else if (PORT == GPIOB){                  \
   __HAL_RCC_GPIOB_CLK_DISABLE();}          \
  else if (PORT == GPIOC) {                 \
   __HAL_RCC_GPIOC_CLK_DISABLE();}    			\
  else if (PORT == GPIOD) {                 \
   __HAL_RCC_GPIOD_CLK_DISABLE();}				  \
  else if (PORT == GPIOH) {                 \
   __HAL_RCC_GPIOH_CLK_DISABLE();}				  \
  else if (PORT ==  GPIO_CLK_ENABLE_ALL){		\
  __HAL_RCC_GPIOA_CLK_DISABLE();				    \
  __HAL_RCC_GPIOB_CLK_DISABLE();				    \
  __HAL_RCC_GPIOC_CLK_DISABLE();				    \
  __HAL_RCC_GPIOD_CLK_DISABLE();				    \
  __HAL_RCC_GPIOH_CLK_DISABLE();}

#endif /* END TEACUP_HAL_CONFIG  */
