
/** \file
  \brief Serial subsystem, ARM specific part.

  To be included from serial.c, for more details see there.

  Other than AVRs, ARMs feature a serial buffer in hardware, so we can get
  away without a software buffer and also without(!) interrupts.

  Code here is heavily inspired by serial_api.c of MBED
*/


#if defined TEACUP_C_INCLUDE && defined __ARM_STM32F4HAL__

#include "arduino.h"
#include "delay.h"
#include "stm32f4xx_hal_conf.h"


#ifdef XONXOFF
  #error XON/XOFF protocol not yet implemented for ARM. \
         See serial-avr.c for inspiration.
#endif

#define SERIAL_TIMEOUT (0xFFFF)

UART_HandleTypeDef huart;

void serial_init()
{

	GPIO_InitTypeDef  igpio;
	__HAL_RCC_GPIOA_CLK_ENABLE();
//	Enable USART2 clock
	__HAL_RCC_USART2_CLK_ENABLE(); //STM32F401 specific


	igpio.Pin = GPIO_PIN_2 | GPIO_PIN_3;
	igpio.Mode = GPIO_MODE_AF_PP;
	igpio.Pull = GPIO_NOPULL;
	igpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	igpio.Alternate = GPIO_AF7_USART2; //STM32F401 specific
	HAL_GPIO_Init(GPIOA, &igpio);



    #if !defined BAUD
    #define BAUD 115200
    #endif
    
//  huart.Instance = USART2;


	huart.Instance = USART2;

	huart.Init.BaudRate = (uint32_t)(BAUD);
	huart.Init.WordLength = UART_WORDLENGTH_8B;
	huart.Init.StopBits = UART_STOPBITS_1;
	huart.Init.Parity = UART_PARITY_NONE;
	huart.Init.Mode = UART_MODE_TX_RX;
	huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart.Init.OverSampling = UART_OVERSAMPLING_16;

	HAL_UART_Init(&huart);


}



uint8_t serial_rxchars(void) {
  return USART2->SR & USART_SR_RXNE;
}

 /** Read one character.*/

uint8_t serial_popchar(void) {
   uint8_t c = 0;
   HAL_UART_Receive(&huart, &c, 1 , SERIAL_TIMEOUT);

   return c;
}




 uint8_t serial_txchars(void) {
   return huart.TxXferCount;
}


/** Send one character.
*/

void serial_writechar(uint8_t data) {
  if ( !serial_txchars())       // Queue full?

    delay_us((1000000 / BAUD * 10) + 7);
	HAL_UART_Transmit(&huart, &data, 1, SERIAL_TIMEOUT);
}







#endif /* defined TEACUP_C_INCLUDE && defined __ARM_STM32F411__ */
