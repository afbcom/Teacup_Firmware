
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
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_conf.h"


#ifdef XONXOFF
  #error XON/XOFF protocol not yet implemented for ARM. \
         See serial-avr.c for inspiration.
#endif

#define SERIAL_TIMEOUT (0xFFFF)

UART_HandleTypeDef huart;
	
void serial_init()
{
	
//	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	GPIO_InitTypeDef  igpio;
	
		// Enable TX/RX clock (GPIOA)
	__HAL_RCC_GPIOA_CLK_ENABLE();
    // Enable USART2 clock
	__HAL_RCC_USART2_CLK_ENABLE(); //STM32F401 specific
	
	
	// AF 4bits per channel
    // Alternate functions from DM00115249.pdf datasheet (page 47; table 9)
//	#undef GPIO_PIN
  //  #define GPIO_PIN(number) (GPIO_PIN_ ## number)

	igpio.Pin = GPIO_PIN_2|GPIO_PIN_3;
	igpio.Mode = GPIO_MODE_AF_PP;
	igpio.Pull = GPIO_PULLUP;
	igpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	igpio.Alternate = GPIO_AF7_USART2; //STM32F401 specific
	HAL_GPIO_Init(GPIOA, &igpio);

//	#undef GPIO_PIN
	
	/* Configure the UART */

    /* 19.3.4 Fractional baud rate generation => reference manual for STM32F411
    Set BRR for 115,200 Hz 
    div = 48MHz/(16*BAUD)
    Mantisse = int(div) << 8
    Divisor = int((div - int(div))*16)
    BRR = Mantisse + Divisor
    */
    #if !defined BAUD
    #define BAUD 115200
    #endif


    #if (BAUD == 115200)
        huart.Init.BaudRate = (uint32_t)(0x01A1);
	// UART_DIV_SAMPLING8(_PCLK_, _BAUD_)   // Does this do what i think??
	// Or are the other macros that will 
    #else
        huart.Init.BaudRate = (uint32_t)(BAUD); // BAUD is configured
						// by HAL_UART_Init
    #endif
	
	huart.Init.WordLength = UART_WORDLENGTH_8B;
	huart.Init.StopBits = UART_STOPBITS_1;
	huart.Init.Parity = UART_PARITY_NONE;
	huart.Init.Mode = UART_MODE_TX_RX;
	huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart.Init.OverSampling = UART_OVERSAMPLING_16;
	
	HAL_UART_Init(&huart);

}

/** Check wether characters can be read.

  Other than the AVR implementation this returns not the number of characters
  in the line, but only wether there is at least one or not.
  
  ***STM-NUCLEO-PORT: THIS IS NOT NEEDED ***
*/
uint8_t serial_rxchars(void) {  
  return huart.RxXferCount;
}

/** Read one character.*/
uint8_t serial_popchar(void) {
  uint8_t c[2] = {0,0};
  HAL_UART_Receive(&huart, c, 1 , SERIAL_TIMEOUT);

  return c;
}

/** Check wether characters can be written
  ***STM-NUCLEO-PORT: THIS IS NOT NEEDED ***
*/
uint8_t serial_txchars(void) {
  return huart.TxXferCount;
}

//Old Code
// void serial_writechar(uint8_t data) {
  // if ( !serial_txchars())       // Queue full?
    // delay_us((1000000 / BAUD * 10) + 7);
  // USART2->DR = (uint32_t)(data & 0x1FF);
// }

//TODO Change or rethink delays
void serial_writechar(uint8_t data) {

    delay_us((1000000 / BAUD * 10) + 7);
	HAL_UART_Transmit(&huart, &data, 1, SERIAL_TIMEOUT);
}


#endif /* defined TEACUP_C_INCLUDE && defined __ARM_STM32F411__ */
