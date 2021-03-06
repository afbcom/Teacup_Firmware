
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
#include "hal_config.h"

#ifdef XONXOFF
  #error XON/XOFF protocol not yet implemented for ARM. \
         See serial-avr.c for inspiration.
#endif

#define SERIAL_TIMEOUT (0xFFFFFFFF)

UART_HandleTypeDef huart;

void serial_init(void){

  GPIO_InitTypeDef  igpio; 

  UART_CLK_ENABLE( SERIAL_UART );

  GPIO_CLK_ENABLE( SERIAL_TX );
  GPIO_CLK_ENABLE( SERIAL_RX ); 

  GPIO_SET_UART_ALT( igpio , SERIAL_UART);
  GPIO_CONFIGURE_PIN( igpio, SERIAL_RX );
  GPIO_CONFIGURE_PIN( igpio, SERIAL_TX );
  
  UART_CONFIGURE( huart, SERIAL_UART );

}

/** Check wether characters can be read.

  Other than the AVR implementation this returns not the number of characters
  in the line, but only wether there is at least one or not.
*/
uint8_t serial_rxchars(void) {  
   return __HAL_UART_GET_FLAG(&huart,UART_FLAG_RXNE);
}

// /** Read one character.*/
uint8_t serial_popchar(void) {

  uint8_t c = 0;

  HAL_UART_Receive(&huart, &c, 1 , SERIAL_TIMEOUT);

  return c;
}

/** Check wether characters can be written
  ***STM-NUCLEO-PORT: THIS IS NOT NEEDED ***
*/
uint8_t serial_txchars(void) {
  return huart.TxXferCount;
}


// //TODO Change or rethink delays
void serial_writechar(uint8_t data) {
  HAL_UART_Transmit(&huart, &data, 1, SERIAL_TIMEOUT);
}




#endif /* defined TEACUP_C_INCLUDE && defined __ARM_STM32F411__ */
