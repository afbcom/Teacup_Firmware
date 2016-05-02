
/** \file
  \brief Parallel 4-bit subsystem

  This implements this custom interface often seen on LCD displays which
  uses 4 data lines (D0..D3) and 3 control lines:

    RS = Register Select: High for data input, Low for instruction input.

    RW = Read/Write:      High for read, Low for write.

    E = Enable:           A line for triggering a read or a write. Its detailed
                          usage is a bit complicated.

    This implementation was written in the hope to be exchangeable with using
    I2C or SPI as display bus for the same display.

    Other than the I2C implementation, this one uses no send buffer. Bytes can
    be sent quickly enough to allow sending them immediately.
*/

#include "parallel-4bit.h"
#include "delay.h"
#include "pinio.h"


// Check for the necessary pins.
#if defined DISPLAY_BUS_4BIT
  #ifndef DISPLAY_BUS_PIN_RS
    #error DISPLAY_BUS_4BIT defined, but not DISPLAY_BUS_PIN_RS.
  #endif
  #ifndef DISPLAY_BUS_PIN_RW
    #error DISPLAY_BUS_4BIT defined, but not DISPLAY_BUS_PIN_RW.
  #endif
  #ifndef DISPLAY_BUS_PIN_E
    #error DISPLAY_BUS_4BIT defined, but not DISPLAY_BUS_PIN_E.
  #endif
  #ifndef DISPLAY_BUS_PIN_D4
    #error DISPLAY_BUS_4BIT defined, but not DISPLAY_BUS_PIN_D4.
  #endif
  #ifndef DISPLAY_BUS_PIN_D5
    #error DISPLAY_BUS_4BIT defined, but not DISPLAY_BUS_PIN_D5.
  #endif
  #ifndef DISPLAY_BUS_PIN_D6
    #error DISPLAY_BUS_4BIT defined, but not DISPLAY_BUS_PIN_D6.
  #endif
  #ifndef DISPLAY_BUS_PIN_D7
    #error DISPLAY_BUS_4BIT defined, but not DISPLAY_BUS_PIN_D7.
  #endif
#endif


static void e_pulse(void) {
  WRITE(DISPLAY_BUS_PIN_E, 1);
  delay_us(1);
  WRITE(DISPLAY_BUS_PIN_E, 0);
}

/**
  Inititalise the subsystem.
*/
void parallel_4bit_init(void) {

  SET_OUTPUT(DISPLAY_BUS_PIN_RS);
  WRITE(DISPLAY_BUS_PIN_RS, 0);

  SET_OUTPUT(DISPLAY_BUS_PIN_RW);
  WRITE(DISPLAY_BUS_PIN_RW, 0);

  SET_OUTPUT(DISPLAY_BUS_PIN_E);
  WRITE(DISPLAY_BUS_PIN_E, 0);

  /**
    Reset the display.
  */
  SET_OUTPUT(DISPLAY_BUS_PIN_D4);
  SET_OUTPUT(DISPLAY_BUS_PIN_D5);
  SET_OUTPUT(DISPLAY_BUS_PIN_D6);
  SET_OUTPUT(DISPLAY_BUS_PIN_D7);

  // Initial write to lcd is 8 bit.
  WRITE(DISPLAY_BUS_PIN_D4, 1);
  WRITE(DISPLAY_BUS_PIN_D5, 1);
  WRITE(DISPLAY_BUS_PIN_D6, 0);
  WRITE(DISPLAY_BUS_PIN_D7, 0);
  e_pulse();
  delay_ms(5);    // Delay, busy flag can't be checked here.

  // Repeat last command.
  e_pulse();
  delay_us(100);   // Delay, busy flag can't be checked here.

  // Repeat last command a third time.
  e_pulse();
  delay_us(100);   // Delay, busy flag can't be checked here.

  // Now configure for 4 bit mode.
  WRITE(DISPLAY_BUS_PIN_D4, 0);
  e_pulse();
  delay_us(100);   // Some displays need this additional delay.
}

/**
  Read a byte from the bus. Doing so is e.g. required to detect wether the
  display is busy.

  \param rs   1: Read data.
              0: Read busy flag / address counter.

  \return Byte read from LCD controller.
*/
static uint8_t parallel_4bit_read(uint8_t rs) {
  uint8_t data;

  if (rs)
    WRITE(DISPLAY_BUS_PIN_RS, 1);         // Read data.
  else
    WRITE(DISPLAY_BUS_PIN_RS, 0);         // Read busy flag.
  WRITE(DISPLAY_BUS_PIN_RW, 1);           // Read mode.

  SET_INPUT(DISPLAY_BUS_PIN_D4);
  SET_INPUT(DISPLAY_BUS_PIN_D5);
  SET_INPUT(DISPLAY_BUS_PIN_D6);
  SET_INPUT(DISPLAY_BUS_PIN_D7);

  data = 0;

  // Read high nibble.
  WRITE(DISPLAY_BUS_PIN_E, 1);
  delay_us(1);
  if (READ(DISPLAY_BUS_PIN_D4)) data |= 0x10;
  if (READ(DISPLAY_BUS_PIN_D5)) data |= 0x20;
  if (READ(DISPLAY_BUS_PIN_D6)) data |= 0x40;
  if (READ(DISPLAY_BUS_PIN_D7)) data |= 0x80;
  WRITE(DISPLAY_BUS_PIN_E, 0);

  delay_us(1);

  // Read low nibble.
  WRITE(DISPLAY_BUS_PIN_E, 1);
  delay_us(1);
  if (READ(DISPLAY_BUS_PIN_D4)) data |= 0x01;
  if (READ(DISPLAY_BUS_PIN_D5)) data |= 0x02;
  if (READ(DISPLAY_BUS_PIN_D6)) data |= 0x04;
  if (READ(DISPLAY_BUS_PIN_D7)) data |= 0x08;
  WRITE(DISPLAY_BUS_PIN_E, 0);

  return data;
}

/**
  Report wether the bus or the connected display is busy.

  \return Wether the bus is busy, which means that eventual new transactions
          would have to wait.
*/
uint8_t parallel_4bit_busy(void) {
  uint8_t status;

  status = parallel_4bit_read(0);

  return status & 0x80;
}

/**
  Send a byte to the bus partner.

  \param data       The byte to be sent.

  \param rs   1 = parallel_4bit_data:        Write data.
              0 = parallel_4bit_instruction: Write instruction.

  Other than other bus implementations we do not buffer here. Writing a byte
  takes just some 5 microseconds and there is nothing supporting such writes in
  hardware, so the overhead of buffering is most likely not worth the effort.
*/
void parallel_4bit_write(uint8_t data, enum rs_e rs) {

  // Wait for the display to become ready.
  while (parallel_4bit_busy());

  // Setup for writing.
  WRITE(DISPLAY_BUS_PIN_RS, rs);          // Write data / instruction.
  WRITE(DISPLAY_BUS_PIN_RW, 0);           // Write mode.

  SET_OUTPUT(DISPLAY_BUS_PIN_D4);
  SET_OUTPUT(DISPLAY_BUS_PIN_D5);
  SET_OUTPUT(DISPLAY_BUS_PIN_D6);
  SET_OUTPUT(DISPLAY_BUS_PIN_D7);

  // Output high nibble.
  WRITE(DISPLAY_BUS_PIN_D4, (data & 0x10) ? 1 : 0);
  WRITE(DISPLAY_BUS_PIN_D5, (data & 0x20) ? 1 : 0);
  WRITE(DISPLAY_BUS_PIN_D6, (data & 0x40) ? 1 : 0);
  WRITE(DISPLAY_BUS_PIN_D7, (data & 0x80) ? 1 : 0);
  e_pulse();

  // Output low nibble.
  WRITE(DISPLAY_BUS_PIN_D4, (data & 0x01) ? 1 : 0);
  WRITE(DISPLAY_BUS_PIN_D5, (data & 0x02) ? 1 : 0);
  WRITE(DISPLAY_BUS_PIN_D6, (data & 0x04) ? 1 : 0);
  WRITE(DISPLAY_BUS_PIN_D7, (data & 0x08) ? 1 : 0);
  e_pulse();

#if 0
        /* all data pins high (inactive) */
        LCD_DATA0_PORT |= _BV(LCD_DATA0_PIN);
        LCD_DATA1_PORT |= _BV(LCD_DATA1_PIN);
        LCD_DATA2_PORT |= _BV(LCD_DATA2_PIN);
        LCD_DATA3_PORT |= _BV(LCD_DATA3_PIN);
#endif /* 0 */
}
