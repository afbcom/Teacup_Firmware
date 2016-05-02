
/** \file

  \brief Code specific to the SSD1306 display.
*/

/**
  D'oh. Shortly before completing this code, the display hardware died. Shows
  just black, for whatever reason. Accordingly I can't test new code any longer
  and writing code without seeing what it does makes no sense.

  What already works:

    - I2C with a queue for small transmissions. Sufficient to queue up sending
      a rendered character. It's filled by displaybus_write() and drained by
      the I2C interrupt. Larger transmissions are handled fine, too, but cause
      wait cycles.

    - 128 byte queue holding characters to send. This queue is filled by
      display_writechar(). It's drained by display_tick(), which processes,
      renders and forwards these characters to the I2C queue.

    - Display initialisation.

    - Clearing the display.

    - Writing text with display_writestr_P().

    - Writing formatted text with sendf_P(display_writechar, ...).

    - Current state of code should clear the display at startup, show a
      greeting message and start displaying current X/Y/Z coordinates, updated
      once per second. All this not in a particularly pretty fashion, but
      working.

  TODO list:

    - Procedures like display_clear() and display_set_cursor() should be queued
      up, too. Just like characters. Fonts start at 0x20, so 0x00..0x1F are
      available for command sequences. For example, setting the cursor could
      queue up 0x04 0x01 0x20 (3 bytes) to set the cursor to line 1, column 32.
      0x04 is the "command", bytes are queued up with display_writechar().

      This is necessary to enforce characters and cursor commands to happen in
      the right order. Currently, writing a few characters, moving the cursor
      elsewhere and writing even more characters results in all characters
      being written to the second position, because characters wait in the
      queue, while cursor movements are executed immediately.

      Code currently in display_set_cursor() would move to display_tick(), then.

    - Lot's of prettification. Like a nice background picture with the Teacup
      logo, like "Welcome to Teacup" as a greeting screen, like writing numbers
      to readable places and so on.

    - Get rid of i2c_test.c.

    - Allow different fonts. Already paraphrased in font.h and font.c. Needs
      a selection menu in Configtool, of course, the same way one can select
      display types.

    - It's a bit unclear wether this 'last_byte' flag to displaybus_write() is
      really ideal. Fact is, I2C transmissions need a start and an explicite
      ending. Also thinkable would be a displaybus_finalise() function
      which puts the marker in place. Saves a lot of shuffling parameters
      around.

      Yet another option would be to make sure the I2C send buffer is drained
      befpre sending the next transmission. I2C code already finalises a
      transmission on buffer drain, so only _reliable_ waiting needs an
      implementation.

      Each variant needs testing, which one gets away with the smallest code.
      Smallest code is likely the fastest code as well.

    - Here's an assistant to convert pictures/bitmaps into C code readable by
      the compiler: http://en.radzio.dxp.pl/bitmap_converter/
*/

#include "display.h"

#if defined TEACUP_C_INCLUDE && defined DISPLAY_TYPE_HD44780

#include "displaybus.h"
#include "delay.h"
#include "sendf.h"
#include "dda.h"


/**
 * Initializes the display's controller configuring the way of
 * displaying data.
 */
void display_init(void) {

  // Minimum initialisation time after power up.
  delay_ms(15);

  displaybus_init(0);

  // Write left to right, no display shifting.
  displaybus_write(0x06, parallel_4bit_instruction);
  // Display ON, cursor not blinking.
  displaybus_write(0x0C, parallel_4bit_instruction);
}

/**
  Clear the screen. This display has a dedicated command for doing it.
*/
void display_clear(void) {
  displaybus_write(0x01, parallel_4bit_instruction);
}

/**
  Sets the cursor to the given position.

  \param line   The vertical cursor position to set, in lines. First line is
                zero. Line height is character height, which is currently
                fixed to 8 pixels.

  \param column The horizontal cursor position to set, in pixels. First
                column is zero.

  Use this for debugging purposes, only. Regular display updates happen in
  display_clock().
*/
void display_set_cursor(uint8_t line, uint8_t column) {
  // Currently unimplemented.
}

/**
  Show a nice greeting. Pure eye candy.
*/
void display_greeting(void) {

  display_clear();
  display_writestr_P(PSTR("Welcome to Teacup"));

  // Forward this to the display immediately.
  while (buf_canread(display)) {
    display_tick();
  }

  // Allow the user to be proud of our work.
  delay_ms(5000);
}

/**
  Regular update of the display. Typically called once a second from clock.c.
*/
void display_clock(void) {

  display_clear();

  update_current_position();
  sendf_P(display_writechar, PSTR("X:%lq Y:%lq"),
          current_position.axis[X], current_position.axis[Y]);
}

/**
  Forwards a character from the display queue to display bus. As this is a
  character based display it's easy.
*/
void display_tick() {
  uint8_t data;

  if (displaybus_busy()) {
    return;
  }

  if (buf_canread(display)) {
    buf_pop(display, data);
    displaybus_write(data, parallel_4bit_data);
  }
}

#endif /* TEACUP_C_INCLUDE && DISPLAY_TYPE_HD44780 */
