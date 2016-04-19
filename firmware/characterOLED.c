
// Derived from LiquidCrystal by David Mellis
// With portions adapted from Elco Jacobs OLEDFourBit
// Modified for 4-bit operation of the Winstar 16x2 Character OLED
// By W. Earl for Adafruit - 6/30/12
// Initialization sequence fixed by Technobly - 9/22/2013
// ported form C++ to C by Stefan Lasse

#include "characterOLED.h"


// On power up, the display is initilaized as:
// 1. Display clear
// 2. Function set:
//    DL="1": 8-bit interface data
//    N="0": 1-line display
//    F="0": 5 x 8 dot character font
// 3. Power turn off
//    PWR=”0”
// 4. Display on/off control: D="0": Display off C="0": Cursor off B="0": Blinking off
// 5. Entry mode set
//    I/D="1": Increment by 1
//    S="0": No shift
// 6. Cursor/Display shift/Mode / Pwr
//    S/C=”0”, R/L=”1”: Shifts cursor position to the right
//    G/C=”0”: Character mode
//    Pwr=”1”: Internal DCDC power on
//
// Note, however, that resetting the Arduino doesn't reset the LCD, so we
// can't assume that its in that state when a sketch starts (and the
// LiquidCrystal constructor is called).



/* =====================================================================
    Display subsystem

  code to communicate with the OLED display.
  It is based on a Winstar 16x2 Character OLED.

  This display here in use is
  - 2 lines with 16 characters each
  - 5x7 dots per character
  - driven in 4-bit-mode
====================================================================== */


/* ---------------------------------------------------------------------
    initialize OLED display
 --------------------------------------------------------------------- */
void OLEDinit(uint8_t ver){

  _oled_ver = ver;
  if(_oled_ver != OLED_V1 && _oled_ver != OLED_V2){
    _oled_ver = OLED_V2; // if error, default to newer version
  }

  _data_pins[0] = data4;
  _data_pins[1] = data5;
  _data_pins[2] = data6;
  _data_pins[3] = data7;

  OLEDpinMode(rs_pin, OUTPUT);
  OLEDpinMode(_rw_pin, OUTPUT);
  OLEDpinMode(_enable_pin, OUTPUT);

  _displayfunction = LCD_FUNCTIONSET | LCD_4BITMODE;

  OLEDbegin(16, 2);
}

/* ---------------------------------------------------------------------
    unknown
 --------------------------------------------------------------------- */
void OLEDpinMode(uint8_t pin, uint8_t mode){

  if (mode) {
    LCD_DDR |= (1 << pin);
  } //output
  else {
    LCD_DDR &= ~(1 << pin);
  } //input
}

/* ---------------------------------------------------------------------
    unknown
 --------------------------------------------------------------------- */
void OLEDdigitalWrite(uint8_t pin,uint8_t value){

  if (value == LOW) {
    LCD_PORT &= ~(1 << pin);
  } //If low, write 0
  else {
    LCD_PORT |= (1 << pin);
  } //if high, write 1
}

/* ---------------------------------------------------------------------
    unknown
 --------------------------------------------------------------------- */
uint8_t OLEDdigitalRead(uint8_t pin){

  return (LCD_PIN & (1 << pin));
}

/* ---------------------------------------------------------------------
    unknown
 --------------------------------------------------------------------- */
void OLEDbegin(uint8_t cols, uint8_t lines){

  _numlines = lines;
  _currline = 0;

  OLEDpinMode(rs_pin, OUTPUT);
  OLEDpinMode(_rw_pin, OUTPUT);
  OLEDpinMode(_enable_pin, OUTPUT);

  OLEDdigitalWrite(rs_pin, LOW);
  OLEDdigitalWrite(_enable_pin, LOW);
  OLEDdigitalWrite(_rw_pin, LOW);

  _delay_us(50000); // give it some time to power up

  // Now we pull both RS and R/W low to begin commands
  for(int i = 0; i < 4; i++){
    OLEDpinMode(_data_pins[i], OUTPUT);
    OLEDdigitalWrite(_data_pins[i], LOW);
  }

  // Initialization sequence is not quite as documented by Winstar.
  // Documented sequence only works on initial power-up.
  // An additional step of putting back into 8-bit mode first is
  // required to handle a warm-restart.
  //
  // In the data sheet, the timing specs are all zeros(!).  These have been tested to
  // reliably handle both warm & cold starts.

  // 4-Bit initialization sequence from Technobly
  OLEDwrite4bits(0x03); // Put back into 8-bit mode
  _delay_us(5000);
  if(_oled_ver == OLED_V2){  // only run extra command for newer displays
    OLEDwrite4bits(0x08);
    _delay_us(5000);
  }

  OLEDwrite4bits(0x02); // Put into 4-bit mode
  _delay_us(5000);
  OLEDwrite4bits(0x02);
  _delay_us(5000);
  OLEDwrite4bits(0x08);
  _delay_us(5000);

  OLEDcommand(0x08);  // Turn Off
  _delay_us(5000);
  OLEDcommand(0x01);  // Clear Display
  _delay_us(5000);
  OLEDcommand(0x06);  // Set Entry Mode
  _delay_us(5000);
  OLEDcommand(0x02);  // Home Cursor
  _delay_us(5000);
  OLEDcommand(0x0C);  // Turn On - enable cursor & blink
  _delay_us(5000);
}

/* ---------------------------------------------------------------------
    clear display
 --------------------------------------------------------------------- */
void OLEDclear(void){

  OLEDcommand(LCD_CLEARDISPLAY);  // clear display, set cursor position to zero
  //  _delay_us(2000);  // this command takes a long time!
}

/* ---------------------------------------------------------------------
    set cursor to home position
 --------------------------------------------------------------------- */
void OLEDhome(void){
  OLEDcommand(LCD_RETURNHOME);  // set cursor position to zero
  //  _delay_us(2000);  // this command takes a long time!
}

/* ---------------------------------------------------------------------
    set cursor to x-y-position (home = 0,0)
 --------------------------------------------------------------------- */
void OLEDsetCursor(uint8_t col, uint8_t row){

  uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};
  if (row >= _numlines){
    row = 0;  //write to first line if out off bounds
  }

  OLEDcommand(LCD_SETDDRAMADDR | (col + row_offsets[row]));
}

/* ---------------------------------------------------------------------
    Turn the display on/off (quickly)
 --------------------------------------------------------------------- */
void OLEDnoDisplay(void){

  _displaycontrol &= ~LCD_DISPLAYON;
  OLEDcommand(LCD_DISPLAYCONTROL | _displaycontrol);
}

/* ---------------------------------------------------------------------
    unknown
 --------------------------------------------------------------------- */
void OLEDdisplay(void){

  _displaycontrol |= LCD_DISPLAYON;
  OLEDcommand(LCD_DISPLAYCONTROL | _displaycontrol);
}

/* ---------------------------------------------------------------------
    Turns the underline cursor off
 --------------------------------------------------------------------- */
void OLEDnoCursor(void){

  _displaycontrol &= ~LCD_CURSORON;
  OLEDcommand(LCD_DISPLAYCONTROL | _displaycontrol);
}

/* ---------------------------------------------------------------------
    Turns the underline cursor on
 --------------------------------------------------------------------- */
void OLEDcursor(void){

  _displaycontrol |= LCD_CURSORON;
  OLEDcommand(LCD_DISPLAYCONTROL | _displaycontrol);
}

/* ---------------------------------------------------------------------
    Turn off the blinking cursor
 --------------------------------------------------------------------- */
void OLEDnoBlink(void){

  _displaycontrol &= ~LCD_BLINKON;
  OLEDcommand(LCD_DISPLAYCONTROL | _displaycontrol);
}

/* ---------------------------------------------------------------------
    Turn on the blinking cursor
 --------------------------------------------------------------------- */
void OLEDblink(void){

  _displaycontrol |= LCD_BLINKON;
  OLEDcommand(LCD_DISPLAYCONTROL | _displaycontrol);
}

/* ---------------------------------------------------------------------
    These commands scroll the display without changing the RAM
 --------------------------------------------------------------------- */
void OLEDscrollDisplayLeft(void){

  OLEDcommand(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}

/* ---------------------------------------------------------------------
    unknown
 --------------------------------------------------------------------- */
void OLEDscrollDisplayRight(void){

  OLEDcommand(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

/* ---------------------------------------------------------------------
    This is for text that flows Left to Right
 --------------------------------------------------------------------- */
void OLEDleftToRight(void){

  _displaymode |= LCD_ENTRYLEFT;
  OLEDcommand(LCD_ENTRYMODESET | _displaymode);
}

/* ---------------------------------------------------------------------
    This is for text that flows Right to Left
 --------------------------------------------------------------------- */
void OLEDrightToLeft(void){

  _displaymode &= ~LCD_ENTRYLEFT;
  OLEDcommand(LCD_ENTRYMODESET | _displaymode);
}

/* ---------------------------------------------------------------------
    This will 'right justify' text from the cursor
 --------------------------------------------------------------------- */
void OLEDautoscroll(void){

  _displaymode |= LCD_ENTRYSHIFTINCREMENT;
  OLEDcommand(LCD_ENTRYMODESET | _displaymode);
}

/* ---------------------------------------------------------------------
    This will 'left justify' text from the cursor
 --------------------------------------------------------------------- */
void OLEDnoAutoscroll(void){

  _displaymode &= ~LCD_ENTRYSHIFTINCREMENT;
  OLEDcommand(LCD_ENTRYMODESET | _displaymode);
}

/* ---------------------------------------------------------------------
    Allows us to fill the first 8 CGRAM locations
    with custom characters
 --------------------------------------------------------------------- */
void OLEDcreateChar(uint8_t location, uint8_t charmap[]){

  location &= 0x7; // we only have 8 locations 0-7
  OLEDcommand(LCD_SETCGRAMADDR | (location << 3));
  for (int i=0; i<8; i++){
    OLEDwriteC(charmap[i]);
  }
}

/* ---------------------------------------------------------------------
    send a command to the display
 --------------------------------------------------------------------- */
inline void OLEDcommand(uint8_t value){

  OLEDsend(value, LOW);
  OLEDwaitForReady();
}

/* ---------------------------------------------------------------------
    unknown
 --------------------------------------------------------------------- */
inline size_t OLEDwriteC(uint8_t value){

  OLEDsend(value, HIGH);
  OLEDwaitForReady();
}

/* ---------------------------------------------------------------------
    write either command or data
 --------------------------------------------------------------------- */
void OLEDsend(uint8_t value, uint8_t mode){

  OLEDdigitalWrite(rs_pin, mode);
  OLEDpinMode(_rw_pin, OUTPUT);
  OLEDdigitalWrite(_rw_pin, LOW);

  OLEDwrite4bits(value >> 4);
  OLEDwrite4bits(value);
}

/* ---------------------------------------------------------------------
    unknown
 --------------------------------------------------------------------- */
void OLEDpulseEnable(void){

  OLEDdigitalWrite(_enable_pin, HIGH);
  _delay_us(50);    // TODO: Timing Spec?
  OLEDdigitalWrite(_enable_pin, LOW);
}

/* ---------------------------------------------------------------------
    unknown
 --------------------------------------------------------------------- */
void OLEDwrite4bits(uint8_t value){

  for(int i = 0; i < 4; i++){
    OLEDpinMode(_data_pins[i], OUTPUT);
    OLEDdigitalWrite(_data_pins[i], (value >> i) & 0x01);
  }

  _delay_us(50); // Timing spec?
  OLEDpulseEnable();
}

/* ---------------------------------------------------------------------
    Poll the busy bit until it goes LOW
 --------------------------------------------------------------------- */
void OLEDwaitForReady(void){

  unsigned char busy = 1;
  OLEDpinMode(_busy_pin, INPUT);
  OLEDdigitalWrite(rs_pin, LOW);
  OLEDdigitalWrite(_rw_pin, HIGH);

  do{
    OLEDdigitalWrite(_enable_pin, LOW);
    OLEDdigitalWrite(_enable_pin, HIGH);

    _delay_us(10);
    busy = OLEDdigitalRead(_busy_pin);
    OLEDdigitalWrite(_enable_pin, LOW);

    OLEDpulseEnable();    // get remaining 4 bits, which are not used.
  } while(busy);

  OLEDpinMode(_busy_pin, OUTPUT);
  OLEDdigitalWrite(_rw_pin, LOW);
}

/* ---------------------------------------------------------------------
    unknown
 --------------------------------------------------------------------- */
size_t OLEDprintCC(const char str[]){

  return OLEDwriteCC(str);
}

/* ---------------------------------------------------------------------
    unknown
 --------------------------------------------------------------------- */
size_t OLEDprintC(char c){

  return OLEDwriteC(c);
}

/* ---------------------------------------------------------------------
    send a string with length to the display
 --------------------------------------------------------------------- */
size_t OLEDwriteCCC(const uint8_t *buffer, size_t size){

  size_t n = 0;
  while(size--){
    n += OLEDwriteC(*buffer++);
  }

  return n;
}

/* ---------------------------------------------------------------------
    send a string to the display
 --------------------------------------------------------------------- */
size_t OLEDwriteCC(const char *str){

  if(str == NULL){
    return 0;
  }

  return OLEDwriteCCC((const uint8_t *)str, strlen(str));
}

