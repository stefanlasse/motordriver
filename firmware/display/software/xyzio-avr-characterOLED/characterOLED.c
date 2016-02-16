/*
* characterOLED.cpp
*
* Created: 12/23/2014 4:52:25 PM
*  Author: mkapoor
*/




// Derived from LiquidCrystal by David Mellis
// With portions adapted from Elco Jacobs OLEDFourBit
// Modified for 4-bit operation of the Winstar 16x2 Character OLED
// By W. Earl for Adafruit - 6/30/12
// Initialization sequence fixed by Technobly - 9/22/2013

#include "characterOLED.h"

//#include "Arduino.h"

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

characterOLED::characterOLED(uint8_t ver)
{
	init(ver);
}

void characterOLED::init(uint8_t ver)
{
	_oled_ver = ver;
	if(_oled_ver != OLED_V1 && _oled_ver != OLED_V2) {
		_oled_ver = OLED_V2; // if error, default to newer version
	}

	
	_data_pins[0] = data4;
	_data_pins[1] = data5;
	_data_pins[2] = data6;
	_data_pins[3] = data7;


	pinMode(rs_pin, OUTPUT);
	pinMode(_rw_pin, OUTPUT);
	pinMode(_enable_pin, OUTPUT);
	
	_displayfunction = LCD_FUNCTIONSET | LCD_4BITMODE;
	
	begin(16, 2);
}

void characterOLED::pinMode(uint8_t pin, uint8_t mode) {
	if (mode) {
		LCD_DDR |= (1 << pin);
	} //output
	else {
		LCD_DDR &= ~(1 << pin);
	} //input
}

void characterOLED::digitalWrite(uint8_t pin,uint8_t value) {
	if (value == LOW) {
		LCD_PORT &= ~(1 << pin);
	} //If low, write 0
	else {
		LCD_PORT |= (1 << pin);
	} //if high, write 1
}

uint8_t characterOLED::digitalRead(uint8_t pin) {
	return (LCD_PIN & (1 << pin));
}

void characterOLED::begin(uint8_t cols, uint8_t lines)
{
	_numlines = lines;
	_currline = 0;
	
	pinMode(rs_pin, OUTPUT);
	pinMode(_rw_pin, OUTPUT);
	pinMode(_enable_pin, OUTPUT);
	
	digitalWrite(rs_pin, LOW);
	digitalWrite(_enable_pin, LOW);
	digitalWrite(_rw_pin, LOW);
	
	_delay_us(50000); // give it some time to power up
	
	// Now we pull both RS and R/W low to begin commands
	
	for (int i = 0; i < 4; i++) {
		pinMode(_data_pins[i], OUTPUT);
		digitalWrite(_data_pins[i], LOW);
	}

	// Initialization sequence is not quite as documented by Winstar.
	// Documented sequence only works on initial power-up.
	// An additional step of putting back into 8-bit mode first is
	// required to handle a warm-restart.
	//
	// In the data sheet, the timing specs are all zeros(!).  These have been tested to
	// reliably handle both warm & cold starts.

	// 4-Bit initialization sequence from Technobly
	write4bits(0x03); // Put back into 8-bit mode
	_delay_us(5000);
	if(_oled_ver == OLED_V2) {  // only run extra command for newer displays
		write4bits(0x08);
		_delay_us(5000);
	}
	write4bits(0x02); // Put into 4-bit mode
	_delay_us(5000);
	write4bits(0x02);
	_delay_us(5000);
	write4bits(0x08);
	_delay_us(5000);
	
	command(0x08);	// Turn Off
	_delay_us(5000);
	command(0x01);	// Clear Display
	_delay_us(5000);
	command(0x06);	// Set Entry Mode
	_delay_us(5000);
	command(0x02);	// Home Cursor
	_delay_us(5000);
	command(0x0C);	// Turn On - enable cursor & blink
	_delay_us(5000);
}

/********** high level commands, for the user! */
void characterOLED::clear()
{
	command(LCD_CLEARDISPLAY);  // clear display, set cursor position to zero
	//  _delay_us(2000);  // this command takes a long time!
}

void characterOLED::home()
{
	command(LCD_RETURNHOME);  // set cursor position to zero
	//  _delay_us(2000);  // this command takes a long time!
}

void characterOLED::setCursor(uint8_t col, uint8_t row)
{
	uint8_t row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
	if ( row >= _numlines )
	{
		row = 0;  //write to first line if out off bounds
	}
	
	command(LCD_SETDDRAMADDR | (col + row_offsets[row]));
}

// Turn the display on/off (quickly)
void characterOLED::noDisplay()
{
	_displaycontrol &= ~LCD_DISPLAYON;
	command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void characterOLED::display()
{
	_displaycontrol |= LCD_DISPLAYON;
	command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// Turns the underline cursor on/off
void characterOLED::noCursor()
{
	_displaycontrol &= ~LCD_CURSORON;
	command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void characterOLED::cursor()
{
	_displaycontrol |= LCD_CURSORON;
	command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// Turn on and off the blinking cursor
void characterOLED::noBlink()
{
	_displaycontrol &= ~LCD_BLINKON;
	command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void characterOLED::blink()
{
	_displaycontrol |= LCD_BLINKON;
	command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// These commands scroll the display without changing the RAM
void characterOLED::scrollDisplayLeft(void)
{
	command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}
void characterOLED::scrollDisplayRight(void)
{
	command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

// This is for text that flows Left to Right
void characterOLED::leftToRight(void)
{
	_displaymode |= LCD_ENTRYLEFT;
	command(LCD_ENTRYMODESET | _displaymode);
}

// This is for text that flows Right to Left
void characterOLED::rightToLeft(void)
{
	_displaymode &= ~LCD_ENTRYLEFT;
	command(LCD_ENTRYMODESET | _displaymode);
}

// This will 'right justify' text from the cursor
void characterOLED::autoscroll(void)
{
	_displaymode |= LCD_ENTRYSHIFTINCREMENT;
	command(LCD_ENTRYMODESET | _displaymode);
}

// This will 'left justify' text from the cursor
void characterOLED::noAutoscroll(void)
{
	_displaymode &= ~LCD_ENTRYSHIFTINCREMENT;
	command(LCD_ENTRYMODESET | _displaymode);
}

// Allows us to fill the first 8 CGRAM locations
// with custom characters
void characterOLED::createChar(uint8_t location, uint8_t charmap[])
{
	location &= 0x7; // we only have 8 locations 0-7
	command(LCD_SETCGRAMADDR | (location << 3));
	for (int i=0; i<8; i++)
	{
		write(charmap[i]);
	}
}

/*********** mid level commands, for sending data/cmds */

inline void characterOLED::command(uint8_t value)
{
	send(value, LOW);
	waitForReady();
}

inline size_t characterOLED::write(uint8_t value)
{
	send(value, HIGH);
	waitForReady();
}

/************ low level data pushing commands **********/

// write either command or data
void characterOLED::send(uint8_t value, uint8_t mode)
{
	digitalWrite(rs_pin, mode);
	pinMode(_rw_pin, OUTPUT);
	digitalWrite(_rw_pin, LOW);
	
	write4bits(value>>4);
	write4bits(value);
}

void characterOLED::pulseEnable(void)
{
	digitalWrite(_enable_pin, HIGH);
	_delay_us(50);    // Timing Spec?
	digitalWrite(_enable_pin, LOW);
}

void characterOLED::write4bits(uint8_t value)
{
	for (int i = 0; i < 4; i++)
	{
		pinMode(_data_pins[i], OUTPUT);
		digitalWrite(_data_pins[i], (value >> i) & 0x01);
	}
	_delay_us(50); // Timing spec?
	pulseEnable();
}

// Poll the busy bit until it goes LOW
void characterOLED::waitForReady(void)
{
	unsigned char busy = 1;
	pinMode(_busy_pin, INPUT);
	digitalWrite(rs_pin, LOW);
	digitalWrite(_rw_pin, HIGH);
	do
	{
		digitalWrite(_enable_pin, LOW);
		digitalWrite(_enable_pin, HIGH);

		_delay_us(10);
		busy = digitalRead(_busy_pin);
		digitalWrite(_enable_pin, LOW);
		
		pulseEnable();		// get remaining 4 bits, which are not used.
	}
	while(busy);
	
	pinMode(_busy_pin, OUTPUT);
	digitalWrite(_rw_pin, LOW);
}

size_t characterOLED::print(const char str[])
{
	return write(str);
}

size_t characterOLED::print(char c)
{
	return write(c);
}

size_t characterOLED::write(const uint8_t *buffer, size_t size)
{
	size_t n = 0;
	while (size--) {
		n += write(*buffer++);
	}
	return n;
}

size_t characterOLED::write(const char *str) {
	if (str == NULL) return 0;
	return write((const uint8_t *)str, strlen(str));
}



//main function
characterOLED lcd(OLED_V2);

void setup()
{
	// Print a message to the LCD.
	lcd.begin(16, 2);
	lcd.print("hello OLED World");
}


int main(void)
{
	_delay_ms(2000); //Startup
	cli();
	setup();
	sei();
	
	uint8_t i = 0;
	char str[16];

	while(1)
	{

		// set the cursor to column 0, line 1
		// (note: line 1 is the second row, since counting begins with 0):
		lcd.setCursor(0, 0);
		lcd.print("hello OLED World");
		lcd.setCursor(0, 1);
		
		lcd.print(i);
		lcd.setCursor(8, 1);
		itoa(i, str, 10);
		lcd.print(str);
		i ++ ;
		_delay_ms(1000);
		lcd.clear();
		
	}
}
