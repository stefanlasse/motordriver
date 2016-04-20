/*
 * characterOLED.h
 *
 * Created: 12/23/2014 4:52:46 PM
 *  Author: mkapoor
 */ 


#ifndef CHARACTEROLED_H_
#define CHARACTEROLED_H_

#ifndef F_CPU
#define F_CPU 20000000
#endif

#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include <avr/io.h>
#include <util/delay.h>

//#include "Print.h"

// OLED hardware versions
#define OLED_V1 0x01
#define OLED_V2 0x02

// commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x28
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80
#define LCD_PWRON 0x17
#define LCD_PWROFF 0x13

// flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

// flags for function set
#define LCD_8BITMODE    0x10
#define LCD_4BITMODE    0x00
#define LCD_JAPANESE    0x00
#define LCD_EUROPEAN_I  0x01
#define LCD_RUSSIAN     0x02
#define LCD_EUROPEAN_II 0x03

// Defines for pins
	// PB3 - enable
	// PB2 - rw (read / write)
	// PB1 - rs (reg select)
	// PB7 - Data 7 / busy
	// PB6 - Data 6
	// PB5 - Data 5
	// PB4 - Data 4

#define _enable_pin	  3
#define _rw_pin		    2
#define rs_pin		    1
#define data4		      4
#define data5		      5
#define data6		      6
#define _busy_pin	    7
#define data7		      7

#define OUTPUT		    1
#define INPUT		      0

// define Port
#define LCD_PORT      PORTB
#define LCD_DDR       DDRB
#define LCD_PIN       PINB

#define LOW			      0
#define HIGH		      1




void OLEDinit(uint8_t ver);

void OLEDbegin(uint8_t cols, uint8_t lines);

void OLEDclear(void);
void OLEDhome(void);

void OLEDnoDisplay(void);
void OLEDdisplay(void);
void OLEDnoBlink(void);
void OLEDblink(void);
void OLEDnoCursor(void);
void OLEDcursor(void);
void OLEDscrollDisplayLeft(void);
void OLEDscrollDisplayRight(void);
void OLEDleftToRight(void);
void OLEDrightToLeft(void);
void OLEDautoscroll(void);
void OLEDnoAutoscroll(void);
void OLEDbrightness(uint8_t bright);
void OLEDdigitalWrite(uint8_t pin,uint8_t value);
uint8_t OLEDdigitalRead(uint8_t pin);

void OLEDcreateChar(uint8_t, uint8_t[]);
void OLEDsetCursor(uint8_t, uint8_t);

void OLEDcommand(uint8_t);

size_t OLEDprintCC(const char[]);
size_t OLEDprintC(char);

size_t OLEDwriteC(uint8_t);
size_t OLEDwriteCCC(const uint8_t *buffer, size_t size);
size_t OLEDwriteCC(const char *str);


void OLEDsend(uint8_t, uint8_t);
void OLEDwrite4bits(uint8_t);
void OLEDpulseEnable(void);
void OLEDwaitForReady(void);
void OLEDpinMode(uint8_t pin, uint8_t mode);



uint8_t _oled_ver; // OLED_V1 = older, OLED_V2 = newer hardware version.
//uint8_t rs_pin; // LOW: command.  HIGH: character.
//uint8_t _rw_pin; // LOW: write to LCD.  HIGH: read from LCD.
//uint8_t _enable_pin; // activated by a HIGH pulse.
//uint8_t _busy_pin;   // HIGH means not ready for next command
uint8_t _data_pins[4];

uint8_t _displayfunction;
uint8_t _displaycontrol;
uint8_t _displaymode;
uint8_t _initialized;
uint8_t _currline;
uint8_t _numlines;




#endif /* CHARACTEROLED_H_ */
