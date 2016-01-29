#define F_CPU 20000000 /* in Hz */

#include <avr/io.h>
#include <util/delay.h>

// Make sure that F_CPU is defined at this point
// Port where WS2801 is connected to (e.g PORTA, PORTC, etc...)
#define WS2801_OUT_PORT PORTC
#define WS2801_OUT_PORT_DDR DDRC

// Pin number where 'clock' input of WS2801 is connected to, digit from 0 to 7
#define WS2801_CLK_PIN PC7

// Pin number where 'data' input of WS2801 is connected to, digit from 0 to 7
#define WS2801_DATA_PIN PC6

// Length of LED strip (in LEDs)
#define WS2801_STRIP_LEN 12

#include "characterOLED.c"
#include "ws2801.c"

void display_something_on_leds()
{
	//S4
	for(uint8_t i=0; i < 2; i++)
	{
		_ws2801_send_byte(0); //R
		_ws2801_send_byte(15); //G
		_ws2801_send_byte(0); //B
	}

	//S5
	for(uint8_t i=0; i < 2; i++)
	{
		_ws2801_send_byte(15); //R
		_ws2801_send_byte(0); //G
		_ws2801_send_byte(0); //B
	}
	//dummy
	for(uint8_t i=0; i < 2; i++)
	{
		_ws2801_send_byte(0); //R
		_ws2801_send_byte(0); //G
		_ws2801_send_byte(0); //B
	}
	//S1
	for(uint8_t i=0; i < 2; i++)
	{
		_ws2801_send_byte(0); //R
		_ws2801_send_byte(15); //G
		_ws2801_send_byte(0); //B
	}
	//S2
	for(uint8_t i=0; i < 2; i++)
	{
		_ws2801_send_byte(0); //R
		_ws2801_send_byte(15); //G
		_ws2801_send_byte(0); //B
	}
	//S3
	for(uint8_t i=0; i < 2; i++)
	{
		_ws2801_send_byte(0); //R
		_ws2801_send_byte(15); //G
		_ws2801_send_byte(0); //B
	}
	
    // Wait for data flush
    _delay_us(_WS2801_LATCH_DELAY);
}


//main function
characterOLED lcd(OLED_V2);

void setup()
{
	// Print a message to the LCD.
	lcd.begin(16, 2);
	lcd.print("LK-Instruments");
}


int main(void)
{
	
    //ws2801_init();
	// Adjust port registers (change to your port here)
    DDRC = (1 << WS2801_CLK_PIN) | (1 << WS2801_DATA_PIN);
    PORTC = (1 << WS2801_CLK_PIN) | (1 << WS2801_DATA_PIN);

    // Zeroing the strip
    ws2801_reset();

		_ws2801_send_byte(0); //R
        _ws2801_send_byte(0); //G
        _ws2801_send_byte(0); //B

	// Wait for data flush
    _delay_us(_WS2801_LATCH_DELAY);
	
	//OLED init
	_delay_ms(2000); //Startup
	cli();
	setup();
	sei();
	
	uint8_t i = 0;
	char str[16];

	while(1)
	{
		
		display_something_on_leds();
		
		// set the cursor to column 0, line 1
		// (note: line 1 is the second row, since counting begins with 0):
		lcd.clear();
		lcd.setCursor(0, 0);
		lcd.print("LK-Instruments");
		lcd.setCursor(0, 1);
		lcd.print("SMC4242");		
		_delay_ms(1000);
		
	}
	return (0);
}