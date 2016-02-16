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

#include <./ws2801.c>


int main(void) {

  // -------- Inits --------- //

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
	
	_delay_ms(2000);

  // ------ Event loop ------ //
  while (1) {

	//for(uint16_t i=0; i < 1; i++)
    //{
		//S4
		_ws2801_send_byte(0); //R 1
        _ws2801_send_byte(3); //G
        _ws2801_send_byte(3); //B
		
	    _ws2801_send_byte(0); //R 2
        _ws2801_send_byte(3); //G
        _ws2801_send_byte(3); //B
		//S5
        _ws2801_send_byte(3); //R 3
        _ws2801_send_byte(0); //G
        _ws2801_send_byte(3); //B
	
	    _ws2801_send_byte(3); //R 4
        _ws2801_send_byte(0); //G
        _ws2801_send_byte(3); //B
		//dummy
	    _ws2801_send_byte(0); //R 5
        _ws2801_send_byte(0); //G
        _ws2801_send_byte(0); //B
		
	    _ws2801_send_byte(0); //R 6
        _ws2801_send_byte(0); //G
        _ws2801_send_byte(0); //B
		//S1
	    _ws2801_send_byte(5); //R 7
        _ws2801_send_byte(0); //G
        _ws2801_send_byte(0); //B
		
	    _ws2801_send_byte(5); //R 8
        _ws2801_send_byte(0); //G
        _ws2801_send_byte(0); //B
		//S2
	    _ws2801_send_byte(0); //R 9
        _ws2801_send_byte(5); //G
        _ws2801_send_byte(0); //B
		
	    _ws2801_send_byte(0); //R 10
        _ws2801_send_byte(5); //G
        _ws2801_send_byte(0); //B
		//S3
	    _ws2801_send_byte(0); //R 11
        _ws2801_send_byte(0); //G
        _ws2801_send_byte(5); //B
		
	    _ws2801_send_byte(0); //R 12
        _ws2801_send_byte(0); //G
        _ws2801_send_byte(5); //B

    //}
    
    // Wait for data flush
    _delay_us(_WS2801_LATCH_DELAY);
	
	_delay_ms(1000);

  }
  return (0);
}
