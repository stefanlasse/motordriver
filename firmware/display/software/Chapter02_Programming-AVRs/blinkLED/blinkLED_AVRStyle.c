                                                    /* Blinker Demo II */

#include <avr/io.h>
#include <util/delay.h>

#define LEDR     PD4
#define LEDG     PD5
#define LEDB     PD6
#define LED_DDR  DDRD
#define LED_PORT PORTD

#define DELAYTIME 1000

#define setBit(sfr, bit)     (_SFR_BYTE(sfr) |= (1 << bit))
#define clearBit(sfr, bit)   (_SFR_BYTE(sfr) &= ~(1 << bit))
#define toggleBit(sfr, bit)  (_SFR_BYTE(sfr) ^= (1 << bit))

int main(void) {

  // Init
  setBit(LED_DDR, LEDR);                     /* set LED pin for output */
  setBit(LED_DDR, LEDG);
  setBit(LED_DDR, LEDB);
  
  setBit(LED_PORT, LEDR);
  setBit(LED_PORT, LEDG);
  setBit(LED_PORT, LEDB);

  // Mainloop
  while (1) {

	//turn blue off and red on
	setBit(LED_PORT, LEDB);
	clearBit(LED_PORT, LEDR);
    _delay_ms(DELAYTIME);

    //turn red off and green on
	setBit(LED_PORT, LEDR);
	clearBit(LED_PORT, LEDG);
    _delay_ms(DELAYTIME);
	
	//turn green off and blue on
	setBit(LED_PORT, LEDG);
	clearBit(LED_PORT, LEDB);
    _delay_ms(DELAYTIME);
	
  }
  return (0);                                          /* end mainloop */
}
