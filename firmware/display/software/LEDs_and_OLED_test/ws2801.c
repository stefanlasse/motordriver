#ifndef _WS2801_
#define _WS2801_
//// Configuration
// Port to where WS2801 is connected (e.g PORTA, PORTC, etc...)
#ifndef WS2801_OUT_PORT
#error "WS2801_OUT_PORT not defined for <ws2801.c>"
#endif

// DDR of port to where WS2801 is connected (e.g DDRA, DDRC, etc...)
#ifndef WS2801_OUT_PORT_DDR
#error "WS2801_OUT_PORT_DDR not defined for <ws2801.c>"
#endif

// Pin number to where 'clock' input of WS2801 is connected, digit from 0 to 7
#ifndef WS2801_CLK_PIN
#error "WS2801_CLK_PIN not defined for <ws2801.c>"
#endif

// Pin number to where 'data' input of WS2801 is connected, digit from 0 to 7
#ifndef WS2801_DATA_PIN
#error "WS2801_DATA_PIN not defined for <ws2801.c>"
#endif

// Length of LED strip
#ifndef WS2801_STRIP_LEN
#error "WS2801_STRIP_LEN not defined for <ws2801.c>"
#endif

// Length of buffer
#define _WS2801_BUF_LEN WS2801_STRIP_LEN*3

// Delay for ws2801 buffer latching in usecs
// (delay after burst of data sent to strip, after which LED brightness changes)
#define _WS2801_LATCH_DELAY 2000

// Length in microseconds (_delay_us) of a single bit send period
#define _WS2801_BIT_PERIOD 20
#define _WS2801_BIT_DELAY (_WS2801_BIT_PERIOD/2)

// Length of a reset period in milliseconds (_delay_ms)
// After that time with 'clk' pin at 'low' WS2801 will latch it's buffer, and
// will be ready for new data bursts.
// Default datasheet value is 500 us.
#define _WS2801_RESET_PERIOD 1000

#include <util/delay.h>

void ws2801_init()
{
    // Initialize port outputs
    WS2801_OUT_PORT_DDR |= (1 << WS2801_CLK_PIN) | (1 << WS2801_DATA_PIN);
    WS2801_OUT_PORT &= ~((1 << WS2801_DATA_PIN) | (1 << WS2801_CLK_PIN));
}

// Send a single byte to the LED controller chain
// This function automatically outputs it MSB-first
void _ws2801_send_byte(uint8_t byte)
{
    uint8_t outbyte;
	//for(uint8_t i=0; i<8; i++)
	for(uint8_t i=7; i<255; i--)
    //for(uint8_t i=7; i>=7; i--)
    {
        outbyte = (byte>>i) & 0x01;

        // Set the data pin to given value, and clk pin to 0
        if(outbyte)
        {
            WS2801_OUT_PORT |= (outbyte << WS2801_DATA_PIN);
        }
        else
        {
            WS2801_OUT_PORT &= ~(outbyte << WS2801_DATA_PIN);
        }
        WS2801_OUT_PORT &= ~(1 << WS2801_CLK_PIN);
        _delay_us(_WS2801_BIT_DELAY);

        // Keep the data pin, and set clk pin to 1 (strobe)
        WS2801_OUT_PORT |= 1 << WS2801_CLK_PIN;
        _delay_us(_WS2801_BIT_DELAY);

        // Zero both clk and data pins
        WS2801_OUT_PORT &= ~((1 << WS2801_DATA_PIN) | (1 << WS2801_CLK_PIN));
    }
}

// Current global color settings
volatile uint8_t _ws2801_current_red = 0;
volatile uint8_t _ws2801_current_green = 0;
volatile uint8_t _ws2801_current_blue = 0;

void ws2801_set_red(uint8_t amount)
{
    _ws2801_current_red = amount;
}
void ws2801_set_green(uint8_t amount)
{
    _ws2801_current_green = amount;
}
void ws2801_set_blue(uint8_t amount)
{
    _ws2801_current_blue = amount;
}
void ws2801_set_brightness(uint8_t amount)
{
    _ws2801_current_red = amount;
    _ws2801_current_green = amount;
    _ws2801_current_blue = amount;
}

void ws2801_adj_red(int8_t amount)
{
    _ws2801_current_red += amount;
}
void ws2801_adj_green(int8_t amount)
{
    _ws2801_current_green += amount;
}
void ws2801_adj_blue(int8_t amount)
{
    _ws2801_current_blue += amount;
}
void ws2801_adj_brightness(int8_t amount)
{
    _ws2801_current_red += amount;
    _ws2801_current_green += amount;
    _ws2801_current_blue += amount;
}

// Fill strip with values from current
void ws2801_draw_plain()
{
    for(uint16_t i=0; i < WS2801_STRIP_LEN; i++)
    {
        _ws2801_send_byte(_ws2801_current_red);
        _ws2801_send_byte(_ws2801_current_green);
        _ws2801_send_byte(_ws2801_current_blue);
    }
    
    // Wait for data flush
    _delay_us(_WS2801_LATCH_DELAY);
}

// Draw whole led-strip
void ws2801_draw_buffer(uint8_t * buffer, uint16_t length)
{
    for(uint16_t i=0; i < length; i++)
    {
        _ws2801_send_byte(buffer[i]);
    }

    // Wait for data flush
    _delay_us(_WS2801_LATCH_DELAY);
}

// Zero all leds
void ws2801_reset()
{
    ws2801_adj_brightness(0);
    ws2801_draw_plain();
}
#endif