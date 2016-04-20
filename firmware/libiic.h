
#ifndef LIBIIC_H_
#define LIBIIC_H_

#ifndef F_CPU
#define F_CPU 20000000
#endif

#define SCL_FREQ 400000
#define TWBR_VALUE (((F_CPU/SCL_FREQ) - 16)/2)

#include <avr/interrupt.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include <avr/io.h>
#include <util/twi.h>


void initIIC(void);
void IICstart(void);
void IICstop(void);
void IICsendByte(uint8_t data);
uint8_t IICreadACK(void);
uint8_t IICreadNACK(void);
uint8_t IICgetStatus(void);
void IICwrite(uint8_t addr, uint8_t* data, uint8_t numDat);
void IICread(uint8_t addr, uint8_t* data, uint8_t numDat);


#endif /* LIBIIC_H_ */
