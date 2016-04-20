

#include "libiic.h"

/* =====================================================================
    I2C subsystem

  code to communicate with I2C devices @ 400 kHz SCL

  For F_CPU = 20 MHz:
  TWPS = 0   (always)
  TWBR = 34  (400 kHz SCL)
  TWBR = 184 (100 kHz SCL)

====================================================================== */

/* ---------------------------------------------------------------------
   initialize I2C system
 --------------------------------------------------------------------- */
void initIIC(void){

  TWBR = TWBR_VALUE;    /* see macro in libiic.h */
  TWSR &= ~((1<<TWPS0) | (1<<TWPS1));
  TWCR = (1<<TWEN);

  return;
}

/* ---------------------------------------------------------------------
   send IIC start
 --------------------------------------------------------------------- */
void IICstart(void){

  TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
  while(!(TWCR & (1<<TWINT)));

  return;
}

/* ---------------------------------------------------------------------
   send IIC stop
 --------------------------------------------------------------------- */
void IICstop(void){

  TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEN);

  return;
}

/* ---------------------------------------------------------------------
   write a single byte to the bus
 --------------------------------------------------------------------- */
void IICsendByte(uint8_t data){

  TWDR = data;
  TWCR = (1<<TWINT) | (1<<TWEN);
  while(!(TWCR & (1<<TWINT)));

  return;
}

/* ---------------------------------------------------------------------
   read a single byte with ACK
 --------------------------------------------------------------------- */
uint8_t IICreadACK(void){

  TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
  while(!(TWCR & (1<<TWINT)));

  return TWDR;
}

/* ---------------------------------------------------------------------
   read a single byte with NACK
 --------------------------------------------------------------------- */
uint8_t IICreadNACK(void){

  TWCR = (1<<TWINT) | (1<<TWEN);
  while (!(TWCR & (1<<TWINT)));

  return TWDR;
}

/* ---------------------------------------------------------------------
   get IIC status register
 --------------------------------------------------------------------- */
uint8_t IICgetStatus(void){

  uint8_t status = 0;

  status = TWSR & 0xF8;

  return status;
}

/* ---------------------------------------------------------------------
   send a number of bytes to I2C slave
 --------------------------------------------------------------------- */
void IICwrite(uint8_t addr, uint8_t* data, uint8_t numDat){

  uint8_t i = 0;

  IICstart();
  if(IICgetStatus() !=  TW_START){
    /* error handling */
  }

  IICsendByte(addr | TW_WRITE);
  if(IICgetStatus() !=  TW_MT_SLA_ACK){
    /* error handling */
  }

  for(i = 0; i < numDat; i++){
    IICsendByte(*data);
    if(IICgetStatus() != TW_MT_DATA_ACK){
      /* error handling */
    }
    data++;
  }

  IICstop();

  return;
}

/* ---------------------------------------------------------------------
   read a number of bytes from an I2C slave
 --------------------------------------------------------------------- */
void IICread(uint8_t addr, uint8_t* data, uint8_t numDat){

  uint8_t i = 0;

  IICstart();
  if(IICgetStatus() != TW_START){
    /* error handling */
  }

  IICsendByte(addr | TW_READ);
  if(IICgetStatus() !=  TW_MR_SLA_ACK){
    /* error handling */
  }

  for(i=0; i < numDat - 1; i++){
    *data = IICreadACK();
    if(IICgetStatus() != TW_MR_DATA_ACK){
      /* error handling */
    }
    data++;
  }

  *data = IICreadNACK();
  if(IICgetStatus() !=  TW_MR_DATA_NACK){
    /* error handling */
  }

  IICstop();

  return;
}
