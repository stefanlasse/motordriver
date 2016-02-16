/* =====================================================================

    Firmware for Motordriver

    3. Physikalisches Institut, University of Stuttgart

 ===================================================================== */

/*
 *   Motordriver: 3. PI, Uni Stuttgart
 *   Copyright (C) 2014, 2015, 2016  Stefan Lasse
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#define F_CPU 20000000UL /* in Hz */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include "lcd-routines.h"

/* ---------------------------------------------------------------------
    some global definitions
 --------------------------------------------------------------------- */
#define FW_VERSION (" 1.4")

#define IDN_STRING_LENGTH 20
#define SERIAL_BUFFERSIZE 64            /* should be enough */
#define NUMBER_OF_PARAMETERS 10         /* amount of parameters to be kept */
#define PARAMETER_LENGTH 20
#define ALLOWED_CMD_DELIMITERS " ,;\t"  /* for cmd/parameter separation */

/* definitions for USART RS232 */
#define BAUDRATE 57600
#define UBRR_VALUE ((F_CPU+BAUDRATE*8)/(BAUDRATE*16)-1)
#define BAUD_REAL (F_CPU/(16*(UBRR_VALUE+1)))
#define BAUD_ERROR ((BAUD_REAL*1000)/BAUDRATE)
/*
#if ((BAUD_ERROR < 990)||(BAUD_ERROR >1010))
    #error "USART error: systematic BAUD error > 1%. This is too high."
#endif
*/

/* definitions for the motor */
#define ON  1
#define OFF 0

#define NO_MOTOR            0
#define MOTOR0              0 /* set as places to shift */
#define MOTOR1              1
#define MOTOR2              2
#define MOTOR3              3
#define DUMMY_MOTOR         4
#define MAX_MOTOR           MOTOR3

#define MOTOR_SENS0         PA0
#define MOTOR_SENS1         PA1
#define MOTOR_SENS2         PA2
#define MOTOR_SENS3         PA3
#define MOTOR_SENS_MAX      MOTOR_SENS3

/* define the button pinout on AVR */
#define NO_BUTTON           42  /* not allowed to be in intervall [0..7] */
#define BUTTON_MOTOR0       PB7
#define BUTTON_MOTOR1       PB6
#define BUTTON_MOTOR2       PB5
#define BUTTON_MOTOR3       PB4
#define BUTTON_MENUESCAPE   PB3
#define BUTTON_ROT_ENC      PB0

/* do not change ALL_BUTTONS */
#define ALL_BUTTONS ((1<<BUTTON_MOTOR0)|     \
                     (1<<BUTTON_MOTOR1)|     \
                     (1<<BUTTON_MOTOR2)|     \
                     (1<<BUTTON_MOTOR3)|     \
                     (1<<BUTTON_MENUESCAPE)| \
                     (1<<BUTTON_ROT_ENC))

/* ---------------------------------------------------------------------
    compiling options
 --------------------------------------------------------------------- */
#define SAVE_INTERNAL_PROGRAM_TO_EEPROM           1


/* ---------------------------------------------------------------------
    data structures
 --------------------------------------------------------------------- */

/* for status */
typedef struct{

  uint8_t status;
  uint8_t inRemoteMode;

}statusVariables;

/* for data send and receive */
typedef struct{

  char *buffer;
  uint8_t readyToProcess;
  uint8_t charCount;

}serialString;

/* for motor information */
#define MOTOR_STEP_UNIT_STEP      0
#define MOTOR_STEP_UNIT_DEGREE    1
#define MOTOR_STEP_UNIT_RADIAN    2

#define MOTOR_MOVE_INFINITE_STOP  0
#define MOTOR_MOVE_INFINITE_CW    1
#define MOTOR_MOVE_INFINITE_CCW   2

typedef struct{

  int32_t  actualPosition;         /* always in steps */
  int32_t  desiredPosition;        /* always in steps */
  int16_t  opticalZeroPosition;    /* as offset from zero position in steps */
  double   stepError;
  uint8_t  isMoving;
  uint8_t  isTurnedOn;
  uint8_t  isMovingInfinite;
  double   gearRatio;              /* initially set to 60:18 */
  double   stepsPerFullRotation;   /* initially set to 400 */
  double   subSteps;               /* could be 1, 2, 4, 8, 16 */
  int8_t   stepUnit;               /* could be: step, degree, radian */
  double   stepMultiplier;         /* multiplies the default step just at manual operation */
  uint16_t waitBetweenSteps;       /* in milliseconds */
  uint16_t delayCounter;           /* counts the waited milliseconds */
  int8_t   angularVelocity;        /* in seconds per full rotation */

}motorInfo;

/* for analog measurements */
typedef struct{

  uint16_t ADCvalue;
  uint8_t  numberOfMeasurements;

}analog;

/* for button recognition */
typedef struct{

  uint8_t inputRegister;
  uint8_t inDebouncingMode;
  uint8_t readyToProcess;

}button;

/* for rotary encoder */
#define NO_MOVE     0
#define CW_MOVE     1
#define CCW_MOVE   -1

#define PHASE_A  (PINB & 1<<PB1)
#define PHASE_B  (PINB & 1<<PB2)
#define DYNAMICS 40

const int16_t table[16] PROGMEM = {0,0,-1,0,0,0,0,1,1,0,0,0,0,-1,0,0};

typedef struct{

  int8_t direction;
  int8_t steps;
  uint8_t readyToProcess;

}rotaryEncoder;

typedef struct{

  int16_t start;    /* in steps */
  int16_t stop;     /* in steps */
  uint8_t active;

}zone;

#define MAX_PROGRAM_STEPS 16

#define PROG_ABSOLUTE_MOVEMENT 0
#define PROG_RELATIVE_MOVEMENT 1

typedef struct{

  uint8_t isActive;
  int16_t position[MAX_MOTOR + 1];
  uint8_t absRel;

}progStep;


/* ---------------------------------------------------------------------
    command parser stuff
 --------------------------------------------------------------------- */
/* to hold one command */
typedef struct{

  char *cmd;
  uint8_t numberOfOptions;
  uint8_t cmdCode;

}command;

/* to hold command parameters */
char **commandParam = NULL;

/* generate the structs that hold the commands */
#define ADD_COMMAND(NUMBER, CMD_NAME, NUMBER_OPT, CMD_CODE)                      \
  const char cmd_ ## NUMBER ## _name[] PROGMEM = CMD_NAME;                       \
  const command cmd_ ## NUMBER ## _ PROGMEM = { .cmd = cmd_ ## NUMBER ## _name,  \
                                                .numberOfOptions = NUMBER_OPT,   \
                                                .cmdCode = CMD_CODE              \
                                              };

/*
  ADD_COMMAND works as follows:
  1st arg: ongoing decimal number
  2nd arg: command name ended with NULL-termination
  3rd arg: number of parameters needed for this command
  4th arg: command code, ongoing in hex >= 0x80, handled by the main-loop

  Then add new defined command to the commandList array.

  Increment TOTAL_NUMBER_OF_COMMANDS.

  Implement what to do with the new command in the main loop and add the
  necessary functions.
*/

ADD_COMMAND(0,  "\0" ,              0, 0x80)  /* no command*/
ADD_COMMAND(1,  "*RST\0",           0, 0x81)  /* reset */
ADD_COMMAND(2,  "*IDN?\0",          0, 0x82)  /* get IDN */
ADD_COMMAND(3,  "*IDN\0",           1, 0x83)  /* set IDN */
ADD_COMMAND(4,  "MOVEABS\0",        3, 0x84)  /* move to absolute position with reference to zero position */
ADD_COMMAND(5,  "MOVEREL\0",        3, 0x85)  /* move relative to actual position */
ADD_COMMAND(6,  "ZERORUN\0",        1, 0x86)  /* calibrate motor position */
ADD_COMMAND(7,  "ENABLE\0",         2, 0x87)  /* turn motor on/off */
ADD_COMMAND(8,  "GETPOS\0",         2, 0x88)  /* get position of motor in [unit] */
ADD_COMMAND(9,  "SAVECONF\0",       0, 0x89)  /* save current machine configuration */
ADD_COMMAND(10, "LOADCONF\0",       0, 0x8A)  /* load last saved machine configuration */
ADD_COMMAND(11, "ISMOVING\0",       1, 0x8B)  /* check if motor is moving */
ADD_COMMAND(12, "GETANALOG\0",      1, 0x8C)  /* returns an ADC measurement */

ADD_COMMAND(13, "GETOPTZEROPOS\0",  1, 0x8D)  /* returns saved optical zero position */
ADD_COMMAND(14, "SETOPTZEROPOS\0",  2, 0x8E)  /* sets the optical zero position in steps */
ADD_COMMAND(15, "GETGEARRATIO\0",   1, 0x8F)  /* returns the mechanical gear ratio */
ADD_COMMAND(16, "SETGEARRATIO\0",   2, 0x90)  /* set mechanical gear ratio for a motor */
ADD_COMMAND(17, "GETFULLROT\0",     1, 0x91)  /* returns steps per full rotation */
ADD_COMMAND(18, "SETFULLROT\0",     2, 0x92)  /* sets steps per full rotation */
ADD_COMMAND(19, "GETSUBSTEPS\0",    1, 0x93)  /* returns the adjusted substeps  */
ADD_COMMAND(20, "SETSUBSTEPS\0",    2, 0x94)  /* sets the substeps for a motor */
ADD_COMMAND(21, "GETWAITTIME\0",    1, 0x95)  /* returns the wait time between two single steps */
ADD_COMMAND(22, "SETWAITTIME\0",    2, 0x96)  /* sets the wait time between two single steps  */
ADD_COMMAND(23, "SETCONSTSPEED\0",  3, 0x97)  /* sets a constant angular velocity */
ADD_COMMAND(24, "FACTORYRESET\0",   0, 0x98)  /* factory reset */

ADD_COMMAND(25, "STOPALL\0",        0, 0x99)  /* stops all movements */
ADD_COMMAND(26, "SETFORBZONE\0",    3, 0x9A)  /* defines a forbidden zone */
ADD_COMMAND(27, "ENABFORBZONE\0",   2, 0x9B)  /* enables/disables the forbidden zone */
ADD_COMMAND(28, "SETPROGSTEP\0",    6, 0x9C)  /* define a program step for manual operation */
ADD_COMMAND(29, "GETMOTSTATE\0",    1, 0x9D)  /* define a program step for manual operation */
ADD_COMMAND(30, "DBGREADOUT\0",     0, 0x9E)  /* DEBUG information GPIO bla bla */


#define TOTAL_NUMBER_OF_COMMANDS 31

const command* const commandList[] PROGMEM = {&cmd_0_,  &cmd_1_,  &cmd_2_,
                                              &cmd_3_,  &cmd_4_,  &cmd_5_,
                                              &cmd_6_,  &cmd_7_,  &cmd_8_,
                                              &cmd_9_,  &cmd_10_, &cmd_11_,
                                              &cmd_12_, &cmd_13_, &cmd_14_,
                                              &cmd_15_, &cmd_16_, &cmd_17_,
                                              &cmd_18_, &cmd_19_, &cmd_20_,
                                              &cmd_21_, &cmd_22_, &cmd_23_,
                                              &cmd_24_, &cmd_25_, &cmd_26_,
                                              &cmd_27_, &cmd_28_, &cmd_29_,
                                              &cmd_30_
                                             };

/* ---------------------------------------------------------------------
    Display menu:
    is implemented as a state machine.
 --------------------------------------------------------------------- */

/* to hold a menu entry */
typedef struct{

  char *displayText;
  uint8_t state;

}menuItem;


/* generate the structs that hold the display texts */
#define ADD_DISPLAY_TEXT(NUMBER, DISP_TEXT)                       \
  const char disp_ ## NUMBER ## _text[] PROGMEM = DISP_TEXT;                                \
  const menuItem disp_ ## NUMBER ## _ PROGMEM = { .displayText = disp_ ## NUMBER ## _text,  \
                                                  .state = NUMBER                           \
                                                };

char *displayBuffer;   /* hold the display contents, initialized in main() */
#define DISPLAY_BUFFER_SIZE 40

/*
  ADD_DISPLAY_TEXT works as follows:

  use the ADD_DISPLAY_TEXT macro:
  1st arg: ongoing number
  2nd arg: menu item text ended with (!) NULL-termination

  Increment the NUMBER_OF_DISPLAY_MENUS macro, when added a new menu item!

  Then add new defined display text to the menuList array.

  add the contents to be changed in the display subsystem
*/


/* NOTE: '\n' will identify a line break on the display */
ADD_DISPLAY_TEXT(0 , "Motor Driver\n \0"               )
ADD_DISPLAY_TEXT(1 , "Change motor\nposition\0"        )
ADD_DISPLAY_TEXT(2 , "Change step\nunit\0"             )
ADD_DISPLAY_TEXT(3 , "Change step\nwait time\0"        )
ADD_DISPLAY_TEXT(4 , "Set step\nmultiplier\0"          )
ADD_DISPLAY_TEXT(5 , "Run zero\ncalibration\0"         )
ADD_DISPLAY_TEXT(6 , "Change motor\nsubstep\0"         )
ADD_DISPLAY_TEXT(7 , "Save current\nconfiguration\0"   )
ADD_DISPLAY_TEXT(8 , "Load last\nconfiguration\0"      )
ADD_DISPLAY_TEXT(9 , "Define optical\nzero position\0" )
ADD_DISPLAY_TEXT(10, "Set constant\nangular speed\0"   )
ADD_DISPLAY_TEXT(11, "Run internal\nprogram\0"         )

#define NUMBER_OF_DISPLAY_MENUS 12

#define MENU_MAIN                   0
#define MENU_CHANGE_POSITION        1
#define MENU_CHANGE_STEP_UNIT       2
#define MENU_CHANGE_WAIT_TIME       3
#define MENU_SET_STEP_MULTIPL       4
#define MENU_RUN_ZERO_CALIBRATION   5
#define MENU_CHANGE_SUBSTEPS        6
#define MENU_SAVE_CONFIG            7
#define MENU_LOAD_CONFIG            8
#define MENU_OPTICAL_ZERO_POS       9
#define MENU_CONST_ANGULAR_SPEED    10
#define MENU_RUN_PROGRAM            11


/* to hold a list of menu entries */
const menuItem* const menuList[] PROGMEM = {&disp_0_,  &disp_1_,  &disp_2_,
                                            &disp_3_,  &disp_4_,  &disp_5_,
                                            &disp_6_,  &disp_7_,  &disp_8_,
                                            &disp_9_,  &disp_10_, &disp_11_
                                           };

/* to keep information where we are in the menu */
#define MENU_CHANGE_MODE    1 /* change to value view */
#define MENU_SCROLL_MODE    2 /* scroll through the menu */
#define MENU_VALUE_CHANGE   3 /* change a selected value */
#define MENU_FAST_MOVE_MODE 4 /* moves a motor 10 times faster */

#define NUMBER_DISPLAY_MENU_STRINGS   2
#define DISPLAY_MENU_STRING_LENGTH    17
#define NUMBER_DISPLAY_VALUE_STRINGS  4
#define DISPLAY_VALUE_STRING_LENGTH   9


typedef struct{

  uint8_t newDisplayedMenu;
  uint8_t currentDisplayedMenu;
  uint8_t newMenuMode;
  uint8_t fastMovingMode;
  int8_t  currentProgramStep;
  uint8_t currentMenuMode;
  uint8_t selectedMotor;          /* keep the selected motor for value changing */
  char**  currentDisplayValue;    /* for values to be changed */
  char**  newDisplayValue;        /* for values to be changed */
  char**  currentMenuText;        /* for menu texts */
  char**  newMenuText;            /* for menu texts */

}menuInfo;


/* ---------------------------------------------------------------------
    global variables
 --------------------------------------------------------------------- */

/* static firmware version */
static const char firmwareVersion[] = FW_VERSION;

volatile motorInfo motor[MAX_MOTOR+1];  /* we got 4 motors [0..3] */
volatile zone forbiddenZone[MAX_MOTOR+1];
volatile serialString rxString;         /* for a received command */
volatile serialString txString;         /* for information to send */
volatile serialString commandString;
volatile statusVariables status;        /* status variables */
volatile analog adc;                    /* ADC information */
volatile menuInfo menu;                 /* information about the menu */
volatile button buttonState;            /* information on the user interface */
volatile rotaryEncoder rotEnc;
volatile progStep programList[MAX_PROGRAM_STEPS];  /* get memory for internal program */


/* ---------------------------------------------------------------------
    EEPROM memory
 --------------------------------------------------------------------- */

/* variable IDN text */
char EEMEM IDNtext[IDN_STRING_LENGTH + 1];

/* keep motor information in EEPROM */
int16_t  EEMEM opticalZeroPositionEE[MAX_MOTOR+1];
double   EEMEM gearRatioEE[MAX_MOTOR+1];
double   EEMEM stepsPerFullRotationEE[MAX_MOTOR+1];
double   EEMEM subStepsEE[MAX_MOTOR+1];
double   EEMEM stepMultiplierEE[MAX_MOTOR+1];
uint8_t  EEMEM stepUnitEE[MAX_MOTOR+1];
uint16_t EEMEM waitBetweenStepsEE[MAX_MOTOR+1];
uint16_t EEMEM forbiddenZoneStartEE[MAX_MOTOR+1];
uint16_t EEMEM forbiddenZoneStopEE[MAX_MOTOR+1];
progStep EEMEM programListEE[MAX_PROGRAM_STEPS];


/* ---------------------------------------------------------------------
    function prototypes
 --------------------------------------------------------------------- */
/* initializer */
void initDataStructs(void);
void initUSART(void);
void initADC(void);
void initMotorDelayTimer(void);

/* functionality */
void sendChar(char c);
void sendText(char *c);
void prepareReset();
void setMotorState(uint8_t motor, uint8_t status);
void moveMotorRelative(uint8_t mot, int16_t steps);
void defineOpticalZeroPosition(uint8_t i, int8_t step);
void motorZeroRun(uint8_t i);
uint16_t getADCvalue(uint8_t sensPin);
void saveConfigToEEPROM(void);
void loadConfigFromEEPROM(void);

void lcd_init(void);
static void lcd_enable(void);
static void lcd_out(uint8_t data);
void lcd_data(uint8_t data);
void lcd_command(uint8_t data);
void lcd_clear(void);
void lcd_home(void);
void lcd_setcursor(uint8_t x, uint8_t y);
void lcd_string(const char *data);
void lcd_generatechar(uint8_t code, const uint8_t *data);

void degreeToSteps(uint8_t mot, double degree, double multiply);
double stepsToDegree(uint8_t mot, int16_t steps);
void radiansToSteps(uint8_t mot, double rad, double multiply);
double stepsToRadian(uint8_t mot, int16_t steps);
void setConstSpeed(uint8_t i, uint8_t state);

void updateDisplay(void);
void changeDisplayMenu(uint8_t i);
void updateDisplayChangeValues(uint8_t thisMenu);
void updateMenu(void);

void initManualOperatingButtons(void);
uint8_t getButtonEvent(void);
int8_t getRotaryEncoderEvent(void);

uint8_t parseCommand(void);
void  commandMoveAbs(char* param0, char* param1, char* param2);
void  commandMoveRel(char* param0, char* param1, char* param2);
void  commandEnable(char* param0, char* param1);
char* commandGetMotorPosition(char* param0, char* param1);
char* commandIsMoving(char* param0);
char* commandGetAnalog(char* param0);
char* commandGetOptZeroPos(char* param0);
void  commandSetOptZeroPos(char* param0, char* param1);
char* commandGetGearRatio(char* param0);
void  commandSetGearRatio(char* param0, char* param1);
char* commandGetFullRotation(char* param0);
void  commandSetFullRotation(char* param0, char* param1);
char* commandGetSubSteps(char* param0);
void  commandSetSubSteps(char* param0, char* param1);
char* commandGetWaitTime(char* param0);
void  commandSetWaitTime(char* param0, char* param1);
void  commandSetConstSpeed(char* param0, char* param1, char* param2);
void  commandFactoryReset(void);
void  commandSetForbiddenZone(char* param0, char* param1, char* param2);
void  commandEnableForbiddenZone(char* param0, char* param1);
void  commandSetProgStep(char* param0, char* param1, char* param2,
                         char* param3, char* param4, char* param5);
void commandGetMotorState(char* param0);
void commandDebugReadout(void);


/* =====================================================================
    initialization functions
====================================================================== */

/* ---------------------------------------------------------------------
    initialize the internal data memory
 --------------------------------------------------------------------- */
void initDataStructs(void){

  uint8_t i, j;

  for(i = 0; i <= MAX_MOTOR; i++){
    motor[i].actualPosition       = 0;
    motor[i].desiredPosition      = 0;
    motor[i].opticalZeroPosition  = 0;
    motor[i].stepError            = 0.0f;
    motor[i].isMoving             = 0;
    motor[i].isTurnedOn           = 0;
    motor[i].isMovingInfinite     = MOTOR_MOVE_INFINITE_STOP;
    motor[i].gearRatio            = 60.0f/18.0f; /* TODO */
    motor[i].stepsPerFullRotation = 400.0f;
    motor[i].subSteps             = 4.0f;
    motor[i].stepMultiplier       = 1.0f;
    motor[i].stepUnit             = MOTOR_STEP_UNIT_DEGREE;
    motor[i].waitBetweenSteps     = 3;
    motor[i].delayCounter         = 2*motor[i].waitBetweenSteps-1;
    motor[i].angularVelocity      = OFF;
  }

  rxString.charCount = 0;
  rxString.readyToProcess = 0;

  txString.charCount = 0;
  txString.readyToProcess = 0;

  status.status = 0;
  status.inRemoteMode = 0;

  adc.ADCvalue = 0;
  adc.numberOfMeasurements = 8;

  menu.newDisplayedMenu = MENU_MAIN;
  menu.currentDisplayedMenu = 42;
  menu.newMenuMode = MENU_SCROLL_MODE;
  menu.fastMovingMode = OFF;
  menu.currentProgramStep = 0;
  menu.currentMenuMode = 42;
  menu.selectedMotor = NO_MOTOR|(1<<DUMMY_MOTOR);  /* DUMMY_MOTOR stays always selected */
  /* strings are initialized in main */

  buttonState.inputRegister = 0;
  buttonState.inDebouncingMode = 0;
  buttonState.readyToProcess = 0;

  rotEnc.direction = NO_MOVE;
  rotEnc.steps = 0;
  rotEnc.readyToProcess = 0;

  for(i = 0; i <= MAX_MOTOR; i++){
    forbiddenZone[i].active = 0;
    forbiddenZone[i].start  = 0;
    forbiddenZone[i].stop   = 0;
  }

  /* initialize program list */
  for(i = 0; i < MAX_PROGRAM_STEPS; i++){
    programList[i].isActive = 0;
    programList[i].absRel = PROG_RELATIVE_MOVEMENT;
    for(j = 0; j <= MAX_MOTOR; j++){
      programList[i].position[j] = 0;
    }
  }
  /* define home position on program step 0 */
  programList[0].isActive = 1;

  return;
}

/* ---------------------------------------------------------------------
   inits the RS232 / USART system
 --------------------------------------------------------------------- */
void initUSART(void){

  /* Configuration:
   - baudrate: see macro above
   - asynchronous mode
   - no parity bit
   - 1 stop bit
   - 8 bit character size
   - no flow control
  */

  /* set baud rate registers */
  UBRR0H = (uint8_t)(UBRR_VALUE >> 8);
  UBRR0L = (uint8_t)(UBRR_VALUE & 0xFF);

  UCSR0B |= (1<<TXEN0) | (1<<RXEN0)        /* enable RX and TX */
           |(1<<RXCIE0);                   /* enable RX interrupt */

  return;
}

/* ---------------------------------------------------------------------
   inits the ADC
 --------------------------------------------------------------------- */
void initADC(void){

  ADMUX   = (1<<REFS0);  /* AVcc voltage reference */
  ADCSRA  = (1<<ADEN)|(1<<ADPS1)|(1<<ADPS0);

  /* start one conversion for initializing */
  ADCSRA  |= (1<<ADSC);

  while(ADCSRA & (1<<ADSC)){
    ;
  }

  (void)ADCW;

  return;
}

/* ---------------------------------------------------------------------
   initialize buffers
 --------------------------------------------------------------------- */
void initBuffers(void){

  uint8_t i,j;

  /* command parameters */
  for(j = 0; j < PARAMETER_LENGTH; j++){
    commandParam[i][j] = 0;
  }

  /* buffers for the display */
  for(j = 0; j < DISPLAY_VALUE_STRING_LENGTH; j++){
    menu.currentDisplayValue[i][j] = 0;
    menu.newDisplayValue[i][j]     = 0;
  }

  for(j = 0; j < DISPLAY_MENU_STRING_LENGTH; j++){
    menu.currentMenuText[i][j] = 0;
    menu.newMenuText[i][j]     = 0;
  }

  memset(rxString.buffer, 0, SERIAL_BUFFERSIZE);
  memset(txString.buffer, 0, SERIAL_BUFFERSIZE);
  memset(commandString.buffer, 0, SERIAL_BUFFERSIZE);
  memset(displayBuffer, 0, DISPLAY_BUFFER_SIZE);

  return;
}



/* =====================================================================
    functionality implementation
====================================================================== */

/* ---------------------------------------------------------------------
    sends a character via RS232
 --------------------------------------------------------------------- */
void sendChar(char c){

  while(!(UCSR0A & (1<<UDRE0))){
    ;
  }

  UDR0 = c;

  return;
}

/* ---------------------------------------------------------------------
    sends a terminated string via RS232
 --------------------------------------------------------------------- */
void sendText(char *c){

  while(*c){
    sendChar(*c);
    c++;
  }
  /* send LF after finishing text sending */
  sendChar('\n');

  memset(txString.buffer, 0, SERIAL_BUFFERSIZE);

  return;
}

/* ---------------------------------------------------------------------
    some necessary things to do for a reset
 --------------------------------------------------------------------- */
void prepareReset(){

  /* turn off all motors */
  setMotorState(MOTOR0, OFF);
  setMotorState(MOTOR1, OFF);
  setMotorState(MOTOR2, OFF);
  setMotorState(MOTOR3, OFF);

  /* stop polling timer for manual operating system */
  TCCR0B = 0;

  /* stop motor movement timer */
  TCCR2B = 0;

  return;
}

/* ---------------------------------------------------------------------
    setMotorState: turns motor on/off
 --------------------------------------------------------------------- */
void setMotorState(uint8_t mot, uint8_t state){

  /* set enable bit as output */
  DDRA |= (1 << (mot + 4));

  if(state == ON){
    PORTA &= ~(1 << (mot + 4));   /* delete pull-up */
    motor[mot].isTurnedOn = ON;
    DDRC |= (1 << mot);           /* moving direction pin */
    DDRC |= (1 << (mot + 4));     /* stepping pin */
  }
  else{
    PORTA |= (1 << (mot + 4));
    motor[mot].isTurnedOn = OFF;
    DDRC &= ~(1 << mot);          /* moving direction pin */
    DDRC &= ~(1 << (mot + 4));    /* stepping pin */
  }

  return;
}


/* ---------------------------------------------------------------------
    inits the motor timer for motor[i].waitBetweenSteps
 --------------------------------------------------------------------- */
void initMotorDelayTimer(void){

  /*
   * the 8-bit Timer/Counter2 is used for that
   */

  TCCR2A |= (1<<WGM21);   /* enable CTC */
  OCR2A   = 77;
  TIMSK2 |= (1<<OCIE2A);  /* enable interrupt */
  TCNT2   = 0;

  /* start the timer/counter */
  TCCR2B |= (1<<CS22)|(1<<CS20);  /* prescaler = 1024 --> 51.2 us per clock */

  return;
}


/* ---------------------------------------------------------------------
    moveMotorBySteps: move motor <steps> steps forward or backward
    This is a relative movement to the actual position.
    NOTE: this function is only used by motorZeroRun()
 --------------------------------------------------------------------- */
void moveMotorRelative(uint8_t mot, int16_t steps){

  int16_t  i;
  uint16_t j;

  /* set direction */
  if(steps == 0){
    /* no move */
    return;
  }
  else if(steps < 0){
    /* move CCW */
    PORTC |= (1 << (2*mot + 1));
  }
  else{
    /* move CW */
    PORTC &= ~(1 << (2*mot + 1));
  }

  /* now move the motor */
  for (i = 0; i < abs(steps); i++){
    PORTC |= (1 << (2*mot));
    _delay_us(2); /* as specified in datasheet */
    PORTC &= ~(1 << (2*mot));

    for(j = 0; j < motor[mot].waitBetweenSteps; j++){
      _delay_ms(1);
    }
  }

  return;
}

/* ---------------------------------------------------------------------
   setting the optical zero position
 --------------------------------------------------------------------- */
void defineOpticalZeroPosition(uint8_t i, int8_t step){

  motor[i].desiredPosition = motor[i].actualPosition + step;
  motor[i].opticalZeroPosition = motor[i].desiredPosition;

  return;
}


/* ---------------------------------------------------------------------
    zero run: position calibration for the motor.
    After zero run motor will have the fast axis position of any
    wave plate.
 --------------------------------------------------------------------- */
void motorZeroRun(uint8_t i){

  uint16_t keepWaitTime = 0;
  double stepsPerRound = 0.0f;
  uint16_t thres = 50;  /* threshold for the ADC reading of the Hall sensor */
  uint16_t j = 0;

  if(forbiddenZone[i].active){
    /* zerorun not allowed if forbidden zone is active */
    return;
  }

  /* stop any motor movements (stop timer/counter2) */
  TCCR2B &= ~0x07;

  keepWaitTime = motor[i].waitBetweenSteps;

  stepsPerRound = motor[i].stepsPerFullRotation
                  * motor[i].gearRatio
                  * motor[i].subSteps;

  /* fist step:
   * move 360 degree to find the roughly position of the magnetic zero point.
   * this will be done with fast moving */
  motor[i].waitBetweenSteps = 1;    /* set 1 ms for fast moving */

  /* in case we are at any possible zero position: move out */
  while(getADCvalue(i) < thres){
    moveMotorRelative(i, -200);
  }

  /* start first search for zero point */
  for(j = 0; j < (uint16_t)round(stepsPerRound); j++){
    moveMotorRelative(i, 1);
    if(getADCvalue(i) < thres){
      /* we found a zero position */
      break;
    }
  }

  /* move 90 degree forward */
  moveMotorRelative(i, (int16_t)round(0.25*stepsPerRound));

  /* now we will find our zero position in
   * 3*<stepsPerRound>/4 steps */

  /* second step:
   * move again to 200 steps before the former found zero position fast.
   * then move slowly to find the exact magnetic zero position. */

  /* now move till 200 steps before the zero-position */
  moveMotorRelative(i, (int16_t)round(0.75*stepsPerRound) - 200);

  /* now get slow to find zero position precisely */
  motor[i].waitBetweenSteps = 5;

  /* and move till the threshold is reached */
  while(getADCvalue(i) > thres){
    moveMotorRelative(i, 1);
  }
  /* and here we found our magnetic zero position :-) */

  /* third step:
   * go to the internal saved optical zero position */
  motor[i].waitBetweenSteps = keepWaitTime;
  moveMotorRelative(i, motor[i].opticalZeroPosition);

  /* now set motor into a defined state */
  motor[i].actualPosition = 0;
  motor[i].desiredPosition = 0;

  /* allow motor movements again */
  initMotorDelayTimer();

  return;
}


/* ---------------------------------------------------------------------
    start an ADC conversion
 --------------------------------------------------------------------- */
uint16_t getADCvalue(uint8_t sensPin){

  uint8_t i = 0;
  uint8_t lowByte, highByte;

  /* select channel */
  ADMUX = (ADMUX & ~(0x1F)) | (sensPin & 0x1F);

  /* reset all values and counters before starting new conversion */
  adc.ADCvalue = 0;

  /* start the conversion */
  for(i = 0; i < adc.numberOfMeasurements; i++){
    ADCSRA |= (1<<ADSC);
    while(ADCSRA & (1<<ADSC)){
      ;
    }
    lowByte = ADCL;
    highByte = ADCH;
    adc.ADCvalue += (highByte<<8) | lowByte;
  }

  return (adc.ADCvalue / adc.numberOfMeasurements);
}

/* ---------------------------------------------------------------------
    save current motor configuration to EEPROM
 --------------------------------------------------------------------- */
void saveConfigToEEPROM(void){

  uint8_t i = 0;

  ATOMIC_BLOCK(ATOMIC_FORCEON){
    for(i = 0; i <= MAX_MOTOR; i++){
      eeprom_update_block(&(motor[i].opticalZeroPosition), &(opticalZeroPositionEE[i]), sizeof(int16_t));
      eeprom_update_block(&(motor[i].gearRatio), &(gearRatioEE[i]), sizeof(double));
      eeprom_update_block(&(motor[i].stepsPerFullRotation), &(stepsPerFullRotationEE[i]), sizeof(double));
      eeprom_update_block(&(motor[i].subSteps), &(subStepsEE[i]), sizeof(double));
      eeprom_update_block(&(motor[i].stepMultiplier), &(stepMultiplierEE[i]), sizeof(double));
      eeprom_update_block(&(motor[i].stepUnit), &(stepUnitEE[i]), sizeof(int8_t));
      eeprom_update_block(&(motor[i].waitBetweenSteps), &(waitBetweenStepsEE[i]), sizeof(int16_t));

      eeprom_update_block(&(forbiddenZone[i].start), &(forbiddenZoneStartEE[i]), sizeof(int16_t));
      eeprom_update_block(&(forbiddenZone[i].stop), &(forbiddenZoneStopEE[i]), sizeof(int16_t));
    }

#if SAVE_INTERNAL_PROGRAM_TO_EEPROM
    /* save internal stored programs */
    for(i = 0; i < MAX_PROGRAM_STEPS; i++){
      eeprom_update_block(&(programList[i]), &(programListEE[i]), sizeof(progStep));
    }
#endif

  }

  return;
}

/* ---------------------------------------------------------------------
    load last stored motor configuration from EEPROM
 --------------------------------------------------------------------- */
void loadConfigFromEEPROM(void){

  uint8_t i = 0;

  ATOMIC_BLOCK(ATOMIC_FORCEON){
    for(i = 0; i <= MAX_MOTOR; i++){
      eeprom_read_block(&(motor[i].opticalZeroPosition), &(opticalZeroPositionEE[i]), sizeof(int16_t));
      eeprom_read_block(&(motor[i].gearRatio), &(gearRatioEE[i]), sizeof(double));
      eeprom_read_block(&(motor[i].stepsPerFullRotation), &(stepsPerFullRotationEE[i]), sizeof(double));
      eeprom_read_block(&(motor[i].subSteps), &(subStepsEE[i]), sizeof(double));
      eeprom_read_block(&(motor[i].stepMultiplier), &(stepMultiplierEE[i]), sizeof(double));
      eeprom_read_block(&(motor[i].stepUnit), &(stepUnitEE[i]), sizeof(int8_t));
      eeprom_read_block(&(motor[i].waitBetweenSteps), &(waitBetweenStepsEE[i]), sizeof(int16_t));

      eeprom_read_block(&(forbiddenZone[i].start), &(forbiddenZoneStartEE[i]), sizeof(int16_t));
      eeprom_read_block(&(forbiddenZone[i].stop), &(forbiddenZoneStopEE[i]), sizeof(int16_t));

      /* activate forbidden zone if necessary */
      if(forbiddenZone[i].start != forbiddenZone[i].stop){
        forbiddenZone[i].active = 1;
      }
    }

#if SAVE_INTERNAL_PROGRAM_TO_EEPROM
    /* load internal stores programs */
    for(i = 0; i < MAX_PROGRAM_STEPS; i++){
      eeprom_read_block(&(programList[i]), &(programListEE[i]), sizeof(progStep));
    }
#endif

  }

  return;
}

/* =====================================================================
    Display subsystem

  code to communicate with the LCD display. It is based on a Hitachi
  LCD driver chip HD44780.

  This display here in use is
  - 2 lines with 16 characters each
  - 5x7 dots per character
  - driven in 4-bit-mode
====================================================================== */

/* Interface to a HD44780 compatible LCD in 4-Bit-mode
 * http://www.mikrocontroller.net/articles/HD44780
 * http://www.mikrocontroller.net/articles/AVR-GCC-Tutorial/LCD-Ansteuerung
 *
 * Pinout is defined in lcd-routines.h
 *
 *
 * Connection from AVR to Display:
 *
 * AVR PD2 --> Display RS (reset)
 * AVR PD3 --> Display EN (enable)
 * AVR PD4 --> Display D4
 * AVR PD5 --> Display D5
 * AVR PD6 --> Display D6
 * AVR PD7 --> Display D7
 *
 */

/* ---------------------------------------------------------------------
   produce enable pulse
 --------------------------------------------------------------------- */
static void lcd_enable(void){

  LCD_PORT |= (1<<LCD_EN);     /* set ENABLE to 1 */
  _delay_us(LCD_ENABLE_US);
  LCD_PORT &= ~(1<<LCD_EN);    /* set ENABLE to 0 */

  return;
}

/* ---------------------------------------------------------------------
   send 4 bit to LCD
 --------------------------------------------------------------------- */
static void lcd_out(uint8_t data){

  data &= 0xF0;                       /* mask upper 4 bits */

  LCD_PORT &= ~(0xF0>>(4-LCD_DB));    /* delete mask */
  LCD_PORT |= (data>>(4-LCD_DB));     /* set bits */
  lcd_enable();

  return;
}

/* ---------------------------------------------------------------------
   initializes the display
 --------------------------------------------------------------------- */
void lcd_init(void){

  /* set uses data-lines to output */
  uint8_t pins = (0x0F << LCD_DB) |
                 (1<<LCD_RS) |
                 (1<<LCD_EN);
  LCD_DDR |= pins;

  /* init output pins to 0 */
  LCD_PORT &= ~pins;

  /* wait until display is ready */
  _delay_ms(LCD_BOOTUP_MS);

  /* soft reset: has to be done 3 times */
  lcd_out(LCD_SOFT_RESET);
  _delay_ms(LCD_SOFT_RESET_MS1);

  lcd_enable();
  _delay_ms(LCD_SOFT_RESET_MS2);

  lcd_enable();
  _delay_ms(LCD_SOFT_RESET_MS3);

  /* enable 4-bit-mode */
  lcd_out(LCD_SET_FUNCTION | LCD_FUNCTION_4BIT);
  _delay_ms(LCD_SET_4BITMODE_MS);

  /* enable 4-bit-mode with 2 rows and 5x7 pixel per character */
  lcd_command(LCD_SET_FUNCTION |
              LCD_FUNCTION_4BIT |
              LCD_FUNCTION_2LINE |
              LCD_FUNCTION_5X7);

  /* Display: ON, Cursor: OFF, blink cursor: OFF */
  lcd_command(LCD_SET_DISPLAY |
              LCD_DISPLAY_ON |
              LCD_CURSOR_OFF |
              LCD_BLINKING_OFF);

  /* increment cursor, no scrolling */
  lcd_command(LCD_SET_ENTRY |
              LCD_ENTRY_INCREASE |
              LCD_ENTRY_NOSHIFT);

  lcd_clear();

  return;
}

/* ---------------------------------------------------------------------
   send a byte to the display
 --------------------------------------------------------------------- */
void lcd_data(uint8_t data){

  LCD_PORT |= (1<<LCD_RS);    /* set reset to 1 */

  lcd_out(data);            /* send upper nibble */
  lcd_out(data<<4);         /* send lower nibble */

  _delay_us(LCD_WRITEDATA_US);

  return;
}

/* ---------------------------------------------------------------------
   send a command to the display
 --------------------------------------------------------------------- */
void lcd_command(uint8_t data){

  LCD_PORT &= ~(1<<LCD_RS);    /* set reset to 0 */

  lcd_out( data );             /* send upper nibble */
  lcd_out( data<<4 );          /* send lower nibble */

  _delay_us( LCD_COMMAND_US );

  return;
}

/* ---------------------------------------------------------------------
   send command to clear all display contents
 --------------------------------------------------------------------- */
void lcd_clear(void){

  lcd_command( LCD_CLEAR_DISPLAY );
  _delay_ms( LCD_CLEAR_DISPLAY_MS );

  return;
}

/* ---------------------------------------------------------------------
   set the cursor to home position
 --------------------------------------------------------------------- */
void lcd_home(void){

  lcd_command( LCD_CURSOR_HOME );
  _delay_ms( LCD_CURSOR_HOME_MS );

  return;
}

/* ---------------------------------------------------------------------
   set cursor to specific position (x=col, y=row)
 --------------------------------------------------------------------- */
void lcd_setcursor(uint8_t x, uint8_t y){

  uint8_t data;

  switch(y){
    case 1:    /* 1st row */
      data = LCD_SET_DDADR + LCD_DDADR_LINE1 + x;
      break;

    case 2:    /* 2nd row */
      data = LCD_SET_DDADR + LCD_DDADR_LINE2 + x;
      break;

    case 3:    /* 3rd row */
      data = LCD_SET_DDADR + LCD_DDADR_LINE3 + x;
      break;

    case 4:    /* 4th row */
      data = LCD_SET_DDADR + LCD_DDADR_LINE4 + x;
      break;

    default:
      return;  /* unknown row */
  }

  lcd_command(data);

  return;
}

/* ---------------------------------------------------------------------
   send a string to the display
 --------------------------------------------------------------------- */
void lcd_string(const char *data){

  while(*data != '\0'){
    lcd_data(*data++);
  }

  return;
}

/* ---------------------------------------------------------------------
   generate a user defined character into LCD's ROM
 --------------------------------------------------------------------- */
void lcd_generatechar(uint8_t code, const uint8_t *data){

  uint8_t i = 0;
  /* set start position of character */
  lcd_command(LCD_SET_CGADR | (code<<3));

  /* transfer bit pattern */
  for (i = 0; i < 8; i++){
    lcd_data(data[i]);
  }

  return;
}

/* ---------------------------------------------------------------------
   set desired motor position if unit is degree
 --------------------------------------------------------------------- */
void degreeToSteps(uint8_t mot, double degree, double multiply){

  double roundedSteps = 0.0;

  roundedSteps = round(degree * multiply *
          ((motor[mot].stepsPerFullRotation
           *motor[mot].gearRatio
           *motor[mot].subSteps)/(360.0f)));

  motor[mot].desiredPosition += (int16_t)roundedSteps;

  /* calculate rounding-error */
  motor[mot].stepError +=
      degree * multiply *
      ((motor[mot].stepsPerFullRotation
       *motor[mot].gearRatio
       *motor[mot].subSteps)/(360.0f))
       - roundedSteps;

  return;
}

/* ---------------------------------------------------------------------
   calculate degree from steps
 --------------------------------------------------------------------- */
double stepsToDegree(uint8_t mot, int16_t steps){

  double radian = 0.0f;

  radian = (steps) * ( (360.0f)/(motor[mot].gearRatio
                                *motor[mot].subSteps
                                *motor[mot].stepsPerFullRotation) );

  return radian;

}

/* ---------------------------------------------------------------------
   set desired motor position if unit is radian
 --------------------------------------------------------------------- */
void radiansToSteps(uint8_t mot, double rad, double multiply){

  double roundedSteps = 0.0f;

  roundedSteps = round(rad * multiply *
          ((motor[mot].stepsPerFullRotation
           *motor[mot].gearRatio
           *motor[mot].subSteps)/(2.0f)));

  motor[mot].desiredPosition += (int16_t)roundedSteps;

  /* calculate rounding-error */
  motor[mot].stepError +=
      rad * multiply *
      ((motor[mot].stepsPerFullRotation
       *motor[mot].gearRatio
       *motor[mot].subSteps)/(2.0f))
       - roundedSteps;

  return;
}

/* ---------------------------------------------------------------------
   calculate radians from steps
 --------------------------------------------------------------------- */
double stepsToRadian(uint8_t mot, int16_t steps){

  double radian = 0.0f;

  radian = (steps) * ( (2.0f)/(motor[mot].gearRatio
                              *motor[mot].subSteps
                              *motor[mot].stepsPerFullRotation) );

  return radian;
}

/* ---------------------------------------------------------------------
   set constant angular velocity
 --------------------------------------------------------------------- */
void setConstSpeed(uint8_t i, uint8_t state){

  ATOMIC_BLOCK(ATOMIC_FORCEON){
    switch(state){
      case MOTOR_MOVE_INFINITE_STOP:
        motor[i].isMovingInfinite = MOTOR_MOVE_INFINITE_STOP;
        motor[i].desiredPosition  = motor[i].actualPosition;
        break;

      case MOTOR_MOVE_INFINITE_CW:
        motor[i].isMovingInfinite = MOTOR_MOVE_INFINITE_CW;
        motor[i].desiredPosition  = motor[i].actualPosition;
        motor[i].desiredPosition += 1;
        break;

      case MOTOR_MOVE_INFINITE_CCW:
        motor[i].isMovingInfinite = MOTOR_MOVE_INFINITE_CCW;
        motor[i].desiredPosition  = motor[i].actualPosition;
        motor[i].desiredPosition += -1;
        break;

      default:
        break;
    }
  }

  return;
}


/* ---------------------------------------------------------------------
   replaces the actual display content with another content if
   necessary
 --------------------------------------------------------------------- */
void updateDisplay(void){

  uint8_t j = 0;

  switch(menu.currentMenuMode){
    case MENU_SCROLL_MODE:
      if(menu.currentDisplayedMenu != menu.newDisplayedMenu){
        changeDisplayMenu(menu.newDisplayedMenu);
        lcd_setcursor(0, 1);  /* line 1 */
        lcd_string(menu.newMenuText[0]);
        strcpy(menu.currentMenuText[0], menu.newMenuText[0]);
        lcd_setcursor(0, 2);  /* line 2 */
        lcd_string(menu.newMenuText[1]);
        strcpy(menu.currentMenuText[1], menu.newMenuText[1]);
        menu.currentDisplayedMenu = menu.newDisplayedMenu;
        for(j = 0; j <= NUMBER_DISPLAY_VALUE_STRINGS; j++){
          sprintf(menu.currentDisplayValue[j], "no text");
        }
      }
      break;

    case MENU_CHANGE_MODE:
    case MENU_VALUE_CHANGE:
      updateDisplayChangeValues(menu.currentDisplayedMenu);
      if(strcmp(menu.currentDisplayValue[0], menu.newDisplayValue[0])){
        lcd_setcursor(0, 1);
        lcd_string(menu.newDisplayValue[0]);
        strcpy(menu.currentDisplayValue[0], menu.newDisplayValue[0]);
      }
      if(strcmp(menu.currentDisplayValue[1], menu.newDisplayValue[1])){
        lcd_setcursor(8, 1);
        lcd_string(menu.newDisplayValue[1]);
        strcpy(menu.currentDisplayValue[1], menu.newDisplayValue[1]);
      }
      if(strcmp(menu.currentDisplayValue[2], menu.newDisplayValue[2])){
        lcd_setcursor(0, 2);
        lcd_string(menu.newDisplayValue[2]);
        strcpy(menu.currentDisplayValue[2], menu.newDisplayValue[2]);
      }
      if(strcmp(menu.currentDisplayValue[3], menu.newDisplayValue[3])){
        lcd_setcursor(8, 2);
        lcd_string(menu.newDisplayValue[3]);
        strcpy(menu.currentDisplayValue[3], menu.newDisplayValue[3]);
      }
      break;

    default:
      break;
  }

  menu.currentMenuMode = menu.newMenuMode;

  return;
}

/* ---------------------------------------------------------------------
   changes the displayed menu item
 --------------------------------------------------------------------- */
void changeDisplayMenu(uint8_t i){

  menuItem *menuPtr;
  char *menuText;

  uint8_t j = 0;
  uint8_t k = 0;
  uint8_t sLen = 0;

  menuPtr  = (menuItem*)pgm_read_word(&menuList[i]);
  menuText = (char*)pgm_read_word(&menuPtr->displayText);

  strcpy_P(displayBuffer, menuText);

  displayBuffer = strtok(displayBuffer, "\n");
  strcpy(menu.newMenuText[0], displayBuffer);
  displayBuffer = strtok(NULL, "\0");
  strcpy(menu.newMenuText[1], displayBuffer);

  /* fill the strings up with spaces */
  for(j = 0; j < NUMBER_DISPLAY_MENU_STRINGS; j++){
    sLen = strlen(menu.newMenuText[j]);
    for(k = sLen; k < DISPLAY_MENU_STRING_LENGTH-1; k++){
      menu.newMenuText[j][k] = ' ';
    }
    menu.newMenuText[j][DISPLAY_MENU_STRING_LENGTH-1] = '\0';
  }

  return;
}

/* ---------------------------------------------------------------------
   update the display after switching to value-changing-mode
   or update changed values for a motor on the display
 --------------------------------------------------------------------- */
void updateDisplayChangeValues(uint8_t thisMenu){

  uint8_t i, j;
  uint8_t state;
  menuItem *menuPtr;

  uint8_t sLen;
  uint8_t c = 0;

  /* determine which values shall be changed */
  menuPtr = (menuItem*)pgm_read_word(&menuList[thisMenu]);
  state = (uint8_t)pgm_read_byte(&menuPtr->state);

  /* load the values of all 4 motors */
  switch(state){
    case MENU_MAIN:   /* main menu point, no values here to change */
      sprintf(menu.newDisplayValue[0], "3.PI  Un");
      sprintf(menu.newDisplayValue[1], "i Stutt.");
      sprintf(menu.newDisplayValue[2], "Firmware");
      sprintf(menu.newDisplayValue[3], FW_VERSION);
      break;

    case MENU_CHANGE_POSITION:
      for(i = 0; i <= MAX_MOTOR; i++){
        c = ' ';
        if(menu.selectedMotor & (1 << i)){
          c = 0x7E;
          if(menu.fastMovingMode){
            c = 0x3E;
          }
        }
        switch(motor[i].stepUnit){
          case MOTOR_STEP_UNIT_STEP:
            sprintf(menu.newDisplayValue[i], "%c%dst", c, motor[i].actualPosition);
            break;

          case MOTOR_STEP_UNIT_DEGREE:
            /* 0xDF is the display code for the degree-circle */
            sprintf(menu.newDisplayValue[i], "%c%.1f%c", c, stepsToDegree(i, motor[i].actualPosition), 0xDF);
            break;

          case MOTOR_STEP_UNIT_RADIAN:
            /* 0xF7 is the display code for a greek pi */
            sprintf(menu.newDisplayValue[i], "%c%.3f%c", c, stepsToRadian(i, motor[i].actualPosition), 0xF7);
            break;

          default:
            break;
        }
      }
      break;

    case MENU_CHANGE_STEP_UNIT:
      for(i = 0; i <= MAX_MOTOR; i++){
        c = (menu.selectedMotor & (1 << i)) ? 0x7E : ' ';
        if(motor[i].stepUnit == MOTOR_STEP_UNIT_STEP){
          sprintf(menu.newDisplayValue[i], "%c%s", c, "step");
        }
        if(motor[i].stepUnit == MOTOR_STEP_UNIT_DEGREE){
          sprintf(menu.newDisplayValue[i], "%c%s", c, "degree");
        }
        if(motor[i].stepUnit == MOTOR_STEP_UNIT_RADIAN){
          sprintf(menu.newDisplayValue[i], "%c%s", c, "radian");
        }
      }
      break;

    case MENU_CHANGE_WAIT_TIME:
      for(i = 0; i <= MAX_MOTOR; i++){
        c = (menu.selectedMotor & (1 << i)) ? 0x7E : ' ';
        sprintf(menu.newDisplayValue[i], "%c%d ms", c, motor[i].waitBetweenSteps);
      }
      break;

    case MENU_SET_STEP_MULTIPL:
      for(i = 0; i <= MAX_MOTOR; i++){
        c = (menu.selectedMotor & (1 << i)) ? 0x7E : ' ';
        sprintf(menu.newDisplayValue[i], "%c%.1fx", c, motor[i].stepMultiplier);
      }
      break;

    case MENU_RUN_ZERO_CALIBRATION:
      for(i = 0; i <= MAX_MOTOR; i++){
        c = (menu.selectedMotor & (1 << i)) ? 0x7E : ' ';
        if(forbiddenZone[i].active){
          sprintf(menu.newDisplayValue[i], "ForbZone");
        }
        else{
          sprintf(menu.newDisplayValue[i], "%cMot %d", c, i);
        }
      }
      break;

    case MENU_CHANGE_SUBSTEPS:
      for(i = 0; i <= MAX_MOTOR; i++){
        c = (menu.selectedMotor & (1 << i)) ? 0x7E : ' ';
        sprintf(menu.newDisplayValue[i], "%c%.0f", c, motor[i].subSteps);
      }
      break;

    case MENU_SAVE_CONFIG:
      /* select the dummy motor */
      sprintf(menu.newDisplayValue[0], "Save all");
      sprintf(menu.newDisplayValue[1], " current");
      sprintf(menu.newDisplayValue[2], "configur");
      sprintf(menu.newDisplayValue[3], "ations  ");
      break;

    case MENU_LOAD_CONFIG:
      /* select the dummy motor */
      sprintf(menu.newDisplayValue[0], "Load all");
      sprintf(menu.newDisplayValue[1], " saved  ");
      sprintf(menu.newDisplayValue[2], "configur");
      sprintf(menu.newDisplayValue[3], "ations  ");
      break;

    case MENU_OPTICAL_ZERO_POS:
      for(i = 0; i <= MAX_MOTOR; i++){
        c = ' ';
        if(menu.selectedMotor & (1 << i)){
          c = 0x7E;
          if(menu.fastMovingMode){
            c = 0x3E;
          }
        }
        sprintf(menu.newDisplayValue[i], "%c%dst", c, motor[i].opticalZeroPosition);
      }
      break;

    case MENU_CONST_ANGULAR_SPEED:
      for(i = 0; i <= MAX_MOTOR; i++){
        c = (menu.selectedMotor & (1 << i)) ? 0x7E : ' ';
        if(forbiddenZone[i].active){
          sprintf(menu.newDisplayValue[i], "ForbZone");
        }
        else{
          if(motor[i].angularVelocity == MOTOR_MOVE_INFINITE_STOP){
            sprintf(menu.newDisplayValue[i], "%cSTOP", c);
          }
          if(motor[i].angularVelocity == MOTOR_MOVE_INFINITE_CW){
            sprintf(menu.newDisplayValue[i], "%cCW", c);
          }
          if(motor[i].angularVelocity == MOTOR_MOVE_INFINITE_CCW){
            sprintf(menu.newDisplayValue[i], "%cCCW", c);
          }
        }
      }
      break;

    case MENU_RUN_PROGRAM:
      sprintf(menu.newDisplayValue[0], "Program ");
      sprintf(menu.newDisplayValue[1], "running ");
      sprintf(menu.newDisplayValue[2], "Step %d" , menu.currentProgramStep);
      sprintf(menu.newDisplayValue[3], "        ");
      break;

    default:  /* in case of fire ;-) */
      break;
  }

  /* fill the new ascii values up with spaces */
  for(i = 0; i < NUMBER_DISPLAY_VALUE_STRINGS; i++){
    sLen = strlen(menu.newDisplayValue[i]);
    for(j = sLen; j < DISPLAY_VALUE_STRING_LENGTH-1; j++){
      menu.newDisplayValue[i][j] = ' ';
    }
    menu.newDisplayValue[i][DISPLAY_VALUE_STRING_LENGTH-1] = '\0';
  }

  return;
}


/* ---------------------------------------------------------------------
   implementing the display state machine
 --------------------------------------------------------------------- */
void updateMenu(void){

  int8_t menuPrompt;
  uint8_t menuState;

  int8_t rotEncVal;
  uint8_t buttonVal;

  uint8_t state;
  menuItem *menuPtr;

  uint8_t i = 0;

  /* first check if we have something to change at all */
  if(buttonState.readyToProcess == 0 && rotEnc.readyToProcess == 0){
    /* nothing to be done */
    return;
  }

  /* now get information about the actual display prompt */
  menuPrompt = menu.currentDisplayedMenu;
  menuState  = menu.currentMenuMode;

  if(menuState == MENU_SCROLL_MODE){
    /* so here we want to scroll through the menu */

    rotEncVal = getRotaryEncoderEvent();  /* get wanted menu prompt */
    menuPrompt = (menuPrompt+(int8_t)rotEncVal) % NUMBER_OF_DISPLAY_MENUS;
    if(menuPrompt < 0){
      menuPrompt = NUMBER_OF_DISPLAY_MENUS - 1;
    }

    menu.newDisplayedMenu = (uint8_t)menuPrompt;

    /* or enter the MENU_CHANGE_MODE */
    if(getButtonEvent() == BUTTON_ROT_ENC && menu.selectedMotor == 0){
      menu.newMenuMode = MENU_CHANGE_MODE;
    }
    if(getButtonEvent() == BUTTON_ROT_ENC && menu.selectedMotor != 0){
      menu.newMenuMode = MENU_VALUE_CHANGE;
    }
  }

  if((menuState == MENU_CHANGE_MODE) || (menuState == MENU_VALUE_CHANGE)){
    /* Well, we want to select a motor to change its above selected value */
    buttonVal = getButtonEvent();
    switch(buttonVal){
      case NO_BUTTON:
        break;

      case BUTTON_MOTOR0:
        menu.selectedMotor ^= (1 << MOTOR0);
        menu.newMenuMode = MENU_VALUE_CHANGE;
        break;

      case BUTTON_MOTOR1:
        menu.selectedMotor ^= (1 << MOTOR1);
        menu.newMenuMode = MENU_VALUE_CHANGE;
        break;

      case BUTTON_MOTOR2:
        menu.selectedMotor ^= (1 << MOTOR2);
        menu.newMenuMode = MENU_VALUE_CHANGE;
        break;

      case BUTTON_MOTOR3:
        menu.selectedMotor ^= (1 << MOTOR3);
        menu.newMenuMode = MENU_VALUE_CHANGE;
        break;

      case BUTTON_ROT_ENC:
        menu.fastMovingMode ^= 1;
        break;

      case BUTTON_MENUESCAPE:
        /* or get back to the MENU_SCROLL_MODE */
        menu.newMenuMode = MENU_SCROLL_MODE;
        menu.currentDisplayedMenu += 1;
        menu.fastMovingMode = OFF;
        break;

      default:
        break;
    }
  }

  if(menuState == MENU_VALUE_CHANGE){
    /* here we have a motor selected and want to change any of its values */
    rotEncVal = getRotaryEncoderEvent();
    if(rotEncVal != 0){
      menuPtr = (menuItem*)pgm_read_word(&menuList[menu.currentDisplayedMenu]);
      state = (uint8_t)pgm_read_byte(&menuPtr->state);

      switch(state){
        case MENU_MAIN:   /* main menu point, no values here to change */
          break;

        case MENU_CHANGE_POSITION:   /* change motor position */
          for(i = MOTOR0; i <= MAX_MOTOR; i++){
            if(menu.selectedMotor & (1 << i)){
              switch(motor[i].stepUnit){
                case MOTOR_STEP_UNIT_STEP:
                  if(menu.fastMovingMode){
                    motor[i].desiredPosition += ((int16_t)rotEncVal)*100;
                  }
                  else{
                    motor[i].desiredPosition += (int16_t)rotEncVal;
                  }
                  break;

                case MOTOR_STEP_UNIT_DEGREE:
                  if(menu.fastMovingMode){
                    degreeToSteps(i, (rotEncVal)*10.0, motor[i].stepMultiplier);
                  }
                  else{
                    degreeToSteps(i, rotEncVal, motor[i].stepMultiplier);
                  }
                  break;

                case MOTOR_STEP_UNIT_RADIAN:
                  /* default step is pi/8 */
                  /* pi/8 is far enough, so no fast moving mode here */
                  radiansToSteps(i, (rotEncVal)*0.125, motor[i].stepMultiplier);
                  break;

                default:
                  break;
              }
            }
          }
          break;

        case MENU_CHANGE_STEP_UNIT:   /* change step units */
          for(i = MOTOR0; i <= MAX_MOTOR; i++){
            if(menu.selectedMotor & (1 << i)){
              motor[i].stepUnit += rotEncVal;
              if(motor[i].stepUnit < 0){
                motor[i].stepUnit = 2;
              }
              if(motor[i].stepUnit > 2){
                motor[i].stepUnit = 0;
              }
            }
          }
          break;

        case MENU_CHANGE_WAIT_TIME:   /* change wait time between steps */
          for(i = MOTOR0; i <= MAX_MOTOR; i++){
            if(menu.selectedMotor & (1 << i)){
              motor[i].waitBetweenSteps += rotEncVal;
              if(motor[i].waitBetweenSteps < 1){
                /* wait time is at least 1 ms */
                motor[i].waitBetweenSteps = 1;
              }
              motor[i].delayCounter = 2*motor[i].waitBetweenSteps-1;
            }
          }
          break;

        case MENU_SET_STEP_MULTIPL:
          for(i = MOTOR0; i <= MAX_MOTOR; i++){
            if(menu.selectedMotor & (1 << i)){
              motor[i].stepMultiplier += (rotEncVal)/10.0;
            }
          }
          break;

        case MENU_RUN_ZERO_CALIBRATION:   /* run ZERO calibration */
          for(i = MOTOR0; i <= MAX_MOTOR; i++){
            if(menu.selectedMotor & (1 << i)){
              motorZeroRun(i);
              menu.selectedMotor ^= (1 << i);
              updateDisplay();
            }
          }
          /* get back to the MENU_SCROLL_MODE when finished calibration*/
          menu.newMenuMode = MENU_SCROLL_MODE;
          menu.currentDisplayedMenu += 1;
          break;

        case MENU_CHANGE_SUBSTEPS:   /* change motor substeps */
          for(i = MOTOR0; i <= MAX_MOTOR; i++){
            if(menu.selectedMotor & (1 << i)){
              if(rotEncVal > 0){
                motor[i].subSteps *= 2.0;
              }
              if(rotEncVal < 0){
                motor[i].subSteps /= 2.0;
              }
              if(motor[i].subSteps < 1){
                motor[i].subSteps = 1;
              }
              if(motor[i].subSteps > 32){
                motor[i].subSteps = 32;
              }
            }
          }
          break;

        case MENU_SAVE_CONFIG:   /* save current configuration */
          saveConfigToEEPROM();
          lcd_clear();
          lcd_string("saved");
          break;

        case MENU_LOAD_CONFIG:   /* load last configuration */
          loadConfigFromEEPROM();
          lcd_clear();
          lcd_string("loaded");
          break;

        case MENU_OPTICAL_ZERO_POS: /* define the optical zero position */
          for(i = MOTOR0; i <= MAX_MOTOR; i++){
            if(menu.selectedMotor & (1 << i)){
              if(menu.fastMovingMode){
                defineOpticalZeroPosition(i, rotEncVal*100);
              }
              else{
                defineOpticalZeroPosition(i, rotEncVal);
              }
            }
          }
          break;

        case MENU_CONST_ANGULAR_SPEED:  /* enter constant moving mode */
          for(i = MOTOR0; i <= MAX_MOTOR; i++){
            if(menu.selectedMotor & (1 << i)){
              if(forbiddenZone[i].active){
                /* const angular speed not allowed if
                 * fobidden zone is set */
                break;
              }
              motor[i].angularVelocity += rotEncVal;
              if(motor[i].angularVelocity < 0){
                motor[i].angularVelocity = 2;
              }
              if(motor[i].angularVelocity > 2){
                motor[i].angularVelocity = 0;
              }
              setConstSpeed(i, motor[i].angularVelocity);
            }
          }
          break;

        case MENU_RUN_PROGRAM:  /* run an internal program defined by CLI */
          if(rotEncVal > 0){
            do{
              /* find the next active program step */
              menu.currentProgramStep += 1;
              if(menu.currentProgramStep >= MAX_PROGRAM_STEPS){
                menu.currentProgramStep = 0;
              }
            }while(programList[menu.currentProgramStep].isActive == 0);
          }

          if(rotEncVal < 0){
            do{
              /* find the next active program step */
              menu.currentProgramStep += -1;
              if(menu.currentProgramStep < 0){
                menu.currentProgramStep = MAX_PROGRAM_STEPS - 1;
              }
            }while(programList[menu.currentProgramStep].isActive == 0);
          }

          if(programList[menu.currentProgramStep].absRel == PROG_ABSOLUTE_MOVEMENT){
            for(i = 0; i <= MAX_MOTOR; i++){
              motor[i].desiredPosition = programList[menu.currentProgramStep].position[i];
            }
          }
          else if(programList[menu.currentProgramStep].absRel == PROG_RELATIVE_MOVEMENT){
            for(i = 0; i <= MAX_MOTOR; i++){
              motor[i].desiredPosition = motor[i].actualPosition
                                         + programList[menu.currentProgramStep].position[i];
            }
          }
          else{
            ;
          }
          break;

        default:  /* in case of fire ;-) */
          break;
      }
    }
  }

  buttonState.readyToProcess = 0;
  rotEnc.readyToProcess = 0;

  return;
}




/* =====================================================================
    Manual operation subsystem
====================================================================== */

/* ---------------------------------------------------------------------
   initializes all buttons and turns on the required interrupts
 --------------------------------------------------------------------- */
void initManualOperatingButtons(void){

  DDRB = 0x00;    /* configure user interface port as input */
  PORTB = 0xFF;   /* set all internal pull-ups */

  /* set up a timer for button/rotary_encoder polling
   *
   * the 8-bit Timer/Counter0 is used for that
   */

  TCCR0A |= (1<<WGM01);   /* enable CTC */
  OCR0A   = 80;           /* 80 is an empirical value for best behavior of the rotEnc. */
  TIMSK0 |= (1<<OCIE0A);  /* enable interrupt */
  TCNT0   = 0;

  /* start the timer/counter */
  TCCR0B |= (1<<CS02)|(1<<CS00);  /* prescaler = 1024 --> 51.2 us per cycle */

  return;
}

/* ---------------------------------------------------------------------
   process a button event (and the pressed rotary encoder)
 --------------------------------------------------------------------- */
uint8_t getButtonEvent(void){

  uint8_t state = 0;
  uint8_t button = NO_BUTTON;

  if(buttonState.readyToProcess){
    /* a button has been pressed */
    state = buttonState.inputRegister^0xFF;   /* invert state register */

    if(state & (1<<BUTTON_MOTOR0)){
      button = BUTTON_MOTOR0;
    }
    else if(state & (1<<BUTTON_MOTOR1)){
      button = BUTTON_MOTOR1;
    }
    else if(state & (1<<BUTTON_MOTOR2)){
      button = BUTTON_MOTOR2;
    }
    else if(state & (1<<BUTTON_MOTOR3)){
      button = BUTTON_MOTOR3;
    }
    else if(state & (1<<BUTTON_MENUESCAPE)){
      button = BUTTON_MENUESCAPE;
    }
    else if(state & (1<<BUTTON_ROT_ENC)){
      button = BUTTON_ROT_ENC;
    }
    else{
      button = NO_BUTTON;
    }
  }

  return button;
}


/* ---------------------------------------------------------------------
   process a rotary encoder event (rotation only)
 --------------------------------------------------------------------- */
int8_t getRotaryEncoderEvent(void){

  /*
   * returns the turned steps of the rotary encoder since
   * since last look-up here
   */

  int8_t steps = 0;

  ATOMIC_BLOCK(ATOMIC_FORCEON){
    if(rotEnc.readyToProcess){
      steps = rotEnc.steps;
      rotEnc.steps = 0;
    }
  }
  return steps;
}



/* =====================================================================
    parser functions
====================================================================== */

/* ---------------------------------------------------------------------
    copy command to internal buffer
 --------------------------------------------------------------------- */
void copyRXstring(void){

  ATOMIC_BLOCK(ATOMIC_FORCEON){
    memcpy(commandString.buffer, rxString.buffer, SERIAL_BUFFERSIZE);
    commandString.readyToProcess = 1;

    memset(rxString.buffer, 0, SERIAL_BUFFERSIZE);
    rxString.charCount = 0;
    rxString.readyToProcess = 0;
  }

  return;
}


/* ---------------------------------------------------------------------
    returns the command code and fills the parameter list
 --------------------------------------------------------------------- */
uint8_t parseCommand(void){

  uint8_t commandCode = 0x80; /* initialize with "no command" */
  uint8_t noOfOpts = 0;
  uint8_t i = 0;
  uint8_t j = 0;
  command *cmdPtr;
  char *token;

  ATOMIC_BLOCK(ATOMIC_FORCEON){
    if(commandString.readyToProcess){
      /* extract command and get cmdCode */
      token = strtok(commandString.buffer, ALLOWED_CMD_DELIMITERS);

      for(i = 0; i < TOTAL_NUMBER_OF_COMMANDS; i++){            /* loop over all commands */
        cmdPtr = (command*)pgm_read_word(&commandList[i]);      /* pointer to struct in flash */
        strcpy_P(commandParam[0], pgm_read_word(&cmdPtr->cmd)); /* contents of command code in flash */

        if(strcmp(token, commandParam[0]) == 0){
          /* found a known command */
          commandCode = (uint8_t)pgm_read_byte(&cmdPtr->cmdCode);
          noOfOpts = (int8_t)pgm_read_byte(&cmdPtr->numberOfOptions);
        }
      }

      /* now parse the command parameters into tokens */
      for(i = 1; i <= noOfOpts; i++){
        token = strtok(NULL, ALLOWED_CMD_DELIMITERS);
        strcpy(commandParam[i], token);
      }

      memset(commandString.buffer, 0, SERIAL_BUFFERSIZE);
      commandString.readyToProcess = 0;
    }
  }

  return commandCode;
}

/* ---------------------------------------------------------------------
    moves the motor to an absolute position
 --------------------------------------------------------------------- */
void commandMoveAbs(char* param0, char* param1, char* param2){

  uint8_t i = 0;
  double actMotorPos = 0.0f;
  double posDiff = 0.0f;

  i = (uint8_t)strtol(param0, (char **)NULL, 10);

  if(i > MAX_MOTOR){
    return;
  }

  if(strcmp(param2, "steps") == 0){
    motor[i].desiredPosition = (int16_t)strtol(param1, (char **)NULL, 10);
  }
  else if(strcmp(param2, "deg") == 0){
    actMotorPos = stepsToDegree(i, motor[i].actualPosition);
    posDiff = atof(param1) - actMotorPos;
    degreeToSteps(i, posDiff, 1.0);
  }
  else if(strcmp(param2, "pi") == 0){
    actMotorPos = stepsToRadian(i, motor[i].actualPosition);
    posDiff = atof(param1) - actMotorPos;
    radiansToSteps(i, posDiff, 1.0);
  }
  else{
    sendText("ERROR: else");
  }

  return;
}

/* ---------------------------------------------------------------------
    moves the motor relative to the actual position
 --------------------------------------------------------------------- */
void commandMoveRel(char* param0, char* param1, char* param2){

  uint8_t i = 0;

  i = (uint8_t)strtol(param0, (char **)NULL, 10);

  if(i > MAX_MOTOR){
    return;
  }

  if(strcmp(param2, "steps") == 0){
    motor[i].desiredPosition += (int16_t)strtol(param1, (char **)NULL, 10);
  }
  if(strcmp(param2, "deg") == 0){
    degreeToSteps(i, atof(param1), 1.0f);
  }
  if(strcmp(param2, "pi") == 0){
    radiansToSteps(i, atof(param1), 1.0f);
  }

  return;
}

/* ---------------------------------------------------------------------
    enables/disables a motor
 --------------------------------------------------------------------- */
void commandEnable(char* param0, char* param1){

  uint8_t i   = 0;
  uint8_t val = 0;

  i   = (uint8_t)strtol(param0, (char **)NULL, 10);
  val = (uint8_t)strtol(param1, (char **)NULL, 10);

  if(i > MAX_MOTOR){
    return;
  }
  else{
    if(val){
      setMotorState(i, ON);
    }
    else{
      setMotorState(i, OFF);
    }
  }

  return;
}

/* ---------------------------------------------------------------------
    returns the actual motor position as string
    the format depends on the given unit
 --------------------------------------------------------------------- */
char* commandGetMotorPosition(char* param0, char* param1){

  uint8_t i = 0;

  i = (uint8_t)strtol(param0, (char **)NULL, 10);

  if(i > MAX_MOTOR){
    sprintf(txString.buffer, "err: unknown motor: %d", i);
  }
  else{
    if(strcmp(param1, "steps") == 0){
      sprintf(txString.buffer, "%d", motor[i].actualPosition);
    }
    else if(strcmp(param1, "deg") == 0){
      sprintf(txString.buffer, "%f", stepsToDegree(i, motor[i].actualPosition));
    }
    else if(strcmp(param1, "pi") == 0){
      sprintf(txString.buffer, "%f", stepsToRadian(i, motor[i].actualPosition));
    }
    else{
      /* wrong unit argument returns in degree */
      sprintf(txString.buffer, "%f", stepsToDegree(i, motor[i].actualPosition));
    }
  }

  return txString.buffer;
}

/* ---------------------------------------------------------------------
    returns whether a motor is moving or not
 --------------------------------------------------------------------- */
char* commandIsMoving(char* param0){

  uint8_t i = 0;

  i = (uint8_t)strtol(param0, (char **)NULL, 10);

  if(i > MAX_MOTOR){
    sprintf(txString.buffer, "err: unknown motor: %d", i);
  }
  else{
    if(motor[i].desiredPosition - motor[i].actualPosition){
      sprintf(txString.buffer, "1");
    }
    else{
      sprintf(txString.buffer, "0");
    }
  }

  return txString.buffer;
}

/* ---------------------------------------------------------------------
    returns a measured analog value
 --------------------------------------------------------------------- */
char* commandGetAnalog(char* param0){

  uint8_t i = 0;
  uint16_t val = 0;

  i = (uint8_t)strtol(param0, (char **)NULL, 10);

  if((i < MOTOR_SENS0) || (i > MOTOR_SENS_MAX)){
    sprintf(txString.buffer, "err: unknown motor: %d", i);
  }
  else{
    val = getADCvalue(i);
    sprintf(txString.buffer, "%d", val);
  }

  return txString.buffer;
}

/* ---------------------------------------------------------------------
    returns the optical zero position
 --------------------------------------------------------------------- */
char* commandGetOptZeroPos(char* param0){

  uint8_t i = 0;

  i = (uint8_t)strtol(param0, (char **)NULL, 10);

  if(i < MOTOR0 || i > MAX_MOTOR){
    sprintf(txString.buffer, "err: unknown motor: %d", i);
  }
  else{
    sprintf(txString.buffer, "%d", motor[i].opticalZeroPosition);
  }

  return txString.buffer;
}

/* ---------------------------------------------------------------------
    sets the optical zero position
    as offset from magnetic zero position
 --------------------------------------------------------------------- */
void commandSetOptZeroPos(char* param0, char* param1){

  uint8_t i = 0;
  int16_t val = 0;

  i = (uint8_t)strtol(param0, (char **)NULL, 10);

  if(i < MOTOR0 || i > MAX_MOTOR){
    sprintf(txString.buffer, "err: unknown motor: %d", i);
    sendText(txString.buffer);
    return;
  }
  else{
    val = (int16_t)strtol(param1, (char **)NULL, 10);
    motor[i].opticalZeroPosition = val;
  }

  return;
}

/* ---------------------------------------------------------------------
    returns the mechanical gear ratio
 --------------------------------------------------------------------- */
char* commandGetGearRatio(char* param0){

  uint8_t i = 0;

  i = (uint8_t)strtol(param0, (char **)NULL, 10);

  if(i < MOTOR0 || i > MAX_MOTOR){
    sprintf(txString.buffer, "err: unknown motor: %d", i);
  }
  else{
    sprintf(txString.buffer, "%f", motor[i].gearRatio);
  }

  return txString.buffer;
}

/* ---------------------------------------------------------------------
    sets the mechanical gear ratio
 --------------------------------------------------------------------- */
void commandSetGearRatio(char* param0, char* param1){

  uint8_t i = 0;
  double val = 0.0;

  i = (uint8_t)strtol(param0, (char **)NULL, 10);

  if(i < MOTOR0 || i > MAX_MOTOR){
    sprintf(txString.buffer, "err: unknown motor: %d", i);
    sendText(txString.buffer);
    return;
  }
  else{
    val = (double)atof(param1);
    motor[i].gearRatio = val;
  }

  return;
}

/* ---------------------------------------------------------------------
    returns the steps per full rotation w/o substeps
 --------------------------------------------------------------------- */
char* commandGetFullRotation(char* param0){

  uint8_t i = 0;

  i = (uint8_t)strtol(param0, (char **)NULL, 10);

  if(i < MOTOR0 || i > MAX_MOTOR){
    sprintf(txString.buffer, "err: unknown motor: %d", i);
  }
  else{
    sprintf(txString.buffer, "%.0f", motor[i].stepsPerFullRotation);
  }

  return txString.buffer;
}

/* ---------------------------------------------------------------------
    sets the steps per full rotation w/o substeps
 --------------------------------------------------------------------- */
void commandSetFullRotation(char* param0, char* param1){

  uint8_t i = 0;
  double val = 0.0;

  i = (uint8_t)strtol(param0, (char **)NULL, 10);

  if(i < MOTOR0 || i > MAX_MOTOR){
    sprintf(txString.buffer, "err: unknown motor: %d", i);
    sendText(txString.buffer);
    return;
  }
  else{
    val = (double)atof(param1);
    motor[i].stepsPerFullRotation = val;
  }

  return;
}

/* ---------------------------------------------------------------------
    returns the adjusted substeps
 --------------------------------------------------------------------- */
char* commandGetSubSteps(char* param0){

  uint8_t i = 0;

  i = (uint8_t)strtol(param0, (char **)NULL, 10);

  if(i < MOTOR0 || i > MAX_MOTOR){
    sprintf(txString.buffer, "err: unknown motor: %d", i);
  }
  else{
    sprintf(txString.buffer, "%.0f", motor[i].subSteps);
  }

  return txString.buffer;
}

/* ---------------------------------------------------------------------
    returns a measured analog value
 --------------------------------------------------------------------- */
void commandSetSubSteps(char* param0, char* param1){

  uint8_t i = 0;
  double val = 0.0;

  i = (uint8_t)strtol(param0, (char **)NULL, 10);

  if(i < MOTOR0 || i > MAX_MOTOR){
    sprintf(txString.buffer, "err: unknown motor: %d", i);
    sendText(txString.buffer);
    return;
  }
  else{
    val = (double)atof(param1);
    motor[i].subSteps = val;
  }

  return;
}

/* ---------------------------------------------------------------------
    returns the wait time between two single steps
 --------------------------------------------------------------------- */
char* commandGetWaitTime(char* param0){

  uint8_t i = 0;

  i = (uint8_t)strtol(param0, (char **)NULL, 10);

  if(i < MOTOR0 || i > MAX_MOTOR){
    sprintf(txString.buffer, "err: unknown motor: %d", i);
  }
  else{
    sprintf(txString.buffer, "%d", motor[i].waitBetweenSteps);
  }

  return txString.buffer;
}

/* ---------------------------------------------------------------------
    sets the wait time between two single steps
 --------------------------------------------------------------------- */
void commandSetWaitTime(char* param0, char* param1){

  uint8_t i = 0;
  uint16_t val = 0;

  i = (uint8_t)strtol(param0, (char **)NULL, 10);

  if(i < MOTOR0 || i > MAX_MOTOR){
    sprintf(txString.buffer, "err: unknown motor");
    sendText(txString.buffer);
    return;
  }
  else{
    val = (uint16_t)atoi(param1);
    motor[i].waitBetweenSteps = val;
  }

  return;
}

/* ---------------------------------------------------------------------
    sets the infinite moving mode
 --------------------------------------------------------------------- */
void commandSetConstSpeed(char* param0, char* param1, char* param2){

  uint8_t i = 0;
  double val = 0.0;
  uint16_t waitTime = 1;

  i = (uint8_t)strtol(param0, (char **)NULL, 10);

  if(i < MOTOR0 || i > MAX_MOTOR){
    sprintf(txString.buffer, "err: unknown motor");
    sendText(txString.buffer);
    return;
  }
  else{
    if(forbiddenZone[i].active){
      /* command not allowed if forbidden zone is set */
      return;
    }

    /* this is the wait-time for a full rotation in seconds */
    val = atof(param2);

    if(strcmp(param1, "STOP") == 0){
      ATOMIC_BLOCK(ATOMIC_FORCEON){
        motor[i].isMovingInfinite = MOTOR_MOVE_INFINITE_STOP;
        motor[i].waitBetweenSteps = 3;
        motor[i].desiredPosition  = motor[i].actualPosition;
      }
      return;
    }

    /* now calculate wait time between two steps in ms */
    waitTime = (uint16_t)round((fabs(val)*1000.0f) / ( motor[i].gearRatio
                                                      *motor[i].stepsPerFullRotation
                                                      *motor[i].subSteps));

    if(waitTime < 1){
      sprintf(txString.buffer, "err: time too short", waitTime);
      sendText(txString.buffer);
      return;
    }
    if(waitTime > 9999){
    sprintf(txString.buffer, "err: time too long");
    sendText(txString.buffer);
    }

    if(strcmp(param1, "CW")  == 0){
      motor[i].isMovingInfinite = MOTOR_MOVE_INFINITE_CW;
      motor[i].waitBetweenSteps = waitTime;
      motor[i].desiredPosition  = motor[i].actualPosition;
      motor[i].desiredPosition += 1;
    }
    if(strcmp(param1, "CCW") == 0){
      motor[i].isMovingInfinite = MOTOR_MOVE_INFINITE_CCW;
      motor[i].waitBetweenSteps = waitTime;
      motor[i].desiredPosition  = motor[i].actualPosition;
      motor[i].desiredPosition += -1;
    }
  }

  return;
}

/* ---------------------------------------------------------------------
    sets the whole system back to factory reset
 --------------------------------------------------------------------- */
void commandFactoryReset(void){

  ATOMIC_BLOCK(ATOMIC_FORCEON){
    initDataStructs();
    saveConfigToEEPROM();
  }

  return;
}

/* ---------------------------------------------------------------------
    defines forbidden zone
 --------------------------------------------------------------------- */
void commandSetForbiddenZone(char* param0, char* param1, char* param2){

  uint8_t i = 0;
  int16_t start = 0;
  int16_t stop  = 0;
  int16_t swap = 0;

  i = (uint8_t)strtol(param0, (char **)NULL, 10);

  if(i < MOTOR0 || i > MAX_MOTOR){
    sprintf(txString.buffer, "err: unknown motor");
    sendText(txString.buffer);
    return;
  }
  else{
    start = (int16_t)atoi(param1);
    stop  = (int16_t)atoi(param2);

    if(start == stop){
      forbiddenZone[i].start  = 0;
      forbiddenZone[i].stop   = 0;
      forbiddenZone[i].active = 0;
      return;
    }

    if(start > stop){
      swap = stop;
      stop = start;
      start = swap;
    }

    forbiddenZone[i].start  = start;
    forbiddenZone[i].stop   = stop;
    forbiddenZone[i].active = 1;
  }

  return;
}

/* ---------------------------------------------------------------------
    enable/disable forbidden zone
 --------------------------------------------------------------------- */
void commandEnableForbiddenZone(char* param0, char* param1){

  uint8_t i = 0;
  uint8_t val = 0;

  i = (uint8_t)strtol(param0, (char **)NULL, 10);

  if(i < MOTOR0 || i > MAX_MOTOR){
    sprintf(txString.buffer, "err: unknown motor");
    sendText(txString.buffer);
    return;
  }
  else{
    val = (uint8_t)strtol(param1, (char **)NULL, 10);
    forbiddenZone[i].active = val;
  }

  return;
}


/* ---------------------------------------------------------------------
    define a program step

    param0: program step number
    param1: position for motor 0
    param2: position for motor 1
    param3: position for motor 2
    param4: position for motor 3
    param5: absolute or relative movement

    always gets param1..param4 in steps (calculated by python interface)
 --------------------------------------------------------------------- */
void commandSetProgStep(char* param0, char* param1, char* param2,
                        char* param3, char* param4, char* param5){

  uint8_t step, i;

  step = (uint8_t)strtol(param0, (char **)NULL, 10);

  if(step >= MAX_PROGRAM_STEPS){
    /* not more than MAX_PROGAM_STEPS allowed */
    return;
  }

  if(strcmp(param5, "NONE") == 0){
    programList[step].isActive = 0;
    programList[step].absRel = PROG_RELATIVE_MOVEMENT;
    for(i = 0; i <= MAX_MOTOR; i++){
      programList[step].position[i] = 0;
    }
    return;
  }

  programList[step].position[0] = (int16_t)strtol(param1, (char **)NULL, 10);
  programList[step].position[1] = (int16_t)strtol(param2, (char **)NULL, 10);
  programList[step].position[2] = (int16_t)strtol(param3, (char **)NULL, 10);
  programList[step].position[3] = (int16_t)strtol(param4, (char **)NULL, 10);

  programList[step].isActive = 1;

  if(strcmp(param5, "ABS") == 0){
    programList[step].absRel = PROG_ABSOLUTE_MOVEMENT;
  }
  if(strcmp(param5, "REL") == 0){
    programList[step].absRel = PROG_RELATIVE_MOVEMENT;
  }

  return;
}

/* ---------------------------------------------------------------------
    returns the on/off state of a motor
 --------------------------------------------------------------------- */
void commandGetMotorState(char* param0){

  uint8_t i = 0;
  uint8_t val = 0;

  i = (uint8_t)strtol(param0, (char **)NULL, 10);

  if(i < MOTOR0 || i > MAX_MOTOR){
    sprintf(txString.buffer, "err: unknown motor");
    sendText(txString.buffer);
    return;
  }
  else{
    if(motor[i].isTurnedOn){
      sprintf(txString.buffer, "1");
    }
    else{
      sprintf(txString.buffer, "0");
    }

    sendText(txString.buffer);
  }

  return;
}

/* ---------------------------------------------------------------------
    debugging output for anything we'd like to know
 --------------------------------------------------------------------- */
void commandDebugReadout(){

  uint8_t i = 0;

  for(i = 0; i < MAX_PROGRAM_STEPS; i++){
    sprintf(txString.buffer, "%d %d %d %d %d %d %d", i, programList[i].position[0],
                                                        programList[i].position[1],
                                                        programList[i].position[2],
                                                        programList[i].position[3],
                                                        programList[i].absRel,
                                                        programList[i].isActive);
    sendText(txString.buffer);
  }

  return;
}


/* =====================================================================
    interrupt routines
====================================================================== */

/* ---------------------------------------------------------------------
    A character in the USART receive buffer is ready for fetch.
    Build up the command string for parsing here. If the
    rxSerial.readyToProcess is NOT zero, the actual command
    in the buffer is not parsed yet. In this case refuse the new char.
 --------------------------------------------------------------------- */
ISR(USART0_RX_vect){

  uint8_t c;

  c = UDR0;

  if(rxString.readyToProcess == 0){ /* actual rxString in buffer not parsed yet */
    if(c == '\r'){
      /* get rid of windoze line ending overhead bullshit */
      return;
    }
    if((c != '\n') && (rxString.charCount < SERIAL_BUFFERSIZE)){
      rxString.buffer[rxString.charCount] = c;
      rxString.charCount++;
    }
    else{
      /* end of command */
      rxString.buffer[rxString.charCount] = '\0';
      rxString.readyToProcess = 1;

      /* here the command is completely received */
      sendChar(0x06); /* therefore send ACK */
    }
  }
  else{ /* actual command in buffer is not processed yet */
    sendChar(0x15);   /* send a NAK */
  }

  return;
}

/* ---------------------------------------------------------------------
    if any button is pressed or turned, this interrupt will
    handle the reaction
 --------------------------------------------------------------------- */
ISR(TIMER0_COMPA_vect){

  uint8_t inputReg = 0;

  static int16_t last = 0;  /* save old rot enc value */

  inputReg = PINB;  /* all buttons are connected to PORTB */

  /* first verify, that PINB differs from ALL_BUTTONS
   * exclude the rotary encoder but include the rotary encoder press-function
   */

  /* check if a button is actually in process */
  if(buttonState.readyToProcess){
    return;
  }
  else{
    /* debouncing the buttons */
    if(buttonState.inDebouncingMode == 0){
      if(buttonState.inputRegister != inputReg){
        buttonState.inputRegister = inputReg;
        buttonState.inDebouncingMode = 1;
      }
      else{
        /* button is still pressed */
        goto ROTARY_ENCODER;
      }
    }
    else{
      if(buttonState.inputRegister == inputReg){
        /* debouncing completed, no register change --> button recognized */
        buttonState.readyToProcess = 1;
      }
      else{
        /* debouncing failed, start again */
        buttonState.inputRegister = 0;
      }

      buttonState.inDebouncingMode = 0;
    }
  }

  /* now care about the rotary encoder (only rotations)
   *
   * code from:
   * http://www.mikrocontroller.net/articles/Drehgeber
   */

ROTARY_ENCODER:
  last = (last << 2) & 0x0F;
  if(PHASE_A){
    last |= 2;
  }
  if(PHASE_B){
    last |= 1;
  }
  if(pgm_read_byte(&table[last])){
    rotEnc.steps += pgm_read_byte(&table[last]);
    rotEnc.readyToProcess = 1;
  }
}

/* ---------------------------------------------------------------------
    handles all motor movement
 --------------------------------------------------------------------- */
ISR(TIMER2_COMPA_vect){

  uint8_t i = 0;
  int16_t stepDiff[MAX_MOTOR + 1];

  uint8_t outputDir  = 0;
  uint8_t outputStep = 0;

  double stepsPerFullRotation[MAX_MOTOR + 1];

  for(i = 0; i <= MAX_MOTOR; i++){
    stepDiff[i] = motor[i].desiredPosition - motor[i].actualPosition;

    stepsPerFullRotation[i] =  motor[i].gearRatio
                              *motor[i].stepsPerFullRotation
                              *motor[i].subSteps;

    if(stepDiff[i] == 0){
      /* no motor movement required */
      continue;
    }
    else{
      /* check if the wait-time between two steps is over */
      if(motor[i].delayCounter){
        /* seems not to be, so decrement */
        motor[i].delayCounter--;

        /* and we also got time here to correct motor step error */
        if(motor[i].stepError > 1.0f){
          motor[i].desiredPosition += 1;
          motor[i].stepError -= 1.0f;
        }
        if(motor[i].stepError < -1.0f){
          motor[i].desiredPosition += -1;
          motor[i].stepError += 1.0f;
        }
      }
      else{
        /* here we just waited the specified time between two steps */
        motor[i].isMoving = 1;

        if(stepDiff[i] < 0){
          /* moving CCW */
          /* check for forbidden zone */
          if(forbiddenZone[i].active
             && ((motor[i].actualPosition + 1) == forbiddenZone[i].stop)){
            motor[i].desiredPosition = motor[i].actualPosition;
            motor[i].isMoving = 0;
          }

          outputDir  |= (1 << (2*i + 1)); /* 1 = CCW, 0 = CW */
          outputStep |= (1 << (2*i));
        }
        else{
          /* moving CW */
          /* check for forbidden zone */
          if(forbiddenZone[i].active
             && ((motor[i].actualPosition + 1) == forbiddenZone[i].start)){
            motor[i].desiredPosition = motor[i].actualPosition;
            motor[i].isMoving = 0;
          }

          outputStep |= (1 << (2*i));
        }
        /* so we will move and therefore set back the delay counter */
        motor[i].delayCounter = 2*motor[i].waitBetweenSteps-1;
      }
    }
  }

  PORTC |= outputDir;     /* set direction */
  _delay_us(5.0);         /* sync */
  PORTC |= outputStep;    /* make exactly one step */
  _delay_us(2.0);         /* sync */
  PORTC = 0;

  /* update motor positions */
  for(i = 0; i <= MAX_MOTOR; i++){
    if(motor[i].isMoving){
      if(stepDiff[i] > 0){
        /* check if we got one full rotation */
        if(((motor[i].actualPosition) + 1) > stepsPerFullRotation[i]){
          /* so set back to 0 */
          motor[i].actualPosition = 0;
          /* correct desired motor position */
          motor[i].desiredPosition -= (int16_t)round(stepsPerFullRotation[i]);
        }
        motor[i].actualPosition++;
        if(motor[i].isMovingInfinite){
          motor[i].desiredPosition += 1;
        }
      }
      else if(stepDiff[i] < 0){
      /* check if we got one full rotation */
        if(((motor[i].actualPosition) - 1) < 0){
          /* so set back to max steps per round */
          motor[i].actualPosition = (int16_t)round(stepsPerFullRotation[i]);
          /* correct desired motor position */
          motor[i].desiredPosition += (int16_t)round(stepsPerFullRotation[i]);
        }
        motor[i].actualPosition--;
        if(motor[i].isMovingInfinite){
          motor[i].desiredPosition += -1;
        }
      }
      motor[i].isMoving = 0;
    }
  }
}


/* =====================================================================
    main loop
====================================================================== */
int main(void){

  uint8_t i;
  uint8_t commandCode;

  /* initialize command parameter list */
  commandParam = (char**)malloc(NUMBER_OF_PARAMETERS * sizeof(char*));
  for(i = 0; i < NUMBER_OF_PARAMETERS; i++){
    commandParam[i] = (char*)malloc(PARAMETER_LENGTH * sizeof(char));
  }

  /* initialize menu strings */
  menu.currentDisplayValue = (char**)malloc(NUMBER_DISPLAY_VALUE_STRINGS * sizeof(char*));
  menu.newDisplayValue     = (char**)malloc(NUMBER_DISPLAY_VALUE_STRINGS * sizeof(char*));
  for(i = 0; i < NUMBER_DISPLAY_VALUE_STRINGS; i++){
    menu.currentDisplayValue[i] = (char*)malloc(DISPLAY_VALUE_STRING_LENGTH * sizeof(char));
    menu.newDisplayValue[i]     = (char*)malloc(DISPLAY_VALUE_STRING_LENGTH * sizeof(char));
  }

  menu.currentMenuText     = (char**)malloc(NUMBER_DISPLAY_MENU_STRINGS * sizeof(char*));
  menu.newMenuText         = (char**)malloc(NUMBER_DISPLAY_MENU_STRINGS * sizeof(char*));
  for(i = 0; i < NUMBER_DISPLAY_MENU_STRINGS; i++){
    menu.currentMenuText[i] = (char*)malloc(DISPLAY_MENU_STRING_LENGTH * sizeof(char));
    menu.newMenuText[i]     = (char*)malloc(DISPLAY_MENU_STRING_LENGTH * sizeof(char));
  }

  /* initialize TX and RX buffers for USART serial interface */
  rxString.buffer = (char*)malloc(SERIAL_BUFFERSIZE * sizeof(char));
  txString.buffer = (char*)malloc(SERIAL_BUFFERSIZE * sizeof(char));
  commandString.buffer = (char*)malloc(SERIAL_BUFFERSIZE * sizeof(char));

  /* initialize displayBuffer */
  displayBuffer = (char*)malloc(DISPLAY_BUFFER_SIZE * sizeof(char));

  lcd_init();
  lcd_home();

RESET:
  initDataStructs();  /* must be the first function after reset! */
  initBuffers();
  initADC();
  initMotorDelayTimer();
  initManualOperatingButtons();
  initUSART();

  /* TODO: detect motors */

  /* turn on all motors */
  for(i = 0; i <= MAX_MOTOR; i++){
    setMotorState(i, ON);
  }

  loadConfigFromEEPROM();

  updateDisplay();

  sei();

  /* start the never ending story */
  for(;;){

    /* check for manual done changes */
    updateMenu();

    /* check for changed values and update them on the display */
    updateDisplay();

    /* check for new received command */
    if(rxString.readyToProcess){
      copyRXstring();
      commandCode = parseCommand();
    }

    switch(commandCode){
      case 0x80:    /* no or unknown command, ignore it */
        break;

      case 0x81:    /* *RST */
        cli();
        prepareReset();
        goto RESET;
        break;

      case 0x82:    /* *IDN? */
        eeprom_read_block((void*)txString.buffer, (const void*)IDNtext, IDN_STRING_LENGTH + 1);
        sendText(txString.buffer);
        break;

      case 0x83:    /* set IDN */
        if(strlen(commandParam[1]) > IDN_STRING_LENGTH){
          break;
        }
        eeprom_update_block((const void*)commandParam[1], (void*)IDNtext, IDN_STRING_LENGTH + 1);
        break;

      case 0x84:    /* MOVEABS */
        commandMoveAbs(commandParam[1], commandParam[2], commandParam[3]);
        break;

      case 0x85:    /* MOVEREL */
        commandMoveRel(commandParam[1], commandParam[2], commandParam[3]);
        break;

      case 0x86:    /* ZERORUN */
        motorZeroRun((uint8_t)atoi(commandParam[1]));
        break;

      case 0x87:    /* ENABLE */
        commandEnable(commandParam[1], commandParam[2]);
        break;

      case 0x88:    /* POS? --> get position in [unit] */
        sendText(commandGetMotorPosition(commandParam[1], commandParam[2]));
        break;

      case 0x89:    /* SAVECONF: save current machine configuration */
        saveConfigToEEPROM();
        break;

      case 0x8A:    /* LOADCONF: load last saved machine configuration */
        loadConfigFromEEPROM();
        break;

      case 0x8B:    /* ISMOVING? */
        sendText(commandIsMoving(commandParam[1]));
        break;

      case 0x8C:    /* GETANALOG */
        sendText(commandGetAnalog(commandParam[1]));
        break;

      case 0x8D:    /* GETOPTZEROPOS */
        sendText(commandGetOptZeroPos(commandParam[1]));
        break;

      case 0x8E:    /* SETOPTZEROPOS */
        commandSetOptZeroPos(commandParam[1], commandParam[2]);
        break;

      case 0x8F:    /* GETGEARRATIO */
        sendText(commandGetGearRatio(commandParam[1]));
        break;

      case 0x90:    /* SETGEARRATIO */
        commandSetGearRatio(commandParam[1], commandParam[2]);
        break;

      case 0x91:    /* GETFULLROT */
        sendText(commandGetFullRotation(commandParam[1]));
        break;

      case 0x92:    /* SETFULLROT */
        commandSetFullRotation(commandParam[1], commandParam[2]);
        break;

      case 0x93:    /* GETSUBSTEPS */
        sendText(commandGetSubSteps(commandParam[1]));
        break;

      case 0x94:    /* SETSUBSTEPS */
        commandSetSubSteps(commandParam[1], commandParam[2]);
        break;

      case 0x95:    /* GETWAITTIME */
        sendText(commandGetWaitTime(commandParam[1]));
        break;

      case 0x96:    /* SETWAITTIME */
        commandSetWaitTime(commandParam[1], commandParam[2]);
        break;

      case 0x97:    /* SETCONSTSPEED */
        commandSetConstSpeed(commandParam[1], commandParam[2], commandParam[3]);
        break;

      case 0x98:    /* FACTORYRESET */
        commandFactoryReset();
        cli();
        prepareReset();
        goto RESET;
        break;

      case 0x99:    /* STOPALL */
        ATOMIC_BLOCK(ATOMIC_FORCEON){
          for(i = 0; i <= MAX_MOTOR; i++){
            motor[i].desiredPosition = motor[i].actualPosition;
            motor[i].isMovingInfinite = MOTOR_MOVE_INFINITE_STOP;
          }
        }
        break;

      case 0x9A:    /* SETFORBZONE */
        commandSetForbiddenZone(commandParam[1], commandParam[2], commandParam[3]);
        break;

      case 0x9B:    /* ENABFORBZONE */
        commandEnableForbiddenZone(commandParam[1], commandParam[2]);
        break;

      case 0x9C:    /* SETPROGSTEP */
        commandSetProgStep(commandParam[1], commandParam[2], commandParam[3],
                           commandParam[4], commandParam[5], commandParam[6]);
        break;

      case 0x9D:    /* GETMOTSTATE */
        commandGetMotorState(commandParam[1]);
        break;

      case 0x9E:
        commandDebugReadout();
        break;

      default:
        break;
    }

    commandCode = 0x80;
  }

  /* if we get here, doomsday is near */
  return 0;
}




































