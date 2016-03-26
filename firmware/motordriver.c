/* =====================================================================

    Firmware for Motordriver / SMCx242

 ===================================================================== */

/*
 *   Motordriver
 *   Copyright (C) 2016 LK-Instruments
 */

#define F_CPU 20000000UL /* in Hz */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <util/twi.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>

//#include "lcd-routines.h"
#include "characterOLED.h"

/* ---------------------------------------------------------------------
    some global definitions
 --------------------------------------------------------------------- */
#define FW_VERSION (" 1.5")

#define IDN_STRING_LENGTH 50
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
#define MAX_MOTOR           MOTOR3  //set to MOTOR1 for 2 or MOTOR3 for 4 channel version

#define MOTOR_SENS0         PA0
#define MOTOR_SENS1         PA1
#define MOTOR_SENS2         PA2
#define MOTOR_SENS3         PA3
#define MOTOR_SENS_MAX      MOTOR_SENS3

/* define the button pinout on AVR */
#define NO_BUTTON           42  /* not allowed to be in intervall [0..7] */
#define BUTTON_MOTOR0       7
#define BUTTON_MOTOR1       6
#define BUTTON_MOTOR2       5
#define BUTTON_MOTOR3       4
#define BUTTON_MENUESCAPE   3
#define BUTTON_ROT_ENC      3
#define BUTTON_ROT_ENC_PRESS 8

/* LED definitions for buttons */
#define BUTT_LED_CHANNELS 48
#define RED     2
#define GREEN   1
#define BLUE    0

#define WS2803_CKI  PC7
#define WS2803_SDI  PC6

#define LED_MOT0    0
#define LED_MOT1    1
#define LED_MOT2    2
#define LED_MOT3    3
#define LED_MESC    4


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
    IIC address and config stuff
 --------------------------------------------------------------------- */
#define SCL_FREQ 400000
#define TWBR_VALUE (((F_CPU/SCL_FREQ) - 16)/2)

#define IIC_BUTTON_PORTEXP_ADDR     0x40

#define IIC_MOTOR0_PORTEXP_ADDR     0x42
#define IIC_MOTOR0_DAC_ADDR         0x12

#define IIC_MOTOR1_PORTEXP_ADDR     0x44
#define IIC_MOTOR1_DAC_ADDR         0x14

#define IIC_MOTOR2_PORTEXP_ADDR     0x46
#define IIC_MOTOR2_DAC_ADDR         0x9A

#define IIC_MOTOR3_PORTEXP_ADDR     0x48
#define IIC_MOTOR3_DAC_ADDR         0x9C

#define EXT_EEPROM_ADDR             0xA0

//#define R_SENS 0.2

/* ---------------------------------------------------------------------
    MCP23017 registers (for IOCON.BANK = 0 which is default)
 --------------------------------------------------------------------- */
#define IODIRA      0x00
#define IODIRB      0x01
#define IPOLA       0x02
#define IPOLB       0x03
#define GPINTENA    0x04
#define GPINTENB    0x05
#define DEFVALA     0x06
#define DEFVALB     0x07
#define INTCONA     0x08
#define INTCONB     0x09
#define IOCONA      0x0A
#define GPPUA       0x0C
#define GPPUB       0x0D
#define INTFA       0x0E
#define INTFB       0x0F
#define INTCAPA     0x10
#define INTCAPB     0x11
#define GPIOA       0x12
#define GPIOB       0x13
#define OLATA       0x14
#define OLATB       0x15

/* ---------------------------------------------------------------------
    MCP23008 registers
 --------------------------------------------------------------------- */
#define IODIR       0x00
#define IPOL        0x01
#define GPINTEN     0x02
#define DEFVAL      0x03
#define INTCON      0x04
#define IOCON       0x05
#define GPPU        0x06
#define INTF        0x07
#define INTCAP      0x08
#define GPIO        0x09
#define OLAT        0x0A

/* ---------------------------------------------------------------------
    bit assignment for DRV8825 <--> MCP23017
 --------------------------------------------------------------------- */
#define PORTEXP_MOTOR_ENABLE        3
#define PORTEXP_MOTOR_DECAY         4
#define PORTEXP_MOTOR_SLEEP         5
#define PORTEXP_MOTOR_RESET         6

#define PORTEXP_MOTOR_FAULT         0
#define PORTEXP_MOTOR_HOME          1
#define PORTEXP_MOTOR_SENSA         4
#define PORTEXP_MOTOR_SENSB         3
#define PORTEXP_MOTOR_SENSC         2
#define PORTEXP_MOTOR_MOTA          5
#define PORTEXP_MOTOR_MOTB          6

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

/* for IIC data handling */
typedef struct{

  uint8_t *data;
  uint8_t operationInProgress;

}iic;

/* for motor information */
#define MOTOR_STEP_UNIT_STEP      0
#define MOTOR_STEP_UNIT_DEGREE    1
#define MOTOR_STEP_UNIT_RADIAN    2

#define MOTOR_MOVE_INFINITE_STOP  0
#define MOTOR_MOVE_INFINITE_CW    1
#define MOTOR_MOVE_INFINITE_CCW   2

typedef struct{

  int16_t  actualPosition;         /* always in steps */
  int16_t  desiredPosition;        /* always in steps */
  int16_t  opticalZeroPosition;    /* as offset from zero position in steps */
  double   stepError;
  uint8_t  isMoving;
  uint8_t  isTurnedOn;
  uint8_t  isMovingInfinite;
  double   gearRatio;              /* initially set to 60:18 for old motors, 60:20 for M101 and 1:1 for M102 */
  double   stepsPerFullRotation;   /* initially set to 400 */
  double   subSteps;               /* could be 1, 2, 4, 8, 16, 32 */
  int8_t   stepUnit;               /* could be: step, degree, radian */
  double   stepMultiplier;         /* multiplies the default step just at manual operation */
  uint16_t waitBetweenSteps;       /* in milliseconds */
  uint16_t delayCounter;           /* counts the waited milliseconds */
  int8_t   angularVelocity;        /* in seconds per full rotation */
  double   current;                /* in ampere */
  int8_t   decay;                  /* could be: slow, fast, mixed */

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

#define PHASE_A  (PINC & 1<<PC5)
#define PHASE_B  (PINC & 1<<PC4)
#define DYNAMICS 40


//const int16_t table[16] PROGMEM = {0,0,-1,0, 0,0,0,0,1,0,0, 0,0,0,0,0};
const int16_t table[16] PROGMEM = {0,0,-1,0, 0,0,0,1,1,0,0, 0,0,-1,0,0};
//const int16_t table[16] PROGMEM = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};

typedef struct{

  int8_t direction;
  int8_t steps;
  uint8_t readyToProcess;
  uint8_t buttonPressed;
  uint8_t buttonDebounce;

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

ADD_COMMAND(13, "GETZEROPOS\0",     1, 0x8D)  /* returns saved optical zero position */
ADD_COMMAND(14, "SETZEROPOS\0",     2, 0x8E)  /* sets the optical zero position in steps */
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
ADD_COMMAND(31, "LED\0",            3, 0x9F)  /* TESTCOMMAND */

ADD_COMMAND(32, "GETCURR\0",        1, 0xA0)  /* returns the adjusted motor current  */
ADD_COMMAND(33, "SETCURR\0",        2, 0xA1)  /* sets the current for a motor */
ADD_COMMAND(34, "GETDECAY\0",       1, 0xA2)  /* returns the motor decay */
ADD_COMMAND(35, "SETDECAY\0",       2, 0xA3)  /* sets the decay for a motor */

#define TOTAL_NUMBER_OF_COMMANDS 36

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
                                              &cmd_30_, &cmd_31_, &cmd_32_,
                                              &cmd_33_, &cmd_34_, &cmd_35_
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
ADD_DISPLAY_TEXT(0 , "LK-Instruments\nSMC4242\0"    )
ADD_DISPLAY_TEXT(1 , "Change motor\nposition\0"     )
ADD_DISPLAY_TEXT(2 , "Set step\nmultiplier\0"       )
ADD_DISPLAY_TEXT(3 , "Change step\nunit\0"          )
ADD_DISPLAY_TEXT(4 , "Run internal\nprogram\0"      )
ADD_DISPLAY_TEXT(5 , "Run with\nconstant speed\0"   )
ADD_DISPLAY_TEXT(6 , "Define zero\nposition\0"      )
ADD_DISPLAY_TEXT(7 , "Run zero\ncalibration\0"      )
ADD_DISPLAY_TEXT(8 , "Enter\nsettings menu\0"       )

ADD_DISPLAY_TEXT(9 , "                \n                \0") //dummy menu

ADD_DISPLAY_TEXT(10, "Change motor\nsubstep\0"      )
ADD_DISPLAY_TEXT(11, "Change motor\ncurrent\0"      )
ADD_DISPLAY_TEXT(12, "Change step\nwait time\0"     )
ADD_DISPLAY_TEXT(13, "Save current\nconfiguration\0")
ADD_DISPLAY_TEXT(14, "Load last\nconfiguration\0"   )


#define NUMBER_OF_DISPLAY_MENUS 9
#define NUMBER_OF_SETTINGS_MENUS 5

#define MENU_MAIN                   0
#define MENU_CHANGE_POSITION        1
#define MENU_SET_STEP_MULTIPL       2
#define MENU_CHANGE_STEP_UNIT       3
#define MENU_RUN_PROGRAM            4
#define MENU_CONST_ANGULAR_SPEED    5
#define MENU_OPTICAL_ZERO_POS       6
#define MENU_RUN_ZERO_CALIBRATION   7
#define MENU_SETTINGS               8

#define MENU_DUMMY                  9

#define MENU_CHANGE_SUBSTEPS        10
#define MENU_CHANGE_CURR            11
#define MENU_CHANGE_WAIT_TIME       12
#define MENU_SAVE_CONFIG            13
#define MENU_LOAD_CONFIG            14


/* to hold a list of menu entries */
const menuItem* const menuList[] PROGMEM = {&disp_0_,  &disp_1_,  &disp_2_,
                                            &disp_3_,  &disp_4_,  &disp_5_,
                                            &disp_6_,  &disp_7_,  &disp_8_,
                                            &disp_9_,  &disp_10_, &disp_11_,
                                            &disp_12_, &disp_13_, &disp_14_
                                           };

/* to keep information where we are in the menu */
#define MENU_CHANGE_MODE    1 /* change to value view */
#define MENU_SCROLL_MODE    2 /* scroll through the menu */
#define MENU_VALUE_CHANGE   3 /* change a selected value */
#define MENU_FAST_MOVE_MODE 4 /* moves a motor 10 times faster */
#define MENU_SETTINGS_MODE  5 /* enter settings submenu */

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
volatile iic IIC;                       /* hold IIC data */
volatile analog adc;                    /* ADC information */
volatile menuInfo menu;                 /* information about the menu */
volatile button buttonState;            /* information on the user interface */
volatile rotaryEncoder rotEnc;
volatile progStep programList[MAX_PROGRAM_STEPS];  /* get memory for internal program */
volatile uint8_t buttLedData[BUTT_LED_CHANNELS];


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
double   EEMEM currentEE[MAX_MOTOR+1];
uint8_t  EEMEM decayEE[MAX_MOTOR+1];
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
//void initADC(void);
void initMotorDelayTimer(void);

/* functionality */
void sendChar(char c);
void sendText(char *c);
void prepareReset();
void setMotorState(uint8_t motor, uint8_t status);
void moveMotorRelative(uint8_t mot, int16_t steps);
void defineOpticalZeroPosition(uint8_t i, int16_t step);
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


uint8_t reverseBitOrder(uint8_t b);

uint8_t getMotorSens(uint8_t mot, uint8_t sens);


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
    motor[i].gearRatio            = 60.0f/20.0f; /* TODO */
    motor[i].stepsPerFullRotation = 400.0f;
    motor[i].subSteps             = 4.0f;
    motor[i].stepMultiplier       = 1.0f;
    motor[i].stepUnit             = MOTOR_STEP_UNIT_DEGREE;
    motor[i].waitBetweenSteps     = 3;
    motor[i].delayCounter         = 2*motor[i].waitBetweenSteps-1;
    motor[i].angularVelocity      = OFF;
    motor[i].current              = 1.0;
    motor[i].decay                = 0;
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
  rotEnc.buttonPressed = 0;
  rotEnc.buttonDebounce = 0;

  IIC.operationInProgress = 0;

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
 /*
void initADC(void){

  ADMUX   = (1<<REFS0);  // AVcc voltage reference
  ADCSRA  = (1<<ADEN)|(1<<ADPS1)|(1<<ADPS0);

  // start one conversion for initializing
  ADCSRA  |= (1<<ADSC);

  while(ADCSRA & (1<<ADSC)){
    ;
  }

  (void)ADCW; // dummy read out

  return;
}
*/

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
  /*
  setMotorState(MOTOR0, OFF);
  setMotorState(MOTOR1, OFF);
  setMotorState(MOTOR2, OFF);
  setMotorState(MOTOR3, OFF);
  */

  /* turn off all available motors */
  uint8_t i;
  for(i = 0; i <= MAX_MOTOR; i++){
    setMotorState(i, OFF);
  }

  /* stop polling timer for manual operating system */
  TCCR0B = 0;

  /* stop motor movement timer */
  TCCR2B = 0;

  return;
}


/* ---------------------------------------------------------------------
    initializes the motor timer for motor[i].waitBetweenSteps
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
    PORTA |= (1 << (2*mot + 1));
  }
  else{
    /* move CW */
    PORTA &= ~(1 << (2*mot + 1));
  }

  /* now move the motor */
  for (i = 0; i < abs(steps); i++){
    PORTA |= (1 << (2*mot));
    _delay_us(2); /* as specified in datasheet */
    PORTA &= ~(1 << (2*mot));

    for(j = 0; j < motor[mot].waitBetweenSteps; j++){
      _delay_ms(1);
    }
  }

  return;
}

/* ---------------------------------------------------------------------
   setting the optical zero position
 --------------------------------------------------------------------- */
void defineOpticalZeroPosition(uint8_t i, int16_t step){

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
  //while(getADCvalue(i) < thres){
  while(getMotorSens(i, PORTEXP_MOTOR_SENSA)){
    moveMotorRelative(i, -200);
  }

  /* start first search for zero point */
  for(j = 0; j < (uint16_t)round(stepsPerRound); j++){
    moveMotorRelative(i, 1);
    //if(getADCvalue(i) < thres){
	if(getMotorSens(i, PORTEXP_MOTOR_SENSA)){
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
  //while(getADCvalue(i) > thres){
  while(!getMotorSens(i, PORTEXP_MOTOR_SENSA)){
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

  resetMotorLogic(i);

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

  // select channel
  ADMUX = (ADMUX & ~(0x1F)) | (sensPin & 0x1F);

  // reset all values and counters before starting new conversion
  adc.ADCvalue = 0;

  // start the conversion
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
	    eeprom_update_block(&(motor[i].current), &(currentEE[i]), sizeof(double));
      eeprom_update_block(&(motor[i].decay), &(decayEE[i]), sizeof(int8_t));
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
	    eeprom_read_block(&(motor[i].current), &(currentEE[i]), sizeof(double));
      eeprom_read_block(&(motor[i].decay), &(decayEE[i]), sizeof(int8_t));
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
void OLEDdisplay(){

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
void OLEDcursor(){

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



#if 0

/* Interface to a HD44780 compatible LCD in 4-Bit-mode
 * http://www.mikrocontroller.net/articles/HD44780
 * http://www.mikrocontroller.net/articles/AVR-GCC-Tutorial/LCD-Ansteuerung
 *
 * Pinout is defined in lcd-routines.h
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


#endif

/* ---------------------------------------------------------------------
   change button LED color/intensity
 --------------------------------------------------------------------- */
void changeButtonLED(uint8_t butt, uint8_t color, uint8_t intensity){
  
  uint8_t chan = 0;
  
  switch(butt) {
	case 0: chan = 6*2+color; break;
	case 1: chan = 6*1+color; break;
	case 2: chan = 6*0+color; break;
	case 3: chan = 6*5+color; break;
	case 4: chan = 6*4+color; break;
	default: chan = 0; break;
  }

  buttLedData[chan]   = intensity;
  buttLedData[chan+3] = intensity;

  return;
}

/* ---------------------------------------------------------------------
   update LEDs
 --------------------------------------------------------------------- */
void updateLEDs(void){
  
  uint16_t i = 0;
  uint8_t  j = 0;
  uint8_t data = 0;
  uint8_t outbyte = 0;

  for(i = BUTT_LED_CHANNELS - 1; i < 255; i--){
	data = buttLedData[i];
	for(uint8_t i=7; i<255; i--)
    {
        outbyte = (data>>i) & 0x01;
        // Set the data pin to given value, and clk pin to 0
        if(outbyte)
        {
            PORTC |= (outbyte << WS2803_SDI);
        }
        else
        {
            PORTC &= ~(outbyte << WS2803_SDI);
        }
        PORTC &= ~(1 << WS2803_CKI);
        _delay_us(10);

        // Keep the data pin, and set clk pin to 1 (strobe)
        PORTC |= 1 << WS2803_CKI;
        _delay_us(10);

        // Zero both clk and data pins
        PORTC &= ~((1 << WS2803_SDI) | (1 << WS2803_CKI));
    }
  }

  return;
}

/* ---------------------------------------------------------------------
   change motor button LED color/intensity
 --------------------------------------------------------------------- */
void changeMotorButtonLED(uint8_t motor, uint8_t enable){
    
  if(enable){ //button lights white
    changeButtonLED(motor, BLUE, 0x0F);
    changeButtonLED(motor, GREEN, 0x0F);
    changeButtonLED(motor, RED, 0x08);
  }
  else{ //button lights green
    changeButtonLED(motor, BLUE, 0x00);
    changeButtonLED(motor, GREEN, 0x0F);
    changeButtonLED(motor, RED, 0x00);
  }
  //updateLEDs();

  return;
}

/* ---------------------------------------------------------------------
   update motor button LEDs
 --------------------------------------------------------------------- */
void updateMotorButtonLEDs(void){
  
  uint8_t i = 0;

  for(i = 0; i <= MAX_MOTOR; i++){
    if(menu.selectedMotor & (1 << i)){
      changeMotorButtonLED(i, 1);
    }
    else{
      changeMotorButtonLED(i, 0);
    }
  }
  updateLEDs();
  
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
        //lcd_setcursor(0, 1);  /* line 1 */
        //lcd_string(menu.newMenuText[0]);
        OLEDsetCursor(0, 0);
        OLEDprintCC(menu.newMenuText[0]);
        strcpy(menu.currentMenuText[0], menu.newMenuText[0]);
        //lcd_setcursor(0, 2);  /* line 2 */
        //lcd_string(menu.newMenuText[1]);
        OLEDsetCursor(0, 1);
        OLEDprintCC(menu.newMenuText[1]);
        strcpy(menu.currentMenuText[1], menu.newMenuText[1]);
        menu.currentDisplayedMenu = menu.newDisplayedMenu;
        for(j = 0; j <= NUMBER_DISPLAY_VALUE_STRINGS; j++){
          sprintf(menu.currentDisplayValue[j], "no text");
        }
      }
      break;
      
    case MENU_SETTINGS_MODE:
      if(menu.currentDisplayedMenu != menu.newDisplayedMenu){
        changeDisplayMenu(menu.newDisplayedMenu);
        //lcd_setcursor(0, 1);  /* line 1 */
        //lcd_string(menu.newMenuText[0]);
        OLEDsetCursor(0, 0);
        OLEDprintCC(menu.newMenuText[0]);
        strcpy(menu.currentMenuText[0], menu.newMenuText[0]);
        //lcd_setcursor(0, 2);  /* line 2 */
        //lcd_string(menu.newMenuText[1]);
        OLEDsetCursor(0, 1);
        OLEDprintCC(menu.newMenuText[1]);
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
        //lcd_setcursor(0, 1);
        //lcd_string(menu.newDisplayValue[0]);
        OLEDsetCursor(0, 0);
        OLEDprintCC(menu.newDisplayValue[0]);
        strcpy(menu.currentDisplayValue[0], menu.newDisplayValue[0]);
      }
      if(strcmp(menu.currentDisplayValue[1], menu.newDisplayValue[1])){
        //lcd_setcursor(8, 1);
        //lcd_string(menu.newDisplayValue[1]);
        OLEDsetCursor(8, 0);
        OLEDprintCC(menu.newDisplayValue[1]);
        strcpy(menu.currentDisplayValue[1], menu.newDisplayValue[1]);
      }
      if(strcmp(menu.currentDisplayValue[2], menu.newDisplayValue[2])){
        //lcd_setcursor(0, 2);
        //lcd_string(menu.newDisplayValue[2]);
        OLEDsetCursor(0, 1);
        OLEDprintCC(menu.newDisplayValue[2]);
        strcpy(menu.currentDisplayValue[2], menu.newDisplayValue[2]);
      }
      if(strcmp(menu.currentDisplayValue[3], menu.newDisplayValue[3])){
        //lcd_setcursor(8, 2);
        //lcd_string(menu.newDisplayValue[3]);
        OLEDsetCursor(8, 1);
        OLEDprintCC(menu.newDisplayValue[3]);
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
      sprintf(menu.newDisplayValue[0], "SMCx242 ");
      sprintf(menu.newDisplayValue[1], "        ");
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
          sprintf(menu.newDisplayValue[i], "%cMot %d", c, i+1);
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
	  
    case MENU_CHANGE_CURR:
      for(i = 0; i <= MAX_MOTOR; i++){
        c = (menu.selectedMotor & (1 << i)) ? 0x7E : ' ';
        sprintf(menu.newDisplayValue[i], "%c%.1f A", c, motor[i].current);
      }
      break;
      
    case MENU_SETTINGS:
      sprintf(menu.newDisplayValue[0], "test    ");
      sprintf(menu.newDisplayValue[1], "test    ");
      sprintf(menu.newDisplayValue[2], "test    ");
      sprintf(menu.newDisplayValue[3], "test    ");
      menu.currentMenuMode = MENU_SETTINGS_MODE;
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
  uint8_t tmp = 0;

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
    if(getButtonEvent() == BUTTON_ROT_ENC_PRESS && menu.selectedMotor == 0){
      menu.newMenuMode = MENU_CHANGE_MODE;
    }
    if(getButtonEvent() == BUTTON_ROT_ENC_PRESS && menu.selectedMotor != 0){
      menu.newMenuMode = MENU_VALUE_CHANGE;
    }
    
    //or enter MENU_SETTINGS_MODE
    if(getButtonEvent() == BUTTON_ROT_ENC_PRESS && menuPrompt == MENU_SETTINGS){
      menu.newMenuMode = MENU_SETTINGS_MODE;
      menu.newDisplayedMenu = MENU_CHANGE_SUBSTEPS;
    }
  }
  
  if(menuState == MENU_SETTINGS_MODE){
    /* so here we want to scroll through the settings menu */
          
    rotEncVal = getRotaryEncoderEvent();  /* get wanted menu prompt */
    menuPrompt = (menuPrompt + (int8_t)rotEncVal);
    if(menuPrompt > 9 + NUMBER_OF_SETTINGS_MENUS){
      menuPrompt = 10;
    }
    if(menuPrompt < 10){
      menuPrompt = 9 + NUMBER_OF_SETTINGS_MENUS;
    }

    menu.newDisplayedMenu = (uint8_t)menuPrompt;

    /* or enter the MENU_CHANGE_MODE */
    if(getButtonEvent() == BUTTON_ROT_ENC_PRESS && menu.selectedMotor == 0){
      menu.newMenuMode = MENU_CHANGE_MODE;
    }
    if(getButtonEvent() == BUTTON_ROT_ENC_PRESS && menu.selectedMotor != 0){
      menu.newMenuMode = MENU_VALUE_CHANGE;
    }
    
    buttonVal = getButtonEvent();
    if(buttonVal == BUTTON_MENUESCAPE){
      /* get back to the MENU_SCROLL_MODE */
      menu.newMenuMode = MENU_SCROLL_MODE;
      menu.newDisplayedMenu = MENU_SETTINGS;
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
        updateMotorButtonLEDs();
        break;

      case BUTTON_MOTOR1:
        menu.selectedMotor ^= (1 << MOTOR1);
        menu.newMenuMode = MENU_VALUE_CHANGE;
        updateMotorButtonLEDs();
        break;

      case BUTTON_MOTOR2:
        menu.selectedMotor ^= (1 << MOTOR2);
        menu.newMenuMode = MENU_VALUE_CHANGE;
        updateMotorButtonLEDs();
        break;

      case BUTTON_MOTOR3:
        menu.selectedMotor ^= (1 << MOTOR3);
        menu.newMenuMode = MENU_VALUE_CHANGE;
        updateMotorButtonLEDs();
        break;

      case BUTTON_ROT_ENC_PRESS:
        menu.fastMovingMode ^= 1;
        break;

      case BUTTON_MENUESCAPE:
        if(menu.currentDisplayedMenu < 10){
          /* get back to the MENU_SCROLL_MODE */
          menu.newMenuMode = MENU_SCROLL_MODE;
          //menu.currentDisplayedMenu += 1;
          menu.currentDisplayedMenu = MENU_DUMMY;
          menu.fastMovingMode = OFF;
        }
        else{
          /* get back to the MENU_SETTINGS_MODE */
          menu.newMenuMode = MENU_SETTINGS_MODE;
          //menu.currentDisplayedMenu += 1;
          menu.currentDisplayedMenu = MENU_DUMMY;
          menu.fastMovingMode = OFF;
        }
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
              updateMotorButtonLEDs();
            }
          }
          /* get back to the MENU_SCROLL_MODE when finished calibration*/
          menu.newMenuMode = MENU_SCROLL_MODE;
          //menu.currentDisplayedMenu += 1;
          menu.currentDisplayedMenu = MENU_DUMMY;
          break;

        case MENU_CHANGE_SUBSTEPS:   /* change motor substeps */
          for(i = MOTOR0; i <= MAX_MOTOR; i++){
            if(menu.selectedMotor & (1 << i)){
              tmp = motor[i].subSteps;
              if(rotEncVal > 0){
                //motor[i].subSteps *= 2.0;
                tmp *= 2.0;
              }
              if(rotEncVal < 0){
                //motor[i].subSteps /= 2.0;
                tmp /= 2.0;
              }
              if(motor[i].subSteps < 1){
                //motor[i].subSteps = 1;
                tmp = 1;
              }
              if(motor[i].subSteps > 32){
                //motor[i].subSteps = 32;
                tmp = 32;
              }
              setSubSteps(i,tmp);
            }
          }
          break;

        case MENU_SAVE_CONFIG:   /* save current configuration */
          saveConfigToEEPROM();
          //lcd_clear();
          //lcd_string("saved");
          OLEDclear();
          OLEDsetCursor(0, 0);
          OLEDprintCC("saved");
          break;

        case MENU_LOAD_CONFIG:   /* load last configuration */
          loadConfigFromEEPROM();
          updateIICvalues();
          //lcd_clear();
          //lcd_string("loaded");
          OLEDclear();
          OLEDsetCursor(0, 0);
          OLEDprintCC("loaded");
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
		  
        case MENU_CHANGE_CURR:
          for(i = MOTOR0; i <= MAX_MOTOR; i++){
            if(menu.selectedMotor & (1 << i)){
              motor[i].current += (rotEncVal)/10.0;
              if(motor[i].current < 0){
                motor[i].current = 0;
              }
              if(motor[i].current > 2.5){
                motor[i].current = 2.5;
              }
              setMotorCurrent(i, motor[i].current);
            }
          }
          break;
          
        case MENU_SETTINGS:        
          OLEDclear();
          OLEDsetCursor(0, 0);
          OLEDprintCC("hello"); //should never show up
          break;

        default:  /* in case of fire ;-) */
          break;
      }
    }
  }

  buttonState.readyToProcess = 0;
  rotEnc.readyToProcess = 0;
  rotEnc.buttonPressed = 0;

  return;
}


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

  TWBR = TWBR_VALUE;    /* see macro above */
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


/* =====================================================================
    resolve I2C addresses
====================================================================== */

/* ---------------------------------------------------------------------
   resolve Port Expander address
 --------------------------------------------------------------------- */
uint8_t getPortExpanderAddress(uint8_t mot){

  uint8_t addr = 0;

  switch(mot){
    case MOTOR0:
      addr = IIC_MOTOR0_PORTEXP_ADDR;
      break;

    case MOTOR1:
      addr = IIC_MOTOR1_PORTEXP_ADDR;
      break;

    case MOTOR2:
      addr = IIC_MOTOR2_PORTEXP_ADDR;
      break;

    case MOTOR3:
      addr = IIC_MOTOR3_PORTEXP_ADDR;
      break;

    default:
      addr = 0;
  }

  return addr;
}

/* ---------------------------------------------------------------------
   resolve DAC address
 --------------------------------------------------------------------- */
uint8_t getDACAddress(uint8_t mot){

  uint8_t addr = 0;

  switch(mot){
    case MOTOR0:
      addr = IIC_MOTOR0_DAC_ADDR;
      break;

    case MOTOR1:
      addr = IIC_MOTOR1_DAC_ADDR;
      break;

    case MOTOR2:
      addr = IIC_MOTOR2_DAC_ADDR;
      break;

    case MOTOR3:
      addr = IIC_MOTOR3_DAC_ADDR;
      break;

    default:
      addr = 0;
  }

  return addr;
}

/* =====================================================================
    Port expander subsystem

    http://ww1.microchip.com/downloads/en/DeviceDoc/21952b.pdf
====================================================================== */

/* ---------------------------------------------------------------------
   initialize I2C port expanders in byte mode
 --------------------------------------------------------------------- */
void initPortExpander(uint8_t addr){

  ATOMIC_BLOCK(ATOMIC_FORCEON){
    /*  register addr  |  register value   |       send it        */
    IIC.data[0] = IOCONA; IIC.data[1] = 0x20; IICwrite(addr, IIC.data, 2);

    IIC.data[0] = IODIRA; IIC.data[1] = 0x00; IICwrite(addr, IIC.data, 2);
    IIC.data[0] = IODIRB; IIC.data[1] = 0xFF; IICwrite(addr, IIC.data, 2);
    IIC.data[0] = GPINTENB; IIC.data[1] = 0x1D; IICwrite(addr, IIC.data, 2);
    IIC.data[0] = DEFVALB; IIC.data[1] = 0x03; IICwrite(addr, IIC.data, 2);
    IIC.data[0] = INTCONB; IIC.data[1] = 0x03; IICwrite(addr, IIC.data, 2);
    IIC.data[0] = GPIOA; IIC.data[1] = 0x78; IICwrite(addr, IIC.data, 2);
  }

  return;
}

/* ---------------------------------------------------------------------
   write port expander register
 --------------------------------------------------------------------- */
void writePortExpanderRegister(uint8_t addr, uint8_t reg, uint8_t val){

  ATOMIC_BLOCK(ATOMIC_FORCEON){
    IIC.data[0] = reg; IIC.data[1] = val;
    IICwrite(addr, IIC.data, 2);
  }

  return;
}

/* ---------------------------------------------------------------------
   read port expander register
 --------------------------------------------------------------------- */
uint8_t readPortExpanderRegister(uint8_t addr, uint8_t reg){

  uint8_t val = 0;

  ATOMIC_BLOCK(ATOMIC_FORCEON){
    IICstart();
    if(IICgetStatus() != TW_START){
      /* error handling */
    }

    IICsendByte(addr | TW_WRITE);
    if(IICgetStatus() !=  TW_MR_SLA_ACK){
      /* error handling */
    }

    IICsendByte(reg);
    if(IICgetStatus() !=  TW_MR_SLA_ACK){
      /* error handling */
    }

    IICstart();
    if(IICgetStatus() != TW_START){
      /* error handling */
    }

    IICsendByte(addr | TW_READ);
    if(IICgetStatus() !=  TW_MR_SLA_ACK){
      /* error handling */
    }

    val = IICreadNACK();
    if(IICgetStatus() !=  TW_MR_DATA_NACK){
      /* error handling */
    }

    IICstop();
  }

  return val;
}

/* ---------------------------------------------------------------------
   because of reversed bit order :'-(
   ... shit happens
 --------------------------------------------------------------------- */
uint8_t reverseBitOrder(uint8_t b){

  b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
  b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
  b = (b & 0xAA) >> 1 | (b & 0x55) << 1;

  return b;
}

/* ---------------------------------------------------------------------
   sets the desired motor substeps
   function takes a value 1...32   
 --------------------------------------------------------------------- */
void setSubSteps(uint8_t mot, uint8_t steps){

  uint8_t addr = 0;
  uint8_t regval = 0;
  
  /*
  if(steps > 5){
    steps = 5;
  }
  */
  
  if(steps > 32){
    steps = 32;
  }
  if(steps < 1){
    steps = 1;
  }
  
  //convert steps from 1...32 to 0...5
  uint8_t tmp_steps = 0;
  tmp_steps = steps;
  steps = 0;
  
  while(tmp_steps != 1){
    tmp_steps = tmp_steps >> 1;
	steps++ ;
  }
  
  addr = getPortExpanderAddress(mot);
  regval = readPortExpanderRegister(addr, GPIOA);

  regval &= 0xF8;

  regval |= (reverseBitOrder(steps) & 0xE0) >> 5;
  writePortExpanderRegister(addr, GPIOA, regval);
  
  motor[mot].subSteps = (1<<steps);

  return;
}

/* ---------------------------------------------------------------------
   reads out the desired motor substeps
   TODO
 --------------------------------------------------------------------- */
uint8_t getSubSteps(uint8_t mot){

  uint8_t steps = 0;

  return steps;
}

/* ---------------------------------------------------------------------
    setMotorState: turns motor on/off
    nENBL: active low
 --------------------------------------------------------------------- */
void setMotorState(uint8_t mot, uint8_t state){

  uint8_t addr = 0;
  uint8_t regval = 0;

  addr = getPortExpanderAddress(mot);
  regval = readPortExpanderRegister(addr, GPIOA);

  if(state){
    regval &= ~(1<<PORTEXP_MOTOR_ENABLE);
  }
  else{
    regval |= (1<<PORTEXP_MOTOR_ENABLE);
  }

  writePortExpanderRegister(addr, GPIOA, regval);
  motor[mot].isTurnedOn = state;

  return;
}

/* ---------------------------------------------------------------------
     sets the motor decay (SLOW = 0, FAST = 1, MIXED = 2)
 --------------------------------------------------------------------- */
void setMotorDecay(uint8_t mot, uint8_t state){

  uint8_t addr = 0;
  uint8_t regval = 0;
  uint8_t iodir = 0;

  addr = getPortExpanderAddress(mot);
  regval = readPortExpanderRegister(addr, GPIOA);
  iodir = readPortExpanderRegister(addr, IODIRA);

  if(state == 1){ //fast decay
    regval |= (1<<PORTEXP_MOTOR_DECAY);
    iodir  &= ~(1<<PORTEXP_MOTOR_DECAY);
  }
  else if(state == 2){ //mixed decay
    iodir  |= (1<<PORTEXP_MOTOR_DECAY); //set as input so pin is "open"
  }
  else{ //slow decay
    regval &= ~(1<<PORTEXP_MOTOR_DECAY);
    iodir  &= ~(1<<PORTEXP_MOTOR_DECAY);
  }

  writePortExpanderRegister(addr, GPIOA, regval);
  writePortExpanderRegister(addr, IODIRA, iodir);

  return;
}

/* ---------------------------------------------------------------------
     resets the internal motor logic (active LOW):
     - step table
     - home position
     - disables H-bridges
 --------------------------------------------------------------------- */
void resetMotorLogic(uint8_t mot){

  uint8_t addr = 0;
  uint8_t regval = 0;

  addr = getPortExpanderAddress(mot);
  regval = readPortExpanderRegister(addr, GPIOA);

  regval &= ~(1<<PORTEXP_MOTOR_RESET);
  writePortExpanderRegister(addr, GPIOA, regval);
  regval |= (1<<PORTEXP_MOTOR_RESET);
  writePortExpanderRegister(addr, GPIOA, regval);

  return;
}

/* ---------------------------------------------------------------------
     send motor to bed (active LOW)
 --------------------------------------------------------------------- */
void setMotorSleep(uint8_t mot){

  uint8_t addr = 0;
  uint8_t regval = 0;

  addr = getPortExpanderAddress(mot);
  regval = readPortExpanderRegister(addr, GPIOA);

  regval &= ~(1<<PORTEXP_MOTOR_SLEEP);

  writePortExpanderRegister(addr, GPIOA, regval);

  return;
}

/* ---------------------------------------------------------------------
     get motor out of bed
 --------------------------------------------------------------------- */
void wakeMotorUp(uint8_t mot){

  uint8_t addr = 0;
  uint8_t regval = 0;

  addr = getPortExpanderAddress(mot);
  regval = readPortExpanderRegister(addr, GPIOA);

  regval |= (1<<PORTEXP_MOTOR_SLEEP);

  writePortExpanderRegister(addr, GPIOA, regval);
  _delay_ms(2);

  return;
}

/* ---------------------------------------------------------------------
     read motor port expander input pin
 --------------------------------------------------------------------- */
uint8_t getMotorSens(uint8_t mot, uint8_t sens){

  uint8_t addr = 0;
  uint8_t regval = 0;
  uint8_t state = 0;

  addr = getPortExpanderAddress(mot);
  regval = readPortExpanderRegister(addr, GPIOB);
  
  //sprintf(txString.buffer, "\nregval=%X", regval);
  //sendText(txString.buffer);
  
  if(regval &= (1<<sens)){
    state = 1;
  }
  else{
    state = 0;
  }

  return state;
}

/* =====================================================================
    DAC subsystem
    http://www.ti.com/lit/ds/symlink/dac081c085.pdf
====================================================================== */

/* ---------------------------------------------------------------------
   initializes the motor current DAC
 --------------------------------------------------------------------- */
void initDAC(uint8_t mot){

  uint8_t addr = 0;

  addr = getDACAddress(mot);
  IIC.data[0] = 0x00; IIC.data[1] = 0x00;
  IICwrite(addr, IIC.data, 2);

  return;
}

/* ---------------------------------------------------------------------
   sets the motor current for DRV8825
 --------------------------------------------------------------------- */
void setMotorCurrent(uint8_t mot, float curr){

  uint8_t addr = 0;
  uint8_t val = 0;
  uint16_t reg = 0;

  if(curr < 0.0){
    curr = 0.0;
  }

  if(curr > 2.5){ /* maximum for DRV8825: 2.5 Ampere */
    curr = 2.5;
  }

  addr = getDACAddress(mot);

  //I = Vref / (5 * Rsense)
  //with Rsense = 0.2 Ohm
  // --> I = Vref
  
  /* 255 / 3.3V * 2.5A = 193 */
  /* 193 <=> 2.5 A, 193/2.5 = 77.2 */

  val = (uint8_t)floor(77.2 * curr);

  IIC.data[0] = (val & 0xF0) >> 4;
  IIC.data[1] = (val & 0x0F) << 4;

  IICwrite(addr, IIC.data, 2);

  return;
}

/* ---------------------------------------------------------------------
   reads out the motor current
 --------------------------------------------------------------------- */
float getMotorCurrent(uint8_t mot){

  float curr = 0.0;
  uint8_t addr = 0;
  uint16_t data = 0;
  uint8_t val = 0;

  addr = getDACAddress(mot);

  IICread(addr, IIC.data, 2);

  data |= (uint16_t)((IIC.data[0] << 8) & 0xFF00);  /* high byte */
  data |= (uint16_t)((IIC.data[1]) & 0x00FF);       /* low byte  */

  val = (data & 0x0FF0) >> 4;

  /* according to setMotorCurrent() max(val) = 193 */
  curr = ((float)val) / 77.2;

  return curr;
}

/* ---------------------------------------------------------------------
   update IIC values for all motors
 --------------------------------------------------------------------- */
void updateIICvalues(void){

  uint8_t i = 0;

  for(i = 0; i <= MAX_MOTOR; i++){
    setMotorCurrent(i, motor[i].current);
    setMotorDecay(i, motor[i].decay);
    setSubSteps(i, (uint8_t)round(motor[i].subSteps));
  }

  return;
}

/* =====================================================================
    Manual operation subsystem
====================================================================== */

/* ---------------------------------------------------------------------
   initializes all buttons and turns on the required interrupts
 --------------------------------------------------------------------- */
void initManualOperatingButtons(void){

  /* only for rotary encoder + its button */
  PORTC |= (1<<PC5)|(1<<PC4)|(1<<PC3);   /* set internal pull-ups */

  /* init port expander for buttons
   *
   * http://www.gammon.com.au/forum/?id=10945
   *
   */
  writePortExpanderRegister(IIC_BUTTON_PORTEXP_ADDR, IOCON, 0x22);
  writePortExpanderRegister(IIC_BUTTON_PORTEXP_ADDR, IODIR, 0xFF);
  writePortExpanderRegister(IIC_BUTTON_PORTEXP_ADDR, IPOL, 0xFF);
  writePortExpanderRegister(IIC_BUTTON_PORTEXP_ADDR, GPINTEN, 0xF8);
  writePortExpanderRegister(IIC_BUTTON_PORTEXP_ADDR, DEFVAL, 0x03);
  writePortExpanderRegister(IIC_BUTTON_PORTEXP_ADDR, INTCON, 0x00);
  writePortExpanderRegister(IIC_BUTTON_PORTEXP_ADDR, GPPU, 0xFF);
  
  //read GPIO register to clear the interrupt
  uint8_t tmp = 0;
  tmp = readPortExpanderRegister(IIC_BUTTON_PORTEXP_ADDR, GPIO);

  /* activate interrupt on INT0 (PD2) */
  EICRA |= (1<<ISC01)|(1<<ISC00);  /* rising edge causes interrupt */
  EIMSK |= (1<<INT0);   /* enable interrupt pin INT0 */

  /* set up a timer for button/rotary_encoder polling
   *
   * the 8-bit Timer/Counter0 is used for that
   */
  TCCR0A |= (1<<WGM01);   /* enable CTC */
  OCR0A   = 40;           /* 40 is an empirical value for best behavior of the rotEnc. */
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
    state = buttonState.inputRegister;
	
	if(rotEnc.buttonPressed){
      button = BUTTON_ROT_ENC_PRESS;
	}
	else if(state & (1<<BUTTON_MOTOR0)){
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
   * returns the turned steps of the rotary encoder
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
      sendChar('a');
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
    sets the desired substeps
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
    //motor[i].subSteps = val;
    setSubSteps(i, (uint8_t)round(val));
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
    returns the adjusted motor current
 --------------------------------------------------------------------- */
char* commandGetMotorCurrent(char* param0){

  uint8_t i = 0;
  double curr = 0.0;

  i = (uint8_t)strtol(param0, (char **)NULL, 10);

  if(i < MOTOR0 || i > MAX_MOTOR){
    sprintf(txString.buffer, "err: unknown motor: %d", i);
  }
  else{
    curr = getMotorCurrent(i);
    sprintf(txString.buffer, "%f", curr);
  }
  
  return txString.buffer;
}

/* ---------------------------------------------------------------------
    sets the desired motor current
 --------------------------------------------------------------------- */
void commandSetMotorCurrent(char* param0, char* param1){
  
  uint8_t i = 0;
  
  i = (uint8_t)strtol(param0, (char **)NULL, 10);
  
  if(i < MOTOR0 || i > MAX_MOTOR){
    sprintf(txString.buffer, "err: unknown motor: %d", i);
    sendText(txString.buffer);
  }
  else{
    motor[i].current = atof(param1);
    setMotorCurrent(i, motor[i].current);
  }

  return;
}

/* ---------------------------------------------------------------------
    returns the motor decay
 --------------------------------------------------------------------- */
char* commandGetMotorDecay(char* param0){

  uint8_t i = 0;

  i = (uint8_t)strtol(param0, (char **)NULL, 10);

  if(i < MOTOR0 || i > MAX_MOTOR){
    sprintf(txString.buffer, "err: unknown motor: %d", i);
  }
  else{
    sprintf(txString.buffer, "%d", motor[i].decay);
  }
  
  return txString.buffer;
}

/* ---------------------------------------------------------------------
    sets the desired motor decay
 --------------------------------------------------------------------- */
void commandSetMotorDecay(char* param0, char* param1){
  
  uint8_t i = 0;
  
  i = (uint8_t)strtol(param0, (char **)NULL, 10);
  
  if(i < MOTOR0 || i > MAX_MOTOR){
    sprintf(txString.buffer, "err: unknown motor: %d", i);
    sendText(txString.buffer);
  }
  else{
    motor[i].decay = atof(param1);
    setMotorDecay(i, motor[i].decay);
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


/* ---------------------------------------------------------------------
    debugging output for anything we'd like to know
 --------------------------------------------------------------------- */
void commandLED(char* param0, char* param1, char* param2){

  uint8_t a, b, c;

  a = (uint8_t)strtol(param0, (char **)NULL, 10);
  b = (uint8_t)strtol(param1, (char **)NULL, 10);
  c = (uint8_t)strtol(param2, (char **)NULL, 16);
  
  sprintf(txString.buffer, "\na=%d\nb=%d\nc=%d", a, b, c);
  sendText(txString.buffer);

  changeButtonLED(a, b, c);
  updateLEDs();
  
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
  static int16_t last = 0;  /* save old rotEnc value */

  inputReg = PINC;  /* rotEnc is connected to PORTC */

  /* handle rotEnc's button */
  if((inputReg^0xFF) & (1<<BUTTON_ROT_ENC)){
    /* button was pressed */
    if(rotEnc.buttonDebounce == 0){
      // enter debouncing mode
	  rotEnc.buttonDebounce = 1;
    }
  }
  else{
      /* button is no longer pressed ...*/
      if(rotEnc.buttonDebounce == 1){
        /* ... and we are already in debouncing mode -> generate event */
        rotEnc.buttonPressed = 1;
        rotEnc.buttonDebounce = 0;
        buttonState.readyToProcess = 1;
      }
  }

  /* now care about the rotary encoder rotations
   *
   * code from:
   * http://www.mikrocontroller.net/articles/Drehgeber
   */
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
    a button seems to be pressed ;-)
 --------------------------------------------------------------------- */
ISR(INT0_vect){
   
  //sendText("INT0");
  
  uint8_t regVal = 0;
  
  regVal = readPortExpanderRegister(IIC_BUTTON_PORTEXP_ADDR, INTCAP);
  
  /* check if a button is actually in process */
  if(buttonState.readyToProcess){
    return;
  }
  else{
    buttonState.inputRegister = regVal;
    buttonState.readyToProcess = 1;
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

  PORTA |= outputDir;     /* set direction */
  _delay_us(1.0);         /* sync */
  PORTA |= outputStep;    /* make exactly one step */
  _delay_us(2.0);         /* sync */
  PORTA &= ~outputStep;
  _delay_us(1.0);
  PORTA = 0;

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

  /* initialize IIC data buffer */
  IIC.data = (char*)malloc(21 * sizeof(char));

  /* initialize displayBuffer */
  displayBuffer = (char*)malloc(DISPLAY_BUFFER_SIZE * sizeof(char));

  /* initialize data for LEDs and WS2803 output pins */
  for(i = 0; i < BUTT_LED_CHANNELS; i++){
    buttLedData[i] = 0;
  }
  DDRC |= (1 << WS2803_CKI) | (1 << WS2803_SDI);
  PORTC |= (1 << WS2803_CKI) | (1 << WS2803_SDI);

  /* initialize PORTA as output for motor Step/Direction */
  DDRA  = 0xFF;
  PORTA = 0;

  //lcd_init();
  //lcd_home();

  /* OLED setup */
  OLEDinit(OLED_V2);
  _delay_ms(500);
  OLEDclear();
  OLEDsetCursor(0, 0);

  initUSART();

  initDataStructs();  /* must be the first function after reset! */
  initBuffers();
  //initADC();
  initIIC();
  
RESET:
  initDataStructs();  /* must be the first function after reset! */ 
  initMotorDelayTimer();
  initManualOperatingButtons();
  
  /* TODO: detect motors if connected */
  
  loadConfigFromEEPROM();

  /* init all available motors */
  for(i = 0; i <= MAX_MOTOR; i++){
    initPortExpander(getPortExpanderAddress(i));
    initDAC(i);
  }
  
  updateIICvalues();
  
  /* turn on all available motors */
  for(i = 0; i <= MAX_MOTOR; i++){
    setMotorState(i, ON);
  }

  updateMenu();
  
  updateDisplay();
  
  //init LEDs with default color pattern
  /*
  for(i = 0; i < 4; i++){
    changeButtonLED(i, GREEN, 0x0F);
  }
  */
  changeButtonLED(LED_MESC, RED, 0x0F);
  //updateLEDs();
  updateMotorButtonLEDs();
  
  commandCode = 0x80;

  sei();  /* turn on interrupts */
  
  //sendText("start loop\0");

  /* start the never ending story */
  for(;;){

    /* check for manual done changes */
    updateMenu();

    /* check for changed values and update them on the display */
    updateDisplay();
    
    //updateMotorButtonLEDs();

    /* check for new received command */
    if(rxString.readyToProcess){
      copyRXstring();
      commandCode = parseCommand();
    }

    switch(commandCode){
      case 0x80:    /* no or unknown command, ignore it */
        break;

      case 0x81:    /* *RST */
        //sendText("reset\0");
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

      case 0x88:    /* GETPOS --> get position in [unit] */
        sendText(commandGetMotorPosition(commandParam[1], commandParam[2]));
        break;

      case 0x89:    /* SAVECONF: save current machine configuration */
        saveConfigToEEPROM();
        break;

      case 0x8A:    /* LOADCONF: load last saved machine configuration */
        loadConfigFromEEPROM();
        updateIICvalues();
        break;

      case 0x8B:    /* ISMOVING */
        sendText(commandIsMoving(commandParam[1]));
        break;

      case 0x8C:    /* GETANALOG */
        sendText(commandGetAnalog(commandParam[1]));
        break;

      case 0x8D:    /* GETZEROPOS */
        sendText(commandGetOptZeroPos(commandParam[1]));
        break;

      case 0x8E:    /* SETZEROPOS */
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

      case 0x9F:    /* LED */
        commandLED(commandParam[1], commandParam[2], commandParam[3]);
        break;
		
      case 0xA0:    /* GETCURR */
        sendText(commandGetMotorCurrent(commandParam[1]));
        break;

      case 0xA1:    /* SETCURR */
        commandSetMotorCurrent(commandParam[1], commandParam[2]);
        break;
        
      case 0xA2:    /* GETDECAY */
        sendText(commandGetMotorDecay(commandParam[1]));
        break;

      case 0xA3:    /* SETDECAY */
        commandSetMotorDecay(commandParam[1], commandParam[2]);
        break;

      default:
        break;
    }

    commandCode = 0x80;
  }

  /* if we get here, doomsday is near */
  return 0;
}




































