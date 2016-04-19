
/*
 *   libavrparse.c: LK-instruments
 *   Copyright (C) 2014  Stefan Lasse
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

#ifndef LIBPARSEAVR_H
#define LIBPARSEAVR_H

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/atomic.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>


#ifndef SERIAL_BUFFERSIZE
  #define SERIAL_BUFFERSIZE 64            /* should be enough */
#endif

#ifndef NUMBER_OF_PARAMETERS
  #define NUMBER_OF_PARAMETERS 10         /* amount of parameters to be kept */
#endif

#ifndef PARAMETER_LENGTH
  #define PARAMETER_LENGTH 20             /* max length of a single parameter */
#endif

#ifndef ALLOWED_CMD_DELIMITERS
  #define ALLOWED_CMD_DELIMITERS " ,;\t"  /* for cmd/parameter separation */
#endif

#ifndef TOTAL_NUMBER_OF_COMMANDS
  #define TOTAL_NUMBER_OF_COMMANDS 1
#endif


/* =====================================================================
    function prototypes
====================================================================== */

//void initUSART(void);
void copyRXstring(void);
void sendChar(char c);
void sendText(char *c);
uint8_t parseCommand(void);




#endif  /* LIBPARSEAVR_H */


