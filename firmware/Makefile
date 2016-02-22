FILENAME = motordriver
#DEV=$(shell dmesg | grep ttyUSB | tail -1 | sed s/.*tty/\\/dev\\/tty\/)
DEV=COM3
#DEV=COM9
#SPEED=57600
SPEED=115200

#GCC_CPUTYPE = atmega644
GCC_CPUTYPE = atmega1284p
#DUDE_CPUTYPE = m644
DUDE_CPUTYPE = m1284p

F_CPU=20000000


OPTIMIZE = -O2
PRINTF_LIB_FLOAT = -Wl,-u,vfprintf -lprintf_flt -lm

#CC = /usr/bin/avr-gcc
#OBJDUMP = /usr/bin/avr-objdump
#OBJCOPY = /usr/bin/avr-objcopy
CC = avr-gcc
OBJDUMP = avr-objdump
OBJCOPY = avr-objcopy

all: $(FILENAME).s $(FILENAME).elf $(FILENAME).lst text

burn:
	avrdude -F -p $(DUDE_CPUTYPE) -P $(DEV) -U flash:w:$(FILENAME).hex -c AVR911

load:
	avrdude -D -F -p $(DUDE_CPUTYPE) -P $(DEV) -c stk500v1 -b $(SPEED) -U flash:w:$(FILENAME).hex

clean: 
	rm -f $(FILENAME).elf $(FILENAME).s $(FILENAME).lst $(FILENAME).hex $(FILENAME).bin $(FILENAME).srec

$(FILENAME).elf: $(FILENAME).c
	$(CC) -g --std=c99 -mmcu=$(GCC_CPUTYPE) -o $(FILENAME).elf $(FILENAME).c -w $(OPTIMIZE) $(PRINTF_LIB_FLOAT)

$(FILENAME).s: $(FILENAME).c
	$(CC) --std=c99 -mmcu=$(GCC_CPUTYPE)  -S $(FILENAME).c -w  $(OPTIMIZE) $(PRINTF_LIB_FLOAT)

$(FILENAME).lst: $(FILENAME).elf
	$(OBJDUMP) -h -S $(FILENAME).elf >$(FILENAME).lst

# Rules for building the .text rom images

text: hex bin srec

hex:  $(FILENAME).hex
bin:  $(FILENAME).bin
srec: $(FILENAME).srec

%.hex: %.elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@

%.srec: %.elf
	$(OBJCOPY) -j .text -j .data -O srec $< $@

%.bin: %.elf
	$(OBJCOPY) -j .text -j .data -O binary $< $@
