FILENAME = motordriver
DEV=$(shell dmesg | grep ttyUSB | tail -1 | sed s/.*tty/\\/dev\\/tty\/)
CPUTYPE = atmega644
F_CPU=20000000

OPTIMIZE = -O2
PRINTF_LIB_FLOAT = -Wl,-u,vfprintf -lprintf_flt -lm

CC = /usr/bin/avr-gcc
OBJDUMP = /usr/bin/avr-objdump
OBJCOPY = /usr/bin/avr-objcopy

all: $(FILENAME).s $(FILENAME).elf $(FILENAME).lst text

burn:
	avrdude -F -p m644 -P $(DEV) -U flash:w:$(FILENAME).hex -c AVR911

load:
	avrdude -D -F -p m644 -P $(DEV) -c stk500v1 -b 57600 -U flash:w:$(FILENAME).hex

clean: 
	rm -f $(FILENAME).elf $(FILENAME).s $(FILENAME).lst $(FILENAME).hex $(FILENAME).bin $(FILENAME).srec

$(FILENAME).elf: $(FILENAME).c
	$(CC) -g -mmcu=$(CPUTYPE) -w $(OPTIMIZE) $(PRINTF_LIB_FLOAT) -o $(FILENAME).elf $(FILENAME).c

$(FILENAME).s: $(FILENAME).c
	$(CC) -mmcu=$(CPUTYPE) -w  $(OPTIMIZE) $(PRINTF_LIB_FLOAT) -S $(FILENAME).c

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
