avr-gcc -g -Os -mmcu=atmega644p -c motordriver.c
avr-gcc -g -mmcu=atmega644p -o motordriver.elf motordriver.o
avr-size -C -x motordriver.elf
