avrdude -D -F -p m644 -P /dev/ttyUSB0 -c stk500v1 -b 115200 -U flash:w:motordriver.hex
