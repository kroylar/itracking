CC=avr-gcc
CFLAGS=-g -Os -Wall -mcall-prologues -mmcu=atmega8
OBJ2HEX=avr-objcopy 
TARGET=avrecho

OBJECTS=$(TARGET).o uart.o

program : $(TARGET).hex
	avrdude -c usbtiny -p m8 -U flash:w:$(TARGET).hex

%.o : %.c
	$(CC) -c $(CFLAGS) $< -o $@

%.obj : $(OBJECTS)
	$(CC) $(CFLAGS) $(OBJECTS) -o $@

%.hex : %.obj

	$(OBJ2HEX) -R .eeprom -O ihex $< $@

clean :
	rm -f *.hex *.obj *.o

