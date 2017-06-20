TARGET=seq
CC=avr-gcc
MCU=atmega48pa
AD_MCU=m48
CFLAGS=-g -mmcu=$(MCU) -Wall -Os -c
LFLAGS=-g -mmcu=$(MCU) -o $(TARGET).out -Wl,-Map,$(TARGET).map
ACOPY=avr-objcopy
ACFLAGS=-R .eeprom -O ihex

AVR=avrdude
PROG=usbtiny
AVRFLAGS=-c $(PROG) -p $(AD_MCU) -U

$(TARGET):
	$(CC) $(CFLAGS) $(TARGET).c
	$(CC) $(LFLAGS) $(TARGET).o
	$(ACOPY) $(AFLAGS) $(TARGET).out $(TARGET).hex

flash: all
	$(AVR) $(AVRFLAGS) flash:w:$(TARGET).hex

all: $(TARGET)

clean:
	rm *.hex
	rm *.map
	rm *.out