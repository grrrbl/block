TARGET=block
MCU=attiny44
SOURCES=block.c uart.c
F_CPU=8000000UL
PROGRAMMER=usbasp

#Ab hier nichts verändern
OBJECTS=$(SOURCES:.c=.o)
CFLAGS=-c -Os -Wall
LDFLAGS=

all: hex eeprom

hex: $(TARGET).hex

eeprom: $(TARGET)_eeprom.hex

$(TARGET).hex: $(TARGET).elf
	avr-objcopy -O ihex -j .data -j .text $(TARGET).elf $(TARGET).hex

$(TARGET)_eeprom.hex: $(TARGET).elf
	avr-objcopy -O ihex -j .eeprom --change-section-lma .eeprom=1 $(TARGET).elf $(TARGET)_eeprom.hex

$(TARGET).elf: $(OBJECTS)
	avr-gcc $(LDFLAGS) -mmcu=$(MCU) $(OBJECTS) -o $(TARGET).elf

assembler: $(SOURCES)
	avr-gcc -S $(CFLAGS) -mmcu=$(MCU) $(SOURCES) -o $(TARGET).s

.c.o:
	avr-gcc $(CFLAGS) -DF_CPU=$(F_CPU) -mmcu=$(MCU) $< -o $@

size:
	avr-size --mcu=$(MCU) -C $(TARGET).elf

program:
	avrdude -p$(MCU) -c$(PROGRAMMER) -Uflash:w:$(TARGET).hex:a

clean_tmp:
	rm -rf *.o
	rm -rf *.elf

clean:
	rm -rf *.o
	rm -rf *.elf
	rm -rf *.hex
	rm -rf *.s
