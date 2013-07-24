AVRCC	        = avr-gcc -mmcu=atmega8 -fomit-frame-pointer
AVROBJCOPY	= avr-objcopy
AVRCFLAGS	= -Wall -Os -g -Wall -I.

all: main.hex

%.o: %.c
	$(AVRCC) $(AVRCFLAGS) -c -o $@ $^

%.hex: %.elf
	$(AVROBJCOPY) -j .text -j .data -O ihex $^ $@

%.bin: %.elf
	$(AVROBJCOPY) -j .text -j .data -O binary $^ $@

main.elf: main.o crc8.o
	$(AVRCC) -o $@ $^

install: main.hex
	sudo avrdude -pm8 -cusbtiny -U flash:w:main.hex:i

clean:
	rm -f *.hex *.bin *.o *.elf
